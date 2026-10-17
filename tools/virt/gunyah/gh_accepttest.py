#!/usr/bin/env python3
"""End-to-end GuestAccept test: the guest really accepts, so UNSHARE must say EBUSY.

Guest = gh_rmproxy.bin (see gh_rmproxy.S): a command loop driven through MMIO
exits. The host dumps the RM-overlaid DTB to find the RM msgq capabilities,
frames the RM RPCs itself, and has the guest send/receive them. Checks:
  - MEM_ACCEPT of a GH_VM_PROTECTED_SHARE handle at a guest-chosen IPA
  - host->guest and guest->host visibility through the shared buffer
  - GH_VM_PROTECTED_UNSHARE while accepted -> EBUSY, pages stay pinned
  - after guest MEM_RELEASE -> UNSHARE succeeds, pins drop
  - the same for a DMA-BUF (udmabuf) share, including a sliced re-share
  - shares still accepted at VM fd close are reclaimed after the VM reset
Run as root. Exit status = number of failures.
"""
import ctypes, errno, fcntl, mmap, os, struct, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gh_vmtest as T
import gh_sharetest as S

RAM_BASE, RAM_SIZE = 0x80000000, 0x400000
DTB_OFF, DTB_SIZE = 0x200000, 0x10000
TXBUF, RXBUF = RAM_BASE + 0x100000, RAM_BASE + 0x110000
GICD, GICR = 0x3FFF0000, 0x3FFD0000
MMIO = 0x10000000
ACCEPT_IPA = 0x90000000
GH_ERROR_MSGQUEUE_EMPTY = 60
RM_RPC_MEM_ACCEPT, RM_RPC_MEM_RELEASE = 0x51000011, 0x51000014

check, expect_errno = S.check, S.expect_errno

class GuestExited(Exception):
    pass

class Guest:
    """Drives gh_rmproxy over MMIO exits. Offsets into struct gunyah_vcpu_run."""
    def __init__(self, vcpu, run):
        self.vcpu, self.run, self.exits = vcpu, run, 0

    def _exit(self):
        while True:
            fcntl.ioctl(self.vcpu, T.GUNYAH_VCPU_RUN)
            self.exits += 1
            reason, = struct.unpack_from("<I", self.run, 8)
            if reason == 1:
                pa, data, ln, is_w, _ = struct.unpack_from("<Q8sIBB", self.run, 16)
                struct.pack_into("<B", self.run, 37, 0)          # resume HANDLED
                return pa, bool(is_w), ln, data
            if reason == 2:
                st, = struct.unpack_from("<I", self.run, 16)
                raise GuestExited({1: "LOAD_FAILED", 2: "EXITED", 3: "CRASHED"}.get(st, st))
            if reason == 3:
                pa, attempt, _ = struct.unpack_from("<QiB", self.run, 16)
                raise RuntimeError("guest page fault at %#x (%d)" % (pa, attempt))
            raise RuntimeError("exit_reason %d" % reason)

    def feed(self, value, at):
        pa, is_w, ln, _ = self._exit()
        if is_w or pa != at or ln != 8:
            raise RuntimeError("expected 8-byte read at %#x, got %s len %d at %#x" % (at, "write" if is_w else "read", ln, pa))
        struct.pack_into("<Q", self.run, 24, value)               # mmio.data

    def take(self):
        pa, is_w, ln, data = self._exit()
        if not is_w or pa != MMIO + 8 or ln != 8:
            raise RuntimeError("expected 8-byte write at DATA, got %s len %d at %#x" % ("write" if is_w else "read", ln, pa))
        return int.from_bytes(data, "little")

    def cmd(self, c): self.feed(c, MMIO)
    def arg(self, v): self.feed(v, MMIO + 8)

    def dump(self, addr, nbytes):
        n = (nbytes + 7) // 8
        self.cmd(1); self.arg(addr); self.arg(n)
        return b"".join(self.take().to_bytes(8, "little") for _ in range(n))[:nbytes]

    def load(self, addr, data):
        data = data + bytes(-len(data) % 8)
        self.cmd(2); self.arg(addr); self.arg(len(data) // 8)
        for i in range(0, len(data), 8):
            self.arg(int.from_bytes(data[i:i + 8], "little"))
        self.take()     # ack: stores are complete only once the guest has run on past them

    def send(self, cap, buf, size):
        self.cmd(3); self.arg(cap); self.arg(size); self.arg(buf)
        return self.take()

    def recv(self, cap, buf, size):
        self.cmd(4); self.arg(cap); self.arg(buf); self.arg(size)
        return self.take(), self.take()

    def off(self):
        self.cmd(5)
        try:
            self._exit()
        except GuestExited as e:
            return e.args[0]
        return "no STATUS exit"

def fdt_find_rm_caps(dtb):
    """(tx_capid, rx_capid) from the node compatible with gunyah-resource-manager."""
    magic, total, off_struct, off_strings = struct.unpack_from(">IIII", dtb, 0)
    if magic != 0xd00dfeed:
        raise RuntimeError("bad FDT magic %#x" % magic)
    def cstr(off):
        end = dtb.index(b"\0", off); return dtb[off:end]
    pos, depth, props, seen = off_struct, 0, [{}], []
    while True:
        tok, = struct.unpack_from(">I", dtb, pos); pos += 4
        if tok == 1:
            name = cstr(pos); pos = (pos + len(name) + 1 + 3) & ~3
            props.append({}); depth += 1; seen.append("  " * depth + name.decode())
        elif tok == 2:
            p = props.pop(); depth -= 1
            if b"compatible" in p: seen[-1] += " compatible=%r" % p[b"compatible"] if seen else ""
            if b"gunyah-resource-manager" in p.get(b"compatible", b""):
                reg = p[b"reg"]
                if len(reg) == 16: return struct.unpack(">QQ", reg), p
                if len(reg) == 8: return struct.unpack(">II", reg), p
                raise RuntimeError("RM reg has %d bytes" % len(reg))
        elif tok == 3:
            ln, nameoff = struct.unpack_from(">II", dtb, pos); pos += 8
            props[-1][cstr(off_strings + nameoff)] = dtb[pos:pos + ln]; pos = (pos + ln + 3) & ~3
        elif tok == 4:
            pass
        elif tok == 9:
            raise RuntimeError("no gunyah-resource-manager node in DTB; nodes: %s" % seen)
        else:
            raise RuntimeError("bad FDT token %d at %#x" % (tok, pos - 4))

class RM:
    """RM RPC client: frames here, transport through the guest proxy."""
    def __init__(self, guest, tx, rx):
        self.g, self.tx, self.rx, self.seq = guest, tx, rx, 0

    def call(self, msg_id, payload):
        self.seq = (self.seq + 1) & 0xffff
        msg = struct.pack("<BBHI", 0x21, 0x01, self.seq, msg_id) + payload
        self.g.load(TXBUF, msg)
        err = self.g.send(self.tx, TXBUF, len(msg))
        if err:
            raise RuntimeError("msgq_send gunyah_error %d" % err)
        for _ in range(2000):
            err, ln = self.g.recv(self.rx, RXBUF, 240)
            if err == GH_ERROR_MSGQUEUE_EMPTY:
                time.sleep(0.0005); continue
            if err:
                raise RuntimeError("msgq_recv gunyah_error %d" % err)
            rep = self.g.dump(RXBUF, ln)
            api, typ, seq, rid, ecode = struct.unpack_from("<BBHII", rep)
            if (typ & 3) == 2 and seq == self.seq and rid == msg_id:
                return ecode, rep[12:]
            print("    (skipping RM message type %#x id %#x)" % (typ, rid))
        raise RuntimeError("RM reply timeout")

    def mem_accept(self, handle, ipa, size):
        payload = (struct.pack("<IBBBBI", handle, 0, 2, 0x10 | 0x80, 0, 0)   # NORMAL, SHARE, IPA_CONTIGUOUS|DONE, validate_label=0
                   + struct.pack("<I", 0)                                       # acl: none (RM uses the stored ACL)
                   + struct.pack("<HHQQ", 1, 0, ipa, size)                      # one sgl entry
                   + struct.pack("<HH", 0, 0))                                  # mem attrs: none
        return self.call(RM_RPC_MEM_ACCEPT, payload)[0]

    def mem_release(self, handle):
        return self.call(RM_RPC_MEM_RELEASE, struct.pack("<II", handle, 0))[0]

def pattern(nwords, tag):
    return b"".join(struct.pack("<Q", (tag << 32) | i) for i in range(nwords))

def main():
    here = os.path.dirname(os.path.abspath(__file__))
    code = open(os.path.join(here, "gh_rmproxy.bin"), "rb").read()
    lck_base = S.vmlck_kb()

    dev = os.open("/dev/gunyah", os.O_RDWR)
    vm = fcntl.ioctl(dev, T.GUNYAH_CREATE_VM)
    mem, uaddr = S.anon(RAM_SIZE)
    mem[0:len(code)] = code
    dtb = T.build_dtb(RAM_BASE, RAM_SIZE, GICD, GICR, rm_rpc=True); mem[DTB_OFF:DTB_OFF + len(dtb)] = dtb
    fcntl.ioctl(vm, T.GH_VM_ANDROID_LEND_USER_MEM, struct.pack("<IIQQQ", 0, T.MEM_RWX, RAM_BASE, RAM_SIZE, uaddr))
    fcntl.ioctl(vm, T.GUNYAH_VM_SET_DTB_CONFIG, struct.pack("<QQ", RAM_BASE + DTB_OFF, DTB_SIZE))
    fcntl.ioctl(vm, T.GUNYAH_VM_SET_BOOT_CONTEXT, struct.pack("<IIQ", T.REG_SET_PC, 0, RAM_BASE))
    vcpu_arg = ctypes.c_uint32(0)
    vcpu = fcntl.ioctl(vm, T.GUNYAH_VM_ADD_FUNCTION, bytearray(struct.pack("<IIQ", T.GUNYAH_FN_VCPU, 4, ctypes.addressof(vcpu_arg))), True)
    run = mmap.mmap(vcpu, fcntl.ioctl(vcpu, T.GUNYAH_VCPU_MMAP_SIZE), flags=mmap.MAP_SHARED, prot=mmap.PROT_READ | mmap.PROT_WRITE)
    fcntl.ioctl(vm, T.GUNYAH_VM_START)
    g = Guest(vcpu, run)
    print("[1] protected VM started with gh_rmproxy (%d B)" % len(code))

    hdr = g.dump(RAM_BASE + DTB_OFF, 8)
    total = struct.unpack_from(">I", hdr, 4)[0]
    dtb_out = g.dump(RAM_BASE + DTB_OFF, total)
    open("/tmp/guest-overlaid.dtb", "wb").write(dtb_out)
    (tx, rx), rmprops = fdt_find_rm_caps(dtb_out)
    print("[2] RM-overlaid DTB %d B (%d exits); RM msgq caps tx=%#x rx=%#x" % (total, g.exits, tx, rx))
    rm = RM(g, tx, rx)

    print("[3] share + guest accept")
    size = 4 << 20
    bufA, addrA = S.anon(size)
    pat = pattern(512, 0xA5A5); bufA[0:len(pat)] = pat
    bufA[size - 8:size] = struct.pack("<Q", 0x1A57BA5E00000000 | 0x1234)
    hA = S.share(vm, 7, S.MEM_RW, addrA, size)
    check("share A 4 MiB label 7", hA not in S.BAD_HANDLES, "handle %#x" % hA)
    lck_shared = S.vmlck_kb()
    ecode = rm.mem_accept(hA, ACCEPT_IPA, size - 4096)
    check("MEM_ACCEPT with wrong size refused by RM", ecode != 0, "err_code %#x" % ecode)
    ecode = rm.mem_accept(hA, ACCEPT_IPA, size)
    check("MEM_ACCEPT handle %#x at IPA %#x" % (hA, ACCEPT_IPA), ecode == 0, "err_code %#x" % ecode)
    if ecode:
        print("cannot continue without an accepted parcel"); return S.fails + 1

    print("[4] visibility through the shared buffer")
    got = g.dump(ACCEPT_IPA, len(pat))
    check("host pattern visible to guest (first 4 KiB)", got == pat, "" if got == pat else "first diff at %d" % next(i for i in range(len(pat)) if got[i] != pat[i]))
    got = g.dump(ACCEPT_IPA + size - 8, 8)
    check("last word visible to guest", got == bufA[size - 8:size], got.hex())
    bufA[0x2000:0x2008] = struct.pack("<Q", 0x5EC0ADDED0000001)        # written after accept
    check("host write after accept visible to guest", g.dump(ACCEPT_IPA + 0x2000, 8) == bufA[0x2000:0x2008])
    g.load(ACCEPT_IPA + 0x1000, struct.pack("<Q", 0xDEADBEEFCAFEBABE))
    check("guest write visible to host", bytes(bufA[0x1000:0x1008]) == struct.pack("<Q", 0xDEADBEEFCAFEBABE), bytes(bufA[0x1000:0x1008]).hex())

    print("[5] UNSHARE while the guest holds the acceptance")
    expect_errno("UNSHARE label 7 -> EBUSY", lambda: S.unshare(vm, 7), errno.EBUSY)
    check("pages still pinned after refused UNSHARE", S.vmlck_kb() == lck_shared, "%d kB" % S.vmlck_kb())
    check("guest still reads the buffer after refused UNSHARE", g.dump(ACCEPT_IPA, 64) == pat[:64])
    expect_errno("second UNSHARE still EBUSY", lambda: S.unshare(vm, 7), errno.EBUSY)

    print("[6] guest releases, then host unshares")
    ecode = rm.mem_release(hA)
    check("MEM_RELEASE handle %#x" % hA, ecode == 0, "err_code %#x" % ecode)
    S.unshare(vm, 7); check("UNSHARE after release succeeds", True)
    check("pins dropped", S.vmlck_kb() == lck_shared - (size >> 10), "%d kB" % S.vmlck_kb())

    print("[7] DMA-BUF share (udmabuf over a sealed memfd) accepted by the guest")
    DSIZE, DIPA = 2 << 20, ACCEPT_IPA + (8 << 20)
    mfd = os.memfd_create("gh-accept", os.MFD_ALLOW_SEALING)
    os.ftruncate(mfd, DSIZE)
    fcntl.fcntl(mfd, fcntl.F_ADD_SEALS, fcntl.F_SEAL_SHRINK)
    dmem = mmap.mmap(mfd, DSIZE)                       # host view of the exporter's pages
    dpat = pattern(512, 0xD3A8); dmem[0:len(dpat)] = dpat
    dmem[DSIZE - 8:DSIZE] = struct.pack("<Q", 0xD3A8E4D000000000 | 0x4321)
    ud = os.open("/dev/udmabuf", os.O_RDWR)
    dbuf = fcntl.ioctl(ud, S.UDMABUF_CREATE, bytearray(struct.pack("<IIQQ", mfd, 1, 0, DSIZE)), True)
    hD = S.share_dmabuf(vm, 9, S.MEM_RW, dbuf, 0, DSIZE)
    check("share dmabuf 2 MiB label 9", hD not in S.BAD_HANDLES, "handle %#x" % hD)
    ecode = rm.mem_accept(hD, DIPA, DSIZE)
    check("MEM_ACCEPT dmabuf handle %#x at IPA %#x" % (hD, DIPA), ecode == 0, "err_code %#x" % ecode)
    if ecode == 0:
        check("host pattern in dmabuf visible to guest", g.dump(DIPA, len(dpat)) == dpat)
        check("last dmabuf word visible to guest", g.dump(DIPA + DSIZE - 8, 8) == dmem[DSIZE - 8:DSIZE])
        dmem[0x3000:0x3008] = struct.pack("<Q", 0x5EC0ADDED0000002)
        check("host write to dmabuf after accept visible to guest", g.dump(DIPA + 0x3000, 8) == dmem[0x3000:0x3008])
        g.load(DIPA + 0x1000, struct.pack("<Q", 0xFEEDFACE0BADF00D))
        check("guest write to dmabuf visible to host", bytes(dmem[0x1000:0x1008]) == struct.pack("<Q", 0xFEEDFACE0BADF00D), bytes(dmem[0x1000:0x1008]).hex())
        expect_errno("UNSHARE dmabuf while accepted -> EBUSY", lambda: S.unshare(vm, 9), errno.EBUSY)
        check("guest still reads dmabuf after refused UNSHARE", g.dump(DIPA, 64) == dpat[:64])
        check("MEM_RELEASE dmabuf", rm.mem_release(hD) == 0)
        S.unshare(vm, 9); check("UNSHARE dmabuf after release succeeds", True)
        check("locked_vm untouched by dmabuf share", S.vmlck_kb() == lck_shared - (size >> 10), "%d kB" % S.vmlck_kb())
        hD2 = S.share_dmabuf(vm, 9, S.MEM_RW, dbuf, 1 << 20, 1 << 20)
        check("re-share dmabuf slice [+1 MiB, 1 MiB) label 9", hD2 not in S.BAD_HANDLES, "handle %#x" % hD2)
        check("accept slice at %#x" % DIPA, rm.mem_accept(hD2, DIPA, 1 << 20) == 0)
        check("slice maps dmabuf offset +1 MiB (sees host word at +1 MiB)", g.dump(DIPA, 8) == dmem[1 << 20:(1 << 20) + 8])
        expect_errno("UNSHARE slice while accepted -> EBUSY", lambda: S.unshare(vm, 9), errno.EBUSY)
        # left accepted on purpose: reclaimed with A and B at teardown below

    print("[8] three live accepted shares at VM teardown")
    hA2 = S.share(vm, 7, S.MEM_RW, addrA, size)
    bufB, addrB = S.anon(1 << 20)
    hB = S.share(vm, 8, S.MEM_R, addrB, 1 << 20)
    check("re-share A (label 7) + B (label 8)", hA2 not in S.BAD_HANDLES and hB not in S.BAD_HANDLES, "handles %#x %#x" % (hA2, hB))
    check("re-accept A at same IPA", rm.mem_accept(hA2, ACCEPT_IPA, size) == 0)
    check("accept B (RO) at %#x" % (ACCEPT_IPA + size), rm.mem_accept(hB, ACCEPT_IPA + size, 1 << 20) == 0)
    expect_errno("UNSHARE A -> EBUSY", lambda: S.unshare(vm, 7), errno.EBUSY)
    expect_errno("UNSHARE B -> EBUSY", lambda: S.unshare(vm, 8), errno.EBUSY)
    dmem[1 << 20:(1 << 20) + 8] = struct.pack("<Q", 0xC0FFEE0000000003)
    check("dmabuf slice still live at teardown time", g.dump(DIPA, 8) == dmem[1 << 20:(1 << 20) + 8])
    st = g.off()
    check("guest SYSTEM_OFF", st == "EXITED", st)
    expect_errno("UNSHARE A after guest exit, no reset yet -> EBUSY", lambda: S.unshare(vm, 7), errno.EBUSY)
    run.close(); os.close(vcpu)
    t0 = time.monotonic(); os.close(vm); dt = (time.monotonic() - t0) * 1000
    check("locked_vm back to baseline after close (reset released the accepts)", S.vmlck_kb() == lck_base, "%d kB vs %d kB, close %.1f ms" % (S.vmlck_kb(), lck_base, dt))
    os.close(dev)
    print("%d failure(s), %d vCPU exits" % (S.fails, g.exits))
    return S.fails

if __name__ == "__main__":
    sys.exit(min(main(), 125))
