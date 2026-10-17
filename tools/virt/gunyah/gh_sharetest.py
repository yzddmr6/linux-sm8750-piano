#!/usr/bin/env python3
"""Host-side test of GH_VM_PROTECTED_SHARE / _SHARE_DMABUF / _UNSHARE on a running
protected VM. The guest here is the gh_vmtest PSCI_OFF stub, so nothing accepts:
this checks argument validation, handle issue, label bookkeeping, pin accounting,
the DMA-BUF (udmabuf) source and the reclaim of live shares at VM fd close.
Run as root (needs /dev/gunyah and /dev/udmabuf). Exit status = number of failures.
"""
import ctypes, errno, fcntl, mmap, os, struct, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gh_vmtest as T

def _IOWR(t, nr, size): return T._IOC(3, t, nr, size)
GH_VM_PROTECTED_SHARE        = _IOWR("A", 0x18, 40)
GH_VM_PROTECTED_SHARE_DMABUF = _IOWR("A", 0x19, 40)
GH_VM_PROTECTED_UNSHARE      = T._IOW("A", 0x1a, 8)
GH_CAP_PROTECTED_SHARE = 1
UDMABUF_CREATE = T._IOW("u", 0x42, 24)
MEM_RW, MEM_R = 0x3, 0x1
BAD_HANDLES = (0, 0xffffffff)

fails = 0
def check(name, cond, detail=""):
    global fails
    print("  [%s] %s %s" % ("ok " if cond else "FAIL", name, detail))
    if not cond: fails += 1

def expect_errno(name, fn, err):
    try:
        fn(); check(name, False, "unexpectedly succeeded")
    except OSError as e:
        check(name, e.errno == err, "errno %d (%s), want %d" % (e.errno, e.strerror, err))

def share(vm, label, flags, uaddr, size, reserved=0):
    buf = bytearray(struct.pack("<IIQQQII", label, flags, 0, size, uaddr, 0, reserved))
    fcntl.ioctl(vm, GH_VM_PROTECTED_SHARE, buf, True)
    return struct.unpack_from("<I", buf, 32)[0]

def share_dmabuf(vm, label, flags, fd, off, size):
    buf = bytearray(struct.pack("<IIQQQiI", label, flags, 0, size, off, fd, 0))
    fcntl.ioctl(vm, GH_VM_PROTECTED_SHARE_DMABUF, buf, True)
    return struct.unpack_from("<I", buf, 36)[0]

def unshare(vm, label, reserved=0):
    fcntl.ioctl(vm, GH_VM_PROTECTED_UNSHARE, struct.pack("<II", label, reserved))

def vmlck_kb():
    """account_locked_vm() charges mm->locked_vm, reported as VmLck"""
    for l in open("/proc/self/status"):
        if l.startswith("VmLck:"): return int(l.split()[1])
    return -1

def anon(size):
    m = mmap.mmap(-1, size, flags=mmap.MAP_PRIVATE | mmap.MAP_ANONYMOUS, prot=mmap.PROT_READ | mmap.PROT_WRITE)
    m[0:size] = bytes(size)
    return m, ctypes.addressof(ctypes.c_char.from_buffer(m))

def run_guest_to_exit(vcpu, run):
    for i in range(8):
        fcntl.ioctl(vcpu, T.GUNYAH_VCPU_RUN)
        reason, = struct.unpack_from("<I", run, 8)
        if reason == 2:
            st, = struct.unpack_from("<I", run, 16)
            return {1: "LOAD_FAILED", 2: "EXITED", 3: "CRASHED"}.get(st, st)
        if reason == 1:
            struct.pack_into("<B", run, 16 + 8 + 8 + 4 + 1, 0)
        elif reason == 3:
            struct.pack_into("<B", run, 16 + 8 + 4, 1)
    return "no STATUS exit"

def main():
    RAM_BASE, RAM_SIZE = 0x80000000, 0x400000
    DTB_OFF, DTB_SIZE = 0x200000, 0x10000
    GICD, GICR = 0x3FFF0000, 0x3FFD0000

    lck_base = vmlck_kb()   # before any VM memory exists; the VM's own parcels charge locked_vm too
    dev = os.open("/dev/gunyah", os.O_RDWR)
    vm = fcntl.ioctl(dev, T.GUNYAH_CREATE_VM)
    print("[0] VM fd %d" % vm)
    check("CHECK_EXTENSION(PROTECTED_SHARE) == 1", fcntl.ioctl(vm, T.GH_VM_CHECK_EXTENSION, GH_CAP_PROTECTED_SHARE) == 1)

    bufA, addrA = anon(4 << 20)
    bufB, addrB = anon(1 << 20)
    expect_errno("share before VM_START -> ENXIO", lambda: share(vm, 7, MEM_RW, addrA, 4 << 20), errno.ENXIO)

    # Protected VM: lent image whose payload is PSCI SYSTEM_OFF.
    mem, uaddr = anon(RAM_SIZE)
    code = struct.pack("<%dI" % len(T.PSCI_OFF), *T.PSCI_OFF); mem[0:len(code)] = code
    dtb = T.build_dtb(RAM_BASE, RAM_SIZE, GICD, GICR); mem[DTB_OFF:DTB_OFF + len(dtb)] = dtb
    fcntl.ioctl(vm, T.GH_VM_ANDROID_LEND_USER_MEM, struct.pack("<IIQQQ", 0, T.MEM_RWX, RAM_BASE, RAM_SIZE, uaddr))
    fcntl.ioctl(vm, T.GUNYAH_VM_SET_DTB_CONFIG, struct.pack("<QQ", RAM_BASE + DTB_OFF, DTB_SIZE))
    fcntl.ioctl(vm, T.GUNYAH_VM_SET_BOOT_CONTEXT, struct.pack("<IIQ", T.REG_SET_PC, 0, RAM_BASE))
    vcpu_arg = ctypes.c_uint32(0)
    vcpu = fcntl.ioctl(vm, T.GUNYAH_VM_ADD_FUNCTION, bytearray(struct.pack("<IIQ", T.GUNYAH_FN_VCPU, 4, ctypes.addressof(vcpu_arg))), True)
    run = mmap.mmap(vcpu, fcntl.ioctl(vcpu, T.GUNYAH_VCPU_MMAP_SIZE), flags=mmap.MAP_SHARED, prot=mmap.PROT_READ | mmap.PROT_WRITE)
    fcntl.ioctl(vm, T.GUNYAH_VM_START)
    print("[1] protected VM started (RUNNING, vCPU not yet run)")

    pin0 = vmlck_kb()
    print("[2] argument validation")
    expect_errno("unaligned size -> EINVAL", lambda: share(vm, 7, MEM_RW, addrA, 4096 + 1), errno.EINVAL)
    expect_errno("zero size -> EINVAL", lambda: share(vm, 7, MEM_RW, addrA, 0), errno.EINVAL)
    expect_errno("unknown flag -> EINVAL", lambda: share(vm, 7, 0x100, addrA, 4096), errno.EINVAL)
    expect_errno("reserved != 0 -> EINVAL", lambda: share(vm, 7, MEM_RW, addrA, 4096, reserved=1), errno.EINVAL)
    expect_errno("unmapped VA -> EFAULT", lambda: share(vm, 7, MEM_RW, 0x10000, 4096), errno.EFAULT)
    expect_errno("unshare unknown label -> ENOENT", lambda: unshare(vm, 99), errno.ENOENT)

    print("[3] userspace buffer shares")
    t0 = time.monotonic(); hA = share(vm, 7, MEM_RW, addrA, 4 << 20); dt = (time.monotonic() - t0) * 1000
    check("share A 4 MiB label 7 RW", hA not in BAD_HANDLES, "handle %#x (%.2f ms)" % (hA, dt))
    check("VmLck +4 MiB", vmlck_kb() - pin0 == 4096, "%d kB" % (vmlck_kb() - pin0))
    expect_errno("duplicate label 7 -> EEXIST", lambda: share(vm, 7, MEM_RW, addrB, 1 << 20), errno.EEXIST)
    check("VmLck unchanged after refused share", vmlck_kb() - pin0 == 4096, "%d kB" % (vmlck_kb() - pin0))
    hB = share(vm, 8, MEM_R, addrB, 1 << 20)
    check("share B 1 MiB label 8 RO", hB not in BAD_HANDLES and hB != hA, "handle %#x" % hB)
    unshare(vm, 7); check("unshare A (guest never accepted)", True)
    check("VmLck -4 MiB after unshare", vmlck_kb() - pin0 == 1024, "%d kB" % (vmlck_kb() - pin0))
    hA2 = share(vm, 7, MEM_RW, addrA, 4 << 20)
    check("re-share A under label 7", hA2 not in BAD_HANDLES, "handle %#x" % hA2)

    print("[4] DMA-BUF (udmabuf over a sealed memfd)")
    try:
        mfd = os.memfd_create("gh-share", os.MFD_ALLOW_SEALING)
        os.ftruncate(mfd, 2 << 20)
        fcntl.fcntl(mfd, fcntl.F_ADD_SEALS, fcntl.F_SEAL_SHRINK)
        with mmap.mmap(mfd, 2 << 20) as mm: mm[0:2 << 20] = bytes(2 << 20)
        ud = os.open("/dev/udmabuf", os.O_RDWR)
        dbuf = fcntl.ioctl(ud, UDMABUF_CREATE, bytearray(struct.pack("<IIQQ", mfd, 1, 0, 2 << 20)), True)
        expect_errno("slice past end -> EINVAL", lambda: share_dmabuf(vm, 9, MEM_RW, dbuf, 1 << 20, 2 << 20), errno.EINVAL)
        expect_errno("wrapping offset -> EOVERFLOW", lambda: share_dmabuf(vm, 9, MEM_RW, dbuf, (1 << 64) - 4096, 8192), errno.EOVERFLOW)
        expect_errno("non-dmabuf fd -> EINVAL", lambda: share_dmabuf(vm, 9, MEM_RW, mfd, 0, 4096), errno.EINVAL)
        hD = share_dmabuf(vm, 9, MEM_RW, dbuf, 1 << 20, 1 << 20)
        check("share dmabuf slice [+1 MiB, 1 MiB) label 9", hD not in BAD_HANDLES, "handle %#x" % hD)
        unshare(vm, 9); check("unshare dmabuf", True)
        hD = share_dmabuf(vm, 9, MEM_RW, dbuf, 0, 2 << 20)
        check("re-share whole dmabuf label 9", hD not in BAD_HANDLES, "handle %#x" % hD)
        check("VmLck unaffected by dmabuf shares", vmlck_kb() - pin0 == 5120, "%d kB" % (vmlck_kb() - pin0))
    except OSError as e:
        check("dmabuf path", False, "setup failed: %s" % e)

    print("[5] guest runs to SYSTEM_OFF with labels 7, 8, 9 still shared")
    st = run_guest_to_exit(vcpu, run)
    check("guest exit STATUS EXITED", st == "EXITED", st)
    expect_errno("share after guest exit -> ENXIO", lambda: share(vm, 10, MEM_RW, addrB, 4096), errno.ENXIO)

    print("[6] close VM fd with live shares -> teardown reclaims")
    run.close(); os.close(vcpu)
    t0 = time.monotonic(); os.close(vm); dt = (time.monotonic() - t0) * 1000
    check("VmLck back to pre-VM baseline after close", vmlck_kb() == lck_base, "%d kB vs %d kB (close %.1f ms)" % (vmlck_kb(), lck_base, dt))
    os.close(dev)
    print("%d failure(s)" % fails)
    return fails

if __name__ == "__main__":
    sys.exit(min(main(), 125))
