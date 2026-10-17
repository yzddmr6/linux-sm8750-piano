/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * gh_rmproxy: minimal bare-metal guest for gh_accepttest.py.
 *
 * Runs at EL1 with the MMU off and executes commands the host feeds through
 * MMIO exits at MMIO_BASE (unmapped in the guest). Every protocol decision --
 * DTB parsing, RM RPC framing -- lives in the host test; the guest only moves
 * bytes and issues the two msgq hypercalls. Cache maintenance brackets every
 * memory access because MMU-off data accesses bypass the caches while the
 * host's lines for the shared buffer sit in them: clean before a load so the
 * host's dirty data reaches memory, clean+invalidate around a store so no
 * dirty host line is written back over it afterwards.
 *
 *   CMD  = MMIO_BASE + 0: guest reads the next command
 *   DATA = MMIO_BASE + 8: guest reads parameters / writes results, 8 bytes each
 *
 *   1 DUMP addr count : count x (ldr [addr], str DATA)
 *   2 LOAD addr count : count x (ldr DATA, str [addr]); then DATA <- 0 as ack
 *   3 SEND cap size buf : msgq_send(cap, size, buf, PUSH); DATA <- x0
 *   4 RECV cap buf size : msgq_recv(cap, buf, size);  DATA <- x0, DATA <- x1 (len)
 *   5 OFF              : PSCI SYSTEM_OFF
 *
 * Build: clang --target=aarch64-linux-gnu -c gh_rmproxy.S -o gh_rmproxy.o &&
 *        llvm-objcopy -O binary -j .text gh_rmproxy.o gh_rmproxy.bin
 */
#define MMIO_BASE_HI	0x1000		/* 0x10000000 */
#define HCALL_MSGQ_SEND	0x801B		/* | 0xC6000000: fast SMC64, vendor hyp */
#define HCALL_MSGQ_RECV	0x801C
#define PSCI_SYSTEM_OFF	0x84000008

	.text
	.globl _start
_start:
	movz	x20, #MMIO_BASE_HI, lsl #16
loop:
	ldr	x0, [x20]
	cmp	x0, #1
	b.eq	do_dump
	cmp	x0, #2
	b.eq	do_load
	cmp	x0, #3
	b.eq	do_send
	cmp	x0, #4
	b.eq	do_recv
	cmp	x0, #5
	b.eq	do_off
	b	loop

do_dump:
	ldr	x1, [x20, #8]
	ldr	x2, [x20, #8]
1:	cbz	x2, loop
	dc	civac, x1
	dsb	sy
	ldr	x3, [x1]
	str	x3, [x20, #8]
	add	x1, x1, #8
	sub	x2, x2, #1
	b	1b

do_load:
	ldr	x1, [x20, #8]
	ldr	x2, [x20, #8]
1:	cbz	x2, 2f
	ldr	x3, [x20, #8]
	dc	civac, x1		/* a dirty host line must not be written back over our store */
	dsb	sy
	str	x3, [x1]
	dc	civac, x1		/* and no stale clean copy may survive for the host to hit */
	dsb	sy
	add	x1, x1, #8
	sub	x2, x2, #1
	b	1b
2:	str	xzr, [x20, #8]		/* ack: the host may now read what we stored */
	b	loop

do_send:
	ldr	x1, [x20, #8]		/* cap */
	ldr	x2, [x20, #8]		/* size */
	ldr	x3, [x20, #8]		/* buf */
	mov	x4, #1			/* GH_MSGQ_TX_PUSH */
	mov	x5, #0
	movz	x0, #HCALL_MSGQ_SEND
	movk	x0, #0xC600, lsl #16
	hvc	#0
	str	x0, [x20, #8]
	b	loop

do_recv:
	ldr	x1, [x20, #8]		/* cap */
	ldr	x2, [x20, #8]		/* buf */
	ldr	x3, [x20, #8]		/* size */
	mov	x4, #0
	movz	x0, #HCALL_MSGQ_RECV
	movk	x0, #0xC600, lsl #16
	hvc	#0
	str	x0, [x20, #8]
	str	x1, [x20, #8]
	b	loop

do_off:
	movz	x0, #(PSCI_SYSTEM_OFF >> 16), lsl #16
	movk	x0, #(PSCI_SYSTEM_OFF & 0xffff)
	hvc	#0
	b	.
