/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM camss

#if !defined(_CAMSS_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _CAMSS_TRACE_H

#include <linux/tracepoint.h>

/* phase: 0 SOF, 1 EOF, 2 DMA completion. Preserve the raw CSID counter
 * and the CNTPCT-before / MONOTONIC / CNTPCT-after cross-stamp so that the
 * conversion and IRQ latency remain independently checkable on hardware.
 */
TRACE_EVENT(camss_frame,
	TP_PROTO(unsigned int hw, unsigned int line, u32 sequence,
		 unsigned int phase, u64 ticks, u64 previous, u64 arch_ticks,
		 u64 arch_after, u64 irq_ns,
		 int buffer, bool valid),
	TP_ARGS(hw, line, sequence, phase, ticks, previous, arch_ticks,
		arch_after, irq_ns, buffer, valid),
	TP_STRUCT__entry(
		__field(unsigned int, hw)
		__field(unsigned int, line)
		__field(u32, sequence)
		__field(unsigned int, phase)
		__field(u64, ticks)
		__field(u64, previous)
		__field(u64, arch_ticks)
		__field(u64, arch_after)
		__field(u64, irq_ns)
		__field(int, buffer)
		__field(bool, valid)
	),
	TP_fast_assign(
		__entry->hw = hw;
		__entry->line = line;
		__entry->sequence = sequence;
		__entry->phase = phase;
		__entry->ticks = ticks;
		__entry->previous = previous;
		__entry->arch_ticks = arch_ticks;
		__entry->arch_after = arch_after;
		__entry->irq_ns = irq_ns;
		__entry->buffer = buffer;
		__entry->valid = valid;
	),
	TP_printk("hw=%u line=%u seq=%u phase=%u qtimer=%llu prev=%llu counter=%llu counter_after=%llu mono=%llu buffer=%d valid=%u",
		  __entry->hw, __entry->line, __entry->sequence, __entry->phase,
		  __entry->ticks, __entry->previous, __entry->arch_ticks,
		  __entry->arch_after, __entry->irq_ns,
		  __entry->buffer, __entry->valid)
);

TRACE_EVENT(camss_frame_error,
	TP_PROTO(unsigned int hw, unsigned int line, u32 sequence,
		 u32 status, u64 irq_ns),
	TP_ARGS(hw, line, sequence, status, irq_ns),
	TP_STRUCT__entry(
		__field(unsigned int, hw)
		__field(unsigned int, line)
		__field(u32, sequence)
		__field(u32, status)
		__field(u64, irq_ns)
	),
	TP_fast_assign(
		__entry->hw = hw;
		__entry->line = line;
		__entry->sequence = sequence;
		__entry->status = status;
		__entry->irq_ns = irq_ns;
	),
	TP_printk("hw=%u line=%u seq=%u status=0x%08x mono=%llu",
		  __entry->hw, __entry->line, __entry->sequence,
		  __entry->status, __entry->irq_ns)
);

#endif

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE camss-trace
#include <trace/define_trace.h>
