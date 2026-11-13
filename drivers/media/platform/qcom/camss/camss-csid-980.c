// SPDX-License-Identifier: GPL-2.0
/*
 * camss-csid-980.c
 *
 * Qualcomm MSM Camera Subsystem - CSID (CSI Decoder) Module
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/of.h>
#include <linux/timekeeping.h>
#include <clocksource/arm_arch_timer.h>
#ifdef CONFIG_ARM64
#include <asm/arch_timer.h>
#endif
#include "camss.h"
#include "camss-csid.h"
#include "camss-csid-gen3.h"
#include "camss-isp-debug.h"

/* Reset and Command Registers */
#define CSID_RST_CFG				0xC
#define		RST_MODE				BIT(0)
#define		RST_LOCATION				BIT(4)

/* Reset and Command Registers */
#define CSID_RST_CMD				0x10
#define		SELECT_HW_RST				BIT(0)
#define		SELECT_IRQ_RST				BIT(2)
#define CSID_IRQ_CMD				0x14
#define		IRQ_CMD_CLEAR				BIT(0)

/* Register Update Commands, RUP/AUP */
#define CSID_RUP_CMD				0x18
#define CSID_AUP_CMD				0x1C
#define		CSID_RUP_AUP_RDI(rdi)			(BIT(8) << (rdi))
#define CSID_RUP_AUP_CMD			0x20
#define		RUP_SET					BIT(0)
#define		MUP					BIT(4)

#define CSID_LITE_RUP_AUP_CMD			0x18
#define		CSID_LITE_RUP_RDI(rdi)		(BIT(4) << (rdi))
#define		CSID_LITE_AUP_RDI(rdi)		(BIT(20) << (rdi))

/* Top level interrupt registers */
#define CSID_TOP_IRQ_STATUS			(csid_is_lite(csid) ? 0x7C : 0x84)
#define CSID_TOP_IRQ_MASK			(csid_is_lite(csid) ? 0x80 : 0x88)
#define CSID_TOP_IRQ_CLEAR			(csid_is_lite(csid) ? 0x84 : 0x8C)
#define CSID_TOP_IRQ_SET			(csid_is_lite(csid) ? 0x88 : 0x90)
#define		INFO_RST_DONE				BIT(0)
#define		CSI2_RX_IRQ_STATUS			BIT(2)
#define		BUF_DONE_IRQ_STATUS			BIT(csid_is_lite(csid) ? 13 : 3)

/* Buffer done interrupt registers */
#define CSID_BUF_DONE_IRQ_STATUS		(csid_is_lite(csid) ? 0x8C : 0xA4)
#define		BUF_DONE_IRQ_STATUS_RDI_OFFSET		(csid_is_lite(csid) ? 1 : 16)
#define CSID_BUF_DONE_IRQ_MASK			(csid_is_lite(csid) ? 0x90 : 0xA8)
#define CSID_BUF_DONE_IRQ_CLEAR			(csid_is_lite(csid) ? 0x94 : 0xAC)
#define CSID_BUF_DONE_IRQ_SET			(csid_is_lite(csid) ? 0x98 : 0xB0)

/* CSI2 RX interrupt registers */
#define CSID_CSI2_RX_IRQ_STATUS			(csid_is_lite(csid) ? 0x9C : 0xB4)
#define CSID_CSI2_RX_IRQ_MASK			(csid_is_lite(csid) ? 0xA0 : 0xB8)
#define CSID_CSI2_RX_IRQ_CLEAR			(csid_is_lite(csid) ? 0xA4 : 0xBC)
#define CSID_CSI2_RX_IRQ_SET			(csid_is_lite(csid) ? 0xA8 : 0xC0)

/* CSI2 RX Configuration */
#define CSID_CSI2_RX_CFG0			(csid_is_lite(csid) ? 0x200 : 0x400)
#define		CSI2_RX_CFG0_NUM_ACTIVE_LANES		0
#define		CSI2_RX_CFG0_PHY_SEL_BASE_IDX		1
#define		CSI2_RX_CFG0_DL0_INPUT_SEL		4
#define		CSI2_RX_CFG0_PHY_NUM_SEL		20
#define		CSI2_RX_CFG0_TPG_MUX_EN			BIT(27)
#define		CSI2_RX_CFG0_TPG_MUX_SEL		GENMASK(29, 28)
#define CSID_CSI2_RX_CFG1			(csid_is_lite(csid) ? 0x204 : 0x404)
#define		CSI2_RX_CFG1_ECC_CORRECTION_EN		BIT(0)
#define		CSI2_RX_CFG1_VC_MODE			BIT(2)

#define MSM_CSID_MAX_SRC_STREAMS_980		(csid_is_lite(csid) ? 4 : 5)

#define CSID_RDI_CFG0(rdi)                                  \
	({                                                  \
		__typeof__(rdi) _rdi = (rdi);               \
		csid_is_lite(csid) ? 0x500 + 0x100 * _rdi : \
				     0xE00 + 0x200 * _rdi;  \
	})
#define		RDI_CFG0_RETIME_BS			BIT(5)
#define		RDI_CFG0_TIMESTAMP_EN			BIT(6)
#define		RDI_CFG0_TIMESTAMP_STB_SEL		BIT(8)
#define		RDI_CFG0_DECODE_FORMAT			12
#define		RDI_CFG0_DT				16
#define		RDI_CFG0_VC				22
#define		RDI_CFG0_DT_ID				27
#define		RDI_CFG0_EN				BIT(31)

/* RDI Control and Configuration */
#define CSID_RDI_CTRL(rdi)                                  \
	({                                                  \
		__typeof__(rdi) _rdi = (rdi);               \
		csid_is_lite(csid) ? 0x504 + 0x100 * _rdi : \
				     0xE04 + 0x200 * _rdi;  \
	})
#define		RDI_CTRL_START_CMD			BIT(0)

#define CSID_RDI_CFG1(rdi)                                  \
	({                                                  \
		__typeof__(rdi) _rdi = (rdi);               \
		csid_is_lite(csid) ? 0x510 + 0x100 * _rdi : \
				     0xE10 + 0x200 * _rdi;  \
	})
#define		RDI_CFG1_DROP_H_EN			BIT(5)
#define		RDI_CFG1_DROP_V_EN			BIT(6)
#define		RDI_CFG1_CROP_H_EN			BIT(7)
#define		RDI_CFG1_CROP_V_EN			BIT(8)
#define		RDI_CFG1_PACKING_FORMAT_MIPI		BIT(15)

/* RDI Pixel Store Configuration */
#define CSID_RDI_PIX_STORE_CFG0(rdi)		(0xE14 + 0x200 * (rdi))
#define		RDI_PIX_STORE_CFG0_EN			BIT(0)
#define		RDI_PIX_STORE_CFG0_MIN_HBI		1

/*
 * IPP (pixel path) of the full CSIDs, fed to the TFE pixel pipeline.  The
 * fourth CSID source pad (port 3) drives the IPP, linked to the VFE PIX
 * line; RDI3 is not exposed.  The TFE pixel pipeline is multi-context;
 * everything here runs in hardware context 0.  CTXT_CFG selects the
 * context whose banked registers (0x1800...) the following writes reach,
 * and CAMIF_FRAME_CFG[4:0] tags the frames with their context; both take a
 * context index, while the IRQ composition takes context masks.
 * CAMIF_FRAME_CFG[26:24] tells the TFE the Bayer order of the frames.
 */
#define CSID_IPP_PORT				3
#define CSID_IPP_CFG0				0x600
#define CSID_IPP_CTRL				0x604
#define CSID_IPP_CAMIF_FRAME_CFG		0x638
#define		CAMIF_FRAME_CFG_CTXT			0
#define		CAMIF_FRAME_CFG_CFA			24
#define			IPP_CFA_RGGB				0
#define			IPP_CFA_GRBG				1
#define			IPP_CFA_BGGR				2
#define			IPP_CFA_GBRG				3
#define CSID_IPP_IRQ_SUBSAMPLE_PATTERN		0x688
#define CSID_IPP_IRQ_SUBSAMPLE_PERIOD		0x68C
#define CSID_IPP_CTXT_CFG			0x3F0
#define CSID_IPP_CFG1				0x1800
#define CSID_IPP_PIX_STORE_CFG			0x1804
#define		IPP_PIX_STORE_EN			BIT(0)
#define		IPP_PIX_STORE_MIN_HBI			1
#define CSID_IPP_IRQ_COMP_CFG0			0x178
#define		IPP_SRC_CTXT_MASK_SHIFT			4
#define		IPP_DST_CTXT_MASK_SHIFT			0
#define CSID_IPP_IRQ_STATUS			0xD4
#define CSID_IPP_IRQ_MASK			0xD8
#define CSID_IPP_IRQ_CLEAR			0xDC
#define CSID_IPP_TOP_IRQ				BIT(8)
#define CSID_PATH_SOF_IRQ				BIT(4)
#define CSID_PATH_EOF_IRQ				BIT(3)
/* Official CSID980 IPP and RDI0 path error masks, excluding SOF/EOF/RUP. */
#define CSID_PATH_FATAL_IRQS			0x241C6001
#define CSID_PATH_NON_FATAL_IRQS			0x12000000
#define CSID_PATH_ERROR_IRQS			(CSID_PATH_FATAL_IRQS | CSID_PATH_NON_FATAL_IRQS)
/* The official CSID980 table selects QTimer with 2 << 8. */
#define CSID_TIMESTAMP_STROBE			(2 << 8)
#define CSID_IPP_CURRENT_SOF			0x6A8
#define CSID_IPP_PREVIOUS_SOF			0x6B0
#define CSID_IPP_CURRENT_EOF			0x6B8
#define CSID_IPP_PREVIOUS_EOF			0x6C0
/* These RDI0 offsets are listed independently in the official CSID980 table;
 * they are not IPP offsets shifted to a presumed RDI address range.
 */
#define CSID_RDI0_CURRENT_SOF			0xEA0
#define CSID_RDI0_PREVIOUS_SOF			0xEA8
#define CSID_RDI0_CURRENT_EOF			0xEB0
#define CSID_RDI0_PREVIOUS_EOF			0xEB8
#define CSID_RDI0_IRQ_MASK			0x118
#define CSID_RDI0_IRQ_CLEAR			0x11C
#define CSID_RDI0_TOP_IRQ			BIT(16)
#define CSID_RDI0_IRQ_SUBSAMPLE_PATTERN		0xE50
#define CSID_RDI0_IRQ_SUBSAMPLE_PERIOD		0xE54
#define CSID_RUP_AUP_IPP			BIT(0)
#define		IPP_CTXT_ID				0
/* TFE bus composite group 1 (FD_Y + FD_C) done, in CSID_BUF_DONE_IRQ_* */
#define BUF_DONE_IRQ_STATUS_IPP_FD		BIT(3)

#define csid_port_is_ipp(csid, port) \
	(!csid_is_lite(csid) && (port) == CSID_IPP_PORT)

static bool csid_rdi_frame_timing(struct csid_device *csid, unsigned int port)
{
	return IS_ENABLED(CONFIG_ARM64) && csid->camss->res->version == CAMSS_8750 &&
	       !csid_is_lite(csid) && port == 0;
}

/* The CSID QTimer strobe uses the 19.2 MHz architectural physical counter.
 * Keep each conversion local to its IRQ: CLOCK_MONOTONIC may be NTP-slewed,
 * while CNTPCT and the receiver latch retain their nominal tick frequency.
 */
#define CSID_COUNTER_RATE                   19200000U
#define CSID_COUNTER_MAX_BRACKET            (CSID_COUNTER_RATE / 20000)
#define CSID_COUNTER_MAX_AGE                CSID_COUNTER_RATE

/* Cross-stamp P_before / ktime_get_ns() / P_after. The midpoint is an
 * estimate inside that measured bracket, not an average IRQ delay.
 * Reject brackets wider than 50 us and events older than one second.
 * At normal NTP rates (+/-500 ppm), bracket, quantisation and rate error
 * then total less than 550 us. This is not nanosecond absolute accuracy.
 */
static u64 csid_timestamp_ns(u64 ticks, u64 before, u64 after, u64 mono)
{
	u64 midpoint, age, elapsed_ns;

	if (!ticks || after < before || ticks > before ||
	    after - before > CSID_COUNTER_MAX_BRACKET)
		return 0;
	midpoint = before + (after - before) / 2;
	age = midpoint - ticks;
	if (age > CSID_COUNTER_MAX_AGE)
		return 0;
	elapsed_ns = div_u64(age * NSEC_PER_SEC, CSID_COUNTER_RATE);
	if (elapsed_ns >= mono)
		return 0;
	return mono - elapsed_ns;
}

struct csid_frame_timestamps {
	u64 sof, previous_sof, eof, previous_eof;
	u64 arch_ticks, arch_after, irq_ns;
};

static u64 csid_read_timestamp(struct csid_device *csid, u32 offset)
{
	u32 high, low, after;

	do {
		high = readl(csid->base + offset + 4);
		low = readl(csid->base + offset);
		after = readl(csid->base + offset + 4);
	} while (high != after);
	return (u64)high << 32 | low;
}

static bool csid_read_frame_timestamps(struct csid_device *csid, unsigned int port,
				     struct csid_frame_timestamps *time)
{
	unsigned int attempt;
	u32 sof, previous_sof, eof, previous_eof;

	if (!IS_ENABLED(CONFIG_ARM64) || csid->camss->res->version != CAMSS_8750)
		return false;
#ifdef CONFIG_ARM64
	if (arch_timer_get_cntfrq() != CSID_COUNTER_RATE)
		return false;
#endif
	if (csid_port_is_ipp(csid, port)) {
		sof = CSID_IPP_CURRENT_SOF;
		previous_sof = CSID_IPP_PREVIOUS_SOF;
		eof = CSID_IPP_CURRENT_EOF;
		previous_eof = CSID_IPP_PREVIOUS_EOF;
	} else if (csid_rdi_frame_timing(csid, port)) {
		sof = CSID_RDI0_CURRENT_SOF;
		previous_sof = CSID_RDI0_PREVIOUS_SOF;
		eof = CSID_RDI0_CURRENT_EOF;
		previous_eof = CSID_RDI0_PREVIOUS_EOF;
	} else {
		return false;
	}

	/* The current and previous registers latch at frame boundaries. Check
	 * that neither current value changed while reading its previous value.
	 */
	for (attempt = 0; attempt < 3; attempt++) {
		time->sof = csid_read_timestamp(csid, sof);
		time->previous_sof = csid_read_timestamp(csid, previous_sof);
		time->eof = csid_read_timestamp(csid, eof);
		time->previous_eof = csid_read_timestamp(csid, previous_eof);
		if (time->sof == csid_read_timestamp(csid, sof) &&
		    time->eof == csid_read_timestamp(csid, eof)) {
#ifdef CONFIG_ARM64
			/* QTimer and CNTPCT share the physical counter on SM8750.
			 * Do not use CNTVCT: its hypervisor offset is independent.
			 */
			time->arch_ticks = __arch_counter_get_cntpct_stable();
			time->irq_ns = ktime_get_ns();
			time->arch_after = __arch_counter_get_cntpct_stable();
			return true;
#else
			return false;
#endif
		}
	}
	return false;
}

static void csid_frame_irq(struct csid_device *csid, int port, u32 status)
{
	struct csid_frame_timestamps time = { 0 };
	u64 sof_ns, eof_ns;
	bool sof = status & CSID_PATH_SOF_IRQ;
	bool eof = status & CSID_PATH_EOF_IRQ;

	/* Frame drop/overflow can leave plausible SOF/EOF ticks. Invalidate
	 * before publishing any markers or completing this IRQ's DMA buffer.
	 * This must also run when the IRQ contains no SOF or EOF at all.
	 */
	if (status & CSID_PATH_ERROR_IRQS)
		camss_frame_error(csid->camss, csid->id, port,
				  status, ktime_get_ns());
	if (!sof && !eof)
		return;
	if (!csid_read_frame_timestamps(csid, port, &time)) {
		/* Zero ticks invalidate the timeline instead of inventing a time. */
		time = (struct csid_frame_timestamps){ .irq_ns = ktime_get_ns() };
	}

	sof_ns = csid_timestamp_ns(time.sof, time.arch_ticks,
				       time.arch_after, time.irq_ns);
	eof_ns = csid_timestamp_ns(time.eof, time.arch_ticks,
				       time.arch_after, time.irq_ns);

	/* A delayed IRQ can contain EOF(n) and SOF(n+1). Dispatch by the
	 * hardware timestamps, before handling the same IRQ's DMA completion.
	 */
	if (eof && (!sof || time.eof < time.sof))
		camss_frame_sync(csid->camss, csid->id, port, false,
				 time.eof, time.previous_eof, time.arch_ticks,
				 time.arch_after, time.irq_ns, eof_ns);
	if (sof)
		camss_frame_sync(csid->camss, csid->id, port, true,
				 time.sof, time.previous_sof, time.arch_ticks,
				 time.arch_after, time.irq_ns, sof_ns);
	if (eof && sof && time.eof >= time.sof)
		camss_frame_sync(csid->camss, csid->id, port, false,
				 time.eof, time.previous_eof, time.arch_ticks,
				 time.arch_after, time.irq_ns, eof_ns);
}

/* RDI IRQ Status in wrapper */
#define CSID_CSI2_RDIN_IRQ_STATUS(rdi) \
	(csid_is_lite(csid) ? 0xEC : 0x114 + 0x10 * (rdi))
#define CSID_CSI2_RDIN_IRQ_CLEAR(rdi) \
	(csid_is_lite(csid) ? 0xF4 : 0x11C + 0x10 * (rdi))
#define		INFO_INPUT_SOF				BIT(12)
#define		INFO_RUP_DONE				BIT(23)
/* Full CSID only: per-RDI interrupt mask and its summary bit in the top mask. */
#define CSID_CSI2_RDIN_IRQ_MASK(rdi)		(0x118 + 0x10 * (rdi))
#define		TOP_IRQ_RDI(rdi)			BIT(16 + (rdi))

static void __csid_full_aup_rup_trigger(struct csid_device *csid)
{
	/* trigger SET in combined register */
	writel(RUP_SET, csid->base + CSID_RUP_AUP_CMD);
}

static void __csid_aup_update(struct csid_device *csid, int port_id)
{
	if (csid_is_lite(csid)) {
		/* CSID Lites in v980 follow the legacy way of a combined RUP
		 * and AUP commands without an explicit SET register.
		 */
		csid->reg_update |= CSID_LITE_AUP_RDI(port_id);
		writel(csid->reg_update, csid->base + CSID_LITE_RUP_AUP_CMD);
	} else {
		csid->aup_update |= csid_port_is_ipp(csid, port_id) ?
				    CSID_RUP_AUP_IPP : CSID_RUP_AUP_RDI(port_id);
		writel(csid->aup_update, csid->base + CSID_AUP_CMD);

		/* CSID Fulls in v980 split AUP and RUP commands, which requires
		 * additional SET operation to make registers modification take
		 * effect.
		 */
		__csid_full_aup_rup_trigger(csid);
	}
}

static void __csid_rup_update(struct csid_device *csid, int port_id)
{
	if (csid_is_lite(csid)) {
		/* CSID Lites in v980 follow the legacy way of a combined RUP
		 * and AUP commands without an explicit SET register.
		 */
		csid->reg_update |= CSID_LITE_RUP_RDI(port_id);
		writel(csid->reg_update, csid->base + CSID_LITE_RUP_AUP_CMD);
	} else {
		csid->rup_update |= csid_port_is_ipp(csid, port_id) ?
				    CSID_RUP_AUP_IPP : CSID_RUP_AUP_RDI(port_id);
		writel(csid->rup_update, csid->base + CSID_RUP_CMD);

		/* CSID Fulls in v980 split AUP and RUP commands, which requires
		 * additional SET operation to make registers modification take
		 * effect.
		 */
		__csid_full_aup_rup_trigger(csid);
	}
}

static void __csid_aup_rup_clear(struct csid_device *csid, int port_id)
{
	/* Hardware clears the registers upon consuming the settings */
	if (csid_is_lite(csid)) {
		csid->reg_update &= ~CSID_LITE_RUP_RDI(port_id);
		csid->reg_update &= ~CSID_LITE_AUP_RDI(port_id);
	} else {
		u32 bit = csid_port_is_ipp(csid, port_id) ?
			  CSID_RUP_AUP_IPP : CSID_RUP_AUP_RDI(port_id);

		csid->aup_update &= ~bit;
		csid->rup_update &= ~bit;
	}
}

static void __csid_configure_rx(struct csid_device *csid,
				struct csid_phy_config *phy)
{
	struct camss *camss = csid->camss;
	int val;

	val = (phy->lane_cnt - 1) << CSI2_RX_CFG0_NUM_ACTIVE_LANES;
	val |= phy->lane_assign << CSI2_RX_CFG0_DL0_INPUT_SEL;

	if (camss->tpg && csid->tpg_linked &&
	    camss->tpg[phy->csiphy_id].testgen.mode != TPG_PAYLOAD_MODE_DISABLED) {
		val |= FIELD_PREP(CSI2_RX_CFG0_TPG_MUX_SEL, phy->csiphy_id + 1);
		val |= CSI2_RX_CFG0_TPG_MUX_EN;
	} else {
		val |= (phy->csiphy_id + CSI2_RX_CFG0_PHY_SEL_BASE_IDX)
			<< CSI2_RX_CFG0_PHY_NUM_SEL;
	}

	writel(val, csid->base + CSID_CSI2_RX_CFG0);

	val = CSI2_RX_CFG1_ECC_CORRECTION_EN;
	writel(val, csid->base + CSID_CSI2_RX_CFG1);
}

static void __csid_configure_rx_vc(struct csid_device *csid, int vc)
{
	int val;

	if (vc > 3) {
		val = readl(csid->base + CSID_CSI2_RX_CFG1);
		val |= CSI2_RX_CFG1_VC_MODE;
		writel(val, csid->base + CSID_CSI2_RX_CFG1);
	}
}

static void __csid_ctrl_rdi(struct csid_device *csid, int enable, u8 rdi)
{
	u32 rdi_ctrl_offset = CSID_RDI_CTRL(rdi);
	int val = 0;

	if (enable)
		val = RDI_CTRL_START_CMD;

	writel(val, csid->base + rdi_ctrl_offset);
}

static void __csid_configure_rdi_pix_store(struct csid_device *csid, u8 rdi)
{
	u32 val;

	/*
	 * Configure pixel store to allow absorption of hblanking or idle time.
	 * This helps with horizontal crop and prevents line buffer conflicts.
	 * Reset state is 0x8 which has MIN_HBI=4, we keep the default MIN_HBI
	 * and just enable the pixel store functionality.
	 */
	val = (4 << RDI_PIX_STORE_CFG0_MIN_HBI) | RDI_PIX_STORE_CFG0_EN;
	writel(val, csid->base + CSID_RDI_PIX_STORE_CFG0(rdi));
}

static void __csid_configure_rdi_stream(struct csid_device *csid, u8 enable, u8 port, u8 vc)
{
	u8 lane_cnt = csid->phy.lane_cnt;
	u32 val;
	bool frame_timing = csid_rdi_frame_timing(csid, port);

	/* Source pads matching RDI channels on hardware.
	 * E.g. Pad 1 -> RDI0, Pad 2 -> RDI1, etc.
	 */
	struct v4l2_mbus_framefmt *input_format = &csid->fmt[MSM_CSID_PAD_FIRST_SRC + port];
	const struct csid_format_info *format = csid_get_fmt_entry(csid->res->formats->formats,
								   csid->res->formats->nformats,
								   input_format->code);

	if (!lane_cnt)
		lane_cnt = 4;

	/*
	 * DT_ID is a two bit bitfield that is concatenated with
	 * the four least significant bits of the five bit VC
	 * bitfield to generate an internal CID value.
	 *
	 * CSID_RDI_CFG0(vc)
	 * DT_ID : 28:27
	 * VC    : 26:22
	 * DT    : 21:16
	 *
	 * CID   : VC 3:0 << 2 | DT_ID 1:0
	 */
	u8 dt_id = vc & 0x03;
	u32 rdi_cfg0_offset = CSID_RDI_CFG0(port);
	u32 rdi_cfg1_offset = CSID_RDI_CFG1(port);
	u32 rdi_ctrl_offset = CSID_RDI_CTRL(port);

	if (frame_timing) {
		writel(0, csid->base + CSID_RDI0_IRQ_MASK);
		val = readl(csid->base + CSID_TOP_IRQ_MASK) & ~CSID_RDI0_TOP_IRQ;
		writel(val, csid->base + CSID_TOP_IRQ_MASK);
	}
	val = RDI_CFG0_TIMESTAMP_EN;
	val |= frame_timing ? CSID_TIMESTAMP_STROBE : RDI_CFG0_TIMESTAMP_STB_SEL;
	val |= RDI_CFG0_RETIME_BS;

	/* note: for non-RDI path, this should be format->decode_format */
	val |= DECODE_FORMAT_PAYLOAD_ONLY << RDI_CFG0_DECODE_FORMAT;
	val |= vc << RDI_CFG0_VC;
	val |= format->data_type << RDI_CFG0_DT;
	val |= dt_id << RDI_CFG0_DT_ID;
	writel(val, csid->base + rdi_cfg0_offset);

	val = RDI_CFG1_PACKING_FORMAT_MIPI;
	writel(val, csid->base + rdi_cfg1_offset);

	/* Configure pixel store using dedicated register in 980 */
	if (!csid_is_lite(csid))
		__csid_configure_rdi_pix_store(csid, port);

	val = 0;
	writel(val, csid->base + rdi_ctrl_offset);
	if (frame_timing && enable) {
		/* Record the old latch values while the path is still disabled.
		 * The same FIFO continuity checks used by IPP then detect even a
		 * lost first SOF. Every physical frame must produce its markers.
		 */
		writel(1, csid->base + CSID_RDI0_IRQ_SUBSAMPLE_PATTERN);
		writel(0, csid->base + CSID_RDI0_IRQ_SUBSAMPLE_PERIOD);
		camss_frame_reset(csid->camss, csid->id, port,
				  csid_read_timestamp(csid, CSID_RDI0_CURRENT_SOF),
				  csid_read_timestamp(csid, CSID_RDI0_CURRENT_EOF));
		writel(~0, csid->base + CSID_RDI0_IRQ_CLEAR);
		writel(IRQ_CMD_CLEAR, csid->base + CSID_IRQ_CMD);
		val = readl(csid->base + CSID_TOP_IRQ_MASK) | CSID_RDI0_TOP_IRQ;
		writel(val, csid->base + CSID_TOP_IRQ_MASK);
		/* CSID980 independently specifies the same marker/error masks
		 * for RDI0 and IPP, including overflow and frame-drop faults.
		 */
		writel(INFO_RUP_DONE | CSID_PATH_SOF_IRQ | CSID_PATH_EOF_IRQ |
		       CSID_PATH_ERROR_IRQS, csid->base + CSID_RDI0_IRQ_MASK);
	}

	val = readl(csid->base + rdi_cfg0_offset);

	if (enable)
		val |= RDI_CFG0_EN;

	writel(val, csid->base + rdi_cfg0_offset);
}

/* Interrupt at the start of every frame on an RDI to report frame starts. */
static void __csid_frame_sync_irq(struct csid_device *csid, u8 enable, u8 rdi)
{
	u32 top;

	if (!csid->res->frame_sync)
		return;

	top = readl(csid->base + CSID_TOP_IRQ_MASK);
	if (enable)
		top |= TOP_IRQ_RDI(rdi);
	else
		top &= ~TOP_IRQ_RDI(rdi);
	writel(enable ? INFO_INPUT_SOF : 0, csid->base + CSID_CSI2_RDIN_IRQ_MASK(rdi));
	writel(top, csid->base + CSID_TOP_IRQ_MASK);
}

/* Bayer order the TFE demosaic works on, from the media bus code */
static u32 __csid_ipp_cfa(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SGRBG14_1X14:
		return IPP_CFA_GRBG;
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SBGGR14_1X14:
		return IPP_CFA_BGGR;
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGBRG14_1X14:
		return IPP_CFA_GBRG;
	default:
		return IPP_CFA_RGGB;
	}
}

static void __csid_configure_ipp_stream(struct csid_device *csid, u8 enable,
					u8 vc)
{
	struct v4l2_mbus_framefmt *input_format =
		&csid->fmt[MSM_CSID_PAD_FIRST_SRC + CSID_IPP_PORT];
	const struct csid_format_info *format =
		csid_get_fmt_entry(csid->res->formats->formats,
				   csid->res->formats->nformats,
				   input_format->code);
	u32 val;

	writel(0, csid->base + CSID_IPP_CTRL);

	if (!enable) {
		writel(0, csid->base + CSID_IPP_IRQ_MASK);
		val = readl(csid->base + CSID_TOP_IRQ_MASK) & ~CSID_IPP_TOP_IRQ;
		writel(val, csid->base + CSID_TOP_IRQ_MASK);
		camss_isp_set_live(CAMSS_ISP_CSID, NULL);
		val = readl(csid->base + CSID_IPP_CFG0) & ~RDI_CFG0_EN;
		writel(val, csid->base + CSID_IPP_CFG0);
		return;
	}

	val = RDI_CFG0_TIMESTAMP_EN | CSID_TIMESTAMP_STROBE;
	val |= RDI_CFG0_RETIME_BS;	/* SOF retiming off, as downstream */
	val |= format->decode_format << RDI_CFG0_DECODE_FORMAT;
	val |= vc << RDI_CFG0_VC;
	val |= format->data_type << RDI_CFG0_DT;
	val |= (vc & 0x03) << RDI_CFG0_DT_ID;
	writel(val, csid->base + CSID_IPP_CFG0);

	/* no crop/drop/binning; pixel store absorbs horizontal blanking */
	writel(0, csid->base + CSID_IPP_CFG1);
	writel((4 << IPP_PIX_STORE_MIN_HBI) | IPP_PIX_STORE_EN,
	       csid->base + CSID_IPP_PIX_STORE_CFG);

	/* frames go to hardware context 0 of the TFE pixel pipeline */
	writel(IPP_CTXT_ID, csid->base + CSID_IPP_CTXT_CFG);
	writel((IPP_CTXT_ID << CAMIF_FRAME_CFG_CTXT) |
	       (__csid_ipp_cfa(input_format->code) << CAMIF_FRAME_CFG_CFA),
	       csid->base + CSID_IPP_CAMIF_FRAME_CFG);
	writel((BIT(IPP_CTXT_ID) << IPP_SRC_CTXT_MASK_SHIFT) |
	       (BIT(IPP_CTXT_ID) << IPP_DST_CTXT_MASK_SHIFT),
	       csid->base + CSID_IPP_IRQ_COMP_CFG0);
	writel(1, csid->base + CSID_IPP_IRQ_SUBSAMPLE_PATTERN);
	writel(0, csid->base + CSID_IPP_IRQ_SUBSAMPLE_PERIOD);
	writel(0, csid->base + CSID_IPP_IRQ_MASK);

	camss_isp_apply_script(CAMSS_ISP_CSID, csid->base);
	camss_frame_reset(csid->camss, csid->id, CSID_IPP_PORT,
			  csid_read_timestamp(csid, CSID_IPP_CURRENT_SOF),
			  csid_read_timestamp(csid, CSID_IPP_CURRENT_EOF));
	writel(~0, csid->base + CSID_IPP_IRQ_CLEAR);
	writel(IRQ_CMD_CLEAR, csid->base + CSID_IRQ_CMD);
	val = readl(csid->base + CSID_TOP_IRQ_MASK) | CSID_IPP_TOP_IRQ;
	writel(val, csid->base + CSID_TOP_IRQ_MASK);
	writel(INFO_RUP_DONE | CSID_PATH_SOF_IRQ | CSID_PATH_EOF_IRQ | CSID_PATH_ERROR_IRQS,
	       csid->base + CSID_IPP_IRQ_MASK);

	val = readl(csid->base + CSID_IPP_CFG0) | RDI_CFG0_EN;
	writel(val, csid->base + CSID_IPP_CFG0);

	camss_isp_set_live(CAMSS_ISP_CSID, csid->base);
}

static void csid_configure_stream(struct csid_device *csid, u8 enable)
{
	u8 i, k;

	__csid_configure_rx(csid, &csid->phy);

	for (i = 0; i < MSM_CSID_MAX_SRC_STREAMS_980; i++) {
		if (csid->phy.en_vc & BIT(i)) {
			if (csid_port_is_ipp(csid, i)) {
				__csid_configure_ipp_stream(csid, enable, 0);
			} else {
				__csid_frame_sync_irq(csid, enable, i);
				__csid_configure_rdi_stream(csid, enable, i, 0);
			}
			__csid_configure_rx_vc(csid, 0);

			for (k = 0; k < CAMSS_INIT_BUF_COUNT; k++) {
				__csid_aup_update(csid, i);
				__csid_rup_update(csid, i);
			}

			if (csid_port_is_ipp(csid, i))
				writel(enable ? RDI_CTRL_START_CMD : 0,
				       csid->base + CSID_IPP_CTRL);
			else
				__csid_ctrl_rdi(csid, enable, i);
		}
	}
}

static int csid_configure_testgen_pattern(struct csid_device *csid, s32 val)
{
	return 0;
}

static void csid_subdev_reg_update(struct csid_device *csid, int port_id,
				   bool clear)
{
	if (clear) {
		__csid_aup_rup_clear(csid, port_id);
		return;
	}

	__csid_aup_update(csid, port_id);
	/* IPP: latch the TFE IQ settings along with the buffer addresses */
	if (csid_port_is_ipp(csid, port_id))
		__csid_rup_update(csid, port_id);
}

/**
 * csid_isr - CSID module interrupt service routine
 * @irq: Interrupt line
 * @dev: CSID device
 *
 * Return IRQ_HANDLED on success
 */
static irqreturn_t csid_isr(int irq, void *dev)
{
	struct csid_device *csid = dev;
	u32 val, buf_done_val;
	u8 reset_done;
	int i;

	val = readl(csid->base + CSID_TOP_IRQ_STATUS);
	writel(val, csid->base + CSID_TOP_IRQ_CLEAR);

	reset_done = val & INFO_RST_DONE;

	buf_done_val = readl(csid->base + CSID_BUF_DONE_IRQ_STATUS);
	writel(buf_done_val, csid->base + CSID_BUF_DONE_IRQ_CLEAR);

	for (i = 0; i < MSM_CSID_MAX_SRC_STREAMS_980; i++) {
		if (csid->phy.en_vc & BIT(i) && csid_port_is_ipp(csid, i)) {
			val = readl(csid->base + CSID_IPP_IRQ_STATUS);

			if (val & INFO_RUP_DONE)
				csid_subdev_reg_update(csid, i, true);
			csid_frame_irq(csid, i, val);
			writel(val, csid->base + CSID_IPP_IRQ_CLEAR);

			if (buf_done_val & BUF_DONE_IRQ_STATUS_IPP_FD)
				camss_buf_done(csid->camss, csid->id, i);
		} else if (csid->phy.en_vc & BIT(i)) {
			val = readl(csid->base + CSID_CSI2_RDIN_IRQ_STATUS(i));

			if (val & INFO_RUP_DONE)
				csid_subdev_reg_update(csid, i, true);
			if (csid_rdi_frame_timing(csid, i))
				csid_frame_irq(csid, i, val);
			writel(val, csid->base + CSID_CSI2_RDIN_IRQ_CLEAR(i));

			if (buf_done_val & BIT(BUF_DONE_IRQ_STATUS_RDI_OFFSET + i))
				camss_buf_done(csid->camss, csid->id, i);

			/* After buf done, so the event carries this frame's number. */
			if (val & INFO_INPUT_SOF && csid->res->frame_sync)
				csid_frame_start(csid, i);
		}
	}

	val = IRQ_CMD_CLEAR;
	writel(val, csid->base + CSID_IRQ_CMD);

	if (reset_done)
		complete(&csid->reset_complete);

	return IRQ_HANDLED;
}

/**
 * csid_reset - Trigger reset on CSID module and wait to complete
 * @csid: CSID device
 *
 * Return 0 on success or a negative error code otherwise
 */
static int csid_reset(struct csid_device *csid)
{
	unsigned long time;
	u32 val;
	int i;

	reinit_completion(&csid->reset_complete);

	val = INFO_RST_DONE | BUF_DONE_IRQ_STATUS;
	writel(val, csid->base + CSID_TOP_IRQ_CLEAR);
	writel(val, csid->base + CSID_TOP_IRQ_MASK);

	val = 0;
	for (i = 0; i < MSM_CSID_MAX_SRC_STREAMS_980; i++) {
		if (csid->phy.en_vc & BIT(i)) {
			/*
			 * Only need to clear buf done IRQ status here,
			 * RUP done IRQ status will be cleared once isr
			 * strobe generated by CSID_RST_CMD
			 */
			if (csid_port_is_ipp(csid, i))
				val |= BUF_DONE_IRQ_STATUS_IPP_FD;
			else
				val |= BIT(BUF_DONE_IRQ_STATUS_RDI_OFFSET + i);
		}
	}
	writel(val, csid->base + CSID_BUF_DONE_IRQ_CLEAR);
	writel(val, csid->base + CSID_BUF_DONE_IRQ_MASK);

	/* Clear all IRQ status with CLEAR bits set */
	val = IRQ_CMD_CLEAR;
	writel(val, csid->base + CSID_IRQ_CMD);

	val = RST_LOCATION | RST_MODE;
	writel(val, csid->base + CSID_RST_CFG);

	val = SELECT_HW_RST | SELECT_IRQ_RST;
	writel(val, csid->base + CSID_RST_CMD);

	time = wait_for_completion_timeout(&csid->reset_complete,
					   msecs_to_jiffies(CSID_RESET_TIMEOUT_MS));

	if (!time) {
		dev_err(csid->camss->dev, "CSID reset timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static void csid_subdev_init(struct csid_device *csid)
{
	csid->testgen.nmodes = CSID_PAYLOAD_MODE_DISABLED;
}

const struct csid_hw_ops csid_ops_980 = {
	.configure_stream = csid_configure_stream,
	.configure_testgen_pattern = csid_configure_testgen_pattern,
	.hw_version = csid_hw_version,
	.isr = csid_isr,
	.reset = csid_reset,
	.src_pad_code = csid_src_pad_code,
	.subdev_init = csid_subdev_init,
	.reg_update = csid_subdev_reg_update,
};
