// SPDX-License-Identifier: GPL-2.0-only
/* GC v2 command preparation for standard DRM gamma LUTs.
 * Register layout and REGDMA v3 opcodes follow Qualcomm's published driver.
 */
#include <linux/bitops.h>
#include <linux/byteorder/generic.h>
#include <linux/errno.h>

#include "dpu_hwio.h"
#include "dpu_hw_catalog.h"
#include "dpu_hw_lm.h"
#include "dpu_hw_dspp.h"
#include "dpu_hw_gcv2.h"

#define REGDMA_V3_DECODE_SELECT 0x18180114
#define REGDMA_SINGLE_WRITE BIT(28)
#define REGDMA_HW_INDEX_WRITE (BIT(28) | BIT(29))
#define GC_ENABLE BIT(0)
#define GC_DATA_OFFSET 0x04
#define GC_INDEX_OFFSET 0x08
#define GC_CHANNEL_STRIDE 0x08
#define GC_LUT_SWAP_OFFSET 0x1c

int dpu_hw_gcv2_pack_regdma(__le32 *words, size_t capacity,
			  u32 dspp_mask, const struct dpu_hw_gc_lut *lut)
{
	const u32 *channels[3];
	unsigned int channel, entry, n = 0;
	unsigned int required = lut ? DPU_GCV2_LUT_WORDS : 4;

	if (!words || !dspp_mask || (dspp_mask & ~DPU_GCV2_DSPP_MASK))
		return -EINVAL;
	if (capacity < required)
		return -ENOSPC;
	/* GC v2 supports a distinct extended precision mode. Its extra bits
	 * are not represented by dpu_hw_gc_lut, so never silently enable it.
	 */
	if (lut && lut->flags)
		return -EOPNOTSUPP;

	words[n++] = cpu_to_le32(REGDMA_V3_DECODE_SELECT);
	words[n++] = cpu_to_le32(dspp_mask);
	if (!lut) {
		words[n++] = cpu_to_le32(REGDMA_SINGLE_WRITE | DPU_GCV2_REG_BASE);
		words[n++] = 0;
		return n;
	}

	/* dpu_crtc's standard conversion packs the green, blue, red channel
	 * LUTs as pairs of 10-bit values in 512 words. Keep that exact order.
	 */
	channels[0] = lut->c0;
	channels[1] = lut->c1;
	channels[2] = lut->c2;
	for (channel = 0; channel < 3; channel++) {
		u32 offset = DPU_GCV2_REG_BASE + channel * GC_CHANNEL_STRIDE;

		words[n++] = cpu_to_le32(REGDMA_SINGLE_WRITE | (offset + GC_INDEX_OFFSET));
		words[n++] = 0;
		/* HW-index writes repeatedly address the LUT port. An ordinary
		 * auto-address-increment or MMIO loop is a different operation.
		 */
		words[n++] = cpu_to_le32(REGDMA_HW_INDEX_WRITE | (offset + GC_DATA_OFFSET));
		words[n++] = cpu_to_le32(PGC_TBL_LEN);
		for (entry = 0; entry < PGC_TBL_LEN; entry++)
			words[n++] = cpu_to_le32(channels[channel][entry]);
	}
	words[n++] = cpu_to_le32(REGDMA_SINGLE_WRITE |
				(DPU_GCV2_REG_BASE + GC_LUT_SWAP_OFFSET));
	words[n++] = cpu_to_le32(BIT(0));
	words[n++] = cpu_to_le32(REGDMA_SINGLE_WRITE | DPU_GCV2_REG_BASE);
	/* The first backend uses the genuine regular 10-bit GC v2 mode.
	 * BIT(2), GC_HIGHPREC_EN, deliberately stays clear.
	 */
	words[n++] = cpu_to_le32(GC_ENABLE);
	return n;
}

int dpu_hw_color_regdma_last_offset(size_t command_words, size_t buffer_bytes,
				    size_t *offset)
{
	size_t last;

	if (!offset || !command_words || command_words > 0x3fff)
		return -EINVAL;
	if (buffer_bytes < 8 || command_words > (buffer_bytes - 8) / sizeof(u32))
		return -ENOSPC;
	last = (command_words * sizeof(u32) + 255) & ~(size_t)255;
	if (last > buffer_bytes - 8)
		return -ENOSPC;
	*offset = last;
	return 0;
}
