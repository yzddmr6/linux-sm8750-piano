/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _DPU_HW_GCV2_H
#define _DPU_HW_GCV2_H

#include <linux/bits.h>
#include <linux/types.h>

struct dpu_hw_gc_lut;

/* DSPP selection bits in the actual REGDMA v3 command stream. */
#define DPU_GCV2_DSPP_MASK GENMASK(20, 17)
#define DPU_GCV2_REG_BASE 0x17c0
#define DPU_GCV2_LUT_WORDS (2 + 3 * (2 + 2 + 512) + 2 + 2)

/* This prepares CPU data only: no mapping, MMIO, queueing or DMA lifetime.
 * Return the command length in dwords or a negative error. The caller must
 * keep the mapped command buffer pinned until REGDMA completion or reset.
 */
int dpu_hw_gcv2_pack_regdma(__le32 *words, size_t capacity,
			  u32 dspp_mask, const struct dpu_hw_gc_lut *lut);

int dpu_hw_color_regdma_last_offset(size_t command_words, size_t buffer_bytes,
				    size_t *offset);

#endif
