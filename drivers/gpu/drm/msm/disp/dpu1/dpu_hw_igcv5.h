/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _DPU_HW_IGCV5_H
#define _DPU_HW_IGCV5_H

#include <linux/types.h>

struct drm_color_lut;

#define DPU_IGCV5_LUT_SIZE 1025
#define DPU_IGCV5_REG_BASE 0x1260
#define DPU_IGCV5_MAIN_ENTRIES 257
#define DPU_IGCV5_DARK_ENTRIES 128
#define DPU_IGCV5_HW_ENTRIES (DPU_IGCV5_MAIN_ENTRIES + DPU_IGCV5_DARK_ENTRIES)
/* 386 padded RGB triplets / two triplets per LUTBUS transfer. */
#define DPU_IGCV5_DATA_WORDS 579
#define DPU_IGCV5_LUT_WORDS (2 + 2 + DPU_IGCV5_DATA_WORDS + 3 + 2)

/* CPU command preparation only. A non-NULL LUT has exactly 1025 DRM entries.
 * The caller owns the buffer, mapping, queue and completion/reset lifetime.
 */
int dpu_hw_igcv5_pack_regdma(__le32 *words, size_t capacity,
			   u32 dspp_mask, const struct drm_color_lut *lut);

#endif
