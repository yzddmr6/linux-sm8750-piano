// SPDX-License-Identifier: GPL-2.0-only
/* IGC v5 LUTBUS layout from Qualcomm's published display driver. */
#include <linux/bitops.h>
#include <linux/byteorder/generic.h>
#include <linux/errno.h>
#include <drm/drm_color_mgmt.h>

#include "dpu_hw_gcv2.h"
#include "dpu_hw_igcv5.h"

#define REGDMA_V3_DECODE_SELECT 0x18180114
#define REGDMA_SINGLE_WRITE BIT(28)
#define REGDMA_LUTBUS_WRITE (BIT(30) | BIT(29))
#define IGC_CONTROL_OFFSET 0x4
#define IGC_ENABLE BIT(8)
#define IGC_HIGH_PRECISION BIT(16)
#define IGC_TRANSFER_WORDS 3
#define IGC_TRANSFER_COUNT (DPU_IGCV5_DATA_WORDS / IGC_TRANSFER_WORDS)

static u16 igcv5_component(const struct drm_color_lut *lut, unsigned int index)
{
	unsigned int entry = index / 3;

	if (entry >= DPU_IGCV5_HW_ENTRIES)
		return 0; /* Pad the odd endpoint to a complete LUTBUS transfer. */
	/*
	 * Hardware main grid: 0, 1/256, ... 1. Dark grid: 0..127/1024.
	 * The stock Piano QDCM sRGB table matches the IEC sRGB EOTF on
	 * these grids exactly; its independent gamma-2.2 table agrees within
	 * one 16-bit code. Expose a uniform 1025-point DRM LUT and select
	 * samples, rather than exposing 385 nonuniform points to userspace.
	 */
	if (entry < DPU_IGCV5_MAIN_ENTRIES)
		entry *= 4;
	else
		entry -= DPU_IGCV5_MAIN_ENTRIES;
	switch (index % 3) {
	case 0:
		return lut[entry].red;
	case 1:
		return lut[entry].green;
	default:
		return lut[entry].blue;
	}
}

int dpu_hw_igcv5_pack_regdma(__le32 *words, size_t capacity,
			   u32 dspp_mask, const struct drm_color_lut *lut)
{
	unsigned int i, n = 0;
	unsigned int required = lut ? DPU_IGCV5_LUT_WORDS : 4;

	if (!words || !dspp_mask || (dspp_mask & ~DPU_GCV2_DSPP_MASK))
		return -EINVAL;
	if (capacity < required)
		return -ENOSPC;
	words[n++] = cpu_to_le32(REGDMA_V3_DECODE_SELECT);
	words[n++] = cpu_to_le32(dspp_mask);
	if (lut) {
		/* LUTBUS block IGC=0, table select=0, three DWORDs per transfer.
		 * Ordinary v5 uses the full 16-bit R/G/B, without the v4 <<4.
		 * Preserve the standard endpoint including zero; the vendor API's
		 * zero-as-default-last sentinel is not part of the DRM LUT ABI.
		 */
		words[n++] = cpu_to_le32(REGDMA_LUTBUS_WRITE);
		words[n++] = cpu_to_le32((IGC_TRANSFER_WORDS << 16) | IGC_TRANSFER_COUNT);
		for (i = 0; i < DPU_IGCV5_DATA_WORDS; i++)
			words[n++] = cpu_to_le32(igcv5_component(lut, i * 2) |
					((u32)igcv5_component(lut, i * 2 + 1) << 16));
		/* Published LUTBUS workaround: three real NO_OP opcodes. */
		words[n++] = 0;
		words[n++] = 0;
		words[n++] = 0;
	}
	words[n++] = cpu_to_le32(REGDMA_SINGLE_WRITE |
				(DPU_IGCV5_REG_BASE + IGC_CONTROL_OFFSET));
	/* Dither remains disabled; this mode changes LUT sampling, not DSC bpc. */
	words[n++] = cpu_to_le32(lut ? IGC_ENABLE | IGC_HIGH_PRECISION : 0);
	return n;
}

#if IS_ENABLED(CONFIG_DRM_MSM_DPU_IGCV5_KUNIT_TEST)
#include "dpu_hw_igcv5-test.c"
#endif
