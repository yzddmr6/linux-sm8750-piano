/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _DPU_LUTDMA_H
#define _DPU_LUTDMA_H

#include <linux/types.h>

struct dpu_kms;
struct dpu_lutdma;
struct drm_color_lut;
struct dpu_hw_ctl;
struct dpu_hw_gc_lut;

struct dpu_lutdma *dpu_lutdma_init(struct dpu_kms *kms);
bool dpu_lutdma_ready(struct dpu_lutdma *dma);
bool dpu_lutdma_idle(struct dpu_lutdma *dma);
bool dpu_lutdma_ctl_valid(struct dpu_lutdma *dma, struct dpu_hw_ctl *ctl);
int dpu_lutdma_submit_color(struct dpu_lutdma *dma, struct dpu_hw_ctl *ctl,
			u32 dspp_mask, const struct dpu_hw_gc_lut *gc_lut,
			const struct drm_color_lut *igc_lut, bool update_igc);
void dpu_lutdma_quiesce(struct dpu_lutdma *dma);
int dpu_lutdma_submit_gc(struct dpu_lutdma *dma, struct dpu_hw_ctl *ctl,
			u32 dspp_mask, const struct dpu_hw_gc_lut *lut);
/* Quiesces before freeing. If reset remains unconfirmed, return with the
 * DMA-owned resources intentionally leaked instead of freeing live storage.
 */
void dpu_lutdma_destroy(struct dpu_lutdma *dma);

#endif
