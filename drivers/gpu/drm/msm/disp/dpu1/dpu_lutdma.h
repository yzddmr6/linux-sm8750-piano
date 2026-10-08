/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _DPU_LUTDMA_H
#define _DPU_LUTDMA_H

#include <linux/types.h>

struct dpu_kms;
struct dpu_lutdma;
struct dpu_hw_ctl;
struct dpu_hw_gc_lut;

struct dpu_lutdma *dpu_lutdma_init(struct dpu_kms *kms);
bool dpu_lutdma_ready(struct dpu_lutdma *dma);
bool dpu_lutdma_idle(struct dpu_lutdma *dma);
void dpu_lutdma_quiesce(struct dpu_lutdma *dma);
int dpu_lutdma_submit_gc(struct dpu_lutdma *dma, struct dpu_hw_ctl *ctl,
			u32 dspp_mask, const struct dpu_hw_gc_lut *lut);
/* Quiesces before freeing. An unconfirmed active queue never returns to
 * the outer uninit/shutdown path; it keeps all dependencies alive.
 */
void dpu_lutdma_destroy(struct dpu_lutdma *dma);

#endif
