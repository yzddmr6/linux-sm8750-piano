// SPDX-License-Identifier: GPL-2.0-only
/* REGDMA v3 DB queue transport, based on Qualcomm's published register ABI. */
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sizes.h>
#include <linux/slab.h>

#include "msm_drv.h"
#include "msm_gem.h"
#include "dpu_hw_gcv2.h"
#include "dpu_hw_igcv5.h"
#include "dpu_hw_vbif.h"
#include "dpu_kms.h"
#include "dpu_hw_dspp.h"
#include "dpu_lutdma.h"

#define REGDMA_V3 0x30000
#define REGDMA_OPMODE 0x4
#define REGDMA_ERROR_STATUS 0x170
#define REGDMA_ERROR_CLEAR 0x1b0
#define REGDMA_ERROR_MASK (GENMASK(2, 0) | GENMASK(19, 16))
#define REGDMA_PMU_CLK_CTRL 0x1f0
#define REGDMA_PMU_FORCE (BIT(0) | BIT(16))
#define REGDMA_QUEUE_STRIDE 0x1000
#define REGDMA_DONE_STATUS 0x44
#define REGDMA_DONE_CLEAR 0x48
#define REGDMA_QUEUE_RESET 0x54
#define REGDMA_DONE BIT(3)
#define CTL_REGDMA_TRIGGER 0xd4
#define REGDMA_WRITE BIT(22)
#define REGDMA_LAST BIT(24)
#define REGDMA_CMD_WORDS GENMASK(13, 0)
#define REGDMA_ALIGN 256
#define REGDMA_BUFFER_SIZE SZ_16K
#define REGDMA_LAST_SIZE (2 * sizeof(u32))

struct dpu_lutdma {
	struct dpu_kms *kms;
	const struct dpu_lutdma_cfg *cfg;
	void __iomem *mmio;
	resource_size_t start, size;
	struct drm_gem_object *bo;
	struct drm_gpuvm *vm;
	__le32 *commands;
	u64 iova;
	struct mutex lock;
	bool faulted, inflight, module_held, pm_held, clock_changed, leaked;
	u32 active_ctl, pmu_before;
	unsigned long ctl_mask;
};

bool dpu_lutdma_ready(struct dpu_lutdma *dma)
{
	return dma && dma->bo && !READ_ONCE(dma->faulted);
}

bool dpu_lutdma_idle(struct dpu_lutdma *dma)
{
	return !dma || !READ_ONCE(dma->inflight);
}

static void __iomem *queue_reg(struct dpu_lutdma *dma, u32 ctl, u32 offset)
{
	resource_size_t reg;

	if (!dma || !dma->mmio || !dma->cfg || ctl < CTL_0 || ctl >= CTL_MAX ||
	    !(dma->ctl_mask & BIT(ctl)) || (offset & 3) ||
	    offset > REGDMA_QUEUE_STRIDE - sizeof(u32))
		return NULL;
	/* CTL_0 == 1 in both the downstream ABI and this driver. */
	reg = (resource_size_t)dma->cfg->db_base +
	      (resource_size_t)ctl * REGDMA_QUEUE_STRIDE + offset;
	if (reg > dma->size || dma->size - reg < sizeof(u32))
		return NULL;
	return dma->mmio + reg;
}

static int queue_reset(struct dpu_lutdma *dma, u32 ctl)
{
	void __iomem *reset = queue_reg(dma, ctl, REGDMA_QUEUE_RESET);
	u32 value;

	if (!reset)
		return -EINVAL;
	writel(BIT(0), dma->mmio + dma->cfg->db_base + REGDMA_OPMODE);
	writel(BIT(0), reset);
	return readl_poll_timeout(reset, value,
				  !value, 10, 2000);
}

static int clocks_get(struct dpu_lutdma *dma)
{
	struct dpu_hw_mdp *mdp = dma->kms->hw_mdp;
	int ret;

	ret = pm_runtime_resume_and_get(&dma->kms->pdev->dev);
	if (ret < 0)
		return ret;
	dma->pm_held = true;
	dma->clock_changed = mdp->ops.setup_clk_force_ctrl(mdp, dma->cfg->clk_ctrl, true);
	dma->pmu_before = readl(dma->mmio + dma->cfg->db_base + REGDMA_PMU_CLK_CTRL);
	writel(dma->pmu_before | REGDMA_PMU_FORCE,
	       dma->mmio + dma->cfg->db_base + REGDMA_PMU_CLK_CTRL);
	wmb();
	return 0;
}

static void clocks_put(struct dpu_lutdma *dma)
{
	/* Only called after completion/reset or before any descriptor was sent. */
	writel(dma->pmu_before, dma->mmio + dma->cfg->db_base + REGDMA_PMU_CLK_CTRL);
	if (dma->clock_changed)
		dma->kms->hw_mdp->ops.setup_clk_force_ctrl(dma->kms->hw_mdp,
							 dma->cfg->clk_ctrl, false);
	dma->clock_changed = false;
	if (dma->pm_held) {
		pm_runtime_put(&dma->kms->pdev->dev);
		dma->pm_held = false;
	}
}

static void setup_qos(struct dpu_lutdma *dma)
{
	struct dpu_hw_vbif *vbif = dma->kms->hw_vbif;
	int level;

	/* Dedicated LUTDMA client policy from the SM8750 hardware catalog. */
	for (level = 0; level < ARRAY_SIZE(dma->cfg->qos); level++)
		vbif->ops.set_qos_remap(vbif, dma->cfg->xin_id, level, dma->cfg->qos[level]);
	vbif->ops.set_limit_conf(vbif, dma->cfg->xin_id, true, dma->cfg->read_ot_limit);
}

static void retire_reference(struct dpu_lutdma *dma)
{
	WRITE_ONCE(dma->inflight, false);
	clocks_put(dma);
	if (dma->module_held) {
		module_put(THIS_MODULE);
		dma->module_held = false;
	}
}

bool dpu_lutdma_ctl_valid(struct dpu_lutdma *dma, struct dpu_hw_ctl *ctl)
{
	return ctl && queue_reg(dma, ctl->idx, REGDMA_QUEUE_RESET);
}

int dpu_lutdma_submit_color(struct dpu_lutdma *dma, struct dpu_hw_ctl *ctl,
			u32 dspp_mask, const struct dpu_hw_gc_lut *gc_lut,
			const struct drm_color_lut *igc_lut, bool update_igc)
{
	u32 done, errors, descriptor;
	__le32 *last;
	size_t capacity = (REGDMA_BUFFER_SIZE - REGDMA_LAST_SIZE) / sizeof(u32);
	size_t last_offset;
	int words, igc_words, ret;

	might_sleep(); /* Called by the process-context atomic commit, never IRQ. */
	if (!dma || !ctl || ctl->idx < CTL_0 || ctl->idx >= CTL_MAX ||
	    !dspp_mask || (dspp_mask & ~DPU_GCV2_DSPP_MASK) ||
	    !queue_reg(dma, ctl->idx, REGDMA_QUEUE_RESET))
		return -EINVAL;
	mutex_lock(&dma->lock);
	if (!dpu_lutdma_ready(dma)) {
		ret = -EIO;
		goto unlock;
	}
	words = dpu_hw_gcv2_pack_regdma(dma->commands, capacity, dspp_mask, gc_lut);
	if (words < 0) {
		ret = words;
		goto unlock;
	}
	if (update_igc) {
		igc_words = dpu_hw_igcv5_pack_regdma(dma->commands + words,
						capacity - words, dspp_mask, igc_lut);
		if (igc_words < 0) {
			ret = igc_words;
			goto unlock;
		}
		words += igc_words;
	}
	ret = dpu_hw_color_regdma_last_offset(words, REGDMA_BUFFER_SIZE, &last_offset);
	if (ret)
		goto unlock;
	ret = clocks_get(dma);
	if (ret)
		goto unlock;
	setup_qos(dma);
	if (!try_module_get(THIS_MODULE)) {
		ret = -ENODEV;
		clocks_put(dma);
		goto unlock;
	}
	dma->module_held = true;
	last = (__le32 *)((u8 *)dma->commands + last_offset);
	last[0] = cpu_to_le32(0x18180114);
	last[1] = 0; /* Real downstream final descriptor: decode-select no blocks. */
	dma_wmb(); /* WC GEM writes must precede descriptor publication. */
	writel(BIT(0), dma->mmio + dma->cfg->db_base + REGDMA_OPMODE);
	writel(REGDMA_ERROR_MASK, dma->mmio + dma->cfg->db_base + REGDMA_ERROR_CLEAR);
	writel(REGDMA_DONE, queue_reg(dma, ctl->idx, REGDMA_DONE_CLEAR));
	dma->active_ctl = ctl->idx;
	WRITE_ONCE(dma->inflight, true);
	drm_dbg_kms(dma->kms->dev, "color REGDMA ctl=%u dspp_mask=%#x iova=%#llx words=%d\n",
		    ctl->idx, dspp_mask, dma->iova, words);
	writel(lower_32_bits(dma->iova), queue_reg(dma, ctl->idx, 0));
	writel(REGDMA_WRITE | (words & REGDMA_CMD_WORDS), queue_reg(dma, ctl->idx, 4));
	wmb();
	writel(lower_32_bits(dma->iova + last_offset), queue_reg(dma, ctl->idx, 0));
	descriptor = REGDMA_WRITE | REGDMA_LAST | 2;
	writel(descriptor, queue_reg(dma, ctl->idx, 4));
	wmb();
	DPU_REG_WRITE(&ctl->hw, CTL_REGDMA_TRIGGER, BIT(0));
	ret = readl_poll_timeout(queue_reg(dma, ctl->idx, REGDMA_DONE_STATUS), done,
				 done & REGDMA_DONE, 10, 20000);
	errors = readl(dma->mmio + dma->cfg->db_base + REGDMA_ERROR_STATUS) & REGDMA_ERROR_MASK;
	if (ret || errors) {
		WRITE_ONCE(dma->faulted, true);
		drm_err(dma->kms->dev, "color REGDMA failure ctl=%u done=%#x error=%#x ret=%d\n",
			ctl->idx, done, errors, ret);
		ret = ret ? ret : -EIO;
		if (queue_reset(dma, ctl->idx)) {
			drm_err(dma->kms->dev, "REGDMA reset unconfirmed; retaining GEM/VM/clocks\n");
			goto unlock;
		}
	} else {
		drm_dbg_kms(dma->kms->dev, "color REGDMA completed ctl=%u status=%#x\n", ctl->idx, done);
	}
	retire_reference(dma);
unlock:
	mutex_unlock(&dma->lock);
	return ret;
}

int dpu_lutdma_submit_gc(struct dpu_lutdma *dma, struct dpu_hw_ctl *ctl,
			u32 dspp_mask, const struct dpu_hw_gc_lut *lut)
{
	return dpu_lutdma_submit_color(dma, ctl, dspp_mask, lut, NULL, false);
}

struct dpu_lutdma *dpu_lutdma_init(struct dpu_kms *kms)
{
	const struct dpu_lutdma_cfg *cfg = kms->catalog->lutdma;
	struct dpu_lutdma *dma;
	struct resource *resource;
	struct dpu_hw_vbif *vbif = kms->hw_vbif;
	int ret;
	u32 i;

	if (!cfg)
		return NULL;
	resource = platform_get_resource_byname(kms->pdev, IORESOURCE_MEM, "regdma");
	if (!resource)
		resource = platform_get_resource_byname(kms->pdev, IORESOURCE_MEM, "regdma_phys");
	if (!resource)
		return NULL; /* Optional resource: no initialized GC2 backend. */
	if (cfg->version != REGDMA_V3 || resource_size(resource) < 0x7000 ||
	    !kms->catalog->ctl || !kms->catalog->ctl_count ||
	    cfg->clk_ctrl <= DPU_CLK_CTRL_NONE || cfg->clk_ctrl >= DPU_CLK_CTRL_MAX ||
	    !kms->catalog->mdp->clk_ctrls[cfg->clk_ctrl].reg_off ||
	    !kms->base.vm || !kms->hw_mdp || !kms->hw_mdp->ops.setup_clk_force_ctrl ||
	    !vbif || !vbif->ops.set_qos_remap || !vbif->ops.set_limit_conf)
		return ERR_PTR(-EINVAL);
	dma = kzalloc_obj(*dma);
	if (!dma)
		return ERR_PTR(-ENOMEM);
	dma->kms = kms;
	dma->cfg = cfg;
	dma->start = resource->start;
	dma->size = resource_size(resource);
	mutex_init(&dma->lock);
	if (!request_mem_region(dma->start, dma->size, "dpu-regdma")) {
		ret = -EBUSY;
		goto free_dma;
	}
	dma->mmio = ioremap(dma->start, dma->size);
	if (!dma->mmio) {
		ret = -ENOMEM;
		goto release_region;
	}
	/* The enum includes CTLs absent on this SoC. Validate only catalog IDs,
	 * including the last u32 register, before allocating or publishing DMA.
	 */
	for (i = 0; i < kms->catalog->ctl_count; i++) {
		u32 ctl = kms->catalog->ctl[i].id;

		if (ctl < CTL_0 || ctl >= CTL_MAX || (dma->ctl_mask & BIT(ctl))) {
			ret = -EINVAL;
			goto unmap;
		}
		dma->ctl_mask |= BIT(ctl);
		if (!queue_reg(dma, ctl, REGDMA_QUEUE_RESET)) {
			ret = -EINVAL;
			goto unmap;
		}
	}
	dma->bo = msm_gem_new(kms->dev, REGDMA_BUFFER_SIZE, MSM_BO_WC);
	if (IS_ERR(dma->bo)) {
		ret = PTR_ERR(dma->bo);
		goto unmap;
	}
	dma->vm = drm_gpuvm_get(kms->base.vm);
	ret = msm_gem_get_and_pin_iova_range(dma->bo, dma->vm, &dma->iova,
					    PAGE_SIZE, (u64)U32_MAX + 1);
	if (ret)
		goto put_bo;
	if ((dma->iova & (REGDMA_ALIGN - 1)) || dma->iova > (u64)U32_MAX + 1 - REGDMA_BUFFER_SIZE) {
		ret = -ERANGE;
		goto unpin;
	}
	dma->commands = msm_gem_get_vaddr(dma->bo);
	if (IS_ERR(dma->commands)) {
		ret = PTR_ERR(dma->commands);
		goto unpin;
	}
	memset(dma->commands, 0, REGDMA_BUFFER_SIZE);
	ret = clocks_get(dma);
	if (ret)
		goto put_vaddr;
	setup_qos(dma);
	/* A reset acknowledgement is required before advertising this engine.
	 * No descriptor using our GEM has been sent during initialization.
	 */
	for (i = 0; i < kms->catalog->ctl_count; i++) {
		ret = queue_reset(dma, kms->catalog->ctl[i].id);
		if (ret)
			break;
	}
	clocks_put(dma);
	if (ret)
		goto put_vaddr;
	return dma;
put_vaddr:
	msm_gem_put_vaddr(dma->bo);
unpin:
	msm_gem_unpin_iova(dma->bo, dma->vm);
put_bo:
	drm_gpuvm_put(dma->vm);
	drm_gem_object_put(dma->bo);
unmap:
	iounmap(dma->mmio);
release_region:
	release_mem_region(dma->start, dma->size);
free_dma:
	kfree(dma);
	return ERR_PTR(ret);
}

void dpu_lutdma_quiesce(struct dpu_lutdma *dma)
{
	unsigned int retries;

	if (!dma)
		return;
	mutex_lock(&dma->lock);
	WRITE_ONCE(dma->faulted, true);
	/* Unbind cannot be vetoed. Prefer leaking the DMA-owned storage to
	 * freeing it without a stop acknowledgement, but do not hold up
	 * shutdown indefinitely. A leaked engine must never be touched again.
	 */
	if (dma->leaked)
		goto unlock;
	for (retries = 0; dma->inflight && retries < 20; retries++) {
		if (!queue_reset(dma, dma->active_ctl)) {
			retire_reference(dma);
			break;
		}
		msleep(100);
	}
	if (dma->inflight) {
		dma->leaked = true;
		drm_err(dma->kms->dev,
			"REGDMA ctl=%u stop unconfirmed after 2 s; leaking DMA resources\n",
			dma->active_ctl);
	}
unlock:
	mutex_unlock(&dma->lock);
}

void dpu_lutdma_destroy(struct dpu_lutdma *dma)
{
	if (!dma)
		return;
	dpu_lutdma_quiesce(dma);
	mutex_lock(&dma->lock);
	if (dma->leaked) {
		mutex_unlock(&dma->lock);
		return;
	}
	msm_gem_kernel_put(dma->bo, dma->vm);
	drm_gpuvm_put(dma->vm);
	iounmap(dma->mmio);
	release_mem_region(dma->start, dma->size);
	mutex_unlock(&dma->lock);
	kfree(dma);
}
