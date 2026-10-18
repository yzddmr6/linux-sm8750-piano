// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cma.h>
#include <linux/dma-map-ops.h>
#include <linux/firmware/qcom/qcom_tzmem.h>
#include <linux/mm.h>
#include <linux/overflow.h>

#include "qcomtee.h"

/**
 * define MAX_OUTBOUND_BUFFER_SIZE - Maximum size of outbound buffers.
 *
 * The size of outbound buffer depends on QTEE callback requests.
 */
#define MAX_OUTBOUND_BUFFER_SIZE SZ_4K

/**
 * define MAX_INBOUND_BUFFER_SIZE - Maximum size of the inbound buffer.
 *
 * The size of the inbound buffer depends on the user's requests,
 * specifically the number of IB and OB arguments. If an invocation
 * requires a size larger than %MAX_INBOUND_BUFFER_SIZE, the user should
 * consider using another form of shared memory with QTEE.
 */
#define MAX_INBOUND_BUFFER_SIZE SZ_4M

/**
 * qcomtee_msg_buffers_alloc() - Allocate inbound and outbound buffers.
 * @oic: context to use for the current invocation.
 * @u: array of arguments for the current invocation.
 *
 * It calculates the size of inbound and outbound buffers based on the
 * arguments in @u. It allocates the buffers from the teedev pool.
 *
 * Return: On success, returns 0. On error, returns < 0.
 */
int qcomtee_msg_buffers_alloc(struct qcomtee_object_invoke_ctx *oic,
			      struct qcomtee_arg *u)
{
	struct tee_context *ctx = oic->ctx;
	struct tee_shm *shm;
	size_t size;
	int i;

	/* Start offset in a message for buffer arguments. */
	size = qcomtee_msg_buffer_args(struct qcomtee_msg_object_invoke,
				       qcomtee_args_len(u));
	if (size > MAX_INBOUND_BUFFER_SIZE)
		return -EINVAL;

	/* Add size of IB arguments. */
	qcomtee_arg_for_each_input_buffer(i, u) {
		size = size_add(size, qcomtee_msg_offset_align(u[i].b.size));
		if (size > MAX_INBOUND_BUFFER_SIZE)
			return -EINVAL;
	}

	/* Add size of OB arguments. */
	qcomtee_arg_for_each_output_buffer(i, u) {
		size = size_add(size, qcomtee_msg_offset_align(u[i].b.size));
		if (size > MAX_INBOUND_BUFFER_SIZE)
			return -EINVAL;
	}

	shm = tee_shm_alloc_priv_buf(ctx, size);
	if (IS_ERR(shm))
		return PTR_ERR(shm);

	/* Allocate inbound buffer. */
	oic->in_shm = shm;
	shm = tee_shm_alloc_priv_buf(ctx, MAX_OUTBOUND_BUFFER_SIZE);
	if (IS_ERR(shm)) {
		tee_shm_free(oic->in_shm);

		return PTR_ERR(shm);
	}
	/* Allocate outbound buffer. */
	oic->out_shm = shm;

	oic->in_msg.addr = tee_shm_get_va(oic->in_shm, 0);
	oic->in_msg.size = tee_shm_get_size(oic->in_shm);
	oic->out_msg.addr = tee_shm_get_va(oic->out_shm, 0);
	oic->out_msg.size = tee_shm_get_size(oic->out_shm);
	/* QTEE assume unused buffers are zeroed. */
	memzero_explicit(oic->in_msg.addr, oic->in_msg.size);
	memzero_explicit(oic->out_msg.addr, oic->out_msg.size);

	return 0;
}

void qcomtee_msg_buffers_free(struct qcomtee_object_invoke_ctx *oic)
{
	tee_shm_free(oic->in_shm);
	tee_shm_free(oic->out_shm);
}

/* Dynamic shared memory pool based on tee_dyn_shm_alloc_helper(). */

static int qcomtee_shm_register(struct tee_context *ctx, struct tee_shm *shm,
				struct page **pages, size_t num_pages,
				unsigned long start)
{
	return qcom_tzmem_shm_bridge_create(shm->paddr, shm->size,
					    &shm->sec_world_id);
}

static int qcomtee_shm_unregister(struct tee_context *ctx, struct tee_shm *shm)
{
	qcom_tzmem_shm_bridge_delete(shm->sec_world_id);

	return 0;
}

static bool qcomtee_large_shm(size_t size)
{
	return get_order(size) > MAX_PAGE_ORDER;
}

static bool qcomtee_cma_shm(struct tee_shm_pool *pool, struct tee_shm *shm,
			   size_t size)
{
	return IS_ENABLED(CONFIG_DMA_CMA) && pool->private_data &&
		(!(shm->flags & TEE_SHM_PRIV) || qcomtee_large_shm(size));
}

static int pool_op_alloc(struct tee_shm_pool *pool, struct tee_shm *shm,
			 size_t size, size_t align)
{
	struct page *page;
	struct page **pages;
	size_t rounded, nr_pages;
	size_t i;
	int ret;

	if (!size || check_add_overflow(size, PAGE_SIZE - 1, &rounded))
		return -EINVAL;
	if (!qcomtee_cma_shm(pool, shm, size)) {
		if (qcomtee_large_shm(size))
			return -ENOMEM;
		return tee_dyn_shm_alloc_helper(shm, size, align, qcomtee_shm_register);
	}

	/*
	 * Use the declared contiguous pool for application-visible buffers.
	 * Its DT allocation range also describes the legacy TA address limit.
	 * Private invocation buffers keep the ordinary small-allocation path.
	 */
	nr_pages = rounded >> PAGE_SHIFT;
	page = cma_alloc(pool->private_data, nr_pages, 0, false);
	if (!page)
		return -ENOMEM;
	pages = kcalloc(nr_pages, sizeof(*pages), GFP_KERNEL);
	if (!pages) {
		ret = -ENOMEM;
		goto release_cma;
	}
	for (i = 0; i < nr_pages; i++)
		pages[i] = pfn_to_page(page_to_pfn(page) + i);
	shm->kaddr = page_address(page);
	shm->paddr = page_to_phys(page);
	shm->size = nr_pages << PAGE_SHIFT;
	shm->pages = pages;
	shm->num_pages = nr_pages;
	memset(shm->kaddr, 0, shm->size);
	ret = qcomtee_shm_register(shm->ctx, shm, pages, nr_pages,
				  (unsigned long)shm->kaddr);
	if (!ret)
		return 0;
	shm->kaddr = NULL;
	shm->pages = NULL;
	kfree(pages);
release_cma:
	cma_release(pool->private_data, page, nr_pages);
	return ret;
}

static void pool_op_free(struct tee_shm_pool *pool, struct tee_shm *shm)
{
	if (!qcomtee_cma_shm(pool, shm, shm->size)) {
		tee_dyn_shm_free_helper(shm, qcomtee_shm_unregister);
		return;
	}
	qcomtee_shm_unregister(shm->ctx, shm);
	cma_release(pool->private_data, shm->pages[0], shm->num_pages);
	kfree(shm->pages);
	shm->pages = NULL;
	shm->kaddr = NULL;
}

static void pool_op_destroy_pool(struct tee_shm_pool *pool)
{
	kfree(pool);
}

static const struct tee_shm_pool_ops pool_ops = {
	.alloc = pool_op_alloc,
	.free = pool_op_free,
	.destroy_pool = pool_op_destroy_pool,
};

struct tee_shm_pool *qcomtee_shm_pool_alloc(void)
{
	struct tee_shm_pool *pool;

	pool = kzalloc_obj(*pool);
	if (!pool)
		return ERR_PTR(-ENOMEM);

	pool->ops = &pool_ops;
	if (IS_ENABLED(CONFIG_DMA_CMA))
		pool->private_data = dev_get_cma_area(NULL);

	return pool;
}
