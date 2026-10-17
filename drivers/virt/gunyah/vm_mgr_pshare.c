// SPDX-License-Identifier: GPL-2.0-only
/*
 * Runtime memory sharing with a protected guest ("GuestAccept").
 *
 * A protected VM owns its stage-2, so the host cannot push a mapping into it
 * (the hypervisor answers DENIED, see Documentation/virt/gunyah). What the
 * host can do is SHARE a resource-manager memparcel to the VM and hand the
 * handle to the VMM; the guest then ACCEPTs that handle itself (RM
 * MEM_ACCEPT with MAP_IPA_CONTIGUOUS) at an IPA of its choosing. Nothing is
 * mapped until the guest agrees, and the guest alone decides where.
 *
 * Lifecycle: a share stays pinned until GH_VM_PROTECTED_UNSHARE reclaims it,
 * which the RM refuses while the guest still holds its acceptance. Whatever is
 * still shared when the VM goes away is reclaimed from the VM teardown path,
 * after the RM has reset the VM and thereby released the guest's acceptances.
 * A parcel the RM will not give back is leaked on purpose: memory the
 * hypervisor may still map must never return to the allocator.
 */

#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/scatterlist.h>

#include "vm_mgr.h"

/*
 * The RM-facing label lives in the same per-VM namespace as the labels the
 * driver stamps on userspace region parcels (gunyah_share_parcel). Keep the
 * two apart so a VMM may reuse a region label for a runtime share. Only the
 * RM sees the bit; the ioctl ABI and this table use the VMM's label as given.
 * The guest is unaffected: it accepts by handle with validate_label = 0.
 */
#define GUNYAH_PSHARE_LABEL_NS		BIT(31)

/* The RM transiently refuses a share while a just-reclaimed parcel settles. */
#define GUNYAH_PSHARE_SHARE_TRIES	8

struct gunyah_vm_pshare {
	struct list_head node;
	u32 label;
	unsigned long npages;
	struct gunyah_rm_mem_parcel parcel;
	/* DMA-BUF source; NULL for a pinned userspace buffer */
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
};

static bool gunyah_vm_pshare_allowed(struct gunyah_vm *ghvm)
{
	bool ok;

	down_read(&ghvm->status_lock);
	switch (ghvm->vm_status) {
	case GUNYAH_RM_VM_STATUS_LOAD:
	case GUNYAH_RM_VM_STATUS_READY:
	case GUNYAH_RM_VM_STATUS_RUNNING:
	case GUNYAH_RM_VM_STATUS_PAUSED:
		ok = true;
		break;
	default:
		ok = false;
		break;
	}
	up_read(&ghvm->status_lock);
	return ok;
}

/* Caller holds pshares_lock */
static struct gunyah_vm_pshare *gunyah_vm_pshare_find(struct gunyah_vm *ghvm,
						      u32 label)
{
	struct gunyah_vm_pshare *ps;

	list_for_each_entry(ps, &ghvm->pshares, node)
		if (ps->label == label)
			return ps;
	return NULL;
}

static u8 gunyah_vm_pshare_perms(u32 flags)
{
	u8 perms = 0;

	if (flags & GUNYAH_MEM_ALLOW_READ)
		perms |= GUNYAH_RM_ACL_R;
	if (flags & GUNYAH_MEM_ALLOW_WRITE)
		perms |= GUNYAH_RM_ACL_W;
	if (flags & GUNYAH_MEM_ALLOW_EXEC)
		perms |= GUNYAH_RM_ACL_X;
	return perms;
}

/* Append [phys, phys + size) to the entry list, merging with the previous run */
static void gunyah_vm_pshare_add_entry(struct gunyah_rm_mem_parcel *parcel,
				       u64 phys, u64 size)
{
	struct gunyah_rm_mem_entry *last;

	if (parcel->n_mem_entries) {
		last = &parcel->mem_entries[parcel->n_mem_entries - 1];
		if (le64_to_cpu(last->phys_addr) + le64_to_cpu(last->size) == phys) {
			last->size = cpu_to_le64(le64_to_cpu(last->size) + size);
			return;
		}
	}
	last = &parcel->mem_entries[parcel->n_mem_entries++];
	last->phys_addr = cpu_to_le64(phys);
	last->size = cpu_to_le64(size);
}

/*
 * SHARE: two ACL entries, the guest with the requested access and the host
 * with RWX (Gunyah grants no more than the host already has).
 */
static int gunyah_vm_pshare_init_parcel(struct gunyah_vm *ghvm,
					struct gunyah_vm_pshare *ps,
					u32 label, u32 flags,
					size_t max_entries)
{
	struct gunyah_rm_mem_parcel *parcel = &ps->parcel;
	u16 host_vmid;
	int ret;

	ret = gunyah_rm_get_vmid(ghvm->rm, &host_vmid);
	if (ret)
		return ret;

	parcel->acl_entries = kcalloc(2, sizeof(*parcel->acl_entries),
				      GFP_KERNEL_ACCOUNT);
	if (!parcel->acl_entries)
		return -ENOMEM;
	/*
	 * Worst case one entry per page: a fragmented host may offer no
	 * contiguous runs at all, so this can be far too large for kmalloc.
	 */
	parcel->mem_entries = kvcalloc(max_entries, sizeof(*parcel->mem_entries),
				       GFP_KERNEL_ACCOUNT);
	if (!parcel->mem_entries) {
		kfree(parcel->acl_entries);
		parcel->acl_entries = NULL;
		return -ENOMEM;
	}

	parcel->mem_type = GUNYAH_RM_MEM_TYPE_NORMAL;
	parcel->label = label | GUNYAH_PSHARE_LABEL_NS;
	parcel->mem_handle = GUNYAH_MEM_HANDLE_INVAL;
	parcel->n_acl_entries = 2;
	parcel->acl_entries[0].vmid = cpu_to_le16(ghvm->vmid);
	parcel->acl_entries[0].perms = gunyah_vm_pshare_perms(flags);
	parcel->acl_entries[1].vmid = cpu_to_le16(host_vmid);
	parcel->acl_entries[1].perms = GUNYAH_RM_ACL_R | GUNYAH_RM_ACL_W |
				       GUNYAH_RM_ACL_X;
	parcel->n_mem_entries = 0;
	ps->label = label;
	return 0;
}

static void gunyah_vm_pshare_free_parcel(struct gunyah_vm_pshare *ps)
{
	kvfree(ps->parcel.mem_entries);
	kfree(ps->parcel.acl_entries);
}

/* Drop the pins behind a parcel whose memory the RM has given back to us */
static void gunyah_vm_pshare_unpin_user(struct gunyah_vm *ghvm,
					struct gunyah_vm_pshare *ps)
{
	struct gunyah_rm_mem_entry *entry;
	unsigned long pfn, n, i;
	size_t e;

	for (e = 0; e < ps->parcel.n_mem_entries; e++) {
		entry = &ps->parcel.mem_entries[e];
		pfn = PHYS_PFN(le64_to_cpu(entry->phys_addr));
		n = le64_to_cpu(entry->size) >> PAGE_SHIFT;
		for (i = 0; i < n; i++)
			unpin_user_page(pfn_to_page(pfn + i));
	}
	account_locked_vm(ghvm->mm_s, ps->npages, false);
}

static void gunyah_vm_pshare_put_dmabuf(struct gunyah_vm_pshare *ps)
{
	dma_buf_unmap_attachment_unlocked(ps->attachment, ps->sgt,
					  DMA_BIDIRECTIONAL);
	dma_buf_detach(ps->dmabuf, ps->attachment);
	dma_buf_put(ps->dmabuf);
}

/* Release the backing of a share the RM no longer holds */
static void gunyah_vm_pshare_free(struct gunyah_vm *ghvm,
				  struct gunyah_vm_pshare *ps)
{
	if (ps->dmabuf)
		gunyah_vm_pshare_put_dmabuf(ps);
	else
		gunyah_vm_pshare_unpin_user(ghvm, ps);
	gunyah_vm_pshare_free_parcel(ps);
	kfree(ps);
}

/* Hand the built parcel to the RM and publish it; @ps stays the caller's on failure */
static int gunyah_vm_pshare_submit(struct gunyah_vm *ghvm,
				   struct gunyah_vm_pshare *ps, u32 *mem_handle)
{
	int ret, tries;

	mutex_lock(&ghvm->pshares_lock);
	if (gunyah_vm_pshare_find(ghvm, ps->label)) {
		ret = -EEXIST;
		goto unlock;
	}

	for (tries = 0; tries < GUNYAH_PSHARE_SHARE_TRIES; tries++) {
		ret = gunyah_rm_mem_share(ghvm->rm, &ps->parcel);
		if (!ret)
			break;
		usleep_range(500, 1500);
	}
	if (ret) {
		dev_warn(ghvm->parent,
			 "Failed to share parcel label %u to VM %u: %d\n",
			 ps->label, ghvm->vmid, ret);
		goto unlock;
	}

	list_add_tail(&ps->node, &ghvm->pshares);
	*mem_handle = ps->parcel.mem_handle;
	dev_dbg(ghvm->parent,
		"Shared label %u (%lu pages, %zu entries, %s) to VM %u: handle %#x\n",
		ps->label, ps->npages, ps->parcel.n_mem_entries,
		ps->dmabuf ? "dmabuf" : "user", ghvm->vmid, *mem_handle);
unlock:
	mutex_unlock(&ghvm->pshares_lock);
	return ret;
}

int gunyah_vm_pshare_user(struct gunyah_vm *ghvm,
			  struct gunyah_protected_share *args)
{
	struct gunyah_vm_pshare *ps;
	struct page **pages;
	unsigned long npages, i;
	unsigned int gup_flags;
	long pinned;
	u64 end;
	int ret;

	if (args->flags & ~GUNYAH_MEM_ALLOW_RWX)
		return -EINVAL;
	if (!args->memory_size || !PAGE_ALIGNED(args->memory_size) ||
	    !PAGE_ALIGNED(args->userspace_addr))
		return -EINVAL;
	if (check_add_overflow(args->userspace_addr, args->memory_size, &end))
		return -EOVERFLOW;
	if (!gunyah_vm_pshare_allowed(ghvm))
		return -ENXIO;

	npages = args->memory_size >> PAGE_SHIFT;
	pages = kvcalloc(npages, sizeof(*pages), GFP_KERNEL_ACCOUNT);
	if (!pages)
		return -ENOMEM;

	ps = kzalloc(sizeof(*ps), GFP_KERNEL_ACCOUNT);
	if (!ps) {
		ret = -ENOMEM;
		goto free_pages;
	}
	ps->npages = npages;
	ret = gunyah_vm_pshare_init_parcel(ghvm, ps, args->label, args->flags,
					   npages);
	if (ret)
		goto free_ps;

	gup_flags = FOLL_LONGTERM;
	if (args->flags & GUNYAH_MEM_ALLOW_WRITE)
		gup_flags |= FOLL_WRITE;
	pinned = pin_user_pages_fast(args->userspace_addr, npages, gup_flags,
				     pages);
	if (pinned < 0) {
		ret = pinned;
		goto free_parcel;
	}
	if (pinned != npages) {
		ret = -EFAULT;
		goto unpin;
	}
	ret = account_locked_vm(current->mm, npages, true);
	if (ret)
		goto unpin;

	/* Physically contiguous runs (THP folios collapse 512:1) become one entry */
	for (i = 0; i < npages; i++)
		gunyah_vm_pshare_add_entry(&ps->parcel, page_to_phys(pages[i]),
					   PAGE_SIZE);

	ret = gunyah_vm_pshare_submit(ghvm, ps, &args->mem_handle);
	if (ret)
		goto unaccount;

	kvfree(pages);
	return 0;

unaccount:
	account_locked_vm(current->mm, npages, false);
unpin:
	unpin_user_pages(pages, pinned);
free_parcel:
	gunyah_vm_pshare_free_parcel(ps);
free_ps:
	kfree(ps);
free_pages:
	kvfree(pages);
	return ret;
}

/* Slice [offset, offset + size) out of the exporter's sg-table as physical runs */
static int gunyah_vm_pshare_sgt_entries(struct gunyah_vm_pshare *ps,
					u64 offset, u64 size)
{
	struct scatterlist *sg;
	u64 skip = offset, remain = size, phys, take;
	unsigned int i;

	for_each_sgtable_sg(ps->sgt, sg, i) {
		if (!remain)
			break;
		if (skip >= sg->length) {
			skip -= sg->length;
			continue;
		}
		phys = sg_phys(sg) + skip;
		take = min_t(u64, (u64)sg->length - skip, remain);
		if (!PAGE_ALIGNED(phys) || !PAGE_ALIGNED(take))
			return -EINVAL;
		gunyah_vm_pshare_add_entry(&ps->parcel, phys, take);
		remain -= take;
		skip = 0;
	}
	return remain ? -EINVAL : 0;
}

int gunyah_vm_pshare_dmabuf(struct gunyah_vm *ghvm,
			    struct gunyah_protected_share_dmabuf *args)
{
	struct gunyah_vm_pshare *ps;
	u64 end;
	int ret;

	if (args->flags & ~GUNYAH_MEM_ALLOW_RWX)
		return -EINVAL;
	if (!args->memory_size || !PAGE_ALIGNED(args->memory_size) ||
	    !PAGE_ALIGNED(args->dmabuf_offset))
		return -EINVAL;
	if (check_add_overflow(args->dmabuf_offset, args->memory_size, &end))
		return -EOVERFLOW;
	if (!gunyah_vm_pshare_allowed(ghvm))
		return -ENXIO;

	ps = kzalloc(sizeof(*ps), GFP_KERNEL_ACCOUNT);
	if (!ps)
		return -ENOMEM;
	ps->npages = args->memory_size >> PAGE_SHIFT;

	ps->dmabuf = dma_buf_get(args->dmabuf_fd);
	if (IS_ERR(ps->dmabuf)) {
		ret = PTR_ERR(ps->dmabuf);
		goto free_ps;
	}
	if (end > ps->dmabuf->size) {
		ret = -EINVAL;
		goto put_dmabuf;
	}

	/* The attachment pins the exporter's backing for the parcel's lifetime */
	ps->attachment = dma_buf_attach(ps->dmabuf, ghvm->parent);
	if (IS_ERR(ps->attachment)) {
		ret = PTR_ERR(ps->attachment);
		goto put_dmabuf;
	}
	ps->sgt = dma_buf_map_attachment_unlocked(ps->attachment,
						  DMA_BIDIRECTIONAL);
	if (IS_ERR(ps->sgt)) {
		ret = PTR_ERR(ps->sgt);
		goto detach;
	}

	/* At most one run per source SG entry */
	ret = gunyah_vm_pshare_init_parcel(ghvm, ps, args->label, args->flags,
					   ps->sgt->orig_nents);
	if (ret)
		goto unmap;
	ret = gunyah_vm_pshare_sgt_entries(ps, args->dmabuf_offset,
					   args->memory_size);
	if (ret)
		goto free_parcel;

	ret = gunyah_vm_pshare_submit(ghvm, ps, &args->mem_handle);
	if (ret)
		goto free_parcel;
	return 0;

free_parcel:
	gunyah_vm_pshare_free_parcel(ps);
unmap:
	dma_buf_unmap_attachment_unlocked(ps->attachment, ps->sgt,
					  DMA_BIDIRECTIONAL);
detach:
	dma_buf_detach(ps->dmabuf, ps->attachment);
put_dmabuf:
	dma_buf_put(ps->dmabuf);
free_ps:
	kfree(ps);
	return ret;
}

int gunyah_vm_punshare(struct gunyah_vm *ghvm, u32 label)
{
	struct gunyah_vm_pshare *ps;
	int ret;

	mutex_lock(&ghvm->pshares_lock);
	ps = gunyah_vm_pshare_find(ghvm, label);
	if (!ps) {
		ret = -ENOENT;
		goto unlock;
	}

	/*
	 * The RM is the authority on whether the guest still holds its
	 * acceptance: it answers MEM_INUSE (-EBUSY) until the guest has done
	 * MEM_RELEASE. Only a successful reclaim may unpin -- freeing pages the
	 * guest's stage-2 still maps would corrupt the guest.
	 */
	ret = gunyah_rm_mem_reclaim(ghvm->rm, &ps->parcel);
	if (ret == -EBUSY) {
		/* Expected while the guest is still using it; the VMM retries */
		dev_dbg(ghvm->parent, "Parcel label %u still accepted by VM %u\n",
			label, ghvm->vmid);
		goto unlock;
	}
	if (ret) {
		dev_warn(ghvm->parent,
			 "Failed to reclaim parcel label %u from VM %u: %d\n",
			 label, ghvm->vmid, ret);
		goto unlock;
	}

	list_del(&ps->node);
	gunyah_vm_pshare_free(ghvm, ps);
unlock:
	mutex_unlock(&ghvm->pshares_lock);
	return ret;
}

/*
 * VM teardown, after the RM reset the VM. The reset released the guest's
 * acceptances, so the reclaims go through now; anything still refused is
 * leaked rather than freed under a possibly live stage-2 mapping.
 */
void gunyah_vm_pshare_release_all(struct gunyah_vm *ghvm)
{
	struct gunyah_vm_pshare *ps, *tmp;
	int ret;

	mutex_lock(&ghvm->pshares_lock);
	list_for_each_entry_safe(ps, tmp, &ghvm->pshares, node) {
		list_del(&ps->node);
		ret = gunyah_rm_mem_reclaim(ghvm->rm, &ps->parcel);
		if (ret) {
			dev_err(ghvm->parent,
				"Failed to reclaim parcel label %u: %d, leaking %lu pages\n",
				ps->label, ret, ps->npages);
			continue;
		}
		gunyah_vm_pshare_free(ghvm, ps);
	}
	mutex_unlock(&ghvm->pshares_lock);
}

MODULE_IMPORT_NS("DMA_BUF");
