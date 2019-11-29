// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2019, Linaro Limited
 */

#include <kernel/abort.h>
#include <kernel/secure_partition.h>
#include <kernel/user_mode_ctx.h>
#include <mm/fobj.h>
#include <mm/mobj.h>
#include <mm/tee_mmu.h>
#include <pta_stmm.h>
#include <tee/tee_svc.h>
#include <zlib.h>

#include "thread_private.h"

static const TEE_UUID stmm_uuid = PTA_STMM_UUID;

static const unsigned int stmm_entry;
static const unsigned int stmm_stack_size = 4 * SMALL_PAGE_SIZE;
static const unsigned int stmm_heap_size = 200 * SMALL_PAGE_SIZE;
static const unsigned int stmm_sec_buf_size = SMALL_PAGE_SIZE;

extern uint8_t stmm_image[];
extern const unsigned int stmm_image_size;
extern const unsigned int stmm_image_uncompressed_size;

static struct sec_part_ctx *sec_part_alloc_ctx(const TEE_UUID *uuid)
{
	TEE_Result res = TEE_SUCCESS;
	struct sec_part_ctx *spc = NULL;

	spc = calloc(1, sizeof(*spc));
	if (!spc)
		return NULL;

	spc->uctx.ctx.ops = &secure_partition_ops;
	spc->uctx.ctx.uuid = *uuid;

	res = vm_info_init(&spc->uctx);
	if (res)
		goto err;

	spc->uctx.ctx.ref_count = 1;
	condvar_init(&spc->uctx.ctx.busy_cv);

	return spc;
err:
	free(spc);
	return NULL;
}


static void clear_vfp_state(struct sec_part_ctx *spc __maybe_unused)
{
#ifdef CFG_WITH_VFP
	thread_user_clear_vfp(&spc->uctx.vfp);
#endif
}

static TEE_Result sec_part_enter_user_mode(struct sec_part_ctx *spc)
{
	TEE_Result res = TEE_SUCCESS;
	uint32_t exceptions = 0;
	uint32_t panic_code = 0;
	uint32_t panicked = 0;

	exceptions = thread_mask_exceptions(THREAD_EXCP_ALL);
	__thread_enter_user_mode(&spc->regs, &panicked, &panic_code);
	thread_unmask_exceptions(exceptions);

	clear_vfp_state(spc);

	if (panicked) {
		abort_print_current_ta();
		DMSG("sec_part panicked with code 0x%"PRIx32, panic_code);
		res = TEE_ERROR_TARGET_DEAD;
	}

	return res;
}

static void init_stmm_regs(struct sec_part_ctx *spc, unsigned long a0,
			   unsigned long a1, unsigned long sp, unsigned long pc)
{
	spc->regs.x[0] = a0;
	spc->regs.x[1] = a1;
	spc->regs.sp = sp;
	spc->regs.pc = pc;
}

static TEE_Result alloc_and_map_sp_fobj(struct sec_part_ctx *spc, size_t sz,
					uint32_t prot, vaddr_t *va)
{
	size_t num_pgs = ROUNDUP(sz, SMALL_PAGE_SIZE) / SMALL_PAGE_SIZE;
	struct fobj *fobj = fobj_ta_mem_alloc(num_pgs);
	struct mobj *mobj = mobj_with_fobj_alloc(fobj, NULL);
	TEE_Result res = TEE_SUCCESS;

	fobj_put(fobj);
	if (!mobj)
		return TEE_ERROR_OUT_OF_MEMORY;
	res = vm_map(&spc->uctx, va, num_pgs * SMALL_PAGE_SIZE,
		     prot, 0, mobj, 0);
	if (res)
		mobj_put(mobj);

	return res;
}

static void *zalloc(void *opaque __unused, unsigned int items,
		    unsigned int size)
{
	return malloc(items * size);
}

static void zfree(void *opaque __unused, void *address)
{
	free(address);
}

static void uncompress_image(void *dst, size_t dst_size, void *src,
			     size_t src_size)
{
	z_stream strm = {
		.next_in = src,
		.avail_in = src_size,
		.next_out = dst,
		.avail_out = dst_size,
		.zalloc = zalloc,
		.zfree = zfree,
	};
	int st = 0;

	st = inflateInit(&strm);
	if (st != Z_OK)
		panic("inflateInit");
	st = inflate(&strm, Z_SYNC_FLUSH);
	if (st != Z_STREAM_END)
		panic("inflate");
	st = inflateEnd(&strm);
	if (st != Z_OK)
		panic("inflateEnd");
}

static TEE_Result load_stmm(struct sec_part_ctx *spc)
{
	struct secure_partition_boot_info *boot_info = NULL;
	struct secure_partition_mp_info *mp_info = NULL;
	TEE_Result res = TEE_SUCCESS;
	vaddr_t sp_addr = 0;
	vaddr_t image_addr = 0;
	vaddr_t heap_addr = 0;
	vaddr_t stack_addr = 0;
	vaddr_t sec_buf_addr = 0;
	unsigned int sp_size;

	sp_size = ROUNDUP(stmm_image_uncompressed_size, SMALL_PAGE_SIZE) +
			stmm_stack_size + stmm_heap_size + stmm_sec_buf_size;
	res = alloc_and_map_sp_fobj(spc, sp_size,
				    TEE_MATTR_PRW, &sp_addr);
	if (res)
		return res;

	image_addr = sp_addr;
	heap_addr = image_addr +
			ROUNDUP(stmm_image_uncompressed_size,
				SMALL_PAGE_SIZE);
	stack_addr = heap_addr + stmm_heap_size;
	sec_buf_addr = stack_addr + stmm_stack_size;

	tee_mmu_set_ctx(&spc->uctx.ctx);
	uncompress_image((void *)image_addr, stmm_image_uncompressed_size,
			 stmm_image, stmm_image_size);

	res = vm_set_prot(&spc->uctx, image_addr,
			  ROUNDUP(stmm_image_uncompressed_size,
				  SMALL_PAGE_SIZE),
			  TEE_MATTR_URX | TEE_MATTR_PR);
	if (res)
		return res;

	res = vm_set_prot(&spc->uctx, heap_addr, stmm_heap_size,
			  TEE_MATTR_URW | TEE_MATTR_PRW);
	if (res)
		return res;

	res = vm_set_prot(&spc->uctx, stack_addr, stmm_stack_size,
			  TEE_MATTR_URW | TEE_MATTR_PRW);
	if (res)
		return res;

	res = vm_set_prot(&spc->uctx, sec_buf_addr, stmm_sec_buf_size,
			  TEE_MATTR_URW | TEE_MATTR_PRW);
	if (res)
		return res;

	DMSG("stmm load address %#"PRIxVA, image_addr);

	boot_info = (struct secure_partition_boot_info *)sec_buf_addr;
	mp_info = (struct secure_partition_mp_info *)(boot_info + 1);
	*boot_info = (struct secure_partition_boot_info){
		.h.type = SP_PARAM_SP_IMAGE_BOOT_INFO,
		.h.version = SP_PARAM_VERSION_1,
		.h.size = sizeof(struct secure_partition_boot_info),
		.h.attr = 0,
		.sp_mem_base = sp_addr,
		.sp_mem_limit = sp_addr + sp_size,
		.sp_image_base = image_addr,
		.sp_stack_base = stack_addr,
		.sp_heap_base = heap_addr,
		.sp_shared_buf_base = sec_buf_addr,
		.sp_image_size = stmm_image_size,
		.sp_pcpu_stack_size = stmm_stack_size,
		.sp_heap_size = stmm_heap_size,
		.sp_shared_buf_size = stmm_sec_buf_size,
		.num_sp_mem_regions = 6,
		.num_cpus = 1,
		.mp_info = mp_info,
	};
	mp_info->mpidr = read_mpidr_el1();
	mp_info->linear_id = 0;
	mp_info->flags = MP_INFO_FLAG_PRIMARY_CPU;

	init_stmm_regs(spc, sec_buf_addr,
		       (vaddr_t)(mp_info + 1) - sec_buf_addr,
		       stack_addr + stmm_stack_size, image_addr + stmm_entry);

	return sec_part_enter_user_mode(spc);
}

TEE_Result sec_part_init_session(const TEE_UUID *uuid,
				 struct tee_ta_session *sess)
{
	struct sec_part_ctx *spc = NULL;
	TEE_Result res = TEE_SUCCESS;

	if (memcmp(uuid, &stmm_uuid, sizeof(*uuid)))
		return TEE_ERROR_ITEM_NOT_FOUND;

	spc = sec_part_alloc_ctx(uuid);
	if (!spc)
		return TEE_ERROR_OUT_OF_MEMORY;

	spc->is_initializing = true;

	sess->ctx = &spc->uctx.ctx;
	tee_ta_push_current_session(sess);
	res = load_stmm(spc);
	tee_ta_pop_current_session();
	tee_mmu_set_ctx(NULL);
	if (res)
		goto err;

	spc->is_initializing = false;
	TAILQ_INSERT_TAIL(&tee_ctxes, &spc->uctx.ctx, link);
	return TEE_SUCCESS;

err:
	sess->ctx = NULL;
	spc->uctx.ctx.ops->destroy(&spc->uctx.ctx);

	return res;
}

static TEE_Result stmm_map_ns_buf(struct sec_part_ctx *spc,
				  struct tee_ta_param *param,
				  vaddr_t *ns_buf_base)
{
	uint32_t exp_pt = TEE_PARAM_TYPES(TEE_PARAM_TYPE_MEMREF_INOUT,
					  TEE_PARAM_TYPE_VALUE_OUTPUT,
					  TEE_PARAM_TYPE_NONE,
					  TEE_PARAM_TYPE_NONE);
	void *param_va[TEE_NUM_PARAMS] = { NULL };
	TEE_Result res = TEE_SUCCESS;

	if (exp_pt != param->types)
		return TEE_ERROR_BAD_PARAMETERS;

	res = tee_mmu_map_param(&spc->uctx, param, param_va);
	if (!res)
		*ns_buf_base = (vaddr_t)param_va[0];
	return res;
}

static TEE_Result stmm_enter_open_session(struct tee_ta_session *s,
					  struct tee_ta_param *param __unused,
					  TEE_ErrorOrigin *eo)
{
	struct sec_part_ctx *spc = to_sec_part_ctx(s->ctx);

	if (spc->is_initializing) {
		/* stmm is initialized in sec_part_init_session() */
		*eo = TEE_ORIGIN_TEE;
		return TEE_ERROR_BAD_STATE;
	}

	return TEE_SUCCESS;
}

static TEE_Result stmm_enter_invoke_cmd(struct tee_ta_session *s,
					uint32_t cmd,
					struct tee_ta_param *param,
					TEE_ErrorOrigin *eo __unused)
{
	struct sec_part_ctx *spc = to_sec_part_ctx(s->ctx);
	TEE_Result res = TEE_SUCCESS;
	vaddr_t ns_buf_base = 0;

	if (cmd != PTA_STMM_COMMUNICATE)
		return TEE_ERROR_NOT_IMPLEMENTED;

	res = stmm_map_ns_buf(spc, param, &ns_buf_base);
	if (res)
		return res;

	spc->regs.x[0] = 0xc4000041; /* 64-bit MM_COMMUNICATE */
	spc->regs.x[1] = ns_buf_base;
	spc->regs.x[2] = param->u[0].mem.size;
	spc->regs.x[3] = 0;

	tee_ta_push_current_session(s);

	res = sec_part_enter_user_mode(spc);
	if (!res)
		param->u[1].val.a = spc->regs.x[1];

	/*
	 * Clear out the parameter mappings added with tee_mmu_map_param()
	 * above.
	 */
	tee_mmu_clean_param(&spc->uctx);

	tee_ta_pop_current_session();

	return res;
}

static void stmm_enter_close_session(struct tee_ta_session *s __unused)
{
}

static void sec_part_dump_state(struct tee_ta_ctx *ctx)
{
	user_mode_ctx_print_mappings(to_user_mode_ctx(ctx));
}

static uint32_t sec_part_get_instance_id(struct tee_ta_ctx *ctx)
{
	return to_sec_part_ctx(ctx)->uctx.vm_info.asid;
}

static void sec_part_ctx_destroy(struct tee_ta_ctx *ctx)
{
	struct sec_part_ctx *spc = to_sec_part_ctx(ctx);

	tee_pager_rem_um_areas(&spc->uctx);
	vm_info_final(&spc->uctx);
	free(spc);
}

static int sp_svc_set_mem_attr(vaddr_t va, unsigned int nr_pages, uint32_t perm)
{
	TEE_Result res;
	struct tee_ta_session *sess = NULL;
	struct sec_part_ctx *spc = NULL;
	uint32_t prot;

	if (va == 0 || nr_pages == 0)
		return SP_RET_INVALID_PARAM;

	res = tee_ta_get_current_session(&sess);
	if (res != TEE_SUCCESS)
		return SP_RET_DENIED;

	spc = to_sec_part_ctx(sess->ctx);

	prot = 0;
	if ((perm & SP_MEM_ATTR_ACCESS_MASK) == SP_MEM_ATTR_ACCESS_RW)
		prot |= TEE_MATTR_URW | TEE_MATTR_PRW;
	else if ((perm & SP_MEM_ATTR_ACCESS_MASK) == SP_MEM_ATTR_ACCESS_RO)
		prot |= TEE_MATTR_UR | TEE_MATTR_PR;

	if ((perm & SP_MEM_ATTR_EXEC_MASK) == SP_MEM_ATTR_EXEC) {
		prot |= TEE_MATTR_UX;
		prot &= ~(TEE_MATTR_UW | TEE_MATTR_PW);
	}

	res = vm_set_prot(&spc->uctx, va, nr_pages * SMALL_PAGE_SIZE, prot);
	return res == TEE_SUCCESS ? SP_RET_SUCCESS : SP_RET_DENIED;
}

static bool return_helper(bool panic, uint32_t panic_code,
			  struct thread_svc_regs *svc_regs)
{
	if (!panic) {
		struct tee_ta_session *sess = NULL;
		struct sec_part_ctx *spc = NULL;
		size_t n = 0;

		tee_ta_get_current_session(&sess);
		spc = to_sec_part_ctx(sess->ctx);

		spc->regs.x[0] = svc_regs->x0;
		spc->regs.x[1] = svc_regs->x1;

		/* Save the state to return to */
		for (n = 19; n <= 29; n++)
			spc->regs.x[n] = spc->regs.x[n];

		/* In case user mode was AArch32 */
		for (n = 5; n <= 14; n++)
			spc->regs.x[n] = *(&svc_regs->x0 + n);

		spc->regs.sp = svc_regs->sp_el0;
		spc->regs.pc = svc_regs->elr;
		spc->regs.cpsr = svc_regs->spsr;
	}

	svc_regs->x0 = 0;
	svc_regs->x1 = panic;
	svc_regs->x2 = panic_code;

	return false;
}

#ifdef ARM32
static void set_svc_retval(struct thread_svc_regs *regs, uint32_t ret_val)
{
	regs->r0 = ret_val;
}
#endif /*ARM32*/

#ifdef ARM64
static void set_svc_retval(struct thread_svc_regs *regs, uint64_t ret_val)
{
	regs->x0 = ret_val;
}
#endif /*ARM64*/

static bool stmm_handle_svc(struct thread_svc_regs *regs)
{
	switch (regs->x0) {
	case SP_SVC_VERSION:
		set_svc_retval(regs, SP_VERSION);
		return true;
	case SP_SVC_EVENT_COMPLETE_64:
		return return_helper(false, 0, regs);
	case SP_SVC_MEMORY_ATTRIBUTES_GET_64:
		set_svc_retval(regs,
			       SP_MEM_ATTR_EXEC | SP_MEM_ATTR_ACCESS_RW);
		return true;
	case SP_SVC_MEMORY_ATTRIBUTES_SET_64:
		set_svc_retval(regs,
			       sp_svc_set_mem_attr(regs->x1, regs->x2,
						   regs->x3));
		return true;
	default:
		EMSG("Undefined syscall 0x%"PRIx32, (uint32_t)regs->x0);
		return return_helper(true, 0xbadfbadf, regs);
	}
}

const struct tee_ta_ops secure_partition_ops __rodata_unpaged = {
	.enter_open_session = stmm_enter_open_session,
	.enter_invoke_cmd = stmm_enter_invoke_cmd,
	.enter_close_session = stmm_enter_close_session,
	.dump_state = sec_part_dump_state,
	.destroy = sec_part_ctx_destroy,
	.get_instance_id = sec_part_get_instance_id,
	.handle_svc = stmm_handle_svc,
};

void sec_part_save_return_state(struct thread_ctx_regs *ctx_regs,
				struct thread_svc_regs *svc_regs)
{
	struct sec_part_ctx *spc = NULL;
	struct tee_ta_session *sess = NULL;
	size_t n;

	tee_ta_get_current_session(&sess);
	spc = to_sec_part_ctx(sess->ctx);

	spc->regs.x[0] = ctx_regs->x[0];
	for (n = 19; n <= 29; n++)
		spc->regs.x[n] = ctx_regs->x[n];

	/* In case user mode was AArch32 */
	for (n = 5; n <= 14; n++)
		spc->regs.x[n] = *(&svc_regs->x0 + n);

	spc->regs.sp = svc_regs->sp_el0;
	spc->regs.pc = svc_regs->elr;
	spc->regs.cpsr = svc_regs->spsr;
}
