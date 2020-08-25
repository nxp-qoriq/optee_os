/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2019, Linaro Limited
 */

#ifndef __KERNEL_SECURE_PARTITION_H
#define __KERNEL_SECURE_PARTITION_H

#include <assert.h>
#include <kernel/tee_ta_manager.h>
#include <kernel/thread.h>
#include <kernel/user_mode_ctx_struct.h>
#include <types_ext.h>
#include <util.h>

#define SP_RET_SUCCESS		0
#define SP_RET_NOT_SUPPORTED	-1
#define SP_RET_INVALID_PARAM	-2
#define SP_RET_DENIED		-3
#define SP_RET_NO_MEM		-5

#define SP_VER_MAJOR_MASK	0x7FFF
#define SP_VER_MINOR_MASK	0xFFFF
#define SP_VER_MAJOR		(0 & SP_VER_MAJOR_MASK)
#define SP_VER_MINOR		(1 & SP_VER_MINOR_MASK)
#define SP_VERSION		((SP_VER_MAJOR << 16) | (SP_VER_MINOR))

#define SP_MEM_ATTR_ACCESS_MASK		3
#define SP_MEM_ATTR_ACCESS_NONE		0
#define SP_MEM_ATTR_ACCESS_RW		1
#define SP_MEM_ATTR_ACCESS_RO		3
#define SP_MEM_ATTR_EXEC_MASK		(1 << 2)
#define SP_MEM_ATTR_EXEC		(0 << 2)
#define SP_MEM_ATTR_NON_EXEC		(1 << 2)

#define SP_SVC_VERSION			0x84000060
#define SP_SVC_EVENT_COMPLETE_64	0xC4000061
#define SP_SVC_MEMORY_ATTRIBUTES_GET_64	0xC4000064
#define SP_SVC_MEMORY_ATTRIBUTES_SET_64	0xC4000065

/* Param header types */
#define SP_PARAM_EP			UINT8_C(0x01)
#define SP_PARAM_IMAGE_BINARY		UINT8_C(0x02)
#define SP_PARAM_BL31			UINT8_C(0x03)
#define SP_PARAM_BL_LOAD_INFO		UINT8_C(0x04)
#define SP_PARAM_BL_PARAMS		UINT8_C(0x05)
#define SP_PARAM_PSCI_LIB_ARGS		UINT8_C(0x06)
#define SP_PARAM_SP_IMAGE_BOOT_INFO	UINT8_C(0x07)

/* Param header version */
#define SP_PARAM_VERSION_1		UINT8_C(0x01)
#define SP_PARAM_VERSION_2		UINT8_C(0x02)

/* void syscall_sys_return(uint32_t ret) __noreturn; */
#define SP_SYSCALL_RETURN		UINT32_C(0)
/* void syscall_log(const void *buf, size_t len); */
#define SP_SYSCALL_LOG			UINT32_C(1)
/* void syscall_panic(uint32_t code) __noreturn; */
#define SP_SYSCALL_PANIC		UINT32_C(2)

/***************************************************************************
 * This structure provides version information and the size of the
 * structure, attributes for the structure it represents
 ***************************************************************************/
struct sp_param_header {
	uint8_t type;		/* type of the structure */
	uint8_t version;	/* version of this structure */
	uint16_t size;		/* size of this structure in bytes */
	uint32_t attr;		/* attributes: unused bits SBZ */
};

/*
 * Flags used by the secure_partition_mp_info structure to describe the
 * characteristics of a cpu. Only a single flag is defined at the moment to
 * indicate the primary cpu.
 */
#define MP_INFO_FLAG_PRIMARY_CPU	UINT32_C(0x00000001)

/*
 * This structure is used to provide information required to initialise a S-EL0
 * partition.
 */
struct secure_partition_mp_info {
	uint64_t		mpidr;
	uint32_t		linear_id;
	uint32_t		flags;
};

struct secure_partition_boot_info {
	struct sp_param_header	h;
	uint64_t		sp_mem_base;
	uint64_t		sp_mem_limit;
	uint64_t		sp_image_base;
	uint64_t		sp_stack_base;
	uint64_t		sp_heap_base;
	uint64_t		sp_ns_comm_buf_base;
	uint64_t		sp_shared_buf_base;
	uint64_t		sp_image_size;
	uint64_t		sp_pcpu_stack_size;
	uint64_t		sp_heap_size;
	uint64_t		sp_ns_comm_buf_size;
	uint64_t		sp_shared_buf_size;
	uint32_t		num_sp_mem_regions;
	uint32_t		num_cpus;
	struct secure_partition_mp_info	*mp_info;
};

struct sec_part_ctx {
	struct user_mode_ctx uctx;
	struct thread_ctx_regs regs;
};

extern const struct tee_ta_ops secure_partition_ops;

static inline bool is_sp_ctx(struct tee_ta_ctx *ctx __maybe_unused)
{
#ifdef CFG_WITH_SECURE_PARTITION
	return ctx && ctx->ops == &secure_partition_ops;
#else
	return false;
#endif
}

static inline struct sec_part_ctx *to_sec_part_ctx(struct tee_ta_ctx *ctx)
{
	assert(is_sp_ctx(ctx));
	return container_of(ctx, struct sec_part_ctx, uctx.ctx);
}

void sec_part_save_return_state(struct thread_ctx_regs *ctx_regs,
				struct thread_svc_regs *svc_regs);

#ifdef CFG_WITH_SECURE_PARTITION
TEE_Result sec_part_init_session(const TEE_UUID *uuid,
				 struct tee_ta_session *s);
#else
static inline TEE_Result
sec_part_init_session(const TEE_UUID *uuid __unused,
		      struct tee_ta_session *s __unused)
{
	return TEE_ERROR_ITEM_NOT_FOUND;
}
#endif

#endif /*__KERNEL_SECURE_PARTITION_H*/
