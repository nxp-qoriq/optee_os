// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright 2022-2023 NXP
 */
#include <acipher.h>
#include <drivers/imx_mu.h>
#include <ele.h>
#include <initcall.h>
#include <kernel/boot.h>
#include <kernel/delay.h>
#include <kernel/panic.h>
#include <kernel/tee_common_otp.h>
#include <key_store.h>
#include <mm/core_memprot.h>
#include <mm/core_mmu.h>
#include <stdint.h>
#include <tee/cache.h>
#include <tee_api_defines.h>
#include <trace.h>
#include <types_ext.h>
#include <utee_types.h>
#include <util.h>
#include <utils_trace.h>

#define ELE_BASE_ADDR MU_BASE
#define ELE_BASE_SIZE MU_SIZE

#define ELE_COMMAND_SUCCEED 0xd6

#define ELE_CMD_SESSION_OPEN	    0x10
#define ELE_CMD_SESSION_CLOSE	    0x11
#define ELE_CMD_RNG_GET		    0xCD
#define ELE_CMD_TRNG_STATE	    0xA4
#define ELE_CMD_GET_INFO	    0xDA
#define ELE_CMD_DERIVE_KEY	    0xA9
#define ELE_CMD_SAB_INIT 0x17

#define IMX_ELE_TRNG_STATUS_READY 0x3

#define ELE_MU_IRQ 0x0

#define CACHELINE_SIZE 64

register_phys_mem_pgdir(MEM_AREA_IO_SEC, MU_BASE, MU_SIZE);

struct get_info_rsp {
	uint32_t rsp_code;
	uint16_t soc_id;
	uint16_t soc_rev;
	uint16_t lifecycle;
	uint8_t sssm_state;
	uint8_t unused_1;
	uint32_t uid[4];
	uint32_t sha256_rom_patch[8];
	uint32_t sha256_firmware[8];
	uint32_t oem_srkh[16];
	uint8_t trng_state;
	uint8_t csal_state;
	uint8_t imem_state;
	uint8_t unused_2;
} __packed;

/*
 * The CRC for the message is computed xor-ing all the words of the message:
 * the header and all the words except the word storing the CRC.
 *
 * @msg MU message to hash
 */
static uint32_t compute_crc(const struct imx_mu_msg *msg)
{
	uint32_t crc = 0;
	uint8_t i = 0;
	uint32_t *payload = (uint32_t *)msg;

	assert(msg);

	for (i = 0; i < msg->header.size - 1; i++)
		crc ^= payload[i];

	return crc;
}

void update_crc(struct imx_mu_msg *msg)
{
	assert(msg);
	/*
	 * The CRC field is the last element of array. The size of the header
	 * is also subtracted from CRC computation.
	 */
	msg->data.u32[msg->header.size - 2] = compute_crc(msg);
}

/*
 * Return the given MU base address, depending on the MMU state.
 *
 * @pa MU physical base address
 * @sz MU size
 */
static vaddr_t imx_ele_init(paddr_t pa, size_t sz)
{
	static bool is_initialized;
	vaddr_t va = 0;

	assert(pa && sz);

	if (cpu_mmu_enabled())
		va = core_mmu_get_va(pa, MEM_AREA_IO_SEC, sz);
	else
		va = (vaddr_t)pa;

	if (!is_initialized) {
		imx_mu_init(va);
		is_initialized = true;
	}

	return va;
}

struct response_code get_response_code(uint32_t word)
{
	struct response_code rsp = {
		.rating_extension = (word & GENMASK_32(31, 16)) >> 16,
		.rating = (word & GENMASK_32(15, 8)) >> 8,
		.status = (word & GENMASK_32(7, 0)) >> 0,
	};

	return rsp;
}

enum ele_status {
	ELE_GENERAL_ERROR = 0x00,
	ELE_INVALID_ADDRESS = 0x02,
	ELE_UNKNOWN_IDENTIFIER,
	ELE_INVALID_ARGUMENT,
	ELE_NVM_ERROR,
	ELE_OUT_OF_MEMORY,
	ELE_UNKNOWN_HANDLE,
	ELE_KEY_STORE_AUTH_FAILED = 0x09,
	ELE_IDENTIFIER_CONFLICT = 0x0B,
	ELE_UNSUPPORTED_COMMAND = 0x0D,
	ELE_KEYSTORE_CONFLICT = 0x0F,
	ELE_NO_SPACE_IN_KEY_STORE = 0x19,
	ELE_OUTPUT_BUFFER_SHORT = 0x1D,
	ELE_CRC_ERROR = 0xB9,
};

static TEE_Result ele_status_to_tee_result(uint32_t word)
{
	struct response_code rsp_code = {};

	rsp_code = get_response_code(word);
	if (rsp_code.status == ELE_COMMAND_SUCCEED)
		return TEE_SUCCESS;

	switch (rsp_code.rating) {
	case ELE_OUT_OF_MEMORY:
	case ELE_NO_SPACE_IN_KEY_STORE:
		return TEE_ERROR_OUT_OF_MEMORY;
	case ELE_INVALID_ARGUMENT:
		return TEE_ERROR_BAD_PARAMETERS;
	case ELE_UNKNOWN_HANDLE:
	case ELE_UNKNOWN_IDENTIFIER:
		return TEE_ERROR_ITEM_NOT_FOUND;
	case ELE_OUTPUT_BUFFER_SHORT:
		return TEE_ERROR_SHORT_BUFFER;
	case ELE_UNSUPPORTED_COMMAND:
		return TEE_ERROR_NOT_SUPPORTED;
	default:
		break;
	}
	return TEE_ERROR_GENERIC;
}

TEE_Result imx_ele_call(struct imx_mu_msg *msg)
{
	TEE_Result res = TEE_ERROR_GENERIC;
	vaddr_t va = 0;

	assert(msg);

	if (msg->header.tag != ELE_REQUEST_TAG) {
		EMSG("Request has invalid tag: %#"PRIx8" instead of %#"PRIx8,
		     msg->header.tag, ELE_REQUEST_TAG);
		return TEE_ERROR_BAD_PARAMETERS;
	}

	va = imx_ele_init(ELE_BASE_ADDR, ELE_BASE_SIZE);
	if (!va) {
		EMSG("Fail to get base address");
		return TEE_ERROR_GENERIC;
	}

	ele_trace_print_msg(*msg);

	res = imx_mu_call(va, msg, true);
	if (res) {
		EMSG("Failed to transmit message: %#" PRIx32, res);
		return res;
	}

	if (msg->header.tag != ELE_RESPONSE_TAG) {
		EMSG("Response has invalid tag: %#" PRIx8
		     " instead of %#" PRIx8,
		     msg->header.tag, ELE_RESPONSE_TAG);
		return TEE_ERROR_GENERIC;
	}

	ele_trace_print_msg(*msg);

	return ele_status_to_tee_result(msg->data.u32[0]);
}

/*
 * Initialize EdgeLock Enclave services
 */
static TEE_Result __maybe_unused imx_ele_sab_init(void)
{
	struct imx_mu_msg msg = {
		.header.version = ELE_VERSION_HSM,
		.header.size = 1,
		.header.tag = ELE_REQUEST_TAG,
		.header.command = ELE_CMD_SAB_INIT,
	};

	return imx_ele_call(&msg);
}

/*
 * Open a session with EdgeLock Enclave. It returns a session handle.
 *
 * @session_handle EdgeLock Enclave session handle
 */
static TEE_Result imx_ele_session_open(uint32_t *session_handle)
{
	TEE_Result res = TEE_ERROR_GENERIC;
	struct open_session_cmd {
		uint8_t rsvd1;
		uint8_t interrupt_num;
		uint16_t rsvd2;
		uint8_t priority;
		uint8_t op_mode;
		uint16_t rsvd3;
	} __packed cmd = {
		.rsvd1 = 0,
		.interrupt_num = ELE_MU_IRQ,
		.rsvd2 = 0,
		.priority = 0,
		.op_mode = 0,
		.rsvd3 = 0,
	};
	struct open_session_rsp {
		uint32_t rsp_code;
		uint32_t session_handle;
	} rsp = { };
	struct imx_mu_msg msg = {
		.header.version = ELE_VERSION_HSM,
		.header.size = SIZE_MSG_32(cmd),
		.header.tag = ELE_REQUEST_TAG,
		.header.command = ELE_CMD_SESSION_OPEN,
	};

	assert(session_handle);

	memcpy(msg.data.u8, &cmd, sizeof(cmd));

	res = imx_ele_call(&msg);
	if (res)
		return res;

	memcpy(&rsp, msg.data.u8, sizeof(rsp));

	*session_handle = rsp.session_handle;

	return TEE_SUCCESS;
}

/*
 * Close a session with EdgeLock Enclave.
 *
 * @session_handle EdgeLock Enclave session handle
 */
static TEE_Result __maybe_unused imx_ele_session_close(uint32_t session_handle)
{
	struct close_session_cmd {
		uint32_t session_handle;
	} cmd = {
		.session_handle = session_handle,
	};
	struct imx_mu_msg msg = {
		.header.version = ELE_VERSION_HSM,
		.header.size = SIZE_MSG_32(cmd),
		.header.tag = ELE_REQUEST_TAG,
		.header.command = ELE_CMD_SESSION_CLOSE,
	};

	memcpy(msg.data.u8, &cmd, sizeof(cmd));

	return imx_ele_call(&msg);
}

static TEE_Result imx_ele_get_device_info(struct get_info_rsp *rsp)
{
	TEE_Result res = TEE_ERROR_GENERIC;
	struct imx_ele_buf output = {};
	struct {
		uint32_t addr_msb;
		uint32_t addr_lsb;
		uint16_t size;
	} __packed cmd = {};
	struct imx_mu_msg msg = {
		.header.version = ELE_VERSION_BASELINE,
		.header.size = SIZE_MSG_32(cmd),
		.header.tag = ELE_REQUEST_TAG,
		.header.command = ELE_CMD_GET_INFO,
	};

	if (!rsp)
		return TEE_ERROR_BAD_PARAMETERS;

	res = imx_ele_buf_alloc(&output, NULL, sizeof(*rsp));
	if (res)
		goto out;

	cmd.addr_msb = output.paddr_msb;
	cmd.addr_lsb = output.paddr_lsb;
	cmd.size = sizeof(*rsp);

	memcpy(msg.data.u8, &cmd, sizeof(cmd));

	res = imx_ele_call(&msg);
	if (res)
		goto out;

	res = imx_ele_buf_copy(&output, (uint8_t *)rsp, sizeof(*rsp));
out:
	imx_ele_buf_free(&output);

	return res;
}

int tee_otp_get_die_id(uint8_t *buffer, size_t len)
{
	static uint32_t uid[4];
	static bool is_fetched;
	struct get_info_rsp rsp = {};

	assert(buffer && len);

	if (is_fetched)
		goto out;

	if (imx_ele_get_device_info(&rsp))
		goto err;

	memcpy(uid, rsp.uid, MIN(sizeof(rsp.uid), len));

	is_fetched = true;
out:
	memcpy(buffer, uid, MIN(sizeof(uid), len));

	return 0;
err:
	panic("Fail to get the device UID");
}

TEE_Result imx_ele_get_global_session_handle(uint32_t *session_handle)
{
	static uint32_t imx_ele_session_handle;
	TEE_Result res = TEE_ERROR_GENERIC;

	if (!session_handle)
		return TEE_ERROR_BAD_PARAMETERS;

	if (imx_ele_session_handle) {
		res = TEE_SUCCESS;
		goto out;
	}

	res = imx_ele_session_open(&imx_ele_session_handle);
	if (res) {
		EMSG("Failed to open global session");
		return res;
	}

out:
	*session_handle = imx_ele_session_handle;
	return res;
}

#ifdef CFG_IMX_ELE_ECC_DRV
static TEE_Result imx_ele_global_init(void)
{
	TEE_Result res = TEE_ERROR_GENERIC;

	res = imx_ele_sab_init();
	if (res) {
		EMSG("Failed to initialize Edgelock Enclave services");
		goto err;
	}

	res = imx_ele_ecc_init();
	if (res)
		EMSG("ELE ECC driver registration failed");

err:
	return res;
}
driver_init(imx_ele_global_init);
#endif

#if defined(CFG_MX93)
static TEE_Result imx_ele_derive_key(const uint8_t *ctx, size_t ctx_size,
				     uint8_t *key, size_t key_size)
{
	TEE_Result res = TEE_ERROR_GENERIC;
	struct key_derive_cmd {
		uint32_t key_addr_msb;
		uint32_t key_addr_lsb;
		uint32_t ctx_addr_msb;
		uint32_t ctx_addr_lsb;
		uint16_t key_size;
		uint16_t ctx_size;
		uint32_t crc;
	} __packed cmd = {};
	struct imx_mu_msg msg = {
		.header.version = ELE_VERSION_BASELINE,
		.header.size = SIZE_MSG_32(cmd),
		.header.tag = ELE_REQUEST_TAG,
		.header.command = ELE_CMD_DERIVE_KEY,
	};
	struct imx_ele_buf ele_ctx = {};
	struct imx_ele_buf ele_key = {};

	assert(ctx && key);

	if (key_size != 16 && key_size != 32)
		return TEE_ERROR_BAD_PARAMETERS;

	res = imx_ele_buf_alloc(&ele_ctx, ctx, ctx_size);
	if (res)
		goto out;

	res = imx_ele_buf_alloc(&ele_key, key, key_size);
	if (res)
		goto out;

	cmd.key_addr_lsb = ele_key.paddr_lsb;
	cmd.key_addr_msb = ele_key.paddr_msb;
	cmd.key_size = key_size;

	cmd.ctx_addr_lsb = ele_ctx.paddr_lsb;
	cmd.ctx_addr_msb = ele_ctx.paddr_msb;
	cmd.ctx_size = ctx_size;

	memcpy(msg.data.u8, &cmd, sizeof(cmd));
	update_crc(&msg);

	res = imx_ele_call(&msg);
	if (res)
		goto out;

	res = imx_ele_buf_copy(&ele_key, key, key_size);
out:
	imx_ele_buf_free(&ele_key);
	imx_ele_buf_free(&ele_ctx);

	return res;
}

TEE_Result tee_otp_get_hw_unique_key(struct tee_hw_unique_key *hwkey)
{
	static const char pattern[] = "TEE_for_HUK_ELE";
	static uint8_t key[HW_UNIQUE_KEY_LENGTH];
	static bool is_fetched;

	if (is_fetched)
		goto out;

	if (imx_ele_derive_key((const uint8_t *)pattern, sizeof(pattern), key,
			       sizeof(key)))
		panic("Fail to get HUK from ELE");

	is_fetched = true;
out:
	memcpy(hwkey->data, key,
	       MIN(sizeof(key), (size_t)HW_UNIQUE_KEY_LENGTH));

	return TEE_SUCCESS;
}

/*
 * Get the current state of the ELE TRNG
 */
static TEE_Result imx_ele_rng_get_trng_state(void)
{
	TEE_Result res = TEE_ERROR_GENERIC;
	struct rng_get_trng_state_msg_rsp {
		uint32_t rsp_code;
		uint8_t trng_state;
		uint8_t csal_state;
	} __packed rsp = { };
	struct imx_mu_msg msg = {
		.header.version = ELE_VERSION_BASELINE,
		.header.size = 1,
		.header.tag = ELE_REQUEST_TAG,
		.header.command = ELE_CMD_TRNG_STATE,
	};

	res = imx_ele_call(&msg);
	if (res)
		return res;

	memcpy(&rsp, msg.data.u8, sizeof(rsp));

	if (rsp.trng_state != IMX_ELE_TRNG_STATUS_READY)
		return TEE_ERROR_BUSY;
	else
		return TEE_SUCCESS;
}

unsigned long plat_get_aslr_seed(void)
{
	TEE_Result res = TEE_ERROR_GENERIC;
	uint64_t timeout = timeout_init_us(10 * 1000);
	struct rng_get_random_cmd {
		uint32_t addr_msb;
		uint32_t addr_lsb;
		uint32_t size;
		uint32_t crc;
	} cmd = { };
	struct imx_mu_msg msg = {
		.header.version = ELE_VERSION_HSM,
		.header.size = SIZE_MSG_32(cmd),
		.header.tag = ELE_REQUEST_TAG,
		.header.command = ELE_CMD_RNG_GET,
	};
	unsigned long aslr __aligned(CACHELINE_SIZE) = 0;

	/*
	 * This function can only be called when the MMU is off. No
	 * virtual/physical address translation is performed, nor cache
	 * maintenance.
	 */
	assert(!cpu_mmu_enabled());

	reg_pair_from_64((uint64_t)&aslr, &cmd.addr_msb, &cmd.addr_lsb);
	cmd.size = sizeof(aslr);

	/*
	 * Check the current TRNG state of the ELE. The TRNG must be
	 * started with a command earlier in the boot to allow the TRNG
	 * to generate enough entropy.
	 */
	while (imx_ele_rng_get_trng_state() == TEE_ERROR_BUSY)
		if (timeout_elapsed(timeout))
			panic("ELE RNG is busy");

	memcpy(msg.data.u8, &cmd, sizeof(cmd));
	update_crc(&msg);

	res = imx_ele_call(&msg);
	if (res)
		panic("Cannot retrieve random data from ELE");

	return aslr;
}
#endif /* CFG_MX93 */
