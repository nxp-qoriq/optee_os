/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright NXP 2023
 */

#ifndef __ELE_H_
#define __ELE_H_

#include <drivers/imx_mu.h>
#include <tee_api_types.h>
#include <trace.h>

/* Definitions for communication protocol */
#define ELE_VERSION_HSM 0x07
#define ELE_REQUEST_TAG 0x17

static inline size_t size_msg(size_t cmd)
{
	size_t words = ROUNDUP(cmd, sizeof(uint32_t)) / sizeof(uint32_t);

	/* Add the header size */
	words = words + 1;

	return words;
}

#define SIZE_MSG_32(_msg) size_msg(sizeof(_msg))

/*
 * The CRC is the last word of the message
 *
 * msg: MU message to hash
 */
void update_crc(struct imx_mu_msg *msg);

/*
 * Initiate a communication with the EdgeLock Enclave. It sends a message
 * and expects an answer.
 *
 * @msg MU message
 */
TEE_Result imx_ele_call(struct imx_mu_msg *msg);
TEE_Result imx_ele_get_global_session_handle(uint32_t *session_handle);

/*
 * Open a Key Management session with EdgeLock Enclave.
 *
 * @key_store_handle: EdgeLock Enclave key store handle
 * @key_mgmt_handle: EdgeLock Enclave Key management handle
 */
TEE_Result imx_ele_key_mgmt_open(uint32_t key_store_handle,
				 uint32_t *key_mgmt_handle);

/*
 * Close Key management with EdgeLock Enclave.
 *
 * @key_mgmt_handle: EdgeLock Enclave key management handle
 */
TEE_Result imx_ele_key_mgmt_close(uint32_t key_mgmt_handle);

#endif /* __ELE_H_ */
