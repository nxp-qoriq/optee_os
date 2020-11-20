// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright 2020 NXP
 *
 * I2C driver for I2C Controller
 *
 */

#ifndef NXP_LS_I2C_INTERNAL_H__
#define NXP_LS_I2C_INTERNAL_H__

#include <io.h>
#include <stdint.h>
#include <tee_api_types.h>

#define TRUE  1
#define FALSE 0

/* Module Disable
 * 0b - The module is enabled. You must clear this field before any other IBCR
 * fields have any effect.
 * 1b - The module is reset and disabled. This is the power-on reset situation.
 * When high, the interface is held in reset, but registers can still be
 * accessed. Status register fields (IBSR) are not valid when the module
 * is disabled.
 */
#define I2C_IBCR_MDIS 0x80

// I2c Bus Interrupt Enable
#define I2C_IBCR_IBIE 0x40

/* Master / Slave Mode 0b - Slave mode 1b - Master mode
 * When you change this field from 0 to 1, the module generates a START signal
 * on the bus and selects the master mode. When you change this field from 1 to
 * 0, the module generates a STOP signal and changes the operation mode from
 * master to slave. You should generate a STOP signal only if IBSR[IBIF]=1.
 * The module clears this field without generating a STOP signal when the
 * master loses arbitration.
 */
#define I2C_IBCR_MSSL 0x20

// 0b - Receive 1b - Transmit
#define I2C_IBCR_TXRX 0x10

/* Data acknowledge disable
 * Values written to this field are only used when the I2C module is a receiver,
 * not a transmitter.
 * 0b - The module sends an acknowledge signal to the bus at the 9th clock bit
 * after receiving one byte of data.
 * 1b - The module does not send an acknowledge-signal response (that is,
 * acknowledge bit = 1).
 */
#define I2C_IBCR_NOACK 0x08

/* Repeat START
 * If the I2C module is the current bus master, and you program RSTA=1, the I2C
 * module generates a repeated START condition. This field always reads as a 0.
 * If you attempt a repeated START at the wrong time, if the bus is owned by
 * another master the result is loss of arbitration.
 */
#define I2C_IBCR_RSTA 0x04

// DMA enable
#define I2C_IBCR_DMAEN 0x02

// Transfer Complete
#define I2C_IBSR_TCF 0x80

// I2C bus Busy. 0b - Bus is idle, 1b - Bus is busy
#define I2C_IBSR_IBB 0x20

// Arbitration Lost. software must clear this field by writing a one to it.
#define I2C_IBSR_IBAL 0x10

// I2C bus interrupt flag
#define I2C_IBSR_IBIF 0x02

// Received acknowledge 0b - Acknowledge received 1b - No acknowledge received
#define I2C_IBSR_RXAK 0x01

//Bus idle interrupt enable
#define I2C_IBIC_BIIE 0x80

// Glitch filter enable
#define I2C_IBDBG_GLFLT_EN 0x08

#define I2C_FLAG_READ 0x00000001

#define I2C_BUS_TEST_BUSY      TRUE
#define I2C_BUS_TEST_IDLE      !I2C_BUS_TEST_BUSY
#define I2C_BUS_TEST_RX_ACK    TRUE
#define I2C_BUS_NO_TEST_RX_ACK !I2C_BUS_TEST_RX_ACK

#define I2C_NUM_RETRIES	   500

#define I2C_ERRATUM_A009203_FIX 0

struct i2c_regs {
	uint8_t ibad;  // I2c Bus Address Register
	uint8_t ibfd;  // I2c Bus Frequency Divider Register
	uint8_t ibcr;  // I2c Bus Control Register
	uint8_t ibsr;  // I2c Bus Status Register
	uint8_t ibdr;  // I2C Bus Data I/O Register
	uint8_t ibic;  // I2C Bus Interrupt Config Register
	uint8_t ibdbg; // I2C Bus Debug Register
};

/*
 * sorted list of clock divisor, ibfd register value pairs
 */
struct i2c_clock_divisor_pair {
	uint16_t divisor;
	uint8_t ibfd; // I2c Bus Frequency Divider Register value
};

/*
 * I2C device operation
 * The EFI_I2C_OPERATION describes a subset of an I2C transaction in which
 * the I2C controller is either sending or receiving bytes from the bus.
 * Some transactions will consist of a single operation while others will
 * be two or more.
 */
struct i2c_operation {
	/* Flags to qualify the I2C operation. */
	uint32_t flags;

	/*
	 * Number of bytes to send to or receive from the I2C device.  A ping
	 * (address only byte/bytes)  is indicated by setting the LengthInBytes
	 * to zero.
	 */
	uint32_t length_in_bytes;

	/*
	 * Pointer to a buffer containing the data to send or to receive from
	 * the I2C device.  The Buffer must be at least LengthInBytes in size.
	 */
	uint8_t *buffer;
};

struct nxp_i2c_data {
	uint8_t i2c_controller;
	vaddr_t base;
	uint64_t i2c_bus_clock;
	uint64_t speed;
};

struct i2c_reg_request {
	uint32_t operation_count;
	struct i2c_operation operation[2];
};

static inline void i2c_io_or8(vaddr_t addr, uint8_t or_data)
{
	io_write8(addr, (io_read8(addr) | or_data));
}

static inline void i2c_io_and8(vaddr_t addr, uint8_t and_data)
{
	io_write8(addr, (io_read8(addr) & and_data));
}

TEE_Result i2c_init(struct nxp_i2c_data *i2c_data);
void i2c_reset(vaddr_t base);
TEE_Result i2c_bus_xfer(vaddr_t base, uint32_t slave_address,
			struct i2c_reg_request *request_packet);
void i2c_test(void);

#endif //NXP_LS_I2C_INTERNAL_H__
