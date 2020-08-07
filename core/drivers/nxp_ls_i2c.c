// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright 2020 NXP
 *
 * I2C driver for I2C Controller
 *
 */
#include <assert.h>
#include <drivers/nxp_gpio.h>
#include <io.h>
#include <string.h>
#include <kernel/dt.h>
#include <kernel/delay.h>
#include <kernel/generic_boot.h>
#include <mm/core_memprot.h>

#include <drivers/nxp_ls_i2c.h>

#ifdef CFG_DT
#include <libfdt.h>
#endif

#define	I2C_CTRL_PATH	17
#define	I2C_CTRL_NUM	8

typedef struct _i2c_controlle_path {
	char path[I2C_CTRL_PATH];
} i2c_controller_path;

static const i2c_controller_path i2c_controller_map[I2C_CTRL_NUM] = {
	{"/soc/i2c@2000000"}, {"/soc/i2c@2010000"}, {"/soc/i2c@2020000"},
	{"/soc/i2c@2030000"}, {"/soc/i2c@2040000"}, {"/soc/i2c@2050000"},
	{"/soc/i2c@2060000"}, {"/soc/i2c@2070000"}
};

/*
 * I2C divisor and ibfd register values when glitch filter is enabled
 * In case of duplicate SCL divisor value, the ibfd value with high MUL value
 * has been selected. A higher MUL value results in a lower sampling rate of
 * the I2C signals. This gives the I2C module greater immunity against glitches
 * in the I2C signals.
 */
static const i2c_clock_divisor_pair i2c_clock_divisor_glitch_enabled[] = {
	{ 34, 0x0 }, { 36, 0x1 }, { 38, 0x2 }, { 40, 0x3 },
	{ 42, 0x4 }, { 44, 0x8 }, { 48, 0x9 }, { 52, 0xA },
	{ 54, 0x7 }, { 56, 0xB }, { 60, 0xC }, { 64, 0x10 },
	{ 68, 0x40 }, { 72, 0x41 }, { 76, 0x42 }, { 80, 0x43 },
	{ 84, 0x44 }, { 88, 0x48 }, { 96, 0x49 }, { 104, 0x4A },
	{ 108, 0x47 }, { 112, 0x4B }, { 120, 0x4C }, { 128, 0x50 },
	{ 136, 0x80 }, { 144, 0x81 }, { 152, 0x82 }, { 160, 0x83 },
	{ 168, 0x84 }, { 176, 0x88 }, { 192, 0x89 }, { 208, 0x8A },
	{ 216, 0x87 }, { 224, 0x8B }, { 240, 0x8C }, { 256, 0x90 },
	{ 288, 0x91 }, { 320, 0x92 }, { 336, 0x8F }, { 352, 0x93 },
	{ 384, 0x98 }, { 416, 0x95 }, { 448, 0x99 }, { 480, 0x96 },
	{ 512, 0x9A }, { 576, 0x9B }, { 640, 0xA0 }, { 704, 0x9D },
	{ 768, 0xA1 }, { 832, 0x9E }, { 896, 0xA2 }, { 960, 0x67 },
	{ 1024, 0xA3 }, { 1152, 0xA4 }, { 1280, 0xA8 }, { 1536, 0xA9 },
	{ 1792, 0xAA }, { 1920, 0xA7 }, { 2048, 0xAB }, { 2304, 0xAC },
	{ 2560, 0xB0 }, { 3072, 0xB1 }, { 3584, 0xB2 }, { 3840, 0xAF },
	{ 4096, 0xB3 }, { 4608, 0xB4 }, { 5120, 0xB8 }, { 6144, 0xB9 },
	{ 7168, 0xBA }, { 7680, 0xB7 }, { 8192, 0xBB }, { 9216, 0xBC },
	{ 10240, 0xBD }, { 12288, 0xBE }, { 15360, 0xBF }
};

/*
 * I2C divisor and ibfd register values when glitch filter is disabled.
 * In case of duplicate SCL divisor value, the ibfd value with high MUL value
 * has been selected. A higher MUL value results in a lower sampling rate of
 * the I2C signals. This gives the I2C module greater immunity against glitches
 * in the I2C signals.
 */
static const i2c_clock_divisor_pair i2c_clock_divisor_glitch_disabled[] = {
	{ 20, 0x0 }, { 22, 0x1 }, { 24, 0x2 }, { 26, 0x3 },
	{ 28, 0x8 }, { 30, 0x5 }, { 32, 0x9 }, { 34, 0x6 },
	{ 36, 0x0A }, { 40, 0x40 }, { 44, 0x41 }, { 48, 0x42 },
	{ 52, 0x43 }, { 56, 0x48 }, { 60, 0x45 }, { 64, 0x49 },
	{ 68, 0x46 }, { 72, 0x4A }, { 80, 0x80 }, { 88, 0x81 },
	{ 96, 0x82 }, { 104, 0x83 }, { 112, 0x88 }, { 120, 0x85 },
	{ 128, 0x89 }, { 136, 0x86 }, { 144, 0x8A }, { 160, 0x8B },
	{ 176, 0x8C }, { 192, 0x90 }, { 208, 0x56 }, { 224, 0x91 },
	{ 240, 0x1F }, { 256, 0x92 }, { 272, 0x8F }, { 288, 0x93 },
	{ 320, 0x98 }, { 352, 0x95 }, { 384, 0x99 }, { 416, 0x96 },
	{ 448, 0x9A }, { 480, 0x5F }, { 512, 0x9B }, { 576, 0x9C },
	{ 640, 0xA0 }, { 768, 0xA1 }, { 896, 0xA2 }, { 960, 0x9F },
	{ 1024, 0xA3 }, { 1152, 0xA4 }, { 1280, 0xA8 }, { 1536, 0xA9 },
	{ 1792, 0xAA }, { 1920, 0xA7 }, { 2048, 0xAB }, { 2304, 0xAC },
	{ 2560, 0xAD }, { 3072, 0xB1 }, { 3584, 0xB2 }, { 3840, 0xAF },
	{ 4096, 0xB3 }, { 4608, 0xB4 }, { 5120, 0xB8 }, { 6144, 0xB9 },
	{ 7168, 0xBA }, { 7680, 0xB7 }, { 8192, 0xBB }, { 9216, 0xBC },
	{ 10240, 0xBD }, { 12288, 0xBE }, { 15360, 0xBF }
};

/*
 * ERR009203 :   I2C may not work reliably with the default setting
 * Description : The clocking circuitry of I2C module may not work reliably due
 * to the slow rise time of SCL signal.
 * Workaround :  Enable the receiver digital filter by setting IBDBG[GLFLT_EN]
 * to 1.
 */
static void i2c_erratum_a009203(vaddr_t  base)
{
	i2c_regs *regs;

	regs = (i2c_regs *)base;

	i2c_io_or8((vaddr_t)&regs->ibdbg, I2C_IBDBG_GLFLT_EN);
}

/*
 * software reset of the entire I2C module.
 * The module is reset and disabled.
 * Status register fields (IBSR) are cleared.

 * @param[in] Base       Base Address of I2c controller's registers

 * @return  TEE_SUCCESS  successfully reset the I2c module
 */
TEE_Result i2c_reset(vaddr_t  base)
{
	i2c_regs *regs;

	regs = (i2c_regs *)base;

	i2c_io_or8((vaddr_t)&regs->ibcr, I2C_IBCR_MDIS);
	i2c_io_or8((vaddr_t)&regs->ibsr, (I2C_IBSR_IBAL | I2C_IBSR_IBIF));
	i2c_io_and8((vaddr_t)&regs->ibcr, ~(I2C_IBCR_IBIE | I2C_IBCR_DMAEN));
	i2c_io_and8((vaddr_t)&regs->ibic, (uint8_t)(~I2C_IBIC_BIIE));

	return TEE_SUCCESS;
}

static uint8_t i2c_get_ibfd(vaddr_t  base, uint16_t clock_divisor)
{
	i2c_regs                       *regs;
	uint8_t                          ibfd; // I2c Bus Frequency Divider Register
	const i2c_clock_divisor_pair   *clock_divisor_pair;
	uint32_t                         clock_dvisor_pair_size;
	uint32_t                         index;

	ibfd = 0;
	regs = (i2c_regs *)base;

	if (io_read8((vaddr_t)&regs->ibdbg) & I2C_IBDBG_GLFLT_EN) {
		clock_divisor_pair = i2c_clock_divisor_glitch_enabled;
		clock_dvisor_pair_size = ARRAY_SIZE(i2c_clock_divisor_glitch_enabled);
	} else {
		clock_divisor_pair = i2c_clock_divisor_glitch_disabled;
		clock_dvisor_pair_size = ARRAY_SIZE(i2c_clock_divisor_glitch_disabled);
	}

	if (clock_divisor > clock_divisor_pair[clock_dvisor_pair_size - 1].divisor) {
		ibfd = clock_divisor_pair[clock_dvisor_pair_size - 1].ibfd;
	} else {
		for (index = 0; index < clock_dvisor_pair_size; index++) {
			if (clock_divisor_pair[index].divisor >= clock_divisor) {
				ibfd = clock_divisor_pair[index].ibfd;
				break;
			}
		}
	}

	return ibfd;
}

TEE_Result i2c_init(nxp_i2c_data *i2c_data)
{
	i2c_regs	*regs;
	uint16_t	clock_divisor;
	uint8_t		ibfd; // I2c Bus Frequency Divider Register
	paddr_t		paddr = 0;
	ssize_t		size = 0;
	int		node = 0;
	vaddr_t		ctrl_base;
	char		ctrl_path[I2C_CTRL_PATH];
	void		*fdt = NULL;

	memset(ctrl_path, 0, I2C_CTRL_PATH);
	strncpy(ctrl_path, i2c_controller_map[i2c_data->i2c_controller].path,
			I2C_CTRL_PATH);

	/*
	 * First get the I2C Controller base address from the DTB
	 * if DTB present and if the I2C Controller defined in it.
	 */
	fdt = get_embedded_dt();

	DMSG("I2C controller path = %s\n", ctrl_path);
	node = fdt_path_offset(fdt, ctrl_path);
	if (node > 0) {
		paddr = _fdt_reg_base_address(fdt, node);
		if (paddr == DT_INFO_INVALID_REG) {
			EMSG("I2C: Unable to get physical base address from device tree");
			return TEE_ERROR_ITEM_NOT_FOUND;
		}

		size = _fdt_reg_size(fdt, node);
		if (size < 0) {
			EMSG("I2C: Unable to get size of physical base address from device tree");
			return TEE_ERROR_ITEM_NOT_FOUND;
		}
	} else {
		EMSG("Unable to get I2C offset node");
		return TEE_ERROR_ITEM_NOT_FOUND;
	}

	DMSG("I2C base address = %lx, Size = %lx\n", paddr, size);

	/* making entry in page table */
	if (!core_mmu_add_mapping(MEM_AREA_IO_SEC, paddr, size)) {
		EMSG("I2C control base MMU PA mapping failure");
		return TEE_ERROR_ITEM_NOT_FOUND;
	}

	/* converting phyical address to virtual address */
	ctrl_base = (vaddr_t)phys_to_virt(paddr, MEM_AREA_IO_SEC);
	i2c_data->base = ctrl_base;

	regs = (i2c_regs *)ctrl_base;
	if (I2C_ERRATUM_A009203_FIX)
		i2c_erratum_a009203(ctrl_base);

	ibfd = 0;
	clock_divisor = (i2c_data->i2c_bus_clock + i2c_data->speed - 1)/i2c_data->speed;
	ibfd = i2c_get_ibfd(ctrl_base, clock_divisor);

	io_write8((vaddr_t)&regs->ibfd, ibfd);

	i2c_reset(ctrl_base);

	return TEE_SUCCESS;
}

static TEE_Result i2c_bus_test_bus_busy(i2c_regs *regs, bool test_busy)
{
	uint32_t  index;
	uint8_t   reg;

	for (index = 0; index < I2C_NUM_RETRIES; index++) {
		reg = io_read8((vaddr_t)&regs->ibsr);

		if (reg & I2C_IBSR_IBAL) {
			io_write8((vaddr_t)&regs->ibsr, reg);
			return TEE_ERROR_BUSY;
		}

		if (test_busy && (reg & I2C_IBSR_IBB))
			break;

		if (!test_busy && !(reg & I2C_IBSR_IBB))
			break;

		udelay(1);
	}

	if (index == I2C_NUM_RETRIES)
		return TEE_ERROR_BUSY;

	return TEE_SUCCESS;
}

static TEE_Result i2c_transfer_complete(i2c_regs *regs, bool test_rx_ack)
{
	uint32_t     index;
	uint8_t      reg;

	for (index = 0; index < I2C_NUM_RETRIES; index++) {
		reg = io_read8((vaddr_t)&regs->ibsr);

		if (reg & I2C_IBSR_IBIF) {
			// Write 1 to clear the IBIF field
			io_write8((vaddr_t)&regs->ibsr, reg);
			break;
		}
		udelay(1);
	}

	if (index == I2C_NUM_RETRIES)
		return TEE_ERROR_BUSY;

	if (test_rx_ack && (reg & I2C_IBSR_RXAK))
		return TEE_ERROR_NO_DATA;

	if (reg & I2C_IBSR_TCF)
		return TEE_SUCCESS;

	return TEE_ERROR_GENERIC;
}

static TEE_Result i2c_read(i2c_regs *regs, uint32_t slave_address,
		i2c_operation *operation, bool is_last_operation)
{
	TEE_Result status;
	uint32_t      index;

	// Write Slave Address
	io_write8((vaddr_t)&regs->ibdr, (slave_address << BIT(0)) | BIT(0));
	status = i2c_transfer_complete(regs, I2C_BUS_TEST_RX_ACK);
	if (status != TEE_SUCCESS)
		return status;

	// select Receive mode.
	i2c_io_and8((vaddr_t)&regs->ibcr, ~I2C_IBCR_TXRX);
	if (operation->length_in_bytes > 1) {
		// Set No ACK = 0
		i2c_io_and8((vaddr_t)&regs->ibcr, ~I2C_IBCR_NOACK);
	}

	// Perform a dummy read to initiate the receive operation.
	io_read8((vaddr_t)&regs->ibdr);

	for (index = 0; index < operation->length_in_bytes; index++) {
		status = i2c_transfer_complete(regs, I2C_BUS_NO_TEST_RX_ACK);
		if (status != TEE_SUCCESS)
			return status;
		if (index == (operation->length_in_bytes - 2)) {
			// Set No ACK = 1
			i2c_io_or8((vaddr_t)&regs->ibcr, I2C_IBCR_NOACK);
		} else if (index == (operation->length_in_bytes - 1)) {
			if (!is_last_operation) {
				// select Transmit mode (for repeat start)
				i2c_io_or8((vaddr_t)&regs->ibcr,
						I2C_IBCR_TXRX);
			} else {
				// Generate Stop Signal
				i2c_io_and8((vaddr_t)&regs->ibcr,
					~(I2C_IBCR_MSSL | I2C_IBCR_TXRX));
				status = i2c_bus_test_bus_busy(regs,
					I2C_BUS_TEST_IDLE);
				if (status != TEE_SUCCESS)
					return status;
			}
		}
		operation->buffer[index] = io_read8((vaddr_t)&regs->ibdr);
	}

	return TEE_SUCCESS;
}

static TEE_Result i2c_write(i2c_regs *regs, uint32_t slave_address,
		i2c_operation *operation)
{
	TEE_Result status = TEE_ERROR_GENERIC;
	uint32_t      index;

	// Write Slave Address
	io_write8((vaddr_t)&regs->ibdr,
			(slave_address << BIT(0)) & (uint8_t)(~(BIT(0))));
	status = i2c_transfer_complete(regs, I2C_BUS_TEST_RX_ACK);
	if (status != TEE_SUCCESS)
		return status;

	// Write Data
	for (index = 0; index < operation->length_in_bytes; index++) {
		io_write8((vaddr_t)&regs->ibdr, operation->buffer[index]);
		status = i2c_transfer_complete(regs, I2C_BUS_TEST_RX_ACK);
		if (status != TEE_SUCCESS)
			return status;
	}

	return TEE_SUCCESS;
}

static TEE_Result i2c_stop(i2c_regs  *regs)
{
	TEE_Result status;
	uint8_t      reg;

	status = TEE_SUCCESS;
	reg = io_read8((vaddr_t)&regs->ibsr);
	if (reg & I2C_IBSR_IBB) {
		// Generate Stop Signal
		i2c_io_and8((vaddr_t)&regs->ibcr,
				~(I2C_IBCR_MSSL | I2C_IBCR_TXRX));
		status = i2c_bus_test_bus_busy(regs, I2C_BUS_TEST_IDLE);
		if (status != TEE_SUCCESS)
			return status;
	}

	// Disable I2c Controller
	i2c_io_or8((vaddr_t)&regs->ibcr, I2C_IBCR_MDIS);

	return status;
}

static TEE_Result i2c_start(i2c_regs  *regs)
{
	TEE_Result status;

	i2c_io_or8((vaddr_t)&regs->ibsr, (I2C_IBSR_IBAL | I2C_IBSR_IBIF));
	i2c_io_and8((vaddr_t)&regs->ibcr, (uint8_t)(~I2C_IBCR_MDIS));

	//Wait controller to be stable
	udelay(1);

	// Generate Start Signal
	i2c_io_or8((vaddr_t)&regs->ibcr, I2C_IBCR_MSSL);
	status = i2c_bus_test_bus_busy(regs, I2C_BUS_TEST_BUSY);
	if (status != TEE_SUCCESS)
		return status;

	// Select Transmit Mode. set No ACK = 1
	i2c_io_or8((vaddr_t)&regs->ibcr, (I2C_IBCR_TXRX | I2C_IBCR_NOACK));

	return status;
}

/*
 * Transfer data to/from I2c slave device
 * Base           Base Address of I2c controller's registers
 * SlaveAddress   Slave Address from which data is to be read
 * RequestPacket  Pointer to an EFI_I2C_REQUEST_PACKET structure
 * describing the I2C transaction
 */
TEE_Result i2c_bus_xfer(vaddr_t base, uint32_t slave_address,
		i2c_reg_request *request_packet)
{
	uint32_t		index;
	i2c_regs		*regs;
	i2c_operation	*operation;
	TEE_Result	status;
	bool			is_last_operation;

	regs = (i2c_regs *)base;
	is_last_operation = FALSE;

	status = i2c_bus_test_bus_busy(regs, I2C_BUS_TEST_IDLE);
	if (status != TEE_SUCCESS)
		goto error_exit;

	status = i2c_start(regs);
	if (status != TEE_SUCCESS)
		goto error_exit;

	for (index = 0, operation = request_packet->operation;
			index < request_packet->operation_count;
			index++, operation++) {
		if (index == (request_packet->operation_count - 1))
			is_last_operation = TRUE;

		// Send repeat start after first transmit/receive
		if (index) {
			i2c_io_or8((vaddr_t)&regs->ibcr, I2C_IBCR_RSTA);
			status = i2c_bus_test_bus_busy(regs, I2C_BUS_TEST_BUSY);
			if (status != TEE_SUCCESS)
				goto error_exit;
		}

		// Read/write data
		if (operation->flags & I2C_FLAG_READ)
			status = i2c_read(regs, slave_address, operation,
					is_last_operation);
		else
			status = i2c_write(regs, slave_address, operation);
		if (status != TEE_SUCCESS)
			goto error_exit;
	}

error_exit:
	i2c_stop(regs);

	return status;
}
