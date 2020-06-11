// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright 2020 NXP
 *
 * Code for DSPI Controller driver
 *
 */

#include <assert.h>
#include <drivers/nxp_dspi.h>
#include <io.h>
#include <platform_config.h>
#include <kernel/dt.h>
#include <kernel/generic_boot.h>
#include <mm/core_memprot.h>
#include <kernel/delay.h>

#ifdef CFG_DT
#include <libfdt.h>
#endif

struct nxp_dspi_data *dspi_data;

/*
 * Calculate the divide scaler value between expected SCK frequency
 * and input clk frequency
 */
static TEE_Result dspi_convert_hz_to_baud(unsigned int *req_pbr,
	unsigned int *req_br, unsigned int speed_hz, unsigned int clkrate)
{
	/* Valid pre-scaler values for baud rate*/
	unsigned int pbr_val[4] = {2, 3, 5, 7};

	/* Valid baud rate scaler values*/
	unsigned int brs_val[16] = {2, 4, 6, 8, 16, 32, 64, 128, 256, 512,
		1024, 2048, 4096, 8192, 16384, 32768};

	unsigned int tmp_val = 0, curr_val = 0;
	unsigned int i = 0, j = 0;
	tmp_val = clkrate / speed_hz;

	for (i = 0; i < ARRAY_SIZE(pbr_val); i++) {
		for (j = 0; j < ARRAY_SIZE(brs_val); j++) {
			curr_val = pbr_val[i] * brs_val[j];
			if (curr_val >= tmp_val) {
				*req_pbr = i;
				*req_br = j;
				return TEE_SUCCESS;
			}
		}
	}

	EMSG("Can not find valid baud rate,speed_hz is %d, ", speed_hz);
	EMSG("clkrate is %d, we use the max prescaler value.\n", clkrate);

	*req_pbr = ARRAY_SIZE(pbr_val) - 1;
	*req_br =  ARRAY_SIZE(brs_val) - 1;
	return TEE_ERROR_ITEM_NOT_FOUND;
}

/* setup speed for slave */
static void dspi_setup_speed(unsigned int speed)
{
	TEE_Result status = TEE_ERROR_GENERIC;
	unsigned int bus_setup, bus_clock;
	unsigned int req_i = 0, req_j = 0;

	bus_clock = dspi_data->bus_clk_hz;

	EMSG("DSPI set_speed: expected SCK speed %u, bus_clk %u.\n",
			speed, bus_clock);

	bus_setup = io_read32(dspi_data->base + DSPI_CTAR0);
	bus_setup &= ~(DSPI_CTAR_DBR | DSPI_CTAR_PBR(0x3) | DSPI_CTAR_BR(0xf));

	status = dspi_convert_hz_to_baud(&req_i, &req_j, speed, bus_clock);
	if (status == TEE_ERROR_ITEM_NOT_FOUND) {
		speed = dspi_data->speed_hz;
		EMSG("DSPI set_speed use default SCK rate %u.\n", speed);
		dspi_convert_hz_to_baud(&req_i, &req_j, speed, bus_clock);
	}

	bus_setup |= (DSPI_CTAR_PBR(req_i) | DSPI_CTAR_BR(req_j));
	io_write32(dspi_data->base + DSPI_CTAR0, bus_setup);

	dspi_data->speed_hz = speed;
}

/* Transferred data to TX FIFO */
static void dspi_tx(uint32_t ctrl, uint8_t data)
{
	int timeout = DSPI_TXRX_WAIT_TIMEOUT;
	int counter = DSPI_SR_TXCTR(io_read32(dspi_data->base + DSPI_SR));
	uint32_t dspi_val_addr = dspi_data->base + DSPI_PUSHR;
	uint32_t dspi_val = ctrl | data;

	/* wait for empty entries in TXFIFO or timeout */
	while (counter >= 4 && timeout--)
		udelay(1);

	if (timeout >= 0)
		io_write32(dspi_val_addr, dspi_val);
	else
		EMSG("dspi_tx: waiting timeout!");
}

/* Read data from RX FIFO */
static uint16_t dspi_rx(void)
{
	int timeout = DSPI_TXRX_WAIT_TIMEOUT;
	int counter = DSPI_SR_RXCTR(io_read32(dspi_data->base + DSPI_SR));
	uint32_t dspi_val_addr = dspi_data->base + DSPI_POPR;
	uint32_t dspi_val;

	/* wait for valid entries in RXFIFO or timeout */
	while (counter == 0 && timeout--)
		udelay(1);

	if (timeout >= 0) {
		dspi_val = io_read32(dspi_val_addr);
		return (uint16_t)DSPI_RFR_RXDATA(dspi_val);
	} else {
		EMSG("dspi_rx: waiting timeout!\n");
		return (uint16_t)(~0);
	}
}

static enum spi_result nxp_dspi_txrx8(struct spi_chip *chip, uint8_t *wdata,
		uint8_t *rdata, size_t num_pkts)
{
	size_t i = 0;
	size_t j = 0;
	static uint32_t ctrl;

	struct nxp_dspi_data *data = container_of(chip, struct nxp_dspi_data,
			chip);

	/*
	* select which CTAR register and slave to be used for TX
	* CTAS selects which CTAR to be used, here we are using CTAR0
	* PCS (peripheral chip select) is selecting the slave.
	*/
	ctrl = ctrl | DSPI_TFR_CTAS(0) | DSPI_TFR_PCS(data->slave_cs);

	if (data->slave_data_size_bits != 8) {
		EMSG("data_size_bits should be 8, not %u",
				data->slave_data_size_bits);
		return SPI_ERR_CFG;
	}

	if (wdata) {
		while (i < num_pkts) {
			dspi_tx(ctrl, wdata[i++]);	/* tx 1 packet */
			if (rdata)
				rdata[j++] = dspi_rx();	/* rx 1 packet */
		}
	}

	/* Capture remaining rdata not read above */
	if (rdata) {
		while (j < num_pkts) {
			rdata[j++] = dspi_rx();	/* rx 1 packet */
		}
		if (j < num_pkts) {
			EMSG("Packets requested %zu, received %zu",
					num_pkts, j);
			return SPI_ERR_PKTCNT;
		}
	}

	return SPI_OK;
}

static enum spi_result nxp_dspi_txrx16(struct spi_chip *chip, uint16_t *wdata,
		uint16_t *rdata, size_t num_pkts)
{
	size_t i = 0;
	size_t j = 0;
	static uint32_t ctrl;

	struct nxp_dspi_data *data = container_of(chip, struct nxp_dspi_data,
			chip);

	/*
	* select which CTAR register and slave to be used for TX
	* CTAS selects which CTAR to be used, here we are using CTAR0
	* PCS (peripheral chip select) is selecting the slave.
	*/
	ctrl = ctrl | DSPI_TFR_CTAS(0) | DSPI_TFR_PCS(data->slave_cs);

	if (data->slave_data_size_bits != 16) {
		EMSG("data_size_bits should be 16, not %u",
				data->slave_data_size_bits);
		return SPI_ERR_CFG;
	}

	if (wdata) {
		while (i < num_pkts) {
			dspi_tx(ctrl, wdata[i++]);	/* tx 1 packet */
			if (rdata)
				rdata[j++] = dspi_rx();	/* rx 1 packet */
		}
	}

	/* Capture remaining rdata not read above */
	if (rdata) {
		while (j < num_pkts) {
			rdata[j++] = dspi_rx();	/* rx 1 packet */
		}
		if (j < num_pkts) {
			EMSG("Packets requested %zu, received %zu",
					num_pkts, j);
			return SPI_ERR_PKTCNT;
		}
	}

	return SPI_OK;
}

static void nxp_dspi_start(struct spi_chip *chip __unused)
{
	unsigned int mcr_val = 0;

	mcr_val  = io_read32(dspi_data->base + DSPI_MCR);
	mcr_val &= ~DSPI_MCR_HALT;

	EMSG("Start DSPI Module");
	io_write32(dspi_data->base + DSPI_MCR, mcr_val);
}

static void nxp_dspi_end(struct spi_chip *chip __unused)
{
	unsigned int mcr_val = 0;

	mcr_val  = io_read32(dspi_data->base + DSPI_MCR);
	mcr_val |= DSPI_MCR_HALT;

	EMSG("Stop DSPI Module");
	io_write32(dspi_data->base + DSPI_MCR, mcr_val);
}

static void dspi_flush_fifo(void)
{
	unsigned int mcr_val = 0;

	mcr_val  = io_read32(dspi_data->base + DSPI_MCR);

	/* flush RX and TX FIFO */
	mcr_val |= (DSPI_MCR_CTXF | DSPI_MCR_CRXF);

	io_write32(dspi_data->base + DSPI_MCR, mcr_val);
}

static void dspi_set_cs_active_state(unsigned int cs, unsigned int state)
{
	EMSG("Set CS active state");
	unsigned int mcr_val = 0;

	mcr_val = io_read32(dspi_data->base + DSPI_MCR);

	if (state & SPI_CS_HIGH)
		/* CSx inactive state is low */
		mcr_val &= ~DSPI_MCR_PCSIS(cs);
	else
		/* CSx inactive state is high */
		mcr_val |= DSPI_MCR_PCSIS(cs);

	io_setbits32(dspi_data->base + DSPI_MCR, mcr_val);
}

static void dspi_set_transfer_state(unsigned int cs, unsigned int state)
{
	EMSG("Set transfer state");
	unsigned int bus_setup = 0;

	bus_setup = io_read32(dspi_data->base + DSPI_CTAR0);

	bus_setup &= ~DSPI_CTAR_SET_MODE_MASK;
	bus_setup |= dspi_data->ctar_val[cs];
	bus_setup &= ~(DSPI_CTAR_CPOL | DSPI_CTAR_CPHA | DSPI_CTAR_LSBFE);

	if (state & SPI_CPOL)
		bus_setup |= DSPI_CTAR_CPOL;
	if (state & SPI_CPHA)
		bus_setup |= DSPI_CTAR_CPHA;
	if (state & SPI_LSB_FIRST)
		bus_setup |= DSPI_CTAR_LSBFE;

	io_write32(dspi_data->base + DSPI_CTAR0, bus_setup);

	dspi_data->slave_data_size_bits =
		((io_read32(dspi_data->base + DSPI_CTAR0) &
		  DSPI_CTAR_FMSZ(15)) == DSPI_CTAR_FMSZ(15)) ? 16 : 8;
}

static void dspi_set_speed(unsigned int speed_max_hz)
{
	EMSG("Set speed");
	dspi_setup_speed(speed_max_hz);
}

static void dspi_config_slave_state(unsigned int bus __unused, unsigned int cs,
		unsigned int speed_max_hz, unsigned int state)
{
	unsigned int sr_val = 0;
	int i;

	/* set default value in clock and transfer attributes register */
	for (i = 0; i < FSL_DSPI_MAX_CHIPSELECT; i++)
		dspi_data->ctar_val[i] = DSPI_CTAR_DEFAULT_VALUE;

	/* configure speed */
	dspi_set_speed(speed_max_hz);

	/* configure transfer state */
	dspi_set_transfer_state(cs, state);

	/* configure active state of CSX */
	dspi_set_cs_active_state(cs, state);

	/* clear FIFO*/
	dspi_flush_fifo();

	/* check module TX and RX status */
	sr_val = io_read32(dspi_data->base + DSPI_SR);
	if ((sr_val & DSPI_SR_TXRXS) != DSPI_SR_TXRXS) {
		EMSG("DSPI RX/TX not ready.");
	}
}

static void dspi_set_master_state(unsigned int mcr_val)
{
	EMSG("Set master state");
	io_write32(dspi_data->base + DSPI_MCR, mcr_val);
}

static void nxp_dspi_configure(struct spi_chip *chip)
{
	struct nxp_dspi_data *data = container_of(chip, struct nxp_dspi_data,
			chip);
	unsigned int mcr_cfg_val;

	mcr_cfg_val = DSPI_MCR_MSTR | DSPI_MCR_PCSIS_MASK |
		DSPI_MCR_CRXF | DSPI_MCR_CTXF;

	/* Configure Master */
	dspi_set_master_state(mcr_cfg_val);

	/* Configure DSPI slave */
	dspi_config_slave_state(data->slave_bus, data->slave_cs,
			data->slave_speed_max_hz, data->slave_mode);
}

static TEE_Result get_info_from_device_tree(void)
{
	const fdt32_t *bus_num = NULL;
	const fdt32_t *chip_select_num = NULL;
	paddr_t paddr = 0;
	ssize_t size = 0;
	int node = 0;
	vaddr_t ctrl_base;

	/*
	 * First get the DSPI Controller base address from the DTB
	 * if DTB present and if the DSPI Controller defined in it.
	 */
	void *fdt = get_embedded_dt();
	node = fdt_path_offset(fdt, "/soc/spi@2120000");

	if (node < 0)
		node = fdt_path_offset(fdt, "/spi@2120000");
	if (node > 0) {
		paddr = _fdt_reg_base_address(fdt, node);
		if (paddr == DT_INFO_INVALID_REG) {
			EMSG("Unable to get physical base address from device tree");
			return TEE_ERROR_ITEM_NOT_FOUND;
		}

		size = _fdt_reg_size(fdt, node);
		if (size < 0) {
			EMSG("Unable to get size of physical base address from device tree");
			return TEE_ERROR_ITEM_NOT_FOUND;;
		}
	} else {
		EMSG("Unable to get DSPI offset node");
		return TEE_ERROR_ITEM_NOT_FOUND;
	}

	/* making entry in page table */
	if (!core_mmu_add_mapping(MEM_AREA_IO_NSEC, paddr, size)) {
		EMSG("DSPI control base MMU PA mapping failure");
		return TEE_ERROR_ITEM_NOT_FOUND;
	}

	/* converting phyical address to virtual address */
	ctrl_base = (vaddr_t)phys_to_virt(paddr, MEM_AREA_IO_NSEC);

	if (ctrl_base > 0)
		dspi_data->base = ctrl_base;
	else {
		EMSG("Unable to get virtual address from DSPI controller");
		return TEE_ERROR_GENERIC;
	}

	dspi_data->bus_clk_hz = DSPI_CLK;

	bus_num = fdt_getprop(fdt, node, "bus-num", NULL);
	chip_select_num = fdt_getprop(fdt, node, "spi-num-chipselects", NULL);

	dspi_data->num_bus = (int)fdt32_to_cpu(*bus_num);
	dspi_data->num_chipselect = (int)fdt32_to_cpu(*chip_select_num);
	dspi_data->speed_hz = DSPI_DEFAULT_SCK_FREQ;

	return TEE_SUCCESS;
}

static const struct spi_ops nxp_dspi_ops = {
	.configure = nxp_dspi_configure,
	.start = nxp_dspi_start,
	.txrx8 = nxp_dspi_txrx8,
	.txrx16 = nxp_dspi_txrx16,
	.end = nxp_dspi_end,
};
DECLARE_KEEP_PAGER(nxp_dspi_ops);

/*
 * Register NXP-Layerscape DSPI controller
 */
static TEE_Result layerscape_dspi_init(void)
{
	/* generic DSPI chip handle */
	dspi_data->chip.ops = &nxp_dspi_ops;

#ifdef CFG_NXP_DSPI_TEST
	dspi_test();
#endif
	return TEE_SUCCESS;
}

/*
 * Initialise NXP DSPI controller
 */
static TEE_Result nxp_dspi_init(void)
{
	TEE_Result status = TEE_ERROR_GENERIC;

	/* allocate memory for DSPI */
	dspi_data = (struct nxp_dspi_data *)malloc(sizeof(
				struct nxp_dspi_data));

	if (dspi_data == NULL) {
		EMSG("Unable to init DSPI");
		return TEE_ERROR_OUT_OF_MEMORY;
	}

	/*
	 * First get the DSPI Controller base address from the DTB,
	 * if DTB present and if the DSPI Controller defined in it.
	 */
	status = get_info_from_device_tree();

	/* Register DSPI Controller */
	if (status == TEE_SUCCESS)
		status = layerscape_dspi_init();
	else {
		EMSG("Unable to get info from device tree");
		/* free allocated memorry for DSPI*/
		free(dspi_data);
	}

	return status;
}

/* Initialise DSPI driver */
driver_init(nxp_dspi_init);
