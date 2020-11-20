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
#include <kernel/boot.h>
#include <kernel/delay.h>
#include <kernel/dt.h>
#include <libfdt.h>
#include <mm/core_memprot.h>
#include <platform_config.h>
#include <util.h>

#define NXP_DSPI_COMPATIBLE "fsl,lx2160a-dspi"

/*
 * Calculate the divide scaler value between expected SCK frequency
 * and input clk frequency
 */
static TEE_Result dspi_convert_hz_to_baud(unsigned int *req_pbr,
					  unsigned int *req_br,
					  unsigned int speed_hz,
					  unsigned int clkrate)
{
	/* Valid pre-scaler values for baud rate*/
	static const unsigned int pbr_val[4] = { 2, 3, 5, 7 };

	/* Valid baud rate scaler values*/
	static const unsigned int brs_val[16] = { 2, 4, 6, 8,
						16, 32, 64, 128,
						256, 512, 1024, 2048,
						4096, 8192, 16384, 32768 };

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
	EMSG("clkrate is %d, we use the max prescaler value.", clkrate);

	return TEE_ERROR_ITEM_NOT_FOUND;
}

/* setup speed for slave */
static void dspi_setup_speed(struct nxp_dspi_data *dspi_data,
			     unsigned int speed)
{
	TEE_Result status = TEE_ERROR_GENERIC;
	unsigned int bus_setup = 0, bus_clock = 0;
	unsigned int req_i = 0, req_j = 0;

	bus_clock = dspi_data->bus_clk_hz;

	DMSG("DSPI set_speed: expected SCK speed %u, bus_clk %u.", speed,
	     bus_clock);

	bus_setup = io_read32(dspi_data->base + DSPI_CTAR0);
	bus_setup &= ~(DSPI_CTAR_BRD | DSPI_CTAR_BRP(0x3) | DSPI_CTAR_BR(0xf));

	status = dspi_convert_hz_to_baud(&req_i, &req_j, speed, bus_clock);

	/* In case of failure scenario with max speed, setting default speed */
	if (status == TEE_ERROR_ITEM_NOT_FOUND) {
		speed = dspi_data->speed_hz;
		EMSG("DSPI set_speed use default SCK rate %u.", speed);
		status = dspi_convert_hz_to_baud(&req_i, &req_j,
						 speed, bus_clock);
	}

	if (status == TEE_SUCCESS) {
		bus_setup |= (DSPI_CTAR_BRP(req_i) | DSPI_CTAR_BR(req_j));
		io_write32(dspi_data->base + DSPI_CTAR0, bus_setup);
		dspi_data->speed_hz = speed;
	} else {
		EMSG("Unable to set speed.");
	}
}

/* Transferred data to TX FIFO */
static void dspi_tx(struct nxp_dspi_data *dspi_data, uint32_t ctrl,
		    uint16_t data)
{
	int timeout = DSPI_TXRX_WAIT_TIMEOUT;
	uint32_t dspi_val_addr = dspi_data->base + DSPI_PUSHR;
	uint32_t dspi_val = ctrl | data;

	/* wait for empty entries in TXFIFO or timeout */
	while (DSPI_SR_TXCTR(io_read32(dspi_data->base + DSPI_SR)) >= 4 &&
	       timeout--)
		mdelay(1000);

	if (timeout >= 0)
		io_write32(dspi_val_addr, dspi_val);
	else
		EMSG("waiting timeout!");
}

/* Read data from RX FIFO */
static uint16_t dspi_rx(struct nxp_dspi_data *dspi_data)
{
	int timeout = DSPI_TXRX_WAIT_TIMEOUT;
	uint32_t dspi_val_addr = dspi_data->base + DSPI_POPR;

	/* wait for valid entries in RXFIFO or timeout */
	while (DSPI_SR_RXCTR(io_read32(dspi_data->base + DSPI_SR)) == 0 &&
	       timeout--)
		mdelay(1000);

	if (timeout >= 0)
		return (uint16_t)DSPI_RFR_RXDATA(io_read32(dspi_val_addr));

	EMSG("waiting timeout!");
	return (uint16_t)(~0);
}

static enum spi_result nxp_dspi_txrx8(struct spi_chip *chip, uint8_t *wdata,
				      uint8_t *rdata, size_t num_pkts)
{
	uint8_t *spi_rd = NULL, *spi_wr = NULL;
	uint32_t ctrl = 0;

	spi_wr = wdata;
	spi_rd = rdata;

	struct nxp_dspi_data *data = container_of(chip, struct nxp_dspi_data,
						  chip);
	unsigned int cs = data->slave_cs;

	/*
	 * Assert PCSn signals between transfers
	 * select which CTAR register and slave to be used for TX
	 * CTAS selects which CTAR to be used, here we are using CTAR0
	 * PCS (peripheral chip select) is selecting the slave.
	 */
	ctrl = DSPI_TFR_CTAS(data->ctar_sel) | DSPI_TFR_PCS(cs);
	if (data->slave_mode & SPI_CONT)
		ctrl |= DSPI_TFR_CONT;

	if (data->slave_data_size_bits != 8) {
		EMSG("data_size_bits should be 8, not %u",
		     data->slave_data_size_bits);
		return SPI_ERR_CFG;
	}

	while (num_pkts) {
		if (wdata && rdata) {
			dspi_tx(data, ctrl, *spi_wr++);
			*spi_rd++ = dspi_rx(data);
		} else if (wdata) {
			dspi_tx(data, ctrl, *spi_wr++);
			dspi_rx(data);
		} else if (rdata) {
			dspi_tx(data, ctrl, DSPI_IDLE_DATA);
			*spi_rd++ = dspi_rx(data);
		}
		num_pkts = num_pkts - 1;
	}

	return SPI_OK;
}

static enum spi_result nxp_dspi_txrx16(struct spi_chip *chip, uint16_t *wdata,
				       uint16_t *rdata, size_t num_pkts)
{
	uint32_t ctrl = 0;
	uint16_t *spi_rd = NULL, *spi_wr = NULL;

	spi_wr = wdata;
	spi_rd = rdata;

	struct nxp_dspi_data *data = container_of(chip, struct nxp_dspi_data,
						  chip);
	unsigned int cs = data->slave_cs;

	/*
	 * Assert PCSn signals between transfers
	 * select which CTAR register and slave to be used for TX
	 * CTAS selects which CTAR to be used, here we are using CTAR0
	 * PCS (peripheral chip select) is selecting the slave.
	 */
	ctrl = DSPI_TFR_CTAS(data->ctar_sel) | DSPI_TFR_PCS(cs);
	if (data->slave_mode & SPI_CONT)
		ctrl |= DSPI_TFR_CONT;

	if (data->slave_data_size_bits != 16) {
		EMSG("data_size_bits should be 16, not %u",
		     data->slave_data_size_bits);
		return SPI_ERR_CFG;
	}

	while (num_pkts) {
		if (wdata && rdata) {
			dspi_tx(data, ctrl, *spi_wr++);
			*spi_rd++ = dspi_rx(data);
		} else if (wdata) {
			dspi_tx(data, ctrl, *spi_wr++);
			dspi_rx(data);
		} else if (rdata) {
			dspi_tx(data, ctrl, DSPI_IDLE_DATA);
			*spi_rd++ = dspi_rx(data);
		}
		num_pkts = num_pkts - 1;
	}

	return SPI_OK;
}

static void nxp_dspi_start(struct spi_chip *chip)
{
	struct nxp_dspi_data *data = container_of(chip, struct nxp_dspi_data,
						  chip);

	DMSG("Start DSPI Module");
	io_clrbits32(data->base + DSPI_MCR, DSPI_MCR_HALT);
}

static void nxp_dspi_end(struct spi_chip *chip)
{
	struct nxp_dspi_data *data = container_of(chip, struct nxp_dspi_data,
						  chip);

	/* De-assert PCSn if in CONT mode */
	if (data->slave_mode & SPI_CONT) {
		unsigned int cs = data->slave_cs;
		unsigned int ctrl = DSPI_TFR_CTAS(data->ctar_sel) |
				    DSPI_TFR_PCS(cs);

		/* Dummy read to deassert */
		dspi_tx(data, ctrl, DSPI_IDLE_DATA);
		dspi_rx(data);
	}

	DMSG("Stop DSPI Module");
	io_setbits32(data->base + DSPI_MCR, DSPI_MCR_HALT);
}

void dspi_flush_fifo(struct nxp_dspi_data *dspi_data)
{
	unsigned int mcr_val = 0;

	mcr_val = io_read32(dspi_data->base + DSPI_MCR);

	/* flush RX and TX FIFO */
	mcr_val |= (DSPI_MCR_CTXF | DSPI_MCR_CRXF);

	io_write32(dspi_data->base + DSPI_MCR, mcr_val);
}

static void dspi_set_cs_active_state(struct nxp_dspi_data *dspi_data,
				     unsigned int cs, unsigned int state)
{
	DMSG("Set CS active state cs=%d state=%d", cs, state);

	if (state & SPI_CS_HIGH)
		/* CSx inactive state is low */
		io_clrbits32(dspi_data->base + DSPI_MCR, DSPI_MCR_PCSIS(cs));
	else
		/* CSx inactive state is high */
		io_setbits32(dspi_data->base + DSPI_MCR, DSPI_MCR_PCSIS(cs));
}

static void dspi_set_transfer_state(struct nxp_dspi_data *dspi_data,
				    unsigned int state)
{
	DMSG("Set transfer state state=%d bits=%d", state,
	     dspi_data->slave_data_size_bits);
	unsigned int bus_setup = 0;

	bus_setup = io_read32(dspi_data->base + DSPI_CTAR0);

	bus_setup &= ~DSPI_CTAR_SET_MODE_MASK;
	bus_setup |= dspi_data->ctar_val;
	bus_setup &= ~(DSPI_CTAR_CPOL | DSPI_CTAR_CPHA | DSPI_CTAR_LSBFE);

	if (state & SPI_CPOL)
		bus_setup |= DSPI_CTAR_CPOL;
	if (state & SPI_CPHA)
		bus_setup |= DSPI_CTAR_CPHA;
	if (state & SPI_LSB_FIRST)
		bus_setup |= DSPI_CTAR_LSBFE;

	if (dspi_data->slave_data_size_bits == 8)
		bus_setup |= DSPI_CTAR_FMSZ(7);
	else if (dspi_data->slave_data_size_bits == 16)
		bus_setup |= DSPI_CTAR_FMSZ(15);

	if (dspi_data->ctar_sel == 0)
		io_write32(dspi_data->base + DSPI_CTAR0, bus_setup);
	else
		io_write32(dspi_data->base + DSPI_CTAR1, bus_setup);
}

static void dspi_set_speed(struct nxp_dspi_data *dspi_data,
			   unsigned int speed_max_hz)
{
	DMSG("Set speed %d", speed_max_hz);
	dspi_setup_speed(dspi_data, speed_max_hz);
}

static void dspi_config_slave_state(struct nxp_dspi_data *dspi_data,
				    unsigned int cs, unsigned int speed_max_hz,
				    unsigned int state)
{
	unsigned int sr_val = 0;

	/* configure speed */
	dspi_set_speed(dspi_data, speed_max_hz);

	/* configure transfer state */
	dspi_set_transfer_state(dspi_data, state);

	/* configure active state of CSX */
	dspi_set_cs_active_state(dspi_data, cs, state);

	/* clear FIFO*/
	dspi_flush_fifo(dspi_data);

	/* check module TX and RX status */
	sr_val = io_read32(dspi_data->base + DSPI_SR);
	if ((sr_val & DSPI_SR_TXRXS) != DSPI_SR_TXRXS)
		EMSG("DSPI RX/TX not ready.");
}

static void dspi_set_master_state(struct nxp_dspi_data *dspi_data,
				  unsigned int mcr_val)
{
	DMSG("Set master state val=0x%x", mcr_val);
	io_write32(dspi_data->base + DSPI_MCR, mcr_val);
}

static void nxp_dspi_configure(struct spi_chip *chip)
{
	struct nxp_dspi_data *data = container_of(chip, struct nxp_dspi_data,
						  chip);
	unsigned int mcr_cfg_val = 0;

	mcr_cfg_val = DSPI_MCR_MSTR | DSPI_MCR_PCSIS_MASK | DSPI_MCR_CRXF |
		      DSPI_MCR_CTXF;

	/* Configure Master */
	dspi_set_master_state(data, mcr_cfg_val);

	/* Configure DSPI slave */
	dspi_config_slave_state(data, data->slave_cs, data->slave_speed_max_hz,
				data->slave_mode);
}

static TEE_Result get_info_from_device_tree(struct nxp_dspi_data *dspi_data)
{
	const fdt32_t *bus_num = NULL;
	const fdt32_t *chip_select_num = NULL;
	size_t size = 0;
	int node = 0;
	vaddr_t ctrl_base = 0;

	/*
	 * First get the DSPI Controller base address from the DTB
	 * if DTB present and if the DSPI Controller defined in it.
	 */
	void *fdt = get_dt();

	if (!fdt) {
		EMSG("No Device Tree found");
		return TEE_ERROR_ITEM_NOT_FOUND;
	}

	node = 0;
	while (node != -FDT_ERR_NOTFOUND) {
		node = fdt_node_offset_by_compatible(fdt, node,
						     NXP_DSPI_COMPATIBLE);
		if (!(_fdt_get_status(fdt, node) & DT_STATUS_OK_SEC))
			continue;

		bus_num = fdt_getprop(fdt, node, "bus-num", NULL);
		if (bus_num && dspi_data->slave_bus ==
			(unsigned int)fdt32_to_cpu(*bus_num)) {
			if (dt_map_dev(fdt, node, &ctrl_base, &size) < 0) {
				EMSG("Unable to get virtual address");
				return TEE_ERROR_GENERIC;
			}
			break;
		}
	}

	dspi_data->base = ctrl_base;

	dspi_data->bus_clk_hz = DSPI_CLK;

	chip_select_num = fdt_getprop(fdt, node, "spi-num-chipselects", NULL);
	if (chip_select_num)
		dspi_data->num_chipselect = (int)fdt32_to_cpu(*chip_select_num);
	else
		return TEE_ERROR_ITEM_NOT_FOUND;

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
 * Initialise NXP DSPI controller
 */
TEE_Result nxp_dspi_init(struct nxp_dspi_data *dspi_data)
{
	TEE_Result status = TEE_ERROR_GENERIC;

	/*
	 * First get the DSPI Controller base address from the DTB,
	 * if DTB present and if the DSPI Controller defined in it.
	 */
	if (dspi_data)
		status = get_info_from_device_tree(dspi_data);

	/* Register DSPI Controller */
	if (status == TEE_SUCCESS)
		/* generic DSPI chip handle */
		dspi_data->chip.ops = &nxp_dspi_ops;
	else
		EMSG("Unable to get info from device tree");

	return status;
}
