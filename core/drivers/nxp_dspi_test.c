// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright 2020 NXP
 * Test suite for DSPI Controller
 */

#include <drivers/nxp_dspi.h>
#include <kernel/tee_time.h>
#include <mm/core_memprot.h>
#include <stdint.h>

extern struct nxp_dspi_data *dspi_data;

/* testing configuration for DSPI3 controller
 * Use FPGA configuration using i2c for board configuration
 *
 * Enable below configuration in RCW file as per DSPI to be used
 * For DSPI1 and DSPI2 add below configurations
 * SDHC1_BASE_PMUX=2
 * SDHC2_BASE_PMUX=2
 * For DSPI3 add below configurations
 * IIC5_PMUX=3
 * SDHC1_BASE_PMUX=3
 * SDHC1_DIR_PMUX=3
 * SDHC1_DS_PMUX=2
 */
static void dspi_test_suite(void)
{
	struct nxp_dspi_data *data = dspi_data;

	EMSG("DSPI TEST Starts");

	uint8_t tx[3] = {0x01, 0x80, 0x04};	/* TX array values */
	uint8_t rx[3] = {0};	/* RX array to be read */
	size_t i, j, len = 3;	/* length of data to be TX */
	enum spi_result res;

	DMSG("dspi_base: 0x%lx\n", data->base);

	/* set slave info */
	data->slave_bus		= 3;
	data->slave_cs		= 1;
	data->slave_speed_max_hz = 1000000;
	data->slave_mode	= 3;
	data->slave_data_size_bits = 8;

	data->chip.ops->end(&data->chip);	/* stop DSPI controller */
	data->chip.ops->configure(&data->chip);		/* configure DSPI chip instance */
	data->chip.ops->start(&data->chip);	/* start DSPI controller */

	for (j = 0; j < 1; j++) {
		EMSG("DSPI test loop: %zu", j);
		/* start TX/RX */
		res = data->chip.ops->txrx8(&data->chip, tx, rx, len);
		if (res) {
			EMSG("DSPI transceive error %d", res);
			break;
		}
		for (i = 0; i < len; i++)
			EMSG("rx[%zu] = 0x%x", i, rx[i]);

		tee_time_busy_wait(20);
	}
	data->chip.ops->end(&data->chip);	/* stop DSPI controller */
}

/*
 * nxp_dspi_init() MUST be run before calling this function!
 *
 * Verify read/write path of DSPI module by writing/reading on flash connected
 * to dSPI controller so the DSPI module will just receive
 * what is transmitted, i.e. 0x01, 0x80, 0x04.
 *
 */
void dspi_test(void)
{
	dspi_test_suite();
}
