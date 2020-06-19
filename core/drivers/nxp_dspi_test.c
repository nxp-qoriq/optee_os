// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright 2020 NXP
 * Test suite for DSPI Controller
 */

#include <drivers/nxp_dspi.h>
#include <kernel/tee_time.h>
#include <mm/core_memprot.h>
#include <stdint.h>

/* Testing configuration for DSPI3 controller
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
static TEE_Result dspi_test_suite(void)
{
	struct nxp_dspi_data dspi_data;
	TEE_Result status = TEE_ERROR_GENERIC;

	EMSG("DSPI TEST Starts");

	uint8_t tx[4] = {0xff, 0x22, 0x33, 0x44};	/* TX array values */
	uint8_t rx[4] = {0};	/* RX array to be read */
	size_t i, j, len = 4;	/* length of data to be TX */
	enum spi_result res;

	/* set slave info */
	dspi_data.slave_bus		= 3;
	dspi_data.slave_cs		= 0;
	dspi_data.slave_speed_max_hz = 1000000;
	dspi_data.slave_mode	= 3;
	dspi_data.slave_data_size_bits = 8;

	/* Initialise DSPI driver */
	status = nxp_dspi_init(&dspi_data);

	if (status == TEE_SUCCESS) {

		DMSG("DSPI Base: 0x%lx\n", dspi_data.base);
		dspi_data.chip.ops->end(&dspi_data.chip);	/* stop DSPI controller */
		dspi_data.chip.ops->configure(&dspi_data.chip);		/* configure DSPI chip instance */
		dspi_data.chip.ops->start(&dspi_data.chip);	/* start DSPI controller */

		for (j = 0; j < 1; j++) {
			EMSG("DSPI test loop: %zu", j);
			/* start TX/RX */
			res = dspi_data.chip.ops->txrx8(&dspi_data.chip, tx, rx, len);
			if (res) {
				EMSG("DSPI transceive error %d", res);
				break;
			}
			for (i = 0; i < len; i++)
				EMSG("rx[%zu] = 0x%x", i, rx[i]);

			tee_time_busy_wait(20);
		}
		dspi_data.chip.ops->end(&dspi_data.chip);	/* stop DSPI controller */
	} else
		EMSG("Unable to init DSPI driver");

	return status;
}

/*
 * Verify read/write path of DSPI module by writing/reading on flash connected
 * to dSPI controller so the DSPI module will just receive
 * what is transmitted, i.e. 0x01, 0x80, 0x04.
 *
 */
void dspi_test(void)
{
	TEE_Result status = TEE_ERROR_GENERIC;

	status = dspi_test_suite();
	if (status == TEE_SUCCESS)
		EMSG("Test passed");
	else
		EMSG("Test failed");
}
