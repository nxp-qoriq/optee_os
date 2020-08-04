// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright 2020 NXP
 *
 * Test Suite for GPIO Controller.
 *
 */

#include <drivers/nxp_gpio.h>
#include <mm/core_memprot.h>
#include <stdint.h>
#include <trace.h>

#define ARRAY_LENGTH(x)		(sizeof(x) / sizeof((x)[0]))
#define GPIO_PIN_31		31
#define GPIO_PIN_30		30

extern struct gpio_chip_data *gc_data;

enum gpio_test_direction {
	GPIO_TEST_DIR_IN,
	GPIO_TEST_DIR_OUT
};

/* Refer board RM to get GPIOs pins for loopback testing
 * here, using PIN 30 and PIN 31 (odd and even pair) in loopback mode
 */
int gpio_pin[] = {GPIO_PIN_31, GPIO_PIN_30};

/* testing loopback mode of PIN 30 and PIN 31 */
static TEE_Result gpio_test_suite(void)
{
	struct gpio_chip_data data;
	TEE_Result status = TEE_ERROR_GENERIC;

	EMSG("GPIO Test Starts");

	/* set slave info */
	data.gpio_controller = CFG_GPIO_CONTROLLER;

	/* Initialise GPIO driver */
	status = nxp_gpio_init(&data);

	int len = ARRAY_LENGTH(gpio_pin);

	if (status == TEE_SUCCESS) {
		DMSG("GPIO Base: 0x%lx\n", data.gpio_base);

		for (int i = 0; i < len; i++) {
			/* Set the GPIO pin direction as output */
			data.chip.ops->set_direction(&data.chip, gpio_pin[i], GPIO_TEST_DIR_OUT);
			EMSG("Get the direction of PIN %d as %d", gpio_pin[i], data.chip.ops->get_direction(&data.chip, gpio_pin[i]));

			/* Set the GPIO pin direction as input */
			data.chip.ops->set_direction(&data.chip, gpio_pin[i^1], GPIO_TEST_DIR_IN);
			EMSG("Get the direction of PIN %d as %d", gpio_pin[i^1], data.chip.ops->get_direction(&data.chip, gpio_pin[i^1]));

			/* If an output pin, write the level (low or high) */
			data.chip.ops->set_value(&data.chip, gpio_pin[i], 1);
			EMSG("Write value 1 on PIN %d", gpio_pin[i]);

			/* If an input pin, read the pin's level (low or high) */
			EMSG("Read the value on PIN %d and read value: %d", gpio_pin[i], data.chip.ops->get_value(&data.chip, gpio_pin[i]));
			EMSG("Read the value on PIN %d and read value: %d", gpio_pin[i^1], data.chip.ops->get_value(&data.chip, gpio_pin[i^1]));

			/* check if values at both PINs (PIN 31 and PIN 30) are same or not */
			if (data.chip.ops->get_value(&data.chip, gpio_pin[i]) == data.chip.ops->get_value(&data.chip, gpio_pin[i^1]))
				EMSG("Test passed successfully.");
			else
				EMSG("Test failed.");
		}
	} else
		EMSG("Unable to init GPIO driver");

	return status;
}

/*
 * nxp_gpio_init() MUST be run before calling this function!
 *
 * gpio_test runs some loopback tests, so the GPIO module will test
 * input/output functionality of GPIO pin
 * Enable GPIO loopback with below configurations:
 * Use FPGA configuration using i2c for board configuration
 *
 * RCW Changes:
 * lx2162aqds/FFGG_NNNN_PPPP_HHHH_RR_18_5/rcw_2000_650_2900_18_5.rcw
 * IIC2_PMUX=1
 * IIC3_PMUX=1
 * IIC4_PMUX=1
 *
 * To test GPIOs, set the DUT to the GPIO mode on selected pins, and enable the
 * complimentary loopback testing using FPGA. For example,
 * - set DUT GPIO direction of first PIN in loopback testing as output/input
 * - set DUT GPIO direction of second PIN in loopback testing as input/output
 * - write value (high/low) on the first PIN
 * - read value (high/low) on the second PIN
 * - compare the read value(from second PIN) and write value(from first PIN)
 * - if the values match then test case passed
 */

void gpio_test(void)
{
	TEE_Result status = TEE_ERROR_GENERIC;

	status = gpio_test_suite();
	if (status == TEE_SUCCESS)
		EMSG("Test passed");
	else
		EMSG("Test failed");

}
