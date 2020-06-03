// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright 2020 NXP
 *
 * GPIO Controller.
 *
 */

#include <gpio.h>
#include <stdlib.h>

/* supported ports for GPIO1 controller */
#define MAX_GPIO_PINS		31

/* map register values to LE by subtracting pin number from MAX GPIO PINS */
#define PIN_SHIFT(x)		(1 << (MAX_GPIO_PINS - x))

/* gpio register offsets */
#define GPIODIR         0x0     //direction register
#define GPIOODR         0x4     //open drain register
#define GPIODAT         0x8     //data register
#define GPIOIER         0xc     //interrupt event register
#define GPIOIMR         0x10    //interrupt mask register
#define GPIOICR         0x14    //interrupt control register
#define GPIOIBE         0x18    //input buffer enable register

/**
 * struct nxp_gpio_chip describes GPIO controller chip instance
 * @chip:       generic GPIO chip handle.
 * @gpio_base:  starting GPIO module base address managed by this GPIO controller.
 */
struct gpio_chip_data {
	struct gpio_chip chip;
	vaddr_t gpio_base;
};

void gpio_test(void);
