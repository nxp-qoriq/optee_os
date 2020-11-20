// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright 2020 NXP
 *
 * GPIO Controller
 *
 */

#include <assert.h>
#include <drivers/nxp_gpio.h>
#include <io.h>
#include <kernel/boot.h>
#include <kernel/dt.h>
#include <libfdt.h>
#include <mm/core_memprot.h>

#define GPIO_CTRL_NUM  4

static const char *gpio_controller_map[GPIO_CTRL_NUM] = {
	 "/soc/gpio@2300000",
	 "/soc/gpio@2310000",
	 "/soc/gpio@2320000",
	 "/soc/gpio@2330000" };

static enum gpio_level gpio_get_value(struct gpio_chip *chip,
				      unsigned int gpio_pin)
{
	vaddr_t gpio_data_addr = 0;
	uint32_t data = 0;
	struct gpio_chip_data *gc_data = container_of(chip,
						      struct gpio_chip_data,
						      chip);

	assert(gpio_pin <= MAX_GPIO_PINS);

	gpio_data_addr = gc_data->gpio_base + GPIODAT;
	data = io_read32(gpio_data_addr);

	if (data & PIN_SHIFT(gpio_pin))
		return GPIO_LEVEL_HIGH;
	else
		return GPIO_LEVEL_LOW;
}

static void gpio_set_value(struct gpio_chip *chip, unsigned int gpio_pin,
			   enum gpio_level value)
{
	vaddr_t gpio_data_addr = 0;
	struct gpio_chip_data *gc_data = container_of(chip,
						      struct gpio_chip_data,
						      chip);

	assert(gpio_pin <= MAX_GPIO_PINS);

	gpio_data_addr = gc_data->gpio_base + GPIODAT;

	if (value == GPIO_LEVEL_HIGH)
		/* if value is high then set pin value */
		io_setbits32(gpio_data_addr, PIN_SHIFT(gpio_pin));
	else
		/* if value is low then clear pin value */
		io_clrbits32(gpio_data_addr, (PIN_SHIFT(gpio_pin)));
}

static enum gpio_dir gpio_get_direction(struct gpio_chip *chip,
					unsigned int gpio_pin)
{
	vaddr_t gpio_dir_addr = 0;
	uint32_t data = 0;
	struct gpio_chip_data *gc_data = container_of(chip,
						      struct gpio_chip_data,
						      chip);

	assert(gpio_pin <= MAX_GPIO_PINS);

	gpio_dir_addr = gc_data->gpio_base + GPIODIR;
	data = io_read32(gpio_dir_addr);

	if (data & PIN_SHIFT(gpio_pin))
		return !GPIO_DIR_OUT;
	else
		return !GPIO_DIR_IN;
}

static void gpio_set_direction(struct gpio_chip *chip, unsigned int gpio_pin,
			       enum gpio_dir direction)
{
	vaddr_t gpio_dir_addr = 0;
	struct gpio_chip_data *gc_data = container_of(chip,
						      struct gpio_chip_data,
						      chip);

	assert(gpio_pin <= MAX_GPIO_PINS);

	gpio_dir_addr = gc_data->gpio_base + GPIODIR;

	if (direction)
		io_setbits32(gpio_dir_addr, PIN_SHIFT(gpio_pin));
	else
		io_clrbits32(gpio_dir_addr, PIN_SHIFT(gpio_pin));
}

static enum gpio_interrupt gpio_get_interrupt(struct gpio_chip *chip,
					      unsigned int gpio_pin)
{
	vaddr_t gpio_ier_addr = 0;
	uint32_t data = 0;
	struct gpio_chip_data *gc_data = container_of(chip,
						      struct gpio_chip_data,
						      chip);

	assert(gpio_pin <= MAX_GPIO_PINS);

	gpio_ier_addr = gc_data->gpio_base + GPIOIER;
	data = io_read32(gpio_ier_addr);

	if (data & PIN_SHIFT(gpio_pin))
		return GPIO_INTERRUPT_ENABLE;
	else
		return GPIO_INTERRUPT_DISABLE;
}

static void gpio_set_interrupt(struct gpio_chip *chip, unsigned int gpio_pin,
			       enum gpio_interrupt interrupt)
{
	vaddr_t gpio_ier_addr = 0;
	struct gpio_chip_data *gc_data = container_of(chip,
						      struct gpio_chip_data,
						      chip);

	assert(gpio_pin <= MAX_GPIO_PINS);

	gpio_ier_addr = gc_data->gpio_base + GPIOIER;

	if (interrupt == GPIO_INTERRUPT_ENABLE)
		io_setbits32(gpio_ier_addr, PIN_SHIFT(gpio_pin));
	else
		io_clrbits32(gpio_ier_addr, PIN_SHIFT(gpio_pin));
}

static TEE_Result get_info_from_device_tree(struct gpio_chip_data *gpio_data)
{
	size_t size = 0;
	int node = 0;
	vaddr_t ctrl_base = 0;

	void *fdt = get_embedded_dt();

	node = fdt_path_offset(fdt, gpio_controller_map
			       [gpio_data->gpio_controller]);

	if (node > 0) {
		if (dt_map_dev(fdt, node, &ctrl_base, &size) < 0) {
			EMSG("Unable to get virtual address");
			return TEE_ERROR_GENERIC;
		}
	} else {
		EMSG("Unable to get gpio offset node");
		return TEE_ERROR_ITEM_NOT_FOUND;
	}

	gpio_data->gpio_base = ctrl_base;

	return TEE_SUCCESS;
}

static const struct gpio_ops nxp_gpio_ops = {
	.get_direction = gpio_get_direction,
	.set_direction = gpio_set_direction,
	.get_value = gpio_get_value,
	.set_value = gpio_set_value,
	.get_interrupt = gpio_get_interrupt,
	.set_interrupt = gpio_set_interrupt,
};
DECLARE_KEEP_PAGER(nxp_gpio_ops);

/*
 * Initialise GPIO Controller
 */
TEE_Result nxp_gpio_init(struct gpio_chip_data *gpio_data)
{
	TEE_Result status = TEE_ERROR_GENERIC;

	/*
	 * First get the GPIO Controller base address from the DTB,
	 * if DTB present and if the GPIO Controller defined in it.
	 */
	status = get_info_from_device_tree(gpio_data);

	/* register NXP-Layerscape GPIO controller  */
	if (status == TEE_SUCCESS) {
		/* set GPIO Input Buffer Enable register */
		io_setbits32(gpio_data->gpio_base + GPIOIBE, 0xffffffff);

		/* generic GPIO chip handle */
		gpio_data->chip.ops = &nxp_gpio_ops;
	} else {
		EMSG("Unable to get info from device tree");
	}

	return status;
}
