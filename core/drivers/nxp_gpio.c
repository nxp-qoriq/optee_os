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
#include <kernel/dt.h>
#include <kernel/generic_boot.h>
#include <mm/core_memprot.h>

#ifdef CFG_DT
#include <libfdt.h>
#endif

/* global data pointer for gpio */
struct gpio_chip_data *gc_data;

static enum gpio_level gpio_get_value(unsigned int gpio_pin)
{
	vaddr_t gpio_data_addr;
	uint32_t data;

	assert(gpio_pin <= MAX_GPIO_PINS);

	gpio_data_addr = gc_data->gpio_base + GPIODAT;
	data = io_read32(gpio_data_addr);

	if (data & PIN_SHIFT(gpio_pin))
		return GPIO_LEVEL_HIGH;
	else
		return GPIO_LEVEL_LOW;
}

static void gpio_set_value(unsigned int gpio_pin, enum gpio_level value)
{
	vaddr_t gpio_data_addr;
	uint32_t data;

	assert(gpio_pin <= MAX_GPIO_PINS);

	gpio_data_addr = gc_data->gpio_base + GPIODAT;
	data = io_read32(gpio_data_addr);

	if (value == GPIO_LEVEL_HIGH)
		/* if value is high then set pin value */
		io_setbits32(gpio_data_addr, (data | PIN_SHIFT(gpio_pin)));
	else
		/* if value is low then clear pin value */
		io_setbits32(gpio_data_addr, (data & ~(PIN_SHIFT(gpio_pin))));
}

static enum gpio_dir gpio_get_direction(unsigned int gpio_pin)
{
	vaddr_t gpio_dir_addr;
	uint32_t data;

	assert(gpio_pin <= MAX_GPIO_PINS);

	gpio_dir_addr = gc_data->gpio_base + GPIODIR;
	data = io_read32(gpio_dir_addr);

	if (data & PIN_SHIFT(gpio_pin))
		return !GPIO_DIR_OUT;
	else
		return !GPIO_DIR_IN;
}

static void gpio_set_direction(unsigned int gpio_pin, enum gpio_dir direction)
{
	vaddr_t gpio_dir_addr;

	assert(gpio_pin <= MAX_GPIO_PINS);

	gpio_dir_addr = gc_data->gpio_base + GPIODIR;

	if (direction)
		io_setbits32(gpio_dir_addr, PIN_SHIFT(gpio_pin));
	else
		io_clrbits32(gpio_dir_addr, PIN_SHIFT(gpio_pin));
}

static enum gpio_interrupt gpio_get_interrupt(unsigned int gpio_pin)
{
	vaddr_t gpio_ier_addr;
	uint32_t data;

	assert(gpio_pin <= MAX_GPIO_PINS);

	gpio_ier_addr = gc_data->gpio_base + GPIOIER;
	data = io_read32(gpio_ier_addr);

	if (data & PIN_SHIFT(gpio_pin))
		return GPIO_INTERRUPT_ENABLE;
	else
		return GPIO_INTERRUPT_DISABLE;
}

static void gpio_set_interrupt(unsigned int gpio_pin,
				enum gpio_interrupt interrupt)
{
	vaddr_t gpio_ier_addr;

	assert(gpio_pin <= MAX_GPIO_PINS);

	gpio_ier_addr = gc_data->gpio_base + GPIOIER;

	if (interrupt == GPIO_INTERRUPT_ENABLE)
		io_setbits32(gpio_ier_addr, PIN_SHIFT(gpio_pin));
	else
		io_clrbits32(gpio_ier_addr, PIN_SHIFT(gpio_pin));
}

static TEE_Result get_base_address_from_device_tree(paddr_t *base_addr)
{
	paddr_t paddr = 0;
	ssize_t size = 0;

	void *fdt = get_embedded_dt();
	int gpio_offset = 0;

	gpio_offset = fdt_path_offset(fdt, "/soc/gpio@2300000");

	if (gpio_offset < 0)
		gpio_offset = fdt_path_offset(fdt, "/gpio@2300000");

	if (gpio_offset > 0) {
		paddr = _fdt_reg_base_address(fdt, gpio_offset);
		if (paddr == DT_INFO_INVALID_REG) {
			EMSG("Unable to get physical base address from device tree");
			return TEE_ERROR_ITEM_NOT_FOUND;
		}

		size = _fdt_reg_size(fdt, gpio_offset);
		if (size < 0) {
			EMSG("Unable to get size of physical base address from device tree");
			return TEE_ERROR_ITEM_NOT_FOUND;
		}
	} else {
		EMSG("Unable to get gpio offset node");
		return TEE_ERROR_ITEM_NOT_FOUND;
	}

	/* make entry in page table */
	if (!core_mmu_add_mapping(MEM_AREA_IO_NSEC, paddr, size)) {
		EMSG("GPIO control base MMU PA mapping failure");
		return TEE_ERROR_ITEM_NOT_FOUND;
	}

	*base_addr = paddr;
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

/* Register NXP-Layerscape GPIO1 controller */
static TEE_Result layerscape_gpio_init(paddr_t gpio_base_phy_addr)
{
	vaddr_t ctrl_base;

	/* generic GPIO chip handle */
	gc_data->chip.ops = &nxp_gpio_ops;

	/* converting phyical address to virtual address */
	ctrl_base = (vaddr_t)phys_to_virt(gpio_base_phy_addr,
					MEM_AREA_IO_NSEC);
	if (ctrl_base > 0)
		gc_data->gpio_base = ctrl_base;
	else {
		EMSG("Unable to get virtual address of GPIO controller");
		return TEE_ERROR_GENERIC;
	}

	/* set GPIO Input Buffer Enable register */
	io_setbits32(gc_data->gpio_base + GPIOIBE, 0xffffffff);

#ifdef CFG_NXP_GPIO_TEST
	/* call GPIO test suite */
	gpio_test();
#endif
	return TEE_SUCCESS;
}

/*
 * Initialise GPIO1 controller
 */
static TEE_Result nxp_gpio_init(void)
{
	TEE_Result status = TEE_ERROR_GENERIC;
	paddr_t gpio_base_phy_addr = 0;

	/* allocate memory for GPIO */
	gc_data = (struct gpio_chip_data *)malloc(sizeof(
						struct gpio_chip_data));

	if (gc_data == NULL) {
		EMSG("Unable to init GPIO1 controller");
		return TEE_ERROR_OUT_OF_MEMORY;
	}

	/*
	 * First get the GPIO1 Controller base address from the DTB,
	 * if DTB present and if the GPIO1 Controller defined in it.
	 */
	status = get_base_address_from_device_tree(&gpio_base_phy_addr);

	/* register NXP-Layerscape GPIO1 controller  */
	if (status == TEE_SUCCESS)
		status = layerscape_gpio_init(gpio_base_phy_addr);
	else {
		EMSG("Unable to get physical address of GPIO1 controller");
		/* free allocated memory for GPIO*/
		free(gc_data);
	}

	return status;
}

/* Initialise gpio driver with GPIO1 controller*/
driver_init(nxp_gpio_init);
