/*
 * Copyright (C) 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/hardware.h>
#include <mach/device.h>
#include <mach/pinctrl.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/i2c/sx150x.h>
#include <linux/gpio.h>

#include "device.h"
#include "mx28evk.h"

/* EEPROM */
#include <linux/i2c/at24.h>
#define EEPROM_ON_MODULE_I2C_ADDR 0x50

#define GPIO_DEBUG_LED_RED		160
#define GPIO_DEBUG_LED_YELLOW	161
#define GPIO_DEBUG_LED_BLUE		162

/* EEPROM  */

/* EEprom on SOM336 */
static struct at24_platform_data at24c64 = {
     .byte_len       = SZ_64K / 8,
     .flags			 = AT24_FLAG_ADDR16,
     .page_size      = 32,
};
static struct gpio_led gpio_leds[] = {
		{
			 .name = "DEBUG-LED-BLUE",
			 .default_trigger = "heartbeat",
			 .active_low = 0,
			 .gpio = GPIO_DEBUG_LED_BLUE,
		},
		{
			 .name = "DEBUG-LED-YELLOW",
			 .default_trigger = "none",
			 .active_low = 0,
			 .gpio = GPIO_DEBUG_LED_YELLOW,
		},
		{
			 .name = "DEBUG-LED-RED",
			 .default_trigger = "none",
			 .active_low = 0,
			 .gpio = GPIO_DEBUG_LED_RED,
		},
};

static struct gpio_led_platform_data gpio_led_info = {
		.leds = gpio_leds,
		.num_leds = ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name = "leds-gpio",
	.id = -1,
	.dev = {
			.platform_data = &gpio_led_info,
	},
};

static uint32_t board_keymap[] = {
	KEY(0, 0, KEY_F1),
	KEY(0, 1, KEY_UP),
	KEY(0, 2, KEY_ENTER),

	KEY(1, 0, KEY_F2),
	KEY(1, 1, KEY_DOWN),
	KEY(1, 3, KEY_VOLUMEUP),

	KEY(2, 0, KEY_F3),
	KEY(2, 1, KEY_LEFT),
	KEY(2, 3, KEY_VOLUMEDOWN),

	KEY(3, 0, KEY_F4),
	KEY(3, 1, KEY_RIGHT),
	KEY(3, 3, KEY_POWER),
};

static struct  matrix_keymap_data __initdata board_map_data = {
	.keymap			= board_keymap,
	.keymap_size		= ARRAY_SIZE(board_keymap),
};
#define KEYPAD_INTERRUPT_GPIO	6
static struct  sxegfkp_platform_data __initdata omap3egf_kp_data = {
	.keymap_data	= &board_map_data,
	.rows		= 4,
	.cols		= 4,
	.rep		= 1,
};

static struct sx150x_platform_data __initdata sx1509_gpio_expander_onmodule_data;
static struct sx150x_platform_data __initdata sx1509_gpio_expander_onboard_data;
static struct i2c_board_info __initdata mxs_i2c_device[] = {
	{
			I2C_BOARD_INFO("sgtl5000-i2c", 0xa),
			.flags = I2C_M_TEN,
	},
	{
			I2C_BOARD_INFO("sxegfkp", 0x3E),
			.platform_data = &omap3egf_kp_data,
	},
	{
			I2C_BOARD_INFO("sx1509q", 0x3F),
			.platform_data = &sx1509_gpio_expander_onmodule_data,
	},
	/*{
			I2C_BOARD_INFO("sx1509q", 0x70),
			.platform_data = &sx1509_gpio_expander_onboard_data,
	},*/
	{
			I2C_BOARD_INFO("tmp102", 0x49),
	},
#ifndef CONFIG_FSL_UTP)
	{	/* Eeprom on module */
	        I2C_BOARD_INFO("24c64", EEPROM_ON_MODULE_I2C_ADDR),
	       .platform_data  = &at24c64,
	},
#endif
};
static void __init init_gpio_expander(void)
{
	sx1509_gpio_expander_onmodule_data.irq_summary = -1;
	sx1509_gpio_expander_onmodule_data.gpio_base = 176;
	sx1509_gpio_expander_onboard_data.irq_summary = -1;
	sx1509_gpio_expander_onboard_data.gpio_base = 160;
}
static void __init init_keypad(void)
{
	omap3egf_kp_data.irq = gpio_to_irq(KEYPAD_INTERRUPT_GPIO);
}

static void __init i2c_device_init(void)
{
	init_gpio_expander();
	init_keypad();
	i2c_register_board_info(0, mxs_i2c_device, ARRAY_SIZE(mxs_i2c_device));

}
#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct flash_platform_data mx28_spi_flash_data = {
	.name = "m25p80",
	.type = "w25x80",
};
#endif

static struct spi_board_info spi_board_info[] __initdata = {
#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
	{
		/* the modalias must be the same as spi device driver name */
		.modalias = "m25p80", /* Name of spi_driver for this device */
		.max_speed_hz = 20000000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 1, /* Framework bus number */
		.chip_select = 0, /* Framework chip select. */
		.platform_data = &mx28_spi_flash_data,
	},
#endif
	{
			/* the modalias must be the same as spi device driver name */
			.modalias = "spidev", /* Name of spi_driver for this device */
			.max_speed_hz = 20000000,     /* max spi clock (SCK) speed in HZ */
			.bus_num = 1, /* Framework bus number */
			.chip_select = 0, /* Framework chip select. */
		},
};

static void spi_device_init(void)
{
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
	printk(KERN_INFO "\nRegistered %d SPI: %d",ARRAY_SIZE(spi_board_info),spi_board_info[0].bus_num);
}

static void __init fixup_board(struct machine_desc *desc, struct tag *tags,
			       char **cmdline, struct meminfo *mi)
{
	mx28_set_input_clk(24000000, 24000000, 32000, 50000000);
}

#if defined(CONFIG_LEDS_MXS) || defined(CONFIG_LEDS_MXS_MODULE)
static struct mxs_pwm_led  mx28evk_led_pwm[2] = {
	[0] = {
		.name = "led-pwm0",
		.pwm = 0,
		},
	[1] = {
		.name = "led-pwm1",
		.pwm = 1,
		},
};

struct mxs_pwm_leds_plat_data mx28evk_led_data = {
	.num = ARRAY_SIZE(mx28evk_led_pwm),
	.leds = mx28evk_led_pwm,
};

static struct resource mx28evk_led_res = {
	.flags = IORESOURCE_MEM,
	.start = PWM_PHYS_ADDR,
	.end   = PWM_PHYS_ADDR + 0x3FFF,
};

static void __init mx28evk_init_leds(void)
{
	struct platform_device *pdev;

	pdev = mxs_get_device("mxs-leds", 0);
	if (pdev == NULL || IS_ERR(pdev))
		return;

	pdev->resource = &mx28evk_led_res;
	pdev->num_resources = 1;
	pdev->dev.platform_data = &mx28evk_led_data;
	mxs_add_device(pdev, 3);
}
#else
static void __init mx28evk_init_leds(void)
{
	;
}
#endif

static void __init mx28evk_device_init(void)
{
	/* Add mx28evk special code */
	i2c_device_init();
	spi_device_init();
	mx28evk_init_leds();
}

static void __init mx28evk_init_machine(void)
{
	mx28_pinctrl_init();
	/* Init iram allocate */
#ifdef CONFIG_VECTORS_PHY_ADDR
	/* reserve the first page for irq vector table*/
	iram_init(MX28_OCRAM_PHBASE + PAGE_SIZE, MX28_OCRAM_SIZE - PAGE_SIZE);
#else
	iram_init(MX28_OCRAM_PHBASE, MX28_OCRAM_SIZE);
#endif

	mx28_gpio_init();
	mx28evk_pins_init();
	mx28_device_init();
	mx28evk_device_init();
	platform_device_register(&leds_gpio);
}

MACHINE_START(MX28EVK, "Freescale MX28EVK board")
	.phys_io	= 0x80000000,
	.io_pg_offst	= ((0xf0000000) >> 18) & 0xfffc,
	.boot_params	= 0x40000100,
	.fixup		= fixup_board,
	.map_io		= mx28_map_io,
	.init_irq	= mx28_irq_init,
	.init_machine	= mx28evk_init_machine,
	.timer		= &mx28_timer.timer,
MACHINE_END
