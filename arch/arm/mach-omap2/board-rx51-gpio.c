/*
 * linux/arch/arm/mach-omap2/board-rx51-gpio.c
 *
 * Copyright (C) 2010 Nokia
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/camera_button.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio_keys.h>

#include <asm/mach-types.h>

#include <mach/mcspi.h>
#include <mach/gpio.h>
#include <mach/mux.h>
#include <mach/board.h>
#include <mach/common.h>
#include <mach/keypad.h>
#include <mach/dma.h>
#include <mach/gpmc.h>
#include <mach/ssi.h>
#include <mach/omap-pm.h>

#define RX51_CAMERA_FOCUS_GPIO		68
#define RX51_CAMERA_LAUNCH_GPIO		69
#define RX51_KB_SLIDE_GPIO		71
#define RX51_PROXIMITY_GPIO		89
#define RX51_CAMERA_SHUTTER_GPIO	110
#define RX51_KB_LOCK_GPIO		113
#define RX51_HEADPHONE_GPIO		177


static struct omap_gpio_switch rx51_gpio_switches[] = {
	{
		.name			= "headphone",
		.gpio			= RX51_HEADPHONE_GPIO,
		.debounce_rising	= 200,
		.debounce_falling	= 200,
	},
	{
		.name			= "camera",
		.gpio			= RX51_CAMERA_SHUTTER_GPIO,
		.debounce_rising	= 30,
		.debounce_falling	= 30,
	}, 
};

static struct gpio_keys_button rx51_gpio_keys_buttons[] = {
	{
		.code			= SW_LID,
		.desc			= "sw_lid",
		.gpio			= RX51_KB_SLIDE_GPIO,
		.type			= EV_SW,
		.debounce_interval	= 100,
		.wakeup			= 1,
	},
	{
		.code			= KEY_SCREENLOCK,
		.gpio			= RX51_KB_LOCK_GPIO,
		.desc			= "kbd lock",
		.active_low		= 1,
		.debounce_interval	= 30,
	},
	{
		.code			= KEY_ZOOM,
		.gpio			= RX51_CAMERA_FOCUS_GPIO,
		.desc			= "camera focus",
		.active_low		= 1,
		.debounce_interval	= 30,
	},
	{
		.code			= KEY_CAMERA,
		.gpio			= RX51_CAMERA_LAUNCH_GPIO,
		.desc			= "camera launch",
		.active_low		= 1,
		.debounce_interval	= 30,
	},
	{
		.code			= KEY_LINEFEED,
		.gpio			= RX51_PROXIMITY_GPIO,
		.desc			= "proximity",
		.active_low		= 1,
		.debounce_interval	= 30,
	}, 
};

static struct gpio_keys_platform_data rx51_gpio_keys_data =
{
	.buttons	= rx51_gpio_keys_buttons,
	.nbuttons	= ARRAY_SIZE(rx51_gpio_keys_buttons),
	.rep		= 0,
};

static struct platform_device rx51_gpio_keys_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.dev		= {
		.platform_data = &rx51_gpio_keys_data,
	},
};

static struct platform_device *rx51_gpio_devices[] = {
	&rx51_gpio_keys_device,
};


void __init rx51_gpio_init(void)
{
	if (!machine_is_nokia_rx51())
		return 0;
	
	omap_register_gpio_switches(rx51_gpio_switches,
				    ARRAY_SIZE(rx51_gpio_switches));
	
	platform_add_devices(rx51_gpio_devices,
			     ARRAY_SIZE(rx51_gpio_devices));
	
}

