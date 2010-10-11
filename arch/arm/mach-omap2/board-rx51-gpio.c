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
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/gpio_keys.h>
#include <linux/platform_device.h>

#include <asm/mach-types.h>

#include <mach/gpio.h>
#include <mach/mux.h>
#include <mach/board.h>
#include <mach/common.h>
#include <mach/keypad.h>


#define RX51_KB_LOCK_GPIO		113
#define RX51_KB_SLIDE_GPIO		71
#define RX51_CAMERA_FOCUS_GPIO		68
#define RX51_CAMERA_SHUTTER_GPIO	69
#define RX51_CAMERA_LENS_COVER_GPIO	110
#define RX51_PROXIMITY_SENSOR_GPIO	89
#define RX51_HEADPHONE_INSERT_GPIO	177

#define SW_LENS_COVER			0x0A


struct gpio_keys_button rx51_gpio_keys_buttons[] = {
	{
		.type			= EV_KEY,
		.code			= KEY_SCREENLOCK,
		.gpio			= RX51_KB_LOCK_GPIO,
		.desc			= "kbd lock",
		.active_low		= 1,
		.debounce_interval	= 30,
	},
	{
		.type			= EV_KEY,
		.code			= KEY_ZOOM,
		.gpio			= RX51_CAMERA_FOCUS_GPIO,
		.desc			= "camera focus",
		.active_low		= 1,
		.debounce_interval	= 30,
	},
	{
		.type			= EV_KEY,
		.code			= KEY_RECORD,
		.gpio			= RX51_CAMERA_SHUTTER_GPIO,
		.desc			= "camera shutter",
		.active_low		= 1,
		.debounce_interval	= 30,
	},
	{
		.type			= EV_KEY,
		.code			= KEY_CAMERA,
		.gpio			= RX51_CAMERA_LENS_COVER_GPIO,
		.desc			= "camera lens cover",
		.wakeup			= 1,
		.debounce_interval	= 100,
	}, 
	{
		.type			= EV_SW,
		.code			= SW_LID,
		.gpio			= RX51_KB_SLIDE_GPIO,
		.desc			= "keyboard slide",
		.wakeup			= 1,
		.debounce_interval	= 100,
	},
};


const int rx51_gpio_keys_buttons_count = ARRAY_SIZE(rx51_gpio_keys_buttons);


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
	platform_add_devices(rx51_gpio_devices,
			     ARRAY_SIZE(rx51_gpio_devices));
}

