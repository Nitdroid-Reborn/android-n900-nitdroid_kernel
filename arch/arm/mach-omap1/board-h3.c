/*
 * linux/arch/arm/mach-omap1/board-h3.c
 *
 * This file contains OMAP1710 H3 specific code.
 *
 * Copyright (C) 2004 Texas Instruments, Inc.
 * Copyright (C) 2002 MontaVista Software, Inc.
 * Copyright (C) 2001 RidgeRun, Inc.
 * Author: RidgeRun, Inc.
 *         Greg Lonnon (glonnon@ridgerun.com) or info@ridgerun.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/major.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/input.h>
#include <linux/clk.h>

#include <linux/i2c/tps65010.h>
#include <linux/i2c/pcf857x.h>

#include <linux/spi/spi.h>
#include <linux/spi/tsc210x.h>

#include <asm/setup.h>
#include <asm/page.h>
#include <mach/hardware.h>
#include <asm/gpio.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <asm/mach/map.h>

#include <media/v4l2-int-device.h>

#include <mach/gpio.h>
#include <mach/gpio-switch.h>
#include <mach/irqs.h>
#include <mach/mux.h>
#include <mach/tc.h>
#include <mach/nand.h>
#include <mach/irda.h>
#include <mach/usb.h>
#include <mach/keypad.h>
#include <mach/dma.h>
#include <mach/common.h>

#define H3_TS_GPIO	48

static int h3_keymap[] = {
	KEY(0, 0, KEY_LEFT),
	KEY(0, 1, KEY_RIGHT),
	KEY(0, 2, KEY_3),
	KEY(0, 3, KEY_F10),
	KEY(0, 4, KEY_F5),
	KEY(0, 5, KEY_9),
	KEY(1, 0, KEY_DOWN),
	KEY(1, 1, KEY_UP),
	KEY(1, 2, KEY_2),
	KEY(1, 3, KEY_F9),
	KEY(1, 4, KEY_F7),
	KEY(1, 5, KEY_0),
	KEY(2, 0, KEY_ENTER),
	KEY(2, 1, KEY_6),
	KEY(2, 2, KEY_1),
	KEY(2, 3, KEY_F2),
	KEY(2, 4, KEY_F6),
	KEY(2, 5, KEY_HOME),
	KEY(3, 0, KEY_8),
	KEY(3, 1, KEY_5),
	KEY(3, 2, KEY_F12),
	KEY(3, 3, KEY_F3),
	KEY(3, 4, KEY_F8),
	KEY(3, 5, KEY_END),
	KEY(4, 0, KEY_7),
	KEY(4, 1, KEY_4),
	KEY(4, 2, KEY_F11),
	KEY(4, 3, KEY_F1),
	KEY(4, 4, KEY_F4),
	KEY(4, 5, KEY_ESC),
	KEY(5, 0, KEY_F13),
	KEY(5, 1, KEY_F14),
	KEY(5, 2, KEY_F15),
	KEY(5, 3, KEY_F16),
	KEY(5, 4, KEY_SLEEP),
	0
};


static struct mtd_partition nor_partitions[] = {
	/* bootloader (U-Boot, etc) in first sector */
	{
	      .name		= "bootloader",
	      .offset		= 0,
	      .size		= SZ_128K,
	      .mask_flags	= MTD_WRITEABLE, /* force read-only */
	},
	/* bootloader params in the next sector */
	{
	      .name		= "params",
	      .offset		= MTDPART_OFS_APPEND,
	      .size		= SZ_128K,
	      .mask_flags	= 0,
	},
	/* kernel */
	{
	      .name		= "kernel",
	      .offset		= MTDPART_OFS_APPEND,
	      .size		= SZ_2M,
	      .mask_flags	= 0
	},
	/* file system */
	{
	      .name		= "filesystem",
	      .offset		= MTDPART_OFS_APPEND,
	      .size		= MTDPART_SIZ_FULL,
	      .mask_flags	= 0
	}
};

static struct flash_platform_data nor_data = {
	.map_name	= "cfi_probe",
	.width		= 2,
	.parts		= nor_partitions,
	.nr_parts	= ARRAY_SIZE(nor_partitions),
};

static struct resource nor_resource = {
	/* This is on CS3, wherever it's mapped */
	.flags		= IORESOURCE_MEM,
};

static struct platform_device nor_device = {
	.name		= "omapflash",
	.id		= 0,
	.dev		= {
		.platform_data	= &nor_data,
	},
	.num_resources	= 1,
	.resource	= &nor_resource,
};

static struct mtd_partition nand_partitions[] = {
#if 0
	/* REVISIT: enable these partitions if you make NAND BOOT work */
	{
		.name		= "xloader",
		.offset		= 0,
		.size		= 64 * 1024,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "bootloader",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 256 * 1024,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "params",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 192 * 1024,
	},
	{
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 2 * SZ_1M,
	},
#endif
	{
		.name		= "filesystem",
		.size		= MTDPART_SIZ_FULL,
		.offset		= MTDPART_OFS_APPEND,
	},
};

/* dip switches control NAND chip access:  8 bit, 16 bit, or neither */
static struct omap_nand_platform_data nand_data = {
	.options	= NAND_SAMSUNG_LP_OPTIONS,
	.parts		= nand_partitions,
	.nr_parts	= ARRAY_SIZE(nand_partitions),
};

static struct resource nand_resource = {
	.flags		= IORESOURCE_MEM,
};

static struct platform_device nand_device = {
	.name		= "omapnand",
	.id		= 0,
	.dev		= {
		.platform_data	= &nand_data,
	},
	.num_resources	= 1,
	.resource	= &nand_resource,
};

static struct resource smc91x_resources[] = {
	[0] = {
		.start	= OMAP1710_ETHR_START,		/* Physical */
		.end	= OMAP1710_ETHR_START + 0xf,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= OMAP_GPIO_IRQ(40),
		.end	= OMAP_GPIO_IRQ(40),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
	},
};

static struct platform_device smc91x_device = {
	.name		= "smc91x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smc91x_resources),
	.resource	= smc91x_resources,
};

#define GPTIMER_BASE		0xFFFB1400
#define GPTIMER_REGS(x)	(0xFFFB1400 + (x * 0x800))
#define GPTIMER_REGS_SIZE	0x46

static struct resource intlat_resources[] = {
	[0] = {
		.start  = GPTIMER_REGS(0),	      /* Physical */
		.end    = GPTIMER_REGS(0) + GPTIMER_REGS_SIZE,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = INT_1610_GPTIMER1,
		.end    = INT_1610_GPTIMER1,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device intlat_device = {
	.name	   = "omap_intlat",
	.id	     = 0,
	.num_resources  = ARRAY_SIZE(intlat_resources),
	.resource       = intlat_resources,
};

static struct resource h3_kp_resources[] = {
	[0] = {
		.start	= INT_KEYBOARD,
		.end	= INT_KEYBOARD,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct omap_kp_platform_data h3_kp_data = {
	.rows		= 8,
	.cols		= 8,
	.keymap		= h3_keymap,
	.keymapsize	= ARRAY_SIZE(h3_keymap),
	.rep		= 1,
	.delay		= 9,
	.dbounce	= 1,
};

static struct platform_device h3_kp_device = {
	.name		= "omap-keypad",
	.id		= -1,
	.dev		= {
		.platform_data = &h3_kp_data,
	},
	.num_resources	= ARRAY_SIZE(h3_kp_resources),
	.resource	= h3_kp_resources,
};


/* Select between the IrDA and aGPS module
 */

static int gpio_irda_enable;
static int gpio_irda_x;
static int gpio_irda_fir;

static int h3_select_irda(struct device *dev, int state)
{
	gpio_set_value_cansleep(gpio_irda_enable, state & IR_SEL);
	return 0;
}

static void set_trans_mode(struct work_struct *work)
{
	struct omap_irda_config *irda_config =
		container_of(work, struct omap_irda_config, gpio_expa.work);
	int mode = irda_config->mode;

	gpio_set_value_cansleep(gpio_irda_x, 1);
	gpio_set_value_cansleep(gpio_irda_fir, !(mode & IR_SIRMODE));
}

static int h3_transceiver_mode(struct device *dev, int mode)
{
	struct omap_irda_config *irda_config = dev->platform_data;

	irda_config->mode = mode;
	cancel_delayed_work(&irda_config->gpio_expa);
	PREPARE_DELAYED_WORK(&irda_config->gpio_expa, set_trans_mode);
	schedule_delayed_work(&irda_config->gpio_expa, 0);

	return 0;
}

static struct omap_irda_config h3_irda_data = {
	.transceiver_cap	= IR_SIRMODE | IR_MIRMODE | IR_FIRMODE,
	.transceiver_mode	= h3_transceiver_mode,
	.select_irda	 	= h3_select_irda,
	.rx_channel		= OMAP_DMA_UART3_RX,
	.tx_channel		= OMAP_DMA_UART3_TX,
	.dest_start		= UART3_THR,
	.src_start		= UART3_RHR,
	.tx_trigger		= 0,
	.rx_trigger		= 0,
};

static struct resource h3_irda_resources[] = {
	[0] = {
		.start	= INT_UART3,
		.end	= INT_UART3,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 irda_dmamask = 0xffffffff;

static struct platform_device h3_irda_device = {
	.name		= "omapirda",
	.id		= 0,
	.dev		= {
		.platform_data	= &h3_irda_data,
		.dma_mask	= &irda_dmamask,
	},
	.num_resources	= ARRAY_SIZE(h3_irda_resources),
	.resource	= h3_irda_resources,
};

static struct platform_device h3_lcd_device = {
	.name		= "lcd_h3",
	.id		= -1,
};

static struct tsc210x_config tsc_platform_data = {
	.use_internal		= 1,
	.monitor		= TSC_VBAT | TSC_TEMP,
	.mclk			= "mclk",
};

static struct spi_board_info h3_spi_board_info[] __initdata = {
	[0] = {
		.modalias	= "tsc2101",
		.bus_num	= 2,
		.chip_select	= 0,
		.irq		= OMAP_GPIO_IRQ(H3_TS_GPIO),
		.max_speed_hz	= 16000000,
		.platform_data	= &tsc_platform_data,
	},
};

static struct platform_device *devices[] __initdata = {
	&nor_device,
	&nand_device,
        &smc91x_device,
	&intlat_device,
	&h3_kp_device,
	&h3_lcd_device,
};

static struct omap_usb_config h3_usb_config __initdata = {
	/* usb1 has a Mini-AB port and external isp1301 transceiver */
	.otg	    = 2,

#ifdef CONFIG_USB_GADGET_OMAP
	.hmc_mode       = 19,   /* 0:host(off) 1:dev|otg 2:disabled */
#elif  defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
	/* NONSTANDARD CABLE NEEDED (B-to-Mini-B) */
	.hmc_mode       = 20,   /* 1:dev|otg(off) 1:host 2:disabled */
#endif

	.pins[1]	= 3,
};

static struct omap_uart_config h3_uart_config __initdata = {
	.enabled_uarts = ((1 << 0) | (1 << 1) | (1 << 2)),
};

static struct omap_lcd_config h3_lcd_config __initdata = {
	.ctrl_name	= "internal",
};

static struct omap_board_config_kernel h3_config[] __initdata = {
	{ OMAP_TAG_USB,		&h3_usb_config },
	{ OMAP_TAG_UART,	&h3_uart_config },
	{ OMAP_TAG_LCD,		&h3_lcd_config },
};

#define H3_NAND_RB_GPIO_PIN	10

static int nand_dev_ready(struct omap_nand_platform_data *data)
{
	return gpio_get_value(H3_NAND_RB_GPIO_PIN);
}

#if defined(CONFIG_VIDEO_OV9640) || defined(CONFIG_VIDEO_OV9640_MODULE)

#include <../drivers/media/video/ov9640.h>

/*
 * Common OV9640 register initialization for all image sizes, pixel formats,
 * and frame rates
 */
const static struct ov9640_reg ov9640_common[] = {

	{ 0x12, 0x80 }, { 0x11, 0x80 }, { 0x13, 0x88 },	/* COM7, CLKRC, COM8 */
	{ 0x01, 0x58 }, { 0x02, 0x24 }, { 0x04, 0x00 },	/* BLUE, RED, COM1 */
	{ 0x0E, 0x81 }, { 0x0F, 0x4F }, { 0x14, 0xcA },	/* COM5, COM6, COM9 */
	{ 0x16, 0x02 }, { 0x1B, 0x01 }, { 0x24, 0x70 },	/* ?, PSHFT, AEW */
	{ 0x25, 0x68 }, { 0x26, 0xD3 }, { 0x27, 0x90 },	/* AEB, VPT, BBIAS */
	{ 0x2A, 0x00 }, { 0x2B, 0x00 }, { 0x32, 0x24 },	/* EXHCH, EXHCL, HREF */
	{ 0x33, 0x02 }, { 0x37, 0x02 }, { 0x38, 0x13 },	/* CHLF, ADC, ACOM */
	{ 0x39, 0xF0 }, { 0x3A, 0x00 }, { 0x3B, 0x01 },	/* OFON, TSLB, COM11 */
	{ 0x3D, 0x90 }, { 0x3E, 0x02 }, { 0x3F, 0xF2 },	/* COM13, COM14, EDGE */
	{ 0x41, 0x02 }, { 0x42, 0xC8 },		/* COM16, COM17 */
	{ 0x43, 0xF0 }, { 0x44, 0x10 }, { 0x45, 0x6C },	/* ?, ?, ? */
	{ 0x46, 0x6C }, { 0x47, 0x44 }, { 0x48, 0x44 },	/* ?, ?, ? */
	{ 0x49, 0x03 }, { 0x59, 0x49 }, { 0x5A, 0x94 },	/* ?, ?, ? */
	{ 0x5B, 0x46 }, { 0x5C, 0x84 }, { 0x5D, 0x5C },	/* ?, ?, ? */
	{ 0x5E, 0x08 }, { 0x5F, 0x00 }, { 0x60, 0x14 },	/* ?, ?, ? */
	{ 0x61, 0xCE },					/* ? */
	{ 0x62, 0x70 }, { 0x63, 0x00 }, { 0x64, 0x04 },	/* LCC1, LCC2, LCC3 */
	{ 0x65, 0x00 }, { 0x66, 0x00 },			/* LCC4, LCC5 */
	{ 0x69, 0x00 }, { 0x6A, 0x3E }, { 0x6B, 0x3F },	/* HV, MBD, DBLV */
	{ 0x6C, 0x40 }, { 0x6D, 0x30 }, { 0x6E, 0x4B },	/* GSP1, GSP2, GSP3 */
	{ 0x6F, 0x60 }, { 0x70, 0x70 }, { 0x71, 0x70 },	/* GSP4, GSP5, GSP6 */
	{ 0x72, 0x70 }, { 0x73, 0x70 }, { 0x74, 0x60 },	/* GSP7, GSP8, GSP9 */
	{ 0x75, 0x60 }, { 0x76, 0x50 }, { 0x77, 0x48 },	/* GSP10,GSP11,GSP12 */
	{ 0x78, 0x3A }, { 0x79, 0x2E }, { 0x7A, 0x28 },	/* GSP13,GSP14,GSP15 */
	{ 0x7B, 0x22 }, { 0x7C, 0x04 }, { 0x7D, 0x07 },	/* GSP16,GST1, GST2 */
	{ 0x7E, 0x10 }, { 0x7F, 0x28 }, { 0x80, 0x36 },	/* GST3, GST4, GST5 */
	{ 0x81, 0x44 }, { 0x82, 0x52 }, { 0x83, 0x60 },	/* GST6, GST7, GST8 */
	{ 0x84, 0x6C }, { 0x85, 0x78 }, { 0x86, 0x8C },	/* GST9, GST10,GST11 */
	{ 0x87, 0x9E }, { 0x88, 0xBB }, { 0x89, 0xD2 },	/* GST12,GST13,GST14 */
	{ 0x8A, 0xE6 }, { 0x13, 0xaF }, { 0x15, 0x02 },	/* GST15, COM8 */
	{ 0x22, 0x8a }, /* GROS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};

static int ov9640_sensor_power_set(int power)
{
	unsigned char expa;
	int err;

	/* read current state of GPIO EXPA outputs */
	err = read_gpio_expa(&expa, 0x27);
	if (err) {
		printk(KERN_ERR "Error reading GPIO EXPA\n");
		return err;
	}
	/* Clear GPIO EXPA P3 (CAMERA_MODULE_EN) to power-up/down sensor */
	if (power)
		expa |= 0x08;
	else
		expa &= ~0x08;

	err = write_gpio_expa(expa, 0x27);
	if (err) {
		printk(KERN_ERR "Error writing to GPIO EXPA\n");
		return err;
	}

	return err;
}

static struct v4l2_ifparm ifparm = {
	.if_type = V4L2_IF_TYPE_BT656,
	.u = {
		.bt656 = {
			 .frame_start_on_rising_vs = 1,
			 .nobt_vs_inv = 1,
			 .mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT,
			 .clock_min = OV9640_XCLK_MIN,
			 .clock_max = OV9640_XCLK_MAX,
		 },
	},
};

static int ov9640_ifparm(struct v4l2_ifparm *p)
{
	*p = ifparm;

	return 0;
}

static struct ov9640_platform_data h3_ov9640_platform_data = {
	.power_set	= ov9640_sensor_power_set,
	.default_regs	= ov9640_common,
	.ifparm		= ov9640_ifparm,
};
#endif

static int h3_pcf_setup(struct i2c_client *client, int gpio,
		unsigned ngpio, void *context)
{
	int	status;

	/* REVISIT someone with schematics should look up the rest
	 * of these signals, and configure them appropriately ...
	 * camera and audio seem to be involved, too.
	 */

	/* P0 - ? */
	gpio_irda_x = gpio + 0;
	status = gpio_request(gpio_irda_x, "irda_x");
	if (status < 0)
		goto done;
	status = gpio_direction_output(gpio_irda_x, 0);
	if (status < 0)
		goto done;

	/* P1 - set if MIR/FIR */
	gpio_irda_fir = gpio + 1;
	status = gpio_request(gpio_irda_fir, "irda_fir");
	if (status < 0)
		goto done;
	status = gpio_direction_output(gpio_irda_fir, 0);
	if (status < 0)
		goto done;

	/* 'P6' enable/disable IRDA_TX and IRDA_RX ... default, off */
	gpio_irda_enable = gpio + 6;
	status = gpio_request(gpio_irda_enable, "irda_enable");
	if (status < 0)
		goto done;
	status = gpio_direction_output(gpio_irda_enable, 0);
	if (status < 0)
		goto done;

	/* register the IRDA device now that it can be operated */
	status = platform_device_register(&h3_irda_device);

done:
	return status;
}

static struct pcf857x_platform_data h3_pcf_data = {
	/* assign these GPIO numbers right after the MPUIO lines */
	.gpio_base	= OMAP_MAX_GPIO_LINES + 16,
	.setup		= h3_pcf_setup,
};

static struct i2c_board_info __initdata h3_i2c_board_info[] = {
       {
		I2C_BOARD_INFO("pcf8574", 0x27),
		.platform_data = &h3_pcf_data,
       }, {
		I2C_BOARD_INFO("tps65013", 0x48),
               /* .irq         = OMAP_GPIO_IRQ(??), */
       },
#if defined(CONFIG_VIDEO_OV9640) || defined(CONFIG_VIDEO_OV9640_MODULE)
	{
		I2C_BOARD_INFO("ov9640", 0x30),
		.platform_data = &h3_ov9640_platform_data,
	},
#endif
	{
		I2C_BOARD_INFO("isp1301_omap", 0x2d),
		.irq		= OMAP_GPIO_IRQ(14),
	},
};

static void __init h3_init(void)
{
	/* Here we assume the NOR boot config:  NOR on CS3 (possibly swapped
	 * to address 0 by a dip switch), NAND on CS2B.  The NAND driver will
	 * notice whether a NAND chip is enabled at probe time.
	 *
	 * H3 support NAND-boot, with a dip switch to put NOR on CS2B and NAND
	 * (which on H2 may be 16bit) on CS3.  Try detecting that in code here,
	 * to avoid probing every possible flash configuration...
	 */
	nor_resource.end = nor_resource.start = omap_cs3_phys();
	nor_resource.end += SZ_32M - 1;

	nand_resource.end = nand_resource.start = OMAP_CS2B_PHYS;
	nand_resource.end += SZ_4K - 1;
	if (gpio_request(H3_NAND_RB_GPIO_PIN, "NAND ready") < 0)
		BUG();
	nand_data.dev_ready = nand_dev_ready;

	/* GPIO10 Func_MUX_CTRL reg bit 29:27, Configure V2 to mode1 as GPIO */
	/* GPIO10 pullup/down register, Enable pullup on GPIO10 */
	omap_cfg_reg(V2_1710_GPIO10);

	/* TSC2101 */
	omap_cfg_reg(W19_1610_GPIO48);
	gpio_request(H3_TS_GPIO, "tsc_irq");
	gpio_direction_input(H3_TS_GPIO);
	omap_cfg_reg(N14_1610_UWIRE_CS0);

	platform_add_devices(devices, ARRAY_SIZE(devices));
	spi_register_board_info(h3_spi_board_info,
				ARRAY_SIZE(h3_spi_board_info));
	omap_board_config = h3_config;
	omap_board_config_size = ARRAY_SIZE(h3_config);
	omap_serial_init();
	omap_register_i2c_bus(1, 100, h3_i2c_board_info,
			      ARRAY_SIZE(h3_i2c_board_info));
	h3_mmc_init();
}

static void __init h3_init_smc91x(void)
{
	omap_cfg_reg(W15_1710_GPIO40);
	if (gpio_request(40, "SMC91x irq") < 0) {
		printk("Error requesting gpio 40 for smc91x irq\n");
		return;
	}
}

static void __init h3_init_irq(void)
{
	omap1_init_common_hw();
	omap_init_irq();
	omap_gpio_init();
	h3_init_smc91x();
}

static void __init h3_map_io(void)
{
	omap1_map_common_io();
}

MACHINE_START(OMAP_H3, "TI OMAP1710 H3 board")
	/* Maintainer: Texas Instruments, Inc. */
	.phys_io	= 0xfff00000,
	.io_pg_offst	= ((0xfef00000) >> 18) & 0xfffc,
	.boot_params	= 0x10000100,
	.map_io		= h3_map_io,
	.init_irq	= h3_init_irq,
	.init_machine	= h3_init,
	.timer		= &omap_timer,
MACHINE_END
