/*
 * linux/arch/arm/mach-omap2/board-rx51-video.c
 *
 * Copyright (C) 2008 Nokia
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/i2c/twl4030.h>
#include <linux/regulator/machine.h>
#include <linux/bootmem.h>

#include <asm/mach-types.h>

#include <mach/mcspi.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/pm.h>
#include <mach/vram.h>
#include <mach/vrfb.h>
#include <mach/dss_boottime.h>
#include <mach/omap-pm.h>

#include <linux/omapfb.h>
#include <mach/display.h>
#include <../drivers/video/omap2/displays/panel-acx565akm.h>

#define ANDROID_PAN_FLIP

#if defined OMAP2_DSS_VRAM_SIZE && OMAP2_DSS_VRAM_SIZE > 0
# define GRALLOC_MEMSIZE	OMAP2_DSS_VRAM_SIZE*1024*1024
#endif

#ifdef ANDROID_PAN_FLIP
# define GRALLOC_WIDTH		800
# define GRALLOC_HEIGHT		960
#else
# define GRALLOC_WIDTH		800
# define GRALLOC_HEIGHT		480
#endif

#define GRALLOC_VID_PLANES	2
#define GRALLOC_VID_PAGES	2


#if defined(CONFIG_FB_OMAP2) || defined(CONFIG_FB_OMAP2_MODULE)

static struct omap2_mcspi_device_config mipid_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,
};

static int twl4030_bklight_level = -1;		/* load from HW at first read */

#define TWL4030_PWM0_ENABLE     (1 << 2)
#define TWL4030_PWM0_CLK_ENABLE (1 << 0)

static const int twl4030_bklight_max = 127;
static int twl4030_bklight_initialized;

static int twl4030_get_bklight_level(void)
{
	if (twl4030_bklight_level == -1) {
		u8 reg;

		twl4030_i2c_read_u8(TWL4030_MODULE_INTBR, &reg, 0x0c);
		if (reg & TWL4030_PWM0_ENABLE) {
			twl4030_i2c_read_u8(TWL4030_MODULE_PWM0, &reg, 0x01);
			twl4030_bklight_level = reg;
		} else {
			twl4030_bklight_level = 0;
		}
	}

	return twl4030_bklight_level;
}

static void twl4030_set_bklight_level(int level)
{
	u8 reg;

	if (!twl4030_bklight_initialized) {
		/* Mux GPIO6 as PWM0 : PMBR1 = xxxx01xx */
		twl4030_i2c_read_u8(TWL4030_MODULE_INTBR, &reg, 0x0d);
		reg &= ~(3 << 2);
		reg |= (1 << 2);
		twl4030_i2c_write_u8(TWL4030_MODULE_INTBR, reg, 0x0d);

		twl4030_bklight_initialized = 1;
	}

	twl4030_i2c_read_u8(TWL4030_MODULE_INTBR, &reg, 0x0c);

	if (level != 0) {
		/* Configure the duty cycle. */
		twl4030_i2c_write_u8(TWL4030_MODULE_PWM0, 0, 0x00);
		twl4030_i2c_write_u8(TWL4030_MODULE_PWM0, level, 0x01);

		/* Enable clock for PWM0 a few microseconds before PWM0 itself.
		   This is not mentioned in TWL4030 spec. but some older boards
		   don't set backlight level properly from time to time
		   without this delay. */
		reg |= TWL4030_PWM0_CLK_ENABLE;
		twl4030_i2c_write_u8(TWL4030_MODULE_INTBR, reg, 0x0c);
		udelay(50);
		reg |= TWL4030_PWM0_ENABLE;
	} else {
		/* Disable PWM0 before disabling its clock, see comment above */
		reg &= ~TWL4030_PWM0_ENABLE;
		twl4030_i2c_write_u8(TWL4030_MODULE_INTBR, reg, 0x0c);
		udelay(50);
		reg &= ~TWL4030_PWM0_CLK_ENABLE;
	}

	twl4030_i2c_write_u8(TWL4030_MODULE_INTBR, reg, 0x0c);

	twl4030_bklight_level = level;
}

static struct spi_board_info rx51_video_spi_board_info[] = {
	[0] = {
		.modalias		= "acx565akm",
		.bus_num		= 1,
		.chip_select		= 2,
		.max_speed_hz		= 6000000,
		.controller_data	= &mipid_mcspi_config,
	},
};

/* acx565akm LCD Panel */
static int acx565akm_enable(struct omap_display *display)
{
	if (display->hw_config.panel_reset_gpio != -1) {
		pr_debug("Release LCD reset\n");
		gpio_set_value(display->hw_config.panel_reset_gpio, 1);
	}

	return 0;
}

static void acx565akm_disable(struct omap_display *display)
{
	if (display->hw_config.panel_reset_gpio != -1) {
		pr_debug("Enable LCD reset\n");
		gpio_set_value(display->hw_config.panel_reset_gpio, 0);
	}
}

static int rx51_set_backlight_level(struct omap_display *display, int level)
{
	twl4030_set_bklight_level(level);

	return 0;
}

static int rx51_get_backlight_level(struct omap_display *display)
{
	return twl4030_get_bklight_level();
}

static struct acx565akm_panel_data acx565akm_data = {
	.bc_connected = 1,
};

static struct omap_dss_display_config acx565akm_display_data = {
	.type = OMAP_DISPLAY_TYPE_SDI,
	.name = "lcd",
	.panel_name = "panel-acx565akm",
	.panel_enable = acx565akm_enable,
	.panel_disable = acx565akm_disable,
	.panel_reset_gpio = -1, /* set later from tag data */
	.max_backlight_level = 127,
	.set_backlight = rx51_set_backlight_level,
	.get_backlight = rx51_get_backlight_level,
	.panel_data = &acx565akm_data,
	.u.sdi = {
		.datapairs = 2,
	},
};

static void __init acx565akm_dev_init(void)
{
	const struct omap_lcd_config *conf;

	conf = omap_get_config(OMAP_TAG_LCD, struct omap_lcd_config);
	if (conf != NULL) {
		int ret = gpio_request(conf->nreset_gpio, "acx565akm-reset");
		if (ret) {
			printk(KERN_ERR "Failed to request GPIO %d for "
				"acx565akm reset\n", conf->nreset_gpio);
		} else {
			gpio_direction_output(conf->nreset_gpio, 1);
			acx565akm_display_data.panel_reset_gpio =
				conf->nreset_gpio;
		}
	}
}

/* TV-out */
static struct omap_dss_display_config venc_display_data = {
	.type = OMAP_DISPLAY_TYPE_VENC,
	.name = "tv",
	.u.venc.type = OMAP_DSS_VENC_TYPE_COMPOSITE,
};

/* DSS */
static struct omap_dss_board_info rx51_dss_data = {
	.get_last_off_on_transaction_id = get_last_off_on_transaction_id,
	.set_min_bus_tput = omap_pm_set_min_bus_tput,
	.num_displays = 2,
	.displays = {
		&acx565akm_display_data,
		&venc_display_data,
	},
	.fifo_thresholds = {
		[OMAP_DSS_GFX] = { .low = 2944, .high = 3008, },
	},
};

static struct platform_device rx51_dss_device = {
	.name          = "omapdss",
	.id            = -1,
	.dev            = {
		.platform_data = &rx51_dss_data,
	},
};

static struct platform_device *rx51_video_devices[] = {
	&rx51_dss_device,
};

/* TV-OUT (VDAC) regulator */
static struct regulator_consumer_supply rx51_vdac_supply = {
	.supply		= "vdac",
	.dev		= &rx51_dss_device.dev,
};

struct regulator_init_data rx51_vdac_data = {
	.constraints = {
		.name			= "VDAC_18",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &rx51_vdac_supply,
};


static struct omapfb_platform_data omapfb_config;

static size_t rx51_vrfb_min_phys_size(int bpp)
{
	unsigned bytespp = bpp >> 3;
	size_t landscape;
	size_t portrait;

	/* For physical screen resolution */
	landscape = omap_vrfb_min_phys_size(GRALLOC_WIDTH, GRALLOC_HEIGHT, bytespp);
	portrait = omap_vrfb_min_phys_size(GRALLOC_HEIGHT, GRALLOC_WIDTH, bytespp);

	return max(landscape, portrait);
}

static size_t rx51_vrfb_min_plane_size(int bpp)
{
	unsigned bytespp = bpp >> 3;
	size_t mem_size;
	size_t xres, yres;

	xres = GRALLOC_WIDTH + 4;
	yres = GRALLOC_WIDTH + 8;
	
	/* VID planes, buffers, resolution. */
	mem_size = GRALLOC_VID_PLANES * GRALLOC_VID_PAGES * PAGE_ALIGN(bytespp * xres * yres);
	
	return mem_size;
}

static void __init rx51_add_gfx_fb(u32 paddr, size_t size, enum omapfb_color_format format)
{
	omapfb_config.mem_desc.region_cnt = 1;

	omapfb_config.mem_desc.region[0].paddr = paddr;
	omapfb_config.mem_desc.region[0].size = size;
	omapfb_config.mem_desc.region[0].format = format;
	omapfb_config.mem_desc.region[0].format_used = 1;

	omapfb_set_platform_data(&omapfb_config);
}

static void __init rx51_detect_vram(void)
{
	unsigned long vram_paddr;
	size_t vram_size;
	unsigned long gfx_paddr;
	size_t gfx_size;
	size_t vid_size;
	enum omapfb_color_format format;

	gfx_paddr = dss_boottime_get_plane_base(0);

	if (gfx_paddr == -1UL)
		return;

	gfx_size = PAGE_ALIGN(rx51_vrfb_min_phys_size(dss_boottime_get_plane_bpp(0)));

#ifdef GRALLOC_MEMSIZE
	vid_size = PAGE_ALIGN( GRALLOC_MEMSIZE - gfx_size );
#else
	vid_size = rx51_vrfb_min_plane_size(16);
#endif
	pr_debug("VRAM: GFX size: %d, VID size:%d", gfx_size, vid_size);
	
	vram_size = gfx_size + vid_size;
	vram_paddr = gfx_paddr - vid_size;
	
	format = dss_boottime_get_plane_format(0);
	
	rx51_add_gfx_fb(gfx_paddr, gfx_size, format);
	
	if (reserve_bootmem(vram_paddr, vram_size, BOOTMEM_EXCLUSIVE) < 0) {
		pr_err("FB: can't reserve VRAM region\n");
		return;
	}
	
	if (omap_vram_add_region(vram_paddr, vram_size) < 0) {
		free_bootmem(vram_paddr, vram_size);
		pr_err("Can't set VRAM region\n");
		return;
	}

	pr_info("VRAM: %zd bytes at 0x%lx. (Detected %zd at %#lx)\n",
		vram_size, vram_paddr, gfx_size, gfx_paddr);
}

static void __init rx51_alloc_vram(void)
{
	unsigned long vram_paddr;
	size_t vram_size;
	size_t gfx_size;
	size_t vid_size;

	vid_size = PAGE_ALIGN(rx51_vrfb_min_plane_size(16));

#ifdef GRALLOC_MEMSIZE
	gfx_size = PAGE_ALIGN(GRALLOC_MEMSIZE - vid_size);
#else
	gfx_size = PAGE_ALIGN(rx51_vrfb_min_phys_size(16));
#endif
	pr_info("VRAM: GFX size: %d, VID size:%d", gfx_size, vid_size);
	
	vram_size = gfx_size + vid_size;
	vram_paddr = virt_to_phys(alloc_bootmem_pages(vram_size));
	
	BUG_ON(vram_paddr & ~PAGE_MASK);
	
	rx51_add_gfx_fb(vram_paddr, gfx_size, OMAPFB_COLOR_RGB565);
	
	if (omap_vram_add_region(vram_paddr, vram_size) < 0) {
		free_bootmem(vram_paddr, vram_size);
		pr_err("Can't set VRAM region\n");
		return;
	}

	pr_info("VRAM: %zd bytes at 0x%lx\n", vram_size, vram_paddr);
}


void __init rx51_video_mem_init(void)
{
	if (dss_boottime_plane_is_enabled(0))
		rx51_detect_vram();
	else
		rx51_alloc_vram();
}

static int __init rx51_video_init(void)
{
	if (!machine_is_nokia_rx51())
		return 0;

	platform_add_devices(rx51_video_devices, ARRAY_SIZE(rx51_video_devices));
	spi_register_board_info(rx51_video_spi_board_info,
			ARRAY_SIZE(rx51_video_spi_board_info));

	acx565akm_dev_init();

	return 0;
}

subsys_initcall(rx51_video_init);

#else

void __init rx51_video_mem_init(void)
{
}

#endif	/* CONFIG_FB_OMAP2 || CONFIG_FB_OMAP2_MODULE */

