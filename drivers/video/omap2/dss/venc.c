/*
 * linux/drivers/video/omap2/dss/venc.c
 *
 * Copyright (C) 2009 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * VENC settings from TI's DSS driver
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DSS_SUBSYS_NAME "VENC"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>

#include <mach/display.h>
#include <mach/cpu.h>

#include "dss.h"

#define VENC_BASE	0x48050C00

/* Venc registers */
#define VENC_REV_ID				0x00
#define VENC_STATUS				0x04
#define VENC_F_CONTROL				0x08
#define VENC_VIDOUT_CTRL			0x10
#define VENC_SYNC_CTRL				0x14
#define VENC_LLEN				0x1C
#define VENC_FLENS				0x20
#define VENC_HFLTR_CTRL				0x24
#define VENC_CC_CARR_WSS_CARR			0x28
#define VENC_C_PHASE				0x2C
#define VENC_GAIN_U				0x30
#define VENC_GAIN_V				0x34
#define VENC_GAIN_Y				0x38
#define VENC_BLACK_LEVEL			0x3C
#define VENC_BLANK_LEVEL			0x40
#define VENC_X_COLOR				0x44
#define VENC_M_CONTROL				0x48
#define VENC_BSTAMP_WSS_DATA			0x4C
#define VENC_S_CARR				0x50
#define VENC_LINE21				0x54
#define VENC_LN_SEL				0x58
#define VENC_L21__WC_CTL			0x5C
#define VENC_HTRIGGER_VTRIGGER			0x60
#define VENC_SAVID__EAVID			0x64
#define VENC_FLEN__FAL				0x68
#define VENC_LAL__PHASE_RESET			0x6C
#define VENC_HS_INT_START_STOP_X		0x70
#define VENC_HS_EXT_START_STOP_X		0x74
#define VENC_VS_INT_START_X			0x78
#define VENC_VS_INT_STOP_X__VS_INT_START_Y	0x7C
#define VENC_VS_INT_STOP_Y__VS_EXT_START_X	0x80
#define VENC_VS_EXT_STOP_X__VS_EXT_START_Y	0x84
#define VENC_VS_EXT_STOP_Y			0x88
#define VENC_AVID_START_STOP_X			0x90
#define VENC_AVID_START_STOP_Y			0x94
#define VENC_FID_INT_START_X__FID_INT_START_Y	0xA0
#define VENC_FID_INT_OFFSET_Y__FID_EXT_START_X	0xA4
#define VENC_FID_EXT_START_Y__FID_EXT_OFFSET_Y	0xA8
#define VENC_TVDETGP_INT_START_STOP_X		0xB0
#define VENC_TVDETGP_INT_START_STOP_Y		0xB4
#define VENC_GEN_CTRL				0xB8
#define VENC_OUTPUT_CONTROL			0xC4
#define VENC_OUTPUT_TEST			0xC8
#define VENC_DAC_B__DAC_C			0xC8

struct venc_config {
	u32 f_control;
	u32 vidout_ctrl;
	u32 sync_ctrl;
	u32 llen;
	u32 flens;
	u32 hfltr_ctrl;
	u32 cc_carr_wss_carr;
	u32 c_phase;
	u32 gain_u;
	u32 gain_v;
	u32 gain_y;
	u32 black_level;
	u32 blank_level;
	u32 x_color;
	u32 m_control;
	u32 bstamp_wss_data;
	u32 s_carr;
	u32 line21;
	u32 ln_sel;
	u32 l21__wc_ctl;
	u32 htrigger_vtrigger;
	u32 savid__eavid;
	u32 flen__fal;
	u32 lal__phase_reset;
	u32 hs_int_start_stop_x;
	u32 hs_ext_start_stop_x;
	u32 vs_int_start_x;
	u32 vs_int_stop_x__vs_int_start_y;
	u32 vs_int_stop_y__vs_ext_start_x;
	u32 vs_ext_stop_x__vs_ext_start_y;
	u32 vs_ext_stop_y;
	u32 avid_start_stop_x;
	u32 avid_start_stop_y;
	u32 fid_int_start_x__fid_int_start_y;
	u32 fid_int_offset_y__fid_ext_start_x;
	u32 fid_ext_start_y__fid_ext_offset_y;
	u32 tvdetgp_int_start_stop_x;
	u32 tvdetgp_int_start_stop_y;
	u32 gen_ctrl;
};

/* from TRM */
static const struct venc_config venc_config_pal_trm = {
	.f_control				= 0,
	.vidout_ctrl				= 1,
	.sync_ctrl				= 0x40,
	.llen					= 0x35F, /* 863 */
	.flens					= 0x270, /* 624 */
	.hfltr_ctrl				= 0,
	.cc_carr_wss_carr			= 0x2F7225ED,
	.c_phase				= 0,
	.gain_u					= 0x111,
	.gain_v					= 0x181,
	.gain_y					= 0x140,
	.black_level				= 0x3B,
	.blank_level				= 0x3B,
	.x_color				= 0x7,
	.m_control				= 0x2,
	.bstamp_wss_data			= 0x3F,
	.s_carr					= 0x2A098ACB,
	.line21					= 0,
	.ln_sel					= 0x01290015,
	.l21__wc_ctl				= 0x0000F603,
	.htrigger_vtrigger			= 0,

	.savid__eavid				= 0x06A70108,
	.flen__fal				= 0x00180270,
	.lal__phase_reset			= 0x00040135,
	.hs_int_start_stop_x			= 0x00880358,
	.hs_ext_start_stop_x			= 0x000F035F,
	.vs_int_start_x				= 0x01A70000,
	.vs_int_stop_x__vs_int_start_y		= 0x000001A7,
	.vs_int_stop_y__vs_ext_start_x		= 0x01AF0000,
	.vs_ext_stop_x__vs_ext_start_y		= 0x000101AF,
	.vs_ext_stop_y				= 0x00000025,
	.avid_start_stop_x			= 0x03530083,
	.avid_start_stop_y			= 0x026C002E,
	.fid_int_start_x__fid_int_start_y	= 0x0001008A,
	.fid_int_offset_y__fid_ext_start_x	= 0x002E0138,
	.fid_ext_start_y__fid_ext_offset_y	= 0x01380001,

	.tvdetgp_int_start_stop_x		= 0x00140001,
	.tvdetgp_int_start_stop_y		= 0x00010001,
	.gen_ctrl				= 0x00FF0000,
};

/* from TRM */
static const struct venc_config venc_config_ntsc_trm = {
	.f_control				= 0,
	.vidout_ctrl				= 1,
	.sync_ctrl				= 0x8040,
	.llen					= 0x359,
	.flens					= 0x20C,
	.hfltr_ctrl				= 0,
	.cc_carr_wss_carr			= 0x043F2631,
	.c_phase				= 0,
	.gain_u					= 0x102,
	.gain_v					= 0x16C,
	.gain_y					= 0x12F,
	.black_level				= 0x43,
	.blank_level				= 0x38,
	.x_color				= 0x7,
	.m_control				= 0x1,
	.bstamp_wss_data			= 0x38,
	.s_carr					= 0x21F07C1F,
	.line21					= 0,
	.ln_sel					= 0x01310011,
	.l21__wc_ctl				= 0x0000F003,
	.htrigger_vtrigger			= 0,

	.savid__eavid				= 0x069300F4,
	.flen__fal				= 0x0016020C,
	.lal__phase_reset			= 0x00060107,
	.hs_int_start_stop_x			= 0x008E0350,
	.hs_ext_start_stop_x			= 0x000F0359,
	.vs_int_start_x				= 0x01A00000,
	.vs_int_stop_x__vs_int_start_y		= 0x020701A0,
	.vs_int_stop_y__vs_ext_start_x		= 0x01AC0024,
	.vs_ext_stop_x__vs_ext_start_y		= 0x020D01AC,
	.vs_ext_stop_y				= 0x00000006,
	.avid_start_stop_x			= 0x03480078,
	.avid_start_stop_y			= 0x02060024,
	.fid_int_start_x__fid_int_start_y	= 0x0001008A,
	.fid_int_offset_y__fid_ext_start_x	= 0x01AC0106,
	.fid_ext_start_y__fid_ext_offset_y	= 0x01060006,

	.tvdetgp_int_start_stop_x		= 0x00140001,
	.tvdetgp_int_start_stop_y		= 0x00010001,
	.gen_ctrl				= 0x00F90000,
};

static const struct venc_config venc_config_pal_bdghi = {
	.f_control				= 0,
	.vidout_ctrl				= 0,
	.sync_ctrl				= 0,
	.hfltr_ctrl				= 0,
	.x_color				= 0,
	.line21					= 0,
	.ln_sel					= 21,
	.htrigger_vtrigger			= 0,
	.tvdetgp_int_start_stop_x		= 0x00140001,
	.tvdetgp_int_start_stop_y		= 0x00010001,
	.gen_ctrl				= 0x00FB0000,

	.llen					= 864-1,
	.flens					= 625-1,
	.cc_carr_wss_carr			= 0x2F7625ED,
	.c_phase				= 0xDF,
	.gain_u					= 0x111,
	.gain_v					= 0x181,
	.gain_y					= 0x140,
	.black_level				= 0x3e,
	.blank_level				= 0x3e,
	.m_control				= 0<<2 | 1<<1,
	.bstamp_wss_data			= 0x42,
	.s_carr					= 0x2a098acb,
	.l21__wc_ctl				= 0<<13 | 0x16<<8 | 0<<0,
	.savid__eavid				= 0x06A70108,
	.flen__fal				= 23<<16 | 624<<0,
	.lal__phase_reset			= 2<<17 | 310<<0,
	.hs_int_start_stop_x			= 0x00920358,
	.hs_ext_start_stop_x			= 0x000F035F,
	.vs_int_start_x				= 0x1a7<<16,
	.vs_int_stop_x__vs_int_start_y		= 0x000601A7,
	.vs_int_stop_y__vs_ext_start_x		= 0x01AF0036,
	.vs_ext_stop_x__vs_ext_start_y		= 0x27101af,
	.vs_ext_stop_y				= 0x05,
	.avid_start_stop_x			= 0x03530082,
	.avid_start_stop_y			= 0x0270002E,
	.fid_int_start_x__fid_int_start_y	= 0x0005008A,
	.fid_int_offset_y__fid_ext_start_x	= 0x002E0138,
	.fid_ext_start_y__fid_ext_offset_y	= 0x01380005,
};

const struct omap_video_timings omap_dss_pal_timings = {
	.x_res = 720,
	.y_res = 574,
	.pixel_clock = 13500,
	.hsw = 64,
	.hfp = 12,
	.hbp = 68,
	.vsw = 5,
	.vfp = 5,
	.vbp = 41,
};
EXPORT_SYMBOL(omap_dss_pal_timings);

const struct omap_video_timings omap_dss_ntsc_timings = {
	.x_res = 720,
	.y_res = 482,
	.pixel_clock = 13500,
	.hsw = 64,
	.hfp = 16,
	.hbp = 58,
	.vsw = 6,
	.vfp = 6,
	.vbp = 31,
};
EXPORT_SYMBOL(omap_dss_ntsc_timings);

static struct {
	void __iomem *base;
	struct mutex venc_lock;
	struct regulator *vdac_reg;
	u32 wss_data;
} venc;

static struct omap_panel venc_panel = {
	.name = "tv-out",
};

static inline void venc_write_reg(int idx, u32 val)
{
	__raw_writel(val, venc.base + idx);
}

static inline u32 venc_read_reg(int idx)
{
	u32 l = __raw_readl(venc.base + idx);
	return l;
}

static void venc_write_config(const struct venc_config *config)
{
	DSSDBG("write venc conf\n");

	venc_write_reg(VENC_LLEN, config->llen);
	venc_write_reg(VENC_FLENS, config->flens);
	venc_write_reg(VENC_CC_CARR_WSS_CARR, config->cc_carr_wss_carr);
	venc_write_reg(VENC_C_PHASE, config->c_phase);
	venc_write_reg(VENC_GAIN_U, config->gain_u);
	venc_write_reg(VENC_GAIN_V, config->gain_v);
	venc_write_reg(VENC_GAIN_Y, config->gain_y);
	venc_write_reg(VENC_BLACK_LEVEL, config->black_level);
	venc_write_reg(VENC_BLANK_LEVEL, config->blank_level);
	venc_write_reg(VENC_M_CONTROL, config->m_control);
	venc_write_reg(VENC_BSTAMP_WSS_DATA, config->bstamp_wss_data | venc.wss_data);
	venc_write_reg(VENC_S_CARR, config->s_carr);
	venc_write_reg(VENC_L21__WC_CTL, config->l21__wc_ctl);
	venc_write_reg(VENC_SAVID__EAVID, config->savid__eavid);
	venc_write_reg(VENC_FLEN__FAL, config->flen__fal);
	venc_write_reg(VENC_LAL__PHASE_RESET, config->lal__phase_reset);
	venc_write_reg(VENC_HS_INT_START_STOP_X, config->hs_int_start_stop_x);
	venc_write_reg(VENC_HS_EXT_START_STOP_X, config->hs_ext_start_stop_x);
	venc_write_reg(VENC_VS_INT_START_X, config->vs_int_start_x);
	venc_write_reg(VENC_VS_INT_STOP_X__VS_INT_START_Y,
		       config->vs_int_stop_x__vs_int_start_y);
	venc_write_reg(VENC_VS_INT_STOP_Y__VS_EXT_START_X,
		       config->vs_int_stop_y__vs_ext_start_x);
	venc_write_reg(VENC_VS_EXT_STOP_X__VS_EXT_START_Y,
		       config->vs_ext_stop_x__vs_ext_start_y);
	venc_write_reg(VENC_VS_EXT_STOP_Y, config->vs_ext_stop_y);
	venc_write_reg(VENC_AVID_START_STOP_X, config->avid_start_stop_x);
	venc_write_reg(VENC_AVID_START_STOP_Y, config->avid_start_stop_y);
	venc_write_reg(VENC_FID_INT_START_X__FID_INT_START_Y,
		       config->fid_int_start_x__fid_int_start_y);
	venc_write_reg(VENC_FID_INT_OFFSET_Y__FID_EXT_START_X,
		       config->fid_int_offset_y__fid_ext_start_x);
	venc_write_reg(VENC_FID_EXT_START_Y__FID_EXT_OFFSET_Y,
		       config->fid_ext_start_y__fid_ext_offset_y);

	venc_write_reg(VENC_DAC_B__DAC_C,  venc_read_reg(VENC_DAC_B__DAC_C));
	venc_write_reg(VENC_VIDOUT_CTRL, config->vidout_ctrl);
	venc_write_reg(VENC_HFLTR_CTRL, config->hfltr_ctrl);
	venc_write_reg(VENC_X_COLOR, config->x_color);
	venc_write_reg(VENC_LINE21, config->line21);
	venc_write_reg(VENC_LN_SEL, config->ln_sel);
	venc_write_reg(VENC_HTRIGGER_VTRIGGER, config->htrigger_vtrigger);
	venc_write_reg(VENC_TVDETGP_INT_START_STOP_X,
		       config->tvdetgp_int_start_stop_x);
	venc_write_reg(VENC_TVDETGP_INT_START_STOP_Y,
		       config->tvdetgp_int_start_stop_y);
	venc_write_reg(VENC_GEN_CTRL, config->gen_ctrl);
	venc_write_reg(VENC_F_CONTROL, config->f_control);
	venc_write_reg(VENC_SYNC_CTRL, config->sync_ctrl);
}

static void venc_reset(void)
{
	int t = 1000;

	venc_write_reg(VENC_F_CONTROL, 1<<8);
	while (venc_read_reg(VENC_F_CONTROL) & (1<<8)) {
		if (--t == 0) {
			DSSERR("Failed to reset venc\n");
			return;
		}
	}

	/* the magical sleep that makes things work */
	msleep(20);
}

static void venc_enable_clocks(int enable)
{
	if (enable)
		dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1 | DSS_CLK_54M |
				DSS_CLK_96M);
	else
		dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1 | DSS_CLK_54M |
				DSS_CLK_96M);
}

static const struct venc_config *venc_timings_to_config(
		struct omap_video_timings *timings)
{
	if (memcmp(&omap_dss_pal_timings, timings, sizeof(*timings)) == 0)
		return &venc_config_pal_trm;

	if (memcmp(&omap_dss_ntsc_timings, timings, sizeof(*timings)) == 0)
		return &venc_config_ntsc_trm;

	BUG();
}

int venc_init(struct platform_device *pdev)
{
	u8 rev_id;

	mutex_init(&venc.venc_lock);

	venc.wss_data = 0;

	venc_panel.timings = omap_dss_pal_timings;

	venc.base = ioremap(VENC_BASE, SZ_1K);
	if (!venc.base) {
		DSSERR("can't ioremap VENC\n");
		return -ENOMEM;
	}

	venc.vdac_reg = regulator_get(&pdev->dev, "vdac");

	if (IS_ERR(venc.vdac_reg)) {
		DSSERR("Can't get vdac regulator as source\n");
		venc.vdac_reg = NULL;
		return -ENODEV;
	}

	venc_enable_clocks(1);

	rev_id = (u8)(venc_read_reg(VENC_REV_ID) & 0xff);
	printk(KERN_INFO "OMAP VENC rev %d\n", rev_id);

	venc_enable_clocks(0);

	return 0;
}

void venc_exit(void)
{
	regulator_put(venc.vdac_reg);
	iounmap(venc.base);
}

static void venc_power_on(struct omap_display *display)
{
	regulator_enable(venc.vdac_reg);

	dispc_enable_digit_errors(0);

	venc_enable_clocks(1);

	venc_reset();
	venc_write_config(venc_timings_to_config(&display->panel->timings));

	dss_set_venc_output(display->hw_config.u.venc.type);
	dss_set_dac_pwrdn_bgz(1);

	if (display->hw_config.u.venc.type == OMAP_DSS_VENC_TYPE_COMPOSITE) {
		if (cpu_is_omap24xx())
			venc_write_reg(VENC_OUTPUT_CONTROL, 0x2);
		else
			venc_write_reg(VENC_OUTPUT_CONTROL, 0xa);
	} else { /* S-Video */
		venc_write_reg(VENC_OUTPUT_CONTROL, 0xd);
	}

	dispc_set_digit_size(display->panel->timings.x_res,
			display->panel->timings.y_res/2);

	if (display->hw_config.panel_enable)
		display->hw_config.panel_enable(display);

	dispc_enable_digit_out(1);

	dispc_enable_digit_errors(1);
}

static void venc_power_off(struct omap_display *display)
{
	venc_write_reg(VENC_OUTPUT_CONTROL, 0);
	dss_set_dac_pwrdn_bgz(0);

	dispc_enable_digit_out(0);

	if (display->hw_config.panel_disable)
		display->hw_config.panel_disable(display);

	venc_enable_clocks(0);

	regulator_disable(venc.vdac_reg);
}

static int venc_enable_display(struct omap_display *display)
{
	int r = 0;

	DSSDBG("venc_enable_display\n");

	mutex_lock(&venc.venc_lock);

	if (display->state != OMAP_DSS_DISPLAY_DISABLED) {
		r = -EINVAL;
		goto err;
	}

	venc_power_on(display);

	display->state = OMAP_DSS_DISPLAY_ACTIVE;
err:
	mutex_unlock(&venc.venc_lock);

	return r;
}

static void venc_disable_display(struct omap_display *display)
{
	DSSDBG("venc_disable_display\n");

	mutex_lock(&venc.venc_lock);

	if (display->state == OMAP_DSS_DISPLAY_DISABLED)
		goto end;

	if (display->state == OMAP_DSS_DISPLAY_SUSPENDED) {
		/* suspended is the same as disabled with venc */
		display->state = OMAP_DSS_DISPLAY_DISABLED;
		goto end;
	}

	venc_power_off(display);

	display->state = OMAP_DSS_DISPLAY_DISABLED;
end:
	mutex_unlock(&venc.venc_lock);
}

static int venc_display_suspend(struct omap_display *display)
{
	int r = 0;

	DSSDBG("venc_display_suspend\n");

	mutex_lock(&venc.venc_lock);

	if (display->state != OMAP_DSS_DISPLAY_ACTIVE) {
		r = -EINVAL;
		goto err;
	}

	venc_power_off(display);

	display->state = OMAP_DSS_DISPLAY_SUSPENDED;
err:
	mutex_unlock(&venc.venc_lock);

	return r;
}

static int venc_display_resume(struct omap_display *display)
{
	int r = 0;

	DSSDBG("venc_display_resume\n");

	mutex_lock(&venc.venc_lock);

	if (display->state != OMAP_DSS_DISPLAY_SUSPENDED) {
		r = -EINVAL;
		goto err;
	}

	venc_power_on(display);

	display->state = OMAP_DSS_DISPLAY_ACTIVE;
err:
	mutex_unlock(&venc.venc_lock);

	return r;
}

static void venc_get_timings(struct omap_display *display,
			struct omap_video_timings *timings)
{
	*timings = venc_panel.timings;
}

static void venc_set_timings(struct omap_display *display,
			struct omap_video_timings *timings)
{
	DSSDBG("venc_set_timings\n");

	/* Reset WSS data when the TV standard changes. */
	if (memcmp(&display->panel->timings, timings, sizeof(*timings)))
		venc.wss_data = 0;

	display->panel->timings = *timings;
	if (display->state == OMAP_DSS_DISPLAY_ACTIVE) {
		/* turn the venc off and on to get new timings to use */
		venc_disable_display(display);
		venc_enable_display(display);
	}
}

static int venc_check_timings(struct omap_display *display,
			struct omap_video_timings *timings)
{
	DSSDBG("venc_check_timings\n");

	if (memcmp(&omap_dss_pal_timings, timings, sizeof(*timings)) == 0)
		return 0;

	if (memcmp(&omap_dss_ntsc_timings, timings, sizeof(*timings)) == 0)
		return 0;

	return -EINVAL;
}

static u32 venc_get_wss(struct omap_display *display)
{
	/* Invert due to VENC_L21_WC_CTL:INV=1 */
	return (venc.wss_data >> 8) ^ 0xfffff;
}

static int venc_set_wss(struct omap_display *display,
			u32 wss)
{
	const struct venc_config *config;

	DSSDBG("venc_set_wss\n");

	mutex_lock(&venc.venc_lock);

	config = venc_timings_to_config(&display->panel->timings);

	/* Invert due to VENC_L21_WC_CTL:INV=1 */
	venc.wss_data = (wss ^ 0xfffff) << 8;

	venc_enable_clocks(1);

	venc_write_reg(VENC_BSTAMP_WSS_DATA, config->bstamp_wss_data | venc.wss_data);

	venc_enable_clocks(0);

	mutex_unlock(&venc.venc_lock);

	return 0;
}

int venc_init_display(struct omap_display *display)
{
	display->panel = &venc_panel;
	display->enable = venc_enable_display;
	display->disable = venc_disable_display;
	display->suspend = venc_display_suspend;
	display->resume = venc_display_resume;
	display->get_timings = venc_get_timings;
	display->set_timings = venc_set_timings;
	display->check_timings = venc_check_timings;
	display->get_wss = venc_get_wss;
	display->set_wss = venc_set_wss;

	return 0;
}

void venc_dump_regs(struct seq_file *s)
{
#define DUMPREG(r) seq_printf(s, "%-35s %08x\n", #r, venc_read_reg(r))

	venc_enable_clocks(1);

	DUMPREG(VENC_F_CONTROL);
	DUMPREG(VENC_VIDOUT_CTRL);
	DUMPREG(VENC_SYNC_CTRL);
	DUMPREG(VENC_LLEN);
	DUMPREG(VENC_FLENS);
	DUMPREG(VENC_HFLTR_CTRL);
	DUMPREG(VENC_CC_CARR_WSS_CARR);
	DUMPREG(VENC_C_PHASE);
	DUMPREG(VENC_GAIN_U);
	DUMPREG(VENC_GAIN_V);
	DUMPREG(VENC_GAIN_Y);
	DUMPREG(VENC_BLACK_LEVEL);
	DUMPREG(VENC_BLANK_LEVEL);
	DUMPREG(VENC_X_COLOR);
	DUMPREG(VENC_M_CONTROL);
	DUMPREG(VENC_BSTAMP_WSS_DATA);
	DUMPREG(VENC_S_CARR);
	DUMPREG(VENC_LINE21);
	DUMPREG(VENC_LN_SEL);
	DUMPREG(VENC_L21__WC_CTL);
	DUMPREG(VENC_HTRIGGER_VTRIGGER);
	DUMPREG(VENC_SAVID__EAVID);
	DUMPREG(VENC_FLEN__FAL);
	DUMPREG(VENC_LAL__PHASE_RESET);
	DUMPREG(VENC_HS_INT_START_STOP_X);
	DUMPREG(VENC_HS_EXT_START_STOP_X);
	DUMPREG(VENC_VS_INT_START_X);
	DUMPREG(VENC_VS_INT_STOP_X__VS_INT_START_Y);
	DUMPREG(VENC_VS_INT_STOP_Y__VS_EXT_START_X);
	DUMPREG(VENC_VS_EXT_STOP_X__VS_EXT_START_Y);
	DUMPREG(VENC_VS_EXT_STOP_Y);
	DUMPREG(VENC_AVID_START_STOP_X);
	DUMPREG(VENC_AVID_START_STOP_Y);
	DUMPREG(VENC_FID_INT_START_X__FID_INT_START_Y);
	DUMPREG(VENC_FID_INT_OFFSET_Y__FID_EXT_START_X);
	DUMPREG(VENC_FID_EXT_START_Y__FID_EXT_OFFSET_Y);
	DUMPREG(VENC_TVDETGP_INT_START_STOP_X);
	DUMPREG(VENC_TVDETGP_INT_START_STOP_Y);
	DUMPREG(VENC_GEN_CTRL);
	DUMPREG(VENC_OUTPUT_CONTROL);
	DUMPREG(VENC_OUTPUT_TEST);

	venc_enable_clocks(0);

#undef DUMPREG
}
