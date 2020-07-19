/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
*/
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2018 KYOCERA Corporation
 * (C) 2019 KYOCERA Corporation
*/
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h>
#include "mdp3.h"
#include "mdp3_hwio.h"
#include "mdss.h"
#include "mdss_dsi.h"
#include "mdss_panel.h"
#include "mdss_debug.h"
#include "kdisp_com.h"

static int kcdisp_debug_flg = 0;

/* DSI Video */
#define KDISP_DSI_VIDEO_MODE_ACTIVE_H_OFFSET		0x24
#define KDISP_DSI_VIDEO_MODE_ACTIVE_V_OFFSET		0x28
#define KDISP_DSI_VIDEO_MODE_TOTAL_OFFSET			0x2C
#define KDISP_DSI_VIDEO_MODE_HSYNC_OFFSET			0x30
#define KDISP_DSI_VIDEO_MODE_VSYNC_OFFSET			0x34
#define KDISP_DSI_VIDEO_MODE_VSYNC_VPOS_OFFSET		0x38

/* DSI Timing */
#define KDISP_DSIPHY_LANE_CFG_CTRL_SIZE			0x0040
#define KDISP_DSIPHY_LANE_CFG_CTRL_NUM			0x0009
#define KDISP_DSIPHY_LANE_CFG_CTRL_OFFSET		0x0004
#define KDISP_DSIPHY_CLKLANE_CFG_CTRL_BASE		0x0100
#define KDISP_DSIPHY_TIMING_CTRL_OFFSET			0x0140


void kdisp_debug_log_output_dsi_cmd_tx(char *bp, int len)
{
	int i;
	char logbuf[512];
	char cmddata[7];

	if(!kcdisp_debug_flg)
		return;

	logbuf[0] = 0;
	for (i = 0; i < len; i++) {
		sprintf(cmddata, "0x%02X, ", bp[i]);
		strcat(logbuf, cmddata);
	}
	pr_err("[DISP]%s  %s\n", __func__, logbuf);
}

void kdisp_debug_dump_mdp_timing_reg(void)
{
	u32 reg;

	if(!kcdisp_debug_flg)
		return;

	reg = MDP3_REG_READ(MDP3_REG_DSI_VIDEO_HSYNC_CTL);
	pr_err("%s: [DISP]MDP3_REG_DSI_VIDEO_HSYNC_CTL=%x\n", __func__, reg);

	reg = MDP3_REG_READ(MDP3_REG_DSI_VIDEO_VSYNC_PERIOD);
	pr_err("%s: [DISP]MDP3_REG_DSI_VIDEO_VSYNC_PERIOD=%x\n", __func__, reg);

	reg = MDP3_REG_READ(MDP3_REG_DSI_VIDEO_VSYNC_PULSE_WIDTH);
	pr_err("%s: [DISP]MDP3_REG_DSI_VIDEO_VSYNC_PULSE_WIDTH=%x\n", __func__, reg);

	reg = MDP3_REG_READ(MDP3_REG_DSI_VIDEO_DISPLAY_HCTL);
	pr_err("%s: [DISP]MDP3_REG_DSI_VIDEO_DISPLAY_HCTL=%x\n", __func__, reg);

	reg = MDP3_REG_READ(MDP3_REG_DSI_VIDEO_DISPLAY_V_START);
	pr_err("%s: [DISP]MDP3_REG_DSI_VIDEO_DISPLAY_V_START=%x\n", __func__, reg);

	reg = MDP3_REG_READ(MDP3_REG_DSI_VIDEO_DISPLAY_V_END);
	pr_err("%s: [DISP]MDP3_REG_DSI_VIDEO_DISPLAY_V_END=%x\n", __func__, reg);

	reg = MDP3_REG_READ(MDP3_REG_DSI_VIDEO_ACTIVE_HCTL);
	pr_err("%s: [DISP]MDP3_REG_DSI_VIDEO_ACTIVE_HCTL=%x\n", __func__, reg);

	reg = MDP3_REG_READ(MDP3_REG_DSI_VIDEO_ACTIVE_V_START);
	pr_err("%s: [DISP]MDP3_REG_DSI_VIDEO_ACTIVE_V_START=%x\n", __func__, reg);

	reg = MDP3_REG_READ(MDP3_REG_DSI_VIDEO_ACTIVE_V_END);
	pr_err("%s: [DISP]MDP3_REG_DSI_VIDEO_ACTIVE_V_END=%x\n", __func__, reg);

	reg = MDP3_REG_READ(MDP3_REG_DSI_VIDEO_BORDER_COLOR);
	pr_err("%s: [DISP]MDP3_REG_DSI_VIDEO_BORDER_COLOR=%x\n", __func__, reg);

	reg = MDP3_REG_READ(MDP3_REG_DSI_VIDEO_UNDERFLOW_CTL);
	pr_err("%s: [DISP]MDP3_REG_DSI_VIDEO_UNDERFLOW_CTL=%x\n", __func__, reg);

	reg = MDP3_REG_READ(MDP3_REG_DSI_VIDEO_HSYNC_SKEW);
	pr_err("%s: [DISP]MDP3_REG_DSI_VIDEO_HSYNC_SKEW=%x\n", __func__, reg);

	reg = MDP3_REG_READ(MDP3_REG_DSI_VIDEO_CTL_POLARITY);
	pr_err("%s: [DISP]MDP3_REG_DSI_VIDEO_CTL_POLARITY=%x\n", __func__, reg);
}

void kdisp_debug_dump_dsi_timing_reg(void __iomem *phy_io_base)
{
	u32 reg;
	int i, off, ln;
	void __iomem *base;

	if(!kcdisp_debug_flg)
		return;

	base = phy_io_base;

	/* Data Lane */
	for (ln = 0; ln < 5; ln++) {
		pr_err("%s: [DISP]Data Lane[%d]\n", __func__, ln);

		off = (ln * KDISP_DSIPHY_LANE_CFG_CTRL_SIZE);
		for (i = 0; i < KDISP_DSIPHY_LANE_CFG_CTRL_NUM; i++) {
			reg = MIPI_INP(base + off);
			pr_err("%s: data_timing_8909[%d]=%x\n", __func__, i, reg);
			off += KDISP_DSIPHY_LANE_CFG_CTRL_OFFSET;
		}
	}

	/* Clock Lane */
	pr_err("%s: [DISP]Clk Lane\n", __func__);

	off = KDISP_DSIPHY_CLKLANE_CFG_CTRL_BASE;
	for (i = 0; i < KDISP_DSIPHY_LANE_CFG_CTRL_NUM; i++) {
		reg = MIPI_INP(base + off);
		pr_err("%s: data_timing_8909[%d]=%x\n", __func__, i, reg);
		off += KDISP_DSIPHY_LANE_CFG_CTRL_OFFSET;
	}

	/* DSI_PHY_TIMING_CTRL */
	off = KDISP_DSIPHY_TIMING_CTRL_OFFSET;	/* phy timing ctrl 0 - 11 */
	for (i = 0; i < 12; i++) {
		reg = MIPI_INP(base + off);
		pr_err("%s: [DISP]DSI_PHY_TIMING_CTRL[%d]=%x\n", __func__, i, reg);
		off += 4;
	}
}

void kdisp_debug_dump_dsi_clkout_timing_reg(void __iomem *phy_io_base)
{
	u32 reg;
	void __iomem *base;

	if(!kcdisp_debug_flg)
		return;

	base = phy_io_base;
	reg = MIPI_INP(base + 0xc4); /* DSI_CLKOUT_TIMING_CTRL */
	pr_err("%s: DSI_CLKOUT_TIMING_CTRL=%x\n", __func__, reg);
}

void kdisp_debug_dump_dsi_video_reg(char __iomem *base)
{
	u32 reg;

	if(!kcdisp_debug_flg)
		return;

	reg = MIPI_INP(base + KDISP_DSI_VIDEO_MODE_ACTIVE_H_OFFSET);
	pr_err("%s: [DISP]DSI_VIDEO_MODE_ACTIVE_H=%x\n", __func__, reg);

	reg = MIPI_INP(base + KDISP_DSI_VIDEO_MODE_ACTIVE_V_OFFSET);
	pr_err("%s: [DISP]DSI_VIDEO_MODE_ACTIVE_V=%x\n", __func__, reg);

	reg = MIPI_INP(base + KDISP_DSI_VIDEO_MODE_TOTAL_OFFSET);
	pr_err("%s: [DISP]DSI_VIDEO_MODE_TOTAL=%x\n", __func__, reg);

	reg = MIPI_INP(base + KDISP_DSI_VIDEO_MODE_HSYNC_OFFSET);
	pr_err("%s: [DISP]DSI_VIDEO_MODE_HSYNC=%x\n", __func__, reg);

	reg = MIPI_INP(base + KDISP_DSI_VIDEO_MODE_VSYNC_OFFSET);
	pr_err("%s: [DISP]DSI_VIDEO_MODE_VSYNC=%x\n", __func__, reg);

	reg = MIPI_INP(base + KDISP_DSI_VIDEO_MODE_VSYNC_VPOS_OFFSET);
	pr_err("%s: [DISP]DSI_VIDEO_MODE_VSYNC_VPOS=%x\n", __func__, reg);
}

ssize_t kdisp_panel_set_test_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc = 0;
	int test_mode = 0;

	rc = kstrtoint(buf, 10, &test_mode);
	if (rc) {
		pr_err("%s: kstrtoint failed. rc=%d\n", __func__, rc);
		return rc;
	}

	kcdisp_debug_flg = test_mode;

	return count;
}

ssize_t kdisp_panel_get_test_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", kcdisp_debug_flg);
}

ssize_t kdisp_panel_get_kctrl_info(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	struct kc_ctrl_info *kctrl_data;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	if (!ctrl_pdata) {
		pr_err("%s: ctrl_pdata is null\n", __func__);
		return -EFAULT;
	}
	kctrl_data = &ctrl_pdata->kc_ctrl_data;

	pr_notice("%s: [DISP]panel_det_en     =%x\n", __func__, kctrl_data->panel_det_en);
	pr_notice("%s: [DISP]panel_det_en     =%x\n", __func__, kctrl_data->panel_det_dsi_en);
//	pr_notice("%s: [DISP]tp_incell_en     =%x\n", __func__, kctrl_data->tp_incell_en);
//	pr_notice("%s: [DISP]dsi_freq_en      =%x\n", __func__, kctrl_data->dsi_freq_en);
//	pr_notice("%s: [DISP]dsi_freq_mode    =%x\n", __func__, kctrl_data->dsi_freq_mode);
//	pr_notice("%s: [DISP]sre_mode_en      =%x\n", __func__, kctrl_data->sre_mode_en);
//	pr_notice("%s: [DISP]current_sre_mode =%x\n", __func__, kctrl_data->current_sre_mode);
//	pr_notice("%s: [DISP]user_sre_mode    =%x\n", __func__, kctrl_data->user_sre_mode);
//	pr_notice("%s: [DISP]sre_mode_max     =%x\n", __func__, kctrl_data->sre_mode_max);
//	pr_notice("%s: [DISP]easywake_en      =%x\n", __func__, kctrl_data->easywake_en);
//	pr_notice("%s: [DISP]easywake_req     =%x\n", __func__, kctrl_data->easywake_req);
//	pr_notice("%s: [DISP]touch_state      =%x\n", __func__, kctrl_data->touch_state);

	return scnprintf(buf, PAGE_SIZE, "%d\n", 0);
}
