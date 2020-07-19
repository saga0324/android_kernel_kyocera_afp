/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 * (C) 2019 KYOCERA Corporation
 *
 * drivers/video/msm/mdss/disp_ext_util.c
 *
 * Copyright (c) 2008-2012, Code Aurora Forum. All rights reserved.
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
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/msm_mdp.h>
#include <linux/io.h>
#include "disp_ext.h"
#include "mdss_dsi.h"

#include "mdp3.h"
#include "mdp3_dma.h"
#include "mdp3_hwio.h"

#define KFBM_DISABLE 0
#define KFBM_ENABLE  1
static int kfbm_stat = KFBM_DISABLE;
static int fb_dm_flag = 0;
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


void disp_ext_set_dmflag(int dmflag)
{
	fb_dm_flag = dmflag;
}

int disp_ext_is_invalid()
{
	if (fb_dm_flag != 0 && strcmp(current->comm, "kdispdiag") != 0) {
		return 1;
	} else {
		return 0;
	}
}

int disp_ext_is_invalid_all(void)
{
	int panel_detect;

	panel_detect = disp_ext_board_get_panel_detect();

	if ((kfbm_stat == KFBM_ENABLE) && (panel_detect == -1)) {
		return 1;
	}

	return 0;
}

int disp_ext_otp_check_value(void)
{
	return 0;
}

void disp_ext_print_dsi_state(struct msm_fb_data_type *mfd)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	pr_notice("mdss_dsi_ctrl_pdata.ctrl_state: %X\n", ctrl_pdata->ctrl_state);
}
void disp_ext_print_display_commit(struct msm_fb_data_type *mfd)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	pr_notice("Display Commit Status: %X\n", ctrl_pdata->display_commit_status);
	pr_notice("Display Commit Count: %u\n", ctrl_pdata->display_commit_count);
}
void disp_ext_print_refresh_state(struct msm_fb_data_type *mfd)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	pr_notice("Refresh check count:    %u\n", ctrl_pdata->refresh_check_count);
	pr_notice("Refresh recovery count: %u\n", ctrl_pdata->refresh_check_error);
}
void disp_ext_dsi_recovery_state(struct msm_fb_data_type *mfd)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	pr_notice("DSI recovery count:    %u\n", ctrl_pdata->dsi_recovery_count);
}
void disp_ext_dsi_internalerror_state(struct msm_fb_data_type *mfd)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	pr_notice("DSI_INTR_ERROR count:    %u\n", ctrl_pdata->dsi_intr_error_count);
}
void disp_ext_debug_log_output_dsi_cmd_tx(char *bp, int len)
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

void disp_ext_debug_dump_mdp_timing_reg(void)
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

void disp_ext_debug_dump_dsi_timing_reg(void __iomem *phy_io_base)
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

void disp_ext_debug_dump_dsi_clkout_timing_reg(void __iomem *phy_io_base)
{
	u32 reg;
	void __iomem *base;

	if(!kcdisp_debug_flg)
		return;

	base = phy_io_base;
	reg = MIPI_INP(base + 0xc4); /* DSI_CLKOUT_TIMING_CTRL */
	pr_err("%s: DSI_CLKOUT_TIMING_CTRL=%x\n", __func__, reg);
}

void disp_ext_debug_dump_dsi_video_reg(char __iomem *base)
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

ssize_t disp_ext_panel_set_test_mode(struct device *dev,
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

ssize_t disp_ext_panel_get_test_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", kcdisp_debug_flg);
}
