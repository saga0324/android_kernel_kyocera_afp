/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
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
#include "disp_ext.h"
#include "mdss_dsi.h"

static int fb_dm_flag = 0;

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

#ifdef CONFIG_DISP_EXT_PROPERTY
static void update_dsi_cmd(struct dsi_panel_cmds *cmds,
	uint8_t dtype, uint8_t cmd_addr, uint8_t *data, int len)
{
	int i;

	for (i = 0; i < cmds->cmd_cnt; i++) {
		if (cmds->cmds[i].dchdr.dtype == dtype
		&& cmds->cmds[i].payload[0] == cmd_addr) {
			if (cmds->cmds[i].dchdr.dlen >= 1 + len) {
				DISP_LOCAL_LOG_EMERG("%s: %02x found at %d\n", __func__, cmd_addr, i);
				memcpy(&cmds->cmds[i].payload[1], data, len);
			}
		}
	}
}

void disp_ext_util_set_kcjprop(struct mdss_panel_data *pdata
                                       , struct fb_kcjprop_data* kcjprop_data)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	DISP_LOCAL_LOG_EMERG("DISP disp_ext_util_set_kcjprop S\n");

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	if (kcjprop_data->rw_display_gamma_valid == 0) {
		if (kcjprop_data->rw_display_gamma_normal_on_off) {
			update_dsi_cmd(&ctrl->on_cmds, DTYPE_GEN_LWRITE, 0xC8,
				kcjprop_data->rw_display_gamma_r, 12);
			update_dsi_cmd(&ctrl->on_cmds, DTYPE_GEN_LWRITE, 0xC9,
				kcjprop_data->rw_display_gamma_g, 12);
			update_dsi_cmd(&ctrl->on_cmds, DTYPE_GEN_LWRITE, 0xCA,
				kcjprop_data->rw_display_gamma_b, 12);
			DISP_LOCAL_LOG_EMERG("%s:gamma ext set\n", __func__);
		}
	}

	if (kcjprop_data->rw_display_cabc_valid == 0) {
		update_dsi_cmd(&ctrl->on_cmds, DTYPE_GEN_LWRITE, 0xB8,
			&kcjprop_data->rw_display_cabc, 1);
		DISP_LOCAL_LOG_EMERG("%s:cabc ext set\n", __func__);
	}

#ifdef CONFIG_DISP_EXT_DIAG
	if (kcjprop_data->rw_display_mipi_err_valid == 0) {
		if (kcjprop_data->rw_display_mipi_err == 1){
			disp_ext_diag_set_mipi_err_chk(1);
			DISP_LOCAL_LOG_EMERG("%s:mipi_err_chk_flg ON\n", __func__);
		} else if (kcjprop_data->rw_display_mipi_err == 0){
			disp_ext_diag_set_mipi_err_chk(0);
			DISP_LOCAL_LOG_EMERG("%s:mipi_err_chk_flg OFF\n", __func__);
		} else {
			DISP_LOCAL_LOG_EMERG("%s:rw_display_mipi_err invalid value \n", __func__);
		}
	}
#endif /* CONFIG_DISP_EXT_DIAG */

	DISP_LOCAL_LOG_EMERG("DISP disp_ext_util_set_kcjprop E\n");
}
#endif /* CONFIG_DISP_EXT_PROPERTY */

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
