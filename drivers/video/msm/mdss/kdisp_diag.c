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
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/kdisp.h>
#include "mdss_fb.h"
#include "mdss_dsi.h"
#include "mdss_mdp.h"
#include "kdisp_com.h"

static struct delayed_work workRefresher;

static int refresher_interval[] = {8, 33, 67};
static unsigned int refresher_rate;
static struct msm_fb_data_type *param_mfd;
static int fb_dm_flag = 0;

extern int mdss_fb_blank_sub(int blank_mode, struct fb_info *info,
			     int op_enable);
extern void mdss_dsi_enable_irq(struct mdss_dsi_ctrl_pdata *ctrl, u32 term);
extern void mdss_dsi_disable_irq(struct mdss_dsi_ctrl_pdata *ctrl, u32 term);


static void kdisp_diag_set_dmflag(int dmflag)
{
	fb_dm_flag = dmflag;
}

int kdisp_diag_get_dmflag(void)
{
	return fb_dm_flag;
}

static int kdisp_diag_reg_write(struct fb_info *info, unsigned int cmd, void __user *argp)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	int ret;
	struct dsi_cmd_desc dm_dsi_cmds;
	struct disp_diag_mipi_write_reg_type mipi_reg_data;
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	ret = copy_from_user(&mipi_reg_data, argp, sizeof(mipi_reg_data));
	if (ret) {
		pr_err("MSMFB_MIPI_REG_WRITE: error 1[%d] \n", ret);
		return ret;
	}

	if ((mipi_reg_data.type != DTYPE_DCS_WRITE)
	 && (mipi_reg_data.type != DTYPE_DCS_WRITE1)
	 && (mipi_reg_data.type != DTYPE_DCS_LWRITE)
	 && (mipi_reg_data.type != DTYPE_GEN_WRITE)
	 && (mipi_reg_data.type != DTYPE_GEN_WRITE1)
	 && (mipi_reg_data.type != DTYPE_GEN_WRITE2)
	 && (mipi_reg_data.type != DTYPE_GEN_LWRITE)) {
		pr_err("MSMFB_MIPI_REG_WRITE: error 2[%d] \n", mipi_reg_data.type);
		return -EINVAL;
	}

	/* Tx command send */
	memset(&dm_dsi_cmds, 0x00, sizeof(dm_dsi_cmds));
	dm_dsi_cmds.dchdr.dtype = mipi_reg_data.type; /* Command type */
	dm_dsi_cmds.dchdr.last = 1; /* Last command */
	dm_dsi_cmds.dchdr.vc = 0; /* Virtual Channel */
	dm_dsi_cmds.dchdr.ack = 0; /* No ACK trigger msg from peripeheral */
	dm_dsi_cmds.dchdr.wait = mipi_reg_data.wait; /* wait response by msleep() */
	dm_dsi_cmds.dchdr.dlen = mipi_reg_data.len; /* Data length */
	dm_dsi_cmds.payload = (char *)mipi_reg_data.data;  /* Data */
	pr_info("@@@ Tx command\n");
	pr_info("    - dtype = 0x%08X\n", dm_dsi_cmds.dchdr.dtype);
	pr_info("    - last  = %d\n", dm_dsi_cmds.dchdr.last);
	pr_info("    - vc    = %d\n", dm_dsi_cmds.dchdr.vc);
	pr_info("    - ack   = %d\n", dm_dsi_cmds.dchdr.ack);
	pr_info("    - wait  = %d\n", dm_dsi_cmds.dchdr.wait);
	pr_info("    - dlen  = %d\n", dm_dsi_cmds.dchdr.dlen);
	pr_info("    - payload:%02X %02X %02X %02X\n",
		dm_dsi_cmds.payload[0], dm_dsi_cmds.payload[1],
		dm_dsi_cmds.payload[2], dm_dsi_cmds.payload[3]);
	pr_info("    -         %02X %02X %02X %02X\n",
		dm_dsi_cmds.payload[4], dm_dsi_cmds.payload[5],
		dm_dsi_cmds.payload[6], dm_dsi_cmds.payload[7]);

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dm_dsi_cmds;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

//	if (pdata->panel_info.type == MIPI_CMD_PANEL)
//		mdss_dsi_clk_ctrl(ctrl_pdata, ctrl_pdata->dsi_clk_handle,DSI_ALL_CLKS, 1);

	if (mipi_reg_data.speed == 1) {
		mdss_dsi_set_tx_power_mode(0, pdata);
	} else {
		mdss_dsi_set_tx_power_mode(1, pdata);
	}

//	if (pdata->panel_info.type == MIPI_CMD_PANEL)
//		mdss_dsi_clk_ctrl(ctrl_pdata, ctrl_pdata->dsi_clk_handle,DSI_ALL_CLKS, 0);

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	ret = copy_to_user(argp, &mipi_reg_data, sizeof(mipi_reg_data));
	if (ret) {
		pr_err("KDISP_MIPI_REG_WRITE: error [%d] \n", ret);
	}

	return ret;
}

static int kdisp_diag_reg_read(struct fb_info *info, unsigned int cmd, void __user *argp)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	int ret;
	struct dsi_cmd_desc dm_dsi_cmds;
	struct disp_diag_mipi_read_reg_type mipi_reg_data;
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	char rbuf[25];

	ret = copy_from_user(&mipi_reg_data, argp, sizeof(mipi_reg_data));
	if (ret) {
		pr_err("KDISP_MIPI_REG_READ: error 1[%d] \n", ret);
		return ret;
	}

	if ((mipi_reg_data.type != DTYPE_DCS_READ)
	 && (mipi_reg_data.type != DTYPE_GEN_READ)
	 && (mipi_reg_data.type != DTYPE_GEN_READ1)
	 && (mipi_reg_data.type != DTYPE_GEN_READ2)) {
		pr_err("KDISP_MIPI_REG_READ: error 2[%d] \n", mipi_reg_data.type);
		return -EINVAL;
	}

	/* Rx command send */
	memset(&dm_dsi_cmds, 0x00, sizeof(dm_dsi_cmds));
	dm_dsi_cmds.dchdr.dtype = mipi_reg_data.type; /* Command type */
	dm_dsi_cmds.dchdr.last = 1; /* Last command */
	dm_dsi_cmds.dchdr.vc = 0; /* Virtual Channel */
	dm_dsi_cmds.dchdr.ack = 1; /* Don't care, dsi_host default ON set */
	dm_dsi_cmds.dchdr.wait = mipi_reg_data.wait; /* wait response by msleep() */
	dm_dsi_cmds.dchdr.dlen = mipi_reg_data.len; /* Data length */
	dm_dsi_cmds.payload = (char *)mipi_reg_data.data; /* Data */
	pr_info("@@@ Rx command\n");
	pr_info("    - dtype = 0x%08X\n", dm_dsi_cmds.dchdr.dtype);
	pr_info("    - last  = %d\n", dm_dsi_cmds.dchdr.last);
	pr_info("    - vc    = %d\n", dm_dsi_cmds.dchdr.vc);
	pr_info("    - ack   = %d\n", dm_dsi_cmds.dchdr.ack);
	pr_info("    - wait  = %d\n", dm_dsi_cmds.dchdr.wait);
	pr_info("    - dlen  = %d\n", dm_dsi_cmds.dchdr.dlen);
	pr_info("    - payload:%02X %02X %02X %02X\n",
		dm_dsi_cmds.payload[0], dm_dsi_cmds.payload[1],
		dm_dsi_cmds.payload[2], dm_dsi_cmds.payload[3]);
	pr_info("    -         %02X %02X %02X %02X\n",
		dm_dsi_cmds.payload[4], dm_dsi_cmds.payload[5],
		dm_dsi_cmds.payload[6], dm_dsi_cmds.payload[7]);

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dm_dsi_cmds;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = (int)mipi_reg_data.rlen;
	cmdreq.cb = NULL;
	cmdreq.rbuf = rbuf;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	memcpy(mipi_reg_data.data, ctrl_pdata->rx_buf.data, ctrl_pdata->rx_buf.len);

	ret = copy_to_user(argp, &mipi_reg_data, sizeof(mipi_reg_data));
	if (ret) {
		pr_err("KDISP_MIPI_REG_READ: error 3[%d] \n", ret);
	}

	return ret;
}

static int kdisp_diag_refresher_start(struct fb_info *info, void __user *argp)
{
	int ret;
	unsigned int rate;

	ret = copy_from_user(&rate, argp, sizeof(rate));
	if (ret) {
		pr_err("%s copy_from_user[%d]\n", __func__, ret);
		return ret;
	}

	if (rate > 2) {
		pr_err("%s invalid rate[%u]\n", __func__, rate);
		return -EINVAL;
	}

	refresher_rate = rate;
	param_mfd = (struct msm_fb_data_type *)info->par;
	param_mfd->msm_fb_backup.info = *info;
	param_mfd->msm_fb_backup.disp_commit.var = info->var;
	schedule_delayed_work(&workRefresher, 0);

	return 0;
}

static int kdisp_diag_refresher_stop(void)
{
	cancel_delayed_work_sync(&workRefresher);

	return 0;
}

static void refresher_workqueue(struct work_struct *p)
{
	mutex_lock(&param_mfd->mdp_sync_pt_data.sync_mutex);

	if (!atomic_read(&param_mfd->commits_pending)) {
		atomic_inc(&param_mfd->mdp_sync_pt_data.commit_cnt);
		atomic_inc(&param_mfd->commits_pending);
		wake_up_all(&param_mfd->commit_wait_q);
	}

	mutex_unlock(&param_mfd->mdp_sync_pt_data.sync_mutex);
	schedule_delayed_work(&workRefresher, msecs_to_jiffies(refresher_interval[refresher_rate]));
}

void kdisp_diag_init(void)
{
	INIT_DELAYED_WORK(&workRefresher, refresher_workqueue);
}

int kdisp_diag_ioctl_handler(struct fb_info *info, unsigned int cmd, void __user *argp)
{
	struct msm_fb_data_type *mfd;
	unsigned int display_standby_flag;
	int ret = 0;

	mfd = (struct msm_fb_data_type *)info->par;
	if (!mfd)
		return -EINVAL;

	switch (cmd) {
	case KDISP_DIAGVALID:
		pr_notice("[DISP]KDISP_DIAGVALID\n");
		kdisp_diag_set_dmflag(0);
		break;
	case KDISP_DIAGINVALID:
		pr_notice("[DISP]KDISP_DIAGINVALID\n");
		kdisp_diag_set_dmflag(1);
		break;
	case KDISP_MIPI_REG_WRITE:
		pr_info("[DISP]KDISP_MIPI_REG_WRITE\n");
		ret = kdisp_diag_reg_write(info, cmd, argp);
		break;

	case KDISP_MIPI_REG_READ:
		pr_info("[DISP]KDISP_MIPI_REG_READ\n");
		ret = kdisp_diag_reg_read(info, cmd, argp);
		break;

	case KDISP_DISPLAY_STANDBY:
		ret = copy_from_user(&display_standby_flag, argp, sizeof(unsigned int));
		if (!ret) {
			switch (display_standby_flag) {
			case 0:
				pr_info("[DISP]KDISP_DISPLAY_STANDBY FB_BLANK_UNBLANK\n");
				mdss_fb_blank_sub(FB_BLANK_UNBLANK, info, mfd->op_enable);
				break;
			case 1:
				pr_info("[DISP]KDISP_DISPLAY_STANDBY FB_BLANK_POWERDOWN\n");
				mdss_fb_blank_sub(FB_BLANK_POWERDOWN, info, mfd->op_enable);
				break;
			default:
				ret = -EINVAL;
			}
		}
		break;

	case KDISP_DISP_DET_GET:
		pr_info("[DISP]KDISP_DISP_DET_GET\n");
		ret = kdisp_connect_get_panel_detect();
		ret = copy_to_user(argp, &ret, sizeof(int));
		break;

	case KDISP_OTP_READ:
		pr_info("[DISP]KDISP_OTP_READ\n");
//		ret = disp_ext_otp_check_value();
		ret = copy_to_user(argp, &ret, sizeof(ret));
		break;

	case KDISP_RESUME_SW_REFRESHER:
		pr_info("[DISP]KDISP_RESUME_SW_REFRESHER \n");
		ret = kdisp_diag_refresher_start(info, argp);
		break;

	case KDISP_SUSPEND_SW_REFRESHER:
		pr_info("[DISP]KDISP_SUSPEND_SW_REFRESHER \n");
		ret = kdisp_diag_refresher_stop();
		break;

	default:
		ret = -ENOSYS;
		pr_err("[DISP]kdisp_diag_ioctl_handler param error \n");
		break;
	}

	return ret;
}
