/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 *
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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
#include <linux/kernel.h>
#include <linux/kc_led.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include "mdss_dsi.h"
#include "disp_ext.h"

#define DETECT_BOARD_NUM 5

static char reg_read_cmd[1] = {0x0A};
static struct dsi_cmd_desc dsi_cmds = {
	{DTYPE_DCS_READ, 1, 0, 1, 0, sizeof(reg_read_cmd)},
	reg_read_cmd
};
static char panel_id[] = {0x08, 0x1C};

static int panel_detection = 0;       /* -1:not panel 0:Not test 1:panel found */

int disp_ext_board_detect_board(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int i;
	int panel_found;
	struct dcs_cmd_req cmdreq;
	char rbuf[5];

	if (panel_detection != 0) {
		DISP_LOCAL_LOG_EMERG("%s: panel Checked(%d)\n", __func__, panel_detection);
		return panel_detection;
	}

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dsi_cmds;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = 1;
	cmdreq.cb = NULL;
	cmdreq.rbuf = rbuf;

	panel_found = 0;

	if (ctrl->panel_data.panel_info.cont_splash_enabled) {
		if (gpio_get_value(ctrl->rst_gpio)) {
			panel_found = 1;
		} else {
			pr_err("%s:rst_gpio is None\n",__func__);
		}
	} else {
		if (gpio_get_value(ctrl->rst_gpio)) {
			for (i = 0; i < DETECT_BOARD_NUM; i++) {
				mdss_dsi_cmdlist_put(ctrl, &cmdreq);
				pr_notice("%s - panel ID: %02x\n", __func__,
					ctrl->rx_buf.data[0]);
				if (ctrl->rx_buf.data[0] == panel_id[0]
				||  ctrl->rx_buf.data[0] == panel_id[1]) {
					panel_found = 1;
					break;
				}
			}
		} else {
			return 0;
		}
	}
	if (panel_found != 1) {
		pr_err("%s: panel not found\n", __func__);
#ifdef ENABLE_LCD_DETECTION
		pr_err("%s: notify not found to led\n", __func__);
		light_led_disp_set_panel(LIGHT_MAIN_WLED_LCD_DIS, LIGHT_LCD_PANEL0);
#endif
		panel_detection = -1;
		return panel_detection;
	}
	pr_notice("%s: panel found\n", __func__);
#ifdef ENABLE_LCD_DETECTION
	pr_notice("%s: notify found to led\n", __func__);
	light_led_disp_set_panel(LIGHT_MAIN_WLED_LCD_EN, LIGHT_LCD_PANEL0);
#endif
	panel_detection = 1;
	return panel_detection;
}

int disp_ext_board_get_panel_detect(void)
{
    return panel_detection;
}
