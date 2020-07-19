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

#define DETECT_BOARD_RETRY 5
#define PANEL_NOT_TEST   0
#define PANEL_FOUND      1
#define PANEL_NOT_FOUND -1

static int panel_detect_status   = PANEL_NOT_TEST;
static int panel_detect = PANEL_NOT_FOUND;
module_param(panel_detect, int, S_IRUGO );
MODULE_PARM_DESC(panel_detect, "Panel Detect");

static int __init set_panel_detect(char *buf)
{
	panel_detect = PANEL_FOUND;
	pr_err("%s: early_param_set! panel_detect=%d\n", __func__, panel_detect);
	return 0;
}
early_param("panel_detect", set_panel_detect);

int disp_ext_board_get_panel_detect(void)
{
	if (panel_detect_status == PANEL_NOT_TEST) {
		panel_detect_status = panel_detect;
		pr_notice("%s: panel detect %d\n", __func__, panel_detect_status);

		if (panel_detect_status == PANEL_FOUND) {
			pr_notice("%s: panel found\n", __func__);
#ifdef ENABLE_LCD_DETECTION
			pr_notice("%s: notify found to led\n", __func__);
			light_led_disp_set_panel(LIGHT_MAIN_WLED_LCD_EN, LIGHT_LCD_PANEL0);
#endif
		} else if (panel_detect_status == PANEL_NOT_FOUND) {
			pr_err("%s: panel not found\n", __func__);
#ifdef ENABLE_LCD_DETECTION
			pr_err("%s: notify not found to led\n", __func__);
			light_led_disp_set_panel(LIGHT_MAIN_WLED_LCD_DIS, LIGHT_LCD_PANEL0);
#endif
		}
	}

	return panel_detect_status;
}
