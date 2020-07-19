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
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/kc_led.h>
#include "mdss_dsi.h"
#include "kdisp_com.h"

#define KDISP_BL_DELAYTIME		20

static int kdisp_bl_ctrl_led_type = 0;

static void on_comp_notify(struct work_struct *p)
{
#ifdef ENABLE_LCD_DETECTION
//	struct kc_ctrl_info *kctrl_data = container_of(p, struct kc_ctrl_info,
//		on_comp_work);

//	if (kctrl_data->wled_disp_power_set == NULL) {
//		pr_err("%s: wled_disp_power_set error\n", __func__);
//	}

	pr_debug("%s: notify ON to LED\n", __func__);

	if (kdisp_bl_ctrl_led_type == 0) {
		light_led_disp_power_set(LIGHT_MAIN_WLED_LCD_PWREN);
	} else if (kdisp_bl_ctrl_led_type == 1){
		lv5216_light_led_disp_power_set(LIGHT_MAIN_WLED_LCD_PWREN);
	} else if (kdisp_bl_ctrl_led_type == 2){
		lv5216_2_light_led_disp_power_set(LIGHT_MAIN_WLED_LCD_PWREN);
	}else{
		cat4004b_light_led_disp_power_set(LIGHT_MAIN_WLED_LCD_PWREN);
	}

//	kctrl_data->wled_disp_power_set(LIGHT_MAIN_WLED_LCD_PWREN);
#endif /* ENABLE_LCD_DETECTION */
}

void kdisp_bl_ctrl_onoff(struct kc_ctrl_info *kctrl_data, bool onoff)
{
#ifdef ENABLE_LCD_DETECTION
	if (kctrl_data->wled_disp_power_set == NULL) {
		pr_err("%s: wled_disp_power_set error\n", __func__);
	}

	if(onoff) {
		schedule_delayed_work(&kctrl_data->on_comp_work, msecs_to_jiffies(KDISP_BL_DELAYTIME));
	} else {
		cancel_delayed_work_sync(&kctrl_data->on_comp_work);
		pr_debug("%s: notify OFF to LED\n", __func__);
		kctrl_data->wled_disp_power_set(LIGHT_MAIN_WLED_LCD_PWRDIS);
	}
#endif /* ENABLE_LCD_DETECTION */
}

void kdisp_bl_ctrl_init(struct device_node *np, struct kc_ctrl_info *kctrl_data)
{
	int rc = 0;
	const char *string;

	kctrl_data->wled_disp_power_set = NULL;
	rc = of_property_read_string(np,
				"kc,led-type", &string);

	if (!rc) {
		
		if (!strcmp(string, "bd6098")) {
			kdisp_bl_ctrl_led_type = 0; //test
			kctrl_data->wled_disp_power_set = light_led_disp_power_set;
		} else if (!strcmp(string, "lv5216")) {
			kdisp_bl_ctrl_led_type = 1; //test
			kctrl_data->wled_disp_power_set = lv5216_light_led_disp_power_set;
		} else if (!strcmp(string, "lv5216_2")) {
			kdisp_bl_ctrl_led_type = 2; //test
			kctrl_data->wled_disp_power_set = lv5216_2_light_led_disp_power_set;
		}else{
			kdisp_bl_ctrl_led_type = 3; //test
			kctrl_data->wled_disp_power_set = cat4004b_light_led_disp_power_set;
		}
	}

	INIT_DELAYED_WORK(&kctrl_data->on_comp_work, on_comp_notify);
}
