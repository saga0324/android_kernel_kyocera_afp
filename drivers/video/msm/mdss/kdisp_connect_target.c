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
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/kc_led.h>
#include <soc/qcom/socinfo.h>

#include "mdss.h"
#include "mdss_dsi.h"
#include "kdisp_com.h"

#define PANEL_FOUND_GPIO_VALUE		0
#define PANEL_CHECK_RETRY_NUM		5

static int panel_detect_status		= PANEL_NOT_TEST;
static int panel_detect				= PANEL_NOT_FOUND;

module_param(panel_detect, int, S_IRUGO );
MODULE_PARM_DESC(panel_detect, "Panel Detect");

static int __init set_panel_detect(char *buf)
{
	panel_detect = PANEL_FOUND;
	pr_err("%s: early_param_set! panel_detect=%d\n", __func__, panel_detect);
	return 0;
}
early_param("panel_detect", set_panel_detect);

static int kdisp_connect_pinctrl_init(struct platform_device *pdev)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	struct kc_ctrl_info *kc_ctrl_data;

	ctrl_pdata = platform_get_drvdata(pdev);
	kc_ctrl_data = &ctrl_pdata->kc_ctrl_data;

	if (!kc_ctrl_data->panel_det_en) {
		pr_debug("%s: panel detect disable\n", __func__);
		return 0;
	}

	kc_ctrl_data->pin_res.pinctrl = ctrl_pdata->pin_res.pinctrl;

	kc_ctrl_data->pin_res.gpio_state_det_active
		= pinctrl_lookup_state(kc_ctrl_data->pin_res.pinctrl,
				KDISP_PINCTRL_STATE_DET_ACTIVE);
	if (IS_ERR_OR_NULL(kc_ctrl_data->pin_res.gpio_state_det_active))
		pr_warn("%s: can not get det_active pinstate\n", __func__);

	kc_ctrl_data->pin_res.gpio_state_det_suspend
		= pinctrl_lookup_state(kc_ctrl_data->pin_res.pinctrl,
				KDISP_PINCTRL_STATE_DET_SUSPEND);
	if (IS_ERR_OR_NULL(kc_ctrl_data->pin_res.gpio_state_det_suspend))
		pr_warn("%s: can not get det_suspend pinstate\n", __func__);

	return 0;

}

static int kdisp_connect_pinctrl_set_state(
	struct kc_ctrl_info *kctrl_data,
	bool active)
{
	struct pinctrl_state *pin_state;
	int rc = -EFAULT;

	if (!kctrl_data->panel_det_en) {
		pr_debug("%s: panel detect disable\n", __func__);
		return 0;
	}

	if (IS_ERR_OR_NULL(kctrl_data->pin_res.pinctrl))
		return PTR_ERR(kctrl_data->pin_res.pinctrl);

	pin_state = active ? kctrl_data->pin_res.gpio_state_det_active
				: kctrl_data->pin_res.gpio_state_det_suspend;

	if (!IS_ERR_OR_NULL(pin_state)) {
		rc = pinctrl_select_state(kctrl_data->pin_res.pinctrl,
				pin_state);
		if (rc)
			pr_err("%s: can not set %s pins\n", __func__,
					active ? KDISP_PINCTRL_STATE_DET_ACTIVE
					: KDISP_PINCTRL_STATE_DET_SUSPEND);
	} else {
		pr_err("%s: invalid '%s' pinstate\n", __func__,
					active ? KDISP_PINCTRL_STATE_DET_ACTIVE
					: KDISP_PINCTRL_STATE_DET_SUSPEND);
	}
	msleep(1);
	return rc;
}

static int kdisp_connect_parse_gpio_params(struct platform_device *pdev,
	struct kc_ctrl_info *kc_ctrl_data)
{
	if (!kc_ctrl_data->panel_det_en) {
		pr_debug("%s: panel detect disable\n", __func__);
		return 0;
	}

	kc_ctrl_data->lcd_det_gpio = of_get_named_gpio(pdev->dev.of_node,
			"kc,lcd_det-gpio", 0);

	if (!gpio_is_valid(kc_ctrl_data->lcd_det_gpio))
		pr_err("%s:%d, lcd_det gpio not specified\n",
			 							__func__, __LINE__);

	return 0;
}

static int kdisp_connect_check_panel_detect(struct kc_ctrl_info *kctrl_data)
{
	int det_check = PANEL_FOUND_GPIO_VALUE;
	int i;

	if (kctrl_data == NULL) {
		pr_err("%s: param error\n", __func__);
		panel_detect_status = PANEL_FOUND;
		return panel_detect_status;
	}

	if ((!kctrl_data->panel_det_en)&&(!kctrl_data->panel_det_dsi_en)) {
		pr_err("%s: panel detect disable\n", __func__);
		panel_detect_status = PANEL_FOUND;
	} else {
		if (panel_detect_status == PANEL_NOT_TEST) {
			panel_detect_status = panel_detect;

			if (kctrl_data->panel_det_en) {
				if (panel_detect == PANEL_FOUND) {
					panel_detect_status = PANEL_FOUND;
					pr_notice("%s: panel found\n", __func__);
				} else {
					kdisp_connect_pinctrl_set_state(kctrl_data, 1);

					for (i=0; i<PANEL_CHECK_RETRY_NUM; i++) {
						msleep(1);
						det_check = gpio_get_value(kctrl_data->lcd_det_gpio);
						if (det_check == PANEL_FOUND_GPIO_VALUE) {
							break;
						}
						pr_notice("%s: panel detect error. retry=%d\n", __func__, i);
					}

					if (det_check == PANEL_FOUND_GPIO_VALUE) {
						panel_detect_status = PANEL_FOUND;
						pr_notice("%s: panel found. splash disable.\n", __func__);
					} else {
						panel_detect_status = PANEL_NOT_FOUND;
						pr_notice("%s: panel not found\n", __func__);
					}

					kdisp_connect_pinctrl_set_state(kctrl_data, 0);
				}
			}
		}
	}

	return panel_detect_status;
}

static void kdisp_connect_notify_to_light(struct kc_ctrl_info *kctrl_data)
{
#ifdef ENABLE_LCD_DETECTION
	if (kctrl_data->wled_disp_set_panel == NULL) {
		pr_err("%s: wled_disp_set_panel error\n", __func__);
	}

	if (panel_detect_status == PANEL_FOUND) {
		kctrl_data->wled_disp_set_panel(LIGHT_MAIN_WLED_LCD_EN, kctrl_data->light_panel_class);
		pr_notice("%s: notify found to led\n", __func__);
	} else {
		kctrl_data->wled_disp_set_panel(LIGHT_MAIN_WLED_LCD_DIS, kctrl_data->light_panel_class);
		pr_err("%s: notify not found to led\n", __func__);
	}
#endif /* ENABLE_LCD_DETECTION */
}

int kdisp_connect_get_panel_detect(void)
{
	return panel_detect_status;
}

void kdisp_connect_init(struct platform_device *pdev,
	struct device_node *np,
	struct kc_ctrl_info *kctrl_data)
{
	int rc = 0;
	const char *string;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;

	if (!np || !kctrl_data) {
		pr_err("%s: Invalid arguments\n", __func__);
		return;
	}

	kctrl_data->panel_det_en = of_property_read_bool(np,
		"kc,panel-detect-enabled");

	kctrl_data->panel_det_dsi_en = of_property_read_bool(np,
		"kc,panel-detect-dsi-enabled");

	pr_notice("%s:panel_det_en=%d panel_det_gpio_en=%d\n",
			__func__, kctrl_data->panel_det_en, kctrl_data->panel_det_dsi_en);




	kdisp_connect_pinctrl_init(pdev);
	kdisp_connect_parse_gpio_params(pdev, kctrl_data);
	kdisp_connect_check_panel_detect(kctrl_data);

	ctrl_pdata = platform_get_drvdata(pdev);
	if (panel_detect == PANEL_NOT_FOUND) {
		ctrl_pdata->panel_data.panel_info.cont_splash_enabled = 0;
		pr_err("%s: splash image disable\n", __func__);
	} else {
		if (gpio_get_value(ctrl_pdata->rst_gpio) == 0) {
			ctrl_pdata->panel_data.panel_info.cont_splash_enabled = 0;
			pr_err("%s: optbit2 splash disable\n", __func__);
		}
	}

	kctrl_data->light_panel_class = 0;
	kctrl_data->wled_disp_set_panel = NULL;
	rc = of_property_read_string(np,
				"kc,led-type", &string);
	if (!rc) {
		if (!strcmp(string, "bd6098")) {
			kctrl_data->wled_disp_set_panel = light_led_disp_set_panel;
		} else if (!strcmp(string, "lv5216")){
			kctrl_data->wled_disp_set_panel = lv5216_light_led_disp_set_panel;
		} else if (!strcmp(string, "lv5216_2")){
			kctrl_data->wled_disp_set_panel = lv5216_2_light_led_disp_set_panel;
			if(strstr(ctrl_pdata->panel_data.panel_info.panel_name,"kc tovis fwvga video mode dsi panel") != NULL){
				kctrl_data->light_panel_class = 1;
			}
			if(strstr(ctrl_pdata->panel_data.panel_info.panel_name,"kc tovis2 fwvga video mode dsi panel") != NULL){
				kctrl_data->light_panel_class = 2;
			}
		}
		else{
			kctrl_data->wled_disp_set_panel = cat4004b_light_led_disp_set_panel;
		}

	}

	kdisp_connect_notify_to_light(kctrl_data);
}
