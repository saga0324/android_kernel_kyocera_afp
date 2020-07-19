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
#include "mdss.h"
#include "mdss_dsi.h"
#include "mdss_panel.h"
#include "kdisp_com.h"

extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds, u32 flags);
int mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);

int kdisp_dsi_panel_on_post1(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct kc_ctrl_info *kctrl_data;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	kctrl_data = &ctrl->kc_ctrl_data;

	pr_debug("%s: ctrl=%pK ndx=%d\n", __func__, ctrl, ctrl->ndx);


	if (kctrl_data->on_post_cmds1.cmd_cnt &&
		!pdata->panel_info.cont_splash_enabled) {
		mdss_dsi_panel_cmds_send(ctrl,
				(struct dsi_panel_cmds *)&kctrl_data->on_post_cmds1, CMD_REQ_COMMIT);
//		ctrl->on_post_time = ktime_get();
	}

	pr_debug("%s:-\n", __func__);
	return 0;
}

int kdisp_dsi_panel_on_post2(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct kc_ctrl_info *kctrl_data;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	kctrl_data = &ctrl_pdata->kc_ctrl_data;

	pr_debug("%s: cmd_cnt=%d\n", __func__, kctrl_data->on_post_cmds2.cmd_cnt);

	if (kctrl_data->on_post_cmds2.cmd_cnt &&
		!pdata->panel_info.cont_splash_enabled) {

//		s64 on_post_wait;

//		on_post_wait = ktime_to_us(ktime_sub(ctrl_pdata->on_post_time, ktime_get()));
//		if (on_post_wait > 0) {
//			pr_err("ctrl_pdata->on_post_time is invalid\n");
//			on_post_wait = 0;
//		}
//		on_post_wait += 120 * 1000;
//		pr_debug("%s on_post_wait: %ld us\n", __func__, (long)on_post_wait);
//		if (on_post_wait > 0)
//			usleep(on_post_wait);
		mdss_dsi_panel_cmds_send(ctrl_pdata,
				(struct dsi_panel_cmds *)&kctrl_data->on_post_cmds2, CMD_REQ_COMMIT);
	}

	pr_debug("%s:-\n", __func__);
	return 0;
}

int kdisp_dsi_event_handler(struct mdss_panel_data *pdata, int event)
{
	int rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s+: event=%d\n", __func__, event);

	switch (event) {
	case KDISP_EVENT_PANEL_ON_POST1:
		kdisp_dsi_panel_on_post1(pdata);
		break;
	case KDISP_EVENT_PANEL_ON_POST2:
		kdisp_dsi_panel_on_post2(pdata);
		break;
	default:
		pr_debug("%s: unhandled event=%d\n", __func__, event);
		rc = 1;
		break;
	}
	pr_debug("%s-:event=%d, rc=%d\n", __func__, event, rc);
	return rc;
}

static void kdisp_mipiic_dsi_ulps_reset(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	u32 tmp;

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	/* ULPS */
	tmp = MIPI_INP(ctrl->ctrl_base + 0xAC);
	tmp &= ~0x1300;
	tmp |= 0x0013;
	MIPI_OUTP(ctrl->ctrl_base + 0xAC, tmp);
	wmb();

	tmp = MIPI_INP(ctrl->ctrl_base + 0xAC);
	wmb();

	usleep_range(1000, 2000);

	/* LP11 */
	tmp = MIPI_INP(ctrl->ctrl_base + 0xAC);
	tmp &= ~0x0013;
	tmp |= 0x1300;
	MIPI_OUTP(ctrl->ctrl_base + 0xAC, tmp);
	wmb();

	tmp = MIPI_INP(ctrl->ctrl_base + 0xAC);
	wmb();

	mdss_dsi_sw_reset(ctrl, false);
	mdss_dsi_host_init(pdata);
	mdss_dsi_op_mode_config(pdata->panel_info.mipi.mode, pdata);

	usleep_range(5000, 6000);
}

int kdisp_mipiic_dsi_panel_on_post(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct kc_ctrl_info *kctrl_data;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	kctrl_data = &ctrl->kc_ctrl_data;

	if (!kctrl_data->mipiic_en) {
		return 0;
	}

	pr_debug("%s: ctrl=%pK ndx=%d\n", __func__, ctrl, ctrl->ndx);

	kdisp_mipiic_dsi_ulps_reset(pdata);

	if (kctrl_data->on_post_cmds.cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl,
				(struct dsi_panel_cmds *)&kctrl_data->on_post_cmds, CMD_REQ_COMMIT);

	pr_debug("%s:-\n", __func__);
	return 0;
}

int kdisp_mipiic_power_ctrl(struct kc_ctrl_info *kctrl_data,
	int enable)
{
	int ret;

	if (!kctrl_data->mipiic_en) {
		return 0;
	}

	pr_debug("%s: mipi_vdd enable=%d\n", __func__, enable);

	if (enable) {
		if (kctrl_data->mipi_vdd) {
			ret = regulator_set_optimum_mode(kctrl_data->mipi_vdd, 10000);
			if (ret < 0) {
				pr_err("failed to set optimum mode mipi_vdd");
				goto mipi_vdd_fail;
			}
			ret = regulator_enable(kctrl_data->mipi_vdd);
			if (ret < 0) {
				pr_err("failed to enable mipi_vdd");
				goto mipi_vdd_fail;
			}
		}

	} else {
		if (kctrl_data->mipi_vdd) {
			ret = regulator_disable(kctrl_data->mipi_vdd);
			if (ret < 0) {
				pr_err("failed to enable mipi_vdd");
			}
			ret = regulator_set_optimum_mode(kctrl_data->mipi_vdd, 100);
			if (ret < 0) {
				pr_err("failed to set optimum mode mipi_vdd");
			}
		}
	}

	pr_debug("%s-\n", __func__);

	return 0;

mipi_vdd_fail:
	return ret;
}

static void kdisp_mipiic_power_init(struct device* dev,
		struct kc_ctrl_info *kctrl_data)
{
	int rc;

	if (!kctrl_data->mipiic_en) {
		return;
	}

	pr_debug("%s+\n",__func__);

	kctrl_data->mipi_vdd = regulator_get(dev, "mipi_vdd");
	if (!IS_ERR(kctrl_data->mipi_vdd)) {
		rc = regulator_set_voltage(kctrl_data->mipi_vdd,
			1800000,1800000);
		if (rc < 0) {
			pr_err("%s:%d, set vltg fail mipi_vdd\n",
							__func__, __LINE__);
			kctrl_data->mipi_vdd = NULL;
		}
	} else {
		pr_err("%s:%d, mipi_vdd is not specified\n",
						__func__, __LINE__);
		kctrl_data->mipi_vdd = NULL;
	}

	pr_debug("%s-\n",__func__);
}

int kdisp_mipiic_panel_power_ctrl(struct kc_ctrl_info *kctrl_data, int enable)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (!kctrl_data->mipiic_en) {
		return 0;
	}

	ctrl_pdata = container_of(kctrl_data, struct mdss_dsi_ctrl_pdata, kc_ctrl_data);

	pr_debug("%s+: enable=%d\n",__func__, enable);
	if (enable) {
		ret = msm_dss_enable_vreg(
			ctrl_pdata->power_data[DSI_PANEL_PM].vreg_config,
			ctrl_pdata->power_data[DSI_PANEL_PM].num_vreg, 1);
		if (ret) {
			pr_err("%s: failed to enable vregs\n", __func__);
			goto error;
		}
	} else {
		ret = msm_dss_enable_vreg(
			ctrl_pdata->power_data[DSI_PANEL_PM].vreg_config,
			ctrl_pdata->power_data[DSI_PANEL_PM].num_vreg, 0);
		if (ret)
			pr_err("%s: failed to disable vregs\n", __func__);
	}

error:
	if (ret) {
			msm_dss_enable_vreg(
				ctrl_pdata->power_data[DSI_PANEL_PM].vreg_config,
				ctrl_pdata->power_data[DSI_PANEL_PM].num_vreg, 0);
	}
	pr_debug("%s-: ret=%d\n",__func__,ret);
	return ret;
}

int kdisp_mipiic_reset_seq(struct kc_ctrl_info *kctrl_data, int enable)
{
	int rc = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata;
	struct mdss_panel_info *pinfo = NULL;

	if (!kctrl_data->mipiic_en) {
		return 0;
	}

	ctrl_pdata = container_of(kctrl_data, struct mdss_dsi_ctrl_pdata, kc_ctrl_data);
	pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (enable) {
		if (gpio_is_valid(kctrl_data->vmipi_12_en_gpio)) {
			rc = gpio_request(kctrl_data->vmipi_12_en_gpio, "vmipi_12_en");
			if (rc) {
				pr_err("request vmipi_12_en gpio failed,rc=%d\n",
									rc);
				goto vmipi_12_en_gpio_err;
			}
		}
		if (gpio_is_valid(kctrl_data->refclk_gpio)) {
			rc = gpio_request(kctrl_data->refclk_gpio, "refclk");
			if (rc) {
				pr_err("request refclk gpio failed,rc=%d\n",
								rc);
				goto refclk_gpio_err;
			}
		}
		if (gpio_is_valid(kctrl_data->ic_rst_gpio)) {
			rc = gpio_request(kctrl_data->ic_rst_gpio, "ic_rst_n");
			if (rc) {
				pr_err("request ic_rst gpio failed,rc=%d\n",
									rc);
				goto ic_rst_gpio_err;
			}
		}

		if (gpio_is_valid(kctrl_data->vmipi_12_en_gpio)) {
			gpio_set_value((kctrl_data->vmipi_12_en_gpio), 1);
			pr_debug("%s: [DISP]vmipi_12_en ON\n", __func__);
		}
		usleep_range(3000, 4000);

		rc = kdisp_mipiic_power_ctrl(kctrl_data, true);
		usleep_range(3000, 4000);

		if (gpio_is_valid(kctrl_data->refclk_gpio)) {
			gpio_set_value((kctrl_data->refclk_gpio), 1);
			pr_debug("%s: [DISP]refclk ON\n", __func__);
		}
		usleep_range(3000, 4000);

		if (gpio_is_valid(kctrl_data->ic_rst_gpio)) {
			gpio_set_value((kctrl_data->ic_rst_gpio), 1);
			pr_debug("%s: [DISP]ic_rst ON\n", __func__);
		}
	} else {
		if (gpio_is_valid(kctrl_data->ic_rst_gpio)) {
			gpio_set_value((kctrl_data->ic_rst_gpio), 0);
			gpio_free(kctrl_data->ic_rst_gpio);
			pr_debug("%s: [DISP]ic_rst OFF\n", __func__);
		}
		usleep_range(3000, 4000);

		if (gpio_is_valid(kctrl_data->refclk_gpio)) {
			gpio_set_value((kctrl_data->refclk_gpio), 0);
			gpio_free(kctrl_data->refclk_gpio);
			pr_debug("%s: [DISP]refclk OFF\n", __func__);
		}
		usleep_range(3000, 4000);

		rc = kdisp_mipiic_power_ctrl(kctrl_data, false);
		usleep_range(3000, 4000);

		if (gpio_is_valid(kctrl_data->vmipi_12_en_gpio)) {
			gpio_set_value((kctrl_data->vmipi_12_en_gpio), 0);
			gpio_free(kctrl_data->vmipi_12_en_gpio);
			pr_debug("%s: [DISP]vmipi_12_en OFF\n", __func__);
		}
		usleep_range(3000, 4000);
	}
	return rc;

ic_rst_gpio_err:
	if (gpio_is_valid(kctrl_data->refclk_gpio))
		gpio_free(kctrl_data->refclk_gpio);
refclk_gpio_err:
	if (gpio_is_valid(kctrl_data->vmipi_12_en_gpio))
		gpio_free(kctrl_data->vmipi_12_en_gpio);
vmipi_12_en_gpio_err:
	return rc;
}

int kdisp_panel_init(struct platform_device *pdev, struct device_node *np,
	struct kc_ctrl_info *kctrl_data)
{
	mdss_dsi_parse_dcs_cmds(np, (struct dsi_panel_cmds *)&kctrl_data->on_post_cmds,
		"kc,mdss-dsi-on-post-command", "kc,mdss-dsi-on-post-command-state");

	mdss_dsi_parse_dcs_cmds(np, (struct dsi_panel_cmds *)&kctrl_data->on_post_cmds1,
		"kc,mdss-dsi-on-post-command1", "kc,mdss-dsi-on-post-command-state1");

	mdss_dsi_parse_dcs_cmds(np, (struct dsi_panel_cmds *)&kctrl_data->on_post_cmds2,
		"kc,mdss-dsi-on-post-command2", "kc,mdss-dsi-on-post-command-state2");

	kctrl_data->mipiic_en = of_property_read_bool(np,
		"kc,panel-mipiic-enabled");

	if (kctrl_data->mipiic_en) {
		kctrl_data->vmipi_12_en_gpio = of_get_named_gpio(pdev->dev.of_node,
				 "kc,platform-vmipi_12_en-gpio", 0);
		if (!gpio_is_valid(kctrl_data->vmipi_12_en_gpio))
			pr_err("%s:%d, vmipi_12_en gpio not specified\n",
							__func__, __LINE__);

		kctrl_data->refclk_gpio = of_get_named_gpio(pdev->dev.of_node,
				 "kc,platform-refclk-gpio", 0);
		if (!gpio_is_valid(kctrl_data->refclk_gpio))
			pr_err("%s:%d, refclk gpio not specified\n",
						__func__, __LINE__);

		kctrl_data->ic_rst_gpio = of_get_named_gpio(pdev->dev.of_node,
				 "kc,platform-ic-reset-gpio", 0);
		if (!gpio_is_valid(kctrl_data->ic_rst_gpio))
			pr_err("%s:%d, ic reset gpio not specified\n",
							__func__, __LINE__);

		kdisp_mipiic_power_init(&pdev->dev, kctrl_data);
	}

	return 0;
}

