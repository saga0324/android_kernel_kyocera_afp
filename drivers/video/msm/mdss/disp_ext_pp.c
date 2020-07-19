/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 */
/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>

#include "mdss_dsi.h"
#include "mdp3_ppp.h"
#include "mdp3_hwio.h"
#include "mdp3_ctrl.h"

static void parse_dt_array(const struct device_node *np,
		const char *name, u32 *out, int len)
{
	int num = 0, rc;
	struct property *data;

	data = of_find_property(np, name, &num);
	num /= sizeof(u32);
	if (!data || num != len)
		pr_err("%s:%d, error reading %s, length found = %d\n",
			__func__, __LINE__, name, num);
	else {
		rc = of_property_read_u32_array(np, name, out, num);
		if (rc)
			pr_err("%s:%d, error reading %s, rc = %d\n",
				__func__, __LINE__, name, rc);
	}
}

void disp_ext_parse_pp(struct device_node *np,
			struct panel_pp_info *pp_info)
{
	pr_info("%s: ENTER\n", __func__);

	parse_dt_array(np, "kc,dma-p-csc-mv", pp_info->mv,
				ARRAY_SIZE(pp_info->mv));
	parse_dt_array(np, "kc,dma-p-csc-pre-bv", pp_info->pre_bv,
				ARRAY_SIZE(pp_info->pre_bv));
	parse_dt_array(np, "kc,dma-p-csc-post-bv", pp_info->post_bv,
				ARRAY_SIZE(pp_info->post_bv));
	parse_dt_array(np, "kc,dma-p-csc-pre-lv", pp_info->pre_lv,
				ARRAY_SIZE(pp_info->pre_lv));
	parse_dt_array(np, "kc,dma-p-csc-post-lv", pp_info->post_lv,
				ARRAY_SIZE(pp_info->post_lv));

	pr_info("%s: EXIT\n", __func__);
}

int disp_ext_pp_config(struct mdp3_session_data *session, struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdp_csc_cfg_data data = {0};

	pr_info("%s: ENTER\n", __func__);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	memcpy(&data.csc_data.csc_mv[0], &ctrl_pdata->pp_info.mv[0], sizeof(ctrl_pdata->pp_info.mv));
	memcpy(&data.csc_data.csc_pre_bv[0], &ctrl_pdata->pp_info.pre_bv[0], sizeof(ctrl_pdata->pp_info.pre_bv));
	memcpy(&data.csc_data.csc_post_bv[0], &ctrl_pdata->pp_info.post_bv[0], sizeof(ctrl_pdata->pp_info.post_bv));
	memcpy(&data.csc_data.csc_pre_lv[0], &ctrl_pdata->pp_info.pre_lv[0], sizeof(ctrl_pdata->pp_info.pre_lv));
	memcpy(&data.csc_data.csc_post_lv[0], &ctrl_pdata->pp_info.post_lv[0], sizeof(ctrl_pdata->pp_info.post_lv));

	ret = mdp3_csc_config_ext_pp(session, &data);

	pr_info("%s: EXIT(%d)\n", __func__, ret);
	return 0;
}

void disp_ext_pp_print_regs(struct msm_fb_data_type *mfd)
{
	uint16_t i;
	struct mdp3_session_data *mdp3_session;
	struct mdp_csc_cfg_data *datap;

	pr_notice("%s START\n", __func__);

	mdp3_session = (struct mdp3_session_data *)mfd->mdp.private1;

	if (!mdp3_session) {
		pr_err("%s: mdp3 is not init yet\n", __func__);
		return;
	}

	if (!mdp3_session->status) {
		pr_err("%s: mdp3 is off\n", __func__);
		return;
	}

	if (!mdp3_session->dma) {
		pr_err("%s: mdp3_session->dma is not init yet\n", __func__);
		return;
	}

	datap = &mdp3_session->dma->ccs_cache;

	pr_notice("MDP3_csc_mv\n");
	for (i = 0; i < sizeof(datap->csc_data.csc_mv)/sizeof(u32); i++) {
		pr_notice("0x%04x\n", datap->csc_data.csc_mv[i]);
	}
	pr_notice("\n");

	pr_notice("MDP3_csc_pre_bv\n");
	for (i = 0; i < sizeof(datap->csc_data.csc_pre_bv)/sizeof(u32); i++) {
		pr_notice("0x%04x\n", datap->csc_data.csc_pre_bv[i]);
	}
	pr_notice("\n");

	pr_notice("MDP3_csc_post_bv\n");
	for (i = 0; i < sizeof(datap->csc_data.csc_post_bv)/sizeof(u32); i++) {
		pr_notice("0x%04x\n", datap->csc_data.csc_post_bv[i]);
	}
	pr_notice("\n");

	pr_notice("MDP3_csc_pre_lv\n");
	for (i = 0; i < sizeof(datap->csc_data.csc_pre_lv)/sizeof(u32); i++) {
		pr_notice("0x%04x\n", datap->csc_data.csc_pre_lv[i]);
	}
	pr_notice("\n");

	pr_notice("MDP3_csc_post_lv\n");
	for (i = 0; i < sizeof(datap->csc_data.csc_post_lv)/sizeof(u32); i++) {
		pr_notice("0x%04x\n", datap->csc_data.csc_post_lv[i]);
	}
	pr_notice("\n");

	pr_notice("MDP3_cc_vect_sel:%d\n", mdp3_session->dma->cc_vect_sel);

	pr_notice("%s END\n", __func__);
}
