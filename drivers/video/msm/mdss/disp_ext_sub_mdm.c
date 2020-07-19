/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 * (C) 2017 KYOCERA Corporation
*/
/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <soc/qcom/smd.h>
#include <soc/qcom/subsystem_notif.h>
#include "mdss_fb.h"
#include "disp_ext_sub_ctrl.h"
#include "disp_ext_sub_panel.h"
#include "disp_ext_sub_mdm.h"

#define DISP_EXT_SUB_SMD_RETRY_COUNT     20          /* times */
#define DISP_EXT_SUB_SMD_RETRY_DELAY     100
#define DISP_EXT_SUB_SMD_PORT_NAME       "subdisp_d2s"
#define DISP_EXT_SUB_SMD_PORT_DEBUG_NAME "subdisp_s2d"
#define DISP_EXT_SUB_MDM_WAIT_MODEM_UP   10000
#define DISP_EXT_SUB_MDM_REQ_TIMEOUT     1000
#define DISP_EXT_SUB_MDM_REQ_RETRY_MAX   5

//------------------------------------------------------------------------------
static void disp_ext_sub_smd_notify(void *priv, unsigned event)
{
	struct disp_ext_sub_smd_data *data = (struct disp_ext_sub_smd_data *)priv;
	int len;
	int r_len;

	pr_debug("%s start event:%d\n",__func__,event);
	switch (event)
	{
	case SMD_EVENT_DATA:
		pr_debug("%s SMD_EVENT_DATA\n",__func__);
		for (;;) {
			len = smd_read_avail(data->smd_ch);
			if (len < 0) {
				pr_err("%s smd read failed. ret:%d\n", __func__, len);
				break;
			}
			if (len == 0) {
				pr_debug("no data.\n");
				break;
			}

			r_len = smd_read_from_cb(data->smd_ch, &data->rx, sizeof(data->rx));
			if (r_len != sizeof(data->rx)) {
				pr_err("%s bad size. r_len:%d\n", __func__, r_len);
				break;
			}
			pr_debug("%s smd_read len:%d r_len:%d\n", __func__, len, r_len);
			if (data->rx.request == data->tx.request &&
				data->rx.seq == data->tx.seq) {
				complete(&data->completion);
			} else {
				pr_warn("%s unexpected response. tx seq:%d request:%d, rx seq:%d request:%d\n",
					__func__, data->tx.seq, data->tx.request, data->rx.seq, data->rx.request);
			}
		}
		break;
	case SMD_EVENT_OPEN:
		data->smd_opened = 1;
		pr_debug("%s SMD_EVENT_OPEN\n",__func__);
		break;
	case SMD_EVENT_CLOSE:
		data->smd_opened = 0;
		pr_debug("%s SMD_EVENT_CLOSE\n",__func__);
		break;
	case SMD_EVENT_STATUS:
		pr_debug("%s SMD_EVENT_STATUS\n",__func__);
		break;
	case SMD_EVENT_REOPEN_READY:
		pr_debug("%s SMD_EVENT_REOPEN_READY\n",__func__);
		break;
	default:
		pr_debug("%s default\n",__func__);
		break;
	}
	pr_debug("%s end\n",__func__);
}

int disp_ext_sub_mdm_wait_for_modem_up(struct disp_ext_sub_pdata *pdata)
{
	int rc;

	if (pdata->modem_state != DISP_SUB_MODEM_STATE_ON) {
		pr_notice("%s: wait for modem up\n", __func__);
		rc = wait_for_completion_timeout(&pdata->mdm_notif.completion, msecs_to_jiffies(DISP_EXT_SUB_MDM_WAIT_MODEM_UP));
		if (!rc) {
			pr_err("%s timeout\n",__func__);
			return -ETIMEDOUT;
		}
		pr_notice("%s: modem is up\n", __func__);
	}

	return 0;
}

int disp_ext_sub_smd_open(struct disp_ext_sub_pdata *pdata)
{
	int rc;
	int count;
	struct disp_ext_sub_smd_data *smd_data;
	struct disp_ext_sub_smd_m2l_data *smd_m2l_data;

	pr_debug("%s start\n",__func__);

	smd_data = &pdata->smd_data;
	smd_m2l_data = &pdata->smd_m2l_data;

	if (!smd_data->smd_ch)
	{
		smd_data->smd_opened = 0;
		rc = smd_named_open_on_edge(smd_data->port_name, SMD_APPS_MODEM, &smd_data->smd_ch,
			smd_data, smd_data->cb_func);
		if (rc < 0) {
			pr_err("%s smd open error\n",__func__);
			smd_data->smd_ch = NULL;
			return rc;
		}
		smd_m2l_data->smd_opened = 0;
		rc = smd_named_open_on_edge(smd_m2l_data->port_name, SMD_APPS_MODEM, &smd_m2l_data->smd_ch,
			smd_m2l_data, smd_m2l_data->cb_func);
		if (rc < 0) {
			pr_err("%s smd open error\n",__func__);
			smd_close(smd_data->smd_ch);
			smd_data->smd_ch = NULL;
			return rc;
		}
	}

	if (!smd_data->smd_opened || !smd_m2l_data->smd_opened) {
		for (count = 0; count < DISP_EXT_SUB_SMD_RETRY_COUNT; ++count) {
			if (smd_data->smd_opened && smd_m2l_data->smd_opened)
				break;
			msleep(DISP_EXT_SUB_SMD_RETRY_DELAY);
		}
		if (count >= DISP_EXT_SUB_SMD_RETRY_COUNT) {
			pr_err("%s smd open wait timeout\n",__func__);
			return -ETIMEDOUT;
		}
	}

	pr_debug("%s end\n",__func__);
	return 0;
}

int disp_ext_sub_smd_send_request( struct disp_ext_sub_pdata *pdata, disp_ext_sub_linux2mdm_tx_type *request )
{
	static uint32_t seq = 1;
	int retry = DISP_EXT_SUB_MDM_REQ_RETRY_MAX;
	int rc;
	struct disp_ext_sub_smd_data *smd_data;

	pr_debug("%s start\n",__func__);

	smd_data = &pdata->smd_data;

	mutex_lock(&smd_data->mutex);

	memcpy(&smd_data->tx, request, sizeof(smd_data->tx));
	smd_data->tx.seq = seq++;
	do {
		rc = disp_ext_sub_mdm_wait_for_modem_up(pdata);
		if (rc < 0) {
			pr_warn("%s modem is not work.\n", __func__);
			break;
		}
		rc = disp_ext_sub_smd_open(pdata);
		if (rc < 0) {
			pr_warn("%s smd open fail. %d\n", __func__, rc);
			continue;
		}

		INIT_COMPLETION(smd_data->completion);
		rc = smd_write(smd_data->smd_ch, &smd_data->tx, sizeof(smd_data->tx));
		if (rc < 0) {
			pr_err("%s smd_write error retry:%d\n",__func__, retry);
			msleep(DISP_EXT_SUB_MDM_REQ_TIMEOUT);
			continue;
		}
		rc = wait_for_completion_timeout(&smd_data->completion, msecs_to_jiffies(DISP_EXT_SUB_MDM_REQ_TIMEOUT));
		if (!rc) {
			pr_err("%s timeout retry:%d\n",__func__, retry);
			rc = -ETIMEDOUT;
			continue;
		}
		rc = smd_data->rx.resp.result;
		if (rc != DISP_EXT_SUB_MDMRES_TYPE_SUCCESS) {
			pr_err("%s response NG rc:%d retry:%d\n",__func__, rc, retry);
			msleep(DISP_EXT_SUB_MDM_REQ_TIMEOUT);
			continue;
		}
		break;
	} while(retry--);

	mutex_unlock(&smd_data->mutex);

	pr_debug("%s end\n",__func__);
	return rc;
}

int disp_ext_sub_smd_send_raw_data(struct disp_ext_sub_pdata *pdata, uint8_t *buf, uint32_t len)
{
	int retry = DISP_EXT_SUB_MDM_REQ_RETRY_MAX;
	int rc;
	struct disp_ext_sub_smd_data *smd_data;

	pr_debug("%s start\n",__func__);

	smd_data = &pdata->smd_data;

	mutex_lock(&smd_data->mutex);

	dynamic_hex_dump("send_raw:", DUMP_PREFIX_NONE, 16, 1, buf, len, false);
	memcpy(&smd_data->tx, buf, sizeof(smd_data->tx));
	do {
		rc = disp_ext_sub_mdm_wait_for_modem_up(pdata);
		if (rc < 0) {
			pr_warn("%s modem is not work.\n", __func__);
			break;
		}
		rc = disp_ext_sub_smd_open(pdata);
		if (rc < 0) {
			pr_warn("%s smd open fail. %d\n", __func__, rc);
			continue;
		}

		INIT_COMPLETION(smd_data->completion);
		rc = smd_write(smd_data->smd_ch, buf, len);
		if (rc < 0) {
			pr_err("%s smd_write error retry:%d\n",__func__, retry);
			msleep(DISP_EXT_SUB_MDM_REQ_TIMEOUT);
			continue;
		}
		rc = wait_for_completion_timeout(&smd_data->completion, msecs_to_jiffies(DISP_EXT_SUB_MDM_REQ_TIMEOUT));
		if (!rc) {
			pr_err("%s timeout retry:%d\n",__func__, retry);
			rc = -ETIMEDOUT;
			continue;
		}
		rc = smd_data->rx.resp.result;
		if (rc != DISP_EXT_SUB_MDMRES_TYPE_SUCCESS) {
			pr_err("%s response NG rc:%d retry:%d\n",__func__, rc, retry);
			msleep(DISP_EXT_SUB_MDM_REQ_TIMEOUT);
			continue;
		}
		break;
	} while(retry--);

	mutex_unlock(&smd_data->mutex);

	pr_debug("%s end\n",__func__);
	return rc;
}

int disp_ext_sub_mdm_change_dispctl(bool modem_mode, struct disp_ext_sub_pdata *pdata)
{
	int ret = 0;
	disp_ext_sub_linux2mdm_tx_type  tx;

	pr_debug("%s start\n",__func__);

	ret = disp_ext_sub_mdm_wait_for_modem_up(pdata);
	if (ret < 0) {
		pr_warn("%s modem is not work.\n", __func__);
		if (!modem_mode) {
			ret = 0;
		}
		return ret;
	}

	tx.request = DISP_EXT_SUB_MDMREQ_DISPCTL;
	tx.dispctl.mode = modem_mode ?
		DISP_EXT_SUB_MDMREQ_DISPCRL_TYPE_START : DISP_EXT_SUB_MDMREQ_DISPCRL_TYPE_STOP;
	memcpy(&tx.dispctl.disp_info, &pdata->subdispinfo, sizeof(tx.dispctl.disp_info));

	ret = disp_ext_sub_smd_send_request(pdata, &tx);

	if (!modem_mode) {
		ret = 0;
	}
	pr_debug("%s end %d\n",__func__,ret);
	return ret;
}

int disp_ext_sub_smd_send_command( struct disp_ext_sub_pdata *pdata, int cmd, int param )
{
	int ret = 0;
	disp_ext_sub_linux2mdm_tx_type  tx;

	pr_debug("%s start\n",__func__);

	tx.request = cmd;
	tx.param = param;

	ret = disp_ext_sub_smd_send_request(pdata, &tx);

	pr_debug("%s end %d\n",__func__,ret);
	return ret;
}

static int disp_ext_sub_smd_init( struct disp_ext_sub_pdata *pdata )
{
	struct disp_ext_sub_smd_data* data;

	pr_debug("%s start\n",__func__);

	data = &pdata->smd_data;
	data->port_name = DISP_EXT_SUB_SMD_PORT_NAME;
	data->smd_opened = 0;
	data->cb_func = disp_ext_sub_smd_notify;
	mutex_init(&data->mutex);
	init_completion(&data->completion);

	pr_debug("%s end\n",__func__);

	return 0;
}

//------------------------------------------------------------------------------
static void disp_ext_sub_mdm_notify_work_func(struct work_struct *work)
{
	struct disp_ext_sub_pdata *pdata =
		container_of(work, struct disp_ext_sub_pdata, mdm_notif.work);
	struct disp_ext_sub_info *sub_info_p =
		container_of(pdata, struct disp_ext_sub_info, pdata);
	struct fb_info * fbi = sub_info_p->fbi;

	pr_debug("%s start\n",__func__);

	lock_fb_info(fbi);
	if (pdata->modem_state == DISP_SUB_MODEM_STATE_ON &&
		pdata->state == DISP_SUB_STATE_ALLWAYSON) {
		pr_notice("%s disp_ext_sub_mdm_change_dispctl\n",__func__);
		disp_ext_sub_mdm_change_dispctl(true, pdata);
	}
	unlock_fb_info(fbi);

	pr_debug("%s end\n",__func__);
}

static void disp_ext_sub_mdm_set_state(struct disp_ext_sub_pdata *pdata,
	unsigned long code)
{
	pr_debug("%s start modem_state:%d code:%ld\n",__func__, pdata->modem_state, code);

	switch (pdata->modem_state) {
	case DISP_SUB_MODEM_STATE_ON:
		switch (code) {
		case SUBSYS_BEFORE_SHUTDOWN:
		case SUBSYS_AFTER_SHUTDOWN:
		case SUBSYS_BEFORE_POWERUP:
		case SUBSYS_POWERUP_FAILURE:
		case SUBSYS_SOC_RESET:
			pr_notice("%s: change modem state off\n", __func__);
			INIT_COMPLETION(pdata->mdm_notif.completion);
			pdata->modem_state = DISP_SUB_MODEM_STATE_OFF;
			break;
		case SUBSYS_AFTER_POWERUP:
		default:
			break;
		}
		break;
	case DISP_SUB_MODEM_STATE_OFF:
		switch (code) {
		case SUBSYS_AFTER_POWERUP:
			pr_notice("%s: change modem state on\n", __func__);
			pdata->modem_state = DISP_SUB_MODEM_STATE_ON;
			complete_all(&pdata->mdm_notif.completion);
			schedule_work(&pdata->mdm_notif.work);
			break;
		case SUBSYS_BEFORE_SHUTDOWN:
		case SUBSYS_AFTER_SHUTDOWN:
		case SUBSYS_BEFORE_POWERUP:
		case SUBSYS_POWERUP_FAILURE:
		case SUBSYS_SOC_RESET:
		default:
			break;
		}
		break;
	default:
		break;
	}

	pr_debug("%s end\n",__func__);
}

static int disp_ext_sub_mdm_change_state_notify(struct notifier_block *this,
	unsigned long code, void *ss_handle)
{
	struct disp_ext_sub_pdata *pdata =
		container_of(this, struct disp_ext_sub_pdata, mdm_notif.nb);

	pr_debug("%s start\n",__func__);

	disp_ext_sub_mdm_set_state(pdata, code);

	pr_debug("%s end\n",__func__);
	return NOTIFY_DONE;
}
//------------------------------------------------------------------------------

static void disp_ext_sub_smd_receive_request(struct work_struct *work)
{
	struct disp_ext_sub_smd_m2l_data *data =
		container_of(work, struct disp_ext_sub_smd_m2l_data, work);
	struct disp_ext_sub_pdata *pdata =
		container_of(work, struct disp_ext_sub_pdata, smd_m2l_data.work);
	struct disp_ext_sub_info *sub_info_p =
		container_of(pdata, struct disp_ext_sub_info, pdata);
	struct fb_info * fbi = sub_info_p->fbi;
	int len;
	int r_len;

	pr_debug("%s start\n",__func__);

	for (;;) {
		len = smd_read_avail(data->smd_ch);
		if (len < 0) {
			pr_err("%s smd read failed. ret:%d\n", __func__, len);
			break;
		}
		if (len == 0) {
			pr_debug("no data.\n");
			break;
		}

		r_len = smd_read(data->smd_ch, &data->rx, sizeof(data->rx));
		if (r_len != sizeof(data->rx)) {
			pr_err("%s bad size. r_len:%d\n", __func__, r_len);
			break;
		}
		pr_debug("%s smd_read len:%d r_len:%d\n", __func__, len, r_len);

		switch (data->rx.request) {
		case DISP_EXT_SUB_MDMREQ_DEBUG_DISPCTL_RESET:
			pr_debug("%s rcv DISP_EXT_SUB_MDMREQ_DEBUG_DISPCTL_RESET\n", __func__);
			lock_fb_info(fbi);
			if (pdata->modem_state == DISP_SUB_MODEM_STATE_ON &&
				pdata->state == DISP_SUB_STATE_ALLWAYSON) {
				pr_notice("%s disp_ext_sub_mdm_change_dispctl false\n",__func__);
				disp_ext_sub_mdm_change_dispctl(false, pdata);
				pr_notice("%s disp_ext_sub_mdm_change_dispctl true\n",__func__);
				disp_ext_sub_mdm_change_dispctl(true, pdata);
			}
			unlock_fb_info(fbi);
			break;
		default:
			pr_warn("%s unknown request. request:%d\n",
				__func__, data->tx.request);
			break;
		}
	}

	pr_debug("%s end\n",__func__);
}

static void disp_ext_sub_smd_m2l_notify(void *priv, unsigned event)
{
	struct disp_ext_sub_smd_m2l_data *data = (struct disp_ext_sub_smd_m2l_data *)priv;
	int len;

	pr_debug("%s start event:%d\n",__func__,event);
	switch (event)
	{
	case SMD_EVENT_DATA:
		pr_debug("%s SMD_EVENT_DATA\n",__func__);
		len = smd_read_avail(data->smd_ch);
		if (len < 0) {
			pr_err("%s smd read failed. ret:%d\n", __func__, len);
			break;
		}
		if (len == 0) {
			pr_debug("no data.\n");
			break;
		}

		schedule_work(&data->work);
		break;
	case SMD_EVENT_OPEN:
		data->smd_opened = 1;
		pr_debug("%s SMD_EVENT_OPEN\n",__func__);
		break;
	case SMD_EVENT_CLOSE:
		data->smd_opened = 0;
		pr_debug("%s SMD_EVENT_CLOSE\n",__func__);
		break;
	case SMD_EVENT_STATUS:
		pr_debug("%s SMD_EVENT_STATUS\n",__func__);
		break;
	case SMD_EVENT_REOPEN_READY:
		pr_debug("%s SMD_EVENT_REOPEN_READY\n",__func__);
		break;
	default:
		pr_debug("%s default\n",__func__);
		break;
	}
	pr_debug("%s end\n",__func__);
}

static int disp_ext_sub_smd_m2l_init( struct disp_ext_sub_pdata *pdata )
{
	struct disp_ext_sub_smd_m2l_data* data;

	pr_debug("%s start\n",__func__);

	data = &pdata->smd_m2l_data;
	data->port_name = DISP_EXT_SUB_SMD_PORT_DEBUG_NAME;
	data->smd_opened = 0;
	data->cb_func = disp_ext_sub_smd_m2l_notify;
	INIT_WORK(&data->work, disp_ext_sub_smd_receive_request);

	pr_debug("%s end\n",__func__);

	return 0;
}
//------------------------------------------------------------------------------

int disp_ext_sub_mdm_init(struct disp_ext_sub_pdata *pdata)
{
	void *handle;
	int ret;

	pr_debug("%s start\n",__func__);

	ret = disp_ext_sub_smd_init(pdata);
	if (ret < 0) {
		pr_err("%s: disp_ext_sub_smd_init failed: %d\n", __func__, ret);
		return ret;
	}

	ret = disp_ext_sub_smd_m2l_init(pdata);
	if (ret < 0) {
		pr_err("%s: disp_ext_sub_smd_m2l_init failed: %d\n", __func__, ret);
		return ret;
	}

	pdata->modem_state = DISP_SUB_MODEM_STATE_OFF;
	pdata->mdm_notif.nb.notifier_call =
		disp_ext_sub_mdm_change_state_notify;
	INIT_WORK(&pdata->mdm_notif.work, disp_ext_sub_mdm_notify_work_func);
	init_completion(&pdata->mdm_notif.completion);

	handle = subsys_notif_register_notifier("modem",
		&pdata->mdm_notif.nb);
	if (IS_ERR_OR_NULL(handle)) {
		ret = PTR_ERR(handle);
		pr_err("%s: subsys_notif_register_notifier failed: %d\n", __func__, ret);
		return ret;
	}

	pr_debug("%s end\n",__func__);

	return 0;
}

static int disp_ext_sub_diag_modem_command(struct fb_info *info, void __user *p)
{
	struct disp_ext_sub_info *sub_info_p = (struct disp_ext_sub_info*)(info->par);
	struct disp_ext_sub_pdata *pdata = &(sub_info_p->pdata);
	uint32_t len;
	uint8_t buf[128] = {0};
	int ret;

	ret = copy_from_user(&len, p, sizeof(len));
	if (ret)
		return ret;

	ret = copy_from_user(buf, ((char*)p) + sizeof(len), min(sizeof(buf), len));
	if (ret)
		return ret;

	ret = disp_ext_sub_smd_send_raw_data(pdata, buf, min(sizeof(buf), len));
	return ret;
}

#ifdef CONFIG_DISP_EXT_SUB_CONTRAST
static int disp_ext_sub_set_diag_contrast(struct fb_info *info, void __user *p)
{
	struct disp_ext_sub_info *sub_info_p = (struct disp_ext_sub_info*)(info->par);
	struct disp_ext_sub_pdata *pdata = &(sub_info_p->pdata);
	int param;
	int ret;

	ret = copy_from_user(&param, p, sizeof(param));
	if (ret)
		return ret;

	disp_ext_sub_set_user_contrast(pdata, param, true);
	
	return 0;
}
#endif /* CONFIG_DISP_EXT_SUB_CONTRAST */


int disp_ext_sub_modem_ioctl(
	struct fb_info *info,
	unsigned int cmd,
	unsigned long arg )
{

	int ret = 0;
	void __user *argp = (void __user *)arg;

	pr_err("%s start\n",__func__);

	switch (cmd) {
	case KC_DISP_EXT_SUB_MODEM_CMD:
		if (!lock_fb_info(info))
			return -ENODEV;
		ret = disp_ext_sub_diag_modem_command(info, argp);
		unlock_fb_info(info);
		break;
	case KC_DISP_EXT_SUB_SET_CONTRAST:
#ifdef CONFIG_DISP_EXT_SUB_CONTRAST
		if (!lock_fb_info(info))
			return -ENODEV;
		ret = disp_ext_sub_set_diag_contrast(info, argp);
		unlock_fb_info(info);
#endif /* CONFIG_DISP_EXT_SUB_CONTRAST */
		break;
	default:
		break;
	}

	pr_debug("%s end %d\n",__func__,ret);
	return ret;
}

ssize_t disp_ext_sub_sysfs_mdm_stat(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct disp_ext_sub_info *sub_info_p = (struct disp_ext_sub_info*)(fbi->par);
	struct disp_ext_sub_pdata *pdata = &(sub_info_p->pdata);
	ssize_t ret;

	ret = sprintf(buf, "%d\n", pdata->modem_state);

	return ret;
}

ssize_t disp_ext_sub_sysfs_mdm_svcdump(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct disp_ext_sub_info *sub_info_p = (struct disp_ext_sub_info*)(fbi->par);
	struct disp_ext_sub_pdata *pdata = &(sub_info_p->pdata);

	if (!lock_fb_info(fbi))
		return count;
	disp_ext_sub_smd_send_command(pdata, DISP_EXT_SUB_MDMREQ_SVCINFO, 0);
	unlock_fb_info(fbi);

	return count;
}

ssize_t disp_ext_sub_sysfs_mdm_drvdump(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct disp_ext_sub_info *sub_info_p = (struct disp_ext_sub_info*)(fbi->par);
	struct disp_ext_sub_pdata *pdata = &(sub_info_p->pdata);

	if (!lock_fb_info(fbi))
		return count;
	disp_ext_sub_smd_send_command(pdata, DISP_EXT_SUB_MDMREQ_DRVINFO, 0);
	unlock_fb_info(fbi);

	return count;
}

ssize_t disp_ext_sub_sysfs_mdm_svcdbglvl(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct disp_ext_sub_info *sub_info_p = (struct disp_ext_sub_info*)(fbi->par);
	struct disp_ext_sub_pdata *pdata = &(sub_info_p->pdata);
	uint32_t param = 0;

	sscanf(buf, "%x", &param);

	if (!lock_fb_info(fbi))
		return count;
	disp_ext_sub_smd_send_command(pdata, DISP_EXT_SUB_MDMREQ_SET_SVCDBGLVL, param);
	unlock_fb_info(fbi);

	return count;
}

ssize_t disp_ext_sub_sysfs_mdm_drvdbglvl(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct disp_ext_sub_info *sub_info_p = (struct disp_ext_sub_info*)(fbi->par);
	struct disp_ext_sub_pdata *pdata = &(sub_info_p->pdata);
	uint32_t param = 0;

	sscanf(buf, "%x", &param);

	if (!lock_fb_info(fbi))
		return count;
	disp_ext_sub_smd_send_command(pdata, DISP_EXT_SUB_MDMREQ_SET_DRVDBGLVL, param);
	unlock_fb_info(fbi);

	return count;
}

ssize_t disp_ext_sub_sysfs_mdm_timehist(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct disp_ext_sub_info *sub_info_p = (struct disp_ext_sub_info*)(fbi->par);
	struct disp_ext_sub_pdata *pdata = &(sub_info_p->pdata);

	if (!lock_fb_info(fbi))
		return count;
	disp_ext_sub_smd_send_command(pdata, DISP_EXT_SUB_MDMREQ_DUMP_TIMEHIST, 0);
	unlock_fb_info(fbi);

	return count;
}

ssize_t disp_ext_sub_sysfs_mdm_svcimage(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct disp_ext_sub_info *sub_info_p = (struct disp_ext_sub_info*)(fbi->par);
	struct disp_ext_sub_pdata *pdata = &(sub_info_p->pdata);

	if (!lock_fb_info(fbi))
		return count;
	disp_ext_sub_smd_send_command(pdata, DISP_EXT_SUB_MDMREQ_DUMP_SVCIMAGE, 0);
	unlock_fb_info(fbi);

	return count;
}

ssize_t disp_ext_sub_sysfs_mdm_drvimage(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct disp_ext_sub_info *sub_info_p = (struct disp_ext_sub_info*)(fbi->par);
	struct disp_ext_sub_pdata *pdata = &(sub_info_p->pdata);

	if (!lock_fb_info(fbi))
		return count;
	disp_ext_sub_smd_send_command(pdata, DISP_EXT_SUB_MDMREQ_DUMP_DRVIMAGE, 0);
	unlock_fb_info(fbi);

	return count;
}

ssize_t disp_ext_sub_sysfs_mdm_timeradj(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct disp_ext_sub_info *sub_info_p = (struct disp_ext_sub_info*)(fbi->par);
	struct disp_ext_sub_pdata *pdata = &(sub_info_p->pdata);
	uint32_t param = 0;

	sscanf(buf, "%u", &param);

	if (!lock_fb_info(fbi))
		return count;
	disp_ext_sub_smd_send_command(pdata, DISP_EXT_SUB_MDMREQ_SET_TIMERADJ, param);
	unlock_fb_info(fbi);

	return count;
}

