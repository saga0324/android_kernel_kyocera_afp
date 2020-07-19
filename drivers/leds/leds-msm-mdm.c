/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
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
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <soc/qcom/smd.h>

#include "leds-msm-mdm.h"

//#define MDM_LED_DEBUG

#ifdef MDM_LED_DEBUG
#define MDM_LED_DEBUG_LOG( msg, ... ) \
pr_notice("[MDM_LED][%s][D](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#else
#define MDM_LED_DEBUG_LOG( msg, ... ) \
pr_debug("[MDM_LED][%s][D](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#endif

#define MDM_LED_NOTICE_LOG( msg, ... ) \
pr_notice("[MDM_LED][%s][N](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

#define MDM_LED_ERR_LOG( msg, ... ) \
pr_err("[MDM_LED][%s][E](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

#define MDM_LED_WARN_LOG( msg, ... ) \
pr_warn("[MDM_LED][%s][W](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)


#define MDM_LED_SMD_SURVIVAL_COUNT     20          /* times */
#define MDM_LED_SMD_RETRY_COUNT        20          /* times */
#define MDM_LED_SMD_RETRY_DELAY        10          /* ms */
#define MDM_LED_SMD_OPEN_RETRY_DELAY   2000        /* ms */
#define MDM_LED_SMD_PORT_NAME          "apr_ledex_svc"   /* port name */

struct mdm_led_data {
	struct mdm_led          mdm_led;
	struct delayed_work     dwork;
	struct mutex            lock;

	int                     request_updated;
	struct tricolor_led_request   request;

	const char*             port_name;
	struct smd_channel*     smd_ch;
	bool                    smd_opened;
	int                     smd_counter;
};

static bool mdm_led_get_request(struct mdm_led_data* data, struct tricolor_led_request *dst)
{
	bool updated;

	MDM_LED_DEBUG_LOG("[IN]");

	mutex_lock(&data->lock);
	MDM_LED_DEBUG_LOG("mutex_lock");

	updated = data->request_updated;
	memcpy(dst, &data->request, sizeof(*dst));
	data->request_updated = false;

	MDM_LED_DEBUG_LOG("mutex_unlock");
	mutex_unlock(&data->lock);

	MDM_LED_DEBUG_LOG("[OUT]");

	return updated;
}

static void mdm_led_set_request(struct mdm_led_data *data,
	const struct tricolor_led_request *request)
{
	MDM_LED_DEBUG_LOG("[IN]");

	mutex_lock(&data->lock);
	MDM_LED_DEBUG_LOG("mutex_lock");

	memcpy(&data->request, request, sizeof(data->request));
	data->request_updated = true;

	MDM_LED_DEBUG_LOG("req color=[0x%08x] mode=[%d] on_time=[%d] off_time=[%d] off_color=[0x%08x]",
		data->request.color, data->request.mode,
		data->request.on_time, data->request.off_time, data->request.off_color);

	MDM_LED_DEBUG_LOG("mutex_unlock");
	mutex_unlock(&data->lock);

	MDM_LED_DEBUG_LOG("[OUT]");
}

static void mdm_led_control_modem_notify(void *priv, unsigned event)
{
	struct mdm_led_data *data = (struct mdm_led_data *)priv;

	MDM_LED_DEBUG_LOG("[IN] event = %u", event);
	switch (event)
	{
	case SMD_EVENT_DATA:
		MDM_LED_DEBUG_LOG("SMD_EVENT_DATA");
		break;
	case SMD_EVENT_OPEN:
		MDM_LED_DEBUG_LOG("SMD_EVENT_OPEN");
		data->smd_opened = true;
		data->smd_counter = 0;
		break;
	case SMD_EVENT_CLOSE:
		MDM_LED_DEBUG_LOG("SMD_EVENT_CLOSE");
		break;
	case SMD_EVENT_STATUS:
		MDM_LED_DEBUG_LOG("SMD_EVENT_STATUS");
		break;
	case SMD_EVENT_REOPEN_READY:
		MDM_LED_DEBUG_LOG("SMD_EVENT_REOPEN_READY");
		break;
	default:
		MDM_LED_DEBUG_LOG("unknown");
		break;
	}
	MDM_LED_DEBUG_LOG("[OUT]");
}

static void mdm_led_send_request_to_modem(struct mdm_led_data *data)
{
	int rc;
	int count;
	struct tricolor_led_request request;

	MDM_LED_DEBUG_LOG("[IN]");

	if (!data->smd_opened || !data->smd_ch)
	{
		data->smd_opened = false;
		rc = smd_named_open_on_edge(data->port_name, SMD_APPS_MODEM, &data->smd_ch, data, mdm_led_control_modem_notify);
		if (rc < 0)
		{
			data->smd_ch = NULL;
			++data->smd_counter;
			if (data->smd_counter < MDM_LED_SMD_SURVIVAL_COUNT)
				MDM_LED_NOTICE_LOG("[LED] Now waiting until LED process on modem is to up. (retry after %d ms)\n", MDM_LED_SMD_OPEN_RETRY_DELAY);
			else
				MDM_LED_WARN_LOG("[LED] LED process on modem is not up (Maybe Modem is dead.)\n");
		}
		else
		{
			for (count = 0; count < MDM_LED_SMD_RETRY_COUNT; ++count)
			{
				if (data->smd_opened)
					break;
				mdelay(MDM_LED_SMD_RETRY_DELAY);
			}
			if (count >= MDM_LED_SMD_RETRY_COUNT)
			{
				smd_close(data->smd_ch);
				data->smd_ch = NULL;
				MDM_LED_WARN_LOG("smd_open() internal error (retry after %d ms)\n", MDM_LED_SMD_OPEN_RETRY_DELAY);
			}
		}

		if (!data->smd_ch)
		{
			MDM_LED_DEBUG_LOG("set retry after %d ms", MDM_LED_SMD_OPEN_RETRY_DELAY);

			schedule_delayed_work(&data->dwork, msecs_to_jiffies(MDM_LED_SMD_OPEN_RETRY_DELAY));
			return;
		}
	}

	if (mdm_led_get_request(data, &request)) {
		MDM_LED_DEBUG_LOG("req color=[0x%08x] mode=[%d] on_time=[%d] off_time=[%d] off_color=[0x%08x]",
			request.color, request.mode,
			request.on_time, request.off_time, request.off_color);
		rc = smd_write(data->smd_ch, &request, sizeof(request));
		MDM_LED_DEBUG_LOG("SMD write %d bytes, rc=%d", sizeof(request), rc);
		msleep(20);
	}
	MDM_LED_DEBUG_LOG("[OUT]");
}

static void mdm_led_work(struct work_struct *work)
{
	struct mdm_led_data *data = container_of(work, struct mdm_led_data, dwork.work);

	MDM_LED_DEBUG_LOG("[IN]");

	mdm_led_send_request_to_modem(data);

	MDM_LED_DEBUG_LOG("[OUT]");
}

static int mdm_led_send_request(struct mdm_led *mdm_led,
	const struct tricolor_led_request *request)
{
	struct mdm_led_data* data = container_of(mdm_led, struct mdm_led_data, mdm_led);

	MDM_LED_DEBUG_LOG("[IN]");

	mdm_led_set_request(data, request);
	schedule_delayed_work(&data->dwork, 0);

	MDM_LED_DEBUG_LOG("[OUT]");

	return 0;
}

struct mdm_led *mdm_led_create(void)
{
	struct mdm_led_data* data;

	MDM_LED_DEBUG_LOG("[IN]");

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		MDM_LED_ERR_LOG("failed to kzalloc.");
		return NULL;
	}

	data->mdm_led.send_request_fn = mdm_led_send_request;
	data->port_name = MDM_LED_SMD_PORT_NAME;
	mutex_init(&data->lock);
	INIT_DELAYED_WORK(&data->dwork, mdm_led_work);

	MDM_LED_DEBUG_LOG("[OUT]");

	return &data->mdm_led;
}

void mdm_led_destroy(struct mdm_led *mdm_led)
{
	struct mdm_led_data* data = container_of(mdm_led,
		struct mdm_led_data, mdm_led);
	MDM_LED_DEBUG_LOG("[IN]");

	if (mdm_led && data) {
		cancel_delayed_work_sync(&data->dwork);
		if (data->smd_ch)
			smd_close(data->smd_ch);
		kfree(data);
	}

	MDM_LED_DEBUG_LOG("[OUT]");
}
