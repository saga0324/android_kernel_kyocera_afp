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
/*
 * Copyright (C) 2007 8D Technologies inc.
 * Raphael Assenat <raph@8d.com>
 * Copyright (C) 2008 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>

#include "leds.h"
#include "leds-msm-mdm.h"
#include "leds-tricolor.h"

#define TRICOLOR_DEBUG			0

#if TRICOLOR_DEBUG
#define TRICOLOR_DEBUG_LOG( msg, ... ) \
pr_notice("[TRICOLOR][%s][D](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#else
#define TRICOLOR_DEBUG_LOG( msg, ... ) \
pr_debug("[TRICOLOR][%s][D](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#endif

#define TRICOLOR_NOTICE_LOG( msg, ... ) \
pr_notice("[TRICOLOR][%s][N](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

#define TRICOLOR_ERR_LOG( msg, ... ) \
pr_err("[TRICOLOR][%s][E](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

#define TRICOLOR_WARN_LOG( msg, ... ) \
pr_warn("[TRICOLOR][%s][W](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

#define TRICOLOR_DRV_NAME		"leds-tricolor"

#define LEDLIGHT_BLINK_NUM		4
#define LEDLIGHT			'L'
#define LEDLIGHT_SET_BLINK		_IOW(LEDLIGHT, 0, T_LEDLIGHT_IOCTL)
#define GPIO_HIGH_VAL			1
#define GPIO_LOW_VAL			0
typedef struct _t_ledlight_ioctl {
	uint32_t data[LEDLIGHT_BLINK_NUM];
}T_LEDLIGHT_IOCTL;

struct tricolor_led_data {
	bool				initialized;
	struct led_classdev		cdev;
	struct miscdevice		mdev;

	int				gpio_red;
	int				gpio_green;
	int				gpio_blue;

	struct mutex			request_lock;
	struct tricolor_led_request	request;

	struct delayed_work		blink_work;
	struct tricolor_led_request	request_for_blink;
	bool				blink_state;

	struct mdm_led			*mdm_led;
	struct wake_lock		wake_lock;
};

static void tricolor_led_get_request(struct tricolor_led_data *data,
	struct tricolor_led_request *dst)
{
	TRICOLOR_DEBUG_LOG("[IN]");

	mutex_lock(&data->request_lock);
	TRICOLOR_DEBUG_LOG("mutex_lock");

	memcpy(dst, &data->request, sizeof(*dst));

	TRICOLOR_DEBUG_LOG("mutex_unlock");
	mutex_unlock(&data->request_lock);

	TRICOLOR_DEBUG_LOG("[OUT]");
}

static void tricolor_led_set_request(struct tricolor_led_data *data, T_LEDLIGHT_IOCTL *st_ioctl)
{
	TRICOLOR_DEBUG_LOG("[IN]");

	mutex_lock(&data->request_lock);
	TRICOLOR_DEBUG_LOG("mutex_lock");

	data->request.color     = TRICOLOR_RGB_OFF;
	data->request.mode      = st_ioctl->data[0];
	data->request.on_time   = st_ioctl->data[1];
	data->request.off_time  = st_ioctl->data[2];
	data->request.off_color = st_ioctl->data[3];

	TRICOLOR_DEBUG_LOG("prep mode=[%d] on_time=[%d] off_time=[%d] off_color=[0x%08x]",
		data->request.mode, data->request.on_time, data->request.off_time, data->request.off_color);

	TRICOLOR_DEBUG_LOG("mutex_unlock");
	mutex_unlock(&data->request_lock);

	TRICOLOR_DEBUG_LOG("[OUT]");
}

static void tricolor_led_set_rgb_brightness(struct tricolor_led_data *data, uint32_t color)
{
	int value = 0;
	TRICOLOR_DEBUG_LOG("[IN]");

	if (gpio_is_valid(data->gpio_red)) {
		value = TRICOLOR_RGB_GET_R(color) ? GPIO_HIGH_VAL : GPIO_LOW_VAL;
		TRICOLOR_DEBUG_LOG("Red %s", (value == GPIO_HIGH_VAL) ? "On": "Off");
		gpio_direction_output(data->gpio_red, value);
	}
	if (gpio_is_valid(data->gpio_green)) {
		value = TRICOLOR_RGB_GET_G(color) ? GPIO_HIGH_VAL : GPIO_LOW_VAL;
		TRICOLOR_DEBUG_LOG("Green %s", (value == GPIO_HIGH_VAL) ? "On": "Off");
		gpio_direction_output(data->gpio_green, value);
	}
	if (gpio_is_valid(data->gpio_blue)) {
		value = TRICOLOR_RGB_GET_B(color) ? GPIO_HIGH_VAL : GPIO_LOW_VAL;
		TRICOLOR_DEBUG_LOG("Blue %s", (value == GPIO_HIGH_VAL) ? "On": "Off");
		gpio_direction_output(data->gpio_blue, value);
	}

	TRICOLOR_DEBUG_LOG("[OUT]");
}

static void tricolor_led_blink_work(struct work_struct *work)
{
	struct tricolor_led_data *data = container_of(work, struct tricolor_led_data, blink_work.work);
	struct tricolor_led_request *request = &data->request_for_blink;
	uint32_t delay_ms;
	TRICOLOR_DEBUG_LOG("[IN]");

	data->blink_state = !data->blink_state;
	if (data->blink_state) {
		tricolor_led_set_rgb_brightness(data, request->color);
	}
	else if (request->off_color & TRICOLOR_RGB_MASK) {
		tricolor_led_set_rgb_brightness(data, request->off_color);
	}
	else {
		tricolor_led_set_rgb_brightness(data, TRICOLOR_RGB_OFF);
	}
	delay_ms = data->blink_state ?
		request->on_time : request->off_time;
	schedule_delayed_work(&data->blink_work, msecs_to_jiffies(delay_ms));

	TRICOLOR_DEBUG_LOG("[OUT]");
}

static void tricolor_led_brightness_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	struct tricolor_led_data *data;
	struct tricolor_led_request request;

	TRICOLOR_DEBUG_LOG("[IN] value=[0x%08x]", value);

	if (led_cdev == NULL) {
		TRICOLOR_WARN_LOG("led_cdev is NULL");
		return;
	}
	data = container_of(led_cdev, struct tricolor_led_data, cdev);

	cancel_delayed_work_sync(&data->blink_work);
	wake_unlock(&data->wake_lock);

	tricolor_led_get_request(data, &request);
	request.color = value;

	if (TRICOLOR_RGB_GET_R(request.color)) {
		if (!gpio_is_valid(data->gpio_red)) {
			TRICOLOR_ERR_LOG("discard RED, not supported");
			request.color &= ~(TRICOLOR_RGB_COLOR_RED);
		}
	}
	if (TRICOLOR_RGB_GET_G(request.color)) {
		if (!gpio_is_valid(data->gpio_green)) {
			TRICOLOR_ERR_LOG("discard GREEN, not supported");
			request.color &= ~(TRICOLOR_RGB_COLOR_GREEN);
			if (gpio_is_valid(data->gpio_blue)) {
				request.color |= TRICOLOR_RGB_COLOR_BLUE;
			}
		}
	}
	if (TRICOLOR_RGB_GET_B(request.color)) {
		if (!gpio_is_valid(data->gpio_blue)) {
			TRICOLOR_ERR_LOG("discard BLUE, not supported");
			request.color &= ~(TRICOLOR_RGB_COLOR_BLUE);
			if (gpio_is_valid(data->gpio_green)) {
				request.color |= TRICOLOR_RGB_COLOR_GREEN;
			}
		}
	}

	if (data->mdm_led &&
		data->mdm_led->confirm_open_fn &&
		data->mdm_led->send_request_fn &&
		data->mdm_led->confirm_open_fn(data->mdm_led)) {
		data->mdm_led->send_request_fn(data->mdm_led, &request);
	}
	else {
		if (request.mode != TRICOLOR_BLINK_REQUEST ||
			(request.color & TRICOLOR_RGB_MASK) == TRICOLOR_RGB_OFF) {
			tricolor_led_set_rgb_brightness(data, request.color);
		} else {
			wake_lock(&data->wake_lock);
			memcpy(&data->request_for_blink, &request, sizeof(data->request_for_blink));
			data->blink_state = false;
			tricolor_led_blink_work(&data->blink_work.work);
		}
	}

	TRICOLOR_DEBUG_LOG("[OUT]");
}

static long tricolor_leds_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int32_t ret = -1;
	T_LEDLIGHT_IOCTL st_ioctl;
	struct tricolor_led_data *data = container_of(filp->private_data,
		struct tricolor_led_data, mdev);

	TRICOLOR_DEBUG_LOG("[IN]");

	if (!filp->private_data || !data) {
		TRICOLOR_WARN_LOG("filp->private_data or data == NULL.");
		return -ENODEV;
	}

	switch (cmd) {
	case LEDLIGHT_SET_BLINK:
		TRICOLOR_DEBUG_LOG("LEDLIGHT_SET_BLINK");
		ret = copy_from_user(&st_ioctl,
			argp,
			sizeof(T_LEDLIGHT_IOCTL));
		if (ret) {
			TRICOLOR_WARN_LOG("Error leds_ioctl(cmd = LEDLIGHT_SET_BLINK)");
			return -EFAULT;
		}
		TRICOLOR_DEBUG_LOG("st_ioctl data[0]=[%d] data[1]=[%d] data[2]=[%d] data[3]=[0x%08x]",
			st_ioctl.data[0], st_ioctl.data[1], st_ioctl.data[2], st_ioctl.data[3]);

		tricolor_led_set_request(data, &st_ioctl);
		break;
	default:
		TRICOLOR_WARN_LOG("default");
		return -EINVAL;
	}

	TRICOLOR_DEBUG_LOG("[OUT]");

	return 0;
}

static struct file_operations tricolor_leds_fops = {
	.owner          = THIS_MODULE,
	.open           = simple_open,
	.unlocked_ioctl = tricolor_leds_ioctl,
	.compat_ioctl   = tricolor_leds_ioctl,
};

static int tricolor_led_create(struct device_node *node,
	struct tricolor_led_data *data, struct device *parent)
{
	int ret;
	const char *linux_name;
	const char *device_name;
	int err = 0;
	int gpio;

	TRICOLOR_DEBUG_LOG("[IN]");

	ret = of_property_read_string(node, "linux,name", &linux_name);
	if (ret) {
		TRICOLOR_ERR_LOG("failed to read linux,name");
		goto fail_read_properties;
	}
	ret = of_property_read_string(node, "device-name", &device_name);
	if (ret) {
		TRICOLOR_ERR_LOG("failed to read device-name");
		goto fail_read_properties;
	}

	wake_lock_init(&data->wake_lock, WAKE_LOCK_SUSPEND, TRICOLOR_DRV_NAME);
	mutex_init(&data->request_lock);
	INIT_DELAYED_WORK(&data->blink_work, tricolor_led_blink_work);

	data->cdev.name = linux_name;
	data->cdev.max_brightness = TRICOLOR_RGB_MAX_BRIGHT_VAL;
	data->cdev.brightness_set = tricolor_led_brightness_set;
	data->cdev.brightness = TRICOLOR_RGB_OFF;

	ret = led_classdev_register(parent, &data->cdev);
	if (ret) {
		TRICOLOR_ERR_LOG("failed to led_classdev_register");
		goto fail_led_classdev_register;
	}

	data->mdev.minor = MISC_DYNAMIC_MINOR;
	data->mdev.name = device_name;
	data->mdev.fops = &tricolor_leds_fops;
	ret = misc_register(&data->mdev);
	if (ret) {
		TRICOLOR_ERR_LOG("failed to misc_register");
		goto fail_misc_register;
	}

	data->gpio_red = -ENOSYS;
	gpio = of_get_named_gpio(node, "kc,rled-gpio", 0);
	if (gpio_is_valid(gpio)) {
		err = gpio_request(gpio, "tricolor-leds");
		if (err < 0) {
			TRICOLOR_ERR_LOG("failed to request GPIO=%d, ret=%d", gpio, err);
		} else {
			data->gpio_red = gpio;
		}
	}

	data->gpio_green = -ENOSYS;
	gpio = of_get_named_gpio(node, "kc,gled-gpio", 0);
	if (gpio_is_valid(gpio)) {
		err = gpio_request(gpio, "tricolor-leds");
		if (err < 0) {
			TRICOLOR_ERR_LOG("failed to request GPIO=%d, ret=%d", gpio, err);
		} else {
			data->gpio_green = gpio;
		}
	}

	data->gpio_blue = -ENOSYS;
	gpio = of_get_named_gpio(node, "kc,bled-gpio", 0);
	if (gpio_is_valid(gpio)) {
		err = gpio_request(gpio, "tricolor-leds");
		if (err < 0) {
			TRICOLOR_ERR_LOG("failed to request GPIO=%d, ret=%d", gpio, err);
		} else {
			data->gpio_blue = gpio;
		}
	}

	if (!gpio_is_valid(data->gpio_red)) {
		TRICOLOR_DEBUG_LOG("red led is not exist");
	}
	if (!gpio_is_valid(data->gpio_green)) {
		TRICOLOR_DEBUG_LOG("green led is not exist");
	}
	if (!gpio_is_valid(data->gpio_blue)) {
		TRICOLOR_DEBUG_LOG("blue led is not exist");
	}

	if (of_property_read_bool(node, "use-mdm-led")) {
		TRICOLOR_DEBUG_LOG("use mdm-led");
		data->mdm_led = mdm_led_create();
		if (!data->mdm_led) {
			TRICOLOR_WARN_LOG("failed to mdm_led_init");
		}
	}

	data->initialized = true;

	TRICOLOR_DEBUG_LOG("[OUT]");
	return 0;

fail_misc_register:
	led_classdev_unregister(&data->cdev);
fail_led_classdev_register:
	wake_lock_destroy(&data->wake_lock);
fail_read_properties:
	TRICOLOR_DEBUG_LOG("[OUT]");
	return ret;
}

static void tricolor_led_delete(struct tricolor_led_data *data)
{
	if (!data->initialized)
		return;
	if (data->mdm_led) {
		mdm_led_destroy(data->mdm_led);
		data->mdm_led = NULL;
	}
	misc_deregister(&data->mdev);
	led_classdev_unregister(&data->cdev);
	wake_lock_destroy(&data->wake_lock);
}

struct tricolor_leds_priv {
	int num_leds;
	struct tricolor_led_data leds[];
};

static inline int sizeof_tricolor_leds_priv(int num_leds)
{
	return sizeof(struct tricolor_leds_priv) +
		(sizeof(struct tricolor_led_data) * num_leds);
}

static struct tricolor_leds_priv * tricolor_leds_create_of(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node, *child;
	struct tricolor_leds_priv *priv;
	int count = 0, ret;
	TRICOLOR_DEBUG_LOG("[IN]");

	/* count LEDs in this device, so we know how much to allocate */
	for_each_child_of_node(np, child)
		count++;
	if (!count) {
		TRICOLOR_DEBUG_LOG("no child");
		return NULL;
	}

	priv = kzalloc(sizeof_tricolor_leds_priv(count), GFP_KERNEL);
	if (!priv) {
		TRICOLOR_ERR_LOG("failed to kzalloc");
		return NULL;
	}

	for_each_child_of_node(np, child) {
		ret = tricolor_led_create(child, &priv->leds[priv->num_leds++], &pdev->dev);

		if (ret < 0) {
			TRICOLOR_ERR_LOG("failed to tricolor_led_create");
			of_node_put(child);
			goto err;
		}
	}

	TRICOLOR_DEBUG_LOG("[OUT]");
	return priv;

err:
	for (count = priv->num_leds - 1; count >= 0; count--)
		tricolor_led_delete(&priv->leds[count]);
	kfree(priv);
	return NULL;
}

static const struct of_device_id of_tricolor_led_match[] = {
	{ .compatible = "tricolor-leds", },
	{},
};

static int tricolor_leds_probe(struct platform_device *pdev)
{
	struct tricolor_leds_priv *priv;

	TRICOLOR_DEBUG_LOG("[IN]");

	priv = tricolor_leds_create_of(pdev);
	if (!priv) {
		TRICOLOR_ERR_LOG("failed to tricolor_led_create_of");
		return -ENODEV;
	}

	platform_set_drvdata(pdev, priv);

	TRICOLOR_DEBUG_LOG("[OUT]");
	return 0;
}

static int tricolor_leds_remove(struct platform_device *pdev)
{
	struct tricolor_leds_priv *priv = dev_get_drvdata(&pdev->dev);
	int i;

	for (i = 0; i < priv->num_leds; i++)
		tricolor_led_delete(&priv->leds[i]);

	platform_set_drvdata(pdev, NULL);
	kfree(priv);

	return 0;
}

static struct platform_driver tricolor_led_driver = {
	.probe		= tricolor_leds_probe,
	.remove		= tricolor_leds_remove,
	.driver		= {
		.name	= TRICOLOR_DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_tricolor_led_match,
	},
};

module_platform_driver(tricolor_led_driver);

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("Tricolor LED driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:leds-tricolor");
