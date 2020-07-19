/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 * (C) 2017 KYOCERA Corporation
 */
/*
 * LEDs driver for GPIOs
 *
 * Copyright (C) 2007 8D Technologies inc.
 * Raphael Assenat <raph@8d.com>
 * Copyright (C) 2008 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#define pr_fmt(fmt)	"LED %s(%d): " fmt, __func__, __LINE__

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/err.h>
//#include <mach/kc_board.h>
#include "leds.h" /* for leds_list */
#include <linux/delay.h>
#include <soc/qcom/smd.h>
#include <linux/leds-gpio_oem.h>

#define ENABLE_DEBUG_LOG        0
#define ENABLE_TEST_PARAM_DUMP  0
#define LED_CTRL_QUEUE_NUM      10
#define LEDS_GPIO_DEBUG         0

enum {
	GPIO_DEFSTATE_OFF  = 0,
	GPIO_DEFSTATE_ON   = 1,
	GPIO_DEFSTATE_KEEP = 2,
};

struct leds_gpio_data {
	struct led_classdev cdev;
	const char* name;
	int gpio;
	int spamp_id;
	int default_state;
	int active_low;
	int blinking;
	struct mutex lock;
};
struct leds_gpio_priv {
	int num;
	struct leds_gpio_data led_data[];
};

/* define STATIC as empty when JTAG debugging. */
#define STATIC_ static


static struct leds_gpio_data *the_data = NULL;
static DEFINE_MUTEX(kc_slcd_lock);
static atomic_t slcd_status = ATOMIC_INIT(0);


STATIC_ struct mutex g_spamp_gpio_lock;
STATIC_ int g_spamp_gpio = -1;
STATIC_ int g_spamp_gpio_status = 0;
STATIC_ int spamp_gpio_init(struct device_node* np);
STATIC_ int leds_gpio_oem_debug_enable = 0;/* '/sys/module/leds_gpio_oem/parameters/debug_enable' */
module_param_named(debug_enable, leds_gpio_oem_debug_enable, int, S_IRUGO | S_IWUSR);
#define debugk(fmt,...)	printk("%s(%d)[0x%p]:" fmt, __FUNCTION__, __LINE__, __builtin_return_address(0), ##__VA_ARGS__)

static void leds_gpio_set_value(
	struct leds_gpio_data *led_data,
	enum led_brightness value)
{
	int gpio = -1;
	int spamp_id;
	int level;

	if (!led_data) {
		return;
	}

	if (value == LED_OFF)
		level = 0;
	else
		level = 1;

	gpio = led_data->gpio;

	spamp_id = led_data->spamp_id;

	if (spamp_id) {
		spamp_gpio_enable(spamp_id, value);
	}

	if (led_data->active_low) {
		value = !value;
	}

	pr_debug("%s(%d):>>kc_slcd_brightness_set gpio=%d(%s) value=%d active_low=%d\n",
		__func__, __LINE__,
		gpio,
		led_data->name,
		value,
		led_data->active_low);
	gpio_set_value(gpio, value);
}

static void leds_gpio_brightness_set(
	struct led_classdev *led_cdev,
	enum led_brightness value)
{
	struct leds_gpio_data *led_data =
		container_of(led_cdev, struct leds_gpio_data, cdev);

	mutex_lock(&kc_slcd_lock);

	pr_debug("start slcd_status=%d brightness=%d\n",  atomic_read(&slcd_status), led_data->cdev.brightness);

	/* Setting GPIOs with I2C/etc requires a task context, and we don't
	 * seem to have a reliable way to know if we're already in one; so
	 * let's just assume the worst.
	 */
	if (led_data->blinking) {
		led_data->blinking = 0;
	}
	else {
		led_data->cdev.brightness = value;
		if ((value != LED_OFF) &&
			(!strcmp(led_data->name, LED_NAME_SUB_LCD))&&(!atomic_read(&slcd_status)))
			pr_err("slcd is off, discard backlight brightness:%d\n", led_data->cdev.brightness);
		else {
			leds_gpio_set_value(led_data, value);
		}
	}

	pr_debug("end brightness=%d\n", led_data->cdev.brightness);

	mutex_unlock(&kc_slcd_lock);
}

static int leds_gpio_blink_set(
	struct led_classdev *led_cdev,
	unsigned long *delay_on,
	unsigned long *delay_off)
{
	struct leds_gpio_data *led_data =
		container_of(led_cdev, struct leds_gpio_data, cdev);

	led_data->blinking = 1;
	return 0;
}

static int leds_gpio_create_led(
	struct leds_gpio_data* template,
	struct leds_gpio_data* led_data,
	struct device *parent)
{
	int state = 0;
	int value = 0;
	int ret;

	led_data->gpio = -1;

	/* skip leds that aren't available */
	if (!gpio_is_valid(template->gpio)) {
		dev_info(parent, "Skipping unavailable LED gpio %d (%s)\n",
				template->gpio, template->name);
		return 0;
	}

	ret = devm_gpio_request(parent, template->gpio, template->name);
	if (ret < 0) {
		dev_err(parent, "devm_gpio_request failed. ret=%d gpio=%d(%s)\n",
			ret,
			template->gpio,
			template->name);
		return ret;
	}

	if (template->default_state == GPIO_DEFSTATE_KEEP) {
		state = !!gpio_get_value_cansleep(template->gpio) ^ template->active_low;
	}
	else {
		state = (template->default_state == LEDS_GPIO_DEFSTATE_ON);
	}

	led_data->cdev.brightness = state ? LED_FULL : LED_OFF;

	value = template->active_low ^ state;
	ret = gpio_direction_output(template->gpio, value);
	if (ret < 0) {
		dev_err(parent, "gpio_direction_output failed. ret=%d gpio=%d(%s) state=%d value=%d\n",
			ret,
			template->gpio,
			template->name,
			state,
			value);
		return ret;
	}

	*led_data = *template;

	led_data->cdev.name = template->name;
	led_data->cdev.brightness_set = leds_gpio_brightness_set;
	led_data->cdev.blink_set = leds_gpio_blink_set;
	led_data->blinking = 0;
	mutex_init(&led_data->lock);

	if (0 == strcmp(template->name, LED_NAME_SUB_LCD)) {
		led_data->spamp_id = SPAMP_ID_SUBBL;
		the_data = led_data;
		pr_debug("the_data=%p\n", the_data);
	}
	else if (0 == strcmp(template->name, LED_NAME_KEYBL)) {
		led_data->spamp_id = SPAMP_ID_KEYBL;
	}
	else {
		led_data->spamp_id = 0;
	}

	ret = led_classdev_register(parent, &led_data->cdev);
	if (ret < 0) {
		dev_err(parent, "led_classdev_register failed. ret=%d gpio=%d(%s)\n",
			ret,
			template->gpio,
			template->name);
		return ret;
	}

	return 0;
}

static void leds_gpio_delete_led(struct leds_gpio_data *led_data)
{
	if (!gpio_is_valid(led_data->gpio))
		return;
	led_classdev_unregister(&led_data->cdev);
}

static inline int sizeof_gpio_leds_priv(int num)
{
	return sizeof(struct leds_gpio_priv) +
		(sizeof(struct leds_gpio_data) * num);
}

static int leds_gpio_of_get_gpio_flags(
	const char* name,
	struct device_node* child,
	enum of_gpio_flags* flags)
{
#if ENABLE_DEBUG_LOG
	int count = of_gpio_count(child);
	int is_sub_lcd = (0 == strcmp(name, LED_NAME_SUB_LCD));
	int type = OEM_get_board();
#endif/* ENABLE_DEBUG_LOG */
	int index = 0;

	if ((of_gpio_count(child) == 2)
	 && (0 == strcmp(name, LED_NAME_SUB_LCD))){
		index = 1;
	}

#if ENABLE_DEBUG_LOG
	pr_info("%s(%d):count=%d is_sub_lcd=%d type=%d index=%d\n",
		__func__, __LINE__, count, is_sub_lcd, type, index);
#endif/* ENABLE_DEBUG_LOG */

	return of_get_gpio_flags(child, index, flags);
}

int spamp_gpio_init(struct device_node* np)
{
	const char* name = "vddled-gpio";
	int gpio;
	int rc;

	if (!np) {
		pr_err("!failed. np is null.\n");
		return -1;
	}

	gpio = of_get_named_gpio(np, name, 0);

	if (!gpio_is_valid(gpio)) {
		pr_err("!of_get_named_gpio failed name[%s]\n", name);
		return -1;
	}

	rc = gpio_request(gpio, name);

	if (rc < 0) {
		pr_err("!gpio_request failed. gpio=%d(%s)\n", gpio, name);
		return rc;
	}

	rc = gpio_direction_output(gpio, 0);

	if (rc < 0) {
		pr_err("!gpio_direction_output failed. gpio=%d(%s)\n", gpio, name);
		return rc;
	}

	g_spamp_gpio = gpio;
	debugk("gpio=%d(%s)\n", gpio, name);

	mutex_init(&g_spamp_gpio_lock);

	return 0;
}

int spamp_gpio_enable(int id, int value)
{
	int status;
	int is_update;

	if (g_spamp_gpio < 0) {
		pr_err("failed. id=%08x value=%d gpio=%d\n", id, value, g_spamp_gpio);
		return -1;
	}

	mutex_lock(&g_spamp_gpio_lock);

	status = g_spamp_gpio_status;

	if (value) {/* enable */
		g_spamp_gpio_status |= id;
		is_update = (status != g_spamp_gpio_status) && (0 == status);
	}
	else {/* disable */
		g_spamp_gpio_status &= ~id;
		is_update = (status != g_spamp_gpio_status) && (0 == g_spamp_gpio_status);
	}

	if (is_update) {
		value  = !!value;

		if (leds_gpio_oem_debug_enable) {
			debugk("id=%08x value=%d status=%08x->%08x >>gpio_set_value(%d,%d)\n",
				id, value, status, g_spamp_gpio_status, g_spamp_gpio, value);
		}
		gpio_set_value(g_spamp_gpio, value);
	}
	else {
		if (leds_gpio_oem_debug_enable) {
			debugk("id=%08x value=%d status=%08x->%08x do nothing.\n",
				id, value, status, g_spamp_gpio_status);
		}
	}

	mutex_unlock(&g_spamp_gpio_lock);

	return 0;
}
EXPORT_SYMBOL(spamp_gpio_enable);

/* Code to create from OpenFirmware platform devices */
static struct leds_gpio_priv *gpio_leds_create_of(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node, *child;
	struct leds_gpio_priv *priv;
	int count;
	int size;
	int rc;

	spamp_gpio_init(np);

	/* count LEDs in this device, so we know how much to allocate */
	count = of_get_child_count(np);
	if (!count) {
		dev_err(&pdev->dev, "of_get_child_count failed.\n");
		return ERR_PTR(-ENODEV);
	}

	for_each_child_of_node(np, child) {
		if (of_get_gpio(child, 0) == -EPROBE_DEFER) {
			dev_err(&pdev->dev, "of_get_gpio(%p) failed.\n", child);
			return ERR_PTR(-EPROBE_DEFER);
		}
	}

	size = sizeof_gpio_leds_priv(count);
	priv = devm_kzalloc(&pdev->dev, size, GFP_KERNEL);
	if (!priv) {
		dev_err(&pdev->dev, "devm_kzalloc failed. count=%d size=%d\n", count, size);
		return ERR_PTR(-ENOMEM);
	}

	for_each_child_of_node(np, child) {
		struct leds_gpio_data led_data = {};
		enum of_gpio_flags flags;
		const char *default_state;

		led_data.name = of_get_property(child, "label", NULL) ? : child->name;
		led_data.gpio = leds_gpio_of_get_gpio_flags(led_data.name, child, &flags);
		led_data.active_low = flags & OF_GPIO_ACTIVE_LOW;
		default_state = of_get_property(child, "default-state", NULL)? : "off";

		pr_debug("%s(%d):[%s] gpio=%d active_low=%d default_state=%d\n",
			__func__, __LINE__,
			led_data.name,
			led_data.gpio,
			led_data.active_low,
			led_data.default_state);

		if (!strcmp(default_state, "keep"))
			led_data.default_state = LEDS_GPIO_DEFSTATE_KEEP;
		else if (!strcmp(default_state, "on"))
			led_data.default_state = LEDS_GPIO_DEFSTATE_ON;
		else
			led_data.default_state = LEDS_GPIO_DEFSTATE_OFF;

		rc = leds_gpio_create_led(&led_data, &priv->led_data[priv->num], &pdev->dev);

		if (rc < 0) {
			pr_err("leds_gpio_create_led failed. gpio=%d(%s)\n",
				led_data.gpio, led_data.name);
			continue;/* we ignore creation error. */
		}

		++priv->num;
	}

	return priv;
}

static const struct of_device_id of_gpio_leds_match[] = {
	{ .compatible = "leds-sublcd-gpio", },
	{},
};

static int leds_gpio_probe(struct platform_device *pdev)
{
	struct leds_gpio_priv *priv;

	priv = gpio_leds_create_of(pdev);

	if (IS_ERR(priv)) {
		dev_err(&pdev->dev, "LED: gpio_leds_create_of failed.\n");
		return -1;
	}

	platform_set_drvdata(pdev, priv);

	return 0;
}

static int leds_gpio_remove(struct platform_device *pdev)
{
	struct leds_gpio_priv *priv = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < priv->num; i++) {
		leds_gpio_delete_led(&priv->led_data[i]);
	}

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver leds_gpio_driver = {
	.probe		= leds_gpio_probe,
	.remove		= leds_gpio_remove,
	.driver		= {
		.name	= "leds-sublcd-gpio",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(of_gpio_leds_match),
	},
};

void kc_slcd_set_lcd_status(bool status)
{
	if (!the_data)
		return;

	mutex_lock(&kc_slcd_lock);
	pr_debug("start slcd_status:%d brightness:%d\n", atomic_read(&slcd_status), the_data->cdev.brightness);
	atomic_set(&slcd_status, status);
	if (status) {
		if (the_data->cdev.brightness != 0) {
			leds_gpio_set_value(the_data, the_data->cdev.brightness);
		}
	} else {
		if (the_data->cdev.brightness != 0) {
			leds_gpio_set_value(the_data, LED_OFF);
		}
	}
	pr_info("end slcd_status:%d brightness:%d\n", atomic_read(&slcd_status), the_data->cdev.brightness);
	mutex_unlock(&kc_slcd_lock);
	return;
}
EXPORT_SYMBOL(kc_slcd_set_lcd_status);

module_platform_driver(leds_gpio_driver);

MODULE_DESCRIPTION("GPIO LED driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:leds-gpio_oem");
