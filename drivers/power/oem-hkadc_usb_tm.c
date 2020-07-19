/* 
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)	"OEM_HKADC_USB_TM %s: " fmt, __func__

#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/power_supply.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/qpnp/qpnp-adc.h>
#include <soc/qcom/oem_fact.h>

static void oem_hkadc_usb_tm_irq(void);
static irqreturn_t oem_hkadc_usb_tm_isr(int irq, void *dev);

#define HKADC_USB_TM_ERR		pr_err
//#define FEATURE_HKADC_USB_TM_DEBUG
#ifdef FEATURE_HKADC_USB_TM_DEBUG
#define HKADC_USB_TM_DEBUG	pr_err
#else
#define HKADC_USB_TM_DEBUG	pr_debug
#endif

#define DRV_NAME "kc,oem_hkadc_usb_tm-driver"
#define DEVICE_NAME "oem_hkadc_usb_tm-gpio"

#define USB_THERM_NORMAL	25
#define USB_THERM_OVERHEAT	95

#define USB_THERM_CHECK_COUNT   2
#define CHECK_CYCLE_MS   100

struct oem_hkadc_usb_tm_device {
	int gpio;
	int irq;
	struct work_struct irq_work;
};

static struct oem_hkadc_usb_tm_device *oem_hkadc_usb_tm_dev;

static int usb_therm_value = USB_THERM_NORMAL;

static int usb_therm_status = 1;

static unsigned int suspend_state = 0;

static struct power_supply *batt_psy = NULL;
static struct power_supply *hkadc_psy = NULL;

struct delayed_work oem_hkadc_usb_tm_work;

static void oem_hkadc_usb_tm_irq(void);
static irqreturn_t oem_hkadc_usb_tm_isr(int irq, void *dev);

static void oem_hkadc_set_usb_therm_state(int therm_status)
{
	int rc = -EINVAL;
	union power_supply_propval ret = {0,};
	
	if(!therm_status && !oem_fact_get_option_bit(OEM_FACT_OPTION_ITEM_01, 3)){
		HKADC_USB_TM_DEBUG("USB_THERM_N:High ¨ Low\n");
		usb_therm_value = USB_THERM_OVERHEAT;
	}else{
		HKADC_USB_TM_DEBUG("USB_THERM_N:Low ¨ High\n");
		usb_therm_value = USB_THERM_NORMAL;
	}
	
	ret.intval = !therm_status;
	rc = batt_psy->set_property(batt_psy, POWER_SUPPLY_PROP_USB_TEMP_LEVEL, &ret);
	if (rc) {
		HKADC_USB_TM_ERR("Unable to set 'USB_TEMP_LEVEL' rc=%d\n", rc);
	}
	
	power_supply_changed(hkadc_psy);
	
	return;
}

static void oem_hkadc_usb_therm_error_detection(struct work_struct *work)
{
	static int check_cnt = 0;
	int new_usb_therm_status = gpio_get_value(oem_hkadc_usb_tm_dev->gpio);
	
	if (usb_therm_status == new_usb_therm_status) {
		check_cnt = 0;
		HKADC_USB_TM_DEBUG("equ usb_therm_status=%d\n", usb_therm_status);
		goto set_irq;
	}
	check_cnt++;
	
	if (USB_THERM_CHECK_COUNT <= check_cnt) {
		HKADC_USB_TM_DEBUG("usb_therm_status old=%d new=%d\n",
			usb_therm_status, new_usb_therm_status);
		usb_therm_status = new_usb_therm_status;
		oem_hkadc_set_usb_therm_state(usb_therm_status);
		check_cnt = 0;
	}

	if (check_cnt) {
		schedule_delayed_work(&oem_hkadc_usb_tm_work, msecs_to_jiffies(CHECK_CYCLE_MS));
		HKADC_USB_TM_DEBUG("set oem_hkadc_usb_tm_work=%dms cnt=%d\n", CHECK_CYCLE_MS, check_cnt);
		return;
	}

set_irq:
	oem_hkadc_usb_tm_irq();
	return;
}

static void oem_hkadc_usb_tm_irq(void)
{
	int rc = 0;

	free_irq(oem_hkadc_usb_tm_dev->irq, 0);
	if (!usb_therm_status) {
		rc = request_irq(oem_hkadc_usb_tm_dev->irq, oem_hkadc_usb_tm_isr, IRQF_TRIGGER_HIGH,"oem_hkadc_usb_tm-irq", 0);
	} else {
		rc = request_irq(oem_hkadc_usb_tm_dev->irq, oem_hkadc_usb_tm_isr, IRQF_TRIGGER_LOW,"oem_hkadc_usb_tm-irq", 0);
	}
	if (rc) {
		HKADC_USB_TM_ERR("couldn't register interrupts rc=%d\n", rc);
	}
}

static irqreturn_t oem_hkadc_usb_tm_isr(int irq, void *dev)
{
	disable_irq_nosync(oem_hkadc_usb_tm_dev->irq);
	
	HKADC_USB_TM_DEBUG("USB_THERM_N irq detected!! GPIO=%d\n", gpio_get_value(oem_hkadc_usb_tm_dev->gpio));
	
	schedule_delayed_work(&oem_hkadc_usb_tm_work, msecs_to_jiffies(CHECK_CYCLE_MS));
	
	return IRQ_HANDLED;
}

static int oem_hkadc_usb_tm_probe(struct platform_device *pdev)
{
	int rc = 0;
	
	INIT_DELAYED_WORK(&oem_hkadc_usb_tm_work, oem_hkadc_usb_therm_error_detection);

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		HKADC_USB_TM_ERR("battery power supply not found deferring probe\n");
		rc = -EPROBE_DEFER;
		goto fail_hkadc_enable;
	}
	
	hkadc_psy = power_supply_get_by_name("hkadc");
	if (!hkadc_psy) {
		HKADC_USB_TM_ERR("hkadc power supply not found deferring probe\n");
		rc = -EPROBE_DEFER;
		goto fail_hkadc_enable;
	}

	if (!pdev->dev.of_node) {
		HKADC_USB_TM_ERR("No platform supplied from device tree.\n");
		rc = -EINVAL;
		goto err_arg;
	}

	oem_hkadc_usb_tm_dev = kzalloc(sizeof(struct oem_hkadc_usb_tm_device), GFP_KERNEL);
	if (!oem_hkadc_usb_tm_dev) {
		HKADC_USB_TM_ERR("kzalloc fail\n");
		rc = -ENOMEM;
		goto err_alloc;
	}
	

	oem_hkadc_usb_tm_dev->gpio = of_get_named_gpio(pdev->dev.of_node, DEVICE_NAME, 0);
	if (oem_hkadc_usb_tm_dev->gpio < 0) {
		HKADC_USB_TM_ERR("of_get_named_gpio failed.\n");
		rc = -EINVAL;
		goto err_gpio;
	}

	rc = gpio_request(oem_hkadc_usb_tm_dev->gpio, DEVICE_NAME);
	if (rc) {
		HKADC_USB_TM_ERR("gpio_request failed.\n");
		goto err_gpio;
	}
	
	usb_therm_status = 1;
	
	oem_hkadc_usb_tm_dev->irq = gpio_to_irq(oem_hkadc_usb_tm_dev->gpio);

	rc = request_irq(oem_hkadc_usb_tm_dev->irq, oem_hkadc_usb_tm_isr, IRQF_TRIGGER_LOW,
							"oem_hkadc_usb_tm-irq", 0);
	if (rc) {
		HKADC_USB_TM_ERR("failed request_irq.\n");
		goto err_irq;
	}

	HKADC_USB_TM_ERR("successful. GPIO(%d)=%d\n", oem_hkadc_usb_tm_dev->gpio, gpio_get_value(oem_hkadc_usb_tm_dev->gpio));
	enable_irq_wake(oem_hkadc_usb_tm_dev->irq);

	return 0;


err_irq:
	gpio_free(oem_hkadc_usb_tm_dev->gpio);

err_gpio:
	kfree(oem_hkadc_usb_tm_dev);

err_alloc:
err_arg:
fail_hkadc_enable:
	HKADC_USB_TM_DEBUG("failed.\n");

	return rc;
}

static int oem_hkadc_usb_tm_remove(struct platform_device *pdev)
{
	free_irq(oem_hkadc_usb_tm_dev->irq, oem_hkadc_usb_tm_dev);
	gpio_free(oem_hkadc_usb_tm_dev->gpio);
	kfree(oem_hkadc_usb_tm_dev);

	return 0;
}

static const struct of_device_id oem_hkadc_usb_tm_of_match[] = {
	{ .compatible = DRV_NAME, },
};

static int oem_hkadc_usb_tm_suspend(struct device *dev)
{
	suspend_state = 1;
	return 0;
}

static int oem_hkadc_usb_tm_resume(struct device *dev)
{
	suspend_state = 0;
	return 0;
}

static const struct dev_pm_ops oem_hkadc_usb_tm_pm_ops = {
	.suspend	= oem_hkadc_usb_tm_suspend,
	.resume		= oem_hkadc_usb_tm_resume,
};

static struct platform_driver oem_hkadc_usb_tm_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = oem_hkadc_usb_tm_of_match,
		.pm 	= &oem_hkadc_usb_tm_pm_ops,
	},
	.probe = oem_hkadc_usb_tm_probe,
	.remove = oem_hkadc_usb_tm_remove,
};

int oem_hkadc_usb_therm_value(void)
{
	return usb_therm_value;
}
EXPORT_SYMBOL(oem_hkadc_usb_therm_value);

int oem_hkadc_usb_therm_error_state(void)
{
	return !usb_therm_status;
}
EXPORT_SYMBOL(oem_hkadc_usb_therm_error_state);

int __init oem_hkadc_usb_tm_init(void)
{
	return platform_driver_register(&oem_hkadc_usb_tm_driver);	
}
module_init(oem_hkadc_usb_tm_init);

static void __exit oem_hkadc_usb_tm_exit(void)
{
	platform_driver_unregister(&oem_hkadc_usb_tm_driver);
}
module_exit(oem_hkadc_usb_tm_exit);

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("OEM HKADC USB TM Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("oem_hkadc_usb_tm");