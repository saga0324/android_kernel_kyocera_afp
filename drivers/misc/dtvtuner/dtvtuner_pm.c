/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
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
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/cdev.h>

#include <linux/dtvtuner_pm.h>
#include <linux/pinctrl/machine.h>

/* delay time (msec) */
#define D_DTVTUNER_DEVICE_PON_WAITTIME			(35)
#define D_DTVTUNER_DEVICE_POFF_WAITTIME			(10)
#define D_DTVTUNER_DEVICE_RECOVERY_WAITTIME		(10)

struct dtvtuner_pm_private_data {
	struct cdev cdev;
	struct device *dd;
	struct dtvtuner_pm_drvdata *drvdata;
	struct pinctrl_state *ldo_active;
	struct pinctrl_state *ldo_sleep;
	struct pinctrl_state *intb_default;
};

struct dtvtuner_pm_drvdata {
	struct platform_device *pdev;
	struct dtvtuner_pm_private_data private_data;
};

static struct class *dtvtuner_pm_class;
static dev_t dtvtuner_pm_minor;

/* forward declarations */
static int dtvtuner_pm_open(struct inode *inode, struct file *filp);
static int dtvtuner_pm_release(struct inode *inode, struct file *filp);
static ssize_t dtvtuner_pm_write(struct file *, const char __user *, size_t, loff_t *);
static int dtvtuner_pm_dev_tuner_recovery(struct dtvtuner_pm_drvdata *drvdata);


/* file operations */
static const struct file_operations dtvtuner_pm_fops = {
	.owner		= THIS_MODULE,
	.open		= dtvtuner_pm_open,
	.release	= dtvtuner_pm_release,
	.write		= dtvtuner_pm_write,
};

/**
 * dtvtuner_pm_dev_init() - Device initialization.
 * @drvdata:	[IN]	driver data
 *
 * Return codes
 *   0 - Success
 *   -EIO - Fail of initialization
 */
 
static int dtvtuner_pm_dev_init(
	struct dtvtuner_pm_drvdata *drvdata
)
{
	struct dtvtuner_pm_private_data *private_data =
				&drvdata->private_data;
	int ret;

	/* Power Ctrl init */
	private_data->cdev.owner = THIS_MODULE;
	private_data->drvdata = drvdata;
	pr_info("dtvtuner_pm_dev_init: cdev_init()\n");
	cdev_init(&private_data->cdev, &dtvtuner_pm_fops);
	if (cdev_add(&private_data->cdev, dtvtuner_pm_minor, 1) != 0) {
		pr_err("dtvtuner_pm: cdev_add failed");
		goto err_request_gpio_rst;
	}

	pr_info("dtvtuner_pm_dev_init: device_create()\n");
	private_data->dd = device_create(dtvtuner_pm_class, NULL,
						 private_data->cdev.dev, private_data,
						 D_DTVTUNER_PM_DRIVER_NAME"%d", drvdata->pdev->id);
	if (IS_ERR(private_data->dd)) {
		pr_err("dtvtuner_pm: device_create failed: %i",
            (int)PTR_ERR(private_data->dd));
		goto err_device_create;
	}

	/* pinctrl operations */
	pr_info("dtvtuner_pm_dev_init: kzalloc()\n");
	drvdata->pdev->dev.pins = kzalloc(sizeof(*(drvdata->pdev->dev.pins)), GFP_KERNEL);
    if (!drvdata->pdev->dev.pins)
        return -ENOMEM;

	pr_info("dtvtuner_pm_dev_init: devm_pinctrl_get()\n");
    drvdata->pdev->dev.pins->p = devm_pinctrl_get(&drvdata->pdev->dev);
    if (IS_ERR(drvdata->pdev->dev.pins->p)) {
        dev_dbg(&drvdata->pdev->dev, "no pinctrl handle\n");
        ret = PTR_ERR(drvdata->pdev->dev.pins->p);
        goto cleanup_alloc;
    }

	pr_info("dtvtuner_pm_dev_init: default_state=pinctrl_lookup_state(dtvtuner_inactive)\n");
    drvdata->pdev->dev.pins->default_state = pinctrl_lookup_state(drvdata->pdev->dev.pins->p,
                    "dtvtuner_inactive");

	if (IS_ERR(drvdata->pdev->dev.pins->default_state)) {
        dev_dbg(&drvdata->pdev->dev, "no default pinctrl state\n");
        ret = 0;
        goto cleanup_get_init;
    }

	pr_info("dtvtuner_pm_dev_init: pinctrl_select_state(default_state)\n");
    ret = pinctrl_select_state(drvdata->pdev->dev.pins->p, drvdata->pdev->dev.pins->default_state);
    if (ret) {
        dev_dbg(&drvdata->pdev->dev, "failed to activate default pinctrl state: default_state\n");
        goto cleanup_get_init;
    }

	pr_info("dtvtuner_pm_dev_init: sleep=pinctrl_lookup_state(dtvtuner_ldo_sleep)\n");
	private_data->ldo_sleep   = pinctrl_lookup_state(drvdata->pdev->dev.pins->p, "dtvtuner_ldo_sleep");
	pr_info("dtvtuner_pm_dev_init: active=pinctrl_lookup_state(dtvtuner_ldo_active)\n");
	private_data->ldo_active  = pinctrl_lookup_state(drvdata->pdev->dev.pins->p, "dtvtuner_ldo_active");
	pr_info("dtvtuner_pm_dev_init: sleep=pinctrl_lookup_state(dtvtuner_intb_default)\n");
	private_data->intb_default  = pinctrl_lookup_state(drvdata->pdev->dev.pins->p, "dtvtuner_intb_default");

	return 0;

err_device_create:
	cdev_del(&private_data->cdev);

err_request_gpio_rst:

	return -EIO;

cleanup_get_init:
	devm_pinctrl_put(drvdata->pdev->dev.pins->p);

cleanup_alloc:
	devm_kfree(&drvdata->pdev->dev, drvdata->pdev->dev.pins);
	drvdata->pdev->dev.pins = NULL;

	/* Only return deferrals */
	if (ret != -EPROBE_DEFER)
		ret = 0;

	return ret;

}

/**
 * dtvtuner_pm_dev_finalize() - Device finalization.
 *
 * Return codes
 *   0 - Success
 */
static int dtvtuner_pm_dev_finalize(void)
{

	return 0;
}

/**
 * dtvtuner_pm_dev_tuner_power_on() - power on dtvtuner pm device.
 * @drvdata:	[IN]	driver data
 *
 * Return codes
 *   0 - Success
 */
static int dtvtuner_pm_dev_tuner_power_on(
	struct dtvtuner_pm_drvdata *drvdata
)
{
	int ret;
	struct dtvtuner_pm_private_data *private_data =
				&drvdata->private_data;

	// power on
	pr_info("dtvtuner_pm_dev_tuner_power_on: pinctrl_select_state(ldo_active)\n");
	ret = pinctrl_select_state(drvdata->pdev->dev.pins->p, private_data->ldo_active);
    if (ret) {
        dev_dbg(&drvdata->pdev->dev, "failed to activate pinctrl state: dtv_ldo_en(active_state)\n");
        goto cleanup_get_pon;
    }

	dev_info(&drvdata->pdev->dev, "PowerOn\n");
	msleep(D_DTVTUNER_DEVICE_PON_WAITTIME);

	// Change GPIO_8,9,10,11 to SPI
	//  -> Controlled from pinctrl

	// host intb release
	pr_info("dtvtuner_pm_dev_tuner_power_on: pinctrl_select_state(intb_default)\n");
	ret = pinctrl_select_state(drvdata->pdev->dev.pins->p, private_data->intb_default);
    if (ret) {
        dev_dbg(&drvdata->pdev->dev, "failed to activate pinctrl state: dtv_host_intb(default_state)\n");
        goto cleanup_get_pon;
    }

	return 0;

cleanup_get_pon:
	devm_pinctrl_put(drvdata->pdev->dev.pins->p);

	/* Only return deferrals */
	if (ret != -EPROBE_DEFER)
		ret = 0;

	return ret;

}

/**
 * dtvtuner_pm_dev_tuner_recovery() - recovery from unusual state.
 * @drvdata:	[IN]	driver data
 *
 * Return codes
 *   0 - Success
 */
static int dtvtuner_pm_dev_tuner_recovery(
	struct dtvtuner_pm_drvdata *drvdata
)
{
	int ret;
	struct dtvtuner_pm_private_data *private_data =
				&drvdata->private_data;

	// LDO_EN="L"
	pr_info("dtvtuner_pm_dev_tuner_recovery: pinctrl_select_state(ldo_sleep)\n");
	ret = pinctrl_select_state(drvdata->pdev->dev.pins->p, private_data->ldo_sleep);
    if (ret) {
        dev_dbg(&drvdata->pdev->dev, "failed to activate pinctrl state: dtv_ldo_en(sleep_state)\n");
        goto cleanup_get_rst;
    }

	// WAIT 10ms
	msleep(D_DTVTUNER_DEVICE_RECOVERY_WAITTIME);

	// Power On Sequence
	dtvtuner_pm_dev_tuner_power_on(drvdata);

	return 0;

cleanup_get_rst:
	devm_pinctrl_put(drvdata->pdev->dev.pins->p);

	/* Only return deferrals */
	if (ret != -EPROBE_DEFER)
		ret = 0;

	return ret;

}

/**
 * dtvtuner_pm_dev_tuner_power_off() - power off dtvtuner pm device.
 * @drvdata:	[IN]	driver data
 *
 * Return codes
 *   0 - Success
 */
static int dtvtuner_pm_dev_tuner_power_off(
	struct dtvtuner_pm_drvdata *drvdata
)
{
	int ret;
	struct dtvtuner_pm_private_data *private_data =
				&drvdata->private_data;

	pr_info("dtvtuner_pm_dev_tuner_power_off: msleep(10)\n");
	msleep(D_DTVTUNER_DEVICE_POFF_WAITTIME);

	// host intb release
	pr_info("dtvtuner_pm_dev_tuner_power_off: pinctrl_select_state(intb_default)\n");
	ret = pinctrl_select_state(drvdata->pdev->dev.pins->p, private_data->intb_default);
    if (ret) {
        dev_dbg(&drvdata->pdev->dev, "failed to activate pinctrl state: dtv_host_intb(default_state)\n");
        goto cleanup_get_poff;
    }

	// power off
	pr_info("dtvtuner_pm_dev_tuner_power_off: pinctrl_select_state(ldo_sleep)\n");
	ret = pinctrl_select_state(drvdata->pdev->dev.pins->p, private_data->ldo_sleep);
    if (ret) {
        dev_dbg(&drvdata->pdev->dev, "failed to activate pinctrl state: dtv_ldo_en(sleep_state)\n");
        goto cleanup_get_poff;
    }

	dev_info(&drvdata->pdev->dev, "PowerOff\n");

	return 0;

cleanup_get_poff:
	devm_pinctrl_put(drvdata->pdev->dev.pins->p);

	/* Only return deferrals */
	if (ret != -EPROBE_DEFER)
		ret = 0;

	return ret;

}

/* copy device-tree data to platform data struct */
static struct dtvtuner_pm_platform_data *
dtvtuner_get_dt_info(struct platform_device *pdev)
{
    struct device_node *node = pdev->dev.of_node;
    struct dtvtuner_pm_platform_data *data;
    int i, rc;
	int gpio;
	const char *label;
	int num_gpios;

    /* Note: memory allocated by devm_kzalloc is freed automatically */
    data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
    if (!data) {
        pr_err("dtvtuner_pm: Unable to allocate platform data\n");
        return NULL;
    }
	data->gpio_pwr = -1;
	data->gpio_isr = -1;

    num_gpios = of_gpio_count(node);
    if (num_gpios == 0) {
        pr_err("dtvtuner_pm: Could not find GPIO definitions\n");
        return NULL;
    }

    for (i = 0; i < num_gpios; i++) {
        gpio = of_get_gpio(node, i);
        rc = of_property_read_string_index(node, "gpio-names",
                        i, &label);
        if (rc) {
            pr_warn("dtvtuner_pm: Could not find gpio-names property\n");
			continue;
		}
		if (!strncmp("DTV_LDO_EN", label, strlen(label))) {
			data->gpio_pwr = gpio;
            pr_warn("dtvtuner_pm: %s:%d\n", label, data->gpio_pwr);
		} else if (!strncmp("DTV_HOST_INTB", label, strlen(label))) {
			data->gpio_isr = gpio;
            pr_warn("dtvtuner_pm: %s:%d\n", label, data->gpio_isr);
		}
    }
	if (data->gpio_pwr == -1 || data->gpio_isr == -1) {
        pr_err("dtvtuner_pm: Could not set gpios\n");
		return NULL;
	}
    return data;
}

/**
 * dtvtuner_pm_probe() - Prove dtvtuner pm driver.
 * @pdev:	[IN]	platform device data
 *
 * Return codes
 *   0 - Success
 *   -ENODEV - fail to probe
 *   -EINVAL - no platform data
 *   -ENOMEM - no memory for driver data
 */
static int dtvtuner_pm_probe(struct platform_device *pdev)
{
	int	ret = -ENODEV;
	struct dtvtuner_pm_platform_data *pfdata;
	struct dtvtuner_pm_drvdata *drvdata;

	if (pdev->dev.of_node) {
		/* get information from device tree */
		pfdata = dtvtuner_get_dt_info(pdev);
		/* get device ID */
		ret = of_property_read_u32(pdev->dev.of_node,
					"cell-index", &pdev->id);
		if (ret) {
			pdev->id = -1;
		}
		dev_warn(&pdev->dev, "pdev->id:%d\n", pdev->id);
		pdev->dev.platform_data = pfdata;
	} else {
		pfdata = pdev->dev.platform_data;
	}
	if (!pfdata) {
		dev_err(&pdev->dev, "No platform data.\n");
		ret = -EINVAL;
		goto err_get_platform_data;
	}

	drvdata = kzalloc(sizeof(struct dtvtuner_pm_drvdata), GFP_KERNEL);
	if (!drvdata) {
		dev_err(&pdev->dev, "No enough memory for dtvtuner_pm\n");
		ret = -ENOMEM;
		goto err_alloc_data;
	}

	drvdata->pdev = pdev;
	platform_set_drvdata(pdev, drvdata);

	ret = dtvtuner_pm_dev_init(drvdata);
	if (ret) {
		dev_err(&pdev->dev, "Fail to initialize\n");
		goto err_gpio_init;
	}

	return 0;

err_gpio_init:
	dtvtuner_pm_dev_finalize();
	kfree(drvdata);
err_alloc_data:
err_get_platform_data:

	return ret;
}

static ssize_t dtvtuner_pm_write(struct file *filp,
								 const char __user *userbuf,
								 size_t count, loff_t *f_pos)
{
	unsigned long value;
	char s[2];
	struct dtvtuner_pm_private_data *private_data;
	struct dtvtuner_pm_drvdata *drvdata;
	int len = min(sizeof(s) - 1, count);
	private_data = filp->private_data;
	drvdata = private_data->drvdata;

	if (copy_from_user(s, userbuf, len)) {
		return -EFAULT;
	}

	s[len] = '\0';
	
    if (strict_strtoul((const char*)s, 0, &value)) {
		pr_err("Invalid value for power_ctrl\n");
		return -EFAULT;
    }

    if (value == D_DTVTUNER_POWER_ON)
        dtvtuner_pm_dev_tuner_power_on(drvdata);
    else if (value == D_DTVTUNER_POWER_OFF)
        dtvtuner_pm_dev_tuner_power_off(drvdata);
    else if (value == D_DTVTUNER_RECOVERY)
        dtvtuner_pm_dev_tuner_recovery(drvdata);

	return 0;
}

static int dtvtuner_pm_open(struct inode *inode, struct file *filp)
{

	struct dtvtuner_pm_private_data *private_data;

	private_data = container_of(inode->i_cdev, struct dtvtuner_pm_private_data, cdev);
	filp->private_data = private_data;

	return 0;
}

static int dtvtuner_pm_release(struct inode *inode, struct file *filp)
{
	struct dtvtuner_pm_private_data *private_data;
	struct dtvtuner_pm_drvdata *drvdata;
	
	private_data = filp->private_data;
	drvdata = private_data->drvdata;
	
	dtvtuner_pm_dev_tuner_power_off(drvdata);
	return 0;
}

/**
 * dtvtuner_pm_remove() - Remove dtvtuner pm driver.
 * @pdev:	[IN]	platform device data
 */
static int dtvtuner_pm_remove(struct platform_device *pdev)
{
	struct dtvtuner_pm_drvdata *drvdata = dev_get_drvdata(&pdev->dev);
	struct dtvtuner_pm_private_data *private_data = &drvdata->private_data;

	device_destroy(dtvtuner_pm_class, private_data->cdev.dev);
	cdev_del(&private_data->cdev);
	dtvtuner_pm_dev_finalize();
	kfree(drvdata);

	return 0;
}


#if 0
#else
#define dtvtuner_pm_suspend	NULL
#define dtvtuner_pm_resume	NULL
#endif

static const struct dev_pm_ops dtvtuner_pm_ops = {
	.suspend	= dtvtuner_pm_suspend,
	.resume		= dtvtuner_pm_resume,
};

static struct of_device_id dtvtuner_pm_match_table[] = {
    {.compatible = D_DTVTUNER_PM_DRIVER_NAME},
    {}
};

/**
 * brief Platform driver data structure of dtvtuner driver
 */
static struct platform_driver dtvtuner_pm_driver = {
	.probe		= dtvtuner_pm_probe,
	.remove		= dtvtuner_pm_remove,
	.driver		= {
		.name		= D_DTVTUNER_PM_DRIVER_NAME,
		.pm = &dtvtuner_pm_ops,
        .of_match_table = dtvtuner_pm_match_table,
	},
};

/**
 * dtvtuner_pm_driver_init() - The module init handler.
 *
 * Return codes
 *   0 - Success
 *   -errno - Failure
 */
static int __init dtvtuner_pm_driver_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&dtvtuner_pm_minor, 0, 1, D_DTVTUNER_PM_DRIVER_NAME);
	if (ret) {
        pr_err("dtvtuner_pm: alloc_chrdev_region failed: %d", ret);
		goto err_devrgn;
	}

	dtvtuner_pm_class = class_create(THIS_MODULE, D_DTVTUNER_PM_DRIVER_NAME);
	if (IS_ERR(dtvtuner_pm_class)) {
		ret = PTR_ERR(dtvtuner_pm_class);
        pr_err("dtvtuner_pm: Error creating class: %d", ret);
        goto err_class;
	}

	ret = platform_driver_register(&dtvtuner_pm_driver);
	if (ret) {
        pr_err("dtvtuner_pm: platform_driver_register failed: %d", ret);
        goto err_register;
	}

	return 0;

err_register:
    class_destroy(dtvtuner_pm_class);
err_class:
    unregister_chrdev_region(0, 1);
err_devrgn:
    return ret;
}

/**
 * dtvtuner_pm_driver_exit() - The module exit handler.
 *
 * Return codes
 *   0 - Success
 *   -errno - Failure
 */
static void __exit dtvtuner_pm_driver_exit(void)
{
	platform_driver_unregister(&dtvtuner_pm_driver);

	class_destroy(dtvtuner_pm_class);
	unregister_chrdev_region(0, 1);
}

module_init(dtvtuner_pm_driver_init);
module_exit(dtvtuner_pm_driver_exit);

MODULE_DESCRIPTION("dtvtuner_pm driver");
MODULE_LICENSE("GPL v2");
