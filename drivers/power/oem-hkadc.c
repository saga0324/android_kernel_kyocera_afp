/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
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
#define pr_fmt(fmt)	"HKADC:%s: " fmt, __func__

#include <linux/module.h>
#include <linux/of.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/power_supply.h>
#include <linux/spmi.h>

#ifdef CONFIG_OEM_HKADC_USB_TM
#include "oem-hkadc_usb_tm.h"
#endif

//#define FEATURE_HKADC_DEBUG
#ifdef FEATURE_HKADC_DEBUG
#define HKADC_ERR       pr_err
#define HKADC_DEBUG     pr_err
#else
#define HKADC_ERR       pr_err
#define HKADC_DEBUG     pr_debug
#endif

#define DEFAULT_TEMP		25
#define DEFAULT_TIME_MS		10000

struct oem_hkac_chip {
	struct device				*dev;
	struct power_supply			hkadc_psy;
	struct qpnp_vadc_chip		*vadc_dev;
	unsigned int				hkadc_monitor_ms;
	unsigned int				hkadc_monitor_resume_ms;
	struct delayed_work			hkadc_monitor_work;
};

static enum power_supply_property oem_hkadc_power_props[] = {
	POWER_SUPPLY_PROP_OEM_PA_THERM,
	POWER_SUPPLY_PROP_OEM_SUBSTRATE_THERM,
	POWER_SUPPLY_PROP_OEM_USB_THERM,
	POWER_SUPPLY_PROP_OEM_CAMERA_THERM,
};

static int get_prop_pa_therm(struct oem_hkac_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX7_HW_ID, &results);
	if (rc) {
		HKADC_ERR("Unable to read pa temperature rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	HKADC_DEBUG("pa therm %d, %lld\n", results.adc_code, results.physical);

	return (int)results.physical;
}

static int get_prop_substrate_therm(struct oem_hkac_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX3_XO_THERM, &results);
	if (rc) {
		HKADC_ERR("Unable to read substrate temperature rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	HKADC_DEBUG("substrate therm %d, %lld\n", results.adc_code, results.physical);

	return (int)results.physical;
}

static int get_prop_usb_therm(struct oem_hkac_chip *chip)
{
#ifdef CONFIG_OEM_HKADC_USB_TM
	return oem_hkadc_usb_therm_value();
#else
	return DEFAULT_TEMP;
#endif
}

static int get_prop_camera_therm(struct oem_hkac_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	rc = qpnp_vadc_read(chip->vadc_dev, P_MUX4_1_1, &results);
	if (rc) {
		HKADC_ERR("Unable to read camera temperature rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	HKADC_DEBUG("camera therm %d, %lld\n", results.adc_code, results.physical);

	return (int)results.physical;
}

static int oem_hkadc_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct oem_hkac_chip *chip = container_of(psy, struct oem_hkac_chip,
								hkadc_psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_OEM_PA_THERM:
		val->intval = get_prop_pa_therm(chip);
		break;
	case POWER_SUPPLY_PROP_OEM_SUBSTRATE_THERM:
		val->intval = get_prop_substrate_therm(chip);
		break;
	case POWER_SUPPLY_PROP_OEM_USB_THERM:
		val->intval = get_prop_usb_therm(chip);
		break;
	case POWER_SUPPLY_PROP_OEM_CAMERA_THERM:
		val->intval = get_prop_camera_therm(chip);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void oem_hkadc_monitor(struct work_struct *work)
{
	struct oem_hkac_chip *chip = container_of(work,
				struct oem_hkac_chip,
				hkadc_monitor_work.work);

	power_supply_changed(&chip->hkadc_psy);

	schedule_delayed_work(&chip->hkadc_monitor_work,
				msecs_to_jiffies(chip->hkadc_monitor_ms));
}

static int oem_hkadc_parse_dt(struct oem_hkac_chip *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}

	/* Normal monitor time ms */
	rc = of_property_read_u32(node, "oem,monitor-time-ms",
						&chip->hkadc_monitor_ms);
	if (rc < 0) {
		chip->hkadc_monitor_ms = DEFAULT_TIME_MS;
		HKADC_ERR("Missing required properties rc=%d\n", rc);
	}

	HKADC_DEBUG("chip->hkadc_monitor_ms=%d\n", chip->hkadc_monitor_ms);

	/* Resume monitor time ms */
	rc = of_property_read_u32(node, "oem,resume-mon-time-ms",
						&chip->hkadc_monitor_resume_ms);
	if (rc < 0) {
		chip->hkadc_monitor_resume_ms = DEFAULT_TIME_MS;
		HKADC_ERR("Missing required properties rc=%d\n", rc);
	}

	HKADC_DEBUG("chip->hkadc_monitor_resume_ms=%d\n", chip->hkadc_monitor_resume_ms);

	return 0;
}

static int oem_hkadc_probe(struct spmi_device *spmi)
{
	int rc = 0;
	struct oem_hkac_chip	*chip;

	chip = devm_kzalloc(&spmi->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	chip->dev = &spmi->dev;
	dev_set_drvdata(&spmi->dev, chip);

	chip->vadc_dev = qpnp_get_vadc(chip->dev, "oem-hkadc");
	if (IS_ERR(chip->vadc_dev)) {
		rc = PTR_ERR(chip->vadc_dev);
		if (rc == -EPROBE_DEFER)
			HKADC_ERR("vadc not found - defer probe rc=%d\n", rc);
		else
			HKADC_ERR("vadc property missing, rc=%d\n", rc);

		goto fail_hkadc_enable;
	}

	rc = oem_hkadc_parse_dt(chip);
	if (rc < 0) {
		dev_err(&spmi->dev, "Unable to parse DT nodes\n");
		return rc;
	}

	chip->hkadc_psy.name = "hkadc";
	chip->hkadc_psy.type = POWER_SUPPLY_TYPE_HKADC;
	chip->hkadc_psy.properties = oem_hkadc_power_props;
	chip->hkadc_psy.num_properties = ARRAY_SIZE(oem_hkadc_power_props);
	chip->hkadc_psy.get_property = oem_hkadc_power_get_property;

	rc = power_supply_register(chip->dev, &chip->hkadc_psy);
	if (rc < 0) {
		HKADC_ERR("hkadc failed to register rc = %d\n", rc);
		goto fail_hkadc_enable;
	}

	INIT_DELAYED_WORK(&chip->hkadc_monitor_work, oem_hkadc_monitor);

	schedule_delayed_work(&chip->hkadc_monitor_work, 0);

	pr_info("probe success:pa_therm = %d, substrate_therm = %d, usb_therm = %d, camera_therm = %d \n",
					get_prop_pa_therm(chip), get_prop_substrate_therm(chip),
					get_prop_usb_therm(chip), get_prop_camera_therm(chip));
	return 0;

fail_hkadc_enable:
	HKADC_ERR("%s: failed.\n", __func__);

	return rc;
}

static int oem_hkadc_remove(struct spmi_device *spmi)
{
	struct oem_hkac_chip *chip = dev_get_drvdata(&spmi->dev);

	cancel_delayed_work_sync(&chip->hkadc_monitor_work);
	power_supply_unregister(&chip->hkadc_psy);
	dev_set_drvdata(&spmi->dev, NULL);
	return 0;
}

static int oem_hkadc_suspend(struct device *dev)
{
	struct oem_hkac_chip *chip = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&chip->hkadc_monitor_work);
	return 0;
}

static int oem_hkadc_resume(struct device *dev)
{
	struct oem_hkac_chip *chip = dev_get_drvdata(dev);

	schedule_delayed_work(&chip->hkadc_monitor_work,
				msecs_to_jiffies(chip->hkadc_monitor_resume_ms));
	return 0;
}

static const struct of_device_id oem_hkadc_of_match[] = {
	{ .compatible = "oem_hkadc-driver", },
	{},
};

static const struct dev_pm_ops oem_hkadc_pm_ops = {
	.resume		= oem_hkadc_resume,
	.suspend	= oem_hkadc_suspend,
};

static struct spmi_driver oem_hkadc_driver = {
	.driver = {
		.name = "oem_hkadc-driver",
		.owner = THIS_MODULE,
		.of_match_table = oem_hkadc_of_match,
		.pm = &oem_hkadc_pm_ops,
	},
	.probe  = oem_hkadc_probe,
	.remove = oem_hkadc_remove,
};

static int __init oem_hkadc_init(void)
{
	return spmi_driver_register(&oem_hkadc_driver);
}
module_init(oem_hkadc_init);

static void __exit oem_hkadc_exit(void)
{
	return spmi_driver_unregister(&oem_hkadc_driver);
}
module_exit(oem_hkadc_exit);

MODULE_DESCRIPTION("oem_hkadc driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("oem_hkadc");
