/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
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
#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/power_supply.h>
#include "oem-chg_control.h"

#define OEM_CHARGER_CMD_DEV_NAME "oem_charger_cmd"


#define FEATURE_CHG_CMD_DEBUG
#ifdef FEATURE_CHG_CMD_DEBUG
#define CHG_CMD_ERR       pr_err
#define CHG_CMD_DEBUG     pr_err
#else
#define CHG_CMD_ERR       pr_err
#define CHG_CMD_DEBUG     pr_debug
#endif

char oem_chg_vadc_dev_name[128] = {0};
static char *oem_vadc_dev_namep = oem_chg_vadc_dev_name;
module_param(oem_vadc_dev_namep, charp, 0644);

static int oem_adc_read;
static int oem_adc_read_cmd(const char *val, struct kernel_param *kp)
{
	int ret;
	struct qpnp_vadc_result result;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	ret = oem_chg_vadc_read(oem_adc_read, &result);

	pr_info("channel = %d physical = %lld raw = %d\n",
			oem_adc_read, result.physical, result.adc_code);

	return 0;
}
module_param_call(oem_adc_read, oem_adc_read_cmd,
					param_get_uint, &oem_adc_read, 0644);

static struct platform_driver oem_charger_cmd_driver = {
	.driver		= {
			.name	= OEM_CHARGER_CMD_DEV_NAME,
			.owner	= THIS_MODULE,
	},
};

static int __init oem_chg_cmd_init(void)
{
	return platform_driver_register(&oem_charger_cmd_driver);
}

static void __exit oem_chg_cmd_exit(void)
{
	platform_driver_unregister(&oem_charger_cmd_driver);
}

late_initcall(oem_chg_cmd_init);
module_exit(oem_chg_cmd_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("OEM charger/battery Command driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:" OEM_CHARGER_CMD_DEV_NAME);
