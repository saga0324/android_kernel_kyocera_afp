/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
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

#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spmi.h>

struct qpnp_diag {
	struct spmi_device *spmi;
};

static struct qpnp_diag *sys_diag_dev;

int oem_qpnp_diag_read(int offset, u8 *val)
{
	struct qpnp_diag *diag = sys_diag_dev;
	int rc;
	u8 sid;
	u16 addr;

	if (!diag) {
		pr_err("%s: dev is NULL\n", __func__);
		return -ENODEV;
	}

	sid = (offset >> 16) & 0xF;
	addr = offset & 0xFFFF;

	rc = spmi_ext_register_readl(diag->spmi->ctrl, sid, addr, val, 1);

	if (rc) {
		dev_err(&diag->spmi->dev,
			"Unable to read from addr=%x, rc(%d)\n", addr, rc);
		return rc;
	}
	return rc;
}
EXPORT_SYMBOL(oem_qpnp_diag_read);

int oem_qpnp_diag_masked_write(int offset, u8 mask, u8 val)
{
	struct qpnp_diag *diag = sys_diag_dev;
	int rc;
	u8 reg;
	u8 sid;
	u16 addr;

	if (!diag) {
		pr_err("%s: dev is NULL\n", __func__);
		return -ENODEV;
	}

	sid = (offset >> 16) & 0xF;
	addr = offset & 0xFFFF;

	rc = spmi_ext_register_readl(diag->spmi->ctrl, sid, addr, &reg, 1);
	if (rc) {
		dev_err(&diag->spmi->dev,
			"Unable to read from addr=%x, rc(%d)\n", addr, rc);
		return rc;
	}

	reg &= ~mask;
	reg |= val & mask;
	rc = spmi_ext_register_writel(diag->spmi->ctrl, sid, addr, &reg, 1);
	if (rc)
		dev_err(&diag->spmi->dev,
			"Unable to write to addr=%x, rc(%d)\n", addr, rc);
	return rc;
}
EXPORT_SYMBOL(oem_qpnp_diag_masked_write);

static int oem_qpnp_diag_probe(struct spmi_device *spmi)
{
	struct qpnp_diag *diag;

	diag = devm_kzalloc(&spmi->dev, sizeof(*diag), GFP_KERNEL);
	if (!diag) {
		pr_err("%s: Cannot allocate\n", __func__);
		return -ENOMEM;
	}

	diag->spmi = spmi;
	dev_set_drvdata(&spmi->dev, diag);
	sys_diag_dev = diag;

	pr_info("%s: driver probe OK\n", __func__);

	return 0;
}

static int oem_qpnp_diag_remove(struct spmi_device *spmi)
{
	dev_set_drvdata(&spmi->dev, NULL);
	kfree(sys_diag_dev);
	return 0;
}

static struct of_device_id spmi_match_table[] = {
	{	.compatible = "oem,qpnp-diag",
	}
};

static struct spmi_driver oem_qpnp_diag_driver = {
	.driver		= {
		.name	= "oem,qpnp-diag",
		.of_match_table = spmi_match_table,
	},
	.probe		= oem_qpnp_diag_probe,
	.remove		= oem_qpnp_diag_remove,
};

static int __init oem_qpnp_diag_init(void)
{
	return spmi_driver_register(&oem_qpnp_diag_driver);
}

static void __exit oem_qpnp_diag_exit(void)
{
	spmi_driver_unregister(&oem_qpnp_diag_driver);
}

module_init(oem_qpnp_diag_init);
module_exit(oem_qpnp_diag_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("OEM QPNP PMIC diag driver");
