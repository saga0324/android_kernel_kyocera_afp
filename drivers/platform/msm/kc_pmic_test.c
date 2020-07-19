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
 *
 */

#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/mm_types.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>

#include <linux/qpnp/pin.h>
#include <linux/qpnp/oem_qpnp_diag.h>

typedef void (*err_diag_func_type)(void);

#define DIAG_LOG_01 KERN_ERR
#define DIAG_LOG_02 KERN_ERR
#define DIAG_LOG_03 KERN_ERR
#define DIAG_LOG_04 KERN_DEBUG
#define DIAG_LOG_05 KERN_DEBUG
#define DIAG_LOG_06 KERN_DEBUG

#define PM8916_VREG_LDO_NUM	18 /* PM8916 VREG LDO Number */
#define PM8916_VREG_SMPS_NUM	4  /* PM8916 VREG SMPS Number */
#define PM8916_VREG_VS_NUM	0  /* PM8916 VREG VS Number  */
#define PM8916_VREG_BOOST_NUM	0  /* PM8916 VREG BOOST Number  */

#define PM_VREG_LDO_NUM		PM8916_VREG_LDO_NUM
#define PM_VREG_SMPS_NUM	(PM8916_VREG_SMPS_NUM)
#define PM_VREG_VS_NUM		PM8916_VREG_VS_NUM
#define PM_VREG_BOOST_NUM	PM8916_VREG_BOOST_NUM

#define PM_VREG_MAX_NUM		(PM_VREG_LDO_NUM + PM_VREG_SMPS_NUM \
				+ PM_VREG_VS_NUM + PM_VREG_BOOST_NUM)

enum {
	PMIC_TEST_RTC       = 0x0000,
	PMIC_TEST_CHG       = 0x0001,
	PMIC_TEST_USB       = 0x0002,
	PMIC_TEST_AMUX      = 0x0003,
	PMIC_TEST_VREG      = 0x0004,
	PMIC_TEST_INT       = 0x0005,
	PMIC_TEST_UI        = 0x0006,
	PMIC_TEST_DVT       = 0x0007,
	PMIC_TEST_SPKR      = 0x0008,
	PMIC_TEST_VID       = 0x0009,
	PMIC_TEST_MIC       = 0x000A,
	PMIC_TEST_RESET     = 0x000B,
	PMIC_TEST_MPP       = 0x000C,
	PMIC_TEST_GPIO      = 0x000D,
	PMIC_TEST_SPMI      = 0x0100,
	PMIC_TEST_QPNP_PIN  = 0x0101,
};

enum {
	PMIC_GPIO_SET_VALUE,
	PMIC_GPIO_GET_VALUE,
};

enum {
	PMIC_VREG_SET_ENABLE,
	PMIC_VREG_SET_DISABLE,
	PMIC_VREG_SET_VOLT,
	PMIC_VREG_SET_MODE,
	PMIC_VREG_SET_FREQ,
};

enum {
	PMIC_QPNP_PIN_CONFIG,
};

enum {
	PMIC_SPMI_WRITE,
	PMIC_SPMI_READ,
};

enum {
	PMIC_VREG_SMPS,
	PMIC_VREG_LDO,
	PMIC_VREG_VS,
	PMIC_VREG_BOOST,
};

typedef struct {
	int data1;
	int data2;
	int data3;
	int data4;
	int data5;
} str_diag_res;

typedef struct {
	int cmd;
	int arg1;
	int arg2;
	int arg3;
	int arg4;
} str_diag_req;

typedef struct {
	const char *id;
	int vreg_type;
	int min_uV;
	int max_uV;
} vreg_setting_data;

static const char *pin_type[3]
	= { "msm-gpio", "pm8916-gpio", "pm8916-mpp"};

static struct regulator *reg_store[PM_VREG_MAX_NUM];
static int reg_volt_set_check[PM_VREG_MAX_NUM];

vreg_setting_data vreg_data[PM_VREG_MAX_NUM] = {
	/* id           vreg_type        min_uV   max_uV       name */
	/* PM8916 */
	{"8916_s1",	PMIC_VREG_SMPS,   500000, 1350000}, /* SMPS_1   */
	{"8916_s2",	PMIC_VREG_SMPS,   900000, 1350000}, /* SMPS_2   */
	{"8916_s3",	PMIC_VREG_SMPS,  1250000, 1350000}, /* SMPS_3   */
	{"8916_s4",	PMIC_VREG_SMPS,  1850000, 2150000}, /* SMPS_4   */

	{"8916_l1",	PMIC_VREG_LDO,   1000000, 1225000}, /* LDO_L1   */
	{"8916_l2",	PMIC_VREG_LDO,   1200000, 1200000}, /* LDO_L2   */
	{"8916_l3",	PMIC_VREG_LDO,    750000, 1375000}, /* LDO_L3   */
	{"8916_l4",	PMIC_VREG_LDO,   1800000, 2100000}, /* LDO_L4   */
	{"8916_l5",	PMIC_VREG_LDO,   1800000, 1800000}, /* LDO_L5   */
	{"8916_l6",	PMIC_VREG_LDO,   1800000, 1800000}, /* LDO_L6   */
	{"8916_l7",	PMIC_VREG_LDO,   1800000, 1900000}, /* LDO_L7   */
	{"8916_l8",	PMIC_VREG_LDO,   2900000, 2900000}, /* LDO_L8   */
	{"8916_l9",	PMIC_VREG_LDO,   3300000, 3300000}, /* LDO_L9   */
	{"8916_l10",	PMIC_VREG_LDO,   2800000, 2800000}, /* LDO_L10  */
	{"8916_l11",	PMIC_VREG_LDO,   2950000, 2950000}, /* LDO_L11  */
	{"8916_l12",	PMIC_VREG_LDO,   1800000, 2950000}, /* LDO_L12  */
	{"8916_l13",	PMIC_VREG_LDO,   3075000, 3075000}, /* LDO_L13  */
	{"8916_l14",	PMIC_VREG_LDO,   1800000, 3300000}, /* LDO_L14  */
	{"8916_l15",	PMIC_VREG_LDO,   1800000, 3300000}, /* LDO_L15  */
	{"8916_l16",	PMIC_VREG_LDO,   1800000, 3300000}, /* LDO_L16  */
	{"8916_l17",	PMIC_VREG_LDO,   2850000, 2850000}, /* LDO_L17  */
	{"8916_l18",	PMIC_VREG_LDO,   2700000, 2700000}, /* LDO_L18  */
};

/*----------------------------------------------------------------------------*/
/*                            GPIO                                            */
/*----------------------------------------------------------------------------*/
static int gpio_diag_set_value(int gpio_no, int value)
{
	int rc = 0;

	printk(DIAG_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);

	rc = gpio_request(gpio_no, "diag");
	if (rc) {
		if (rc == -EBUSY)
			printk(DIAG_LOG_01 "[%04d]:%s() gpio_request BUSY\n",
							__LINE__, __func__);
		else {
			printk(DIAG_LOG_01 "[%04d]:%s() gpio_request ERR\n",
							__LINE__, __func__);
			return -EFAULT;
		}
	}

	gpio_set_value_cansleep(gpio_no, value);
	if (!rc)
		gpio_free(gpio_no);
	printk(DIAG_LOG_03 "[%04d]:%s() end.\n", __LINE__, __func__);
	return 0;
}

static int gpio_diag_get_value(int gpio_no)
{
	int value = -1;
	int rc = 0;

	printk(DIAG_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);

	rc = gpio_request(gpio_no, "diag");
	if (rc) {
		if (rc == -EBUSY)
			printk(DIAG_LOG_01 "[%04d]:%s() gpio_request BUSY\n",
							__LINE__, __func__);
		else {
			printk(DIAG_LOG_01 "[%04d]:%s() gpio_request ERR\n",
							__LINE__, __func__);
			return -EFAULT;
		}
	}

	value = gpio_get_value_cansleep(gpio_no);
	if (!rc)
		gpio_free(gpio_no);
	printk(DIAG_LOG_03 "[%04d]:%s() end. value = %d\n", __LINE__,
							__func__, value);
	return value;
}

static int gpio_diag(unsigned long arg)
{
	int rc = 0;
	str_diag_req req;
	str_diag_res res;
	printk(DIAG_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);
	if (copy_from_user(&req, (void *)arg, sizeof(req))) {
		printk(DIAG_LOG_01 "[%04d]:%s() copy_from_user ERR\n",
							__LINE__, __func__);
		return -EINVAL;
	}
	printk(DIAG_LOG_06 "[%04d]:%s() cmd = 0x%08X, port_no = %d\n",
				__LINE__, __func__, req.cmd, req.arg1);
	memset(&res, 0x00, sizeof(res));

	switch (req.cmd) {
	case PMIC_GPIO_SET_VALUE:
		rc = gpio_diag_set_value(req.arg1, req.arg2);
		break;

	case PMIC_GPIO_GET_VALUE:
		res.data1 = gpio_diag_get_value(req.arg1);
		if (res.data1 < 0) {
			printk(DIAG_LOG_01 "[%04d]:%s() res.data1 < 0\n",
							__LINE__, __func__);
			return -EINVAL;
		}
		break;

	default:
		return -EINVAL;
	}
	if (copy_to_user((void *)arg, &res, sizeof(res))) {
		printk(DIAG_LOG_01 "[%04d]:%s() copy_to_user ERR\n",
							__LINE__, __func__);
		return -EINVAL;
	}
	printk(DIAG_LOG_03 "[%04d]:%s() end. rc = %d\n",
							__LINE__, __func__, rc);
	return rc;
}

/*----------------------------------------------------------------------------*/
/*                            VREG                                            */
/*----------------------------------------------------------------------------*/
static int vreg_diag_enable(int vreg_no)
{
	printk(DIAG_LOG_06 "[%04d]:%s() start.  vreg_no = %d\n", __LINE__,
							__func__, vreg_no);
	vreg_no -= 1;
	if (!reg_store[vreg_no]) {
		reg_store[vreg_no] = regulator_get(NULL, vreg_data[vreg_no].id);
		if (IS_ERR(reg_store[vreg_no])) {
			reg_store[vreg_no] = NULL;
			printk(DIAG_LOG_01 "[%04d]:%s() ERR\n",
							__LINE__, __func__);
			return -ENODEV;
		}
	}
	if (vreg_data[vreg_no].vreg_type != PMIC_VREG_VS) {
		if (!reg_volt_set_check[vreg_no]) {
			if (regulator_set_voltage(reg_store[vreg_no],
				vreg_data[vreg_no].min_uV,
				vreg_data[vreg_no].max_uV)) {
				printk(DIAG_LOG_01 "[%04d]:%s() ERR\n",
							__LINE__, __func__);
				goto reg_ldo_put;
			}
			reg_volt_set_check[vreg_no] = 1;
		}
	}
	if (regulator_enable(reg_store[vreg_no]))
		goto reg_ldo_disable;
	printk(DIAG_LOG_06 "[%04d]:%s() end.\n", __LINE__, __func__);
	return 0;

reg_ldo_disable:
	if (vreg_data[vreg_no].vreg_type != PMIC_VREG_VS) {
		regulator_set_voltage(reg_store[vreg_no], 0,
						vreg_data[vreg_no].max_uV);
		regulator_force_disable(reg_store[vreg_no]);
	}
	else
		regulator_disable(reg_store[vreg_no]);
reg_ldo_put:
	regulator_put(reg_store[vreg_no]);
	reg_store[vreg_no] = NULL;
	reg_volt_set_check[vreg_no] = 0;
	printk(DIAG_LOG_01 "[%04d]:%s() ERR\n", __LINE__, __func__);
	return -ENODEV;
}
static int vreg_diag_disable(int vreg_no)
{
	printk(DIAG_LOG_06 "[%04d]:%s() start.  vreg_no = %d\n", __LINE__,
							__func__, vreg_no);
	vreg_no -= 1;
	if (!reg_store[vreg_no]) {
		reg_store[vreg_no] = regulator_get(NULL, vreg_data[vreg_no].id);
		if (IS_ERR(reg_store[vreg_no])) {
			reg_store[vreg_no] = NULL;
			printk(DIAG_LOG_01 "[%04d]:%s() ERR\n", __LINE__,
								__func__);
			return -ENODEV;
		}
	}
	if (vreg_data[vreg_no].vreg_type != PMIC_VREG_VS) {
		regulator_set_voltage(reg_store[vreg_no], 0,
						vreg_data[vreg_no].max_uV);
		regulator_force_disable(reg_store[vreg_no]);
	}
	else
		regulator_disable(reg_store[vreg_no]);

	regulator_put(reg_store[vreg_no]);
	reg_store[vreg_no] = NULL;
	reg_volt_set_check[vreg_no] = 0;
	printk(DIAG_LOG_06 "[%04d]:%s() end.\n", __LINE__, __func__);
	return 0;
}

static int vreg_diag_set_volt(int vreg_no, int min_uV, int max_uV)
{
	printk(DIAG_LOG_06 "[%04d]:%s() start.  vreg_no = %d, min_uV = %d,\
		max_uV = %d\n", __LINE__, __func__, vreg_no, min_uV, max_uV);

	vreg_no -= 1;
	if (vreg_data[vreg_no].vreg_type == PMIC_VREG_VS)
		return 0;
	if (!reg_store[vreg_no]) {
		reg_store[vreg_no] = regulator_get(NULL, vreg_data[vreg_no].id);
		if (IS_ERR(reg_store[vreg_no])) {
			reg_store[vreg_no] = NULL;
			printk(DIAG_LOG_01 "[%04d]:%s() ERR\n",
							__LINE__, __func__);
			return -ENODEV;
		}
	}
	regulator_set_voltage(reg_store[vreg_no], min_uV, max_uV);
	reg_volt_set_check[vreg_no] = 1;
	printk(DIAG_LOG_06 "[%04d]:%s() end.\n", __LINE__, __func__);
	return 0;
}

static int vreg_diag_set_mode(int vreg_no, int mode)
{
	int rc = 0;
	printk(DIAG_LOG_06 "[%04d]:%s() start.  vreg_no = %d, mode = %d\n",
					__LINE__, __func__, vreg_no, mode);
	vreg_no -= 1;
	if (!reg_store[vreg_no]) {
		reg_store[vreg_no] = regulator_get(NULL, vreg_data[vreg_no].id);
		if (IS_ERR(reg_store[vreg_no])) {
			reg_store[vreg_no] = NULL;
			printk(DIAG_LOG_01 "[%04d]:%s() ERR\n",
							__LINE__, __func__);
			return -ENODEV;
		}
	}
	rc = regulator_set_mode(reg_store[vreg_no], mode);
	regulator_put(reg_store[vreg_no]);
	reg_store[vreg_no] = NULL;
	printk(DIAG_LOG_03 "[%04d]:%s() end.\n", __LINE__, __func__);
	return rc;
}

#if 0
static int vreg_diag_set_freq(int vreg_no, int freq)
{
	int rc = 0;
	printk(DIAG_LOG_06 "[%04d]:%s() start. vreg_no = %d, freq = %d\n",
					__LINE__, __func__, vreg_no, freq);
	vreg_no -= 1;
	rc = rpm_vreg_set_frequency((enum rpm_vreg_id_8974)vreg_no,
						(enum rpm_vreg_freq)freq);

	printk(DIAG_LOG_03 "[%04d]:%s() end. rc = %d\n",
							__LINE__, __func__, rc);
	return rc;
}
#endif

static int vreg_diag(unsigned long arg)
{
	int rc = 0;
	str_diag_req req;
	str_diag_res res;
	printk(DIAG_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);
	if (copy_from_user(&req, (void *)arg, sizeof(req))) {
		printk(DIAG_LOG_01 "[%04d]:%s() copy_from_user ERR\n",
							__LINE__, __func__);
		return -EINVAL;
	}
	printk(DIAG_LOG_06 "[%04d]:%s() cmd = 0x%08X  vreg_no = %d\n",
				__LINE__, __func__, req.cmd, req.arg1);
	memset(&res, 0x00, sizeof(res));
	switch (req.cmd) {
	case PMIC_VREG_SET_ENABLE:
		rc = vreg_diag_enable(req.arg1);
		break;

	case PMIC_VREG_SET_DISABLE:
		rc = vreg_diag_disable(req.arg1);
		break;

	case PMIC_VREG_SET_VOLT:
		rc = vreg_diag_set_volt(req.arg1, req.arg2, req.arg2);
		break;

	case PMIC_VREG_SET_MODE:
		rc = vreg_diag_set_mode(req.arg1, req.arg2);
		break;
#if 0
	case PMIC_VREG_SET_FREQ:
		rc = vreg_diag_set_freq(req.arg1, req.arg2);
		break;
#endif
	default:
		return -EINVAL;
	}
	if (copy_to_user((void *)arg, &res, sizeof(res))) {
		printk(DIAG_LOG_01 "[%04d]:%s() copy_to_user ERR\n",
							__LINE__, __func__);
		return -EINVAL;
	}
	printk(DIAG_LOG_03 "[%04d]:%s() end.  rc = %d\n", __LINE__,
								__func__, rc);
	return rc;
}

/*----------------------------------------------------------------------------*/
/*                         QPNP                                               */
/*----------------------------------------------------------------------------*/
static int qpnp_pin_diag_config(int arg1, int arg2, int arg3, int arg4)
{
	int rc = 0;
	struct qpnp_pin_cfg pin_config;
	int pin_no, gpio_no, gpio_type;

	gpio_type = 0xFF & arg1;
	pin_no    = 0xFF & (arg1 >> 8);

	/* kernel/include/linux/qpnp/pin.h */
	pin_config.mode		 = 0xFF & arg2;		/* Mode          */
	pin_config.output_type	 = 0xFF & (arg2 >> 8);	/* Output type   */
	pin_config.invert	 = 0xFF & (arg2 >> 16);	/* Invert source */
	pin_config.pull		 = 0xFF & (arg2 >> 24);	/* Pull Up       */
	pin_config.vin_sel	 = 0xFF & arg3; 	/* Voltage       */
	pin_config.out_strength	 = 0xFF & (arg3 >> 8);	/* Out Strength  */
	pin_config.src_sel	 = 0xFF & (arg3 >> 16);	/* Source Select */
	pin_config.master_en	 = 0xFF & (arg3 >> 24);	/* Master enable */
	pin_config.aout_ref	 = 0xFF & arg4;		/* Analog Output */
	pin_config.ain_route	 = 0xFF & (arg4 >> 8);	/* Analog Input  */
	pin_config.cs_out	 = 0xFF & (arg4 >> 16);	/* Current Sink  */

	gpio_no = qpnp_pin_map(pin_type[gpio_type], pin_no);

	printk(DIAG_LOG_06 "[%04d]:%s() %s-%d, gpio_no = %d, mode = %d, \n"
		"output_type = %d, invert = %d, pull = %d, vin_sel = %d,\n"
		"out_strength = %d, src_sel = %d, master_en = %d,"
		"aout_ref = %d, ain_route = %d, cs_out = %d\n",
				__LINE__, __func__, pin_type[gpio_type],
				pin_no, gpio_no, pin_config.mode,
				pin_config.output_type, pin_config.invert,
				pin_config.pull, pin_config.vin_sel,
				pin_config.out_strength, pin_config.src_sel,
				pin_config.master_en, pin_config.aout_ref,
				pin_config.ain_route, pin_config.cs_out);

	if (gpio_no < 0) {
		printk(DIAG_LOG_01 "[%04d]:%s() qpnp_pin_map ERR\n",
							__LINE__, __func__);
		return -EINVAL;
	}

	printk(DIAG_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);

	rc = qpnp_pin_config(gpio_no, &pin_config);

	printk(DIAG_LOG_03 "[%04d]:%s() end. rc = %d\n", __LINE__,
								__func__, rc);
	return rc;
}

static int qpnp_diag_qpnp_pin(unsigned long arg)
{
	int rc = 0;
	str_diag_req req;
	str_diag_res res;

	printk(DIAG_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);
	if (copy_from_user(&req, (void *)arg, sizeof(req))) {
		printk(DIAG_LOG_01 "[%04d]:%s() copy_from_user ERR\n",
							__LINE__, __func__);
		return -EINVAL;
	}
	printk(DIAG_LOG_06 "[%04d]:%s() cmd = 0x%08X\n", __LINE__,
							__func__, req.cmd);
	memset(&res, 0x00, sizeof(res));

	switch (req.cmd) {
	case PMIC_QPNP_PIN_CONFIG:
		rc = qpnp_pin_diag_config(req.arg1, req.arg2, req.arg3,
								req.arg4);
		break;
	default:
		return -EINVAL;
	}

	if (copy_to_user((void *)arg, &res, sizeof(res))) {
		printk(DIAG_LOG_01 "[%04d]:%s() copy_to_user ERR\n",
							__LINE__, __func__);
		return -EINVAL;
	}
	printk(DIAG_LOG_03 "[%04d]:%s() end. rc = %d\n", __LINE__,
								__func__, rc);
	return rc;
}

static int qpnp_diag_spmi(unsigned long arg)
{
	int rc = 0;
	str_diag_req req;
	str_diag_res res;
	u8 val = 0, mask;

	printk(DIAG_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);
	if (copy_from_user(&req, (void *)arg, sizeof(req))) {
		printk(DIAG_LOG_01 "[%04d]:%s() copy_from_user ERR\n",
							__LINE__, __func__);
		return -EINVAL;
	}
	printk(DIAG_LOG_06 "[%04d]:%s() cmd = 0x%08X\n", __LINE__,
							__func__, req.cmd);
	memset(&res, 0x00, sizeof(res));

	switch (req.cmd) {
	case PMIC_SPMI_WRITE:
		val  = 0xFF & req.arg2;
		mask = 0xFF & (req.arg2 >> 8);
		rc = oem_qpnp_diag_masked_write(req.arg1, mask, val);
		break;
	case PMIC_SPMI_READ:
		rc = oem_qpnp_diag_read(req.arg1, &val);
		if (!rc)
			res.data1 = val;
		break;
	default:
		return -EINVAL;
	}

	if (copy_to_user((void *)arg, &res, sizeof(res))) {
		printk(DIAG_LOG_01 "[%04d]:%s() copy_to_user ERR\n",
							__LINE__, __func__);
		return -EINVAL;
	}
	printk(DIAG_LOG_03 "[%04d]:%s() end. rc = %d\n", __LINE__,
								__func__, rc);
	return rc;
}

static ssize_t diag_read(struct file *fp, char __user *buf, size_t count,
								loff_t *pos)
{
	return 0;
}
static ssize_t diag_write(struct file *fp, const char __user *buf,
						size_t count, loff_t *pos)
{
	return count;
}

static int diag_open(struct inode *ip, struct file *fp)
{
	int rc;

	printk(DIAG_LOG_06 "[%04d]:%s() start.\n", __LINE__, __func__);

	rc = nonseekable_open(ip, fp);
	printk(DIAG_LOG_03 "[%04d]:%s() end. rc = %d\n",
							__LINE__, __func__, rc);
	if (rc) {
		return rc;
	}
	return 0;
}

static int diag_release(struct inode *ip, struct file *fp)
{
	return 0;
}

static long diag_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	int rc = 0;

	printk(DIAG_LOG_06 "[%04d]:%s() start.  cmd = 0x%08X\n",
						__LINE__, __func__, cmd);

	switch (cmd) {
	case PMIC_TEST_GPIO:
		rc = gpio_diag(arg);
		break;
	case PMIC_TEST_VREG:
		rc = vreg_diag(arg);
		break;
	case PMIC_TEST_QPNP_PIN: /* QPNP PIN */
		rc = qpnp_diag_qpnp_pin(arg);
		break;
	case PMIC_TEST_SPMI:     /* SPMI */
		rc = qpnp_diag_spmi(arg);
		break;
	default:
		return -EINVAL;
	}

	printk(DIAG_LOG_03 "[%04d]:%s() end. rc = %d\n",
						__LINE__, __func__, rc);
	return rc;
}

static const struct file_operations diag_fops = {
	.owner		 = THIS_MODULE,
	.read		 = diag_read,
	.write		 = diag_write,
	.open		 = diag_open,
	.release 	 = diag_release,
	.unlocked_ioctl	 = diag_ioctl,
};

static struct miscdevice diag_dev = {
	.minor		 = MISC_DYNAMIC_MINOR,
	.name		 = "kc_pmic_test",
	.fops		 = &diag_fops,
};

static int __init diag_init(void)
{
	return misc_register(&diag_dev);
}

module_init(diag_init);

MODULE_DESCRIPTION("PMIC Diag");
MODULE_LICENSE("GPL v2");
