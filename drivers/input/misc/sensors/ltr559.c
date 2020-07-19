/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2019 KYOCERA Corporation
 */
/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <linux/ctype.h>
#include <linux/pm_runtime.h>
#include <linux/device.h>
#include "ltr559.h"
#include <linux/irq.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
#include "sensor_driver.h"
#endif

/*struct sensor_input_info_str {
    struct input_dev* dev;
    void (*param_func)( struct input_dev *dev );
    struct attribute_group* attr_grp;
};
extern struct sensor_input_info_str prox_input_info;
extern struct sensor_input_info_str light_input_info;
*/


#define SENSOR_NAME			"proximity"
#define LTR559_DRV_NAME		"ltr559"
#define LTR559_MANUFAC_ID	0x05

#define VENDOR_NAME				"lite-on"
#define LTR559_SENSOR_NAME		"ltr559als"
#define DRIVER_VERSION		"1.0"

#define SYS_AUTHORITY		(S_IRUGO|S_IWUGO)

#define	WAKE_LOCK_TIME_SCHEDULING	(msecs_to_jiffies(20))
#define	WAKE_LOCK_TIME_DETECT		(msecs_to_jiffies(200))
#define	WAKE_LOCK_TIME_NODETECT		(msecs_to_jiffies(1000))

struct ps_thre {
	int noise;
	int th_hi;
	int th_lo;
};
static struct ps_thre psthre_data[] = {
	{50,  45,  35},
	{100,  45,  35},//{100, 24,  16},
	{200,  45,  35},//{200, 40,  30},
	{400,  45,  35},//{400, 80, 50},
	{1200,  45,  35},//{1200,200, 100},
	{1650,  45,  35},//{1650,200, 100},
};

int noise_cal =0;

struct ltr559_data {

	struct i2c_client *client;

	/* pinctrl data*/
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	struct pinctrl_state *pin_sleep;

	struct ltr559_platform_data *platform_data;

	/* regulator data */
	bool power_on;
	bool power_state;
	struct regulator *vdd;
	struct regulator *vio;

	/* interrupt type is level-style */
	struct mutex lockw;
	struct mutex op_lock;

	struct delayed_work ps_work;
	struct delayed_work als_work;

	struct wake_lock wake_lock;

	u8 ps_open_state;
	u8 als_open_state;

	u16 irq;
	u16 intr_gpio;

	u32 ps_state;
	u32 last_lux;
	

	u16	prox_thres_hi;
	u16	prox_thres_lo;
	
	bool cali_update;
	u32 dynamic_noise;
};

struct ltr559_reg {
	const char *name;
	u8 addr;
	u16 defval;
	u16 curval;
};

enum ltr559_reg_tbl{
	REG_ALS_CONTR,
	REG_PS_CONTR,
	REG_ALS_PS_STATUS,
	REG_INTERRUPT,
	REG_PS_LED,
	REG_PS_N_PULSES,
	REG_PS_MEAS_RATE,
	REG_ALS_MEAS_RATE,
	REG_MANUFACTURER_ID,
	REG_INTERRUPT_PERSIST,
	REG_PS_THRES_LOW,
	REG_PS_THRES_UP,
	REG_ALS_THRES_LOW,
	REG_ALS_THRES_UP,
	REG_ALS_DATA_CH1,
	REG_ALS_DATA_CH0,
	REG_PS_DATA
};
struct ltr559_data *g_ltr559_data;
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
static struct ltr559_data *client_ltr559_data = NULL;
#endif

// "/dev/block/by-name/oemdata -> /dev/block/mmcblk0p48"
#define OEMDATA_PARTITION 			"/dev/block/mmcblk0p48"
#define OEM_FLAG					0xA5B6C7D8
#define PS_CAL_FLAG					0x23DD99BC

struct ps_cal_for_partition_data{
	u32 ps_cal_flag;
	int ps_cal_partition_val;
	u8 reserve [8];
};

struct oem_data {
	u32 oem_flag;
	struct ps_cal_for_partition_data ps_p_val;
	u8 reserve[492];
};

static int ps_doned_cal = 0;

static int ltr559_als_set_enable(struct ltr559_data *data,
		unsigned int enable);
static int ltr559_ps_set_enable(struct ltr559_data *data,
		unsigned int enable);
static ssize_t ltr559_ps_dynamic_caliberate(struct ltr559_data *data);
int ltr559_device_init(struct i2c_client *client);

static  struct ltr559_reg reg_tbl[] = {
		{
				.name   = "ALS_CONTR",
				.addr   = 0x80,
				.defval = 0x00,
				.curval = 0x19,
		},
		{
				.name = "PS_CONTR",
				.addr = 0x81,
				.defval = 0x00,
				.curval = 0x03,
		},
		{
				.name = "ALS_PS_STATUS",
				.addr = 0x8c,
				.defval = 0x00,
				.curval = 0x00,
		},
		{
				.name = "INTERRUPT",
				.addr = 0x8f,
				.defval = 0x00,
				.curval = 0x01,
		},
		{
				.name = "PS_LED",
				.addr = 0x82,
				.defval = 0x7f,
				.curval = 0x7a,
		},
		{
				.name = "PS_N_PULSES",
				.addr = 0x83,
				.defval = 0x01,
				.curval = 0x0c,
		},
		{
				.name = "PS_MEAS_RATE",
				.addr = 0x84,
				.defval = 0x02,
				.curval = 0x00,
		},
		{
				.name = "ALS_MEAS_RATE",
				.addr = 0x85,
				.defval = 0x03,
				.curval = 0x02,	/* 200ms */
		},
		{
				.name = "MANUFACTURER_ID",
				.addr = 0x87,
				.defval = 0x05,
				.curval = 0x05,
		},
		{
				.name = "INTERRUPT_PERSIST",
				.addr = 0x9e,
				.defval = 0x00,
				.curval = 0x23,
		},
		{
				.name = "PS_THRES_LOW",
				.addr = 0x92,
				.defval = 0x0000,
				.curval = 0x0200,
		},
		{
				.name = "PS_THRES_UP",
				.addr = 0x90,
				.defval = 0x07ff,     //0xff  datasheet
				.curval = 0x0300,
		},
		{
				.name = "ALS_THRES_LOW",
				.addr = 0x99,
				.defval = 0x0000,
				.curval = 0x0000,
		},
		{
				.name = "ALS_THRES_UP",
				.addr = 0x97,
				.defval = 0xffff,
				.curval = 0x0000,
		},
		{
				.name = "ALS_DATA_CH1",
				.addr = 0x88,
				.defval = 0x0000,
				.curval = 0x0000,
		},
		{
				.name = "ALS_DATA_CH0",
				.addr = 0x8a,
				.defval = 0x0000,
				.curval = 0x0000,
		},
		{
				.name = "PS_DATA",
				.addr = 0x8d,
				.defval = 0x0000,
				.curval = 0x0000,
		},
};

#define I2C_RETRY_MAX		5

static int ltr559_i2c_read_byte_data(struct i2c_client *client, uint8_t addr)
{
	int read_val = 0;
	int retry = I2C_RETRY_MAX + 1;

	while(retry--){
		read_val = i2c_smbus_read_byte_data(client, addr);
		if(read_val >= 0) {
			pr_debug("%s(%d) read data : [0x%08x]", __func__, __LINE__, read_val);
			break;
		} else {
			if(retry == 0) {
				pr_err("i2c transfer error, retry it multiple times still error\n");
				break;
			}
			usleep_range(3000, 5000);
		}
	}
	return read_val;
}
static int ltr559_i2c_read_word_data(struct i2c_client *client, uint8_t addr)
{
	int read_val = 0;
	int retry = I2C_RETRY_MAX + 1;

	while(retry--){
		read_val = i2c_smbus_read_word_data(client, addr);
		if(read_val >= 0) {
			pr_debug("%s(%d) read data : [0x%08x]", __func__, __LINE__, read_val);
			break;
		} else {
			if(retry == 0) {
				pr_err("%s i2c transfer error, retry it multiple times still error\n", __func__);
				break;
			}
			usleep_range(3000, 5000);
		}
	}
	return read_val;
}

static int ltr559_i2c_write_byte_data(struct i2c_client *client, uint8_t addr, uint8_t val)
{
	int ret = 0;
	int retry = I2C_RETRY_MAX + 1;

	while(retry--){
		ret = i2c_smbus_write_byte_data(client, addr, val);
		if(ret == 0){
			pr_debug("%s(%d) write success", __func__, __LINE__);
			break;
		} else {
			if(retry == 0) {
				pr_err("%s i2c transfer error, retry it multiple times still error\n", __func__);
				break;
			}
			usleep_range(3000, 5000);
		}
	}
	return ret;
}

static int ltr559_i2c_write_word_data(struct i2c_client *client, uint8_t addr, uint16_t val)
{
	int ret = 0;
	int retry = I2C_RETRY_MAX + 1;

	while(retry--){
		ret = i2c_smbus_write_word_data(client, addr, val);
		if(ret == 0){
			pr_debug("%s(%d) write success", __func__, __LINE__);
			break;
		} else {
			if(retry == 0) {
				pr_err("%s i2c transfer error, retry it multiple times still error\n", __func__);
				break;
			}
			usleep_range(3000, 5000);
		}
	}
	return ret;
}


static int handle_partition_info(const char *filename, char *buf, loff_t offset, int length, bool flag)
{
	struct file *filep;
	mm_segment_t old_fs;
	loff_t pos;
	int ret = 0;

	filep= filp_open(filename, O_RDWR, 0);
	if(IS_ERR(filep))
	{
		printk(KERN_CRIT"open %s err!\n",filename);
		return -ENOENT;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	pos = 0;
	if(flag==0){
		if(filep->f_op->read){
			ret = filep->f_op->read(filep, buf, length, &pos);
		}else{
			ret = vfs_read(filep, buf, length, &pos);
		}
	}else{
		if(filep->f_op->write){
			ret = filep->f_op->write(filep, buf, length, &pos);
		}else{
			ret = vfs_write(filep, buf, length, &pos);
		}
	}
	
	set_fs(old_fs);
	filp_close(filep, NULL);

	return ret;
}

static int get_part_ps_calibration_val(void)
{
	char *p_buf = NULL;
	struct oem_data *oem_data_buf;
	int ret = -1;
	int part_len = sizeof(struct oem_data);
	int ps_cal_val = 0;

	p_buf = kmalloc(part_len, GFP_KERNEL);
	if(!p_buf){
		printk("[ltr559] p_buf malloc failed !\n");
		return -ENOMEM;
	}
	
	memset(p_buf, 0, part_len);
	ret = handle_partition_info(OEMDATA_PARTITION, p_buf, 0, part_len, 0);
	if(ret != part_len){
		printk("[ltr559] read partition failed ! ret = %d part_len = %d\n", ret, part_len);
		return ret;
	}
	
	oem_data_buf = (struct oem_data *)p_buf;
	if(oem_data_buf->oem_flag && (oem_data_buf->oem_flag == OEM_FLAG) && oem_data_buf->ps_p_val.ps_cal_flag
			&& (oem_data_buf->ps_p_val.ps_cal_flag == PS_CAL_FLAG)){
		ps_cal_val = oem_data_buf->ps_p_val.ps_cal_partition_val;
		ps_doned_cal = 1;
	}else{
		printk("[ltr559] ps is not calibrated !\n");
	}
	
	kfree(p_buf);

	return ps_cal_val;
}

static int ltr559_regulator_configure(struct ltr559_data *data, bool on)
{
	int rc;

	if (!on) {

		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
				LTR559_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0,
				LTR559_VIO_MAX_UV);

		regulator_put(data->vio);
	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			dev_err(&data->client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd,
				LTR559_VDD_MIN_UV, LTR559_VDD_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,
					"Regulator set failed vdd rc=%d\n",
					rc);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			dev_err(&data->client->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio,
				LTR559_VIO_MIN_UV, LTR559_VIO_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,
				"Regulator set failed vio rc=%d\n", rc);
				goto reg_vio_put;
			}
		}
	}

	return 0;
reg_vio_put:
	regulator_put(data->vio);

reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, LTR559_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}


static int ltr559_regulator_power_on(struct ltr559_data *data, bool on)
{
	int rc = 0;

	if (!on) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			rc = regulator_enable(data->vdd);
			dev_err(&data->client->dev,
					"Regulator vio re-enabled rc=%d\n", rc);
			/*
			 * Successfully re-enable regulator.
			 * Enter poweron delay and returns error.
			 */
			if (!rc) {
				rc = -EBUSY;
				goto enable_delay;
			}
		}
		return rc;
	} else {
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			regulator_disable(data->vdd);
			return rc;
		}
	}

enable_delay:
	msleep(130);
	dev_dbg(&data->client->dev,
		"Sensor regulator power on =%d\n", on);
	return rc;
}


static int ltr559_platform_hw_power_on(bool on)
{
	struct ltr559_data *data;
	int err = 0;

	if (g_ltr559_data == NULL)
		return -ENODEV;

	data = g_ltr559_data;
	if (data->power_on != on) {
		err = ltr559_regulator_power_on(data, on);
		if (err)
			dev_err(&data->client->dev,
					"Can't configure regulator!\n");
		else
			data->power_on = on;
	}

	return err;
}


static int ltr559_platform_hw_init(void)
{
	struct i2c_client *client;
	struct ltr559_data *data;
	int error;

	if (g_ltr559_data == NULL)
		return -ENODEV;

	data = g_ltr559_data;
	client = data->client;

	error = ltr559_regulator_configure(data, true);
	if (error < 0) {
		dev_err(&client->dev, "unable to configure regulator\n");
		return error;
	}

	return 0;
}
static void ltr559_platform_hw_exit(void)
{
	struct ltr559_data *data = g_ltr559_data;

	if (data == NULL)
		return;

	ltr559_regulator_configure(data, false);

}
static int ltr559_ps_read(struct i2c_client *client)
{
	int psdata;

	psdata = ltr559_i2c_read_word_data(client,LTR559_PS_DATA_0);

	return psdata;
}

static int ltr559_chip_reset(struct i2c_client *client)
{
	int ret;

	ret = ltr559_i2c_write_byte_data(client, LTR559_PS_CONTR, MODE_PS_StdBy);/* ps standby mode */
	if(ret < 0)
	{
		dev_err(&client->dev, "%s write error\n",__func__);
		return ret;
	}
	ret = ltr559_i2c_write_byte_data(client, LTR559_ALS_CONTR, 0x02);    /* reset */
	if(ret < 0)
	{
		dev_err(&client->dev, "%s reset chip fail\n",__func__);
		return ret;
	}

	return ret;
}

static void ltr559_set_ps_threshold(struct i2c_client *client, u8 addr, u16 value)
{
	ltr559_i2c_write_word_data(client, addr, value );
}

static int ltr559_ps_enable(struct i2c_client *client, int on)
{
	struct ltr559_data *data = i2c_get_clientdata(client);
	int ret=0;
	int contr_data;
	int i, ps_cal = 0;

	/*Load p-sensor calibration data*/
	if(!ps_doned_cal){
		ps_cal = get_part_ps_calibration_val();
		if(ps_doned_cal && (ps_cal >= 0) && (ps_cal < 2000)){
			for (i = 0; i < 6; i++){
				psthre_data[i].th_hi = ps_cal;
				psthre_data[i].th_lo = ps_cal -10;
			}
		}
	}

	if (on) {

		ret = ltr559_device_init(client);

		ltr559_set_ps_threshold(client, LTR559_PS_THRES_LOW_0, 0);
		ltr559_set_ps_threshold(client, LTR559_PS_THRES_UP_0, data->platform_data->prox_threshold);
		ret = ltr559_i2c_write_byte_data(client, LTR559_PS_CONTR, reg_tbl[REG_PS_CONTR].curval);
		if(ret<0){
			pr_err("%s: enable=(%d) failed!\n", __func__, on);
			return ret;
		}
		contr_data = ltr559_i2c_read_byte_data(client, LTR559_PS_CONTR);
		if(contr_data != reg_tbl[REG_PS_CONTR].curval){
			pr_err("%s: enable=(%d) failed!\n", __func__, on);
			return -EFAULT;
		}

		usleep_range(WAKEUP_DELAY * 1000, WAKEUP_DELAY * 1000);

		data->ps_state = 1;
		//input_report_abs(data->input_dev_ps, ABS_DISTANCE, data->ps_state);
		ltr559_ps_dynamic_caliberate(data);


	} else {
		ret = ltr559_i2c_write_byte_data(client, LTR559_PS_CONTR, MODE_PS_StdBy);
		if(ret<0){
			pr_err("%s: enable=(%d) failed!\n", __func__, on);
			return ret;
		}
		contr_data = ltr559_i2c_read_byte_data(client, LTR559_PS_CONTR);
		if(contr_data != reg_tbl[REG_PS_CONTR].defval){
			pr_err("%s:  enable=(%d) failed!\n", __func__, on);
			return -EFAULT;
		}
	}
	printk("%d: gaodazhuang enable=(%d) OK\n", __LINE__, on);
	return ret;
}

/*
 * Ambient Light Sensor Congfig
 */
static int ltr559_als_enable(struct i2c_client *client, int on)
{
	struct ltr559_data *data = i2c_get_clientdata(client);
	int ret;

	if (on) {
		printk("%d: gaodazhuang enable=(%d)\n", __LINE__, on);
		ret = ltr559_i2c_write_byte_data(client, LTR559_ALS_CONTR, reg_tbl[REG_ALS_CONTR].curval);
		if(ret < 0)
		{
			printk("%s gaodazhuang write error\n",__func__);
			return ret;
		}
		usleep_range(WAKEUP_DELAY * 1000, WAKEUP_DELAY * 1000);
		ret |= ltr559_i2c_read_byte_data(client, LTR559_ALS_DATA_CH0_1);
		cancel_delayed_work_sync(&data->als_work);
		schedule_delayed_work(&data->als_work,msecs_to_jiffies(data->platform_data->als_poll_interval));
	} else {
		cancel_delayed_work_sync(&data->als_work);
		ret = ltr559_i2c_write_byte_data(client, LTR559_ALS_CONTR, MODE_ALS_StdBy);
		if(ret < 0)
		{
			dev_err(&client->dev, "%s write error\n",__func__);
			return ret;
		}
		printk("%s gaodazhuang write error\n",__func__);
	}

	printk("%d: gaodazhuang enable=(%d) ret=%d\n", __LINE__, on, ret);
	return ret;
}

static int ltr559_als_read(struct i2c_client *client)
{
		int alsval_ch0;
		int alsval_ch1;
		int luxdata;
		int ch1_co, ch0_co, ratio;
		int ch0_c[4] = {17743,42785,5926,0};
		int ch1_c[4] = {-11059,19548,-1185,0};

		alsval_ch1 = ltr559_i2c_read_word_data(client, LTR559_ALS_DATA_CH1_0);//0x88
		if (alsval_ch1 < 0)
				return -EIO;

		alsval_ch0 = ltr559_i2c_read_word_data(client, LTR559_ALS_DATA_CH0_0);//0x8a
		if (alsval_ch0 < 0 )
				return -EIO;

		if ((alsval_ch0 + alsval_ch1) == 0){
				ratio = 1000;
		}else{
			ratio = alsval_ch1 * 1000 / (alsval_ch1 + alsval_ch0);
		}

		if (ratio < 450) {
				ch0_co = ch0_c[0];
				ch1_co = ch1_c[0];
		} else if ((ratio >= 450) && (ratio < 640)) {
				ch0_co = ch0_c[1];
				ch1_co = ch1_c[1];
		} else if ((ratio >= 640) && (ratio < 850)) {
				ch0_co = ch0_c[2];
				ch1_co = ch1_c[2];
		} else if (ratio >= 850) {
				ch0_co = ch0_c[3];
				ch1_co = ch1_c[3];
		}
		luxdata = (int)(alsval_ch0 * ch0_co - alsval_ch1 * ch1_co) / 65000;
		return luxdata;
}


static void ltr559_ps_work_func(struct work_struct *work)
{
	struct ltr559_data *data = container_of(work, struct ltr559_data, ps_work.work);
	struct i2c_client *client=data->client;
	int als_ps_status;
	int psdata;
	static u32 ps_state_last = 1;
	//int j = 0;
	long wake_lock_time;

	mutex_lock(&data->op_lock);
	/*in the case of transfer error, retry it multiple times*/
	als_ps_status = ltr559_i2c_read_byte_data(client, LTR559_ALS_PS_STATUS);
	if (als_ps_status < 0){
		pr_err("failed. i2c transfer error\n");
		goto workout;
	}
	/* Here should check data status,ignore interrupt status. */
	/* Bit 0: PS Data
	 * Bit 1: PS interrupt
	 * Bit 2: ASL Data
	 * Bit 3: ASL interrupt
	 * Bit 4: ASL Gain 0: ALS measurement data is in dynamic range 2 (2 to 64k lux)
	 *                 1: ALS measurement data is in dynamic range 1 (0.01 to 320 lux)
	 */
	if ((data->ps_open_state == 1) && (als_ps_status & 0x02)) {
		psdata = ltr559_i2c_read_word_data(client,LTR559_PS_DATA_0);
		if (psdata < 0) {
				goto workout;
		}
		if(psdata >= data->platform_data->prox_threshold){
			data->ps_state = 0;    /* near */
			ltr559_set_ps_threshold(client, LTR559_PS_THRES_LOW_0, data->platform_data->prox_hsyteresis_threshold);
			ltr559_set_ps_threshold(client, LTR559_PS_THRES_UP_0, 0x07ff);
		} else if (psdata <= data->platform_data->prox_hsyteresis_threshold){
			data->ps_state = 1;    /* far */
			#if 0 //delete because factory p-sensor calibration is enough
			/*dynamic calibration */
			if (data->dynamic_noise > 20 && psdata < (data->dynamic_noise - 50) ) {
				data->dynamic_noise = psdata;
				for(j=0; j<ARRAY_SIZE(psthre_data); j++) {
					if(psdata < psthre_data[j].noise) {
						data->platform_data->prox_threshold = psdata + psthre_data[j].th_hi;
						data->platform_data->prox_hsyteresis_threshold = psdata + psthre_data[j].th_lo;
						break;
					}
				}
				if(j == ARRAY_SIZE(psthre_data)) {
					data->platform_data->prox_threshold = 1700;
					data->platform_data->prox_hsyteresis_threshold = 1680;
					pr_err("ltr559 the proximity sensor rubber or structure is error!\n");
				}
			}
			#endif
			ltr559_set_ps_threshold(client, LTR559_PS_THRES_LOW_0, 0);
			ltr559_set_ps_threshold(client, LTR559_PS_THRES_UP_0, data->platform_data->prox_threshold);
		} else {
			data->ps_state = ps_state_last;
		}
		if((ps_state_last != data->ps_state) || (data->ps_state == 0) || (data->ps_state == 1))
		{
			wake_lock_time = data->ps_state ? WAKE_LOCK_TIME_NODETECT : WAKE_LOCK_TIME_DETECT;
			wake_lock_timeout(&data->wake_lock, wake_lock_time);
			#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
			sensor_interrupt(SENSOR_PROX, !data->ps_state);
			#else
			input_report_abs(prox_input_info.dev, ABS_DISTANCE, data->ps_state);
			input_sync(prox_input_info.dev);
			#endif
			pr_info("%s, report ABS_DISTANCE=%s\n",__func__, data->ps_state ? "far" : "near");
			ps_state_last = data->ps_state;
		}
		else
			printk("%s, ps_state still %s\n", __func__, data->ps_state ? "far" : "near");
	}
workout:
	enable_irq(data->irq);
	mutex_unlock(&data->op_lock);
}

static void ltr559_als_work_func(struct work_struct *work)
{
	struct ltr559_data *data = container_of(work, struct ltr559_data, als_work.work);
	struct i2c_client *client=data->client;
	int als_ps_status;
	int als_data;

	mutex_lock(&data->op_lock);

	if(!data->als_open_state)
		goto workout;

	als_ps_status = ltr559_i2c_read_byte_data(client, LTR559_ALS_PS_STATUS);
	if (als_ps_status < 0)
		goto workout;


	if ((data->als_open_state == 1) && (als_ps_status & 0x04)) {
		als_data = ltr559_als_read(client);
		if (als_data > 50000)
			als_data = 50000;

		if ((als_data >= 0) && (als_data != data->last_lux)) {
			data->last_lux = als_data;
			#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
			sensor_report_data(SENSOR_LIGHT, als_data);
			#else
			input_report_abs(light_input_info.dev, ABS_MISC, als_data);
			input_sync(light_input_info.dev);
			#endif
		}
	}

	schedule_delayed_work(&data->als_work,msecs_to_jiffies(data->platform_data->als_poll_interval));
workout:
	mutex_unlock(&data->op_lock);
}

static irqreturn_t ltr559_irq_handler(int irq, void *arg)
{
	int ret;
	struct ltr559_data *data = (struct ltr559_data *)arg;
	ret = gpio_get_value(data->intr_gpio);
	if (NULL == data)
		return IRQ_HANDLED;
	disable_irq_nosync(data->irq);
	wake_lock_timeout(&data->wake_lock, WAKE_LOCK_TIME_SCHEDULING);
	schedule_delayed_work(&data->ps_work, 0);
	return IRQ_HANDLED;
}

static int ltr559_gpio_irq(struct ltr559_data *data)
{
	    int ret;

        data->intr_gpio = of_get_named_gpio_flags(data->client->dev.of_node, "qcom,irq-gpio", 0, NULL);
        if (!gpio_is_valid(data->intr_gpio)){  
               printk(KERN_ERR"invalid gpio : %d\n", data->intr_gpio);  
               goto ltr559_gpio_free;  
        }
        ret = gpio_request(data->intr_gpio, "ltr559_irq_pin");  
        if (ret != 0) {  
                       printk(KERN_ERR"gaodazhuang ltr559 request hall_switch gpio : %d failed \n", data->intr_gpio);
                       goto ltr559_free_data;  
               }  
        data->irq = gpio_to_irq(data->intr_gpio);
        ret = request_threaded_irq(data->irq, NULL, ltr559_irq_handler,IRQF_TRIGGER_FALLING | IRQ_TYPE_LEVEL_LOW | IRQF_ONESHOT, LTR559_DRV_NAME, data);
        if(ret != 0){
               printk(KERN_ERR "gaodazhuang hall switch request irq failed! \n");
               goto ltr559_free_data;
               }
	device_init_wakeup(&data->client->dev, 1);
	wake_lock_init(&data->wake_lock, WAKE_LOCK_SUSPEND, "ps_sensor");

	//irq_set_irq_wake(data->irq, 1);
        return 0;

ltr559_gpio_free:
               gpio_free(data->intr_gpio); 
                return ret; 
ltr559_free_data:
               kfree(data);
                return ret; 
}


static void ltr559_gpio_irq_free(struct ltr559_data *data)
{
	free_irq(data->irq, data);
	gpio_free(data->intr_gpio);
}

static ssize_t ltr559_show_enable_ps(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ltr559_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", data->ps_open_state);
}

static ssize_t ltr559_store_enable_ps(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	/* If proximity work,then ALS must be enable */
	unsigned long val;
	char *after;
	struct ltr559_data *data = dev_get_drvdata(dev);

	val = simple_strtoul(buf, &after, 10);

	printk(KERN_INFO "enable 559 PS sensor -> %ld\n", val);

	mutex_lock(&data->lockw);
	ltr559_ps_set_enable(data, (unsigned int)val);
	mutex_unlock(&data->lockw);

	return size;
}

static ssize_t ltr559_show_enable_als(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ltr559_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", data->als_open_state);
}

static ssize_t ltr559_store_enable_als(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	/* If proximity work,then ALS must be enable */
	unsigned long val;
	char *after;
	struct ltr559_data *data = dev_get_drvdata(dev);

	val = simple_strtoul(buf, &after, 10);

	printk(KERN_INFO "enable 559 ALS sensor -> %ld\n", val);

	mutex_lock(&data->lockw);
	ltr559_als_set_enable(data, (unsigned int)val);
	mutex_unlock(&data->lockw);
	return size;
}

static ssize_t ltr559_driver_info_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
		return sprintf(buf, "Chip: %s %s\nVersion: %s\n",
						VENDOR_NAME, LTR559_SENSOR_NAME, DRIVER_VERSION);
}

static ssize_t ltr559_show_debug_regs(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 val,high,low;
	int i;
	char *after;
	struct ltr559_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	after = buf;

	after += sprintf(after, "%-17s%5s%14s%16s\n", "Register Name", "address", "default", "current");
	for (i = 0; i < sizeof(reg_tbl)/sizeof(reg_tbl[0]); i++) {
			if (reg_tbl[i].name == NULL || reg_tbl[i].addr == 0) {
					break;
			}
			if (i < 10) {
					val = ltr559_i2c_read_byte_data(client, reg_tbl[i].addr);
					after += sprintf(after, "%-20s0x%02x\t  0x%02x\t\t  0x%02x\n", reg_tbl[i].name, reg_tbl[i].addr, reg_tbl[i].defval, val);
			} else {
					low = ltr559_i2c_read_byte_data(client, reg_tbl[i].addr);
					high = ltr559_i2c_read_byte_data(client, reg_tbl[i].addr+1);
					after += sprintf(after, "%-20s0x%02x\t0x%04x\t\t0x%04x\n", reg_tbl[i].name, reg_tbl[i].addr, reg_tbl[i].defval,(high << 8) + low);
			}
	}
	after += sprintf(after, "\nYou can echo '0xaa=0xbb' to set the value 0xbb to the register of address 0xaa.\n ");

	return (after - buf);
}

static ssize_t ltr559_store_debug_regs(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	/* If proximity work,then ALS must be enable */
	char *after, direct;
	u8 addr, val;
	struct ltr559_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	addr = simple_strtoul(buf, &after, 16);
	direct = *after;
	val = simple_strtoul((after+1), &after, 16);

	if (!((addr >= 0x80 && addr <= 0x93)
				|| (addr >= 0x97 && addr <= 0x9e)))
		return -EINVAL;

	mutex_lock(&data->lockw);
	if (direct == '=')
		ltr559_i2c_write_byte_data(client, addr, val);
	else
		printk("%s: register(0x%02x) is: 0x%02x\n", __func__, addr, ltr559_i2c_read_byte_data(client, addr));
	mutex_unlock(&data->lockw);

	return (after - buf);
}

static ssize_t ltr559_show_adc_data(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ltr559_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int ps_data;
	ps_data = ltr559_i2c_read_word_data(client, LTR559_PS_DATA_0);
	//ps_data = data->ps_state;
	if (ps_data < 0)
		return -EINVAL;
	else
		return sprintf(buf, "%d\n", ps_data);
}

static ssize_t ltr559_show_lux_data(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int lux;
	struct ltr559_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;

	lux = ltr559_als_read(client);

	return sprintf(buf, "%d\n", lux);
}

static ssize_t ltr559_show_ps_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ltr559_data *data = dev_get_drvdata(dev);
	int ps_status;
	ps_status = data->ps_state;
	if (ps_status < 0)
		return -EINVAL;
	else
		return sprintf(buf, "%d\n", ps_status);
}

static ssize_t ltr559_show_psthre_data(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i;
	char *after;
	after = buf;
	after += sprintf(after, "%-10s%s%8s\n", "noise", "th_hi", "th_lo");
	for (i = 0; i < 6; i++)
		after += sprintf(after, "%d\t  %d\t  %d\n", psthre_data[i].noise, psthre_data[i].th_hi, psthre_data[i].th_lo);
	return (after - buf);
}

static ssize_t ltr559_store_psthre_data(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long val;
	int error, i;
	error = kstrtoul(buf, 10, &val);
	if (error)
		return error;
	if (val < 0)
		return -EINVAL;
	for (i = 0; i < 6; i++){
	psthre_data[i].th_hi = (int) val;
	psthre_data[i].th_lo = (int) val -10;
	}
	return size;
}

static ssize_t ltr559_show_cal_value(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ltr559_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int ps_data, cal_value;
	ps_data = ltr559_i2c_read_word_data(client, LTR559_PS_DATA_0);
	cal_value = ps_data - noise_cal;
	if (cal_value < 0)
		return -EINVAL;
	else
		return sprintf(buf, "%d\n", cal_value);
}

static ssize_t ltr559_ps_calibration_partition_val_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int cal_val = 0;
	
	cal_val = get_part_ps_calibration_val();
	if(cal_val < 0)
		cal_val = 0;
	
	return sprintf(buf, "%d\n", cal_val);
}

static ssize_t ltr559_ps_calibration_partition_val_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	unsigned long ps_cal_val;
	char *p_buf = NULL;
	struct oem_data *oem_data_buf;
	int ret = -1;
	int part_len = sizeof(struct oem_data);
	int i;

	ret = kstrtoul(buf, 10, &ps_cal_val);
	if (ret)
		return ret;

	if ((ps_cal_val < 0) || (ps_cal_val > 2000)){
		printk("[ltr559] ps calibration val invalid!\n");
		return -EINVAL;
	}

	p_buf = kmalloc(part_len, GFP_KERNEL);
	if(!p_buf){
		printk("[ltr559] p_buf malloc failed !\n");
		return -ENOMEM;
	}

	memset(p_buf, 0, part_len);
	ret = handle_partition_info(OEMDATA_PARTITION, p_buf, 0, part_len, 0);		//partition read
	if(ret != part_len){
		printk("[ltr559] read partition failed !\n");
		return ret;
	}

	oem_data_buf = (struct oem_data *)p_buf;
	if(!(oem_data_buf->oem_flag) || (oem_data_buf->oem_flag != OEM_FLAG)){
		oem_data_buf->oem_flag = OEM_FLAG;
	}
	
	if(!(oem_data_buf->ps_p_val.ps_cal_flag) || (oem_data_buf->ps_p_val.ps_cal_flag != PS_CAL_FLAG)){
		oem_data_buf->ps_p_val.ps_cal_flag = PS_CAL_FLAG;
	}

	oem_data_buf->ps_p_val.ps_cal_partition_val = (int)ps_cal_val;

	p_buf = (char *)oem_data_buf;
	ret = handle_partition_info(OEMDATA_PARTITION, p_buf, 0, part_len, 1);		//partition write
	if(ret != part_len){
		printk("[ltr559] write partition failed !\n");
		kfree(p_buf);
		return ret;
	}else{
		for (i = 0; i < 6; i++){
			psthre_data[i].th_hi = (int) ps_cal_val;
			psthre_data[i].th_lo = (int) ps_cal_val -10;
		}
	}

	kfree(p_buf);

	return size;
}

/*------------*/

static DEVICE_ATTR(debug_regs, S_IRUGO | S_IWUSR, ltr559_show_debug_regs,ltr559_store_debug_regs);
static DEVICE_ATTR(jcenable_als_sensor, S_IRUGO | S_IWUSR, ltr559_show_enable_als,ltr559_store_enable_als);

static DEVICE_ATTR(jcenable_ps_sensor, S_IRUGO | S_IWUSR, ltr559_show_enable_ps,ltr559_store_enable_ps);
static DEVICE_ATTR(info, S_IRUGO, ltr559_driver_info_show, NULL);
static DEVICE_ATTR(raw_adc, S_IRUGO, ltr559_show_adc_data, NULL);
static DEVICE_ATTR(als_lux, S_IRUGO, ltr559_show_lux_data, NULL);
static DEVICE_ATTR(ps_status, S_IRUGO, ltr559_show_ps_status, NULL);
static DEVICE_ATTR(psthre_data, S_IRUGO | S_IWUSR, ltr559_show_psthre_data,ltr559_store_psthre_data);
static DEVICE_ATTR(cal_value, S_IRUGO, ltr559_show_cal_value, NULL);
static DEVICE_ATTR(ps_calibration_partition, S_IRUGO | S_IWUSR, ltr559_ps_calibration_partition_val_show, ltr559_ps_calibration_partition_val_store);

static struct attribute *ltr559_attributes[] = {
		&dev_attr_jcenable_ps_sensor.attr,
		&dev_attr_info.attr,
		&dev_attr_jcenable_als_sensor.attr,
		&dev_attr_debug_regs.attr,
		&dev_attr_raw_adc.attr,
		&dev_attr_als_lux.attr,
		&dev_attr_ps_status.attr,
		&dev_attr_psthre_data.attr,
		&dev_attr_cal_value.attr,
		&dev_attr_ps_calibration_partition.attr,
		NULL,
};

static const struct attribute_group ltr559_attr_group = {
		.attrs = ltr559_attributes,
};

static int ltr559_als_set_enable(struct ltr559_data *data,
		unsigned int enable)
{
	//struct ltr559_data *data = container_of(sensors_cdev, struct ltr559_data, als_cdev);
	int ret = 0;

	if ((enable != 0) && (enable != 1)) {
		pr_err("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}

	ret = ltr559_als_enable(data->client, enable);
	if(ret < 0){
		pr_err("%s: enable(%d) failed!\n", __func__, enable);
		return -EFAULT;
	}

	data->als_open_state = enable;
	pr_err("%s: enable=(%d), data->als_open_state=%d\n", __func__, enable, data->als_open_state);
	return ret;
}

static ssize_t ltr559_ps_dynamic_caliberate(struct ltr559_data *data)
{
	//struct ltr559_data *data = container_of(sensors_cdev, struct ltr559_data, ps_cdev);
	struct ltr559_platform_data *pdata = data->platform_data;
	int i = 0, j = 0;
	int ps;
	int data_total=0;
	int noise = 0;
	int count = 5;
	int max = 0;
	int max_noise = 0;

	if(!data)
	{
		pr_err("ltr559_data is null!!\n");
		return -EFAULT;
	}

	/* wait for register to be stable */
	usleep_range(30000, 30000);

	for (i = 0; i < count; i++) {
		/* wait for ps value be stable */

		usleep_range(15000, 15000);

		ps = ltr559_ps_read(data->client);
		if (ps < 0) {
			i--;
			continue;
		}

		if(ps & 0x8000){
			i--;
			continue;
		} else {
			noise = ps;
			if(ps == 0)
				i--;
		}
		if (ps > max_noise)
			max_noise = ps;
		data_total += ps;

		if (max++ > 10) {
			pr_err("ltr559 read data error!\n");
			return -EFAULT;
		}
	}

	noise = data_total/count;
	if((max_noise < (noise + 70)) && (max_noise > (noise + 20)))
		noise = max_noise;
	data->dynamic_noise = noise;
/*if the noise twice bigger than boot, we treat it as covered mode */
	   if(pdata->prox_default_noise == 0){
		   pdata->prox_default_noise = data->dynamic_noise;
	   }
	   else if(data->dynamic_noise > (pdata->prox_default_noise + 200)){
		   noise = pdata->prox_default_noise;
		   data->dynamic_noise = pdata->prox_default_noise;
	   }
	   else if((data->dynamic_noise + 200) < pdata->prox_default_noise){
		   pdata->prox_default_noise = data->dynamic_noise;
	   }

	for(j=0; j<ARRAY_SIZE(psthre_data); j++) {
		if(noise < psthre_data[j].noise) {
			noise_cal = noise;
			pdata->prox_threshold = noise + psthre_data[j].th_hi;
			pdata->prox_hsyteresis_threshold = noise + psthre_data[j].th_lo;
			break;
		}
	}
	if(j == ARRAY_SIZE(psthre_data)) {
		pdata->prox_threshold = 1700;
		pdata->prox_hsyteresis_threshold = 1680;
		pr_err("ltr559 the proximity sensor rubber or structure is error!\n");
		return -EAGAIN;
	}

	//if (data->ps_state == 1) {
		//ltr559_set_ps_threshold(data->client, LTR559_PS_THRES_LOW_0, 0);
		ltr559_set_ps_threshold(data->client, LTR559_PS_THRES_UP_0, data->platform_data->prox_threshold);
	//} else if (data->ps_state == 0) {
		ltr559_set_ps_threshold(data->client, LTR559_PS_THRES_LOW_0, data->platform_data->prox_hsyteresis_threshold);
		//ltr559_set_ps_threshold(data->client, LTR559_PS_THRES_UP_0, 0x07ff);
	//}

	data->cali_update = true;

	return 0;
}

static int ltr559_ps_set_enable(struct ltr559_data *data,
		unsigned int enable)
{
	//struct ltr559_data *data = container_of(sensors_cdev, struct ltr559_data, ps_cdev);
	int ret = 0;

	if ((enable != 0) && (enable != 1)) {
		pr_err("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}

	ret = ltr559_ps_enable(data->client, enable);
	if(ret < 0){
		pr_err("%s: enable(%d) failed!\n", __func__, enable);
		return -EFAULT;
	}

	data->ps_open_state = enable;

	pr_err("%s: enable=(%d), data->ps_open_state=%d\n", __func__, enable, data->ps_open_state);
	return ret;
}

static int ltr559_suspend(struct device *dev)
{
	struct ltr559_data *data = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&data->lockw);
	ret |= ltr559_als_enable(data->client, 0);
	mutex_unlock(&data->lockw);
	if (data->ps_open_state == 1)
		enable_irq_wake(data->irq);
	return ret;
}

static int ltr559_resume(struct device *dev)
{
	struct ltr559_data *data = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&data->lockw);
	if (data->als_open_state == 1)
		ret = ltr559_als_enable(data->client, 1);
	mutex_unlock(&data->lockw);
	if (data->ps_open_state == 1)
		disable_irq_wake(data->irq);
	return ret;
}

static int ltr559_check_chip_id(struct i2c_client *client)
{
	int id;

	id = ltr559_i2c_read_byte_data(client, LTR559_MANUFACTURER_ID);
	printk("%s read the  LTR559_MANUFAC_ID is 0x%x\n", __func__, id);
	if (id != LTR559_MANUFAC_ID) {
		return -EINVAL;
	}
	return 0;
}

int ltr559_device_init(struct i2c_client *client)
{
	int retval = 0;
	int i;

#if 0
	retval = ltr559_i2c_write_byte_data(client, LTR559_ALS_CONTR, 0x02);    /* reset chip */
	if(retval < 0)
	{
		printk("%s   ltr559_i2c_write_byte_data(LTR559_ALS_CONTR, 0x02);  ERROR !!!.\n",__func__);
		return -EIO;
	}
#endif

	usleep_range(WAKEUP_DELAY * 1000, WAKEUP_DELAY * 1000);
	for (i = 3; i < sizeof(reg_tbl)/sizeof(reg_tbl[0]); i++) {
		if (reg_tbl[i].name == NULL || reg_tbl[i].addr == 0) {
				break;
		}
		if (reg_tbl[i].defval != reg_tbl[i].curval) {
			if (i < 10) {
				retval = ltr559_i2c_write_byte_data(client, reg_tbl[i].addr, reg_tbl[i].curval);
				if(retval < 0)
				{
					dev_err(&client->dev, "%s write error\n",__func__);
					return retval;
				}
			}
			#if 0
			else {
				retval = ltr559_i2c_write_byte_data(client, reg_tbl[i].addr, reg_tbl[i].curval & 0xff);
				if(retval < 0)
				{
					dev_err(&client->dev, "%s write error\n",__func__);
					return retval;
				}
				retval = ltr559_i2c_write_byte_data(client, reg_tbl[i].addr + 1, reg_tbl[i].curval >> 8);
				if(retval < 0)
				{
					dev_err(&client->dev, "%s write error\n",__func__);
					return retval;
				}
			}
			#endif

		}
	}

	return retval;
}




static int ltr559_parse_dt(struct device *dev, struct ltr559_data *data)
{
	struct ltr559_platform_data *pdata = data->platform_data;
	/* set functions of platform data */
	pdata->init = ltr559_platform_hw_init;
	pdata->exit = ltr559_platform_hw_exit;
	pdata->power_on = ltr559_platform_hw_power_on;

	pdata->prox_threshold = 120;
	pdata->prox_hsyteresis_threshold = 80;
	pdata->als_poll_interval = 200;

	return 0;
}

int ltr559_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	//struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct ltr559_data *data;
	struct ltr559_platform_data *pdata;
	int ret = 0;

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct ltr559_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		client->dev.platform_data = pdata;
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			dev_err(&client->dev, "No platform data\n");
			return -ENODEV;
		}
	}
	/* data memory allocation */
	data = kzalloc(sizeof(struct ltr559_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "kzalloc failed\n");
		ret = -ENOMEM;
		goto exit_kfree_pdata;
	}
	g_ltr559_data = data;
	data->client = client;
	data->platform_data = pdata;
	ret = ltr559_parse_dt(&client->dev, data);
	if (ret) {
		dev_err(&client->dev, "can't parse platform data\n");
		ret = -EFAULT;
		goto exit_kfree_data;
	}

	if (pdata->init)
		ret = pdata->init();
	if (pdata->power_on)
		ret = pdata->power_on(true);
	/* set client data as ltr559_data*/
	i2c_set_clientdata(client, data);

	ret = ltr559_check_chip_id(client);
	if(ret){
		ret = -ENXIO;
		printk("%s: the manufacture id is not match\n",__func__);
		goto exit_kfree_data;
	}

	ret = ltr559_device_init(client);
	if (ret) {
		ret = -ENXIO;
		printk("%s: device init failed\n",__func__);
		goto exit_kfree_data;
	}
	
	/* request gpio and irq */
	ret = ltr559_gpio_irq(data);
	if (ret) {
		ret = -ENXIO;
		dev_err(&client->dev,"gpio_irq failed\n");
		goto exit_chip_reset;
	}

	/* init delayed works */
	INIT_DELAYED_WORK(&data->ps_work, ltr559_ps_work_func);
	INIT_DELAYED_WORK(&data->als_work, ltr559_als_work_func);

	/* init mutex */
	mutex_init(&data->lockw);
	mutex_init(&data->op_lock);

	/* create sysfs group */
	ret = sysfs_create_group(&client->dev.kobj, &ltr559_attr_group);
	if (ret){
		ret = -EROFS;
		dev_err(&client->dev,"Unable to creat  client sysfs group\n");
		goto exit_remove_sysfs_group;
	}
	/* Enable / disable to trigger calibration at boot */
	pdata->prox_default_noise=0;
	
	#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
	client_ltr559_data = data;
	#endif

	dev_dbg(&client->dev,"probe succece\n");
	return 0;

exit_remove_sysfs_group:
	sysfs_remove_group(&client->dev.kobj, &ltr559_attr_group);
exit_chip_reset:
	ltr559_chip_reset(client);
exit_kfree_pdata:
	if (pdata && (client->dev.of_node))
		devm_kfree(&client->dev, pdata);
	data->platform_data= NULL;
    pdata = NULL;

exit_kfree_data:
	kfree(data);
	return ret;
}

static int ltr559_remove(struct i2c_client *client)
{
	struct ltr559_data *data = i2c_get_clientdata(client);
	struct ltr559_platform_data *pdata=data->platform_data;

	if (data == NULL || pdata == NULL)
		return 0;

	ltr559_ps_enable(client, 0);
	ltr559_als_enable(client, 0);
	ltr559_gpio_irq_free(data);

	sysfs_remove_group(&client->dev.kobj, &ltr559_attr_group);

	cancel_delayed_work_sync(&data->ps_work);
	cancel_delayed_work_sync(&data->als_work);
	device_init_wakeup(&client->dev, 0);

	if (pdata && (client->dev.of_node))
		devm_kfree(&client->dev, pdata);
	pdata = NULL;

	kfree(data);
	data = NULL;

	return 0;
}

static struct i2c_device_id ltr559_id[] = {
		{"ltr559", 0},
		{}
};

static struct of_device_id ltr_match_table[] = {
		  {.compatible = "qcom,ltr559" },
		{ },
};

MODULE_DEVICE_TABLE(i2c, ltr559_id);
static SIMPLE_DEV_PM_OPS(ltr559_pm_ops, ltr559_suspend, ltr559_resume);
static struct i2c_driver ltr559_driver = {
		.driver = {
				.name = LTR559_DRV_NAME,
				.owner = THIS_MODULE,
				.pm = &ltr559_pm_ops,
				.of_match_table = ltr_match_table,
		},
		.probe = ltr559_probe,
		.remove = ltr559_remove,
		.id_table = ltr559_id,
};

static int ltr559_driver_init(void)
{
		pr_info("Driver ltr5590 init.\n");
		return i2c_add_driver(&ltr559_driver);
};

static void ltr559_driver_exit(void)
{
		pr_info("Unload ltr559 module...\n");
		i2c_del_driver(&ltr559_driver);
}
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
static int32_t ltr559_initialize = 0;

int ps_als_init(void)
{
	int ret = ltr559_driver_init();
	if (ret == 0) {
		pr_debug("ltr559_driver_init ok\n");
		ltr559_initialize = 1;
	}
	return ret;
}
void ps_als_exit(void)
{
	if (ltr559_initialize) {
		ltr559_driver_exit();
		client_ltr559_data = NULL;
	}
}

int32_t als_get_initialize_state(void){
	return ltr559_initialize;
}
int32_t ps_get_initialize_state(void){
	return ltr559_initialize;
}

int32_t als_sensor_activate(bool enable)
{
	if (!ltr559_initialize || !client_ltr559_data) {
		pr_err("%s initialize fail\n", __func__);
		return -1;
	}
	return ltr559_als_set_enable(client_ltr559_data, (unsigned int)enable);
}
int32_t ps_sensor_activate(bool enable)
{
	if (!ltr559_initialize || !client_ltr559_data) {
		pr_err("%s initialize fail\n", __func__);
		return -1;
	}
	return ltr559_ps_set_enable(client_ltr559_data, (unsigned int)enable);
}
#else
module_init(ltr559_driver_init);
module_exit(ltr559_driver_exit);
#endif
MODULE_AUTHOR("Lite-On Technology Corp.");
MODULE_DESCRIPTION("Lite-On LTR-559 Proximity and Light Sensor Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
