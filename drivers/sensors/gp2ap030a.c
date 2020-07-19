 /*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 */
/* drivers/input/misc/gp2ap020a.c - GP2AP020A00F ambient light sensor and proximity sensor driver
 *
 * Copyright (C) 2012 Sharp Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/gpio.h>
//#include <mach/vreg.h>
#include <linux/regulator/consumer.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include "gp2ap030a.h"
#include <linux/wakelock.h>
#include <linux/of_gpio.h>

#define DBG_PSALS 						(0)

#define DEVICE_NAME 						"GP2AP030A00F"
#define DEVICE_FILE_NAME					"psals_sensor"
#define LIGHT_SENSOR_NAME					"light sensor"
#define PROXIMITY_SENSOR_NAME				"proximity sensor"

#define REG_ADR_00	0x00
#define REG_ADR_01	0x01
#define REG_ADR_02	0x02
#define REG_ADR_03	0x03
#define REG_ADR_04	0x04
#define REG_ADR_05	0x05
#define REG_ADR_06	0x06
#define REG_ADR_07	0x07
#define REG_ADR_08	0x08
#define REG_ADR_09	0x09
#define REG_ADR_0A	0x0A
#define REG_ADR_0B	0x0B
#define REG_ADR_0C	0x0C
#define REG_ADR_0D	0x0D
#define REG_ADR_0E	0x0E
#define REG_ADR_0F	0x0F
#define REG_ADR_10	0x10
#define REG_ADR_11	0x11
#define REG_ADR_MAX	0x12

#define ABS_LUX_REPORT						(ABS_MISC)
#define ABS_DISTANCE_REPORT					(ABS_DISTANCE)
#define GP2AP_PROXIMITY_DUMMY_VALUE         -1

#define GP2AP_LUXVALUE_MAX					10000
#define GP2AP_LUXVALUE_TABLE_MAX			50

#define GP2AP_DEV_STATUS_INIT				0x00000000
#define GP2AP_DEV_STATUS_SUSPEND			0x00000001
#define GP2AP_DEV_STATUS_SUSPEND_INT		0x00000002
#define GP2AP_DEV_STATUS_RESUME				0x00000004

#define ALS_POLLING_CNT_RESET_NONE			0x00000000
#define ALS_POLLING_CNT_RESET_MAX_DATA		0x00000001
#define ALS_POLLING_CNT_RESET_DISABLE		0x00000002
#define ALS_POLLING_CNT_RESET_STORE_POLL	0x00000004
#define ALS_POLLING_CNT_RESET_STORE_TIME	0x00000008
#define ALS_POLLING_CNT_RESET_INIT			0x00000010
#define ALS_POLLING_CNT_RESET_RESUME		0x00000020

#define GP2AP_OP_MODE_NONE					0x00000000
#define GP2AP_OP_MODE_PS					0x00000001
#define GP2AP_OP_MODE_ALS					0x00000002
#define GP2AP_OP_MODE_PS_ALS					0x00000003
#define GP2AP_OP_MODE_NONE_MASK					0x08

#define GP2AP_SENSOR_NONE					0x00000000
#define GP2AP_SENSOR_PS						0x00000001
#define GP2AP_SENSOR_ALS					0x00000002

#define GP2AP_WAKE_LOCK_TIME				(HZ * 10)
#define GP2AP_WAKE_LOCK_INPUT_TIME			(HZ)
#define GP2AP_ERR_MAX_CNT					5
#define GP2AP_COLOR_VARI					10

#define ALS_ON_WAIT_MS						210
#define ALS_ON_DELAY_TIMER_MS				(ALS_ON_WAIT_MS + 10)
#define PS_ON_WAIT_MS						5
#define PS_OFF_WAIT_MS						10

#define ALS_IRDATA_MAX						0x3FFF
#define ALS_CDATA_MAX						0x3FFF
#define ALS_GET_DATA_INTERVAL_MS			1000
#define ALS_POLL_DELAY_MS_DEF				250
#define ALS_MEAN_TIME_DEF					4

#define PS_JUDGE_COUNT						4
#define I2C_WRITE_MSG_NUM					1
#define I2C_READ_MSG_NUM					2

#define LUX_CALC_GAMMA_ALS					2
#define LUX_CALC_GAMMA_PSALS					8
#define LUX_CALC_GAMMA_UNIT					1

#define GPIO_ON								(1)
#define GPIO_OFF							(0)

#define PS_DETECTION_MASK					0x08

#define PSALS_LOG(md, fmt, ...) \
printk(md "[PSALS]%s(%d): " fmt, __func__, __LINE__, ## __VA_ARGS__)
#if DBG_PSALS
#define PSALS_DBG_LOG(md, fmt, ...) \
printk(md "[PSALS]%s(%d): " fmt, __func__, __LINE__, ## __VA_ARGS__)
#else
#define PSALS_DBG_LOG(md, fmt, ...)
#endif /* DBG_PSALS */

enum vsensor_power
{
	VSENSOR_OFF = 0,
	VSENSOR_ON,
};

enum vpro_power
{
	VPRO_OFF = 0,
	VPRO_ON,
};

enum sensor_enable
{
	SENSOR_DISABLE = 0,
	SENSOR_ENABLE,
};

enum op_flag
{
	OP_OFF = 0,
	OP_ON,
};

struct gp2ap_data
{
	struct i2c_client *client;
	struct mutex psals_mutex;
	struct delayed_work dwork;
	struct delayed_work als_on_dwork;
	struct delayed_work als_data_dwork;
	struct input_dev *input_dev_als;
	struct input_dev *input_dev_ps;
	int enable_ps_sensor;
	int enable_als_sensor;
	u32 ps_detection;
	u32 ps_data;
	u32 als_poll_delay;
	s32 ps_irq;
	s32 cdata;
	s32 irdata;
	u32 als_lux;
	u32 luxValue_table[GP2AP_LUXVALUE_TABLE_MAX];
	u32 als_lux_ave;
	u32 als_polling_cnt;
	u32 als_mean_times;
	u32 als_polling_cnt_reset;
	u32 op_mode;
	u32 op_sensor;
	u32 ratio_reg;
	s32 cdata_reg;
	s32 irdata_reg;
};

struct reg_data
{
	u8 reg;
	u8 data;
};

struct alsproxsns_regulator_data {
    struct regulator* vdd_reg;
    uint32_t min_uV;
    uint32_t max_uV;
    uint32_t on_load_uA;
    uint32_t off_load_uA;
};

static struct alsproxsns_regulator_data als_reg_data;
static struct alsproxsns_regulator_data prox_reg_data;
static struct alsproxsns_regulator_data als_reg_data_l5;
static struct alsproxsns_regulator_data prox_reg_data_l5;

static struct i2c_client *client_gp2ap = NULL;

static struct wake_lock gp2ap_wake_lock;
static struct wake_lock gp2ap_wake_lock_input;

static struct reg_data reg_init_data[] =
{
	{REG_ADR_00, 0x00},
	{REG_ADR_01, 0x5B},
	{REG_ADR_02, 0x5A},
	{REG_ADR_03, 0x38},
};

static u16 nv_proximity_sensor_near[GP2AP_COLOR_VARI] =
    {0x0012,0x0012,0x0012,0x0012,0x0012,0x0012,0x0012,0x0012,0x0012,0x0012};
static u16 nv_proximity_sensor_far[GP2AP_COLOR_VARI] =
    {0x000C,0x000C,0x000C,0x000C,0x000C,0x000C,0x000C,0x000C,0x000C,0x000C};
static u8 nv_photosensor_th0[GP2AP_COLOR_VARI] =
    {0x2D,0x2D,0x2D,0x2D,0x2D,0x2D,0x2D,0x2D,0x2D,0x2D};
static u8 nv_photosensor_th1[GP2AP_COLOR_VARI] =
    {0x3C,0x3C,0x3C,0x3C,0x3C,0x3C,0x3C,0x3C,0x3C,0x3C};
static u8 nv_photosensor_th2[GP2AP_COLOR_VARI] =
    {0x4E,0x4E,0x4E,0x4E,0x4E,0x4E,0x4E,0x4E,0x4E,0x4E};
static u8 nv_photosensor_th3[GP2AP_COLOR_VARI] =
    {0x61,0x61,0x61,0x61,0x61,0x61,0x61,0x61,0x61,0x61};
static u8 nv_photosensor_th4[GP2AP_COLOR_VARI] =
    {0x61,0x61,0x61,0x61,0x61,0x61,0x61,0x61,0x61,0x61};
static u16 nv_photosensor_a0[GP2AP_COLOR_VARI] =
    {0x123E,0x123E,0x123E,0x123E,0x123E,0x123E,0x123E,0x123E,0x123E,0x123E};
static u16 nv_photosensor_a1[GP2AP_COLOR_VARI] =
    {0x3A52,0x3A52,0x3A52,0x3A52,0x3A52,0x3A52,0x3A52,0x3A52,0x3A52,0x3A52};
static u16 nv_photosensor_a2[GP2AP_COLOR_VARI] =
    {0x04E2,0x04E2,0x04E2,0x04E2,0x04E2,0x04E2,0x04E2,0x04E2,0x04E2,0x04E2};
static u16 nv_photosensor_a3[GP2AP_COLOR_VARI] =
    {0x03D4,0x03D4,0x03D4,0x03D4,0x03D4,0x03D4,0x03D4,0x03D4,0x03D4,0x03D4};
static u16 nv_photosensor_a4[GP2AP_COLOR_VARI] =
    {0x04C4,0x04C4,0x04C4,0x04C4,0x04C4,0x04C4,0x04C4,0x04C4,0x04C4,0x04C4};
static u16 nv_photosensor_b0[GP2AP_COLOR_VARI] =
    {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000};
static u16 nv_photosensor_b1[GP2AP_COLOR_VARI] =
    {0x5910,0x5910,0x5910,0x5910,0x5910,0x5910,0x5910,0x5910,0x5910,0x5910};
static u16 nv_photosensor_b2[GP2AP_COLOR_VARI] =
    {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000};
static u16 nv_photosensor_b3[GP2AP_COLOR_VARI] =
    {0x03F2,0x03F2,0x03F2,0x03F2,0x03F2,0x03F2,0x03F2,0x03F2,0x03F2,0x03F2};
static u16 nv_photosensor_b4[GP2AP_COLOR_VARI] =
    {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000};
static u8 nv_photosensor_1r[GP2AP_COLOR_VARI] =
    {0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02};
static u8 nv_photosensor_2r[GP2AP_COLOR_VARI] =
    {0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08};
static u16 nv_prox_photo_colvar[1] = {0x0000};

static u32 nv_status = 0;
static atomic_t g_dev_status;
static atomic_t g_update_threshold_flg;
static struct workqueue_struct *als_polling_wq;
static struct workqueue_struct *ps_polling_wq;
static s32 gp2ap_ps_irq_cnt = 0;

#if 0
static u32 gpio_config_prox_int_pd[] = {
   GPIO_CFG(GPIO_PROX_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};
static u32 gpio_config_prox_int_pu[] = {
   GPIO_CFG(GPIO_PROX_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
};
#endif

static s32 gp2ap_i2c_read(struct i2c_client *client, u8 reg, u8 *rbuf, int len);
static s32 gp2ap_i2c_write(struct i2c_client *client, u8 reg, u8 *wbuf, int len);
static void gp2ap_enable_ps_irq(struct gp2ap_data *data);
static void gp2ap_disable_ps_irq(struct gp2ap_data *data);
static u32 LuxCalculation(u32 cdata, u32 irdata, struct gp2ap_data *data);
static int gp2ap_get_als_data(struct i2c_client *client, u32 *cdata, u32 *irdata);
static int gp2ap_set_ps_threshold(struct i2c_client *client, u16 low, u16 high);
static u32 RatioCalculation(struct i2c_client *client, struct gp2ap_data *data);
static int gp2ap_set_op_mode(struct i2c_client *client, u32 sensor_kind, enum op_flag onoff);
static void gp2ap_enable_als_sensor(struct i2c_client *client, enum sensor_enable enable);
static void gp2ap_enable_ps_sensor(struct i2c_client *client, enum sensor_enable enable);
static int gp2ap_init_client(struct i2c_client *client);
static int gp2ap_fin_client(struct i2c_client *client);
static void gp2ap_reschedule_work(struct gp2ap_data *data, unsigned long delay);
static void gp2ap_put_luxValue(struct gp2ap_data *data, u32 luxValue);
static void gp2ap_als_on_work_handler(struct work_struct *work);
static void gp2ap_als_data_work_handler(struct work_struct *work);
static void gp2ap_work_handler(struct work_struct *work);
static irqreturn_t gp2ap_interrupt(int vec, void *info);
static ssize_t gp2ap_show_enable_ps_sensor(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t gp2ap_store_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count);
static ssize_t gp2ap_show_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t gp2ap_store_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count);
static ssize_t gp2ap_show_als_poll_delay(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t gp2ap_store_als_poll_delay(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count);
static ssize_t gp2ap_show_ps_detection(struct device *dev,
				struct device_attribute *attr, char *buf);
static void gp2ap_set_als_mean_times(struct gp2ap_data *data, u32 mean_times);
static void gp2ap_set_sensor_nv(unsigned long ulArg);
static int gp2ap_open(struct inode *inode_type, struct file *file);
static int gp2ap_release(struct inode *inode_type, struct file *file);
static long gp2ap_ioctl(struct file *file_type,
				  unsigned int unCmd, unsigned long ulArg);
static int gp2ap_suspend(struct i2c_client *client, pm_message_t mesg);
static int gp2ap_resume(struct i2c_client *client);


static s32 gp2ap_i2c_read(struct i2c_client *client, u8 reg, u8 *rbuf, int len)
{
	int ret = 0;
	struct i2c_msg i2cMsg[I2C_READ_MSG_NUM];
	u8 buff;
	int retry = GP2AP_ERR_MAX_CNT;
	int i;

	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	PSALS_DBG_LOG(KERN_INFO, "reg=0x%02X,len=%d\n", reg, len);
	if (client == NULL)
	{
		return -ENODEV;
	}

	while (retry--)
	{
		i2cMsg[0].addr = client->addr;
		i2cMsg[0].flags = 0;
		i2cMsg[0].len = 1;
		i2cMsg[0].buf = &buff;
		buff = reg;
		i2cMsg[1].addr = client->addr;
		i2cMsg[1].flags = I2C_M_RD;
		i2cMsg[1].len = len;
		i2cMsg[1].buf = rbuf;

		ret = i2c_transfer(client->adapter, &i2cMsg[0], I2C_READ_MSG_NUM);
		PSALS_DBG_LOG(KERN_INFO, "i2c_transfer() called. ret=%d\n",ret);
		if (ret == I2C_READ_MSG_NUM)
		{
			PSALS_DBG_LOG(KERN_INFO, "end. exec mesg=%d\n", (int)ret);
			for (i = 0; i < len; i++)
			{
				PSALS_DBG_LOG(KERN_INFO, "i2c read reg=0x%02X,value=0x%02X\n",
					(unsigned int)(reg + i), (unsigned int)*(rbuf + i));
			}
			PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
			return 0;
		}
	}
	PSALS_LOG(KERN_ERR, "i2c transfer error(%d).\n", ret);
	return -1;
}

static s32 gp2ap_i2c_write(struct i2c_client *client, u8 reg, u8 *wbuf, int len)
{
	int ret = 0;
	struct i2c_msg i2cMsg[I2C_WRITE_MSG_NUM];
	static u8 buff[REG_ADR_MAX];
	int retry = GP2AP_ERR_MAX_CNT;
	int i;

	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	PSALS_DBG_LOG(KERN_INFO, "reg=0x%02X,len=%d\n", reg, len);
	if (client == NULL)
	{
		return -ENODEV;
	}

	while (retry--)
	{
		buff[0] = reg;
		memcpy(&buff[1], wbuf, len);

		i2cMsg[0].addr = client->addr;
		i2cMsg[0].flags = 0;
		i2cMsg[0].len = len + 1;
		i2cMsg[0].buf = (u8 *)buff;

		ret = i2c_transfer(client->adapter, &i2cMsg[0], I2C_WRITE_MSG_NUM);
		PSALS_DBG_LOG(KERN_INFO, "i2c_transfer() called. ret=%d\n",ret);
		if (ret == I2C_WRITE_MSG_NUM)
		{
			PSALS_DBG_LOG(KERN_INFO, "end. exec mesg=%d\n", ret);
			for (i = 0; i < len; i++)
			{
				PSALS_DBG_LOG(KERN_INFO,"i2c write reg=0x%02X,value=0x%02X\n",
					(unsigned int)(reg + i), (unsigned int)*(wbuf + i));
			}
			PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
			return 0;
		}
	}
	PSALS_LOG(KERN_ERR, "i2c transfer error(%d).\n", ret);

	return -1;
}

static void gp2ap_enable_ps_irq(struct gp2ap_data *data)
{
	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	PSALS_DBG_LOG(KERN_INFO, "before gp2ap_ps_irq_cnt=%d\n", gp2ap_ps_irq_cnt);
	if (gp2ap_ps_irq_cnt <= 0)
	{
		PSALS_DBG_LOG(KERN_INFO, "enable_irq\n");
		enable_irq(data->ps_irq);
		gp2ap_ps_irq_cnt++;
	}
	PSALS_DBG_LOG(KERN_INFO, "after gp2ap_ps_irq_cnt=%d\n", gp2ap_ps_irq_cnt);
	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
}

static void gp2ap_disable_ps_irq(struct gp2ap_data *data)
{
	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	PSALS_DBG_LOG(KERN_INFO, "before gp2ap_ps_irq_cnt=%d\n", gp2ap_ps_irq_cnt);
	if (gp2ap_ps_irq_cnt > 0)
	{
		gp2ap_ps_irq_cnt--;
		disable_irq(data->ps_irq);
		PSALS_DBG_LOG(KERN_INFO, "disable_irq\n");
	}
	PSALS_DBG_LOG(KERN_INFO, "after gp2ap_ps_irq_cnt=%d\n", gp2ap_ps_irq_cnt);
	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
}

static u32 LuxCalculation(u32 cdata, u32 irdata, struct gp2ap_data *data)
{
	u32 lux;
	u32 ratio;
	u16 alpha = 0;
	u16 beta = 0;
	u16 gamma = LUX_CALC_GAMMA_ALS;
	u16 colvar = nv_prox_photo_colvar[0];
	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	PSALS_DBG_LOG(KERN_INFO, "irdata=%d,cdata=%d,op_mode=0x%08X\n",
						irdata, cdata, (unsigned int)data->op_mode);

	if((data->op_mode == GP2AP_OP_MODE_ALS) &&
					( (cdata >= 0xFDE8) || (irdata >= 0xFDE8)) )
	{
		return GP2AP_LUXVALUE_MAX;
	}
	else if((data->op_mode == GP2AP_OP_MODE_PS_ALS) &&
					( (cdata >= 0x3E80) || (irdata >= 0x3E80)) )
	{
		return GP2AP_LUXVALUE_MAX;
	}


	if (cdata == 0)
	{
		ratio = 100;
	}
	else
	{
		ratio = (irdata * 100) / cdata;
	}

	if ((irdata == 0) || (cdata == 0))
	{
		lux = 0;
		PSALS_DBG_LOG(KERN_INFO, "set lux=%d\n", lux);
	}
	else
	{
		if(data->op_mode == GP2AP_OP_MODE_PS_ALS)
		{
			gamma = nv_photosensor_2r[colvar];
		}
		else
		{
			gamma = nv_photosensor_1r[colvar];
		}

		if (ratio <= nv_photosensor_th0[colvar])
		{
			alpha = nv_photosensor_a0[colvar];
			beta = nv_photosensor_b0[colvar];
		}
		else if (ratio <= nv_photosensor_th1[colvar])
		{
			alpha = nv_photosensor_a1[colvar];
			beta = nv_photosensor_b1[colvar];
		}
		else if (ratio <= nv_photosensor_th2[colvar])
		{
			alpha = nv_photosensor_a2[colvar];
			beta = nv_photosensor_b2[colvar];
		}
		else if (ratio <= nv_photosensor_th3[colvar])
		{
			alpha = nv_photosensor_a3[colvar];
			beta = nv_photosensor_b3[colvar];
		}
		else
		{
			PSALS_DBG_LOG(KERN_INFO, "ratio > %d, data->als_lux = %d\n"
					  , nv_photosensor_th3[colvar], data->als_lux);
			return data->als_lux;
		}
		PSALS_DBG_LOG(KERN_INFO, "ratio=%d,alpha=0x%04X,beta=0x%04X,"
					  "gamma=0x%04X\n", ratio, alpha, beta, gamma);
		lux = (gamma * ((alpha * cdata - beta * irdata) / 100))/100
													/ LUX_CALC_GAMMA_UNIT;
	}
	PSALS_DBG_LOG(KERN_INFO, "end. calc lux=%d\n", lux);

	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
	return lux;
}

static int gp2ap_get_als_data
(struct i2c_client *client, u32 *cdata, u32 *irdata)
{
	int ret = 0;
	u8 buf[4];

	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	ret = gp2ap_i2c_read(client, REG_ADR_0C, buf, 4);
	if (ret < 0)
	{
		goto exit;
	}

	*cdata = (buf[1] << 8) | buf[0];
	*irdata = (buf[3] << 8) | buf[2];

	PSALS_DBG_LOG(KERN_INFO, "cdata=0x%04X,irdata=0x%04X\n", *cdata, *irdata);
exit:
	PSALS_DBG_LOG(KERN_INFO, "[OUT] ret=%d\n", ret);
	return ret;
}


static int gp2ap_set_ps_threshold(struct i2c_client *client, u16 low, u16 high)
{
	int ret = 0;
	u8 buf[4];
	u8 mode[1];
	u8 s_mode[1];

	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	PSALS_DBG_LOG(KERN_INFO, "low=0x%04X,high=0x%04X\n", low, high);
	ret = gp2ap_i2c_read(client, REG_ADR_00, mode, 1);
	if (ret < 0)
	{
		PSALS_LOG(KERN_ERR, "gp2ap_i2c_read error. ret=%d\n", ret);
		return ret;
	}

	s_mode[0] = mode[0] & GP2AP_OP_MODE_NONE_MASK;
	gp2ap_i2c_write(client, REG_ADR_00, s_mode, 1);

	buf[0] = (u8)(0x00FF & low);
	buf[1] = (u8)(0x00FF & (low >> 8));
	buf[2] = (u8)(0x00FF & high);
	buf[3] = (u8)(0x00FF & (high >> 8));
	ret = gp2ap_i2c_write(client, REG_ADR_08, buf, sizeof(buf));

	ret = gp2ap_i2c_write(client, REG_ADR_00, mode, 1);

	PSALS_DBG_LOG(KERN_INFO, "[OUT] ret=%d\n", ret);
	return ret;
}


static u32 RatioCalculation(struct i2c_client *client, struct gp2ap_data *data)
{
	u32 cdata;
	u32 irdata;
	u32 ratio;
	int ret;

	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	ret = gp2ap_get_als_data(client, &cdata, &irdata);


	if (ret < 0)
	{
		PSALS_LOG(KERN_ERR, "gp2ap_get_als_data error. ret=%d\n", ret);
		return ret;
	}

	data->cdata_reg = cdata;
	data->irdata_reg = irdata;

	if (cdata == 0)
	{
		ratio = 100;
	}
	else
	{
		ratio = (irdata * 100) / cdata;
	}

	PSALS_DBG_LOG(KERN_INFO, "ratio=%d\n",ratio);
	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");

	return ratio;
}

static int gp2ap_set_op_mode
(struct i2c_client *client, u32 sensor_kind, enum op_flag onoff)
{
	struct gp2ap_data *data = i2c_get_clientdata(client);
	int ret = 0;
	u8 buf[1];
	u32 pre_op_mode = data->op_mode;

	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	PSALS_DBG_LOG(KERN_INFO, "sensor_kind=0x%08X, onoff=%d, ",
										sensor_kind, onoff);
	PSALS_DBG_LOG(KERN_INFO, "before op_mode=0x%08X,op_sensor=0x%08X\n",
				(unsigned int)data->op_mode, (unsigned int)data->op_sensor);

	if (onoff == OP_ON)
		data->op_sensor |= sensor_kind;
	else
		data->op_sensor &= ~sensor_kind;

	switch (data->op_sensor)
	{
		case GP2AP_SENSOR_NONE:
			data->op_mode = GP2AP_OP_MODE_NONE;
			break;
		case GP2AP_SENSOR_PS:
		case GP2AP_SENSOR_PS | GP2AP_SENSOR_ALS:
			data->op_mode = GP2AP_OP_MODE_PS_ALS;
			break;
		case GP2AP_SENSOR_ALS:
			data->op_mode = GP2AP_OP_MODE_ALS;
			break;
		default:
			PSALS_LOG(KERN_ERR, "illegal status. op_sensor=0x%08X\n",
									(unsigned int)data->op_sensor);
			data->op_mode = GP2AP_OP_MODE_NONE;
			break;
	}

	if (pre_op_mode != data->op_mode)
	{
		if(pre_op_mode == GP2AP_OP_MODE_PS_ALS)
		{
			buf[0] = 0x00;
			ret = gp2ap_i2c_write(client, REG_ADR_00, buf, 1);
			if (ret < 0)
			{
				PSALS_LOG(KERN_ERR, "gp2ap_i2c_write error. ret=%d\n", ret);
			}
		}

		switch (data->op_mode)
		{
			case GP2AP_OP_MODE_NONE:
				buf[0] = 0x00;
				break;
			case GP2AP_OP_MODE_ALS:
				buf[0] = 0x5B;
				break;
			case GP2AP_OP_MODE_PS_ALS:
				buf[0] = 0x63;
				break;
			case GP2AP_OP_MODE_PS:
			default:
				PSALS_LOG(KERN_ERR, "illegal status. op_mode=0x%08X\n",
										(unsigned int)data->op_mode);
				break;
		}

		if(data->op_mode == GP2AP_OP_MODE_ALS)
		{
			ret = gp2ap_i2c_write(client, REG_ADR_01, buf, 1);
			if (ret < 0)
			{
				PSALS_LOG(KERN_ERR, "gp2ap_i2c_write error. ret=%d\n", ret);
			}

			PSALS_DBG_LOG(KERN_INFO, "op_mode=GP2AP_OP_MODE_ALS");
			buf[0] = 0x90;
		}
		else if(data->op_mode == GP2AP_OP_MODE_PS_ALS)
		{
			ret = gp2ap_i2c_write(client, REG_ADR_01, buf, 1);
			if (ret < 0)
			{
				PSALS_LOG(KERN_ERR, "gp2ap_i2c_write error. ret=%d\n", ret);
			}
			PSALS_DBG_LOG(KERN_INFO, "op_mode=GP2AP_OP_MODE_PS_ALS");
			buf[0] = 0xC0;
		}

		ret = gp2ap_i2c_write(client, REG_ADR_00, buf, 1);
		if (ret < 0)
		{
			PSALS_LOG(KERN_ERR, "gp2ap_i2c_write error. ret=%d\n", ret);
		}
	}

	PSALS_DBG_LOG(KERN_INFO, "after op_mode=0x%08X,op_sensor=0x%08X\n",
				(unsigned int)data->op_mode, (unsigned int)data->op_sensor);
	PSALS_DBG_LOG(KERN_INFO, "[OUT] ret=%d\n", ret);

	return ret;
}

static void gp2ap_enable_als_sensor
(struct i2c_client *client, enum sensor_enable enable)
{
	struct gp2ap_data *data = i2c_get_clientdata(client);
	int ret = 0;

	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	PSALS_DBG_LOG(KERN_INFO, "enable=%d, enable_ps_sensor=%d\n", enable,
												data->enable_ps_sensor);
	if (enable == SENSOR_ENABLE)
	{
        ret = regulator_set_optimum_mode(als_reg_data_l5.vdd_reg, als_reg_data_l5.on_load_uA);
        if( ret < 0 ) {
            PSALS_LOG(KERN_ERR, "regulator_set_optimum_mode(L5) fail. ret=%d\n", ret);
        }
        ret = regulator_set_optimum_mode(als_reg_data.vdd_reg, als_reg_data.on_load_uA);
        if( ret < 0 ) {
            PSALS_LOG(KERN_ERR, "regulator_set_optimum_mode fail. ret=%d\n", ret);
        }
        usleep_range(1000,1000);
		PSALS_DBG_LOG(KERN_INFO, "queue_delayed_work(als_on_dwork) call. "
														"delay_ms=%d\n", 0);
		queue_delayed_work(als_polling_wq, &data->als_on_dwork,
										msecs_to_jiffies(0));
	}
	else
	{
		mutex_unlock(&data->psals_mutex);
		cancel_delayed_work_sync(&data->als_on_dwork);
		cancel_delayed_work_sync(&data->als_data_dwork);
		mutex_lock(&data->psals_mutex);

		ret = gp2ap_set_op_mode(client, GP2AP_SENSOR_ALS, OP_OFF);
		if (ret < 0)
		{
			PSALS_LOG(KERN_ERR, "gp2ap_set_op_mode error. ret=%d\n", ret);
		}

		data->als_polling_cnt_reset |= ALS_POLLING_CNT_RESET_DISABLE;
		PSALS_DBG_LOG(KERN_INFO, "als_polling_cnt_reset=0x%08X\n",
									data->als_polling_cnt_reset);
		ret = regulator_set_optimum_mode(als_reg_data.vdd_reg, als_reg_data.off_load_uA);
        if( ret < 0 ) {
            PSALS_LOG(KERN_ERR, "regulator_set_optimum_mode fail. ret=%d\n", ret);
        }
        ret = regulator_set_optimum_mode(als_reg_data_l5.vdd_reg, als_reg_data_l5.off_load_uA);
        if( ret < 0 ) {
            PSALS_LOG(KERN_ERR, "regulator_set_optimum_mode(L5) fail. ret=%d\n", ret);
        }
	}
	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
}

static void gp2ap_enable_ps_sensor
(struct i2c_client *client, enum sensor_enable enable)
{
	struct gp2ap_data *data = i2c_get_clientdata(client);
	int ret = 0;
	u16 colvar = nv_prox_photo_colvar[0];

	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	PSALS_DBG_LOG(KERN_INFO, "enable=%d, enable_als_sensor=%d\n",
								enable, data->enable_als_sensor);
	if (enable == 1)
	{
        ret = regulator_set_optimum_mode(prox_reg_data.vdd_reg, prox_reg_data.on_load_uA);
        if( ret < 0 ) {
            PSALS_LOG(KERN_ERR, "regulator_set_optimum_mode fail. ret=%d\n", ret);
        }
        ret = regulator_set_optimum_mode(prox_reg_data_l5.vdd_reg, prox_reg_data_l5.on_load_uA);
        if( ret < 0 ) {
            PSALS_LOG(KERN_ERR, "regulator_set_optimum_mode(L5) fail. ret=%d\n", ret);
        }
        usleep_range(1000,1000);

		if (data->enable_als_sensor > 0)
			gp2ap_enable_als_sensor(client_gp2ap, SENSOR_DISABLE);

		ret = gp2ap_set_ps_threshold(client, nv_proximity_sensor_far[colvar],
					nv_proximity_sensor_near[colvar]);
		if (ret < 0)
		{
			PSALS_LOG(KERN_ERR, "gp2ap_set_ps_threshold error. ret=%d\n", ret);
			goto exit;
		}

		ret = gp2ap_set_op_mode(client, GP2AP_SENSOR_PS, OP_ON);
		if (ret < 0)
		{
			PSALS_LOG(KERN_ERR, "gp2ap_set_op_mode error. ret=%d\n", ret);
			goto exit;
		}

		if(data->op_mode == GP2AP_OP_MODE_PS_ALS)
		{
			msleep(30);
			data->ratio_reg = RatioCalculation(client, data);

		}

		if (data->enable_als_sensor > 0)
			gp2ap_enable_als_sensor(client_gp2ap, SENSOR_ENABLE);
		gp2ap_enable_ps_irq(data);
	}
	else
	{
		gp2ap_disable_ps_irq(data);

		if (data->enable_als_sensor > 0)
		{
			gp2ap_enable_als_sensor(client_gp2ap, SENSOR_DISABLE);
		}

		ret = gp2ap_set_op_mode(client, GP2AP_SENSOR_PS, OP_OFF);
		if (ret < 0)
		{
			PSALS_LOG(KERN_ERR, "gp2ap_set_op_mode error. ret=%d\n", ret);
			goto exit;
		}

		if (data->enable_als_sensor > 0)
		{
			gp2ap_enable_als_sensor(client_gp2ap, SENSOR_ENABLE);
		}

        ret = regulator_set_optimum_mode(prox_reg_data.vdd_reg, prox_reg_data.off_load_uA);
        if( ret < 0 ) {
            PSALS_LOG(KERN_ERR, "regulator_set_optimum_mode fail. ret=%d\n", ret);
        }
        ret = regulator_set_optimum_mode(prox_reg_data_l5.vdd_reg, prox_reg_data_l5.off_load_uA);
        if( ret < 0 ) {
            PSALS_LOG(KERN_ERR, "regulator_set_optimum_mode(L5) fail. ret=%d\n", ret);
        }
	}

exit:
	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
}

static int gp2ap_init_client(struct i2c_client *client)
{
	int ret = 0;
	int i;

	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
#if 0
	ret = gpio_tlmm_config(gpio_config_prox_int_pu[0], GPIO_CFG_ENABLE);
	PSALS_DBG_LOG(KERN_INFO, "gpio_tlmm_config(prox_int_pu) called. ret=%d\n",
																		ret);
	if (ret < 0)
	{
		PSALS_LOG(KERN_ERR, "gpio_tlmm_config error. ret=%d\n", ret);
		goto exit;
	}
#endif
	for (i = 0; i < sizeof(reg_init_data)/sizeof(reg_init_data[0]); i++)
	{
		ret = gp2ap_i2c_write(client, reg_init_data[i].reg,
								&reg_init_data[i].data, 1);
		if (ret < 0)
		{
			PSALS_LOG(KERN_ERR, "gp2ap_i2c_write error. ret=%d\n", ret);
			goto exit;
		}
	}

exit:
	PSALS_DBG_LOG(KERN_INFO, "[OUT] ret=%d\n", ret);
	return ret;
}

static int gp2ap_fin_client(struct i2c_client *client)
{
	int ret = 0;

	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
#if 0
	ret = gpio_tlmm_config(gpio_config_prox_int_pd[0], GPIO_CFG_ENABLE);
	PSALS_DBG_LOG(KERN_INFO, "gpio_tlmm_config(prox_int_pd) called. ret=%d\n",
																		ret);
	if (ret < 0)
	{
		PSALS_LOG(KERN_ERR, "gpio_tlmm_config error. ret=%d\n", ret);
		goto exit;
	}
exit:
#endif
	PSALS_DBG_LOG(KERN_INFO, "[OUT] ret=%d\n", ret);
	return ret;
}

static void gp2ap_reschedule_work(struct gp2ap_data *data,
					  unsigned long delay)
{
	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");

	PSALS_DBG_LOG(KERN_INFO, "queue_delayed_work(dwork) call. delay_ms=%lu\n",
																	delay);
	queue_delayed_work(ps_polling_wq, &data->dwork, msecs_to_jiffies(delay));

	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
}

static void gp2ap_put_luxValue(struct gp2ap_data *data, u32 luxValue)
{
	u32 cnt = 1;
	u32 mean_temp = 0;
	u32 mean_times = 0;

	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	PSALS_DBG_LOG(KERN_INFO, "luxValue=%d,als_polling_cnt_reset=0x%08X\n",
									luxValue, data->als_polling_cnt_reset);
	PSALS_DBG_LOG(KERN_INFO, "before als_polling_cnt=%d\n",
									data->als_polling_cnt);
	if (data == NULL)
	{
		PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
		return;
	}

	data->als_lux = luxValue;
	PSALS_DBG_LOG(KERN_INFO, "data->als_lux=%d\n",
									data->als_lux);
	if (data->als_polling_cnt_reset)
	{
		data->als_polling_cnt_reset = ALS_POLLING_CNT_RESET_NONE;
		PSALS_DBG_LOG(KERN_INFO, "als_polling_cnt_reset=0x%08X\n",
									data->als_polling_cnt_reset);
		data->als_lux_ave = luxValue;
		data->als_polling_cnt = 0;

		input_report_abs(data->input_dev_als, ABS_LUX_REPORT,
										data->als_lux_ave);
		input_sync(data->input_dev_als);
		PSALS_DBG_LOG(KERN_INFO, "ABS_LUX_REPORT(als_lux_ave=%d)\n",
												data->als_lux_ave);
	}
	else
	{
		data->luxValue_table[data->als_polling_cnt] = luxValue;
		data->als_polling_cnt++;

		if (data->als_polling_cnt >= data->als_mean_times)
		{
			for(cnt = 0; cnt < data->als_mean_times; cnt++)
			{
				PSALS_DBG_LOG(KERN_INFO, "luxValue_table[%d]=%d\n",
									cnt, data->luxValue_table[cnt]);
				mean_temp += data->luxValue_table[cnt] * (cnt + 1);
				mean_times += cnt + 1;
			}
			data->als_lux_ave = mean_temp / mean_times;
			data->als_polling_cnt = 0;
			input_report_abs(data->input_dev_als, ABS_LUX_REPORT,
											data->als_lux_ave);
			input_sync(data->input_dev_als);
			PSALS_DBG_LOG(KERN_INFO, "ABS_LUX_REPORT(als_lux_ave=%d)\n",
													data->als_lux_ave);
		}
	}
	PSALS_DBG_LOG(KERN_INFO, "after als_polling_cnt = %d\n",
									data->als_polling_cnt);
	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
}

static void gp2ap_als_on_work_handler(struct work_struct *work)
{
	struct gp2ap_data *data = container_of(work, struct gp2ap_data,
												als_on_dwork.work);
	struct i2c_client *client = data->client;
	int ret = 0;
	int als_on_wait_ms = 0;

	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	mutex_lock(&data->psals_mutex);

	PSALS_DBG_LOG(KERN_INFO, "enable_ps_sensor=%d\n", data->enable_ps_sensor);
	ret = gp2ap_set_op_mode(client, GP2AP_SENSOR_ALS, OP_ON);
	if (ret < 0)
	{
		PSALS_LOG(KERN_ERR, "gp2ap_set_op_mode error. ret=%d\n", ret);
		goto exit;
	}
	als_on_wait_ms = ALS_ON_DELAY_TIMER_MS;

	PSALS_DBG_LOG(KERN_INFO, "queue_delayed_work(als_data_dwork) call. "
									"delay_ms=%d\n", als_on_wait_ms);
	queue_delayed_work(als_polling_wq, &data->als_data_dwork,
							msecs_to_jiffies(als_on_wait_ms));

exit:
	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
	mutex_unlock(&data->psals_mutex);
}

static void gp2ap_als_data_work_handler(struct work_struct *work)
{
	struct gp2ap_data *data = container_of(work, struct gp2ap_data,
											als_data_dwork.work);
	struct i2c_client *client = data->client;

	u32 irdata;
	u32 cdata;
	int ret = 0;
	int colvar = nv_prox_photo_colvar[0];
	u32 luxValue = 0;
	int als_on_wait_ms = 0;

	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");

	mutex_lock(&data->psals_mutex);
	if(atomic_read(&g_update_threshold_flg) == true)
	{
		atomic_set(&g_update_threshold_flg, false);
		gp2ap_set_ps_threshold(client,nv_proximity_sensor_far[colvar], nv_proximity_sensor_near[colvar]);
	}
	PSALS_DBG_LOG(KERN_INFO, "enable_ps_sensor=%d\n", data->enable_ps_sensor);
	ret = gp2ap_get_als_data(client, &cdata, &irdata);
	if (ret < 0)
	{
		PSALS_LOG(KERN_ERR, "gp2ap_get_als_data error. ret=%d\n", ret);
		goto exit;
	}

	data->cdata = cdata;
	data->irdata = irdata;
	luxValue = LuxCalculation(cdata, irdata, data);

	luxValue = luxValue < GP2AP_LUXVALUE_MAX ? luxValue : GP2AP_LUXVALUE_MAX;
	gp2ap_put_luxValue(data, luxValue);

	if (data->enable_ps_sensor <= 0)
	{
		ret = gp2ap_set_op_mode(client, GP2AP_SENSOR_ALS, OP_OFF);
		if (ret < 0)
		{
			PSALS_LOG(KERN_ERR, "gp2ap_set_op_mode error. ret=%d\n", ret);
			goto exit;
		}

		als_on_wait_ms = data->als_poll_delay - ALS_ON_WAIT_MS;
		PSALS_DBG_LOG(KERN_INFO, "queue_delayed_work(als_on_dwork) call. "
										"delay_ms=%d\n", als_on_wait_ms);
		queue_delayed_work(als_polling_wq, &data->als_on_dwork,
							msecs_to_jiffies(als_on_wait_ms));
	}
	else
	{
		als_on_wait_ms = data->als_poll_delay;
		PSALS_DBG_LOG(KERN_INFO, "queue_delayed_work(als_data_dwork) call. "
											"delay_ms=%d\n", als_on_wait_ms);
		queue_delayed_work(als_polling_wq, &data->als_data_dwork,
								msecs_to_jiffies(als_on_wait_ms));
	}

exit:
	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
	mutex_unlock(&data->psals_mutex);
}

static void gp2ap_work_handler(struct work_struct *work)
{
	struct gp2ap_data *data = container_of(work, struct gp2ap_data, dwork.work);
	struct i2c_client *client = data->client;
	int ret = 0;
	u32 dev_status_tmp = 0;
	u16 irdata;
	int colvar = nv_prox_photo_colvar[0];

	u8 detection[1] = {0};
	u8 buf[2];
	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	mutex_lock(&data->psals_mutex);
	msleep(30);
	data->ratio_reg = RatioCalculation(client, data);
	ret = gp2ap_i2c_read(client, REG_ADR_00, detection, 1);
	if(ret < 0)
	{
		PSALS_LOG(KERN_ERR, "gp2ap_i2c_read error. ret=%d\n", ret);
	}

	detection[0] &= PS_DETECTION_MASK;
	if(detection[0])
	{
		if((data->cdata_reg < 0x3E80) || (data->irdata_reg < 0x3E80))
		{
			if(data->ratio_reg < 80)
			{
				PSALS_DBG_LOG(KERN_INFO, "detection\n");
				buf[0] = 0x23;
				ret = gp2ap_i2c_write(client, REG_ADR_01, buf, 1);
				if(ret < 0)
				{
					PSALS_LOG(KERN_ERR, "gp2ap_i2c_write error. ret=%d\n", ret);
				}
			}
			else
			{
				msleep(30);
				ret = gp2ap_i2c_read(client, REG_ADR_0E, buf, 2);
				if(ret < 0)
				{
					PSALS_LOG(KERN_ERR, "gp2ap_i2c_read error. ret=%d\n", ret);
				}

				irdata = (buf[1] << 8) | buf[0];
				PSALS_DBG_LOG(KERN_INFO, "irdata=0x%04X\n", irdata);
				if(irdata >= 0x44C)
				{
					PSALS_DBG_LOG(KERN_INFO, "no detection\n");
					buf[0] = 0xC0;
					ret = gp2ap_i2c_write(client, REG_ADR_00, buf, 1);
					if(ret < 0)
					{
						PSALS_LOG(KERN_ERR, "gp2ap_i2c_write error. ret=%d\n", ret);
					}
					detection[0] = 0x00;
				}
				else
				{
					PSALS_DBG_LOG(KERN_INFO, "detection\n");
					buf[0] = 0x23;
					ret = gp2ap_i2c_write(client, REG_ADR_01, buf, 1);
					if(ret < 0)
					{
						PSALS_LOG(KERN_ERR, "gp2ap_i2c_write error. ret=%d\n", ret);
					}

				}
			}
		}
		else
		{
			buf[0] = 0xC0;
			ret = gp2ap_i2c_write(client, REG_ADR_00, buf, 1);
			if(ret < 0)
			{
				PSALS_LOG(KERN_ERR, "gp2ap_i2c_write error. ret=%d\n", ret);
			}
			detection[0] = 0x00;

		}
	}
	else
	{
		PSALS_DBG_LOG(KERN_INFO, "no detection\n");
		buf[0] = 0x63;
		ret = gp2ap_i2c_write(client, REG_ADR_01, buf, 1);
		if(ret < 0)
		{
			PSALS_LOG(KERN_ERR, "gp2ap_i2c_write error. ret=%d\n", ret);
		}

	}

	if(detection[0])
	{
		data->ps_detection = 1;
	}
	else
	{
		data->ps_detection = 0;
	}

	if (data->ps_detection == 0)
	{
		wake_lock_timeout(&gp2ap_wake_lock_input, GP2AP_WAKE_LOCK_INPUT_TIME);
		input_report_abs(data->input_dev_ps, ABS_DISTANCE_REPORT, GP2AP_PROXIMITY_DUMMY_VALUE);
		input_report_abs(data->input_dev_ps, ABS_DISTANCE_REPORT, 1);
		input_sync(data->input_dev_ps);
		PSALS_LOG(KERN_NOTICE, "ABS_DISTANCE_REPORT(detect:far)\n");
	}
	else
	{
		input_report_abs(data->input_dev_ps, ABS_DISTANCE_REPORT, GP2AP_PROXIMITY_DUMMY_VALUE);
		input_report_abs(data->input_dev_ps, ABS_DISTANCE_REPORT, 0);
		input_sync(data->input_dev_ps);
		PSALS_LOG(KERN_NOTICE, "ABS_DISTANCE_REPORT(detect:near)\n");
	}

	dev_status_tmp = atomic_read(&g_dev_status);
	PSALS_DBG_LOG(KERN_INFO, "dev_status_tmp=0x%08X\n", dev_status_tmp);

	PSALS_DBG_LOG(KERN_INFO, "enable_irq(%d) call.\n", data->ps_irq);
	enable_irq(data->ps_irq);

	if(atomic_read(&g_update_threshold_flg) == true)
	{
		atomic_set(&g_update_threshold_flg, false);
		gp2ap_set_ps_threshold(client,nv_proximity_sensor_far[colvar], nv_proximity_sensor_near[colvar]);
	}
	dev_status_tmp &= ~(GP2AP_DEV_STATUS_RESUME | GP2AP_DEV_STATUS_SUSPEND_INT);
	PSALS_DBG_LOG(KERN_INFO, "dev_status_tmp=0x%08X\n", dev_status_tmp);
	atomic_set(&g_dev_status, dev_status_tmp );

	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
	mutex_unlock(&data->psals_mutex);
}

static irqreturn_t gp2ap_interrupt(int vec, void *info)
{
	struct i2c_client *client=(struct i2c_client *)info;
	struct gp2ap_data *data = i2c_get_clientdata(client);
	u32 dev_status_tmp = 0;

	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	disable_irq_nosync(data->ps_irq);
	dev_status_tmp = atomic_read(&g_dev_status);
	PSALS_DBG_LOG(KERN_INFO, "dev_status_tmp=0x%08X\n", dev_status_tmp);
	if (dev_status_tmp & GP2AP_DEV_STATUS_SUSPEND)
	{
		PSALS_DBG_LOG(KERN_INFO, "set_status=%d\n",
					GP2AP_DEV_STATUS_SUSPEND_INT);
		atomic_set(&g_dev_status, dev_status_tmp | GP2AP_DEV_STATUS_SUSPEND_INT);
		wake_lock_timeout(&gp2ap_wake_lock, GP2AP_WAKE_LOCK_TIME);
	}
	else
	{
		gp2ap_reschedule_work(data, 0);
	}
	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");

	return IRQ_HANDLED;
}

static ssize_t gp2ap_show_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct gp2ap_data *data = input_get_drvdata(input_data);

	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
	return sprintf(buf, "%d\n", data->enable_ps_sensor);
}

static ssize_t gp2ap_store_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct gp2ap_data *data = input_get_drvdata(input_data);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	mutex_lock(&data->psals_mutex);
	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	PSALS_DBG_LOG(KERN_INFO, "val=%lu\n", val);
	PSALS_DBG_LOG(KERN_INFO, "before enable_ps_sensor=%d\n",
								(int)data->enable_ps_sensor);
	PSALS_LOG(KERN_NOTICE, "%s enable=%lu\n",__func__, val);
	if (val == 1)
	{
		if (data->enable_ps_sensor <= 0)
		{
			data->enable_ps_sensor=1;
			gp2ap_enable_ps_sensor(data->client, SENSOR_ENABLE);

		}
		else
		{
			data->enable_ps_sensor++;
		}
	}
	else
	{
		data->enable_ps_sensor--;

		if (data->enable_ps_sensor > 0)
		{
			PSALS_DBG_LOG(KERN_INFO, "no transaction.\n");
		}
		else
		{
			gp2ap_enable_ps_sensor(data->client, SENSOR_DISABLE);
			if (data->ps_detection != 0)
			{
				data->ps_detection = 0;
				input_report_abs(data->input_dev_ps, ABS_DISTANCE_REPORT, GP2AP_PROXIMITY_DUMMY_VALUE);
				input_report_abs(data->input_dev_ps, ABS_DISTANCE_REPORT, 1);
				input_sync(data->input_dev_ps);
				PSALS_LOG(KERN_NOTICE, "ABS_DISTANCE_REPORT(detect:far) [PS sensor DISABLE]\n");
			}
		}
	}
	PSALS_DBG_LOG(KERN_INFO, "after enable_ps_sensor=%d\n",
							(int)data->enable_ps_sensor);
	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
	mutex_unlock(&data->psals_mutex);

	return count;
}

static DEVICE_ATTR(ps_enable, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
				   gp2ap_show_enable_ps_sensor, gp2ap_store_enable_ps_sensor);

static ssize_t gp2ap_show_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct gp2ap_data *data = input_get_drvdata(input_data);

	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
	return sprintf(buf, "%d\n", data->enable_als_sensor);
}

static ssize_t gp2ap_store_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct gp2ap_data *data = input_get_drvdata(input_data);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	mutex_lock(&data->psals_mutex);
	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	PSALS_DBG_LOG(KERN_INFO, "val=%lu\n", val);
	PSALS_DBG_LOG(KERN_INFO, "before enable_als_sensor=%d\n",
							(int)data->enable_als_sensor);
	if (val == 1)
	{
		if (data->enable_als_sensor <= 0)
		{
			data->enable_als_sensor = 1;
			gp2ap_enable_als_sensor(data->client, SENSOR_ENABLE);
		}
		else
			data->enable_als_sensor++;
	}
	else
	{
		data->enable_als_sensor--;
		if (data->enable_als_sensor > 0)
		{
			PSALS_DBG_LOG(KERN_INFO, "no transaction.\n");
		}
		else
		{
			gp2ap_enable_als_sensor(data->client, SENSOR_DISABLE);
		}
	}
	PSALS_DBG_LOG(KERN_INFO, "after enable_als_sensor=%d\n",
							(int)data->enable_als_sensor);
	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
	mutex_unlock(&data->psals_mutex);

	return count;
}

static DEVICE_ATTR(als_enable, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
				   gp2ap_show_enable_als_sensor, gp2ap_store_enable_als_sensor);

static ssize_t gp2ap_show_als_poll_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct gp2ap_data *data = input_get_drvdata(input_data);

	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
	return sprintf(buf, "%d\n", data->als_poll_delay * 1000);
}

static ssize_t gp2ap_store_als_poll_delay
(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct gp2ap_data *data = input_get_drvdata(input_data);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	mutex_lock(&data->psals_mutex);
	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	PSALS_DBG_LOG(KERN_INFO, "val=%lu(us)\n", val);

	if (val < (ALS_ON_WAIT_MS * 1000))
	{
		PSALS_LOG(KERN_ERR, "bad param. val=%lu(us)\n", val);
		goto exit;
	}

	if (val < (ALS_GET_DATA_INTERVAL_MS / GP2AP_LUXVALUE_TABLE_MAX) * 1000)
		val = (ALS_GET_DATA_INTERVAL_MS / GP2AP_LUXVALUE_TABLE_MAX) * 1000;

	PSALS_DBG_LOG(KERN_INFO, "checked val(us)=%lu\n", val);

	data->als_poll_delay = val / 1000;

	data->als_mean_times = ALS_GET_DATA_INTERVAL_MS / data->als_poll_delay;
	data->als_polling_cnt_reset |= ALS_POLLING_CNT_RESET_STORE_POLL;
	PSALS_DBG_LOG(KERN_INFO, "als_polling_cnt_reset=0x%08X\n",
								data->als_polling_cnt_reset);

	if (data->enable_als_sensor > 0)
	{
		mutex_unlock(&data->psals_mutex);
		cancel_delayed_work_sync(&data->als_on_dwork);
		cancel_delayed_work_sync(&data->als_data_dwork);
		mutex_lock(&data->psals_mutex);
		PSALS_DBG_LOG(KERN_INFO, "queue_delayed_work(als_on_dwork) call. "
													"delay_ms=%d\n", 0);
		queue_delayed_work(als_polling_wq, &data->als_on_dwork,
											msecs_to_jiffies(0));
	}

exit:
	PSALS_DBG_LOG(KERN_INFO, "als_poll_delay=%d, als_mean_times=%d\n",
						data->als_poll_delay, data->als_mean_times);
	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
	mutex_unlock(&data->psals_mutex);
	return count;
}

static DEVICE_ATTR(als_poll_delay, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
				   gp2ap_show_als_poll_delay, gp2ap_store_als_poll_delay);

static ssize_t gp2ap_show_ps_detection(struct device *dev,
				struct device_attribute *attr, char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct gp2ap_data *data = input_get_drvdata(input_data);

    PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
    PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
    return sprintf(buf, "%d\n", data->ps_detection);
}

static DEVICE_ATTR(ps_detection, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
                    gp2ap_show_ps_detection, NULL);

static struct attribute *gp2ap_als_attributes[] = {
    &dev_attr_als_enable.attr,
    &dev_attr_als_poll_delay.attr,
    NULL
};

static struct attribute *gp2ap_ps_attributes[] = {
    &dev_attr_ps_enable.attr,
    &dev_attr_ps_detection.attr,
    NULL
};

static const struct attribute_group gp2ap_als_attr_group = {
	.attrs = gp2ap_als_attributes,
};

static const struct attribute_group gp2ap_ps_attr_group = {
    .attrs = gp2ap_ps_attributes,
};

static void gp2ap_set_als_mean_times(struct gp2ap_data *data, u32 mean_times)
{
	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	PSALS_DBG_LOG(KERN_INFO, "mean_times=%lu\n", (unsigned long int)mean_times);
	if ((mean_times == 0) ||
		((ALS_GET_DATA_INTERVAL_MS / mean_times) < ALS_ON_WAIT_MS))
	{
		PSALS_LOG(KERN_ERR, "bad param. mean_times=%lu\n",
						(unsigned long int)mean_times);
		goto exit;
	}

	if (mean_times > GP2AP_LUXVALUE_TABLE_MAX)
	{
		data->als_mean_times = GP2AP_LUXVALUE_TABLE_MAX;
	}
	else
	{
		data->als_mean_times = mean_times;
	}
	data->als_poll_delay = ALS_GET_DATA_INTERVAL_MS / data->als_mean_times;
	data->als_polling_cnt_reset |= ALS_POLLING_CNT_RESET_STORE_TIME;
	PSALS_DBG_LOG(KERN_INFO, "als_polling_cnt_reset=0x%08X\n",
								data->als_polling_cnt_reset);

	if (data->enable_als_sensor > 0)
	{
		mutex_unlock(&data->psals_mutex);
		cancel_delayed_work_sync(&data->als_on_dwork);
		cancel_delayed_work_sync(&data->als_data_dwork);
		mutex_lock(&data->psals_mutex);
		PSALS_DBG_LOG(KERN_INFO, "queue_delayed_work(als_on_dwork) call. "
														"delay_ms=%d\n", 0);
		queue_delayed_work(als_polling_wq, &data->als_on_dwork,
										msecs_to_jiffies(0));
	}

exit:
	PSALS_DBG_LOG(KERN_INFO, "als_poll_delay=%d, als_mean_times=%d\n",
						data->als_poll_delay, data->als_mean_times);
	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
}

static void gp2ap_set_sensor_nv(unsigned long ulArg)
{
	T_PSALS_IOCTL_NV* nv_data_type
							= (T_PSALS_IOCTL_NV*)ulArg;
	int i;

	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	switch (nv_data_type->ulItem)
	{
    case en_NV_PROXIMITY_SENSOR_NEAR_I:
        PSALS_DBG_LOG(KERN_INFO, "en_NV_PROXIMITY_SENSOR_NEAR_I");
        nv_status |= (0x01<<en_NV_PROXIMITY_SENSOR_NEAR_I);
        memcpy(nv_proximity_sensor_near,
               nv_data_type->ucData,
               min((size_t)nv_data_type->ulLength, sizeof(nv_proximity_sensor_near)));
        for (i = 0; i < ARRAY_SIZE(nv_proximity_sensor_near); i++)
        {
            PSALS_DBG_LOG(KERN_INFO, "nv_proximity_sensor_near[%d][%04X]",
                           i, nv_proximity_sensor_near[i]);
        }
        break;

    case en_NV_PROXIMITY_SENSOR_FAR_I:
        PSALS_DBG_LOG(KERN_INFO, "en_NV_PROXIMITY_SENSOR_FAR_I");
        nv_status |= (0x01<<en_NV_PROXIMITY_SENSOR_FAR_I);
        memcpy(nv_proximity_sensor_far,
               nv_data_type->ucData,
               min((size_t)nv_data_type->ulLength, sizeof(nv_proximity_sensor_far)));
        for (i = 0; i < ARRAY_SIZE(nv_proximity_sensor_far); i++)
        {
            PSALS_DBG_LOG(KERN_INFO, "nv_proximity_sensor_far[%d][%04X]",
                           i, nv_proximity_sensor_far[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_TH_0_I:
        PSALS_DBG_LOG(KERN_INFO, "en_NV_PHOTO_SENSOR_TH_0_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_TH_0_I);
        memcpy(nv_photosensor_th0,
               nv_data_type->ucData,
               min((size_t)nv_data_type->ulLength, sizeof(nv_photosensor_th0)));
        for (i = 0; i < ARRAY_SIZE(nv_photosensor_th0); i++)
        {
            PSALS_DBG_LOG(KERN_INFO, "nv_photosensor_th0[%d][%04X]",
                           i, nv_photosensor_th0[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_TH_1_I:
        PSALS_DBG_LOG(KERN_INFO, "en_NV_PHOTO_SENSOR_TH_1_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_TH_1_I);
        memcpy(nv_photosensor_th1,
               nv_data_type->ucData,
               min((size_t)nv_data_type->ulLength, sizeof(nv_photosensor_th1)));
        for (i = 0; i < ARRAY_SIZE(nv_photosensor_th1); i++)
        {
            PSALS_DBG_LOG(KERN_INFO, "nv_photosensor_th1[%d][%04X]",
                           i, nv_photosensor_th1[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_TH_2_I:
        PSALS_DBG_LOG(KERN_INFO, "en_NV_PHOTO_SENSOR_TH_2_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_TH_2_I);
        memcpy(nv_photosensor_th2,
               nv_data_type->ucData,
               min((size_t)nv_data_type->ulLength, sizeof(nv_photosensor_th2)));
        for (i = 0; i < ARRAY_SIZE(nv_photosensor_th2); i++)
        {
            PSALS_DBG_LOG(KERN_INFO, "nv_photosensor_th2[%d][%04X]",
                           i, nv_photosensor_th2[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_TH_3_I:
        PSALS_DBG_LOG(KERN_INFO, "en_NV_PHOTO_SENSOR_TH_3_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_TH_3_I);
        memcpy(nv_photosensor_th3,
               nv_data_type->ucData,
               min((size_t)nv_data_type->ulLength, sizeof(nv_photosensor_th3)));
        for (i = 0; i < ARRAY_SIZE(nv_photosensor_th3); i++)
        {
            PSALS_DBG_LOG(KERN_INFO, "nv_photosensor_th3[%d][%04X]",
                           i, nv_photosensor_th3[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_TH_4_I:
        PSALS_DBG_LOG(KERN_INFO, "en_NV_PHOTO_SENSOR_TH_4_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_TH_4_I);
        memcpy(nv_photosensor_th4,
               nv_data_type->ucData,
               min((size_t)nv_data_type->ulLength, sizeof(nv_photosensor_th4)));
        for (i = 0; i < ARRAY_SIZE(nv_photosensor_th4); i++)
        {
            PSALS_DBG_LOG(KERN_INFO, "nv_photosensor_th4[%d][%04X]",
                           i, nv_photosensor_th4[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_A_0_I:
        PSALS_DBG_LOG(KERN_INFO, "en_NV_PHOTO_SENSOR_A_0_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_A_0_I);
        memcpy(nv_photosensor_a0,
               nv_data_type->ucData,
               min((size_t)nv_data_type->ulLength, sizeof(nv_photosensor_a0)));
        for (i = 0; i < ARRAY_SIZE(nv_photosensor_a0); i++)
        {
            PSALS_DBG_LOG(KERN_INFO, "nv_photosensor_a0[%d][%04X]",
                           i, nv_photosensor_a0[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_A_1_I:
        PSALS_DBG_LOG(KERN_INFO, "en_NV_PHOTO_SENSOR_A_1_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_A_1_I);
        memcpy(nv_photosensor_a1,
               nv_data_type->ucData,
               min((size_t)nv_data_type->ulLength, sizeof(nv_photosensor_a1)));
        for (i = 0; i < ARRAY_SIZE(nv_photosensor_a1); i++)
        {
            PSALS_DBG_LOG(KERN_INFO, "nv_photosensor_a1[%d][%04X]",
                           i, nv_photosensor_a1[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_A_2_I:
        PSALS_DBG_LOG(KERN_INFO, "en_NV_PHOTO_SENSOR_A_2_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_A_2_I);
        memcpy(nv_photosensor_a2,
               nv_data_type->ucData,
               min((size_t)nv_data_type->ulLength, sizeof(nv_photosensor_a2)));
        for (i = 0; i < ARRAY_SIZE(nv_photosensor_a2); i++)
        {
            PSALS_DBG_LOG(KERN_INFO, "nv_photosensor_a2[%d][%04X]",
                           i, nv_photosensor_a2[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_A_3_I:
        PSALS_DBG_LOG(KERN_INFO, "en_NV_PHOTO_SENSOR_A_3_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_A_3_I);
        memcpy(nv_photosensor_a3,
               nv_data_type->ucData,
               min((size_t)nv_data_type->ulLength, sizeof(nv_photosensor_a3)));
        for (i = 0; i < ARRAY_SIZE(nv_photosensor_a3); i++)
        {
            PSALS_DBG_LOG(KERN_INFO, "nv_photosensor_a3[%d][%04X]",
                           i, nv_photosensor_a3[i]);
        }
        break;


    case en_NV_PHOTO_SENSOR_A_4_I:
        PSALS_DBG_LOG(KERN_INFO, "en_NV_PHOTO_SENSOR_A_4_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_A_4_I);
        memcpy(nv_photosensor_a4,
               nv_data_type->ucData,
               min((size_t)nv_data_type->ulLength, sizeof(nv_photosensor_a4)));
        for (i = 0; i < ARRAY_SIZE(nv_photosensor_a4); i++)
        {
            PSALS_DBG_LOG(KERN_INFO, "nv_photosensor_a4[%d][%04X]",
                           i, nv_photosensor_a4[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_B_0_I:
        PSALS_DBG_LOG(KERN_INFO, "en_NV_PHOTO_SENSOR_B_0_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_B_0_I);
        memcpy(nv_photosensor_b0,
               nv_data_type->ucData,
               min((size_t)nv_data_type->ulLength, sizeof(nv_photosensor_b0)));
        for (i = 0; i < ARRAY_SIZE(nv_photosensor_b0); i++)
        {
            PSALS_DBG_LOG(KERN_INFO, "nv_photosensor_b0[%d][%04X]",
                           i, nv_photosensor_b0[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_B_1_I:
        PSALS_DBG_LOG(KERN_INFO, "en_NV_PHOTO_SENSOR_B_1_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_B_1_I);
        memcpy(nv_photosensor_b1,
               nv_data_type->ucData,
               min((size_t)nv_data_type->ulLength, sizeof(nv_photosensor_b1)));
        for (i = 0; i < ARRAY_SIZE(nv_photosensor_b1); i++)
        {
            PSALS_DBG_LOG(KERN_INFO, "nv_photosensor_b1[%d][%04X]",
                           i, nv_photosensor_b1[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_B_2_I:
        PSALS_DBG_LOG(KERN_INFO, "en_NV_PHOTO_SENSOR_B_2_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_B_2_I);
        memcpy(nv_photosensor_b2,
               nv_data_type->ucData,
               min((size_t)nv_data_type->ulLength, sizeof(nv_photosensor_b2)));
        for (i = 0; i < ARRAY_SIZE(nv_photosensor_b2); i++)
        {
            PSALS_DBG_LOG(KERN_INFO, "nv_photosensor_b2[%d][%04X]",
                           i, nv_photosensor_b2[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_B_3_I:
        PSALS_DBG_LOG(KERN_INFO, "en_NV_PHOTO_SENSOR_B_3_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_B_3_I);
        memcpy(nv_photosensor_b3,
               nv_data_type->ucData,
               min((size_t)nv_data_type->ulLength, sizeof(nv_photosensor_b3)));
        for (i = 0; i < ARRAY_SIZE(nv_photosensor_b3); i++)
        {
            PSALS_DBG_LOG(KERN_INFO, "nv_photosensor_b3[%d][%04X]",
                           i, nv_photosensor_b3[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_B_4_I:
        PSALS_DBG_LOG(KERN_INFO, "en_NV_PHOTO_SENSOR_B_4_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_B_4_I);
        memcpy(nv_photosensor_b4,
               nv_data_type->ucData,
               min((size_t)nv_data_type->ulLength, sizeof(nv_photosensor_b4)));
        for (i = 0; i < ARRAY_SIZE(nv_photosensor_b4); i++)
        {
            PSALS_DBG_LOG(KERN_INFO, "nv_photosensor_b4[%d][%04X]",
                           i, nv_photosensor_b4[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_1_R_I:
        PSALS_DBG_LOG(KERN_INFO, "en_NV_PHOTO_SENSOR_1_R_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_1_R_I);
        memcpy(nv_photosensor_1r,
               nv_data_type->ucData,
               min((size_t)nv_data_type->ulLength, sizeof(nv_photosensor_1r)));
        for (i = 0; i < ARRAY_SIZE(nv_photosensor_1r); i++)
        {
            PSALS_DBG_LOG(KERN_INFO, "nv_photosensor_1r[%d][%04X]",
                           i, nv_photosensor_1r[i]);
        }
        break;

    case en_NV_PHOTO_SENSOR_2_R_I:
        PSALS_DBG_LOG(KERN_INFO, "en_NV_PHOTO_SENSOR_2_R_I");
        nv_status |= (0x01<<en_NV_PHOTO_SENSOR_2_R_I);
        memcpy(nv_photosensor_2r,
               nv_data_type->ucData,
               min((size_t)nv_data_type->ulLength, sizeof(nv_photosensor_2r)));
        for (i = 0; i < ARRAY_SIZE(nv_photosensor_2r); i++)
        {
            PSALS_DBG_LOG(KERN_INFO, "nv_photosensor_2r[%d][%04X]",
                           i, nv_photosensor_2r[i]);
        }
        break;
#ifdef CONFIG_KC_COLOR_VARIATION
	case en_NV_PROX_PHOTO_COLVAR_I:
		PSALS_DBG_LOG(KERN_INFO, "en_NV_PROX_PHOTO_COLVAR_I\n");
		nv_status |= (0x01<<en_NV_PROX_PHOTO_COLVAR_I);
		memcpy(nv_prox_photo_colvar,
				nv_data_type->ucData,(size_t)nv_data_type->ulLength);
		if (nv_prox_photo_colvar[0] >= GP2AP_COLOR_VARI)
		{
			nv_prox_photo_colvar[0] = 0;
		}
		for (i = 0; i < sizeof(nv_prox_photo_colvar)/
				sizeof(nv_prox_photo_colvar[0]); i++)
		{
			PSALS_DBG_LOG(KERN_INFO, "nv_prox_photo_colvar[%d]=0x%04X\n",
											i, nv_prox_photo_colvar[i]);
		}
		break;
#endif
	default :
		PSALS_LOG(KERN_ERR, "set_sensor_nv: Can't set nv data\n");
		break;
	}
	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
}

static int gp2ap_open(struct inode *inode_type, struct file *file)
{
	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
	return 0;
}

static int gp2ap_release(struct inode *inode_type, struct file *file)
{
	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
	return 0;
}

static long gp2ap_ioctl(struct file *file_type,
						  unsigned int unCmd, unsigned long ulArg)
{
	s32 nRet = -EINVAL;
	u32 dev_status = 0;
	T_PSALS_IOCTL_PS_DETECTION ps_detection_type;
	T_PSALS_IOCTL_ALS_MEAN_TIMES als_mean_times_type;
	T_PSALS_IOCTL_ALS_LUX_AVE als_lux_ave_type;
	T_PSALS_IOCTL_PS_THRESHOLD ps_threshold_type;

	struct gp2ap_data *data = i2c_get_clientdata(client_gp2ap);

	mutex_lock(&data->psals_mutex);
	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");

	memset((void*)&ps_detection_type, 0,
						sizeof(T_PSALS_IOCTL_PS_DETECTION));
	memset((void*)&als_mean_times_type, 0,
						sizeof(T_PSALS_IOCTL_ALS_MEAN_TIMES));
	memset((void*)&als_lux_ave_type, 0,
						sizeof(T_PSALS_IOCTL_ALS_LUX_AVE));
	memset((void*)&ps_threshold_type, 0,
						sizeof(T_PSALS_IOCTL_PS_THRESHOLD));

	switch (unCmd)
	{
		case IOCTL_PS_DETECTION_GET:
			PSALS_DBG_LOG(KERN_INFO, "IOCTL_PS_DETECTION_GET START\n");
			nRet = copy_from_user(&ps_detection_type,
					(void __user *)ulArg, sizeof(T_PSALS_IOCTL_PS_DETECTION));
			if (nRet)
			{
				PSALS_LOG(KERN_ERR, "error : (unCmd = IOCTL_PS_DETECTION_GET)"
																		"\n");
				mutex_unlock(&data->psals_mutex);
				return -EFAULT;
			}
			ps_detection_type.ulps_detection = data->ps_detection;
			PSALS_DBG_LOG(KERN_INFO, "ulps_detection=%lu\n",
							ps_detection_type.ulps_detection);
			nRet = copy_to_user((void *)(ulArg),
					 &ps_detection_type, sizeof(T_PSALS_IOCTL_PS_DETECTION));
			if (nRet)
			{
				PSALS_LOG(KERN_ERR, "error : (unCmd = IOCTL_PS_DETECTION_GET)"
																		"\n");
				mutex_unlock(&data->psals_mutex);
				return -EFAULT;
			}
			PSALS_DBG_LOG(KERN_INFO, "IOCTL_PS_DETECTION_GET END\n");
			break;
		case IOCTL_ALS_MEAN_TIMES_SET:
			PSALS_DBG_LOG(KERN_INFO, "IOCTL_ALS_MEAN_TIMES_SET START\n");
			nRet = copy_from_user(&als_mean_times_type,
					(void __user *)ulArg, sizeof(T_PSALS_IOCTL_ALS_MEAN_TIMES));
			if (nRet)
			{
				PSALS_LOG(KERN_ERR, "error : (unCmd = IOCTL_ALS_MEAN_TIMES_SET)"
																		"\n");
				mutex_unlock(&data->psals_mutex);
				return -EFAULT;
			}
			gp2ap_set_als_mean_times(data, als_mean_times_type.ulals_mean_times);
			nRet = copy_to_user((void *)(ulArg),
					 &als_mean_times_type, sizeof(T_PSALS_IOCTL_ALS_MEAN_TIMES));
			if (nRet)
			{
				PSALS_LOG(KERN_ERR, "error : (unCmd = IOCTL_ALS_MEAN_TIMES_SET)"
																		"\n");
				mutex_unlock(&data->psals_mutex);
				return -EFAULT;
			}
			break;
		case IOCTL_ALS_LUX_AVE_GET:
			PSALS_DBG_LOG(KERN_INFO, "IOCTL_ALS_LUX_AVE_GET START\n");
			nRet = copy_from_user(&als_lux_ave_type,
					(void __user *)ulArg, sizeof(T_PSALS_IOCTL_ALS_LUX_AVE));
			if (nRet)
			{
				PSALS_LOG(KERN_ERR, "error : (unCmd = IOCTL_ALS_LUX_AVE_GET)"
																		"\n");
				mutex_unlock(&data->psals_mutex);
				return -EFAULT;
			}
			als_lux_ave_type.ulals_lux_ave = data->als_lux_ave;
			als_lux_ave_type.lcdata = data->cdata;
			als_lux_ave_type.lirdata = data->irdata;
			PSALS_DBG_LOG(KERN_INFO, "ulals_lux_ave=%lu,lcdata=0x%04X,"
				"lirdata=0x%04X\n", als_lux_ave_type.ulals_lux_ave,
				(unsigned int)als_lux_ave_type.lcdata,
				(unsigned int)als_lux_ave_type.lirdata);
			nRet = copy_to_user((void *)(ulArg),
					 &als_lux_ave_type, sizeof(T_PSALS_IOCTL_ALS_LUX_AVE));
			if (nRet)
			{
				PSALS_DBG_LOG(KERN_INFO, "error : (unCmd = "
								"IOCTL_ALS_LUX_AVE_GET)\n");
				mutex_unlock(&data->psals_mutex);
				return -EFAULT;
			}
			break;
		case IOCTL_PSALS_NV_DATA_SET:
			{
				T_PSALS_IOCTL_NV sensor_nv_type;
				PSALS_DBG_LOG(KERN_INFO, "IOCTL_PSALS_NV_DATA_SET START\n");
				memset((void*)&sensor_nv_type, 0,
							sizeof(T_PSALS_IOCTL_NV));
				nRet = copy_from_user(&sensor_nv_type,
						(void __user *)ulArg, sizeof(T_PSALS_IOCTL_NV));
				if (!nRet)
				{
					gp2ap_set_sensor_nv((unsigned long)&sensor_nv_type);
					nRet = 0;
				}
			}
			break;
		case IOCTL_PS_THRESHOLD_SET:
			{
				int colvar = nv_prox_photo_colvar[0];
				PSALS_DBG_LOG(KERN_INFO, "IOCTL_PS_THRESHOLD_SET\n");
				nRet = copy_from_user(&ps_threshold_type,
					(void __user *)ulArg, sizeof(T_PSALS_IOCTL_PS_THRESHOLD));
				if (nRet)
				{
				PSALS_LOG(KERN_ERR, "error : (unCmd = IOCTL_PS_THRSHOLD_SET)"
																		"\n");
					mutex_unlock(&data->psals_mutex);
					return -EFAULT;
				}

				if(data->enable_als_sensor > 0){
					atomic_set(&g_update_threshold_flg, true);
				} else {
					dev_status = atomic_read(&g_dev_status);
					dev_status &= GP2AP_DEV_STATUS_SUSPEND_INT;
					if(dev_status){
						atomic_set(&g_update_threshold_flg, true);
					}else{
						atomic_set(&g_update_threshold_flg, false);
						gp2ap_set_ps_threshold(client_gp2ap, nv_proximity_sensor_far[colvar],
										nv_proximity_sensor_near[colvar]);
					}
				}
			}
			break;
		default:
			PSALS_LOG(KERN_ERR, "default err\n");
			break;
	}
	PSALS_DBG_LOG(KERN_INFO, "[OUT] nRet=%d\n",nRet);
	mutex_unlock(&data->psals_mutex);
	return nRet;
}

static struct file_operations gp2ap_fops = {
	.owner = THIS_MODULE,
	.open = gp2ap_open,
	.release = gp2ap_release,
	.unlocked_ioctl = gp2ap_ioctl,
};

static struct miscdevice gp2ap_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_FILE_NAME,
	.fops = &gp2ap_fops,
};

static int gp2ap_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct gp2ap_data *data;
	int err = 0;
	int ps_irq_num = 0;

	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	data = kzalloc(sizeof(struct gp2ap_data), GFP_KERNEL);
	if (!data)
	{
		err = -ENOMEM;
		goto exit;
	}
	data->client = client;

	i2c_set_clientdata(client, data);

	data->enable_als_sensor = 0;
	data->enable_ps_sensor = 0;
	data->ps_detection = 0;
	data->ps_data = 0;
	data->als_poll_delay = ALS_POLL_DELAY_MS_DEF;
	data->cdata = 0;
	data->irdata = 0;
	data->als_lux = 0;
	memset(data->luxValue_table, 0, sizeof(data->luxValue_table));
	data->als_lux_ave = 0;
	data->als_polling_cnt = 0;
	data->als_mean_times = ALS_MEAN_TIME_DEF;
	data->als_polling_cnt_reset = ALS_POLLING_CNT_RESET_INIT;
	PSALS_DBG_LOG(KERN_INFO, "als_polling_cnt_reset=0x%08X\n",
								data->als_polling_cnt_reset);
	data->op_mode = GP2AP_OP_MODE_NONE;
	data->op_sensor = GP2AP_SENSOR_NONE;
	data->ratio_reg = 0;
	data->cdata_reg = 0;
	data->irdata_reg = 0;
	atomic_set(&g_update_threshold_flg,false);

	mutex_init(&data->psals_mutex);
	wake_lock_init(&gp2ap_wake_lock, WAKE_LOCK_SUSPEND, "gp2ap_ps");
	wake_lock_init(&gp2ap_wake_lock_input, WAKE_LOCK_SUSPEND, "gp2ap_ps_input");

    of_property_read_u32(client->dev.of_node, "light-vdd-l5-min-voltage", &als_reg_data_l5.min_uV);
    of_property_read_u32(client->dev.of_node, "light-vdd-l5-max-voltage", &als_reg_data_l5.max_uV);
    of_property_read_u32(client->dev.of_node, "light-vdd-l5-on-load-current", &als_reg_data_l5.on_load_uA);
    of_property_read_u32(client->dev.of_node, "light-vdd-l5-off-load-current", &als_reg_data_l5.off_load_uA);
    PSALS_LOG(KERN_NOTICE, "[ALS_PS]%s regulator min_uV = %d, max_uV = %d, on_load_uA = %d, off_load_uA = %d\n",
        __func__, als_reg_data_l5.min_uV, als_reg_data_l5.max_uV, als_reg_data_l5.on_load_uA, als_reg_data_l5.off_load_uA);

    als_reg_data_l5.vdd_reg = regulator_get(&client->dev, "light-vdd-l5");
    if( IS_ERR(als_reg_data_l5.vdd_reg) ) {
        PSALS_LOG(KERN_ERR, "failed regulator_get \n");
        goto exit_kfree;
    }

    err = regulator_set_voltage(als_reg_data_l5.vdd_reg, als_reg_data_l5.min_uV, als_reg_data_l5.max_uV);
    if( err ) {
        PSALS_LOG(KERN_ERR, "[ALS_PS]%s regulator_set_voltage(L5) fail. err=%d\n", __func__, err);
        goto exit_kfree;
    }

    err = regulator_set_optimum_mode(als_reg_data_l5.vdd_reg, als_reg_data_l5.on_load_uA);
    if( err < 0 ) {
        PSALS_LOG(KERN_ERR, "[ALS_PS]%s regulator_set_optimum_mode(L5) fail. err=%d\n", __func__, err);
        goto exit_kfree;
    }
    usleep_range(1000,1000);

    err = regulator_enable(als_reg_data_l5.vdd_reg);
    if( err ) {
        PSALS_LOG(KERN_ERR, "[ALS_PS]%s regulator_enable fail. err=%d\n", __func__, err);
        goto exit_kfree;
    }

    of_property_read_u32(client->dev.of_node, "prox-vdd-l5-min-voltage", &prox_reg_data_l5.min_uV);
    of_property_read_u32(client->dev.of_node, "prox-vdd-l5-max-voltage", &prox_reg_data_l5.max_uV);
    of_property_read_u32(client->dev.of_node, "prox-vdd-l5-on-load-current", &prox_reg_data_l5.on_load_uA);
    of_property_read_u32(client->dev.of_node, "prox-vdd-l5-off-load-current", &prox_reg_data_l5.off_load_uA);
    PSALS_LOG(KERN_NOTICE, "[ALS_PS]%s regulator min_uV = %d, max_uV = %d, on_load_uA = %d, off_load_uA = %d\n",
        __func__, prox_reg_data_l5.min_uV, prox_reg_data_l5.max_uV, prox_reg_data_l5.on_load_uA, prox_reg_data_l5.off_load_uA);

    prox_reg_data_l5.vdd_reg = regulator_get(&client->dev, "prox-vdd-l5");
    if( IS_ERR(prox_reg_data_l5.vdd_reg) ) {
        PSALS_LOG(KERN_ERR, "failed regulator_get \n");
        goto exit_kfree;
    }

    err = regulator_set_voltage(prox_reg_data_l5.vdd_reg, prox_reg_data_l5.min_uV, prox_reg_data_l5.max_uV);
    if( err ) {
        PSALS_LOG(KERN_ERR, "[ALS_PS]%s regulator_set_voltage(L5) fail. err=%d\n", __func__, err);
        goto exit_kfree;
    }

    err = regulator_set_optimum_mode(prox_reg_data_l5.vdd_reg, prox_reg_data_l5.on_load_uA);
    if( err < 0 ) {
        PSALS_LOG(KERN_ERR, "[ALS_PS]%s regulator_set_optimum_mode(L5) fail. err=%d\n", __func__, err);
        goto exit_kfree;
    }
    usleep_range(1000,1000);

    err = regulator_enable(prox_reg_data_l5.vdd_reg);
    if( err ) {
        PSALS_LOG(KERN_ERR, "[ALS_PS]%s regulator_enable(L5) fail. err=%d\n", __func__, err);
        goto exit_kfree;
    }

    ps_irq_num = of_get_named_gpio(client->dev.of_node, "sharp,irq-gpio", 0);
    if( ps_irq_num < 0 ) {
        PSALS_LOG(KERN_ERR, "failed to get irq GPIO=%d\n", ps_irq_num);
            goto exit_kfree;
    }
    err = gpio_request(ps_irq_num,DEVICE_NAME);
    if( err ) {
        PSALS_LOG(KERN_ERR, "failed to request GPIO=%d, err=%d\n",
            ps_irq_num, err);
            goto exit_kfree;
    }

	data->ps_irq = gpio_to_irq(ps_irq_num);
	err = request_any_context_irq(data->ps_irq, gp2ap_interrupt,
			IRQ_TYPE_EDGE_FALLING, DEVICE_NAME, (void *)client);
	PSALS_DBG_LOG(KERN_INFO, "ps_irq_num=%d, ps_irq=%d\n", ps_irq_num, data->ps_irq);
	PSALS_DBG_LOG(KERN_INFO, "request_any_context_irq() called. err=%d\n",err);
	if (err < 0)
	{
		PSALS_LOG(KERN_ERR, "Could not allocate GPIO_PROX_INT(%d) ! err=%d\n",
				 ps_irq_num,err);

		goto exit_gpio_free2;
	}
	gp2ap_ps_irq_cnt++;
	gp2ap_disable_ps_irq(data);

    of_property_read_u32(client->dev.of_node, "light-vdd-min-voltage", &als_reg_data.min_uV);
    of_property_read_u32(client->dev.of_node, "light-vdd-max-voltage", &als_reg_data.max_uV);
    of_property_read_u32(client->dev.of_node, "light-vdd-on-load-current", &als_reg_data.on_load_uA);
    of_property_read_u32(client->dev.of_node, "light-vdd-off-load-current", &als_reg_data.off_load_uA);
    PSALS_LOG(KERN_NOTICE, "[ALS_PS]%s regulator min_uV = %d, max_uV = %d, on_load_uA = %d, off_load_uA = %d\n",
        __func__, als_reg_data.min_uV, als_reg_data.max_uV, als_reg_data.on_load_uA, als_reg_data.off_load_uA);

    als_reg_data.vdd_reg = regulator_get(&client->dev, "light-vdd");
    if( IS_ERR(als_reg_data.vdd_reg) ) {
        PSALS_LOG(KERN_ERR, "failed regulator_get \n");
        goto exit_kfree;
    }

    err = regulator_set_voltage(als_reg_data.vdd_reg, als_reg_data.min_uV, als_reg_data.max_uV);
    if( err ) {
        PSALS_LOG(KERN_ERR, "[ALS_PS]%s regulator_set_voltage fail. err=%d\n", __func__, err);
        goto exit_kfree;
    }

    err = regulator_set_optimum_mode(als_reg_data.vdd_reg, als_reg_data.on_load_uA);
    if( err < 0 ) {
        PSALS_LOG(KERN_ERR, "[ALS_PS]%s regulator_set_optimum_mode fail. err=%d\n", __func__, err);
        goto exit_kfree;
    }
    usleep_range(1000,1000);

    err = regulator_enable(als_reg_data.vdd_reg);
    if( err ) {
        PSALS_LOG(KERN_ERR, "[ALS_PS]%s regulator_enable fail. err=%d\n", __func__, err);
        goto exit_kfree;
    }

    of_property_read_u32(client->dev.of_node, "prox-vdd-min-voltage", &prox_reg_data.min_uV);
    of_property_read_u32(client->dev.of_node, "prox-vdd-max-voltage", &prox_reg_data.max_uV);
    of_property_read_u32(client->dev.of_node, "prox-vdd-on-load-current", &prox_reg_data.on_load_uA);
    of_property_read_u32(client->dev.of_node, "prox-vdd-off-load-current", &prox_reg_data.off_load_uA);
    PSALS_LOG(KERN_NOTICE, "[ALS_PS]%s regulator min_uV = %d, max_uV = %d, on_load_uA = %d, off_load_uA = %d\n",
        __func__, prox_reg_data.min_uV, prox_reg_data.max_uV, prox_reg_data.on_load_uA, prox_reg_data.off_load_uA);

    prox_reg_data.vdd_reg = regulator_get(&client->dev, "prox-vdd");
    if( IS_ERR(prox_reg_data.vdd_reg) ) {
        PSALS_LOG(KERN_ERR, "failed regulator_get \n");
        goto exit_kfree;
    }

    err = regulator_set_voltage(prox_reg_data.vdd_reg, prox_reg_data.min_uV, prox_reg_data.max_uV);
    if( err ) {
        PSALS_LOG(KERN_ERR, "[ALS_PS]%s regulator_set_voltage fail. err=%d\n", __func__, err);
        goto exit_kfree;
    }

    err = regulator_set_optimum_mode(prox_reg_data.vdd_reg, prox_reg_data.on_load_uA);
    if( err < 0 ) {
        PSALS_LOG(KERN_ERR, "[ALS_PS]%s regulator_set_optimum_mode fail. err=%d\n", __func__, err);
        goto exit_kfree;
    }
    usleep_range(1000,1000);

    err = regulator_enable(prox_reg_data.vdd_reg);
    if( err ) {
        PSALS_LOG(KERN_ERR, "[ALS_PS]%s regulator_enable fail. err=%d\n", __func__, err);
        goto exit_kfree;
    }

	INIT_DELAYED_WORK(&data->dwork, gp2ap_work_handler);
	INIT_DELAYED_WORK(&data->als_on_dwork, gp2ap_als_on_work_handler);
	INIT_DELAYED_WORK(&data->als_data_dwork, gp2ap_als_data_work_handler);

	err = gp2ap_init_client(client);
	if (err)
	{
		PSALS_LOG(KERN_ERR, "Failed gp2ap_init_client\n");
		goto exit_gpio_free2;
	}

	data->input_dev_als = input_allocate_device();
	if (!data->input_dev_als)
	{
		err = -ENOMEM;
		PSALS_LOG(KERN_ERR, "Failed to allocate input device als\n");
		goto exit_free_irq;
	}

	data->input_dev_ps = input_allocate_device();
	if (!data->input_dev_ps)
	{
		err = -ENOMEM;
		PSALS_LOG(KERN_ERR, "Failed to allocate input device ps\n");
		goto exit_free_dev_als;
	}

	set_bit(EV_ABS, data->input_dev_als->evbit);
	set_bit(EV_ABS, data->input_dev_ps->evbit);
	input_set_capability(data->input_dev_als, EV_ABS, ABS_LUX_REPORT);
	input_set_capability(data->input_dev_ps, EV_ABS, ABS_DISTANCE_REPORT);
	input_set_abs_params(data->input_dev_als, ABS_LUX_REPORT, 0,
										GP2AP_LUXVALUE_MAX, 0, 0);
	input_set_abs_params(data->input_dev_ps, ABS_DISTANCE_REPORT, 0, 1, 0, 0);
	data->input_dev_als->name = LIGHT_SENSOR_NAME;
	data->input_dev_ps->name = PROXIMITY_SENSOR_NAME;
	data->input_dev_als->id.bustype = BUS_I2C;
	data->input_dev_ps->id.bustype = BUS_I2C;
	data->input_dev_als->dev.parent = &client->dev;
	data->input_dev_ps->dev.parent = &client->dev;
	input_set_drvdata(data->input_dev_als, data);
	input_set_drvdata(data->input_dev_ps, data);
	err = input_register_device(data->input_dev_als);
	PSALS_DBG_LOG(KERN_INFO, "input_register_device(input_dev_als) called. "
															"err=%d\n",err);
	if (err)
	{
		err = -ENOMEM;
		PSALS_LOG(KERN_ERR, "Unable to register input device als: %s\n",
			   data->input_dev_als->name);
		goto exit_free_dev_ps;
	}

	err = input_register_device(data->input_dev_ps);
	PSALS_DBG_LOG(KERN_INFO, "input_register_device(input_dev_ps) called. "
															"err=%d\n",err);
	if (err)
	{
		err = -ENOMEM;
		PSALS_LOG(KERN_ERR, "Unable to register input device ps: %s\n",
			   data->input_dev_ps->name);
		goto exit_unregister_dev_als;
	}

	err = sysfs_create_group(&data->input_dev_als->dev.kobj, &gp2ap_als_attr_group);
	PSALS_DBG_LOG(KERN_INFO, "sysfs_create_group(input_dev_als) called. "
														"err=%d\n",err);
	if (err)
	{
		PSALS_LOG(KERN_ERR, "Failed sysfs_create_group\n");
		goto exit_unregister_dev_ps;
	}

	err = sysfs_create_group(&data->input_dev_ps->dev.kobj, &gp2ap_ps_attr_group);
	PSALS_DBG_LOG(KERN_INFO, "sysfs_create_group(input_dev_ps) called. "
														"err=%d\n",err);
	if (err)
	{
		PSALS_LOG(KERN_ERR, "Failed sysfs_create_group\n");
		goto exit_sysfs_remove_als;
	}

	device_init_wakeup(&client->dev, 1);
	atomic_set(&g_dev_status, GP2AP_DEV_STATUS_INIT);

	err = misc_register(&gp2ap_device);
	PSALS_DBG_LOG(KERN_INFO, "misc_register() called. err=%d\n",err);
	if (err)
	{
		PSALS_LOG(KERN_ERR, "misc_register failed\n");
		goto exit_sysfs_remove_ps;
	}
	client_gp2ap = client;
	PSALS_DBG_LOG(KERN_INFO, "[OUT] err=%d\n", err);
	return 0;

exit_sysfs_remove_ps:
	sysfs_remove_group(&data->input_dev_ps->dev.kobj, &gp2ap_ps_attr_group);
exit_sysfs_remove_als:
	sysfs_remove_group(&data->input_dev_als->dev.kobj, &gp2ap_als_attr_group);
exit_unregister_dev_ps:
	input_unregister_device(data->input_dev_ps);
exit_unregister_dev_als:
	input_unregister_device(data->input_dev_als);
exit_free_dev_ps:
	input_free_device(data->input_dev_ps);
exit_free_dev_als:
	input_free_device(data->input_dev_als);
exit_free_irq:
	free_irq(data->ps_irq, client);
exit_gpio_free2:
	gpio_free(ps_irq_num);
exit_kfree:
	kfree(data);
exit:
	PSALS_DBG_LOG(KERN_INFO, "[OUT] err=%d\n", err);
	return err;
}

static int gp2ap_remove(struct i2c_client *client)
{
	struct gp2ap_data *data = i2c_get_clientdata(client);

	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	sysfs_remove_group(&data->input_dev_als->dev.kobj, &gp2ap_als_attr_group);
	sysfs_remove_group(&data->input_dev_ps->dev.kobj, &gp2ap_ps_attr_group);

	input_unregister_device(data->input_dev_als);
	input_unregister_device(data->input_dev_ps);

	input_free_device(data->input_dev_als);
	input_free_device(data->input_dev_ps);

	wake_lock_destroy(&gp2ap_wake_lock);
	wake_lock_destroy(&gp2ap_wake_lock_input);

	free_irq(data->ps_irq, client);

	misc_deregister(&gp2ap_device);

	data->enable_ps_sensor = 0;
	data->enable_als_sensor = 0;

	gp2ap_enable_als_sensor(client, SENSOR_DISABLE);
	gp2ap_enable_ps_sensor(client, SENSOR_DISABLE);

	gp2ap_fin_client(client);

	mutex_destroy(&data->psals_mutex);

	kfree(data);
	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
	return 0;
}

static int gp2ap_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct gp2ap_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->psals_mutex);
	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");

	if (device_may_wakeup(&client->dev))
	{
		if( data->enable_ps_sensor > 0 ){
			enable_irq_wake(data->ps_irq);
		}
	}
	else
	{
		PSALS_LOG(KERN_ERR, "failed device_may_wakeup\n");
	}
	mutex_unlock(&data->psals_mutex);
	cancel_delayed_work_sync(&data->als_on_dwork);
	cancel_delayed_work_sync(&data->als_data_dwork);
	mutex_lock(&data->psals_mutex);
	PSALS_DBG_LOG(KERN_INFO, "cancel_delayed_work(als_on_dwork) call. ");
	PSALS_DBG_LOG(KERN_INFO, "cancel_delayed_work(als_data_dwork) call. ");
	atomic_set(&g_dev_status, GP2AP_DEV_STATUS_SUSPEND);
	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
	mutex_unlock(&data->psals_mutex);

	if(data->enable_als_sensor > 0){
		if(als_reg_data.vdd_reg){
			ret = regulator_set_optimum_mode(als_reg_data.vdd_reg, als_reg_data.off_load_uA);
			if (ret < 0) {
				PSALS_LOG(KERN_ERR, "regulator_set_optimum_mode fail. ret=%d\n", ret);
			}
			PSALS_DBG_LOG(KERN_INFO, "regulator_set_optimum_mode ret=%d\n",ret);
		}
		else{
			PSALS_LOG(KERN_ERR, "error : als_reg_data [NULL] enable als_sensor failed\n");
		}
		if(als_reg_data_l5.vdd_reg){
			ret = regulator_set_optimum_mode(als_reg_data_l5.vdd_reg, als_reg_data_l5.off_load_uA);
			if (ret < 0) {
				PSALS_LOG(KERN_ERR, "regulator_set_optimum_mode(L5) fail. ret=%d\n", ret);
			}
			PSALS_DBG_LOG(KERN_INFO, "regulator_set_optimum_mode(L5) ret=%d\n",ret);
		}
		else{
			PSALS_LOG(KERN_ERR, "error : als_reg_data(L5) [NULL] enable als_sensor failed\n");
		}
	}

	return 0;
}

static int gp2ap_resume(struct i2c_client *client)
{
	struct gp2ap_data *data = i2c_get_clientdata(client);
	u32 dev_status_tmp = 0;
	int ret;

	mutex_lock(&data->psals_mutex);
	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	PSALS_DBG_LOG(KERN_INFO, "data->enable_ps_sensor=%d\n",
								data->enable_ps_sensor);
	if (device_may_wakeup(&client->dev))
	{
		if( data->enable_ps_sensor > 0 ){
			disable_irq_wake(data->ps_irq);
		}
	}
	else
	{
		PSALS_LOG(KERN_ERR, "failed device_may_wakeup\n");
	}
	data->als_polling_cnt_reset |= ALS_POLLING_CNT_RESET_RESUME;
	PSALS_DBG_LOG(KERN_INFO, "als_polling_cnt_reset=0x%08X\n",
								data->als_polling_cnt_reset);

	if (data->enable_ps_sensor > 0)
	{
		dev_status_tmp = (atomic_read(&g_dev_status) &
								  GP2AP_DEV_STATUS_SUSPEND_INT);
		atomic_set(&g_dev_status, GP2AP_DEV_STATUS_INIT |
								  GP2AP_DEV_STATUS_RESUME | dev_status_tmp);
		dev_status_tmp = atomic_read(&g_dev_status);
		PSALS_DBG_LOG(KERN_INFO, "dev_status_tmp=0x%08X\n", dev_status_tmp);

		if( atomic_read(&g_dev_status) & GP2AP_DEV_STATUS_SUSPEND_INT ) {
			gp2ap_reschedule_work(data, 0);
		}
	}
	else
	{
		atomic_set(&g_dev_status, GP2AP_DEV_STATUS_INIT);
	}

	if (data->enable_als_sensor > 0)
	{
		if(als_reg_data.vdd_reg){
			ret = regulator_set_optimum_mode(als_reg_data.vdd_reg, als_reg_data.on_load_uA);
			if (ret < 0) {
				PSALS_LOG(KERN_ERR, "regulator_set_optimum_mode fail. ret=%d\n", ret);
				return ret;
			}
			PSALS_DBG_LOG(KERN_INFO, "regulator_set_optimum_mode ret=%d\n",ret);
			usleep_range(1000,1000);
		}
		else{
			PSALS_LOG(KERN_ERR, "error : als_reg_data [NULL] enable als_sensor failed\n");
			return -EFAULT;
		}
		if(als_reg_data_l5.vdd_reg){
			ret = regulator_set_optimum_mode(als_reg_data_l5.vdd_reg, als_reg_data_l5.on_load_uA);
			if (ret < 0) {
				PSALS_LOG(KERN_ERR, "regulator_set_optimum_mode(L5) fail. ret=%d\n", ret);
				return ret;
			}
			PSALS_DBG_LOG(KERN_INFO, "regulator_set_optimum_mode(L5) ret=%d\n",ret);
			usleep_range(1000,1000);
		}
		else{
			PSALS_LOG(KERN_ERR, "error : als_reg_data(L5) [NULL] enable als_sensor failed\n");
			return -EFAULT;
		}

		mutex_unlock(&data->psals_mutex);
		cancel_delayed_work_sync(&data->als_on_dwork);
		cancel_delayed_work_sync(&data->als_data_dwork);
		mutex_lock(&data->psals_mutex);
		PSALS_DBG_LOG(KERN_INFO, "queue_delayed_work(als_on_dwork) call. "
													"delay_ms=%d\n", 0);
		queue_delayed_work(als_polling_wq, &data->als_on_dwork,
											msecs_to_jiffies(0));
	}

	wake_unlock(&gp2ap_wake_lock);

	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
	mutex_unlock(&data->psals_mutex);
	return 0;
}

static const struct i2c_device_id gp2ap_id[] = {
	{DEVICE_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, gp2ap_id);

static struct i2c_driver gp2ap_driver = {
	.driver = {
		.name	= DEVICE_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = gp2ap_suspend,
	.resume = gp2ap_resume,
	.probe	= gp2ap_probe,
	.remove = gp2ap_remove,
	.id_table = gp2ap_id,
};

static int __init gp2ap_init(void)
{
	s32 rc = 0;

	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");

	als_polling_wq = create_singlethread_workqueue("als_polling_wq");
	if (!als_polling_wq)
	{
		PSALS_LOG(KERN_ERR, "can't create queue : als_polling_wq \n");
		goto REGIST_ERR1;
	}

	ps_polling_wq = create_singlethread_workqueue("ps_polling_wq");
	if (!ps_polling_wq)
	{
		PSALS_LOG(KERN_ERR, "can't create queue : ps_polling_wq \n");
		goto REGIST_ERR2;
	}

	rc = i2c_add_driver(&gp2ap_driver);
	PSALS_DBG_LOG(KERN_INFO, "i2c_add_driver() called. rc=%d\n", rc);
	if (rc != 0)
	{
		PSALS_LOG(KERN_ERR, "can't add i2c driver\n");
		goto REGIST_ERR3;
	}

	PSALS_DBG_LOG(KERN_INFO, "[OUT] rc=%d\n", rc);
	return rc;

REGIST_ERR3:
	if (ps_polling_wq != NULL)
	{
		flush_workqueue(ps_polling_wq);
		destroy_workqueue(ps_polling_wq);
		ps_polling_wq = NULL;
	}

REGIST_ERR2:
	if (als_polling_wq != NULL)
	{
		flush_workqueue(als_polling_wq);
		destroy_workqueue(als_polling_wq);
		als_polling_wq = NULL;
	}

REGIST_ERR1:
	return -ENOTSUPP;
	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
}

static void __exit gp2ap_exit(void)
{
	PSALS_DBG_LOG(KERN_INFO, "[IN]\n");
	if (als_polling_wq != NULL)
	{
		flush_workqueue(als_polling_wq);
		destroy_workqueue(als_polling_wq);
		als_polling_wq = NULL;
	}

	if (ps_polling_wq != NULL)
	{
		flush_workqueue(ps_polling_wq);
		destroy_workqueue(ps_polling_wq);
		ps_polling_wq = NULL;
	}

	i2c_del_driver(&gp2ap_driver);

	i2c_unregister_device(client_gp2ap);
	client_gp2ap = NULL;

	PSALS_DBG_LOG(KERN_INFO, "[OUT]\n");
}

module_init(gp2ap_init);
module_exit(gp2ap_exit);

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("Optical Sensor driver for GP2AP030A00F");
MODULE_LICENSE("GPL");
