/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 */
/* include/asm/mach-msm/htc_pwrsink.h
 *
 * Copyright (C) 2008 HTC Corporation.
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2011 Code Aurora Forum. All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include "timed_output.h"

/* @OEMTAG@ ADD-S */
#ifndef __devinit
  #define __devinit
  #define __devexit
#endif/* __devinit */
#ifndef __devexit_p
  #define __devexit_p(f)	f
#endif/* __devexit_p */
/* @OEMTAG@ ADD-E */

enum vib_status
{
	VIB_STANDBY,
	VIB_ON,
	VIB_OFF,
};
enum vib_add_time_flag
{
	VIB_ADD_TIME_FLAG_OFF,
	VIB_ADD_TIME_FLAG_ON,
};

#define DEBUG_VIB_DRV2604				0
#define VIB_TEST						1
#define VIB_TEST_DEBUGFS				1

#define VIB_DRV_NAME					"DRV2604"
#define VIB_ON_WORK_NUM					(5)
#define I2C_RETRIES_NUM					(5)
#define I2C_WRITE_MSG_NUM				(1)
#define I2C_READ_MSG_NUM				(2)
#define VIB_STANDBY_DELAY_TIME			(1000)
#define VIB_TIME_MIN					(25)
#define VIB_TIME_MAX					(15000)
/* @OEMTAG@ MOD-S */
/* #define MSMGPIO_LIMTR_EN				( 37) */
/* #define MSMGPIO_LIMTR_INTRIG			( 33) */
int g_gpio_vib_en = -1;
int g_gpio_vib_intrig = -1;
#define MSMGPIO_LIMTR_EN				g_gpio_vib_en
#define MSMGPIO_LIMTR_INTRIG			g_gpio_vib_intrig
/* @OEMTAG@ MOD-E */
#define VIB_I2C_EN_DELAY				(250)	/* us */

/* @OEMTAG@ I06-SWD0347-00005 ADD-S */
#define VIB_HAPTICS_ON  (1)
#define VIB_HAPTICS_OFF (0)

#define HAPTICS_TIME_10         10  /* 10ms */
#define HAPTICS_TIME_20         20  /* 20ms */
#define HAPTICS_TIME_30         30  /* 30ms */
#define HAPTICS_TIME_40         40  /* 40ms */
#define HAPTICS_TIME_50         50  /* 50ms */
#define HAPTICS_TIME_60         60  /* 60ms */
#define HAPTICS_TIME_70         70  /* 70ms */
#define HAPTICS_TIME_80         80  /* 80ms */
#define HAPTICS_TIME_90         90  /* 90ms */
#define HAPTICS_TIME_100        100 /* 100ms */
/* @OEMTAG@ I06-SWD0347-00005 ADD-E */

struct vib_on_work_data
{
	struct work_struct	work_vib_on;
	int					time;
};
struct drv2604_work_data {
	struct vib_on_work_data vib_on_work_data[VIB_ON_WORK_NUM];
	struct work_struct work_vib_off;
	struct work_struct work_vib_standby;
};
struct drv2604_data_t {
	struct i2c_client *drv2604_i2c_client;
	struct hrtimer vib_off_timer;
	struct hrtimer vib_standby_timer;
	int work_vib_on_pos;
	enum vib_status vib_cur_status;
	enum vib_add_time_flag add_time_flag;
};

/* @OEMTAG@ I06-SWD0347-00005 ADD-S */
static atomic_t haptics;
/* @OEMTAG@ I06-SWD0347-00005 ADD-E */

static struct mutex vib_mutex;
static u8 write_buf[2] = {0x00, 0x00};
static u8 read_buf[4] = {0x00, 0x00, 0x00, 0x00};
struct drv2604_work_data drv2604_work;
struct drv2604_data_t drv2604_data;
void _vib_gpio_set_value(int g, int v, int verify);

/* @OEMTAG@ I12-SWD0347-00001 ADD-S */
enum vib_strength_level {
    VIB_STRENGTH_OFF = 0,
	VIB_STRENGTH_LOW = 1,
	VIB_STRENGTH_MED = 2,
	VIB_STRENGTH_HIGH = 3
};
#define	VIB_STRENGTH_NUM	VIB_STRENGTH_HIGH+1
#define	VIB_STRENGTH_MIN	VIB_STRENGTH_OFF
#define	VIB_STRENGTH_DEF	VIB_STRENGTH_MED
#define	VIB_STRENGTH_MAX	VIB_STRENGTH_HIGH

static enum vib_strength_level	vib_strength = VIB_STRENGTH_DEF;
/* @OEMTAG@ I12-SWD0347-00006 ADD-S */
static enum vib_strength_level	vib_curr_strength = VIB_STRENGTH_DEF;
/* @OEMTAG@ I12-SWD0347-00006 ADD-E */

static struct {
	u32 rated[VIB_STRENGTH_NUM];
	u32 clamp[VIB_STRENGTH_NUM];
/* @OEMTAG@ I12-SWD0347-00005 ADD-S */
	u32 comp[VIB_STRENGTH_NUM];
	u32 bemf[VIB_STRENGTH_NUM];
	u32 gain[VIB_STRENGTH_NUM];
/* @OEMTAG@ I12-SWD0347-00005 ADD-E */
} vib_pow;
/* @OEMTAG@ I12-SWD0347-00001 ADD-E */

#define VIB_LOG(md, fmt, ... ) \
printk(md "[VIB] %s(%d): " fmt, __func__, __LINE__, ## __VA_ARGS__)
#if DEBUG_VIB_DRV2604
#define VIB_DEBUG_LOG(md, fmt, ... ) \
printk(md "[VIB] %s(%d): " fmt, __func__, __LINE__, ## __VA_ARGS__)
#define vib_gpio_set_value(g, v)	_vib_gpio_set_value(g, v, 1)
#else
#define VIB_DEBUG_LOG(md, fmt, ... )
#define vib_gpio_set_value(g, v)	_vib_gpio_set_value(g, v, 0)
#endif /* DEBUG_VIB_DRV2604 */

#define debugk(fmt,...)	\
printk("%s(%d)[0x%p]:" fmt, __func__, __LINE__, __builtin_return_address(0), ##__VA_ARGS__)

/* @OEMTAG@ I12-SWD0347-00001 MOD-S */
#define VIB_SET_REG(reg, data) { \
	write_buf[0] = (u8)(reg); \
	write_buf[1] = (u8)(data); \
	drv2604_i2c_write_data(drv2604_data.drv2604_i2c_client, write_buf, sizeof(write_buf)); }
/* @OEMTAG@ I12-SWD0347-00001 MOD-E */

#define VIB_GET_REG(reg) { \
	drv2604_i2c_read_data(drv2604_data.drv2604_i2c_client, reg, read_buf, 1); }



#if VIB_TEST
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/ioctl.h>

/* @OEMTAG@ ADD-S */
#if VIB_TEST_DEBUGFS
#include <linux/debugfs.h>
#include <linux/syscalls.h>
#endif/* VIB_TEST_DEBUGFS */
/* @OEMTAG@ ADD-E */

#define VIB_TEST_IOC_MAGIC 'v'
#define IOCTL_VIB_TEST_CTRL _IOWR(VIB_TEST_IOC_MAGIC, 1, vib_test_param)

#define VIB_TEST_SET_VOLTAGE	0x0001
#define VIB_TEST_CALIBRATION	0x0020
#define VIB_TEST_RD_REGISTER	0x0021
#define VIB_TEST_WR_REGISTER	0x0022

#define VIB_TEST_STATUS_SUCCESS	(0)
#define VIB_TEST_STATUS_FAIL	(-1)

typedef struct {
	u16 req_code;
	u8 data[4];
} vib_test_param;

/* for voltage */
typedef struct {
	u16 rated_vol;
	u16 clamp_vol;
} vib_test_set_voltage_req_data;

typedef struct {
	u16 status;
	u8 data[2];
} vib_test_rsp_voltage_data;

/* for register */
typedef struct {
	u16 reserved;
	u8 reg;
	u8 data;
} vib_test_set_rdwr_reg_req_data;

typedef struct {
	u16 reserved;
	u8 data[2];
} vib_test_rsp_rdwr_reg_data;

/* for calibration */
typedef struct {
	u8 data[4];
} vib_test_rsp_calibration_data;

#endif /* VIB_TEST */



static int drv2604_i2c_write_data(struct i2c_client *client, u8 *buf, u16 len)
{
	int ret = 0;
	int retry = 0;
	struct i2c_msg msg[I2C_WRITE_MSG_NUM];

	if (client == NULL || buf == NULL)
	{
		VIB_LOG(KERN_ERR, "client=0x%08x,buf=0x%08x\n",
				(unsigned int)client, (unsigned int)buf);
		return 0;
	}

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = len;
	msg[0].buf = buf;

	do
	{
		ret = i2c_transfer(client->adapter, msg, I2C_WRITE_MSG_NUM);
		VIB_DEBUG_LOG(KERN_INFO, "i2c_transfer(write) ret=%d\n", ret);
	} while ((ret != I2C_WRITE_MSG_NUM) && (++retry < I2C_RETRIES_NUM));

	if (ret != I2C_WRITE_MSG_NUM)
	{
		ret = -1;
		pr_err("VIB: i2c write error (try:%d)\n", retry);
	}
	else
	{
		ret = 0;
	}

	return ret;
}

static int drv2604_i2c_read_data(struct i2c_client *client, u8 reg, u8 *buf, u16 len)
{
	int ret = 0;
	int retry = 0;
	u8 start_reg = 0;
	struct i2c_msg msg[I2C_READ_MSG_NUM];

	if (client == NULL || buf == NULL)
	{
		VIB_LOG(KERN_ERR, "client=0x%08x\n",
				(unsigned int)client);
		return 0;
	}

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;
	start_reg = reg;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = buf;

	do
	{
		ret = i2c_transfer(client->adapter, msg, I2C_READ_MSG_NUM);
	} while ((ret != I2C_READ_MSG_NUM) && (++retry < I2C_RETRIES_NUM));

	if (ret != I2C_READ_MSG_NUM)
	{
		ret = -1;
		pr_err("VIB: i2c read error (try:%d)\n", retry);
	}
	else
	{
		ret = 0;
	}

	return ret;
}


/*
 * Initialization Procedure
 */
static void drv2604_initialization(void)
{
	VIB_DEBUG_LOG(KERN_INFO, "called.\n");

	/* After power up, wait at least 250us before the DRV2604 will accept I2C commands. */
	udelay(VIB_I2C_EN_DELAY);

	/* Assert the EN pin (logic high). The EN pin may be asserted any time during or after the 250us wait period. */
	vib_gpio_set_value(MSMGPIO_LIMTR_EN, 1);
	udelay(VIB_I2C_EN_DELAY);

	VIB_SET_REG(0x01, 0x00);/* @OEMTAG@ I68-SWD0111-00021 ADD */
/* @OEMTAG@ I12-SWD0347-00001 MOD-S */
	VIB_SET_REG(0x16, vib_pow.rated[VIB_STRENGTH_DEF]);
	VIB_SET_REG(0x17, vib_pow.clamp[VIB_STRENGTH_DEF]);
/* @OEMTAG@ I12-SWD0347-00001 MOD-E */
	VIB_SET_REG(0x18, 0x08);
	VIB_SET_REG(0x19, 0x8b);/* @OEMTAG@ I68-SWD0364-00004 MOD */
	VIB_SET_REG(0x1a, 0xb9);/* @OEMTAG@ MOD */
	VIB_SET_REG(0x1b, 0x13);/* @OEMTAG@ MOD */
	VIB_SET_REG(0x1c, 0xf5);
	VIB_SET_REG(0x1d, 0x84);/* @OEMTAG@ MOD */
	VIB_SET_REG(0x03, 0x00);
	VIB_SET_REG(0x01, 0x03);

	vib_gpio_set_value(MSMGPIO_LIMTR_EN, 0);

	memset(read_buf,  0x00, sizeof(read_buf));
	memset(write_buf, 0x00, sizeof(write_buf));

	VIB_DEBUG_LOG(KERN_INFO, "end.\n");
}


/* @OEMTAG@ I06-SWD0347-00005 ADD-S */
static bool drv2604_vib_chk_hapticstime(int value)
{
	if (value > HAPTICS_TIME_10 &&
	    value <= HAPTICS_TIME_100)
	{
		return true;
	}

	return false;
}

static int drv2604_vib_set_hapticstime(int value)
{
	if (value > HAPTICS_TIME_10 &&
	    value <= HAPTICS_TIME_40)
	{
		value = value + HAPTICS_TIME_40;
	}
	else if (value > HAPTICS_TIME_40 &&
	         value <= HAPTICS_TIME_60)
	{
		value = value + HAPTICS_TIME_30;
	}
	else if (value > HAPTICS_TIME_60 &&
	         value <= HAPTICS_TIME_80)
	{
		value = value + HAPTICS_TIME_20;
	}
	else if (value > HAPTICS_TIME_80 &&
	         value <= HAPTICS_TIME_100)
	{
		value = value + HAPTICS_TIME_10;
	}

	return value;
}

static int drv2604_vib_haptics(int value)
{
	/* range confirmation of haptics time */
	if (drv2604_vib_chk_hapticstime(value))
	{
		/* haptics flg on */
		atomic_set(&haptics, VIB_HAPTICS_ON);
		/* add haptics time */
		value = drv2604_vib_set_hapticstime(value);
	}

	return value;
}
/* @OEMTAG@ I06-SWD0347-00005 ADD-E */

/* @OEMTAG@ I12-SWD0347-00001 ADD-S */
/* @OEMTAG@ I12-SWD0347-00006 MOD-S */
static void drv2604_set_vib_strength(enum vib_strength_level level)
{
	if (level <= VIB_STRENGTH_MAX)
	{
		udelay(VIB_I2C_EN_DELAY);
		VIB_SET_REG(0x16, vib_pow.rated[level]);
		VIB_SET_REG(0x17, vib_pow.clamp[level]);
/* @OEMTAG@ I12-SWD0347-00005 ADD-S */
		VIB_SET_REG(0x18, vib_pow.comp[level]);
		VIB_SET_REG(0x19, vib_pow.bemf[level]);
		VIB_SET_REG(0x1a, vib_pow.gain[level]);
/* @OEMTAG@ I12-SWD0347-00005 ADD-E */

		pr_info("VIB: strength=%u\n", (unsigned int)level);
	}
	else
	{
		pr_err("VIB: strength parameter [%u] is out of range.\n", (unsigned int)level);
	}

	return;
}
/* @OEMTAG@ I12-SWD0347-00006 MOD-E */
/* @OEMTAG@ I12-SWD0347-00001 ADD-E */

static void drv2604_set_vib(enum vib_status status, int time)
{
	enum vib_status cur_status = drv2604_data.vib_cur_status;

	VIB_DEBUG_LOG(KERN_INFO, "called. status=%d,time=%d,cur_status=%d\n",
							  status, time, cur_status);
	mutex_lock(&vib_mutex);

	switch (status) {
		case VIB_ON:
			VIB_DEBUG_LOG(KERN_INFO, "VIB_ON\n");
			/* STANDBY => ON */
			if (cur_status == VIB_STANDBY)
			{
				/* enable */
				vib_gpio_set_value(MSMGPIO_LIMTR_EN, 1);
				udelay(VIB_I2C_EN_DELAY);
			}
			else
			{
				VIB_DEBUG_LOG(KERN_INFO, "VIB_ON standby cancel skip.\n");
			}

			/* OFF/STANDBY => ON */
			if (cur_status != VIB_ON)
			{
/* @OEMTAG@ I12-SWD0347-00006 ADD-S */
				/* set strength */
				if (vib_curr_strength != vib_strength)
				{
					vib_curr_strength = vib_strength;
					drv2604_set_vib_strength(vib_strength);
				}
/* @OEMTAG@ I12-SWD0347-00006 ADD-E */

				/* start vibrator */
				vib_gpio_set_value(MSMGPIO_LIMTR_INTRIG, 1);
			}
			else
			{
				VIB_DEBUG_LOG(KERN_INFO, "VIB_ON skip.\n");
			}
			/* Set vib off timer (ON => OFF) */
			VIB_DEBUG_LOG(KERN_INFO, "hrtimer_start(vib_off_timer). time=%d\n", time);
			hrtimer_start(&drv2604_data.vib_off_timer,
							ktime_set(time / 1000, 
							(time % 1000) * 1000000),
							HRTIMER_MODE_REL);

			drv2604_data.vib_cur_status = status;
			VIB_DEBUG_LOG(KERN_INFO, "set cur_status=%d\n", drv2604_data.vib_cur_status);
			break;
		case VIB_OFF:
			VIB_DEBUG_LOG(KERN_INFO, "VIB_OFF\n");
			/* ON => OFF */
			if (cur_status == VIB_ON)
			{
				/* stop vibrator */
				vib_gpio_set_value(MSMGPIO_LIMTR_INTRIG, 0);
				drv2604_data.vib_cur_status = status;
				VIB_DEBUG_LOG(KERN_INFO, "set cur_status=%d\n", drv2604_data.vib_cur_status);

				/* Set vib standby timer (OFF => STANDBY) */
				VIB_DEBUG_LOG(KERN_INFO, "hrtimer_start(vib_standby_timer).\n");
				hrtimer_start(&drv2604_data.vib_standby_timer,
								ktime_set(VIB_STANDBY_DELAY_TIME / 1000,
								(VIB_STANDBY_DELAY_TIME % 1000) * 1000000),
								HRTIMER_MODE_REL);
			}
			else
			{
				VIB_DEBUG_LOG(KERN_INFO, "VIB_OFF skip.\n");
			}
			break;
		case VIB_STANDBY:
			VIB_DEBUG_LOG(KERN_INFO, "VIB_STANDBY\n");
			/* OFF => STANDBY */
			if (cur_status == VIB_OFF)
			{
				/* disable */
				vib_gpio_set_value(MSMGPIO_LIMTR_EN, 0);
				drv2604_data.vib_cur_status = status;
				VIB_DEBUG_LOG(KERN_INFO, "set cur_status=%d\n", drv2604_data.vib_cur_status);
			}
			else
			{
				VIB_DEBUG_LOG(KERN_INFO, "VIB_STANDBY skip.\n");
			}
			break;
		default:
			VIB_LOG(KERN_ERR, "parameter error. status=%d\n", status);
			break;
	}
	mutex_unlock(&vib_mutex);
	return;
}

static void drv2604_vib_on(struct work_struct *work)
{
	struct vib_on_work_data *work_data = container_of
										(work, struct vib_on_work_data, work_vib_on);

	VIB_DEBUG_LOG(KERN_INFO, "called. work=0x%08x\n", (unsigned int)work);
	VIB_DEBUG_LOG(KERN_INFO, "work_data=0x%08x,time=%d\n",
					(unsigned int)work_data, work_data->time);
	drv2604_set_vib(VIB_ON, work_data->time);

	return;
}

static void drv2604_vib_off(struct work_struct *work)
{
	VIB_DEBUG_LOG(KERN_INFO, "called. work=0x%08x\n", (unsigned int)work);
	drv2604_set_vib(VIB_OFF, 0);
	return;
}

static void drv2604_vib_standby(struct work_struct *work)
{
	VIB_DEBUG_LOG(KERN_INFO, "called. work=0x%08x\n", (unsigned int)work);

	drv2604_set_vib(VIB_STANDBY, 0);
	return;
}

static void drv2604_timed_vib_on(struct timed_output_dev *dev, int timeout_val)
{
	int ret = 0;

	VIB_DEBUG_LOG(KERN_INFO, "called. dev=0x%08x, timeout_val=%d\n",
					(unsigned int)dev, timeout_val);
	drv2604_work.vib_on_work_data[drv2604_data.work_vib_on_pos].time = timeout_val;

	ret = schedule_work
			(&(drv2604_work.vib_on_work_data[drv2604_data.work_vib_on_pos].work_vib_on));
	if (ret != 0)
	{
		drv2604_data.work_vib_on_pos++;
		if (drv2604_data.work_vib_on_pos >= VIB_ON_WORK_NUM) {
			drv2604_data.work_vib_on_pos = 0;
		}
		VIB_DEBUG_LOG(KERN_INFO, "schedule_work(). work_vib_on_pos=%d\n",
			drv2604_data.work_vib_on_pos);
		VIB_DEBUG_LOG(KERN_INFO, "vib_on_work_data[%d].time=%d\n",
			drv2604_data.work_vib_on_pos,
			drv2604_work.vib_on_work_data[drv2604_data.work_vib_on_pos].time);
	}
	return;
}

static void drv2604_timed_vib_off(struct timed_output_dev *dev)
{
	int ret = 0;

	VIB_DEBUG_LOG(KERN_INFO, "called. dev=0x%08x\n", (unsigned int)dev);
	ret = schedule_work(&drv2604_work.work_vib_off);
	if (ret == 0)
	{
		VIB_LOG(KERN_ERR, "schedule_work error. ret=%d\n",ret);
	}
	return;
}

static void drv2604_timed_vib_standby(struct timed_output_dev *dev)
{
	int ret = 0;

	VIB_DEBUG_LOG(KERN_INFO, "called. dev=0x%08x\n", (unsigned int)dev);
	ret = schedule_work(&drv2604_work.work_vib_standby);
	if (ret == 0)
	{
		VIB_LOG(KERN_ERR, "schedule_work error. ret=%d\n",ret);
	}
	return;
}

static void drv2604_enable(struct timed_output_dev *dev, int value)
{
	VIB_DEBUG_LOG(KERN_INFO, "called. dev=0x%08x,value=%d\n", (unsigned int)dev, value);
	VIB_DEBUG_LOG(KERN_INFO, "add_time_flag=%d\n", drv2604_data.add_time_flag);
	if ((value <= 0) && (drv2604_data.add_time_flag == VIB_ADD_TIME_FLAG_ON))
	{
		VIB_DEBUG_LOG(KERN_INFO, "skip. value=%d,add_time_flag=%d\n",
						value, drv2604_data.add_time_flag);
		return;
	}

/* @OEMTAG@ I06-SWD0347-00005 ADD-S */
	if (atomic_read(&haptics) && !(value))
	{
		atomic_set(&haptics, VIB_HAPTICS_OFF);
		return;
	}
/* @OEMTAG@ I06-SWD0347-00005 ADD-E */

	VIB_DEBUG_LOG(KERN_INFO, "hrtimer_cancel(vib_off_timer)\n");
	hrtimer_cancel(&drv2604_data.vib_off_timer);

/* @OEMTAG@ I06-SWD0347-00005 ADD-S */
	value = drv2604_vib_haptics(value);
/* @OEMTAG@ I06-SWD0347-00005 ADD-E */
	if (value <= 0)
	{
		drv2604_timed_vib_off(dev);
	}
	else
	{
		VIB_DEBUG_LOG(KERN_INFO, "hrtimer_cancel(vib_standby_timer)\n");
		hrtimer_cancel(&drv2604_data.vib_standby_timer);
		if (value < VIB_TIME_MIN)
		{
			value = VIB_TIME_MIN;
			drv2604_data.add_time_flag = VIB_ADD_TIME_FLAG_ON;
			VIB_DEBUG_LOG(KERN_INFO, "set add_time_flag=%d\n", drv2604_data.add_time_flag);
		}
		else
		{
			drv2604_data.add_time_flag = VIB_ADD_TIME_FLAG_OFF;
			VIB_DEBUG_LOG(KERN_INFO, "set add_time_flag=%d\n", drv2604_data.add_time_flag);
		}
		drv2604_timed_vib_on(dev, value);
	}
	return;
}

static int drv2604_get_vib_time(struct timed_output_dev *dev)
{
	int ret = 0;

	VIB_DEBUG_LOG(KERN_INFO, "called. dev=0x%08x\n", (unsigned int)dev);
	mutex_lock(&vib_mutex);

	ret = hrtimer_active(&drv2604_data.vib_off_timer);
	if (ret != 0)
	{
		ktime_t r = hrtimer_get_remaining(&drv2604_data.vib_off_timer);
		struct timeval t = ktime_to_timeval(r);
		mutex_unlock(&vib_mutex);

		return t.tv_sec * 1000 + t.tv_usec / 1000;
	}
	mutex_unlock(&vib_mutex);
	return 0;
}

static enum hrtimer_restart drv2604_off_timer_func(struct hrtimer *timer)
{
	VIB_DEBUG_LOG(KERN_INFO, "called. timer=0x%08x\n", (unsigned int)timer);
	drv2604_data.add_time_flag = VIB_ADD_TIME_FLAG_OFF;
	VIB_DEBUG_LOG(KERN_INFO, "set add_time_flag=%d\n", drv2604_data.add_time_flag);

	drv2604_timed_vib_off(NULL);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart drv2604_standby_timer_func(struct hrtimer *timer)
{
	VIB_DEBUG_LOG(KERN_INFO, "called. timer=0x%08x\n", (unsigned int)timer);
	drv2604_timed_vib_standby(NULL);
	return HRTIMER_NORESTART;
}

static struct timed_output_dev drv2604_output_dev = {
	.name = "vibrator",
	.get_time = drv2604_get_vib_time,
	.enable = drv2604_enable,
};

/* @OEMTAG@ I12-SWD0347-00001 ADD-S */
static ssize_t strength_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{

	return scnprintf(buf, PAGE_SIZE, "%u\n", (unsigned int)vib_strength);
}


static ssize_t strength_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	int res;
	unsigned long tmp;
	enum vib_strength_level vib_level;

	res = kstrtoul(buf, 0, &tmp);
	if (res < 0)
		return -1;

	vib_level = (enum vib_strength_level)tmp;
	if (vib_level < VIB_STRENGTH_MIN)
		vib_level = VIB_STRENGTH_MIN;
	if (vib_level > VIB_STRENGTH_MAX)
		vib_level = VIB_STRENGTH_MAX;

	vib_strength = vib_level;

	VIB_DEBUG_LOG(KERN_DEBUG, "receive strength=%u\n", (unsigned int)vib_strength);

	return size;
}


static DEVICE_ATTR(strength, S_IRUGO | S_IWUSR, strength_show, strength_store);
/* @OEMTAG@ I12-SWD0347-00001 ADD-E */

static int __devinit drv2604_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	int count = 0;

	VIB_DEBUG_LOG(KERN_INFO, "called. id=0x%08x\n", (unsigned int)id);
	drv2604_data.drv2604_i2c_client = client;
	drv2604_data.vib_cur_status = VIB_STANDBY;
	drv2604_data.add_time_flag = VIB_ADD_TIME_FLAG_OFF;

/* @OEMTAG@ I06-SWD0347-00005 ADD-S */
	atomic_set(&haptics, VIB_HAPTICS_OFF);
/* @OEMTAG@ I06-SWD0347-00005 ADD-E */

/* @OEMTAG@ I12-SWD0347-00001 ADD-S */
	memset(&vib_pow, 0x00, sizeof vib_pow);

	if (client->dev.of_node)
	{
		of_property_read_u32_array(client->dev.of_node, "oem,rated", vib_pow.rated, VIB_STRENGTH_NUM);
		of_property_read_u32_array(client->dev.of_node, "oem,clamp", vib_pow.clamp, VIB_STRENGTH_NUM);
/* @OEMTAG@ I12-SWD0347-00005 ADD-S */
		of_property_read_u32_array(client->dev.of_node, "oem,comp", vib_pow.comp, VIB_STRENGTH_NUM);
		of_property_read_u32_array(client->dev.of_node, "oem,bemf", vib_pow.bemf, VIB_STRENGTH_NUM);
		of_property_read_u32_array(client->dev.of_node, "oem,gain", vib_pow.gain, VIB_STRENGTH_NUM);
/* @OEMTAG@ I12-SWD0347-00005 ADD-E */
		g_gpio_vib_en = of_get_named_gpio(client->dev.of_node, "oem,vib-en-gpio", 0);/* @OEMTAG@ I68-SWD0111-00021 ADD-S */
		ret = gpio_request(g_gpio_vib_en, "vib_en");
		if (ret) {
			pr_err("vib_en gpio_request failed.\n");
		}
		g_gpio_vib_intrig = of_get_named_gpio(client->dev.of_node, "oem,vib-intrig-gpio", 0);
		ret = gpio_request(g_gpio_vib_intrig, "vib_intrig");
		if (ret) {
			pr_err("vib_intrig gpio_request failed.\n");
		}/* @OEMTAG@ I68-SWD0111-00021 ADD-E */
	}
/* @OEMTAG@ I12-SWD0347-00001 ADD-E */

	mutex_init(&vib_mutex);

	for (count = 0; count < VIB_ON_WORK_NUM; count++)
	{
		INIT_WORK(&(drv2604_work.vib_on_work_data[count].work_vib_on),
					drv2604_vib_on);
		drv2604_work.vib_on_work_data[count].time = 0;
	}
	drv2604_data.work_vib_on_pos = 0;
	INIT_WORK(&drv2604_work.work_vib_off, drv2604_vib_off);
	INIT_WORK(&drv2604_work.work_vib_standby, drv2604_vib_standby);

	hrtimer_init(&drv2604_data.vib_off_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	drv2604_data.vib_off_timer.function = drv2604_off_timer_func;
	hrtimer_init(&drv2604_data.vib_standby_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	drv2604_data.vib_standby_timer.function = drv2604_standby_timer_func;

	ret = timed_output_dev_register(&drv2604_output_dev);
	VIB_DEBUG_LOG(KERN_INFO, "timed_output_dev_register() ret=%d\n", ret);
	if (ret != 0)
	{
		VIB_LOG(KERN_ERR, "timed_output_dev_register() ERROR ret=%d\n", ret);
		goto probe_dev_register_error;
	}

/* @OEMTAG@ I12-SWD0347-00001 ADD-S */
	/* add "strength" onto "/sys/class/timed_output/vibrator". */
	ret = device_create_file(drv2604_output_dev.dev, &dev_attr_strength);
	if (ret != 0)
	{
		pr_err("device_create_file() ERROR ret=%d\n", ret);
		goto probe_dev_register_error;
	}
/* @OEMTAG@ I12-SWD0347-00001 ADD-E */

	VIB_DEBUG_LOG(KERN_INFO, "timed_output_dev_register() ok\n");

	drv2604_initialization();

	VIB_DEBUG_LOG(KERN_INFO, "timed_output_dev_register() end\n");

/* @OEMTAG@ I12-SWD0347-00001 ADD-S */
	VIB_DEBUG_LOG(KERN_INFO, "vib: Vibrator driver started: {%x,%x} {%x,%x} {%x,%x} {%x,%x}\n",
		vib_pow.rated[0], vib_pow.clamp[0],
		vib_pow.rated[1], vib_pow.clamp[1],
		vib_pow.rated[2], vib_pow.clamp[2],
		vib_pow.rated[3], vib_pow.clamp[3]);
/* @OEMTAG@ I12-SWD0347-00001 ADD-E */

	return 0;

probe_dev_register_error:
	mutex_destroy(&vib_mutex);
	return ret;
}

static int32_t __devexit drv2604_remove(struct i2c_client *pst_client)
{
	int ret = 0;

	VIB_DEBUG_LOG(KERN_INFO, "called. pst_client=0x%08x\n", (unsigned int)pst_client);

	timed_output_dev_unregister(&drv2604_output_dev);

	mutex_destroy(&vib_mutex);
	return ret;
}

static int32_t drv2604_suspend(struct i2c_client *pst_client, pm_message_t mesg)
{
	VIB_DEBUG_LOG(KERN_INFO, "called. pst_client=0x%08x,mesg=%d\n",
					(unsigned int)pst_client, mesg.event);
	VIB_DEBUG_LOG(KERN_INFO, "end.\n");
	return 0;
}

static int32_t drv2604_resume(struct i2c_client *pst_client)
{
	VIB_DEBUG_LOG(KERN_INFO, "called. pst_client=0x%08x\n", (unsigned int)pst_client);
	VIB_DEBUG_LOG(KERN_INFO, "end.\n");
	return 0;
}


#if VIB_TEST
static int vibrator_test_open(struct inode *ip, struct file *fp)
{
	VIB_DEBUG_LOG(KERN_INFO, "called.\n");
	VIB_DEBUG_LOG(KERN_INFO, "end.\n");
	return 0;
}

static int vibrator_test_release(struct inode *ip, struct file *fp)
{
	VIB_DEBUG_LOG(KERN_INFO, "called.\n");
	VIB_DEBUG_LOG(KERN_INFO, "end.\n");
	return 0;
}

static int vibrator_test_set(u16 rated_mv, u16 clamp_mv)
{
	unsigned long	vreg;

	VIB_DEBUG_LOG(KERN_INFO, "called. rated=%u(mV) clamp=%u(mV)\n", rated_mv, clamp_mv);

	if (rated_mv)
	{
		if (rated_mv > clamp_mv)
			clamp_mv = rated_mv;

		vib_gpio_set_value(MSMGPIO_LIMTR_EN, 1);
		udelay(VIB_I2C_EN_DELAY);

		/* V_AVG_ABS = Rated Voltage[7:0] / 255 * 5.28V */
		/* Vrms = 1.16 * V_AVG_ABS */
		vreg = rated_mv;
		vreg *= 25500;
		vreg /= 528;
		vreg /= 100;
		VIB_DEBUG_LOG(KERN_INFO, "Rated=%d => %lu\n", rated_mv, vreg);
		/* Rounded */
		vreg += 5;
		vreg /= 10;
		VIB_DEBUG_LOG(KERN_INFO, "set reg [%lu=%lx]\n", vreg, vreg);
		VIB_SET_REG(0x16, (u8)vreg);


		/* V_OD = ODClamp[7:0] / 255 * 5.6V */
		vreg = clamp_mv;
		vreg *= 2550;
		vreg /= 56;
		vreg /= 100;
		VIB_DEBUG_LOG(KERN_INFO, "ODClamp=%d => %lu\n", clamp_mv, vreg);
		/* Rounded */
		vreg += 5;
		vreg /= 10;
		VIB_DEBUG_LOG(KERN_INFO, "set reg [%lu=%lx]\n", vreg, vreg);
		VIB_SET_REG(0x17, (u8)vreg);

		vib_gpio_set_value(MSMGPIO_LIMTR_INTRIG, 1);
	}
	else
	{
		vib_gpio_set_value(MSMGPIO_LIMTR_INTRIG, 0);
		vib_gpio_set_value(MSMGPIO_LIMTR_EN, 0);
	}

	VIB_DEBUG_LOG(KERN_INFO, "end.\n");

	return 0;
}

static int vibrator_test_set_voltage(u8 *data)
{
	int ret = 0;
	vib_test_set_voltage_req_data *req_data =
		(vib_test_set_voltage_req_data *)data;
	vib_test_rsp_voltage_data *rsp_data = (vib_test_rsp_voltage_data *)data;
	s16 status = VIB_TEST_STATUS_SUCCESS;

	VIB_DEBUG_LOG(KERN_INFO, "called. rated_vol=%u clamp_vol=%u\n", req_data->rated_vol, req_data->clamp_vol);

	if (status == VIB_TEST_STATUS_SUCCESS)
	{
		ret = vibrator_test_set(req_data->rated_vol, req_data->clamp_vol);
	}

	if (ret < 0) {
		VIB_LOG(KERN_ERR, "vibrator_config error. ret=%d\n", ret);
		status = VIB_TEST_STATUS_FAIL;
	}

	memset(rsp_data, 0x00, sizeof(vib_test_rsp_voltage_data));
	rsp_data->status = (u32)status;

	VIB_DEBUG_LOG(KERN_INFO, "end. ret=%d\n", ret);
	return ret;
}

static int vibrator_test_read_register(u8 *data)
{
	vib_test_set_rdwr_reg_req_data *req_data = (vib_test_set_rdwr_reg_req_data *)data;
	vib_test_rsp_rdwr_reg_data *rsp_data = (vib_test_rsp_rdwr_reg_data *)data;

	vib_gpio_set_value(MSMGPIO_LIMTR_EN, 1);
	udelay(VIB_I2C_EN_DELAY);
	VIB_GET_REG(req_data->reg);
	vib_gpio_set_value(MSMGPIO_LIMTR_EN, 0);

	VIB_DEBUG_LOG(KERN_INFO, "read. reg=%X data=%x\n", req_data->reg, read_buf[0]);

	rsp_data->reserved = 0;
	rsp_data->data[0] = req_data->reg;
	rsp_data->data[1] = read_buf[0];

	return 0;
}

static int vibrator_test_write_register(u8 *data)
{
	vib_test_set_rdwr_reg_req_data *req_data = (vib_test_set_rdwr_reg_req_data *)data;
	vib_test_rsp_rdwr_reg_data *rsp_data = (vib_test_rsp_rdwr_reg_data *)data;

	VIB_DEBUG_LOG(KERN_INFO, "write. reg=%X data=%x\n", req_data->reg, req_data->data);

	vib_gpio_set_value(MSMGPIO_LIMTR_EN, 1);
	udelay(VIB_I2C_EN_DELAY);
	VIB_SET_REG(req_data->reg, req_data->data);
	vib_gpio_set_value(MSMGPIO_LIMTR_EN, 0);

	rsp_data->reserved = 0;
	return 0;
}

/*
 * AUTO CALIBRATION PROCEDURE
 */
static void vibrator_test_auto_calibration(u8 *data)
{
	vib_test_rsp_calibration_data *rsp_data = (vib_test_rsp_calibration_data *)data;
	int cnt;

	VIB_DEBUG_LOG(KERN_INFO, "auto calibration start\n");

	vib_gpio_set_value(MSMGPIO_LIMTR_EN, 1);
	udelay(VIB_I2C_EN_DELAY);

	VIB_SET_REG(0x01, 0x00);
/* @OEMTAG@ I12-SWD0347-00001 MOD-S */
	VIB_SET_REG(0x16, 0x49);/* @OEMTAG@ I68-SWD0364-00004 MOD */
	VIB_SET_REG(0x17, 0x74);/* @OEMTAG@ I68-SWD0364-00004 MOD */
/* @OEMTAG@ I12-SWD0347-00001 MOD-E */
	VIB_SET_REG(0x1a, 0xb9);/* @OEMTAG@ I68-SWD0111-00021 MOD */
	VIB_SET_REG(0x1b, 0x13);/* @OEMTAG@ I68-SWD0111-00021 MOD */
	VIB_SET_REG(0x1c, 0xf5);
	VIB_SET_REG(0x1d, 0x84);/* @OEMTAG@ I68-SWD0111-00021 MOD */
	VIB_SET_REG(0x01, 0x07);
	VIB_SET_REG(0x1e, 0x20);/* @OEMTAG@ I68-SWD0111-00021 MOD */
	VIB_SET_REG(0x0c, 0x01);

	for (cnt = 0; cnt < 50; ++cnt)
	{
		VIB_GET_REG(0x0c);
		if (!(read_buf[0] & 0x01))
			break;
		mdelay(10);
	}

	VIB_DEBUG_LOG(KERN_INFO, "auto calibration end\n");

	if (read_buf[0] & 0x01)
	{
		VIB_LOG(KERN_ERR, "auto calibration invalid\n");
	}
	else
	{
		VIB_DEBUG_LOG(KERN_INFO, "auto calibration normal end\n");

		VIB_GET_REG(0x18);
		VIB_DEBUG_LOG(KERN_INFO, "ACalComp 0x18 [7:0] = %02x\n", (unsigned)read_buf[0]);
		VIB_GET_REG(0x19);
		VIB_DEBUG_LOG(KERN_INFO, "ACalBEMF 0x19 [7:0] = %02x\n", (unsigned)read_buf[0]);
		VIB_GET_REG(0x1a);
		VIB_DEBUG_LOG(KERN_INFO, "BEMFGain 0x1a [1:0] = %02x\n", (unsigned)read_buf[0]);
	}

	VIB_GET_REG(0x18);
	rsp_data->data[0] = read_buf[0];
	VIB_GET_REG(0x19);
	rsp_data->data[1] = read_buf[0];
	VIB_GET_REG(0x1a);
	rsp_data->data[2] = read_buf[0];
	rsp_data->data[3] = 0;


	vib_gpio_set_value(MSMGPIO_LIMTR_EN, 0);
}

int vibrator_do_request(vib_test_param* param)
{
	vib_test_param test_param = *param;
	int ret = 0;

	VIB_DEBUG_LOG(KERN_INFO, "req_code=0x%04X\n", test_param.req_code);

	switch (test_param.req_code) {
	case VIB_TEST_SET_VOLTAGE:
		VIB_LOG(KERN_ERR, "VIB_TEST_SET_VOLTAGE\n");
		ret = vibrator_test_set_voltage(&test_param.data[0]);
		if (ret < 0) {
			VIB_LOG(KERN_ERR, "vibrator_test_set_voltage() error. ret=%d\n", ret);
			break;
		}
		*param = test_param;
		break;

	case VIB_TEST_CALIBRATION:
		VIB_LOG(KERN_ERR, "VIB_TEST_CALIBRATION\n");
		vibrator_test_auto_calibration(&test_param.data[0]);
		*param = test_param;
		break;

	case VIB_TEST_RD_REGISTER:
		VIB_LOG(KERN_ERR, "VIB_TEST_RD_REGISTER\n");
		vibrator_test_read_register(&test_param.data[0]);
		*param = test_param;
		break;

	case VIB_TEST_WR_REGISTER:
		VIB_LOG(KERN_ERR, "VIB_TEST_WR_REGISTER\n");
		vibrator_test_write_register(&test_param.data[0]);
		*param = test_param;
		break;

	default:
		VIB_LOG(KERN_ERR, "req_code error. req_code=0x%04X\n", test_param.req_code);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static long vibrator_test_ioctl
(struct file *file, unsigned int cmd, unsigned long arg)
{
	int rc = 0;
	u64 ret2 = 0;
	vib_test_param test_param;
	VIB_DEBUG_LOG(KERN_INFO, "called. cmd=0x%08X\n", cmd);

	switch (cmd) {
		case IOCTL_VIB_TEST_CTRL:
			VIB_DEBUG_LOG(KERN_INFO, "cmd=IOCTL_VIB_TEST_CTRL\n");
			ret2 = copy_from_user(&test_param, (void *)arg, sizeof(test_param));
			VIB_DEBUG_LOG(KERN_INFO, "copy_from_user() called. ret2=%lu\n",
							(long unsigned int)ret2);
			if (ret2) {
				VIB_LOG(KERN_ERR, "copy_from_user() error. ret2=%lu\n",
						(long unsigned int)ret2);
				rc = -EINVAL;
				break;
			}

			rc = vibrator_do_request(&test_param);

			if (rc < 0) {
				break;
			}

			switch (test_param.req_code) {
				case VIB_TEST_SET_VOLTAGE:
					ret2 = copy_to_user((void *)arg, &test_param, sizeof(vib_test_param));
					VIB_DEBUG_LOG(KERN_INFO, "copy_to_user() called. ret2=%lu\n",
									(long unsigned int)ret2);
					if (ret2) {
						VIB_LOG(KERN_ERR, "copy_to_user() error. ret2=%llu\n", ret2);
						rc = -EINVAL;
					}
					break;

				case VIB_TEST_CALIBRATION:
					ret2 = copy_to_user((void *)arg, &test_param, sizeof(vib_test_param));
					if (ret2) {
						VIB_LOG(KERN_ERR, "VIB_TEST_CALIBRATION error ret2=%llu\n", ret2);
						rc = -EINVAL;
					}
					break;
				case VIB_TEST_RD_REGISTER:
					ret2 = copy_to_user((void *)arg, &test_param, sizeof(vib_test_param));
					if (ret2) {
						VIB_LOG(KERN_ERR, "VIB_TEST_RD_REGISTER error ret2=%llu\n", ret2);
						rc = -EINVAL;
					}
					break;
				case VIB_TEST_WR_REGISTER:
					ret2 = copy_to_user((void *)arg, &test_param, sizeof(vib_test_param));
					if (ret2) {
						VIB_LOG(KERN_ERR, "VIB_TEST_RD_REGISTER error ret2=%llu\n", ret2);
						rc = -EINVAL;
					}
					break;
				default:
					VIB_LOG(KERN_ERR, "req_code error. req_code=0x%04X\n", test_param.req_code);
					rc = -EINVAL;
					break;
			}
			break;
		default:
			VIB_LOG(KERN_ERR, "cmd error. cmd=0x%08X\n", cmd);
			rc = -EINVAL;
			break;
	}

	VIB_DEBUG_LOG(KERN_INFO, "end. rc=%d\n", rc);
	return rc;
}

static const struct file_operations vibrator_test_fops = {
	.owner			= THIS_MODULE,
	.open			= vibrator_test_open,
	.release		= vibrator_test_release,
	.unlocked_ioctl	= vibrator_test_ioctl,
};

static struct miscdevice vibrator_test_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "kc_vibrator_test",
	.fops = &vibrator_test_fops,
};

/* @OEMTAG@ ADD-S */
#if VIB_TEST_DEBUGFS
static struct dentry *vib_debugfs_root;
#define IS_VIB_CMD(cmd) \
		(cmd == VIB_TEST_SET_VOLTAGE \
	  || cmd == VIB_TEST_CALIBRATION \
	  || cmd == VIB_TEST_RD_REGISTER \
	  || cmd == VIB_TEST_WR_REGISTER)
#ifndef htons
  #define htons(v)	((((v) << 8) & 0xFF00) | (((v) >> 8) & 0x00FF))
#endif/* htons */
#define DM_SUBSYSTEM_ID		0x4BFC
#define DM_SUBSYSTEM_CMD	0x3001

static ssize_t vib_debugfs_write(
	struct file *file,
	const char __user* ubuf,
	size_t len,
	loff_t *off)
{
	vib_test_param param;
	char buf[64] = "";
	unsigned int sub_id = 0;
	unsigned int sub_cmd  = 0;
	int req;
	int arg[4] = { 0, 0, 0, 0 };
	int rc;

	if (len < 20) {
		debugk("!!:len(%d) < 20.\n", len);
		return len;
	}

	/* this is ugly, but sscanf(ubuf, "%04x%04x...", ...) doesn't work anyway. */
	sprintf(buf, "%.4s %.4s %.4s",
		ubuf + 0,
		ubuf + 4,
		ubuf + 8);
	rc = sscanf(buf, "%04x %04x %04x",
			&sub_id,
			&sub_cmd,
			&req);

	if (rc != 3) {
		debugk("!!:rc(%d) == 3\n", rc);
		return len;
	}

	if (!((sub_id == DM_SUBSYSTEM_ID) && (sub_cmd == DM_SUBSYSTEM_CMD))) {
		debugk("!!:sub_id(%04X)==%04X && sub_cmd(%04X)==%04X\n",
			sub_id, sub_cmd, DM_SUBSYSTEM_ID, DM_SUBSYSTEM_CMD);
		return len;
	}

	req = htons(req);

	if (!IS_VIB_CMD(req)) {
		debugk("!!:req_code(%04X) in (%04X, %04X, %04X, %04X)\n",
			req,
			VIB_TEST_SET_VOLTAGE, VIB_TEST_CALIBRATION,
			VIB_TEST_RD_REGISTER, VIB_TEST_WR_REGISTER);
		return len;
	}

	arg[0] = 0;
	arg[1] = 0;
	arg[2] = 0;
	arg[3] = 0;

	sprintf(buf, "%.2s %.2s %.2s %.2s",
			ubuf + 12,
			ubuf + 14,
			ubuf + 16,
			ubuf + 18);
	rc = sscanf(buf, "%02x %02x %02x %02x",
			&arg[0],
			&arg[1],
			&arg[2],
			&arg[3]);

	param.req_code = req;
	param.data[0]  = arg[0];
	param.data[1]  = arg[1];
	param.data[2]  = arg[2];
	param.data[3]  = arg[3];

	debugk("buf[%s] len=%d sub_id=%04x sub_cmd=%04x req=%04x arg=%02x.%02x.%02x.%02x\n",
		buf, (int)len,
		sub_id, sub_cmd,
		req, arg[0], arg[1], arg[2], arg[3]);

	rc = vibrator_do_request(&param);

	if (rc < 0) {
		return len;
	}

	switch (req) {
	case VIB_TEST_CALIBRATION:
		printk("%s(%d):%04X%04X%04X%02X%02X%02X%02X\n", __func__, __LINE__,
			DM_SUBSYSTEM_ID, DM_SUBSYSTEM_CMD,
			req, param.data[0], param.data[1], param.data[2], param.data[3]);
		break;
	case VIB_TEST_RD_REGISTER:
		printk("%s(%d):%04X%04X%04X%04X%02X%02X\n", __func__, __LINE__,
			DM_SUBSYSTEM_ID, DM_SUBSYSTEM_CMD,
			req, 0x0000, param.data[2], param.data[3]);
		break;
	}

	return len;
}

static const struct file_operations vib_debugfs_fops = {
	.open	= simple_open,
	.write  = vib_debugfs_write,
	.llseek = no_llseek,
	.owner  = THIS_MODULE,
};
int vib_debugfs_init(void)
{
	struct dentry* vib_dentry;
	struct vib_data* pdata = pdata;

	vib_debugfs_root = debugfs_create_dir("vibrator", NULL);

	if (!vib_debugfs_root || IS_ERR(vib_debugfs_root)) {
		return -ENODEV;
	}

	vib_dentry = debugfs_create_file(
					"dm",
					S_IWUSR,
					vib_debugfs_root,
					0,
					&vib_debugfs_fops);

	if (!vib_dentry) {
		debugk("debugfs_create_file(dm) failed.\n");
		debugfs_remove(vib_debugfs_root);
		vib_debugfs_root = NULL;
		return -ENODEV;
	}

	return 0;
}
#endif/* VIB_TEST_DEBUGFS */
/* @OEMTAG@ ADD-E */

void vibrator_test_init(void)
{
	misc_register(&vibrator_test_dev);

#if VIB_TEST_DEBUGFS
	vib_debugfs_init();
#endif/* VIB_TEST_DEBUGFS */
}
#endif /* VIB_TEST */


static struct i2c_device_id drv2604_idtable[] = {
	{VIB_DRV_NAME, 0},
	{ },
};
MODULE_DEVICE_TABLE(i2c, drv2604_idtable);

static struct i2c_driver drv2604_driver = {
	.driver		= {
		.name	= VIB_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= drv2604_probe,
	.remove		= __devexit_p(drv2604_remove),
	.suspend	= drv2604_suspend,
	.resume		= drv2604_resume,
	.id_table	= drv2604_idtable,
};

static int __init drv2604_init(void)
{
	int ret = 0;

	VIB_DEBUG_LOG(KERN_INFO, "called.\n");
#if VIB_TEST
	vibrator_test_init();
#endif /* VIB_TEST */
	ret = i2c_add_driver(&drv2604_driver);
	VIB_DEBUG_LOG(KERN_INFO, "i2c_add_driver() ret=%d\n", ret);
	if (ret != 0)
	{
		VIB_LOG(KERN_ERR, "i2c_add_driver() ret=%d\n", ret);
	}

	return ret;
}

static void __exit drv2604_exit(void)
{
	VIB_DEBUG_LOG(KERN_INFO, "called.\n");
	i2c_del_driver(&drv2604_driver);
	VIB_DEBUG_LOG(KERN_INFO, "i2c_del_driver()\n");

	return;
}

void _vib_gpio_set_value(int g, int v, int verify)
{
	int v2;

	if (g < 0) {
		debugk("gpio_set_value(%d,%d) failed.\n", g, v);
		return;
	}

	gpio_set_value(g, v);

	if (!verify) {
		return;
	}

	v2 = gpio_get_value(g);

	if (v == v2) {
		debugk("gpio_set_value(%d,%d) ok.\n", g, v);
	}
	else {
		debugk("gpio_set_value(%d,%d) failed. v2=%d\n", g, v, v2);
	}
}

module_init(drv2604_init);
module_exit(drv2604_exit);
MODULE_DESCRIPTION("timed output DRV2604 vibrator device");
MODULE_LICENSE("GPL");
