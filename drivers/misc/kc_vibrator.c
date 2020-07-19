/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 * (C) 2017 KYOCERA Corporation
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
#include <linux/of_gpio.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include "timed_output.h"

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

#define VIB_TEST
//#define DEBUG_VIB_LC8983XX 1

#define VIB_DRV_NAME					"LC8983XX"
#define VIB_ON_WORK_NUM					(5)
#define I2C_RETRIES_NUM					(5)
#define I2C_WRITE_MSG_NUM				(1)
#define I2C_READ_MSG_NUM				(2)
#define VIB_STANDBY_DELAY_TIME			(1000)
#define VIB_TIME_MIN					(20)
#define VIB_TIME_MAX					(15000)
#define VIB_WRITE_ALL_LEN				(10)
#define VIB_WRITE_CHG_LEN				(2)

#define VIB_EN_INTERVAL_TIME_MAX		(30)

#define SENSOR_VIB_INTERLOCKING

struct vib_on_work_data
{
	struct work_struct	work_vib_on;
	int					time;
	int					intensity;
};
struct lc8983xx_work_data {
	struct vib_on_work_data vib_on_work_data[VIB_ON_WORK_NUM];
	struct work_struct work_vib_off;
	struct work_struct work_vib_standby;
};
struct lc8983xx_data_t {
	struct i2c_client *lc8983xx_i2c_client;
	struct hrtimer vib_off_timer;
	struct hrtimer vib_standby_timer;
	int work_vib_on_pos;
	enum vib_status vib_cur_status;
	enum vib_add_time_flag add_time_flag;
	struct regulator *vib_regulator;
	int reset_gpio;
	int en_gpio;
};

static struct mutex vib_mutex;
static struct timed_output_dev lc8983xx_output_dev;
												   /* addr	0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09 */
static u8 write_all_intensity_2[VIB_WRITE_ALL_LEN] = {0x01, 0x0C, 0x0B, 0x00, 0x18, 0x18, 0x60, 0x60, 0x20, 0x00};
static u8 write_all_intensity_3[VIB_WRITE_ALL_LEN] = {0x01, 0x0C, 0x0B, 0x00, 0x18, 0x18, 0x60, 0x60, 0x20, 0x00};
static u8 write_all_intensity_4[VIB_WRITE_ALL_LEN] = {0x01, 0x0C, 0x0B, 0x00, 0x18, 0x18, 0x60, 0x60, 0x20, 0x00};
												   /* addr	0x01 */
static u8 write_chg_intensity_2[VIB_WRITE_CHG_LEN] = {0x01, 0x0C};
static u8 write_chg_intensity_3[VIB_WRITE_CHG_LEN] = {0x01, 0x0C};
static u8 write_chg_intensity_4[VIB_WRITE_CHG_LEN] = {0x01, 0x0C};

#ifdef DEBUG_VIB_LC8983XX
static u8 read_buf[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
#endif /* DEBUG_VIB_LC8983XX */
struct lc8983xx_work_data lc8983xx_work;
struct lc8983xx_data_t lc8983xx_data;

#ifdef DEBUG_VIB_LC8983XX
#define VIB_V_LOG(msg, ...) \
	pr_notice("[VIB]%s[V](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#define VIB_D_LOG(msg, ...) \
	pr_notice("[VIB]%s[D](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#else
#define VIB_V_LOG(msg, ...)
#define VIB_D_LOG(msg, ...) \
	pr_debug ("[VIB]%s[D](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#endif

#define VIB_E_LOG(msg, ...) \
	pr_err   ("[VIB]%s[E](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#define VIB_N_LOG(msg, ...) \
	pr_notice("[VIB]%s[N](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

enum vib_intensity{
	VIB_INTENSITY_1 = 1,
	VIB_INTENSITY_2,
	VIB_INTENSITY_3,
	VIB_INTENSITY_4,
	VIB_INTENSITY_5
};

static atomic_t g_vib_intensity = ATOMIC_INIT(VIB_INTENSITY_3);

enum vib_strength_level {
	VIB_STRENGTH_LOW = 1,
	VIB_STRENGTH_MED = 2,
	VIB_STRENGTH_HIGH = 3
};
#define VIB_STRENGTH_MIN	VIB_STRENGTH_LOW
#define VIB_STRENGTH_DEF	VIB_STRENGTH_HIGH
#define VIB_STRENGTH_MAX	VIB_STRENGTH_HIGH

static enum vib_strength_level vib_strength = VIB_STRENGTH_DEF;

static int lc8983xx_i2c_write_data(struct i2c_client *client, u8 *buf, u16 len)
{
	int ret = 0;
	int retry = 0;
	struct i2c_msg msg[I2C_WRITE_MSG_NUM];
#ifdef DEBUG_VIB_LC8983XX
	int i = 0;
#endif /* DEBUG_VIB_LC8983XX */

	VIB_V_LOG("called.");
	if (client == NULL || buf == NULL)
	{
		VIB_E_LOG("fail client=0x%p", client);
		return 0;
	}

	VIB_D_LOG("addr=0x%02x len=%d", client->addr, len);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = len;
	msg[0].buf = buf;

	do
	{
		ret = i2c_transfer(client->adapter, msg, I2C_WRITE_MSG_NUM);
		VIB_V_LOG("i2c_transfer(write) ret=%d", ret);
	} while ((ret != I2C_WRITE_MSG_NUM) && (++retry < I2C_RETRIES_NUM));

	if (ret != I2C_WRITE_MSG_NUM)
	{
		ret = -1;
		VIB_E_LOG("i2c write error (try:%d)", retry);
	}
	else
	{
		ret = 0;
		VIB_V_LOG("i2c write success");
#ifdef DEBUG_VIB_LC8983XX
		for (i = 1; i < len; i++)
		{
			VIB_V_LOG("i2c write reg=0x%02x,value=0x%02x",
						  (unsigned int)(*buf + i - 1), (unsigned int)*(buf + i));
		}
#endif /* DEBUG_VIB_LC8983XX */
	}

	VIB_V_LOG("finish. ret=%d", ret);
	return ret;
}

#if defined(DEBUG_VIB_LC8983XX) || defined(VIB_TEST)
static int lc8983xx_i2c_read_data(struct i2c_client *client, u8 reg, u8 *buf, u16 len)
{
	int ret = 0;
	int retry = 0;
	u8 start_reg = 0;
	struct i2c_msg msg[I2C_READ_MSG_NUM];
	int i = 0;

	VIB_V_LOG("called.");
	if (client == NULL || buf == NULL)
	{
		VIB_E_LOG("fail client=0x%p", client);
		return 0;
	}

	VIB_D_LOG("addr=0x%02x reg=0x%02x len=%d", client->addr, reg, len);

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
		VIB_V_LOG("i2c_transfer(read) reg=0x%02x,ret=%d",
					  (unsigned int)reg, ret);
	} while ((ret != I2C_READ_MSG_NUM) && (++retry < I2C_RETRIES_NUM));

	if(ret != I2C_READ_MSG_NUM)
	{
		ret = -1;
		VIB_E_LOG("i2c read error (try:%d)", retry);
	}
	else
	{
		ret = 0;
		VIB_V_LOG("i2c read success");
		for (i = 0; i < len; i++)
		{
			VIB_D_LOG("i2c read reg=0x%02x,value=0x%02x",
						  (unsigned int)(reg + i), (unsigned int)*(buf + i));
		}
	}

	VIB_V_LOG("finish. ret=%d", ret);
	return ret;
}
#endif /* DEBUG_VIB_LC8983XX || VIB_TEST */

static int lc8983xx_activate_gpios(void)
{
	int ret = 0;

	ret = gpio_request_one(lc8983xx_data.reset_gpio, GPIOF_OUT_INIT_LOW, "LC8983XX RST");
	VIB_V_LOG("gpio_request_one(GPIO%d) ret=%d", lc8983xx_data.reset_gpio, ret);
	if (ret != 0)
	{
		VIB_E_LOG("gpio_request_one(GPIO%d) failed. ret=%d", lc8983xx_data.reset_gpio, ret);
		return ret;
	}
	ret = gpio_request_one(lc8983xx_data.en_gpio, GPIOF_OUT_INIT_LOW, "LC8983XX EN");
	VIB_V_LOG("gpio_request_one(GPIO%d) ret=%d", lc8983xx_data.en_gpio, ret);
	if (ret != 0)
	{
		VIB_E_LOG("gpio_request_one(GPIO%d) failed. ret=%d", lc8983xx_data.en_gpio, ret);
		gpio_free(lc8983xx_data.reset_gpio);
		return ret;
	}

	return 0;
}

static void lc8983xx_suspend_gpios(void)
{
	gpio_free(lc8983xx_data.reset_gpio);
	VIB_V_LOG("gpio_free(GPIO%d)", lc8983xx_data.reset_gpio);
	gpio_free(lc8983xx_data.en_gpio);
	VIB_V_LOG("gpio_free(GPIO%d)", lc8983xx_data.en_gpio);

	return;
}

#ifdef SENSOR_VIB_INTERLOCKING
static void vibrator_send_uevent(struct device *dev, int on)
{
	char event_string[20];
	char *envp[] = { event_string, NULL };

	if (!dev) {
		dev_err(dev, "dev NULL\n");
		return;
	}

	if (!&dev->kobj) {
		dev_err(dev, "&dev->kobj NULL\n");
		return;
	}

	switch (on) {
		case VIB_ON:
			sprintf(event_string, "KC_VIB=ON");
			break;
		case VIB_OFF:
		case VIB_STANDBY:
			sprintf(event_string, "KC_VIB=OFF");
			break;
	}

	kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, envp);

	pr_debug("%s: dev = 0x%lx, &dev->kobj = 0x%lx\n", __func__,
				(unsigned long)dev, (unsigned long)&dev->kobj);
	return;
}
#endif

static void lc8983xx_set_vib(enum vib_status status, int time, int intensity)
{
	enum vib_status cur_status = VIB_STANDBY;
	int ret = 0;
	u8 *write_buf;
	static ktime_t time_vib_start = { 0 };
	ktime_t time_vib_end;
	s64 interval_ms;
	s64 enabled_ms;
#ifdef SENSOR_VIB_INTERLOCKING
    static int vib_lasttime_val = 0;
#endif

	mutex_lock(&vib_mutex);

	cur_status = lc8983xx_data.vib_cur_status;
	VIB_D_LOG("called. status=%d,time=%d,intensity=%d,cur_status=%d",
							  status, time, intensity, cur_status);

	switch (status) {
		case VIB_ON:
			VIB_D_LOG("VIB_ON");
			if (cur_status == VIB_STANDBY)
			{
				ret = lc8983xx_activate_gpios();
				if (ret != 0)
				{
					VIB_E_LOG("lc8983xx_activate_gpios failed. ret=%d", ret);
					goto gpio_activate_error;
				}
				gpio_set_value_cansleep(lc8983xx_data.reset_gpio, 1);

				VIB_V_LOG("usleep(200) start.");
				usleep(200);
				VIB_V_LOG("usleep(200) end.");

				if(intensity == VIB_INTENSITY_4) {
					write_buf = write_all_intensity_4;
				} else if(intensity == VIB_INTENSITY_2) {
					write_buf = write_all_intensity_2;
				} else {
					write_buf = write_all_intensity_3;
				}
				lc8983xx_i2c_write_data(lc8983xx_data.lc8983xx_i2c_client,
										(u8 *)write_buf,
										VIB_WRITE_ALL_LEN);

#ifdef DEBUG_VIB_LC8983XX
				lc8983xx_i2c_read_data(lc8983xx_data.lc8983xx_i2c_client,
										0x01,
										(u8 *)read_buf,
										sizeof(read_buf));
#endif /* DEBUG_VIB_LC8983XX */
			}
			else
			{
				if(intensity == VIB_INTENSITY_4) {
					write_buf = write_chg_intensity_4;
				} else if(intensity == VIB_INTENSITY_2) {
					write_buf = write_chg_intensity_2;
				} else {
					write_buf = write_chg_intensity_3;
				}
				lc8983xx_i2c_write_data(lc8983xx_data.lc8983xx_i2c_client,
										(u8 *)write_buf,
										VIB_WRITE_CHG_LEN);

#ifdef DEBUG_VIB_LC8983XX
				lc8983xx_i2c_read_data(lc8983xx_data.lc8983xx_i2c_client,
										0x01,
										(u8 *)read_buf,
										VIB_WRITE_CHG_LEN-1);
#endif /* DEBUG_VIB_LC8983XX */
				VIB_V_LOG("VIB_ON standby cancel skip.");
			}

			if (cur_status != VIB_ON)
			{
				time_vib_start = ktime_get();
				gpio_set_value_cansleep(lc8983xx_data.en_gpio, 1);
			}
			else
			{
				VIB_V_LOG("VIB_ON skip.");
			}
			VIB_V_LOG("hrtimer_start(vib_off_timer). time=%d", time);
			hrtimer_start(&lc8983xx_data.vib_off_timer,
						  ktime_set(time / 1000,
						  (time % 1000) * 1000000),
						  HRTIMER_MODE_REL);

			lc8983xx_data.vib_cur_status = status;
			VIB_V_LOG("set cur_status=%d", lc8983xx_data.vib_cur_status);
			break;
		case VIB_OFF:
			VIB_D_LOG("VIB_OFF");
			if (cur_status == VIB_ON)
			{
				gpio_set_value_cansleep(lc8983xx_data.en_gpio, 0);
				time_vib_end = ktime_get();

				lc8983xx_data.vib_cur_status = status;
				VIB_V_LOG("set cur_status=%d", lc8983xx_data.vib_cur_status);

				VIB_V_LOG("hrtimer_start(vib_standby_timer).");
				hrtimer_start(&lc8983xx_data.vib_standby_timer,
							  ktime_set(VIB_STANDBY_DELAY_TIME / 1000, 
							  (VIB_STANDBY_DELAY_TIME % 1000) * 1000000),
							  HRTIMER_MODE_REL);

				enabled_ms = ktime_to_ms(ktime_sub(time_vib_end, time_vib_start));
				if (0 < enabled_ms && enabled_ms <= VIB_EN_INTERVAL_TIME_MAX)
				{
					interval_ms = enabled_ms;
				}
				else
				{
					interval_ms = VIB_EN_INTERVAL_TIME_MAX;
				}
				VIB_V_LOG("enabled %lldms, sleep %lldms for interval.", enabled_ms, interval_ms);
				usleep(interval_ms * 1000);
			}
			else
			{
				VIB_V_LOG("VIB_OFF skip.");
			}
			break;
		case VIB_STANDBY:
			VIB_D_LOG("VIB_STANDBY");
			if (cur_status == VIB_OFF)
			{
				gpio_set_value_cansleep(lc8983xx_data.reset_gpio, 0);
				lc8983xx_suspend_gpios();

				lc8983xx_data.vib_cur_status = status;
				VIB_V_LOG("set cur_status=%d", lc8983xx_data.vib_cur_status);
			}
			else
			{
				VIB_V_LOG("VIB_STANDBY skip.");
			}
			break;
		default:
			VIB_E_LOG("parameter error. status=%d", status);
			break;
	}

#ifdef SENSOR_VIB_INTERLOCKING
	if( (vib_lasttime_val == VIB_ON && status != VIB_ON) ||
		(vib_lasttime_val != VIB_ON && status == VIB_ON)){
		vibrator_send_uevent(lc8983xx_output_dev.dev, status);
		vib_lasttime_val = status;
	}
#endif

gpio_activate_error:
	mutex_unlock(&vib_mutex);
	VIB_V_LOG("end.");
	return;
}

static void lc8983xx_vib_on(struct work_struct *work)
{
	struct vib_on_work_data *work_data = container_of
								  (work, struct vib_on_work_data, work_vib_on);

	VIB_D_LOG("called.");
	VIB_V_LOG("time=%d", work_data->time);
	lc8983xx_set_vib(VIB_ON, work_data->time, work_data->intensity);

	return;
}

static void lc8983xx_vib_off(struct work_struct *work)
{
	VIB_D_LOG("called.");
	lc8983xx_set_vib(VIB_OFF, 0, 0);
	return;
}

static void lc8983xx_vib_standby(struct work_struct *work)
{
	VIB_D_LOG("called.");

	lc8983xx_set_vib(VIB_STANDBY, 0, 0);
	return;
}

static void lc8983xx_timed_vib_on(struct timed_output_dev *dev, int timeout_val)
{
	int ret = 0;

	VIB_V_LOG("called. timeout_val=%d", timeout_val);

	lc8983xx_work.vib_on_work_data[lc8983xx_data.work_vib_on_pos].time = timeout_val;
	lc8983xx_work.vib_on_work_data[lc8983xx_data.work_vib_on_pos].intensity =
											(int)atomic_read(&g_vib_intensity);

	VIB_V_LOG("called. g_vib_intensity=0x%08x",
					lc8983xx_work.vib_on_work_data[lc8983xx_data.work_vib_on_pos].intensity);

	ret = schedule_work
		  (&(lc8983xx_work.vib_on_work_data[lc8983xx_data.work_vib_on_pos].work_vib_on));
	if (ret != 0)
	{
		lc8983xx_data.work_vib_on_pos++;
		if (lc8983xx_data.work_vib_on_pos >= VIB_ON_WORK_NUM) {
			lc8983xx_data.work_vib_on_pos = 0;
		}
		VIB_V_LOG("schedule_work(). work_vib_on_pos=%d",
					  lc8983xx_data.work_vib_on_pos);
		VIB_V_LOG("vib_on_work_data[%d].time=%d intensity=0x%08x",
					  lc8983xx_data.work_vib_on_pos,
					  lc8983xx_work.vib_on_work_data[lc8983xx_data.work_vib_on_pos].time,
					  lc8983xx_work.vib_on_work_data[lc8983xx_data.work_vib_on_pos].intensity);
	}
	return;
}

static void lc8983xx_timed_vib_off(struct timed_output_dev *dev)
{
	int ret = 0;

	VIB_V_LOG("called.");
	ret = schedule_work(&lc8983xx_work.work_vib_off);
	if	(ret == 0)
	{
		VIB_E_LOG("schedule_work error. ret=%d",ret);
	}
	return;
}

static void lc8983xx_timed_vib_standby(struct timed_output_dev *dev)
{
	int ret = 0;

	VIB_V_LOG("called.");
	ret = schedule_work(&lc8983xx_work.work_vib_standby);
	if	(ret == 0)
	{
		VIB_E_LOG("schedule_work error. ret=%d",ret);
	}
	return;
}

static void lc8983xx_enable(struct timed_output_dev *dev, int value)
{
	VIB_D_LOG("called. value=%d", value);
	VIB_V_LOG("add_time_flag=%d", lc8983xx_data.add_time_flag);
	if ((value <= 0) && (lc8983xx_data.add_time_flag == VIB_ADD_TIME_FLAG_ON))
	{
		VIB_V_LOG("skip. value=%d,add_time_flag=%d",
					  value, lc8983xx_data.add_time_flag);
		return;
	}

	VIB_V_LOG("hrtimer_cancel(vib_off_timer)");
	hrtimer_cancel(&lc8983xx_data.vib_off_timer);

	if (value <= 0)
	{
		lc8983xx_timed_vib_off(dev);
	}
	else
	{
		VIB_V_LOG("hrtimer_cancel(vib_standby_timer)");
		hrtimer_cancel(&lc8983xx_data.vib_standby_timer);
		if (value < VIB_TIME_MIN)
		{
			value = VIB_TIME_MIN;
			lc8983xx_data.add_time_flag = VIB_ADD_TIME_FLAG_ON;
			VIB_V_LOG("set add_time_flag=%d", lc8983xx_data.add_time_flag);
		}
		else
		{
			lc8983xx_data.add_time_flag = VIB_ADD_TIME_FLAG_OFF;
			VIB_V_LOG("set add_time_flag=%d", lc8983xx_data.add_time_flag);
		}
		lc8983xx_timed_vib_on(dev, value);
	}
	return;
}

static int lc8983xx_get_vib_time(struct timed_output_dev *dev)
{
	int ret = 0;

	VIB_V_LOG("called.");
	mutex_lock(&vib_mutex);

	ret = hrtimer_active(&lc8983xx_data.vib_off_timer);
	if (ret != 0)
	{
		ktime_t r = hrtimer_get_remaining(&lc8983xx_data.vib_off_timer);
		struct timeval t = ktime_to_timeval(r);
		mutex_unlock(&vib_mutex);

		return t.tv_sec * 1000 + t.tv_usec / 1000;
	}
	mutex_unlock(&vib_mutex);
	return 0;
}

static enum hrtimer_restart lc8983xx_off_timer_func(struct hrtimer *timer)
{
	VIB_V_LOG("called.");
	lc8983xx_data.add_time_flag = VIB_ADD_TIME_FLAG_OFF;
	VIB_V_LOG("set add_time_flag=%d", lc8983xx_data.add_time_flag);

	lc8983xx_timed_vib_off(NULL);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart lc8983xx_standby_timer_func(struct hrtimer *timer)
{
	VIB_V_LOG("called.");
	lc8983xx_timed_vib_standby(NULL);
	return HRTIMER_NORESTART;
}

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
	int vib_intensity;

	res = kstrtoul(buf, 0, &tmp);
	if (res < 0)
		return -1;

	vib_level = (enum vib_strength_level)tmp;
	if (vib_level < VIB_STRENGTH_MIN)
		vib_level = VIB_STRENGTH_MIN;
	if (vib_level > VIB_STRENGTH_MAX)
		vib_level = VIB_STRENGTH_MAX;

	vib_strength = vib_level;

	VIB_D_LOG("receive strength=%u", (unsigned int)vib_strength);

	switch(vib_strength) {
	case VIB_STRENGTH_LOW:
		vib_intensity = VIB_INTENSITY_2;
		break;
	case VIB_STRENGTH_MED:
		vib_intensity = VIB_INTENSITY_3;
		break;
	case VIB_STRENGTH_HIGH:
		vib_intensity = VIB_INTENSITY_4;
		break;
	default:
		vib_intensity = VIB_INTENSITY_2;
		break;
	}
	atomic_set(&g_vib_intensity, vib_intensity);
	VIB_D_LOG("convert strength to intensity, intensity=%d", vib_intensity);
	return size;
}
static DEVICE_ATTR(strength, S_IRUGO | S_IWUSR, strength_show, strength_store);

#ifdef VIB_TEST
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/ioctl.h>

#define VIB_GET_REG(reg, data) \
	lc8983xx_i2c_read_data(lc8983xx_data.lc8983xx_i2c_client, reg, &data, 1)

#define VIB_SET_REG(reg, data) \
	lc8983xx_i2c_write_data(lc8983xx_data.lc8983xx_i2c_client, &reg, 2)

#define VIB_TEST_IOC_MAGIC 'v'
#define IOCTL_VIB_TEST_CTRL _IOWR(VIB_TEST_IOC_MAGIC, 1, vib_test_param)

#define VIB_TEST_SET_VOLTAGE	0x0001
#define VIB_TEST_RD_REGISTER	0x0021
#define VIB_TEST_WR_REGISTER	0x0022

#define VIB_TEST_STATUS_SUCCESS	(0)
#define VIB_TEST_STATUS_FAIL	(-1)

typedef struct {
	u16 req_code;
	u8 data[4];
} vib_test_param;

typedef struct {
	u16 voltage;
	u8 reserved[2];
} vib_test_set_voltage_req_data;

typedef struct {
	u16 status;
	u8 reserved[2];
} vib_test_rsp_data;

typedef struct {
	u16 reserved;
	u8 reg;
	u8 data;
} vib_test_set_rdwr_reg_req_data;

typedef struct {
	u16 reserved;
	u8 data[2];
} vib_test_rsp_rdwr_reg_data;

static int vibrator_test_open(struct inode *ip, struct file *fp)
{
	VIB_D_LOG("called.");
	VIB_D_LOG("end.");
	return 0;
}

static int vibrator_test_release(struct inode *ip, struct file *fp)
{
	VIB_D_LOG("called.");
	VIB_D_LOG("end.");
	return 0;
}

static int vibrator_test_set(int value)
{
	u8 write_buf[VIB_WRITE_ALL_LEN];
	int ret = 0;

	VIB_D_LOG("called. value=%d", value);

	if (value) {
		ret = lc8983xx_activate_gpios();
		if (ret != 0)
		{
			VIB_N_LOG("lc8983xx_activate_gpios failed. ret=%d", ret);
			ret = 0;
		}
		gpio_set_value_cansleep(lc8983xx_data.reset_gpio, 1);

		usleep(200);

		memcpy(write_buf, write_all_intensity_3, sizeof(write_buf));
		write_buf[1] = value;

		ret = lc8983xx_i2c_write_data(lc8983xx_data.lc8983xx_i2c_client,
								(u8 *)write_buf,
								VIB_WRITE_ALL_LEN);
		gpio_set_value_cansleep(lc8983xx_data.en_gpio, 1);
	} else {
		gpio_set_value_cansleep(lc8983xx_data.en_gpio, 0);
		usleep(VIB_EN_INTERVAL_TIME_MAX*1000);
		gpio_set_value_cansleep(lc8983xx_data.reset_gpio, 0);
		lc8983xx_suspend_gpios();
	}
	VIB_D_LOG("end. ret=0x%x", ret);
	return ret;
}

#define SET_VALUE_MIN (0)
#define SET_VALUE_MAX (0x0F)
static int vibrator_test_set_voltage(u8 *data)
{
	int ret = 0;
	int value=0;
	vib_test_set_voltage_req_data *req_data =
	(vib_test_set_voltage_req_data *)data;
	vib_test_rsp_data *rsp_data = (vib_test_rsp_data *)data;
	s16 status = VIB_TEST_STATUS_SUCCESS;

	VIB_D_LOG("called.");

	value = req_data->voltage;

	if (value) {
		if ((value < SET_VALUE_MIN) ||
			(value > SET_VALUE_MAX)) {
			VIB_E_LOG("Invalid value");
			status = VIB_TEST_STATUS_FAIL;
		}
	}

	if (status == VIB_TEST_STATUS_SUCCESS) {
		ret = vibrator_test_set(value);
	}

	if (ret < 0) {
		VIB_E_LOG("vibrator_test_set error.ret=%d", ret);
		status = VIB_TEST_STATUS_FAIL;
	}

	memset(rsp_data, 0x00, sizeof(vib_test_rsp_data));
	rsp_data->status = (u32)status;

	VIB_D_LOG("end. ret=%d", ret);
	return ret;
}

static int vibrator_test_read_register(u8 *data)
{
	int ret;
	vib_test_set_rdwr_reg_req_data *req_data = (vib_test_set_rdwr_reg_req_data *)data;
	vib_test_rsp_rdwr_reg_data *rsp_data = (vib_test_rsp_rdwr_reg_data *)data;

	ret = VIB_GET_REG(req_data->reg, req_data->data);

	VIB_D_LOG("read. reg=0x%x data=0x%x ret=%d", req_data->reg, req_data->data, ret);

	rsp_data->reserved = ret;
	rsp_data->data[0] = req_data->reg;
	rsp_data->data[1] = req_data->data;

	return 0;
}

static int vibrator_test_write_register(u8 *data)
{
	int ret;
	vib_test_set_rdwr_reg_req_data *req_data = (vib_test_set_rdwr_reg_req_data *)data;
	vib_test_rsp_rdwr_reg_data *rsp_data = (vib_test_rsp_rdwr_reg_data *)data;

	ret = VIB_SET_REG(req_data->reg, req_data->data);

	VIB_D_LOG("write. reg=0x%x data=0x%x ret=%d", req_data->reg, req_data->data, ret);

	rsp_data->reserved = ret;
	return 0;
}

static long vibrator_test_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int rc = 0;
	int ret = 0;
	u64 ret2 = 0;
	vib_test_param test_param;
	VIB_D_LOG("called. cmd=0x%08X", cmd);

	switch (cmd) {
	case IOCTL_VIB_TEST_CTRL:
		VIB_D_LOG("cmd=IOCTL_VIB_TEST_CTRL");
		ret2 = copy_from_user(&test_param, (void *)arg, sizeof(test_param));
		VIB_D_LOG("copy_from_user() called. ret2=%lu", (long unsigned int)ret2);
		VIB_D_LOG("copy_from_user() req_code=0x%04X,data=0x%02X%02X%02X%02X",
			test_param.req_code, test_param.data[0], test_param.data[1], test_param.data[2], test_param.data[3]);
		if (ret2) {
			VIB_E_LOG("copy_from_user() error. ret2=%lu", (long unsigned int)ret2);
			rc = -EINVAL;
			break;
		}
		switch (test_param.req_code) {
		case VIB_TEST_SET_VOLTAGE:
			VIB_D_LOG("VIB_TEST_SET_VOLTAGE");
			ret = vibrator_test_set_voltage(&test_param.data[0]);
			if (ret < 0) {
				VIB_E_LOG("vibrator_test_set_voltage() error. ret=%d", ret);
			}
			ret2 = copy_to_user((void *)arg, &test_param, sizeof(vib_test_param));
			VIB_D_LOG("copy_to_user() called. ret2=%lu", (long unsigned int)ret2);
			VIB_D_LOG("copy_to_user() req_code=0x%04X,data=0x%02X%02X%02X%02X",
				test_param.req_code, test_param.data[0], test_param.data[1], test_param.data[2], test_param.data[3]);
			if (ret2) {
				VIB_E_LOG("copy_to_user() error. ret2=%lu", (long unsigned int)ret2);
				rc = -EINVAL;
			}
			break;
		case VIB_TEST_RD_REGISTER:
			VIB_D_LOG("VIB_TEST_RD_REGISTER");
			vibrator_test_read_register(&test_param.data[0]);
			ret2 = copy_to_user((void *)arg, &test_param, sizeof(vib_test_param));
			if (ret2) {
				VIB_E_LOG("VIB_TEST_RD_REGISTER error ret2=%llu", ret2);
				rc = -EINVAL;
			}
			break;
		case VIB_TEST_WR_REGISTER:
			VIB_D_LOG("VIB_TEST_WR_REGISTER");
			vibrator_test_write_register(&test_param.data[0]);
			ret2 = copy_to_user((void *)arg, &test_param, sizeof(vib_test_param));
			if (ret2) {
				VIB_E_LOG("VIB_TEST_RD_REGISTER error ret2=%llu", ret2);
				rc = -EINVAL;
			}
			break;
		default:
			VIB_E_LOG("req_code error. req_code=0x%04X", test_param.req_code);
			rc = -EINVAL;
			break;
		}
		break;
	default:
		VIB_E_LOG("cmd error. cmd=0x%08X", cmd);
		rc = -EINVAL;
		break;
	}

	VIB_D_LOG("end. rc=%d", rc);
	return rc;
}

static const struct file_operations vibrator_test_fops = {
	.owner			= THIS_MODULE,
	.open			= vibrator_test_open,
	.release		= vibrator_test_release,
	.unlocked_ioctl = vibrator_test_ioctl,
};

static struct miscdevice vibrator_test_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "kc_vibrator_test",
	.fops = &vibrator_test_fops,
};

void vibrator_test_init(void)
{
	misc_register(&vibrator_test_dev);
}
#endif

static struct timed_output_dev lc8983xx_output_dev = {
	.name = "vibrator",
	.get_time = lc8983xx_get_vib_time,
	.enable = lc8983xx_enable,
};

static int lc8983xx_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	int count = 0;

	VIB_V_LOG("called. client=0x%p", client);

	lc8983xx_data.lc8983xx_i2c_client = client;
	lc8983xx_data.vib_cur_status = VIB_STANDBY;
	lc8983xx_data.add_time_flag = VIB_ADD_TIME_FLAG_OFF;

	mutex_init(&vib_mutex);

	for (count = 0; count < VIB_ON_WORK_NUM; count++)
	{
		INIT_WORK(&(lc8983xx_work.vib_on_work_data[count].work_vib_on),
				  lc8983xx_vib_on);
		lc8983xx_work.vib_on_work_data[count].time = 0;
		lc8983xx_work.vib_on_work_data[count].intensity = VIB_INTENSITY_3;
	}
	lc8983xx_data.work_vib_on_pos = 0;
	INIT_WORK(&lc8983xx_work.work_vib_off, lc8983xx_vib_off);
	INIT_WORK(&lc8983xx_work.work_vib_standby, lc8983xx_vib_standby);

	hrtimer_init(&lc8983xx_data.vib_off_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	lc8983xx_data.vib_off_timer.function = lc8983xx_off_timer_func;
	hrtimer_init(&lc8983xx_data.vib_standby_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	lc8983xx_data.vib_standby_timer.function = lc8983xx_standby_timer_func;

	ret = timed_output_dev_register(&lc8983xx_output_dev);
	VIB_V_LOG("timed_output_dev_register() ret=%d", ret);
	if (ret != 0)
	{
		VIB_E_LOG("timed_output_dev_register() ret=%d", ret);
		goto probe_dev_register_error;
	}

	lc8983xx_data.reset_gpio = of_get_named_gpio(client->dev.of_node, "kc,reset-gpio", 0);
	if (!gpio_is_valid(lc8983xx_data.reset_gpio)) {
		VIB_E_LOG("No valid RESET GPIO specified %d", lc8983xx_data.reset_gpio);
		ret = -ENODEV;
		goto probe_gpio_is_valid_error;
	}

	lc8983xx_data.en_gpio = of_get_named_gpio(client->dev.of_node, "kc,en-gpio", 0);
	if (!gpio_is_valid(lc8983xx_data.en_gpio)) {
		VIB_E_LOG("No valid EN GPIO specified %d", lc8983xx_data.en_gpio);
		ret = -ENODEV;
		goto probe_gpio_is_valid_error;
	}

	ret = device_create_file(lc8983xx_output_dev.dev, &dev_attr_strength);
	if (ret < 0)
	{
		VIB_E_LOG("device_create_file() ERROR ret=%d", ret);
		ret = -ENODEV;
		goto sysfs_fail;
	}

	return 0;

sysfs_fail:
probe_gpio_is_valid_error:
	timed_output_dev_unregister(&lc8983xx_output_dev);
probe_dev_register_error:
	mutex_destroy(&vib_mutex);
	return ret;

}

static int32_t lc8983xx_remove(struct i2c_client *pst_client)
{
	int ret = 0;

	VIB_V_LOG("called.");
	lc8983xx_suspend_gpios();

	timed_output_dev_unregister(&lc8983xx_output_dev);

	mutex_destroy(&vib_mutex);
	return ret;
}

static int32_t lc8983xx_suspend(struct i2c_client *pst_client, pm_message_t mesg)
{
	VIB_V_LOG("called.");
	VIB_V_LOG("end.");
	return 0;
}

static int32_t lc8983xx_resume(struct i2c_client *pst_client)
{
	VIB_V_LOG("called.");
	VIB_V_LOG("end.");
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id lc8983xx_match_table[] = {
	{ .compatible = "kc,vibrator" },
	{ },
};
#else
#define lc8983xx_match_table NULL
#endif

static const struct i2c_device_id lc8983xx_idtable[] = {
	{ "LC8983XX", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, lc8983xx_idtable);

static struct i2c_driver lc8983xx_driver = {
	.driver		= {
		.name	= VIB_DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lc8983xx_match_table,
	},
	.probe		= lc8983xx_probe,
	.remove		= lc8983xx_remove,
	.suspend	= lc8983xx_suspend,
	.resume		= lc8983xx_resume,
	.id_table	= lc8983xx_idtable,
};

static int __init lc8983xx_init(void)
{

	int ret = 0;

	VIB_V_LOG("called.");

#ifdef VIB_TEST
	vibrator_test_init();
#endif
	ret = i2c_add_driver(&lc8983xx_driver);
	VIB_V_LOG("i2c_add_driver() ret=%d", ret);
	if (ret != 0)
	{
		VIB_E_LOG("i2c_add_driver() ret=%d", ret);
	}

	return ret;
}

static void __exit lc8983xx_exit(void)
{
	VIB_V_LOG("called.");
	i2c_del_driver(&lc8983xx_driver);
	VIB_V_LOG("i2c_del_driver()");

	return;
}

module_init(lc8983xx_init);
module_exit(lc8983xx_exit);
MODULE_DESCRIPTION("timed output LC8983XX vibrator device");
MODULE_LICENSE("GPL");
