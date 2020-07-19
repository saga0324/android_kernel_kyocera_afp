/*
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
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include <linux/workqueue.h>
#include <linux/wakelock.h>

#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>

#include <misc/swic/swic.h>
#include "swic_dm_driver.h"

#define SWIC_DEBUG 0
#if SWIC_DEBUG
#define SWIC_INF_PRINT(fmt, ...) printk(KERN_INFO  fmt, ##__VA_ARGS__)
#define SWIC_DBG_PRINT(fmt, ...) printk(KERN_DEBUG fmt, ##__VA_ARGS__)
#else
#define SWIC_INF_PRINT(fmt, ...) printk(KERN_INFO  fmt, ##__VA_ARGS__)
#define SWIC_DBG_PRINT(fmt, ...)
#endif

#define SWIC_REG_00H							0x00
#define SWIC_REG_01H							0x01
#define SWIC_REG_02H							0x02
#define SWIC_REG_03H							0x03
#define SWIC_REG_04H							0x04
#define SWIC_REG_05H							0x05
#define SWIC_REG_06H							0x06
#define SWIC_REG_07H							0x07
#define SWIC_REG_08H							0x08
#define SWIC_REG_10H							0x10
#define SWIC_REG_11H							0x11

#define SWIC_REG_00H_M							0x07
#define SWIC_REG_01H_M							0xFE
#define SWIC_REG_02H_M							0xFF
#define SWIC_REG_03H_M							0x1F
#define SWIC_REG_04H_M							0x0F
#define SWIC_REG_05H_M							0x0F
#define SWIC_REG_06H_M							0x0F
#define SWIC_REG_07H_M							0xFF
#define SWIC_REG_08H_M							0x07
#define SWIC_REG_10H_M							0x07
#define SWIC_REG_11H_M							0x07

#define SWIC_DATA_CHG_DET_OFF					0x01
#define SWIC_DATA_CHG_DET_ON					0x03

#define SWIC_DATA_05H_ON						SWIC_REG_05H_M
#define SWIC_DATA_05H_OFF						0x00
#define SWIC_DATA_05H_SW						0x0C
#define SWIC_DATA_06H_NO						SWIC_REG_06H_M
#define SWIC_DATA_06H_STANDBY					0x0C
#define SWIC_DATA_06H_ALL						0x00
#define SWIC_DATA_ALL_NORMAL					0x00
#define SWIC_DATA_ADC_ON						0x04

#define SWIC_DATA_01H_DSHORT_DET				0x02
#define SWIC_DATA_01H_VBUS_DET					0x04
#define SWIC_DATA_01H_DETECT					(SWIC_DATA_01H_DSHORT_DET | SWIC_DATA_01H_VBUS_DET)
#define SWIC_DATA_01H_PLUGOUT_DET				0xF8
#define SWIC_DATA_04H_DET_ID					0x01
#define SWIC_DATA_04H_DET_CHG					0x02
#define SWIC_DATA_04H_DET						(SWIC_DATA_04H_DET_ID | SWIC_DATA_04H_DET_CHG)
#define SWIC_DATA_04H_SW_OFF					0x04
#define SWIC_DATA_04H_SW_ON						0x08
#define SWIC_DATA_04H_SW						(SWIC_DATA_04H_SW_ON | SWIC_DATA_04H_SW_OFF)
#define SWIC_DATA_RID_OPEN						0x1F
#define SWIC_DATA_AUDIO_MIC_MONO				0x1E
#define SWIC_DATA_AUDIO_MIC_MONO_CHG			0x1C
#define SWIC_DATA_AUDIO_MIC_STEREO				0x18
#define SWIC_DATA_AUDIO_STEREO					0x14
#define SWIC_DATA_AUDIO_SW_ON_INS				0x12
#define SWIC_DATA_AUDIO_CHG_SW_ON_INS			0x11
#define SWIC_DATA_CHG_DET_MODE1					0x01
#define SWIC_DATA_CHG_DET_MODE2					0x02
#define SWIC_DATA_CHG_DET_MODE3					0x03
#define SWIC_DATA_CHG_DET_MODE4					0x07
#define SWIC_DATA_COMP_OVP							0x04

#define SWIC_SET_DM_INFO(p1, p2, p3, p4)		{dm_info_sw_set_mode = p1; dm_info_detection_id_state = p2; dm_info_interrupt_case = p3; dm_info_chg_det_mode = p4;}
#define MAX_CALLBACK_CLIENTS                    4

#define SWIC_REG_WRITE_DATA(client, reg, val)	swic_write_data(client, reg, reg##_M, val)
#define SWIC_REG_READ_DATA(client, reg) swic_read_data(client, reg, reg##_M)

#define SWIC_I2C_WRITE_SIZE						1
#define SWIC_I2C_READ_SIZE						2
#define SWIC_I2C_RETRIES_NUM					5
#define SWIC_I2C_RETRIES_WAIT					1

#define SWIC_MBHC_TIMEOUT						1

#define SWIC_REG_READ_RETRY						10

#define SWIC_I2C_RESUME_RETRY_NUM			10

struct lc824204_swic {
	struct i2c_client	*client;
	struct input_dev *hs_sw_input;
	struct work_struct	work;
};

struct swic_cb_info {
	struct swic_event_callback *cb_tbl[MAX_CALLBACK_CLIENTS];
};

struct swic_read_reg_data {
	u8 sw_state;
	u8 id_state;
	u8 interrupt_case;
	u8 chg_det_mode;
};

#if SWIC_DEBUG
struct swic_dump_data {
	u8 reg_00h;
	u8 reg_01h;
	u8 reg_02h;
	u8 reg_03h;
	u8 reg_04h;
	u8 reg_05h;
	u8 reg_06h;
	u8 reg_07h;
	u8 reg_08h;
	u8 reg_10h;
	u8 reg_11h;
};

static struct swic_dump_data dump_data;
#endif

typedef enum {
	SWIC_DETECT_EARPHONE,
	SWIC_DETECT_CHG_EARPHONE,
	SWIC_DETECT_USB,
	SWIC_DETECT_PLUGOUT,
	SWIC_DETECT_SWITCH_ON,
	SWIC_DETECT_SWITCH_OFF,
	SWIC_DETECT_UNKNOWN,
} swic_detect_type;

static struct lc824204_swic *swic_info;
static u8 swic_set_mode = SWIC_UNINITIALIZE;

static u8 swic_set_mode_prev = SWIC_UNINITIALIZE;

static u8 dm_info_sw_set_mode = 0;
static u8 dm_info_detection_id_state = 0;
static u8 dm_info_interrupt_case = 0;
static u8 dm_info_chg_det_mode = 0;

static u8 dm_set_swic_det_wait = 10;

static int32_t swic_irq = 0;

static struct swic_cb_info *swic_cb_info = NULL;

static struct wake_lock swic_wake_lock;

static int g_swic_int_gpio = -1;
static int g_swic_hs_det_gpio = -1;
static int g_swic_vbus_rst_gpio = -1;

static int mic_exist = 0;
static int mic_check = 0;
static int mic_check_timeout = false;
wait_queue_head_t	swic_mic_wait;

static int earphone_sw_state = 0;

static int swic_device_available = true;

static int32_t swic_write_data(struct i2c_client *client, u8 reg, u8 reg_mask, u8 val);
static int32_t swic_read_data(struct i2c_client *client, u8 reg, u8 reg_mask);
static void swic_fix_accessory(u8 val);
static irqreturn_t swic_interrupt(int32_t irq, void *dev_id);
static void swic_workqueue(struct work_struct *work);
static void swic_interrupt_exec(void);
static swic_detect_type swic_type_detect(u8 *set_mode, struct swic_read_reg_data *reg_data);
static u8 swic_interrupt_earphone(struct swic_read_reg_data *reg_data);
static void swic_interrupt_switch(struct swic_read_reg_data *reg_data, int switch_state);
static void swic_usb_connected(void);
static void swic_interrupt_plug_out(void);
static void swic_interrupt_chg_earphone(struct swic_read_reg_data *reg_data);
static int32_t swic_mic_check(void);
static int32_t swic_suspend(struct i2c_client *client, pm_message_t mesg);
static int32_t swic_resume(struct i2c_client *client);

#if SWIC_DEBUG
static unsigned int read_count = 0;

static int swic_dump_dev_open(struct inode *ip, struct file *fp)
{
	read_count = 0;

	return 0;
}
static ssize_t swic_dump(struct file *fp, char __user *buf,
				size_t count, loff_t *pos)
{
	char dump_text[512];
	unsigned int len;
	snprintf(dump_text, 512,
	         "00H: 0x%02x\n01H: 0x%02x\n02H: 0x%02x\n03H: 0x%02x\n04H: 0x%02x\n"
	         "05H: 0x%02x\n06H: 0x%02x\n07H: 0x%02x\n08H: 0x%02x\n10H: 0x%02x\n11H: 0x%02x\n"
	         "GPIO 27:%d\nGPIO 93:%d\n",
	         dump_data.reg_00h,
	         dump_data.reg_01h,
	         dump_data.reg_02h,
	         dump_data.reg_03h,
	         dump_data.reg_04h,
	         dump_data.reg_05h,
	         dump_data.reg_06h,
	         dump_data.reg_07h,
	         dump_data.reg_08h,
	         dump_data.reg_10h,
	         dump_data.reg_11h,
	         gpio_get_value(g_swic_vbus_rst_gpio),
	         gpio_get_value(g_swic_hs_det_gpio));
	len = strlen(dump_text);
	len -= read_count;
	if (count > len) {
		count = len;
	}
	if (count > 0) {
		if (copy_to_user(buf, dump_text + read_count, count)) {
			return -EFAULT;
		}
	}
	read_count += count;

	return count;
}

static const char swic_dump_dev_shortname[] = "swic_dump";
static struct file_operations swic_dump_dev_fops = {
	.owner = THIS_MODULE,
	.open = swic_dump_dev_open,
	.read = swic_dump,
};

static struct miscdevice swic_dump_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = swic_dump_dev_shortname,
	.fops = &swic_dump_dev_fops,
};
#endif

static int32_t swic_suspend(struct i2c_client *client, pm_message_t mesg)
{
	SWIC_DBG_PRINT("%s: \n", __func__);
	swic_device_available = false;
	if (device_may_wakeup(&client->dev))
	{
		SWIC_DBG_PRINT("%s: enable irq wake. \n", __func__);
		enable_irq_wake(swic_irq);
		disable_irq(swic_irq);
	}
	return 0;
}

static int32_t swic_resume(struct i2c_client *client)
{
	SWIC_DBG_PRINT("%s: \n", __func__);
	swic_device_available = true;
	if (device_may_wakeup(&client->dev))
	{
		SWIC_DBG_PRINT("%s: disable irq wake. \n", __func__);
		disable_irq_wake(swic_irq);
		enable_irq(swic_irq);
	}
	return 0;
}

static void swic_i2c_disable_unprepare(struct i2c_client *client)
{
	i2c_clk_disable_unprepare(client->adapter);
}

static void swic_i2c_prepare_enable(struct i2c_client *client)
{
	int32_t ret = 0;

	ret = i2c_clk_prepare_enable(client->adapter);
	if (ret) {
		printk(KERN_ERR "LC824204 swic_i2c_prepare_enable failed.\n");
	}
}

static int32_t swic_write_data(struct i2c_client *client, u8 reg, u8 reg_mask, u8 val)
{
	int32_t err = 0;
	int32_t retry_count = 0;
	int32_t i;
	struct i2c_msg msg[1];
	unsigned char data[2];

	SWIC_DBG_PRINT("%s: reg=%x, reg_mask=%x, val=%x\n", __func__, reg, reg_mask, val);

	if (!client || !client->adapter)
	{
		printk(KERN_WARNING "LC824204 adapter info failed.\n");
		return -ENODEV;
	}

	data[0] = reg;
	data[1] = (val & reg_mask);
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	do {
		for(i=0; i<SWIC_I2C_RESUME_RETRY_NUM; i++) {
			if(swic_device_available == true)
				break;
			msleep(1);
		}
		if(i >= SWIC_I2C_RESUME_RETRY_NUM) {
			printk(KERN_ERR "LC824204 I2C not resume error.\n");
			return -EBUSY;
		}
		err = i2c_transfer(client->adapter, msg, SWIC_I2C_WRITE_SIZE);
		if (err == SWIC_I2C_WRITE_SIZE)
		{
			return 0;
		}
		msleep(SWIC_I2C_RETRIES_WAIT);
	} while (++retry_count < SWIC_I2C_RETRIES_NUM);

	printk(KERN_WARNING "LC824204 %02xH write failed.\n", reg);
	return err;
}

static int32_t swic_read_data(struct i2c_client *client, u8 reg, u8 reg_mask)
{
	int32_t err = 0;
	int32_t retry_count = 0;
	int32_t i;
	struct i2c_msg msg[2];
	u8 reg_buf, data_buf = 0;

	SWIC_DBG_PRINT("%s: reg=%x, reg_mask=%x\n", __func__, reg, reg_mask);

	if (!client || !client->adapter)
	{
		printk(KERN_WARNING "LC824204 adapter info failed.\n");
		return -ENODEV;
	}

	reg_buf = reg;
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg_buf;
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = &data_buf;

	do {
		for(i=0; i<SWIC_I2C_RESUME_RETRY_NUM; i++) {
			if(swic_device_available == true)
				break;
			msleep(1);
		}
		if(i >= SWIC_I2C_RESUME_RETRY_NUM) {
			printk(KERN_ERR "LC824204 I2C not resume error.\n");
			return -EBUSY;
		}
		err = i2c_transfer(client->adapter, msg, SWIC_I2C_READ_SIZE);
		if (err == SWIC_I2C_READ_SIZE)
		{
			SWIC_DBG_PRINT("%s: val=%x\n", __func__, (data_buf & reg_mask));
			return (data_buf & reg_mask);
		}
		msleep(SWIC_I2C_RETRIES_WAIT);
	} while (++retry_count < SWIC_I2C_RETRIES_NUM);

	printk(KERN_WARNING "LC824204 %02xH read failed.\n", reg);
	return 0;
}

u8 swic_set_swic_det_wait(int32_t *val)
{
	if (!val) {
		return -EINVAL;
	}
	SWIC_INF_PRINT("%s: val[0]:%x\n", __func__, val[0]);
	dm_set_swic_det_wait = val[0];
	return 0;
}

u8 swic_get_dminfo(unsigned char cmd, int32_t *val)
{
	if (!val) {
		return -EINVAL;
	}

	switch (cmd)
	{
		case SWIC_DM_DRIVER_GET_SW_SET_MODE:
		{
			*val = dm_info_sw_set_mode;
		}
		break;
		case SWIC_DM_DRIVER_GET_DETECTION_ID_STATE:
		{
			*val = dm_info_detection_id_state;
		}
		break;
		case SWIC_DM_DRIVER_GET_INTERRUPT_CASE:
		{
			*val = dm_info_interrupt_case;
		}
		break;
		case SWIC_DM_DRIVER_GET_CHG_DET_MODE:
		{
			*val = dm_info_chg_det_mode;
		}
		break;
		default:
		{
		}
		break;
	}
	return 0;
}

static void swic_fix_accessory(u8 val)
{
	int32_t i;

	if (swic_is_fix_accessory(val) != true) {
		return;
	}
	SWIC_INF_PRINT("%s: ### val:%02x\n", __func__, val);
	if (0 <= SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_02H, val)) {
		if(swic_set_mode == val) {
			SWIC_INF_PRINT("%s: ### accessory no change\n", __func__);
			return;
		}

		printk("%s: ### fix_accessory = 0x%x\n", __func__, val);
		swic_set_mode_prev = swic_set_mode;
		swic_set_mode = val;
		if (swic_cb_info == NULL) {
			return;
		}
		for (i = 0; i < MAX_CALLBACK_CLIENTS; ++i) {
			if (swic_cb_info->cb_tbl[i] &&
			    swic_cb_info->cb_tbl[i]->fn) {
				swic_cb_info->cb_tbl[i]->fn(val);
			}
		}
	}
}

static irqreturn_t swic_interrupt(int32_t irq, void *dev_id)
{
	if (swic_irq != irq)
	{
		return IRQ_NONE;
	}
	schedule_work(&swic_info->work);

	return IRQ_HANDLED;
}

static void swic_workqueue(struct work_struct *work)
{
	wake_lock(&swic_wake_lock);

	SWIC_DBG_PRINT("%s: \n", __func__);
	swic_i2c_prepare_enable(swic_info->client);
	swic_interrupt_exec();
	swic_i2c_disable_unprepare(swic_info->client);

	wake_unlock(&swic_wake_lock);
}

static void swic_interrupt_exec()
{
	u8 set_mode = SWIC_UNINITIALIZE;
	struct swic_read_reg_data reg_data;
	swic_detect_type det_type = SWIC_DETECT_UNKNOWN;
	u8 val;

#if SWIC_DEBUG
	dump_data.reg_00h = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_00H);
	dump_data.reg_01h = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_01H);
	dump_data.reg_02h = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_02H);
	dump_data.reg_03h = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_03H);
	dump_data.reg_04h = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_04H);
	dump_data.reg_05h = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_05H);
	dump_data.reg_06h = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_06H);
	dump_data.reg_07h = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_07H);
	dump_data.reg_08h = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_08H);
	dump_data.reg_10h = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_10H);
	dump_data.reg_11h = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_11H);

	SWIC_DBG_PRINT(" %02xH: 0x%02x\n", SWIC_REG_00H, dump_data.reg_00h);
	SWIC_DBG_PRINT(" %02xH: 0x%02x\n", SWIC_REG_01H, dump_data.reg_01h);
	SWIC_DBG_PRINT(" %02xH: 0x%02x\n", SWIC_REG_02H, dump_data.reg_02h);
	SWIC_DBG_PRINT(" %02xH: 0x%02x\n", SWIC_REG_03H, dump_data.reg_03h);
	SWIC_DBG_PRINT(" %02xH: 0x%02x\n", SWIC_REG_04H, dump_data.reg_04h);
	SWIC_DBG_PRINT(" %02xH: 0x%02x\n", SWIC_REG_05H, dump_data.reg_05h);
	SWIC_DBG_PRINT(" %02xH: 0x%02x\n", SWIC_REG_06H, dump_data.reg_06h);
	SWIC_DBG_PRINT(" %02xH: 0x%02x\n", SWIC_REG_07H, dump_data.reg_07h);
	SWIC_DBG_PRINT(" %02xH: 0x%02x\n", SWIC_REG_08H, dump_data.reg_08h);
	SWIC_DBG_PRINT(" %02xH: 0x%02x\n", SWIC_REG_10H, dump_data.reg_10h);
	SWIC_DBG_PRINT(" %02xH: 0x%02x\n", SWIC_REG_11H, dump_data.reg_11h);
#endif

	val = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_11H);
	if((val & SWIC_DATA_COMP_OVP) == SWIC_DATA_COMP_OVP) {
		SWIC_INF_PRINT("%s: detect OVP [%02xH:0x%02x]\n", __func__, SWIC_REG_11H, val);
	}

	det_type = swic_type_detect(&set_mode, &reg_data);
	SWIC_DBG_PRINT("%s: ##start# set_mode:%x, sw_state:%x, interrupt_case:%x, id_state:%x, chg_det_mode:%x\n",
	                __func__, set_mode, reg_data.sw_state, reg_data.interrupt_case,
	                                    reg_data.id_state, reg_data.chg_det_mode);

	switch (det_type) {
		case SWIC_DETECT_EARPHONE:
			set_mode = swic_interrupt_earphone(&reg_data);
			if (set_mode != SWIC_NO_ACCESSORY) {
				SWIC_SET_DM_INFO(reg_data.sw_state, reg_data.id_state,
				                 reg_data.interrupt_case, reg_data.chg_det_mode);
			}
			break;
		case SWIC_DETECT_USB:
			if (set_mode == SWIC_AC_ADAPTER) {
				SWIC_SET_DM_INFO(reg_data.sw_state, reg_data.id_state,
				                 reg_data.interrupt_case, reg_data.chg_det_mode);
			}
			swic_usb_connected();
			break;
		case SWIC_DETECT_SWITCH_ON:
			swic_interrupt_switch(&reg_data, 1);
			break;
		case SWIC_DETECT_SWITCH_OFF:
			swic_interrupt_switch(&reg_data, 0);
			break;
		case SWIC_DETECT_PLUGOUT:
			set_mode = SWIC_NO_ACCESSORY;
			swic_interrupt_plug_out();
			break;
		case SWIC_DETECT_CHG_EARPHONE:
			SWIC_SET_DM_INFO(reg_data.sw_state, reg_data.id_state,
			                 reg_data.interrupt_case, reg_data.chg_det_mode);
			swic_interrupt_chg_earphone(&reg_data);
			break;
		case SWIC_DETECT_UNKNOWN:
		default:
			break;
	}

#if SWIC_DEBUG
	dump_data.reg_01h = reg_data.sw_state;
	dump_data.reg_02h = swic_set_mode;
	dump_data.reg_03h = reg_data.id_state;
	dump_data.reg_04h = reg_data.interrupt_case;
	dump_data.reg_10h = reg_data.chg_det_mode;
	dump_data.reg_05h = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_05H);
	dump_data.reg_06h = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_06H);
	dump_data.reg_07h = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_07H);
#endif

	SWIC_DBG_PRINT("%s: ##end  # set_mode:%x, sw_state:%x, interrupt_case:%x, id_state:%x, chg_det_mode:%x\n",
	                __func__, set_mode, reg_data.sw_state, reg_data.interrupt_case,
	                                    reg_data.id_state, reg_data.chg_det_mode);
}

static swic_detect_type swic_type_detect(u8 *set_mode, struct swic_read_reg_data *reg_data)
{
	swic_detect_type det_type = SWIC_DETECT_UNKNOWN;
	struct swic_read_reg_data reg_data_prev;
	u8 retry_cnt = 0;

	if (swic_set_mode == SWIC_NO_ACCESSORY  || swic_set_mode == SWIC_UNINITIALIZE) {
		reg_data->sw_state = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_01H);
		if (0 <= SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_07H, SWIC_DATA_ADC_ON)) {
			reg_data->id_state = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_03H);
		}
		reg_data->interrupt_case = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_04H);
		reg_data->chg_det_mode = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_10H);

		do {
			SWIC_DBG_PRINT("%s: ### sw_state:%x, interrupt_case:%x, id_state:%x, chg_det_mode:%x\n",
		            __func__, reg_data->sw_state, reg_data->interrupt_case,
			                  reg_data->id_state, reg_data->chg_det_mode);
			if (SWIC_REG_READ_RETRY == retry_cnt) {
				SWIC_DBG_PRINT("%s: ### sw_state_prev:%x, interrupt_case_prev:%x, "
				               "id_state_prev:%x, chg_det_mode_prev:%x\n", __func__ ,
				                reg_data_prev.sw_state, reg_data_prev.interrupt_case,
                                reg_data_prev.id_state, reg_data_prev.chg_det_mode);
				SWIC_INF_PRINT("%s: retry count over.\n", __func__);
				SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_05H, SWIC_DATA_05H_ON);
				SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_05H, SWIC_DATA_05H_SW);
				return SWIC_DETECT_UNKNOWN;
			}
			retry_cnt++;
			reg_data_prev.sw_state = reg_data->sw_state;
			reg_data_prev.id_state = reg_data->id_state;
			reg_data_prev.interrupt_case = reg_data->interrupt_case;
			reg_data_prev.chg_det_mode = reg_data->chg_det_mode;

			msleep(dm_set_swic_det_wait * 10);   /* (10ms * DM setting) sleep */

			reg_data->sw_state = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_01H);
			if (0 <= SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_07H, SWIC_DATA_ADC_ON)) {
				reg_data->id_state = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_03H);
			}
			reg_data->interrupt_case = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_04H);
			reg_data->chg_det_mode = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_10H);

			if (reg_data->sw_state == reg_data_prev.sw_state &&
			    reg_data->id_state == reg_data_prev.id_state &&
			    reg_data->interrupt_case == reg_data_prev.interrupt_case &&
			    reg_data->chg_det_mode == reg_data_prev.chg_det_mode) {
				break;
			}
		} while (1);

		if (!(reg_data->interrupt_case & SWIC_DATA_04H_SW)) {
			SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_05H, SWIC_DATA_05H_ON);
			SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_05H, SWIC_DATA_05H_SW);
		}

		*set_mode = (reg_data->id_state << 3) | (reg_data->sw_state & SWIC_DATA_01H_DETECT);

		/* Plug out check */
		/* (04H[0] == 1b) or (04H[1] == 1b) => (04H[1:0] & 0x03) != 0 */
		/* 01H[7:1] == 0x7C => (01H[7:0] & 0xFE) == 0xF8 */
		if ((reg_data->interrupt_case & SWIC_DATA_04H_DET) &&
			((reg_data->sw_state & SWIC_REG_01H_M) == SWIC_DATA_01H_PLUGOUT_DET)) {
			SWIC_DBG_PRINT("%s:SWIC_DETECT_PLUGOUT\n", __func__);
			return SWIC_DETECT_PLUGOUT;
		}
		/* Detect USB( or AC Charger). */
		/* 04H[0] == 0b and 04H[1] == 1b => (04H[1:0] & 0x03) == 0x02 */
		if ((reg_data->interrupt_case & SWIC_DATA_04H_DET) == SWIC_DATA_04H_DET_CHG) {
			SWIC_DBG_PRINT("%s:SWIC_DETECT_USB\n", __func__);
			if ((*set_mode == SWIC_USB_MODE) &&
			    ((reg_data->chg_det_mode == SWIC_DATA_CHG_DET_MODE1) ||
			     (reg_data->chg_det_mode == SWIC_DATA_CHG_DET_MODE2) ||
			     (reg_data->chg_det_mode == SWIC_DATA_CHG_DET_MODE3) ||
			     (reg_data->chg_det_mode == SWIC_DATA_CHG_DET_MODE4))) {
				*set_mode = SWIC_AC_ADAPTER;
			}
			det_type = SWIC_DETECT_USB;
		}
		/* Detect EarPhone.
		 * (  04H[0] == 1b and 04H[1] == 1b) or
		 * (  04H[0] == 1b and 04H[1] == 0b) or
		 * (!(04H[0] == 0b and 04H[1] == 1b))
		 */
		else {
			SWIC_DBG_PRINT("%s:SWIC_DETECT_EARPHONE\n", __func__);
			det_type = SWIC_DETECT_EARPHONE;
		}
	}
	/* swic_set_mode != SWIC_NO_ACCESSORY && swic_set_mode != SWIC_UNINITIALIZE */
	else {
		reg_data->sw_state = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_01H);
		reg_data->interrupt_case = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_04H);

		if (!(reg_data->interrupt_case & SWIC_DATA_04H_SW)) {
			SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_05H, SWIC_DATA_05H_ON);
			if ((reg_data->interrupt_case & SWIC_DATA_04H_DET)) {
				SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_05H, SWIC_DATA_05H_SW);
			} else {
				SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_05H, SWIC_DATA_05H_OFF);
			}
		}

		if (0 <= SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_07H, SWIC_DATA_ADC_ON)) {
			reg_data->id_state = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_03H);
		}
		reg_data->chg_det_mode = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_10H);

		*set_mode = (reg_data->id_state << 3) | (reg_data->sw_state & SWIC_DATA_01H_DETECT);

		if(swic_set_mode == SWIC_USB_MODE &&
			 (reg_data->id_state == SWIC_DATA_AUDIO_MIC_MONO ||
			  reg_data->id_state == SWIC_DATA_AUDIO_MIC_STEREO ||
			  reg_data->id_state == SWIC_DATA_AUDIO_MIC_MONO_CHG ||
			  reg_data->id_state == SWIC_DATA_AUDIO_CHG_SW_ON_INS ||
			  reg_data->id_state == SWIC_DATA_AUDIO_SW_ON_INS)) {
			SWIC_DBG_PRINT("%s:SWIC_DETECT_EARPHONE already USB_MODE\n", __func__);
			det_type = SWIC_DETECT_EARPHONE;
		} else if ((reg_data->interrupt_case & SWIC_DATA_04H_DET) &&
			((reg_data->sw_state & SWIC_REG_01H_M) == SWIC_DATA_01H_PLUGOUT_DET)) {
			/* Plug out check */
			/* (04H[0] == 1b) or (04H[1] == 1b) => (04H[1:0] & 0x03) != 0 */
			/* 01H[7:1] == 0x7C => (01H[7:0] & 0xFE) == 0xF8 */
			SWIC_DBG_PRINT("%s:SWIC_DETECT_PLUGOUT\n", __func__);
			det_type = SWIC_DETECT_PLUGOUT;
		} else if ((reg_data->interrupt_case == SWIC_DATA_04H_SW && earphone_sw_state == 0) ||
		            reg_data->interrupt_case == SWIC_DATA_04H_SW_ON) {
			SWIC_DBG_PRINT("%s:SWIC_DETECT_SWITCH_ON\n", __func__);
			det_type = SWIC_DETECT_SWITCH_ON;
		} else if ((reg_data->interrupt_case == SWIC_DATA_04H_SW && earphone_sw_state == 1) ||
		            reg_data->interrupt_case == SWIC_DATA_04H_SW_OFF) {
			SWIC_DBG_PRINT("%s:SWIC_DETECT_SWITCH_OFF\n", __func__);
			det_type = SWIC_DETECT_SWITCH_OFF;
		} else if ((reg_data->interrupt_case & SWIC_DATA_04H_SW)  &&
		           (reg_data->interrupt_case & SWIC_DATA_04H_DET_ID)) {
			SWIC_DBG_PRINT("%s:SWIC_DETECT_ID_and_SWITCH_ON_or_OFF\n", __func__);
			det_type = SWIC_DETECT_UNKNOWN;
			SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_05H, SWIC_DATA_05H_ON);
			SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_05H, SWIC_DATA_05H_OFF);
		} else if ((reg_data->sw_state & SWIC_REG_01H_M) == SWIC_DATA_01H_PLUGOUT_DET) {
			SWIC_INF_PRINT("%s:SWIC_DETECT_CHG_EARPHONE PLUGOUT\n", __func__);
			det_type = SWIC_DETECT_PLUGOUT;
		} else {
			SWIC_DBG_PRINT("%s:SWIC_DETECT_CHG_EARPHONE\n", __func__);
			det_type = SWIC_DETECT_CHG_EARPHONE;
		}
	}

	return det_type;
}
static u8 swic_interrupt_earphone(struct swic_read_reg_data *reg_data)
{
	u8 set_mode = SWIC_UNINITIALIZE;

	/* 03H == 0x1E or 03H == 0x18 */
	if (reg_data->id_state == SWIC_DATA_AUDIO_MIC_MONO ||
		reg_data->id_state == SWIC_DATA_AUDIO_MIC_STEREO) {
		SWIC_DBG_PRINT("%s:03H == 0x1E or 03H == 0x18\n", __func__);
		SWIC_DBG_PRINT("%s:SWIC_AUDIO_MIC_STEREO\n", __func__);
		SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_05H, SWIC_DATA_05H_OFF);
		SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_06H, SWIC_DATA_06H_ALL);
		gpio_set_value(g_swic_vbus_rst_gpio, 1);
		/* 02H = 0xC0 */
		set_mode = SWIC_AUDIO_MIC_STEREO;
		swic_fix_accessory(set_mode);

		if (!swic_mic_check()) {
			SWIC_DBG_PRINT("%s:SWIC_AUDIO_CHG_STEREO\n", __func__);
			/* 02H = 0xC4 */
			set_mode = SWIC_AUDIO_CHG_STEREO;
			swic_fix_accessory(set_mode);
		}

		SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_07H, SWIC_DATA_ADC_ON);
	}
	/* 03H == 0x1C */
	else if (reg_data->id_state == SWIC_DATA_AUDIO_MIC_MONO_CHG) {
		SWIC_DBG_PRINT("%s:SWIC_AUDIO_CHG_MIC_MONO\n", __func__);
		SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_05H, SWIC_DATA_05H_OFF);
		SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_06H, SWIC_DATA_06H_ALL);
		set_mode = SWIC_AUDIO_CHG_MIC_MONO;
		swic_fix_accessory(set_mode);
		swic_mic_check();
		SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_07H, SWIC_DATA_ADC_ON);
	}
	/* 03H != 0x12 and 03H != 0x11 */
	else if (reg_data->id_state != SWIC_DATA_AUDIO_SW_ON_INS &&
	         reg_data->id_state != SWIC_DATA_AUDIO_CHG_SW_ON_INS) {
		SWIC_DBG_PRINT("%s:03H != 0x12 and 03H != 0x11\n", __func__);
		SWIC_DBG_PRINT("%s:SWIC_NO_ACCESSORY\n", __func__);
		set_mode = SWIC_NO_ACCESSORY;
		swic_fix_accessory(set_mode);
	} else {
		/* 01H[2] == 0 */
		if ((reg_data->sw_state & SWIC_DATA_01H_VBUS_DET) == 0) {
			SWIC_DBG_PRINT("%s:01H[2] == 0\n", __func__);
			SWIC_DBG_PRINT("%s:SWIC_AUDIO_MIC_STEREO\n", __func__);
			set_mode = SWIC_AUDIO_MIC_STEREO;
			swic_fix_accessory(set_mode);
			gpio_set_value(g_swic_vbus_rst_gpio, 1);
			if (!swic_mic_check())
			{
				SWIC_DBG_PRINT("%s:SWIC_AUDIO_CHG_STEREO\n", __func__);
				set_mode = SWIC_AUDIO_CHG_STEREO;
				swic_fix_accessory(set_mode);
			}

		} else {
			SWIC_DBG_PRINT("%s:SWIC_AUDIO_CHG_STEREO\n", __func__);
			set_mode = SWIC_AUDIO_CHG_STEREO;
			swic_fix_accessory(set_mode);
		}
		SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_05H, SWIC_DATA_05H_OFF);
		SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_06H, SWIC_DATA_06H_ALL);
		SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_07H, SWIC_DATA_ADC_ON);
	}
	return set_mode;
}

static void swic_interrupt_switch(struct swic_read_reg_data *reg_data, int sw_state)
{
	msleep(10);
	reg_data->sw_state = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_01H);
	if (0 <= SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_07H, SWIC_DATA_ADC_ON)) {
		reg_data->id_state = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_03H);
	}
	reg_data->chg_det_mode = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_10H);

	SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_05H, SWIC_DATA_05H_ON);
	SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_05H, SWIC_DATA_05H_OFF);

	if (sw_state == 1) {
		SWIC_DBG_PRINT("%s:HS SW ON\n", __func__);
		/* 03H == 0x12 or 03H == 0x11 */
		if (reg_data->id_state == SWIC_DATA_AUDIO_SW_ON_INS ||
		    reg_data->id_state == SWIC_DATA_AUDIO_CHG_SW_ON_INS) {
			earphone_sw_state = 1;
			input_report_key(swic_info->hs_sw_input, KEY_MEDIA, 1);
			input_sync(swic_info->hs_sw_input);
			SWIC_INF_PRINT("%s: KEY_MEDIA==1 earphone_sw:ON\n", __func__);
		}
	} else {
		SWIC_DBG_PRINT("%s:HS SW OFF\n", __func__);
		/* 03H != 0x12 and 03H != 0x11 */
		if (reg_data->id_state != SWIC_DATA_AUDIO_SW_ON_INS &&
		    reg_data->id_state != SWIC_DATA_AUDIO_CHG_SW_ON_INS) {
			earphone_sw_state = 0;
			input_report_key(swic_info->hs_sw_input, KEY_MEDIA, 0);
			input_sync(swic_info->hs_sw_input);
			SWIC_INF_PRINT("%s: KEY_MEDIA==0 earphone_sw:OFF\n", __func__);
		}
	}
}

static void swic_usb_connected(void)
{
	swic_fix_accessory(SWIC_USB_MODE);
}

static void swic_interrupt_plug_out(void)
{
	SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_05H, SWIC_DATA_05H_ON);
	SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_05H, SWIC_DATA_05H_SW);

	swic_fix_accessory(SWIC_NO_ACCESSORY);
	if (gpio_get_value(g_swic_vbus_rst_gpio))
		gpio_set_value(g_swic_vbus_rst_gpio, 0);
	if (!gpio_get_value(g_swic_hs_det_gpio))
		gpio_set_value(g_swic_hs_det_gpio, 1);

	SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_07H, SWIC_DATA_ALL_NORMAL);
	SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_05H, SWIC_DATA_05H_SW);
	SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_06H, SWIC_DATA_06H_STANDBY);
	mic_exist = 0;
	mic_check = 1;
	mic_check_timeout = false;
	wake_up(&swic_mic_wait);
	earphone_sw_state = 0;
}

static void swic_interrupt_chg_earphone(struct swic_read_reg_data *reg_data)
{
	msleep(100);
	reg_data->sw_state = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_01H);
	if (0 <= SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_07H, SWIC_DATA_ADC_ON)) {
		reg_data->id_state = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_03H);
	}
	reg_data->chg_det_mode = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_10H);

	SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_05H, SWIC_DATA_05H_ON);
	SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_05H, SWIC_DATA_05H_OFF);
}

int32_t swic_reg_cbfunc(struct swic_event_callback* cb)
{
	int32_t i;

	if (swic_cb_info == NULL) {
		swic_cb_info = kzalloc(sizeof(struct swic_cb_info), GFP_KERNEL);
		if (swic_cb_info == NULL) {
			return -1;
		}
	}
	for (i = 0; i < MAX_CALLBACK_CLIENTS; ++i) {
		if (swic_cb_info->cb_tbl[i] == NULL) {
			swic_cb_info->cb_tbl[i] = cb;
			return 0;
		}
	}
	return -1;
}

int32_t swic_unreg_cbfunc(struct swic_event_callback* cb)
{
	int32_t i;

	if (swic_cb_info != NULL) {
		for (i = 0; i < MAX_CALLBACK_CLIENTS; ++i) {
			if (swic_cb_info->cb_tbl[i] == cb) {
				swic_cb_info->cb_tbl[i] = NULL;
				return 0;
			}
		}
	}
	return -1;
}

void swic_set_mic_exist(bool mic)
{
	SWIC_DBG_PRINT("%s:before %d -> after  %d\n",__func__, mic_exist, mic ? 1:0);
	mic_exist = mic ? 1:0;
	if (mic_check_timeout) {
		SWIC_DBG_PRINT("%s:already timeout.",__func__);
		mic_check_timeout = false;
		if (SWIC_AUDIO_MIC_STEREO == SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_02H) &&
			mic_exist == 0) {
			SWIC_INF_PRINT("%s:write 02H SWIC_AUDIO_CHG_STEREO for 02H.",__func__);
			gpio_set_value(g_swic_vbus_rst_gpio, 0);
			SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_02H, SWIC_AUDIO_CHG_STEREO);
		}
	} else {
		mic_check = 1;
		wake_up(&swic_mic_wait);
	}
}

static int32_t swic_mic_check(void)
{
	long ret;

	mic_check = 0;
	msleep(100);
	mic_check_timeout = false;
	SWIC_DBG_PRINT("%s:mic_check before %d\n",__func__, gpio_get_value(g_swic_hs_det_gpio));
	gpio_set_value(g_swic_hs_det_gpio, 0);
	SWIC_DBG_PRINT("%s:mic_check after %d\n", __func__, gpio_get_value(g_swic_hs_det_gpio));
	ret = wait_event_interruptible_timeout(swic_mic_wait, mic_check, SWIC_MBHC_TIMEOUT * HZ);
	mic_check_timeout = ret == 0 ? true : false;
	mic_exist = mic_check_timeout ? 1 : mic_exist;

	if (!mic_exist) {
		gpio_set_value(g_swic_vbus_rst_gpio, 0);
	}
	return mic_exist;
}

swic_accessory_enum swic_get_accessory(void)
{
	return swic_set_mode;
}

static int32_t swic_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int32_t ret = 0;

	SWIC_INF_PRINT("%s: probe start!\n", __func__);

	swic_info = kzalloc(sizeof(struct lc824204_swic), GFP_KERNEL);

	if (!swic_info)
	{
		printk(KERN_ERR "LC824204 failed to allocate driver data\n");
		return -ENOMEM;
	}
	i2c_set_clientdata(client, swic_info);
	swic_info->client = client;

	if (!client->dev.of_node) {
		pr_err("%s: No platform supplied from device tree.\n", __func__);
		return -EINVAL;
	}

	g_swic_int_gpio = of_get_named_gpio(client->dev.of_node, "kc,swic_int-gpio", 0);
	if (!gpio_is_valid(g_swic_int_gpio)) {
		pr_err("%s: of_get_named_gpio failed.\n", __func__);
		return -EINVAL;
	}
	g_swic_hs_det_gpio = of_get_named_gpio(client->dev.of_node, "kc,swic_hs_det-gpio", 0);
	if (!gpio_is_valid(g_swic_hs_det_gpio)) {
		pr_err("%s: of_get_named_gpio failed.\n", __func__);
		return -EINVAL;
	}
	g_swic_vbus_rst_gpio = of_get_named_gpio(client->dev.of_node, "kc,swic_vbus_rst-gpio", 0);
	if (!gpio_is_valid(g_swic_vbus_rst_gpio)) {
		pr_err("%s: of_get_named_gpio failed.\n", __func__);
		return -EINVAL;
	}
	ret = gpio_request(g_swic_int_gpio, "swic_int");
	if (ret < 0)
	{
		printk(KERN_ERR "LC824204 failed to request GPIO %d, ret %d\n", g_swic_int_gpio, ret);
		goto failed_free_i2c;
	}
	ret = gpio_direction_input(g_swic_int_gpio);
	if (ret < 0)
	{
		printk(KERN_ERR "LC824204 failed to configure direction for GPIO %d, error %d\n", g_swic_int_gpio, ret);
		goto failed_free_i2c;
	}
	swic_irq = gpio_to_irq(g_swic_int_gpio);
	if (swic_irq < 0)
	{
		printk(KERN_ERR "LC824204 Unable to get irq number for GPIO %d, error %d\n", g_swic_int_gpio, swic_irq);
		goto failed_free_i2c;
	}

	wake_lock_init( &swic_wake_lock, WAKE_LOCK_SUSPEND, "swic" );

	INIT_WORK(&swic_info->work, swic_workqueue);

	ret = request_any_context_irq(swic_irq, swic_interrupt, IRQF_TRIGGER_FALLING, "swic", swic_info);
	if (ret < 0)
	{
		printk(KERN_ERR "LC824204 Unable to claim irq %d; error %d\n", swic_irq, ret);
		goto failed_free_all;
	}
	disable_irq(swic_irq);

	msleep(2);
	ret = SWIC_REG_WRITE_DATA(client, SWIC_REG_05H, SWIC_DATA_05H_ON);
	if (ret < 0)
	{
		printk(KERN_ERR "%s:LC824204 Unable to write in register 0x%02x; error %d\n",
		                 __func__, SWIC_REG_05H, ret);
		goto failed_free_all;
	}
	ret = SWIC_REG_WRITE_DATA(client, SWIC_REG_05H, SWIC_DATA_05H_SW);
	if (ret < 0)
	{
		printk(KERN_ERR "%s:LC824204 Unable to write in register 0x%02x; error %d\n",
		                 __func__, SWIC_REG_05H, ret);
		goto failed_free_all;
	}
	ret = SWIC_REG_WRITE_DATA(client, SWIC_REG_06H, SWIC_DATA_06H_STANDBY);
	if (ret < 0)
	{
		printk(KERN_ERR "%s:LC824204 Unable to write in register 0x%02x; error %d\n",
		                 __func__, SWIC_REG_06H, ret);
		goto failed_free_all;
	}
	ret = SWIC_REG_WRITE_DATA(client, SWIC_REG_00H, SWIC_DATA_CHG_DET_OFF);
	if (ret < 0)
	{
		printk(KERN_ERR "%s:LC824204 Unable to write in register 0x%02x; error %d\n",
		                 __func__, SWIC_REG_00H, ret);
		goto failed_free_all;
	}
	ret = SWIC_REG_WRITE_DATA(client, SWIC_REG_00H, SWIC_DATA_CHG_DET_ON);
	if (ret < 0)
	{
		printk(KERN_ERR "%s:LC824204 Unable to write in register 0x%02x; error %d\n",
		                 __func__, SWIC_REG_00H, ret);
		goto failed_free_all;
	}

	if (!swic_info->hs_sw_input) {
		swic_info->hs_sw_input = input_allocate_device();
		if (!swic_info->hs_sw_input) {
			SWIC_INF_PRINT("Can't allocate swic input device\n");
			ret = -ENOMEM;
			goto failed_free_all;
		}
		swic_info->hs_sw_input->name = "swic_hs_key";
		ret = input_register_device(swic_info->hs_sw_input);
		if (ret) {
			SWIC_INF_PRINT("Can't register pon key: %d\n", ret);
			goto failed_free_all;
		}
	}
	__set_bit(INPUT_PROP_NO_DUMMY_RELEASE, swic_info->hs_sw_input->propbit);
	input_set_capability(swic_info->hs_sw_input, EV_KEY, KEY_MEDIA);

	init_waitqueue_head(&swic_mic_wait);

	schedule_work(&swic_info->work);
	enable_irq(swic_irq);
	device_init_wakeup(&client->dev, 1);

#if SWIC_DEBUG
	ret = misc_register(&swic_dump_device);
	if (ret) {
		pr_err("%s:device file create failed", __func__);
	}
#endif

	SWIC_INF_PRINT("%s: probe success!\n", __func__);

	return 0;

failed_free_all:
	if (swic_info->hs_sw_input)
		input_free_device(swic_info->hs_sw_input);
	free_irq(swic_irq, swic_info);
	swic_irq = 0;
failed_free_i2c:
	i2c_set_clientdata(client, NULL);
	kfree(swic_info);
	swic_info = NULL;

	return ret;
}

static void swic_shutdown(struct i2c_client *client)
{
	int32_t ret = 0;

	SWIC_INF_PRINT("%s: shutdown start!\n", __func__);

	ret = SWIC_REG_WRITE_DATA(client, SWIC_REG_06H, SWIC_DATA_06H_NO);
	if (ret < 0)
	{
		printk(KERN_ERR "%s:LC824204 Unable to write in register 0x%02x; error %d\n",
		                __func__, SWIC_REG_06H, ret);
	}
	ret = SWIC_REG_READ_DATA(client, SWIC_REG_06H);
	if (ret < 0)
	{
		printk(KERN_ERR "%s:LC824204 Unable to read in register 0x%02x; error %d\n",
		                 __func__, SWIC_REG_06H, ret);
	}

	if (swic_info->hs_sw_input)
		input_unregister_device(swic_info->hs_sw_input);

	i2c_set_clientdata(client, NULL);
	cancel_work_sync(&swic_info->work);
	free_irq(swic_irq, swic_info);
	swic_irq = 0;
	kfree(swic_info);
	swic_info = NULL;
	wake_lock_destroy( &swic_wake_lock );
	swic_set_mode = SWIC_UNINITIALIZE;
#if SWIC_DEBUG
	misc_deregister(&swic_dump_device);
#endif
}

static struct of_device_id lc824204_table[] = {
	{ .compatible = "kc,LC824204"},
	{ },
};

static const struct i2c_device_id lc824204_id[] = {
	{ "LC824204", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lc824204_id);

static struct i2c_driver swic_driver = {
	.driver = {
		.name   = "LC824204",
		.of_match_table = lc824204_table,
	},
	.probe		= swic_probe,
	.shutdown	= swic_shutdown,
	.suspend	= swic_suspend,
	.resume		= swic_resume,
	.id_table	= lc824204_id,
};

module_i2c_driver(swic_driver);

MODULE_AUTHOR("KYOCERA");
MODULE_DESCRIPTION("LC824204 SWIC Driver");
MODULE_LICENSE("GPL v2");
