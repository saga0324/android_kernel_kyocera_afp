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
 * (C) 2018 KYOCERA Corporation
 * (C) 2019 KYOCERA Corporation
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include <linux/workqueue.h>

#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>

#include <misc/swic/swic.h>
#include "swic_dm_driver.h"

#define UNIT_TEST 0

#define SWIC_DEBUG 0
#if SWIC_DEBUG
#define SWIC_INF_PRINT(fmt, ...) printk(KERN_INFO  fmt, ##__VA_ARGS__)
#define SWIC_DBG_PRINT(fmt, ...) printk(KERN_DEBUG fmt, ##__VA_ARGS__)
#else
#define SWIC_INF_PRINT(fmt, ...) printk(KERN_INFO  fmt, ##__VA_ARGS__)
#define SWIC_DBG_PRINT(fmt, ...)
#endif

/*--- SWIC REGISTER ADDRESS ---*/
#define SWIC_REG_00H  0x00
#define SWIC_REG_01H  0x01

#define SWIC_REG_00H_M  0xFF
#define SWIC_REG_01H_M  0xFF

/*--- SWIC SET DATA VALUE ---*/
#define SWIC_DATA_00H_MIC_DETECT_SUCCESS 0x04
#define SWIC_DATA_01H_DEFAULT            0xFF
#define SWIC_DATA_01H_USB_MODE           0xFB
#define SWIC_DATA_01H_ALL_SWITCHES_OFF   0x7F
#define SWIC_DATA_01H_ALL_SWITCHES_ON    0x89
#define SWIC_DATA_01H_START_MIC_DETECT   0xF7
#define SWIC_DATA_00H_REG_ACEESS_FAIL	 0x00
#define SWIC_DATA_01H_REG_ACEESS_FAIL	 0x00

#define SWIC_I2C_WRITE_SIZE                     1
#define SWIC_I2C_READ_SIZE                      2
#define SWIC_I2C_RESUME_RETRY_NUM               10

#define SWIC_RETRIES_NUM  2

#if UNIT_TEST
#define SWIC_REG_RETRIES_NUM_FOR_MIC_DETECT     2
#else
#define SWIC_REG_RETRIES_NUM_FOR_MIC_DETECT     10
#endif

#define SWIC_REG_WRITE_DATA(client, reg, val)  swic_write_data(client, reg, reg##_M, val)
#define SWIC_REG_READ_DATA(client, reg)        swic_read_data(client, reg, reg##_M)

#define SWIC_SET_DM_INFO(p1, p2)            {dm_info_swic_reg00_state = p1; dm_info_swic_reg01_state = p2;}
#define MAX_CALLBACK_CLIENTS                    4

#define SWIC_WAIT_1MSEC        1
#define SWIC_WAIT_50MSEC       50
#define SWIC_WAIT_100MSEC      100

#define SWIC_SIFT_8BIT         8
#define SWIC_SIFT_16BIT        16
#define SWIC_SIFT_24BIT        24

#define SWIC_RTN_COMPLETE                0
#define SWIC_RTN_ERR_REG_W_MIC_DETECT   -1
#define SWIC_RTN_ERR_REG_R_MIC_DETECT   -2
#define SWIC_RTN_ERR_CHK_MIC_DETECT     -3
#define SWIC_RTN_ERR_REG_W_ALL_SW_ON    -4
#define SWIC_RTN_ERR_REG_W_ALL_SW_OFF   -5
#define SWIC_RTN_ERR_REG_W_SET_DEFAULT  -6
#define SWIC_RTN_ERR_REG_W_USB_CONNECT  -7

typedef enum
{
	EN_EARPHONE_IN_PROC_REG_W_MIC_DETECT = 0,
	EN_EARPHONE_IN_PROC_REG_R_MIC_DETECT,
	EN_EARPHONE_IN_PROC_CHK_MIC_DETECT,
	EN_EARPHONE_IN_PROC_REG_W_ALL_SW_ON,
	EN_EARPHONE_IN_PROC_MAX,
} e_EARPHONE_IN_PROC_STATE;

typedef enum
{
	EN_EARPHONE_OUT_PROC_REG_W_ALL_SW_OFF = 0,
	EN_EARPHONE_OUT_PROC_REG_W_SET_DEFAULT,
	EN_EARPHONE_OUT_PROC_REG_W_USB_CONNECT,
	EN_EARPHONE_OUT_PROC_MAX,
} e_EARPHONE_OUT_PROC_STATE;

struct swic_reg
{
	u8 err_cnt;
};

typedef struct
{
	u8	run;
	u8	detect_mic;
	int32_t	retry_cnt;
	e_EARPHONE_IN_PROC_STATE	next_proc, prev_proc;
	struct swic_reg  mic_detect_reg_w;
	struct swic_reg  mic_detect_reg_r;
	struct swic_reg  mic_detect_check;
	struct swic_reg  all_sw_on_reg_w;
} t_EARPHONE_IN_PROC_INFO;

typedef struct
{
	u8 run;
	e_EARPHONE_OUT_PROC_STATE	next_proc, prev_proc;
	struct swic_reg  all_switch_off_reg_w;
	struct swic_reg  set_default_reg_w;
	struct swic_reg  usb_connenct_reg_w;
} t_EARPHONE_OUT_PROC_INFO;

struct pi3a6386_swic {
	struct i2c_client	*client;
	struct work_struct	work;
	t_EARPHONE_IN_PROC_INFO		earphone_in_proc_info;
	t_EARPHONE_OUT_PROC_INFO	earphone_out_proc_info;
};

struct swic_cb_info {
	struct swic_event_callback *cb_tbl[MAX_CALLBACK_CLIENTS];
};

typedef int32_t (*swic_earphone_in_out_proc_func)(void);

typedef struct
{
	swic_earphone_in_out_proc_func fn;
	u8	retry_num;
} t_EARPHONE_IN_PROC_TBL;

typedef struct
{
	swic_earphone_in_out_proc_func fn;
	u8	retry_num;
} t_EARPHONE_OUT_PROC_TBL;

static struct pi3a6386_swic *swic_info;

static u8 swic_set_mode = SWIC_UNINITIALIZE;

static u8 swic_set_mode_prev = SWIC_UNINITIALIZE;

static u8 swic_interrupt_audio = 0;

struct swic_read_reg_data {
	u8 reg00_data;
	u8 reg01_data;
};

static u8 dm_info_ccic_reg01_state = 0;
static u8 dm_info_ccic_reg02_state = 0;
static u8 dm_info_ccic_reg03_state = 0;
static u8 dm_info_ccic_reg04_state = 0;
static u8 dm_info_swic_reg00_state = 0;
static u8 dm_info_swic_reg01_state = 0;

static struct swic_cb_info *swic_cb_info = NULL;

static int g_swic_usbsw_pon_gpio = -1;
static int g_swic_hs_det_gpio = -1;

static int swic_device_available = true;

static int32_t swic_write_data(struct i2c_client *client, u8 reg, u8 reg_mask, u8 val);
static int32_t swic_read_data(struct i2c_client *client, u8 reg, u8 reg_mask);
static void swic_fix_accessory(u8 val);
static void swic_reset_ic(void);
static int32_t swic_insert_earphone_proc(void);
static int32_t swic_remove_earphone_proc(void);
static int32_t swic_suspend(struct i2c_client *client, pm_message_t mesg);
static int32_t swic_resume(struct i2c_client *client);
static void swic_update_reg_data_for_dm(u8 reg00h_data, u8 reg01_data);
static void swic_set_next_proc_for_earphone_in(e_EARPHONE_IN_PROC_STATE next_proc);
static void swic_set_next_proc_for_earphone_out(e_EARPHONE_OUT_PROC_STATE next_proc);
static void swic_earphone_in_proc_init(void);
static int32_t swic_earphone_in_proc_reg_w_mic_detect(void);
static int32_t swic_earphone_in_proc_reg_r_mic_detect(void);
static int32_t swic_earphone_in_proc_chk_mic_detect(void);
static int32_t swic_earphone_in_proc_reg_w_all_sw_on(void);
static void swic_earphone_out_proc_init(void);
static int32_t swic_earphone_out_proc_reg_w_all_sw_off(void);
static int32_t swic_earphone_out_proc_reg_w_set_default(void);
static int32_t swic_earphone_out_proc_reg_w_usb_connect(void);

static const t_EARPHONE_IN_PROC_TBL earphone_in_proc_tbl[EN_EARPHONE_IN_PROC_MAX] =
{
/* STAGE1:EN_EARPHONE_IN_PROC_REG_W_MIC_DETECT */
	{ swic_earphone_in_proc_reg_w_mic_detect, 	SWIC_RETRIES_NUM },
/* STAGE2:EN_EARPHONE_IN_PROC_REG_R_MIC_DETECT */
	{ swic_earphone_in_proc_reg_r_mic_detect, 	SWIC_RETRIES_NUM },
/* STAGE3:EN_EARPHONE_IN_PROC_CHK_MIC_DETECT */
	{ swic_earphone_in_proc_chk_mic_detect, 	SWIC_RETRIES_NUM },
/* STAGE4:EN_EARPHONE_IN_PROC_REG_W_ALL_SW_ON */
	{ swic_earphone_in_proc_reg_w_all_sw_on,	SWIC_RETRIES_NUM },
};

static const t_EARPHONE_OUT_PROC_TBL earphone_out_proc_tbl[EN_EARPHONE_OUT_PROC_MAX] =
{
/* STAGE1:EN_EARPHONE_OUT_PROC_REG_W_ALL_SW_OFF */
	{ swic_earphone_out_proc_reg_w_all_sw_off, 	SWIC_RETRIES_NUM },
/* STAGE2:EN_EARPHONE_OUT_PROC_REG_W_SET_DEFAULT */
	{ swic_earphone_out_proc_reg_w_set_default, SWIC_RETRIES_NUM },
/* STAGE3:EN_EARPHONE_OUT_PROC_REG_W_USB_CONNECT */
	{ swic_earphone_out_proc_reg_w_usb_connect, SWIC_RETRIES_NUM },
};

static int32_t swic_suspend(struct i2c_client *client, pm_message_t mesg)
{
	SWIC_DBG_PRINT("%s: \n", __func__);
	swic_device_available = false;
	return 0;
}

static int32_t swic_resume(struct i2c_client *client)
{
	SWIC_DBG_PRINT("%s: \n", __func__);
	swic_device_available = true;
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
		printk(KERN_ERR "PI3A386 swic_i2c_prepare_enable failed.\n");
	}
}

static int32_t swic_write_data(struct i2c_client *client, u8 reg, u8 reg_mask, u8 val)
{
	int32_t err = 0;
	int32_t i;
	struct i2c_msg msg[1];
	unsigned char data[2];

	SWIC_DBG_PRINT("%s: reg=%x, reg_mask=%x, val=%x\n", __func__, reg, reg_mask, val);

	if (!client || !client->adapter)
	{
		printk(KERN_WARNING "PI3A6386 adapter info failed.\n");
		return -ENODEV;
	}

	data[0] = reg;
	data[1] = (val & reg_mask);
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	for(i=0; i<SWIC_I2C_RESUME_RETRY_NUM; i++) {
		if(swic_device_available == true)
			break;
		msleep(1);
	}
	if(i >= SWIC_I2C_RESUME_RETRY_NUM) {
		printk(KERN_ERR "PI3A6386 I2C not resume error.\n");
		return -EBUSY;
	}
	err = i2c_transfer(client->adapter, msg, SWIC_I2C_WRITE_SIZE);
	if (err == SWIC_I2C_WRITE_SIZE)
	{
		return 0;
	}

	printk(KERN_WARNING "PI3A6386 %02xH write failed.\n", reg);
	return err;
}

static int32_t swic_read_data(struct i2c_client *client, u8 reg, u8 reg_mask)
{
	int32_t err = 0;
	int32_t i;
	struct i2c_msg msg[2];
	u8 reg_buf, data_buf = 0;

	SWIC_DBG_PRINT("%s: reg=%x, reg_mask=%x\n", __func__, reg, reg_mask);

	if (!client || !client->adapter)
	{
		printk(KERN_WARNING "PI3A6386 adapter info failed.\n");
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

	for(i=0; i<SWIC_I2C_RESUME_RETRY_NUM; i++) {
		if(swic_device_available == true)
			break;
		msleep(1);
	}
	if(i >= SWIC_I2C_RESUME_RETRY_NUM) {
		printk(KERN_ERR "PI3A6386 I2C not resume error.\n");
		return -EBUSY;
	}
	err = i2c_transfer(client->adapter, msg, SWIC_I2C_READ_SIZE);
	if (err == SWIC_I2C_READ_SIZE)
	{
		SWIC_DBG_PRINT("%s: val=%x\n", __func__, (data_buf & reg_mask));
		return data_buf & reg_mask;
	}

	printk(KERN_WARNING "PI3A6386 %02xH read failed.\n", reg);
	return err;
}

static void swic_update_reg_data_for_dm(u8 reg00h_data, u8 reg01_data)
{
	struct swic_read_reg_data reg_data;

	/* Save SWIC Register State for DM */
	reg_data.reg00_data = reg00h_data;
	SWIC_DBG_PRINT("%s:### DM INFO REG00H=[0x%02x]\n", __func__, reg_data.reg00_data);
	reg_data.reg01_data = reg01_data;
	SWIC_DBG_PRINT("%s:### DM INFO REG01H=[0x%02x]\n", __func__, reg_data.reg01_data);

	SWIC_SET_DM_INFO(reg_data.reg00_data, reg_data.reg01_data);
}

static void swic_set_next_proc_for_earphone_in(e_EARPHONE_IN_PROC_STATE next_proc)
{
	swic_info->earphone_in_proc_info.prev_proc = swic_info->earphone_in_proc_info.next_proc;
	swic_info->earphone_in_proc_info.next_proc = next_proc;
	SWIC_DBG_PRINT("%s:### next_proc=[%d], prev_proc=[%d]\n", __func__, next_proc, swic_info->earphone_in_proc_info.prev_proc);
}

static void swic_set_next_proc_for_earphone_out(e_EARPHONE_OUT_PROC_STATE next_proc)
{
	swic_info->earphone_out_proc_info.prev_proc = swic_info->earphone_out_proc_info.next_proc;
	swic_info->earphone_out_proc_info.next_proc = next_proc;
	SWIC_DBG_PRINT("%s:### next_proc=[%d], prev_proc=[%d]\n", __func__, next_proc, swic_info->earphone_out_proc_info.prev_proc);
}

static void swic_reset_ic(void)
{
	SWIC_INF_PRINT("%s:### swic reset\n", __func__);
	gpio_set_value(g_swic_usbsw_pon_gpio, 0);
	msleep(SWIC_WAIT_1MSEC);
	gpio_set_value(g_swic_usbsw_pon_gpio, 1);
}

void swic_detect_audio(bool is_audio)
{
	if(is_audio)
	{
		swic_interrupt_audio = 1;
	}
	else
	{
		swic_interrupt_audio = 0;
	}
}

void swic_notify_from_cclogic(swic_state_enum en_state)
{
	SWIC_INF_PRINT("%s:### en_state=[0x%02x]\n", __func__, en_state);
	switch(en_state)
	{
		case SWIC_MODE_PLUGOUT_TO_AUDIO:
			swic_i2c_prepare_enable(swic_info->client);

			swic_insert_earphone_proc();

			swic_i2c_disable_unprepare(swic_info->client);
			break;
		case SWIC_MODE_AUDIO_TO_PLUGOUT:
			swic_i2c_prepare_enable(swic_info->client);

			swic_remove_earphone_proc();

			swic_i2c_disable_unprepare(swic_info->client);
			break;
		case SWIC_MODE_PLUGOUT_TO_USB:
			swic_fix_accessory(SWIC_USB_MODE);
			break;
		case SWIC_MODE_USB_TO_PLUGOUT:
			swic_fix_accessory(SWIC_NO_ACCESSORY);
			break;
		default:
			break;
	}
}

static void swic_earphone_in_proc_init(void)
{
	swic_info->earphone_in_proc_info.run = 1;
	swic_info->earphone_in_proc_info.next_proc = swic_info->earphone_in_proc_info.prev_proc = EN_EARPHONE_IN_PROC_REG_W_MIC_DETECT;
	swic_info->earphone_in_proc_info.retry_cnt = 0;
	swic_info->earphone_in_proc_info.mic_detect_reg_w.err_cnt = 0;
	swic_info->earphone_in_proc_info.mic_detect_reg_r.err_cnt = 0;
	swic_info->earphone_in_proc_info.mic_detect_check.err_cnt = 0;
	swic_info->earphone_in_proc_info.all_sw_on_reg_w.err_cnt = 0;
}

static int32_t swic_earphone_in_proc_reg_w_mic_detect(void)
{
	int32_t ret = SWIC_RTN_COMPLETE;

	if(0 > SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_01H, SWIC_DATA_01H_START_MIC_DETECT))
	{
		/* I2C Reg Failed */
		if(earphone_in_proc_tbl[EN_EARPHONE_IN_PROC_REG_W_MIC_DETECT].retry_num <= ++swic_info->earphone_in_proc_info.mic_detect_reg_w.err_cnt)
		{
			/* Set ErrLog */
			SWIC_INF_PRINT("%s:###[RegW] Mic Detect Start(Err!!) err_cnt=[%d/%d]\n",
								__func__, swic_info->earphone_in_proc_info.mic_detect_reg_w.err_cnt, earphone_in_proc_tbl[EN_EARPHONE_IN_PROC_REG_W_MIC_DETECT].retry_num);
			/* Update DM INFO */
			swic_update_reg_data_for_dm(SWIC_DATA_00H_REG_ACEESS_FAIL, SWIC_DATA_01H_REG_ACEESS_FAIL);
			ret = SWIC_RTN_ERR_REG_W_MIC_DETECT;
			swic_info->earphone_in_proc_info.run = 0;
		}
		else
		{
			/* SWIC Reset & Retry */
			SWIC_DBG_PRINT("%s:###[RegW] Mic Detect Start(Retry!) err_cnt=[%d/%d]\n",
								__func__, swic_info->earphone_in_proc_info.mic_detect_reg_w.err_cnt, earphone_in_proc_tbl[EN_EARPHONE_IN_PROC_REG_W_MIC_DETECT].retry_num);
			swic_reset_ic();
		}
	}
	else
	{
		/* I2C Reg Success */
		SWIC_DBG_PRINT("%s:###[RegW] Mic Detect Start!(Success!) err_cnt=[%d/%d]\n",
								__func__, swic_info->earphone_in_proc_info.mic_detect_reg_w.err_cnt, earphone_in_proc_tbl[EN_EARPHONE_IN_PROC_REG_W_MIC_DETECT].retry_num);
		/* next stage */
		swic_set_next_proc_for_earphone_in(EN_EARPHONE_IN_PROC_REG_R_MIC_DETECT);
		SWIC_DBG_PRINT("%s:###[Wait] %d ms Wait for Mic Detect Proc \n", __func__, SWIC_WAIT_100MSEC);
		msleep(SWIC_WAIT_100MSEC);
	}
	return ret;
}

static int32_t swic_earphone_in_proc_reg_r_mic_detect(void)
{
	int32_t ret = SWIC_RTN_COMPLETE;

	swic_info->earphone_in_proc_info.detect_mic = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_00H);

	if(0 > swic_info->earphone_in_proc_info.detect_mic)
	{
		/* I2C Reg Failed */
		if(earphone_in_proc_tbl[EN_EARPHONE_IN_PROC_REG_R_MIC_DETECT].retry_num <= ++swic_info->earphone_in_proc_info.mic_detect_reg_r.err_cnt)
		{
			/* Set ErrLog */
			SWIC_INF_PRINT("%s:###[RegR] Check Mic Detect(Err!!) err_cnt=[%d/%d]\n",
								__func__, swic_info->earphone_in_proc_info.mic_detect_reg_r.err_cnt, earphone_in_proc_tbl[EN_EARPHONE_IN_PROC_REG_R_MIC_DETECT].retry_num);
			/* Update DM INFO */
			swic_update_reg_data_for_dm(SWIC_DATA_00H_REG_ACEESS_FAIL, SWIC_DATA_01H_REG_ACEESS_FAIL);
			ret = SWIC_RTN_ERR_REG_R_MIC_DETECT;
			swic_info->earphone_in_proc_info.run = 0;
		}
		else
		{
			/* SWIC Reset & go back to STAGE1  */
			SWIC_DBG_PRINT("%s:###[RegR] Check Mic Detect(Retry!) err_cnt=[%d/%d]\n",
								__func__, swic_info->earphone_in_proc_info.mic_detect_reg_r.err_cnt, earphone_in_proc_tbl[EN_EARPHONE_IN_PROC_REG_R_MIC_DETECT].retry_num);
			swic_info->earphone_in_proc_info.detect_mic = 0;
			swic_reset_ic();
			swic_set_next_proc_for_earphone_in(EN_EARPHONE_IN_PROC_REG_W_MIC_DETECT);
		}
	}
	else
	{
		/* I2C Reg Success */
		/* next stage */
		SWIC_DBG_PRINT("%s:###[RegR] Check Mic Detect!(Success!) err_cnt=[%d/%d]\n",
								__func__, swic_info->earphone_in_proc_info.mic_detect_reg_r.err_cnt, earphone_in_proc_tbl[EN_EARPHONE_IN_PROC_REG_R_MIC_DETECT].retry_num);
		swic_set_next_proc_for_earphone_in(EN_EARPHONE_IN_PROC_CHK_MIC_DETECT);
	}
	return ret;
}

static int32_t swic_earphone_in_proc_chk_mic_detect(void)
{
	int32_t ret = SWIC_RTN_COMPLETE;
	u8 reg_data = 0;

	if(0 == (swic_info->earphone_in_proc_info.detect_mic & SWIC_DATA_00H_MIC_DETECT_SUCCESS))
	{
		SWIC_DBG_PRINT("%s:###[Chk] Check Mic Detect OK or NG (OK!) retry_cnt=[%d/%d], err_cnt=[%d/%d]\n",
													 				__func__, swic_info->earphone_in_proc_info.retry_cnt, SWIC_REG_RETRIES_NUM_FOR_MIC_DETECT, 
																		swic_info->earphone_in_proc_info.mic_detect_check.err_cnt, earphone_in_proc_tbl[EN_EARPHONE_IN_PROC_CHK_MIC_DETECT].retry_num);
		/* Update DM INFO */
		reg_data = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_01H);
		swic_update_reg_data_for_dm(swic_info->earphone_in_proc_info.detect_mic, reg_data);
		/* next stage */
		swic_info->earphone_in_proc_info.retry_cnt = 0;
		swic_set_next_proc_for_earphone_in(EN_EARPHONE_IN_PROC_REG_W_ALL_SW_ON);
	}
	else
	{
		SWIC_DBG_PRINT("%s:###[Chk] Check Mic Detect OK or NG (NG!) data=[0x%02x], retry_cnt=[%d/%d], err_cnt=[%d/%d]\n",
													 				__func__, swic_info->earphone_in_proc_info.detect_mic, swic_info->earphone_in_proc_info.retry_cnt, SWIC_REG_RETRIES_NUM_FOR_MIC_DETECT, 
																		swic_info->earphone_in_proc_info.mic_detect_check.err_cnt, earphone_in_proc_tbl[EN_EARPHONE_IN_PROC_CHK_MIC_DETECT].retry_num);
		if(SWIC_REG_RETRIES_NUM_FOR_MIC_DETECT <= ++swic_info->earphone_in_proc_info.retry_cnt)
		{
			if(earphone_in_proc_tbl[EN_EARPHONE_IN_PROC_CHK_MIC_DETECT].retry_num <= ++swic_info->earphone_in_proc_info.mic_detect_check.err_cnt)
			{
				/* Set ErrLog */
				SWIC_INF_PRINT("%s:###[Chk] Check Mic Detect OK or NG (Retry Err!!) retry_cnt=[%d/%d], err_cnt=[%d/%d]\n",
															 				__func__, swic_info->earphone_in_proc_info.retry_cnt, SWIC_REG_RETRIES_NUM_FOR_MIC_DETECT, 
																					swic_info->earphone_in_proc_info.mic_detect_check.err_cnt, earphone_in_proc_tbl[EN_EARPHONE_IN_PROC_CHK_MIC_DETECT].retry_num);
				/* Update DM INFO */
				reg_data = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_01H);
				swic_update_reg_data_for_dm(swic_info->earphone_in_proc_info.detect_mic, reg_data);
				ret = SWIC_RTN_ERR_CHK_MIC_DETECT;
				swic_info->earphone_in_proc_info.run = 0;
			}
			else
			{
				/* SWIC Reset & go back to STAGE1 */
				swic_info->earphone_in_proc_info.detect_mic = 0;
				swic_info->earphone_in_proc_info.retry_cnt = 0;
				swic_reset_ic();
				swic_set_next_proc_for_earphone_in(EN_EARPHONE_IN_PROC_REG_W_MIC_DETECT);
			}
		}
		else
		{
			SWIC_DBG_PRINT("%s:###[Chk] Check Mic Detect OK or NG (Retry(Max:10times)!) retry_cnt=[%d/%d], err_cnt=[%d/%d]\n",
														 				__func__, swic_info->earphone_in_proc_info.retry_cnt, SWIC_REG_RETRIES_NUM_FOR_MIC_DETECT, 
																			swic_info->earphone_in_proc_info.mic_detect_check.err_cnt, earphone_in_proc_tbl[EN_EARPHONE_IN_PROC_CHK_MIC_DETECT].retry_num);
			swic_set_next_proc_for_earphone_in(EN_EARPHONE_IN_PROC_REG_R_MIC_DETECT);
			SWIC_DBG_PRINT("%s:###[Wait] %d ms Wait for Mic Detect Proc \n", __func__, SWIC_WAIT_100MSEC);
			msleep(SWIC_WAIT_100MSEC);
		}
	}
	return ret;
}

static int32_t swic_earphone_in_proc_reg_w_all_sw_on(void)
{
	int32_t ret = SWIC_RTN_COMPLETE;
	u8 reg_data = 0;

	if(0 > SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_01H, SWIC_DATA_01H_ALL_SWITCHES_ON))
	{
		/* I2C Reg Failed */
		if(earphone_in_proc_tbl[EN_EARPHONE_IN_PROC_REG_W_ALL_SW_ON].retry_num <= ++swic_info->earphone_in_proc_info.all_sw_on_reg_w.err_cnt)
		{
			/* Set ErrLog */
			SWIC_INF_PRINT("%s:###[RegW] All Switches ON(Err!!) err_cnt=[%d/%d]\n",
								__func__, swic_info->earphone_in_proc_info.all_sw_on_reg_w.err_cnt, earphone_in_proc_tbl[EN_EARPHONE_IN_PROC_REG_W_ALL_SW_ON].retry_num);
			/* Update DM INFO */
			swic_update_reg_data_for_dm(SWIC_DATA_00H_REG_ACEESS_FAIL, SWIC_DATA_01H_REG_ACEESS_FAIL);
			ret = SWIC_RTN_ERR_REG_W_ALL_SW_ON;
			swic_info->earphone_in_proc_info.run = 0;
		}
		else
		{
			/* SWIC Reset & go back to STAGE1 */
			SWIC_DBG_PRINT("%s:###[RegW] All Switches ON(Retry!) err_cnt=[%d/%d]\n",
								__func__, swic_info->earphone_in_proc_info.all_sw_on_reg_w.err_cnt, earphone_in_proc_tbl[EN_EARPHONE_IN_PROC_REG_W_ALL_SW_ON].retry_num);
			swic_reset_ic();
			swic_set_next_proc_for_earphone_in(EN_EARPHONE_IN_PROC_REG_W_MIC_DETECT);
		}
	}
	else
	{
		/* I2C Reg Success */
		SWIC_DBG_PRINT("%s:###[RegW] All Switches ON(Success!) err_cnt=[%d/%d]\n",
								__func__, swic_info->earphone_in_proc_info.all_sw_on_reg_w.err_cnt, earphone_in_proc_tbl[EN_EARPHONE_IN_PROC_REG_W_ALL_SW_ON].retry_num);
		/* HS_DET_EN (MSM_GPIO27) =High */
		gpio_set_value(g_swic_hs_det_gpio, 1);
		SWIC_DBG_PRINT("%s:###[GPIO] HS_DET_EN(MSM_GPIO27) =High\n", __func__);
		msleep(SWIC_WAIT_50MSEC);
		swic_fix_accessory(SWIC_AUDIO_MODE);
		/* Update DM INFO */
		reg_data = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_01H);
		swic_update_reg_data_for_dm(swic_info->earphone_in_proc_info.detect_mic, reg_data);
		swic_info->earphone_in_proc_info.run = 0;
	}
	return ret;
}

static void swic_earphone_out_proc_init(void)
{
	swic_info->earphone_out_proc_info.run = 1;
	swic_info->earphone_out_proc_info.next_proc = swic_info->earphone_out_proc_info.prev_proc = EN_EARPHONE_OUT_PROC_REG_W_ALL_SW_OFF;
	swic_info->earphone_out_proc_info.all_switch_off_reg_w.err_cnt = 0;
	swic_info->earphone_out_proc_info.set_default_reg_w.err_cnt = 0;
	swic_info->earphone_out_proc_info.usb_connenct_reg_w.err_cnt = 0;
}

static int32_t swic_earphone_out_proc_reg_w_all_sw_off(void)
{
	int32_t ret = SWIC_RTN_COMPLETE;

	if(0 > SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_01H, SWIC_DATA_01H_ALL_SWITCHES_OFF))
	{
		/* I2C Reg Failed */
		if(earphone_out_proc_tbl[EN_EARPHONE_OUT_PROC_REG_W_ALL_SW_OFF].retry_num <= ++swic_info->earphone_out_proc_info.all_switch_off_reg_w.err_cnt)
		{
			/* Set ErrLog */
			SWIC_INF_PRINT("%s:###[RegW] All Switches OFF(Err!!) err_cnt=[%d/%d]\n",
									__func__, swic_info->earphone_out_proc_info.all_switch_off_reg_w.err_cnt, earphone_out_proc_tbl[EN_EARPHONE_OUT_PROC_REG_W_ALL_SW_OFF].retry_num);
			ret = SWIC_RTN_ERR_REG_W_ALL_SW_OFF;
			swic_info->earphone_out_proc_info.run = 0;
		}
		else
		{
			/* SWIC Reset & Retry */
			SWIC_DBG_PRINT("%s:###[RegW] All Switches OFF(Retry!) err_cnt=[%d/%d]\n",
									__func__, swic_info->earphone_out_proc_info.all_switch_off_reg_w.err_cnt, earphone_out_proc_tbl[EN_EARPHONE_OUT_PROC_REG_W_ALL_SW_OFF].retry_num);
			swic_reset_ic();
		}
	}
	else
	{
		/* I2C Reg Success */
		SWIC_DBG_PRINT("%s:###[RegW] All Switches OFF(Success!) err_cnt=[%d/%d]\n",
									__func__, swic_info->earphone_out_proc_info.all_switch_off_reg_w.err_cnt, earphone_out_proc_tbl[EN_EARPHONE_OUT_PROC_REG_W_ALL_SW_OFF].retry_num);
		/* next stage */
		swic_set_next_proc_for_earphone_out(EN_EARPHONE_OUT_PROC_REG_W_SET_DEFAULT);
		msleep(SWIC_WAIT_50MSEC);
	}
	return ret;
}

static int32_t swic_earphone_out_proc_reg_w_set_default(void)
{
	int32_t ret = SWIC_RTN_COMPLETE;

	if(0 > SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_01H, SWIC_DATA_01H_DEFAULT))
	{
		/* I2C Reg Failed */
		if(earphone_out_proc_tbl[EN_EARPHONE_OUT_PROC_REG_W_SET_DEFAULT].retry_num <= ++swic_info->earphone_out_proc_info.set_default_reg_w.err_cnt)
		{
			/* Set ErrLog */
			SWIC_INF_PRINT("%s:###[RegW] Set Default(Err!!) err_cnt=[%d/%d]\n",
									__func__, swic_info->earphone_out_proc_info.set_default_reg_w.err_cnt, earphone_out_proc_tbl[EN_EARPHONE_OUT_PROC_REG_W_SET_DEFAULT].retry_num);
			ret = SWIC_RTN_ERR_REG_W_SET_DEFAULT;
			swic_info->earphone_out_proc_info.run = 0;
		}
		else
		{
			/* SWIC Reset & go back to STAGE1 */
			SWIC_DBG_PRINT("%s:###[RegW] Set Default(Retry!) err_cnt=[%d/%d]\n",
									__func__, swic_info->earphone_out_proc_info.set_default_reg_w.err_cnt, earphone_out_proc_tbl[EN_EARPHONE_OUT_PROC_REG_W_SET_DEFAULT].retry_num);
			swic_reset_ic();
			swic_set_next_proc_for_earphone_out(EN_EARPHONE_OUT_PROC_REG_W_ALL_SW_OFF);
		}
	}
	else
	{
		/* I2C Reg Success */
		SWIC_DBG_PRINT("%s:###[RegW] Set Default(Success!) err_cnt=[%d/%d]\n",
									__func__, swic_info->earphone_out_proc_info.set_default_reg_w.err_cnt, earphone_out_proc_tbl[EN_EARPHONE_OUT_PROC_REG_W_SET_DEFAULT].retry_num);
		/* next stage */
		swic_set_next_proc_for_earphone_out(EN_EARPHONE_OUT_PROC_REG_W_USB_CONNECT);
		msleep(SWIC_WAIT_50MSEC);
	}
	return ret;
}

static int32_t swic_earphone_out_proc_reg_w_usb_connect(void)
{
	int32_t ret = SWIC_RTN_COMPLETE;

	if(0 > SWIC_REG_WRITE_DATA(swic_info->client, SWIC_REG_01H, SWIC_DATA_01H_USB_MODE))
	{
		/* I2C Reg Failed */
		if(earphone_out_proc_tbl[EN_EARPHONE_OUT_PROC_REG_W_USB_CONNECT].retry_num <= ++swic_info->earphone_out_proc_info.usb_connenct_reg_w.err_cnt)
		{
			/* Set ErrLog */
			SWIC_INF_PRINT("%s:###[RegW] USB Connect(Err!!) err_cnt=[%d/%d]\n",
									__func__, swic_info->earphone_out_proc_info.usb_connenct_reg_w.err_cnt, earphone_out_proc_tbl[EN_EARPHONE_OUT_PROC_REG_W_USB_CONNECT].retry_num);
			ret = SWIC_RTN_ERR_REG_W_USB_CONNECT;
			swic_info->earphone_out_proc_info.run = 0;
		}
		else
		{
			/* SWIC Reset & go back to STAGE1 */
			SWIC_DBG_PRINT("%s:###[RegW] USB Connect(Retry!) err_cnt=[%d/%d]\n",
									__func__, swic_info->earphone_out_proc_info.usb_connenct_reg_w.err_cnt, earphone_out_proc_tbl[EN_EARPHONE_OUT_PROC_REG_W_USB_CONNECT].retry_num);
			swic_reset_ic();
			swic_set_next_proc_for_earphone_out(EN_EARPHONE_OUT_PROC_REG_W_ALL_SW_OFF);
		}
	}
	else
	{
		/* I2C Reg Success */
		SWIC_DBG_PRINT("%s:###[RegW] USB Connect(Success!) err_cnt=[%d/%d]\n",
									__func__, swic_info->earphone_out_proc_info.usb_connenct_reg_w.err_cnt, earphone_out_proc_tbl[EN_EARPHONE_OUT_PROC_REG_W_USB_CONNECT].retry_num);
		msleep(SWIC_WAIT_50MSEC);
		swic_fix_accessory(SWIC_NO_ACCESSORY);
		swic_info->earphone_out_proc_info.run = 0;
	}
	return ret;
}

static int32_t swic_insert_earphone_proc(void)
{
	int32_t ret = SWIC_RTN_COMPLETE;

	SWIC_DBG_PRINT("%s:### start\n", __func__);
	/* initialize */
	swic_earphone_in_proc_init();

	do
	{
		/* Check for removed earphone */
		if(!swic_interrupt_audio)
		{
			SWIC_DBG_PRINT("%s:### Force termination for insert earphone sequence !! next_proc=[%d], prev_proc=[%d]\n",
								__func__, swic_info->earphone_in_proc_info.next_proc, swic_info->earphone_in_proc_info.prev_proc);
			return SWIC_RTN_COMPLETE;
		}
		/* Exe sequence of earphone insert */
		if(EN_EARPHONE_IN_PROC_MAX > swic_info->earphone_in_proc_info.next_proc)
		{
			ret = earphone_in_proc_tbl[swic_info->earphone_in_proc_info.next_proc].fn();
		}
		else
		{
			SWIC_INF_PRINT("%s:### Not Supported Proccess\n", __func__);
			swic_info->earphone_in_proc_info.run = 0;
		}
	} while(swic_info->earphone_in_proc_info.run);

	SWIC_DBG_PRINT("%s:### end\n", __func__);
	return ret;
}

static int32_t swic_remove_earphone_proc(void)
{
	int32_t ret = SWIC_RTN_COMPLETE;

	SWIC_DBG_PRINT("%s:### start\n", __func__);
	/* initialize */
	swic_earphone_out_proc_init();

	/* HS_DET_EN(MSM_GPIO27) = Low */
	gpio_set_value(g_swic_hs_det_gpio, 0);
	SWIC_DBG_PRINT("%s:### [GPIO] HS_DET_EN(MSM_GPIO27) =Low\n", __func__);

	do
	{
		/* Check for insert earphone */
		if(swic_interrupt_audio)
		{
			SWIC_DBG_PRINT("%s:### Force termination for remove earphone sequence !! next_proc=[%d], prev_proc=[%d]\n",
									__func__, swic_info->earphone_out_proc_info.next_proc, swic_info->earphone_out_proc_info.prev_proc);
			return SWIC_RTN_COMPLETE;
		}
		/* Exe sequence of earphone removed */
		if(EN_EARPHONE_OUT_PROC_MAX > swic_info->earphone_out_proc_info.next_proc)
		{
			ret = earphone_out_proc_tbl[swic_info->earphone_out_proc_info.next_proc].fn();
		}
		else
		{
			SWIC_INF_PRINT("%s:### Not Supported Proccess\n", __func__);
			swic_info->earphone_out_proc_info.run = 0;
		}
	} while(swic_info->earphone_out_proc_info.run);

	SWIC_DBG_PRINT("%s:### end\n", __func__);
	return ret;
}

u8 swic_get_dminfo(unsigned char cmd, int32_t *val)
{
	int32_t data[2];
    
	if (!val) {
		return -EINVAL;
	}

	SWIC_DBG_PRINT("%s:### start cmd=[0x%x]\n", __func__, cmd);

	switch (cmd)
	{
		case SWIC_DM_DRIVER_GET_SWIC_REG_STATET_MODE:
		{
			SWIC_DBG_PRINT("%s:### dm_info ccic_reg01=[0x%02x]\n", __func__, dm_info_ccic_reg01_state);
			SWIC_DBG_PRINT("%s:### dm_info ccic_reg02=[0x%02x]\n", __func__, dm_info_ccic_reg02_state);
			SWIC_DBG_PRINT("%s:### dm_info ccic_reg03=[0x%02x]\n", __func__, dm_info_ccic_reg03_state);
			SWIC_DBG_PRINT("%s:### dm_info ccic_reg04=[0x%02x]\n", __func__, dm_info_ccic_reg04_state);
			SWIC_DBG_PRINT("%s:### dm_info swic_reg00=[0x%02x]\n", __func__, dm_info_swic_reg00_state);
			SWIC_DBG_PRINT("%s:### dm_info swic_reg01=[0x%02x]\n", __func__, dm_info_swic_reg01_state);

			data[0] = (int32_t)((dm_info_ccic_reg04_state << SWIC_SIFT_24BIT)|(dm_info_ccic_reg03_state << SWIC_SIFT_16BIT)|(dm_info_ccic_reg02_state << SWIC_SIFT_8BIT)|dm_info_ccic_reg01_state);
			data[1] = (int32_t)((dm_info_swic_reg01_state << SWIC_SIFT_8BIT) | dm_info_swic_reg00_state);
			*val++ = data[0];
            *val =   data[1];
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

u8 swic_set_dminfo_from_cclogic(u8 ccic_reg01, u8 ccic_reg02, u8 ccic_reg03, u8 ccic_reg04)
{
	dm_info_ccic_reg01_state = ccic_reg01;
	dm_info_ccic_reg02_state = ccic_reg02;
	dm_info_ccic_reg03_state = ccic_reg03;
	dm_info_ccic_reg04_state = ccic_reg04;
	return 0;
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

swic_accessory_enum swic_get_accessory(void)
{
	return swic_set_mode;
}

static int32_t swic_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int32_t ret = 0;
#if SWIC_DEBUG
	u8 val_1, val_2 = 0;
#endif

	SWIC_INF_PRINT("%s: probe start!\n", __func__);

	swic_info = kzalloc(sizeof(struct pi3a6386_swic), GFP_KERNEL);

	if (!swic_info)
	{
		printk(KERN_ERR "PI3A6386 failed to allocate driver data\n");
		ret = -ENOMEM;
		goto failed_free_all;
	}
	i2c_set_clientdata(client, swic_info);
	swic_info->client = client;

	if (!client->dev.of_node) {
		pr_err("%s: No platform supplied from device tree.\n", __func__);
		ret = -EINVAL;
		goto failed_free_all;
	}

	g_swic_usbsw_pon_gpio = of_get_named_gpio(client->dev.of_node, "kc,swic_usbsw_pon-gpio", 0);
	if (!gpio_is_valid(g_swic_usbsw_pon_gpio)) {
		pr_err("%s: of_get_named_gpio failed (g_swic_usbsw_pon_gpio).\n", __func__);
		ret = -EINVAL;
		goto failed_free_all;
	}
	g_swic_hs_det_gpio = of_get_named_gpio(client->dev.of_node, "kc,swic_hs_det-gpio", 0);
	if (!gpio_is_valid(g_swic_hs_det_gpio)) {
		pr_err("%s: of_get_named_gpio failed (g_swic_hs_det_gpio).\n", __func__);
		ret = -EINVAL;
		goto failed_free_all;
	}

	msleep(2);

#if SWIC_DEBUG
	val_1 = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_00H);
	SWIC_DBG_PRINT("%s:### SWIC READ reg=[%02xH], data=[0x%02x]\n", __func__, SWIC_REG_00H, val_1);

	val_2 = SWIC_REG_READ_DATA(swic_info->client, SWIC_REG_01H);
	SWIC_DBG_PRINT("%s:### SWIC READ reg=[%02xH], data=[0x%02x]\n", __func__, SWIC_REG_01H, val_2);
#endif

	SWIC_INF_PRINT("%s: probe success!\n", __func__);

	return 0;

failed_free_all:
	i2c_set_clientdata(client, NULL);
	kfree(swic_info);
	swic_info = NULL;

	return ret;
}

static void swic_shutdown(struct i2c_client *client)
{
	SWIC_INF_PRINT("%s: shutdown start!\n", __func__);

	i2c_set_clientdata(client, NULL);
	cancel_work_sync(&swic_info->work);
	kfree(swic_info);
	kfree(swic_cb_info);
	swic_info = NULL;
	swic_cb_info = NULL;
	swic_set_mode = SWIC_UNINITIALIZE;
}

static struct of_device_id pi3a6386_table[] = {
	{ .compatible = "kc,PI3A6386"},
	{ },
};

static const struct i2c_device_id pi3a6386_id[] = {
	{ "PI3A6386", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pi3a6386_id);

static struct i2c_driver swic_driver = {
	.driver = {
		.name   = "PI3A6386",
		.of_match_table = pi3a6386_table,
	},
	.probe		= swic_probe,
	.shutdown	= swic_shutdown,
	.suspend	= swic_suspend,
	.resume		= swic_resume,
	.id_table	= pi3a6386_id,
};

module_i2c_driver(swic_driver);

MODULE_AUTHOR("KYOCERA");
MODULE_DESCRIPTION("PI3A6386 SWIC Driver");
MODULE_LICENSE("GPL v2");
