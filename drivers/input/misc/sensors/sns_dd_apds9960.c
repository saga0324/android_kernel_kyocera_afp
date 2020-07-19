/* This software is contributed or developed by KYOCERA Corporation.
 * (C) 2017 KYOCERA Corporation
 * (C) 2018 KYOCERA Corporation
 */
/******************************************************************************
 * MODULE       : rohm_rpr0521_i2c.c
 * FUNCTION     : Driver source for RPR0521,
 *              : Proximity Sensor(PS) and Ambient Light Sensor(ALS) IC.
 * AUTHOR       : Masafumi Seike
 * PROGRAMMED   : Sensor System Development Group, ROHM CO.,LTD.
 * MODIFICATION : Modified by ROHM, JUN/24/2014
 * REMARKS      :
 * COPYRIGHT    : Copyright (C) 2014 - ROHM CO.,LTD.
 *              : This program is free software; you can redistribute it and/or
 *              : modify it under the terms of the GNU General Public License
 *              : as published by the Free Software Foundation; either version 2
 *              : of the License, or (at your option) any later version.
 *              :
 *              : This program is distributed in the hope that it will be useful,
 *              : but WITHOUT ANY WARRANTY; without even the implied warranty of
 *              : MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *              : GNU General Public License for more details.
 *              :
 *              : You should have received a copy of the GNU General Public License
 *              : along with this program; if not, write to the Free Software
 *              : Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *****************************************************************************/
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/input.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/power_supply.h>

#include "sns_dd_apds9960_priv.h"
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
#include "sensor_driver.h"
#else
#include "sensor_util.h"
#endif	/*CONFIG_USE_MICON_SOFT_STRUCTURE*/
#include "als_sensor.h"
#include "ps_sensor.h"
#include "gs_sensor.h"
#include <linux/sensor_prevention.h>

#ifdef CONFIG_OEM_HKADC
#define TEMPERATURE_AVAILABLE
#endif

#define DUMMY_TEMP	1000
static int camera_temp = DUMMY_TEMP;
module_param(camera_temp, int, 0644);

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
#define	WAKE_LOCK_TIME_DETECT	(msecs_to_jiffies(200))
#define	WAKE_LOCK_TIME_NODETECT	(msecs_to_jiffies(1000))
#define GS_DEFAULT_IGNRTIME		(500)

static struct i2c_client *client_data = NULL;
static uint32_t initialize_done = 0;
static uint32_t g_ignr_time = GS_DEFAULT_IGNRTIME;
#endif	/*CONFIG_USE_MICON_SOFT_STRUCTURE*/


/******************************* define *******************************/
struct apds_reg_data {
	struct regulator* v_reg;
	uint32_t min_uV;
	uint32_t max_uV;
	uint32_t on_load_uA;
	uint32_t off_load_uA;
};

struct apds_power_ctrl_data {
	bool enabled;
	ktime_t power_off_time;
	int prev_mode;
	struct mutex apds_power_mutex;
	int power_off_on_interval_ms;
	int power_on_wait_ms;
	int power_normal_wait_ms;
};

#define NUM_OF_ALS_VAL 4

struct als_val {
	int		index;
	bool		ave_enable;
	uint32_t	report_lux;
	CALC_DATA	calc_data[NUM_OF_ALS_VAL];
};

#define ERRINFO_I2C_RESET_OCCURRED		0x00000001
#define ERRINFO_DEV_RESET_OCCURRED		0x00000002
#define ERRINFO_I2C_RECOVERY_FAILED		0x01000000
#define ERRINFO_I2C_RECOVERY_FAIL_MASK	0x0F000000
#define ERRINFO_DEV_RECOVERY_FAILED		0x20000000
#define ERRINFO_DEV_RECOVERY_FAIL_MASK	0xF0000000

/* structure of peculiarity to use by system */
typedef struct {
	struct i2c_client	*client;	/* structure pointer for i2c bus            */
	struct hrtimer		timer;		/* structure for timer handler              */
	struct work_struct	als_work;	/* structure for work queue                 */
	struct delayed_work	monitor_dwork;
	struct delayed_work	gs_ignrdet_dwork;
	int			delay_time;	/* delay time to set from application       */
	struct ps_sensor_info	ps_info;
	struct als_sensor_info	als_info;
	struct gs_sensor_info	gs_info;
	struct apds_reg_data	apds_vcc;
	struct apds_power_ctrl_data	apds_power;
	bool		apds_vdd_is_ldo;
	int			vprox_gpio;
	apds9960_dd_init_arg		init_data;
	POWERON_ARG		power_data;
	POWERON_ARG		power_data_last;
	int			als_en_cnt;
	int			ps_en_cnt;
	int			gs_en_cnt;
    bool		is_gs_ignr_enable;
    bool		tch_reject;
	DEVICE_VAL		dev_val;
	struct als_val		als_val;
	READ_DATA_PS_BUF	ps_val;
	READ_DATA_GS_BUF	gs_val;
	unsigned char		gs_cal_data[APDS9960_DD_GFIFO_MAX];
	int			gs_cal_cnt;
	int			cal_offset_up;
	int			cal_offset_down;
	int			cal_offset_left;
	int			cal_offset_right;
	int			ps_offset_ur;
	int			ps_offset_dl;
	int			ps_det;
	int			int_flg;
	int			color;
	bool			als_en_first;
	struct als_imit		imit;
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*gpio_state_active;
	struct pinctrl_state	*gpio_state_suspend;
	struct mutex		control_lock;
	struct mutex		ps_data_lock;
	struct mutex		als_data_lock;
	struct mutex		gs_data_lock;
	uint32_t		errinfo;
	uint32_t		errrecord;
	struct power_supply	*psy;
	int			err_monitor_enable;
	bool		cont_exec;
	bool		cal_check;
	unsigned char	cont_exec_cnt;
} APDS_DATA;

/* logical functions */
static int                  get_from_device(DEVICE_VAL *calc_data, struct i2c_client *client);
static void                 als_work_func(struct work_struct *work);
static enum hrtimer_restart als_timer_func(struct hrtimer *timer);
static int                  apds_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int                  apds_remove(struct i2c_client *client);
static void                 apds_shutdown(struct i2c_client *client);
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
int apds_init(void);
void apds_exit(void);
#else
static int __init apds_init(void);
static void __exit apds_exit(void);
#endif
/* access functions */
static int apds_driver_init(apds9960_dd_init_arg *data, struct i2c_client *client);
static int apds_driver_reset(struct i2c_client *client);
static int apds_i2c_read(const struct i2c_client *client, uint8_t reg, uint8_t *rbuf, int len);
static int apds_i2c_write(const struct i2c_client *client, uint8_t reg, const uint8_t *wbuf, int len);
static int apds_i2c_read_byte_data(const struct i2c_client *client,
                                      u8 command);
static int apds_i2c_write_byte_data(const struct i2c_client *client,
                                      u8 command, u8 value);
static int apds_i2c_write_i2c_block_data(const struct i2c_client *client, u8 command,
                                           u8 length, const u8 *values);
//static int apds_driver_power_on_off(POWERON_ARG data, struct i2c_client *client);
static int apds_driver_read_power_state(PWR_ST *pwr_st, struct i2c_client *client);
static int als_driver_read_data(READ_DATA_ALS_BUF *data, struct i2c_client *client);
static int apds_driver_general_read(GENREAD_ARG data, struct i2c_client *client);
static int apds_suspend(struct i2c_client *client, pm_message_t mesg);
static int apds_resume(struct i2c_client *client);
static void ps_sensor_report_event_proc(APDS_DATA *apds, uint32_t detect);

/**************************** variable declaration ****************************/
static struct workqueue_struct *monitor_workqueue;
static struct workqueue_struct *gs_ignore_detection_workqueue;

/**************************** structure declaration ****************************/
/* I2C device IDs supported by this driver */
static const struct i2c_device_id apds_id[] = {
    { APDS9960_DD_I2C_NAME, 0 },
    { }
};

/* represent an I2C device driver */
static struct i2c_driver i2c_driver_info = {
	.driver = {                      /* device driver model driver */
		.name = APDS9960_DD_I2C_NAME,
	},
	.probe    = apds_probe,        /* callback for device binding */
	.remove   = apds_remove,       /* callback for device unbinding */
	.shutdown = apds_shutdown,
	.id_table = apds_id,           /* list of I2C devices supported by this driver */
	.suspend  = apds_suspend,
	.resume   = apds_resume,
};

/* gain table */
#define GAIN_FACTOR (4)
static const struct GAIN_TABLE {
	unsigned char DATA0;
	unsigned char DATA1;
} GAIN_TABLE[GAIN_FACTOR] = {
	{  1,   1},   /*  0 */
	{  4,   0},   /*  1 */
	{ 16,   0},   /*  2 */
	{ 64,   0},   /*  3 */
};

#define ALS_DATA_MAX_FACTOR (64)
static const struct ALS_DATA_MAX_TABLE {
	unsigned char atime;
	unsigned short als_data_max;
} ALS_DATA_MAX_TABLE[ALS_DATA_MAX_FACTOR] = {
	{0xFF, 0x0401},
	{0xFE, 0x0801},
	{0xFD, 0x0C01},
	{0xFC, 0x1001},
	{0xFB, 0x1401},
	{0xFA, 0x1801},
	{0xF9, 0x1C01},
	{0xF8, 0x2001},
	{0xF7, 0x2401},
	{0xF6, 0x2801},
	{0xF5, 0x2C01},
	{0xF4, 0x3001},
	{0xF3, 0x3401},
	{0xF2, 0x3801},
	{0xF1, 0x3C01},
	{0xF0, 0x4001},
	{0xEF, 0x4401},
	{0xEE, 0x4801},
	{0xED, 0x4C01},
	{0xEC, 0x5001},
	{0xEB, 0x5401},
	{0xEA, 0x5801},
	{0xE9, 0x5C01},
	{0xE8, 0x6001},
	{0xE7, 0x6401},
	{0xE6, 0x6801},
	{0xE5, 0x6C01},
	{0xE4, 0x7001},
	{0xE3, 0x7401},
	{0xE2, 0x7801},
	{0xE1, 0x7C01},
	{0xE0, 0x8001},
	{0xDF, 0x8401},
	{0xDE, 0x8801},
	{0xDD, 0x8C01},
	{0xDC, 0x9001},
	{0xDB, 0x9401},
	{0xDA, 0x9801},
	{0xD9, 0x9C01},
	{0xD8, 0xA001},
	{0xD7, 0xA401},
	{0xD6, 0xA801},
	{0xD5, 0xAC01},
	{0xD4, 0xB001},
	{0xD3, 0xB401},
	{0xD2, 0xB801},
	{0xD1, 0xBC01},
	{0xD0, 0xC001},
	{0xCF, 0xC401},
	{0xCE, 0xC801},
	{0xCD, 0xCC01},
	{0xCC, 0xD001},
	{0xCB, 0xD401},
	{0xCA, 0xD801},
	{0xC9, 0xDC01},
	{0xC8, 0xE001},
	{0xC7, 0xE401},
	{0xC6, 0xE801},
	{0xC5, 0xEC01},
	{0xC4, 0xF001},
	{0xC3, 0xF401},
	{0xC2, 0xF801},
	{0xC1, 0xFC01},
	{0xC0, 0xFFFF},
};

#define NUM_OF_COLOR 5
#define ALS_EXP_OF_NV_TH 1000
#define ALS_EXP_OF_NV_COEF 10000
#define ALS_MAX_LUX_VAL 10000
#define ALS_MAX_DX_VAL 0x3A98
#define ALS_D0_D1_TH 0x0100

static u8 nv_proximity_detect			= 0xA0;
static u8 nv_proximity_no_detect		= 0x64;
static u8 nv_proximity_offset_ur		= 0x00;
static u8 nv_proximity_offset_dl		= 0x00;
static u8 nv_gesture_offset_ur			= 0x00;
static u8 nv_gesture_offset_dl			= 0x00;
static u8 nv_proximity_temp1			= 0x14;
static u8 nv_proximity_temp2			= 0x28;
static u8 nv_gesture_enter				= 0xA0;
static u8 nv_gesture_exit				= 0x40;
static u8 nv_gesture_offset_up			= 0x00;
static u8 nv_gesture_offset_down		= 0x00;
static u8 nv_gesture_offset_left		= 0x00;
static u8 nv_gesture_offset_right		= 0x00;

static u16 nv_gs_correction[] =
	{0x0001, 0x0001, 0x0001, 0x0001};

static u16 nv_photosensor_th0[NUM_OF_COLOR] =
	{0x019A, 0x019A, 0x019A, 0x019A, 0x019A};
static u16 nv_photosensor_th1[NUM_OF_COLOR] =
	{0x0258, 0x0258, 0x0258, 0x0258, 0x0258};
static u16 nv_photosensor_th2[NUM_OF_COLOR] =
	{0x0352, 0x0352, 0x0352, 0x0352, 0x0352};
static u16 nv_photosensor_th3[NUM_OF_COLOR] =
	{0x04B0, 0x04B0, 0x04B0, 0x04B0, 0x04B0};
static u16 nv_photosensor_th4[NUM_OF_COLOR] =
	{0x07D0, 0x07D0, 0x07D0, 0x07D0, 0x07D0};
static u16 nv_photosensor_a0[NUM_OF_COLOR] =
	{0x0ABE, 0x0ABE, 0x0ABE, 0x0ABE, 0x0ABE};
static u16 nv_photosensor_a1[NUM_OF_COLOR] =
	{0x0111, 0x0111, 0x0111, 0x0111, 0x0111};
static u16 nv_photosensor_a2[NUM_OF_COLOR] =
	{0x00F2, 0x00F2, 0x00F2, 0x00F2, 0x00F2};
static u16 nv_photosensor_a3[NUM_OF_COLOR] =
	{0x007D, 0x007D, 0x007D, 0x007D, 0x007D};
static u16 nv_photosensor_a4[NUM_OF_COLOR] =
	{0x007D, 0x007D, 0x007D, 0x007D, 0x007D};
static u16 nv_photosensor_b0[NUM_OF_COLOR] =
	{0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
static u16 nv_photosensor_b1[NUM_OF_COLOR] =
	{0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
static u16 nv_photosensor_b2[NUM_OF_COLOR] =
	{0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
static u16 nv_photosensor_b3[NUM_OF_COLOR] =
	{0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
static u16 nv_photosensor_b4[NUM_OF_COLOR] =
	{0x0000, 0x0000, 0x0000, 0x0000, 0x0000};


enum apds_power_ctrl_mode{
	APDS_POWER_CTRL_OFF = 0,
	APDS_POWER_CTRL_LOW,
	APDS_POWER_CTRL_NORMAL,
	APDS_POWER_CTRL_MAX
};
/************************************************************
 *                      logic function                      *
 ***********************************************************/
/******************************************************************************
 * NAME       : get_from_device
 * FUNCTION   : periodically reads the data from sensor(thread of work)
 * REMARKS    :
 *****************************************************************************/
static int get_from_device(DEVICE_VAL *dev_val, struct i2c_client *client)
{
#define LEDBIT_MASK    (3)
#define GAIN_VAL_MASK  (0x3)
	int           result;
	GENREAD_ARG   gene_read;
	unsigned char alsps_ctl, read_time;

	/* initialize the returning value */
	dev_val->time        = 0;
	dev_val->gain        = 0;
	dev_val->led_current = 0;

	/* get measure time parameter */
	gene_read.adr_reg = APDS9960_DD_ATIME_ADDR;
	gene_read.addr    = &read_time;
	gene_read.size    = sizeof(read_time);
	result = apds_driver_general_read(gene_read, client);
	if (result < 0) {
		SENSOR_ERR_LOG("ERROR! read data of itime.");
		return (result);
	}
	dev_val->time = read_time;

	/* get gain parameter */
	gene_read.adr_reg = APDS9960_DD_CONTROL_ADDR;
	gene_read.addr    = &alsps_ctl;
	gene_read.size    = sizeof(alsps_ctl);
	result = apds_driver_general_read(gene_read, client);
	if (result < 0) {
		SENSOR_ERR_LOG("ERROR! read data of gain.");
		return (result);
	}
	dev_val->gain        = alsps_ctl & GAIN_VAL_MASK;

	return (0);
#undef LEDBIT_MASK
#undef GAIN_VAL_MASK
}

static int als_get_coef(CALC_DATA *calc_data, APDS_DATA *apds)
{
	bool d1_large_val;

	d1_large_val = calc_data->d1 > ((256 - calc_data->atime) * 220) ? true : false;
	if (apds->cont_exec) {
		if (!d1_large_val) {
			apds->cont_exec_cnt++;
			if (apds->cont_exec_cnt == 10)
				apds->cont_exec = false;
		} else {
			apds->cont_exec_cnt = 0;
		}
	}

	if (apds->cont_exec) {
		calc_data->src = LIGHT_SOLAR;
		calc_data->alpha = nv_photosensor_a3[apds->color];
	} else if (calc_data->ratio >= nv_photosensor_th3[apds->color]) {
		if(calc_data->als_data0 <= ALS_D0_D1_TH && calc_data->als_data1 <= ALS_D0_D1_TH) {
			calc_data->src = LIGHT_INTERMEDIATE;
			calc_data->alpha = nv_photosensor_a0[apds->color];
			calc_data->beta = nv_photosensor_b0[apds->color];
		}
	} else if (d1_large_val) {
		calc_data->src = LIGHT_SOLAR;
		calc_data->alpha = nv_photosensor_a3[apds->color];
		apds->cont_exec = true;
		apds->cont_exec_cnt = 0;
	} else {
		if (calc_data->ratio < nv_photosensor_th0[apds->color]) {
			calc_data->src = LIGHT_FLUORESCENT;
			calc_data->alpha = nv_photosensor_a0[apds->color];
			calc_data->beta = nv_photosensor_b0[apds->color];
		} else if ((calc_data->ratio >= nv_photosensor_th0[apds->color]) &&
			   (calc_data->ratio < nv_photosensor_th1[apds->color])) {
			calc_data->src = LIGHT_LED_BULB;
			calc_data->alpha = nv_photosensor_a1[apds->color];
			calc_data->beta = nv_photosensor_b1[apds->color];
		} else if (calc_data->ratio >= nv_photosensor_th1[apds->color]) {
			calc_data->src = LIGHT_CANDESCENT;
			calc_data->alpha = nv_photosensor_a2[apds->color];
			calc_data->beta = nv_photosensor_b2[apds->color];
		} else {
			calc_data->src = LIGHT_SRC_MAX;
			SENSOR_ERR_LOG("als_get_coef Unexpected.");
			return -1;
		}
	}
	return 0;
}

static int als_calculate_data(READ_DATA_ALS_BUF data, APDS_DATA *apds)
{
	int *i;
	int j, k;
	int prev;
	CALC_DATA *calc_data;
	CALC_DATA *calc_data_h;
	CALC_DATA calc_data_tmp;
	DEVICE_VAL *dev_val;
	int result;
	unsigned long temp = 0;
	unsigned long max_tmp = 0;
	unsigned long lux_tmp = 0;
	unsigned long alsit;
	unsigned char atime_max_tmp;
	unsigned short atime_data_max;

	i = &apds->als_val.index;
	calc_data = &apds->als_val.calc_data[*i];
	calc_data_h = apds->als_val.calc_data;
	dev_val = &apds->dev_val;

	/* set the value of measured als data */
	calc_data->als_data0  = data.als_data0;
	calc_data->als_data1  = data.als_data1;
	calc_data->atime      = data.atime;
	calc_data->gain_data0 = GAIN_TABLE[dev_val->gain].DATA0;

	/* set max range */
	if (calc_data->gain_data0 == 0) {
		/* issue error value when gain is 0 */
		SENSOR_ERR_LOG("gain value error");
		return -1;
	}

	calc_data->d0 = calc_data->als_data0;
	calc_data->d1 = calc_data->als_data1;
	if (!calc_data->d0)
		calc_data->ratio = 1 * ALS_EXP_OF_NV_TH;
	else
		calc_data->ratio =
			(calc_data->d1 * ALS_EXP_OF_NV_TH) / calc_data->d0;

	result = als_get_coef(calc_data, apds);

	alsit = (256 - calc_data->atime) * 2780;
	temp = calc_data->d0 * calc_data->alpha / (calc_data->gain_data0 * (alsit / ALS_EXP_OF_NV_TH));
	calc_data->lux = temp;

	atime_max_tmp = 0xFF - calc_data->atime;
	if (!(atime_max_tmp < ALS_DATA_MAX_FACTOR))
		atime_max_tmp = (ALS_DATA_MAX_FACTOR - 1);
	atime_data_max = ALS_DATA_MAX_TABLE[atime_max_tmp].als_data_max;
	if (calc_data->d0 > atime_data_max || calc_data->d1 > atime_data_max){
		prev = *i - 1;
		if (prev < 0)
			prev = NUM_OF_ALS_VAL - 1;
		calc_data->lux = apds->als_val.calc_data[prev].lux;
	}

	if (!calc_data->d0)
		calc_data->lux = 0;

	SENSOR_D_LOG("lux=%lu(%lu) d0=%lu d1=%lu g0=%u atime=%u als_max=%u ratio=%lu src=%d a=%d alsit=%lu",
		calc_data->lux, temp, calc_data->d0, calc_data->d1,
		calc_data->gain_data0, calc_data->atime,
		atime_data_max,
		calc_data->ratio, calc_data->src,
		calc_data->alpha, alsit);

	if (apds->als_val.ave_enable) {
		temp = 0;
		for (j = 0; j < NUM_OF_ALS_VAL; j++) {
			k = (*i + j + 1) % NUM_OF_ALS_VAL;
			temp += calc_data_h[k].lux * (j + 1);
			if (max_tmp < calc_data_h[k].lux) {
				max_tmp = calc_data_h[k].lux;
			}
		}
		apds->als_val.report_lux = temp / 10;
		lux_tmp = apds->als_val.report_lux * 3;
		if (calc_data->src != LIGHT_SOLAR) {
			if((max_tmp / 2) > lux_tmp) {
				apds->als_val.report_lux = (lux_tmp * 15) / 10;
			} else if(max_tmp <= lux_tmp) {
				apds->als_val.report_lux = max_tmp;
			} else {
				apds->als_val.report_lux = lux_tmp;
			}
		}
	} else {
		apds->als_val.report_lux = calc_data_h[0].lux;
	}

	if (apds->als_val.report_lux > ALS_MAX_LUX_VAL)
		apds->als_val.report_lux = ALS_MAX_LUX_VAL;

	prev = *i - 1;
	if (prev < 0)
		prev = NUM_OF_ALS_VAL - 1;
	if (apds->als_val.report_lux <= 50 || apds->als_val.calc_data[prev].src != calc_data->src) {
		memcpy(&calc_data_tmp, calc_data, sizeof(CALC_DATA));
		apds->als_val.index = 0;
		apds->als_val.ave_enable = false;
		memset(apds->als_val.calc_data, 0,sizeof(CALC_DATA) * NUM_OF_ALS_VAL);
		memcpy(apds->als_val.calc_data, &calc_data_tmp, sizeof(CALC_DATA));
	}

	apds->als_val.index++;
	apds->als_val.index %= NUM_OF_ALS_VAL;
	if ((!apds->als_val.ave_enable) &&
	    (apds->als_val.index == (NUM_OF_ALS_VAL - 1)))
		apds->als_val.ave_enable = true;

	return 0;
}

/******************************************************************************
 * NAME       : als_driver_read_data
 * FUNCTION   : read the value of ALS data and status in RPR0521
 * REMARKS    :
 *****************************************************************************/
static int als_driver_read_data(READ_DATA_ALS_BUF *data, struct i2c_client *client)
{
	int           result;
	READ_DATA_ALS_BUF multi;
	GENREAD_ARG   gene_read;
	APDS_DATA *apds = i2c_get_clientdata(client);

	/* read start address */
	gene_read.adr_reg = APDS9960_DD_CDATAL_ADDR;
	gene_read.addr    = (char *)&multi;
	gene_read.size    = sizeof(multi);

	/* block read */
	result = apds_driver_general_read(gene_read, client);
	if (result > 0) {
		data->als_data0 = CONVERT_TO_BE(multi.als_data0);
		data->als_data1 = CONVERT_TO_BE(multi.als_data1);
		data->atime = apds_i2c_read_byte_data(apds->client, APDS9960_DD_ATIME_ADDR);
		result          = 0;
	} else {
		data->als_data0 = 0;
		data->als_data1 = 0;
		data->atime = 0;
        SENSOR_ERR_LOG("error apds_driver_general_read :%d",result);
	}
	if (unlikely(apds->imit.imit_flg)) {
		data->als_data0 = apds->imit.imit_d0;
		data->als_data1 = apds->imit.imit_d1;
		data->atime = 0;
	}

	SENSOR_D_LOG("data->als_data0=%d, data->als_data1=%d", data->als_data0, data->als_data1);
	return (result);
}

/******************************************************************************
 * NAME       : ps_driver_read_data
 * FUNCTION   : read the value of PS data and status in RPR0521
 * REMARKS    :
 *****************************************************************************/
static int ps_driver_read_data(struct i2c_client *client)
{
	int           result;
	APDS_DATA *apds = i2c_get_clientdata(client);
	char data;
	GENREAD_ARG   gene_read;

	/* read start address */
	gene_read.adr_reg = APDS9960_DD_PDATA_ADDR;
	gene_read.addr    = &data;
	gene_read.size    = sizeof(data);

	/* block read */
	result = apds_driver_general_read(gene_read, client);
	if (likely(result > 0)) {
		apds->ps_val.ps_ctrl = 0;
		apds->ps_val.ps_data = data;
		apds->ps_val.ps_flag = 0;
		result = 0;
	} else {
		apds->ps_val.ps_ctrl = 0;
		apds->ps_val.ps_data = 0;
		apds->ps_val.ps_flag = 0;
        SENSOR_ERR_LOG("error apds_driver_general_read :%d",result);
	}

	return (result);
}
static void gs_data_correction(struct i2c_client *client, unsigned char *gfifo)
{
	int i;

	for (i = 0; i < 4; i++) {
		if (gfifo[i] * nv_gs_correction[i] > APDS9960_DD_GFIFO_DATA_MAX)
			gfifo[i] = APDS9960_DD_GFIFO_DATA_MAX;
		else
			gfifo[i] = gfifo[i] * nv_gs_correction[i];
	}
}
static int gs_driver_clear_fifo(struct i2c_client *client)
{
	int result;

	result = apds_i2c_read_byte_data(client, APDS9960_DD_GCONF4_ADDR);
	if (result < 0) {
		SENSOR_ERR_LOG("ERROR! read data");
		return result;
	} else {
		apds_i2c_write_byte_data(client, APDS9960_DD_GCONF4_ADDR,
			(result | APDS9960_DD_GCONF4_FIFO_CLR));
	}

	return result;
}
static int gs_driver_read_data(struct i2c_client *client)
{
	int           result;
	APDS_DATA *apds = i2c_get_clientdata(client);
	char gstatus[2];
	unsigned char gfifo[APDS9960_DD_GFIFO_MAX];
	int gfifo_read_size;
	int i;
	bool fifo_read_done = false;
	GENREAD_ARG   gene_read_status;
	GENREAD_ARG   gene_read_fifo;

	/* read start address */
	gene_read_status.adr_reg = APDS9960_DD_GFIFO_LVL_ADDR;
	gene_read_status.addr    = gstatus;
	gene_read_status.size    = sizeof(gstatus);

	/* block read */
	result = apds_driver_general_read(gene_read_status, client);
	if (likely(result > 0)) {
		if (gstatus[1] & APDS9960_DD_GSTATUS_GFOV_MASK) {
			SENSOR_ERR_LOG("Gesture FIFO overflow.");
			result = gs_driver_clear_fifo(client);
			apds->gs_val.gs_lvl = 0;
			apds->gs_val.gs_data[0] = 0;
			return (result);
		}
		gfifo_read_size = gstatus[0]*4;
		if (gfifo_read_size > APDS9960_DD_GFIFO_MAX) {
			SENSOR_ERR_LOG("Gesture FIFO greater than buffer.");
			gfifo_read_size = APDS9960_DD_GFIFO_MAX;
		}
		if (gfifo_read_size == 0) {
			SENSOR_ERR_LOG("Gesture FIFO is empty.");
		}
		gene_read_fifo.adr_reg = APDS9960_DD_GFIFO0_ADDR;
		gene_read_fifo.addr    = gfifo;
		gene_read_fifo.size    = gfifo_read_size;
		result = apds_driver_general_read(gene_read_fifo, client);
		if (likely(result > 0)) {
			SENSOR_D_LOG("FIFO size:%d", gfifo_read_size);
			for (i = 0; i < (gfifo_read_size / 4); i++) {
				gs_data_correction(client, &gfifo[i*4]);
				SENSOR_D_LOG("[%02X %02X %02X %02X]",
					gfifo[i],
					gfifo[i+1],
					gfifo[i+2],
					gfifo[i+3]);
			}
			memcpy(apds->gs_val.gs_data, gfifo, gfifo_read_size);
			apds->gs_val.gs_lvl = gstatus[0];
			fifo_read_done = true;
		} else {
			apds->gs_val.gs_lvl = 0;
			apds->gs_val.gs_data[0] = 0;
		}
	} else {
		apds->gs_val.gs_lvl = 0;
		apds->gs_val.gs_data[0] = 0;
	}

	if (fifo_read_done) {
		result = apds_i2c_read_byte_data(client, APDS9960_DD_GCONF4_ADDR);
		if (result > 0) {
			apds->gs_val.gs_gmode = (result & APDS9960_DD_GMODE);
		}
	}

	return (result);
}
static int gs_check_status(APDS_DATA *apds)
{
	if (apds->gs_val.gs_lvl)
		return 1;
	else
		return 0;
}

static u8 apds_offset_add_correct(u8 cur, int offset)
{
	int tmp;
	u8 new_offset;

	if(cur & 0x80)
		tmp = offset - (cur & 0x7F);
	else
		tmp = offset + cur;

	if(tmp > 127)
		tmp = 127;

	if(tmp < 0)
		new_offset = (u8)abs(tmp) | 0x80;
	else
		new_offset = (u8)tmp;

	SENSOR_D_LOG("cur %02x, offset %d, new %02x", cur, offset, new_offset);

	return new_offset;
}

static int gs_offset_write(APDS_DATA *apds)
{
	int rc = 0;
	u8 value;

	value = apds_offset_add_correct(nv_gesture_offset_up, apds->cal_offset_up);
	rc  = apds_i2c_write_byte_data(apds->client, APDS9960_DD_GOFFSET_U_ADDR, value);
	apds->init_data.dd_goffset_u = value;

	value = apds_offset_add_correct(nv_gesture_offset_down, apds->cal_offset_down);
	rc |= apds_i2c_write_byte_data(apds->client, APDS9960_DD_GOFFSET_D_ADDR, value);
	apds->init_data.dd_goffset_d = value;

	value = apds_offset_add_correct(nv_gesture_offset_left, apds->cal_offset_left);
	rc |= apds_i2c_write_byte_data(apds->client, APDS9960_DD_GOFFSET_L_ADDR, value);
	apds->init_data.dd_goffset_l = value;

	value = apds_offset_add_correct(nv_gesture_offset_right, apds->cal_offset_right);
	rc |= apds_i2c_write_byte_data(apds->client, APDS9960_DD_GOFFSET_R_ADDR, value);
	apds->init_data.dd_goffset_r = value;

	if(rc)
		SENSOR_ERR_LOG("I2C Write Error");

	return rc;
}

static int gs_sensor_offset_correct(APDS_DATA *apds, int udlr, int orient, int zero_cnt)
{
	if(zero_cnt >= 3){
		SENSOR_ERR_LOG("%d: correct1 -3", udlr);
		return -3;
	} else if(orient >= 21 * GS_CAL_CNT){
		orient = orient/(8 * GS_CAL_CNT) -2;
		if(orient)
			SENSOR_ERR_LOG("%d: correct2.1 +%d", udlr, orient);
		else {
			orient = 1;
			SENSOR_ERR_LOG("%d: correct2.2 +%d", udlr, orient);
		}
		return orient;
	} else if(orient >= 16 * GS_CAL_CNT){
		SENSOR_ERR_LOG("%d: correct3 +1", udlr);
		return 1;
	} else {
		SENSOR_ERR_LOG("%d: correct4 0", udlr);
		return 0;
	}
}

static void ps_offset_write(APDS_DATA *apds)
{
	int rc = 0;
	u8 value;

	value = apds_offset_add_correct(nv_gesture_offset_ur, apds->ps_offset_ur);
	rc  = apds_i2c_write_byte_data(apds->client, APDS9960_DD_POFFSET_UR_ADDR , value);
	apds->init_data.dd_poffset_ur = value;

	value = apds_offset_add_correct(nv_gesture_offset_dl, apds->ps_offset_dl);
	rc |= apds_i2c_write_byte_data(apds->client, APDS9960_DD_POFFSET_DL_ADDR , value);
	apds->init_data.dd_poffset_dl = value;

	if(rc)
		SENSOR_ERR_LOG("I2C Write Error");

	return;
}

static void ps_sensor_cal_check(APDS_DATA *apds)
{
	int rc;
	int loop = 0;
	bool first = true;

	SENSOR_D_LOG("start");

	rc  = apds_i2c_write_byte_data(apds->client, APDS9960_DD_GTHR_IN_ADDR, 0xFF);
	rc |= apds_i2c_write_byte_data(apds->client, APDS9960_DD_GTHR_OUT_ADDR, 0xFF);
	if(rc){
		SENSOR_ERR_LOG("I2C write Error");
		return;
	}

	apds->ps_offset_ur = 0;
	apds->ps_offset_dl = 0;
	ps_offset_write(apds);
	msleep(80);

	rc = ps_driver_read_data(apds->client);
	if(rc){
		SENSOR_ERR_LOG("I2C read Error");
		return;
	}

	while(loop < 10){
		if(apds->ps_val.ps_data >= 5 && apds->ps_val.ps_data <= 15){
			SENSOR_ERR_LOG("ps CAL Done: data = %d", apds->ps_val.ps_data);
			break;
		} else if(apds->ps_val.ps_data == 0)
			if(first){
				SENSOR_ERR_LOG("First time skip: data = %d", apds->ps_val.ps_data);
			} else {
				SENSOR_ERR_LOG("offset -6: data = %d", apds->ps_val.ps_data);
				apds->ps_offset_ur -= 6;
				apds->ps_offset_dl -= 6;
		} else if(apds->ps_val.ps_data <= 24){
			SENSOR_ERR_LOG("offset +1: data = %d", apds->ps_val.ps_data);
			apds->ps_offset_ur++;
			apds->ps_offset_dl++;
		} else if(apds->ps_val.ps_data >= 25){
			SENSOR_ERR_LOG("offset +%d: data = %d",
					apds->ps_val.ps_data / 8 - 2, apds->ps_val.ps_data);
			apds->ps_offset_ur += apds->ps_val.ps_data / 8 - 2;
			apds->ps_offset_dl += apds->ps_val.ps_data / 8 - 2;
		}
		first = false;
		ps_offset_write(apds);
		loop++;
		msleep(80);
		rc |= ps_driver_read_data(apds->client);
		if(rc){
			SENSOR_ERR_LOG("I2C Read Error");
			apds->ps_offset_ur = 0;
			apds->ps_offset_dl = 0;
			ps_offset_write(apds);
			return;
		}
	}

	SENSOR_D_LOG("end");
}

static void gs_sensor_cal_check(APDS_DATA *apds)
{
	int i, rc, fifo, cal_fifo;
	int up = 0;
	int down = 0;
	int left = 0;
	int right = 0;
	int up_zero = 0;
	int down_zero = 0;
	int left_zero = 0;
	int right_zero = 0;
	int cal_err = 0;
	static int loop = 0;

	SENSOR_D_LOG("start");

	for (i = 0; i < apds->gs_val.gs_lvl ; i++) {
		fifo = i * 4;
		cal_fifo = apds->gs_cal_cnt * 4 + i * 4;
		apds->gs_cal_data[cal_fifo+0] = apds->gs_val.gs_data[fifo+0];
		apds->gs_cal_data[cal_fifo+1] = apds->gs_val.gs_data[fifo+1];
		apds->gs_cal_data[cal_fifo+2] = apds->gs_val.gs_data[fifo+2];
		apds->gs_cal_data[cal_fifo+3] = apds->gs_val.gs_data[fifo+3];
		apds->gs_cal_cnt++;
	}

	if(apds->gs_cal_cnt >= GS_CAL_CNT){
		for (i = 0; i < GS_CAL_CNT; i++) {
			fifo = i * 4;
			up    += apds->gs_cal_data[fifo+0];
			down  += apds->gs_cal_data[fifo+1];
			left  += apds->gs_cal_data[fifo+2];
			right += apds->gs_cal_data[fifo+3];
			SENSOR_D_LOG("%02d: udlr %03d %03d %03d %03d", i,
				apds->gs_cal_data[fifo+0], apds->gs_cal_data[fifo+1],
				apds->gs_cal_data[fifo+2], apds->gs_cal_data[fifo+3]);
			if(!apds->gs_cal_data[fifo+0])
				up_zero++;
			if(!apds->gs_cal_data[fifo+1])
				down_zero++;
			if(!apds->gs_cal_data[fifo+2])
				left_zero++;
			if(!apds->gs_cal_data[fifo+3])
				right_zero++;
			if( apds->gs_cal_data[fifo+0] > GS_MAX_OFFSET ||
				apds->gs_cal_data[fifo+1] > GS_MAX_OFFSET ||
				apds->gs_cal_data[fifo+2] > GS_MAX_OFFSET ||
				apds->gs_cal_data[fifo+3] > GS_MAX_OFFSET){
				cal_err = 1;
				break;
			}
		}

		if(cal_err){
			SENSOR_ERR_LOG("CAL ERR: over data");
			gs_driver_read_data(apds->client);
			apds->cal_check = false;
			apds->gs_cal_cnt = 0;
			loop = 0;
			return;
		}

		if((up   < 16*GS_CAL_CNT && down  < 16*GS_CAL_CNT &&
			left < 16*GS_CAL_CNT && right < 16*GS_CAL_CNT &&
			up_zero <= 2 && down_zero <= 2 && left_zero <= 2 && right_zero <= 2)
			||  loop >= 10){
			SENSOR_ERR_LOG("CAL complete. loop %d", loop);
			ps_sensor_cal_check(apds);
			rc  = apds_i2c_write_byte_data(apds->client, APDS9960_DD_GTHR_IN_ADDR, nv_gesture_enter);
			rc |= apds_i2c_write_byte_data(apds->client, APDS9960_DD_GTHR_OUT_ADDR, nv_gesture_exit);
			if(rc)
				SENSOR_ERR_LOG("I2C Write Error");
			gs_driver_read_data(apds->client);
			apds->cal_check = false;
			apds->gs_cal_cnt = 0;
			loop = 0;
			return;
		}

		apds->cal_offset_up += gs_sensor_offset_correct(apds, 1, up, up_zero);
		apds->cal_offset_down  += gs_sensor_offset_correct(apds, 2, down, down_zero);
		apds->cal_offset_left  += gs_sensor_offset_correct(apds, 3, left, left_zero);
		apds->cal_offset_right += gs_sensor_offset_correct(apds, 4, right, right_zero);
		rc = gs_offset_write(apds);
		if(rc)
			SENSOR_ERR_LOG("I2C Write Error");
		apds->gs_cal_cnt = 0;
		loop++;
		SENSOR_ERR_LOG("CAL continue");
	}
	SENSOR_D_LOG("end");
}

static void gs_sensor_report_event_proc(APDS_DATA *apds)
{
	struct ps_sensor_info *info = &apds->ps_info;
	int i;
	int fifo;

	SENSOR_D_LOG("start");

	if (unlikely(!info)) {
		SENSOR_ERR_LOG("fail bad parm --> info");
		return;
	}

	if (!atomic_read(&info->valid)) {
		SENSOR_ERR_LOG("gs sensor invalid. skip.");
		return;
	}

	if(!apds->cal_check){
		if(!apds->is_gs_ignr_enable){
			for (i = 0; i < apds->gs_val.gs_lvl ; i++) {
				fifo = i * 4;
				sensor_report_data(SENSOR_GESTURE,(
					apds->gs_val.gs_data[fifo+0] <<  0 |
					apds->gs_val.gs_data[fifo+1] <<  8 |
					apds->gs_val.gs_data[fifo+2] << 16 |
					apds->gs_val.gs_data[fifo+3] << 24));
			}
			if (!apds->gs_val.gs_gmode) {
				apds->tch_reject = false;
				sensor_interrupt(SENSOR_GESTURE, GESTURE_STATUS_GMODE_EXIT);
			} else
				apds->tch_reject = true;
		} else if (apds->tch_reject) {
			SENSOR_ERR_LOG("Stop GS by Touch");
			apds->tch_reject = false;
			sensor_interrupt(SENSOR_GESTURE, GESTURE_STATUS_GMODE_EXIT);
		}
	} else {
		gs_sensor_cal_check(apds);
	}
	SENSOR_D_LOG("end");
}

/******************************************************************************
 * NAME       : als_work_func
 * FUNCTION   : periodically reads the data from sensor(thread of work)
 * REMARKS    :
 *****************************************************************************/
static void als_work_func(struct work_struct *als_work)
{
	int           result;
	READ_DATA_ALS_BUF read_data_buf;
	APDS_DATA   *apds;
	long          get_timer;
	long          wait_sec;
	unsigned long wait_nsec;
	PWR_ST        pwr_st;

	SENSOR_D_LOG("start");
	read_data_buf.als_data0 = 0;
	read_data_buf.als_data1 = 0;
	apds = container_of(als_work, APDS_DATA, als_work);

	/* read the state of sensor */
	result = apds_driver_read_power_state(&pwr_st, apds->client);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("power_state error ");
		goto exit;
	}
	/* check the state of sensor */
	if (unlikely(pwr_st.als_state == CTL_STANDBY)) {
		goto exit;
	}

	mutex_lock(&apds->als_data_lock);

	result = als_driver_read_data(&read_data_buf, apds->client);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("ERROR! read data");
		goto exit_lock;
	}
	/* read value from device */
	result = get_from_device(&apds->dev_val, apds->client);
	if (result < 0) {
		SENSOR_ERR_LOG("ERROR! read data from device.");
		goto exit_lock;
	}
	result = als_calculate_data(read_data_buf, apds);
	if (result) {
		SENSOR_ERR_LOG("ERROR! calculate als data.");
		goto exit_lock;
	}
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
	sensor_report_data(SENSOR_LIGHT, apds->als_val.report_lux);
#else
	als_sensor_report_event(&apds->als_info, apds->als_val.report_lux, apds->als_en_first);
#endif	/*CONFIG_USE_MICON_SOFT_STRUCTURE*/
	if (unlikely(apds->als_en_first))
		apds->als_en_first = false;


	/* the setting value from application */
	get_timer = apds->delay_time;
	/* 125ms(8Hz) at least */
	wait_sec  = (get_timer / SM_TIME_UNIT);
	wait_nsec = ((get_timer - (wait_sec * SM_TIME_UNIT)) * MN_TIME_UNIT);
	result = hrtimer_start(&apds->timer, ktime_set(wait_sec, wait_nsec), HRTIMER_MODE_REL);
	if (result != 0) {
		SENSOR_ERR_LOG("can't start timer");
		goto exit_lock;
	}
exit_lock:
	mutex_unlock(&apds->als_data_lock);
exit:
	SENSOR_D_LOG("end");
	return;
}

/******************************************************************************
 * NAME       : als_timer_func
 * FUNCTION   : call work function (thread of timer)
 * REMARKS    :
 *****************************************************************************/
static enum hrtimer_restart als_timer_func(struct hrtimer *timer)
{
    APDS_DATA *apds;
    int         result;

    apds = container_of(timer, APDS_DATA, timer);
    result = queue_work(monitor_workqueue, &apds->als_work);
    if (result == 0) {
        SENSOR_ERR_LOG("can't register que.");
        SENSOR_ERR_LOG("result = 0x%x", result);
    }

    return (HRTIMER_NORESTART);
}

static int ps_control_interrupt(APDS_DATA *apds, char mode)
{
	int result;
	char int_l = 0x00;
	char int_h = 0xFF;

	if (mode == PROX_STATUS_NEAR) {
		int_l = apds->init_data.dd_pitlo;
	} else {
		int_h = apds->init_data.dd_pithi;
	}
	result = apds_i2c_write(apds->client, APDS9960_DD_PILT_ADDR, &int_l, 1);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("Failed to change interrupt");
		return result;
	}
	result = apds_i2c_write(apds->client, APDS9960_DD_PIHT_ADDR, &int_h, 1);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("Failed to change interrupt");
		return result;
	}

	SENSOR_D_LOG("interrupt setting changed. pintl:0x%x pinth:0x%x",
		int_l,
		int_h);

	return 0;
}

static int ps_check_status(APDS_DATA *apds)
{
	int tmp;

	if (apds->ps_val.ps_data > apds->init_data.dd_pithi)
		tmp = PROX_STATUS_NEAR;
	else if (apds->ps_val.ps_data < apds->init_data.dd_pitlo)
		tmp = PROX_STATUS_FAR;
	else
		return 0;

	if (apds->ps_det != tmp) {
		apds->ps_det = tmp;
		return 1;
	} else {
		return 0;
	}
}

static void ps_sensor_report_event_proc(APDS_DATA *apds, uint32_t detect)
{
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
	long wake_lock_time;
	struct ps_sensor_info *info = &apds->ps_info;
	SENSOR_D_LOG("start [%d]", detect);

	if (unlikely(!info)) {
		SENSOR_ERR_LOG("fail bad parm --> info");
		return;
	}

	if (!atomic_read(&info->valid)) {
		SENSOR_ERR_LOG("ps sensor invalid. skip.");
		return;
	}

	sensor_interrupt(SENSOR_PROX, detect);
	wake_lock_time = detect ? WAKE_LOCK_TIME_DETECT : WAKE_LOCK_TIME_NODETECT;
	wake_lock_timeout(&apds->ps_info.wake_lock, wake_lock_time);
#else
	SENSOR_D_LOG("start");
	ps_sensor_report_event(&apds->ps_info, detect);
#endif	/*CONFIG_USE_MICON_SOFT_STRUCTURE*/
	SENSOR_D_LOG("end");
}

/******************************************************************************
 * NAME       : apds_irq_handler
 * FUNCTION   : interruption function (irq)
 * REMARKS    :
 *****************************************************************************/
static irqreturn_t apds_irq_handler(int irq, void *dev_id)
{
	APDS_DATA *apds;
	int         result;
	PWR_ST        pwr_st;
	GENREAD_ARG   gene_data;
	unsigned char ps_intr;
	char pint_clear = 0;

	SENSOR_D_LOG("start");
	apds = dev_id;

	/* read the int state of sensor */
	gene_data.adr_reg = APDS9960_DD_STATUS_ADDR;
	gene_data.addr    = &ps_intr;
	gene_data.size    = sizeof(ps_intr);
	result = apds_driver_general_read(gene_data, apds->client);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("general read can't execute ");
		SENSOR_ERR_LOG("can't read interrupt register ");
		goto exit;
	}

	/* clear interrupt flag */
	if (ps_intr & APDS9960_DD_PINT_STATUS)
		apds_i2c_write(apds->client, APDS9960_DD_CMD_CLR_PS_INT, &pint_clear, 1);

	/* read the power state of sensor */
	result = apds_driver_read_power_state(&pwr_st, apds->client);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("power_state error ");
		goto exit;
	}
	/* check the state of sensor */
	if (unlikely(pwr_st.ps_state == CTL_STANDBY && pwr_st.gs_state == CTL_STANDBY)) {
		gs_driver_clear_fifo(apds->client);
		goto exit;
	}

	mutex_lock(&apds->ps_data_lock);
	apds->ps_val.ps_intr = ps_intr;

	if (apds->ps_val.ps_intr & APDS9960_DD_PINT_STATUS) {
		result = ps_driver_read_data(apds->client);
		if (unlikely(result < 0)) {
			SENSOR_ERR_LOG("ERROR! read data");
			goto exit_lock;
		}
		if (pwr_st.ps_state == CTL_STANDALONE){
			if ((ps_check_status(apds) || !apds->int_flg)) {
				ps_sensor_report_event_proc(apds, apds->ps_det);
			}
			result = ps_control_interrupt(apds, (char)apds->ps_det);
			if (unlikely(result)) {
				SENSOR_ERR_LOG("Failed to change mode[%d]", result);
				goto exit_lock;
			}
		}
	}

	if (apds->ps_val.ps_intr & APDS9960_DD_GINT_STATUS) {
		result = gs_driver_read_data(apds->client);
		if (unlikely(result < 0)) {
			SENSOR_ERR_LOG("ERROR! read data");
			goto exit_lock;
		}
		if (gs_check_status(apds)) {
			gs_sensor_report_event_proc(apds);
		}
	}
	SENSOR_N_LOG("ps_intr=0x%x ps_ctrl=0x%x ps_data=%d ps_flag=%d psth=%d pstl=%d",
							apds->ps_val.ps_intr,
							apds->ps_val.ps_ctrl,
							apds->ps_val.ps_data,
							apds->ps_val.ps_flag,
							apds->init_data.dd_pithi,
							apds->init_data.dd_pitlo
	);


exit_lock:
	mutex_unlock(&apds->ps_data_lock);
exit:
	if (apds->ps_val.ps_intr & APDS9960_DD_PINT_STATUS)
		apds->int_flg = true;
	SENSOR_D_LOG("end");
	return (IRQ_HANDLED);
}

static void apds_set_errinfo(APDS_DATA *apds, uint32_t errinfo)
{
	static DEFINE_MUTEX(errinfo_write_lock);
	SENSOR_ERR_LOG("start errinfo[0x%x] apds->errinfo[0x%x]", errinfo, apds->errinfo);

	mutex_lock(&errinfo_write_lock);
	SENSOR_N_LOG("some errors occured. errinfo[0x%x]", errinfo);
	apds->errinfo |= errinfo;
	apds->errrecord |= errinfo;
	mutex_unlock(&errinfo_write_lock);

	if (errinfo & ERRINFO_DEV_RECOVERY_FAIL_MASK) {
		apds->ps_det = PROX_STATUS_FAR;
		ps_sensor_report_event_proc(apds, apds->ps_det);
	}

	SENSOR_V_LOG("end");
}
static void apds_clear_errinfo(APDS_DATA *apds, uint32_t errinfo)
{
	static DEFINE_MUTEX(errinfo_write_lock);
	SENSOR_V_LOG("start errinfo[0x%x] apds->errinfo[0x%x]", errinfo, apds->errinfo);

	mutex_lock(&errinfo_write_lock);
	apds->errinfo &= ~errinfo;
	mutex_unlock(&errinfo_write_lock);

	SENSOR_V_LOG("end");
}


static bool apds_is_device_dead(APDS_DATA *apds)
{
	return ((apds->errinfo & ERRINFO_DEV_RECOVERY_FAIL_MASK) != 0) ? true : false;
}
static bool apds_is_i2c_dead(APDS_DATA *apds)
{
	return ((apds->errinfo & ERRINFO_I2C_RECOVERY_FAIL_MASK) != 0) ? true : false;
}

static unsigned char get_enable_state(uint32_t enable_sensor)
{
	unsigned char	work = 0;

	if (enable_sensor & ENABLE_ALS)
		work |= APDS9960_DD_ENABLE_ALS;
	if (enable_sensor & ENABLE_PS)
		work |= (APDS9960_DD_ENABLE_PS | APDS9960_DD_ENABLE_PS_INT);
	if (enable_sensor & ENABLE_GS)
		work |= (APDS9960_DD_ENABLE_GS | APDS9960_DD_ENABLE_PS);
	if (work > 0)
		work |= APDS9960_DD_ENABLE_POWER;

	return work;
}

static uint32_t get_enable_sensor(unsigned char als_enable, unsigned char ps_enable, unsigned char gs_enable)
{
	uint32_t enable_sensor = 0;

	if (als_enable) enable_sensor |= ENABLE_ALS;
	if (ps_enable)  enable_sensor |= ENABLE_PS;
	if (gs_enable)  enable_sensor |= ENABLE_GS;

	return enable_sensor;
}

static int apds_confirm_regs(APDS_DATA *apds)
{
	int result;
	apds9960_dd_init_arg arg;
	uint32_t enable_sensor;
	unsigned char	work;

	SENSOR_V_LOG("start");

	result = apds_i2c_read(apds->client, APDS9960_DD_CONFIRM_START_REG,
		&arg.dd_enable, APDS9960_DD_CONFIRM_REGS_SIZE);
	if (result < 0) {
		SENSOR_ERR_LOG("i2c transfer failed.");
		return -1;
	}

	enable_sensor = get_enable_sensor(
						apds->power_data.power_als,
						apds->power_data.power_ps,
						apds->power_data.power_gs);
	work = get_enable_state(enable_sensor);
	if (arg.dd_enable != work) {
		SENSOR_ERR_LOG("unexpected dd_enable[0x%x]. expect[0x%x]", arg.dd_enable, work);
		return -1;
	}

	if (arg.dd_atime != apds->init_data.dd_atime) {
		SENSOR_ERR_LOG("unexpected dd_atime[0x%x]. expect[0x%x]", arg.dd_atime, apds->init_data.dd_atime);
		return -1;
	}
	if (arg.dd_wtime != apds->init_data.dd_wtime) {
		SENSOR_ERR_LOG("unexpected dd_wtime[0x%x]. expect[0x%x]", arg.dd_wtime, apds->init_data.dd_wtime);
		return -1;
	}

	if (arg.dd_pitlo != 0x00 && arg.dd_pitlo != apds->init_data.dd_pitlo) {
		SENSOR_ERR_LOG("unexpected dd_pitlo[0x%x]. expect[0x00 / 0x%x]", arg.dd_pitlo, apds->init_data.dd_pitlo);
		return -1;
	}

	if (arg.dd_pithi != 0xFF && arg.dd_pithi != apds->init_data.dd_pithi) {
		SENSOR_ERR_LOG("unexpected dd_pithi[0x%x]. expect[0xFF / 0x%x]", arg.dd_pithi, apds->init_data.dd_pithi);
		return -1;
	}

	if (arg.dd_pers != apds->init_data.dd_pers) {
		SENSOR_ERR_LOG("unexpected dd_pers[0x%x]. expect[0x%x]", arg.dd_pers, apds->init_data.dd_pers);
		return -1;
	}
	if (arg.dd_ppulse != apds->init_data.dd_ppulse) {
		SENSOR_ERR_LOG("unexpected dd_ppulse[0x%x]. expect[0x%x]", arg.dd_ppulse, apds->init_data.dd_ppulse);
		return -1;
	}
	if (arg.dd_control != apds->init_data.dd_control) {
		SENSOR_ERR_LOG("unexpected dd_control[0x%x]. expect[0x%x]", arg.dd_control, apds->init_data.dd_control);
		return -1;
	}
	if (arg.dd_config2 != apds->init_data.dd_config2) {
		SENSOR_ERR_LOG("unexpected dd_config2[0x%x]. expect[0x%x]", arg.dd_config2, apds->init_data.dd_config2);
		return -1;
	}
	if (arg.dd_id != apds->init_data.dd_id) {
		SENSOR_ERR_LOG("unexpected dd_id[0x%x]. expect[0x%x]", arg.dd_id, apds->init_data.dd_id);
		return -1;
	}
	if (arg.dd_poffset_ur != apds->init_data.dd_poffset_ur) {
		SENSOR_ERR_LOG("unexpected dd_poffset_ur[0x%x]. expect[0x%x]", arg.dd_poffset_ur, apds->init_data.dd_poffset_ur);
		return -1;
	}
	if (arg.dd_poffset_dl != apds->init_data.dd_poffset_dl) {
		SENSOR_ERR_LOG("unexpected dd_poffset_dl[0x%x]. expect[0x%x]", arg.dd_poffset_dl, apds->init_data.dd_poffset_dl);
		return -1;
	}
	if (arg.dd_config3 != apds->init_data.dd_config3) {
		SENSOR_ERR_LOG("unexpected dd_config3[0x%x]. expect[0x%x]", arg.dd_config3, apds->init_data.dd_config3);
		return -1;
	}
	if (arg.dd_gthr_in != apds->init_data.dd_gthr_in) {
		SENSOR_ERR_LOG("unexpected dd_gthr_in[0x%x]. expect[0x%x]", arg.dd_gthr_in, apds->init_data.dd_gthr_in);
		return -1;
	}
	if (arg.dd_gthr_out != apds->init_data.dd_gthr_out) {
		SENSOR_ERR_LOG("unexpected dd_gthr_out[0x%x]. expect[0x%x]", arg.dd_gthr_out, apds->init_data.dd_gthr_out);
		return -1;
	}
	if (arg.dd_gconf1 != apds->init_data.dd_gconf1) {
		SENSOR_ERR_LOG("unexpected dd_gconf1[0x%x]. expect[0x%x]", arg.dd_gconf1, apds->init_data.dd_gconf1);
		return -1;
	}
	if (arg.dd_gconf2 != apds->init_data.dd_gconf2) {
		SENSOR_ERR_LOG("unexpected dd_gconf2[0x%x]. expect[0x%x]", arg.dd_gconf2, apds->init_data.dd_gconf2);
		return -1;
	}
	if (arg.dd_goffset_u != apds->init_data.dd_goffset_u) {
		SENSOR_ERR_LOG("unexpected dd_goffset_u[0x%x]. expect[0x%x]", arg.dd_goffset_u, apds->init_data.dd_goffset_u);
		return -1;
	}
	if (arg.dd_goffset_d != apds->init_data.dd_goffset_d) {
		SENSOR_ERR_LOG("unexpected dd_goffset_d[0x%x]. expect[0x%x]", arg.dd_goffset_d, apds->init_data.dd_goffset_d);
		return -1;
	}
	if (arg.dd_gpulse != apds->init_data.dd_gpulse) {
		SENSOR_ERR_LOG("unexpected dd_gpulse[0x%x]. expect[0x%x]", arg.dd_gpulse, apds->init_data.dd_gpulse);
		return -1;
	}
	if (arg.dd_goffset_l != apds->init_data.dd_goffset_l) {
		SENSOR_ERR_LOG("unexpected dd_goffset_l[0x%x]. expect[0x%x]", arg.dd_goffset_l, apds->init_data.dd_goffset_l);
		return -1;
	}
	if (arg.dd_goffset_r != apds->init_data.dd_goffset_r) {
		SENSOR_ERR_LOG("unexpected dd_goffset_r[0x%x]. expect[0x%x]", arg.dd_goffset_r, apds->init_data.dd_goffset_r);
		return -1;
	}
	if (arg.dd_gconf3 != apds->init_data.dd_gconf3) {
		SENSOR_ERR_LOG("unexpected dd_gconf3[0x%x]. expect[0x%x]", arg.dd_gconf3, apds->init_data.dd_gconf3);
		return -1;
	}

	if (apds->power_data.power_gs)
		work = APDS9960_DD_GCONF4_GS_EN;
	else
		work = APDS9960_DD_GCONF4_GS_DIS;
	if ((arg.dd_gconf4 & APDS9960_DD_GCONF4_GS_EN_MASK) != work) {
		SENSOR_ERR_LOG("unexpected dd_gconf4[0x%x]. expect[0x%x]", arg.dd_gconf4, work);
		return -1;
	}

	SENSOR_V_LOG("end");
	return 0;
}

static int als_enable_proc(APDS_DATA *apds, unsigned char enable);
static int ps_enable_proc(APDS_DATA *apds, unsigned char enable);
static int gs_enable_proc(APDS_DATA *apds, unsigned char enable);
static int apds_recover_state(APDS_DATA *apds)
{
	int result;

	SENSOR_V_LOG("start");

	mutex_lock(&apds->ps_data_lock);
	mutex_lock(&apds->als_data_lock);
	result = apds_driver_init(&apds->init_data, apds->client);
	mutex_unlock(&apds->als_data_lock);
	mutex_unlock(&apds->ps_data_lock);
	if (result < 0) {
		SENSOR_ERR_LOG("fail apds_driver_init");
		return result;
	}
	if (apds->power_data_last.power_als) {
		result = als_enable_proc(apds, true);
		if (result < 0) {
			SENSOR_ERR_LOG("fail als_enable_proc");
			return result;
		}
	}
	if (apds->power_data_last.power_ps) {
		result = ps_enable_proc(apds, true);
		if (result < 0) {
			SENSOR_ERR_LOG("fail ps_enable_proc");
			return result;
		}
	}
	if (apds->power_data_last.power_gs) {
		result = gs_enable_proc(apds, true);
		if (result < 0) {
			SENSOR_ERR_LOG("fail gs_enable_proc");
			return result;
		}
	}

	SENSOR_V_LOG("end");
	return 0;
}

static int apds_recovery(APDS_DATA *apds)
{
#define RECOVERY_RETRY_MAX	3
	int retries = RECOVERY_RETRY_MAX;
	int result;
	SENSOR_N_LOG("start");

	apds_set_errinfo(apds, ERRINFO_DEV_RESET_OCCURRED);
	while (retries--) {
		result = apds_driver_reset(apds->client);
		if (result < 0) {
			SENSOR_ERR_LOG("apds_driver_reset error result:%d retries:%d",result,retries);
			continue;
		}

		apds_clear_errinfo(apds, ERRINFO_I2C_RECOVERY_FAILED);
		result = apds_recover_state(apds);
		return result;
	}

	SENSOR_ERR_LOG("recovery failed");
	apds_set_errinfo(apds, ERRINFO_DEV_RECOVERY_FAILED);
	free_irq(apds->client->irq, apds);
	return -1;
#undef RECOVERY_RETRY_MAX
}

static void apds_check_error(APDS_DATA *apds)
{
	int ret;

	SENSOR_V_LOG("start");
	ret = apds_confirm_regs(apds);
	if (ret < 0) {
		ret = apds_recovery(apds);
		if (ret < 0) {
			SENSOR_ERR_LOG("failed apds_recovery");
		}
	}

	SENSOR_V_LOG("end");
	return;
}

#ifdef CONFIG_USE_PROXIMITY_TEMPERATURE_CORRECTION
static int ps_get_temperature(APDS_DATA *apds, int *temperature)
{
	int	ret = -1;
#ifdef TEMPERATURE_AVAILABLE
	union power_supply_propval propval = {20,};
	SENSOR_V_LOG("start");

	if (!apds->psy) {
		apds->psy = power_supply_get_by_name("hkadc");
	}
	if (apds->psy) {
		ret = apds->psy->get_property(apds->psy, POWER_SUPPLY_PROP_OEM_USB_THERM, &propval);
		*temperature = propval.intval;
	}
#else
	SENSOR_V_LOG("start TEMPERATURE_AVAILABLE is not defined.");
#endif
	if (camera_temp != DUMMY_TEMP) {
		*temperature = camera_temp;
		ret = 0;
	}
	SENSOR_V_LOG("end");
	return ret;
}
#endif

static void get_new_device_configs(APDS_DATA *apds,
	apds9960_dd_init_arg *arg, uint32_t enable_sensor)
{
	u8	ps_th_base;
	u8	ps_tl_base;
#ifdef CONFIG_USE_PROXIMITY_TEMPERATURE_CORRECTION
	int	result;
	u8	ps_th;
	u8	ps_tl;
	int	temperature = 20;
	int	coef_1;
	int	coef_2;
	int	coef_temp;
#endif

	SENSOR_V_LOG("start");

	mutex_lock(&apds->ps_data_lock);
	ps_th_base = nv_proximity_detect;
	ps_tl_base = nv_proximity_no_detect;
	if (enable_sensor & ENABLE_GS) {
		arg->dd_atime = APDS9960_DD_ATIME_GS_EN;
		arg->dd_ppulse = APDS9960_DD_PPULSE_GS_EN;
		arg->dd_control = APDS9960_DD_CONTROL_GS_EN;
		arg->dd_config2 = APDS9960_DD_CONFIG2_GS_EN;
		arg->dd_poffset_ur = apds_offset_add_correct(nv_gesture_offset_ur, apds->ps_offset_ur);
		arg->dd_poffset_dl = apds_offset_add_correct(nv_gesture_offset_dl, apds->ps_offset_dl);
		arg->dd_gconf4 = (APDS9960_DD_GCONF4_GS_EN | APDS9960_DD_GCONF4_FIFO_CLR);
	} else {
		if (enable_sensor & ENABLE_PS) {
			arg->dd_atime = APDS9960_DD_ATIME_PS_EN;
		} else {
			arg->dd_atime = APDS9960_DD_ATIME_DIS;
		}
		arg->dd_poffset_ur = nv_proximity_offset_ur;
		arg->dd_poffset_dl = nv_proximity_offset_dl;
		arg->dd_ppulse = APDS9960_DD_PPULSE_GS_DIS;
		arg->dd_control = APDS9960_DD_CONTROL_GS_DIS;
		arg->dd_config2 = APDS9960_DD_CONFIG2_GS_DIS;
		arg->dd_gconf4 = APDS9960_DD_GCONF4_GS_DIS;
	}
	arg->dd_gthr_in = nv_gesture_enter;
	arg->dd_gthr_out = nv_gesture_exit;
	arg->dd_goffset_u = apds_offset_add_correct(nv_gesture_offset_up, apds->cal_offset_up);
	arg->dd_goffset_d = apds_offset_add_correct(nv_gesture_offset_down, apds->cal_offset_down);
	arg->dd_goffset_l = apds_offset_add_correct(nv_gesture_offset_left, apds->cal_offset_left);
	arg->dd_goffset_r = apds_offset_add_correct(nv_gesture_offset_right, apds->cal_offset_right);
#ifdef CONFIG_USE_PROXIMITY_TEMPERATURE_CORRECTION
	coef_1 = nv_proximity_temp1;
	coef_2 = nv_proximity_temp2;
	mutex_unlock(&apds->ps_data_lock);
	result = ps_get_temperature(apds, &temperature);

	if (result < 0 || !coef_2) {
		SENSOR_V_LOG("invalid param. result[%d] coef_2[%d]", result, coef_2);
		ps_th = ps_th_base;
		ps_tl = ps_tl_base;
	} else {
		SENSOR_V_LOG("ps_th_base[%x] ps_tl_base[%x] coef_1[%x] coef_2[%x] temperature[%d]",
			ps_th_base, ps_tl_base, coef_1, coef_2, temperature);

		if (temperature < -20)
			temperature = -20;

		if (temperature <= 10) {
			coef_temp = temperature + 20;

			ps_th = (u16)((long)ps_th_base + (-1 * coef_1 * coef_temp / coef_2) + coef_1);
			ps_tl = (u16)((long)ps_tl_base + (-1 * coef_1 * coef_temp / coef_2) + coef_1);

			if (ps_th > REG_PSTH_MAX || ps_tl > REG_PSTL_MAX) {
				SENSOR_N_LOG("invalid ps_th[%d] or ps_tl[%d]. set default", ps_th, ps_tl);
				ps_th = ps_th_base;
				ps_tl = ps_tl_base;
			}
			if (ps_tl > ps_th) {
				SENSOR_N_LOG("notice. ps_tl > ps_th");
			}
		} else {
			ps_th = ps_th_base;
			ps_tl = ps_tl_base;
		}
	}

	arg->dd_pithi = ps_th;
	arg->dd_pitlo = ps_tl;

	SENSOR_V_LOG("end ps_th[%d] ps_tl[%d] ps_offset_ur[%d] ps_offset_dl[%d]", ps_th, ps_tl, arg->dd_poffset_ur, arg->dd_poffset_dl);
#else
	mutex_unlock(&apds->ps_data_lock);
	arg->dd_pithi = ps_th_base;
	arg->dd_pitlo = ps_tl_base;
	SENSOR_V_LOG("end");
#endif
	return;
}

static bool update_device_configs(APDS_DATA *apds, uint32_t enable_sensor)
{
	bool	changed;
	apds9960_dd_init_arg	arg;

	SENSOR_V_LOG("start");

	get_new_device_configs(apds, &arg, enable_sensor);

	mutex_lock(&apds->ps_data_lock);
	if (arg.dd_atime != apds->init_data.dd_atime
		|| arg.dd_pitlo != apds->init_data.dd_pitlo
		|| arg.dd_pithi != apds->init_data.dd_pithi
		|| arg.dd_ppulse != apds->init_data.dd_ppulse
		|| arg.dd_control != apds->init_data.dd_control
		|| arg.dd_poffset_ur != apds->init_data.dd_poffset_ur
		|| arg.dd_poffset_dl != apds->init_data.dd_poffset_dl
		|| arg.dd_config2 != apds->init_data.dd_config2
		|| arg.dd_gthr_in != apds->init_data.dd_gthr_in
		|| arg.dd_gthr_out != apds->init_data.dd_gthr_out
		|| arg.dd_goffset_u != apds->init_data.dd_goffset_u
		|| arg.dd_goffset_d != apds->init_data.dd_goffset_d
		|| arg.dd_goffset_l != apds->init_data.dd_goffset_l
		|| arg.dd_goffset_r != apds->init_data.dd_goffset_r
		|| arg.dd_gconf4 != apds->init_data.dd_gconf4) {
		apds->init_data.dd_atime = arg.dd_atime;
		apds->init_data.dd_pitlo = arg.dd_pitlo;
		apds->init_data.dd_pithi = arg.dd_pithi;
		apds->init_data.dd_ppulse = arg.dd_ppulse;
		apds->init_data.dd_control = arg.dd_control;
		apds->init_data.dd_poffset_ur = arg.dd_poffset_ur;
		apds->init_data.dd_poffset_dl = arg.dd_poffset_dl;
		apds->init_data.dd_config2 = arg.dd_config2;
		apds->init_data.dd_gthr_in = arg.dd_gthr_in;
		apds->init_data.dd_gthr_out = arg.dd_gthr_out;
		apds->init_data.dd_goffset_u = arg.dd_goffset_u;
		apds->init_data.dd_goffset_d = arg.dd_goffset_d;
		apds->init_data.dd_goffset_l = arg.dd_goffset_l;
		apds->init_data.dd_goffset_r = arg.dd_goffset_r;
		apds->init_data.dd_gconf4 = arg.dd_gconf4;
		changed = true;
	} else {
		changed = false;
	}
	mutex_unlock(&apds->ps_data_lock);

	SENSOR_V_LOG("end");
	return changed;
}

static int ps_send_config(APDS_DATA *apds)
{
	int		result;
	apds9960_dd_init_arg	regs;

	SENSOR_V_LOG("start");
	mutex_lock(&apds->ps_data_lock);
	regs.dd_atime = apds->init_data.dd_atime;
	regs.dd_wtime = apds->init_data.dd_wtime;
	regs.dd_pitlo = apds->init_data.dd_pitlo;
	regs.dd_pithi = apds->init_data.dd_pithi;
	regs.dd_pers = apds->init_data.dd_pers;
	regs.dd_ppulse = apds->init_data.dd_ppulse;
	regs.dd_control = apds->init_data.dd_control;
	regs.dd_config2 = apds->init_data.dd_config2;
	regs.dd_poffset_ur = apds->init_data.dd_poffset_ur;
	regs.dd_poffset_dl = apds->init_data.dd_poffset_dl;
	regs.dd_config3 = apds->init_data.dd_config3;
	regs.dd_gconf3 = apds->init_data.dd_gconf3;
	regs.dd_gthr_in = apds->init_data.dd_gthr_in;
	regs.dd_gthr_out = apds->init_data.dd_gthr_out;
	regs.dd_gconf1 = apds->init_data.dd_gconf1;
	regs.dd_gconf2 = apds->init_data.dd_gconf2;
	regs.dd_goffset_u = apds->init_data.dd_goffset_u;
	regs.dd_goffset_d = apds->init_data.dd_goffset_d;
	regs.dd_gpulse = apds->init_data.dd_gpulse;
	regs.dd_goffset_l = apds->init_data.dd_goffset_l;
	regs.dd_goffset_r = apds->init_data.dd_goffset_r;
	regs.dd_gconf3 = apds->init_data.dd_gconf3;
	regs.dd_gconf4 = apds->init_data.dd_gconf4;
	mutex_unlock(&apds->ps_data_lock);

	result = apds_i2c_write(apds->client, APDS9960_DD_ATIME_ADDR, &regs.dd_atime, 1);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("failed to set config");
		return -1;
	}
	result = apds_i2c_write(apds->client, APDS9960_DD_WTIME_ADDR, &regs.dd_wtime, 1);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("failed to set config");
		return -1;
	}
	result = apds_i2c_write(apds->client, APDS9960_DD_PILT_ADDR, &regs.dd_pitlo, 1);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("failed to set config");
		return -1;
	}
	result = apds_i2c_write(apds->client, APDS9960_DD_PIHT_ADDR, &regs.dd_pithi, 2);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("failed to set config");
		return -1;
	}
	result = apds_i2c_write(apds->client, APDS9960_DD_PPULSE_ADDR, &regs.dd_ppulse, 3);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("failed to set config");
		return -1;
	}
	result = apds_i2c_write(apds->client, APDS9960_DD_POFFSET_UR_ADDR, &regs.dd_poffset_ur, 11);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("failed to set config");
		return -1;
	}
	result = apds_i2c_write(apds->client, APDS9960_DD_GOFFSET_R_ADDR, &regs.dd_goffset_r, 3);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("failed to set config");
		return -1;
	}

	SENSOR_V_LOG("end");
	return 0;
}

static void ps_check_thresholds_update(APDS_DATA *apds)
{
	uint32_t enable_sensor;

	SENSOR_V_LOG("start");
	if (!apds->power_data.power_ps) {
		return;
	}

	enable_sensor = get_enable_sensor(
						apds->power_data.power_als,
						apds->power_data.power_ps,
						apds->power_data.power_gs);
	if (!update_device_configs(apds, enable_sensor)) {
		return;
	}

	disable_irq(apds->client->irq);
	ps_send_config(apds);
	enable_irq(apds->client->irq);

	SENSOR_V_LOG("end");
	return;
}

static void apds_schedule_monitor(APDS_DATA *apds)
{
	SENSOR_V_LOG("start");
	queue_delayed_work(monitor_workqueue, &apds->monitor_dwork, msecs_to_jiffies(5000));
}

static void apds_monitor_func(struct work_struct *work)
{
	APDS_DATA   *apds;

	SENSOR_D_LOG("start");

	apds = container_of(work, APDS_DATA, monitor_dwork.work);
	mutex_lock(&apds->control_lock);

	if ((apds->power_data.power_ps || apds->power_data.power_als || apds->power_data.power_gs) &&
	    !apds_is_device_dead(apds)) {
		if ((apds->err_monitor_enable) && !apds->cal_check) {
			ps_check_thresholds_update(apds);
			apds_check_error(apds);
		}
		apds_schedule_monitor(apds);
	}

	mutex_unlock(&apds->control_lock);
	SENSOR_D_LOG("end");
}

static void apds_gs_ignrdetoff_schedule(APDS_DATA *apds)
{
	SENSOR_D_LOG("start");
	queue_delayed_work(gs_ignore_detection_workqueue, &apds->gs_ignrdet_dwork, msecs_to_jiffies(g_ignr_time));
}

static void apds_gs_ignrdet_func(struct work_struct *work)
{
	APDS_DATA   *apds;

	SENSOR_D_LOG("start");

	apds = container_of(work, APDS_DATA, gs_ignrdet_dwork.work);
	mutex_lock(&apds->control_lock);
	apds->is_gs_ignr_enable = false;
	mutex_unlock(&apds->control_lock);
	SENSOR_D_LOG("end");
}

void apds_gs_ignore_detection(bool enable)
{
	APDS_DATA *apds;

	SENSOR_D_LOG("start");

	if(client_data == NULL){
		SENSOR_D_LOG("Sensor during boot");
		return;
	}
	apds = i2c_get_clientdata(client_data);

	if(!apds->gs_en_cnt){
		SENSOR_D_LOG("skip due to not using gs_sensor");
		return;
	}

	if(enable){
	    flush_workqueue(gs_ignore_detection_workqueue);

		mutex_lock(&apds->control_lock);
		apds->is_gs_ignr_enable = true;
		mutex_unlock(&apds->control_lock);
	} else {
	    apds_gs_ignrdetoff_schedule(apds);
	}
	SENSOR_D_LOG("end");
}

static int apds_power_ctrl(APDS_DATA *apds, enum apds_power_ctrl_mode request_mode);
static int als_enable_proc(APDS_DATA *apds, unsigned char enable)
{
	int result;
	uint32_t enable_sensor;
	unsigned char power_set;
	long get_timer;
	long wait_sec;
	unsigned long wait_nsec;

	SENSOR_D_LOG("start. enable=%d", enable);
	if (unlikely(apds->power_data.power_als == enable)) {
		SENSOR_D_LOG("same status. skip");
		return 0;
	}

	if (enable) {
		result = apds_power_ctrl(apds, APDS_POWER_CTRL_NORMAL);
		if(unlikely(result < 0)) {
			SENSOR_ERR_LOG("failed to apds_power_ctrl");
			return result;
		}
	}

	enable_sensor = get_enable_sensor(
						enable,
						apds->power_data.power_ps,
						apds->power_data.power_gs);
	result = update_device_configs(apds, enable_sensor);
	if (result) {
		result = ps_send_config(apds);
		if (unlikely(result < 0)) {
			SENSOR_ERR_LOG("failed to send config");
			return result;
		}
	}

	power_set = get_enable_state(enable_sensor);
	result = apds_i2c_write_byte_data(apds->client,
					   APDS9960_DD_ENABLE_ADDR, power_set);
	if (unlikely(result < 0)) {
		/* i2c communication error */
		SENSOR_ERR_LOG("apds_i2c_write_byte_data error result:%d",result);
		return result;
	}

	if (enable) {
		mutex_lock(&apds->als_data_lock);
		apds->als_val.index = 0;
		apds->als_val.ave_enable = false;
		apds->als_en_first = true;
		memset(apds->als_val.calc_data, 0,
			sizeof(CALC_DATA) * NUM_OF_ALS_VAL);
		mutex_unlock(&apds->als_data_lock);

		/* the setting value from application */
		get_timer = ALS_ON_DELAY_MS;
		/* 125ms(8Hz) at least */
		wait_sec  = (get_timer / SM_TIME_UNIT);
		wait_nsec = ((get_timer - (wait_sec * SM_TIME_UNIT)) * MN_TIME_UNIT);
		result = hrtimer_start(&apds->timer,
				       ktime_set(wait_sec, wait_nsec),
				       HRTIMER_MODE_REL);
		if (unlikely(result)) {
			SENSOR_ERR_LOG("can't start timer");
			return result;
		}

		apds_schedule_monitor(apds);
	} else {
		hrtimer_cancel(&apds->timer);
		cancel_work_sync(&apds->als_work);
		hrtimer_cancel(&apds->timer);

		if ((!apds->power_data.power_ps) && (!apds->power_data.power_gs)) {
			result = apds_power_ctrl(apds, APDS_POWER_CTRL_LOW);
			if(unlikely(result < 0)) {
				SENSOR_ERR_LOG("failed to apds_power_ctrl");
				return result;
			}
		}
	}

	apds->power_data.power_als = enable;

	SENSOR_D_LOG("end. enable=%d", enable);
	return 0;
}

static int ps_enable_proc(APDS_DATA *apds, unsigned char enable)
{
	int result;
	uint32_t enable_sensor;
	unsigned char power_set;
	char pint_clear = 0;

	SENSOR_D_LOG("start. enable=%d", enable);
	SENSOR_N_LOG("enable=%d", enable);
	if (unlikely(apds->power_data.power_ps == enable)) {
		SENSOR_D_LOG("same status. skip");
		return 0;
	}

	if (enable) {
		result = apds_power_ctrl(apds, APDS_POWER_CTRL_NORMAL);
		if(unlikely(result < 0)) {
			SENSOR_ERR_LOG("failed to apds_power_ctrl");
			return result;
		}
	}

	enable_sensor = get_enable_sensor(
						apds->power_data.power_als,
						enable,
						apds->power_data.power_gs);
	result = update_device_configs(apds, enable_sensor);
	if (result) {
		result = ps_send_config(apds);
		if (unlikely(result < 0)) {
			SENSOR_ERR_LOG("failed to send config");
			return result;
		}
	}

	power_set = get_enable_state(enable_sensor);
	result = apds_i2c_write_byte_data(apds->client,
					   APDS9960_DD_ENABLE_ADDR, power_set);
	if (unlikely(result < 0)) {
		/* i2c communication error */
		SENSOR_ERR_LOG("apds_i2c_write_byte_data error result:%d",result);
		return result;
	}

	apds->int_flg = false;

	if (enable) {
		if (!apds->power_data.power_gs) {
			enable_irq(apds->client->irq);
			apds_schedule_monitor(apds);
		}
		apds_i2c_write(apds->client, APDS9960_DD_CMD_CLR_PS_INT, &pint_clear, 1);
	} else {
		if (!apds->power_data.power_gs) {
			disable_irq(apds->client->irq);

			if (!apds->power_data.power_als) {
				result = apds_power_ctrl(apds, APDS_POWER_CTRL_LOW);
				if(unlikely(result < 0)) {
					SENSOR_ERR_LOG("failed to apds_power_ctrl");
					return result;
				}
			}
		}
		apds->ps_det = PROX_STATUS_FAR;
		ps_sensor_report_event_proc(apds, apds->ps_det);
	}

	apds->power_data.power_ps = enable;

	SENSOR_D_LOG("end. enable=%d", enable);
	return 0;
}

static int gs_enable_proc(APDS_DATA *apds, unsigned char enable)
{
	int result;
	uint32_t enable_sensor;
	unsigned char power_set;

	SENSOR_D_LOG("start. enable=%d", enable);
	SENSOR_N_LOG("enable=%d", enable);
	if (unlikely(apds->power_data.power_gs == enable)) {
		SENSOR_D_LOG("same status. skip");
		return 0;
	}

	if (enable) {
		result = apds_power_ctrl(apds, APDS_POWER_CTRL_NORMAL);
		if(unlikely(result < 0)) {
			SENSOR_ERR_LOG("failed to apds_power_ctrl");
			return result;
		}
	}

	enable_sensor = get_enable_sensor(
						apds->power_data.power_als,
						apds->power_data.power_ps,
						enable);
	result = update_device_configs(apds, enable_sensor);
	if (result) {
		result = ps_send_config(apds);
		if (unlikely(result < 0)) {
			SENSOR_ERR_LOG("failed to send config");
			return result;
		}
	}

	power_set = get_enable_state(enable_sensor);
	result = apds_i2c_write_byte_data(apds->client,
					   APDS9960_DD_ENABLE_ADDR, power_set);
	if (unlikely(result < 0)) {
		/* i2c communication error */
		SENSOR_ERR_LOG("apds_i2c_write_byte_data error result:%d",result);
		return result;
	}

	if (!apds->power_data.power_ps) {
		if (enable) {
			enable_irq(apds->client->irq);

			apds_schedule_monitor(apds);
		} else {
			disable_irq(apds->client->irq);

			if (!apds->power_data.power_als) {
				result = apds_power_ctrl(apds, APDS_POWER_CTRL_LOW);
				if(unlikely(result < 0)) {
					SENSOR_ERR_LOG("failed to apds_power_ctrl");
					return result;
				}
			}
		}
	}

	apds->power_data.power_gs = enable;

	SENSOR_D_LOG("end. enable=%d", enable);
	return 0;
}

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t als_val_show(char *buf)
{
	APDS_DATA *apds = i2c_get_clientdata(client_data);
	ssize_t count = 0;

	SENSOR_D_LOG("start");
	mutex_lock(&apds->als_data_lock);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"%d\n", apds->als_val.report_lux);

	SENSOR_D_LOG("end");
	mutex_unlock(&apds->als_data_lock);
	return count;
}
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int als_val_show(struct sensor_api_info *sai, char *buf)
{
	APDS_DATA *apds;
	int count = 0;

	SENSOR_D_LOG("start");
	apds = container_of(sai, APDS_DATA, als_info.sai);
	mutex_lock(&apds->als_data_lock);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"%d\n", apds->als_val.report_lux);

	SENSOR_D_LOG("end");
	mutex_unlock(&apds->als_data_lock);
	return count;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE */


#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t ps_val_show(char *buf)
{
	APDS_DATA *apds = i2c_get_clientdata(client_data);
	ssize_t count = 0;

	SENSOR_D_LOG("start");
	mutex_lock(&apds->ps_data_lock);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"%d\n", apds->ps_det);

	SENSOR_D_LOG("end");
	mutex_unlock(&apds->ps_data_lock);
	return count;
}
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int ps_val_show(struct sensor_api_info *sai, char *buf)
{
	APDS_DATA *apds;
	int count = 0;

	SENSOR_D_LOG("start");
	apds = container_of(sai, APDS_DATA, ps_info.sai);
	mutex_lock(&apds->ps_data_lock);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"%d\n", apds->ps_det);

	SENSOR_D_LOG("end");
	mutex_unlock(&apds->ps_data_lock);
	return count;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE */

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t als_status_show(char *buf)
{
	APDS_DATA *apds;
	ssize_t count = 0;
	int i;
	CALC_DATA *calc_data_h;

	SENSOR_D_LOG("start");
	apds = i2c_get_clientdata(client_data);
	mutex_lock(&apds->als_data_lock);

	calc_data_h = apds->als_val.calc_data;

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"report_lux = %d\n", apds->als_val.report_lux);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	i = apds->als_val.index - 1;
	if (i < 0) {
		i = NUM_OF_ALS_VAL - 1;
		SENSOR_ERR_LOG("apds->als_val.index error :%d->%d",(apds->als_val.index-1),i);
	}

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"index[%d], ave_enable[%d]\n", i, apds->als_val.ave_enable);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	for (i = 0; i < NUM_OF_ALS_VAL; i++) {
		count += scnprintf(buf + count, PAGE_SIZE - count,
			"  calc_data[%d].lux = %ld\n", i, calc_data_h[i].lux);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
			"  calc_data[%d].d0 = %ld\n", i, calc_data_h[i].d0);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
			"  calc_data[%d].d1 = %ld\n", i, calc_data_h[i].d1);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].gain_data0 = %d\n",
				   i, calc_data_h[i].gain_data0);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].gain_data1 = %d\n",
				   i, calc_data_h[i].gain_data1);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].als_data0 = %d\n",
				   i, calc_data_h[i].als_data0);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].als_data1 = %d\n",
				   i, calc_data_h[i].als_data1);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].ratio = %ld\n",
				   i, calc_data_h[i].ratio);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		switch(calc_data_h[i].src) {
		case LIGHT_FLUORESCENT:
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "  calc_data[%d].src = "
					   "LIGHT_FLUORESCENT\n", i);
			break;
		case LIGHT_LED_BULB:
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "  calc_data[%d].src = "
					   "LIGHT_LED_BULB\n", i);

			break;
		case LIGHT_INTERMEDIATE:
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "  calc_data[%d].src = "
					   "LIGHT_INTERMEDIATE\n", i);

			break;
		case LIGHT_SOLAR:
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "  calc_data[%d].src = "
					   "LIGHT_SOLAR\n", i);

			break;
		case LIGHT_CANDESCENT:
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "  calc_data[%d].src = "
					   "LIGHT_CANDESCENT\n", i);

			break;
		default:
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "  calc_data[%d].src = "
					   "LIGHT_SRC_MAX\n", i);

			break;
		}
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].alpha = 0x%04x\n",
				   i, calc_data_h[i].alpha);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].beta = 0x%04x\n",
				   i, calc_data_h[i].beta);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
						"--------\n");
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
		"errinfo = %d (detail:0x%08x)\n",
		(apds->errinfo & ERRINFO_DEV_RECOVERY_FAIL_MASK) ? 4 :
		(apds->errinfo & ERRINFO_I2C_RECOVERY_FAIL_MASK) ? 3 :
		(apds->errinfo & ERRINFO_DEV_RESET_OCCURRED) ? 2 :
		(apds->errinfo & ERRINFO_I2C_RESET_OCCURRED) ? 1 : 0,
		apds->errinfo);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
		"errrecord = %d (detail:0x%08x)\n",
		(apds->errrecord & ERRINFO_DEV_RECOVERY_FAIL_MASK) ? 4 :
		(apds->errrecord & ERRINFO_I2C_RECOVERY_FAIL_MASK) ? 3 :
		(apds->errrecord & ERRINFO_DEV_RESET_OCCURRED) ? 2 :
		(apds->errrecord & ERRINFO_I2C_RESET_OCCURRED) ? 1 : 0,
		apds->errrecord);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	SENSOR_D_LOG("end");
	mutex_unlock(&apds->als_data_lock);
	return count;
fail_exit:
	SENSOR_D_LOG("end - fail");
	mutex_unlock(&apds->als_data_lock);
	return PAGE_SIZE - 1;
}
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int als_status_show(struct sensor_api_info *sai, char *buf)
{
	APDS_DATA *apds;
	int count = 0;
	int i;
	CALC_DATA *calc_data_h;

	SENSOR_D_LOG("start");
	apds = container_of(sai, APDS_DATA, als_info.sai);
	mutex_lock(&apds->als_data_lock);

	calc_data_h = apds->als_val.calc_data;

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"report_lux = %d\n", apds->als_val.report_lux);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	i = apds->als_val.index - 1;
	if (i < 0) {
		i = NUM_OF_ALS_VAL - 1;
		SENSOR_ERR_LOG("apds->als_val.index error :%d->%d",(apds->als_val.index-1),i);
	}

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"index[%d], ave_enable[%d]\n", i, apds->als_val.ave_enable);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	for (i = 0; i < NUM_OF_ALS_VAL; i++) {
		count += scnprintf(buf + count, PAGE_SIZE - count,
			"  calc_data[%d].lux = %ld\n", i, calc_data_h[i].lux);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
			"  calc_data[%d].d0 = %ld\n", i, calc_data_h[i].d0);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
			"  calc_data[%d].d1 = %ld\n", i, calc_data_h[i].d1);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].gain_data0 = %d\n",
				   i, calc_data_h[i].gain_data0);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].gain_data1 = %d\n",
				   i, calc_data_h[i].gain_data1);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].als_data0 = %d\n",
				   i, calc_data_h[i].als_data0);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].als_data1 = %d\n",
				   i, calc_data_h[i].als_data1);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].ratio = %ld\n",
				   i, calc_data_h[i].ratio);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		switch(calc_data_h[i].src) {
		case LIGHT_FLUORESCENT:
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "  calc_data[%d].src = "
					   "LIGHT_FLUORESCENT\n", i);
			break;
		case LIGHT_LED_BULB:
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "  calc_data[%d].src = "
					   "LIGHT_LED_BULB\n", i);

			break;
		case LIGHT_INTERMEDIATE:
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "  calc_data[%d].src = "
					   "LIGHT_INTERMEDIATE\n", i);

			break;
		case LIGHT_SOLAR:
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "  calc_data[%d].src = "
					   "LIGHT_SOLAR\n", i);

			break;
		case LIGHT_CANDESCENT:
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "  calc_data[%d].src = "
					   "LIGHT_CANDESCENT\n", i);

			break;
		default:
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "  calc_data[%d].src = "
					   "LIGHT_SRC_MAX\n", i);

			break;
		}
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].alpha = 0x%04x\n",
				   i, calc_data_h[i].alpha);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].beta = 0x%04x\n",
				   i, calc_data_h[i].beta);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
						"--------\n");
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
		"errinfo = %d (detail:0x%08x)\n",
		(apds->errinfo & ERRINFO_DEV_RECOVERY_FAIL_MASK) ? 4 :
		(apds->errinfo & ERRINFO_I2C_RECOVERY_FAIL_MASK) ? 3 :
		(apds->errinfo & ERRINFO_DEV_RESET_OCCURRED) ? 2 :
		(apds->errinfo & ERRINFO_I2C_RESET_OCCURRED) ? 1 : 0,
		apds->errinfo);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
		"errrecord = %d (detail:0x%08x)\n",
		(apds->errrecord & ERRINFO_DEV_RECOVERY_FAIL_MASK) ? 4 :
		(apds->errrecord & ERRINFO_I2C_RECOVERY_FAIL_MASK) ? 3 :
		(apds->errrecord & ERRINFO_DEV_RESET_OCCURRED) ? 2 :
		(apds->errrecord & ERRINFO_I2C_RESET_OCCURRED) ? 1 : 0,
		apds->errrecord);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	SENSOR_D_LOG("end");
	mutex_unlock(&apds->als_data_lock);
	return count;
fail_exit:
	SENSOR_D_LOG("end - fail");
	mutex_unlock(&apds->als_data_lock);
	return PAGE_SIZE - 1;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE */

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t ps_status_show(char *buf)
{
	APDS_DATA *apds;
	ssize_t count = 0;
	unsigned short ps_data;
	GENREAD_ARG gene_read;
	int result;

	SENSOR_D_LOG("start");
	apds = i2c_get_clientdata(client_data);
	mutex_lock(&apds->control_lock);
	mutex_lock(&apds->ps_data_lock);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps detect = %d\n", apds->ps_det);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps interrupt flg = %d\n", apds->int_flg);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps count = %d\n", apds->ps_val.ps_data);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps int flag = %d\n", apds->ps_val.ps_flag);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
						"--------\n");
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	/* read start address */
	gene_read.adr_reg = APDS9960_DD_PDATA_ADDR;
	gene_read.addr    = (char *)&ps_data;
	gene_read.size    = sizeof(ps_data);

	/* block read */
	result = apds_driver_general_read(gene_read, apds->client);
	if (result > 0) {
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"current ps count = %d, int flag = %d\n",
				CONVERT_TO_BE(ps_data) & PS_DATA_MASK,
				(CONVERT_TO_BE(ps_data) & PS_FLAG_MASK) ? 1 : 0);
	} else {
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"current ps count = 0, int flag = 0\n");
	}
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps_th = %d\n", apds->init_data.dd_pithi);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps_tl = %d\n", apds->init_data.dd_pitlo);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps_offset_ur = %d\n", apds->init_data.dd_poffset_ur);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps_offset_dl = %d\n", apds->init_data.dd_poffset_dl);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	count += scnprintf(buf + count, PAGE_SIZE - count,
						"--------\n");
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
		"errinfo = %d (detail:0x%08x)\n",
		(apds->errinfo & ERRINFO_DEV_RECOVERY_FAIL_MASK) ? 4 :
		(apds->errinfo & ERRINFO_I2C_RECOVERY_FAIL_MASK) ? 3 :
		(apds->errinfo & ERRINFO_DEV_RESET_OCCURRED) ? 2 :
		(apds->errinfo & ERRINFO_I2C_RESET_OCCURRED) ? 1 : 0,
		apds->errinfo);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
		"errrecord = %d (detail:0x%08x)\n",
		(apds->errrecord & ERRINFO_DEV_RECOVERY_FAIL_MASK) ? 4 :
		(apds->errrecord & ERRINFO_I2C_RECOVERY_FAIL_MASK) ? 3 :
		(apds->errrecord & ERRINFO_DEV_RESET_OCCURRED) ? 2 :
		(apds->errrecord & ERRINFO_I2C_RESET_OCCURRED) ? 1 : 0,
		apds->errrecord);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	SENSOR_D_LOG("end");
	mutex_unlock(&apds->ps_data_lock);
	mutex_unlock(&apds->control_lock);
	return count;
fail_exit:
	SENSOR_D_LOG("end - fail");
	mutex_unlock(&apds->ps_data_lock);
	mutex_unlock(&apds->control_lock);
	return PAGE_SIZE - 1;
}
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int ps_status_show(struct sensor_api_info *sai, char *buf)
{
	APDS_DATA *apds;
	int count = 0;
	unsigned short ps_data;
	GENREAD_ARG gene_read;
	int result;

	SENSOR_D_LOG("start");
	apds = container_of(sai, APDS_DATA, ps_info.sai);
	mutex_lock(&apds->control_lock);
	mutex_lock(&apds->ps_data_lock);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps detect = %d\n", apds->ps_det);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps interrupt flg = %d\n", apds->int_flg);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps count = %d\n", apds->ps_val.ps_data);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps int flag = %d\n", apds->ps_val.ps_flag);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
						"--------\n");
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	/* read start address */
	gene_read.adr_reg = APDS9960_DD_PDATA_ADDR;
	gene_read.addr    = (char *)&ps_data;
	gene_read.size    = sizeof(ps_data);

	/* block read */
	result = apds_driver_general_read(gene_read, apds->client);
	if (result > 0) {
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"current ps count = %d, int flag = %d\n",
				CONVERT_TO_BE(ps_data) & PS_DATA_MASK,
				(CONVERT_TO_BE(ps_data) & PS_FLAG_MASK) ? 1 : 0);
	} else {
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"current ps count = 0, int flag = 0\n");
	}
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps_th = %d\n", apds->init_data.dd_pithi);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps_tl = %d\n", apds->init_data.dd_pitlo);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps_offset_ur = %d\n", apds->init_data.dd_poffset_ur);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps_offset_dl = %d\n", apds->init_data.dd_poffset_dl);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	count += scnprintf(buf + count, PAGE_SIZE - count,
						"--------\n");
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
		"errinfo = %d (detail:0x%08x)\n",
		(apds->errinfo & ERRINFO_DEV_RECOVERY_FAIL_MASK) ? 4 :
		(apds->errinfo & ERRINFO_I2C_RECOVERY_FAIL_MASK) ? 3 :
		(apds->errinfo & ERRINFO_DEV_RESET_OCCURRED) ? 2 :
		(apds->errinfo & ERRINFO_I2C_RESET_OCCURRED) ? 1 : 0,
		apds->errinfo);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
		"errrecord = %d (detail:0x%08x)\n",
		(apds->errrecord & ERRINFO_DEV_RECOVERY_FAIL_MASK) ? 4 :
		(apds->errrecord & ERRINFO_I2C_RECOVERY_FAIL_MASK) ? 3 :
		(apds->errrecord & ERRINFO_DEV_RESET_OCCURRED) ? 2 :
		(apds->errrecord & ERRINFO_I2C_RESET_OCCURRED) ? 1 : 0,
		apds->errrecord);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	SENSOR_D_LOG("end");
	mutex_unlock(&apds->ps_data_lock);
	mutex_unlock(&apds->control_lock);
	return count;
fail_exit:
	SENSOR_D_LOG("end - fail");
	mutex_unlock(&apds->ps_data_lock);
	mutex_unlock(&apds->control_lock);
	return PAGE_SIZE - 1;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t als_imit_show(char *buf)
{
	APDS_DATA *apds= i2c_get_clientdata(client_data);
	ssize_t count = 0;

	SENSOR_D_LOG("start");
	mutex_lock(&apds->als_data_lock);

	count += scnprintf(buf, PAGE_SIZE - count,
				   "imit_flg : %d "
				   "imit_d0 : %d "
				   "imit_d1 : %d\n",
				   apds->imit.imit_flg,
				   apds->imit.imit_d0,
				   apds->imit.imit_d1);
	SENSOR_D_LOG("end");
	mutex_unlock(&apds->als_data_lock);
	return count;
}
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int als_imit_show(struct sensor_api_info *sai, char *buf)
{
	APDS_DATA *apds;
	int count = 0;

	SENSOR_D_LOG("start");
	apds = container_of(sai, APDS_DATA, als_info.sai);
	mutex_lock(&apds->als_data_lock);

	count += scnprintf(buf, PAGE_SIZE - count,
				   "imit_flg: %d"
				   "imit_d0: %d"
				   "imit_d1: %d\n",
				   apds->imit.imit_flg,
				   apds->imit.imit_d0,
				   apds->imit.imit_d1);
	SENSOR_D_LOG("end");
	mutex_unlock(&apds->als_data_lock);
	return count;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t als_imit_store(const char *buf)
{
	APDS_DATA *apds;

	SENSOR_D_LOG("start");
	apds = i2c_get_clientdata(client_data);
	mutex_lock(&apds->als_data_lock);

	sscanf(buf, "%d %d %d",
		&apds->imit.imit_flg,
		&apds->imit.imit_d0,
		&apds->imit.imit_d1);

	SENSOR_ERR_LOG("imit_flg: %d imit_d0: %d imit_d1: %d",
					   apds->imit.imit_flg,
					   apds->imit.imit_d0,
					   apds->imit.imit_d1);

	SENSOR_D_LOG("end");
	mutex_unlock(&apds->als_data_lock);
	return 0;
}
#else  /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int als_imit_store(struct sensor_api_info *sai, const char *buf)
{
	APDS_DATA *apds;

	SENSOR_D_LOG("start");
	apds = container_of(sai, APDS_DATA, als_info.sai);
	mutex_lock(&apds->als_data_lock);

	sscanf(buf, "%d %d %d",
		&apds->imit.imit_flg,
		&apds->imit.imit_d0,
		&apds->imit.imit_d1);

	SENSOR_ERR_LOG("imit_flg: %d imit_d0: %d imit_d1: %d",
					   apds->imit.imit_flg,
					   apds->imit.imit_d0,
					   apds->imit.imit_d1);

	SENSOR_D_LOG("end");
	mutex_unlock(&apds->als_data_lock);
	return 0;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

#ifndef CONFIG_USE_MICON_SOFT_STRUCTURE
static int als_enable_show(struct sensor_api_info *sai, char *buf)
{
	APDS_DATA *apds;
	int count = 0;

	SENSOR_D_LOG("start");
	apds = container_of(sai, APDS_DATA, als_info.sai);
	mutex_lock(&apds->control_lock);

	count += scnprintf(buf, PAGE_SIZE - count,
				   "%d\n", apds->als_en_cnt);
	SENSOR_D_LOG("end");
	mutex_unlock(&apds->control_lock);
	return count;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

#ifndef CONFIG_USE_MICON_SOFT_STRUCTURE
static int ps_enable_show(struct sensor_api_info *sai, char *buf)
{
	APDS_DATA *apds;
	int count = 0;

	SENSOR_D_LOG("start");
	apds = container_of(sai, APDS_DATA, ps_info.sai);
	mutex_lock(&apds->control_lock);

	count += scnprintf(buf, PAGE_SIZE - count,
				   "%d\n", apds->ps_en_cnt);
	SENSOR_D_LOG("end");
	mutex_unlock(&apds->control_lock);
	return count;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

#ifndef CONFIG_USE_MICON_SOFT_STRUCTURE
static int als_enable_store(struct sensor_api_info *sai, unsigned int enable)
{
	APDS_DATA *apds;
	int result = 0;

	apds = container_of(sai, APDS_DATA, als_info.sai);

	if (enable > APDS9960_DD_POWER_ENABLE) {
		SENSOR_ERR_LOG("als_enable_store error enable:%d", enable);
		return -EINVAL;
	}

	SENSOR_D_LOG("start. als_en_cnt = %d, enable = %d",
					apds->als_en_cnt, enable);
	mutex_lock(&apds->control_lock);
	if (enable) {
		if (apds->als_en_cnt <= 0) {
			apds->als_en_cnt = 1;
		} else {
			apds->als_en_cnt++;
			goto exit;
		}
	} else {
		apds->als_en_cnt--;
		if (apds->als_en_cnt < 0) {
			apds->als_en_cnt = 0;
			goto exit;
		} else if (apds->als_en_cnt > 0){
			goto exit;
		}
	}

	result = als_enable_proc(apds, enable);
	if (result) {
		SENSOR_ERR_LOG("Failed to enable light sensor[%d]", result);
		if (enable)
			apds->als_en_cnt--;
		else
			apds->als_en_cnt++;
	}
exit:
	SENSOR_D_LOG("end. apds->als_en_cnt = %d", apds->als_en_cnt);
	mutex_unlock(&apds->control_lock);
	return result;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

#ifndef CONFIG_USE_MICON_SOFT_STRUCTURE
static int ps_enable_store(struct sensor_api_info *sai, unsigned int enable)
{
	APDS_DATA *apds;
	int result = 0;

	apds = container_of(sai, APDS_DATA, ps_info.sai);

	if (enable > APDS9960_DD_POWER_ENABLE) {
		SENSOR_ERR_LOG("ps_enable_store error enable:%d", enable);
		return -EINVAL;
	}

	SENSOR_D_LOG("start. ps_en_cnt = %d, enable = %d",
					apds->ps_en_cnt, enable);
	mutex_lock(&apds->control_lock);
	if (enable) {
		if (apds->ps_en_cnt <= 0) {
			apds->ps_en_cnt = 1;
		} else {
			apds->ps_en_cnt++;
			goto exit;
		}
	} else {
		apds->ps_en_cnt--;
		if (apds->ps_en_cnt < 0) {
			apds->ps_en_cnt = 0;
			goto exit;
		} else if (apds->ps_en_cnt > 0){
			goto exit;
		}
	}

	result = ps_enable_proc(apds, enable);
	if (result) {
		SENSOR_ERR_LOG("Failed to enable proximity sensor[%d]", result);
		if (enable)
			apds->ps_en_cnt--;
		else
			apds->ps_en_cnt++;
	}
exit:
	SENSOR_D_LOG("end. apds->ps_en_cnt = %d", apds->ps_en_cnt);
	mutex_unlock(&apds->control_lock);
	return result;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t als_properties_store(const char *buf)
{
	APDS_DATA *apds;
	int cnt;
	int color;
	int offset;
	int work[15] = {0};
	const char *p;

	SENSOR_D_LOG("start");
	apds = i2c_get_clientdata(client_data);

	cnt = sscanf(buf, "%d %n", &color, &offset);
	if (cnt != 1) {
		SENSOR_ERR_LOG("sscanf errors cnt:%d",cnt);
		goto fail_exit;
	}
	if ((color < 0) ||(color >= NUM_OF_COLOR)) {
		SENSOR_ERR_LOG("error color:%d",color);
		goto fail_exit;
	}

	p = buf + offset;
	cnt = sscanf(p, "%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",
		&work[ 0], &work[ 1], &work[ 2], &work[ 3], &work[ 4],
		&work[ 5], &work[ 6], &work[ 7], &work[ 8], &work[ 9],
		&work[10], &work[11], &work[12], &work[13], &work[14]);

	if (cnt != 15) {
		SENSOR_ERR_LOG("sscanf errors cnt:%d",cnt);
		goto fail_exit;
	}

	mutex_lock(&apds->als_data_lock);
	apds->color = color;
	nv_photosensor_th0[apds->color] = work[ 0];
	nv_photosensor_th1[apds->color] = work[ 1];
	nv_photosensor_th2[apds->color] = work[ 2];
	nv_photosensor_th3[apds->color] = work[ 3];
	nv_photosensor_th4[apds->color] = work[ 4];
	nv_photosensor_a0[apds->color]  = work[ 5];
	nv_photosensor_a1[apds->color]  = work[ 6];
	nv_photosensor_a2[apds->color]  = work[ 7];
	nv_photosensor_a3[apds->color]  = work[ 8];
	nv_photosensor_a4[apds->color]  = work[ 9];
	nv_photosensor_b0[apds->color]  = work[10];
	nv_photosensor_b1[apds->color]  = work[11];
	nv_photosensor_b2[apds->color]  = work[12];
	nv_photosensor_b3[apds->color]  = work[13];
	nv_photosensor_b4[apds->color]  = work[14];
	mutex_unlock(&apds->als_data_lock);
	SENSOR_D_LOG("end");
	return 0;
fail_exit:
	SENSOR_D_LOG("end - fail");
	return (-EINVAL);
}
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int als_properties_store(struct sensor_api_info *sai, const char *buf)
{
	APDS_DATA *apds;
	int cnt;
	int color;
	int offset;
	int work[15] = {0};
	const char *p;

	SENSOR_D_LOG("start");
	apds = container_of(sai, APDS_DATA, als_info.sai);

	cnt = sscanf(buf, "%d %n", &color, &offset);
	if (cnt != 1) {
		SENSOR_ERR_LOG("sscanf errors cnt:%d",cnt);
		goto fail_exit;
	}
	if ((color < 0) ||(color >= NUM_OF_COLOR)) {
		SENSOR_ERR_LOG("error color:%d",color);
		goto fail_exit;
	}

	p = buf + offset;
	cnt = sscanf(p, "%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",
		&work[ 0], &work[ 1], &work[ 2], &work[ 3], &work[ 4],
		&work[ 5], &work[ 6], &work[ 7], &work[ 8], &work[ 9],
		&work[10], &work[11], &work[12], &work[13], &work[14]);

	if (cnt != 15) {
		SENSOR_ERR_LOG("sscanf errors cnt:%d",cnt);
		goto fail_exit;
	}

	mutex_lock(&apds->als_data_lock);
	apds->color = color;
	nv_photosensor_th0[apds->color] = work[ 0];
	nv_photosensor_th1[apds->color] = work[ 1];
	nv_photosensor_th2[apds->color] = work[ 2];
	nv_photosensor_th3[apds->color] = work[ 3];
	nv_photosensor_th4[apds->color] = work[ 4];
	nv_photosensor_a0[apds->color]  = work[ 5];
	nv_photosensor_a1[apds->color]  = work[ 6];
	nv_photosensor_a2[apds->color]  = work[ 7];
	nv_photosensor_a3[apds->color]  = work[ 8];
	nv_photosensor_a4[apds->color]  = work[ 9];
	nv_photosensor_b0[apds->color]  = work[10];
	nv_photosensor_b1[apds->color]  = work[11];
	nv_photosensor_b2[apds->color]  = work[12];
	nv_photosensor_b3[apds->color]  = work[13];
	nv_photosensor_b4[apds->color]  = work[14];
	mutex_unlock(&apds->als_data_lock);
	SENSOR_D_LOG("end");
	return 0;
fail_exit:
	SENSOR_D_LOG("end - fail");
	return (-EINVAL);
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t als_properties_show(char *buf)
{
	APDS_DATA *apds;
	ssize_t count = 0;

	SENSOR_D_LOG("start");
	apds = i2c_get_clientdata(client_data);
	mutex_lock(&apds->als_data_lock);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "color is [%d]\n",
			   apds->color);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_th0[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_th0[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_th1[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_th1[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_th2[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_th2[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_th3[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_th3[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_th4[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_th4[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_a0[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_a0[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_a1[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_a1[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_a2[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_a2[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_a3[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_a3[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_a4[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_a4[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_b0[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_b0[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_b1[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_b1[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_b2[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_b2[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_b3[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_b3[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_b4[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_b4[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	SENSOR_D_LOG("end");
	mutex_unlock(&apds->als_data_lock);
	return count;
fail_exit:
	SENSOR_D_LOG("end - fail");
	mutex_unlock(&apds->als_data_lock);
	return PAGE_SIZE - 1;
}
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int als_properties_show(struct sensor_api_info *sai, char *buf)
{
	APDS_DATA *apds;
	int count = 0;

	SENSOR_D_LOG("start");
	apds = container_of(sai, APDS_DATA, als_info.sai);
	mutex_lock(&apds->als_data_lock);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "color is [%d]\n",
			   apds->color);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_th0[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_th0[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_th1[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_th1[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_th2[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_th2[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_th3[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_th3[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_th4[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_th4[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_a0[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_a0[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_a1[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_a1[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_a2[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_a2[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_a3[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_a3[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_a4[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_a4[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_b0[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_b0[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_b1[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_b1[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_b2[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_b2[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_b3[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_b3[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_b4[%d] is [0x%04x]\n",
			   apds->color,
			   nv_photosensor_b4[apds->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	SENSOR_D_LOG("end");
	mutex_unlock(&apds->als_data_lock);
	return count;
fail_exit:
	SENSOR_D_LOG("end - fail");
	mutex_unlock(&apds->als_data_lock);
	return PAGE_SIZE - 1;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t ps_properties_store(const char *buf)
{
	APDS_DATA *apds;
	int cnt;
	int color;
	int offset;
	int work[20] = {0};
	const char *p;

	SENSOR_D_LOG("start");

	apds = i2c_get_clientdata(client_data);
	mutex_lock(&apds->control_lock);

	cnt = sscanf(buf, "%d %n", &color, &offset);
	if (cnt != 1) {
		SENSOR_ERR_LOG("sscanf errors cnt:%d",cnt);
		goto fail_exit;
	}
	if ((color < 0) ||(color >= NUM_OF_COLOR)) {
		SENSOR_ERR_LOG("error color:%d",color);
		goto fail_exit;
	}

	p = buf + offset;
	cnt = sscanf(p, "%x %x %x %x %x %x %x %x %x %x"
		 "%x %x %x %x %x %x %x %x",
		&work[0], &work[1], &work[2], &work[3], &work[4],
		&work[5], &work[6], &work[7], &work[8], &work[9],
		&work[10], &work[11], &work[12], &work[13], &work[14],
		&work[15], &work[16], &work[17]);
	if (cnt != 18) {
		SENSOR_ERR_LOG("sscanf errors cnt:%d",cnt);
		goto fail_exit;
	}

	mutex_lock(&apds->ps_data_lock);
	apds->color = color;
	nv_proximity_detect = work[0];
	nv_proximity_no_detect = work[1];
	nv_proximity_offset_ur = work[2];
	nv_proximity_offset_dl = work[3];
	nv_gesture_offset_ur = work[4];
	nv_gesture_offset_dl = work[5];
	nv_gesture_enter = work[8];
	nv_gesture_exit = work[9];
	nv_gesture_offset_up = work[10];
	nv_gesture_offset_down = work[11];
	nv_gesture_offset_left = work[12];
	nv_gesture_offset_right = work[13];
	mutex_unlock(&apds->ps_data_lock);

	ps_check_thresholds_update(apds);

	SENSOR_D_LOG("end");
	mutex_unlock(&apds->control_lock);
	return 0;
fail_exit:
	SENSOR_D_LOG("end - fail");
	mutex_unlock(&apds->control_lock);
	return (-EINVAL);
}
ssize_t gs_properties_store(const char *buf)
{
	APDS_DATA *apds;
	int cnt;
	int sens;
	int color;
	int offset;
	int work[6] = {0};
	const char *p;

	SENSOR_D_LOG("start");
// blank
	cnt = sscanf(buf, "%d", &sens);
	SENSOR_D_LOG("end %d", sens);
return 0;
//
	apds = i2c_get_clientdata(client_data);
	mutex_lock(&apds->control_lock);

	cnt = sscanf(buf, "%d %n", &color, &offset);
	if (cnt != 1) {
		SENSOR_ERR_LOG("sscanf errors cnt:%d",cnt);
		goto fail_exit;
	}
	if ((color < 0) ||(color >= NUM_OF_COLOR)) {
		SENSOR_ERR_LOG("error color:%d",color);
		goto fail_exit;
	}

	p = buf + offset;
	cnt = sscanf(p, "%x %x %x %x %x %x",
		&work[0], &work[1], &work[2], &work[3], &work[4], &work[5]);
	if (cnt != 6 && cnt != 3) {
		SENSOR_ERR_LOG("sscanf errors cnt:%d",cnt);
		goto fail_exit;
	}

	mutex_lock(&apds->ps_data_lock);
	apds->color = color;
	nv_proximity_detect = work[0];
	nv_proximity_no_detect = work[1];
	nv_proximity_offset_ur = work[2];
	nv_proximity_offset_dl = work[3];
	if (cnt > 3) {
		nv_proximity_temp1 = work[4];
		nv_proximity_temp2 = work[5];
	}
	mutex_unlock(&apds->ps_data_lock);

	ps_check_thresholds_update(apds);

	SENSOR_D_LOG("end");
	mutex_unlock(&apds->control_lock);
	return 0;
fail_exit:
	SENSOR_D_LOG("end - fail");
	mutex_unlock(&apds->control_lock);
	return (-EINVAL);
}

void apds_gs_calibration(void)
{
	APDS_DATA *apds;
	int rc;

	SENSOR_D_LOG("start");

	if(client_data == NULL){
		SENSOR_ERR_LOG("client_data is NULL");
		return;
	}
	apds = i2c_get_clientdata(client_data);
	mutex_lock(&apds->control_lock);
	mutex_lock(&apds->ps_data_lock);
	if(!apds->cal_check){
		apds->cal_check = true;

		rc = apds_i2c_write_byte_data(apds->client, APDS9960_DD_GTHR_IN_ADDR, 0x00);
		rc |= apds_i2c_write_byte_data(apds->client, APDS9960_DD_GTHR_OUT_ADDR, 0x00);
		rc |= apds_i2c_write_byte_data(apds->client, APDS9960_DD_GCONF4_ADDR, 0x03);
		if (unlikely(rc)) {
			SENSOR_ERR_LOG("I2C communication error");
			apds->cal_check = false;
		}
		apds->cal_offset_up = 0;
		apds->cal_offset_down = 0;
		apds->cal_offset_left = 0;
		apds->cal_offset_right = 0;
		gs_offset_write(apds);
	} else 
		SENSOR_ERR_LOG("During calibration now");

	mutex_unlock(&apds->ps_data_lock);
	mutex_unlock(&apds->control_lock);
	SENSOR_D_LOG("end");
}

#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int ps_properties_store(struct sensor_api_info *sai, const char *buf)
{
	APDS_DATA *apds;
	int cnt;
	int color;
	int offset;
	int work[6] = {0};
	const char *p;

	SENSOR_D_LOG("start");
	apds = container_of(sai, APDS_DATA, ps_info.sai);
	mutex_lock(&apds->control_lock);

	cnt = sscanf(buf, "%d %n", &color, &offset);
	if (cnt != 1) {
		SENSOR_ERR_LOG("sscanf errors cnt:%d",cnt);
		goto fail_exit;
	}
	if ((color < 0) ||(color >= NUM_OF_COLOR)) {
		SENSOR_ERR_LOG("error color:%d",color);
		goto fail_exit;
	}

	p = buf + offset;
	cnt = sscanf(p, "%x %x %x %x %x %x",
		&work[0], &work[1], &work[2], &work[3], &work[4], &work[5]);
	if (cnt != 6 && cnt != 3) {
		SENSOR_ERR_LOG("sscanf errors cnt:%d",cnt);
		goto fail_exit;
	}

	mutex_lock(&apds->ps_data_lock);
	apds->color = color;
	nv_proximity_detect = work[0];
	nv_proximity_no_detect = work[1];
	nv_proximity_offset_ur = work[2];
	nv_proximity_offset_dl = work[3];
	if (cnt > 3) {
		nv_proximity_temp1 = work[4];
		nv_proximity_temp2 = work[5];
	}
	mutex_unlock(&apds->ps_data_lock);

	ps_check_thresholds_update(apds);

	SENSOR_D_LOG("end");
	mutex_unlock(&apds->control_lock);
	return 0;
fail_exit:
	SENSOR_D_LOG("end - fail");
	mutex_unlock(&apds->control_lock);
	return (-EINVAL);
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t ps_properties_show(char *buf)
{
	APDS_DATA *apds;
	ssize_t count = 0;

	SENSOR_D_LOG("start");
	apds = i2c_get_clientdata(client_data);
	mutex_lock(&apds->ps_data_lock);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "color is [%d]\n",
			   apds->color);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_proximity_detect[%d] is [0x%02x]\n",
			   apds->color,
			   nv_proximity_detect);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_proximity_no_detect[%d] is [0x%02x]\n",
			   apds->color,
			   nv_proximity_no_detect);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_proximity_offset_ur[%d] is [0x%02x]\n",
			   apds->color,
			   nv_proximity_offset_ur);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_proximity_offset_dl[%d] is [0x%02x]\n",
			   apds->color,
			   nv_proximity_offset_dl);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_gesture_offset_ur[%d] is [0x%02x]\n",
			   apds->color,
			   nv_gesture_offset_ur);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_gesture_offset_dl[%d] is [0x%02x]\n",
			   apds->color,
			   nv_gesture_offset_dl);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_proximity_temp1[%d] is [0x%02x]\n",
			   apds->color,
			   nv_proximity_temp1);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_proximity_temp2[%d] is [0x%02x]\n",
			   apds->color,
			   nv_proximity_temp2);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_gesture_enter[%d] is [0x%02x]\n",
			   apds->color,
			   nv_gesture_enter);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_gesture_exit[%d] is [0x%02x]\n",
			   apds->color,
			   nv_gesture_exit);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_gesture_offset_up[%d] is [0x%02x]\n",
			   apds->color,
			   nv_gesture_offset_up);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_gesture_offset_down[%d] is [0x%02x]\n",
			   apds->color,
			   nv_gesture_offset_down);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_gesture_offset_left[%d] is [0x%02x]\n",
			   apds->color,
			   nv_gesture_offset_left);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_gesture_offset_right[%d] is [0x%02x]\n",
			   apds->color,
			   nv_gesture_offset_right);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "gesture_up_correction[%d] is [0x%04x]\n",
			   apds->color,
			   nv_gs_correction[0]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "gesture_down_correction[%d] is [0x%04x]\n",
			   apds->color,
			   nv_gs_correction[1]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "gesture_left_correction[%d] is [0x%04x]\n",
			   apds->color,
			   nv_gs_correction[2]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "gesture_right_correction[%d] is [0x%04x]\n",
			   apds->color,
			   nv_gs_correction[3]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	SENSOR_D_LOG("end");
	mutex_unlock(&apds->ps_data_lock);
	return count;
fail_exit:
	SENSOR_D_LOG("end - fail");
	mutex_unlock(&apds->ps_data_lock);
	return PAGE_SIZE - 1;
}
ssize_t gs_properties_show(char *buf)
{
	APDS_DATA *apds;
	ssize_t count = 0;

	SENSOR_D_LOG("start");

	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "%d",
			   1);
	SENSOR_D_LOG("end %ld", count);
return count;

	apds = i2c_get_clientdata(client_data);
	mutex_lock(&apds->ps_data_lock);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "color is [%d]\n",
			   apds->color);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_proximity_detect[%d] is [0x%04x]\n",
			   apds->color,
			   nv_proximity_detect);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_proximity_no_detect[%d] is [0x%04x]\n",
			   apds->color,
			   nv_proximity_no_detect);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_proximity_offset[%d] is [0x%04x]\n",
			   apds->color,
			   nv_proximity_offset_ur);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_proximity_temp is [0x%02x,0x%02x]\n",
			   nv_proximity_temp1,
			   nv_proximity_temp2);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	SENSOR_D_LOG("end");
	mutex_unlock(&apds->ps_data_lock);
	return count;
fail_exit:
	SENSOR_D_LOG("end - fail");
	mutex_unlock(&apds->ps_data_lock);
	return PAGE_SIZE - 1;
}
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int ps_properties_show(struct sensor_api_info *sai, char *buf)
{
	APDS_DATA *apds;
	int count = 0;

	SENSOR_D_LOG("start");
	apds = container_of(sai, APDS_DATA, ps_info.sai);
	mutex_lock(&apds->ps_data_lock);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "color is [%d]\n",
			   apds->color);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_proximity_detect[%d] is [0x%04x]\n",
			   apds->color,
			   nv_proximity_detect);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_proximity_no_detect[%d] is [0x%04x]\n",
			   apds->color,
			   nv_proximity_no_detect);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_proximity_offset[%d] is [0x%04x]\n",
			   apds->color,
			   nv_proximity_offset_ur);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_proximity_temp is [0x%02x,0x%02x]\n",
			   nv_proximity_temp1,
			   nv_proximity_temp2);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	SENSOR_D_LOG("end");
	mutex_unlock(&apds->ps_data_lock);
	return count;
fail_exit:
	SENSOR_D_LOG("end - fail");
	mutex_unlock(&apds->ps_data_lock);
	return PAGE_SIZE - 1;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t apds_err_monitor_enable_show(char *buf)
{
	APDS_DATA *apds;
	ssize_t count = 0;

	SENSOR_D_LOG("start");
	apds = i2c_get_clientdata(client_data);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "err_monitor_enable [%d]\n",
			   apds->err_monitor_enable);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	SENSOR_D_LOG("end");
	return count;
fail_exit:
	SENSOR_D_LOG("end - fail");
	return PAGE_SIZE - 1;
}
ssize_t apds_err_monitor_enable_store(const char *buf)
{
	APDS_DATA *apds;
	int enable;
	int cnt;

	SENSOR_D_LOG("start");

	apds = i2c_get_clientdata(client_data);

	cnt = sscanf(buf, "%d", &enable);
	if (cnt != 1) {
		SENSOR_D_LOG("end - fail");
		return 0;
	}

	apds->err_monitor_enable = enable;

	SENSOR_D_LOG("end");

	return 0;
}

ssize_t apds_irq_enable_store(const char *buf)
{
	APDS_DATA *apds;
	int enable;
	int cnt;

	SENSOR_D_LOG("start");

	apds = i2c_get_clientdata(client_data);

	cnt = sscanf(buf, "%d", &enable);
	if (cnt != 1) {
		SENSOR_D_LOG("end - fail");
		return 0;
	}

	if(enable){
		SENSOR_D_LOG("IRQ enable");
		enable_irq(apds->client->irq);
	} else{
		SENSOR_D_LOG("IRQ disable");
		disable_irq(apds->client->irq);
	}

	SENSOR_D_LOG("end");

	return 0;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int apds_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int result = 0;
	APDS_DATA *apds = i2c_get_clientdata(client);

	SENSOR_D_LOG("start");
	mutex_lock(&apds->control_lock);

	apds->power_data_last.power_als = apds->power_data.power_als;
	apds->power_data_last.power_ps = apds->power_data.power_ps;
	apds->power_data_last.power_gs = apds->power_data.power_gs;

	if ((apds->power_data.power_als == APDS9960_DD_POWER_DISABLE) &&
	    (apds->power_data.power_ps == APDS9960_DD_POWER_DISABLE) &&
	    (apds->power_data.power_gs == APDS9960_DD_POWER_DISABLE)) {
		SENSOR_D_LOG("skip");
		return 0;
	}

	if (apds->power_data.power_als == APDS9960_DD_POWER_ENABLE) {
		result = als_enable_proc(apds, APDS9960_DD_POWER_DISABLE);
		if (result)
			SENSOR_ERR_LOG("Failed to disable light sensor[%d]", result);
	}
	if (apds->power_data.power_ps == APDS9960_DD_POWER_ENABLE) {
		enable_irq_wake(client->irq);
		SENSOR_D_LOG("call enable_irq_wake().");
	}
	if (apds->power_data.power_gs == APDS9960_DD_POWER_ENABLE) {
		result = gs_enable_proc(apds, APDS9960_DD_POWER_DISABLE);
		if (result)
			SENSOR_ERR_LOG("Failed to disable gesture sensor[%d]", result);
	}

	SENSOR_D_LOG("end");
	return 0;
}

static int apds_resume(struct i2c_client *client)
{
	int result = 0;
	APDS_DATA *apds = i2c_get_clientdata(client);

	SENSOR_D_LOG("start");
	if ((apds->power_data_last.power_als == APDS9960_DD_POWER_DISABLE) &&
	    (apds->power_data_last.power_ps == APDS9960_DD_POWER_DISABLE) &&
	    (apds->power_data_last.power_gs == APDS9960_DD_POWER_DISABLE)) {
		SENSOR_D_LOG("skip");
		goto exit;
	}


	if (apds->power_data_last.power_als == APDS9960_DD_POWER_ENABLE) {
		result = als_enable_proc(apds, apds->power_data_last.power_als);
		if (result)
			SENSOR_ERR_LOG("Failed to enable light sensor[%d]", result);
	}
	if (apds->power_data_last.power_ps == APDS9960_DD_POWER_ENABLE) {
		disable_irq_wake(client->irq);
		SENSOR_D_LOG("call disable_irq_wake().");
	}
	if (apds->power_data_last.power_gs == APDS9960_DD_POWER_ENABLE) {
		result = gs_enable_proc(apds, apds->power_data_last.power_gs);
		if (result)
			SENSOR_ERR_LOG("Failed to enable gesture sensor[%d]", result);
	}

exit:
	SENSOR_D_LOG("end");
	mutex_unlock(&apds->control_lock);
	return result;
}

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t ps_valid_show(char *buf)
{
	APDS_DATA *apds;
	int count = 0;

	SENSOR_D_LOG("start");
	apds = i2c_get_clientdata(client_data);

	count += scnprintf(buf, PAGE_SIZE - count,
				   "%u\n", atomic_read(&apds->ps_info.valid));
	SENSOR_D_LOG("end");
	return count;

}
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int ps_valid_show(struct sensor_api_info *sai, char *buf)
{
	APDS_DATA *apds;
	int count = 0;

	SENSOR_D_LOG("start");
	apds = container_of(sai, APDS_DATA, ps_info.sai);

	count += scnprintf(buf, PAGE_SIZE - count,
				   "%u\n", atomic_read(&apds->ps_info.valid));
	SENSOR_D_LOG("end");
	return count;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t ps_valid_store(unsigned int valid)
{
	APDS_DATA *apds;
	int last_valid;

	apds = i2c_get_clientdata(client_data);

	SENSOR_D_LOG("start. valid = %d", valid);

	last_valid = atomic_read(&apds->ps_info.valid);

	mutex_lock(&apds->ps_data_lock);
	if (!valid)
		ps_sensor_report_event_proc(apds, 0);

	atomic_set(&apds->ps_info.valid, valid);

	if (valid && !last_valid)
		ps_sensor_report_event_proc(apds, apds->ps_det);

	mutex_unlock(&apds->ps_data_lock);

	SENSOR_D_LOG("end");
	return 0;

}
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int ps_valid_store(struct sensor_api_info *sai, unsigned int valid)
{
	APDS_DATA *apds;
	int last_valid;

	apds = container_of(sai, APDS_DATA, ps_info.sai);

	SENSOR_D_LOG("start. valid = %d", valid);

	last_valid = atomic_read(&apds->ps_info.valid);

	mutex_lock(&apds->ps_data_lock);
	if (!valid)
		ps_sensor_report_event_proc(apds, 0);

	atomic_set(&apds->ps_info.valid, valid);

	if (valid && !last_valid)
		ps_sensor_report_event_proc(apds, apds->ps_det);

	mutex_unlock(&apds->ps_data_lock);

	SENSOR_D_LOG("end");
	return 0;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t als_flush_store(unsigned int flush)
{
	APDS_DATA *apds;
	int result = 0;

	SENSOR_D_LOG("start");

	apds = i2c_get_clientdata(client_data);
	als_sensor_report_flush_event(&apds->als_info);

	SENSOR_D_LOG("end");
	return result;
}
#else /* CONFIG_USE_MICON_SOFT_STRUCTURE */
static int als_flush_store(struct sensor_api_info *sai, unsigned int flush)
{
	APDS_DATA *apds;
	int result = 0;

	SENSOR_D_LOG("start");

	apds = container_of(sai, APDS_DATA, als_info.sai);
	als_sensor_report_flush_event(&apds->als_info);

	SENSOR_D_LOG("end");
	return result;
}
#endif /* !CONFIG_USE_MICON_SOFT_STRUCTURE */

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t ps_flush_store(unsigned int flush)
{
	APDS_DATA *apds;
	int result = 0;

	SENSOR_D_LOG("start");

	apds = i2c_get_clientdata(client_data);
	ps_sensor_report_flush_event(&apds->ps_info);

	SENSOR_D_LOG("end");
	return result;
}
#else /* CONFIG_USE_MICON_SOFT_STRUCTURE */
static int ps_flush_store(struct sensor_api_info *sai, unsigned int flush)
{
	APDS_DATA *apds;
	int result = 0;

	SENSOR_D_LOG("start");

	apds = container_of(sai, APDS_DATA, ps_info.sai);
	ps_sensor_report_flush_event(&apds->ps_info);

	SENSOR_D_LOG("end");
	return result;
}
#endif /* !CONFIG_USE_MICON_SOFT_STRUCTURE */

#ifndef CONFIG_USE_MICON_SOFT_STRUCTURE
const static struct ps_sensor_info ps_info_base = {
	.sai  = {
		.enable_store = ps_enable_store,
		.enable_show = ps_enable_show,
		.value_show = ps_val_show,
		.status_show = ps_status_show,
		.prop_store = ps_properties_store,
		.prop_show = ps_properties_show,
		.valid_store = ps_valid_store,
		.valid_show = ps_valid_show,
		.flush_store = ps_flush_store,
	}
};

const static struct als_sensor_info als_info_base = {
	.sai  = {
		.enable_store = als_enable_store,
		.enable_show = als_enable_show,
		.value_show = als_val_show,
		.status_show = als_status_show,
		.imit_store = als_imit_store,
		.imit_show = als_imit_show,
		.prop_store = als_properties_store,
		.prop_show = als_properties_show,
		.flush_store = als_flush_store,
	}
};

const static struct gs_sensor_info gs_info_base = {
	.sai  = {
		.enable_store = gs_enable_store,
		.enable_show = gs_enable_show,
		.value_show = gs_val_show,
		.status_show = gs_status_show,
		.imit_store = gs_imit_store,
		.imit_show = gs_imit_show,
		.prop_store = gs_properties_store,
		.prop_show = gs_properties_show,
		.flush_store = gs_flush_store,
	}
};
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

static int apds_parse_dt(APDS_DATA *apds)
{
	struct device_node *of_node = apds->client->dev.of_node;

	apds->apds_vdd_is_ldo = of_property_read_bool(of_node, "adps9960,apds-supply-is-ldo");

	if (apds->apds_vdd_is_ldo) {
		apds->vprox_gpio = of_get_named_gpio(of_node, "adps9960,vprox-gpio", 0);
		if(apds->vprox_gpio < 0) {
			SENSOR_ERR_LOG("failed to get VPROXIMITY GPIO=%d",
							apds->vprox_gpio);
			return -1;
		}
	} else {
		of_property_read_u32(of_node,
			"apds-vcc-min-voltage", &apds->apds_vcc.min_uV);
		of_property_read_u32(of_node,
			"apds-vcc-max-voltage", &apds->apds_vcc.max_uV);
		of_property_read_u32(of_node,
			"apds-vcc-on-load-current", &apds->apds_vcc.on_load_uA);
		of_property_read_u32(of_node,
			"apds-vcc-off-load-current", &apds->apds_vcc.off_load_uA);
		SENSOR_D_LOG("regulator min_uV = %d, max_uV = %d, "
			     "on_load_uA = %d, off_load_uA = %d",
					apds->apds_vcc.min_uV,
					apds->apds_vcc.max_uV,
					apds->apds_vcc.on_load_uA,
					apds->apds_vcc.off_load_uA);
	}

	of_property_read_u32(of_node,
			"adps9960,apds-power-offon-interval-ms",
			&apds->apds_power.power_off_on_interval_ms);
	of_property_read_u32(of_node,
			"adps9960,apds-power-on-wait-ms",
			&apds->apds_power.power_on_wait_ms);
	of_property_read_u32(of_node,
			"adps9960,apds-power-normal-wait-ms",
			&apds->apds_power.power_normal_wait_ms);

	return 0;
}

static void apds_regulator_low(APDS_DATA *apds)
{
	int err;
	SENSOR_D_LOG("start");
	if (!apds->apds_vdd_is_ldo) {
		err = regulator_set_optimum_mode(apds->apds_vcc.v_reg, apds->apds_vcc.off_load_uA);
		if( err < 0 ) {
			SENSOR_ERR_LOG("[ALS_PS]%s regulator_set_optimum_mode fail. err=%d\n", __func__, err);
		}
	}
	SENSOR_D_LOG("end");
}

static void apds_regulator_normal(APDS_DATA *apds)
{
	int err;
	SENSOR_D_LOG("start");
	if (!apds->apds_vdd_is_ldo) {
		err = regulator_set_optimum_mode(apds->apds_vcc.v_reg, apds->apds_vcc.on_load_uA);
		if( err < 0 ) {
			SENSOR_ERR_LOG("[ALS_PS]%s regulator_set_optimum_mode fail. err=%d\n", __func__, err);
		}
	}
	SENSOR_D_LOG("end");
}

static int apds_regulator_enable(APDS_DATA *apds, bool enable)
{
	int err = 0;
	SENSOR_D_LOG("start enable[%d]", enable);

	if (enable != apds->apds_power.enabled) {
		if (enable) {
			if (ktime_to_ms(ktime_sub(ktime_get(), apds->apds_power.power_off_time)) < apds->apds_power.power_off_on_interval_ms) {
				usleep_range(apds->apds_power.power_off_on_interval_ms*1000,
							 apds->apds_power.power_off_on_interval_ms*1000);
			}
			if (apds->apds_vdd_is_ldo) {
				gpio_set_value(apds->vprox_gpio, 1);
			} else {
				err = regulator_enable(apds->apds_vcc.v_reg);
			}
			if (!err) {
				apds->apds_power.power_off_time = ktime_get();
			} else  {
				SENSOR_ERR_LOG("regulator_enable fail. err=%d\n", err);
			}
		} else {
			if (apds->apds_vdd_is_ldo) {
				gpio_set_value(apds->vprox_gpio, 0);
			} else {
				err = regulator_disable(apds->apds_vcc.v_reg);
			}
			if (!err) {
				apds->apds_power.power_off_time = ktime_get();
			} else {
				SENSOR_ERR_LOG("regulator_disable fail. err=%d\n", err);
			}
		}

		if (!err) {
			apds->apds_power.enabled = enable;
		}
	}

	SENSOR_D_LOG("end. err[%d]", err);

	return err;
}

static int apds_pinctrl_init(APDS_DATA *apds)
{
	int ret = 0;
	struct device *dev = &apds->client->dev;

	apds->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(apds->pinctrl)) {
		SENSOR_ERR_LOG("Target does not use pinctrl");
		ret = PTR_ERR(apds->pinctrl);
		apds->pinctrl = NULL;
		return ret;
	}

	apds->gpio_state_active
		= pinctrl_lookup_state(apds->pinctrl, "active");
	if (IS_ERR_OR_NULL(apds->gpio_state_active)) {
		SENSOR_ERR_LOG("Can not get active pinstate");
		ret = PTR_ERR(apds->gpio_state_active);
		apds->pinctrl = NULL;
		return ret;
	}

	apds->gpio_state_suspend
		= pinctrl_lookup_state(apds->pinctrl, "suspend");
	if (IS_ERR_OR_NULL(apds->gpio_state_suspend)) {
		SENSOR_ERR_LOG("Can not get suspend pinstate");
		ret = PTR_ERR(apds->gpio_state_suspend);
		apds->pinctrl = NULL;
		return ret;
	}

	return 0;
}

static int apds_pinctrl_select(APDS_DATA *apds, bool on)
{
	struct pinctrl_state *pins_state;
	int ret;

	pins_state = on ? apds->gpio_state_active
					: apds->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(apds->pinctrl, pins_state);
		if (ret) {
			SENSOR_ERR_LOG("can not set %s pins",
				on ? "active" : "suspend");
			return ret;
		}
	} else
		SENSOR_ERR_LOG("not a valid '%s' pinstate",
				on ? "active" : "suspend");

	return 0;
}

static int apds_power_ctrl(APDS_DATA *apds, enum apds_power_ctrl_mode request_mode)
{
	int ret = 0;
	int wait_ms = 0;

	SENSOR_D_LOG("start prev_mode[%d] request_mode[%d]", apds->apds_power.prev_mode, request_mode);
	if (request_mode >= APDS_POWER_CTRL_MAX) {
		SENSOR_ERR_LOG("param err");
		return -1;
	}

	mutex_lock(&apds->apds_power.apds_power_mutex);
	if (request_mode != apds->apds_power.prev_mode) {
		switch(request_mode) {
		case APDS_POWER_CTRL_OFF:
			ret = apds_pinctrl_select(apds, false);
			if (ret) {
				SENSOR_ERR_LOG("apds_pinctrl_select err ret:%d",ret);
				break;
			}
			apds_regulator_low(apds);
			ret = apds_regulator_enable(apds, false);
			break;
		case APDS_POWER_CTRL_LOW:
			ret = apds_pinctrl_select(apds, true);
			if (ret) {
				SENSOR_ERR_LOG("apds_pinctrl_select err ret:%d",ret);
				break;
			}
			apds_regulator_low(apds);
			ret = apds_regulator_enable(apds, true);
			wait_ms = (apds->apds_power.prev_mode == APDS_POWER_CTRL_OFF) ?
				apds->apds_power.power_on_wait_ms : 0;
			break;
		case APDS_POWER_CTRL_NORMAL:
			ret = apds_pinctrl_select(apds, true);
			if (ret) {
				SENSOR_ERR_LOG("apds_pinctrl_select err ret:%d",ret);
				break;
			}
			apds_regulator_normal(apds);
			ret = apds_regulator_enable(apds, true);
			wait_ms = (apds->apds_power.prev_mode == APDS_POWER_CTRL_OFF) ?
				apds->apds_power.power_on_wait_ms : apds->apds_power.power_normal_wait_ms;
			break;
		default:
			break;
		}
	}

	if (!ret) {
		if (wait_ms) {
			SENSOR_D_LOG("wait %dms", wait_ms);
			usleep_range(wait_ms*1000,wait_ms*1000);
		}
		apds->apds_power.prev_mode = request_mode;
	} else {
		SENSOR_ERR_LOG("power_ctrl fail. ret=%d\n", ret);
	}

	mutex_unlock(&apds->apds_power.apds_power_mutex);

	SENSOR_D_LOG("end. ret[%d]", ret);
	return ret;
}

static int apds_power_src_enable(APDS_DATA *apds, bool enable)
{
	if (enable) {
		if (apds->apds_vdd_is_ldo) {
			if (gpio_request(apds->vprox_gpio, "vproximity_en")) {
				SENSOR_ERR_LOG("gpio_request failed.");
				return -1;
			}
		} else {
			apds->apds_vcc.v_reg = regulator_get(&apds->client->dev, "apds-vcc");
			if (IS_ERR(apds->apds_vcc.v_reg)) {
				SENSOR_ERR_LOG("Failed regulator_get for apds-vcc");
				return -1;
			}
		}
	} else {
		if (apds->apds_vdd_is_ldo) {
			gpio_free(apds->vprox_gpio);
		} else {
			regulator_put(apds->apds_vcc.v_reg);
		}
	}
	return 0;
}

/******************************************************************************
 * NAME       : apds_probe
 * FUNCTION   : initialize system
 * REMARKS    :
 *****************************************************************************/
static int apds_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	APDS_DATA *apds;
	int result;

	SENSOR_N_LOG("start");

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
	initialize_done = 0;
#endif	/*CONFIG_USE_MICON_SOFT_STRUCTURE*/

	result = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if (unlikely(!result)) {
		SENSOR_ERR_LOG("need I2C_FUNC_I2C");
		result = -ENODEV;
		goto err_check_functionality_failed;
	}
	apds = kzalloc(sizeof(*apds), GFP_KERNEL);
	if (unlikely(apds == NULL)) {
		SENSOR_ERR_LOG("kzalloc failed");
		result = -ENOMEM;
		goto err_alloc_data_failed;
	}
	INIT_WORK(&apds->als_work, als_work_func);
	INIT_DELAYED_WORK(&apds->monitor_dwork, apds_monitor_func);
	INIT_DELAYED_WORK(&apds->gs_ignrdet_dwork, apds_gs_ignrdet_func);
	mutex_init(&apds->control_lock);
	mutex_init(&apds->ps_data_lock);
	mutex_init(&apds->als_data_lock);
	mutex_init(&apds->gs_data_lock);
	mutex_lock(&apds->control_lock);
	mutex_lock(&apds->ps_data_lock);
	mutex_lock(&apds->als_data_lock);
	mutex_lock(&apds->gs_data_lock);
	apds->client = client;
	i2c_set_clientdata(client, apds);

#ifndef CONFIG_USE_MICON_SOFT_STRUCTURE
	apds->als_info = als_info_base;
#endif /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
	apds->als_info.dev = &client->dev;
	result = als_sensor_register(&apds->als_info);
	if (unlikely(result)) {
		SENSOR_ERR_LOG("Failed to als_sensor_register");
		goto err_als_register_failed;
	}

#ifndef CONFIG_USE_MICON_SOFT_STRUCTURE
	apds->ps_info = ps_info_base;
#else
//	atomic_set(&apds->ps_info.valid, 1);
#endif /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
	apds->ps_info.dev = &client->dev;
	result = ps_sensor_register(&apds->ps_info);
	if (unlikely(result)) {
		SENSOR_ERR_LOG("Failed to ps_sensor_register");
		goto err_ps_register_failed;
	}

#ifndef CONFIG_USE_MICON_SOFT_STRUCTURE
	apds->gs_info = gs_info_base;
#endif /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
	apds->gs_info.dev = &client->dev;
	result = gs_sensor_register(&apds->gs_info);
	if (unlikely(result)) {
		SENSOR_ERR_LOG("Failed to gs_sensor_register");
		goto err_gs_register_failed;
	}

	result = apds_parse_dt(apds);
	if (unlikely(result)) {
		SENSOR_ERR_LOG("failed to parse dt");
		goto err_drv_init_failed;
	}
	apds->apds_power.enabled = false;

	result = apds_power_src_enable(apds, true);
	if (unlikely(result)) {
		SENSOR_ERR_LOG("apds_power_src_enable failed");
		goto err_drv_init_failed;
	}

	result = apds_pinctrl_init(apds);
	if (unlikely(result)) {
		SENSOR_ERR_LOG("failed to apds_pinctrl_init");
		goto err_gpio_free;
	}

	apds->apds_power.prev_mode = APDS_POWER_CTRL_OFF;
	mutex_init(&apds->apds_power.apds_power_mutex);
	result = apds_power_ctrl(apds, APDS_POWER_CTRL_NORMAL);
	if (unlikely(result)) {
		SENSOR_ERR_LOG("failed to power on device");
		goto err_gpio_free;
	}

	memset(&apds->init_data, 0, sizeof(apds->init_data));
	apds->init_data.dd_enable		= APDS9960_DD_ENABLE_NONE;
	apds->init_data.dd_atime		= APDS9960_DD_ATIME_DIS;
	apds->init_data.dd_wtime		= 0xFF;
	apds->init_data.dd_ailtl		= 0x00;
	apds->init_data.dd_ailth		= 0x00;
	apds->init_data.dd_aihtl		= 0x00;
	apds->init_data.dd_aihth		= 0x00;
	apds->init_data.dd_pitlo		= nv_proximity_no_detect;
	apds->init_data.dd_pithi		= nv_proximity_detect;
	apds->init_data.dd_pers			= 0x10;
	apds->init_data.dd_config1		= 0x40;
	apds->init_data.dd_ppulse		= APDS9960_DD_PPULSE_GS_DIS;
	apds->init_data.dd_control		= APDS9960_DD_CONTROL_GS_DIS;
	apds->init_data.dd_config2		= APDS9960_DD_CONFIG2_GS_DIS;
	apds->init_data.dd_id			= 0xAB;
	apds->init_data.dd_status		= APDS9960_DD_CONFIG2_GS_DIS;
	apds->init_data.dd_poffset_ur	= nv_proximity_offset_ur;
	apds->init_data.dd_poffset_dl	= nv_proximity_offset_dl;
	apds->init_data.dd_config3		= 0x00;
	apds->init_data.dd_gthr_in		= nv_gesture_enter;
	apds->init_data.dd_gthr_out		= nv_gesture_exit;
	apds->init_data.dd_gconf1		= 0x03;
	apds->init_data.dd_gconf2		= 0x00;
	apds->init_data.dd_goffset_u	= nv_gesture_offset_up;
	apds->init_data.dd_goffset_d	= nv_gesture_offset_down;
	apds->init_data.dd_gpulse		= 0xD8;
	apds->init_data.dd_goffset_l	= nv_gesture_offset_left;
	apds->init_data.dd_goffset_r	= nv_gesture_offset_right;
	apds->init_data.dd_gconf3		= 0x00;
	apds->init_data.dd_gconf4		= APDS9960_DD_GCONF4_GS_DIS;

	apds->cal_check					= false;
	apds->cal_offset_up				= 0;
	apds->cal_offset_down			= 0;
	apds->cal_offset_left			= 0;
	apds->cal_offset_right			= 0;
	apds->ps_offset_ur				= 0;
	apds->ps_offset_dl				= 0;

	apds->is_gs_ignr_enable = false;
	result = apds_driver_init(&apds->init_data, client);
	if (unlikely(result)) {
		SENSOR_ERR_LOG("failed to driver init");
		goto err_power_on_failed;
	}
	atomic_set(&apds->ps_info.valid, 1);

	result = apds_power_ctrl(apds, APDS_POWER_CTRL_LOW);
	if (unlikely(result)) {
		SENSOR_ERR_LOG("failed to power on device");
		goto err_power_on_failed;
	}

	apds->power_data.power_als = APDS9960_DD_POWER_DISABLE;
	apds->power_data.power_ps = APDS9960_DD_POWER_DISABLE;
	apds->power_data_last.power_als = APDS9960_DD_POWER_DISABLE;
	apds->power_data_last.power_ps = APDS9960_DD_POWER_DISABLE;
	apds->int_flg = false;
	apds->err_monitor_enable = 1;
	apds->cont_exec = false;

	/* check whether to use interrupt or not */
	if (client->irq) {
		/* interrupt process */
		result = request_threaded_irq(client->irq, NULL, apds_irq_handler,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, client->name, apds);

		/* 1 : interrupt mode/ 0 : polling mode */
		if (result == 0) {
			disable_irq(client->irq);
		} else {
			SENSOR_ERR_LOG("request IRQ Failed==>result : %d", result);
			SENSOR_ERR_LOG("client->irq        = 0x%x", client->irq);
			SENSOR_ERR_LOG("apds_irq_handler = 0x%lx", (long)apds_irq_handler);
			SENSOR_ERR_LOG("interrupt flag     = 0x%x", IRQF_TRIGGER_FALLING);
			SENSOR_ERR_LOG("interrupt name     = %s", client->name);
			SENSOR_ERR_LOG("base address       = 0x%lx", (long)apds);
			goto err_power_on_failed;
		}
	}
	/* timer process */
	hrtimer_init(&apds->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	apds->timer.function = als_timer_func;

	/* initialize static variable */
	apds->delay_time = ALS_DATA_DELAY_MS;

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
	client_data = client;
	initialize_done = 1;
	wake_lock_init(&apds->ps_info.wake_lock, WAKE_LOCK_SUSPEND, "ps_sensor");
#endif	/*CONFIG_USE_MICON_SOFT_STRUCTURE*/

	mutex_unlock(&apds->gs_data_lock);
	mutex_unlock(&apds->control_lock);
	mutex_unlock(&apds->ps_data_lock);
	mutex_unlock(&apds->als_data_lock);
	SENSOR_N_LOG("end");
	return (result);
err_power_on_failed:
	apds_power_ctrl(apds, APDS_POWER_CTRL_OFF);
err_gpio_free:
	apds_power_src_enable(apds, false);
err_drv_init_failed:
	gs_sensor_unregister(&apds->gs_info);
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
	atomic_set(&apds->ps_info.valid, 0);
#endif /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
err_gs_register_failed:
	ps_sensor_unregister(&apds->ps_info);
err_ps_register_failed:
	als_sensor_unregister(&apds->als_info);
err_als_register_failed:
	mutex_unlock(&apds->gs_data_lock);
	mutex_unlock(&apds->control_lock);
	mutex_unlock(&apds->ps_data_lock);
	mutex_unlock(&apds->als_data_lock);
	kfree(apds);
err_alloc_data_failed:
err_check_functionality_failed:

	return (result);

}

/******************************************************************************
 * NAME       : apds_remove
 * FUNCTION   : close system
 * REMARKS    :
 *****************************************************************************/
static int apds_remove(struct i2c_client *client)
{
	APDS_DATA *apds;

	apds = i2c_get_clientdata(client);
	als_enable_proc(apds, 0);
	ps_enable_proc(apds, 0);
	gs_enable_proc(apds, 0);
	apds_power_ctrl(apds, APDS_POWER_CTRL_OFF);
	apds_power_src_enable(apds, false);
	ps_sensor_unregister(&apds->ps_info);
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
	atomic_set(&apds->ps_info.valid, 0);
#endif /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
	als_sensor_unregister(&apds->als_info);
	kfree(apds);

	return 0;
}

/******************************************************************************
 * NAME       : apds_shutdown
 * FUNCTION   : shutdown
 * REMARKS    :
 *****************************************************************************/
static void apds_shutdown(struct i2c_client *client)
{
	APDS_DATA *apds;

	SENSOR_V_LOG("start");
	apds = i2c_get_clientdata(client);
	apds_power_ctrl(apds, APDS_POWER_CTRL_OFF);
	SENSOR_V_LOG("end");
}

/******************************************************************************
 * NAME       : apds_init
 * FUNCTION   : register driver to kernel
 * REMARKS    :
 *****************************************************************************/
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
int apds_init(void)
{
	SENSOR_V_LOG("start");
    monitor_workqueue = create_singlethread_workqueue("monitor_workqueue");
    if (!monitor_workqueue) {
        SENSOR_ERR_LOG("create_singlethread_workqueue error");
        return (-ENOMEM);
    }
    gs_ignore_detection_workqueue = create_singlethread_workqueue("gs_ignore_detection_workqueue");
    if (!gs_ignore_detection_workqueue) {
        destroy_workqueue(monitor_workqueue);
        SENSOR_ERR_LOG("create_singlethread_workqueue error");
        return (-ENOMEM);
    }

	SENSOR_V_LOG("end");
    return (i2c_add_driver(&i2c_driver_info));
}
EXPORT_SYMBOL(apds_init);
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int __init apds_init(void)
{
    monitor_workqueue = create_singlethread_workqueue("monitor_workqueue");
    if (!monitor_workqueue) {
        SENSOR_ERR_LOG("create_singlethread_workqueue error");
        return (-ENOMEM);
    }
    gs_ignore_detection_workqueue = create_singlethread_workqueue("gs_ignore_detection_workqueue");
    if (!gs_ignore_detection_workqueue) {
        SENSOR_ERR_LOG("create_singlethread_workqueue error");
        destroy_workqueue(monitor_workqueue);
        return (-ENOMEM);
    }

    return (i2c_add_driver(&i2c_driver_info));
}
#endif /*CONFIG_USE_MICON_SOFT_STRUCTURE*/

/******************************************************************************
 * NAME       : apds_exit
 * FUNCTION   : remove driver from kernel
 * REMARKS    :
 *****************************************************************************/
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
void apds_exit(void)
{
    i2c_del_driver(&i2c_driver_info);
    if (monitor_workqueue) {
        destroy_workqueue(monitor_workqueue);
    }
    if (gs_ignore_detection_workqueue) {
        destroy_workqueue(gs_ignore_detection_workqueue);
    }

    return;
}
EXPORT_SYMBOL(apds_exit);
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static void __exit apds_exit(void)
{
    i2c_del_driver(&i2c_driver_info);
    if (monitor_workqueue) {
        destroy_workqueue(monitor_workqueue);
    }
    if(gs_ignore_detection_workqueue) {
        destroy_workqueue(gs_ignore_detection_workqueue);
    }

    return;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

/************************************************************
 *                     access function                      *
 ***********************************************************/
#define I2C_WRITE_MSG_NUM	1
#define I2C_READ_MSG_NUM	2
#define I2C_RETRY_MAX		3

static int apds_i2c_read(const struct i2c_client *client, uint8_t reg, uint8_t *rbuf, int len)
{
	int ret = 0;
	struct i2c_msg i2cMsg[I2C_READ_MSG_NUM];
	uint8_t buff;
	int retry = I2C_RETRY_MAX + 1;
	int i;
	APDS_DATA   *apds;

	SENSOR_V_LOG("start reg[%02X] len[%d]", reg, len );

	if (client == NULL) {
        SENSOR_ERR_LOG("client null");
		return -ENODEV;
	}

	apds = i2c_get_clientdata(client);
	if (apds_is_device_dead(apds) || apds_is_i2c_dead(apds)) {
        SENSOR_ERR_LOG("apds_is_i2c_dead error");
		return -1;
	}

	while (retry--) {
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
		if (ret == I2C_READ_MSG_NUM) {
			for (i = 0; i < len; i++) {
				SENSOR_D_LOG("i2c read reg[%02X] value[%02X]",
				             (unsigned int)(reg + i), (unsigned int)*(rbuf + i));
			}
			SENSOR_V_LOG("end - return len=%d", len);
			return len;
		} else {
			SENSOR_ERR_LOG("i2c transfer error[%d] in while.",ret );
			apds_set_errinfo(i2c_get_clientdata(client), ERRINFO_I2C_RESET_OCCURRED);
		}
	}
	apds_set_errinfo(i2c_get_clientdata(client), ERRINFO_I2C_RECOVERY_FAILED);
	SENSOR_ERR_LOG("i2c transfer error[%d]",ret );
	return -1;
}

static int apds_i2c_write(const struct i2c_client *client, uint8_t reg, const uint8_t *wbuf, int len)
{
#define BUF_SIZE	0x20
	int ret = 0;
	struct i2c_msg i2cMsg[I2C_WRITE_MSG_NUM];
	uint8_t buff[BUF_SIZE];
	int retry = I2C_RETRY_MAX + 1;
	int i;
	APDS_DATA   *apds;

	SENSOR_V_LOG("start reg[%02X] len[%d]", reg, len );

	if (unlikely(client == NULL)) {
		SENSOR_ERR_LOG("client null");
		return -ENODEV;
	}

	apds = i2c_get_clientdata(client);
	if (apds_is_device_dead(apds) || apds_is_i2c_dead(apds)) {
		SENSOR_ERR_LOG("ps_als_is_device_dead error");
		return -1;
	}

	if (unlikely(len > BUF_SIZE - 1)) {
		SENSOR_ERR_LOG("size over len[%d]", len);
		return -ENOMEM;
	}

	while (retry--) {
		buff[0] = reg;
		memcpy(&buff[1], wbuf, len);

		i2cMsg[0].addr = client->addr;
		i2cMsg[0].flags = 0;
		i2cMsg[0].len = len + 1;
		i2cMsg[0].buf = (uint8_t *)buff;

		ret = i2c_transfer(client->adapter, &i2cMsg[0], I2C_WRITE_MSG_NUM);

		if (ret == I2C_WRITE_MSG_NUM) {
			for (i = 0; i < len; i++) {
				SENSOR_D_LOG("i2c write reg[%02X] value[%02X]",
				             (unsigned int)(reg + i), (unsigned int)*(wbuf + i));
			}
			return 0;
		} else {
			SENSOR_ERR_LOG("i2c transfer error[%d] in while.",ret );
			apds_set_errinfo(i2c_get_clientdata(client), ERRINFO_I2C_RESET_OCCURRED);
		}
	}
	apds_set_errinfo(i2c_get_clientdata(client), ERRINFO_I2C_RECOVERY_FAILED);
	SENSOR_ERR_LOG("i2c transfer error[%d]",ret );

	return -1;
#undef BUF_SIZE
}

static int apds_i2c_read_byte_data(const struct i2c_client *client,
                                      u8 command)
{
	int ret;
	uint8_t buff = 0;

	ret = apds_i2c_read(client, command, &buff, 1);

	return (ret < 0) ? ret : buff;
}

static int apds_i2c_write_byte_data(const struct i2c_client *client,
                                      u8 command, u8 value)
{
	int ret;

	ret = apds_i2c_write(client, command, &value, 1);

	return ret;
}

static int apds_i2c_write_i2c_block_data(const struct i2c_client *client, u8 command,
                                           u8 length, const u8 *values)
{
	int ret;

	ret = apds_i2c_write(client, command, values, length);

	return ret;
}

/******************************************************************************
 * NAME       : apds_driver_init
 * FUNCTION   : initialize RPR0521
 * REMARKS    :
 *****************************************************************************/
static int apds_driver_init(apds9960_dd_init_arg *data, struct i2c_client *client)
{
	int result;
	int len;

	/* check the parameter */
	if (data->dd_enable > APDS9960_DD_ENABLE_MAX) {
		SENSOR_ERR_LOG("data->dd_enable = 0x%x", data->dd_enable);
		return (-EINVAL);
	}

	len = (APDS9960_DD_CONFIG2_ADDR - APDS9960_DD_ENABLE_ADDR + 1);
	result = apds_i2c_write_i2c_block_data(client,
						APDS9960_DD_ENABLE_ADDR,
						len,
						(unsigned char *)&data->dd_enable);
	if (result < 0) {
		SENSOR_ERR_LOG("apds_i2c_write_i2c_block_data error result%d", result);
		return (result);
	}

	len = (APDS9960_DD_GCONF4_ADDR - APDS9960_DD_POFFSET_UR_ADDR + 1);
	result = apds_i2c_write_i2c_block_data(client,
						APDS9960_DD_POFFSET_UR_ADDR,
						len,
						(unsigned char *)&data->dd_poffset_ur);

	return (result);
}

static void apds_driver_param_reset(APDS_DATA *apds)
{
	uint32_t enable_sensor;

	SENSOR_D_LOG("start.");

	apds->power_data_last = apds->power_data;

	if (apds->power_data_last.power_als) {
		hrtimer_cancel(&apds->timer);
		cancel_work_sync(&apds->als_work);
		hrtimer_cancel(&apds->timer);
	}
	if (apds->power_data_last.power_ps || apds->power_data_last.power_gs) {
		disable_irq(apds->client->irq);
	}

	apds->power_data.power_als = APDS9960_DD_POWER_DISABLE;
	apds->power_data.power_ps = APDS9960_DD_POWER_DISABLE;
	apds->power_data.power_gs = APDS9960_DD_POWER_DISABLE;

	enable_sensor = get_enable_sensor(
					APDS9960_DD_POWER_DISABLE,
					APDS9960_DD_POWER_DISABLE,
					APDS9960_DD_POWER_DISABLE);
	update_device_configs(apds, enable_sensor);

	SENSOR_D_LOG("end.");
}

/******************************************************************************
 * NAME       : apds_driver_reset
 * FUNCTION   : reset RPR0521 register
 * REMARKS    :
 *****************************************************************************/
static int apds_driver_reset(struct i2c_client *client)
{
	int result;
	APDS_DATA *apds = i2c_get_clientdata(client_data);

	apds_driver_param_reset(apds);

	result = apds_power_ctrl(apds, APDS_POWER_CTRL_OFF);
	if(unlikely(result < 0)) {
		SENSOR_ERR_LOG("failed to apds_power_ctrl");
		return result;
	}

	msleep(10);

	result = apds_power_ctrl(apds, APDS_POWER_CTRL_NORMAL);
	if(unlikely(result < 0)) {
		SENSOR_ERR_LOG("failed to apds_power_ctrl");
		return result;
	}

	return (result);
}

/******************************************************************************
 * NAME       : apds_driver_read_power_state
 * FUNCTION   : read the value of PS and ALS status in RPR0521
 * REMARKS    :
 *****************************************************************************/
static int apds_driver_read_power_state(PWR_ST *pwr_st, struct i2c_client *client)
{
	int result;

	/* read control state of ps and als */
	result = apds_i2c_read_byte_data(client, APDS9960_DD_ENABLE_ADDR);
	if (unlikely(result < 0)) {
		pwr_st->als_state = CTL_STANDBY;
		pwr_st->ps_state  = CTL_STANDBY;
		pwr_st->ps_state  = CTL_STANDBY;
		SENSOR_ERR_LOG("apds_i2c_read_byte_data error result:%d",result);
	} else {
		/* check power state of als from control state */
		if (result & APDS9960_DD_ENABLE_ALS) {
			pwr_st->als_state = CTL_STANDALONE;
		} else {
			pwr_st->als_state = CTL_STANDBY;
		}

		/* check power state of ps from control state */
		if ((result & APDS9960_DD_ENABLE_PS) && (result & APDS9960_DD_ENABLE_PS_INT)) {
			pwr_st->ps_state = CTL_STANDALONE;
		} else {
			pwr_st->ps_state = CTL_STANDBY;
		}

		if ((result & APDS9960_DD_ENABLE_GS) && (result & APDS9960_DD_ENABLE_PS)) {
			pwr_st->gs_state = CTL_STANDALONE;
		} else {
			pwr_st->gs_state = CTL_STANDBY;
		}
	}

	return (result);
}

/******************************************************************************
 * NAME       : apds_driver_general_read
 * FUNCTION   : read general multi bytes
 * REMARKS    :
 *****************************************************************************/
static int apds_driver_general_read(GENREAD_ARG data, struct i2c_client *client)
{
	int            result;

	if (data.size == 0) {
		SENSOR_ERR_LOG("error data.size:%d",data.size);
		return (-EINVAL);
	}
	/* check the parameter of register */
	if ((data.adr_reg < APDS9960_DD_ENABLE_ADDR) || (data.adr_reg > APDS9960_DD_GFIFO3_ADDR)) {
		SENSOR_ERR_LOG("parameter error");
		return (-EINVAL);
	}

	result = apds_i2c_read(client, data.adr_reg, data.addr, data.size);
	if (result < 0) {
		SENSOR_ERR_LOG("transfer error");
	}

	return (result);
}

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE

bool als_sensor_get_en_first(void)
{
	APDS_DATA *apds = i2c_get_clientdata(client_data);
	SENSOR_D_LOG("start");
	SENSOR_D_LOG("end");
	return apds->als_en_first;
}

int32_t als_sensor_activate(bool enable)
{
	APDS_DATA *apds;
	int result = 0;

	apds = i2c_get_clientdata(client_data);

	if (unlikely(enable > APDS9960_DD_POWER_ENABLE)) {
		SENSOR_D_LOG("INVALID enable val=%d",enable);
		return -EINVAL;
	}

	SENSOR_D_LOG("start. als_en_cnt = %d, enable = %d",
					apds->als_en_cnt, enable);
	mutex_lock(&apds->control_lock);
	if (enable) {
		if (apds->als_en_cnt <= 0) {
			apds->als_en_cnt = 1;
		} else {
			apds->als_en_cnt++;
			goto exit;
		}
	} else {
		apds->als_en_cnt--;
		if (apds->als_en_cnt < 0) {
			apds->als_en_cnt = 0;
			goto exit;
		} else if (apds->als_en_cnt > 0){
			goto exit;
		}
	}

	result = als_enable_proc(apds, enable);
	if (result) {
		SENSOR_ERR_LOG("Failed to enable light sensor[%d]", result);
		if (enable)
			apds->als_en_cnt--;
		else
			apds->als_en_cnt++;
	}
exit:
	SENSOR_D_LOG("end. apds->als_en_cnt = %d", apds->als_en_cnt);
	mutex_unlock(&apds->control_lock);
	return result;
}

int32_t ps_sensor_activate(bool enable)
{
	APDS_DATA *apds = i2c_get_clientdata(client_data);
	int result = 0;

	if (unlikely(enable > APDS9960_DD_POWER_ENABLE)) {
		SENSOR_D_LOG("INVALID enable val=%d",enable);
		return -EINVAL;
	}

	SENSOR_D_LOG("start. ps_en_cnt = %d, enable = %d",
					apds->ps_en_cnt, enable);
	mutex_lock(&apds->control_lock);
	if (enable) {
		if (apds->ps_en_cnt <= 0) {
			apds->ps_en_cnt = 1;
		} else {
			apds->ps_en_cnt++;
			goto exit;
		}
	} else {
		apds->ps_en_cnt--;
		if (apds->ps_en_cnt < 0) {
			apds->ps_en_cnt = 0;
			goto exit;
		} else if (apds->ps_en_cnt > 0){
			goto exit;
		}
	}

	result = ps_enable_proc(apds, enable);
	if (result) {
		SENSOR_ERR_LOG("Failed to enable proximity sensor[%d]", result);
		if (enable)
			apds->ps_en_cnt--;
		else
			apds->ps_en_cnt++;
	}
exit:
	SENSOR_D_LOG("end. apds->ps_en_cnt = %d", apds->ps_en_cnt);
	mutex_unlock(&apds->control_lock);
	return result;
}

int32_t gs_sensor_activate(bool enable)
{
	APDS_DATA *apds = i2c_get_clientdata(client_data);
	int result = 0;

	if (unlikely(enable > APDS9960_DD_POWER_ENABLE)) {
		SENSOR_D_LOG("INVALID enable val=%d",enable);
		return -EINVAL;
	}

	SENSOR_D_LOG("start. gs_en_cnt = %d, enable = %d",
					apds->gs_en_cnt, enable);
	mutex_lock(&apds->control_lock);
	if (enable) {
		if (apds->gs_en_cnt <= 0) {
			apds->gs_en_cnt = 1;
		} else {
			apds->gs_en_cnt++;
			goto exit;
		}
	} else {
		apds->gs_en_cnt--;
		if (apds->gs_en_cnt < 0) {
			apds->gs_en_cnt = 0;
			goto exit;
		} else if (apds->gs_en_cnt > 0){
			goto exit;
		}
	}

	result = gs_enable_proc(apds, enable);
	if (result) {
		SENSOR_ERR_LOG("Failed to enable gesture sensor[%d]", result);
		if (enable)
			apds->gs_en_cnt--;
		else
			apds->gs_en_cnt++;
	}
exit:
	SENSOR_D_LOG("end. apds->ps_en_cnt = %d", apds->ps_en_cnt);
	mutex_unlock(&apds->control_lock);
	return result;
}

uint32_t ps_sensor_get_count(void)
{
	uint32_t ps_count = 0;
	APDS_DATA *apds = i2c_get_clientdata(client_data);

	ps_driver_read_data(client_data);
	ps_count = (uint32_t)apds->ps_val.ps_data;

	return ps_count;
}

int32_t als_get_initialize_state(void){
	SENSOR_D_LOG("start");
	SENSOR_D_LOG("end. ret = %d", initialize_done);
	return initialize_done;
}
int32_t ps_get_initialize_state(void){
	SENSOR_D_LOG("start");
	SENSOR_D_LOG("end. ret = %d", initialize_done);
	return initialize_done;
}
int32_t gs_get_initialize_state(void){
	SENSOR_D_LOG("start");
	SENSOR_D_LOG("end. ret = %d", initialize_done);
	return initialize_done;
}
uint32_t gs_get_ignore_time(void)
{
	SENSOR_D_LOG("start");
	SENSOR_D_LOG("end. ret = %d", g_ignr_time);
	return g_ignr_time;
}
void gs_set_ignore_time(uint32_t time_ms)
{
	SENSOR_D_LOG("start");
    g_ignr_time = time_ms;
	SENSOR_D_LOG("end. ret = %d", g_ignr_time);
}

#else	/*CONFIG_USE_MICON_SOFT_STRUCTURE*/

module_init(apds_init);
module_exit(apds_exit);
#endif	/*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

MODULE_DESCRIPTION("Proximity Sensor & Ambient Light Sensor & Gesture Sensor Driver");
MODULE_LICENSE("GPL");
