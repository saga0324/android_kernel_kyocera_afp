/* This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
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

#include "rohm_rpr0521_i2c.h"
#include "rohm_rpr0521_i2c_config.h"
#include "rohm_rpr0521_i2c_if.h"
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
#include "sensor_driver.h"
#else
#include "sensor_util.h"
#endif	/*CONFIG_USE_MICON_SOFT_STRUCTURE*/
#include "als_sensor.h"
#include "ps_sensor.h"

#ifdef CONFIG_OEM_HKADC
#define TEMPERATURE_AVAILABLE
#endif

#define DUMMY_TEMP	1000
static int camera_temp = DUMMY_TEMP;
module_param(camera_temp, int, 0644);

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
#define	WAKE_LOCK_TIME_DETECT	(msecs_to_jiffies(200))
#define	WAKE_LOCK_TIME_NODETECT	(msecs_to_jiffies(1000))

static struct i2c_client *client_rpr0521 = NULL;
static uint32_t rpr0521_initialize = 0;
#endif	/*CONFIG_USE_MICON_SOFT_STRUCTURE*/


/******************************* define *******************************/
struct ps_als_reg_data {
	struct regulator* v_reg;
	uint32_t min_uV;
	uint32_t max_uV;
	uint32_t on_load_uA;
	uint32_t off_load_uA;
};

struct ps_als_power_ctrl_data {
	bool enabled;
	ktime_t power_off_time;
	int prev_mode;
	struct mutex ps_als_power_mutex;
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

#define ERRINFO_I2C_RESET_OCCURRED	0x00000001
#define ERRINFO_SOFT_RESET_OCCURRED	0x00000002
#define ERRINFO_I2C_RECOVERY_FAILED	0x10000000
#define ERRINFO_RECOVERY_FAILED		0x20000000
#define ERRINFO_RECOVERY_FAIL_MASK	0xF0000000

/* structure of peculiarity to use by system */
typedef struct {
	struct i2c_client	*client;	/* structure pointer for i2c bus            */
	struct hrtimer		timer;		/* structure for timer handler              */
	struct work_struct	als_work;	/* structure for work queue                 */
	struct delayed_work	monitor_dwork;
	int			delay_time;	/* delay time to set from application       */
	struct ps_sensor_info	ps_info;
	struct als_sensor_info	als_info;
	struct ps_als_reg_data	ps_als_vcc;
	struct ps_als_power_ctrl_data	ps_als_power;
	bool		ps_als_vdd_is_ldo;
	bool		ps_als_is_PMIC_supply_ctrl;
	int			vprox_gpio;
	INIT_ARG		init_data;
	POWERON_ARG		power_data;
	POWERON_ARG		power_data_last;
	int			als_en_cnt;
	int			ps_en_cnt;
	DEVICE_VAL		dev_val;
	struct als_val		als_val;
	READ_DATA_PS_BUF	ps_val;
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
	uint32_t		errinfo;
	struct power_supply	*psy;
} PS_ALS_DATA;

/* logical functions */
static int                  get_from_device(DEVICE_VAL *calc_data, struct i2c_client *client);
static void                 als_work_func(struct work_struct *work);
static enum hrtimer_restart als_timer_func(struct hrtimer *timer);
static int                  ps_als_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int                  ps_als_remove(struct i2c_client *client);
static void                 ps_als_shutdown(struct i2c_client *client);
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
int ps_als_init(void);
void ps_als_exit(void);
#else
static int __init ps_als_init(void);
static void __exit ps_als_exit(void);
#endif
/* access functions */
static int ps_als_driver_init(INIT_ARG *data, struct i2c_client *client);
static int ps_als_driver_reset(struct i2c_client *client);
static int ps_als_i2c_read(const struct i2c_client *client, uint8_t reg, uint8_t *rbuf, int len);
static int ps_als_i2c_write(const struct i2c_client *client, uint8_t reg, const uint8_t *wbuf, int len);
static int ps_als_i2c_read_byte_data(const struct i2c_client *client,
                                      u8 command);
static int ps_als_i2c_write_byte_data(const struct i2c_client *client,
                                      u8 command, u8 value);
static int ps_als_i2c_write_i2c_block_data(const struct i2c_client *client, u8 command,
                                           u8 length, const u8 *values);
//static int ps_als_driver_power_on_off(POWERON_ARG data, struct i2c_client *client);
static int ps_als_driver_read_power_state(PWR_ST *pwr_st, struct i2c_client *client);
static int als_driver_read_data(READ_DATA_ALS_BUF *data, struct i2c_client *client);
static int ps_als_driver_general_read(GENREAD_ARG data, struct i2c_client *client);
static int ps_als_suspend(struct i2c_client *client, pm_message_t mesg);
static int ps_als_resume(struct i2c_client *client);
static void ps_sensor_report_event_proc(PS_ALS_DATA *ps_als, uint32_t detect);

/**************************** variable declaration ****************************/
static const char              rpr0521_driver_ver[] = RPR0521_DRIVER_VER;
static struct workqueue_struct *rohm_workqueue;

/**************************** structure declaration ****************************/
/* I2C device IDs supported by this driver */
static const struct i2c_device_id ps_als_id[] = {
    { RPR0521_I2C_NAME, 0 }, /* rohm rpr0521 driver */
    { }
};

/* represent an I2C device driver */
static struct i2c_driver rpr0521_driver = {
	.driver = {                      /* device driver model driver */
		.name = RPR0521_I2C_NAME,
	},
	.probe    = ps_als_probe,        /* callback for device binding */
	.remove   = ps_als_remove,       /* callback for device unbinding */
	.shutdown = ps_als_shutdown,
	.id_table = ps_als_id,           /* list of I2C devices supported by this driver */
	.suspend  = ps_als_suspend,
	.resume   = ps_als_resume,
};

/* gain table */
#define GAIN_FACTOR (16)
static const struct GAIN_TABLE {
	unsigned char DATA0;
	unsigned char DATA1;
} GAIN_TABLE[GAIN_FACTOR] = {
	{  1,   1},   /*  0 */
	{  0,   0},   /*  1 */
	{  0,   0},   /*  2 */
	{  0,   0},   /*  3 */
	{  2,   1},   /*  4 */
	{  2,   2},   /*  5 */
	{  0,   0},   /*  6 */
	{  0,   0},   /*  7 */
	{  0,   0},   /*  8 */
	{  0,   0},   /*  9 */
	{ 64,  64},   /* 10 */
	{  0,   0},   /* 11 */
	{  0,   0},   /* 12 */
	{  0,   0},   /* 13 */
	{128,  64},   /* 14 */
	{128, 128}    /* 15 */
};

#define NUM_OF_COLOR 5
#define ALS_EXP_OF_NV_TH 1000
#define ALS_EXP_OF_NV_COEF 10000
#define ALS_MAX_LUX_VAL 10000
#define ALS_MAX_DX_VAL 0x3A98
#define ALS_D0_D1_TH 0x0100

static u16 nv_proximity_detect[NUM_OF_COLOR] =
	{0x012C, 0x012C, 0x012C, 0x012C, 0x012C};
static u16 nv_proximity_no_detect[NUM_OF_COLOR] =
	{0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x00FF};
static u16 nv_proximity_offset[NUM_OF_COLOR] =
	{0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
static u16 nv_photosensor_th0[NUM_OF_COLOR] =
	{0x00FA, 0x00FA, 0x00FA, 0x00FA, 0x00FA};
static u16 nv_photosensor_th1[NUM_OF_COLOR] =
	{0x0190, 0x0190, 0x0190, 0x0190, 0x0190};
static u16 nv_photosensor_th2[NUM_OF_COLOR] =
	{0x02BC, 0x02BC, 0x02BC, 0x02BC, 0x02BC};
static u16 nv_photosensor_th3[NUM_OF_COLOR] =
	{0x0578, 0x0578, 0x0578, 0x0578, 0x0578};
static u16 nv_photosensor_th4[NUM_OF_COLOR] =
	{0x0640, 0x0640, 0x0640, 0x0640, 0x0640};
static u16 nv_photosensor_a0[NUM_OF_COLOR] =
	{0x8EF8, 0x8EF8, 0x8EF8, 0x8EF8, 0x8EF8};
static u16 nv_photosensor_a1[NUM_OF_COLOR] =
	{0x6590, 0x6590, 0x6590, 0x6590, 0x6590};
static u16 nv_photosensor_a2[NUM_OF_COLOR] =
	{0x96D2, 0x96D2, 0x96D2, 0x96D2, 0x96D2};
static u16 nv_photosensor_a3[NUM_OF_COLOR] =
	{0x1EFA, 0x1EFA, 0x1EFA, 0x1EFA, 0x1EFA};
static u16 nv_photosensor_a4[NUM_OF_COLOR] =
	{0x1AF3, 0x1AF3, 0x1AF3, 0x1AF3, 0x1AF3};
static u16 nv_photosensor_b0[NUM_OF_COLOR] =
	{0x5A3C, 0x5A3C, 0x5A3C, 0x5A3C, 0x5A3C};
static u16 nv_photosensor_b1[NUM_OF_COLOR] =
	{0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
static u16 nv_photosensor_b2[NUM_OF_COLOR] =
	{0xD6D8, 0xD6D8, 0xD6D8, 0xD6D8, 0xD6D8};
static u16 nv_photosensor_b3[NUM_OF_COLOR] =
	{0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
static u16 nv_photosensor_b4[NUM_OF_COLOR] =
	{0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

static u8 nv_proximity_temp[] =
	{0x14, 0x28, 0xFF};

enum ps_als_power_ctrl_mode{
	PS_ALS_POWER_CTRL_OFF = 0,
	PS_ALS_POWER_CTRL_LOW,
	PS_ALS_POWER_CTRL_NORMAL,
	PS_ALS_POWER_CTRL_MAX
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
#define GAIN_VAL_MASK  (0xF)
	int           result;
	GENREAD_ARG   gene_read;
	unsigned char alsps_ctl, read_time;

	/* initialize the returning value */
	dev_val->time        = 0;
	dev_val->gain        = 0;
	dev_val->led_current = 0;

	/* get measure time parameter */
	gene_read.adr_reg = REG_MODECONTROL;
	gene_read.addr    = &read_time;
	gene_read.size    = sizeof(read_time);
	result = ps_als_driver_general_read(gene_read, client);
	if (result < 0) {
		SENSOR_ERR_LOG("ERROR! read data of itime.");
		return (result);
	}
	dev_val->time = read_time & 0xF;

	/* get gain parameter */
	gene_read.adr_reg = REG_ALSPSCONTROL;
	gene_read.addr    = &alsps_ctl;
	gene_read.size    = sizeof(alsps_ctl);
	result = ps_als_driver_general_read(gene_read, client);
	if (result < 0) {
		SENSOR_ERR_LOG("ERROR! read data of gain.");
		return (result);
	}
	dev_val->led_current = alsps_ctl & LEDBIT_MASK;
	dev_val->gain        = (alsps_ctl >> 2) & GAIN_VAL_MASK;

	return (0);
#undef LEDBIT_MASK
#undef GAIN_VAL_MASK
}

static int als_get_coef(CALC_DATA *calc_data, PS_ALS_DATA *ps_als)
{
	if (calc_data->ratio < nv_photosensor_th0[ps_als->color]) {
		calc_data->src = LIGHT_FLUORESCENT;
		calc_data->alpha = nv_photosensor_a0[ps_als->color];
		calc_data->beta = nv_photosensor_b0[ps_als->color];
	} else if ((calc_data->ratio >= nv_photosensor_th0[ps_als->color]) &&
		   (calc_data->ratio < nv_photosensor_th1[ps_als->color])) {
		calc_data->src = LIGHT_LED_BULB;
		calc_data->alpha = nv_photosensor_a1[ps_als->color];
		calc_data->beta = nv_photosensor_b1[ps_als->color];
	} else if ((calc_data->ratio >= nv_photosensor_th1[ps_als->color]) &&
		   (calc_data->ratio < nv_photosensor_th2[ps_als->color])) {
		calc_data->src = LIGHT_SOLAR;
		calc_data->alpha = nv_photosensor_a2[ps_als->color];
		calc_data->beta = nv_photosensor_b2[ps_als->color];
	} else if ((calc_data->ratio >= nv_photosensor_th2[ps_als->color]) &&
		   (calc_data->ratio < nv_photosensor_th3[ps_als->color])) {
		calc_data->src = LIGHT_INTERMEDIATE;
		calc_data->alpha = nv_photosensor_a3[ps_als->color];
		calc_data->beta = nv_photosensor_b3[ps_als->color];
	} else if (((calc_data->ratio >= nv_photosensor_th3[ps_als->color]) &&
		    (calc_data->ratio <  nv_photosensor_th4[ps_als->color])) ||
		    (calc_data->als_data0 <= ALS_D0_D1_TH && calc_data->als_data1 <= ALS_D0_D1_TH)) {
		calc_data->src = LIGHT_CANDESCENT;
		calc_data->alpha = nv_photosensor_a4[ps_als->color];
		calc_data->beta = nv_photosensor_b4[ps_als->color];
	} else {
		calc_data->src = LIGHT_SRC_MAX;
		return -1;
	}
	return 0;
}

static int als_calculate_data(READ_DATA_ALS_BUF data, PS_ALS_DATA *ps_als)
{
	int *i;
	int j, k;
	int prev;
	CALC_DATA *calc_data;
	CALC_DATA *calc_data_h;
	DEVICE_VAL *dev_val;
	int result;
	unsigned long temp = 0;

	i = &ps_als->als_val.index;
	calc_data = &ps_als->als_val.calc_data[*i];
	calc_data_h = ps_als->als_val.calc_data;
	dev_val = &ps_als->dev_val;

	/* set the value of measured als data */
	calc_data->als_data0  = data.als_data0;
	calc_data->als_data1  = data.als_data1;
	calc_data->gain_data0 = GAIN_TABLE[dev_val->gain].DATA0;
	calc_data->gain_data1 = GAIN_TABLE[dev_val->gain].DATA1;

	/* set max range */
	if (calc_data->gain_data0 == 0) {
		/* issue error value when gain is 0 */
		SENSOR_ERR_LOG("gain value error");
		return -1;
	}

	calc_data->d0 = calc_data->als_data0 / calc_data->gain_data0;
	calc_data->d1 = calc_data->als_data1 / calc_data->gain_data1;
	if (!calc_data->d0)
		calc_data->ratio = 1 * ALS_EXP_OF_NV_TH;
	else
		calc_data->ratio =
			(calc_data->d1 * ALS_EXP_OF_NV_TH) / calc_data->d0;

	result = als_get_coef(calc_data, ps_als);
	if (result) {
		prev = *i - 1;
		if (prev < 0)
			prev = NUM_OF_ALS_VAL - 1;
		calc_data->lux = ps_als->als_val.calc_data[prev].lux;
	} else {
		temp = ((calc_data->alpha * calc_data->d0) -
				  (calc_data->beta * calc_data->d1));
		calc_data->lux =  temp / ALS_EXP_OF_NV_COEF;
	}

	if (calc_data->lux > ALS_MAX_LUX_VAL)
		calc_data->lux = ALS_MAX_LUX_VAL;

	if ((calc_data->d0 > ALS_MAX_DX_VAL) || (calc_data->d1 > ALS_MAX_DX_VAL))
		calc_data->lux = ALS_MAX_LUX_VAL;

	if (!calc_data->d0)
		calc_data->lux = 0;

	SENSOR_D_LOG("lux=%lu d0=%lu d1=%lu g0=%u g1=%u rawd0=%u rawd1=%u ratio=%lu src=%d a=%u b=%u",
		calc_data->lux, calc_data->d0, calc_data->d1,
		calc_data->gain_data0, calc_data->gain_data1,
		calc_data->als_data0, calc_data->als_data1,
		calc_data->ratio, calc_data->src,
		calc_data->alpha, calc_data->beta);

	if (ps_als->als_val.ave_enable) {
		temp = 0;
		for (j = 0; j < NUM_OF_ALS_VAL; j++) {
			k = (*i + j + 1) % NUM_OF_ALS_VAL;
			temp += calc_data_h[k].lux * (j + 1);
		}
		ps_als->als_val.report_lux = temp / 10;
	} else {
		ps_als->als_val.report_lux = calc_data_h[0].lux;
	}

	ps_als->als_val.index++;
	ps_als->als_val.index %= NUM_OF_ALS_VAL;
	if ((!ps_als->als_val.ave_enable) &&
	    (ps_als->als_val.index == (NUM_OF_ALS_VAL - 1)))
		ps_als->als_val.ave_enable = true;

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
	PS_ALS_DATA *ps_als = i2c_get_clientdata(client);

	/* read start address */
	gene_read.adr_reg = REG_ALSDATA0;
	gene_read.addr    = (char *)&multi;
	gene_read.size    = sizeof(multi);

	/* block read */
	result = ps_als_driver_general_read(gene_read, client);
	if (result > 0) {
		data->als_data0 = CONVERT_TO_BE(multi.als_data0);
		data->als_data1 = CONVERT_TO_BE(multi.als_data1);
		result          = 0;
	} else {
		data->als_data0 = 0;
		data->als_data1 = 0;
	}
	if (unlikely(ps_als->imit.imit_flg)) {
		data->als_data0 = ps_als->imit.imit_d0;
		data->als_data1 = ps_als->imit.imit_d1;
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
	PS_ALS_DATA *ps_als = i2c_get_clientdata(client);
	char data[3];
	unsigned short ps_data;
	GENREAD_ARG   gene_read;

	/* read start address */
	gene_read.adr_reg = REG_PERSISTENCE;
	gene_read.addr    = data;
	gene_read.size    = 3;

	/* block read */
	result = ps_als_driver_general_read(gene_read, client);
	if (likely(result > 0)) {
		ps_data = data[1] | (data[2] << 8);
		ps_als->ps_val.ps_ctrl = data[0];
		ps_als->ps_val.ps_data = ps_data & PS_DATA_MASK;
		ps_als->ps_val.ps_flag = (ps_data & PS_FLAG_MASK) ? 1 : 0;
		result = 0;
	} else {
		ps_als->ps_val.ps_ctrl = 0;
		ps_als->ps_val.ps_data = 0;
		ps_als->ps_val.ps_flag = 0;
	}

	return (result);
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
	PS_ALS_DATA   *ps_als;
	long          get_timer;
	long          wait_sec;
	unsigned long wait_nsec;
	PWR_ST        pwr_st;

	SENSOR_D_LOG("start");
	read_data_buf.als_data0 = 0;
	read_data_buf.als_data1 = 0;
	ps_als = container_of(als_work, PS_ALS_DATA, als_work);

	/* read the state of sensor */
	result = ps_als_driver_read_power_state(&pwr_st, ps_als->client);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("power_state error ");
		goto exit;
	}
	/* check the state of sensor */
	if (unlikely(pwr_st.als_state == CTL_STANDBY)) {
		goto exit;
	}

	mutex_lock(&ps_als->als_data_lock);

	result = als_driver_read_data(&read_data_buf, ps_als->client);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("ERROR! read data");
		goto exit_lock;
	}
	/* read value from device */
	result = get_from_device(&ps_als->dev_val, ps_als->client);
	if (result < 0) {
		SENSOR_ERR_LOG("ERROR! read data from device.");
		goto exit_lock;
	}
	result = als_calculate_data(read_data_buf, ps_als);
	if (result) {
		SENSOR_ERR_LOG("ERROR! calculate als data.");
		goto exit_lock;
	}
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
	sensor_report_data(SENSOR_LIGHT, ps_als->als_val.report_lux);
#else
	als_sensor_report_event(&ps_als->als_info, ps_als->als_val.report_lux, ps_als->als_en_first);
#endif	/*CONFIG_USE_MICON_SOFT_STRUCTURE*/
	if (unlikely(ps_als->als_en_first))
		ps_als->als_en_first = false;


	/* the setting value from application */
	get_timer = ps_als->delay_time;
	/* 125ms(8Hz) at least */
	wait_sec  = (get_timer / SM_TIME_UNIT);
	wait_nsec = ((get_timer - (wait_sec * SM_TIME_UNIT)) * MN_TIME_UNIT);
	result = hrtimer_start(&ps_als->timer, ktime_set(wait_sec, wait_nsec), HRTIMER_MODE_REL);
	if (result != 0) {
		SENSOR_ERR_LOG("can't start timer");
		goto exit_lock;
	}
exit_lock:
	mutex_unlock(&ps_als->als_data_lock);
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
    PS_ALS_DATA *ps_als;
    int         result;

    ps_als = container_of(timer, PS_ALS_DATA, timer);
    result = queue_work(rohm_workqueue, &ps_als->als_work);
    if (result == 0) {
        SENSOR_ERR_LOG("can't register que.");
        SENSOR_ERR_LOG("result = 0x%x", result);
    }

    return (HRTIMER_NORESTART);
}

static int ps_control_interrupt(PS_ALS_DATA *ps_als, char mode)
{
	int result;
	char interrupt;

	if ((ps_als->ps_val.ps_intr & PS_INT_MODE_MASK) == mode)
		return 0;
	interrupt = ps_als->ps_val.ps_intr & ~PS_INT_MODE_MASK;
	interrupt |= mode;
	result = ps_als_i2c_write_byte_data(ps_als->client, REG_INTERRUPT, interrupt);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("Failed to change interrupt");
		return result;
	}
	SENSOR_D_LOG("interrupt setting changed. 0x%x --> 0x%x",
		ps_als->ps_val.ps_intr,
		interrupt);
	ps_als->ps_val.ps_intr = interrupt;
	return 0;
}

static int ps_check_status(PS_ALS_DATA *ps_als)
{
	int tmp;

	if (ps_als->ps_val.ps_data > ps_als->init_data.psth_upper)
		tmp = PROX_STATUS_NEAR;
	else if (ps_als->ps_val.ps_data < ps_als->init_data.psth_low)
		tmp = PROX_STATUS_FAR;
	else
		return 0;

	if (ps_als->ps_det != tmp) {
		ps_als->ps_det = tmp;
		return 1;
	} else {
		return 0;
	}
}

static void ps_sensor_report_event_proc(PS_ALS_DATA *ps_als, uint32_t detect)
{
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
	long wake_lock_time;
	struct ps_sensor_info *info = &ps_als->ps_info;
	SENSOR_D_LOG("start");

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
	wake_lock_timeout(&ps_als->ps_info.wake_lock, wake_lock_time);
#else
	SENSOR_D_LOG("start");
	ps_sensor_report_event(&ps_als->ps_info, detect);
#endif	/*CONFIG_USE_MICON_SOFT_STRUCTURE*/
	SENSOR_D_LOG("end");
}

/******************************************************************************
 * NAME       : ps_irq_handler
 * FUNCTION   : interruption function (irq)
 * REMARKS    :
 *****************************************************************************/
static irqreturn_t ps_irq_handler(int irq, void *dev_id)
{
	PS_ALS_DATA *ps_als;
	int         result;
	PWR_ST        pwr_st;
	GENREAD_ARG   gene_data;
	unsigned char ps_intr;

	SENSOR_D_LOG("start");
	ps_als = dev_id;

	/* clear interrupt flag */
	gene_data.adr_reg = REG_INTERRUPT;
	gene_data.addr    = &ps_intr;
	gene_data.size    = sizeof(ps_intr);
	result = ps_als_driver_general_read(gene_data, ps_als->client);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("general read can't execute ");
		SENSOR_ERR_LOG("can't read interrupt register ");
		goto exit;
	}

	/* read the state of sensor */
	result = ps_als_driver_read_power_state(&pwr_st, ps_als->client);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("power_state error ");
		goto exit;
	}
	/* check the state of sensor */
	if (unlikely(pwr_st.ps_state == CTL_STANDBY))
		goto exit;

	mutex_lock(&ps_als->ps_data_lock);
	ps_als->ps_val.ps_intr = ps_intr;

	result = ps_driver_read_data(ps_als->client);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("ERROR! read data");
		goto exit_lock;
	}
	SENSOR_N_LOG("ps_intr=0x%x ps_ctrl=0x%x ps_data=%d ps_flag=%d psth=%d pstl=%d",
							ps_als->ps_val.ps_intr,
							ps_als->ps_val.ps_ctrl,
							ps_als->ps_val.ps_data,
							ps_als->ps_val.ps_flag,
							ps_als->init_data.psth_upper,
							ps_als->init_data.psth_low
	);

	if (unlikely((ps_als->ps_val.ps_ctrl & IR_FLAG_MASK) == IR_VERY_STRONG)) {
		result = ps_control_interrupt(ps_als, PS_THH_BOTH_OUTSIDE);
		if (unlikely(result)) {
			SENSOR_ERR_LOG("Failed to change mode[%d]", result);
			goto exit_lock;
		}
	} else {
		if (ps_check_status(ps_als) || !ps_als->int_flg) {
			ps_sensor_report_event_proc(ps_als, ps_als->ps_det);
		}

		if ((ps_als->ps_val.ps_intr & PS_INT_MODE_MASK) == PS_THH_BOTH_WINDOW) {
			if ((ps_als->ps_val.ps_flag  && ps_als->ps_det == PROX_STATUS_FAR) ||
			    (!ps_als->ps_val.ps_flag && ps_als->ps_det == PROX_STATUS_NEAR)) {
				SENSOR_N_LOG("strange status detected."
					"change int mode to BOTH_OUTSIDE. flag[%d] det[%d]",
					ps_als->ps_val.ps_flag, ps_als->ps_det);
				result = ps_control_interrupt(ps_als, PS_THH_BOTH_OUTSIDE);
				if (unlikely(result)) {
					SENSOR_ERR_LOG("Failed to change mode[%d]", result);
					goto exit_lock;
				}
			}
		} else {
			result = ps_control_interrupt(ps_als, PS_THH_BOTH_WINDOW);
			if (unlikely(result)) {
				SENSOR_ERR_LOG("Failed to change mode[%d]", result);
				goto exit_lock;
			}
		}
	}
exit_lock:
	mutex_unlock(&ps_als->ps_data_lock);
exit:
	ps_als->int_flg = true;
	SENSOR_D_LOG("end");
	return (IRQ_HANDLED);
}

static void ps_als_set_errinfo(PS_ALS_DATA *ps_als, uint32_t errinfo)
{
	static DEFINE_MUTEX(errinfo_write_lock);
	SENSOR_ERR_LOG("start errinfo[0x%x] ps_als->errinfo[0x%x]", errinfo, ps_als->errinfo);

	mutex_lock(&errinfo_write_lock);
	SENSOR_N_LOG("some errors occured. errinfo[0x%x]", errinfo);
	ps_als->errinfo |= errinfo;
	mutex_unlock(&errinfo_write_lock);

	if (errinfo & ERRINFO_RECOVERY_FAIL_MASK) {
		ps_als->ps_det = PROX_STATUS_FAR;
		ps_sensor_report_event_proc(ps_als, ps_als->ps_det);
	}

	SENSOR_V_LOG("end");
}

static bool ps_als_is_device_dead(PS_ALS_DATA *ps_als)
{
	return ((ps_als->errinfo & ERRINFO_RECOVERY_FAIL_MASK) != 0) ? true : false;
}

static int ps_als_confirm_regs(PS_ALS_DATA *ps_als)
{
	RPR0521_REGS	regs;
	int		ret = 0;
	unsigned char	work;
	u16		ps_th;
	u16		ps_tl;
	u16		ps_offset;

	SENSOR_V_LOG("start");

	ret = ps_als_i2c_read(ps_als->client, REG_SYSTEMCONTROL,
		&regs.systemcontrol, 4);
	if (ret < 0) {
		SENSOR_ERR_LOG("i2c transfer failed.");
		return -1;
	}

	if (regs.systemcontrol != 0x0A) {
		SENSOR_ERR_LOG("unexpected systemcontrol[0x%x]. expect[0x%x]", regs.systemcontrol, 0x0A);
		return -1;
	}

	work = PS_ALS_SET_MODE_CONTROL;
	if (ps_als->power_data.power_als)
		work |= PWRON_ALS_EN;
	if (ps_als->power_data.power_ps)
		work |= PWRON_PS_EN;

	if (regs.modecontrol != work) {
		SENSOR_ERR_LOG("unexpected modecontrol[0x%x]. expect[0x%x]", regs.modecontrol, work);
		return -1;
	}
	if (regs.alspscontrol != PS_ALS_SET_ALSPS_CONTROL) {
		SENSOR_ERR_LOG("unexpected alspscontrol[0x%x]. expect[0x%x]", regs.alspscontrol, PS_ALS_SET_ALSPS_CONTROL);
		return -1;
	}
	if (regs.persistence != PS_ALS_SET_INTR_PERSIST) {
		SENSOR_ERR_LOG("unexpected persistence[0x%x]. expect[0x%x]", regs.persistence, PS_ALS_SET_INTR_PERSIST);
		return -1;
	}

	if (ps_als->power_data.power_ps) {
		ret = ps_als_i2c_read(ps_als->client, REG_PSTH_LSB,
			&regs.psth_lsb, 4);
		if (ret < 0) {
			SENSOR_ERR_LOG("i2c transfer failed.");
			return -1;
		}
		ret = ps_als_i2c_read(ps_als->client, REG_PSOFFSET_LSB,
			&regs.psoffset_lsb, 2);
		if (ret < 0) {
			SENSOR_ERR_LOG("i2c transfer failed.");
			return -1;
		}

		mutex_lock(&ps_als->ps_data_lock);
		ps_th = ps_als->init_data.psth_upper;
		ps_tl = ps_als->init_data.psth_low;
		ps_offset = ps_als->init_data.ps_offset;
		mutex_unlock(&ps_als->ps_data_lock);

		if ((regs.psth_lsb != (u8)(ps_th >> 0)) ||
		    (regs.psth_msb != (u8)(ps_th >> 8)) ||
		    (regs.pstl_lsb != (u8)(ps_tl >> 0)) ||
		    (regs.pstl_msb != (u8)(ps_tl >> 8)) ||
		    (regs.psoffset_lsb != (u8)(ps_offset >> 0)) ||
		    (regs.psoffset_msb != (u8)(ps_offset >> 8))) {
			SENSOR_ERR_LOG("unexpected threshold or offsets");
			return -1;
		}
	}

	SENSOR_V_LOG("end");
	return 0;
}

static int als_enable_proc(PS_ALS_DATA *ps_als, unsigned char enable);
static int ps_enable_proc(PS_ALS_DATA *ps_als, unsigned char enable);
static int ps_als_recover_state(PS_ALS_DATA *ps_als)
{
	int result;

	SENSOR_V_LOG("start");

	mutex_lock(&ps_als->ps_data_lock);
	mutex_lock(&ps_als->als_data_lock);
	result = ps_als_driver_init(&ps_als->init_data, ps_als->client);
	mutex_unlock(&ps_als->als_data_lock);
	mutex_unlock(&ps_als->ps_data_lock);
	if (result < 0) {
		SENSOR_ERR_LOG("fail ps_als_driver_init");
		return result;
	}
	if (ps_als->power_data.power_als) {
		result = als_enable_proc(ps_als, false);
		if (result < 0) {
			SENSOR_ERR_LOG("fail als_enable_proc");
			return result;
		}
		result = als_enable_proc(ps_als, true);
		if (result < 0) {
			SENSOR_ERR_LOG("fail als_enable_proc");
			return result;
		}
	}
	if (ps_als->power_data.power_ps) {
		result = ps_enable_proc(ps_als, false);
		if (result < 0) {
			SENSOR_ERR_LOG("fail ps_enable_proc");
			return result;
		}
		result = ps_enable_proc(ps_als, true);
		if (result < 0) {
			SENSOR_ERR_LOG("fail ps_enable_proc");
			return result;
		}
	}

	SENSOR_V_LOG("end");
	return 0;
}

static int ps_als_recovery(PS_ALS_DATA *ps_als)
{
#define RECOVERY_RETRY_MAX	3
	int retries = RECOVERY_RETRY_MAX;
	int result;
	SENSOR_N_LOG("start");

	while (retries--) {
		ps_als_set_errinfo(ps_als, ERRINFO_SOFT_RESET_OCCURRED);
		result = ps_als_driver_reset(ps_als->client);
		if (result < 0) {
			continue;
		}
		msleep(10);

		result = ps_als_i2c_read_byte_data(ps_als->client, REG_SYSTEMCONTROL);
		if (result != 0x0A) {
			continue;
		}

		result = ps_als_recover_state(ps_als);
		return result;
	}

	SENSOR_ERR_LOG("recovery failed");
	ps_als_set_errinfo(ps_als, ERRINFO_RECOVERY_FAILED);
	return -1;
#undef RECOVERY_RETRY_MAX
}

static void ps_als_check_error(PS_ALS_DATA *ps_als)
{
	int ret;

	SENSOR_V_LOG("start");
	ret = ps_als_confirm_regs(ps_als);
	if (ret < 0) {
		ret = ps_als_recovery(ps_als);
		if (ret < 0) {
			SENSOR_ERR_LOG("failed ps_als_recovery");
		}
	}

	SENSOR_V_LOG("end");
	return;
}

static int ps_get_temperature(PS_ALS_DATA *ps_als, int *temperature)
{
	int	ret = -1;
#ifdef TEMPERATURE_AVAILABLE
	union power_supply_propval propval = {20,};
	SENSOR_V_LOG("start");

	if (!ps_als->psy) {
		ps_als->psy = power_supply_get_by_name("hkadc");
	}
	if (ps_als->psy) {
		ret = ps_als->psy->get_property(ps_als->psy, POWER_SUPPLY_PROP_OEM_CAMERA_THERM, &propval);
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

static void ps_get_new_thresholds(PS_ALS_DATA *ps_als,
	u16 *ps_th_ret, u16 *ps_tl_ret, u16 *ps_offset_ret)
{
	int	result;
	u16	ps_th;
	u16	ps_tl;
	u16	ps_offset;
	u16	ps_th_base;
	u16	ps_tl_base;
	int	temperature = 20;
	int	coef_1;
	int	coef_2;
	int	coef_temp;

	SENSOR_V_LOG("start");

	mutex_lock(&ps_als->ps_data_lock);
	ps_th_base = nv_proximity_detect[ps_als->color];
	ps_tl_base = nv_proximity_no_detect[ps_als->color];
	ps_offset = nv_proximity_offset[ps_als->color];
	coef_1 = nv_proximity_temp[0];
	coef_2 = nv_proximity_temp[1];
	mutex_unlock(&ps_als->ps_data_lock);
	result = ps_get_temperature(ps_als, &temperature);

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

			ps_th = (u16)((long)ps_th_base + coef_1 - ((coef_temp * coef_1) / coef_2));
			ps_tl = (u16)((long)ps_tl_base + coef_1 - ((coef_temp * coef_1) / coef_2));

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

	*ps_th_ret = ps_th;
	*ps_tl_ret = ps_tl;
	*ps_offset_ret = ps_offset;

	SENSOR_V_LOG("end ps_th[%d] ps_tl[%d] ps_offset[%d]", ps_th, ps_tl, ps_offset);
	return;
}

static bool ps_update_thresholds(PS_ALS_DATA *ps_als)
{
	bool	changed;
	u16	ps_th;
	u16	ps_tl;
	u16	ps_offset;

	SENSOR_V_LOG("start");

	ps_get_new_thresholds(ps_als, &ps_th, &ps_tl, &ps_offset);

	mutex_lock(&ps_als->ps_data_lock);
	if (ps_th != ps_als->init_data.psth_upper || ps_tl != ps_als->init_data.psth_low
		|| ps_offset != ps_als->init_data.ps_offset) {
		ps_als->init_data.psth_upper = ps_th;
		ps_als->init_data.psth_low = ps_tl;
		ps_als->init_data.ps_offset = ps_offset;
		changed = true;
	} else {
		changed = false;
	}
	mutex_unlock(&ps_als->ps_data_lock);

	SENSOR_V_LOG("end");
	return changed;
}

static int ps_send_config(PS_ALS_DATA *ps_als)
{
	int		result;
	RPR0521_REGS	regs;

	SENSOR_V_LOG("start");
	mutex_lock(&ps_als->ps_data_lock);
	regs.psth_lsb = ps_als->init_data.psth_upper >> 0;
	regs.psth_msb = ps_als->init_data.psth_upper >> 8;
	regs.pstl_lsb = ps_als->init_data.psth_low >> 0;
	regs.pstl_msb = ps_als->init_data.psth_low >> 8;
	regs.psoffset_lsb = ps_als->init_data.ps_offset >> 0;
	regs.psoffset_msb = ps_als->init_data.ps_offset >> 8;
	mutex_unlock(&ps_als->ps_data_lock);

	result = ps_als_i2c_write(ps_als->client, REG_PSTH, &regs.psth_lsb, 4);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("failed to set thresholds");
		return -1;
	}
	result = ps_als_i2c_write(ps_als->client, REG_PSOFFSET, &regs.psoffset_lsb, 2);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("failed to set offset");
		return -1;
	}
	result = ps_als_i2c_write_byte_data(ps_als->client, REG_INTERRUPT, PS_ALS_SET_INTR);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("failed to set interrupt reg");
		return -1;
	}

	SENSOR_V_LOG("end");
	return 0;
}

static void ps_check_thresholds_update(PS_ALS_DATA *ps_als)
{
	SENSOR_V_LOG("start");
	if (!ps_als->power_data.power_ps) {
		return;
	}

	if (!ps_update_thresholds(ps_als)) {
		return;
	}

	disable_irq(ps_als->client->irq);
	ps_send_config(ps_als);
	enable_irq(ps_als->client->irq);

	SENSOR_V_LOG("end");
	return;
}

static void ps_als_schedule_monitor(PS_ALS_DATA *ps_als)
{
	SENSOR_V_LOG("start");
	queue_delayed_work(rohm_workqueue, &ps_als->monitor_dwork, msecs_to_jiffies(5000));
}

static void ps_als_monitor_func(struct work_struct *work)
{
	PS_ALS_DATA   *ps_als;

	SENSOR_D_LOG("start");
	ps_als = container_of(work, PS_ALS_DATA, monitor_dwork.work);
	mutex_lock(&ps_als->control_lock);

	if ((ps_als->power_data.power_ps || ps_als->power_data.power_als) &&
	    !ps_als_is_device_dead(ps_als)) {
		ps_check_thresholds_update(ps_als);
		ps_als_check_error(ps_als);
		ps_als_schedule_monitor(ps_als);
	}

	mutex_unlock(&ps_als->control_lock);
	SENSOR_D_LOG("end");
}

static int ps_als_power_ctrl(PS_ALS_DATA *ps_als, enum ps_als_power_ctrl_mode request_mode);
static int als_enable_proc(PS_ALS_DATA *ps_als, unsigned char enable)
{
	int result;
	unsigned char power_set;
	long get_timer;
	long wait_sec;
	unsigned long wait_nsec;

	SENSOR_D_LOG("start. enable=%d", enable);
	if (unlikely(ps_als->power_data.power_als == enable)) {
		SENSOR_D_LOG("same status. skip");
		return 0;
	}

	if (enable) {
		result = ps_als_power_ctrl(ps_als, PS_ALS_POWER_CTRL_NORMAL);
		if(unlikely(result < 0)) {
			SENSOR_ERR_LOG("failed to ps_als_power_ctrl");
			return result;
		}
	}

	result = ps_als_i2c_read_byte_data(ps_als->client, REG_MODECONTROL);
	if (unlikely(result < 0)) {
		/* i2c communication error */
		return result;
	}

	power_set  = (unsigned char)(result & ~INIT_ALS_MODE_MASK);

	if (enable) {
		power_set |= PWRON_ALS_EN;
	} else {
		power_set &= ~PWRON_ALS_EN;
	}

	result = ps_als_i2c_write_byte_data(ps_als->client,
					   REG_MODECONTROL, power_set);
	if (unlikely(result < 0)) {
		/* i2c communication error */
		return result;
	}

	if (enable) {
		mutex_lock(&ps_als->als_data_lock);
		ps_als->als_val.index = 0;
		ps_als->als_val.ave_enable = false;
		ps_als->als_en_first = true;
		memset(ps_als->als_val.calc_data, 0,
			sizeof(CALC_DATA) * NUM_OF_ALS_VAL);
		mutex_unlock(&ps_als->als_data_lock);

		/* the setting value from application */
		get_timer = ALS_ON_DELAY_MS;
		/* 125ms(8Hz) at least */
		wait_sec  = (get_timer / SM_TIME_UNIT);
		wait_nsec = ((get_timer - (wait_sec * SM_TIME_UNIT)) * MN_TIME_UNIT);
		result = hrtimer_start(&ps_als->timer,
				       ktime_set(wait_sec, wait_nsec),
				       HRTIMER_MODE_REL);
		if (unlikely(result)) {
			SENSOR_ERR_LOG("can't start timer");
			return result;
		}

		ps_als_schedule_monitor(ps_als);
	} else {
		hrtimer_cancel(&ps_als->timer);
		cancel_work_sync(&ps_als->als_work);
		hrtimer_cancel(&ps_als->timer);

		if (!ps_als->power_data.power_ps) {
			result = ps_als_power_ctrl(ps_als, PS_ALS_POWER_CTRL_LOW);
			if(unlikely(result < 0)) {
				SENSOR_ERR_LOG("failed to ps_als_power_ctrl");
				return result;
			}
		}
	}

	ps_als->power_data.power_als = enable;

	SENSOR_D_LOG("end. enable=%d", enable);
	return 0;
}

static int ps_enable_proc(PS_ALS_DATA *ps_als, unsigned char enable)
{
	int result;
	unsigned char power_set;

	SENSOR_D_LOG("start. enable=%d", enable);
	SENSOR_N_LOG("enable=%d", enable);
	if (unlikely(ps_als->power_data.power_ps == enable)) {
		SENSOR_D_LOG("same status. skip");
		return 0;
	}

	if (enable) {
		result = ps_als_power_ctrl(ps_als, PS_ALS_POWER_CTRL_NORMAL);
		if(unlikely(result < 0)) {
			SENSOR_ERR_LOG("failed to ps_als_power_ctrl");
			return result;
		}
		ps_update_thresholds(ps_als);
		result = ps_send_config(ps_als);
		if (unlikely(result < 0)) {
			SENSOR_ERR_LOG("failed to send config");
			return result;
		}
	}

	result = ps_als_i2c_read_byte_data(ps_als->client, REG_MODECONTROL);
	if (unlikely(result < 0)) {
		/* i2c communication error */
		return result;
	}

	if (enable) {
		enable_irq(ps_als->client->irq);
	}

	power_set  = (unsigned char)(result & ~INIT_PS_MODE_MASK);

	if (enable)
		power_set |= PWRON_PS_EN;
	else
		power_set &= ~PWRON_PS_EN;

	result = ps_als_i2c_write_byte_data(ps_als->client,
					   REG_MODECONTROL, power_set);
	if (unlikely(result < 0)) {
		/* i2c communication error */
		if (enable) {
			disable_irq(ps_als->client->irq);
		}
		return result;
	}

	ps_als->int_flg = false;

	if (enable) {
		ps_als_schedule_monitor(ps_als);
	} else {
		disable_irq(ps_als->client->irq);

		ps_als->ps_det = PROX_STATUS_FAR;
		ps_sensor_report_event_proc(ps_als, ps_als->ps_det);

		if (!ps_als->power_data.power_als) {
			result = ps_als_power_ctrl(ps_als, PS_ALS_POWER_CTRL_LOW);
			if(unlikely(result < 0)) {
				SENSOR_ERR_LOG("failed to ps_als_power_ctrl");
				return result;
			}
		}
	}

	ps_als->power_data.power_ps = enable;

	SENSOR_D_LOG("end. enable=%d", enable);
	return 0;
}

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t als_val_show(char *buf)
{
	PS_ALS_DATA *ps_als = i2c_get_clientdata(client_rpr0521);
	ssize_t count = 0;

	SENSOR_D_LOG("start");
	mutex_lock(&ps_als->als_data_lock);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"%d\n", ps_als->als_val.report_lux);

	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->als_data_lock);
	return count;
}
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int als_val_show(struct sensor_api_info *sai, char *buf)
{
	PS_ALS_DATA *ps_als;
	int count = 0;

	SENSOR_D_LOG("start");
	ps_als = container_of(sai, PS_ALS_DATA, als_info.sai);
	mutex_lock(&ps_als->als_data_lock);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"%d\n", ps_als->als_val.report_lux);

	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->als_data_lock);
	return count;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE */


#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t ps_val_show(char *buf)
{
	PS_ALS_DATA *ps_als = i2c_get_clientdata(client_rpr0521);
	ssize_t count = 0;

	SENSOR_D_LOG("start");
	mutex_lock(&ps_als->ps_data_lock);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"%d\n", ps_als->ps_det);

	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->ps_data_lock);
	return count;
}
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int ps_val_show(struct sensor_api_info *sai, char *buf)
{
	PS_ALS_DATA *ps_als;
	int count = 0;

	SENSOR_D_LOG("start");
	ps_als = container_of(sai, PS_ALS_DATA, ps_info.sai);
	mutex_lock(&ps_als->ps_data_lock);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"%d\n", ps_als->ps_det);

	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->ps_data_lock);
	return count;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE */

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t als_status_show(char *buf)
{
	PS_ALS_DATA *ps_als;
	ssize_t count = 0;
	int i;
	CALC_DATA *calc_data_h;

	SENSOR_D_LOG("start");
	ps_als = i2c_get_clientdata(client_rpr0521);
	mutex_lock(&ps_als->als_data_lock);

	calc_data_h = ps_als->als_val.calc_data;

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"report_lux = %d\n", ps_als->als_val.report_lux);
	if (count >= PAGE_SIZE)
		goto fail_exit;

	i = ps_als->als_val.index - 1;
	if (i < 0)
		i = NUM_OF_ALS_VAL - 1;

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"index[%d], ave_enable[%d]\n", i, ps_als->als_val.ave_enable);
	if (count >= PAGE_SIZE)
		goto fail_exit;

	for (i = 0; i < NUM_OF_ALS_VAL; i++) {
		count += scnprintf(buf + count, PAGE_SIZE - count,
			"  calc_data[%d].lux = %ld\n", i, calc_data_h[i].lux);
		if (count >= PAGE_SIZE)
			goto fail_exit;

		count += scnprintf(buf + count, PAGE_SIZE - count,
			"  calc_data[%d].d0 = %ld\n", i, calc_data_h[i].d0);
		if (count >= PAGE_SIZE)
			goto fail_exit;

		count += scnprintf(buf + count, PAGE_SIZE - count,
			"  calc_data[%d].d1 = %ld\n", i, calc_data_h[i].d1);
		if (count >= PAGE_SIZE)
			goto fail_exit;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].gain_data0 = %d\n",
				   i, calc_data_h[i].gain_data0);
		if (count >= PAGE_SIZE)
			goto fail_exit;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].gain_data1 = %d\n",
				   i, calc_data_h[i].gain_data1);
		if (count >= PAGE_SIZE)
			goto fail_exit;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].als_data0 = %d\n",
				   i, calc_data_h[i].als_data0);
		if (count >= PAGE_SIZE)
			goto fail_exit;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].als_data1 = %d\n",
				   i, calc_data_h[i].als_data1);
		if (count >= PAGE_SIZE)
			goto fail_exit;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].ratio = %ld\n",
				   i, calc_data_h[i].ratio);
		if (count >= PAGE_SIZE)
			goto fail_exit;

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
		if (count >= PAGE_SIZE)
			goto fail_exit;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].alpha = 0x%04x\n",
				   i, calc_data_h[i].alpha);
		if (count >= PAGE_SIZE)
			goto fail_exit;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].beta = 0x%04x\n",
				   i, calc_data_h[i].beta);
		if (count >= PAGE_SIZE)
			goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
						"--------\n");
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
		"errinfo = %d (detail:0x%08x)\n",
		(ps_als->errinfo & ERRINFO_RECOVERY_FAIL_MASK) ? 3 :
		(ps_als->errinfo & ERRINFO_SOFT_RESET_OCCURRED) ? 2 :
		(ps_als->errinfo & ERRINFO_I2C_RESET_OCCURRED) ? 1 : 0,
		ps_als->errinfo);
	if (count >= PAGE_SIZE)
		goto fail_exit;

	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->als_data_lock);
	return count;
fail_exit:
	SENSOR_D_LOG("end - fail");
	mutex_unlock(&ps_als->als_data_lock);
	return PAGE_SIZE - 1;
}
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int als_status_show(struct sensor_api_info *sai, char *buf)
{
	PS_ALS_DATA *ps_als;
	int count = 0;
	int i;
	CALC_DATA *calc_data_h;

	SENSOR_D_LOG("start");
	ps_als = container_of(sai, PS_ALS_DATA, als_info.sai);
	mutex_lock(&ps_als->als_data_lock);

	calc_data_h = ps_als->als_val.calc_data;

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"report_lux = %d\n", ps_als->als_val.report_lux);
	if (count >= PAGE_SIZE)
		goto fail_exit;

	i = ps_als->als_val.index - 1;
	if (i < 0)
		i = NUM_OF_ALS_VAL - 1;

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"index[%d], ave_enable[%d]\n", i, ps_als->als_val.ave_enable);
	if (count >= PAGE_SIZE)
		goto fail_exit;

	for (i = 0; i < NUM_OF_ALS_VAL; i++) {
		count += scnprintf(buf + count, PAGE_SIZE - count,
			"  calc_data[%d].lux = %ld\n", i, calc_data_h[i].lux);
		if (count >= PAGE_SIZE)
			goto fail_exit;

		count += scnprintf(buf + count, PAGE_SIZE - count,
			"  calc_data[%d].d0 = %ld\n", i, calc_data_h[i].d0);
		if (count >= PAGE_SIZE)
			goto fail_exit;

		count += scnprintf(buf + count, PAGE_SIZE - count,
			"  calc_data[%d].d1 = %ld\n", i, calc_data_h[i].d1);
		if (count >= PAGE_SIZE)
			goto fail_exit;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].gain_data0 = %d\n",
				   i, calc_data_h[i].gain_data0);
		if (count >= PAGE_SIZE)
			goto fail_exit;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].gain_data1 = %d\n",
				   i, calc_data_h[i].gain_data1);
		if (count >= PAGE_SIZE)
			goto fail_exit;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].als_data0 = %d\n",
				   i, calc_data_h[i].als_data0);
		if (count >= PAGE_SIZE)
			goto fail_exit;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].als_data1 = %d\n",
				   i, calc_data_h[i].als_data1);
		if (count >= PAGE_SIZE)
			goto fail_exit;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].ratio = %ld\n",
				   i, calc_data_h[i].ratio);
		if (count >= PAGE_SIZE)
			goto fail_exit;

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
		if (count >= PAGE_SIZE)
			goto fail_exit;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].alpha = 0x%04x\n",
				   i, calc_data_h[i].alpha);
		if (count >= PAGE_SIZE)
			goto fail_exit;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].beta = 0x%04x\n",
				   i, calc_data_h[i].beta);
		if (count >= PAGE_SIZE)
			goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
						"--------\n");
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
		"errinfo = %d (detail:0x%08x)\n",
		(ps_als->errinfo & ERRINFO_RECOVERY_FAIL_MASK) ? 3 :
		(ps_als->errinfo & ERRINFO_SOFT_RESET_OCCURRED) ? 2 :
		(ps_als->errinfo & ERRINFO_I2C_RESET_OCCURRED) ? 1 : 0,
		ps_als->errinfo);
	if (count >= PAGE_SIZE)
		goto fail_exit;

	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->als_data_lock);
	return count;
fail_exit:
	SENSOR_D_LOG("end - fail");
	mutex_unlock(&ps_als->als_data_lock);
	return PAGE_SIZE - 1;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE */

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t ps_status_show(char *buf)
{
	PS_ALS_DATA *ps_als;
	ssize_t count = 0;
	unsigned short ps_data;
	GENREAD_ARG gene_read;
	int result;

	SENSOR_D_LOG("start");
	ps_als = i2c_get_clientdata(client_rpr0521);
	mutex_lock(&ps_als->control_lock);
	mutex_lock(&ps_als->ps_data_lock);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps detect = %d\n", ps_als->ps_det);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps interrupt flg = %d\n", ps_als->int_flg);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps count = %d\n", ps_als->ps_val.ps_data);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps int flag = %d\n", ps_als->ps_val.ps_flag);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
						"--------\n");
	if (count >= PAGE_SIZE)
		goto fail_exit;

	/* read start address */
	gene_read.adr_reg = REG_PSDATA;
	gene_read.addr    = (char *)&ps_data;
	gene_read.size    = sizeof(ps_data);

	/* block read */
	result = ps_als_driver_general_read(gene_read, ps_als->client);
	if (result > 0) {
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"current ps count = %d, int flag = %d\n",
				CONVERT_TO_BE(ps_data) & PS_DATA_MASK,
				(CONVERT_TO_BE(ps_data) & PS_FLAG_MASK) ? 1 : 0);
	} else {
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"current ps count = 0, int flag = 0\n");
	}
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps_th = %d\n", ps_als->init_data.psth_upper);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps_tl = %d\n", ps_als->init_data.psth_low);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps_offset = %d\n", ps_als->init_data.ps_offset);
	if (count >= PAGE_SIZE)
		goto fail_exit;

	count += scnprintf(buf + count, PAGE_SIZE - count,
						"--------\n");
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
		"errinfo = %d (detail:0x%08x)\n",
		(ps_als->errinfo & ERRINFO_RECOVERY_FAIL_MASK) ? 3 :
		(ps_als->errinfo & ERRINFO_SOFT_RESET_OCCURRED) ? 2 :
		(ps_als->errinfo & ERRINFO_I2C_RESET_OCCURRED) ? 1 : 0,
		ps_als->errinfo);
	if (count >= PAGE_SIZE)
		goto fail_exit;

	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->ps_data_lock);
	mutex_unlock(&ps_als->control_lock);
	return count;
fail_exit:
	SENSOR_D_LOG("end - fail");
	mutex_unlock(&ps_als->ps_data_lock);
	mutex_unlock(&ps_als->control_lock);
	return PAGE_SIZE - 1;
}
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int ps_status_show(struct sensor_api_info *sai, char *buf)
{
	PS_ALS_DATA *ps_als;
	int count = 0;
	unsigned short ps_data;
	GENREAD_ARG gene_read;
	int result;

	SENSOR_D_LOG("start");
	ps_als = container_of(sai, PS_ALS_DATA, ps_info.sai);
	mutex_lock(&ps_als->control_lock);
	mutex_lock(&ps_als->ps_data_lock);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps detect = %d\n", ps_als->ps_det);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps interrupt flg = %d\n", ps_als->int_flg);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps count = %d\n", ps_als->ps_val.ps_data);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps int flag = %d\n", ps_als->ps_val.ps_flag);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
						"--------\n");
	if (count >= PAGE_SIZE)
		goto fail_exit;

	/* read start address */
	gene_read.adr_reg = REG_PSDATA;
	gene_read.addr    = (char *)&ps_data;
	gene_read.size    = sizeof(ps_data);

	/* block read */
	result = ps_als_driver_general_read(gene_read, ps_als->client);
	if (result > 0) {
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"current ps count = %d, int flag = %d\n",
				CONVERT_TO_BE(ps_data) & PS_DATA_MASK,
				(CONVERT_TO_BE(ps_data) & PS_FLAG_MASK) ? 1 : 0);
	} else {
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"current ps count = 0, int flag = 0\n");
	}
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps_th = %d\n", ps_als->init_data.psth_upper);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps_tl = %d\n", ps_als->init_data.psth_low);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps_offset = %d\n", ps_als->init_data.ps_offset);
	if (count >= PAGE_SIZE)
		goto fail_exit;

	count += scnprintf(buf + count, PAGE_SIZE - count,
						"--------\n");
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
		"errinfo = %d (detail:0x%08x)\n",
		(ps_als->errinfo & ERRINFO_RECOVERY_FAIL_MASK) ? 3 :
		(ps_als->errinfo & ERRINFO_SOFT_RESET_OCCURRED) ? 2 :
		(ps_als->errinfo & ERRINFO_I2C_RESET_OCCURRED) ? 1 : 0,
		ps_als->errinfo);
	if (count >= PAGE_SIZE)
		goto fail_exit;

	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->ps_data_lock);
	mutex_unlock(&ps_als->control_lock);
	return count;
fail_exit:
	SENSOR_D_LOG("end - fail");
	mutex_unlock(&ps_als->ps_data_lock);
	mutex_unlock(&ps_als->control_lock);
	return PAGE_SIZE - 1;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t als_imit_show(char *buf)
{
	PS_ALS_DATA *ps_als= i2c_get_clientdata(client_rpr0521);
	ssize_t count = 0;

	SENSOR_D_LOG("start");
	mutex_lock(&ps_als->als_data_lock);

	count += scnprintf(buf, PAGE_SIZE - count,
				   "imit_flg : %d "
				   "imit_d0 : %d "
				   "imit_d1 : %d\n",
				   ps_als->imit.imit_flg,
				   ps_als->imit.imit_d0,
				   ps_als->imit.imit_d1);
	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->als_data_lock);
	return count;
}
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int als_imit_show(struct sensor_api_info *sai, char *buf)
{
	PS_ALS_DATA *ps_als;
	int count = 0;

	SENSOR_D_LOG("start");
	ps_als = container_of(sai, PS_ALS_DATA, als_info.sai);
	mutex_lock(&ps_als->als_data_lock);

	count += scnprintf(buf, PAGE_SIZE - count,
				   "imit_flg: %d"
				   "imit_d0: %d"
				   "imit_d1: %d\n",
				   ps_als->imit.imit_flg,
				   ps_als->imit.imit_d0,
				   ps_als->imit.imit_d1);
	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->als_data_lock);
	return count;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t als_imit_store(const char *buf)
{
	PS_ALS_DATA *ps_als;

	SENSOR_D_LOG("start");
	ps_als = i2c_get_clientdata(client_rpr0521);
	mutex_lock(&ps_als->als_data_lock);

	sscanf(buf, "%d %d %d",
		&ps_als->imit.imit_flg,
		&ps_als->imit.imit_d0,
		&ps_als->imit.imit_d1);

	SENSOR_ERR_LOG("imit_flg: %d imit_d0: %d imit_d1: %d",
					   ps_als->imit.imit_flg,
					   ps_als->imit.imit_d0,
					   ps_als->imit.imit_d1);

	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->als_data_lock);
	return 0;
}
#else  /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int als_imit_store(struct sensor_api_info *sai, const char *buf)
{
	PS_ALS_DATA *ps_als;

	SENSOR_D_LOG("start");
	ps_als = container_of(sai, PS_ALS_DATA, als_info.sai);
	mutex_lock(&ps_als->als_data_lock);

	sscanf(buf, "%d %d %d",
		&ps_als->imit.imit_flg,
		&ps_als->imit.imit_d0,
		&ps_als->imit.imit_d1);

	SENSOR_ERR_LOG("imit_flg: %d imit_d0: %d imit_d1: %d",
					   ps_als->imit.imit_flg,
					   ps_als->imit.imit_d0,
					   ps_als->imit.imit_d1);

	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->als_data_lock);
	return 0;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

#ifndef CONFIG_USE_MICON_SOFT_STRUCTURE
static int als_enable_show(struct sensor_api_info *sai, char *buf)
{
	PS_ALS_DATA *ps_als;
	int count = 0;

	SENSOR_D_LOG("start");
	ps_als = container_of(sai, PS_ALS_DATA, als_info.sai);
	mutex_lock(&ps_als->control_lock);

	count += scnprintf(buf, PAGE_SIZE - count,
				   "%d\n", ps_als->als_en_cnt);
	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->control_lock);
	return count;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

#ifndef CONFIG_USE_MICON_SOFT_STRUCTURE
static int ps_enable_show(struct sensor_api_info *sai, char *buf)
{
	PS_ALS_DATA *ps_als;
	int count = 0;

	SENSOR_D_LOG("start");
	ps_als = container_of(sai, PS_ALS_DATA, ps_info.sai);
	mutex_lock(&ps_als->control_lock);

	count += scnprintf(buf, PAGE_SIZE - count,
				   "%d\n", ps_als->ps_en_cnt);
	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->control_lock);
	return count;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

#ifndef CONFIG_USE_MICON_SOFT_STRUCTURE
static int als_enable_store(struct sensor_api_info *sai, unsigned int enable)
{
	PS_ALS_DATA *ps_als;
	int result = 0;

	ps_als = container_of(sai, PS_ALS_DATA, als_info.sai);

	if (enable > PS_ALS_ENABLE)
		return -EINVAL;

	SENSOR_D_LOG("start. als_en_cnt = %d, enable = %d",
					ps_als->als_en_cnt, enable);
	mutex_lock(&ps_als->control_lock);
	if (enable) {
		if (ps_als->als_en_cnt <= 0) {
			ps_als->als_en_cnt = 1;
		} else {
			ps_als->als_en_cnt++;
			goto exit;
		}
	} else {
		ps_als->als_en_cnt--;
		if (ps_als->als_en_cnt < 0) {
			ps_als->als_en_cnt = 0;
			goto exit;
		} else if (ps_als->als_en_cnt > 0){
			goto exit;
		}
	}

	result = als_enable_proc(ps_als, enable);
	if (result) {
		SENSOR_ERR_LOG("Failed to enable light sensor[%d]", result);
		if (enable)
			ps_als->als_en_cnt--;
		else
			ps_als->als_en_cnt++;
	}
exit:
	SENSOR_D_LOG("end. ps_als->als_en_cnt = %d", ps_als->als_en_cnt);
	mutex_unlock(&ps_als->control_lock);
	return result;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

#ifndef CONFIG_USE_MICON_SOFT_STRUCTURE
static int ps_enable_store(struct sensor_api_info *sai, unsigned int enable)
{
	PS_ALS_DATA *ps_als;
	int result = 0;

	ps_als = container_of(sai, PS_ALS_DATA, ps_info.sai);

	if (enable > PS_ALS_ENABLE)
		return -EINVAL;

	SENSOR_D_LOG("start. ps_en_cnt = %d, enable = %d",
					ps_als->ps_en_cnt, enable);
	mutex_lock(&ps_als->control_lock);
	if (enable) {
		if (ps_als->ps_en_cnt <= 0) {
			ps_als->ps_en_cnt = 1;
		} else {
			ps_als->ps_en_cnt++;
			goto exit;
		}
	} else {
		ps_als->ps_en_cnt--;
		if (ps_als->ps_en_cnt < 0) {
			ps_als->ps_en_cnt = 0;
			goto exit;
		} else if (ps_als->ps_en_cnt > 0){
			goto exit;
		}
	}

	result = ps_enable_proc(ps_als, enable);
	if (result) {
		SENSOR_ERR_LOG("Failed to enable proximity sensor[%d]", result);
		if (enable)
			ps_als->ps_en_cnt--;
		else
			ps_als->ps_en_cnt++;
	}
exit:
	SENSOR_D_LOG("end. ps_als->ps_en_cnt = %d", ps_als->ps_en_cnt);
	mutex_unlock(&ps_als->control_lock);
	return result;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t als_properties_store(const char *buf)
{
	PS_ALS_DATA *ps_als;
	int cnt;
	int color;
	int offset;
	int work[15] = {0};
	const char *p;

	SENSOR_D_LOG("start");
	ps_als = i2c_get_clientdata(client_rpr0521);

	cnt = sscanf(buf, "%d %n", &color, &offset);
	if (cnt != 1)
		goto fail_exit;
	if ((color < 0) ||(color >= NUM_OF_COLOR))
		goto fail_exit;

	p = buf + offset;
	cnt = sscanf(p, "%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",
		&work[ 0], &work[ 1], &work[ 2], &work[ 3], &work[ 4],
		&work[ 5], &work[ 6], &work[ 7], &work[ 8], &work[ 9],
		&work[10], &work[11], &work[12], &work[13], &work[14]);

	if (cnt != 15)
		goto fail_exit;

	mutex_lock(&ps_als->als_data_lock);
	ps_als->color = color;
	nv_photosensor_th0[ps_als->color] = work[ 0];
	nv_photosensor_th1[ps_als->color] = work[ 1];
	nv_photosensor_th2[ps_als->color] = work[ 2];
	nv_photosensor_th3[ps_als->color] = work[ 3];
	nv_photosensor_th4[ps_als->color] = work[ 4];
	nv_photosensor_a0[ps_als->color]  = work[ 5];
	nv_photosensor_a1[ps_als->color]  = work[ 6];
	nv_photosensor_a2[ps_als->color]  = work[ 7];
	nv_photosensor_a3[ps_als->color]  = work[ 8];
	nv_photosensor_a4[ps_als->color]  = work[ 9];
	nv_photosensor_b0[ps_als->color]  = work[10];
	nv_photosensor_b1[ps_als->color]  = work[11];
	nv_photosensor_b2[ps_als->color]  = work[12];
	nv_photosensor_b3[ps_als->color]  = work[13];
	nv_photosensor_b4[ps_als->color]  = work[14];
	mutex_unlock(&ps_als->als_data_lock);
	SENSOR_D_LOG("end");
	return 0;
fail_exit:
	SENSOR_D_LOG("end - fail");
	return (-EINVAL);
}
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int als_properties_store(struct sensor_api_info *sai, const char *buf)
{
	PS_ALS_DATA *ps_als;
	int cnt;
	int color;
	int offset;
	int work[15] = {0};
	const char *p;

	SENSOR_D_LOG("start");
	ps_als = container_of(sai, PS_ALS_DATA, als_info.sai);

	cnt = sscanf(buf, "%d %n", &color, &offset);
	if (cnt != 1)
		goto fail_exit;
	if ((color < 0) ||(color >= NUM_OF_COLOR))
		goto fail_exit;

	p = buf + offset;
	cnt = sscanf(p, "%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",
		&work[ 0], &work[ 1], &work[ 2], &work[ 3], &work[ 4],
		&work[ 5], &work[ 6], &work[ 7], &work[ 8], &work[ 9],
		&work[10], &work[11], &work[12], &work[13], &work[14]);

	if (cnt != 15)
		goto fail_exit;

	mutex_lock(&ps_als->als_data_lock);
	ps_als->color = color;
	nv_photosensor_th0[ps_als->color] = work[ 0];
	nv_photosensor_th1[ps_als->color] = work[ 1];
	nv_photosensor_th2[ps_als->color] = work[ 2];
	nv_photosensor_th3[ps_als->color] = work[ 3];
	nv_photosensor_th4[ps_als->color] = work[ 4];
	nv_photosensor_a0[ps_als->color]  = work[ 5];
	nv_photosensor_a1[ps_als->color]  = work[ 6];
	nv_photosensor_a2[ps_als->color]  = work[ 7];
	nv_photosensor_a3[ps_als->color]  = work[ 8];
	nv_photosensor_a4[ps_als->color]  = work[ 9];
	nv_photosensor_b0[ps_als->color]  = work[10];
	nv_photosensor_b1[ps_als->color]  = work[11];
	nv_photosensor_b2[ps_als->color]  = work[12];
	nv_photosensor_b3[ps_als->color]  = work[13];
	nv_photosensor_b4[ps_als->color]  = work[14];
	mutex_unlock(&ps_als->als_data_lock);
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
	PS_ALS_DATA *ps_als;
	ssize_t count = 0;

	SENSOR_D_LOG("start");
	ps_als = i2c_get_clientdata(client_rpr0521);
	mutex_lock(&ps_als->als_data_lock);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "color is [%d]\n",
			   ps_als->color);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_th0[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_th0[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_th1[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_th1[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_th2[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_th2[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_th3[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_th3[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_th4[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_th4[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_a0[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_a0[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_a1[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_a1[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_a2[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_a2[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_a3[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_a3[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_a4[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_a4[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_b0[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_b0[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_b1[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_b1[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_b2[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_b2[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_b3[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_b3[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_b4[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_b4[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;

	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->als_data_lock);
	return count;
fail_exit:
	SENSOR_D_LOG("end - fail");
	mutex_unlock(&ps_als->als_data_lock);
	return PAGE_SIZE - 1;
}
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int als_properties_show(struct sensor_api_info *sai, char *buf)
{
	PS_ALS_DATA *ps_als;
	int count = 0;

	SENSOR_D_LOG("start");
	ps_als = container_of(sai, PS_ALS_DATA, als_info.sai);
	mutex_lock(&ps_als->als_data_lock);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "color is [%d]\n",
			   ps_als->color);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_th0[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_th0[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_th1[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_th1[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_th2[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_th2[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_th3[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_th3[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_th4[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_th4[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_a0[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_a0[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_a1[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_a1[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_a2[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_a2[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_a3[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_a3[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_a4[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_a4[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_b0[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_b0[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_b1[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_b1[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_b2[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_b2[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_b3[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_b3[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_b4[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_b4[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;

	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->als_data_lock);
	return count;
fail_exit:
	SENSOR_D_LOG("end - fail");
	mutex_unlock(&ps_als->als_data_lock);
	return PAGE_SIZE - 1;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t ps_properties_store(const char *buf)
{
	PS_ALS_DATA *ps_als;
	int cnt;
	int color;
	int offset;
	int work[6] = {0};
	const char *p;

	SENSOR_D_LOG("start");
	ps_als = i2c_get_clientdata(client_rpr0521);
	mutex_lock(&ps_als->control_lock);

	cnt = sscanf(buf, "%d %n", &color, &offset);
	if (cnt != 1)
		goto fail_exit;
	if ((color < 0) ||(color >= NUM_OF_COLOR))
		goto fail_exit;

	p = buf + offset;
	cnt = sscanf(p, "%x %x %x %x %x %x",
		&work[0], &work[1], &work[2], &work[3], &work[4], &work[5]);
	if (cnt != 6 && cnt != 3)
		goto fail_exit;

	mutex_lock(&ps_als->ps_data_lock);
	ps_als->color = color;
	nv_proximity_detect[ps_als->color] = work[0];
	nv_proximity_no_detect[ps_als->color] = work[1];
	nv_proximity_offset[ps_als->color] = work[2];
	if (cnt > 3) {
		nv_proximity_temp[0] = work[3];
		nv_proximity_temp[1] = work[4];
		nv_proximity_temp[2] = work[5];
	}
	mutex_unlock(&ps_als->ps_data_lock);

	ps_check_thresholds_update(ps_als);

	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->control_lock);
	return 0;
fail_exit:
	SENSOR_D_LOG("end - fail");
	mutex_unlock(&ps_als->control_lock);
	return (-EINVAL);
}
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int ps_properties_store(struct sensor_api_info *sai, const char *buf)
{
	PS_ALS_DATA *ps_als;
	int cnt;
	int color;
	int offset;
	int work[6] = {0};
	const char *p;

	SENSOR_D_LOG("start");
	ps_als = container_of(sai, PS_ALS_DATA, ps_info.sai);
	mutex_lock(&ps_als->control_lock);

	cnt = sscanf(buf, "%d %n", &color, &offset);
	if (cnt != 1)
		goto fail_exit;
	if ((color < 0) ||(color >= NUM_OF_COLOR))
		goto fail_exit;

	p = buf + offset;
	cnt = sscanf(p, "%x %x %x %x %x %x",
		&work[0], &work[1], &work[2], &work[3], &work[4], &work[5]);
	if (cnt != 6 && cnt != 3)
		goto fail_exit;

	mutex_lock(&ps_als->ps_data_lock);
	ps_als->color = color;
	nv_proximity_detect[ps_als->color] = work[0];
	nv_proximity_no_detect[ps_als->color] = work[1];
	nv_proximity_offset[ps_als->color] = work[2];
	if (cnt > 3) {
		nv_proximity_temp[0] = work[3];
		nv_proximity_temp[1] = work[4];
		nv_proximity_temp[2] = work[5];
	}
	mutex_unlock(&ps_als->ps_data_lock);

	ps_check_thresholds_update(ps_als);

	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->control_lock);
	return 0;
fail_exit:
	SENSOR_D_LOG("end - fail");
	mutex_unlock(&ps_als->control_lock);
	return (-EINVAL);
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t ps_properties_show(char *buf)
{
	PS_ALS_DATA *ps_als;
	ssize_t count = 0;

	SENSOR_D_LOG("start");
	ps_als = i2c_get_clientdata(client_rpr0521);
	mutex_lock(&ps_als->ps_data_lock);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "color is [%d]\n",
			   ps_als->color);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_proximity_detect[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_proximity_detect[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_proximity_no_detect[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_proximity_no_detect[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_proximity_offset[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_proximity_offset[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_proximity_temp is [0x%02x,0x%02x,0x%02x]\n",
			   nv_proximity_temp[0],
			   nv_proximity_temp[1],
			   nv_proximity_temp[2]);
	if (count >= PAGE_SIZE)
		goto fail_exit;

	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->ps_data_lock);
	return count;
fail_exit:
	SENSOR_D_LOG("end - fail");
	mutex_unlock(&ps_als->ps_data_lock);
	return PAGE_SIZE - 1;
}
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int ps_properties_show(struct sensor_api_info *sai, char *buf)
{
	PS_ALS_DATA *ps_als;
	int count = 0;

	SENSOR_D_LOG("start");
	ps_als = container_of(sai, PS_ALS_DATA, ps_info.sai);
	mutex_lock(&ps_als->ps_data_lock);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "color is [%d]\n",
			   ps_als->color);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_proximity_detect[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_proximity_detect[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_proximity_no_detect[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_proximity_no_detect[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_proximity_offset[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_proximity_offset[ps_als->color]);
	if (count >= PAGE_SIZE)
		goto fail_exit;
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_proximity_temp is [0x%02x,0x%02x,0x%02x]\n",
			   nv_proximity_temp[0],
			   nv_proximity_temp[1],
			   nv_proximity_temp[2]);
	if (count >= PAGE_SIZE)
		goto fail_exit;

	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->ps_data_lock);
	return count;
fail_exit:
	SENSOR_D_LOG("end - fail");
	mutex_unlock(&ps_als->ps_data_lock);
	return PAGE_SIZE - 1;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int ps_als_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int result = 0;
	PS_ALS_DATA *ps_als = i2c_get_clientdata(client);

	SENSOR_D_LOG("start");
	mutex_lock(&ps_als->control_lock);

	ps_als->power_data_last.power_als = ps_als->power_data.power_als;
	ps_als->power_data_last.power_ps = ps_als->power_data.power_ps;

	if ((ps_als->power_data.power_als == PS_ALS_DISABLE) &&
	    (ps_als->power_data.power_ps == PS_ALS_DISABLE)) {
		SENSOR_D_LOG("skip");
		return 0;
	}

	if (ps_als->power_data.power_als == PS_ALS_ENABLE) {
		result = als_enable_proc(ps_als, PS_ALS_DISABLE);
		if (result)
			SENSOR_ERR_LOG("Failed to disable light sensor[%d]", result);
	}
	if (ps_als->power_data.power_ps == PS_ALS_ENABLE) {
		enable_irq_wake(client->irq);
		SENSOR_D_LOG("call enable_irq_wake().");
	}

	SENSOR_D_LOG("end");
	return 0;
}

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t ps_valid_show(char *buf)
{
	PS_ALS_DATA *ps_als;
	int count = 0;

	SENSOR_D_LOG("start");
	ps_als = i2c_get_clientdata(client_rpr0521);

	count += scnprintf(buf, PAGE_SIZE - count,
				   "%u\n", atomic_read(&ps_als->ps_info.valid));
	SENSOR_D_LOG("end");
	return count;

}
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int ps_valid_show(struct sensor_api_info *sai, char *buf)
{
	PS_ALS_DATA *ps_als;
	int count = 0;

	SENSOR_D_LOG("start");
	ps_als = container_of(sai, PS_ALS_DATA, ps_info.sai);

	count += scnprintf(buf, PAGE_SIZE - count,
				   "%u\n", atomic_read(&ps_als->ps_info.valid));
	SENSOR_D_LOG("end");
	return count;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t ps_valid_store(unsigned int valid)
{
	PS_ALS_DATA *ps_als;
	int last_valid;

	ps_als = i2c_get_clientdata(client_rpr0521);

	SENSOR_D_LOG("start. valid = %d", valid);

	last_valid = atomic_read(&ps_als->ps_info.valid);

	mutex_lock(&ps_als->ps_data_lock);
	if (!valid)
		ps_sensor_report_event_proc(ps_als, 0);

	atomic_set(&ps_als->ps_info.valid, valid);

	if (valid && !last_valid)
		ps_sensor_report_event_proc(ps_als, ps_als->ps_det);

	mutex_unlock(&ps_als->ps_data_lock);

	SENSOR_D_LOG("end");
	return 0;

}
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int ps_valid_store(struct sensor_api_info *sai, unsigned int valid)
{
	PS_ALS_DATA *ps_als;
	int last_valid;

	ps_als = container_of(sai, PS_ALS_DATA, ps_info.sai);

	SENSOR_D_LOG("start. valid = %d", valid);

	last_valid = atomic_read(&ps_als->ps_info.valid);

	mutex_lock(&ps_als->ps_data_lock);
	if (!valid)
		ps_sensor_report_event_proc(ps_als, 0);

	atomic_set(&ps_als->ps_info.valid, valid);

	if (valid && !last_valid)
		ps_sensor_report_event_proc(ps_als, ps_als->ps_det);

	mutex_unlock(&ps_als->ps_data_lock);

	SENSOR_D_LOG("end");
	return 0;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t als_flush_store(unsigned int flush)
{
	PS_ALS_DATA *ps_als;
	int result = 0;

	SENSOR_D_LOG("start");

	ps_als = i2c_get_clientdata(client_rpr0521);
	als_sensor_report_flush_event(&ps_als->als_info);

	SENSOR_D_LOG("end");
	return result;
}
#else /* CONFIG_USE_MICON_SOFT_STRUCTURE */
static int als_flush_store(struct sensor_api_info *sai, unsigned int flush)
{
	PS_ALS_DATA *ps_als;
	int result = 0;

	SENSOR_D_LOG("start");

	ps_als = container_of(sai, PS_ALS_DATA, als_info.sai);
	als_sensor_report_flush_event(&ps_als->als_info);

	SENSOR_D_LOG("end");
	return result;
}
#endif /* !CONFIG_USE_MICON_SOFT_STRUCTURE */

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t ps_flush_store(unsigned int flush)
{
	PS_ALS_DATA *ps_als;
	int result = 0;

	SENSOR_D_LOG("start");

	ps_als = i2c_get_clientdata(client_rpr0521);
	ps_sensor_report_flush_event(&ps_als->ps_info);

	SENSOR_D_LOG("end");
	return result;
}
#else /* CONFIG_USE_MICON_SOFT_STRUCTURE */
static int ps_flush_store(struct sensor_api_info *sai, unsigned int flush)
{
	PS_ALS_DATA *ps_als;
	int result = 0;

	SENSOR_D_LOG("start");

	ps_als = container_of(sai, PS_ALS_DATA, ps_info.sai);
	ps_sensor_report_flush_event(&ps_als->ps_info);

	SENSOR_D_LOG("end");
	return result;
}
#endif /* !CONFIG_USE_MICON_SOFT_STRUCTURE */

static int ps_als_resume(struct i2c_client *client)
{
	int result = 0;
	PS_ALS_DATA *ps_als = i2c_get_clientdata(client);

	SENSOR_D_LOG("start");
	if ((ps_als->power_data_last.power_als == PS_ALS_DISABLE) &&
	    (ps_als->power_data_last.power_ps == PS_ALS_DISABLE)) {
		SENSOR_D_LOG("skip");
		goto exit;
	}


	if (ps_als->power_data_last.power_als == PS_ALS_ENABLE) {
		result = als_enable_proc(ps_als, ps_als->power_data_last.power_als);
		if (result)
			SENSOR_ERR_LOG("Failed to enable light sensor[%d]", result);
	}
	if (ps_als->power_data_last.power_ps == PS_ALS_ENABLE) {
		disable_irq_wake(client->irq);
		SENSOR_D_LOG("call disable_irq_wake().");
	}

exit:
	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->control_lock);
	return result;
}

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
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

static int ps_als_parse_dt(PS_ALS_DATA *ps_als)
{
	struct device_node *of_node = ps_als->client->dev.of_node;

	ps_als->ps_als_vdd_is_ldo = of_property_read_bool(of_node, "rpr0521,psals-supply-is-ldo");
	ps_als->ps_als_is_PMIC_supply_ctrl = of_property_read_bool(of_node, "rpr0521,psals-PMIC-supply-ctrl");

	if (ps_als->ps_als_vdd_is_ldo) {
		ps_als->vprox_gpio = of_get_named_gpio(of_node, "rpr0521,vprox-gpio", 0);
		if(ps_als->vprox_gpio < 0) {
			SENSOR_ERR_LOG("failed to get VPROXIMITY GPIO=%d",
							ps_als->vprox_gpio);
			return -1;
		}
	} else {
		if(ps_als->ps_als_is_PMIC_supply_ctrl){
			of_property_read_u32(of_node,
				"psals-vcc-min-voltage", &ps_als->ps_als_vcc.min_uV);
			of_property_read_u32(of_node,
				"psals-vcc-max-voltage", &ps_als->ps_als_vcc.max_uV);
			of_property_read_u32(of_node,
				"psals-vcc-on-load-current", &ps_als->ps_als_vcc.on_load_uA);
			of_property_read_u32(of_node,
				"psals-vcc-off-load-current", &ps_als->ps_als_vcc.off_load_uA);
			SENSOR_D_LOG("regulator min_uV = %d, max_uV = %d, "
				     "on_load_uA = %d, off_load_uA = %d",
						ps_als->ps_als_vcc.min_uV,
						ps_als->ps_als_vcc.max_uV,
						ps_als->ps_als_vcc.on_load_uA,
						ps_als->ps_als_vcc.off_load_uA);
		}
	}

	of_property_read_u32(of_node,
			"rpr0521,psals-power-offon-interval-ms",
			&ps_als->ps_als_power.power_off_on_interval_ms);
	of_property_read_u32(of_node,
			"rpr0521,psals-power-on-wait-ms",
			&ps_als->ps_als_power.power_on_wait_ms);
	of_property_read_u32(of_node,
			"rpr0521,psals-power-normal-wait-ms",
			&ps_als->ps_als_power.power_normal_wait_ms);

	return 0;
}

static void ps_als_regulator_low(PS_ALS_DATA *ps_als)
{
	int err;
	SENSOR_D_LOG("start");
	if (!ps_als->ps_als_vdd_is_ldo) {
		if(ps_als->ps_als_is_PMIC_supply_ctrl){
			err = regulator_set_optimum_mode(ps_als->ps_als_vcc.v_reg, ps_als->ps_als_vcc.off_load_uA);
			if( err < 0 ) {
				SENSOR_ERR_LOG("[ALS_PS]%s regulator_set_optimum_mode fail. err=%d\n", __func__, err);
			}
		} else {
			SENSOR_D_LOG("[ALS_PS]%s Don't control PMIC supply\n", __func__);
		}
	}
	SENSOR_D_LOG("end");
}

static void ps_als_regulator_normal(PS_ALS_DATA *ps_als)
{
	int err;
	SENSOR_D_LOG("start");
	if (!ps_als->ps_als_vdd_is_ldo) {
		if(ps_als->ps_als_is_PMIC_supply_ctrl){
			err = regulator_set_optimum_mode(ps_als->ps_als_vcc.v_reg, ps_als->ps_als_vcc.on_load_uA);
			if( err < 0 ) {
				SENSOR_ERR_LOG("[ALS_PS]%s regulator_set_optimum_mode fail. err=%d\n", __func__, err);
			}
		} else {
			SENSOR_D_LOG("[ALS_PS]%s Don't control PMIC supply\n", __func__);
		}
	}
	SENSOR_D_LOG("end");
}

static int ps_als_regulator_enable(PS_ALS_DATA *ps_als, bool enable)
{
	int err = 0;
	SENSOR_D_LOG("start enable[%d]", enable);

	if (enable != ps_als->ps_als_power.enabled) {
		if (enable) {
			if (ktime_to_ms(ktime_sub(ktime_get(), ps_als->ps_als_power.power_off_time)) < ps_als->ps_als_power.power_off_on_interval_ms) {
				usleep_range(ps_als->ps_als_power.power_off_on_interval_ms*1000,
							 ps_als->ps_als_power.power_off_on_interval_ms*1000);
			}
			if (ps_als->ps_als_vdd_is_ldo) {
				gpio_set_value(ps_als->vprox_gpio, 1);
			} else {
				err = regulator_enable(ps_als->ps_als_vcc.v_reg);
			}
			if (!err) {
				ps_als->ps_als_power.power_off_time = ktime_get();
			} else  {
				SENSOR_ERR_LOG("regulator_enable fail. err=%d\n", err);
			}
		} else {
			if (ps_als->ps_als_vdd_is_ldo) {
				gpio_set_value(ps_als->vprox_gpio, 0);
			} else {
				err = regulator_disable(ps_als->ps_als_vcc.v_reg);
			}
			if (!err) {
				ps_als->ps_als_power.power_off_time = ktime_get();
			} else {
				SENSOR_ERR_LOG("regulator_disable fail. err=%d\n", err);
			}
		}

		if (!err) {
			ps_als->ps_als_power.enabled = enable;
		}
	}

	SENSOR_D_LOG("end. err[%d]", err);

	return err;
}

static int ps_als_pinctrl_init(PS_ALS_DATA *ps_als)
{
	int ret = 0;
	struct device *dev = &ps_als->client->dev;

	ps_als->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(ps_als->pinctrl)) {
		SENSOR_ERR_LOG("Target does not use pinctrl");
		ret = PTR_ERR(ps_als->pinctrl);
		ps_als->pinctrl = NULL;
		return ret;
	}

	ps_als->gpio_state_active
		= pinctrl_lookup_state(ps_als->pinctrl, "active");
	if (IS_ERR_OR_NULL(ps_als->gpio_state_active)) {
		SENSOR_ERR_LOG("Can not get active pinstate");
		ret = PTR_ERR(ps_als->gpio_state_active);
		ps_als->pinctrl = NULL;
		return ret;
	}

	ps_als->gpio_state_suspend
		= pinctrl_lookup_state(ps_als->pinctrl, "suspend");
	if (IS_ERR_OR_NULL(ps_als->gpio_state_suspend)) {
		SENSOR_ERR_LOG("Can not get suspend pinstate");
		ret = PTR_ERR(ps_als->gpio_state_suspend);
		ps_als->pinctrl = NULL;
		return ret;
	}

	return 0;
}

static int ps_als_pinctrl_select(PS_ALS_DATA *ps_als, bool on)
{
	struct pinctrl_state *pins_state;
	int ret;

	pins_state = on ? ps_als->gpio_state_active
					: ps_als->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(ps_als->pinctrl, pins_state);
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

static int ps_als_power_ctrl(PS_ALS_DATA *ps_als, enum ps_als_power_ctrl_mode request_mode)
{
	int ret = 0;
	int wait_ms = 0;

	SENSOR_D_LOG("start prev_mode[%d] request_mode[%d]", ps_als->ps_als_power.prev_mode, request_mode);
	if (request_mode >= PS_ALS_POWER_CTRL_MAX) {
		SENSOR_ERR_LOG("param err");
		return -1;
	}

	mutex_lock(&ps_als->ps_als_power.ps_als_power_mutex);
	if (request_mode != ps_als->ps_als_power.prev_mode) {
		switch(request_mode) {
		case PS_ALS_POWER_CTRL_OFF:
			ret = ps_als_pinctrl_select(ps_als, false);
			if (ret) {
				break;
			}
			ps_als_regulator_low(ps_als);
			ret = ps_als_regulator_enable(ps_als, false);
			break;
		case PS_ALS_POWER_CTRL_LOW:
			ret = ps_als_pinctrl_select(ps_als, true);
			if (ret) {
				break;
			}
			ps_als_regulator_low(ps_als);
			ret = ps_als_regulator_enable(ps_als, true);
			wait_ms = (ps_als->ps_als_power.prev_mode == PS_ALS_POWER_CTRL_OFF) ?
				ps_als->ps_als_power.power_on_wait_ms : 0;
			break;
		case PS_ALS_POWER_CTRL_NORMAL:
			ret = ps_als_pinctrl_select(ps_als, true);
			if (ret) {
				break;
			}
			ps_als_regulator_normal(ps_als);
			ret = ps_als_regulator_enable(ps_als, true);
			wait_ms = (ps_als->ps_als_power.prev_mode == PS_ALS_POWER_CTRL_OFF) ?
				ps_als->ps_als_power.power_on_wait_ms : ps_als->ps_als_power.power_normal_wait_ms;
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
		ps_als->ps_als_power.prev_mode = request_mode;
	} else {
		SENSOR_ERR_LOG("power_ctrl fail. ret=%d\n", ret);
	}

	mutex_unlock(&ps_als->ps_als_power.ps_als_power_mutex);

	SENSOR_D_LOG("end. ret[%d]", ret);
	return ret;
}

static int ps_als_power_src_enable(PS_ALS_DATA *ps_als, bool enable)
{
	if (enable) {
		if (ps_als->ps_als_vdd_is_ldo) {
			if (gpio_request(ps_als->vprox_gpio, "vproximity_en")) {
				SENSOR_ERR_LOG("gpio_request failed.");
				return -1;
			}
		} else {
			ps_als->ps_als_vcc.v_reg = regulator_get(&ps_als->client->dev, "psals-vcc");
			if (IS_ERR(ps_als->ps_als_vcc.v_reg)) {
				SENSOR_ERR_LOG("Failed regulator_get for psals-vcc");
				return -1;
			}
		}
	} else {
		if (ps_als->ps_als_vdd_is_ldo) {
			gpio_free(ps_als->vprox_gpio);
		} else {
			regulator_put(ps_als->ps_als_vcc.v_reg);
		}
	}
	return 0;
}

/******************************************************************************
 * NAME       : ps_als_probe
 * FUNCTION   : initialize system
 * REMARKS    :
 *****************************************************************************/
static int ps_als_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	PS_ALS_DATA *ps_als;
	int result;

	SENSOR_N_LOG("called ps_als_probe for RPR0521!!");

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
	rpr0521_initialize = 0;
#endif	/*CONFIG_USE_MICON_SOFT_STRUCTURE*/

	result = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if (unlikely(!result)) {
		SENSOR_ERR_LOG("need I2C_FUNC_I2C");
		result = -ENODEV;
		goto err_check_functionality_failed;
	}
	ps_als = kzalloc(sizeof(*ps_als), GFP_KERNEL);
	if (unlikely(ps_als == NULL)) {
		result = -ENOMEM;
		goto err_alloc_data_failed;
	}
	INIT_WORK(&ps_als->als_work, als_work_func);
	INIT_DELAYED_WORK(&ps_als->monitor_dwork, ps_als_monitor_func);
	mutex_init(&ps_als->control_lock);
	mutex_init(&ps_als->ps_data_lock);
	mutex_init(&ps_als->als_data_lock);
	mutex_lock(&ps_als->control_lock);
	mutex_lock(&ps_als->ps_data_lock);
	mutex_lock(&ps_als->als_data_lock);
	ps_als->client = client;
	i2c_set_clientdata(client, ps_als);

#ifndef CONFIG_USE_MICON_SOFT_STRUCTURE
	ps_als->als_info = als_info_base;
#endif /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
	ps_als->als_info.dev = &client->dev;
	result = als_sensor_register(&ps_als->als_info);
	if (unlikely(result)) {
		SENSOR_ERR_LOG("Failed to als_sensor_register");
		goto err_als_register_failed;
	}

#ifndef CONFIG_USE_MICON_SOFT_STRUCTURE
	ps_als->ps_info = ps_info_base;
#else
//	atomic_set(&ps_als->ps_info.valid, 1);
#endif /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
	ps_als->ps_info.dev = &client->dev;
	result = ps_sensor_register(&ps_als->ps_info);
	if (unlikely(result)) {
		SENSOR_ERR_LOG("Failed to ps_sensor_register");
		goto err_ps_register_failed;
	}

	result = ps_als_parse_dt(ps_als);
	if (unlikely(result)) {
		SENSOR_ERR_LOG("failed to parse dt");
		goto err_drv_init_failed;
	}
	ps_als->ps_als_power.enabled = false;

	result = ps_als_power_src_enable(ps_als, true);
	if (unlikely(result)) {
		goto err_drv_init_failed;
	}

	result = ps_als_pinctrl_init(ps_als);
	if (unlikely(result)) {
		SENSOR_ERR_LOG("failed to ps_als_pinctrl_init");
		goto err_gpio_free;
	}

	ps_als->ps_als_power.prev_mode = PS_ALS_POWER_CTRL_OFF;
	mutex_init(&ps_als->ps_als_power.ps_als_power_mutex);
	result = ps_als_power_ctrl(ps_als, PS_ALS_POWER_CTRL_NORMAL);
	if (unlikely(result)) {
		SENSOR_ERR_LOG("failed to power on device");
		goto err_gpio_free;
	}

	ps_als->init_data.mode_ctl    = PS_ALS_SET_MODE_CONTROL;
	ps_als->init_data.psals_ctl   = PS_ALS_SET_ALSPS_CONTROL;
	ps_als->init_data.persist     = PS_ALS_SET_INTR_PERSIST;
	ps_als->init_data.intr        = PS_ALS_SET_INTR;
	ps_als->init_data.psth_upper  = nv_proximity_detect[ps_als->color];
	ps_als->init_data.psth_low    = nv_proximity_no_detect[ps_als->color];
	ps_als->init_data.alsth_upper = PS_ALS_SET_ALS_TH;
	ps_als->init_data.alsth_low   = PS_ALS_SET_ALS_TL;
	ps_als->init_data.ps_offset   = nv_proximity_offset[ps_als->color];

	result = ps_als_driver_init(&ps_als->init_data, client);
	if (unlikely(result)) {
		SENSOR_ERR_LOG("failed to driver init");
		goto err_power_on_failed;
	}
	atomic_set(&ps_als->ps_info.valid, 1);

	result = ps_als_power_ctrl(ps_als, PS_ALS_POWER_CTRL_LOW);
	if (unlikely(result)) {
		SENSOR_ERR_LOG("failed to power on device");
		goto err_power_on_failed;
	}

	ps_als->power_data.power_als = PS_ALS_DISABLE;
	ps_als->power_data.power_ps = PS_ALS_DISABLE;
	ps_als->power_data_last.power_als = PS_ALS_DISABLE;
	ps_als->power_data_last.power_ps = PS_ALS_DISABLE;

	/* check whether to use interrupt or not */
	if (client->irq) {
		/* interrupt process */
		result = request_threaded_irq(client->irq, NULL, ps_irq_handler,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->name, ps_als);

		/* 1 : interrupt mode/ 0 : polling mode */
		if (result == 0) {
			disable_irq(client->irq);
		} else {
			SENSOR_ERR_LOG("request IRQ Failed==>result : %d", result);
			SENSOR_ERR_LOG("client->irq        = 0x%x", client->irq);
			SENSOR_ERR_LOG("ps_irq_handler = 0x%lx", (long)ps_irq_handler);
			SENSOR_ERR_LOG("interrupt flag     = 0x%x", IRQF_TRIGGER_FALLING);
			SENSOR_ERR_LOG("interrupt name     = %s", client->name);
			SENSOR_ERR_LOG("base address       = 0x%lx", (long)ps_als);
			goto err_power_on_failed;
		}
	}
	/* timer process */
	hrtimer_init(&ps_als->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ps_als->timer.function = als_timer_func;

	/* initialize static variable */
	ps_als->delay_time = ALS_DATA_DELAY_MS;

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
	client_rpr0521 = client;
	rpr0521_initialize = 1;
	wake_lock_init(&ps_als->ps_info.wake_lock, WAKE_LOCK_SUSPEND, "ps_sensor");
#endif	/*CONFIG_USE_MICON_SOFT_STRUCTURE*/

	mutex_unlock(&ps_als->control_lock);
	mutex_unlock(&ps_als->ps_data_lock);
	mutex_unlock(&ps_als->als_data_lock);
	SENSOR_N_LOG("called ps_als_probe for RPR0521!!");
	return (result);
err_power_on_failed:
	ps_als_power_ctrl(ps_als, PS_ALS_POWER_CTRL_OFF);
err_gpio_free:
	ps_als_power_src_enable(ps_als, false);
err_drv_init_failed:
	ps_sensor_unregister(&ps_als->ps_info);
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
	atomic_set(&ps_als->ps_info.valid, 0);
#endif /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
err_ps_register_failed:
	als_sensor_unregister(&ps_als->als_info);
err_als_register_failed:
	mutex_unlock(&ps_als->control_lock);
	mutex_unlock(&ps_als->ps_data_lock);
	mutex_unlock(&ps_als->als_data_lock);
	kfree(ps_als);
err_alloc_data_failed:
err_check_functionality_failed:

	return (result);

}

/******************************************************************************
 * NAME       : ps_als_remove
 * FUNCTION   : close system
 * REMARKS    :
 *****************************************************************************/
static int ps_als_remove(struct i2c_client *client)
{
	PS_ALS_DATA *ps_als;

	ps_als = i2c_get_clientdata(client);
	als_enable_proc(ps_als, 0);
	ps_enable_proc(ps_als, 0);
	ps_als_power_ctrl(ps_als, PS_ALS_POWER_CTRL_OFF);
	ps_als_power_src_enable(ps_als, false);
	ps_sensor_unregister(&ps_als->ps_info);
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
	atomic_set(&ps_als->ps_info.valid, 0);
#endif /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
	als_sensor_unregister(&ps_als->als_info);
	kfree(ps_als);

	return 0;
}

/******************************************************************************
 * NAME       : ps_als_shutdown
 * FUNCTION   : shutdown
 * REMARKS    :
 *****************************************************************************/
static void ps_als_shutdown(struct i2c_client *client)
{
	PS_ALS_DATA *ps_als;

	SENSOR_V_LOG("start");
	ps_als = i2c_get_clientdata(client);
	ps_als_power_ctrl(ps_als, PS_ALS_POWER_CTRL_OFF);
	SENSOR_V_LOG("end");
}

/******************************************************************************
 * NAME       : ps_als_init
 * FUNCTION   : register driver to kernel
 * REMARKS    :
 *****************************************************************************/
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
int ps_als_init(void)
{
    rohm_workqueue = create_singlethread_workqueue("rohm_workqueue");
    if (!rohm_workqueue) {
        return (-ENOMEM);
    }

    return (i2c_add_driver(&rpr0521_driver));
}
EXPORT_SYMBOL(ps_als_init);
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int __init ps_als_init(void)
{
    rohm_workqueue = create_singlethread_workqueue("rohm_workqueue");
    if (!rohm_workqueue) {
        return (-ENOMEM);
    }

    return (i2c_add_driver(&rpr0521_driver));
}
#endif /*CONFIG_USE_MICON_SOFT_STRUCTURE*/

/******************************************************************************
 * NAME       : ps_als_exit
 * FUNCTION   : remove driver from kernel
 * REMARKS    :
 *****************************************************************************/
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
void ps_als_exit(void)
{
    i2c_del_driver(&rpr0521_driver);
    if (rohm_workqueue) {
        destroy_workqueue(rohm_workqueue);
    }

    return;
}
EXPORT_SYMBOL(ps_als_exit);
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static void __exit ps_als_exit(void)
{
    i2c_del_driver(&rpr0521_driver);
    if (rohm_workqueue) {
        destroy_workqueue(rohm_workqueue);
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

static int ps_als_i2c_read(const struct i2c_client *client, uint8_t reg, uint8_t *rbuf, int len)
{
	int ret = 0;
	struct i2c_msg i2cMsg[I2C_READ_MSG_NUM];
	uint8_t buff;
	int retry = I2C_RETRY_MAX + 1;
	int i;

	SENSOR_V_LOG("start reg[%02X] len[%d]", reg, len );

	if (client == NULL) {
		return -ENODEV;
	}

	if (ps_als_is_device_dead(i2c_get_clientdata(client))) {
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
		SENSOR_V_LOG("i2c_transfer() called. ret[%d]",ret);
		if (ret == I2C_READ_MSG_NUM) {
			SENSOR_V_LOG("end. exec mesg[%d]",(int)ret);
			for (i = 0; i < len; i++) {
				SENSOR_V_LOG("i2c read reg[%02X] value[%02X]",
				             (unsigned int)(reg + i), (unsigned int)*(rbuf + i));
			}
			SENSOR_V_LOG("end - return len=%d", len);
			return len;
		} else {
			SENSOR_ERR_LOG("i2c transfer error[%d] in while.",ret );
			ps_als_set_errinfo(i2c_get_clientdata(client), ERRINFO_I2C_RESET_OCCURRED);
		}
	}
	ps_als_set_errinfo(i2c_get_clientdata(client), ERRINFO_I2C_RECOVERY_FAILED);
	SENSOR_ERR_LOG("i2c transfer error[%d]",ret );
	return -1;
}

static int ps_als_i2c_write(const struct i2c_client *client, uint8_t reg, const uint8_t *wbuf, int len)
{
#define BUF_SIZE	0x20
	int ret = 0;
	struct i2c_msg i2cMsg[I2C_WRITE_MSG_NUM];
	uint8_t buff[BUF_SIZE];
	int retry = I2C_RETRY_MAX + 1;
	int i;

	SENSOR_V_LOG("start reg[%02X] len[%d]", reg, len );

	if (unlikely(client == NULL)) {
		return -ENODEV;
	}

	if (ps_als_is_device_dead(i2c_get_clientdata(client))) {
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
		SENSOR_V_LOG("i2c_transfer() called. ret[%d]",ret);

		if (ret == I2C_WRITE_MSG_NUM) {
			SENSOR_V_LOG("end. exec mesg[%d]",ret);
			for (i = 0; i < len; i++) {
				SENSOR_V_LOG("i2c write reg[%02X] value[%02X]",
				             (unsigned int)(reg + i), (unsigned int)*(wbuf + i));
			}
			SENSOR_V_LOG("end - return 0");
			return 0;
		} else {
			SENSOR_ERR_LOG("i2c transfer error[%d] in while.",ret );
			ps_als_set_errinfo(i2c_get_clientdata(client), ERRINFO_I2C_RESET_OCCURRED);
		}
	}
	ps_als_set_errinfo(i2c_get_clientdata(client), ERRINFO_I2C_RECOVERY_FAILED);
	SENSOR_ERR_LOG("i2c transfer error[%d]",ret );

	return -1;
#undef BUF_SIZE
}

static int ps_als_i2c_read_byte_data(const struct i2c_client *client,
                                      u8 command)
{
	int ret;
	uint8_t buff = 0;

	ret = ps_als_i2c_read(client, command, &buff, 1);

	return (ret < 0) ? ret : buff;
}

static int ps_als_i2c_write_byte_data(const struct i2c_client *client,
                                      u8 command, u8 value)
{
	int ret;

	ret = ps_als_i2c_write(client, command, &value, 1);

	return ret;
}

static int ps_als_i2c_write_i2c_block_data(const struct i2c_client *client, u8 command,
                                           u8 length, const u8 *values)
{
	int ret;

	ret = ps_als_i2c_write(client, command, values, length);

	return ret;
}

/******************************************************************************
 * NAME       : ps_als_driver_init
 * FUNCTION   : initialize RPR0521
 * REMARKS    :
 *****************************************************************************/
static int ps_als_driver_init(INIT_ARG *data, struct i2c_client *client)
{
	struct init_func_write_data {
		unsigned char mode_ctl;
		unsigned char psals_ctl;
		unsigned char persist;
		unsigned char reserved0;
		unsigned char reserved1;
		unsigned char reserved2;
		unsigned char reserved3;
		unsigned char reserved4;
		unsigned char reserved5;
		unsigned char intr;
		unsigned char psth_hl;
		unsigned char psth_hh;
		unsigned char psth_ll;
		unsigned char psth_lh;
		unsigned char alsth_hl;
		unsigned char alsth_hh;
		unsigned char alsth_ll;
		unsigned char alsth_lh;
		unsigned char psoff_ll;
		unsigned char psoff_lh;
	} write_data;
	int result;

	/* not check parameters are psth_upper, psth_low, alsth_upper, alsth_low */
	/* check the PS orerating mode */
	if ((data->mode_ctl & 0xF) > MEASUREMENT_MAX) {
		SENSOR_ERR_LOG("data->mode_ctl = 0x%x", data->mode_ctl);
		return (-EINVAL);
	}

	if (0 != (data->mode_ctl & INIT_MODE_MASK)) {
		SENSOR_ERR_LOG("data->mode_ctl = 0x%x", data->mode_ctl);
		return (-EINVAL);
	}

	/* check the parameter of ps and als control */
	if (data->psals_ctl > REG_ALSPSCTL_MAX) {
		SENSOR_ERR_LOG("data->psals_ctl = 0x%x", data->psals_ctl);
		return (-EINVAL);
	}

	/* check the parameter of ps interrupt persistence */
	if (data->persist > (PSGAIN_MAX | PERSISTENCE_MAX)) {
		SENSOR_ERR_LOG("data->persist = 0x%x", data->persist);
		return (-EINVAL);
	}
	/* check the parameter of interrupt */
	if (data->intr > REG_INTERRUPT_MAX) {
		SENSOR_ERR_LOG("data->intr = 0x%x", data->intr);
		return (-EINVAL);
	}
	/* check the parameter of proximity sensor threshold high */
	if (data->psth_upper > REG_PSTH_MAX) {
		SENSOR_ERR_LOG("data->psth_upper = 0x%x", data->psth_upper);
		return (-EINVAL);
	}
	/* check the parameter of proximity sensor threshold low */
	if (data->psth_low > REG_PSTL_MAX) {
		SENSOR_ERR_LOG("data->psth_low = 0x%x", data->psth_low);
		return (-EINVAL);
	}
	write_data.mode_ctl  = data->mode_ctl;
	write_data.psals_ctl = data->psals_ctl;
	write_data.persist   = data->persist;
	write_data.reserved0 = 0;
	write_data.reserved1 = 0;
	write_data.reserved2 = 0;
	write_data.reserved3 = 0;
	write_data.reserved4 = 0;
	write_data.reserved5 = 0;
	write_data.intr      = data->intr;
	write_data.psth_hl   = CONVERT_TO_BE(data->psth_upper) & MASK_CHAR;
	write_data.psth_hh   = CONVERT_TO_BE(data->psth_upper) >> 8;
	write_data.psth_ll   = CONVERT_TO_BE(data->psth_low) & MASK_CHAR;
	write_data.psth_lh   = CONVERT_TO_BE(data->psth_low) >> 8;
	write_data.alsth_hl  = CONVERT_TO_BE(data->alsth_upper) & MASK_CHAR;
	write_data.alsth_hh  = CONVERT_TO_BE(data->alsth_upper) >> 8;
	write_data.alsth_ll  = CONVERT_TO_BE(data->alsth_low) & MASK_CHAR;
	write_data.alsth_lh  = CONVERT_TO_BE(data->alsth_low) >> 8;
	write_data.psoff_ll  = CONVERT_TO_BE(data->ps_offset) & MASK_CHAR;
	write_data.psoff_lh  = CONVERT_TO_BE(data->ps_offset) >> 8;
	result = ps_als_i2c_write_i2c_block_data(client,
						REG_MODECONTROL,
						sizeof(write_data),
						(unsigned char *)&write_data);

	return (result);
}

/******************************************************************************
 * NAME       : ps_als_driver_reset
 * FUNCTION   : reset RPR0521 register
 * REMARKS    :
 *****************************************************************************/
static int ps_als_driver_reset(struct i2c_client *client)
{
    int result;

    /* set soft ware reset */
    result = ps_als_i2c_write_byte_data(client, REG_SYSTEMCONTROL, (REG_SW_RESET | REG_INT_RESET));

    return (result);
}

/******************************************************************************
 * NAME       : ps_als_driver_read_power_state
 * FUNCTION   : read the value of PS and ALS status in RPR0521
 * REMARKS    :
 *****************************************************************************/
static int ps_als_driver_read_power_state(PWR_ST *pwr_st, struct i2c_client *client)
{
	int result;

	/* read control state of ps and als */
	result = ps_als_i2c_read_byte_data(client, REG_MODECONTROL);
	if (unlikely(result < 0)) {
		pwr_st->als_state = CTL_STANDBY;
		pwr_st->ps_state  = CTL_STANDBY;
	} else {
		/* check power state of als from control state */
		if (result & PWRON_ALS_EN) {
			pwr_st->als_state = CTL_STANDALONE;
		} else {
			pwr_st->als_state = CTL_STANDBY;
		}

		/* check power state of ps from control state */
		if (result & PWRON_PS_EN) {
			pwr_st->ps_state = CTL_STANDALONE;
		} else {
			pwr_st->ps_state = CTL_STANDBY;
		}
	}

	return (result);
}

/******************************************************************************
 * NAME       : ps_als_driver_general_read
 * FUNCTION   : read general multi bytes
 * REMARKS    :
 *****************************************************************************/
static int ps_als_driver_general_read(GENREAD_ARG data, struct i2c_client *client)
{
	int            result;

	if (data.size == 0) {
		return (-EINVAL);
	}
	/* check the parameter of register */
	if ((data.adr_reg < REG_SYSTEMCONTROL) || (data.adr_reg > REG_PSOFFSET_MSB)) {
		return (-EINVAL);
	}

	result = ps_als_i2c_read(client, data.adr_reg, data.addr, data.size);
	if (result < 0) {
		SENSOR_ERR_LOG("transfer error");
	}

	return (result);
}

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE

bool als_sensor_get_en_first(void)
{
	PS_ALS_DATA *ps_als = i2c_get_clientdata(client_rpr0521);
	SENSOR_D_LOG("start");
	SENSOR_D_LOG("end");
	return ps_als->als_en_first;
}

int32_t als_sensor_activate(bool enable)
{
	PS_ALS_DATA *ps_als;
	int result = 0;

	ps_als = i2c_get_clientdata(client_rpr0521);

	if (unlikely(enable > PS_ALS_ENABLE)) {
		SENSOR_D_LOG("INVALID enable val=%d",enable);
		return -EINVAL;
	}

	SENSOR_D_LOG("start. als_en_cnt = %d, enable = %d",
					ps_als->als_en_cnt, enable);
	mutex_lock(&ps_als->control_lock);
	if (enable) {
		if (ps_als->als_en_cnt <= 0) {
			ps_als->als_en_cnt = 1;
		} else {
			ps_als->als_en_cnt++;
			goto exit;
		}
	} else {
		ps_als->als_en_cnt--;
		if (ps_als->als_en_cnt < 0) {
			ps_als->als_en_cnt = 0;
			goto exit;
		} else if (ps_als->als_en_cnt > 0){
			goto exit;
		}
	}

	result = als_enable_proc(ps_als, enable);
	if (result) {
		SENSOR_ERR_LOG("Failed to enable light sensor[%d]", result);
		if (enable)
			ps_als->als_en_cnt--;
		else
			ps_als->als_en_cnt++;
	}
exit:
	SENSOR_D_LOG("end. ps_als->als_en_cnt = %d", ps_als->als_en_cnt);
	mutex_unlock(&ps_als->control_lock);
	return result;
}

int32_t ps_sensor_activate(bool enable)
{
	PS_ALS_DATA *ps_als = i2c_get_clientdata(client_rpr0521);
	int result = 0;

	if (unlikely(enable > PS_ALS_ENABLE)) {
		SENSOR_D_LOG("INVALID enable val=%d",enable);
		return -EINVAL;
	}

	SENSOR_D_LOG("start. ps_en_cnt = %d, enable = %d",
					ps_als->ps_en_cnt, enable);
	mutex_lock(&ps_als->control_lock);
	if (enable) {
		if (ps_als->ps_en_cnt <= 0) {
			ps_als->ps_en_cnt = 1;
		} else {
			ps_als->ps_en_cnt++;
			goto exit;
		}
	} else {
		ps_als->ps_en_cnt--;
		if (ps_als->ps_en_cnt < 0) {
			ps_als->ps_en_cnt = 0;
			goto exit;
		} else if (ps_als->ps_en_cnt > 0){
			goto exit;
		}
	}

	result = ps_enable_proc(ps_als, enable);
	if (result) {
		SENSOR_ERR_LOG("Failed to enable proximity sensor[%d]", result);
		if (enable)
			ps_als->ps_en_cnt--;
		else
			ps_als->ps_en_cnt++;
	}
exit:
	SENSOR_D_LOG("end. ps_als->ps_en_cnt = %d", ps_als->ps_en_cnt);
	mutex_unlock(&ps_als->control_lock);
	return result;
}

uint32_t ps_sensor_get_count(void)
{
	uint32_t ps_count = 0;
	PS_ALS_DATA *ps_als = i2c_get_clientdata(client_rpr0521);

	ps_driver_read_data(client_rpr0521);
	ps_count = (uint32_t)ps_als->ps_val.ps_data;
	
	return ps_count;
}

int32_t als_get_initialize_state(void){
	return rpr0521_initialize;
}
int32_t ps_get_initialize_state(void){
	return rpr0521_initialize;
}

#else	/*CONFIG_USE_MICON_SOFT_STRUCTURE*/

module_init(ps_als_init);
module_exit(ps_als_exit);
#endif	/*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

MODULE_DESCRIPTION("ROHM Proximity Sensor & Ambient Light Sensor Driver");
MODULE_LICENSE("GPL");
