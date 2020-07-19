/* This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 * (C) 2017 KYOCERA Corporation
 * (C) 2019 KYOCERA Corporation
 */
/******************************************************************************
 * MODULE     : rohm_rpr0521_i2c.h
 * FUNCTION   : Driver header for RPR0521, Ambient Light Sensor(ALS) IC
 * PROGRAMMED : Sensor application development group
 * AUTHOR     : Masafumi Seike
 * REMARKS    :
 * COPYRIGHT  : Copyright (C) 2014 - ROHM CO.,LTD.
 *            : This program is free software; you can redistribute it and/or
 *            : modify it under the terms of the GNU General Public License
 *            : as published by the Free Software Foundation; either version 2
 *            : of the License, or (at your option) any later version.
 *            :
 *            : This program is distributed in the hope that it will be useful,
 *            : but WITHOUT ANY WARRANTY; without even the implied warranty of
 *            : MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *            : GNU General Public License for more details.
 *            :
 *            : You should have received a copy of the GNU General Public License
 *            : along with this program; if not, write to the Free Software
 *            : Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *****************************************************************************/
#ifndef _ROHM_RPR0521_I2C_H_
#define _ROHM_RPR0521_I2C_H_

#include "rohm_rpr0521_i2c_if.h"

#define RPR0521_DRIVER_VER ("0.1.0")
#define CALC_ERROR         (0x80000000)
#define SM_TIME_UNIT       (1000)
#define MN_TIME_UNIT       (1000000)
#define MASK_CHAR          (0xFF)
#define CLR_LOW2BIT        (0xFC)
#define CLR_LOW4BIT        (0xF0)
#define INIT_MODE_MASK     (0xC0)
#define INIT_PS_MODE_MASK  (0x40)
#define INIT_ALS_MODE_MASK (0x80)
#define UNRELATEDNESS      (0xFF)
#define IRQ_NON_USE        (0)
#define IRQ_USE            (1)
#define MASK_LONG          (0xFFFFFFFF)
#define PS_DATA_MASK       (0x7FFF)
#define PS_FLAG_MASK       (0x8000)
#define HOST_ENABLE        (1)
#define HOST_DISABLE       (0)

#ifdef _ALS_BIG_ENDIAN_
#define CONVERT_TO_BE(value) ((((value) >> 8) & 0xFF) | (((value) << 8) & 0xFF00))
#else
#define CONVERT_TO_BE(value) (value)
#endif

/* structure to read data value from sensor */
typedef struct {
    unsigned short ps_data;          /* data value of PS data from sensor        */
    unsigned short als_data0;        /* data value of ALS data0 from sensor      */
    unsigned short als_data1;        /* data value of ALS data1 from sensor      */
} READ_DATA_BUF;

typedef struct {
    unsigned short als_data0;        /* data value of ALS data0 from sensor      */
    unsigned short als_data1;        /* data value of ALS data1 from sensor      */
} READ_DATA_ALS_BUF;

typedef struct {
    unsigned char  ps_intr;          /* data value of interrupt from sensor      */
    unsigned char  ps_ctrl;          /* data value of PS control from sensor     */
    unsigned short ps_data;          /* data value of PS data from sensor        */
    unsigned char  ps_flag;          /* data value of PS_INT_TH_FLAG from sensor */
} READ_DATA_PS_BUF;

/* structure to set initial value to sensor */
typedef struct {
    unsigned char  mode_ctl;         /* value of PS and ALS function             */
    unsigned char  psals_ctl;        /* value of PS and ALS control              */
    unsigned char  persist;          /* value of PS interrupt persistence        */
    unsigned char  intr;             /* interruption setting value               */
    unsigned short psth_upper;       /* threshold value of high level for PS     */
    unsigned short psth_low;         /* threshold value of low level for PS      */
    unsigned short alsth_upper;      /* threshold value of high level for ALS    */
    unsigned short alsth_low;        /* threshold value of low level for ALS     */
    unsigned short ps_offset;        /* offset value of PS     */
} INIT_ARG;

/* structure to read state value from sensor */
typedef struct {
    unsigned char als_state;         /* state value of ALS from sensor           */
    unsigned char ps_state;          /* state value of PS from sensor            */
} PWR_ST;

/* structure to activate sensor */
typedef struct {
    unsigned char power_als;         /* value of whether to start ALS or not     */
    unsigned char power_ps;          /* value of whether to start PS or not      */
    unsigned char intr;              /* value of whether to use interrupt or not */
} POWERON_ARG;

enum light_source {
	LIGHT_FLUORESCENT = 0,
	LIGHT_LED_BULB,
	LIGHT_INTERMEDIATE,
	LIGHT_SOLAR,
	LIGHT_CANDESCENT,
	LIGHT_SRC_MAX,
};

typedef struct {
    unsigned long  lux;
    unsigned long  d0;
    unsigned long  d1;
    unsigned char  gain_data0;
    unsigned char  gain_data1;
    unsigned short als_data0;
    unsigned short als_data1;
    unsigned long  ratio;
    enum light_source src;
    unsigned short alpha;
    unsigned short beta;
} CALC_DATA;

typedef struct {
    unsigned long positive;
    unsigned long decimal;
} CALC_ANS;

typedef struct {
    unsigned char time;
    unsigned char gain;
    unsigned char led_current;
} DEVICE_VAL;

/************ define parameter for register ************/
/* REG_SYSTEMCONTROL(0x40) */
#define REG_SW_NOTRESET     (0 << 7)
#define REG_SW_RESET        (1 << 7)
#define REG_INT_NOTRESET    (0 << 6)
#define REG_INT_RESET       (1 << 6)

/* REG_MODECONTROL(0x41) */
#define PWRON_ALS           (7)
#define PWRON_PS            (6)
#define PWRON_ALS_EN        (1 << PWRON_ALS)
#define PWRON_PS_EN         (1 << PWRON_PS)
#define PS_PULSE_200        (0 << 5)
#define PS_PULSE_330        (1 << 5)
#define NORMAL_MODE         (0 << 4)
#define LOW_NOISE_MODE      (1 << 4)
#define PWRON_PS_ALS        (0x05)

#define MEASUREMENT_MAX     (0x0C)

/* REG_ALSPSCONTROL(0x42) */
#define LEDCURRENT_025MA    (0)
#define LEDCURRENT_050MA    (1)
#define LEDCURRENT_100MA    (2)
#define LEDCURRENT_200MA    (3)
#define ALSGAIN_X1X1        (0x0 << 2)
#define ALSGAIN_X1X2        (0x1 << 2)
#define ALSGAIN_X2X2        (0x5 << 2)
#define ALSGAIN_X64X64      (0xA << 2)
#define ALSGAIN_X128X64     (0xE << 2)
#define ALSGAIN_X128X128    (0xF << 2)
#define REG_ALSPSCTL_MAX    (0x3F)

/* REG_PSCONTROL(0x43) */
#define PERSISTENCE_MAX     (0x0F)
#define PSGAIN_MAX          (0x20)
#define PSGAIN_X1           (0x0 << 4)
#define PSGAIN_X2           (0x1 << 4)
#define PSGAIN_X3           (0x2 << 4)
#define IR_FLAG_MASK        (0xC0)
#define IR_VERY_STRONG      (3 << 6)

/* REG_INTERRUPT(0x4A) */
#define PS_THH_ONLY         (0 << 4)
#define PS_THH_BOTH_HYS     (1 << 4)
#define PS_THH_BOTH_OUTSIDE (2 << 4)
#define PS_THH_BOTH_WINDOW  (3 << 4)
#define PS_INT_MODE_MASK    (0x30)
#define POLA_ACTIVEL        (0 << 3)
#define POLA_INACTIVEL      (1 << 3)
#define OUTPUT_ANYTIME      (0 << 2)
#define OUTPUT_LATCH        (1 << 2)
#define MODE_NONUSE         (0)
#define MODE_PROXIMITY      (1)
#define MODE_ILLUMINANCE    (2)
#define MODE_BOTH           (3)
#define REG_INTERRUPT_MAX   (0x3F)

/* moved mode of ALS or PS  */
#define CTL_STANDBY         (0)
#define CTL_STANDALONE      (1)

/* REG_PSTH(0x4B) */
#define REG_PSTH_MAX        (0xFFF)

/* REG_PSTL(0x4D) */
#define REG_PSTL_MAX        (0xFFF)

#define ALS_ON_DELAY_MS     300
#define ALS_DATA_DELAY_MS   250

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
int ps_als_init(void);
void ps_als_exit(void);
bool als_sensor_get_en_first(void);
ssize_t als_val_show(char *buf);
ssize_t ps_val_show(char *buf);
ssize_t als_status_show(char *buf);
ssize_t ps_status_show(char *buf);
ssize_t als_imit_show(char *buf);
ssize_t als_imit_store(const char *buf);
ssize_t als_properties_store(const char *buf);
ssize_t als_properties_show(char *buf);
ssize_t ps_properties_store(const char *buf);
ssize_t ps_properties_show(char *buf);
ssize_t ps_valid_show(char *buf);
ssize_t ps_valid_store(unsigned int valid);
ssize_t als_flush_store(unsigned int flush);
ssize_t ps_flush_store(unsigned int flush);
int32_t als_get_initialize_state(void);
int32_t ps_get_initialize_state(void);
#endif /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
int32_t ps_sensor_activate(bool enable);
int32_t als_sensor_activate(bool enable);
uint32_t ps_sensor_get_count(void);
void ps_calib_start(void);
void ps_get_threshold(uint16_t* th_detect, uint16_t* th_no_detect, uint16_t* distance_th);
void prox_cal_result_set(int8_t result);
void prox_cal_interrupt(int8_t cal_result);
#endif /* _ROHM_RPR0521_I2C_H_ */
