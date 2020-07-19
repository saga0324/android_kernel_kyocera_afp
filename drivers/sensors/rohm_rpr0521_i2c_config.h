/* This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 */
/******************************************************************************
 * MODULE     : rohm_rpr0521_i2c_config.h
 * FUNCTION   : Configuration header for RPR0521, Ambient Light Sensor(ALS) IC
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
#ifndef _ROHM_RPR0521_I2C_CONFIG_H_
#define _ROHM_RPR0521_I2C_CONFIG_H_

#include "rohm_rpr0521_i2c.h"

/* recommend value
   #define PS_ALS_SET_MODE_CONTROL   (NORMAL_MODE | PS_PULSE_200 | PWRON_PS_ALS)
   #define PS_ALS_SET_ALSPS_CONTROL  (LEDCURRENT_100MA | ALSGAIN_X1X1)
   #define PS_ALS_SET_INTR_PERSIST   (3)
   #define PS_ALS_SET_INTR           (PS_THH_ONLY | POLA_ACTIVEL | OUTPUT_LATCH | MODE_NONUSE)
   #define PS_ALS_SET_PS_TH          (50)
   #define PS_ALS_SET_PS_TL          (0x000)
   #define PS_ALS_SET_ALS_TH         (0xFFFF)
   #define PS_ALS_SET_ALS_TL         (0x0000)
   #define PS_ALS_SET_MIN_DELAY_TIME (125)

   #define COEFFICIENT (4)
    const unsigned long data0_coefficient[COEFFICIENT] = {192,141,127,117};
    const unsigned long data1_coefficient[COEFFICIENT] = {316,108, 86, 74};
    const unsigned long judge_coefficient[COEFFICIENT] = { 29, 65, 85,158};
 */

#define PS_ALS_SET_MODE_CONTROL   (LOW_NOISE_MODE | PS_PULSE_330 | PWRON_PS_ALS)
#define PS_ALS_SET_ALSPS_CONTROL  (LEDCURRENT_200MA | ALSGAIN_X2X2)
#define PS_ALS_SET_INTR_PERSIST   (PSGAIN_X2 | 0x1)
//#define PS_ALS_SET_INTR           (PS_THH_BOTH_WINDOW | POLA_ACTIVEL | OUTPUT_ANYTIME | MODE_PROXIMITY)
#define PS_ALS_SET_INTR           (PS_THH_BOTH_OUTSIDE | POLA_ACTIVEL | OUTPUT_ANYTIME | MODE_PROXIMITY)
#define PS_ALS_SET_PS_TH          (0x0400)
#define PS_ALS_SET_PS_TL          (0x0100)
#define PS_ALS_SET_ALS_TH         (0xFFFF)
#define PS_ALS_SET_ALS_TL         (0x0000)
#define PS_ALS_SET_PS_OFFSET      (0x0000)
#define PS_ALS_SET_MIN_DELAY_TIME (125)

#define COEFFICIENT               (4)
const long          data0_coefficient[COEFFICIENT] = {708, 852, 614, 207};
const long          data1_coefficient[COEFFICIENT] = {-16, 676, 380,  89};
const unsigned long judge_coefficient[COEFFICIENT] = { 20,  80, 140, 231};

/*
    PS_ALS_SET_MODE_CONTROL : ALS,PS function setting
    The input value is defined in rohm_rpr0521_i2c.h
    Select PS_PULSE and PS Operating mode and Measurement time
    Input selected definition to PS_ALS_SET_MODE_CONTROL.

    PS_ALS_SET_ALSPS_CONTROL : ALS/PS control setting
    Select only one definition from LEDCURRENT_025MA to LEDCURRENT_200MA.
    Select only one definition from ALSGAIN_X1X1 to ALSGAIN_X128X128.
    Add two definitions you selected.
    Input selected definition to PS_ALS_SET_ALSPS_CONTROL.

    PS_ALS_SET_INTR_PERSIST : Interrupt persistence setting
    Input the value from 0 to 15 to PS_ALS_SET_INTR_PERSIST.

    PS_ALS_SET_INTR : Interrupt setting
    The input value is defined in rohm_rpr0521_i2c.h
    Select only one definition from PS_THH_ONLY to PS_THH_BOTH_OUTSIDE.
    Select POLA_ACTIVEL or POLA_INACTIVEL.
    Select OUTPUT_ANYTIME or OUTPUT_LATCH.
    Select only one definition from MODE_NONUSE to MODE_BOTH.
    Add four definitions you selected.
    Input selected definition to PS_ALS_SET_INTR.

    PS_ALS_SET_PSTH_H : PS interrupt High threshold
    Input the value from 0 to 4095 to PS_ALS_SET_PSTH_H.

    PS_ALS_SET_PSTH_L : PS interrupt Low threshold
    Input the value from 0 to 4095 to PS_ALS_SET_PSTH_L.

    PS_ALS_SET_ALSTH_H : ALS data0 threshold high
    Input the value from 0 to 65535 to PS_ALS_SET_ALSTH_H.

    PS_ALS_SET_ALSTH_L : ALS data0 threshold low
    Input the value from 0 to 65535 to PS_ALS_SET_ALSTH_L.

    PS_ALS_SET_MIN_DELAY_TIME : set the minimum measuring time to update
    Input the value from 0 to 4294967295 to PS_ALS_SET_MIN_DELAY_TIME.

 */

#endif /* _ROHM_RPR0521_I2C_CONFIG_H_ */

