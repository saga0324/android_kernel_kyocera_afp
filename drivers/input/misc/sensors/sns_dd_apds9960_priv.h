/* This software is contributed or developed by KYOCERA Corporation.
 * (C) 2017 KYOCERA Corporation
 */
/*==============================================================================

    S E N S O R S   GESTURE, RGB, AMBIENT LIGHT AND PROXIMITY  D R I V E R

DESCRIPTION

   Implements the combo driver for Avago Gesture, RGB, ALS and Proximity Sensor APDS-9960
   This driver has 4 sub-modules:
   1. The common handler that provides the driver interface
   2. The RGB and CCT driver that handles RGB and CCT data type
   3. The ALS and proximity driver that handles ambient light and proximity data type
   4. The Gesture driver that handles Gesture data type

********************************************************************************
* Copyright (c) 2013, Avago Technologies.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     1. Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*     2. Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     3. Neither the name of Avago nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************

==============================================================================*/
/*-----------------------------------------------------------------------------
 * Copyright (c) 2012 - 2013 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
  -----------------------------------------------------------------------------*/
/*==============================================================================

                      EDIT HISTORY FOR FILE

  This section contains comments describing changes made to the module.
  Notice that changes are listed in reverse chronological order.

when         who     what, where, why
----------   ---     -----------------------------------------------------------
06-Sep-2013   KK      (ver 1.00) First draft
25-Sep-2013   KK      (ver 1.01) Verify Polling mode in DDK
                      Things to NOTE: Disable all active sensors before enabling gesture (no co-exist)
                                      If gesture is active, other sensors are not allowed to run
26-Sep-2013   KK      (ver 1.02) Fix PROXIMITY and IR_GESTURE calibration

06-Nov-2013   KK      (ver 1.03) Revise from apds9950 base code

07-Nov-2013   KK      (ver 1.04) Revise code base on input from Maansy

==============================================================================*/

#ifndef _SNSDAPDS9960PRIV_H
#define _SNSDAPDS9960PRIV_H
/*
#include "fixed_point.h"
#include "sns_ddf_util.h"
#ifndef ALSPRX_OEM_BUILD
#include "msg_diag_service.h"
#endif
*/

/*=======================================================================

                  INTERNAL DEFINITIONS

========================================================================*/

// Enable the following macro to view debug messages
//#define APDS9960_DD_DEBUG

#ifdef ALSPRX_OEM_BUILD
#define NULL  0
#define TRUE  true
#define FALSE false
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))
#define abs(a) (((a)>=0)?(a): (-a))
#endif

/* Register Addresses define */
#define APDS9960_DD_ENABLE_ADDR	      0x80
#define APDS9960_DD_ATIME_ADDR	      0x81
#define APDS9960_DD_WTIME_ADDR	      0x83
#define APDS9960_DD_AILTL_ADDR	      0x84
#define APDS9960_DD_AILTH_ADDR	      0x85
#define APDS9960_DD_AIHTL_ADDR	      0x86
#define APDS9960_DD_AIHTH_ADDR	      0x87
#define APDS9960_DD_PILT_ADDR	      0x89 // <-APDS9960_DD_PITLO_ADDR
#define APDS9960_DD_PIHT_ADDR	      0x8B // <-APDS9960_DD_PITHI_ADDR
#define APDS9960_DD_PERS_ADDR	      0x8C
#define APDS9960_DD_CONFIG1_ADDR	  0x8D // <-APDS9960_DD_CONFIG_ADDR
#define APDS9960_DD_PPULSE_ADDR	      0x8E
#define APDS9960_DD_CONTROL_ADDR      0x8F
#define APDS9960_DD_CONFIG2_ADDR      0x90 // <-APDS9960_DD_AUX_ADDR
#define APDS9960_DD_REV_ADDR	      0x91
#define APDS9960_DD_ID_ADDR           0x92
#define APDS9960_DD_STATUS_ADDR	      0x93
#define APDS9960_DD_CDATAL_ADDR	      0x94
#define APDS9960_DD_CDATAH_ADDR	      0x95
#define APDS9960_DD_RDATAL_ADDR	      0x96
#define APDS9960_DD_RDATAH_ADDR	      0x97
#define APDS9960_DD_GDATAL_ADDR	      0x98
#define APDS9960_DD_GDATAH_ADDR	      0x99
#define APDS9960_DD_BDATAL_ADDR	      0x9A
#define APDS9960_DD_BDATAH_ADDR	      0x9B
#define APDS9960_DD_PDATA_ADDR	      0x9C
#define APDS9960_DD_POFFSET_UR_ADDR   0x9D
#define APDS9960_DD_POFFSET_DL_ADDR   0x9E
#define	APDS9960_DD_CONFIG3_ADDR      0x9F // <-APDS9960_DD_CONFIG2_ADDR
#define	APDS9960_DD_GTHR_IN_ADDR      0xA0
#define APDS9960_DD_GTHR_OUT_ADDR     0xA1
#define	APDS9960_DD_GCONF1_ADDR       0xA2
#define	APDS9960_DD_GCONF2_ADDR	      0xA3
#define	APDS9960_DD_GOFFSET_U_ADDR    0xA4
#define	APDS9960_DD_GOFFSET_D_ADDR    0xA5
#define	APDS9960_DD_GPULSE_ADDR	      0xA6
#define	APDS9960_DD_GOFFSET_L_ADDR    0xA7
#define	APDS9960_DD_GOFFSET_R_ADDR    0xA9
#define	APDS9960_DD_GCONF3_ADDR	      0xAA
#define	APDS9960_DD_GCONF4_ADDR	      0xAB // <-APDS9960_DD_GCTRL_ADDR
#define	APDS9960_DD_GFUSE1_ADDR	      0xAC
#define	APDS9960_DD_GFUSE0_ADDR	      0xAD
#define	APDS9960_DD_GFIFO_LVL_ADDR    0xAE
#define	APDS9960_DD_GSTATUS_ADDR      0xAF

#define	APDS9960_DD_TEST2_ADDR	      0xC3	// use for soft_reset

#define	APDS9960_DD_GFIFO0_ADDR	      0xFC	// U
#define	APDS9960_DD_GFIFO1_ADDR	      0xFD	// D
#define	APDS9960_DD_GFIFO2_ADDR	      0xFE	// L
#define	APDS9960_DD_GFIFO3_ADDR	      0xFF	// R

#define APDS9960_DD_CMD_CLR_PS_INT    0xE5
#define APDS9960_DD_CMD_CLR_ALS_INT   0xE6
#define APDS9960_DD_CMD_CLR_ALL_INT   0xE7

/* Register Value define : ATIME */
#define APDS9960_DD_100MS_ADC_TIME        0xDB  /* 100.8ms integration time */
#define APDS9960_DD_50MS_ADC_TIME         0xED  /* 50.4ms integration time */
#define APDS9960_DD_27MS_ADC_TIME         0xF6  /* 27ms integration time */
#define APDS9960_DD_2_7MS_ADC_TIME        0xFF  /* 2.7ms integration time */

/* for PPULSE and GPULSE */
#define	APDS9960_DD_PULSE_LEN_4US	      0x00
#define	APDS9960_DD_PULSE_LEN_8US	      0x40
#define	APDS9960_DD_PULSE_LEN_16US	      0x80
#define	APDS9960_DD_PULSE_LEN_32US	      0xC0

/* Register Value define : CONTROL */
#define APDS9960_DD_AGAIN_1X              0x00  /* 1X ALS GAIN */
#define APDS9960_DD_AGAIN_4X              0x01  /* 4X ALS GAIN */
#define APDS9960_DD_AGAIN_16X             0x02  /* 16X ALS GAIN */
#define APDS9960_DD_AGAIN_64X             0x03  /* 64X ALS GAIN */

#define APDS9960_DD_PGAIN_1X              0x00  /* 1X PRX GAIN */
#define APDS9960_DD_PGAIN_2X              0x04  /* 2X PRX GAIN */
#define APDS9960_DD_PGAIN_4X              0x08  /* 4X PRX GAIN */
#define APDS9960_DD_PGAIN_8X              0x0C  /* 8X PRX GAIN */

#define APDS9960_DD_PDRIVE_100MA          0x00  /* PRX 100mA LED drive */
#define APDS9960_DD_PDRIVE_50MA           0x40  /* PRX 50mA LED drive */
#define APDS9960_DD_PDRIVE_25MA           0x80  /* PRX 25mA LED drive */
#define APDS9960_DD_PDRIVE_12_5MA         0xC0  /* PRX 12.5mA LED drive */

/* Register Value define : AUX */
#define APDS9960_DD_LED_BOOST_100         0x00  /* 100% LED Boost */
#define APDS9960_DD_LED_BOOST_150         0x10  /* 150% LED Boost */
#define APDS9960_DD_LED_BOOST_200         0x20  /* 200% LED Boost */
#define APDS9960_DD_LED_BOOST_300         0x30  /* 300% LED Boost */

/* Register Value define : STATUS */
#define	APDS9960_DD_ASAT_STATUS           0x80 /* ALS Saturation */
#define	APDS9960_DD_PSAT_STATUS	      0x40 /* PS Saturation - analog saturated, not a proximity detection */
#define	APDS9960_DD_PINT_STATUS	      0x20 /* PS Interrupt status */
#define	APDS9960_DD_AINT_STATUS	      0x10 /* ALS Interrupt status */
#define	APDS9960_DD_GINT_STATUS	      0x04 /* Gesture Interrupt status */
#define	APDS9960_DD_PVALID_STATUS	      0x02 /* PS data valid status */
#define	APDS9960_DD_AVALID_STATUS	      0x01 /* ALS data valid status */

/* Register Value define : GCONF1 */
#define	APDS9960_DD_GFIFO_1_LEVEL	      0x00
#define	APDS9960_DD_GFIFO_4_LEVEL	      0x40
#define	APDS9960_DD_GFIFO_8_LEVEL	      0x80
#define	APDS9960_DD_GFIFO_16_LEVEL	      0xC0

/* Register Value define : GCONF2 */
#define	APDS9960_DD_GTIME_0MS	          0x00
#define	APDS9960_DD_GTIME_2_8MS           0x01
#define	APDS9960_DD_GTIME_5_6MS           0x02
#define	APDS9960_DD_GTIME_8_4MS           0x03
#define	APDS9960_DD_GTIME_14MS            0x04
#define	APDS9960_DD_GTIME_22_4MS	  0x05
#define	APDS9960_DD_GTIME_30_8MS	  0x06
#define	APDS9960_DD_GTIME_39_2MS	  0x07

#define APDS9960_DD_GLDRIVE_100MA         0x00  /* Gesture 100mA LED drive */
#define APDS9960_DD_GLDRIVE_50MA          0x08  /* Gesture 50mA LED drive */
#define APDS9960_DD_GLDRIVE_25MA          0x10  /* Gesture 25mA LED drive */
#define APDS9960_DD_GLDRIVE_12_5MA        0x18  /* Gesture 12.5mA LED drive */

#define APDS9960_DD_GGAIN_1X              0x00  /* 1X Gesture GAIN */
#define APDS9960_DD_GGAIN_2X              0x20  /* 2X Gesture GAIN */
#define APDS9960_DD_GGAIN_4X              0x40  /* 4X Gesture GAIN */
#define APDS9960_DD_GGAIN_8X              0x60  /* 8X Gesture GAIN */

/* Register Value define : GST_CTRL */
#define APDS9960_DD_GFIFO_CLR             0x04  /* Clears the GFIFO, GINT, GVALID, GFIFO_OV and GFIFO_LVL */
#define APDS9960_DD_GIEN                  0x02  /* Gesture interrupt enable */
#define APDS9960_DD_GMODE                 0x01  /* GMODE */

/* Register Value define : GSTATUS */
#define APDS9960_DD_GFIFO_OV              0x02  /* Gesture FIFO overflow */
#define APDS9960_DD_GVALID                0x01  /* Gesture Valid */

/* attributes for gst data type */
/* The res and accuracy for ALS are not static values. They're calculated from the previous data */
#define APDS9960_DD_GST_PWR             11300 /* unit of uA */ /* LED average current + IC current */
#define APDS9960_DD_GST_LO_PWR          1   /* unit of uA */

/* attributes for rgb data type */
/* The res and accuracy for ALS are not static values. They're calculated from the previous data */
#define APDS9960_DD_CCT_PWR             200 /* unit of uA */
#define APDS9960_DD_CCT_RES             500 /* unit of this data type is K */
#define APDS9960_DD_CCT_LO_PWR          1   /* unit of uA */
#define APDS9960_DD_CCT_ACCURACY        500 /* unit of this data type is K */

/* attributes for rgb data type */
/* The res and accuracy for ALS are not static values. They're calculated from the previous data */
#define APDS9960_DD_RGB_PWR             200 /* unit of uA */
#define APDS9960_DD_RGB_RES             1   /* unit of this data type is ADC count */
#define APDS9960_DD_RGB_LO_PWR          1   /* unit of uA */
#define APDS9960_DD_RGB_ACCURACY        1   /* unit of this data is 1 ADC count */

/* attributes for als data type */
/* The res and accuracy for ALS are not static values. They're calculated from the previous data */
#define APDS9960_DD_ALS_PWR             200 /* unit of uA */
#define APDS9960_DD_ALS_RES             FX_FLTTOFIX_Q16(0.1)   /* unit of this data type is lux */
#define APDS9960_DD_ALS_LO_PWR          1       /* unit of uA */
#define APDS9960_DD_ALS_ACCURACY        FX_FLTTOFIX_Q16(0.1)   /* unit of this data type is lux */

/* attributes for proximity data type */
#define APDS9960_DD_PRX_PWR             370   /* unit of uA */ /* LED average current + IC current */
#define APDS9960_DD_PRX_LO_PWR          1     /* unit of uA */
#define APDS9960_DD_PRX_RES             FX_FLTTOFIX_Q16(0.001)   /* unit of this data type is meter */
#define APDS9960_DD_PRX_ACCURACY        1      /* unit of this data type unit which is mm */

/* attribute for NV items */

#define APDS9960_DD_PRX_CAL_THRESHOLD     100

#define APDS9960_DD_PRX_FACTOR                     20
#define APDS9960_DD_ALS_FACTOR                     100

#define APDS9960_DD_PRXDIST_TB_MAX_VALUE           255    /* 8 bits */
#define APDS9960_DD_NUM_SENSOR_TYPES               2
#define APDS9960_DD_MAX_SUB_DEV                    3
#define APDS9960_DD_HANDLE_ALIGN                   4

#define APDS9960_DD_ALS_CALIBRATED_LUX             300 /* 300 calibrated lux */

#define APDS9960_DD_GST_SENSITIVITY_LEVEL1	50
#define APDS9960_DD_GST_SENSITIVITY_LEVEL2	20

#define	APDS9960_DD_PPULSE_FOR_PS		8
#define APDS9960_DD_PPULSE_LEN_FOR_PS	        APDS9960_DD_PULSE_LEN_8US
#define APDS9960_DD_PDRIVE_FOR_PS		APDS9960_DD_PDRIVE_100MA
#define APDS9960_DD_PGAIN_FOR_PS		APDS9960_DD_PGAIN_2X

#define	APDS9960_DD_PPULSE_FOR_GESTURE		10
#define APDS9960_DD_PPULSE_LEN_FOR_GESTURE	APDS9960_DD_PULSE_LEN_16US
#define APDS9960_DD_PDRIVE_FOR_GESTURE		APDS9960_DD_PDRIVE_100MA
#define APDS9960_DD_PGAIN_FOR_GESTURE		APDS9960_DD_PGAIN_4X

#define APDS9960_DD_GPULSE		10
#define APDS9960_DD_GPULSE_LEN	        APDS9960_DD_PULSE_LEN_32US
#define	APDS9960_DD_GGAIN		APDS9960_DD_GGAIN_4X
#define APDS9960_DD_GTIME		APDS9960_DD_GTIME_2_8MS
#define APDS9960_DD_GDRIVE		APDS9960_DD_GLDRIVE_100MA

#define	APDS9960_DD_PS_LED_BOOST	APDS9960_DD_LED_BOOST_100
#define	APDS9960_DD_GESTURE_LED_BOOST	APDS9960_DD_LED_BOOST_100

#define APDS9960_DD_PRX_NEAR_THRESHOLD    100
#define APDS9960_DD_PRX_FAR_THRESHOLD     70

#define	APDS9960_DD_NEAR_THRESHOLD_LOW	0
#define APDS9960_DD_FAR_THRESHOLD_HIGH	255

#define APDS9960_DD_PS_CALIBRATED_XTALK_BASELINE  10
#define APDS9960_DD_PS_CALIBRATED_XTALK		  20

#define	APDS9960_DD_GST_GTHR_IN		40	// 50 open air, 40 for dark window
#define APDS9960_DD_GST_GTHR_OUT	30	// 40 open air, 30 for dark window

#define APDS9960_DD_GST_THRESHOLD_OUT (APDS9960_DD_GST_GTHR_OUT-10)

/*=======================================================================

                  Macros

========================================================================*/
/* Negative ADC counts will be treated as zero */
#define ALSPRX_CONV_TO_UNSIGNED(var, bits) ((var & (1<<(bits-1))) ? (0) : (var))

#define APDS9960_DD_DEFAULT_ALS_CHANGE_PCNT APDS9960_DD_ALS_CHANGE_0_78_PCNT
#define PCNT_OF_RAW(x, raw) (raw>>x)

/*=======================================================================

                  TYPE DEFINITIONS

========================================================================*/

typedef enum
{
  APDS9960_DD_ALS_CHANGE_MIN_PCNT = 0,
  APDS9960_DD_ALS_CHANGE_50_PCNT  = 1,  /* x>>1 */
  APDS9960_DD_ALS_CHANGE_25_PCNT  = 2,  /* x>>2 */
  APDS9960_DD_ALS_CHANGE_12_5_PCNT  = 3,  /* x>>3 */
  APDS9960_DD_ALS_CHANGE_6_25_PCNT  = 4,  /* x>>4 */
  APDS9960_DD_ALS_CHANGE_3_125_PCNT  = 5,  /* x>>5 */
  APDS9960_DD_ALS_CHANGE_1_56_PCNT  = 6,  /* x>>6 */
  APDS9960_DD_ALS_CHANGE_0_78_PCNT  = 7,  /* x>>7 */
  APDS9960_DD_ALS_CHANGE_MAX_PCNT
} apds9960_dd_als_change_pcnt_e;

typedef enum
{
  APDS9960_DD_RES_14BIT  = 0,  /* 27ms integration time */
  APDS9960_DD_RES_15BIT = 1,   /* 50ms integration time */
  APDS9960_DD_RES_16BIT = 2   /* 100ms integration time */
} apds9960_dd_res_e;

typedef enum
{
  APDS9960_DD_ALS_GAIN_1X    = 0,    /* 1x AGAIN */
  APDS9960_DD_ALS_GAIN_4X    = 1,    /* 4x AGAIN */
  APDS9960_DD_ALS_GAIN_16X   = 2,    /* 16x AGAIN */
  APDS9960_DD_ALS_GAIN_64X   = 3     /* 64x AGAIN */
} apds9960_dd_als_gain_e;

typedef enum
{
  APDS9960_DD_PEND_STATE_IDLE,                     /* idle state */
  APDS9960_DD_PEND_STATE_PEND,                     /* waiting a response */
  APDS9960_DD_PEND_STATE_RSVD                      /* reserved */
} apds9960_dd_pend_state_e;

typedef enum
{
  APDS9960_DD_PRX_FAR_AWAY,
  APDS9960_DD_PRX_NEAR_BY,
  APDS9960_DD_PRX_NEAR_BY_UNKNOWN
} apds9960_dd_prx_nearby_e;

/* data structure for proximity common handler */

/* The state of a device */
typedef enum
{
  APDS9960_DD_DEV_STOPPED,                         /* 0 */
  APDS9960_DD_DEV_CONFIGURED,                      /* 1 */
  APDS9960_DD_DEV_GET_DATA,                        /* 2 */
  APDS9960_DD_DEV_MAX_STATES                       /* 3 */
}apds9960_dd_device_state_e;

typedef enum
{
  APDS9960_DD_NV_SOURCE_DEFAULT,
  APDS9960_DD_NV_SOURCE_REG
}apds9960_dd_nv_source_e;

/* Error codes indicating reasons for test failures */
typedef enum
{
    APDS9960_DD_SUCCESS        = 0,
    APDS9960_DD_PRX_CAL_FAILED = 1,
    APDS9960_DD_ALS_CAL_FAILED = 2,
    APDS9960_DD_GST_CAL_FAILED = 3
} apds9960_dd_test_err_e;

typedef enum
{

  DIR_NONE,
  DIR_LEFT,
  DIR_RIGHT,
  DIR_UP,
  DIR_DOWN,
  DIR_UP_RIGHT,
  DIR_RIGHT_DOWN,
  DIR_DOWN_LEFT,
  DIR_LEFT_UP,
  DIR_NEAR,
  DIR_FAR,
  DIR_CIRCLE_CW,
  DIR_CIRCLE_ACW,
  DIR_UP_U_TURN,
  DIR_DOWN_U_TURN,
  DIR_LEFT_U_TURN,
  DIR_RIGHT_U_TURN,
  DIR_ALL
} apds9960_dd_gst_event_e;

typedef enum {
  NA_STATE,
  NEAR_STATE,
  FAR_STATE,
  CIRCLE_CW_STATE,
  CIRCLE_ACW_STATE,
  ALL_STATE
} apds9960_dd_gst_state_e;

typedef enum
{
  CIRCLE_NA,
  CIRCLE_0,
  CIRCLE_1,
  CIRCLE_2,
  CIRCLE_3,
  CIRCLE_4,
  CIRCLE_5,
  CIRCLE_6,
  CIRCLE_7,
  CIRCLE_ALL
} apds9960_dd_gst_circle_state_e;

typedef enum
{
  GESTURE_ZONE_NA, // init
  GESTURE_ZONE_1, // -ve UD, +ve LR
  GESTURE_ZONE_2, // +ve UD, +ve LR
  GESTURE_ZONE_3, // +ve UD, -ve LR
  GESTURE_ZONE_4, // -ve UD, -ve LR
  GESTURE_ZONE_UNKNOWN, // unknown
  GESTURE_ALL_ZONE
} apds9960_dd_gst_zone_e;

/* structure to set initial value to sensor */
typedef struct {
	unsigned char  dd_enable;
	unsigned char  dd_atime;
	unsigned char  dd_reserved_82;
	unsigned char  dd_wtime;
	unsigned char  dd_ailtl;
	unsigned char  dd_ailth;
	unsigned char  dd_aihtl;
	unsigned char  dd_aihth;
	unsigned char  dd_reserved_88;
	unsigned char  dd_pitlo;
	unsigned char  dd_reserved_8a;
	unsigned char  dd_pithi;
	unsigned char  dd_pers;
	unsigned char  dd_config1;
	unsigned char  dd_ppulse;
	unsigned char  dd_control;
	unsigned char  dd_config2;
	unsigned char  dd_reserved_91;
	unsigned char  dd_id;
	unsigned char  dd_status;
	unsigned char  dd_reserved_94_9c[9];
	unsigned char  dd_poffset_ur;
	unsigned char  dd_poffset_dl;
	unsigned char  dd_config3;
	unsigned char  dd_gthr_in;
	unsigned char  dd_gthr_out;
	unsigned char  dd_gconf1;
	unsigned char  dd_gconf2;
	unsigned char  dd_goffset_u;
	unsigned char  dd_goffset_d;
	unsigned char  dd_gpulse;
	unsigned char  dd_goffset_l;
	unsigned char  dd_reserved_a8;
	unsigned char  dd_goffset_r;
	unsigned char  dd_gconf3;
	unsigned char  dd_gconf4;
}apds9960_dd_init_arg;

#define APDS9960_DD_ATIME_DIS				(0xDD)
#define APDS9960_DD_ATIME_PS_EN				(0xEE)
#define APDS9960_DD_ATIME_GS_EN				(0xF0)
#define APDS9960_DD_ATIME_MAX				(0xFF)

#define APDS9960_DD_PPULSE_GS_DIS			(0x88)
#define APDS9960_DD_PPULSE_GS_EN			(0xD8)
#define APDS9960_DD_CONTROL_GS_DIS			(0x01)
#define APDS9960_DD_CONTROL_GS_EN			(0x01)
#define APDS9960_DD_CONFIG2_GS_DIS			(0x11)
#define APDS9960_DD_CONFIG2_GS_EN			(0x21)
#define APDS9960_DD_GCONF4_GS_DIS			(0x00)
#define APDS9960_DD_GCONF4_GS_EN			(0x02)
#define APDS9960_DD_GCONF4_FIFO_CLR			(0x04)
#define APDS9960_DD_GCONF4_GS_EN_MASK		(0x02)

#define APDS9960_DD_GSTATUS_GFOV_MASK		(0x02)
#define APDS9960_DD_GSTATUS_FIFO_CLEAR		(0x04)
#define APDS9960_DD_GFIFO_LVL_MAX			(0xFF)
#define APDS9960_DD_GFIFO_MAX				(APDS9960_DD_GFIFO_LVL_MAX * 4)
#define APDS9960_DD_GFIFO_DATA_MAX			(0xFF)


/*      APDS9960_DD_ENABLE_ADDR 0x80 */
#define APDS9960_DD_ENABLE_NONE				(0x00)
#define APDS9960_DD_ENABLE_POWER			(0x01)
#define APDS9960_DD_ENABLE_ALS				(0x02)
#define APDS9960_DD_ENABLE_PS				(0x04)
#define APDS9960_DD_ENABLE_WAIT				(0x08)
#define APDS9960_DD_ENABLE_ALS_INT			(0x10)
#define APDS9960_DD_ENABLE_PS_INT			(0x20)
#define APDS9960_DD_ENABLE_GS				(0x40)
#define APDS9960_DD_ENABLE_MAX				(0x7F)

#define APDS9960_DD_POWER_DISABLE		(0x00)
#define APDS9960_DD_POWER_ENABLE		(0x01)

#define APDS9960_DD_CONFIRM_START_REG		(APDS9960_DD_ENABLE_ADDR)
#define APDS9960_DD_CONFIRM_END_REG			(APDS9960_DD_GCONF4_ADDR)
#define APDS9960_DD_CONFIRM_REGS_SIZE		(APDS9960_DD_CONFIRM_END_REG - APDS9960_DD_CONFIRM_START_REG + 1)

#define GESTURE_STATUS_GMODE_EXIT			(1)

#define APDS9960_DD_I2C_NAME "sns_dd_apds9960"

typedef struct {
	unsigned char	gs_lvl;
	unsigned char	gs_data[APDS9960_DD_GFIFO_MAX];
	unsigned char	gs_gmode;
} READ_DATA_GS_BUF;

#define ENABLE_ALS				(0x01)
#define ENABLE_PS				(0x02)
#define ENABLE_GS				(0x04)

#define SM_TIME_UNIT       (1000)
#define MN_TIME_UNIT       (1000000)
#define PS_DATA_MASK       (0x7FFF)
#define PS_FLAG_MASK       (0x8000)

#ifdef _ALS_BIG_ENDIAN_
#define CONVERT_TO_BE(value) ((((value) >> 8) & 0xFF) | (((value) << 8) & 0xFF00))
#else
#define CONVERT_TO_BE(value) (value)
#endif

/* structure to read data value from sensor */
typedef struct {
	unsigned short ps_data;			 /* data value of PS data from sensor		 */
	unsigned short als_data0;		 /* data value of ALS data0 from sensor		 */
	unsigned short als_data1;		 /* data value of ALS data1 from sensor		 */
} READ_DATA_BUF;

typedef struct {
	unsigned short als_data0;		 /* data value of ALS data0 from sensor		 */
	unsigned short als_data1;		 /* data value of ALS data1 from sensor		 */
	unsigned char atime;
} READ_DATA_ALS_BUF;

typedef struct {
	unsigned char  ps_intr;			 /* data value of interrupt from sensor		 */
	unsigned char  ps_ctrl;			 /* data value of PS control from sensor	 */
	unsigned short ps_data;			 /* data value of PS data from sensor		 */
	unsigned char  ps_flag;			 /* data value of PS_INT_TH_FLAG from sensor */
} READ_DATA_PS_BUF;

/* structure to set initial value to sensor */
typedef struct {
	unsigned char  mode_ctl;		 /* value of PS and ALS function			 */
	unsigned char  psals_ctl;		 /* value of PS and ALS control				 */
	unsigned char  persist;			 /* value of PS interrupt persistence		 */
	unsigned char  intr;			 /* interruption setting value				 */
	unsigned short psth_upper;		 /* threshold value of high level for PS	 */
	unsigned short psth_low;		 /* threshold value of low level for PS		 */
	unsigned short alsth_upper;		 /* threshold value of high level for ALS	 */
	unsigned short alsth_low;		 /* threshold value of low level for ALS	 */
	unsigned short ps_offset;		 /* offset value of PS	   */
} INIT_ARG;

/* structure to read state value from sensor */
typedef struct {
	unsigned char als_state;		 /* state value of ALS from sensor			 */
	unsigned char ps_state;			 /* state value of PS from sensor			 */
	unsigned char gs_state;
} PWR_ST;

/* structure to activate sensor */
typedef struct {
	unsigned char power_als;		 /* value of whether to start ALS or not	 */
	unsigned char power_ps;			 /* value of whether to start PS or not		 */
	unsigned char power_gs;
	unsigned char intr;				 /* value of whether to use interrupt or not */
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
	unsigned char  atime;
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

/* moved mode of ALS or PS  */
#define CTL_STANDBY         (0)
#define CTL_STANDALONE      (1)

/* REG_PSTH */
#define REG_PSTH_MAX        (0xFFF)

/* REG_PSTL */
#define REG_PSTL_MAX        (0xFFF)

#define ALS_ON_DELAY_MS     300
#define ALS_DATA_DELAY_MS   250

#define GS_CAL_CNT          20
#define GS_MAX_OFFSET       200
#define PS_MAX_OFFSET       200

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
int apds_init(void);
void apds_exit(void);
bool als_sensor_get_en_first(void);
int32_t ps_sensor_activate(bool enable);
int32_t als_sensor_activate(bool enable);
int32_t gs_sensor_activate(bool enable);
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
ssize_t gs_properties_store(const char *buf);
ssize_t gs_properties_show(char *buf);
ssize_t ps_valid_show(char *buf);
ssize_t ps_valid_store(unsigned int valid);
ssize_t als_flush_store(unsigned int flush);
ssize_t ps_flush_store(unsigned int flush);
ssize_t apds_err_monitor_enable_store(const char *buf);
ssize_t apds_err_monitor_enable_show(char *buf);
ssize_t apds_irq_enable_store(const char *buf);
uint32_t ps_sensor_get_count(void);
int32_t als_get_initialize_state(void);
int32_t ps_get_initialize_state(void);
int32_t gs_get_initialize_state(void);
uint32_t gs_get_ignore_time(void);
void gs_set_ignore_time(uint32_t time_ms);
void apds_gs_calibration(void);
#endif /*CONFIG_USE_MICON_SOFT_STRUCTURE*/

// IF
/* structure to activate sensor */
typedef struct {
	unsigned char power_als; /* value of whether to start ALS or not */
	unsigned char intr;		 /* value of whether to use interrupt or not */
} POWERON_ALS_ARG;

/* structure to activate sensor */
typedef struct {
	unsigned char power_ps;	 /* value of whether to start PS or not */
	unsigned char intr;		 /* value of whether to use interrupt or not */
} POWERON_PS_ARG;

/* structure to read register value from sensor */
typedef struct {
	unsigned char adr_reg;	 /* start register value */
	unsigned char *addr;	 /* address to save value which read from sensor */
	unsigned char size;		 /* length to read */
} GENREAD_ARG;

#endif /* End include guard  _SNSDAPDS9960PRIV_H */
