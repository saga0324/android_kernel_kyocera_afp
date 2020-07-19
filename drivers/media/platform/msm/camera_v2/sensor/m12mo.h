/* This program is free software; you can redistribute it and/or modify
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2017 KYOCERA Corporation
 */

#ifndef M12MO_H
#define M12MO_H

#include <linux/types.h>
#include "msm_sensor.h"

/***************** M12MO Definitions ******************/
#define M12MO_JPEG_RAW_SIZE_MAX         (8 * 1024 * 1024)   /* 8MB */
#define M12MO_JPEG_RAW_TRANSFER_NUM     (2)                 /* 2 frame */

#define M12MO_MAKERNOTE_ISP_DATA_SIZE   (1024)              /* 1024 Byte */

#define m12mo_get_aspect_ratio(w, h)    (((w) * 10 / (h)) - 10)

typedef struct _m12mo_rect_value_t {
    uint8_t set;
    uint16_t width;
    uint16_t height;
    uint16_t ratio;
} m12mo_rect_value_t;

typedef struct _m12mo_roi_holder_t {
    int is_valid;
    struct sensor_roi_info_t roi;
} m12mo_roi_holder_t;

#define M12MO_MON_SIZE_3_25M_WIDTH  (2080)
#define M12MO_MON_SIZE_3_25M_HEIGHT (1560)
#define M12MO_MON_SIZE_2_5M_WIDTH   (2080)
#define M12MO_MON_SIZE_2_5M_HEIGHT  (1170)
#define M12MO_MON_SIZE_1_33M_WIDTH  (1536)
#define M12MO_MON_SIZE_1_33M_HEIGHT (864)
#define M12MO_MON_SIZE_SVGA_WIDTH   (800)
#define M12MO_MON_SIZE_SVGA_HEIGHT  (600)

#define M12MO_LOG_MUTEX_LOCK(a)   {CDBG("%s:LOCK   " #a "\n", __func__); mutex_lock(&a);}
#define M12MO_LOG_MUTEX_UNLOCK(a) {CDBG("%s:UNLOCK " #a "\n", __func__); mutex_unlock(&a);}

#define M12MO_ZOOM_MIN_LEVEL (0)
#define M12MO_ZOOM_MAX_LEVEL (60)

enum m12mo_i2c_led_clients {
    M12MO_LED_CLIENT_CAMERA,
    M12MO_LED_CLIENT_LIGHTHAL,
};

/***************** M12MO I2C Commands *****************/

/* M12MO I2C Commands - command */
enum m12mo_i2c_command {
    M12MO_I2C_CMD_CATEGORY_READ = 1,
    M12MO_I2C_CMD_CATEGORY_WRITE,
    M12MO_I2C_CMD_MEMORY_BYTE_READ,
    M12MO_I2C_CMD_MEMORY_BYTE_WRITE,
    M12MO_I2C_CMD_MEMORY_WORD_READ,
    M12MO_I2C_CMD_MEMORY_WORD_WRITE,
    M12MO_I2C_CMD_MEMORY_DWORD_READ,
    M12MO_I2C_CMD_MEMORY_DWORD_WRITE,
};

/* M12MO I2C Commands - header size */
#define M12MO_I2C_CMD_CATEGORY_READ_HEAD_LEN    (5) /*   1: Category parameter RAM Read           */
#define M12MO_I2C_CMD_CATEGORY_RECEIVE_HEAD_LEN (1) /*      Category parameter RAM Read Receive   */
#define M12MO_I2C_CMD_CATEGORY_WRITE_HEAD_LEN   (4) /*   2: Category parameter RAM Write          */
#define M12MO_I2C_CMD_MEMORY_HEAD_LEN           (8) /* 3-8: M-12MO memory read/write (8/16/32bit) */
#define M12MO_I2C_CMD_MEMORY_RECEIVE_HEAD_LEN   (3) /*      M-12MO memory read receive            */

/* M12MO I2C Commands - length limit */
#define M12MO_I2C_CATEGORY_READ_MAX     (32)
#define M12MO_I2C_CATEGORY_WRITE_MAX    (28)
#define M12MO_I2C_MEMORY_DATA_MAX       (0x10000)

/**************** M12MO I2C Error Code ****************/
#define M12MO_I2C_ERR_EFFECT_COM_BYTE_NO    (0xF0)  /* -16: Effective communication byte number error */
#define M12MO_I2C_ERR_COMMAND_CODE          (0xF1)  /* -15: Command code error The command code       */
#define M12MO_I2C_ERR_CATEGORY              (0xF2)  /* -14: The command code is an unexpected value   */
#define M12MO_I2C_ERR_BYTE_NO               (0xF3)  /* -13: Invalid byte number was specified         */
#define M12MO_I2C_ERR_RW_BYTE_LONG          (0xF4)  /* -12: Reading/writing bytes long error          */
#define M12MO_I2C_ERR_MODE_BE_SWITCHED      (0xFA)  /*  -6: The mode is being switched                */


/*************** M12MO I2C IRQ Factors ****************/
enum m12mo_i2c_irq_factors {
    M12MO_I2C_IRQ_NONE,
    M12MO_I2C_IRQ_BOOT_COMPLETE,
    M12MO_I2C_IRQ_RESERVED2,
    M12MO_I2C_IRQ_SDRAM_ERROR,
    M12MO_I2C_IRQ_RAM_ACTIVATE,
    M12MO_I2C_IRQ_FLASHROM_UPDATE,
    M12MO_I2C_IRQ_HARD_ERROR,
};

/******* M12MO I2C Category Prameter - Category *******/
enum m12mo_i2c_category {   
    M12MO_I2C_CAT_SYS       = 0x00,
    M12MO_I2C_CAT_PRM_MOD   = 0x01,
    M12MO_I2C_CAT_MON       = 0x02,
    M12MO_I2C_CAT_AE        = 0x03,
    M12MO_I2C_CAT_WB        = 0x06,
    M12MO_I2C_CAT_EXIF      = 0x07,
    M12MO_I2C_CAT_LENS      = 0x0A,
    M12MO_I2C_CAT_CAP_PRM   = 0x0B,
    M12MO_I2C_CAT_CAP_CTL   = 0x0C,
    M12MO_I2C_CAT_TST       = 0x0D,
    M12MO_I2C_CAT_ADJ       = 0x0E,
    M12MO_I2C_CAT_INI       = 0x0F,     /* NOT DEFINED Specifications */
    M12MO_I2C_CAT_IQ_COM    = 0x10,
    M12MO_I2C_CAT_IQ_MON    = 0x11,
    M12MO_I2C_CAT_IQ_CAP    = 0x12,
    M12MO_I2C_CAT_IQ_MOV    = 0x13,
    M12MO_I2C_CAT_IQ_PRM    = 0x14,
};

/********* M12MO I2C Category Prameter - Byte *********/

/* M12MO I2C Category Prameter - 00: System Parameter */
enum m12mo_i2c_byte_system {
    M12MO_I2C_CAT_SYS_CUSTOMER_CODE             = 0x00,
    M12MO_I2C_CAT_SYS_PROJECT_CODE              = 0x01,
    M12MO_I2C_CAT_SYS_VER_FIRMWARE              = 0x02,
    M12MO_I2C_CAT_SYS_VER_FIRMWARE_L            = 0x03,
    M12MO_I2C_CAT_SYS_VER_HARDWARE              = 0x04,
    M12MO_I2C_CAT_SYS_VER_HARDWARE_L            = 0x05,
    M12MO_I2C_CAT_SYS_VER_PARAMETER             = 0x06,
    M12MO_I2C_CAT_SYS_VER_PARAMETER_L           = 0x07,
    M12MO_I2C_CAT_SYS_VER_AWB                   = 0x08,
    M12MO_I2C_CAT_SYS_VER_AWB_L                 = 0x09,
    M12MO_I2C_CAT_SYS_USER_VERSION_MANAGEMENT   = 0x0A,
    M12MO_I2C_CAT_SYS_SYS_MODE                  = 0x0B,
    M12MO_I2C_CAT_SYS_INT_ENABLE                = 0x10,
    M12MO_I2C_CAT_SYS_INT_FACTOR                = 0x1C,
    M12MO_I2C_CAT_SYS_HW_I2C_CHECK              = 0x46,
};

/* M12MO I2C Category Prameter - 01: Monitor&Still parameter A */
enum m12mo_i2c_byte_monitor_still_a {
    M12MO_I2C_CAT_PRM_MOD_MON_SIZE              = 0x01,
    M12MO_I2C_CAT_PRM_MOD_JPEG_DATA_TYPE        = 0x0A,
    M12MO_I2C_CAT_PRM_MOD_SEND_UNIT             = 0x18,
    M12MO_I2C_CAT_PRM_MOD_SEND_UNIT_2           = 0x19,
};

/* M12MO I2C Category Prameter - 02: Monitor&Still parameter B */
enum m12mo_i2c_byte_monitor_still_b {
    M12MO_I2C_CAT_MON_AHS_MON                   = 0x00,
    M12MO_I2C_CAT_MON_ZOOM                      = 0x01,
    M12MO_I2C_CAT_MON_EFFECT                    = 0x14,
    M12MO_I2C_CAT_MON_ATSCENE_LOCK              = 0x1A,
    M12MO_I2C_CAT_MON_ATSCENE_MANUAL            = 0x1B,
    M12MO_I2C_CAT_MON_ATSCENE_DETECT_START      = 0x1C,
    M12MO_I2C_CAT_MON_ATSCENE_PARAM_UPDATE      = 0x1D,
    M12MO_I2C_CAT_MON_ATSCENE_TIME_MONTH        = 0x23,
    M12MO_I2C_CAT_MON_ATSCENE_TIME_HOUR         = 0x24,
    M12MO_I2C_CAT_MON_ATSCENE_TIME_MINUTE       = 0x25,
    M12MO_I2C_CAT_MON_AHS_MON_STATUS            = 0x26,
    M12MO_I2C_CAT_MON_CONTRAST_CTL              = 0x27,
    M12MO_I2C_CAT_MON_CURRENT_SCENE             = 0x36,
    M12MO_I2C_CAT_MON_FPS_UPDATE                = 0x6A,
    M12MO_I2C_CAT_MON_FPS_CTRL                  = 0x6B,
    M12MO_I2C_CAT_MON_ROI_UPDATE_TRG            = 0xA0,
    M12MO_I2C_CAT_MON_ROI_LEFT_POSITION         = 0xA1,
};

/* M12MO I2C Category Prameter - 03: Exposure Control */
enum m12mo_i2c_byte_exposure {
    M12MO_I2C_CAT_AE_AE_LOCK                    = 0x00,
    M12MO_I2C_CAT_AE_AE_MODE                    = 0x01,
    M12MO_I2C_CAT_AE_AE_TARGET                  = 0x02,
    M12MO_I2C_CAT_AE_AE_SPEED                   = 0x03,
    M12MO_I2C_CAT_AE_AE_TARGET_MODE             = 0x04,
    M12MO_I2C_CAT_AE_ISOSEL                     = 0x05,
    M12MO_I2C_CAT_AE_FLICKER                    = 0x06,
    M12MO_I2C_CAT_AE_FLICKER_AUTO_RESULT        = 0x07,
    M12MO_I2C_CAT_AE_FLICKER_SELECT             = 0x08,
    M12MO_I2C_CAT_AE_EV_BIAS                    = 0x09,
    M12MO_I2C_CAT_AE_AE_PARAM_UPDATE            = 0x0D,
    M12MO_I2C_CAT_AE_NOW_GAIN                   = 0x0E,
    M12MO_I2C_CAT_AE_NOW_GAIN_L                 = 0x0F,
    M12MO_I2C_CAT_AE_NOW_EXPOSURE               = 0x10,
    M12MO_I2C_CAT_AE_NOW_EXPOSURE_L             = 0x11,
    M12MO_I2C_CAT_AE_MANUAL_GAIN_MON            = 0x12,
    M12MO_I2C_CAT_AE_MANUAL_GAIN_MON_L          = 0x13,
    M12MO_I2C_CAT_AE_MANUAL_SHUT_MON            = 0x14,
    M12MO_I2C_CAT_AE_MANUAL_SHUT_MON_L          = 0x15,
    M12MO_I2C_CAT_AE_MAX_EXPOSURE_MON           = 0x16,
    M12MO_I2C_CAT_AE_MAX_EXPOSURE_MON_L         = 0x17,
    M12MO_I2C_CAT_AE_MAX_EXPOSURE_CAP           = 0x18,
    M12MO_I2C_CAT_AE_MAX_EXPOSURE_CAP_L         = 0x19,
    M12MO_I2C_CAT_AE_MAX_GAIN_MON               = 0x1A,
    M12MO_I2C_CAT_AE_MAX_GAIN_MON_L             = 0x1B,
    M12MO_I2C_CAT_AE_MAX_GAIN_CAP               = 0x1C,
    M12MO_I2C_CAT_AE_MAX_GAIN_CAP_L             = 0x1D,
    M12MO_I2C_CAT_AE_NOW_EV                     = 0x1E,
    M12MO_I2C_CAT_AE_NOW_EV_L                   = 0x1F,
    M12MO_I2C_CAT_AE_AUTO_BRACCKET_EV           = 0x20,
    M12MO_I2C_CAT_AE_INITIAL_EV                 = 0x2A,
    M12MO_I2C_CAT_AE_INITIAL_EV_L               = 0x2B,
    M12MO_I2C_CAT_AE_NOW_BV                     = 0x32,
    M12MO_I2C_CAT_AE_NOW_BV_L                   = 0x33,
    M12MO_I2C_CAT_AE_NOW_ISO                    = 0x37,
    M12MO_I2C_CAT_AE_NOW_ISO_L                  = 0x38,
    M12MO_I2C_CAT_AE_LED_LEVEL                  = 0x3A,
    M12MO_I2C_CAT_AE_STROBE_EN                  = 0x3C,
    M12MO_I2C_CAT_AE_STROBE_STATUS              = 0x3D,
    M12MO_I2C_CAT_AE_AUTO_BRACKET_EV1           = 0x98,
    M12MO_I2C_CAT_AE_AUTO_BRACKET_EV2           = 0x99,
    M12MO_I2C_CAT_AE_ROI_UPDATE_TRG             = 0xA0,
    M12MO_I2C_CAT_AE_ROI_LEFT_POSITION          = 0xA1,
    M12MO_I2C_CAT_AE_ROI_LEFT_POSITION_L        = 0xA2,
    M12MO_I2C_CAT_AE_ROI_TOP_POSITION           = 0xA3,
    M12MO_I2C_CAT_AE_ROI_TOP_POSITION_L         = 0xA4,
    M12MO_I2C_CAT_AE_ROI_RIGHT_POSITION         = 0xA5,
    M12MO_I2C_CAT_AE_ROI_RIGHT_POSITION_L       = 0xA6,
    M12MO_I2C_CAT_AE_ROI_BOTTOM_POSITION        = 0xA7,
    M12MO_I2C_CAT_AE_ROI_BOTTOM_POSITION_L      = 0xA8,
};

/* M12MO I2C Category Prameter - 06: White balance control */
enum m12mo_i2c_byte_white_balance {
    M12MO_I2C_CAT_WB_AWB_LOCK                   = 0x00,
    M12MO_I2C_CAT_WB_AWB_MODE                   = 0x02,
    M12MO_I2C_CAT_WB_AWB_MANUAL                 = 0x03,
    M12MO_I2C_CAT_WB_AWB_SPEED                  = 0x04,
    M12MO_I2C_CAT_WB_AWB_RANGE                  = 0x05,
    M12MO_I2C_CAT_WB_AWB_DETECT_TEMP            = 0x06,
    M12MO_I2C_CAT_WB_AWB_DETECT_TEMP0           = 0x07,
    M12MO_I2C_CAT_WB_AWB_DETECT_COLOR_X         = 0x08,
    M12MO_I2C_CAT_WB_AWB_DETECT_COLOR_X0        = 0x09,
    M12MO_I2C_CAT_WB_AWB_DETECT_COLOR_Y         = 0x0A,
    M12MO_I2C_CAT_WB_AWB_DETECT_COLOR_Y0        = 0x0B,
    M12MO_I2C_CAT_WB_AWB_PARAM_UPDATE           = 0x0C,
};

/* M12MO I2C Category Prameter - 07: Exif information */
enum m12mo_i2c_byte_exif {
    M12MO_I2C_CAT_EXIF_INFO_EXPTIME             = 0x00,
    M12MO_I2C_CAT_EXIF_INFO_EXPTIME6            = 0x01,
    M12MO_I2C_CAT_EXIF_INFO_EXPTIME5            = 0x02,
    M12MO_I2C_CAT_EXIF_INFO_EXPTIME4            = 0x03,
    M12MO_I2C_CAT_EXIF_INFO_EXPTIME3            = 0x04,
    M12MO_I2C_CAT_EXIF_INFO_EXPTIME2            = 0x05,
    M12MO_I2C_CAT_EXIF_INFO_EXPTIME1            = 0x06,
    M12MO_I2C_CAT_EXIF_INFO_EXPTIME0            = 0x07,
    M12MO_I2C_CAT_EXIF_INFO_TV                  = 0x08,
    M12MO_I2C_CAT_EXIF_INFO_TV6                 = 0x09,
    M12MO_I2C_CAT_EXIF_INFO_TV5                 = 0x0A,
    M12MO_I2C_CAT_EXIF_INFO_TV4                 = 0x0B,
    M12MO_I2C_CAT_EXIF_INFO_TV3                 = 0x0C,
    M12MO_I2C_CAT_EXIF_INFO_TV2                 = 0x0D,
    M12MO_I2C_CAT_EXIF_INFO_TV1                 = 0x0E,
    M12MO_I2C_CAT_EXIF_INFO_TV0                 = 0x0F,
    M12MO_I2C_CAT_EXIF_INFO_AV                  = 0x10,
    M12MO_I2C_CAT_EXIF_INFO_AV6                 = 0x11,
    M12MO_I2C_CAT_EXIF_INFO_AV5                 = 0x12,
    M12MO_I2C_CAT_EXIF_INFO_AV4                 = 0x13,
    M12MO_I2C_CAT_EXIF_INFO_AV3                 = 0x14,
    M12MO_I2C_CAT_EXIF_INFO_AV2                 = 0x15,
    M12MO_I2C_CAT_EXIF_INFO_AV1                 = 0x16,
    M12MO_I2C_CAT_EXIF_INFO_AV0                 = 0x17,
    M12MO_I2C_CAT_EXIF_INFO_BV                  = 0x18,
    M12MO_I2C_CAT_EXIF_INFO_BV6                 = 0x19,
    M12MO_I2C_CAT_EXIF_INFO_BV5                 = 0x1A,
    M12MO_I2C_CAT_EXIF_INFO_BV4                 = 0x1B,
    M12MO_I2C_CAT_EXIF_INFO_BV3                 = 0x1C,
    M12MO_I2C_CAT_EXIF_INFO_BV2                 = 0x1D,
    M12MO_I2C_CAT_EXIF_INFO_BV1                 = 0x1E,
    M12MO_I2C_CAT_EXIF_INFO_BV0                 = 0x1F,
    M12MO_I2C_CAT_EXIF_INFO_EBV                 = 0x20,
    M12MO_I2C_CAT_EXIF_INFO_EBV6                = 0x21,
    M12MO_I2C_CAT_EXIF_INFO_EBV5                = 0x22,
    M12MO_I2C_CAT_EXIF_INFO_EBV4                = 0x23,
    M12MO_I2C_CAT_EXIF_INFO_EBV3                = 0x24,
    M12MO_I2C_CAT_EXIF_INFO_EBV2                = 0x25,
    M12MO_I2C_CAT_EXIF_INFO_EBV1                = 0x26,
    M12MO_I2C_CAT_EXIF_INFO_EBV0                = 0x27,
    M12MO_I2C_CAT_EXIF_INFO_ISO                 = 0x28,
    M12MO_I2C_CAT_EXIF_INFO_ISO0                = 0x29,
    M12MO_I2C_CAT_EXIF_INFO_FLASH               = 0x2A,
    M12MO_I2C_CAT_EXIF_INFO_FLASH0              = 0x2B,
    M12MO_I2C_CAT_EXIF_INFO_SDR                 = 0x2C,
    M12MO_I2C_CAT_EXIF_INFO_SDR0                = 0x2D,
    M12MO_I2C_CAT_EXIF_INFO_QVAL                = 0x2E,
    M12MO_I2C_CAT_EXIF_INFO_QVAL0               = 0x2F,
};

/* M12MO I2C Category Prameter - 0A: AF / Optical Zoom Control */
enum m12mo_i2c_byte_lens_control {
    M12MO_I2C_CAT_LENS_AF_MODE                  = 0x00,
    M12MO_I2C_CAT_LENS_AF_RANGE                 = 0x01,
    M12MO_I2C_CAT_LENS_AF_START                 = 0x02,
    M12MO_I2C_CAT_LENS_AF_RESULT                = 0x03,
    M12MO_I2C_CAT_LENS_CAF_STATUS               = 0x04,
    M12MO_I2C_CAT_LENS_AF_BUSY                  = 0x0B,
    M12MO_I2C_CAT_LENS_AF_AREA_MODE             = 0x30,
    M12MO_I2C_CAT_LENS_AF_AE_MODE               = 0x31,
    M12MO_I2C_CAT_LENS_AF_ROI_UPDATE_TRG        = 0xA0,
    M12MO_I2C_CAT_LENS_AF_ROI_LEFT_POSITION     = 0xA1,
    M12MO_I2C_CAT_LENS_AF_ROI_LEFT_POSITION_L   = 0xA2,
    M12MO_I2C_CAT_LENS_AF_ROI_TOP_POSITION      = 0xA3,
    M12MO_I2C_CAT_LENS_AF_ROI_TOP_POSITION_L    = 0xA4,
    M12MO_I2C_CAT_LENS_AF_ROI_RIGHT_POSITION    = 0xA5,
    M12MO_I2C_CAT_LENS_AF_ROI_RIGHT_POSITION_L  = 0xA6,
    M12MO_I2C_CAT_LENS_AF_ROI_BOTTOM_POSITION   = 0xA7,
    M12MO_I2C_CAT_LENS_AF_ROI_BOTTOM_POSITION_L = 0xA8,
};

/* M12MO I2C Category Prameter - 0B: Still Picture Parameter */
enum m12mo_i2c_byte_still_parameter {
    M12MO_I2C_CAT_CAP_PRM_YUVOUT_MAIN           = 0x00,
    M12MO_I2C_CAT_CAP_PRM_MAIN_IMAGE_SIZE       = 0x01,
    M12MO_I2C_CAT_CAP_PRM_YUVOUT_THUMB          = 0x0A,
    M12MO_I2C_CAT_CAP_PRM_THUMB_IMAGE_SIZE      = 0x0B,
    M12MO_I2C_CAT_CAP_PRM_JPEG_SIZE_MAX         = 0x0F,
    M12MO_I2C_CAT_CAP_PRM_JPEG_SIZE_MAX2        = 0x10,
    M12MO_I2C_CAT_CAP_PRM_JPEG_SIZE_MAX1        = 0x11,
    M12MO_I2C_CAT_CAP_PRM_JPEG_SIZE_MAX0        = 0x12,
    M12MO_I2C_CAT_CAP_PRM_JPEG_RATIO            = 0x17,
    M12MO_I2C_CAT_CAP_PRM_WDR_EN                = 0x2C,
    M12MO_I2C_CAT_CAP_PRM_WDR_LVL               = 0x2D,
};

/* M12MO I2C Category Prameter - 0C: Still Picture control */
enum m12mo_i2c_byte_still_control {
    M12MO_I2C_CAT_CAP_CTL_CAP_MODE              = 0x00,
    M12MO_I2C_CAT_CAP_CTL_START_CAP             = 0x05,
    M12MO_I2C_CAT_CAP_CTL_CAP_SEL_FRAME_MAIN    = 0x06,
    M12MO_I2C_CAT_CAP_CTL_CAP_TRANSFER_START    = 0x09,
    M12MO_I2C_CAT_CAP_CTL_IMAGE_SIZE            = 0x0D,
    M12MO_I2C_CAT_CAP_CTL_IMAGE_SIZE_2          = 0x0E,
    M12MO_I2C_CAT_CAP_CTL_IMAGE_SIZE_1          = 0x0F,
    M12MO_I2C_CAT_CAP_CTL_IMAGE_SIZE_0          = 0x10,
    M12MO_I2C_CAT_CAP_CTL_CAP_TRANSFER_NUM      = 0x1C,
    M12MO_I2C_CAT_CAP_CTL_CAP_DEBUG_DATA_ADD    = 0x20,
    M12MO_I2C_CAT_CAP_CTL_CAP_DEBUG_DATA_ADD2   = 0x21,
    M12MO_I2C_CAT_CAP_CTL_CAP_DEBUG_DATA_ADD1   = 0x22,
    M12MO_I2C_CAT_CAP_CTL_CAP_DEBUG_DATA_ADD0   = 0x23,
};

/* M12MO I2C Category Prameter - 0D: Production and Endurance test */
enum m12mo_i2c_byte_test {
    M12MO_I2C_CAT_TST_ADD_SHOW                  = 0x06,
    M12MO_I2C_CAT_TST_LOG_STR_LEN               = 0x07,
    M12MO_I2C_CAT_TST_LOG_STR_ADD               = 0x08,
    M12MO_I2C_CAT_TST_LOG_STR_ADD2              = 0x09,
    M12MO_I2C_CAT_TST_LOG_STR_ADD1              = 0x0A,
    M12MO_I2C_CAT_TST_LOG_STR_ADD0              = 0x0B,
    M12MO_I2C_CAT_TST_LOG_SEL                   = 0x0C,
    M12MO_I2C_CAT_TST_LOG_SEL0                  = 0x0D,
    M12MO_I2C_CAT_TST_LOG_ACT                   = 0x0E,
    M12MO_I2C_CAT_TST_LOG_MODE                  = 0x0F,
    M12MO_I2C_CAT_TST_LOG_CTL                   = 0x10,
    M12MO_I2C_CAT_TST_LOG_CTL2                  = 0x11,
    M12MO_I2C_CAT_TST_LOG_CTL1                  = 0x12,
    M12MO_I2C_CAT_TST_LOG_CTL0                  = 0x13,
    M12MO_I2C_CAT_TST_LOG_DATA_LEN              = 0x14,
    M12MO_I2C_CAT_TST_LOG_DATA_LEN0             = 0x15,
    M12MO_I2C_CAT_TST_LOG_NUM                   = 0x16,
    M12MO_I2C_CAT_TST_LOG_NUM_L                 = 0x17,
    M12MO_I2C_CAT_TST_MANUAL_LENS_DRIVE         = 0x18,
    M12MO_I2C_CAT_TST_MANUAL_LENS_DRIVE_L       = 0x19,
    M12MO_I2C_CAT_TST_OBVALUE                   = 0x1A,
    M12MO_I2C_CAT_TST_OBVALUE_L                 = 0x1B,
    M12MO_I2C_CAT_TST_OUTPUT_FIXED_PATTERN_FROM_SENSOR  = 0x1D,
    M12MO_I2C_CAT_TST_SENSOR_PGDATA0            = 0x1E,
    M12MO_I2C_CAT_TST_SENSOR_PGDATA0_L          = 0x1F,
    M12MO_I2C_CAT_TST_SENSOR_PGDATA1            = 0x20,
    M12MO_I2C_CAT_TST_SENSOR_PGDATA1_L          = 0x21,
    M12MO_I2C_CAT_TST_SENSOR_PGDATA2            = 0x22,
    M12MO_I2C_CAT_TST_SENSOR_PGDATA2_L          = 0x23,
    M12MO_I2C_CAT_TST_SENSOR_PGDATA3            = 0x24,
    M12MO_I2C_CAT_TST_SENSOR_PGDATA3_L          = 0x25,
    M12MO_I2C_CAT_TST_LBMIN                     = 0x26,
    M12MO_I2C_CAT_TST_LBMIN_L                   = 0x27,
    M12MO_I2C_CAT_TST_SENSOR_REG_ADR            = 0x34,
    M12MO_I2C_CAT_TST_SENSOR_REG_ADR_L          = 0x35,
    M12MO_I2C_CAT_TST_SENSOR_REG_DATA           = 0x36,
    M12MO_I2C_CAT_TST_LED_TEST                  = 0xF0,
    M12MO_I2C_CAT_TST_TORCH_TEST_BRIGHTNESS1    = 0xF4,
    M12MO_I2C_CAT_TST_TORCH_TEST_BRIGHTNESS2    = 0xF5,
};

/* M12MO I2C Category Prameter - 0E: Factory Adjustment */
enum m12mo_i2c_byte_adjustment {
    M12MO_I2C_CAT_ADJ_RGB_ADJUST                = 0x14,
    M12MO_I2C_CAT_ADJ_RGB_ADJUST_RESULT         = 0x15,
    M12MO_I2C_CAT_ADJ_RGB_ADJUST_RESULT_ADD     = 0x16,
    M12MO_I2C_CAT_ADJ_RGB_ADJUST_RESULT_ADD2    = 0x17,
    M12MO_I2C_CAT_ADJ_RGB_ADJUST_RESULT_ADD1    = 0x18,
    M12MO_I2C_CAT_ADJ_RGB_ADJUST_RESULT_ADD0    = 0x19,
    M12MO_I2C_CAT_ADJ_AWB_REF_IN_RG             = 0x27,
    M12MO_I2C_CAT_ADJ_AWB_REF_IN_RG_L           = 0x28,
    M12MO_I2C_CAT_ADJ_AWB_REF_IN_BG             = 0x29,
    M12MO_I2C_CAT_ADJ_AWB_REF_IN_BG_L           = 0x2A,
    M12MO_I2C_CAT_ADJ_AWB_REF_OUT_RG            = 0x2B,
    M12MO_I2C_CAT_ADJ_AWB_REF_OUT_RG_L          = 0x2C,
    M12MO_I2C_CAT_ADJ_AWB_REF_OUT_BG            = 0x2D,
    M12MO_I2C_CAT_ADJ_AWB_REF_OUT_BG_L          = 0x2E,
};

/* M12MO I2C Category Prameter - 0F: Frimware command */
enum m12mo_i2c_byte_initialize {
    M12MO_I2C_CAT_INI_GET_CHECK_SUM             = 0x0A,     /* NOT DEFINED Specifications */
    M12MO_I2C_CAT_INI_SET_START_ADDRESS         = 0x0C,     /* NOT DEFINED Specifications */
    M12MO_I2C_CAT_INI_INT_FACTOR                = 0x10,     /* NOT DEFINED Specifications */
    M12MO_I2C_CAT_INI_START                     = 0x12,     /* NOT DEFINED Specifications */
};

/* M12MO I2C Category Prameter - 10: IQ Param Setting Common */
enum m12mo_i2c_byte_iq_common {
    M12MO_I2C_CAT_IQ_COM_IQ_UPDATE_TRG_COMMON   = 0x00,
    M12MO_I2C_CAT_IQ_COM_FSHD_EN                = 0x01,
    M12MO_I2C_CAT_IQ_COM_SELECT_BASE_FSHD_TBL   = 0x02,
    M12MO_I2C_CAT_IQ_COM_SHD_MCC_SW_THOLD_BV    = 0x03,
    M12MO_I2C_CAT_IQ_COM_SHD_MCC_SW_THOLD_X     = 0x04,
    M12MO_I2C_CAT_IQ_COM_SHD_MCC_SW_THOLD_Y     = 0x05,
    M12MO_I2C_CAT_IQ_COM_SHD_MCC_SW_THOLD_CTMP  = 0x06,
    M12MO_I2C_CAT_IQ_COM_PZSFT_CTRL             = 0x07,
    M12MO_I2C_CAT_IQ_COM_PZSFTA2_CTRL           = 0x08,
    M12MO_I2C_CAT_IQ_COM_PZSFTC1_CTRL           = 0x09,
    M12MO_I2C_CAT_IQ_COM_PZSFTC2_CTRL           = 0x0A,
    M12MO_I2C_CAT_IQ_COM_PGAIN_EN               = 0x0B,
    M12MO_I2C_CAT_IQ_COM_CFIXEN                 = 0x0C,
    M12MO_I2C_CAT_IQ_COM_CFIXB                  = 0x0D,
    M12MO_I2C_CAT_IQ_COM_CFIXR                  = 0x0E,
    M12MO_I2C_CAT_IQ_COM_MCC_SW_TBL             = 0x0F,
    M12MO_I2C_CAT_IQ_COM_EDGE_ADJ_THR           = 0x10,
    M12MO_I2C_CAT_IQ_COM_EDGESUP_ADJ            = 0x11,
    M12MO_I2C_CAT_IQ_COM_SCALE_FACTOR_ZOOM      = 0x12,
    M12MO_I2C_CAT_IQ_COM_SPR2G_STG              = 0x13,
    M12MO_I2C_CAT_IQ_COM_MSPR_G_STG             = 0x14,
    M12MO_I2C_CAT_IQ_COM_CAC_EN                 = 0x15,
    M12MO_I2C_CAT_IQ_COM_CAC_ACT                = 0x16,
    M12MO_I2C_CAT_IQ_COM_CAC_BV                 = 0x17,
    M12MO_I2C_CAT_IQ_COM_CAC_BV_L               = 0x18,
    M12MO_I2C_CAT_IQ_COM_CAC_RESULT             = 0x19,
};

/* M12MO I2C Category Prameter - 11: IQ Param Setting Monitor */
enum m12mo_i2c_byte_iq_monitor {
    M12MO_I2C_CAT_IQ_MON_IQ_UPDATE_TRG_MON      = 0x00,
    M12MO_I2C_CAT_IQ_MON_DPC_MON                = 0x01,
    M12MO_I2C_CAT_IQ_MON_LNR_MON                = 0x02,
    M12MO_I2C_CAT_IQ_MON_ELF_MON                = 0x03,
    M12MO_I2C_CAT_IQ_MON_CAG_ZOOM_MON           = 0x04,
    M12MO_I2C_CAT_IQ_MON_CAGFOCUSMON            = 0x05,
    M12MO_I2C_CAT_IQ_MON_B2YOFFSET_MON          = 0x06,
    M12MO_I2C_CAT_IQ_MON_SHARP_MON              = 0x07,
    M12MO_I2C_CAT_IQ_MON_MOIRE_ZOOM_MON         = 0x08,
    M12MO_I2C_CAT_IQ_MON_MOIRE_FOCUS_MON        = 0x09,
    M12MO_I2C_CAT_IQ_MON_HF_MON                 = 0x0A,
    M12MO_I2C_CAT_IQ_MON_CCYC_MON               = 0x0B,
    M12MO_I2C_CAT_IQ_MON_CCYB_MON               = 0x0C,
    M12MO_I2C_CAT_IQ_MON_MCC_MON                = 0x0D,
    M12MO_I2C_CAT_IQ_MON_TONE_MON               = 0x0E,
    M12MO_I2C_CAT_IQ_MON_GAM_SW_MON             = 0x0F,
    M12MO_I2C_CAT_IQ_MON_GAM_RGB_MON            = 0x10,
    M12MO_I2C_CAT_IQ_MON_GAM_R_MON              = 0x11,
    M12MO_I2C_CAT_IQ_MON_GAM_G_MON              = 0x12,
    M12MO_I2C_CAT_IQ_MON_GAM_B_MON              = 0x13,
    M12MO_I2C_CAT_IQ_MON_GAM_YB_MON             = 0x14,
    M12MO_I2C_CAT_IQ_MON_YC_MON                 = 0x15,
    M12MO_I2C_CAT_IQ_MON_YBLEND_MON             = 0x16,
    M12MO_I2C_CAT_IQ_MON_YNR_MON                = 0x17,
    M12MO_I2C_CAT_IQ_MON_EEHPF_SELECT_MON       = 0x18,
    M12MO_I2C_CAT_IQ_MON_EEHPF_OFF1_MON         = 0x19,
    M12MO_I2C_CAT_IQ_MON_EECOR_SELECT_MON       = 0x1B,
    M12MO_I2C_CAT_IQ_MON_EECOR_OFF1_MON         = 0x1C,
    M12MO_I2C_CAT_IQ_MON_EESCL_TBL_MON          = 0x1E,
    M12MO_I2C_CAT_IQ_MON_EECLIP_TBL_MON         = 0x1F,
    M12MO_I2C_CAT_IQ_MON_EESCL_SCLVAL_MON       = 0x20,
    M12MO_I2C_CAT_IQ_MON_EESCL_CLIPVAL_MON      = 0x21,
    M12MO_I2C_CAT_IQ_MON_EESCL_CRVVAL_MON       = 0x22,
    M12MO_I2C_CAT_IQ_MON_CLPF_MON               = 0x23,
    M12MO_I2C_CAT_IQ_MON_CSY_MON                = 0x24,
    M12MO_I2C_CAT_IQ_MON_SPRLT_CTL_MON          = 0x25,
    M12MO_I2C_CAT_IQ_MON_SPRLT_OFF_MON          = 0x26,
    M12MO_I2C_CAT_IQ_MON_SYNCNOMON              = 0x2D,
    M12MO_I2C_CAT_IQ_MON_TCBLND_MON             = 0x2E,
};

/* M12MO I2C Category Prameter - 12: IQ Param Setting Capture */
enum m12mo_i2c_byte_iq_capture {
    M12MO_I2C_CAT_IQ_CAP_IQ_UPDATE_TRG_CAP      = 0x00,
    M12MO_I2C_CAT_IQ_CAP_DPC_CAP                = 0x01,
    M12MO_I2C_CAT_IQ_CAP_LNR_CAP                = 0x02,
    M12MO_I2C_CAT_IQ_CAP_ELF_CAP                = 0x03,
    M12MO_I2C_CAT_IQ_CAP_CAG_ZOOM_CAP           = 0x04,
    M12MO_I2C_CAT_IQ_CAP_CAG_FOCUS_CAP          = 0x05,
    M12MO_I2C_CAT_IQ_CAP_B2YOFFSET_CAP          = 0x06,
    M12MO_I2C_CAT_IQ_CAP_SHARP_CAP              = 0x07,
    M12MO_I2C_CAT_IQ_CAP_MOIRE_ZOOM_CAP         = 0x08,
    M12MO_I2C_CAT_IQ_CAP_MOIRE_FOCUS_CAP        = 0x09,
    M12MO_I2C_CAT_IQ_CAP_HF_CAP                 = 0x0A,
    M12MO_I2C_CAT_IQ_CAP_CCYC_CAP               = 0x0B,
    M12MO_I2C_CAT_IQ_CAP_CCYB_CAP               = 0x0C,
    M12MO_I2C_CAT_IQ_CAP_MCC_CAP                = 0x0D,
    M12MO_I2C_CAT_IQ_CAP_TONE_CAP               = 0x0E,
    M12MO_I2C_CAT_IQ_CAP_GAM_SW_CAP             = 0x0F,
    M12MO_I2C_CAT_IQ_CAP_GAM_RGB_CAP            = 0x10,
    M12MO_I2C_CAT_IQ_CAP_GAM_R_CAP              = 0x11,
    M12MO_I2C_CAT_IQ_CAP_GAM_G_CAP              = 0x12,
    M12MO_I2C_CAT_IQ_CAP_GAM_B_CAP              = 0x13,
    M12MO_I2C_CAT_IQ_CAP_GAM_YB_CAP             = 0x14,
    M12MO_I2C_CAT_IQ_CAP_YC_CAP                 = 0x15,
    M12MO_I2C_CAT_IQ_CAP_YBLEND_CAP             = 0x16,
    M12MO_I2C_CAT_IQ_CAP_YNR_CAP                = 0x17,
    M12MO_I2C_CAT_IQ_CAP_EEHPF_SELECT_CAP       = 0x18,
    M12MO_I2C_CAT_IQ_CAP_EEHPF_OFF1_CAP         = 0x19,
    M12MO_I2C_CAT_IQ_CAP_EECOR_SELECT_CAP       = 0x1B,
    M12MO_I2C_CAT_IQ_CAP_EECOR_OFF1_CAP         = 0x1C,
    M12MO_I2C_CAT_IQ_CAP_EESCL_TBL_CAP          = 0x1E,
    M12MO_I2C_CAT_IQ_CAP_EECLIP_TBL_CAP         = 0x1F,
    M12MO_I2C_CAT_IQ_CAP_EESCL_SCLVAL_CAP       = 0x20,
    M12MO_I2C_CAT_IQ_CAP_EESCL_CLIPVAL_CAP      = 0x21,
    M12MO_I2C_CAT_IQ_CAP_EESCL_CRVVAL_CAP       = 0x22,
    M12MO_I2C_CAT_IQ_CAP_CLPF_CAP               = 0x23,
    M12MO_I2C_CAT_IQ_CAP_CSY_CAP                = 0x24,
    M12MO_I2C_CAT_IQ_CAP_SPRLT_CTL_CAP          = 0x25,
    M12MO_I2C_CAT_IQ_CAP_SPRLT_OFF_CAP          = 0x26,
    M12MO_I2C_CAT_IQ_CAP_SPR_CTL_CAP            = 0x27,
    M12MO_I2C_CAT_IQ_CAP_SPR_OFF_CAP            = 0x28,
    M12MO_I2C_CAT_IQ_CAP_LDC_CTL_ZOOM_CAP       = 0x29,
    M12MO_I2C_CAT_IQ_CAP_LDC_CTL_FOCUS_CAP      = 0x2A,
    M12MO_I2C_CAT_IQ_CAP_LDC_ADR_ZOOM_CAP       = 0x2B,
    M12MO_I2C_CAT_IQ_CAP_LDC_ADR_FOCUS_CAP      = 0x2C,
    M12MO_I2C_CAT_IQ_CAP_SYNC_NO_CAP            = 0x2D,
    M12MO_I2C_CAT_IQ_CAP_TCBLND_CAP             = 0x2E,
};

/* M12MO I2C Category Prameter - 12: IQ Param Setting Movie */
enum m12mo_i2c_byte_iq_movie {
    M12MO_I2C_CAT_IQ_MOV_IQ_UPDATE_TRG_MOVIE    = 0x00,
    M12MO_I2C_CAT_IQ_MOV_DPC_MOVIE              = 0x01,
    M12MO_I2C_CAT_IQ_MOV_LNR_MOVIE              = 0x02,
    M12MO_I2C_CAT_IQ_MOV_ELF_MOVIE              = 0x03,
    M12MO_I2C_CAT_IQ_MOV_CAG_ZOOM_MOVIE         = 0x04,
    M12MO_I2C_CAT_IQ_MOV_CAGFOCUSMOVIE          = 0x05,
    M12MO_I2C_CAT_IQ_MOV_B2YOFFSET_MOVIE        = 0x06,
    M12MO_I2C_CAT_IQ_MOV_SHARP_MOVIE            = 0x07,
    M12MO_I2C_CAT_IQ_MOV_MOIRE_ZOOM_MOVIE       = 0x08,
    M12MO_I2C_CAT_IQ_MOV_MOIRE_FOCUS_MOVIE      = 0x09,
    M12MO_I2C_CAT_IQ_MOV_HF_MOVIE               = 0x0A,
    M12MO_I2C_CAT_IQ_MOV_CCYC_MOVIE             = 0x0B,
    M12MO_I2C_CAT_IQ_MOV_CCYB_MOVIE             = 0x0C,
    M12MO_I2C_CAT_IQ_MOV_MCC_MOVIE              = 0x0D,
    M12MO_I2C_CAT_IQ_MOV_TONE_MOVIE             = 0x0E,
    M12MO_I2C_CAT_IQ_MOV_GAM_SW_MOVIE           = 0x0F,
    M12MO_I2C_CAT_IQ_MOV_GAM_RGB_MOVIE          = 0x10,
    M12MO_I2C_CAT_IQ_MOV_GAM_R_MOVIE            = 0x11,
    M12MO_I2C_CAT_IQ_MOV_GAM_G_MOVIE            = 0x12,
    M12MO_I2C_CAT_IQ_MOV_GAM_B_MOVIE            = 0x13,
    M12MO_I2C_CAT_IQ_MOV_GAM_YB_MOVIE           = 0x14,
    M12MO_I2C_CAT_IQ_MOV_YC_MOVIE               = 0x15,
    M12MO_I2C_CAT_IQ_MOV_YBLEND_MOVIE           = 0x16,
    M12MO_I2C_CAT_IQ_MOV_YNR_MOVIE              = 0x17,
    M12MO_I2C_CAT_IQ_MOV_EEHPF_SELECT_MOVIE     = 0x18,
    M12MO_I2C_CAT_IQ_MOV_EEHPF_OFF1_MOVIE       = 0x19,
    M12MO_I2C_CAT_IQ_MOV_EECOR_SELECT_MOVIE     = 0x1B,
    M12MO_I2C_CAT_IQ_MOV_EECOR_OFF2_MOVIE       = 0x1D,
    M12MO_I2C_CAT_IQ_MOV_EESCL_TBL_MOVIE        = 0x1E,
    M12MO_I2C_CAT_IQ_MOV_EECLIP_TBL_MOVIE       = 0x1F,
    M12MO_I2C_CAT_IQ_MOV_EESCL_SCLVAL_MOVIE     = 0x20,
    M12MO_I2C_CAT_IQ_MOV_EESCL_CLIPVAL_MOVIE    = 0x21,
    M12MO_I2C_CAT_IQ_MOV_EESCL_CRVVAL_MOVIE     = 0x22,
    M12MO_I2C_CAT_IQ_MOV_CLPF_MOVIE             = 0x23,
    M12MO_I2C_CAT_IQ_MOV_CSY_MOVIE              = 0x24,
    M12MO_I2C_CAT_IQ_MOV_SPRLT_CTL_MOVIE        = 0x25,
    M12MO_I2C_CAT_IQ_MOV_SPRLT_OFF_MOVIE        = 0x26,
    M12MO_I2C_CAT_IQ_MOV_SYNCNOMOVIE            = 0x2D,
    M12MO_I2C_CAT_IQ_MOV_TCBLND_MOVIE           = 0x2E,
};

/* M12MO I2C Category Prameter - 12: IQ Param Setting */
enum m12mo_i2c_byte_iq_setting {
    M12MO_I2C_CAT_IQ_PRM_IQ_PRM_SET_FLG             = 0x00,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_SV                  = 0x01,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_SV_H0               = 0x02,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_SV_L1               = 0x03,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_SV_L0               = 0x04,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_BV                  = 0x05,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_BV_H0               = 0x06,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_BV_L1               = 0x07,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_BV_L0               = 0x08,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_AV                  = 0x09,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_AV_H0               = 0x0A,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_AV_L1               = 0x0B,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_AV_L0               = 0x0C,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_EV                  = 0x0D,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_EV_H0               = 0x0E,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_EV_L1               = 0x0F,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_EV_L0               = 0x10,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_TV                  = 0x11,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_TV_H0               = 0x12,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_TV_L1               = 0x13,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_TV_L0               = 0x14,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_CURRENT_TEMP        = 0x15,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_CURRENT_TEMP0       = 0x16,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_CURRENT_COLOR_X     = 0x17,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_CURRENT_COLOR_X_H0  = 0x18,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_CURRENT_COLOR_X_L1  = 0x19,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_CURRENT_COLOR_X_L0  = 0x1A,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_CURRENT_COLOR_Y     = 0x1B,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_CURRENT_COLOR_Y_H0  = 0x1C,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_CURRENT_COLOR_Y_L1  = 0x1D,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_CURRENT_COLOR_Y_L0  = 0x1E,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_SENSOR_GAIN         = 0x1F,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_SENSOR_GAIN_H0      = 0x20,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_SENSOR_GAIN_L1      = 0x21,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_SENSOR_GAIN_L0      = 0x22,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_ZOOM                = 0x23,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_ZOOM_H0             = 0x24,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_ZOOM_L1             = 0x25,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_ZOOM_L0             = 0x26,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_ZOOM_RATE           = 0x27,
    M12MO_I2C_CAT_IQ_PRM_SET_IQ_ZOOM_RATE0          = 0x28,
};

/********* M12MO I2C Category Prameter - Value ********/

/* 0F: Firmware command */
#define M12MO_I2C_VAL_START_UPDATER_ADDR    (0x01000100)    /* NOT DEFINED Specifications */

#define M12MO_I2C_VAL_START_CAMERA_FW       (0x01)          /* NOT DEFINED Specifications */
#define M12MO_I2C_VAL_START_FROM_ADDR       (0x02)          /* NOT DEFINED Specifications */

#define M12MO_I2C_VAL_INT_EN_FWDL           (0x04)          /* NOT DEFINED Specifications */

/* 00: System Parameter - 0x0B: SYS_MODE */
#define M12MO_I2C_VAL_MODE_SYSTEM_INIT      (0x00)
#define M12MO_I2C_VAL_MODE_PARAM_SETTING    (0x01)
#define M12MO_I2C_VAL_MODE_MONITOR          (0x02)
#define M12MO_I2C_VAL_MODE_STILL_CAPTURE    (0x03)

/* 00: System Parameter - 0x10: INT_ENABLE */
#define M12MO_I2C_VAL_INT_MASK_ALL          (0xFF)
#define M12MO_I2C_VAL_INT_INITIALIZED       (0x00)

#define M12MO_I2C_VAL_INT_STATUS_SOUND          (0x80)
#define M12MO_I2C_VAL_INT_STATLENS_INIT         (0x40)
#define M12MO_I2C_VAL_INT_STATUS_FD             (0x20)
#define M12MO_I2C_VAL_INT_STATUS_CAF            (0x10)
#define M12MO_I2C_VAL_INT_STATUS_CAPTURE        (0x08)
#define M12MO_I2C_VAL_INT_STATUS_ZOOM           (0x04)          /* NOT DEFINED Specifications */
#define M12MO_I2C_VAL_INT_STATUS_AF             (0x02)
#define M12MO_I2C_VAL_INT_STATUS_MODE           (0x01)

/* 00: System Parameter - 0x46: HW_I2C_CHECK */
#define M12MO_I2C_VAL_HW_CHK_REAR_CAM           (0x01)

#define M12MO_I2C_VAL_HW_CHK_MASK               (M12MO_I2C_VAL_HW_CHK_REAR_CAM)

/* 01: Monitor&Still parameter A - 0x01: MON_SIZE */
#define M12MO_I2C_VAL_MON_SIZE_VGA              (0x17)
#define M12MO_I2C_VAL_MON_SIZE_FWVGA            (0x1D)
#define M12MO_I2C_VAL_MON_SIZE_HD               (0x21)
#define M12MO_I2C_VAL_MON_SIZE_FHD              (0x28)
#define M12MO_I2C_VAL_MON_SIZE_3_25M            (0x56)
#define M12MO_I2C_VAL_MON_SIZE_2_5M             (0x57)
#define M12MO_I2C_VAL_MON_SIZE_SVGA             (0x1F)
#define M12MO_I2C_VAL_MON_SIZE_1_33M            (0x3C)

#define M12MO_I2C_VAL_MON_SIZE_INVALID          (0xFF)      /* NOT DEFINED Specifications */

/* 02: Monitor&Still parameter B - 0x00: AHS_MON */
#define M12MO_I2C_VAL_AHS_MON_OFF               (0x00)
#define M12MO_I2C_VAL_AHS_MON_ON                (0x01)

/* 02: Monitor&Still parameter B - 0x14: EFFECT */
#define M12MO_I2C_VAL_EFFECT_OFF                (0x00)
#define M12MO_I2C_VAL_EFFECT_NEGA               (0x01)
#define M12MO_I2C_VAL_EFFECT_GRAY               (0x02)
#define M12MO_I2C_VAL_EFFECT_SEPIA              (0x03)

/* 02: Monitor&Still parameter B - 0x1A: ATSCENE_LOCK */
#define M12MO_I2C_VAL_ATSCENE_LOCK_OFF          (0x00)
#define M12MO_I2C_VAL_ATSCENE_LOCK_ON           (0x01)

/* 02: Monitor&Still parameter B - 0x1B: ATSCENE_MANUAL */
#define M12MO_I2C_VAL_ATSCENE_MAN_AUTO          (0x00)
#define M12MO_I2C_VAL_ATSCENE_MAN_PARTY         (0x01)
#define M12MO_I2C_VAL_ATSCENE_MAN_PORTRAIT      (0x02)
#define M12MO_I2C_VAL_ATSCENE_MAN_LANDSCAPE     (0x03)
#define M12MO_I2C_VAL_ATSCENE_MAN_NIGHT         (0x04)
#define M12MO_I2C_VAL_ATSCENE_MAN_BACKLIGHT     (0x05)
#define M12MO_I2C_VAL_ATSCENE_MAN_MEMO          (0x06)
#define M12MO_I2C_VAL_ATSCENE_MAN_BEACH         (0x07)
#define M12MO_I2C_VAL_ATSCENE_MAN_SPORTS        (0x08)

/* 02: Monitor&Still parameter B - 0x1C: ATSCENE_DETECT_START */
#define M12MO_I2C_VAL_ATSCENE_DETECT_STOP       (0x00)
#define M12MO_I2C_VAL_ATSCENE_DETECT_START      (0x01)

/* 02: Monitor&Still parameter B - 0x1D: ATSCENE_PARAM_UPDATE */
#define M12MO_I2C_VAL_ATSCENE_PARAM_UPDATE      (0x01)

/* 02: Monitor&Still parameter B - 0x36: CURRENT_SCENE */
#define M12MO_I2C_VAL_CURSCENE_AUTO             (0x00)
#define M12MO_I2C_VAL_CURSCENE_PORTRAIT         (0x01)
#define M12MO_I2C_VAL_CURSCENE_PARTY            (0x02)
#define M12MO_I2C_VAL_CURSCENE_LANDSCAPE        (0x03)
#define M12MO_I2C_VAL_CURSCENE_NIGHT            (0x04)
#define M12MO_I2C_VAL_CURSCENE_BACKLIGHT        (0x05)
#define M12MO_I2C_VAL_CURSCENE_BEACH            (0x06)
#define M12MO_I2C_VAL_CURSCENE_SNOW             (0x07)
#define M12MO_I2C_VAL_CURSCENE_MAPTEXT          (0x08)
#define M12MO_I2C_VAL_CURSCENE_QRCODE           (0x09)
#define M12MO_I2C_VAL_CURSCENE_SUNSET           (0x0A)

/* 02: Monitor&Still parameter B - 0x6B: FPS_CTRL */
#define M12MO_I2C_VAL_FPS_CTRL_AUTO             (0x00)
#define M12MO_I2C_VAL_FPS_CTRL_120FPS           (0x01)
#define M12MO_I2C_VAL_FPS_CTRL_60FPS            (0x02)
#define M12MO_I2C_VAL_FPS_CTRL_30FPS            (0x03)
#define M12MO_I2C_VAL_FPS_CTRL_24FPS            (0x04)
#define M12MO_I2C_VAL_FPS_CTRL_20FPS            (0x05)
#define M12MO_I2C_VAL_FPS_CTRL_15FPS            (0x06)
#define M12MO_I2C_VAL_FPS_CTRL_12FPS            (0x07)
#define M12MO_I2C_VAL_FPS_CTRL_10FPS            (0x08)
#define M12MO_I2C_VAL_FPS_CTRL_7_5FPS           (0x09)
#define M12MO_I2C_VAL_FPS_CTRL_28FPS            (0x0A)

/* 03: Exposure control - 0x00: AE_LOCK */
#define M12MO_I2C_VAL_AE_LOCK_OFF               (0x00)
#define M12MO_I2C_VAL_AE_LOCK_ON                (0x01)

/* 03: Exposure Control - 0x01: AE_MODE */
#define M12MO_I2C_VAL_AE_MODE_OFF               (0x00)
#define M12MO_I2C_VAL_AE_MODE_ALL               (0x01)
#define M12MO_I2C_VAL_AE_MODE_CENTER            (0x02)
#define M12MO_I2C_VAL_AE_MODE_ROI               (0x10)

/* 03: Exposure Control - 0x06: FLICKER */
#define M12MO_I2C_VAL_AE_FLICKER_AUTO           (0x00)
#define M12MO_I2C_VAL_AE_FLICKER_50HZ           (0x01)
#define M12MO_I2C_VAL_AE_FLICKER_60HZ           (0x02)
#define M12MO_I2C_VAL_AE_FLICKER_OFF            (0x04)

/* 03: Exposure Control - 0x09: EV_BIAS */
#define M12MO_I2C_VAL_AE_EV_BIAS_OFFSET         (6)
#define M12MO_I2C_VAL_AE_EV_BIAS_MIN            (0x00)
#define M12MO_I2C_VAL_AE_EV_BIAS_MAX            (0x0C)

#define M12MO_I2C_VAL_AE_EV_BIAS_INVALID        (INT_MIN)   /* NOT DEFINED Specifications */

/* 03: Exposure control - 0x0D: AE_PARAM_UPDATE */
#define M12MO_I2C_VAL_AE_PARAM_UPDATE           (0x01)

/* 03: Exposure control - 0x3C: STROBE_EN */
#define M12MO_I2C_VAL_AE_STROBE_EN_OFF          (0x00)
#define M12MO_I2C_VAL_AE_STROBE_EN_AUTO         (0x01)
#define M12MO_I2C_VAL_AE_STROBE_EN_ON           (0x02)
#define M12MO_I2C_VAL_AE_STROBE_EN_TORCH        (0x03)

/* 06: White balance control - 0x00: AWB_LOCK */
#define M12MO_I2C_VAL_AWB_LOCK_OFF              (0x00)
#define M12MO_I2C_VAL_AWB_LOCK_ON               (0x01)

/* 06: White balance control - 0x02: AWB_MODE */
#define M12MO_I2C_VAL_AWB_MODE_USERSET          (0x00)
#define M12MO_I2C_VAL_AWB_MODE_AUTO             (0x01)
#define M12MO_I2C_VAL_AWB_MODE_MANUAL           (0x02)
#define M12MO_I2C_VAL_AWB_MODE_TRACKLMT         (0x03)
#define M12MO_I2C_VAL_AWB_MODE_ROI              (0x10)

/* 06: White balance control - 0x03: AWB_MANUAL */
#define M12MO_I2C_VAL_AWB_MN_INCANDESCENT       (0x01)
#define M12MO_I2C_VAL_AWB_MN_FLUORESCENT_HIGH   (0x02)
#define M12MO_I2C_VAL_AWB_MN_FLUORESCENT_LOW    (0x03)
#define M12MO_I2C_VAL_AWB_MN_DAYLIGHT           (0x04)
#define M12MO_I2C_VAL_AWB_MN_CLOUDY             (0x05)
#define M12MO_I2C_VAL_AWB_MN_SHADE              (0x06)
#define M12MO_I2C_VAL_AWB_MN_HORIZON            (0x07)
#define M12MO_I2C_VAL_AWB_MN_USER_SETTING       (0x08)

/* 06: White balance control - 0x0C: AWB_PARAM_UPDATE */
#define M12MO_I2C_VAL_AWB_PARAM_UPDATE          (0x01)

/* 07: Exif information - 0x2A: INFO_FLASH */
#define M12MO_I2C_VAL_FLASH_FIRED               (0x0009)
#define M12MO_I2C_VAL_FLASH_OFF                 (0x0010)
#define M12MO_I2C_VAL_FLASH_AUTO_NOT_FIRE       (0x0018)
#define M12MO_I2C_VAL_FLASH_AUTO_FIRED          (0x0019)
#define M12MO_I2C_VAL_FLASH_NO_FUNCTION         (0x0020)
#define M12MO_I2C_VAL_FLASH_FIRED_REDEYE        (0x0049)
#define M12MO_I2C_VAL_FLASH_AUTO_FIRED_REDEYE   (0x0059)

/* 0A: AF / Optical Zoom Control - 0x00: AF_MODE */
#define M12MO_I2C_VAL_AF_MODE_SINGLE_CONT       (0x01)
#define M12MO_I2C_VAL_AF_MODE_CONTINUOUS_CONT   (0x06)
#define M12MO_I2C_VAL_AF_MODE_SINGLE_HYBRID     (0x0A)
#define M12MO_I2C_VAL_AF_MODE_CONTINUOUS_HYBRID (0x0B)

/* 0B: Still Picture Parameter - 0x01: MAIN_IMAGE_SIZE */
#define M12MO_I2C_VAL_MAIN_SIZE_13M             (0x2C)
#define M12MO_I2C_VAL_MAIN_SIZE_8M              (0x26)
#define M12MO_I2C_VAL_MAIN_SIZE_FHD             (0x19)
#define M12MO_I2C_VAL_MAIN_SIZE_FWVGA           (0x0C)
#define M12MO_I2C_VAL_MAIN_SIZE_5M              (0x20)
#define M12MO_I2C_VAL_MAIN_SIZE_3M              (0x39)
#define M12MO_I2C_VAL_MAIN_SIZE_1_2M            (0x14)
#define M12MO_I2C_VAL_MAIN_SIZE_3_25M           (0x3A)
#define M12MO_I2C_VAL_MAIN_SIZE_2_5M            (0x3B)

/* 0B: Still Picture Parameter - 0x0A: YUVOUT_THUMB */
#define M12MO_I2C_VAL_YUVOUT_THUMB_420      (0x05)
#define M12MO_I2C_VAL_YUVOUT_THUMB_NV21     (0x06)
#define M12MO_I2C_VAL_YUVOUT_THUMB_NV12     (0x07)

/* 0B: Still Picture Parameter - 0x0B: THUMB_IMAGE_SIZE */
#define M12MO_I2C_VAL_THUMB_SIZE_QQVGA      (0x02)
#define M12MO_I2C_VAL_THUMB_SIZE_QVGA       (0x04)
#define M12MO_I2C_VAL_THUMB_SIZE_360x240    (0x05)
#define M12MO_I2C_VAL_THUMB_SIZE_WQVGA      (0x06)
#define M12MO_I2C_VAL_THUMB_SIZE_FWQVGA     (0x07)
#define M12MO_I2C_VAL_THUMB_SIZE_VGA        (0x0B)
#define M12MO_I2C_VAL_THUMB_SIZE_800x480    (0x0C)
#define M12MO_I2C_VAL_THUMB_SIZE_FWVGA      (0x0E)
#define M12MO_I2C_VAL_THUMB_SIZE_480P       (0x11)
#define M12MO_I2C_VAL_THUMB_SIZE_HHD        (0x12)
#define M12MO_I2C_VAL_THUMB_SIZE_QuadVGA    (0x13)

/* 0C: Still Picture control - 0x00: CAP_MODE */
#define M12MO_I2C_VAL_CAP_MODE_SINGLE       (0x00)
#define M12MO_I2C_VAL_CAP_MODE_BURST        (0x01)
#define M12MO_I2C_VAL_CAP_MODE_ZSL          (0x0E)
#define M12MO_I2C_VAL_CAP_MODE_DUAL         (0x11)

#define M12MO_I2C_VAL_CAP_MODE_INVALID      (0xFF)      /* NOT DEFINED Specifications */

/* 0C: Still Picture control - 0x05: START_CAP */
#define M12MO_I2C_VAL_START_CAP_NONE        (0x00)
#define M12MO_I2C_VAL_START_STOP_SHOT       (0x05)
#define M12MO_I2C_VAL_START_SINGLE_SHOT     (0x0C)
#define M12MO_I2C_VAL_START_STOP_STREAM     (0x0A)
#define M12MO_I2C_VAL_START_RESTART_STREAM  (0x0B)

/* 0C: Still Picture control - 0x09: CAP_TRANSFER_START */
#define M12MO_I2C_VAL_TX_MAIN_IMAGE         (0x01)
#define M12MO_I2C_VAL_TX_BAYER_IMAGE        (0x03)
#define M12MO_I2C_VAL_TX_IQBIN              (0x05)
#define M12MO_I2C_VAL_TX_JPG_RAW_IQBIN      (0x10)

/* 0D: Production and Endurance test - 0xF0: LED_TEST */
#define M12MO_I2C_VAL_LEDTST_INIT            (0xFF)
#define M12MO_I2C_VAL_LEDTST_TORCH_OFF       (0x10)
#define M12MO_I2C_VAL_LEDTST_LED1_TORCH_ON   (0x01)
#define M12MO_I2C_VAL_LEDTST_LED2_TORCH_ON   (0x02)
#define M12MO_I2C_VAL_LEDTST_LED1_2_TORCH_ON (0x03)
#define M12MO_I2C_VAL_LEDTST_LED1_FLASH      (0x04)
#define M12MO_I2C_VAL_LEDTST_LED2_FLASH      (0x08)
#define M12MO_I2C_VAL_LEDTST_LED1_2_FLASH    (0x0C)

#endif
