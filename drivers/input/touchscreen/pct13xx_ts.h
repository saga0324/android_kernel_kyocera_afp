/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 */
/*
 * include/linux/input/pct13xx_ts.h
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
 */

#ifndef __LINUX_INPUT_PCT13XX_TS_H__
#define __LINUX_INPUT_PCT13XX_TS_H__

#include <linux/pct13xx_display_touch.h>

/* bit operation - local */
#define BIT_CLEAR(REG,MASK)			((REG) &= ~(MASK))
#define BIT_SET(REG,MASK)			((REG) |=  (MASK))
#define IS_BIT_CLEAR(REG,MASK)		(((REG) & (MASK)) == 0x00)
#define IS_BIT_SET(REG,MASK)		(((REG) & (MASK)) == (MASK))

/* Device configuration */
#define PCT13XX_FORCE_MAX			65535
#define PCT13XX_AREA_MAX			1023
#define PCT13XX_SEQ_RW				0x80
#define PCT13XX_IB_PARAM_OFFSET		0x80
#define PCT13XX_PARAM_VER_OFFSET	16
#define PCT13XX_PARAM_LEN_OFFSET	17
#define PCT13XX_PARAM_CRC_OFFSET	18
#define PCT13XX_PARAM_OFFSET		32
#define PCT13XX_MAX_DRIVE_NUM		16
#define PCT13XX_MAX_SENSE_NUM		23

/* delay time */
#define PCT13XX_DELAY_US_BETWEEN_I2C_REG	400

/* pjp171 register talbe*/
#define PCT13XX_REG_SWITCH_BANK				0x7F

/* bank 0 */
#define PCT13XX_REG_DRIVE_NUM				0x04
#define PCT13XX_REG_SENSE_NUM				0x05
#define PCT13XX_REG_HAND_SHAKING			0x7C
	#define PCT13XX_RW_ENABLE				0x5A
	#define PCT13XX_RW_DISABLE				0x00
	#define PCT13XX_DEV_READY				0xA5

/* bank 1 */
#define PCT13XX_REG_PD						0x14
	#define PCT13XX_PD						0x02
#define PCT13XX_REG_FLASH_START				0x55
#define PCT13XX_REG_FLASH_SIZE				0x56
	#define PCT13XX_FLASH_SIZE				0x01
#define PCT13XX_REG_USER_MODE				0x5E
	#define PCT13XX_CPU_READ				0x00
	#define PCT13XX_USER_CMD				0x01
#define PCT13XX_REG_BIST_CONTROL			0x50
	#define PCT13XX_BIST_ONE				0x03
	#define PCT13XX_BIST_TWO				0x00
#define PCT13XX_REG_BIST_FIN				0x54
#define PCT13XX_REG_BIST_PASS				0x53
#define PCT13XX_REG_START					0x5F
	#define PCT13XX_START					0x01
#define PCT13XX_REG_FLASH_CMD				0x62
	#define PCT13XX_FLASH_STANDBY			0x00
	#define PCT13XX_FLASH_READY   			0x01
#define PCT13XX_REG_FLASH_XADR				0x64
#define PCT13XX_REG_FLASH_YADR				0x65
#define PCT13XX_REG_IFREN					0x6E
	#define PCT13XX_IB_ENABLE				0x01
	#define PCT13xx_IB_DISABLE				0x00
#define PCT13XX_REG_FLASH_DATA				0x71

/* bank 2 */
#define PCT13XX_REG_BYPASS					0x04
#define PCT13XX_REG_SRAM_SEL				0x09
	#define PCT13XX_MED3_DATA				0x00
	#define PCT13XX_RAW_DATA_FRM_0			0x05
	#define PCT13XX_RAW_DATA_FRM_1			0x06
	#define PCT13XX_SRAM_SEL				0x07
#define PCT13XX_REG_SRAM_NCS				0x0C
	#define PCT13XX_SRAM_NCS_ONE			0x00
	#define PCT13XX_SRAM_NCS_TWO			0x01
#define PCT13XX_REG_SRAM_DATA				0x0D
#define PCT13XX_REG_FRM_STATUS				0x18
#define PCT13XX_REG_DRIVE_START				0x1A
#define PCT13XX_REG_SENSE_START				0x1B
#define PCT13XX_REG_DRIVE_STOP				0x1C
#define PCT13XX_REG_SENSE_STOP				0x1D

/* bank 5 */
#define PCT13XX_REG_ID_BASE					0x01
#define PCT13XX_REG_X_BASE					0x02
#define PCT13XX_REG_Y_BASE					0x04
#define PCT13XX_REG_AREA_BASE				0x06
#define PCT13XX_REG_FORCE_BASE				0x08
#define PCT13XX_REG_REL_X					0x5A
#define PCT13XX_REG_REL_Y					0x5C
#define PCT13XX_REG_BANK_IDENTIFIER			0x72
	#define PCT13XX_BANK_IDENTIFIER			0x55
#define PCT13XX_REG_ANOTHER_PID				0x74
#define PCT13XX_REG_GEST_TYPE				0x7D
	#define PCT13XX_MOVE_CURSOR				0x01
	#define PCT13XX_ZOOM					0x09
#define PCT13XX_REG_OBJ_NUM					0x7C

/* bank 6 */
#define PCT13XX_REG_USER_ADDR				0x51
#define PCT13XX_REG_USER_DATA				0x52
#define PCT13XX_REG_STATUS					0x53
#define PCT13XX_REG_BOOT_STA				0x54
	#define PCT13XX_BOOT_READY				(1<<0)
	#define PCT13XX_CONTROL_READY			(1<<7)
#define PCT13XX_REG_ALC						0x55
	#define PCT13XX_ALC_OFF					0x01
	#define PCT13XX_ALC_ON					0x00
#define PCT13XX_REG_SHUTDOWN				0x7A
	#define PCT13XX_SHUTDOWN				0xAA
	#define PCT13XX_HARD_RESET				0xCC
	#define PCT13XX_RESUME					0xDD
#define PCT13XX_REG_WD						0x7D
	#define PCT13XX_WD_DISABLE				0xAD

/* user reg*/
#define PCT13XX_REG_PID						0x00
	#define PCT1332QN_PID					0x8B
#define PCT13XX_REG_HW_REV_ID				0x01
	#define PCT1332QN_HW_REV_ID				0x02
#define PCT13XX_REG_FW_MJREV_ID				0x02
#define PCT13XX_REG_FW_MNREV_ID				0x03
#define PCT13XX_REG_REPORT_NUM				0x0D
	#define PCT13XX_MAX_REPORT_NUM			5
#define PCT13XX_REG_HEIGHT_HI				0x10
#define PCT13XX_REG_HEIGHT_LO				0x11
#define PCT13XX_REG_WIDTH_HI				0x12
#define PCT13XX_REG_WIDTH_LO				0x13
#define PCT13XX_REG_CRC_CALC				0x6E
	#define PCT13XX_CRC_FW					(1<<1)
	#define PCT13XX_CRC_REG					(1<<2)
	#define PCT13XX_CRC_BUSY				(1<<0)
#define PCT13XX_REG_DEBUG_LO				0x70
#define PCT13XX_REG_DEBUG_HI				0x71

#define PCT13XX_REG_VERSION_HI				0x77
#define PCT13XX_REG_VERSION_LO				0x78

/* report status */
#define MOTION_REPORT_STATUS				(1<<4)
#define WATCHDOG_RESET_STATUS				(1<<7)

//#define TOUCH_DRIVER_DEBUG

#ifdef TOUCH_DRIVER_DEBUG
#define TOUCH_D_LOG(msg, ...) \
	pr_notice("[TOUCH][%s][D](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#else
#define TOUCH_D_LOG(msg, ...) \
	pr_debug("[TOUCH][%s][D](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#endif

#define TOUCH_N_LOG(msg, ...) \
	pr_debug("[TOUCH][%s][N](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

#define TOUCH_E_LOG(msg, ...) \
	pr_err("[TOUCH][%s][F](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

/* drv_debug commands */
#define PT_DBG_SUSPEND                  4
#define PT_DBG_RESUME                   5
#define PT_HANDSHAKE_ENABLE             6
#define PT_HANDSHAKE_DISABLE            7
#define KC_TOUCH_FACTORY_LOCK           202
#define KC_TOUCH_FACTORY_UNLOCK         203

#define GET_DATA_TYPE_RAW_DATA          0
#define GET_DATA_TYPE_TOUCH_DELTA_DATA  1
#define GET_DATA_TYPE_RAW_DATA_NEW      2

/* Debug buffer */
#define PCT13XX_MAX_PRBUF_SIZE		PIPE_BUF

/* wait for boot status */
enum {
	BOOT_COMPLETE,
	BOOT_NAV_READY,
};

/* reset event */
enum {
	POWER_HARD_RESET,
};

/* power status */
enum {
    TOUCH_POWERON,
    TOUCH_POWEROFF,
    TOUCH_UPDATE
};
/* sysfs operations */
enum {
	SYSFS_NULL = 0,
	SYSFS_READ = 1,
	SYSFS_WRITE,
	SYSFS_VERSION,
	SYSFS_SWITCH_BANK,
	SYSFS_RAW_DATA,
	SYSFS_PREPARE_RAW_DATA,
	SYSFS_MED3_DATA,
};

struct sysfs_data {
	uint8_t command;
	uint8_t reg[3];
	uint8_t data;
	uint8_t val0;
	uint8_t val1;
	uint16_t number;
};

struct press_diff_data {
	char delta_data1[TS_PCT13XX_MAX_DRIVE_NUM * TS_PCT13XX_MAX_SENSE_NUM * 2];
	char delta_data2[TS_PCT13XX_MAX_DRIVE_NUM * TS_PCT13XX_MAX_SENSE_NUM * 2];
	char delta_data3[TS_PCT13XX_MAX_DRIVE_NUM * TS_PCT13XX_MAX_SENSE_NUM * 2];
	char delta_data4[TS_PCT13XX_MAX_DRIVE_NUM * TS_PCT13XX_MAX_SENSE_NUM * 2];
	char delta_data5[TS_PCT13XX_MAX_DRIVE_NUM * TS_PCT13XX_MAX_SENSE_NUM * 2];
};

#endif

