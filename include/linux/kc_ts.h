/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 *
 * include/linux/kc_ts.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef __LINUX_KC_TS_H
#define __LINUX_KC_TS_H

#include <linux/types.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/cdev.h>

#define KC_TS_MAX_FINGER	5

#define TS_ERR_CHARGE			0x10
#define TS_ERR_DISCONNECTION	0x20
#define TS_ERR_FW_DL			0x40

#define ESD_POLLING_TIME		5000
#define ESD_FIRST_POLLING_TIME	10000
#define DEVICE_RESET_RETRY_CNT	3

#define TS_PCT13XX_MAX_DRIVE_NUM		17
#define TS_PCT13XX_MAX_SENSE_NUM		8
#define TS_DIAG_TEST_REQ_NUM			7

#define KC_TS_DEV_DBG(fmt, arg...)		if(ts_log_level & 0x01) pr_notice(fmt, ## arg)
#define KC_TS_DEV_INFO(fmt, arg...)		if(ts_log_level & 0x02) pr_notice(fmt, ## arg)
#define KC_TS_DEV_TOUCH(fmt, arg...)		if(ts_log_level & 0x04) pr_notice(fmt, ## arg)
#define KC_TS_DEV_I2C(fmt, arg...)		if(ts_log_level & 0x08) pr_notice(fmt, ## arg)

enum ts_config_type {
	TS_CHARGE_CABLE = 0,
	TS_CHARGE_A_S1,
	TS_CHARGE_A_S2,
	TS_DISCHARGE,
	TS_WIRELESS,
	TS_INITIAL_VALUE,
	TS_EXTENDED,
	TS_CONFIG_MAX,
};

struct ts_diag_event {
	int x;
	int y;
	int width;
};

struct ts_diag_type {
	struct ts_diag_event ts[KC_TS_MAX_FINGER];
	int diag_count;
};

struct ts_log_data {
	int flag;
	int data;
};

struct frame_buffer_read_data {
	uint16_t length;
	uint8_t buffer[TS_PCT13XX_MAX_DRIVE_NUM * TS_PCT13XX_MAX_SENSE_NUM * 2];
};

struct ts_req_param {
	uint8_t req_param[TS_DIAG_TEST_REQ_NUM];
};
struct ts_res_result {
	uint8_t result_max[TS_PCT13XX_MAX_DRIVE_NUM * TS_PCT13XX_MAX_SENSE_NUM * 2];
	uint8_t result_min[TS_PCT13XX_MAX_DRIVE_NUM * TS_PCT13XX_MAX_SENSE_NUM * 2];
};
struct ts_test_result {
	struct ts_req_param req_param;
	struct ts_res_result res_result;
};

struct ts_press_diff_test_res_data {
	uint8_t cmd_result;
	uint8_t delta_data_diff[TS_PCT13XX_MAX_DRIVE_NUM * TS_PCT13XX_MAX_SENSE_NUM * 2];
};

struct ts_press_diff_test_data {
	uint8_t param;
	uint16_t judgment_value;
	struct ts_press_diff_test_res_data press_diff_test_result;
};

struct kc_ts_data {
	const struct kc_ts_operations	*tops;
	struct device					*dev;
	void							*vdata;
	struct mutex					lock;
	struct cdev						device_cdev;
	int								device_major;
	struct class					*device_class;
	struct ts_diag_type				*diag_data;
	int								test_result;
	struct ts_test_result			*open_short_result;
	struct frame_buffer_read_data	*frame_data;
	struct ts_press_diff_test_data	*press_diff_test_data;
};

struct kc_ts_operations {
	void (*get_touch_info)(struct kc_ts_data *kd);
	int (*mode_check)(struct kc_ts_data *kd);
	int (*get_system_info)(struct kc_ts_data *kd, u8 *dp);
	int (*short_test)(struct kc_ts_data *kd);
	int (*open_test)(struct kc_ts_data *kd);
	int (*free_fingers)(struct kc_ts_data *kd);
	int (*get_raw_data)(struct kc_ts_data *kd);
	int (*get_touch_delta_data)(struct kc_ts_data *kd);
	int (*get_raw_data_new)(struct kc_ts_data *kd);
	int (*set_user_data)(struct kc_ts_data *kd, unsigned char color);
	int (*open_short_test)(struct kc_ts_data *kd);
	int (*press_diff_test)(struct kc_ts_data *kd);
};

extern unsigned int ts_event_report;
extern unsigned int ts_log_level;
extern unsigned int ts_esd_recovery;
extern unsigned int ts_config_switching;

int kc_ts_probe(struct kc_ts_data *ts);
void kc_ts_remove(struct kc_ts_data *ts);
int ts_ctrl_init(struct kc_ts_data *ts, const struct file_operations *fops);
int ts_ctrl_exit(struct kc_ts_data *ts);

#define IOC_MAGIC 't'
#define IOCTL_SET_CONF_STAT _IOW(IOC_MAGIC, 1, enum ts_config_type)
#define IOCTL_SET_USER_DATA _IOW(IOC_MAGIC, 3, unsigned char)
#define IOCTL_CHECK_FW _IOW(IOC_MAGIC, 11, struct ts_nv_data)
#define IOCTL_GET_INFO _IOW(IOC_MAGIC, 12, unsigned char)

#define IOCTL_DIAG_START _IO(IOC_MAGIC, 0xA1)
#define IOCTL_MULTI_GET _IOR(IOC_MAGIC, 0xA2, struct ts_diag_type)
#define IOCTL_COODINATE_GET _IOR(IOC_MAGIC, 0xA3, struct ts_diag_type)
#define IOCTL_DIAG_END _IO(IOC_MAGIC, 0xA4)
#define IOCTL_DIAG_LOG_LEVEL _IOW(IOC_MAGIC, 0xA5, unsigned char)
#define IOCTL_DIAG_EVENT_CTRL _IOW(IOC_MAGIC, 0xA6, unsigned char)
#define IOCTL_DIAG_RESET_HW _IO(IOC_MAGIC, 0xA7)
#define IOCTL_GET_GOLDEN_REFERENCE _IOWR(IOC_MAGIC, 0xA8, unsigned char)
#define IOCTL_DIAG_GET_C_REFERENCE _IOWR(IOC_MAGIC, 0xA9, unsigned char)
#define IOCTL_DIAG_GET_DELTA _IOR(IOC_MAGIC, 0xA0, unsigned char)
#define IOCTL_DIAG_SUSPEND_RESUME _IOW(IOC_MAGIC, 0xAA, unsigned char)

#define IOCTL_DIAG_SHORT_TEST _IOR(IOC_MAGIC, 0xD0, unsigned char)
#define IOCTL_DIAG_OPEN_TEST _IOR(IOC_MAGIC, 0xD1, unsigned char)

#define IOCTL_DIAG_MODE_CHECK _IOR(IOC_MAGIC, 0xD4, int)
#define IOCTL_DIAG_GET_SYSTEM_INFO  _IOR(IOC_MAGIC, 0xD5, int)
#define IOCTL_DIAG_GET_RAW_DATA _IOR(IOC_MAGIC, 0xD6, struct frame_buffer_read_data)
#define IOCTL_DIAG_GET_TOUCH_DELTA_DATA _IOR(IOC_MAGIC, 0xD7, struct frame_buffer_read_data)
#define IOCTL_DIAG_GET_RAW_DATA_NEW _IOR(IOC_MAGIC, 0xD8, struct frame_buffer_read_data)
#define IOCTL_DIAG_OPEN_SHORT_TEST _IOR(IOC_MAGIC, 0xD9, struct ts_test_result)
#define IOCTL_DIAG_PRESS_DIFF_TEST _IOR(IOC_MAGIC, 0xDA, struct ts_press_diff_test_data)

#endif /* __LINUX_KC_TS_H */
