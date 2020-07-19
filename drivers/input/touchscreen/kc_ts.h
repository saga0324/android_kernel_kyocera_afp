/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 * (C) 2019 KYOCERA Corporation
 *
 * drivers/input/touchscreen/kc_ts.h
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

#define KC_TS_MAX_FINGER	16

#define KC_TS_DEV_DBG(fmt, arg...)		if(ts_log_level & 0x01) pr_notice(fmt, ## arg)
#define KC_TS_DEV_INFO(fmt, arg...)		if(ts_log_level & 0x02) pr_notice(fmt, ## arg)
#define KC_TS_DEV_TOUCH(fmt, arg...)		if(ts_log_level & 0x04) pr_notice(fmt, ## arg)
#define KC_TS_DEV_I2C(fmt, arg...)		if(ts_log_level & 0x08) pr_notice(fmt, ## arg)

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

struct kc_ts_data {
	const struct kc_ts_operations		*tops;
	struct device				*dev;
	void					*vdata;
	struct mutex				lock;
	struct cdev				device_cdev;
	int					device_major;
	struct class				*device_class;
	struct ts_diag_type			*diag_data;
};

struct kc_ts_operations {
	void (*get_touch_info)(struct kc_ts_data *ts);
	int (*mode_check)(struct kc_ts_data *ts);
	int (*get_system_info)(struct kc_ts_data *ts, u8 *dp);
#ifdef CONFIG_TOUCHSCREEN_KC_SENSOR_SWITCH
	int (*sensor_switch)(struct kc_ts_data *ts, int value);
#endif /* CONFIG_TOUCHSCREEN_KC_SENSOR_SWITCH */
#ifdef CONFIG_TOUCHSCREEN_KC_TOUCHMODE_IOCTL
	int (*set_touchmode)(struct kc_ts_data *ts, int value);
#endif /* CONFIG_TOUCHSCREEN_KC_TOUCHMODE_IOCTL */
};

extern unsigned int ts_event_control;
extern unsigned int ts_log_level;
extern unsigned int ts_esd_recovery;
extern unsigned int ts_config_switching;
extern unsigned int ts_error_status;

int kc_ts_probe(struct kc_ts_data *ts);
void kc_ts_remove(struct kc_ts_data *ts);
int ts_ctrl_init(struct kc_ts_data *ts, const struct file_operations *fops);
int ts_ctrl_exit(struct kc_ts_data *ts);

#define IOC_MAGIC 't'
#define IOCTL_SET_CONF_STAT _IOW(IOC_MAGIC, 1, enum ts_config_type)
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

#ifdef CONFIG_TOUCHSCREEN_KC_SENSOR_SWITCH
#define IOCTL_SET_SENSOR_SWITCH _IOW(IOC_MAGIC, 0xC7, int)
#endif /* CONFIG_TOUCHSCREEN_KC_SENSOR_SWITCH */
#ifdef CONFIG_TOUCHSCREEN_KC_TOUCHMODE_IOCTL
#define IOCTL_SET_TOUCHMODE     _IOW(IOC_MAGIC, 0xC8, int)
#endif /* CONFIG_TOUCHSCREEN_KC_TOUCHMODE_IOCTL */
#define IOCTL_GET_TOUCHEVENTCONTROL     _IOR(IOC_MAGIC, 0xC9, int)

#define IOCTL_DIAG_MODE_CHECK _IOR(IOC_MAGIC, 0xD4, int)
#define IOCTL_DIAG_GET_SYSTEM_INFO  _IOR(IOC_MAGIC, 0xD5, int)

#endif /* __LINUX_KC_TS_H */
