/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 */
/*
 * include/linux/pct13xx_display_touch.h
 *
 * PixArt pct13xx TouchScreen driver.
 *
 * Copyright (c) 2015 PixArt Imaging Inc.
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

#ifndef KC_TOUCH_DISPLAY
#define KC_TOUCH_DISPLAY

#include <linux/notifier.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/kc_ts.h>

#define PCT13XX_ID_MAX		5

#pragma pack(1)
struct point_data {
	uint8_t id;
	uint16_t x;
	uint16_t y;
	uint16_t force;
	uint16_t area;
};

struct touch_report {
	uint8_t status;
	uint8_t total_touch;
	struct point_data point_data[PCT13XX_ID_MAX];
};
#pragma pack()

struct ts_config_nv {
	size_t size;
	u16 ver;
	u8 *data;
};

/**
 * struct pct13xx_data
 * @gpio_irq: gpio pin control -> this pin is used to control the
 *	interrupt pin
 *
 */
struct pct13xx_data {
	bool irq_enabled;
	bool is_suspended;
	uint8_t power;
	uint8_t app_power;
	uint8_t pid;
	uint8_t handshake_stat;
	uint32_t gpio_irq;
	uint32_t err_esd_cnt;
	uint32_t err_irq_cnt;
	unsigned int	vdd_gpio;
	unsigned int	reset_gpio;
	unsigned int	irq_gpio;
	int touch_cnt;
	struct i2c_client *client;
    struct input_dev *input;
	struct workqueue_struct *esd_wq;
	struct workqueue_struct *resume_wq;
	struct device *dev;
	struct mutex system_lock;
	struct mutex mt_lock;
	struct mutex handshake_lock;
	struct delayed_work		esdwork;
	struct work_struct		resumework;

	struct kc_ts_data kc_ts;
	struct point_data touch_info[PCT13XX_ID_MAX];
	struct ts_config_nv					config_nv[TS_CONFIG_MAX];
};

extern void pct13xx_display_touch(struct pct13xx_data *pct_data);
extern int pct13xx_trans_run_mode(struct pct13xx_data *display_touch, int order);
extern int pct13xx_trans_shutdown_mode(struct pct13xx_data *display_touch, int order);

#endif /* KC_TOUCH_DISPLAY */