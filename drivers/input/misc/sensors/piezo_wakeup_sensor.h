/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2017 KYOCERA Corporation
 * (C) 2018 KYOCERA Corporation
 * (C) 2019 KYOCERA Corporation
 */
/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
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

#ifndef __PIEZOWAKEUP_SENSOR_H_INCLUDED
#define __PIEZOWAKEUP_SENSOR_H_INCLUDED

#include <linux/device.h>

#define PIEZOWAKEUP_DUMMY_VALUE	-1
#define PIEZOWAKEUP_STATUS_NEAR	1
#define PIEZOWAKEUP_STATUS_FAR		0

struct piezo_wakeup_sensor_info {
	struct device		*dev;
	struct device		*event_dev;
	struct wake_lock	wake_lock;
	atomic_t		valid;
};

#ifdef CONFIG_INPUT_SENSOR_PIEZO_WAKEUP
extern int piezo_wakeup_sensor_register(struct piezo_wakeup_sensor_info *info);
extern int piezo_wakeup_sensor_unregister(struct piezo_wakeup_sensor_info *info);
extern int piezo_wakeup_sensor_report_event(struct piezo_wakeup_sensor_info *info, uint32_t detect);
extern int piezo_wakeup_sensor_report_flush_event(struct piezo_wakeup_sensor_info *info);
#else
static inline int piezo_wakeup_sensor_register(struct piezo_wakeup_sensor_info *info) {return 0;}
static inline int piezo_wakeup_sensor_unregister(struct piezo_wakeup_sensor_info *info)  {return 0;}
static inline int piezo_wakeup_sensor_report_event(struct piezo_wakeup_sensor_info *info, uint32_t detect)
	{return 0;}
static inline int piezo_wakeup_sensor_report_flush_event(struct piezo_wakeup_sensor_info *info)
	{return 0;}
#endif /* CONFIG_INPUT_SENSOR_PIEZO_WAKEUP */

#endif		/* __PIEZOWAKEUP_SENSOR_H_INCLUDED */
