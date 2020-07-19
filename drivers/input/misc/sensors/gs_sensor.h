/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2017 KYOCERA Corporation
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

#ifndef __GS_SENSOR_H_INCLUDED
#define __GS_SENSOR_H_INCLUDED

#include <linux/device.h>
#ifndef CONFIG_USE_MICON_SOFT_STRUCTURE
#include "sensor_api.h"
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

#define GESTURE_DUMMY_VALUE	-1
#define GESTURE_STATUS_NEAR	1
#define GESTURE_STATUS_FAR		0

struct gs_sensor_info {
	struct device		*dev;
	struct device		*event_dev;
#ifndef CONFIG_USE_MICON_SOFT_STRUCTURE
	struct sensor_api_info	sai;
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/
	struct wake_lock	wake_lock;
	atomic_t		valid;
};

#if defined(CONFIG_INPUT_SENSOR_GESTURE) && !defined(CONFIG_USE_MICON_SOFT_STRUCTURE)
extern int gs_sensor_register(struct gs_sensor_info *info);
extern int gs_sensor_unregister(struct gs_sensor_info *info);
extern int gs_sensor_report_event(struct gs_sensor_info *info, uint32_t detect);
extern int gs_sensor_report_flush_event(struct gs_sensor_info *info);
#else
static inline int gs_sensor_register(struct gs_sensor_info *info) {return 0;}
static inline int gs_sensor_unregister(struct gs_sensor_info *info)  {return 0;}
static inline int gs_sensor_report_event(struct gs_sensor_info *info, uint32_t detect)
	{return 0;}
static inline int gs_sensor_report_flush_event(struct gs_sensor_info *info)
	{return 0;}
#endif

#endif		/* __gS_SENSOR_H_INCLUDED */
