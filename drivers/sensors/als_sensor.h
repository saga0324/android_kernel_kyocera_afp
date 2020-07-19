/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
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

#ifndef __ALS_SENSOR_H_INCLUDED
#define __ALS_SENSOR_H_INCLUDED

#include <linux/device.h>
#ifndef CONFIG_USE_MICON_SOFT_STRUCTURE
#include "sensor_api.h"
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

struct als_sensor_info {
	struct device	*dev;
	struct device	*event_dev;
#ifndef CONFIG_USE_MICON_SOFT_STRUCTURE
	struct sensor_api_info sai;
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/
};

struct als_imit {
	int imit_flg;
	unsigned int imit_d0;
	unsigned int imit_d1;
};

#if defined(CONFIG_INPUT_SENSOR_LIGHT) && !defined(CONFIG_USE_MICON_SOFT_STRUCTURE)
extern int als_sensor_register(struct als_sensor_info *info);
extern int als_sensor_unregister(struct als_sensor_info *info);
extern int als_sensor_report_event(struct als_sensor_info *info, uint32_t lux, bool als_en_first);
extern int als_sensor_report_flush_event(struct als_sensor_info *info);
#else
static inline int als_sensor_register(struct als_sensor_info *info) {return 0;}
static inline int als_sensor_unregister(struct als_sensor_info *info)  {return 0;}
static inline int als_sensor_report_event(struct als_sensor_info *info, uint32_t lux, bool als_en_first)
	{return 0;}
static inline int als_sensor_report_flush_event(struct als_sensor_info *info)
	{return 0;}
#endif

#endif		/* __ALS_SENSOR_H_INCLUDED */
