/*
 * This software is contributed or developed by KYOCERA Corporation.
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

#ifndef __VOICETRIGGER_SENSOR_H_INCLUDED
#define __VOICETRIGGER_SENSOR_H_INCLUDED

#include <linux/device.h>

struct voice_trigger_sensor_info {
	struct device		*dev;
	struct device		*event_dev;
	struct wake_lock	wake_lock;
	atomic_t		valid;
};

#ifdef CONFIG_INPUT_SENSOR_VOICE_TRIGGER
extern int voice_trigger_sensor_register(struct voice_trigger_sensor_info *info);
extern int voice_trigger_sensor_unregister(struct voice_trigger_sensor_info *info);
extern int voice_trigger_sensor_report_event(struct voice_trigger_sensor_info *info, uint32_t detect);
extern int voice_trigger_sensor_report_flush_event(struct voice_trigger_sensor_info *info);
#else
static inline int voice_trigger_sensor_register(struct voice_trigger_sensor_info *info) {return 0;}
static inline int voice_trigger_sensor_unregister(struct voice_trigger_sensor_info *info)  {return 0;}
static inline int voice_trigger_sensor_report_event(struct voice_trigger_sensor_info *info, uint32_t detect)
	{return 0;}
static inline int voice_trigger_sensor_report_flush_event(struct voice_trigger_sensor_info *info)
	{return 0;}
#endif /* CONFIG_INPUT_SENSOR_VOICE_TRIGGER */

#endif		/* __VOICETRIGGER_SENSOR_H_INCLUDED */
