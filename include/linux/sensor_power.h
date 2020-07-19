/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
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
#ifndef SENSOR_POWER_H
#define SENSOR_POWER_H

enum sensor_index{
	SENSOR_INDEX_ACC,
	SENSOR_INDEX_MAG,
	SENSOR_INDEX_GYRO,
	SENSOR_INDEX_ORI,
	SENSOR_INDEX_PSALS,
	SENSOR_INDEX_DEVICE_MAX,
};

typedef void (*kc_sensor_power_func)(void);

struct sensor_power_callback {
    kc_sensor_power_func power_on;
    kc_sensor_power_func power_off;
};

extern void sensor_power_on(enum sensor_index id);
extern void sensor_power_off(enum sensor_index id);
extern void sensor_power_reset(enum sensor_index id);
extern int32_t sensor_power_reg_cbfunc(struct sensor_power_callback* cb);
extern int32_t sensor_power_unreg_cbfunc(struct sensor_power_callback* cb);

#endif