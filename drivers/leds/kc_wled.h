/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 * (C) 2017 KYOCERA Corporation
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

#ifndef KC_WLED_H
#define KC_WLED_H

enum kc_wled_table {
	WLED_TBL_NORM,
	WLED_TBL_LOW_BAT,
	WLED_TBL_NUM
};

#define WLED_BRIGHTNESS_NUM		256
#define WLED_BRIGHTNESS_MAX		255
#define WLED_BRIGHTNESS_DEFAULT	84
#define WLED_BRIGHTNESS_OFF		00

void cat4004b_work(int level, enum kc_wled_table tbl, int gpio);

#endif
