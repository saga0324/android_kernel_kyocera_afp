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
#ifndef QLPOWER_H
#define QLPOWER_H

#include <linux/device.h>

extern void qlpower_on_cbfunc(void);
extern void qlpower_off_cbfunc(void);
extern void qlpower_on(void);
extern void qlpower_off(void);
extern void qlpower_reset(void);
extern int32_t qlpower_initialize(struct device *dev);
extern int32_t qlpower_remove(struct device *dev);
#endif
