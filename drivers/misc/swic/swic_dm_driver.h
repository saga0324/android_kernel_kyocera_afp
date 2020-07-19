/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 * (C) 2018 KYOCERA Corporation
 * (C) 2019 KYOCERA Corporation
 */

#ifndef SWIC_DM_DRIVER_H
#define SWIC_DM_DRIVER_H

#include <linux/ioctl.h>

#define SWIC_DM_DRIVER_IOCTL_01					0x10

#define SWIC_DM_DRIVER_GET_SWIC_REG_STATET_MODE	 0


extern u8 swic_get_dminfo(unsigned char cmd, int32_t *val);
extern u8 swic_set_swic_det_wait(int32_t *val);

#endif
