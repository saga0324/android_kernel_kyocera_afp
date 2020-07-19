/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 */
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

#ifndef KEY_DM_DRIVER_H
#define KEY_DM_DRIVER_H

#include <linux/ioctl.h>

#define KEY_DM_DRIVER_OK   0

#define KEY_DM_DRIVER_IOCTL_01  0x10
#define KEY_DM_DRIVER_IOCTL_02  0x11
#define KEY_DM_DRIVER_IOCTL_03  0x12
#define KEY_DM_DRIVER_IOCTL_04  0x13
#define KEY_DM_DRIVER_IOCTL_05  0x14
#define KEY_DM_DRIVER_IOCTL_06  0x15
#define KEY_DM_DRIVER_IOCTL_07  0x16

#define KEY_DM_PRESS_COMMAND 0
#define KEY_DM_CHECK_COMMAND 1
#define KEY_DM_KEY_GET_EVENT_COOMAND 2
#define KEY_DM_START_PORT_CHECK_COMMAND 3
#define KEY_DM_DIAG_PORT_EVENT_COMMAND 4
#define KEY_DM_END_DIAG_PORT_CHECK_COMMAND 5

extern unsigned char key_cmd(unsigned char cmd, int *val);
extern void key_set_code(unsigned int code);
extern unsigned char pwrkey_cmd(unsigned char cmd, int *val);
extern unsigned char matrixkey_cmd(unsigned char cmd, int *val);
extern void key_dm_driver_set_port(unsigned char in_swmode);

#endif
