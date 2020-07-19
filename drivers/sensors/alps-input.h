/* This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 */
/* drivers/input/misc/alps-input.h
 *
 * Input device driver for alps sensor
 *
 * Copyright (C) 2011-2014 ALPS ELECTRIC CO., LTD. All Rights Reserved.
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

#ifndef ___ALPS_INPUT_H_INCLUDED
#define ___ALPS_INPUT_H_INCLUDED

extern int	accsns_get_acceleration_data(int *xyz);
extern void	accsns_activate(int flgatm, int flg, int dtime);
#ifdef CONFIG_INPUT_HSCDTD
extern int	hscdtd_get_magnetic_field_data(int *xyz);
extern void	hscdtd_activate(int flgatm, int flg, int dtime);
extern int	hscdtd_self_test_A(void);
extern int	hscdtd_self_test_B(void);
extern int	hscdtd_get_hardware_data(int *xyz);
#else
static int	hscdtd_get_magnetic_field_data(int *xyz) {xyz[0] = xyz[1] = xyz[2] = 0; return -1;}
static void	hscdtd_activate(int flgatm, int flg, int dtime) {return;}
static int	hscdtd_self_test_A(void) {return 0;}
static int	hscdtd_self_test_B(void) {return 0;}
static int	hscdtd_get_hardware_data(int *xyz) {xyz[0] = xyz[1] = xyz[2] = 0; return -1;}
#endif
#endif
