/* drivers/input/misc/alps_acc.h
 *
* Input device driver for accelerometer sensor (common)
 *
 * Copyright (C) 2011-2014 ALPS ELECTRIC CO., LTD. All Rights Reserved.
 *
 */

#ifndef ___ALPS_ACC_H_INCLUDED
#define ___ALPS_ACC_H_INCLUDED

struct acc_if_funcs {
    int (*activate)(int flgatm, int flg, int dtime);
    int (*get_data)(int *xyz);
};

#if defined (CONFIG_INPUT_SENSOR_ACC_U2DH)
void accsns_func_register_u2dh(struct acc_if_funcs *f);
#elif defined (CONFIG_INPUT_SENSOR_ACC_MXC400XXC)
void accsns_func_register_mxc400xxc(struct acc_if_funcs *f);
#endif

#endif /* ___ALPS_ACC_IO_H_INCLUDED */

