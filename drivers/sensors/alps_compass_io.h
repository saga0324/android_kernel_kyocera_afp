/* drivers/input/misc/alps_compass_io.h
 *
 * I/O controll header for alps sensor
 *
 * Copyright (C) 2011-2014 ALPS ELECTRIC CO., LTD. All Rights Reserved.
 *
 */

#ifndef ___ALPS_IO_H_INCLUDED
#define ___ALPS_IO_H_INCLUDED

#include <linux/ioctl.h>

#define ALPSIO   0xAF

#define ALPSIO_SET_MAGACTIVATE   _IOW(ALPSIO, 0x00, int)
#define ALPSIO_SET_ACCACTIVATE   _IOW(ALPSIO, 0x01, int)
#define ALPSIO_SET_DELAY         _IOW(ALPSIO, 0x02, int)
#define ALPSIO_ACT_SELF_TEST_A   _IOR(ALPSIO, 0x03, int)
#define ALPSIO_ACT_SELF_TEST_B   _IOR(ALPSIO, 0x04, int)
#define ALPSIO_GET_HWDATA        _IOR(ALPSIO, 0x05, int[3])

#endif

