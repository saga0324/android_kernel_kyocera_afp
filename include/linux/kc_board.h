/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 */
/* Copyright (c) 2010-2012, Code Aurora Forum. All rights reserved.
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
 */
#ifndef _KC_BOARD_H_
#define _KC_BOARD_H_

/* ----------------------------------------------------*/
/* HW ID                                               */
/* ----------------------------------------------------*/
typedef enum {
    OEM_BOARD0_TYPE     = 0,
    OEM_BOARD1_TYPE     = 1,
    OEM_BOARD2_TYPE     = 2,
    OEM_BOARD3_TYPE     = 3,
    OEM_BOARD4_TYPE     = 4,
    OEM_BOARD5_TYPE     = 5,
    OEM_BOARD6_TYPE     = 6,
    OEM_BOARD7_TYPE     = 7,
    OEM_BOARD_FAIL_TYPE = 99,
} oem_board_type;

/* ----------------------------------------------------*/
/* VENDOR CEL                                          */
/* ----------------------------------------------------*/
typedef enum {
    OEM_VENDOR0_TYPE     = 0,
    OEM_VENDOR1_TYPE     = 1,
    OEM_VENDOR2_TYPE     = 2,
    OEM_VENDOR3_TYPE     = 3,
    OEM_VENDOR4_TYPE     = 4,
    OEM_VENDOR5_TYPE     = 5,
    OEM_VENDOR6_TYPE     = 6,
    OEM_VENDOR7_TYPE     = 7,
    OEM_VENDOR_FAIL_TYPE = 99,
} oem_vendor_type;

extern oem_board_type OEM_get_board(void);
extern oem_vendor_type OEM_get_vendor(void);
#endif /* _KC_BOARD_H_ */
