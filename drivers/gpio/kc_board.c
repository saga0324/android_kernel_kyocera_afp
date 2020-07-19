/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
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

#include <linux/kernel.h>
#include <linux/kc_board.h>
#include <soc/qcom/smsm.h>

extern void *kc_smem_alloc(unsigned smem_type, unsigned buf_size);

static oem_board_type _board_type = OEM_BOARD_FAIL_TYPE;
static oem_vendor_type _vendor_type = OEM_VENDOR_FAIL_TYPE;

/* get hw id */
oem_board_type OEM_get_board(void)
{
  oem_board_type* phwid = NULL;

  if(_board_type == OEM_BOARD_FAIL_TYPE)
  {
    phwid = (oem_board_type *) kc_smem_alloc( SMEM_HW_ID, 4);
    _board_type = *phwid;
  }

  return(_board_type);
}

/* get vendor cel */
oem_vendor_type OEM_get_vendor(void)
{
  oem_vendor_type* pvendor = NULL;
  if(_vendor_type == OEM_VENDOR_FAIL_TYPE)
  {
    pvendor = (oem_vendor_type *) kc_smem_alloc ( SMEM_VENDOR_ID, 4);
    _vendor_type = *pvendor;
  }

  return (_vendor_type);
}
