/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 *
 * drivers/video/msm/mdss/disp_ext_sub_dbg.h
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
#ifndef DISP_EXT_SUB_DBG_H
#define DISP_EXT_SUB_DBG_H

#include "disp_ext_sub_ctrl.h"

void disp_ext_sub_img_dump( void* img_p, uint32_t size, uint32_t byte_size );
void disp_ext_sub_seq_log( uint8_t *data_p, uint32_t size );
void disp_ext_sub_pdata_dump( struct disp_ext_sub_pdata *pdata );

#endif /* DISP_EXT_SUB_DBG_H */
