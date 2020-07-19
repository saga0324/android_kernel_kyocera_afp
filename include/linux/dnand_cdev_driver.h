#ifndef DNAND_CDEV_DRIVER_H
#define DNAND_CDEV_DRIVER_H
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 and
 *  only version 2 as published by the Free Software Foundation.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/types.h>
#include "dnand_clid.h"
#include "dnand_status.h"
#include "dnand_k_api.h"

#define DNAND_CDEV_DRIVER_IOCTL_01 (0x10)
#define DNAND_CDEV_DRIVER_IOCTL_02 (0x11)

typedef struct dnand_data_type_struct
{
	uint32_t cid;
	uint32_t offset;
	uint8_t *pbuf;
	uint32_t size;
} dnand_data_type;

#endif
