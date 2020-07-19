#ifndef DNAND_DRV_H
#define DNAND_DRV_H
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
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

#define DNAND_DRV_SECTOR_BLK_SIZE            (512)

typedef enum
{
	DNAND_DEV_READ    = 0,
	DNAND_DEV_WRITE,
	DNAND_DEV_MAX
} dnand_dev_rw_type;

int32_t dnand_drv_read(uint32_t sector, uint32_t num_sector, uint8_t *pbuf);
int32_t dnand_drv_write(uint32_t sector, uint32_t num_sector, uint8_t *pbuf);

int __init dnand_drv_init(void);
#endif // DNAND_DRV_H
