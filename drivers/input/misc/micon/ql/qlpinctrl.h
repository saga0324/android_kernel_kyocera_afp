/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2019 KYOCERA Corporation
 */
/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#ifndef QLPINCTRL_H
#define QLPINCTRL_H

#include <linux/device.h>

enum qlpin_id {
	QLPIN_DEFAULT = 0,
	QLPIN_ACTIVE,
	QLPIN_SUSPEND,
	QLPIN_CS_ACTIVE,
	QLPIN_CS_SUSPEND,
	QLPIN_CS_RESET,
	QLPIN_RST_ACTIVE,
	QLPIN_RST_SUSPEND,
	QLPIN_VMICON_11_ACTIVE,
	QLPIN_VMICON_11_SUSPEND,
	QLPIN_VMICON_18_ACTIVE,
	QLPIN_VMICON_18_SUSPEND,
	QLPIN_SLEEP_CLK_ACTIVE,
	QLPIN_SLEEP_CLK_SUSPEND,
	QLPIN_VSENSOR_18_ACTIVE,
	QLPIN_VSENSOR_18_SUSPEND,
#ifdef CONFIG_QL_SPI_CTRL_FROMCLIENT
	QLPIN_SPI_ACTIVE,
	QLPIN_SPI_SUSPEND,
	QLPIN_SPI_MICON_CS_REQUEST,
#endif
	QLPIN_ID_MAX
};

extern int32_t qlpinctrl_select(enum qlpin_id id);
extern int32_t qlpinctrl_initialize(struct device *dev);
extern int32_t qlpinctrl_remove(struct device *dev);
#endif
