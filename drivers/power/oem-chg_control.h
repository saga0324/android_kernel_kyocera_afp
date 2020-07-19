/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef OEM_CHG_COMTROL_H
#define OEM_CHG_COMTROL_H

#include <linux/qpnp/qpnp-adc.h>

extern int oem_chg_vadc_read(enum qpnp_vadc_channels channel,
				struct qpnp_vadc_result *result);

#endif
