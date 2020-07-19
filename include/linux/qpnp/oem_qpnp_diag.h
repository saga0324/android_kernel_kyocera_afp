/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __OEM_QPNP_DIAG_H_
#define __OEM_QPNP_DIAG_H_

int oem_qpnp_diag_read(int offset, u8 *val);
int oem_qpnp_diag_masked_write(int offset, u8 mask, u8 val);

#endif /* __OEM_QPNP_DIAG_H_ */
