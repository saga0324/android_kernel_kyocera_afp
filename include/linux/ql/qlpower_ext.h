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
#ifndef QLPOWER_EXT_H
#define QLPOWER_EXT_H

typedef void (*qlpower_func)(void);

struct qlpower_callback {
    qlpower_func power_on;
    qlpower_func power_off;
};

extern void qlpower_spi_request(char* name);
extern void qlpower_spi_release(char* name);
extern int32_t qlpower_reg_cbfunc(struct qlpower_callback* cb);
extern int32_t qlpower_unreg_cbfunc(struct qlpower_callback* cb);
#endif
