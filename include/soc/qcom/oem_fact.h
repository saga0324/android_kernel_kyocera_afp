/* This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
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

#ifndef _OEM_FACT_H_
#define _OEM_FACT_H_

enum {
        OEM_FACT_OPTION_ITEM_01,
        OEM_FACT_OPTION_ITEM_02,
        OEM_FACT_OPTION_ITEM_03,
        OEM_FACT_OPTION_ITEM_04
};

extern int oem_fact_get_option_bit( unsigned char item, unsigned char bit );

#endif /* _OEM_FACT_H_ */
