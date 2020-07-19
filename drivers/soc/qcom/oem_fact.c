/* This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */


#include <linux/kernel.h>
#include <soc/qcom/smsm.h>
#include <soc/qcom/oem_fact.h>

#define OEM_FACT_OPTION_SMEM_ID        SMEM_FACTORY_OPTIONS
#define OEM_FACT_OPTION_SMEM_SIZE      4

#define OEM_FACT_OPTION_ITEM_SMEM_MIN  OEM_FACT_OPTION_ITEM_01
#define OEM_FACT_OPTION_ITEM_SMEM_MAX  OEM_FACT_OPTION_ITEM_04


int oem_fact_get_option_bit( unsigned char item, unsigned char bit )
{
        int           result    = 0;
        unsigned char *smem_ptr = NULL;
        unsigned int  offset;

        if ( ( item >= OEM_FACT_OPTION_ITEM_SMEM_MIN ) && ( item <= OEM_FACT_OPTION_ITEM_SMEM_MAX ) ) {
                if ( bit > 7 ) {
                        return result;
                }
                smem_ptr = kc_smem_alloc( OEM_FACT_OPTION_SMEM_ID, OEM_FACT_OPTION_SMEM_SIZE );
                if ( smem_ptr ) {
                        offset = item - OEM_FACT_OPTION_ITEM_SMEM_MIN;
                        result = ( *( smem_ptr + offset ) & ( 0x01 << bit ) ) > 0 ? 1 : 0;
                }
        }

        return result;
}

