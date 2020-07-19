/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
                              <crash_reason.c>
DESCRIPTION

EXTERNALIZED FUNCTIONS

This software is contributed or developed by KYOCERA Corporation.
(C) 2019 KYOCERA Corporation

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License version 2 and
only version 2 as published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
02110-1301, USA.

*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
#define pr_fmt(fmt) "crash_reason: " fmt
/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
 * INCLUDE FILES
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/rtc.h>
#include <linux/bio.h>
#include <linux/genhd.h>
#include <linux/delay.h>
#include <linux/pstore.h>
#include <linux/pstore_ram.h>
#include <linux/proc_fs.h>
#include <linux/kc_phymap.h>
#include <linux/crash_reason.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <soc/qcom/smsm.h>
#include "resetlog.h"

/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
 * FUNCTION DEFINITIONS
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
/*
 * check_smem_crash_reason()
 *
 * Note: Check crash reason from smem.
 */
static unsigned char check_smem_crash_reason( ram_log_info_type *plog_info )
{
	unsigned char ret = 0;
	unsigned char cmp_system[ CRASH_SYSTEM_SIZE ];
	unsigned char cmp_kind[ CRASH_KIND_SIZE ];

	memcpy( cmp_system, plog_info->crash_system, CRASH_SYSTEM_SIZE );

	/* system check */
	if ( (0 == strncmp(cmp_system, CRASH_SYSTEM_KERNEL    , strlen(CRASH_SYSTEM_KERNEL   ))) ||
		 (0 == strncmp(cmp_system, CRASH_SYSTEM_MODEM     , strlen(CRASH_SYSTEM_MODEM    ))) ||
		 (0 == strncmp(cmp_system, CRASH_SYSTEM_PRONTO    , strlen(CRASH_SYSTEM_PRONTO   ))) ||
		 (0 == strncmp(cmp_system, CRASH_SYSTEM_ADSP      , strlen(CRASH_SYSTEM_ADSP     ))) ||
		 (0 == strncmp(cmp_system, CRASH_SYSTEM_VENUS     , strlen(CRASH_SYSTEM_VENUS    ))) ||
		 (0 == strncmp(cmp_system, CRASH_SYSTEM_ANDROID   , strlen(CRASH_SYSTEM_ANDROID  ))) ||
		 (0 == strncmp(cmp_system, CRASH_SYSTEM_DM_VERITY , strlen(CRASH_SYSTEM_DM_VERITY))) )
	{
		ret = 1;
	}

	memcpy( cmp_kind, plog_info->crash_kind, CRASH_KIND_SIZE );

	/* kind check */
	if ( (0 == strncmp(cmp_kind, CRASH_KIND_PANIC            , strlen(CRASH_KIND_PANIC           ))) ||
		 (0 == strncmp(cmp_kind, CRASH_KIND_FATAL            , strlen(CRASH_KIND_FATAL           ))) ||
		 (0 == strncmp(cmp_kind, CRASH_KIND_EXCEPTION        , strlen(CRASH_KIND_EXCEPTION       ))) ||
		 (0 == strncmp(cmp_kind, CRASH_KIND_WDOG_HW          , strlen(CRASH_KIND_WDOG_HW         ))) ||
		 (0 == strncmp(cmp_kind, CRASH_KIND_WDOG_SW          , strlen(CRASH_KIND_WDOG_SW         ))) ||
		 (0 == strncmp(cmp_kind, CRASH_KIND_SYS_SERVER       , strlen(CRASH_KIND_SYS_SERVER      ))) ||
		 (0 == strncmp(cmp_kind, CRASH_KIND_PWR_KEY          , strlen(CRASH_KIND_PWR_KEY         ))) ||
		 (0 == strncmp(cmp_kind, CRASH_KIND_KDFS_REBOOT      , strlen(CRASH_KIND_KDFS_REBOOT     ))) ||
		 (0 == strncmp(cmp_kind, CRASH_KIND_DEVICE_CORRUPTED , strlen(CRASH_KIND_DEVICE_CORRUPTED))) )
	{
		ret = 1;
	}
	return ret;
}

/*
 * set_smem_crash_info_data()
 *
 * Note: Set qalog info data to smem.
 */
void set_smem_crash_info_data( const char *pdata )
{
	ram_log_info_type *plog_info;
	size_t            len_data;

	mb();
	plog_info = (ram_log_info_type *)kc_smem_alloc( SMEM_CRASH_LOG, SIZE_SMEM_ALLOC );

	if ( ( plog_info == NULL ) || ( pdata == NULL ) ) {
		return;
	}
	if ( strnlen( (const char *)plog_info->crash_info_data, CRASH_INFO_DATA_SIZE ) > 0 ) {
		return;
	}
	memset( plog_info->crash_info_data, '\0', sizeof(plog_info->crash_info_data) );
	len_data = strnlen( pdata, CRASH_INFO_DATA_SIZE );
	memcpy( plog_info->crash_info_data, pdata, len_data );
	mb();
}

/*
 * set_smem_crash_info_data_add_line()
 *
 * Note: Set qalog info line data to smem.
 */
void set_smem_crash_info_data_add_line( const unsigned int line, const char *func )
{
	char buf[33];
	memset( buf, '\0', sizeof(buf) );
	snprintf( buf, sizeof(buf), "%x;%s", line, func);
	set_smem_crash_info_data( (const char *)buf );
}

/*
 * set_smem_crash_info_data_add_reg()
 *
 * Note: Set qalog info reg data to smem.
 */
void set_smem_crash_info_data_add_reg( const unsigned long crash_pc, const unsigned long crash_lr, const char *task_comm )
{
	char buf[33];

	memset( buf, '\0', sizeof(buf) );
	snprintf( buf, sizeof(buf), "%lx,%lx,%s", crash_pc, crash_lr, task_comm );
	set_smem_panic_info_data( (const char *)buf );
}

/*
 * set_smem_panic_info_data()
 *
 * Note: Set qalog panic info data to smem.
 */
void set_smem_panic_info_data( const char *pdata )
{
	ram_log_info_type *plog_info;
	size_t            len_data;

	mb();
	plog_info = (ram_log_info_type *)kc_smem_alloc( SMEM_CRASH_LOG, SIZE_SMEM_ALLOC );

	if ( ( plog_info == NULL ) || ( pdata == NULL ) ) {
		return;
	}
	if ( strnlen( (const char *)plog_info->panic_info_data, CRASH_INFO_DATA_SIZE ) > 0 ) {
		return;
	}
	memset( plog_info->panic_info_data, '\0', sizeof(plog_info->panic_info_data) );
	len_data = strnlen( pdata, CRASH_INFO_DATA_SIZE );
	memcpy( plog_info->panic_info_data, pdata, len_data );
	mb();
}

/*
 * get_smem_crash_system()
 *
 * Note: Get crash system from smem.
 */
void get_smem_crash_system(unsigned char* crash_system,unsigned int bufsize)
{
	ram_log_info_type *pSmemLogInfo;

	mb();
	pSmemLogInfo = (ram_log_info_type *)kc_smem_alloc(SMEM_CRASH_LOG, SIZE_SMEM_ALLOC);

	if (pSmemLogInfo == NULL) {
		return;
	}
	if (check_smem_crash_reason(pSmemLogInfo) == 0) {
		return;
	}
	if (bufsize > CRASH_SYSTEM_SIZE) {
		bufsize = CRASH_SYSTEM_SIZE;
	}

	memcpy( crash_system, &pSmemLogInfo->crash_system[0], bufsize );
}

static int __init crash_reason_init(void) {
	printk(KERN_INFO "crash_reason initialized\n");
	return 0;
}

static void __exit crash_reason_exit(void) {
	printk(KERN_INFO "crash_reason exit\n");
}

module_init(crash_reason_init);
module_exit(crash_reason_exit);
