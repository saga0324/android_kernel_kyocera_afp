/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
                              <kcjlog.h>
DESCRIPTION

EXTERNALIZED FUNCTIONS

This software is contributed or developed by KYOCERA Corporation.
(C) 2015 KYOCERA Corporation
(C) 2016 KYOCERA Corporation
(C) 2017 KYOCERA Corporation
(C) 2018 KYOCERA Corporation
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

*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
#ifndef _KCJLOG_H_
#define _KCJLOG_H_

/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
 * ENUM DECLARATIONS
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
typedef enum{
	SYSTEM_KERNEL,
	SYSTEM_MODEM,
	SYSTEM_PRONTO,
	SYSTEM_ADSP,
	SYSTEM_VENUS,
	SYSTEM_ANDROID,
	SYSTEM_DM_VERITY,
	SYSTEM_UNKNOWN,
	SYSTEM_MAX
}crash_system_type;

typedef enum{
	KIND_PANIC,
	KIND_FATAL,
	KIND_EXCEPTION,
	KIND_WDOG_HW,
	KIND_WDOG_SW,
	KIND_SYS_SERVER,
	KIND_PWR_KEY,
	KIND_KDFS_REBOOT,
	KIND_DEVICE_CORRUPTED,
	KIND_UNKNOWN,
	KIND_MAX
}crash_kind_type;

/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
 * FUNCTION DEFINITIONS
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
#ifdef CONFIG_ANDROID_KCJLOG
/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
 * RESET LOG
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
/*
 * set_smem_kcjlog()
 *
 * Note: Set kcjlog to smem.
 */
void set_smem_kcjlog(crash_system_type system, crash_kind_type kind);

/*
 * set_kcj_regsave_addr()
 *
 * Note: Set regsave addr to uninit ram.
 */
void set_kcj_regsave_addr(unsigned int regsave_addr);

/*
 * set_kcj_fixed_info_modem()
 *
 * Note: Set kcj modem fixed info to uninit ram.
 */
void set_kcj_fixed_info_modem(void);

/*
 * set_kcj_crash_info()
 *
 * Note: Set crash info to uninit ram.
 */
void set_kcj_crash_info(void);

/*
 * clear_kcj_crash_info()
 *
 * Note: Clear crash info from uninit ram.
 */
void clear_kcj_crash_info(void);

/*
 * set_modemlog_info()
 *
 * Note: Set modem log info from smem to uninit ram.
 */
void set_modemlog_info(void);

/*
 * dump_kcj_log()
 *
 * Note: Dump kcj log from uninit ram to eMMC.
 */
void dump_kcj_log(void);

#else /* CONFIG_ANDROID_KCJLOG */
/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
 * STUB FUNCTION DEFINITIONS
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
static inline void set_smem_kcjlog(
	crash_system_type system,
	crash_kind_type kind){}
static inline void set_kcj_pet_time(void){}
static inline void set_kcj_regsave_addr(unsigned int regsave_addr){}
static inline void set_kcj_crash_info(void){}
static inline void clear_kcj_crash_info(void){}
static inline void set_modemlog_info(void){}
static inline void dump_kcj_log(void){}
static inline void set_kcj_fixed_info_modem(void){}
#endif /* CONFIG_ANDROID_KCJLOG */
#endif /* _KCJLOG_H_ */

