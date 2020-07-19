/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
                              <kcjlog.h>
DESCRIPTION

EXTERNALIZED FUNCTIONS

This software is contributed or developed by KYOCERA Corporation.
(C) 2015 KYOCERA Corporation
(C) 2016 KYOCERA Corporation

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
 * set_kcj_pet_time()
 *
 * Note: Set utc time to uninit ram.
 */
void set_kcj_pet_time(void);

/*
 * set_kcj_regsave_addr()
 *
 * Note: Set regsave addr to uninit ram.
 */
void set_kcj_regsave_addr(unsigned long regsave_addr);

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

/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
 * QA LOG
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
/*
 * set_smem_crash_info_data()
 *
 * Note: Set qalog info data to smem.
 */
void set_smem_crash_info_data(const char *pdata);

/*
 * set_smem_crash_info_data_add_line()
 *
 * Note: Set qalog info line data to smem.
 */
void set_smem_crash_info_data_add_line(
	const unsigned int line,
	const char *func);

/*
 * set_smem_crash_info_data_add_reg()
 *
 * Note: Set qalog info reg data to smem.
 */
void set_smem_crash_info_data_add_reg(
	const unsigned long crash_pc,
	const unsigned long crash_lr,
	const char *task_comm);

/*
 * set_smem_panic_info_data()
 *
 * Note: Set qalog panic info data to smem.
 */
void set_smem_panic_info_data(const char *pdata);

/*
 * get_smem_crash_system()
 *
 * Note: Get crash system from smem.
 */
void get_smem_crash_system(
	unsigned char* crash_system,
	unsigned int bufsize);

/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
 * eMMC settings
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/

/*
 * get_emmc_info_settings()
 *
 * Note: Read the settings value of eMMC.
 */
int get_emmc_info_settings(unsigned int index, unsigned long* value);


#else /* CONFIG_ANDROID_KCJLOG */
/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
 * STUB FUNCTION DEFINITIONS
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
static inline void set_smem_kcjlog(
	crash_system_type system,
	crash_kind_type kind){}
static inline void set_kcj_pet_time(void){}
static inline void set_kcj_regsave_addr(unsigned long regsave_addr){}
static inline void set_kcj_crash_info(void){}
static inline void clear_kcj_crash_info(void){}
static inline void set_modemlog_info(void){}
static inline void dump_kcj_log(void){}
static inline void set_smem_crash_info_data(const char *pdata){}
static inline void set_smem_crash_info_data_add_line(
	const unsigned int line,
	const char *func){}
static inline void set_smem_crash_info_data_add_reg(
	const unsigned long crash_pc,
	const unsigned long crash_lr,
	const char *task_comm){}
static inline void set_smem_panic_info_data(const char *pdata){}
static inline void get_smem_crash_system(
	unsigned char* crash_system,
	unsigned int bufsize){}
static inline int get_emmc_info_settings(int index, unsigned long* value){return 0;};

#endif /* CONFIG_ANDROID_KCJLOG */
#ifdef KC_LOG_CNFD_ENABLED
void logger_flush_records(void *log);
#endif /* KC_LOG_CNFD_ENABLED */
#endif /* _KCJLOG_H_ */

