/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
                              <crash_reason.h>
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

*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
#ifndef _CRASH_REASON_H_
#define _CRASH_REASON_H_

/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
 * FUNCTION DEFINITIONS
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
#ifdef CONFIG_ANDROID_CRASH_REASON
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

#else /* CONFIG_ANDROID_CRASH_REASON */
/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
 * STUB FUNCTION DEFINITIONS
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
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
#endif /* CONFIG_ANDROID_CRASH_REASON */
#endif /* _CRASH_REASON_H_ */

