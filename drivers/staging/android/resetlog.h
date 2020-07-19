/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
                              <resetlog.h>
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
#ifndef _RESET_LOG_H
#define _RESET_LOG_H

#define HEADER_VERSION           "v4.1.0"
#define HEADER_VERSION_SIZE      (8)

#include <linux/kc_phymap.h>
#ifdef MSM_UNINIT_RAM_BASE
#define MSM_KCJLOG_BASE         (MSM_UNINIT_RAM_BASE)
#endif/*MSM_UNINIT_RAM_BASE*/

#define SIZE_SMEM_ALLOC         (512)
#define SIZE_CONTROL_INFO       (1024)
#define SIZE_KERNEL_LOG         (512 * 1024)
#define SIZE_LOGCAT_MAIN        (512 * 1024)
#define SIZE_LOGCAT_SYSTEM      (512 * 1024)
#define SIZE_LOGCAT_EVENTS      (256 * 1024)
#define SIZE_LOGCAT_RADIO       (512 * 1024)
#define SIZE_M_LOG              (642 * 1024)

#ifdef  MSM_KCJLOG_BASE
#define ADDR_CONTROL_INFO       (MSM_KCJLOG_BASE)
#define ADDR_LOGCAT_MAIN        (ADDR_CONTROL_INFO       + SIZE_CONTROL_INFO      )
#define ADDR_LOGCAT_SYSTEM      (ADDR_LOGCAT_MAIN        + SIZE_LOGCAT_MAIN       )
#define ADDR_LOGCAT_EVENTS      (ADDR_LOGCAT_SYSTEM      + SIZE_LOGCAT_SYSTEM     )
#define ADDR_LOGCAT_RADIO       (ADDR_LOGCAT_EVENTS      + SIZE_LOGCAT_EVENTS     )

#endif/*MSM_KCJLOG_BASE*/

enum {
    LOGGER_INFO_MAIN,
    LOGGER_INFO_SYSTEM,
    LOGGER_INFO_EVENTS,
    LOGGER_INFO_RADIO,
    LOGGER_INFO_MAX,
};

enum {
    BOOTMODE_ID_UNKNOWN,
    BOOTMODE_ID_ANDROID,
    BOOTMODE_ID_ZEMI,
    BOOTMODE_ID_ANDROID_COMB,
    BOOTMODE_ID_BEF_ANDROID_COMB,
    BOOTMODE_ID_NOT_SUPPORT,
    BOOTMODE_ID_MAX
};

#define CRASH_MAGIC_CODE              "KC ERROR"

#define CRASH_SYSTEM_KERNEL           "KERNEL"
#define CRASH_SYSTEM_MODEM            "MODEM"
#define CRASH_SYSTEM_PRONTO           "PRONTO"
#define CRASH_SYSTEM_ADSP             "ADSP"
#define CRASH_SYSTEM_VENUS            "VENUS"
#define CRASH_SYSTEM_ANDROID          "ANDROID"
#define CRASH_SYSTEM_DM_VERITY        "DM-VERITY"
#define CRASH_SYSTEM_UNKNOWN          "UNKNOWN"

#define CRASH_KIND_PANIC              "KERNEL PANIC"
#define CRASH_KIND_FATAL              "ERR FATAL"
#define CRASH_KIND_EXCEPTION          "EXCEPTION"
#define CRASH_KIND_WDOG_HW            "HW WATCH DOG"
#define CRASH_KIND_WDOG_SW            "SW WATCH DOG"
#define CRASH_KIND_SYS_SERVER         "SYSTEM SERVER CRASH"
#define CRASH_KIND_PWR_KEY            "PWR KEY"
#define CRASH_KIND_KDFS_REBOOT        "KDFS REBOOT"
#define CRASH_KIND_DEVICE_CORRUPTED   "DEVICE CORRUPTED"
#define CRASH_KIND_UNKNOWN            "UNKNOWN"

#define KERNEL_LOG_MAGIC_CODE         (0x4E52454B)

/* RAM_CONSOLE Contol */
typedef struct {
    uint32_t                    sig;
    uint32_t                    start;
    uint32_t                    size;
} ram_console_header_type;

typedef struct {
    ram_console_header_type     header;
    unsigned char               msg[1];
} ram_console_type;

/* Log Control */
typedef struct {
    uint32_t                    w_off;
    uint32_t                    head;
} logger_log_info;

#define MAGIC_CODE_SIZE         (16)
#define CRASH_SYSTEM_SIZE       (16)
#define CRASH_KIND_SIZE         (32)
#define CRASH_TIME_SIZE         (48)

#define VERSION_SIZE            (64)
#define MODEL_SIZE              (32)
#define CRASH_INFO_DATA_SIZE    (32)
#define PET_TIME_SIZE           (36)

#define BANNER_SIZE             (192)
#define PRODUCT_SIZE            (16)
#define SOFT_VERSION_SIZE       (32)
#define BASEBAND_VERSION_SIZE   (48)
#define BUILD_DATE_SIZE         (32)

#define MINFO_SIZE              (10)

typedef struct {
    unsigned char   linux_ver[VERSION_SIZE];
    unsigned char   modem_ver[VERSION_SIZE];
    unsigned char   model[MODEL_SIZE];
    uint32_t        reserved[40];
} kcy_info_type ;

typedef struct {
    unsigned char   linux_banner[BANNER_SIZE];
    unsigned char   product_number[PRODUCT_SIZE];
    unsigned char   software_version[SOFT_VERSION_SIZE];
    unsigned char   baseband_version[BASEBAND_VERSION_SIZE];
    unsigned char   android_build_date[BUILD_DATE_SIZE];
    unsigned char   modem_build_date[BUILD_DATE_SIZE];
} kcd_info_type ;

typedef union{
    kcy_info_type   kcy;
    kcd_info_type   kcd;
} kcj_info_type ;

typedef struct {
    unsigned char   magic_code[MAGIC_CODE_SIZE];
    unsigned char   crash_system[CRASH_SYSTEM_SIZE];
    unsigned char   crash_kind[CRASH_KIND_SIZE];
    unsigned char   crash_time[CRASH_TIME_SIZE];
    uint32_t        ver_format;
    uint32_t        boot_mode;
    uint32_t        padding0[2];
    kcj_info_type   kcj_info;
    unsigned char   crash_info_data[CRASH_INFO_DATA_SIZE];
    unsigned char   panic_info_data[CRASH_INFO_DATA_SIZE];
    logger_log_info info[LOGGER_INFO_MAX];
    uint32_t        regsave_addr;
    uint32_t        regsave_addr_check;
    uint32_t        m_info[MINFO_SIZE];
    uint32_t        log_buf_addr;
    uint32_t        log_buf_len_addr;
    uint32_t        log_first_idx_addr;
    uint32_t        log_next_idx_addr;
    uint32_t        reserved[96];
} ram_log_info_type;

#endif /* _RESET_LOG_H */

