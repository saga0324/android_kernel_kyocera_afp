/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
                              <resetlog.h>
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
#ifndef _RESET_LOG_H
#define _RESET_LOG_H

#define HEADER_VERION           "v3.0.0"
#define HEADER_VERION_SIZE      (8)

#ifdef  __KERNEL__      /* linux kernel */
#define MSM_KCJLOG_BASE         (MSM_UNINIT_RAM_BASE)

#else /*__KERNEL__*/    /* boot system */
#ifdef  SCL_UNINIT_RAM_BASE
#define MSM_KCJLOG_BASE         (SCL_UNINIT_RAM_BASE)
#endif/*UNINIT_RAM_BASE*/
#endif/*__KERNEL__*/


#ifdef  MSM_KCJLOG_BASE
#define SIZE_SMEM_ALLOC         (512)
#define SIZE_CONTROL_INFO       (1024)
#define SIZE_KERNEL_LOG         (512 * 1024)
#define SIZE_LOGCAT_MAIN        (512 * 1024)
#define SIZE_LOGCAT_SYSTEM      (512 * 1024)
#define SIZE_LOGCAT_EVENTS      (256 * 1024)
#define SIZE_LOGCAT_RADIO       (512 * 1024)
#define SIZE_SMEM_EVENT_LOG     ( 50 * 1024)
#define SIZE_MODEM_F3_LOG       (512 * 1024)
#define SIZE_MODEM_ERR_DATA_LOG ( 16 * 1024)
#define SIZE_MODEM_EXT_DATA_LOG ( 64 * 1024)

#define SIZE_OCIMEM             (24576)
#define SIZE_RPM_CODE           (131072)
#define SIZE_RPM_DATA           (65536)
#define SIZE_RPM_MSG            (16384)
#define SIZE_LPM                (65536)
#define SIZE_RESET_STATUS       (16)
#define SIZE_CPU_CONTEXT_INFO   (  1 * 1024)

#define SIZE_KERNEL_BOOT_LOG    (64 * 1024)
#define SIZE_SBL1_BOOT_LOG      (16 + 64 + (4 * 1024))
#define SIZE_APPSBL_BOOT_LOG    (16 +      (8 * 1024))

#define ADDR_KCJ_RAM_CONSOLE    (0xFBD00000 + SIZE_CONTROL_INFO)

#define ADDR_CONTROL_INFO       (MSM_KCJLOG_BASE)
#define ADDR_KERNEL_LOG         (ADDR_CONTROL_INFO       + SIZE_CONTROL_INFO      )
#define ADDR_LOGCAT_MAIN        (ADDR_KERNEL_LOG         + SIZE_KERNEL_LOG        )
#define ADDR_LOGCAT_SYSTEM      (ADDR_LOGCAT_MAIN        + SIZE_LOGCAT_MAIN       )
#define ADDR_LOGCAT_EVENTS      (ADDR_LOGCAT_SYSTEM      + SIZE_LOGCAT_SYSTEM     )
#define ADDR_LOGCAT_RADIO       (ADDR_LOGCAT_EVENTS      + SIZE_LOGCAT_EVENTS     )
#define ADDR_SMEM_EVENT_LOG     (ADDR_LOGCAT_RADIO       + SIZE_LOGCAT_RADIO      )
#define ADDR_MODEM_F3_LOG       (ADDR_SMEM_EVENT_LOG     + SIZE_SMEM_EVENT_LOG    )
#define ADDR_MODEM_ERR_DATA_LOG (ADDR_MODEM_F3_LOG       + SIZE_MODEM_F3_LOG      )
#define ADDR_MODEM_EXT_DATA_LOG (ADDR_MODEM_ERR_DATA_LOG + SIZE_MODEM_ERR_DATA_LOG)
#define ADDR_CPU_CONTEXT_INFO   (ADDR_MODEM_EXT_DATA_LOG + SIZE_MODEM_EXT_DATA_LOG)

#define ADDR_USB_DUMP_START     (ADDR_CPU_CONTEXT_INFO   + SIZE_CPU_CONTEXT_INFO  )
#define ADDR_OCIMEM             (ADDR_USB_DUMP_START)
#define ADDR_RPM_CODE           (ADDR_OCIMEM             + SIZE_OCIMEM            )
#define ADDR_RPM_DATA           (ADDR_RPM_CODE           + SIZE_RPM_CODE          )
#define ADDR_RPM_MSG            (ADDR_RPM_DATA           + SIZE_RPM_DATA          )
#define ADDR_LPM                (ADDR_RPM_MSG            + SIZE_RPM_MSG           )
#define ADDR_RESET_STATUS       (ADDR_LPM                + SIZE_LPM               )
#define ADDR_USB_DUMP_END       (ADDR_RESET_STATUS       + SIZE_RESET_STATUS      )

#define ADDR_KERNEL_BOOT_LOG    (ADDR_USB_DUMP_END)
#define ADDR_SBL1_BOOT_LOG      (ADDR_KERNEL_BOOT_LOG    + SIZE_KERNEL_BOOT_LOG   )
#define ADDR_APPSBL_BOOT_LOG    (ADDR_SBL1_BOOT_LOG      + SIZE_SBL1_BOOT_LOG     )

#endif/*MSM_KCJLOG_BASE*/

enum {
    LOGGER_INFO_MAIN,
    LOGGER_INFO_SYSTEM,
    LOGGER_INFO_EVENTS,
    LOGGER_INFO_RADIO,
    LOGGER_INFO_MAX,
};

#define CRASH_MAGIC_CODE         "KC ERROR"

#define CRASH_SYSTEM_KERNEL      "KERNEL"
#define CRASH_SYSTEM_MODEM       "MODEM"
#define CRASH_SYSTEM_PRONTO      "PRONTO"
#define CRASH_SYSTEM_ADSP        "ADSP"
#define CRASH_SYSTEM_VENUS       "VENUS"
#define CRASH_SYSTEM_ANDROID     "ANDROID"
#define CRASH_SYSTEM_UNKNOWN     "UNKNOWN"

#define CRASH_KIND_PANIC         "KERNEL PANIC"
#define CRASH_KIND_FATAL         "ERR FATAL"
#define CRASH_KIND_EXCEPTION     "EXCEPTION"
#define CRASH_KIND_WDOG_HW       "HW WATCH DOG"
#define CRASH_KIND_WDOG_SW       "SW WATCH DOG"
#define CRASH_KIND_SYS_SERVER    "SYSTEM SERVER CRASH"
#define CRASH_KIND_PWR_KEY       "PWR KEY"
#define CRASH_KIND_KDFS_REBOOT   "KDFS REBOOT"
#define CRASH_KIND_UNKNOWN       "UNKNOWN"


/* RAM_CONSOLE Contol */
typedef struct {
    unsigned long               sig;
    unsigned long               start;
    unsigned long               size;
} ram_console_header_type;

typedef struct {
    ram_console_header_type     header;
    unsigned char               msg[1];
} ram_console_type;

/* Log Control */
typedef struct {
    unsigned long           w_off;
    unsigned long           head;
} logger_log_info;

#define MAGIC_CODE_SIZE         (16)
#define CRASH_SYSTEM_SIZE       (16)
#define CRASH_KIND_SIZE         (32)
#define CRASH_TIME_SIZE         (48)

#define VERSION_SIZE            (64)
#define MODEL_SIZE              (32)
#define CRASH_INFO_DATA_SIZE    (32)
#define PET_TIME_SIZE           (36)

#define BANNER_SIZE             (160)
#define PRODUCT_SIZE            (16)
#define SOFT_VERSION_SIZE       (16)
#define BASEBAND_VERSION_SIZE   (16)
#define BUILD_DATE_SIZE         (32)

typedef struct {
    unsigned char   linux_ver[VERSION_SIZE];
    unsigned char   modem_ver[VERSION_SIZE];
    unsigned char   model[MODEL_SIZE];
    unsigned long   reserved[28];
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
    unsigned long   ver_format;
    unsigned long   padding0[3];
    kcj_info_type   kcj_info;
    unsigned char   crash_info_data[CRASH_INFO_DATA_SIZE];
    unsigned char   panic_info_data[CRASH_INFO_DATA_SIZE];
    logger_log_info info[LOGGER_INFO_MAX];
    unsigned long   reserved0[4];
    unsigned long   regsave_addr;
    unsigned long   regsave_addr_check;
    unsigned long   padding1[2];
    unsigned long   m_info[11];
    unsigned long   ocimem_size;
    unsigned long   rpm_code_size;
    unsigned long   rpm_data_size;
    unsigned long   rpm_msg_size;
    unsigned long   lpm_size;
    unsigned long   reset_status_size;
    unsigned long   pmic_pon_status_size;
    unsigned long   padding2;
    unsigned long   reset_info_addr;
    unsigned long   kernel_log_magic;
    unsigned long   kernel_log_start;
    unsigned long   kernel_log_size;
    unsigned long   kernel_log_reserved;
    unsigned long   reserved[100];
} ram_log_info_type;

#endif /* _RESET_LOG_H */

