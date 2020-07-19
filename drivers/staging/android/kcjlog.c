/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
                              <kcjlog.c>
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

*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
#define pr_fmt(fmt) "KCJLOG: " fmt
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
#include <linux/printk.h>
#include <linux/proc_fs.h>
#include <linux/kc_phymap.h>
#include <linux/kcjlog.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/vmalloc.h>
#include <soc/qcom/smsm.h>
#include "resetlog.h"

/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
 * CONSTATNT / DEFINE DECLARATIONS
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
static const char crash_system[SYSTEM_MAX][CRASH_SYSTEM_SIZE] = {
	CRASH_SYSTEM_KERNEL,
	CRASH_SYSTEM_MODEM,
	CRASH_SYSTEM_PRONTO,
	CRASH_SYSTEM_ADSP,
	CRASH_SYSTEM_VENUS,
	CRASH_SYSTEM_ANDROID,
	CRASH_SYSTEM_DM_VERITY,
	CRASH_SYSTEM_UNKNOWN
};

static const char crash_kind[KIND_MAX][CRASH_KIND_SIZE] = {
	CRASH_KIND_PANIC,
	CRASH_KIND_FATAL,
	CRASH_KIND_EXCEPTION,
	CRASH_KIND_WDOG_HW,
	CRASH_KIND_WDOG_SW,
	CRASH_KIND_SYS_SERVER,
	CRASH_KIND_PWR_KEY,
	CRASH_KIND_KDFS_REBOOT,
	CRASH_KIND_DEVICE_CORRUPTED,
	CRASH_KIND_UNKNOWN
};

#define EMMC_INDEX_MAX              (4)
#define EMMC_SECTOR_BLK_SIZE        (512)   /* eMMC Sector Size */

#define SIZE_TO_SECTOR(size)        (((size % EMMC_SECTOR_BLK_SIZE) > 0) ? \
                                     ( size / EMMC_SECTOR_BLK_SIZE) + 1  : \
                                     ( size / EMMC_SECTOR_BLK_SIZE))

#define EMMC_INFO_SECTOR_SIZE       (SIZE_TO_SECTOR(sizeof(emmc_info_type)))
#define EMMC_INFO_START_SECTOR      (0)

#define EMMC_REC_SIZE               (0x440000)
#define EMMC_REC_SECTOR_SIZE        (SIZE_TO_SECTOR(EMMC_REC_SIZE))

#define EMMC_REC_START_SECTOR(idx)  ((EMMC_INFO_START_SECTOR + EMMC_INFO_SECTOR_SIZE) + \
                                     (EMMC_REC_SECTOR_SIZE * idx))

#define EMMC_DEV_ERROR              (-1)
#define EMMC_NO_ERROR               (0)
#define EMMC_DEV_INVALID            (-1)

#define EMMC_PAGE_SIZE              (4*1024)
#define RESETLOG_MAX_WAITCOUNT      (10)

#ifdef ARRAY_SIZEOF
#undef ARRAY_SIZEOF
#endif

#define LOGCAT_NAME_MAIN            "logcat_main"       /* everything else           */
#define LOGCAT_NAME_SYSTEM          "logcat_system"     /* system/framework messages */
#define LOGCAT_NAME_EVENTS          "logcat_events"     /* system/hardware events    */
#define LOGCAT_NAME_RADIO           "logcat_radio"      /* radio-related messages    */

#define DUMP_RESET_FLAG_RAM_SIZE    (4)
#define RESETLOG_UNINIT_RAM_SIZE    (MSM_UNINIT_RAM_SIZE - DUMP_RESET_FLAG_RAM_SIZE)

static dev_t log_dev_t = EMMC_DEV_INVALID;
static dev_t log_parent_dev = MKDEV(MMC_BLOCK_MAJOR, 0);
const static char log_partition_label[] = "log";

/* ResetLog control info */
typedef struct {
	unsigned char          crash_system[CRASH_SYSTEM_SIZE];
	unsigned char          crash_kind[CRASH_KIND_SIZE];
	unsigned char          crash_time[CRASH_TIME_SIZE];
	uint32_t               ver_format;
	uint32_t               boot_mode;
	uint32_t               padding0[2];
	kcj_info_type          kcj_info;
	unsigned char          crash_info_data[CRASH_INFO_DATA_SIZE];
	unsigned char          panic_info_data[CRASH_INFO_DATA_SIZE];
	logger_log_info        info[LOGGER_INFO_MAX];
	uint32_t               reserved0[4];
	uint32_t               m_info[24];
	uint32_t               kernel_log_magic;
	uint32_t               kernel_log_buf_len;
	uint32_t               kernel_log_first_idx;
	uint32_t               kernel_log_next_idx;
	uint32_t               reserved1[60];
} emmc_rec_info_type;

typedef struct {
	unsigned char          logging_ver[HEADER_VERSION_SIZE]; /* resetlog.h version */
	uint32_t               write_index;                      /* Index of write (0 - (EMMC_INDEX_MAX - 1)) */
	unsigned char          valid_flg[EMMC_INDEX_MAX];        /* valid(1) or invalid(0) */
	logger_log_info        fotalog[LOGGER_INFO_MAX];         /* Fotalog Info */
	uint32_t               settings[40];                     /* eMMc settings */
	uint32_t               reserved[76];                     /* customize area */
	emmc_rec_info_type     rec_info[EMMC_INDEX_MAX];         /* Record Info */
} emmc_info_type;

typedef struct {
    void __iomem                *addr_control_info;
    void __iomem                *addr_logcat_main;
    void __iomem                *addr_logcat_system;
    void __iomem                *addr_logcat_events;
    void __iomem                *addr_logcat_radio;
} addr_offset_type;

/* logcat save */
typedef struct
{
	char            *log_name;
	unsigned char   *buf_addr;
	unsigned int    size;
} logcat_set_info;

typedef struct
{
	unsigned char       *buffer;
	struct miscdevice   misc;
	size_t              size;
	size_t              w_off;
	logger_log_info     *log_info;
} logcat_regist_info;

static uint32_t bark_regaddr = 0;

static addr_offset_type addr_info;
static void __iomem *uninit_ram_virt_addr = NULL;
static void __iomem *kcj_get_uninit_ram_base_addr( void );

/* logcat save */
static logcat_set_info set_info[ LOGGER_INFO_MAX ] =
{
	{ LOGCAT_NAME_MAIN  , NULL, SIZE_LOGCAT_MAIN   },
	{ LOGCAT_NAME_SYSTEM, NULL, SIZE_LOGCAT_SYSTEM },
	{ LOGCAT_NAME_EVENTS, NULL, SIZE_LOGCAT_EVENTS },
	{ LOGCAT_NAME_RADIO , NULL, SIZE_LOGCAT_RADIO  }
};

static logcat_regist_info *regist_info[ LOGGER_INFO_MAX ] = { NULL };

static int build_date_changed_handler( const char *kmessage, struct kernel_param *kp );
static int build_date_get_handler( char *buffer, const struct kernel_param *kp );

static struct kparam_string build_date = {
       .string = ((ram_log_info_type *)ADDR_CONTROL_INFO)->kcj_info.kcd.android_build_date,
       .maxlen = BUILD_DATE_SIZE,
};

module_param_call(build_date, build_date_changed_handler, build_date_get_handler, &build_date,
                  S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);

static uint32_t boot_mode;

/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
 * PROTOTYPE DECLARATION
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
extern void rtc_time_to_tm(unsigned long time, struct rtc_time *tm);

/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
 * FUNCTION DEFINITIONS
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
/*
 * check_smem_kcjlog()
 *
 * Note: Check kcjlog from smem.
 */
static unsigned char check_smem_kcjlog( ram_log_info_type *plog_info )
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
 * set_smem_kcjlog()
 *
 * Note: Set kcjlog to smem.
 */
void set_smem_kcjlog(crash_system_type system, crash_kind_type kind)
{
	ram_log_info_type *plog_info;

	mb();
	plog_info = (ram_log_info_type *)kc_smem_alloc(SMEM_CRASH_LOG, SIZE_SMEM_ALLOC);

	if ((plog_info == NULL) || (system >= SYSTEM_MAX) || (kind >= KIND_MAX)) {
		return;
	}
	if (check_smem_kcjlog(plog_info) == 1) {
		return;
	}
	memcpy( &plog_info->crash_system[0], crash_system[system], strlen(crash_system[system]) );
	memcpy( &plog_info->crash_kind[0],   crash_kind[kind],     strlen(crash_kind[kind])     );
	mb();
}

/*
 * clear_smem_kcjlog()
 *
 * Note: Clear kcjlog from smem.
 */
static void clear_smem_kcjlog(void)
{
	ram_log_info_type *plog_info;

	mb();
	plog_info = (ram_log_info_type *)kc_smem_alloc(SMEM_CRASH_LOG, SIZE_SMEM_ALLOC);

	if (plog_info == NULL) {
		return;
	}
	memset( &plog_info->crash_system[0], 0x00, CRASH_SYSTEM_SIZE );
	memset( &plog_info->crash_kind[0],   0x00, CRASH_KIND_SIZE   );
	mb();
}

/*
 * clear_smem_crash_info_data()
 *
 * Note: Clear qalog info data from smem.
 */
static void clear_smem_crash_info_data( void )
{
	ram_log_info_type *plog_info;

	mb();
	plog_info = (ram_log_info_type *)kc_smem_alloc( SMEM_CRASH_LOG, SIZE_SMEM_ALLOC );

	if ( plog_info == NULL ) {
		return;
	}
	memset( plog_info->crash_info_data, 0x00, sizeof(plog_info->crash_info_data) );
	mb();
}

/*
 * clear_smem_panic_info_data()
 *
 * Note: Clear qalog panic info data from smem.
 */
static void clear_smem_panic_info_data( void )
{
	ram_log_info_type *plog_info;

	mb();
	plog_info = (ram_log_info_type *)kc_smem_alloc( SMEM_CRASH_LOG, SIZE_SMEM_ALLOC );

	if ( plog_info == NULL ) {
		return;
	}
	memset( plog_info->panic_info_data, 0x00, sizeof(plog_info->panic_info_data) );
	mb();
}

/*
 * get_crash_time()
 *
 * Note: Get utc time.
 */
static void get_crash_time(unsigned char *buf, size_t bufsize)
{
	struct timespec ts_now;
	struct rtc_time tm;
	char   tmp_buf[CRASH_TIME_SIZE] = {};
	size_t copy_size;

	ts_now = current_kernel_time();
	rtc_time_to_tm(ts_now.tv_sec, &tm);

	snprintf( tmp_buf, sizeof(tmp_buf), "%d-%02d-%02d %02d:%02d:%02d.%09lu UTC",
				tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
				tm.tm_hour, tm.tm_min, tm.tm_sec, ts_now.tv_nsec);

	copy_size = min(bufsize, (size_t)CRASH_TIME_SIZE);
	memcpy_toio( buf, tmp_buf, copy_size );
}

/*
 * set_kcj_regsave_addr()
 *
 * Note: Set regsave addr to uninit ram.
 */
void set_kcj_regsave_addr(unsigned int regsave_addr)
{
	ram_log_info_type *pPanicInfo;

	pPanicInfo   = (ram_log_info_type *)addr_info.addr_control_info;
	pPanicInfo->regsave_addr = regsave_addr;
	pPanicInfo->regsave_addr_check = ~regsave_addr;
	bark_regaddr = regsave_addr;
}

/*
 * set_kcj_fixed_info()
 *
 * Note: Set kcj fixed info to uninit ram.
 */
static void set_kcj_fixed_info(void)
{
	int               banner_len;
	ram_log_info_type *pPanicInfo;
	ram_log_info_type *pSmemLogInfo;
	char*	log_buf_addr       = log_buf_addr_get();
	u32*	log_buf_len_addr   = log_buf_len_addr_get();
	u32*	log_first_idx_addr = log_first_idx_addr_get();
	u32*	log_next_idx_addr  = log_next_idx_addr_get();

	pPanicInfo   = (ram_log_info_type *)addr_info.addr_control_info;
	pSmemLogInfo = (ram_log_info_type *)kc_smem_alloc(SMEM_CRASH_LOG, SIZE_SMEM_ALLOC);

	if ( (pPanicInfo == NULL) || (pSmemLogInfo == NULL) ) {
		return;
	}

	memcpy_toio( &pPanicInfo->magic_code[0]  , CRASH_MAGIC_CODE   , strlen(CRASH_MAGIC_CODE  ) );

	pPanicInfo->ver_format = 1;
	pSmemLogInfo->ver_format = 1;

	/* set linux_banner to uninit ram & smem crash log */
	banner_len = strlen( linux_banner );
	if('\n' == linux_banner[banner_len - 1]) banner_len = banner_len - 1;
	if ( banner_len > BANNER_SIZE ) banner_len = BANNER_SIZE;
	memcpy_toio( &pPanicInfo->kcj_info.kcd.linux_banner[0], &linux_banner[0], banner_len );
	memcpy( &pSmemLogInfo->kcj_info.kcd.linux_banner[0], &linux_banner[0], banner_len );
	memcpy_toio( &pPanicInfo->kcj_info.kcd.software_version[0] , BUILD_DISPLAY_ID   , strlen(BUILD_DISPLAY_ID));
	memcpy_toio( &pPanicInfo->kcj_info.kcd.product_number[0]   , PRODUCT_MODEL_NAME , strlen(PRODUCT_MODEL_NAME) );

	pPanicInfo->log_buf_addr       = (uint32_t)virt_to_phys((void *)log_buf_addr);
	pPanicInfo->log_buf_len_addr   = (uint32_t)virt_to_phys((void *)log_buf_len_addr);
	pPanicInfo->log_first_idx_addr = (uint32_t)virt_to_phys((void *)log_first_idx_addr);
	pPanicInfo->log_next_idx_addr  = (uint32_t)virt_to_phys((void *)log_next_idx_addr);

	boot_mode = pPanicInfo->boot_mode;

}

/*
* set_kcj_fixed_info_modem()
 *
 * Note: Set kcj modem fixed info to uninit ram.
 */
void set_kcj_fixed_info_modem(void)
{
    ram_log_info_type *pPanicInfo;
    ram_log_info_type *pSmemLogInfo;

    mb();
    pPanicInfo   = (ram_log_info_type *)addr_info.addr_control_info;
    pSmemLogInfo = (ram_log_info_type *)kc_smem_alloc(SMEM_CRASH_LOG, SIZE_SMEM_ALLOC);

    if ( (pPanicInfo == NULL) || (pSmemLogInfo == NULL) ) {
            return;
    }

    /* set modem info on smem to uninit ram */
    memcpy_toio( &pPanicInfo->kcj_info.kcd.baseband_version[0], &pSmemLogInfo->kcj_info.kcd.baseband_version[0], BASEBAND_VERSION_SIZE );
    memcpy_toio( &pPanicInfo->kcj_info.kcd.modem_build_date[0], &pSmemLogInfo->kcj_info.kcd.modem_build_date[0], BUILD_DATE_SIZE );
}

/*
 * set_kcj_crash_info()
 *
 * Note: Set crash info to uninit ram.
 */
void set_kcj_crash_info(void)
{
	ram_log_info_type *pPanicInfo;
	ram_log_info_type *pSmemLogInfo;

	mb();
	pPanicInfo   = (ram_log_info_type *)addr_info.addr_control_info;
	pSmemLogInfo = (ram_log_info_type *)kc_smem_alloc(SMEM_CRASH_LOG, SIZE_SMEM_ALLOC);

	if ( (pPanicInfo == NULL) || (pSmemLogInfo == NULL) ) {
		return;
	}
	if (check_smem_kcjlog(pSmemLogInfo) == 0) {
		clear_kcj_crash_info();
		return;
	}
	set_kcj_fixed_info();
	set_modemlog_info();

	memset_io( &pPanicInfo->crash_system[0], 0x00, CRASH_SYSTEM_SIZE );
	memset_io( &pPanicInfo->crash_kind[0]  , 0x00, CRASH_KIND_SIZE );

	memcpy_toio( &pPanicInfo->crash_system[0], &pSmemLogInfo->crash_system[0], CRASH_SYSTEM_SIZE );
	memcpy_toio( &pPanicInfo->crash_kind[0]  , &pSmemLogInfo->crash_kind[0]  , CRASH_KIND_SIZE );

	get_crash_time( &pPanicInfo->crash_time[0], CRASH_TIME_SIZE );
}

/*
 * set_kcj_unknown_info()
 *
 * Note: Set crash unknown info to uninit ram.
 */
static void set_kcj_unknown_info(void)
{
	ram_log_info_type *pPanicInfo;

	pPanicInfo = (ram_log_info_type *)addr_info.addr_control_info;

	if ( pPanicInfo == NULL ) {
		return;
	}

	memcpy_toio( &pPanicInfo->crash_system[0], CRASH_SYSTEM_UNKNOWN, strlen(CRASH_SYSTEM_UNKNOWN) );
	memcpy_toio( &pPanicInfo->crash_kind[0]  , CRASH_KIND_UNKNOWN  , strlen(CRASH_KIND_UNKNOWN  ) );
}

/*
 * clear_kcj_crash_info()
 *
 * Note: Clear crash info from uninit ram.
 */
void clear_kcj_crash_info( void )
{
	ram_log_info_type *pPanicInfo;

	pPanicInfo = (ram_log_info_type *)addr_info.addr_control_info;

	if ( pPanicInfo == NULL ) {
		return;
	}

	memset_io( &pPanicInfo->crash_system[0], 0x00, CRASH_SYSTEM_SIZE );
	memset_io( &pPanicInfo->crash_kind[0]  , 0x00, CRASH_KIND_SIZE );
	memset_io( &pPanicInfo->crash_time[0]  , 0x00, CRASH_TIME_SIZE );
}

/*
 * set_modemlog_info()
 *
 * Note: Set modem log info from smem to uninit ram.
 */
void set_modemlog_info(void)
{
	int i;
	ram_log_info_type *pSmemLogInfo;
	ram_log_info_type *pPanicInfo;

	mb();
	pSmemLogInfo = (ram_log_info_type *)kc_smem_alloc(SMEM_CRASH_LOG, SIZE_SMEM_ALLOC);
	pPanicInfo   = (ram_log_info_type *)addr_info.addr_control_info;

	if ( (pSmemLogInfo == NULL) || (pPanicInfo == NULL) ) {
		return;
	}

	for( i = 0 ; i < MINFO_SIZE ; i++ )
	{
		pPanicInfo->m_info[i]  = pSmemLogInfo->m_info[i];
	}

    memcpy_toio( &pPanicInfo->kcj_info.kcd.baseband_version[0], &pSmemLogInfo->kcj_info.kcd.baseband_version[0], BASEBAND_VERSION_SIZE );
    memcpy_toio( &pPanicInfo->kcj_info.kcd.modem_build_date[0], &pSmemLogInfo->kcj_info.kcd.modem_build_date[0], BUILD_DATE_SIZE       );
}

/*
 * resetlog_emmc_end_bio()
 *
 * Note: Resetlog emmc end bio.
 */
static void resetlog_emmc_end_bio(struct bio *bio, int err)
{
	if (IS_ERR(&err)) {
		pr_err("bio error occured\n");
		return;
	}
	if (bio->bi_private) {
		complete((struct completion*)(bio->bi_private));
	}
	bio_put(bio);
}

/*
 * resetlog_emmc_submit()
 *
 * Note: Resetlog emmc submit.
 */
static int resetlog_emmc_submit(struct block_device *log_bdev, int rw,
				uint32_t sector, uint32_t size,
				struct page *log_page)
{
	int ret = 0;
	struct bio *log_bio;
	DECLARE_COMPLETION_ONSTACK(waithandle);

	log_bio = bio_alloc(GFP_KERNEL, 1);

	if (IS_ERR(log_bio)) {
			pr_err("Could not alloc bio\n");
			return -ENOMEM;
	}

	/* setup bio params */
	log_bio->bi_iter.bi_sector = sector;
	log_bio->bi_bdev = log_bdev;

	if (bio_add_page(log_bio, log_page, size, 0) != size) {
			pr_err("Could not add page\n");
			ret = -ENOMEM;
			goto out;
	}

	log_bio->bi_vcnt = 1;
	log_bio->bi_iter.bi_idx = 0;
	log_bio->bi_iter.bi_size = size;
	log_bio->bi_end_io = resetlog_emmc_end_bio;
	log_bio->bi_rw = rw;
	log_bio->bi_private = &waithandle;

	if (!(rw & REQ_WRITE)) {
		bio_set_pages_dirty(log_bio);
	}
	bio_get(log_bio);
	submit_bio(rw, log_bio);
	wait_for_completion(&waithandle);

out:
	bio_put(log_bio);
	return ret;
}

/*
 * resetlog_emmc_drv_read()
 *
 * Note: Resetlog emmc drv read.
 */
static int32_t resetlog_emmc_drv_read(uint32_t sector, uint32_t num_sector, uint8_t *pbuf)
{
	int ret = EMMC_NO_ERROR;
	struct block_device *log_bdev;
	struct page *log_page;
	int total;

	if (log_dev_t == EMMC_DEV_INVALID) {
		pr_err("resetlog_emmc_drv not initialized\n");
		return EMMC_DEV_ERROR;
	}

	/* open block_device of log */
	log_bdev = blkdev_get_by_dev(log_dev_t, FMODE_READ, NULL);
	if (IS_ERR(log_bdev)) {
		pr_err("Could not get log_bdev\n");
		return EMMC_DEV_ERROR;
	}

	total = (num_sector * EMMC_SECTOR_BLK_SIZE);
	log_page = alloc_pages(GFP_KERNEL, get_order(total));
	if (!log_page) {
		pr_err("Could not alloc pages total=%d\n", total);
		ret = EMMC_DEV_ERROR;
		goto out_put;
	}

	ret = resetlog_emmc_submit(log_bdev, 0,
						sector, total, log_page);
	if (ret) {
		pr_err("Could not sumbmit log io @sector=%d, @num_sector=%d\n",
				sector, num_sector);
		ret = EMMC_DEV_ERROR;
		goto out_free;
	}
	ret = EMMC_NO_ERROR;
	memcpy(pbuf, page_address(log_page), total);
out_free:
	__free_pages(log_page, get_order(total));
out_put:
	blkdev_put(log_bdev, FMODE_READ);

	return ret;
}

/*
 * resetlog_emmc_drv_write()
 *
 * Note: Resetlog emmc drv write.
 */
static int32_t resetlog_emmc_drv_write(uint32_t sector, uint32_t num_sector, uint8_t *pbuf)
{
	int ret;
	struct block_device *log_bdev;
	struct page *log_page;
	int total;

	if (log_dev_t == EMMC_DEV_INVALID) {
		pr_err("resetlog_emmc_drv not initialized\n");
		return EMMC_DEV_ERROR;
	}

	/* open block_device of log */
	log_bdev = blkdev_get_by_dev(log_dev_t, FMODE_WRITE, NULL);
	if (IS_ERR(log_bdev)) {
		pr_err("Could not get log_bdev\n");
		return EMMC_DEV_ERROR;
	}

	/* setup page */
	total = (num_sector * EMMC_SECTOR_BLK_SIZE);
	log_page = alloc_pages(GFP_KERNEL, get_order(total));
	if (!log_page) {
		pr_err("Could not alloc pages total=%d\n", total);
		ret = EMMC_DEV_ERROR;
		goto out_put;
	}
	memcpy_fromio(page_address(log_page), pbuf, total);

	ret = resetlog_emmc_submit(log_bdev, (REQ_WRITE | REQ_FLUSH),
				sector, total, log_page);
	if (ret) {
		pr_err("Could not sumbmit log io @sector=%d, @num_sector=%d\n",
				sector, num_sector);
		ret = EMMC_DEV_ERROR;
		goto out_free;
	}
	ret = EMMC_NO_ERROR;

out_free:
	__free_pages(log_page, get_order(total));
out_put:
	blkdev_put(log_bdev, FMODE_WRITE);
	return ret;
}

/*
 * resetlog_emmc_drv_init()
 *
 * Note: Resetlog emmc drv init.
 */
static int __init resetlog_emmc_drv_init(void)
{
	int ret = EMMC_DEV_ERROR;
	struct block_device *bdev;
	struct disk_part_iter piter;
	struct hd_struct *part;
	int count = 0;

	do {
		/* Although we configured log as late_initcall(),
		 * block device would not appear in some situation.
		 * We wait for it a bit here.
		 */
		bdev = blkdev_get_by_dev(log_parent_dev, FMODE_READ, NULL);
		if (!IS_ERR(bdev))
			break;
		if (count < RESETLOG_MAX_WAITCOUNT) {
			pr_warn("waiting for bdev initialized... count=%d\n", count);
			count++;
			mdelay(10);
		} else {
			/* give up */
			pr_err("Could not get bdev\n");
			ret = EMMC_DEV_ERROR;
			return ret;
		}
	} while (1);

	/* search partition for log */
	disk_part_iter_init(&piter, bdev->bd_disk, DISK_PITER_INCL_EMPTY);
	while ((part = disk_part_iter_next(&piter))) {
		if (0 == strcmp(log_partition_label,
						part->info->volname)) {
			log_dev_t = part_devt(part);
			pr_info("found log partition, partno=%d, major=%d, minor=%d, start_sect=%llu, nr_sects=%llu\n",
				part->partno, MAJOR(log_dev_t), MINOR(log_dev_t), part->start_sect, part->nr_sects);
			ret = EMMC_NO_ERROR;
			break;
		}
	}
	disk_part_iter_exit(&piter);
	blkdev_put(bdev, FMODE_READ);

	return ret;
}

/*
 * resetlog_flash_read_sub()
 *
 * Note: Resetlog flash read sub.
 */
static uint32_t resetlog_flash_read_sub( uint32_t *s_sect, unsigned char *rd_buff, uint32_t rd_size )
{
	uint32_t num_sect;

	num_sect = (rd_size / EMMC_SECTOR_BLK_SIZE);
	if (num_sect == 0) {
		return 0;
	}

	if (resetlog_emmc_drv_read(*s_sect, num_sect, rd_buff) < 0) {
		return 0;
	}
	*s_sect += num_sect;

	return (num_sect * EMMC_SECTOR_BLK_SIZE);
}

/*
 * resetlog_flash_read()
 *
 * Note: Resetlog flash read.
 */
static int resetlog_flash_read( uint32_t *s_sect, unsigned char *rd_buff, uint32_t rd_size )
{
	uint32_t		i;
	uint32_t		ret_size;
	uint32_t		tmp_size;
	unsigned char	tmp_buff[EMMC_SECTOR_BLK_SIZE];

	if (rd_size >= EMMC_SECTOR_BLK_SIZE) {
		ret_size = resetlog_flash_read_sub(s_sect, rd_buff, rd_size);
		if (ret_size == 0) {
			return -1;
		}
	} else {
		if (resetlog_flash_read_sub(s_sect, tmp_buff, EMMC_SECTOR_BLK_SIZE) == 0) {
			return -1;
		}
		for (i = 0; i < rd_size; i++) {
			rd_buff[i] = tmp_buff[i];
		}
		goto READ_END;
	}

	if (ret_size < rd_size) {
		tmp_size = (rd_size - ret_size);
		if (resetlog_flash_read_sub(s_sect, tmp_buff, EMMC_SECTOR_BLK_SIZE) == 0) {
			return -1;
		}
		for (i = 0; i < tmp_size; i++) {
			rd_buff[ret_size + i] = tmp_buff[i];
		}
	}
READ_END:
	return 0;
}

/*
 * resetlog_flash_write_sub()
 *
 * Note: Resetlog flash write sub.
 */
static uint32_t resetlog_flash_write_sub( uint32_t *s_sect, unsigned char *wr_buff, uint32_t wr_size )
{
	uint32_t num_sect;

	num_sect = (wr_size / EMMC_SECTOR_BLK_SIZE);
	if (num_sect == 0) {
		return 0;
	}
	if (resetlog_emmc_drv_write(*s_sect, num_sect, wr_buff) < 0) {
		return 0;
	}
	*s_sect += num_sect;

	return (num_sect * EMMC_SECTOR_BLK_SIZE);
}

/*
 * resetlog_flash_write()
 *
 * Note: Resetlog flash write.
 */
static int resetlog_flash_write( uint32_t *s_sect, unsigned char *wr_buff, uint32_t wr_size )
{
	uint32_t		i;
	uint32_t		ret_size;
	uint32_t		tmp_size;
	uint32_t		w_cnt;
	unsigned char	tmp_buff[EMMC_SECTOR_BLK_SIZE];

	for (i = 0; i < EMMC_SECTOR_BLK_SIZE; i++) {
		tmp_buff[i] = 0;
	}

	if (wr_size >= EMMC_SECTOR_BLK_SIZE) {
		ret_size = 0;
		w_cnt = wr_size/EMMC_PAGE_SIZE;

		for(i = 0; i < w_cnt; i++)
		{
			tmp_size = resetlog_flash_write_sub(s_sect, (wr_buff+ret_size), EMMC_PAGE_SIZE);
			if (tmp_size == 0) {
				return -1;
			}
			ret_size += tmp_size;
		}

		if(wr_size % EMMC_PAGE_SIZE)
		{
			if((wr_size-ret_size) >= EMMC_SECTOR_BLK_SIZE)
			{
				tmp_size = resetlog_flash_write_sub(s_sect, (wr_buff+ret_size), (wr_size-ret_size));
				if (tmp_size == 0) {
					return -1;
				}
				ret_size += tmp_size;
			}
		}

	} else {
		for (i = 0; i < wr_size; i++) {
			tmp_buff[i] = wr_buff[i];
		}
		if (resetlog_flash_write_sub(s_sect, tmp_buff, EMMC_SECTOR_BLK_SIZE) == 0) {
			return -1;
		}
		goto WRITE_END;
	}

	if (ret_size < wr_size) {
		tmp_size = (wr_size - ret_size);
		for (i = 0; i < tmp_size; i++) {
			tmp_buff[i] = wr_buff[ret_size + i];
		}
		if (resetlog_flash_write_sub(s_sect, tmp_buff, EMMC_SECTOR_BLK_SIZE) == 0) {
			return -1;
		}
	}
WRITE_END:
	return 0;
}

/*
 * resetlog_clear_info()
 *
 * Note: Resetlog clear info.
 */
static void resetlog_clear_info( emmc_info_type *emmc_log_info )
{
	uint32_t i;

	emmc_log_info->write_index = 0;
	for (i = 0; i < EMMC_INDEX_MAX; i++ ) {
		emmc_log_info->valid_flg[i] = 0;
	}
}

/*
 * resetlog_to_eMMC_kyocera_write()
 *
 * Note: Resetlog to eMMC kyocera write.
 */
static void resetlog_to_eMMC_kyocera_write(void)
{
	int 				ret;
	uint				write_sector;   /* sector to write data on the EMMC. */
	uint				read_sector;    /* sector to read data on the EMMC.  */
	unsigned long		write_buff;     /* address of the data buffer.       */
	unsigned long		write_size;     /* size of the data to be written.   */
	unsigned long		write_index;
	ram_log_info_type   *pLogInfo = (ram_log_info_type *)addr_info.addr_control_info;
	emmc_info_type		*Emmc_log_info = NULL;
	void				*m_log = NULL;

	Emmc_log_info = (emmc_info_type *)vzalloc(sizeof(emmc_info_type));

	if(NULL == Emmc_log_info) {
		pr_err("Emmc_log_info vzalloc error occured\n");
		return;
	}

	/* Read Control info */
	read_sector = EMMC_INFO_START_SECTOR;
	ret = resetlog_flash_read( &read_sector, (unsigned char *)Emmc_log_info, sizeof(emmc_info_type) );
	if ( ret < 0 ) goto out;

	/* Check resetlog.h version */
	if ( memcmp(&Emmc_log_info->logging_ver[0], (void *)HEADER_VERSION, strlen(HEADER_VERSION)) != 0 ) {
		resetlog_clear_info(Emmc_log_info);
	}
	memcpy( &Emmc_log_info->logging_ver[0], (void *)HEADER_VERSION, strlen(HEADER_VERSION) );

	/* Check write_sector */
	if (Emmc_log_info->write_index >= EMMC_INDEX_MAX) {
		resetlog_clear_info(Emmc_log_info);
	}
	write_index = Emmc_log_info->write_index;

	/* Set write_sector */
	write_sector = EMMC_REC_START_SECTOR(write_index);

	/* Copy to eMMC from SMEM */
	memset( &Emmc_log_info->rec_info[write_index], 0x00, sizeof(emmc_rec_info_type) );
	memcpy_fromio( &Emmc_log_info->rec_info[write_index].crash_system[0], pLogInfo->crash_system, CRASH_SYSTEM_SIZE );
	strncat( &Emmc_log_info->rec_info[write_index].crash_system[0], "(SSR)", CRASH_SYSTEM_SIZE-1 );
	memcpy_fromio( &Emmc_log_info->rec_info[write_index].crash_kind[0]  , pLogInfo->crash_kind  , CRASH_KIND_SIZE   );
	memcpy_fromio( &Emmc_log_info->rec_info[write_index].crash_time[0]  , pLogInfo->crash_time  , CRASH_TIME_SIZE   );
	memcpy_fromio( &Emmc_log_info->rec_info[write_index].kcj_info.kcd.linux_banner[0]      , pLogInfo->kcj_info.kcd.linux_banner      , BANNER_SIZE           );
	memcpy_fromio( &Emmc_log_info->rec_info[write_index].kcj_info.kcd.product_number[0]    , pLogInfo->kcj_info.kcd.product_number    , PRODUCT_SIZE          );
	memcpy_fromio( &Emmc_log_info->rec_info[write_index].kcj_info.kcd.software_version[0]  , pLogInfo->kcj_info.kcd.software_version  , SOFT_VERSION_SIZE     );
	memcpy_fromio( &Emmc_log_info->rec_info[write_index].kcj_info.kcd.baseband_version[0]  , pLogInfo->kcj_info.kcd.baseband_version  , BASEBAND_VERSION_SIZE );
	memcpy_fromio( &Emmc_log_info->rec_info[write_index].kcj_info.kcd.android_build_date[0], pLogInfo->kcj_info.kcd.android_build_date, BUILD_DATE_SIZE       );
	memcpy_fromio( &Emmc_log_info->rec_info[write_index].kcj_info.kcd.modem_build_date[0]  , pLogInfo->kcj_info.kcd.modem_build_date  , BUILD_DATE_SIZE       );

	Emmc_log_info->rec_info[write_index].ver_format = pLogInfo->ver_format;
	Emmc_log_info->rec_info[write_index].boot_mode = boot_mode;

	/* WRITE : KERNEL LOG */
	write_buff = (unsigned long)log_buf_addr_get();
	write_size = (unsigned long)log_buf_len_get();
	ret = resetlog_flash_write( &write_sector, (unsigned char *)write_buff, write_size );
	if ( ret < 0 ) goto out;

	Emmc_log_info->rec_info[write_index].kernel_log_magic     = KERNEL_LOG_MAGIC_CODE;
	Emmc_log_info->rec_info[write_index].kernel_log_buf_len   = (uint32_t)log_buf_len_get();
	Emmc_log_info->rec_info[write_index].kernel_log_first_idx = *(uint32_t *)log_first_idx_addr_get();
	Emmc_log_info->rec_info[write_index].kernel_log_next_idx  = *(uint32_t *)log_next_idx_addr_get();

	/* WRITE : LOGCAT MAIN */
	write_buff = (unsigned long)addr_info.addr_logcat_main;
	write_size = SIZE_LOGCAT_MAIN;
	ret = resetlog_flash_write( &write_sector, (unsigned char *)write_buff, write_size );
	if ( ret < 0 ) goto out;

	Emmc_log_info->rec_info[write_index].info[LOGGER_INFO_MAIN].w_off = pLogInfo->info[LOGGER_INFO_MAIN].w_off;
	Emmc_log_info->rec_info[write_index].info[LOGGER_INFO_MAIN].head  = pLogInfo->info[LOGGER_INFO_MAIN].head;

	/* WRITE : LOGCAT SYSTEM */
	write_buff = (unsigned long)addr_info.addr_logcat_system;
	write_size = SIZE_LOGCAT_SYSTEM;
	ret = resetlog_flash_write( &write_sector, (unsigned char *)write_buff, write_size );
	if ( ret < 0 ) goto out;

	Emmc_log_info->rec_info[write_index].info[LOGGER_INFO_SYSTEM].w_off = pLogInfo->info[LOGGER_INFO_SYSTEM].w_off;
	Emmc_log_info->rec_info[write_index].info[LOGGER_INFO_SYSTEM].head  = pLogInfo->info[LOGGER_INFO_SYSTEM].head;

	/* WRITE : LOGCAT EVENTS */
	write_buff = (unsigned long)addr_info.addr_logcat_events;
	write_size = SIZE_LOGCAT_EVENTS;
	ret = resetlog_flash_write( &write_sector, (unsigned char *)write_buff, write_size );
	if ( ret < 0 ) goto out;

	Emmc_log_info->rec_info[write_index].info[LOGGER_INFO_EVENTS].w_off = pLogInfo->info[LOGGER_INFO_EVENTS].w_off;
	Emmc_log_info->rec_info[write_index].info[LOGGER_INFO_EVENTS].head  = pLogInfo->info[LOGGER_INFO_EVENTS].head;

	/* WRITE : LOGCAT RADIO */
	write_buff = (unsigned long)addr_info.addr_logcat_radio;
	write_size = SIZE_LOGCAT_RADIO;
	ret = resetlog_flash_write( &write_sector, (unsigned char *)write_buff, write_size );
	if ( ret < 0 ) goto out;

	Emmc_log_info->rec_info[write_index].info[LOGGER_INFO_RADIO].w_off = pLogInfo->info[LOGGER_INFO_RADIO].w_off;
	Emmc_log_info->rec_info[write_index].info[LOGGER_INFO_RADIO].head  = pLogInfo->info[LOGGER_INFO_RADIO].head;

	/* WRITE : M LOG */
	m_log = vzalloc(SIZE_M_LOG);
	if(NULL != m_log) {
		write_buff = (unsigned long)m_log;
		write_size = SIZE_M_LOG;
		ret = resetlog_flash_write( &write_sector, (unsigned char *)write_buff, write_size );
		vfree(m_log);
		if ( ret < 0 ) goto out;
	}
	else {
		pr_err("m_log vzalloc error occured\n");
	}

	/* Write to eMMC */
	Emmc_log_info->valid_flg[write_index] = 1;
	Emmc_log_info->write_index = ((write_index + 1) % EMMC_INDEX_MAX);
	write_buff = (unsigned long)Emmc_log_info;
	write_size = sizeof(emmc_info_type);
	write_sector = EMMC_INFO_START_SECTOR;
	resetlog_flash_write( &write_sector, (unsigned char *)write_buff, write_size );

out:
	vfree((void *)Emmc_log_info);
	return;
}

/*
 * dump_kcj_log()
 *
 * Note: Dump kcj log from uninit ram to eMMC.
 */
void dump_kcj_log( void )
{
	ram_log_info_type *pPanicInfo   = (ram_log_info_type *)addr_info.addr_control_info;

	if ( (0 == strncmp((const char *)pPanicInfo->crash_system, CRASH_SYSTEM_KERNEL    , strlen(CRASH_SYSTEM_KERNEL   ))) ||
		 (0 == strncmp((const char *)pPanicInfo->crash_system, CRASH_SYSTEM_MODEM     , strlen(CRASH_SYSTEM_MODEM    ))) ||
		 (0 == strncmp((const char *)pPanicInfo->crash_system, CRASH_SYSTEM_PRONTO    , strlen(CRASH_SYSTEM_PRONTO   ))) ||
		 (0 == strncmp((const char *)pPanicInfo->crash_system, CRASH_SYSTEM_ADSP      , strlen(CRASH_SYSTEM_ADSP     ))) ||
		 (0 == strncmp((const char *)pPanicInfo->crash_system, CRASH_SYSTEM_VENUS     , strlen(CRASH_SYSTEM_VENUS    ))) ||
		 (0 == strncmp((const char *)pPanicInfo->crash_system, CRASH_SYSTEM_ANDROID   , strlen(CRASH_SYSTEM_ANDROID  ))) ||
		 (0 == strncmp((const char *)pPanicInfo->crash_system, CRASH_SYSTEM_DM_VERITY , strlen(CRASH_SYSTEM_DM_VERITY))) ||
		 (0 == strncmp((const char *)pPanicInfo->crash_system, CRASH_SYSTEM_UNKNOWN   , strlen(CRASH_SYSTEM_UNKNOWN  ))) )
	{
		resetlog_to_eMMC_kyocera_write();
	}

	clear_kcj_crash_info();
	clear_smem_kcjlog();
	set_kcj_unknown_info();
	clear_smem_crash_info_data();
	clear_smem_panic_info_data();
}

/*
 * kcjlog_init()
 *
 * Note: kcjlog init.
 */
static int __init kcjlog_init(void)
{
	if ( NULL == kcj_get_uninit_ram_base_addr() ) return -1;

	if(bark_regaddr != 0)
	{
		set_kcj_regsave_addr(bark_regaddr);
	}

	set_kcj_fixed_info();
	set_kcj_unknown_info();

	return 0;
}

static int build_date_changed_handler( const char *kmessage, struct kernel_param *kp )
{
	if(addr_info.addr_control_info != NULL) {
		memcpy_toio( (unsigned char *)(((ram_log_info_type *)addr_info.addr_control_info)->kcj_info.kcd.android_build_date),
					kmessage, BUILD_DATE_SIZE );
		return 0;
	}
	return -1;
}

static int build_date_get_handler( char *buffer, const struct kernel_param *kp )
{
	char temp_buf[BUILD_DATE_SIZE];
	memset(temp_buf, 0x00, sizeof(temp_buf));

	if(addr_info.addr_control_info != NULL) {
		memcpy_fromio(temp_buf, (unsigned char *)(((ram_log_info_type *)addr_info.addr_control_info)->kcj_info.kcd.android_build_date),
					BUILD_DATE_SIZE );
		return strlcpy(buffer, temp_buf, sizeof(temp_buf));
	}
	return -1;
}

static int kcj_uninit_ram_ioremap( void )
{
	/* uninit ram  */
	if ( !request_mem_region(MSM_UNINIT_RAM_BASE, RESETLOG_UNINIT_RAM_SIZE, "kc_uninit") )
	{
		printk( "memory region request fail for uninit ram." );
		return -1;
	}

	uninit_ram_virt_addr = ioremap_nocache( MSM_UNINIT_RAM_BASE, RESETLOG_UNINIT_RAM_SIZE );
	if( NULL == uninit_ram_virt_addr )
	{
		printk( "ioremap fail for uninit ram." );
		return -1;
	}

	addr_info.addr_control_info       = uninit_ram_virt_addr;
	addr_info.addr_logcat_main        = uninit_ram_virt_addr              + SIZE_CONTROL_INFO;
	addr_info.addr_logcat_system      = addr_info.addr_logcat_main        + SIZE_LOGCAT_MAIN;
	addr_info.addr_logcat_events      = addr_info.addr_logcat_system      + SIZE_LOGCAT_SYSTEM;
	addr_info.addr_logcat_radio       = addr_info.addr_logcat_events      + SIZE_LOGCAT_EVENTS;

	printk( "[Y.Y] kcj_uninit_ram_ioremap: uninit_ram_virt_addr = 0x%p\n", uninit_ram_virt_addr );

	return 0;
}

static void __iomem *kcj_get_uninit_ram_base_addr( void )
{
	if ( NULL == uninit_ram_virt_addr )
	{
		if ( -1 == kcj_uninit_ram_ioremap() )
		{
			return NULL;
		}
	}

	return( uninit_ram_virt_addr );
}

int __init kcj_uninitram_init( void )
{
	addr_info.addr_control_info       = NULL;
	addr_info.addr_logcat_main        = NULL;
	addr_info.addr_logcat_system      = NULL;
	addr_info.addr_logcat_events      = NULL;
	addr_info.addr_logcat_radio       = NULL;

	return( kcj_uninit_ram_ioremap() );
}

/*
 * logcat_save_open()
 *
 * Note: open logcat-data.
 */
static int logcat_save_open( struct inode *inode, struct file *file )
{
	return 0;
}

/*
 * logcat_save_close()
 *
 * Note: close logcat-data.
 */
static int logcat_save_close( struct inode *inode, struct file *file )
{
	return 0;
}

/*
 * logcat_save_write()
 *
 * Note: write logcat-data.
 */
static ssize_t logcat_save_write( struct file *file, const char __user *buf, size_t count, loff_t *offset )
{
	int  len;
	int  logcat_type;
	char *tmp_buf = NULL;
	struct miscdevice *wk_misc;

	if( count == 0 )
	{
		pr_err( "[%s] Illegal write size=%zu.\n", __func__, count );
		return -1;
	}

	wk_misc = ( struct miscdevice * )file->private_data;

	/* check logcat-type */
	for( logcat_type = 0; logcat_type < LOGGER_INFO_MAX; logcat_type++ )
	{
		if( ( strncmp( regist_info[ logcat_type ]->misc.name,
					   wk_misc->name,
					   strlen( regist_info[ logcat_type ]->misc.name ) ) ) == 0 )
		{
			break;
		}
	}

	if( logcat_type >= LOGGER_INFO_MAX )
	{
		pr_err( "[%s] No regist data.\n", __func__ );
		return -1;
	}

	tmp_buf = kcalloc( 1, count, GFP_KERNEL );

	if( tmp_buf == NULL )
	{
		pr_err( "[%s] failed to allocate memory.\n", __func__ );
		return -1;
	}

	if( copy_from_user( tmp_buf, buf, count ) )
	{
		pr_err( "[%s] failed to copy(from user).\n", __func__ );
		kfree( tmp_buf );
		return -1;
	}

	len = min( count, ( regist_info[ logcat_type ]->size - regist_info[ logcat_type ]->w_off ) );

	memcpy_toio( ( regist_info[ logcat_type ]->buffer + regist_info[ logcat_type ]->w_off ), tmp_buf, len );

	if( count != len )
		memcpy_toio( regist_info[ logcat_type ]->buffer, ( tmp_buf + len ), ( count - len ) );

	kfree( tmp_buf );

	regist_info[ logcat_type ]->w_off = ( ( regist_info[ logcat_type ]->w_off + count ) & ( regist_info[ logcat_type ]->size - 1 ) );
	regist_info[ logcat_type ]->log_info->w_off = regist_info[ logcat_type ]->w_off;

	return ( count );
}

static const struct file_operations logcat_save_ops =
{
	.owner   = THIS_MODULE,
	.open    = logcat_save_open,
	.release = logcat_save_close,
	.write   = logcat_save_write,
};

/*
 * logcat_regist()
 *
 * Note: regist logcat-data.
 */
static logcat_regist_info *logcat_regist( char *log_name, unsigned char *buff_addr, int size, logger_log_info *info_addr )
{
	int ret = 0;
	logcat_regist_info *wk_info;

	wk_info = kzalloc( sizeof( logcat_regist_info ), GFP_KERNEL );

	if( wk_info == NULL )
	{
		pr_err( "[%s] failed to allocate memory.\n", __func__ );
		return NULL;
	}

	wk_info->buffer          = buff_addr;
	wk_info->size            = size;
	wk_info->w_off           = 0;
	wk_info->log_info        = info_addr;
	wk_info->log_info->w_off = 0;
	wk_info->log_info->head  = 0;

	wk_info->misc.minor  = MISC_DYNAMIC_MINOR;
	wk_info->misc.name   = log_name;
	wk_info->misc.fops   = &logcat_save_ops;
	wk_info->misc.parent = NULL;

	/* finally, initialize the misc device for this log */
	ret = misc_register( &wk_info->misc );

	if( ret != 0 )
	{
		pr_err( "[%s] failed to register misc device for '%s'.\n",
				__func__, wk_info->misc.name );
		return NULL;
	}

	pr_info( "\"%s\" created. size=%luK, addr=0x%p\n",
				wk_info->misc.name, ( ( unsigned long )wk_info->size >> 10 ), wk_info->buffer );

	return ( wk_info );
}

/*
 * logcat_save_init()
 *
 * Note: logcat_save init.
 */
static int __init logcat_save_init( void )
{
	int logcat_type;

	if( kcj_get_uninit_ram_base_addr() == NULL )
	{
		pr_err( "[%s] failed to get uninit-ram-address.\n", __func__ );
		return 1;
	}

	set_info[ LOGGER_INFO_MAIN ].buf_addr   = ( uninit_ram_virt_addr                    + SIZE_CONTROL_INFO  );
	set_info[ LOGGER_INFO_SYSTEM ].buf_addr = ( set_info[ LOGGER_INFO_MAIN ].buf_addr   + SIZE_LOGCAT_MAIN   );
	set_info[ LOGGER_INFO_EVENTS ].buf_addr = ( set_info[ LOGGER_INFO_SYSTEM ].buf_addr + SIZE_LOGCAT_SYSTEM );
	set_info[ LOGGER_INFO_RADIO ].buf_addr  = ( set_info[ LOGGER_INFO_EVENTS ].buf_addr + SIZE_LOGCAT_EVENTS );

	for( logcat_type = 0; logcat_type < LOGGER_INFO_MAX; logcat_type++ )
	{
		regist_info[ logcat_type ] = logcat_regist( set_info[ logcat_type ].log_name,
													set_info[ logcat_type ].buf_addr,
													set_info[ logcat_type ].size,
													&( ( ram_log_info_type * )uninit_ram_virt_addr )->info[ logcat_type ] );

		if( regist_info == NULL )
		{
			break;
		}
	}

	return 0;
}

/*
 * logcat_save_exit()
 *
 * Note: logcat_save exit.
 */
static void __exit logcat_save_exit( void )
{
	int logcat_type;

	for( logcat_type = 0; logcat_type < LOGGER_INFO_MAX; logcat_type++ )
	{
		if( regist_info[ logcat_type ] != NULL )
			kfree( regist_info[ logcat_type ] );
	}
}

pure_initcall(kcj_uninitram_init);
device_initcall(kcjlog_init);
late_initcall(resetlog_emmc_drv_init);
device_initcall(logcat_save_init);
module_exit(logcat_save_exit);
