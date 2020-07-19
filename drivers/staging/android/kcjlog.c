/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
                              <kcjlog.c>
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

*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
#define pr_fmt(fmt) "KCJLOG: " fmt
/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
 * INCLUDE FILES
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/

#include <linux/module.h>
#include <linux/rtc.h>
#include <linux/bio.h>
#include <linux/genhd.h>
#include <linux/delay.h>
#include <linux/pstore.h>
#include <linux/pstore_ram.h>
#include <linux/proc_fs.h>
#include <linux/kcjlog.h>
#include <soc/qcom/smsm.h>
#include <mach/msm_iomap.h>
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
#define ARRAY_SIZEOF(a) (sizeof(a)/sizeof((a)[0]))

static dev_t log_dev_t = EMMC_DEV_INVALID;
static dev_t log_parent_dev = MKDEV(MMC_BLOCK_MAJOR, 0);
const static char log_partition_label[] = "log";

/* ResetLog control info */
typedef struct {
	unsigned char          crash_system[CRASH_SYSTEM_SIZE];
	unsigned char          crash_kind[CRASH_KIND_SIZE];
	unsigned char          crash_time[CRASH_TIME_SIZE];
	unsigned long          ver_format;
	unsigned long          padding0[3];
	kcj_info_type          kcj_info;
	unsigned char          crash_info_data[CRASH_INFO_DATA_SIZE];
	unsigned char          panic_info_data[CRASH_INFO_DATA_SIZE];
	logger_log_info        info[LOGGER_INFO_MAX];
	unsigned long          reserved0[4];
	unsigned long          regsave_addr;
	unsigned long          regsave_addr_check;
	unsigned long          padding1[2];
	unsigned long          m_info[6];
	unsigned long          padding2[2];
	unsigned long          ocimem_size;
	unsigned long          rpm_code_size;
	unsigned long          rpm_data_size;
	unsigned long          rpm_msg_size;
	unsigned long          lpm_size;
	unsigned long          reset_status_size;
	unsigned long          pmic_pon_status_size;
	unsigned long          padding3;
	unsigned long          kernel_log_magic;
	unsigned long          kernel_log_start;
	unsigned long          kernel_log_size;
	unsigned long          kernel_log_reserved;
	unsigned long          reserved1[108];
} emmc_rec_info_type;

typedef struct {
	unsigned char          logging_ver[HEADER_VERION_SIZE]; /* resetlog.h version */
	unsigned long          write_index;                     /* Index of write (0 - (EMMC_INDEX_MAX - 1)) */
	unsigned char          valid_flg[EMMC_INDEX_MAX];       /* valid(1) or invalid(0) */
	logger_log_info        fotalog[LOGGER_INFO_MAX];        /* Fotalog Info */
	unsigned long          settings[40];                    /* eMMc settings */
	unsigned long          reserved[76];                    /* customize area */
	emmc_rec_info_type     rec_info[EMMC_INDEX_MAX];        /* Record Info */
} emmc_info_type;
emmc_info_type		Emmc_log_info;

static unsigned long	bark_regaddr = 0;
static struct persistent_ram_zone *ram_console_zone;

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

	/* system check */
	if ( (0 == strncmp((const char *)plog_info->crash_system, CRASH_SYSTEM_KERNEL  , strlen(CRASH_SYSTEM_KERNEL ))) ||
		 (0 == strncmp((const char *)plog_info->crash_system, CRASH_SYSTEM_MODEM   , strlen(CRASH_SYSTEM_MODEM  ))) ||
		 (0 == strncmp((const char *)plog_info->crash_system, CRASH_SYSTEM_PRONTO  , strlen(CRASH_SYSTEM_PRONTO ))) ||
		 (0 == strncmp((const char *)plog_info->crash_system, CRASH_SYSTEM_ADSP    , strlen(CRASH_SYSTEM_ADSP   ))) ||
		 (0 == strncmp((const char *)plog_info->crash_system, CRASH_SYSTEM_VENUS   , strlen(CRASH_SYSTEM_VENUS  ))) ||
		 (0 == strncmp((const char *)plog_info->crash_system, CRASH_SYSTEM_ANDROID , strlen(CRASH_SYSTEM_ANDROID))) )
	{
		ret = 1;
	}
	/* kind check */
	if ( (0 == strncmp((const char *)plog_info->crash_kind, CRASH_KIND_PANIC      , strlen(CRASH_KIND_PANIC     ))) ||
		 (0 == strncmp((const char *)plog_info->crash_kind, CRASH_KIND_FATAL      , strlen(CRASH_KIND_FATAL     ))) ||
		 (0 == strncmp((const char *)plog_info->crash_kind, CRASH_KIND_EXCEPTION  , strlen(CRASH_KIND_EXCEPTION ))) ||
		 (0 == strncmp((const char *)plog_info->crash_kind, CRASH_KIND_WDOG_HW    , strlen(CRASH_KIND_WDOG_HW   ))) ||
		 (0 == strncmp((const char *)plog_info->crash_kind, CRASH_KIND_WDOG_SW    , strlen(CRASH_KIND_WDOG_SW   ))) ||
		 (0 == strncmp((const char *)plog_info->crash_kind, CRASH_KIND_SYS_SERVER , strlen(CRASH_KIND_SYS_SERVER))) ||
		 (0 == strncmp((const char *)plog_info->crash_kind, CRASH_KIND_PWR_KEY    , strlen(CRASH_KIND_PWR_KEY   ))) ||
		 (0 == strncmp((const char *)plog_info->crash_kind, CRASH_KIND_KDFS_REBOOT, strlen(CRASH_KIND_KDFS_REBOOT))) )
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
	if (check_smem_kcjlog(pSmemLogInfo) == 0) {
		return;
	}
	if (bufsize > CRASH_SYSTEM_SIZE) {
		bufsize = CRASH_SYSTEM_SIZE;
	}

	memcpy( crash_system, &pSmemLogInfo->crash_system[0], bufsize );
}

/*
 * get_crash_time()
 *
 * Note: Get utc time.
 */
static void get_crash_time(unsigned char *buf, unsigned int bufsize)
{
	struct timespec ts_now;
	struct rtc_time tm;

	ts_now = current_kernel_time();
	rtc_time_to_tm(ts_now.tv_sec, &tm);

	memset( buf ,0x00, bufsize );
	snprintf( buf, bufsize, "%d-%02d-%02d %02d:%02d:%02d.%09lu UTC",
				tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
				tm.tm_hour, tm.tm_min, tm.tm_sec, ts_now.tv_nsec);
}

/*
 * set_kcj_regsave_addr()
 *
 * Note: Set regsave addr to uninit ram.
 */
void set_kcj_regsave_addr(unsigned long regsave_addr)
{
	ram_log_info_type *pPanicInfo;

	pPanicInfo   = (ram_log_info_type *)ADDR_CONTROL_INFO;
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
	ram_log_info_type *pPanicInfo;

	pPanicInfo   = (ram_log_info_type *)ADDR_CONTROL_INFO;
	memcpy( &pPanicInfo->magic_code[0]  , CRASH_MAGIC_CODE   , strlen(CRASH_MAGIC_CODE  ) );
	pPanicInfo->ver_format = 0;
	memcpy( &pPanicInfo->kcj_info.kcy.linux_ver[0]   , BUILD_DISPLAY_ID   , strlen(BUILD_DISPLAY_ID  ) );
	memcpy( &pPanicInfo->kcj_info.kcy.model[0]       , PRODUCT_MODEL_NAME , strlen(PRODUCT_MODEL_NAME) );
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
	pPanicInfo   = (ram_log_info_type *)ADDR_CONTROL_INFO;
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

	memset( &pPanicInfo->crash_system[0], 0x00, CRASH_SYSTEM_SIZE );
	memset( &pPanicInfo->crash_kind[0]  , 0x00, CRASH_KIND_SIZE );

	memcpy( &pPanicInfo->crash_system[0], &pSmemLogInfo->crash_system[0], CRASH_SYSTEM_SIZE );
	memcpy( &pPanicInfo->crash_kind[0]  , &pSmemLogInfo->crash_kind[0]  , CRASH_KIND_SIZE );

	memcpy( &pPanicInfo->crash_info_data[0], &pSmemLogInfo->crash_info_data[0], CRASH_INFO_DATA_SIZE );
	memcpy( &pPanicInfo->panic_info_data[0], &pSmemLogInfo->panic_info_data[0], CRASH_INFO_DATA_SIZE );

	get_crash_time( &pPanicInfo->crash_time[0], CRASH_TIME_SIZE );
}

/*
 * set_kcj_unknown_info()
 *
 * Note: Set crash unknown info to uninit ram.
 */
static void set_kcj_unknown_info( void )
{
	ram_log_info_type *pPanicInfo;

	pPanicInfo = (ram_log_info_type *)ADDR_CONTROL_INFO;

	if ( pPanicInfo == NULL ) {
		return;
	}

	memcpy( &pPanicInfo->crash_system[0], CRASH_SYSTEM_UNKNOWN, strlen(CRASH_SYSTEM_UNKNOWN) );
	memcpy( &pPanicInfo->crash_kind[0]  , CRASH_KIND_UNKNOWN  , strlen(CRASH_KIND_UNKNOWN  ) );
}

/*
 * clear_kcj_crash_info()
 *
 * Note: Clear crash info from uninit ram.
 */
void clear_kcj_crash_info( void )
{
	ram_log_info_type *pPanicInfo;

	pPanicInfo = (ram_log_info_type *)ADDR_CONTROL_INFO;

	if ( pPanicInfo == NULL ) {
		return;
	}

	memset( &pPanicInfo->crash_system[0], 0x00, CRASH_SYSTEM_SIZE );
	memset( &pPanicInfo->crash_kind[0]  , 0x00, CRASH_KIND_SIZE );
	memset( &pPanicInfo->crash_time[0]  , 0x00, CRASH_TIME_SIZE );

	memset( &pPanicInfo->crash_info_data[0], 0x00, CRASH_INFO_DATA_SIZE );
	memset( &pPanicInfo->panic_info_data[0], 0x00, CRASH_INFO_DATA_SIZE );
}

/*
 * set_modemlog_info()
 *
 * Note: Set modem log info from smem to uninit ram.
 */
void set_modemlog_info(void)
{
	ram_log_info_type *pSmemLogInfo;
	ram_log_info_type *pPanicInfo;

	mb();
	pSmemLogInfo = (ram_log_info_type *)kc_smem_alloc(SMEM_CRASH_LOG, SIZE_SMEM_ALLOC);
	pPanicInfo   = (ram_log_info_type *)ADDR_CONTROL_INFO;

	if ( (pSmemLogInfo == NULL) || (pPanicInfo == NULL) ) {
		return;
	}

	pPanicInfo->m_info[0]  = pSmemLogInfo->m_info[0];
	pPanicInfo->m_info[1]  = pSmemLogInfo->m_info[1];
	pPanicInfo->m_info[2]  = pSmemLogInfo->m_info[2];
	pPanicInfo->m_info[3]  = pSmemLogInfo->m_info[3];
	pPanicInfo->m_info[4]  = pSmemLogInfo->m_info[4];
	pPanicInfo->m_info[5]  = pSmemLogInfo->m_info[5];
	pPanicInfo->m_info[8]  = pSmemLogInfo->m_info[8];
	pPanicInfo->m_info[9]  = pSmemLogInfo->m_info[9];
	pPanicInfo->m_info[10] = pSmemLogInfo->m_info[10];

	memcpy( &pPanicInfo->kcj_info.kcy.modem_ver[0]   , &pSmemLogInfo->kcj_info.kcy.modem_ver[0]   , VERSION_SIZE );

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
	log_bio->bi_sector = sector;
	log_bio->bi_bdev = log_bdev;

	if (bio_add_page(log_bio, log_page, size, 0) != size) {
			pr_err("Could not add page\n");
			ret = -ENOMEM;
			goto out;
	}

	log_bio->bi_vcnt = 1;
	log_bio->bi_idx = 0;
	log_bio->bi_size = size;
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
	memcpy(page_address(log_page), pbuf, total);

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
	int count;

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
static void resetlog_to_eMMC_kyocera_write( void )
{
	long				ret;
	uint				write_sector;   /* sector to write data on the EMMC. */
	uint				read_sector;    /* sector to read data on the EMMC.  */
	unsigned long		write_buff;     /* address of the data buffer.       */
	unsigned long		write_size;     /* size of the data to be written.   */
	unsigned long		write_index;
	ram_log_info_type	*pLogInfo = (ram_log_info_type *)ADDR_CONTROL_INFO;

	/* Read Control info */
	read_sector = EMMC_INFO_START_SECTOR;
	ret = resetlog_flash_read( &read_sector, (unsigned char *)&Emmc_log_info, sizeof(Emmc_log_info) );
	if ( ret < 0 ) return;

	/* Check resetlog.h version */
	if ( memcmp(&Emmc_log_info.logging_ver[0], (void *)HEADER_VERION, strlen(HEADER_VERION)) != 0 ) {
		resetlog_clear_info(&Emmc_log_info);
	}
	memcpy( &Emmc_log_info.logging_ver[0], (void *)HEADER_VERION, strlen(HEADER_VERION) );

	/* Check write_sector */
	if (Emmc_log_info.write_index >= EMMC_INDEX_MAX) {
		resetlog_clear_info(&Emmc_log_info);
	}
	write_index = Emmc_log_info.write_index;

	/* Set write_sector */
	write_sector = EMMC_REC_START_SECTOR(write_index);

	/* Copy to eMMC from SMEM */
	memcpy( &Emmc_log_info.rec_info[write_index].crash_system[0], pLogInfo->crash_system, CRASH_SYSTEM_SIZE );
	strncat( &Emmc_log_info.rec_info[write_index].crash_system[0], "(SSR)", CRASH_SYSTEM_SIZE-1 );
	memcpy( &Emmc_log_info.rec_info[write_index].crash_kind[0]  , pLogInfo->crash_kind  , CRASH_KIND_SIZE   );
	memcpy( &Emmc_log_info.rec_info[write_index].crash_time[0]  , pLogInfo->crash_time  , CRASH_TIME_SIZE   );
	memcpy( &Emmc_log_info.rec_info[write_index].kcj_info.kcy.linux_ver[0] , pLogInfo->kcj_info.kcy.linux_ver   , VERSION_SIZE      );
	memcpy( &Emmc_log_info.rec_info[write_index].kcj_info.kcy.modem_ver[0] , pLogInfo->kcj_info.kcy.modem_ver   , VERSION_SIZE      );
	memcpy( &Emmc_log_info.rec_info[write_index].kcj_info.kcy.model[0]     , pLogInfo->kcj_info.kcy.model       , MODEL_SIZE        );

	/* WRITE : KERNEL LOG */
	write_buff = (unsigned long)ADDR_KERNEL_LOG;
	write_size = SIZE_KERNEL_LOG;
	ret = resetlog_flash_write( &write_sector, (unsigned char *)write_buff, write_size );
	if ( ret < 0 ) return;

	Emmc_log_info.rec_info[write_index].kernel_log_start = ((ram_console_type *)ADDR_KERNEL_LOG)->header.start;
	Emmc_log_info.rec_info[write_index].kernel_log_size  = ((ram_console_type *)ADDR_KERNEL_LOG)->header.size;

	/* WRITE : LOGCAT MAIN */
	write_buff = (unsigned long)ADDR_LOGCAT_MAIN;
	write_size = SIZE_LOGCAT_MAIN;
	ret = resetlog_flash_write( &write_sector, (unsigned char *)write_buff, write_size );
	if ( ret < 0 ) return;

	Emmc_log_info.rec_info[write_index].info[LOGGER_INFO_MAIN].w_off = pLogInfo->info[LOGGER_INFO_MAIN].w_off;
	Emmc_log_info.rec_info[write_index].info[LOGGER_INFO_MAIN].head  = pLogInfo->info[LOGGER_INFO_MAIN].head;

	/* WRITE : LOGCAT SYSTEM */
	write_buff = (unsigned long)ADDR_LOGCAT_SYSTEM;
	write_size = SIZE_LOGCAT_SYSTEM;
	ret = resetlog_flash_write( &write_sector, (unsigned char *)write_buff, write_size );
	if ( ret < 0 ) return;

	Emmc_log_info.rec_info[write_index].info[LOGGER_INFO_SYSTEM].w_off = pLogInfo->info[LOGGER_INFO_SYSTEM].w_off;
	Emmc_log_info.rec_info[write_index].info[LOGGER_INFO_SYSTEM].head  = pLogInfo->info[LOGGER_INFO_SYSTEM].head;

	/* WRITE : LOGCAT EVENTS */
	write_buff = (unsigned long)ADDR_LOGCAT_EVENTS;
	write_size = SIZE_LOGCAT_EVENTS;
	ret = resetlog_flash_write( &write_sector, (unsigned char *)write_buff, write_size );
	if ( ret < 0 ) return;

	Emmc_log_info.rec_info[write_index].info[LOGGER_INFO_EVENTS].w_off = pLogInfo->info[LOGGER_INFO_EVENTS].w_off;
	Emmc_log_info.rec_info[write_index].info[LOGGER_INFO_EVENTS].head  = pLogInfo->info[LOGGER_INFO_EVENTS].head;

	/* WRITE : LOGCAT RADIO */
	write_buff = (unsigned long)ADDR_LOGCAT_RADIO;
	write_size = SIZE_LOGCAT_RADIO;
	ret = resetlog_flash_write( &write_sector, (unsigned char *)write_buff, write_size );
	if ( ret < 0 ) return;

	Emmc_log_info.rec_info[write_index].info[LOGGER_INFO_RADIO].w_off = pLogInfo->info[LOGGER_INFO_RADIO].w_off;
	Emmc_log_info.rec_info[write_index].info[LOGGER_INFO_RADIO].head  = pLogInfo->info[LOGGER_INFO_RADIO].head;

	/* WRITE : SMEM EVENT LOG */
	write_buff = (unsigned long)ADDR_SMEM_EVENT_LOG;
	write_size = SIZE_SMEM_EVENT_LOG;
	ret = resetlog_flash_write( &write_sector, (unsigned char *)write_buff, write_size );
	if ( ret < 0 ) return;

	Emmc_log_info.rec_info[write_index].m_info[4] = pLogInfo->m_info[9];
	Emmc_log_info.rec_info[write_index].m_info[5] = pLogInfo->m_info[10];

	/* WRITE : F3 TRACE */
	write_buff = (unsigned long)ADDR_MODEM_F3_LOG;
	write_size = SIZE_MODEM_F3_LOG;
	ret = resetlog_flash_write( &write_sector, (unsigned char *)write_buff, write_size );
	if ( ret < 0 ) return;

	Emmc_log_info.rec_info[write_index].m_info[0] = pLogInfo->m_info[2];
	Emmc_log_info.rec_info[write_index].m_info[1] = pLogInfo->m_info[3];

	/* WRITE : ERR DATA(QUALCOMM) */
	write_buff = (unsigned long)ADDR_MODEM_ERR_DATA_LOG;
	write_size = SIZE_MODEM_ERR_DATA_LOG;
	ret = resetlog_flash_write( &write_sector, (unsigned char *)write_buff, write_size );
	if ( ret < 0 ) return;

	Emmc_log_info.rec_info[write_index].m_info[2] = pLogInfo->m_info[5];

	/* WRITE : ERR DATA(KC) */
	write_buff = (unsigned long)ADDR_MODEM_EXT_DATA_LOG;
	write_size = SIZE_MODEM_EXT_DATA_LOG;
	ret = resetlog_flash_write( &write_sector, (unsigned char *)write_buff, write_size );
	if ( ret < 0 ) return;

	Emmc_log_info.rec_info[write_index].m_info[2] = pLogInfo->m_info[7];

	/* WRITE : CPU CONTEXT INFO */
	write_buff = (unsigned long)ADDR_CPU_CONTEXT_INFO;
	write_size = SIZE_CPU_CONTEXT_INFO;
	ret = resetlog_flash_write( &write_sector, (unsigned char *)write_buff, write_size );
	if ( ret < 0 ) return;

	/* WRITE : USB DUMP DATA(QUALCOMM) */
	memset(ADDR_USB_DUMP_START, 0x00, (ADDR_USB_DUMP_END - ADDR_USB_DUMP_START));
	write_buff = (unsigned long)ADDR_USB_DUMP_START;
	write_size = ADDR_USB_DUMP_END - ADDR_USB_DUMP_START;
	ret = resetlog_flash_write( &write_sector, (unsigned char *)write_buff, write_size );
	if ( ret < 0 ) return;

	Emmc_log_info.rec_info[write_index].ocimem_size       = SIZE_OCIMEM;
	Emmc_log_info.rec_info[write_index].rpm_code_size     = SIZE_RPM_CODE;
	Emmc_log_info.rec_info[write_index].rpm_data_size     = SIZE_RPM_DATA;
	Emmc_log_info.rec_info[write_index].rpm_msg_size      = SIZE_RPM_MSG;
	Emmc_log_info.rec_info[write_index].lpm_size          = SIZE_LPM;
	Emmc_log_info.rec_info[write_index].reset_status_size = SIZE_RESET_STATUS;

	/* Write to eMMC */
	Emmc_log_info.valid_flg[write_index] = 1;
	Emmc_log_info.write_index = ((write_index + 1) % EMMC_INDEX_MAX);
	write_buff = (unsigned long)&Emmc_log_info;
	write_size = sizeof(Emmc_log_info);
	write_sector = EMMC_INFO_START_SECTOR;
	resetlog_flash_write( &write_sector, (unsigned char *)write_buff, write_size );
}

/*
 * dump_kcj_log()
 *
 * Note: Dump kcj log from uninit ram to eMMC.
 */
void dump_kcj_log( void )
{
	ram_log_info_type *pPanicInfo   = (ram_log_info_type *)ADDR_CONTROL_INFO;
	unsigned long save_m1;
	unsigned long save_m2;
	unsigned long save_m3;

	if ( (0 == strncmp((const char *)pPanicInfo->crash_system, CRASH_SYSTEM_KERNEL  , strlen(CRASH_SYSTEM_KERNEL ))) ||
		 (0 == strncmp((const char *)pPanicInfo->crash_system, CRASH_SYSTEM_MODEM   , strlen(CRASH_SYSTEM_MODEM  ))) ||
		 (0 == strncmp((const char *)pPanicInfo->crash_system, CRASH_SYSTEM_PRONTO  , strlen(CRASH_SYSTEM_PRONTO ))) ||
		 (0 == strncmp((const char *)pPanicInfo->crash_system, CRASH_SYSTEM_ADSP	, strlen(CRASH_SYSTEM_ADSP   ))) ||
		 (0 == strncmp((const char *)pPanicInfo->crash_system, CRASH_SYSTEM_VENUS   , strlen(CRASH_SYSTEM_VENUS  ))) ||
		 (0 == strncmp((const char *)pPanicInfo->crash_system, CRASH_SYSTEM_ANDROID , strlen(CRASH_SYSTEM_ANDROID))) ||
		 (0 == strncmp((const char *)pPanicInfo->crash_system, CRASH_SYSTEM_UNKNOWN , strlen(CRASH_SYSTEM_UNKNOWN))) )
	{
		save_m1 = pPanicInfo->m_info[2];
		save_m2 = pPanicInfo->m_info[3];
		save_m3 = pPanicInfo->m_info[9];

		memset( (void *)ADDR_MODEM_F3_LOG, 0x00, SIZE_MODEM_F3_LOG );
		memset( (void *)ADDR_MODEM_ERR_DATA_LOG, 0x00, SIZE_MODEM_ERR_DATA_LOG );
		memset( (void *)ADDR_SMEM_EVENT_LOG, 0x00, SIZE_SMEM_EVENT_LOG );

		pPanicInfo->m_info[2] = 0x00;
		pPanicInfo->m_info[3] = 0x00;
		pPanicInfo->m_info[9] = 0x00;

		memset(ADDR_CPU_CONTEXT_INFO, 0x00, SIZE_CPU_CONTEXT_INFO);

		resetlog_to_eMMC_kyocera_write();

		pPanicInfo->m_info[2] = save_m1;
		pPanicInfo->m_info[3] = save_m2;
		pPanicInfo->m_info[9] = save_m3;
	}

	clear_kcj_crash_info();
	clear_smem_kcjlog();
	set_kcj_unknown_info();
	clear_smem_crash_info_data();
	clear_smem_panic_info_data();
}

#ifdef KC_LOG_CNFD_ENABLED
/*
 * get_emmc_info_settings()
 *
 * Note: Read the Settings value of eMMC.
 */
int get_emmc_info_settings(unsigned int index, unsigned long* value)
{
	struct block_device *bdev = NULL;
	struct block_device *logp_bdev = NULL;
	struct disk_part_iter piter;
	struct page *logp_page = NULL;
	struct bio *logp_bio = NULL;
	struct hd_struct *part;
	emmc_info_type* logp_header = NULL;
	dev_t logp_devt = EMMC_DEV_INVALID;
	int total = 0;
	int ret = -1;
	DECLARE_COMPLETION_ONSTACK(waithandle);

	if ((index >= ARRAY_SIZEOF(logp_header->settings)) || (NULL == value)) {
		pr_err("get_emmc_info_settings: Parameter Error\n");
		return -1;
	}

	bdev = blkdev_get_by_dev(log_parent_dev, FMODE_READ, NULL);
	if (IS_ERR(bdev)) {
		pr_err("blkdev_get_by_dev Error: %ld\n", (long)bdev);
		return -1;
	}

	disk_part_iter_init(&piter, bdev->bd_disk, DISK_PITER_INCL_EMPTY);
	while ((part = disk_part_iter_next(&piter))) {
		if (0 == strcmp("log", part->info->volname)) {
			pr_err("get_emmc_info_settings: open block_device of log\n");
			logp_devt = part_devt(part);
			logp_bdev = blkdev_get_by_dev(logp_devt, FMODE_READ, NULL);
			break;
		}
	}
	disk_part_iter_exit(&piter);
	blkdev_put(bdev, FMODE_READ);
	if ((EMMC_DEV_INVALID == logp_devt) || (IS_ERR(logp_bdev))){
		pr_err("get_emmc_info_settings: not found log partition\n");
		return -1;
	}

	total = EMMC_SECTOR_BLK_SIZE;
	logp_page = alloc_pages(GFP_KERNEL, get_order(total));
	if (!logp_page) {
		pr_err("get_emmc_info_settings: alloc_pages Error\n");
		__free_pages(logp_page, get_order(total));
		blkdev_put(logp_bdev, FMODE_READ);
		return -1;
	}

	logp_bio = bio_alloc(GFP_KERNEL, 1);
	if (!logp_bio) {
		pr_err("get_emmc_info_settings: bio_alloc Error\n");
		__free_pages(logp_page, get_order(total));
		blkdev_put(logp_bdev, FMODE_READ);
		return -1;
	}

	logp_bio->bi_sector = 0;
	logp_bio->bi_bdev = logp_bdev;
	if (bio_add_page(logp_bio, logp_page, total, 0) != total) {
		pr_err("get_emmc_info_settings: bio_add_page : Could not add page\n");
		bio_put(logp_bio);
		__free_pages(logp_page, get_order(total));
		blkdev_put(logp_bdev, FMODE_READ);
		return  -1;
	}

	logp_bio->bi_vcnt = 1;
	logp_bio->bi_idx = 0;
	logp_bio->bi_size = total;
	logp_bio->bi_end_io = resetlog_emmc_end_bio;
	logp_bio->bi_rw = 0;
	logp_bio->bi_private = &waithandle;

	bio_set_pages_dirty(logp_bio);

	bio_get(logp_bio);
	submit_bio(0, logp_bio);
	wait_for_completion(&waithandle);
	bio_put(logp_bio);

	logp_header = page_address(logp_page);
	if (logp_header) {
		*value = logp_header->settings[index];
		ret = 0;
	} else {
		pr_err("get_emmc_info_settings: Settings Read Error\n");
	}
	__free_pages(logp_page, get_order(total));
	blkdev_put(logp_bdev, FMODE_READ);
	return ret;
}
#endif /* KC_LOG_CNFD_ENABLED */

/*
 * kcjlog_init()
 *
 * Note: kcjlog init.
 */
static int __init kcjlog_init(void)
{
	memset( ADDR_CONTROL_INFO, 0x00, SIZE_CONTROL_INFO );
	if(bark_regaddr != 0)
	{
		set_kcj_regsave_addr(bark_regaddr);
	}
	set_kcj_fixed_info();
	set_kcj_unknown_info();
	return 0;
}

/*
 * kcj_last_kmsg_read()
 *
 * Note: read last_kmsg.
 */
static ssize_t kcj_last_kmsg_read(struct file *file, char __user *buf, size_t len, loff_t *offset )
{
	loff_t pos = *offset;
	ssize_t count;
	struct persistent_ram_zone *prz = ram_console_zone;
	size_t old_log_size = persistent_ram_old_size(prz);
	const char *old_log = persistent_ram_old(prz);
	char *str;
	int ret;

	if (dmesg_restrict && !capable(CAP_SYSLOG))
		return -EPERM;

	/* Main last_kmsg log */
	if (pos < old_log_size) {
		count = min(len, (size_t)(old_log_size - pos));
		if (copy_to_user(buf, old_log + pos, count))
			return -EFAULT;
		goto out;
	}

	/* ECC correction notice */
	pos -= old_log_size;
	count = persistent_ram_ecc_string(prz, NULL, 0);
	if (pos < count) {
		str = kmalloc(count, GFP_KERNEL);
		if (!str)
			return -ENOMEM;
		persistent_ram_ecc_string(prz, str, count + 1);
		count = min(len, (size_t)(count - pos));
		ret = copy_to_user(buf, str + pos, count);
		kfree(str);
		if (ret)
			return -EFAULT;
		goto out;
	}
	/* EOF */
	return 0;

out:
	*offset += count;
	return count;
}

static const struct file_operations kcj_last_kmsg_file_ops = {
	.owner = THIS_MODULE,
	.read = kcj_last_kmsg_read,
};

/*
 * kcj_last_kmsg_late_init()
 *
 * Note: last_kmsg init.
 */
static int __init kcj_last_kmsg_late_init( void )
{
	struct proc_dir_entry *entry;

	if ( ram_console_zone == NULL )
		goto out;

	if (persistent_ram_old_size(ram_console_zone) == 0)
	{
		pr_err("last_kmsg is not found.\n");
		goto out;
	}

	entry = proc_create( "last_kmsg", S_IFREG | S_IRUGO, NULL, &kcj_last_kmsg_file_ops);
	if ( !entry )
	{
		printk( "%s: failed to create proc entry\n", __func__ );
	    persistent_ram_free_old(ram_console_zone);
		goto out;
	}
out:
	return 0;
}

/*
 * kcj_pstore_open()
 *
 * Note: callback from pstore. no operation.
 */
static int kcj_pstore_open(struct pstore_info *psi)
{
	/* NOP */
	return 0;
}

/*
 * kcj_pstore_close()
 *
 * Note: callback from pstore. no operation.
 */
static int kcj_pstore_close(struct pstore_info *psi)
{
	/* NOP */
	return 0;
}

/*
 * kcj_pstore_read()
 *
 * Note: callback from pstore. no operation.
 */
static ssize_t kcj_pstore_read(u64 *id, enum pstore_type_id *type,
							int *count, struct timespec *timespec,
							char **buf, struct pstore_info *psi)
{
	/* NOP */
	return 0;
}

/*
 * kcj_pstore_write()
 *
 * Note: callback from pstore. write console to uninit ram.
 */
static int kcj_pstore_write(enum pstore_type_id type,
							enum kmsg_dump_reason reason, u64 *id,
							unsigned int part, int count, size_t size,
							struct pstore_info *psi)
{
	/* Write console to uninit ram */
    persistent_ram_write(ram_console_zone, psi->buf, size);
	return 0;
}

/*
 * kcj_pstore_erase()
 *
 * Note: callback from pstore. no operation.
 */
static int kcj_pstore_erase(enum pstore_type_id type, u64 id, int count,
							struct timespec time, struct pstore_info *psi)
{
	/* NOP */
	return 0;
}

static struct pstore_info kcj_pstore_info = {
	.owner		= THIS_MODULE,
	.name		= "kcjlog",
	.open		= kcj_pstore_open,
	.close		= kcj_pstore_close,
	.read		= kcj_pstore_read,
	.write		= kcj_pstore_write,
	.erase		= kcj_pstore_erase,
};

/*
 * kcj_ram_console_init()
 *
 * Note: kcj ram console init.
 */
static int __init kcj_ram_console_init(void)
{
	struct persistent_ram_zone *prz;
	struct persistent_ram_ecc_info ecc_info;

    /* set default ecc */
	ecc_info.block_size =   128;
	ecc_info.ecc_size   =    16;
	ecc_info.symsize    =     8;
	ecc_info.poly       = 0x11d;

	/* create persistent ram */
	prz = persistent_ram_new(ADDR_KCJ_RAM_CONSOLE, SIZE_KERNEL_LOG, 0, &ecc_info);
	if (IS_ERR(prz)) {
		int err = PTR_ERR(prz);
		pr_err( "failed kcjlog persistent_ram err:%d\n", err);
		goto out;
	}
	ram_console_zone = prz;
	persistent_ram_zap(ram_console_zone);

	/* register  pstore */
	kcj_pstore_info.bufsize = 1024; /* LOG_LINE_MAX */
	kcj_pstore_info.buf		= kmalloc(kcj_pstore_info.bufsize, GFP_KERNEL);
	if (kcj_pstore_info.buf == NULL)
	{
		pr_err( "failed kcjlog kmalloc:\n");
		goto out;
	}
	spin_lock_init(&kcj_pstore_info.buf_lock);
	pstore_register(&kcj_pstore_info);
out:
    return 0;
}

device_initcall(kcjlog_init);
device_initcall(kcj_ram_console_init);
late_initcall(kcj_last_kmsg_late_init);
late_initcall(resetlog_emmc_drv_init);
