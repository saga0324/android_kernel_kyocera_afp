/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2018 KYOCERA Corporation
 * (C) 2019 KYOCERA Corporation
 */

#ifndef __QL_SPI_DRIVER_H
#define __QL_SPI_DRIVER_H

#include"qlspi_tlc.h"
#include"qlspi.h"

#define QL_SPI_DRV_NAME 			"qlspi_slave_drv_name"

#define QL_SPI_DRV_DMESG_TAG        	" ql_spi : "

#define QLSPI_SYSFS_FOLDER			"eos_if"

#define MAX_AHB_BURST_READ_SIZE 	(64*1024)
#define MAX_AHB_READ_SIZE 	(4)

#define MAX_WAIT    	50				// read AHB status count

#define QLSPI_REGRW_SYSFS_ENTRY			//Create sysfs entries to perform register read/write

#define	SPI_SCALING_EN		//Scaling the Frequency of SPI

//#define	QLSPI_LINUX_TEST_CODE

/* Prints MOSI and MISO values from SPI driver */
//#define SPI_DBG
//#define	SPI_DBG_W


/* Disables,  M4toAP (SW_INTR2) interrupt disable and enable from AP */
#define	OPT_INTR

/* Combine DMA addr reg write and DMA burst size0 reg write to single SPI transaction */
#define	OPT_AHB_READ

/* Spurious interrupt check enable/disable*/
//#define	CHK_SPURIOUS_INTR

/* Enable SW_INTR1 with GPIO instead of SW_INTR_1 REG*/
#define	SW_INTR1_GPIO

/* Optimize AHB write query , write B0 03 optimization 1, if write operations are continuous*/
#define	OPT_AHB_WR

/* B1 02 optimization, send only 1 byte instead of 2 */
#define TWO_CLKS_AFTER_AHB_WRITE

/* Normal AHB read test */
//#define	AHB_READ_TEST

/*	DMA read burst size optimization, write only size0 and size1 registers	*/
#define	OPT_READ_BURST_SIZE

/*	Disable clearing of interrupt sts in QLSPI */
#define	OPT_DIS_CLR_INT_STS

/*	Enable chip selection control in SPI slave driver */
//#define	EN_CS_CONTROL

// added define for new shub read and write changes
#undef OPT_SPI_SINGLE_TXRX
//#define OPT_SPI_SINGLE_TXRX

/* Check for underflow */
//#define DMA_FIFO_UNDERFLOW_CHECK

/* Introduce DMA FIFO underflow error condition */
#define DMA_FIFO_UNDERFLOW_EMULATE
#define	USE_OWN_WQ

/* Debug option, Enable prints of cpu frequencies on reception of interrupt from s3*/
//#define	READ_CPU_PROC

#ifdef READ_CPU_PROC
#define 	NO_OF_CPUFREQ_PROCS	(4)
#endif

//Spi slave driver statuses

#define SPI_STATUS_OK    	0

#define SPI_STATUS_ERR     -1
#define SPI_STATUS_ERR_AHB     -2
#define SPI_STATUS_ERR_DMA     -3

/* DMA Register bit fields */
#define DMA_FIFO_EMPTY		(1 << 0)
#define DMA_FIFO_UNDERFLOW	(1 << 1)



#undef QLSPI_LINUX_DEBUG_EN

#ifdef QLSPI_LINUX_DEBUG_EN
#define QLSPI_ERR(fmt, args...)     pr_err(QL_SPI_DRV_DMESG_TAG"%s %d ERROR: "fmt, __FUNCTION__, __LINE__, ##args)
#define QLSPI_WARN(fmt, args...)    pr_notice(QL_SPI_DRV_DMESG_TAG"%s %d WARN: "fmt, __FUNCTION__, __LINE__, ##args)
#define QLSPI_LOG(fmt, args...)     pr_info(QL_SPI_DRV_DMESG_TAG"%s %d INFO: "fmt, __FUNCTION__, __LINE__, ##args)
#define QLSPI_DBG(fmt, args...)     pr_debug(QL_SPI_DRV_DMESG_TAG"%s %d DEBUG: "fmt, __FUNCTION__, __LINE__, ##args)
#define QLSPI_PRINTK(fmt, args...)  printk(KERN_NOTICE QL_SPI_DRV_DMESG_TAG fmt, ##args)

#else
#define QLSPI_ERR(fmt, args...)     pr_err(QL_SPI_DRV_DMESG_TAG"%s %d ERROR: "fmt, __FUNCTION__, __LINE__, ##args)
#define QLSPI_WARN(fmt, args...)
#define QLSPI_LOG(fmt, args...)
#define QLSPI_DBG(fmt, args...)
#endif


//KC_CUSTOMIX
#define GPIO_HIGH							(1)
#define GPIO_LOW							(0)

enum MCU_mode {
    MCU_MODE_NORMAL = 0x00,
    MCU_MODE_SLEEP,
    MCU_MODE_NO_SLEEP,
    MCU_MODE_BOOT,				// Firmware download pending
    MCU_MODE_MAX
};

enum EOS_status {
	CLIENT_INITIALIZING = -5,
	FW_DOWNLOADING = -4,
	NOT_INITIALIZE = -3,
	RECOVERING = -2,
	DEAD = -1,
	NORMAL = 0
};

enum SPI_FREQ_ID{
	SPI_FREQ_LOW_ID =0,
    SPI_FREQ_HIGH_ID =1,
};


struct SPIDEV_DATA {

	struct spi_device	*spi;

	struct device* dev;

	u32			speed_hz;

	u32			speed_high_hz;

	u32			speed_low_hz;

	struct work_struct work;

	int mcu_mode;

	struct mutex mutex_spi_cs;

#ifdef  EN_CS_CONTROL
	int32_t cs_gpio;
#endif

	int32_t irq_gpio;

#ifdef SW_INTR1_GPIO

	int32_t	irq_out_gpio;

#endif
    int32_t rst_gpio;
    int32_t fabric_rdy_gpio;

	int irq_no;

	struct kobject *kobj;

};

extern struct spi_device *spi_slave_dev;

extern struct work_struct fwdl_work;
extern struct completion boot_done_wait;
extern struct completion power_done_wait;
extern struct completion firstdl_done_wait;
extern uint8_t firstdl_retry_cnt;

#ifndef OPT_INTR

void en_intr_from_s3(void) ;
void dis_intr_from_s3(void) ;

#endif

int reg_isr(struct SPIDEV_DATA *spi_priv);
int qlspi_set_boot_mode(unsigned char mode);
int qlspi_get_boot_mode(void);
void en_intr_to_s3(void) ;
unsigned char read_devid(void);
void qlspi_ope_wake_irq(bool enable);


int ql_mutex_init(void);
void ql_mutex_lock(void);
void ql_mutex_unlock(void);

#ifdef SPI_SCALING_EN
	int qlspi_change_freq(enum SPI_FREQ_ID freq_id);
	int set_spi_freq(uint32_t freq);
#endif

int tlc_reg_read( unsigned int addr, u8 *readBuf, unsigned int length);
int shub_read_dma_status(uint8_t* buf);

int get_S3toAP_Value(void);
#ifdef SW_INTR1_GPIO
int get_APtoS3_value(void);
void set_APtoS3_value(uint8_t value);
#endif
void ql_spi_any_device_init(void);
bool ql_spi_get_micon_status(void);
uint8_t ql_spi_get_shutdown_status(void);
enum EOS_status ql_spi_get_eoss3_status(void);
void ql_spi_set_eoss3_status(enum EOS_status status);
int32_t micon_dev_write(uint32_t adr, const uint8_t *data, uint8_t size);
int32_t micon_dev_read(uint32_t adr, uint8_t *data, uint16_t size);
void ql_spi_power_offon(void);
void ql_spi_micon_recovery_wakelock(void);
void ql_spi_pm_stay_awake(void);
void ql_spi_pm_relax(void);
int qlspi_get_fab_dl_errcount(void);
void qlspi_fab_dl_errcount_inc(void);

#endif /* __QL_SPI_DRIVER_H */
