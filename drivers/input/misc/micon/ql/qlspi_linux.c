/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2018 KYOCERA Corporation
 * (C) 2019 KYOCERA Corporation
 */
/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/*
	QLI Linux SPI slave driver
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>
#include <linux/irq_work.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/atomic.h>
#include <linux/ktime.h>

#include <linux/ql/qlspi_linux.h>
#include <linux/ql/qlspi.h>
#include <linux/ql/qlspi_linux_plat.h>
#include <linux/ql/qlmcu.h>
#include <linux/ql/qleos_s3.h>
#include <linux/ql/qleos_common.h>
#include <linux/ql/model_file_loader.h>

#include "qlpinctrl.h"
#include "qlpower.h"
#include <linux/ql/qlmcu.h>
#include <linux/ql/qlpower_ext.h>

extern int reset_slave_mcu(void);

#ifdef USE_OWN_WQ
static struct workqueue_struct *ql_wq;
#endif

static ktime_t recovery_start_time;
static ktime_t recovery_curr_time;
wait_queue_head_t bootdone_wait_int;

#define SPI_RESUME_RETRY_NUM 300
#define SPI_RETRY_NUM 5
#define WAKELOCK_TIME_MICON_RECOVERY    (15000) //15000msec
#define FWDL_RETRY_NUM  3

static int spi_parse_dt(struct device *dev,struct SPIDEV_DATA *spi_priv);
static void ql_spi_sensor_dev_probe(struct work_struct *probe_work);
static void ql_spi_any_device_init_work(struct work_struct *devinit_work);
static ssize_t ql_fw_version_show(struct device *dev,struct device_attribute *attr,char *buf);
static ssize_t ql_host_cmd_store(struct device *dev,struct device_attribute *attr,const char *buf,size_t count );
static ssize_t ql_host_cmd_show(struct device *dev, struct device_attribute *attr,char *buf );
static ssize_t ql_reg_read_show(struct device *dev,struct device_attribute *attr,char *buf);
static ssize_t ql_reg_read_store(struct device *dev, struct device_attribute *attr,const char *buf,size_t count );
static ssize_t ql_reg_write_store(struct device *dev, struct device_attribute *attr,const char *buf,size_t count );
static ssize_t ql_reg_write_show(struct device *dev,struct device_attribute *attr,char *buf);
static ssize_t ql_status_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t ql_status_store(struct device *dev, struct device_attribute *attr,const char *buf,size_t count );
static ssize_t fwdl_sequence_show(struct device *dev, struct device_attribute *attr, char *buf);
#ifdef READ_CPU_PROC

#include <linux/uaccess.h>
#include <linux/syscalls.h>



struct file *cpu_freq_file_handle[NO_OF_CPUFREQ_PROCS]={0};

 struct file *openFile(char *path,int flag);
 void closeFile(struct file *fp);
 int readFile(struct file *fp,char *buf,int readlen) ;
 int get_file_size(struct file* fp);

char *cpu_freq_filename[]={	"/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_cur_freq",	"/sys/devices/system/cpu/cpu1/cpufreq/cpuinfo_cur_freq",	"/sys/devices/system/cpu/cpu2/cpufreq/cpuinfo_cur_freq",	"/sys/devices/system/cpu/cpu3/cpufreq/cpuinfo_cur_freq"};

#endif

extern atomic_t g_interrupt_done;

/* Combined A0 to A8 and also dont send dummy byte  */
#define COMBINE_A0_A8

static int qlspi_read_test;
static int qlspi_write_test;
static int qlspi_s3_intr;

#ifdef QLSPI_REGRW_SYSFS_ENTRY

static int qlspi_wr;
static int qlspi_rd;

uint32_t qlspi_addr=0x20000000,qlspi_value=0;

#ifdef QLSPI_REGRW_SYSFS_ENTRY
static int qlspi_tlc_wr;
static int qlspi_tlc_rd;

uint32_t qlspi_tlc_addr=SPITLC_DEVICE_ID_BYTE,qlspi_tlc_value=0;
#endif
#endif

//static struct mutex mutex_spi_internal;
static DEFINE_MUTEX(mutex_ap2s3_ope);
static DEFINE_MUTEX(lock_qlspi_apis);

static spinlock_t gpio_irq_lock;
volatile int is_spi_intr = 0;


struct spi_device *spi_slave_dev;

#ifdef DMA_FIFO_UNDERFLOW_EMULATE
//extern int spi_underflow_flag;
int spi_underflow_flag;
#endif

#ifdef OPT_AHB_WR

//static bool prev_op_ahb_w=0;

#endif

#define CREATE_CMD(operation, addr) (unsigned char) ((operation << OPER_BIT_LOC) | (addr & ADDR_MASK))


QL_Status trigger_isr_handler (void *data);
void dump_tlc_reg(uint8_t addr);
void tlc_dump(void);

uint8_t cmd_response[40];
uint8_t result[8];
static int len;

uint8_t firstdl_retry_cnt = 0;
atomic_t is_enable_irq;
static atomic_t s_ShutdownDoneFlg = ATOMIC_INIT(false);
static atomic_t eoss3_is_resume = ATOMIC_INIT(false);
static atomic_t fabric_dl_errcnt = ATOMIC_INIT(0);
static atomic_t is_firmware_chk_error = ATOMIC_INIT(false);
static int inline __get_INT_gpio_value(void);
#ifdef SW_INTR1_GPIO
static int inline __get_intr_1_gpio_value(void);
static void inline __set_intr_1_gpio_value(int value);
#endif
static ssize_t set_command_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count);
static ssize_t set_command_show(struct device *dev,
	struct device_attribute *attr, char *buf);

static ssize_t get_command_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count);
static ssize_t get_command_show(struct device *dev,
	struct device_attribute *attr, char *buf);

static ssize_t set_get_command_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count);
static ssize_t set_get_command_show(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t model_file_write_sq(struct device *dev,
	struct device_attribute *attr, char *buf);

static void ql_spi_any_dev_remove(struct spi_device *client);
static void ql_spi_any_dev_suspend(struct device *dev );
static void ql_spi_any_dev_resume(struct device *dev);
static int ql_spi_suspend( struct device *dev );
static int ql_spi_resume( struct device *dev );
static void ql_spi_any_dev_shutdown(struct spi_device *client);
static int ql_spi_remove(struct spi_device *spi);
static void ql_spi_shutdown( struct spi_device *spi );

struct completion boot_done_wait;
struct completion power_done_wait;
struct completion firstdl_done_wait;
static int8_t eoss3_status = DEAD;
#ifdef CONFIG_PSEUDO_SALEAE_LOG
#define PSEUDO_INT_TAG "pseudo[INT] : "
#define PSEUDO_SPI_TAG "pseudo[SPI] : "
#define PSEUDO_INT_LOG(fmt, args...) \
	pr_info(PSEUDO_INT_TAG fmt "\n", ##args)
#define PSEUDO_SPI_LOG(fmt, args...) \
	pr_info(PSEUDO_SPI_TAG fmt "\n", ##args)
#define STATUS_DUMMY_VAL	-1
enum pseudo_log_type {
	TYPE_APtoS3 = 0,
	TYPE_S3toAP,
	TYPE_SPI_MISO,
	TYPE_SPI_MOSI
};
static int8_t cur_APtoS3_sts = GPIO_LOW;
static int8_t cur_S3toAP_sts = GPIO_LOW;
static int8_t prv_APtoS3_sts = GPIO_LOW;
static int8_t prv_S3toAP_sts = GPIO_LOW;
static uint32_t spi_logger_pktID = 0;
#endif /*CONFIG_PSEUDO_SALEAE_LOG*/

#define SPI_ACCESS_SPEED_MODE_LOW   (0x00)
#define SPI_ACCESS_SPEED_MODE_HIGH  (0x01)
#define SPI_ACCESS_SPEED_LOW        (640000)
#define SPI_ACCESS_SPEED_HIGH       (19200000)

/*********************** FUNCTIONS DEFINITIONS ***********************************/

#ifdef CONFIG_PSEUDO_SALEAE_LOG
void set_previous_data(void){
	prv_APtoS3_sts = cur_APtoS3_sts;
	prv_S3toAP_sts = cur_S3toAP_sts;
}

void set_new_current_data(const enum pseudo_log_type type, uint8_t value)
{
	ENTERED();
	switch (type)
	{
		case TYPE_APtoS3:
			cur_APtoS3_sts = value;
			break;
		case TYPE_S3toAP:
			cur_S3toAP_sts = value;
			break;
		default:
			QLSPI_ERR("INVALID pseudo_log_type[0x%02x]", type);
			break;
	}
	EXITED();
}

void record_pseudo_INT_log(const enum pseudo_log_type type, uint8_t new_value)
{
	ENTERED();
	set_previous_data();
	set_new_current_data(type, new_value);
	PSEUDO_INT_LOG("%d,%d", prv_APtoS3_sts, prv_S3toAP_sts);
	PSEUDO_INT_LOG("%d,%d", cur_APtoS3_sts, cur_S3toAP_sts);
	EXITED();
}
#endif /*CONFIG_PSEUDO_SALEAE_LOG*/

int ql_mutex_init()
{
	ENTERED();
	EXITED();

	return QL_STATUS_OK;
}

void ql_mutex_lock()
{
	ENTERED();
	mutex_lock(&lock_qlspi_apis);
	EXITED();
}

void ql_mutex_unlock()
{
	ENTERED();
	if(mutex_is_locked(&lock_qlspi_apis)) {
		mutex_unlock(&lock_qlspi_apis);
	}
	EXITED();
}

static int inline __get_INT_gpio_value(void)
{
	struct SPIDEV_DATA* spi_priv=spi_get_drvdata(spi_slave_dev);
	ENTERED();
	EXITED();
	return gpio_get_value(spi_priv->irq_gpio);
}

int get_S3toAP_Value(void)
{
	ENTERED();
	EXITED();
	return __get_INT_gpio_value();
}

uint8_t ql_spi_get_shutdown_status(void)
{
	ENTERED();
	EXITED();
    return !!(uint8_t)atomic_read(&s_ShutdownDoneFlg);
}


#define MAX_RECOVERING_TIME	(15 * 1000 * 1000)	/* 15sec */

enum EOS_status ql_spi_get_eoss3_status(void)
{
	s64 diff_time_us = 0;
	ENTERED();
	if(eoss3_status == DEAD){
		return eoss3_status;
	}
	if (eoss3_status == RECOVERING) {
		recovery_curr_time = ktime_get();
		diff_time_us = ktime_to_us(ktime_sub(recovery_curr_time, recovery_start_time));
		if ((diff_time_us >= MAX_RECOVERING_TIME) || (diff_time_us < 0)){
			QLSPI_ERR("Max recovering time exceeded. diff_time_us:[%lld]", diff_time_us);
			eoss3_status = DEAD;
		}
	}
	EXITED();
	return eoss3_status;
}

void ql_spi_set_eoss3_status(enum EOS_status status)
{
	ENTERED();
	if ((status == RECOVERING) && ((eoss3_status != RECOVERING) && (eoss3_status != DEAD))) {
		recovery_start_time = ktime_get();
	}
	eoss3_status = status;
	EXITED();
}

#ifdef SW_INTR1_GPIO
static int inline __get_intr_1_gpio_value(void)
{
	struct SPIDEV_DATA* spi_priv=spi_get_drvdata(spi_slave_dev);

	ENTERED();
	EXITED();
	return gpio_get_value(spi_priv->irq_out_gpio);
}

static void inline __set_intr_1_gpio_value(int value)
{
	struct SPIDEV_DATA* spi_priv=spi_get_drvdata(spi_slave_dev);
	QLSPI_DBG(" Operate GPIO AP to S3 => %s \n",((value==GPIO_HIGH)? "HIGH":"LOW"));
	gpio_set_value(spi_priv->irq_out_gpio, value);
#ifdef CONFIG_PSEUDO_SALEAE_LOG
	record_pseudo_INT_log(TYPE_APtoS3, value);
#endif /*CONFIG_PSEUDO_SALEAE_LOG*/
	QLSPI_DBG("End\n");
}

int get_APtoS3_value(void)
{
	ENTERED();
	EXITED();
	return __get_intr_1_gpio_value();
}

void set_APtoS3_value(uint8_t value)
{
	ENTERED();
	EXITED();
	__set_intr_1_gpio_value((int)value);
}

#endif

#ifdef CONFIG_QL_SPI_CTRL_FROMCLIENT
static void qlspi_access_request(void)
{
	ENTERED();
    qlpinctrl_select(QLPIN_SPI_MICON_CS_REQUEST);
    qlpinctrl_select(QLPIN_SPI_ACTIVE);
	EXITED();
}

static void qlspi_access_free(void)
{
	ENTERED();
    qlpinctrl_select(QLPIN_SPI_SUSPEND);
    qlpinctrl_select(QLPIN_CS_ACTIVE);
	EXITED();
}
#endif /* CONFIG_QL_SPI_CTRL_FROMCLIENT */
int shub_spi_write( unsigned char cmd, const u8 *data, unsigned int len)
{

#ifndef OPT_SPI_SINGLE_TXRX

	struct spi_message m;
	struct spi_transfer t[1];
	u8* tx_buf;
	int err = 0;

#if defined(SPI_DBG_W)||defined(CONFIG_PSEUDO_SALEAE_LOG) 
	int i=0;
#endif

	struct SPIDEV_DATA* spi_priv=spi_get_drvdata(spi_slave_dev);

	ENTERED();
	//mutex_lock(&spi_priv->mutex_spi_cs);

	tx_buf=os_malloc(1+len);

	if(tx_buf==NULL)
	{
		QLSPI_ERR(" SPI tx_buf allocation failed \n");

		//mutex_unlock(&spi_priv->mutex_spi_cs);

		return -1;
	}

#ifdef CONFIG_PSEUDO_SALEAE_LOG
	//RegAddr
	QLSPI_DBG("Write[MOSI] Log");
	PSEUDO_SPI_LOG("%d,0x%02x,0x00",spi_logger_pktID, (uint8_t)cmd);
	//MOSI LOG
	for(i=0; i<len;i++){
		PSEUDO_SPI_LOG("%d,0x%02x,0x00", spi_logger_pktID, (uint8_t)data[i]);
	}
	++spi_logger_pktID;
#endif /*CONFIG_PSEUDO_SALEAE_LOG*/

	spi_message_init(&m);

	memset(&t[0], 0, sizeof t[0]);

	memcpy(&tx_buf[0],&cmd,1);
	memcpy(&tx_buf[1],data,len);

	t[0].tx_buf = tx_buf;
	t[0].rx_buf =NULL;
	t[0].len = len+1;
	t[0].bits_per_word = 8;
	t[0].speed_hz = spi_priv->speed_hz;

	spi_message_add_tail(&t[0], &m);

#ifdef EN_CS_CONTROL

	#ifdef READ_CPU_PROC
	QLSPI_DBG("WR CS_LOW\n");
	#endif

	gpio_set_value(spi_priv->cs_gpio, 0);

#endif
#ifdef CONFIG_QL_SPI_CTRL_FROMCLIENT
    qlspi_access_request();
#endif
	err = spi_sync(spi_priv->spi, &m);

#ifdef CONFIG_QL_SPI_CTRL_FROMCLIENT
    qlspi_access_free();
#endif
#ifdef EN_CS_CONTROL

	gpio_set_value(spi_priv->cs_gpio, 1);

#endif


#ifdef SPI_DBG_W

			QLSPI_DBG(" Transfer TX :");

			for(i=0;i<len+1;i++)
			{
				QLSPI_DBG(" 0x%x ",tx_buf[i]);
			}
		/*
			QLSPI_DBG(" Transfer RX\n");

			for(i=0;i<len+1;i++)
			{
				QLSPI_DBG(" 0x%x ",rx_buf[i]);
			}
		*/

		QLSPI_DBG(" \n");
#endif
	os_free(tx_buf);

	//mutex_unlock(&spi_priv->mutex_spi_cs);

	EXITED();
	return err;

#else


	struct spi_message m;
	struct spi_transfer t[2];
	int err = 0;

#ifdef SPI_DBG
	int i=0;
#endif

	struct SPIDEV_DATA* spi_priv=spi_get_drvdata(spi_slave_dev);

	ENTERED();
	//mutex_lock(&spi_priv->mutex_spi_cs);

	spi_message_init(&m);

	memset(&t[0], 0, sizeof t[0]);
	memset(&t[1], 0, sizeof t[1]);

	t[0].tx_buf = &cmd;
	t[0].len = sizeof(cmd);
	t[0].bits_per_word = 8;
	t[0].speed_hz = spi_priv->speed_hz;

	spi_message_add_tail(&t[0], &m);

	t[1].tx_buf = data;
	t[1].len = len;
	t[1].bits_per_word = 8;
	t[1].speed_hz = spi_priv->speed_hz;

#ifdef SPI_DBG

	QLSPI_DBG(" MOSI \n");
	QLSPI_DBG(" 0x%x , ",cmd);

	for(i=0;i<len;i++)
	{
		QLSPI_DBG(" 0x%x ",data[i]);
	}
	QLSPI_DBG(" \n");
#endif

	spi_message_add_tail(&t[1], &m);

#ifdef EN_CS_CONTROL

	gpio_set_value(spi_priv->cs_gpio, 0);

#endif

	err = spi_sync(spi_priv->spi, &m);

#ifdef EN_CS_CONTROL

	gpio_set_value(spi_priv->cs_gpio, 1);

#endif
	//mutex_unlock(&spi_priv->mutex_spi_cs);

	EXITED();
	return err;
#endif

}


int shub_spi_read( unsigned char reg_addr, u8 *data, unsigned int len)
{

#ifdef OPT_SPI_SINGLE_TXRX
	struct spi_message m;

	struct spi_transfer t[1];

	unsigned char* tx_buf,*rx_buf;

	unsigned char add_tx_buf[3] = {0};			//addr byte , dummy , dummy

	int err = 0;

	struct SPIDEV_DATA*  spi_priv=spi_get_drvdata(spi_slave_dev);

#ifdef SPI_DBG
	int i=0;
#endif

	ENTERED();
	//mutex_lock(&spi_priv->mutex_spi_cs);

	spi_message_init(&m);

	tx_buf=os_malloc(3+len);
	rx_buf=os_malloc(3+len);

	if(tx_buf==NULL||rx_buf==NULL)
	{
		QLSPI_ERR(" SPI tx,rx buf allocation failed \n");

		//mutex_unlock(&spi_priv->mutex_spi_cs);

		return -1;
	}


	memset(&t[0], 0, sizeof t[0]);

	add_tx_buf[0] = reg_addr;

	memcpy(tx_buf,add_tx_buf,sizeof(add_tx_buf));

	t[0].tx_buf = tx_buf;
	t[0].rx_buf = rx_buf;
	t[0].len = len+3;
	t[0].bits_per_word = 8;
	t[0].speed_hz = spi_priv->speed_hz;

	spi_message_add_tail(&t[0], &m);

#ifdef EN_CS_CONTROL

	gpio_set_value(spi_priv->cs_gpio, 0);

#endif
#ifdef CONFIG_QL_SPI_CTRL_FROMCLIENT
    qlspi_access_request();
#endif
	err = spi_sync(spi_priv->spi, &m);

#ifdef CONFIG_QL_SPI_CTRL_FROMCLIENT
    qlspi_access_free();
#endif

#ifdef EN_CS_CONTROL

	gpio_set_value(spi_priv->cs_gpio, 1);

#endif

	memcpy(data,&rx_buf[3],len);


#ifdef SPI_DBG

		QLSPI_DBG(" Transfer RX\n");

		for(i=0;i<len+3;i++)
		{
			QLSPI_DBG(" 0x%x ",rx_buf[i]);
		}

		QLSPI_DBG(" \n");
#endif
	os_free(rx_buf);
	os_free(tx_buf);

	//mutex_unlock(&spi_priv->mutex_spi_cs);

	EXITED();
	return err;

#else
	struct spi_message m;
	struct spi_transfer t[2];

	unsigned char add_tx_buf[3] = {0};			//addr byte , dummy , dummy

	int err = 0;

	struct SPIDEV_DATA*  spi_priv=spi_get_drvdata(spi_slave_dev);

#if defined(SPI_DBG_W)||defined(CONFIG_PSEUDO_SALEAE_LOG) 

	int i=0;
#endif

	ENTERED();
	//mutex_lock(&spi_priv->mutex_spi_cs);

	spi_message_init(&m);

	memset(&t[0], 0, sizeof t[0]);
	memset(&t[1], 0, sizeof t[1]);

	add_tx_buf[0] = reg_addr;

	t[0].tx_buf = add_tx_buf;
	t[0].len = sizeof(add_tx_buf);
	t[0].bits_per_word = 8;
	t[0].speed_hz = spi_priv->speed_hz;


#ifdef SPI_DBG

		QLSPI_DBG(" MOSI\n");

		for(i=0;i<t[0].len;i++)
		{
			QLSPI_DBG(" 0x%x ",add_tx_buf[i]);
		}
		QLSPI_DBG(" \n");
#endif

	spi_message_add_tail(&t[0], &m);

	t[1].rx_buf = data;
	t[1].len = len;
	t[1].bits_per_word = 8;
	t[1].speed_hz = spi_priv->speed_hz;

	spi_message_add_tail(&t[1], &m);

#ifdef EN_CS_CONTROL

	gpio_set_value(spi_priv->cs_gpio, 0);

#endif
	err = spi_sync(spi_priv->spi, &m);

#ifdef EN_CS_CONTROL

	gpio_set_value(spi_priv->cs_gpio, 1);

#endif

#ifdef SPI_DBG

		QLSPI_DBG("MISO \n");

		for(i=0;i<len;i++)
		{
			QLSPI_DBG(" 0x%x ",data[i]);
		}
		QLSPI_DBG("\n");

#endif

#ifdef CONFIG_PSEUDO_SALEAE_LOG
	//RegAddr
	QLSPI_DBG("Read[MISO] Log");
	PSEUDO_SPI_LOG("%d,0x%02x,0x00", spi_logger_pktID, (uint8_t)reg_addr);
	//MISTERIOUS 2Byte Data Recode
	PSEUDO_SPI_LOG("%d,0x00,0x00", spi_logger_pktID);
	PSEUDO_SPI_LOG("%d,0x00,0x00", spi_logger_pktID);
	//MISO LOG
	for(i=0; i<len;i++){
		PSEUDO_SPI_LOG("%d,0x00,0x%02x", spi_logger_pktID, (uint8_t)data[i]);
	}
	++spi_logger_pktID;
#endif /*CONFIG_PSEUDO_SALEAE_LOG*/

	//mutex_unlock(&spi_priv->mutex_spi_cs);

	EXITED();
	return err;

#endif

}

#ifdef SPI_SCALING_EN

int qlspi_change_freq(enum SPI_FREQ_ID freq_id)
{

	struct SPIDEV_DATA*  spi_priv=spi_get_drvdata(spi_slave_dev);
  ENTERED();
  if(freq_id== SPI_FREQ_LOW_ID )
    	spi_priv->speed_hz = spi_priv->speed_low_hz;
	else if(freq_id == SPI_FREQ_HIGH_ID )
    	spi_priv->speed_hz = spi_priv->speed_high_hz;

  QLSPI_DBG("QLSPI Freq changed to %d\n", spi_priv->speed_hz);
  EXITED();

	return 0;
}
EXPORT_SYMBOL_GPL(qlspi_change_freq);
#endif

static bool check_spi_resumed(void)
{
	int8_t i;
	for(i = 0; i < SPI_RESUME_RETRY_NUM; i++){
		if(atomic_read(&eoss3_is_resume)){
			break;
		} else {
			QLSPI_DBG("not resumed yet. retry[%d]", i);
			usleep_range(10000, 10100);
		}
	}
	if(i == SPI_RESUME_RETRY_NUM){
		QLSPI_ERR("end return[EBUSY]");
		return -EBUSY;
	}
	return SPI_STATUS_OK;
}

static int tlc_reg_read_proc( unsigned char cmd, u8 *readBuf, unsigned int length)
{
	int8_t i;
	int32_t ret;
	for(i = 0; i < SPI_RETRY_NUM; i++){
		ret = shub_spi_read( cmd, readBuf, length);
		if (ret == 0){
			ret = SPI_STATUS_OK;
			break;
		} else if((ret == -EBUSY) || (ret == -ETIMEDOUT) || (ret == -EINPROGRESS) || ((ret == -EIO) && (i < SPI_RETRY_NUM - 1))) {
			QLSPI_ERR("fault sns_spi_write_proc()-->ret[%d] RETRY[%d]",ret ,i );
			msleep(100);
		} else {
			QLSPI_ERR("-TLC- SPI Read Error!!\n");
			ret = SPI_STATUS_ERR;
			break;
		}
	}
	return ret;
}

int tlc_reg_read( unsigned int addr, u8 *readBuf, unsigned int length)
{
	unsigned char cmd = 0;
	int32_t ret;

	ENTERED();

	cmd = CREATE_CMD(QLULPSH_CMD_READ, addr);
	if(check_spi_resumed() == SPI_STATUS_OK){
		ret = tlc_reg_read_proc(cmd, readBuf, length);
	} else {
		QLSPI_ERR("SPI does not resumed!!\n");
		ret = -EBUSY;
	}

	EXITED();
	return ret;
}

static int tlc_reg_write_proc(unsigned char cmd, u8 *writeBuf, unsigned int length)
{
	int32_t ret;
	int8_t i;
	for(i = 0; i < SPI_RETRY_NUM; i++){
		ret = shub_spi_write(cmd, writeBuf, length);
		if(ret == 0) {
			ret = SPI_STATUS_OK;
			break;
		} else if((ret == -EBUSY) || (ret == -ETIMEDOUT) || (ret == -EINPROGRESS) || ((ret == -EIO) && (i < SPI_RETRY_NUM - 1))) {
			QLSPI_ERR("fault sns_spi_write_proc()-->ret[%d] RETRY[%d]",ret ,i );
			msleep(100);
		} else {
			QLSPI_ERR("SPI write Other error (H/W Reset ON)");
			ret = SPI_STATUS_ERR;
			break;
		}
	}
	return ret;
}

int tlc_reg_write(unsigned int addr, u8 *writeBuf, unsigned int length)
{
	unsigned char cmd = 0;
	int32_t ret;

	ENTERED();
	cmd = CREATE_CMD(QLULPSH_CMD_WRITE, addr);
	if(check_spi_resumed() == SPI_STATUS_OK){
		ret = tlc_reg_write_proc(cmd, writeBuf, length);
	} else {
		QLSPI_ERR("SPI does not resumed!!\n");
		ret = -EBUSY;
	}

	EXITED();
	return ret;
}



 //Trigger interrupt to S3

QL_Status ql_spi_trigger_intr(void)
{

#ifdef SW_INTR1_GPIO

	//struct SPIDEV_DATA* spi_priv=spi_get_drvdata(spi_slave_dev);

	ENTERED();
	set_APtoS3_value(GPIO_HIGH);
	//gpio_set_value(spi_priv->irq_out_gpio,1);
	//gpio_set_value(spi_priv->irq_out_gpio,0);

	//QLSPI_DBG(" Sent SW1 INTR1  via GPIO 2\n");

	EXITED();
	return QL_STATUS_OK;

#else

  int ret = 0;

  unsigned int trigger_int = 0x1;

  ENTERED();
  trigger_int = 0x1;

  ret = shub_ahb_write( SW_INTR_1_REG, (uint8_t *)(&trigger_int), 4);

    if(ret < 0){
        QLSPI_ERR(" Error writing of SW_INTR1_REG , ret %d\n",ret);

			return QL_STATUS_ERROR;
    }

	EXITED();
	return QL_STATUS_OK;
#endif

}

void dump_tlc_reg(uint8_t addr)
{
	uint8_t buff=0;
	QL_Status sts;

	QLSPI_DBG("Start\n");
	buff=0;

	sts=tlc_reg_read((uint32_t)addr, (uint8_t*)&buff, 1);
	if(sts!=QL_STATUS_OK)
	{
		QLSPI_ERR(" Reg 0x%x FAILED\n",addr);
		return;
	}

	QLSPI_DBG(" Reg 0x%x  = 0x%08x \n",addr,buff);
}

void tlc_dump()
{
	QLSPI_DBG(" ====================\n");

	dump_tlc_reg(SPITLC_AHB_STATUS);
	dump_tlc_reg(SPITLC_AHB_ACCESS_CTL);

	QLSPI_DBG(" Mem addr bytes 3-0\n");
	dump_tlc_reg(SPITLC_MEM_ADDR_BYTE_3);
	dump_tlc_reg(SPITLC_MEM_ADDR_BYTE_2);
	dump_tlc_reg(SPITLC_MEM_ADDR_BYTE_1);
	dump_tlc_reg(SPITLC_MEM_ADDR_BYTE_0);

	QLSPI_DBG(" Mem data bytes 3-0\n");

	dump_tlc_reg(SPITLC_MEM_DATA_BYTE_3);
	dump_tlc_reg(SPITLC_MEM_DATA_BYTE_2);
	dump_tlc_reg(SPITLC_MEM_DATA_BYTE_1);
	dump_tlc_reg(SPITLC_MEM_DATA_BYTE_0);

	QLSPI_DBG(" DMA addr bytes 3-0\n");

	dump_tlc_reg(SPITLC_DMA_ADDR3);
	dump_tlc_reg(SPITLC_DMA_ADDR2);
	dump_tlc_reg(SPITLC_DMA_ADDR1);
	dump_tlc_reg(SPITLC_DMA_ADD0);

	QLSPI_DBG(" DMA burst size1, 0 \n");
	dump_tlc_reg(SPITLC_DMA_BRUST_SIZE1);
	dump_tlc_reg(SPITLC_DMA_BRUST_SIZE0);

	QLSPI_DBG(" ====================\n");

}

void dump_reg(uint32_t addr)
{
	uint32_t buff=0;
	QL_Status sts;

	QLSPI_DBG("Start\n");
	buff=0;

	sts=QLSPI_Read_S3_Mem(addr, (uint8_t*)&buff, 4);
	if(sts!=QL_STATUS_OK)
	{
		QLSPI_ERR(" Reg 0x%x FAILED\n",addr);
		return;
	}

	QLSPI_DBG(" Reg 0x%x  = 0x%08x \n",addr,buff);
}


static ssize_t qlspi_read_check(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct SPIDEV_DATA* spi_priv=spi_get_drvdata(spi_slave_dev);
	int ret=0;

	unsigned char devid=0;

#ifdef	AHB_READ_TEST

	uint32_t reg=0;

	uint32_t data=0;

	int status = SPI_STATUS_OK;
#endif

	QLSPI_DBG("Start\n");
	qlspi_read_test = 0;

	if(tlc_reg_read(SPITLC_DEVICE_ID_BYTE ,&devid,1)!=SPI_STATUS_OK)
	{
		return sprintf(buf, " Could not read Device ID \n");
	}

	QLSPI_DBG("  Device ID 0x%x \n",devid);

	if(devid!=EOSS3_SPI_DEVID)
	{
		QLSPI_ERR(" Device ID read FAILED \n");

		return sprintf(buf, "Device ID read FAILED \n");
	}

	dump_reg(SW_INTR_1_REG);
	dump_reg(SW_INTR_1_EN_AP_REG);
	dump_reg(SW_INTR_1_EN_M4);

	dump_reg(SW_INTR_2_REG);
	dump_reg(SW_INTR_2_EN_AP_REG);
	dump_reg(SW_INTR_2_EN_M4_REG);


#ifdef FABRIC_LOADER

	QLSPI_DBG(" Fabric INTR CTRL regs \n");

	dump_reg(FABRIC_INTR_STS_REG);
	dump_reg(FABRIC_INTR_STS_REG_RAW);
	dump_reg(FABRIC_INTR_TYPE);
	dump_reg(FABRIC_INTR_POL);
	dump_reg(FABRIC_INTR_EN_AP);

#endif

#ifdef AHB_READ_TEST
	QLSPI_DBG(" Starting AHB read test \n");

	tlc_dump();

	reg=0x40004000;

	QLSPI_DBG(" Reading M4 reg %x \n",reg);

	data=0;

	dump_reg(reg);
	status = shub_ahb_read(reg,(uint8_t*)&data,4);
	if(status != SPI_STATUS_OK)
	{
		QLSPI_ERR(" ### M4 reg AHB read %x failed ,data %x \n",reg,data);

		tlc_dump();
	
		handle_shub_ahb_status(status);
	}
	QLSPI_DBG(" %x = %x \n",reg,data);


	QLSPI_DBG(" Reading AON reg \n");

	//reg=0x20060000;
	reg=0x2006d000;

	dump_reg(reg);

	data=0;
	status = shub_ahb_read(reg,(uint8_t*)&data,4);
	if(status != SPI_STATUS_OK)
	{
		QLSPI_ERR(" ### AON reg %x , AHB read failed ,data %x \n",reg,data);

		tlc_dump();
	
		handle_shub_ahb_status(status);
	}
	QLSPI_DBG("  %x = %x \n",reg,data);

	QLSPI_DBG(" Reading SRAM reg \n");

	reg=0x20000000;

	dump_reg(reg);

	data=0;
	status = shub_ahb_read(reg,(uint8_t*)&data,4);
	if(status != SPI_STATUS_OK)
	{
		QLSPI_ERR(" ### SRAM reg %x , AHB read failed ,data %x \n",reg,data);

		tlc_dump();
	
		handle_shub_ahb_status(status);
	}
	QLSPI_DBG("  %x = %x \n",reg,data);




#endif


	ret=gpio_get_value(spi_priv->irq_gpio);
	QLSPI_DBG(" S3toAP intr line sts %d \n",ret);


	return sprintf(buf, " qlspi read test SUCCESS \n");

}

#define TEST_SIZE		(15)	//16	//(QL_SPI_WRITE_MAX_LEN)

uint8_t aucbuff[TEST_SIZE] = {0};
//uint32_t test_addr= (0x20060000)	;	//0x20000000;
uint32_t test_addr= (0x2006d000)	;	//0x20000000;

static ssize_t qlspi_write_check(struct device *dev, struct device_attribute *attr, char *buf)
{

	int i=0;


	ENTERED();

		qlspi_write_test = 0;

		for(i=0;i<TEST_SIZE;i++)
		{
			aucbuff[i]=i;

		}

		QLSPI_DBG(" Starting write at %x\n",test_addr);
		QLSPI_Write_S3_Mem(test_addr,(uint8_t*)aucbuff,TEST_SIZE);
		QLSPI_DBG(" Write done size %d B \n",TEST_SIZE);

		memset(aucbuff,0,TEST_SIZE);

		QLSPI_DBG(" Starting Read \n");
		QLSPI_Read_S3_Mem(test_addr,(uint8_t*)aucbuff,TEST_SIZE);
		QLSPI_DBG(" Read done size %d B \n",TEST_SIZE);

		for(i=0;i<TEST_SIZE;i++)
		{
			if(aucbuff[i]!=(i&0xFF))
			{
				QLSPI_ERR(" Verify fail %x = %x\n",i,aucbuff[i]);
				return sprintf(buf, " qlspi Write test FAILED\n");

			}
		}


		EXITED();



	EXITED();
	return sprintf(buf, " qlspi Write test SUCCESS \n");

}

//AP to M4 intr check

static ssize_t qlspi_s3_intr_check(struct device *dev, struct device_attribute *attr, char *buf)
{

#ifndef SW_INTR1_GPIO
	unsigned int reg_clr_val ;

	QLSPI_DBG("Start\n");

	qlspi_s3_intr = 0;

	reg_clr_val = 0;

	ql_spi_trigger_intr();

	if(QLSPI_Read_S3_Mem(SW_INTR_1_REG, (unsigned char *)&reg_clr_val, 4) < SPI_STATUS_OK)
	QLSPI_ERR(" Read SW INTR 1 (AP2M4) reg FAILED \n");

	QLSPI_DBG(" SW_INTR1 after trigger = %x \n",reg_clr_val);
#else

//struct SPIDEV_DATA* spi_priv=spi_get_drvdata(spi_slave_dev);

	QLSPI_DBG("Start\n");

	qlspi_s3_intr = 0;

	QLSPI_Trigger_Intr();


#endif

	QLSPI_DBG("End\n");
	return sprintf(buf, "\n Sent INTR to M4 \n");

}

static ssize_t set_command_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	char *cbuf = kstrdup(buf, GFP_KERNEL);
	char *tok, *end = cbuf;
	uint32_t data[40];
	int i = 0, j = 0;
	uint8_t param[30] = {0};
	int data_bytes = 0;

	ENTERED();

	while (end != NULL) {
		tok = strsep(&end, " ");
		if (kstrtouint(tok, 0, &data[i++])) {
			QLSPI_ERR("Invalid data\n");
			return 0;
		}
	}

	for (j = 0; j < i; j++) {
		QLSPI_DBG("data[%d] 0x%x\n", j, data[j]);
	}

	for (j = 0; j < data[1]; j++) {
		param[j] = data[j+2];
	}

	memset(result, 0, 8);
	data_bytes = data[1];
	set_command(data[0], data_bytes, param, result);

	EXITED();
	return count;
}

static ssize_t set_command_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int i = 0;
	int count = 0;

	ENTERED();
	for (i = 0; i < 8; i++)
		count += sprintf(&buf[count], "0x%02x\t", result[i]);

	count += sprintf(&buf[count], "\n");

	EXITED();
	return count;
}

static ssize_t get_command_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	uint32_t command = 0;
	int data_bytes = 0;
	ENTERED();

	sscanf(buf,"%x %d",&command, &len);
	QLSPI_DBG("command = 0x%x\t len = %d\n", command, len);

#ifdef DMA_FIFO_UNDERFLOW_EMULATE
	spi_underflow_flag = 0;
	if (command == 0x0F0F0F) {
		if (len == 1) {
			QLSPI_DBG("+4 bytes");
			spi_underflow_flag = 1;
			len = 16;
		} if (len == 2) {
			spi_underflow_flag = 2;
			QLSPI_DBG("-4 bytes");
			len = 16;
		}
	}
#endif
	data_bytes = len;
	get_command(command, data_bytes, cmd_response);

	EXITED();
	return count;
}

static ssize_t get_command_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int i = 0;
	int count = 0;

	ENTERED();
	for (i = 0; i < (len + 8); i++)
		count += sprintf(&buf[count], "0x%02x\t", cmd_response[i]);

	count += sprintf(&buf[count], "\n");

	EXITED();
	return count; //sprintf(buf, "%u\n", cmd_response[0]);
}

static ssize_t set_get_command_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	char *cbuf = kstrdup(buf, GFP_KERNEL);
	char *tok, *end = cbuf;
	uint32_t data[40] = {0};
	int i = 0, j = 0;
	uint8_t param[30] = {0};
	int data_bytes = 0, res_bytes = 0;

	ENTERED();
	while (end != NULL) {
		tok = strsep(&end, " ");
		if (kstrtouint(tok, 0, &data[i++])) {
			QLSPI_ERR("Invalid data\n");
			return 0;
		}
	}

	len = data[i-1];

	for (j = 0; j < i; j++) {
		QLSPI_DBG("data[%d] 0x%x\n", j, data[j]);
	}

	for (j = 0; j < data[1]; j++) {
		param[j] = data[j+2];
	}
	data_bytes = data[1];
	res_bytes = len;

	set_get_command(data[0], data_bytes, param, res_bytes, cmd_response);

	EXITED();
	return count;
}

static ssize_t set_get_command_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ENTERED();
	EXITED();
	return get_command_show(NULL, NULL, buf);
}

#ifdef QLSPI_REGRW_SYSFS_ENTRY

static ssize_t qlspi_s3_wr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 wr=0;

	qlspi_wr=0;

	QLSPI_DBG(" WR Buffer is %s \n",buf);

	sscanf(buf,"%x%x%x",&qlspi_addr,&qlspi_value,&wr);

	QLSPI_DBG(" Reg addr %x , value %x \n",qlspi_addr,qlspi_value);

	if(1==wr)
	{

		if(QLSPI_Write_S3_Mem(qlspi_addr,(uint8_t*)&qlspi_value,4)!=QL_STATUS_OK)
		{
			QLSPI_DBG(" APP Write op FAILED \n");
		}

		QLSPI_DBG("APP Write op DONE\n");

	}
	else
	{
		QLSPI_DBG(" Read address stored \n");
	}

	return count;

}


static ssize_t qlspi_s3_rd(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef AHB_READ_TEST
	int status = SPI_STATUS_OK ;
#endif
	qlspi_rd=0;
	QLSPI_DBG(" Buffer is %s \n",buf);

	qlspi_value=0;
	if(QLSPI_Read_S3_Mem(qlspi_addr,(uint8_t*)&qlspi_value,4)!=QL_STATUS_OK)
	{
		sprintf(buf, "\n Read op FAILED \n");
	}
	QLSPI_DBG(" Reg read value = 0x%x\n",qlspi_value);

#ifdef AHB_READ_TEST

	qlspi_value=0;
	status = shub_ahb_read(qlspi_addr,(uint8_t*)&qlspi_value,4);
	if(status != QL_STATUS_OK)
	{
		sprintf(buf, "\n Read op FAILED \n");
		handle_shub_ahb_status(status);
	}
	QLSPI_DBG(" NORMAL AHB READ Reg read value = 0x%x\n",qlspi_value);

#endif

	return	sprintf(buf, "0x%x\n",qlspi_value);


}
#ifdef QLSPI_REGRW_SYSFS_ENTRY
static ssize_t qlspi_s3_tlc_wr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 wr=0;

	qlspi_tlc_wr=0;

	QLSPI_DBG(" WR Buffer is %s \n",buf);

	QLSPI_DBG(" Count is %d \n",(int)count);

	sscanf(buf,"%x%x%x",&qlspi_tlc_addr,&qlspi_tlc_value,&wr);

	qlspi_tlc_addr&=0xFF;
	qlspi_tlc_value&=0xFF;

	QLSPI_DBG(" Reg addr %x , value %x \n",qlspi_tlc_addr,qlspi_tlc_value);

	if(1==wr)
	{

		if(tlc_reg_write(qlspi_tlc_addr,(uint8_t*)&qlspi_tlc_value,ONE_BYTE)!=SPI_STATUS_OK)
		{
			QLSPI_DBG(" APP Write op FAILED \n");
		}

		QLSPI_DBG("APP Write op DONE\n");

	}
	else
	{
		QLSPI_DBG(" Read address stored \n");
	}

	return count;

}


static ssize_t qlspi_s3_tlc_rd(struct device *dev, struct device_attribute *attr, char *buf)
{
	qlspi_tlc_rd=0;

	QLSPI_DBG(" Buffer is %s \n",buf);

	qlspi_tlc_value=0;

	QLSPI_DBG(" Reg addr %x , value %x \n",qlspi_tlc_addr,qlspi_tlc_value);

	if(tlc_reg_read(qlspi_tlc_addr,(uint8_t*)&qlspi_tlc_value,ONE_BYTE)!=SPI_STATUS_OK)
	{
		sprintf(buf, "\n Read op FAILED \n");
	}
	QLSPI_DBG(" Reg read value = 0x%x\n",qlspi_tlc_value);

	return	sprintf(buf, "0x%x\n",qlspi_tlc_value);
}
#endif

#endif

static struct work_struct pwork;
static struct work_struct devinit_work;
static struct delayed_work power_ready_work;
#define SPI_SCALE_ENABLED	(1)
#define SPI_SCALE_DISABLED	(0)

static void ql_spi_sensor_dev_probe(struct work_struct *pwork)
{
    static bool is_1st_time_this_proc = true;
    struct SPIDEV_DATA *spi_priv=spi_get_drvdata(spi_slave_dev);
    QLSPI_LOG("func ql_spi_sensor_dev_probe() START");
    sensor_ql_micon_probe(spi_priv->spi);
    /* Temporary processing because reset_save_and_taskoff() is not implemented. */
    if(is_1st_time_this_proc){
        is_1st_time_this_proc = false;
    } else {
        if (sensor_ql_micon_load_param() != 0) {
            QLSPI_ERR("micon parameter load error.\n");
        }
    }
    complete(&boot_done_wait);
    firstdl_retry_cnt = 0;
    QLSPI_DBG("complete!! (pass the wait_for_completion_interruptible_timeout in fwdl_sequence_show())\n");
    QLSPI_LOG("END ql_spi_sensor_dev_probe\n");
}

void ql_spi_micon_recovery_wakelock(void)
{
    struct SPIDEV_DATA *spi_priv=spi_get_drvdata(spi_slave_dev);
    pm_wakeup_event(spi_priv->dev, WAKELOCK_TIME_MICON_RECOVERY);
    QLSPI_ERR("wakelock 15sec for micon recovery.");
}

void ql_spi_pm_stay_awake(void)
{
    struct SPIDEV_DATA *spi_priv=spi_get_drvdata(spi_slave_dev);
    pm_stay_awake(spi_priv->dev);
}

void ql_spi_pm_relax(void)
{
    struct SPIDEV_DATA *spi_priv=spi_get_drvdata(spi_slave_dev);
    pm_relax(spi_priv->dev);

}

static int firmware_chk(void)
{
	uint8_t reg_wdata = 0x03;
	struct hc_info hc;
	int ret = 0;
	QLSPI_LOG("firmware_chk() START");

	QLSPI_DBG(" RegAdr=[0x%x],Data=[0x%x]\n", SPITLC_AHB_ACCESS_CTL, reg_wdata);
	if(tlc_reg_write(SPITLC_AHB_ACCESS_CTL,&reg_wdata,ONE_BYTE) != SPI_STATUS_OK)
	{
		QLSPI_ERR("Reg Write Error!! SPITLC_AHB_ACCESS_CTL(0x30)\n");
		ret = -1;
		goto exit;
	}
	//HC:scale change
	hc.command = prepare_host_command(SYSTEM, SYS_SPI_SCALING, SYS_SET_SPI_SCALE_FREQ);
	hc.param_data_len = 1;
	hc.param_data[0] = SPI_SCALE_ENABLED;
	hc.response_len = 0;
	ret = qleoss3_hostcmd(&hc);
	if(ret){
		QLSPI_ERR("SPI Scale change Error!!\n");
		ret = -1;
		goto exit;
	}

	//crccheck
	ret = check_crc_error();
	if(ret){
		QLSPI_ERR("CRC check Failed \n");
		return -1;
	}
exit:
	QLSPI_LOG("firmware_chk() END[%d]", ret);
	return ret;
}

static int dummy_spi_sync_locked(void)
{
	unsigned char cmd=0;

	struct spi_message m;
	struct spi_transfer t[1];
	int err = 0;

	struct SPIDEV_DATA* spi_priv=spi_get_drvdata(spi_slave_dev);

	ENTERED();
	spi_message_init(&m);

	memset(&t[0], 0, sizeof t[0]);

	t[0].tx_buf = &cmd;
	t[0].len = sizeof(cmd);
	t[0].bits_per_word = 8;
	t[0].speed_hz = spi_priv->speed_hz;

	spi_message_add_tail(&t[0], &m);

	err = spi_sync_locked(spi_priv->spi, &m);

	if(err<0)
	{
		QLSPI_ERR("%s spi_sync_locked failed, ret %d \n", __func__, err);
		return SPI_STATUS_ERR;
	}

	EXITED();
	return SPI_STATUS_OK;
}

int32_t micon_dev_write(uint32_t adr, const uint8_t *data, uint8_t size)
{
	int32_t ret = SPI_STATUS_OK;
	QLSPI_LOG("%s: START", __func__);

	if (eoss3_status == NORMAL) {
		ret = QLSPI_Write_S3_Mem(adr, (void*)data, size);
		if( (ret == SPI_STATUS_ERR_AHB) || (ret == SPI_STATUS_ERR_DMA)){
			hw_reboot_exec();
			return SPI_STATUS_ERR;
		}
	} else {
		QLSPI_ERR("The status of eoss3 is invalid. status = [%d]\n", eoss3_status);
		ret = SPI_STATUS_ERR;
	}

	QLSPI_LOG("%s: END", __func__);
	return ret;
}

int32_t micon_dev_read(uint32_t adr, uint8_t *data, uint16_t size)
{
	int32_t ret = SPI_STATUS_OK;
	QLSPI_LOG("%s: START", __func__);

	if (eoss3_status == NORMAL) {
		ret = QLSPI_Read_S3_Mem(adr, (void*)data, size);
	    if( (ret == SPI_STATUS_ERR_AHB) || (ret == SPI_STATUS_ERR_DMA)){
	        hw_reboot_exec();
	        return SPI_STATUS_ERR;
	    }
	} else {
		QLSPI_ERR("The status of eoss3 is invalid. status = [%d]\n", eoss3_status);
		ret = SPI_STATUS_ERR;
	}

	QLSPI_LOG("%s: END", __func__);
	return ret;
}

void ql_spi_power_offon(void)
{
	struct SPIDEV_DATA *spi_priv=spi_get_drvdata(spi_slave_dev);
	QLSPI_LOG("%s: START", __func__);

	spi_bus_lock(spi_priv->spi->master);
	qlpower_off();
    usleep_range(10000, 10100);
	qlpower_on();
	spi_bus_unlock(spi_priv->spi->master);
	//schedule_delayed_work(&power_ready_work, msecs_to_jiffies(500));
	msleep(500);
	spi_priv->mcu_mode = MCU_MODE_BOOT;
	QLSPI_LOG("%s: END", __func__);
}

int ql_spi_power_reset(void)
{
	struct SPIDEV_DATA *spi_priv=spi_get_drvdata(spi_slave_dev);
	QLSPI_LOG("%s: START", __func__);

	qlpower_off_cbfunc();
	spi_bus_lock(spi_priv->spi->master);
	dummy_spi_sync_locked();
	qlpower_reset();
	spi_bus_unlock(spi_priv->spi->master);
	qlpower_on_cbfunc();

	QLSPI_LOG("%s: END", __func__);
	return SPI_STATUS_OK;
}
EXPORT_SYMBOL_GPL(ql_spi_power_reset);

static void ql_spi_power_ready_work(struct work_struct *work)
{
	QLSPI_LOG("%s: START", __func__);
	complete(&power_done_wait);
	QLSPI_LOG("%s: END", __func__);
}

static int reset_sequence_exec(void)
{
	uint8_t reg_wdata = 0x03;
	int ret = SPI_STATUS_OK;
	struct SPIDEV_DATA *spi_priv = spi_get_drvdata(spi_slave_dev);

	QLSPI_LOG("%s: START", __func__);

	qlpinctrl_select(QLPIN_CS_ACTIVE);
	qlspi_change_freq(SPI_FREQ_LOW_ID);

	QLSPI_DBG("[Reset] RegAdr=[0x%x], Data=[0x%x]\n", SPITLC_AHB_ACCESS_CTL, reg_wdata);
	ret = tlc_reg_write(SPITLC_AHB_ACCESS_CTL, &reg_wdata, ONE_BYTE);
	if (ret != SPI_STATUS_OK)
	{
		QLSPI_ERR("[Reset] Reg Write Error!! SPITLC_AHB_ACCESS_CTL(0x30) [%d]\n", ret);
		return SPI_STATUS_ERR;
	}

	ret = reset_slave_mcu();
	if (ret != QL_STATUS_OK) {
		QLSPI_ERR("[Reset] reset_slave_mcu Error!! [%d]", ret);
		return SPI_STATUS_ERR;
	}

	usleep_range(1000, 2000);
	qlpinctrl_select(QLPIN_RST_SUSPEND);

	spi_bus_lock(spi_priv->spi->master);
	qlpinctrl_select(QLPIN_CS_SUSPEND);
	usleep_range(1000, 2000);
	qlpinctrl_select(QLPIN_CS_RESET);
	usleep_range(3000, 4000);
	qlpinctrl_select(QLPIN_RST_ACTIVE);
	usleep_range(3000, 4000);
	qlpinctrl_select(QLPIN_CS_ACTIVE);
	spi_bus_unlock(spi_priv->spi->master);

	QLSPI_DBG("[Reset] RegAdr=[0x%x], Data=[0x%x]\n", SPITLC_AHB_ACCESS_CTL, reg_wdata);
	ret = tlc_reg_write(SPITLC_AHB_ACCESS_CTL, &reg_wdata, ONE_BYTE);
	if (ret != SPI_STATUS_OK)
	{
		QLSPI_ERR("[Reset] Reg Write Error!! SPITLC_AHB_ACCESS_CTL(0x30) [%d]\n", ret);
		return SPI_STATUS_ERR;
	}

#ifdef CMOS_CLK_INPUT
	ret = initialize_s3_host_interface();
	if (ret != QL_STATUS_OK)
	{
		QL_TRACE_MCU_ERROR("Initialize S3 Host Interface Failed \n");
		return QL_STATUS_ERROR;
	}
#endif
	qlspi_change_freq(SPI_FREQ_HIGH_ID);

	QLSPI_LOG("%s: END", __func__);

	return SPI_STATUS_OK;
}

#define TMP_BUFFER_SIZE_MAX	(64)

void fwdl_sequence_when_recovering(void)
{
	char tmp_buf[TMP_BUFFER_SIZE_MAX] = {0};
	QLSPI_DBG("start");
	fwdl_sequence_show(NULL, NULL, tmp_buf);
	QLSPI_DBG("end");
}

#define BOOT_FWDLDONE_WAITTIME	(msecs_to_jiffies(10000))
#define POWER_DONE_WAITTIME	(msecs_to_jiffies(1000))
#define FIRSTDL_DONE_WAITTIME	(msecs_to_jiffies(3000))

int qlspi_get_fab_dl_errcount(void)
{
    return atomic_read(&fabric_dl_errcnt);
}
EXPORT_SYMBOL_GPL(qlspi_get_fab_dl_errcount);

void qlspi_fab_dl_errcount_inc(void)
{
    atomic_inc(&fabric_dl_errcnt);
}
EXPORT_SYMBOL_GPL(qlspi_fab_dl_errcount_inc);

#ifdef FABRIC_LOADER
static FAB_Status get_fabric_status(void)
{
    int fab_errcnt = qlspi_get_fab_dl_errcount();
    return (fab_errcnt < ALLOW_FABDL_COUNT) ? FAB_NORMAL : FAB_DEAD;
}
#endif /* FABRIC_LOADER */

static bool is_fabric_ok(void)
{
    bool result = true;
#ifdef FABRIC_LOADER
    struct SPIDEV_DATA* spi_priv=spi_get_drvdata(spi_slave_dev);
    int fabric_state;

    QLSPI_DBG("start");
    if(get_fabric_status() == FAB_DEAD){
        QLSPI_ERR("Fabric Status is DEAD. skip FabricReady Line check.");
        return true;
    }
    fabric_state = gpio_get_value(spi_priv->fabric_rdy_gpio);
    if(fabric_state == GPIO_LOW){
        msleep(500);
        fabric_state = gpio_get_value(spi_priv->fabric_rdy_gpio);
        if(fabric_state == GPIO_LOW){
            qlspi_fab_dl_errcount_inc();
            result = false;
        }
    }
    QLSPI_DBG("Fabric Ready State [%s]",
                (fabric_state == 1) ? "HIGH" : "LOW");
    QLSPI_DBG("end");
#endif
    return result;
}

static void fwdl_retry_pre_sequence(void)
{
	QLSPI_DBG("start");
	qlspi_disable_irq();
	ql_spi_power_offon();
	reset_sequence_exec();
	QLSPI_DBG("end");
}

static void fwdl_client_init_OKNG_determination(int timeleft)
{
	static uint8_t fwchkerr_num = 0;

    QLSPI_DBG("start");
	if((timeleft <= 0) || (fwchkerr_num >= FWDL_RETRY_NUM)){
		QLSPI_ERR("[FWDL] boot sequence and any device initialization is timedout[%d]", timeleft);
		ql_spi_set_eoss3_status(DEAD);
		QLSPI_ERR("cannot use eoss3 due to device error");
	} else {
		if( atomic_read(&is_firmware_chk_error) && (fwchkerr_num < FWDL_RETRY_NUM) ){
			QLSPI_ERR("[FWDL] fwdl check error. EOSS3 reboot after 5sec.");
			fwchkerr_num++;
			hw_reboot_exec();
		} else {
			QLSPI_DBG("[FWDL] FirmwareDownload and device initialization end. [%d]", timeleft);
			ql_spi_set_eoss3_status(NORMAL);
			fwchkerr_num = 0;
		}
	}
    QLSPI_DBG("end");
}

static int fwdl_sequence_core(void)
{
	int failed_num = 0;
	long timeleft = 0;
    bool fabric_rdy;
	int ret = SPI_STATUS_OK;

	QLSPI_DBG("start");

	while(failed_num < FWDL_RETRY_NUM){
		qlspi_enable_irq();
		QLSPI_DBG("[FWDL] START SensorFWDL[num=%d]\n", failed_num);
		reinit_completion(&firstdl_done_wait);
		if (mcu_fw_download()) {
			QLSPI_ERR("[FWDL] firmaware download failed. Retry!\n");
			goto retry;
		}
		QLSPI_DBG("[FWDL] Wait for interrupt...");
		timeleft = wait_for_completion_timeout(&firstdl_done_wait, FIRSTDL_DONE_WAITTIME);
		fabric_rdy = is_fabric_ok();
		if ( (timeleft <= 0) || !fabric_rdy) {
			QLSPI_ERR("[FWDL] No response. Retry! (timeleft:%ld)", timeleft);
retry:
			fwdl_retry_pre_sequence();
			failed_num++;
		} else {
			QLSPI_DBG("[FWDL] Done.");
			break;
		}
	}
	if( unlikely(failed_num >= FWDL_RETRY_NUM) ){
		QLSPI_ERR("[FWDL]expired firmware download retry num. device is dead.");
		ret = SPI_STATUS_ERR;
	}

	QLSPI_DBG("end [%d]", ret);
	return ret;
}

static ssize_t fwdl_sequence_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	long timeleft = 0;
	int8_t current_status = ql_spi_get_eoss3_status();
	QLSPI_LOG("[FWDL] fwdl_sequence_show() START");

	if(current_status == DEAD){
		QLSPI_ERR("[FWDL] EOSS3 is dead. end.");
		return sprintf(buf, "eoss3 is dead.\n");
	}
	else if (current_status == NORMAL){
		return sprintf(buf, "Already done.\n");
	}

	QLSPI_DBG("[FWDL] Wait for power done completion...");
	timeleft = wait_for_completion_timeout(&power_done_wait, POWER_DONE_WAITTIME);
	if(timeleft <= 0){
		QLSPI_ERR("[FWDL] no completion of power_done_wait[%ld]", timeleft);
	}

	QLSPI_DBG("[FWDL] START ResetSEQ\n");
	ql_spi_set_eoss3_status(FW_DOWNLOADING);
	if (reset_sequence_exec() != SPI_STATUS_OK) {
		QLSPI_ERR("[FWDL] reset sequence failed\n");
		ql_spi_set_eoss3_status(DEAD);
		return sprintf(buf, "reset seq failed\n");
	}

	if( fwdl_sequence_core() != SPI_STATUS_OK ){
		QLSPI_ERR("[FWDL] Error. device is dead.");
		ql_spi_set_eoss3_status(DEAD);
		return sprintf(buf, "no interrupt\n");
	}

	//wait for initializing client driver.
	QLSPI_DBG("[FWDL] Wait for completion...");
	ql_spi_pm_stay_awake(); //keeping awake until client initializing process ended.
	timeleft = wait_for_completion_timeout(&boot_done_wait, BOOT_FWDLDONE_WAITTIME);
	fwdl_client_init_OKNG_determination(timeleft);
	ql_spi_pm_relax();
	QLSPI_LOG("[FWDL] END SensorFWDL\n");

	return sprintf(buf, "fwdl end\n");
}

static ssize_t model_file_write_sq(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t eos_status;
	int32_t ret = QL_STATUS_OK;

	QLSPI_DBG("[MDWR] start.");
	eos_status = ql_spi_get_eoss3_status();
	if((eos_status == DEAD) && (eos_status == NOT_INITIALIZE)){
		ret = QL_STATUS_ERROR;
		QLSPI_ERR("EOS S3 cannot use.");
		goto dlend;
	}

	ret = start_model_file_loader();
dlend:
	QLSPI_DBG("[MDWR] end. result[%s]",
				((ret == QL_STATUS_OK) ? "SUCCESS":"FAILURE"));
	return sprintf(buf, "mddl end [%s]\n", (ret == QL_STATUS_OK) ? "SUCCESS":"FAILURE");
}


static void ql_spi_any_device_init_work(struct work_struct *devinit_work)
{
	ENTERED();
	//Check Firmware download result.
	if(firmware_chk()) {
		QLSPI_ERR("FW download error.");
		atomic_set(&is_firmware_chk_error, true);
		complete(&boot_done_wait);
		return;
	}
	ql_spi_set_eoss3_status(CLIENT_INITIALIZING);
	//sensor probe
	schedule_work(&pwork);
	atomic_set(&is_firmware_chk_error, false);
	EXITED();
}

void ql_spi_any_device_init(void)
{
	QLSPI_LOG("ql_spi_any_device_init() START");
	schedule_work(&devinit_work);
	QLSPI_LOG("ql_spi_any_device_init() END");
}

static DEVICE_ATTR(qlspi_read_test, S_IRUGO, qlspi_read_check, NULL);
static DEVICE_ATTR(qlspi_write_test, S_IRUGO, qlspi_write_check, NULL);
static DEVICE_ATTR(qlspi_s3_intr, S_IRUGO, qlspi_s3_intr_check, NULL);
static DEVICE_ATTR(set_command,
		S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		set_command_show,
		set_command_store
);
static DEVICE_ATTR(get_command,
		S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		get_command_show,
		get_command_store
);
static DEVICE_ATTR(set_get_command,
		S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		set_get_command_show,
		set_get_command_store
);

#ifdef QLSPI_REGRW_SYSFS_ENTRY
static DEVICE_ATTR(qlspi_rd, 0660, qlspi_s3_rd,NULL );
static DEVICE_ATTR(qlspi_wr, 0660, NULL, qlspi_s3_wr);

static DEVICE_ATTR(qlspi_tlc_rd, 0660, qlspi_s3_tlc_rd,NULL );
static DEVICE_ATTR(qlspi_tlc_wr, 0660, NULL, qlspi_s3_tlc_wr);

#endif

static DEVICE_ATTR(fwdl, S_IRUGO, fwdl_sequence_show, NULL);
static DEVICE_ATTR(mf_wrsq, S_IRUGO, model_file_write_sq, NULL);


static DEVICE_ATTR(fw_version,S_IRUSR|S_IRGRP,ql_fw_version_show,NULL);
static DEVICE_ATTR(host_cmd,  S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,ql_host_cmd_show,ql_host_cmd_store);
static DEVICE_ATTR(reg_write,S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,ql_reg_write_show,ql_reg_write_store);
static DEVICE_ATTR(reg_read,S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,ql_reg_read_show,ql_reg_read_store);
static DEVICE_ATTR(status,S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,ql_status_show,ql_status_store);

static struct attribute *spi_attributes[] = {

	&dev_attr_qlspi_read_test.attr,
	&dev_attr_qlspi_write_test.attr,
	&dev_attr_qlspi_s3_intr.attr,
	&dev_attr_set_command.attr,
	&dev_attr_get_command.attr,
	&dev_attr_set_get_command.attr,
#ifdef QLSPI_REGRW_SYSFS_ENTRY
	&dev_attr_qlspi_rd.attr,
	&dev_attr_qlspi_wr.attr,

	&dev_attr_qlspi_tlc_rd.attr,
	&dev_attr_qlspi_tlc_wr.attr,

#endif
	&dev_attr_fwdl.attr,
	&dev_attr_mf_wrsq.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_host_cmd.attr,
	&dev_attr_reg_write.attr,
	&dev_attr_reg_read.attr,
	&dev_attr_status.attr,
    NULL,
};

static struct attribute_group attr_group = {
		    .attrs = spi_attributes,
};

static int spi_parse_dt(struct device *dev,struct SPIDEV_DATA *spi_priv)
{
	struct device_node *np = dev->of_node;

	int ret = 0;
	int error_chk = 0;
//	void* ptr;

	QLSPI_DBG("Start\n");

	ret = of_get_named_gpio(np, "qlspi,irq-gpio", 0);
	if (ret < 0) {
		QLSPI_ERR("failed to get \"qlspi,irq-gpio\"\n");
		goto err;
	}

	QLSPI_DBG("Got the irq gpio %d\n", ret);

	spi_priv->irq_gpio = ret;

	ret = of_property_read_u32(np, "spi-max-frequency", &spi_priv->speed_high_hz);
	if (ret < 0) {
		QLSPI_ERR("missing \"spi-max-frequency, ret %d\"\n", ret);
		spi_priv->speed_high_hz = SPI_ACCESS_SPEED_HIGH;
	}
	QLSPI_DBG("spi high speed from dev tree %d \n", spi_priv->speed_high_hz);

	ret = of_property_read_u32(np, "spi-min-frequency", &spi_priv->speed_low_hz);
	if (ret < 0) {
		QLSPI_ERR("missing \"spi-min-frequency, ret %d\"\n", ret);
		spi_priv->speed_low_hz = SPI_ACCESS_SPEED_LOW;
	}
	QLSPI_DBG("spi low speed from dev tree %d \n", spi_priv->speed_low_hz);

	// default AP side SPI Frequency is set to 1 MHz
	// client has to request to raise this frequency
	// via a command through the sensor driver.
	spi_priv->speed_hz=1000000;


#ifdef EN_CS_CONTROL

	ret = of_get_named_gpio(np, "qlspi,cs-gpio", 0);
	if (ret < 0) {
		QLSPI_ERR("failed to get \"qlspi,cs-gpio\"\n");
		goto err;
	}

	spi_priv->cs_gpio= ret;

	QLSPI_DBG("Got the chipselect gpio %d\n", ret);

#endif


#ifdef SW_INTR1_GPIO

	ret = of_get_named_gpio(np, "qlspi,irq-out-gpio", 0);
	if (ret < 0) {
		QLSPI_ERR("failed to get \"qlspi,irq-out-gpio\"\n");
		goto err;
	}

	spi_priv->irq_out_gpio= ret;

	QLSPI_DBG("Got the irq_out_gpio %d\n", ret);

#endif

	ret = of_get_named_gpio(np, "qlspi,rst-gpio", 0);
	if (ret < 0) {
		QLSPI_ERR("failed to get \"qlspi,rst-gpio\"\n");
		goto err;
	}

	spi_priv->rst_gpio= ret;
	error_chk = gpio_request(spi_priv->rst_gpio, "qlspi,reset_gpio");
	if (error_chk < 0) {
		QLSPI_ERR("Reset GPIO (%d) req failed %d\n", spi_priv->rst_gpio, error_chk);
	}

	QLSPI_DBG("Got the rst_gpio %d\n", ret);

#ifdef FABRIC_LOADER
	ret = of_get_named_gpio(np, "qlspi,fabrdy-gpio", 0);
	if (ret < 0) {
		QLSPI_ERR("failed to get \"qlspi,fabrdy-gpio\"\n");
		goto err;
	}

	spi_priv->fabric_rdy_gpio= ret;
	QLSPI_DBG("Got the fabric_ready_gpio %d\n", ret);
#endif

err:
	return ret;
}

int qlspi_get_boot_mode(void)
{
	struct SPIDEV_DATA *spi_priv=spi_get_drvdata(spi_slave_dev);

	ENTERED();
	EXITED();
	return (MCU_MODE_BOOT == spi_priv->mcu_mode)?1:0;
}
EXPORT_SYMBOL_GPL(qlspi_get_boot_mode);


int qlspi_set_boot_mode(unsigned char mode)
{
	struct SPIDEV_DATA *spi_priv=spi_get_drvdata(spi_slave_dev);

	ENTERED();
	if(mode>=MCU_MODE_MAX)
	{
		QLSPI_ERR(" Invalid mcu mode %d  \n",mode);
		return QL_STATUS_ERROR;
	}
	spi_priv->mcu_mode=mode;

	if(MCU_MODE_NORMAL==mode)
	{
#ifdef USE_OWN_WQ
				//queue_work(ql_wq,&spi_priv->work);
#else
				//schedule_work(&spi_priv->work);
#endif
	}

	EXITED();
	return QL_STATUS_OK;
}
EXPORT_SYMBOL_GPL(qlspi_set_boot_mode);


static void s3_intr_work(struct work_struct *work)
{

#ifdef READ_CPU_PROC

	uint8_t freq[10]={0};
	mm_segment_t oldfs;
	int cpufreqfil_size=0;
	int i=0;
#endif
#ifdef CHK_SPURIOUS_INTR

	unsigned int reg_clr_val=0;

	unsigned int fabric_intr_sts=0;	

	if(QLSPI_Read_S3_Mem(SW_INTR_2_REG, (unsigned char *)&reg_clr_val, 4) < SPI_STATUS_OK)
	QLSPI_ERR(" SW_INTR2 RD FAILED, %d \n",reg_clr_val);

#ifdef FABRIC_LOADER

	if(QLSPI_Read_S3_Mem(FABRIC_INTR_STS_REG, (unsigned char *)&fabric_intr_sts, 4) < SPI_STATUS_OK)
	QLSPI_ERR(" FB_INTR_STS RD FAILED, %d \n",fabric_intr_sts);

#endif

	reg_clr_val &= 0x1;
	fabric_intr_sts &=0x1;

	if( !(reg_clr_val ) && !(fabric_intr_sts ))
	{
		QLSPI_ERR(" Spurious intr from M4 \n");
	}
	else
	{
		if(reg_clr_val)
		{
			QLSPI_DBG(" M4 INTR %x \n",reg_clr_val);
		}

#ifdef FABRIC_LOADER
		if(fabric_intr_sts )
		{
			QLSPI_DBG(" FABINTR %x \n",fabric_intr_sts);
		}
#endif

#endif

#ifndef OPT_INTR

		QLSPI_LOG("  s3 intr to ap DIS\n");
		dis_intr_from_s3();

#endif


#ifdef READ_CPU_PROC

		QLSPI_DBG(" FREQ_RD_S \n");

		oldfs = get_fs();
		set_fs(KERNEL_DS);

		for(i=0;i<NO_OF_CPUFREQ_PROCS;i++)
		{
			if(cpu_freq_file_handle[i]!=NULL)
			{

				cpufreqfil_size=get_file_size(cpu_freq_file_handle[i]);

				if(readFile(cpu_freq_file_handle[i],freq,cpufreqfil_size)<0)
				{
					//QLSPI_DBG(" cpu %d freq file read freq FAILED\n",i);
					break;
				}

				QLSPI_DBG(" cpu[%d] freq %s \n",i,freq);
			}
			else
			{
				QLSPI_ERR(" cpu_freq_file_handle[%d] is NULL \n",i);
			}
		}
		set_fs(oldfs);
		QLSPI_DBG(" FREQ_RD_E \n");

#endif
	ENTERED();

	if(atomic_read(&s_ShutdownDoneFlg)){
		QLSPI_DBG("end -skip due to shutdown sequence running.");
		return;
	}

		trigger_isr_handler(NULL);

#ifdef CHK_SPURIOUS_INTR

		if(reg_clr_val)
#endif
		{
#ifndef		OPT_DIS_CLR_INT_STS
			clear_intr_sts_s3();
#endif
		}

#ifdef FABRIC_LOADER
#ifdef CHK_SPURIOUS_INTR
		if(fabric_intr_sts)
#endif
		{
			//clear_intr_sts_fabric();
		}
#endif


#ifndef OPT_INTR

	en_intr_from_s3();
	QLSPI_LOG("  s3 intr to ap EN \n");

#endif



#ifdef CHK_SPURIOUS_INTR
	}
#endif

	EXITED();

}

void qlspi_ope_wake_irq(bool enable)
{
    struct SPIDEV_DATA *spi_priv=spi_get_drvdata(spi_slave_dev);
    QLSPI_LOG("start");
    if(enable == true){
        enable_irq_wake(spi_priv->spi->irq);
        QLSPI_LOG("enable_irq_wake");
    } else {
        disable_irq_wake(spi_priv->spi->irq);
        QLSPI_LOG("disable_irq_wake");
    }
    QLSPI_LOG("end");
}
void qlspi_disable_irq(void)
{
	struct SPIDEV_DATA *spi_priv=spi_get_drvdata(spi_slave_dev);

	ENTERED();
	if (atomic_cmpxchg(&is_enable_irq, 1, 0) == 1) {
		disable_irq(spi_priv->spi->irq);
	}
	EXITED();
}
EXPORT_SYMBOL_GPL(qlspi_disable_irq);

void qlspi_enable_irq(void)
{
	struct SPIDEV_DATA *spi_priv=spi_get_drvdata(spi_slave_dev);

	ENTERED();
	if (atomic_cmpxchg(&is_enable_irq, 0, 1) == 0) {
		enable_irq(spi_priv->spi->irq);
	}
	EXITED();
}
EXPORT_SYMBOL_GPL(qlspi_enable_irq);

int qlspi_is_spi_intr_set_zero(void)
{
    QLSPI_LOG("start");

	spin_lock_irq(&gpio_irq_lock);
	is_spi_intr = 0;
	spin_unlock_irq(&gpio_irq_lock);

    QLSPI_LOG("end");
	return 0;
}
EXPORT_SYMBOL_GPL(qlspi_is_spi_intr_set_zero);

int qlspi_check_for_gpio_if_low(void)
{
	struct SPIDEV_DATA *spi_priv=spi_get_drvdata(spi_slave_dev);
	volatile int ret = 0;

	unsigned long timeout;

    QLSPI_LOG("start");

	//spin_lock_irq(&gpio_irq_lock);
	//is_spi_intr = 0;
	//spin_unlock_irq(&gpio_irq_lock);

	timeout = jiffies + msecs_to_jiffies(1000);
#if 1
	do {
		/* While it is checking for gpio to go low,
		 * if interrupt get generated again, break the loop and come out */
		//printk(KERN_ALERT "ql_spi 1\n");
		if (time_after(jiffies, timeout)){
			QLSPI_LOG("end [timeout]");
			return -1;
		}
		//printk(KERN_ALERT "ql_spi 2\n");
		if (is_spi_intr) {
			break;
		}
		//printk(KERN_ALERT "ql_spi 3\n");
		ret = gpio_get_value(spi_priv->irq_gpio);
	} while (ret != 0);
#endif
#ifdef CONFIG_PSEUDO_SALEAE_LOG
	record_pseudo_INT_log(TYPE_S3toAP, GPIO_LOW);
#endif /*CONFIG_PSEUDO_SALEAE_LOG*/
    QLSPI_LOG("end");
	return 0;
}
EXPORT_SYMBOL_GPL(qlspi_check_for_gpio_if_low);


int qlspi_check_for_gpio_if_high(void)
{
	struct SPIDEV_DATA *spi_priv=spi_get_drvdata(spi_slave_dev);
	int ret = 0;

	QLSPI_DBG("Start\n");

	ret = gpio_get_value(spi_priv->irq_gpio);

	QLSPI_DBG("M4->AP GPIO = %d\n",ret);

	return ret;
}
EXPORT_SYMBOL_GPL(qlspi_check_for_gpio_if_high);

int qlspi_reset_dma(void)
{
	uint8_t data = 1;
	uint8_t read_val = 0;
//	uint32_t dummy_data = 0;

	QLSPI_DBG("qlspi_reset_dma\n");

	if (tlc_reg_write(SPITLC_DMA_DEBUG_CTL1, &data, 1) != SPI_STATUS_OK) {
		QLSPI_ERR("TLC write failed\n");
	}
	if (tlc_reg_read(SPITLC_DMA_DEBUG_CTL1, &read_val, 1) != SPI_STATUS_OK) {
		QLSPI_ERR("TLC read failed\n");
	}

	if (tlc_reg_write(SPITLC_DMA_DEBUG_CTL0, &data, 1) != SPI_STATUS_OK) {
		QLSPI_ERR("TLC write failed\n");
	}


	data = 0;
	if (tlc_reg_write(SPITLC_DMA_DEBUG_CTL1, &data, 1) != SPI_STATUS_OK) {
		QLSPI_ERR("TLC write failed\n");
	}

	if (tlc_reg_read(SPITLC_DMA_DEBUG_CTL1, &read_val, 1) != SPI_STATUS_OK) {
		QLSPI_ERR("TLC read failed\n");
	}

	if (tlc_reg_write(SPITLC_DMA_DEBUG_CTL0, &data, 1) != SPI_STATUS_OK) {
		QLSPI_ERR("TLC write failed\n");
	}
#ifdef DMA_FIFO_UNDERFLOW_EMULATE
	spi_underflow_flag = 0;
#endif
	QLSPI_DBG("End\n");
	return 0;
}
EXPORT_SYMBOL_GPL(qlspi_reset_dma);


static irqreturn_t spi_slave_isr(int irq, void *data)
{
	struct SPIDEV_DATA *spi_priv = data;

	QLSPI_DBG("Start\n");
#ifdef CONFIG_PSEUDO_SALEAE_LOG
	record_pseudo_INT_log(TYPE_S3toAP, GPIO_HIGH);
#endif /*CONFIG_PSEUDO_SALEAE_LOG*/
//	if(MCU_MODE_BOOT!=spi_priv->mcu_mode)//will get interrupts only after fw download was completed
//	{
		spin_lock(&gpio_irq_lock);
		is_spi_intr = 1;
		spin_unlock(&gpio_irq_lock);
		QLSPI_DBG("QLSPI_ISR\n");
#ifdef USE_OWN_WQ
		queue_work(ql_wq,&spi_priv->work);
#else
		schedule_work(&spi_priv->work);
#endif
//	}

	QLSPI_DBG("End\n");
	return IRQ_HANDLED;
}

int reg_isr(struct SPIDEV_DATA *spi_priv)
{
	int retval = 0;

	QLSPI_DBG("Start\n");

	//retval = request_irq(spi_priv->spi->irq, spi_slave_isr,IRQF_TRIGGER_RISING|IRQF_NO_THREAD ,"ql_irq", spi_priv);
	retval = request_any_context_irq(spi_priv->spi->irq, spi_slave_isr,IRQF_TRIGGER_RISING|IRQF_NO_THREAD ,"ql_irq", spi_priv);

    if (retval < 0) {
        QLSPI_ERR("Request IRQ %d Fail\n", spi_priv->spi->irq);
		return retval;
    }

#ifdef USE_OWN_WQ

	ql_wq= alloc_workqueue("qlspi_wq", WQ_HIGHPRI | WQ_UNBOUND |
							WQ_MEM_RECLAIM, 1);
	if (!ql_wq) {
		retval = -ENOMEM;
		free_irq(spi_priv->spi->irq,spi_priv);
	}

#endif

	INIT_WORK(&spi_priv->work, s3_intr_work);

	QLSPI_DBG("End\n");
	return retval;

}

static int ql_spi_probe(struct spi_device *spi)
{
	int retval = 0;
	QL_Status qlsts=QL_STATUS_OK;

#ifdef	QLSPI_LINUX_TEST_CODE

	u32 slave_por_addr =0;
	QL_Status  ql_status;

#endif

	struct SPIDEV_DATA *spi_priv;

	struct QLSPI_Platform  spi_plat;

	QLSPI_LOG(" QLSPI slave probe\n");
	eoss3_status = NOT_INITIALIZE;
	atomic_set(&is_enable_irq, 0);

	/* Allocate driver data */

	spi_priv = kzalloc(sizeof(*spi_priv), GFP_KERNEL);
	if (!spi_priv)
	{
		QLSPI_ERR(" Spi slave drv DS alloc FAILED \n");
		return -ENOMEM;
	}

	atomic_set(&is_enable_irq, 1);
	atomic_set(&eoss3_is_resume, true);

    
	//other sensor init start
	//sensor probe
	INIT_WORK(&pwork, ql_spi_sensor_dev_probe);
	INIT_WORK(&devinit_work, ql_spi_any_device_init_work);
	INIT_DELAYED_WORK(&power_ready_work, ql_spi_power_ready_work);

	QLSPI_LOG("  Priv DS allocted \n");

	spi_set_drvdata(spi, spi_priv);

	spi_priv->spi = spi;

	spi_priv->dev=&spi->dev;

	spi_slave_dev=spi;

	spi_priv->mcu_mode =MCU_MODE_BOOT;

	//mutex_init(&spi_priv->mutex_spi_cs);

	//mutex_init(&mutex_spi_internal);

	init_completion(&boot_done_wait);
	init_completion(&power_done_wait);
	init_completion(&firstdl_done_wait);

	QLSPI_LOG("  Mutexes initialized \n");

	retval=spi_parse_dt(&spi->dev,spi_priv);
	if(retval<0)
	{
		QLSPI_ERR(" Getting device tree info FAILED \n");

		goto err_parse_dt;
	}

	QLSPI_LOG("  Device tree read \n");

#ifdef EN_CS_CONTROL

	spi_plat.cs_gpio=spi_priv->cs_gpio;

#endif

	spi_plat.irq_gpio=spi_priv->irq_gpio;

#ifdef SW_INTR1_GPIO

	spi_plat.irq_out_gpio=spi_priv->irq_out_gpio;

#endif

	qlpinctrl_initialize(&spi->dev);
	qlpower_initialize(&spi->dev);
	qlpower_spi_request(QL_SPI_DRV_NAME);

	QLSPI_LOG("  Calling QLSPI init \n");

	qlsts=QLSPI_Init(&spi_plat);
	if(QL_STATUS_OK!=qlsts)
		{
		QLSPI_ERR("  QLSPI init  FAILED\n");
		retval=-EFAULT;
		goto err_qlspi_init_fail;
		}

	QLSPI_DBG("  QLSPI init DONE\n");

	spi_bus_lock(spi_priv->spi->master);
	qlpinctrl_select(QLPIN_CS_SUSPEND);
	//dummy_spi_sync_locked();
	qlpower_on();
	spi_bus_unlock(spi_priv->spi->master);
	schedule_delayed_work(&power_ready_work, msecs_to_jiffies(500));

	spi_priv->mcu_mode = MCU_MODE_BOOT;

#ifdef QLSPI_LINUX_TEST_CODE

msleep(500);

slave_por_addr =0;

ql_status = QLSPI_Read_S3_Mem(MISC_POR_0_ADDR,(uint8_t *)&slave_por_addr, 4);

QLSPI_DBG(" *** Reset reg on probe  %x \n",slave_por_addr);

if (ql_status != QL_STATUS_OK)
{
	QLSPI_ERR("Failure reading reset reg \n");
	goto err_qlspi_init_fail;
}

#endif

//sysfs entry for this driver

	QLSPI_LOG("  Creating sysfs entries for spi slave drv \n");

	spi_priv->kobj = kobject_create_and_add(QLSPI_SYSFS_FOLDER, kernel_kobj);			//  /sys/kernel/QLSPI_SYSFS_FOLDER

	if (!spi_priv->kobj ) {

		QLSPI_ERR("Failure in creating %s sysfs dir \n",QLSPI_SYSFS_FOLDER);
		retval = -EFAULT;
		goto err_kobj_create;
	}

	QLSPI_DBG("  /sys/kernel/%s directory created \n",QLSPI_SYSFS_FOLDER);


	retval = sysfs_create_group(spi_priv->kobj , &attr_group);
	if(retval < 0) {
		QLSPI_ERR("Failure in creating sys entries \n");
		goto err_sysfs_create;
	}

	/* mutex for synchronising between polling gpio and isr */
	spin_lock_init(&gpio_irq_lock);
	device_init_wakeup(spi_priv->dev, 1);
	QLSPI_LOG("  sysfs files created \n");

	QLSPI_LOG(" Registering ISR \n");

	retval = reg_isr(spi_priv);
	if(retval<0)
	{
		QLSPI_ERR(" Registering ISR FAILED\n");
		goto err_reg_isr;
	}

	QLSPI_DBG("  ISR registered \n");

	qlspi_enable_irq();

#ifdef	QLSPI_LINUX_TEST_CODE

	dump_reg(SW_INTR_1_REG);
	dump_reg(SW_INTR_1_EN_AP_REG);
	dump_reg(SW_INTR_1_EN_M4);

	dump_reg(SW_INTR_2_REG);
	dump_reg(SW_INTR_2_EN_AP_REG);
	dump_reg(SW_INTR_2_EN_M4_REG);

#endif

	QLSPI_DBG("  SPI probe completed\n");

	return 0;

	QLSPI_ERR(" QL spi driver probe error \n");


err_reg_isr:
	sysfs_remove_group(spi_priv->kobj ,&attr_group);

err_sysfs_create:

	kobject_put(spi_priv->kobj );

err_kobj_create:

err_qlspi_init_fail:

err_parse_dt:

	QLSPI_ERR("Failure in %s\n", __func__);

	//mutex_destroy(&mutex_spi_internal);

	//mutex_destroy(&spi_priv->mutex_spi_cs);

	spi_slave_dev=NULL;

	kfree(spi_priv);

	return retval;

}

static void ql_spi_any_dev_remove(struct spi_device *client)
{
	ENTERED();
	sensor_ql_micon_remove(client);
	EXITED();
}

static int ql_spi_remove(struct spi_device *spi)
{
	struct SPIDEV_DATA *spi_priv=spi_get_drvdata(spi_slave_dev);

	ENTERED();
	ql_spi_any_dev_remove(spi_priv->spi);
#ifdef READ_CPU_PROC

	int i=0;

	for(i=0;i<NO_OF_CPUFREQ_PROCS;i++)
	{
		if(NULL!=cpu_freq_file_handle[i])
		{
			closeFile(cpu_freq_file_handle[i]);
		}
	}

#endif
	device_init_wakeup(spi_priv->dev, 0);
	disable_irq(spi->irq);



#ifdef USE_OWN_WQ
	destroy_workqueue(ql_wq);
#endif

#ifndef OPT_INTR

	dis_intr_from_s3();

#endif

	free_irq(spi->irq,spi_priv);

	sysfs_remove_group(spi_priv->kobj ,&attr_group);

	kobject_put(spi_priv->kobj );

	//mutex_destroy(&spi_priv->mutex_spi_cs);

	qlpower_remove(&spi->dev);
	qlpinctrl_remove(&spi->dev);

	spi_slave_dev=NULL;

	kfree(spi_priv);

	EXITED();
	return 0;
}

static void spi_scale_ahb_clocks(uint8_t mode)
{
	struct hc_info hc = {0};
	int8_t ret = QL_STATUS_OK;

	QLSPI_LOG("start");

	hc.command = prepare_host_command(SYSTEM,
		SYS_SPI_SCALING, SYS_SET_SPI_SCALE_FREQ);
	hc.param_data_len = 1;
	hc.param_data[0] = (uint32_t)mode;
	hc.response_len = 0;
	QLSPI_DBG("SetSPIScaleAHBClocks Mode[0x%08x]", hc.param_data[0]);
	ret = qleoss3_hostcmd(&hc);

	QLSPI_LOG("end[%d]", ret);
	return;
}

static void ql_spi_any_dev_suspend(struct device *dev)
{
	ENTERED();
	//add suspend sequence of each client here.
	sensor_ql_micon_suspend(dev);
	EXITED();
}
static int ql_spi_suspend( struct device *dev )
{

	ENTERED();
	ql_spi_any_dev_suspend(dev);
	spi_scale_ahb_clocks(SPI_ACCESS_SPEED_MODE_LOW);
	qlspi_change_freq(SPI_FREQ_LOW_ID);
	atomic_set(&eoss3_is_resume, false);
	EXITED();
	return 0;
}

static void ql_spi_any_dev_resume(struct device *dev)
{
	ENTERED();
	//add resume sequence of each client here.
	sensor_ql_micon_resume(dev);
	EXITED();
}

static int ql_spi_precheck_can_resume(void)
{
	unsigned long timeout;
	int8_t ql_state = ql_spi_get_eoss3_status();
	bool outputflg = false;

	ENTERED();
	timeout = jiffies + msecs_to_jiffies(15 * 1000);
	while(ql_state != NORMAL){
		if((ql_state == DEAD) || (ql_state == NOT_INITIALIZE)){
			QLSPI_ERR("Eoss3 is not used[%d].", ql_state);
			return -1;
		}
		QLSPI_ERR("Eoss3 is probably recovering. just a moment...");
		outputflg = true;
		if (time_after(jiffies, timeout)){
			QLSPI_LOG("end [timeout]. change micon state => dead.");
			ql_spi_set_eoss3_status(DEAD);
			return -1;
		}
		msleep(100);
	    ql_state = ql_spi_get_eoss3_status();
	}
	if(outputflg){
		QLSPI_ERR("Go ahead resume seq.");
	}

	EXITED();
	return 0;
}

static int ql_spi_resume( struct device *dev )
{

	ENTERED();
	atomic_set(&eoss3_is_resume, true);
	ql_spi_precheck_can_resume();
	spi_scale_ahb_clocks(SPI_ACCESS_SPEED_MODE_HIGH);
	qlspi_change_freq(SPI_FREQ_HIGH_ID);
	ql_spi_any_dev_resume(dev);
	EXITED();
	return 0;
}

static void ql_spi_any_dev_shutdown(struct spi_device *client)
{
	ENTERED();
	//add shutdown sequence of each client here.
	sensor_ql_micon_shutdown(client);
	EXITED();
}

static void ql_spi_shutdown( struct spi_device *spi )
{
	struct SPIDEV_DATA *spi_priv=spi_get_drvdata(spi_slave_dev);
	ENTERED();
	ql_spi_any_dev_shutdown(spi_priv->spi);
	disable_irq(spi->irq);
    atomic_set(&s_ShutdownDoneFlg, true);
	if(ql_wq != NULL){
		destroy_workqueue(ql_wq);
		ql_wq = NULL;
	}
	atomic_set(&eoss3_is_resume, false);
	qlpower_spi_release(QL_SPI_DRV_NAME);
	EXITED();
}


EXPORT_SYMBOL_GPL(QLSPI_Init);
EXPORT_SYMBOL_GPL(QLSPI_Read_S3_Mem);
EXPORT_SYMBOL_GPL(QLSPI_Write_S3_Mem);
EXPORT_SYMBOL_GPL(QLSPI_Register_Isr );
EXPORT_SYMBOL_GPL(QLSPI_Trigger_Intr );
EXPORT_SYMBOL_GPL(QLSPI_Clear_S3_Intr );


static struct of_device_id ql_spi_match_table[] = {
	{ .compatible = "qlspi,qlspi_eoss3"},
	{ },
};

static const struct spi_device_id ql_spi_id[] = {
	{ QL_SPI_DRV_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(spi, ql_spi_id);

static SIMPLE_DEV_PM_OPS(ql_spi_pm, ql_spi_suspend, ql_spi_resume);

static struct spi_driver ql_spi_driver = {

	.driver = {

		.name = QL_SPI_DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ql_spi_match_table,
		.pm = &ql_spi_pm,
	},
	.probe    = ql_spi_probe,


    .remove   = ql_spi_remove,
	.shutdown = ql_spi_shutdown,
	.id_table = ql_spi_id,
};

static int __init ql_spi_drv_init(void)
{
	QLSPI_LOG("  ql spi slave driver started \n");

	return spi_register_driver(&ql_spi_driver );
}

static void __exit ql_spi_drv_exit(void)
{
	spi_unregister_driver(&ql_spi_driver );

	QLSPI_LOG(" ql spi slave exiting\n");
}

static ssize_t ql_fw_version_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
  int i;
  uint32_t fw_version[FW_TYPE_MAX] ={0};
  struct hc_info hc ={0};

  QLSPI_LOG(" START");
  hc.command = prepare_host_command(SYSTEM, SYS_FW_VERSION, SYS_GET_FW_VERSION);
  hc.param_data_len = 0;
  hc.response_len = 48;
  qleoss3_hostcmd(&hc);

  if(hc.response_len>0){
    for (i = KC_M4_L; i < FW_TYPE_MAX; i++){
        fw_version[i] = (((uint32_t)hc.response[(i*4)+0] & 0x000000FF) << 24)
                        | (((uint32_t)hc.response[(i*4)+1] & 0x000000FF) << 16)
                        | (((uint32_t)hc.response[(i*4)+2] & 0x000000FF) << 8)
                        | ((uint32_t)hc.response[(i*4)+3] & 0x000000FF) ;
    }
  }

  QLSPI_LOG("END");
  return   sprintf(buf, "%08X %08X %08X %08X %08X %08X %08X %08X %08X %08X %08X %08X\n", 
                    fw_version[KC_M4_L],
                    fw_version[KC_M4_H],
                    fw_version[QL_M4_L],
                    fw_version[QL_M4_H],
                    fw_version[KC_FFE_L],
                    fw_version[KC_FFE_H],
                    fw_version[QL_FFE_L],
                    fw_version[QL_FFE_H],
                    fw_version[KC_FABRIC_L],
                    fw_version[KC_FABRIC_H],
                    fw_version[QL_FABRIC_L],
                    fw_version[QL_FABRIC_H]);
}

#define RES_MAX 128
static uint8_t diag_res[RES_MAX];
static ssize_t ql_host_cmd_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{

    ssize_t ret = 0;

    QLSPI_LOG("START");

    ret = sprintf( buf, "%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",
                    diag_res[0],diag_res[1],diag_res[2],diag_res[3],
                    diag_res[4],diag_res[5],diag_res[6],diag_res[7],
                    diag_res[8],diag_res[9],diag_res[10],diag_res[11],
                    diag_res[12],diag_res[13],diag_res[14],diag_res[15],
                    diag_res[16],diag_res[17],diag_res[18],diag_res[19],
                    diag_res[20],diag_res[21],diag_res[22],diag_res[23],
                    diag_res[24],diag_res[25],diag_res[26],diag_res[27],
                    diag_res[28],diag_res[29],diag_res[30],diag_res[31],
                    diag_res[32],diag_res[33],diag_res[34],diag_res[35],
                    diag_res[36],diag_res[37],diag_res[38],diag_res[39],
                    diag_res[40],diag_res[41],diag_res[42],diag_res[43],
                    diag_res[44],diag_res[45],diag_res[46],diag_res[47],
                    diag_res[48],diag_res[49],diag_res[50],diag_res[51],
                    diag_res[52],diag_res[53],diag_res[54],diag_res[55],
                    diag_res[56],diag_res[57],diag_res[58],diag_res[59],
                    diag_res[60],diag_res[61],diag_res[62],diag_res[63],
                    diag_res[64],diag_res[65],diag_res[66],diag_res[67],
                    diag_res[68],diag_res[69],diag_res[70],diag_res[71],
                    diag_res[72],diag_res[73],diag_res[74],diag_res[75],
                    diag_res[76],diag_res[77],diag_res[78],diag_res[79],
                    diag_res[80],diag_res[81],diag_res[82],diag_res[83],
                    diag_res[84],diag_res[85],diag_res[86],diag_res[87],
                    diag_res[88],diag_res[89],diag_res[90],diag_res[91],
                    diag_res[92],diag_res[93],diag_res[94],diag_res[95],
                    diag_res[96],diag_res[97],diag_res[98],diag_res[99],
                    diag_res[100],diag_res[101],diag_res[102],diag_res[103],
                    diag_res[104],diag_res[105],diag_res[106],diag_res[107],
                    diag_res[108],diag_res[109],diag_res[110],diag_res[111],
                    diag_res[112],diag_res[113],diag_res[114],diag_res[115],
                    diag_res[116],diag_res[117],diag_res[118],diag_res[119],
                    diag_res[120],diag_res[121],diag_res[122],diag_res[123],
                    diag_res[124],diag_res[125],diag_res[126],diag_res[127] );

    QLSPI_LOG("END");

    return ret;

}

static ssize_t ql_host_cmd_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    uint32_t req_cmd[4] 	= {0};
    uint32_t req_param[16] 	= {0};
	uint8_t set_cmd[4] 		= {0};
	uint8_t set_param[16] 	= {0};
    struct hc_info hc 		= {0};
    uint8_t purpose = 0x00;
    uint8_t type = 0x00;
    uint8_t cmd =0x00;
	int i = 0;

    QLSPI_LOG("START");

    sscanf( buf, "%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",
            &req_cmd[0],
            &req_cmd[1],
            &req_cmd[2],
            &req_cmd[3],
            &req_param[0],
            &req_param[1],
            &req_param[2],
            &req_param[3],
            &req_param[4],
            &req_param[5],
            &req_param[6],
            &req_param[7],
            &req_param[8],
            &req_param[9],
            &req_param[10],
            &req_param[11],
            &req_param[12],
            &req_param[13],
            &req_param[14],
            &req_param[15]);

	for(i = 0; i < ARRAY_SIZE(req_cmd); i++){
		set_cmd[i] = (uint8_t)(req_cmd[i] & 0x000000FF);
	}
	for(i = 0; i < ARRAY_SIZE(req_param); i++){
		set_param[i] = (uint8_t)(req_param[i] & 0x000000FF);
	}

    QLSPI_LOG("CMD =0x%02x%02x%02x%02x\n",
            set_cmd[0],
            set_cmd[1],
            set_cmd[2],
            set_cmd[3]);
    QLSPI_LOG("PRM = %02x %02x %02x %02x %02x %02x %02x %02x "
					"%02x %02x %02x %02x %02x %02x %02x %02x\n",
            set_param[0],
            set_param[1],
            set_param[2],
            set_param[3],
            set_param[4],
            set_param[5],
            set_param[6],
            set_param[7],
            set_param[8],
			set_param[9],
			set_param[10],
			set_param[11],
			set_param[12],
			set_param[13],
			set_param[14],
			set_param[15]);

    purpose = (uint8_t)(set_cmd[1] & 0x000000FF);
    type 	= (uint8_t)(set_cmd[2] & 0x000000FF);
    cmd 	= (uint8_t)(set_cmd[3] & 0x000000FF);

    hc.command = prepare_host_command(purpose, type,cmd);
    hc.param_data_len = 16;

    memcpy(hc.param_data,set_param,sizeof(set_param));
    hc.response_len = RES_MAX;
    qleoss3_hostcmd(&hc);

    memcpy(&diag_res, &hc.response, RES_MAX);

    QLSPI_LOG("END");

    return count;
}

static ssize_t ql_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint32_t ret;
    int8_t ql_status = 0;
    int8_t fabric_status = 0;

    QLSPI_LOG("start");
    ql_status = ql_spi_get_eoss3_status();

#ifdef FABRIC_LOADER
    fabric_status = get_fabric_status();
    pr_info("ql_status = %d , fabric_status = %d\n",ql_status,fabric_status);
#endif

    ret = sprintf( buf, "%d %d\n", ql_status,fabric_status);
    QLSPI_LOG("end");

    return ret;
}

static ssize_t ql_status_store(struct device *dev, struct device_attribute *attr,const char *buf,size_t count )
{
    uint32_t dummy_val = 0;

    QLSPI_LOG("start");

    sscanf(buf, "%d", &dummy_val);
    switch(dummy_val){
        case 0:
            ql_spi_set_eoss3_status(NORMAL);
            break;
        case 1:
            ql_spi_set_eoss3_status(DEAD);
            break;
        case 2:
            ql_spi_set_eoss3_status(RECOVERING);
            break;
        case 3:
            ql_spi_set_eoss3_status(NOT_INITIALIZE);
            break;
        case 4:
            ql_spi_set_eoss3_status(FW_DOWNLOADING);
            break;
        case 5:
            ql_spi_set_eoss3_status(CLIENT_INITIALIZING);
            break;
        default:
            ql_spi_set_eoss3_status(NORMAL);
            break;
    }
    QLSPI_LOG("end");
    return count;
}


static     uint32_t rw_addr = 0;
static     uint32_t rw_len = 0;
static     uint32_t ql_reg_write_ret =0;

static ssize_t ql_reg_write_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    uint32_t ret;

    QLSPI_LOG("start");

    ret = sprintf( buf, "%x", ql_reg_write_ret);

    QLSPI_LOG("end");
    return ret;

}
    
static ssize_t ql_reg_write_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{

    uint32_t ret;
    uint32_t data[65] = {0};
    uint8_t  data_uint8[64];
    int i = 0;
    const int ADDRES_BYTE_SIZE =4;
    const int LENGTH_BYTE_SIZE =1;
    const int DATA_HEAD =ADDRES_BYTE_SIZE+LENGTH_BYTE_SIZE;

    QLSPI_LOG("start");

    ql_reg_write_ret = 0;

    sscanf(buf, "%x %x"
        " %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x"
        " %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x"
        " %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x"
        " %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",
        &data[ 0],
        &data[ 1],
        &data[ 2],
        &data[ 3],
        &data[ 4],
        &data[ 5],
        &data[ 6],
        &data[ 7],
        &data[ 8],
        &data[ 9],
        &data[10],
        &data[11],
        &data[12],
        &data[13],
        &data[14],
        &data[15],
        &data[16],
        &data[17],
        &data[18],
        &data[19],
        &data[20],
        &data[21],
        &data[22],
        &data[23],
        &data[24],
        &data[25],
        &data[26],
        &data[27],
        &data[28],
        &data[29],
        &data[30],
        &data[31],
        &data[32],
        &data[33],
        &data[34],
        &data[35],
        &data[36],
        &data[37],
        &data[38],
        &data[39],
        &data[40],
        &data[41],
        &data[42],
        &data[43],
        &data[44],
        &data[45],
        &data[46],
        &data[47],
        &data[48],
        &data[49],
        &data[50],
        &data[51],
        &data[52],
        &data[53],
        &data[54],
        &data[55],
        &data[56],
        &data[57],
        &data[58],
        &data[59],
        &data[60],
        &data[61],
        &data[62],
        &data[63],
        &data[64],
        &data[65]);

    rw_addr = (((uint32_t)(data[0]  << 24) | (data[1] << 16) | (data[2]  << 8)  | (data[3])));
    rw_len = data[4];
    QLSPI_LOG("addr  = 0x%08x ,len  = 0x%08x",rw_addr,rw_len);

    if(rw_len == 0){
      ql_reg_write_ret = 0xff;
    }

    for(i = 0 ; i < rw_len; i++){
      data_uint8[i] = (uint8_t)data[i+DATA_HEAD];
      QLSPI_LOG("data = 0x%02x\n",data_uint8[i]);
    }

    ret = QLSPI_Write_S3_Mem(rw_addr,data_uint8,rw_len);
    if (ret == QL_STATUS_OK) {
      QLSPI_LOG("sns_device_write OK");
    }
    else {
      QLSPI_LOG("sns_device_write NG");
      ql_reg_write_ret = 0xff;
    }

    QLSPI_LOG("end");
    return count;
}


static ssize_t ql_reg_read_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{

    uint32_t data[5] = {0};

    QLSPI_LOG("start");

    sscanf(buf, "%02x %02x %02x %02x %02x", &data[0],&data[1],&data[2],&data[3], &data[4]);
    rw_addr = ((uint32_t)(data[0]  << 24 | data[1] << 16 | data[2]  << 8  | data[3]));
    rw_len = data[4];

    QLSPI_LOG("end");

    return count;
}

static ssize_t ql_reg_read_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret = 0;
    uint8_t ql_reg_read_rslt[65] = {0};

    QLSPI_LOG("start");

    if(rw_len == 0){
      ql_reg_read_rslt[0] = 0xFF;
    }
    else{

        QLSPI_LOG("addr  = 0x%08x ,len  = 0x%08x",rw_addr,rw_len);
        ret = QLSPI_Read_S3_Mem(rw_addr,ql_reg_read_rslt,rw_len);

        if (ret == QL_STATUS_OK) {
          QLSPI_LOG("sns_device_write OK");
        }
        else {
          QLSPI_LOG("sns_device_write NG");
          ql_reg_read_rslt[0] = 0xFF;
        }
    }

    ret = sprintf( buf, "%x"
        " %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x"
        " %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x"
        " %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x"
        " %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",
                    ql_reg_read_rslt[0],ql_reg_read_rslt[1],ql_reg_read_rslt[2],ql_reg_read_rslt[3],
                    ql_reg_read_rslt[4],ql_reg_read_rslt[5],ql_reg_read_rslt[6],ql_reg_read_rslt[7],
                    ql_reg_read_rslt[8],ql_reg_read_rslt[9],ql_reg_read_rslt[10],ql_reg_read_rslt[11],
                    ql_reg_read_rslt[12],ql_reg_read_rslt[13],ql_reg_read_rslt[14],ql_reg_read_rslt[15],
                    ql_reg_read_rslt[16],ql_reg_read_rslt[17],ql_reg_read_rslt[18],ql_reg_read_rslt[19],
                    ql_reg_read_rslt[20],ql_reg_read_rslt[21],ql_reg_read_rslt[22],ql_reg_read_rslt[23],
                    ql_reg_read_rslt[24],ql_reg_read_rslt[25],ql_reg_read_rslt[26],ql_reg_read_rslt[27],
                    ql_reg_read_rslt[28],ql_reg_read_rslt[29],ql_reg_read_rslt[30],ql_reg_read_rslt[31],
                    ql_reg_read_rslt[32],ql_reg_read_rslt[33],ql_reg_read_rslt[34],ql_reg_read_rslt[35],
                    ql_reg_read_rslt[36],ql_reg_read_rslt[37],ql_reg_read_rslt[38],ql_reg_read_rslt[39],
                    ql_reg_read_rslt[40],ql_reg_read_rslt[41],ql_reg_read_rslt[42],ql_reg_read_rslt[43],
                    ql_reg_read_rslt[44],ql_reg_read_rslt[45],ql_reg_read_rslt[46],ql_reg_read_rslt[47],
                    ql_reg_read_rslt[48],ql_reg_read_rslt[49],ql_reg_read_rslt[50],ql_reg_read_rslt[51],
                    ql_reg_read_rslt[52],ql_reg_read_rslt[53],ql_reg_read_rslt[54],ql_reg_read_rslt[55],
                    ql_reg_read_rslt[56],ql_reg_read_rslt[57],ql_reg_read_rslt[58],ql_reg_read_rslt[59],
                    ql_reg_read_rslt[60],ql_reg_read_rslt[61],ql_reg_read_rslt[62],ql_reg_read_rslt[63],
                    ql_reg_read_rslt[64] );

    QLSPI_LOG("%x"
        " %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x"
        " %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x"
        " %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x"
        " %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n",
                    ql_reg_read_rslt[0],ql_reg_read_rslt[1],ql_reg_read_rslt[2],ql_reg_read_rslt[3],
                    ql_reg_read_rslt[4],ql_reg_read_rslt[5],ql_reg_read_rslt[6],ql_reg_read_rslt[7],
                    ql_reg_read_rslt[8],ql_reg_read_rslt[9],ql_reg_read_rslt[10],ql_reg_read_rslt[11],
                    ql_reg_read_rslt[12],ql_reg_read_rslt[13],ql_reg_read_rslt[14],ql_reg_read_rslt[15],
                    ql_reg_read_rslt[16],ql_reg_read_rslt[17],ql_reg_read_rslt[18],ql_reg_read_rslt[19],
                    ql_reg_read_rslt[20],ql_reg_read_rslt[21],ql_reg_read_rslt[22],ql_reg_read_rslt[23],
                    ql_reg_read_rslt[24],ql_reg_read_rslt[25],ql_reg_read_rslt[26],ql_reg_read_rslt[27],
                    ql_reg_read_rslt[28],ql_reg_read_rslt[29],ql_reg_read_rslt[30],ql_reg_read_rslt[31],
                    ql_reg_read_rslt[32],ql_reg_read_rslt[33],ql_reg_read_rslt[34],ql_reg_read_rslt[35],
                    ql_reg_read_rslt[36],ql_reg_read_rslt[37],ql_reg_read_rslt[38],ql_reg_read_rslt[39],
                    ql_reg_read_rslt[40],ql_reg_read_rslt[41],ql_reg_read_rslt[42],ql_reg_read_rslt[43],
                    ql_reg_read_rslt[44],ql_reg_read_rslt[45],ql_reg_read_rslt[46],ql_reg_read_rslt[47],
                    ql_reg_read_rslt[48],ql_reg_read_rslt[49],ql_reg_read_rslt[50],ql_reg_read_rslt[51],
                    ql_reg_read_rslt[52],ql_reg_read_rslt[53],ql_reg_read_rslt[54],ql_reg_read_rslt[55],
                    ql_reg_read_rslt[56],ql_reg_read_rslt[57],ql_reg_read_rslt[58],ql_reg_read_rslt[59],
                    ql_reg_read_rslt[60],ql_reg_read_rslt[61],ql_reg_read_rslt[62],ql_reg_read_rslt[63],
                    ql_reg_read_rslt[64] );

    QLSPI_LOG("end");
    return ret;
}

module_init(ql_spi_drv_init);
module_exit(ql_spi_drv_exit);

MODULE_DESCRIPTION("QuickLogic SPI Slave Driver");
MODULE_AUTHOR("Quick Logic Software (India) Private Limited.");
MODULE_LICENSE("GPL");
