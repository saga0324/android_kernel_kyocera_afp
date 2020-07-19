/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 * (C) 2018 KYOCERA Corporation
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
//#include "sensor_micon_ml630q790.h"
#include <linux/sensor_micon_common.h>
#include <linux/lapis/ml630q790.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/semaphore.h>
#include <linux/poll.h>
/* ------------------------------------------------
  Define
------------------------------------------------ */

#define SSIO_MASK_WRITE             (0x7f)
#define SSIO_MASK_READ              (0x80)
#define SPI_RESUME_RETRY_NUM        300
#define SPI_RETRY_NUM               5
#define STATUS_READ_RETRY_NUM       98
#define STATUS_READ_RETRY_NUM_2     10
#define WAITEVENT_TIMEOUT           (3000)
#define WORK_QUEUE_NUM              230
#define WQ_TYPE_DUMMYVAL            -1


/* ------------------------------------------------
  Variables
------------------------------------------------ */
typedef struct t_SNS_WorkQueue {
    struct work_struct  work;
    bool                status;
    int32_t             option;
    int32_t             type;
} SNS_WorkQueue;

static uint fw_version = 0x00000000;
module_param(fw_version, uint, S_IRUGO );
MODULE_PARM_DESC(fw_version, "micon-fw-version");

static struct workqueue_struct *sns_wq_int;
static struct workqueue_struct *sns_wq;
static DEFINE_SPINLOCK(ml630q790_wq_lock);
static DEFINE_MUTEX(ml630q790_mutex);
static DEFINE_MUTEX(ml630q790_irq_mutex);
static DEFINE_MUTEX(ml630q790_snsapp_mutex);
static struct spi_device *client_ic;
static struct semaphore s_tSnsrSema;
static struct mutex s_tDataMutex;
static struct mutex s_tSpiMutex;
static bool spi_error = false;
static int32_t g_workqueue_used = 0;
static int32_t IntIrqFlg;
static int32_t SnsWorkCnt;
static wait_queue_head_t s_tWaitInt;
static atomic_t g_InitializeState;
static atomic_t s_drvStatus = ATOMIC_INIT(ML630Q790_POWER_OFF);
atomic_t g_FWUpdateStatus;
atomic_t g_MiconDebug;
atomic_t g_bIsResume;
//atomic_t g_ResetStatus;
atomic_t g_bIsIntIrqEnable;
uint16_t g_lastCmdError;
static int32_t g_nIntIrqNo;
static struct micon_bdata g_micon_bdata;
static SNS_WorkQueue  s_tSnsWork[WORK_QUEUE_NUM];

static HostCmdRes diag_host_cmd_res;
static uint8_t dmcmd_reg_rw_rslt[65] = {0};
/* ------------------------------------------------
  Prototype Functions
------------------------------------------------ */

static int32_t sns_spi_write_proc(uint8_t adr, const uint8_t *data, uint8_t size);
static int32_t sns_spi_read_proc(uint8_t adr, uint8_t *data, uint16_t size);
static int32_t sns_waitcmd(uint16_t intBit);
static void sns_timeout_dump(uint8_t *reg);

static irqreturn_t ml630q790_irq_handler(int32_t irq, void *dev_id);
static void ml630q790_int_app_work_func(struct work_struct *work);
static void ml630q790_int_work_func(struct work_struct *work);
static void ml630q790_wq_init(void);
static int32_t ml630q790_wq_create( struct workqueue_struct *queue,
									void (*func)(struct work_struct *),
									int32_t option, int32_t type );
static void ml630q790_wq_delete(struct work_struct *work);

static int32_t client_dev_initialize(void);
static void ml630q790_BRMP_direction(void);
static void ml630q790_FW_BRMP_ctrl(void);
int32_t ml630q790_update_fw_exe(bool boot, uint8_t *arg_iData, uint32_t arg_iLen);
int32_t ml630q790_update_fw(uint8_t *arg_iData, uint32_t arg_iLen);
int32_t ml630q790_update_fw_seq(uint8_t *arg_iData, uint32_t arg_iLen);
static void get_DMcmd_host_cmd_res(uint32_t* res);
static ssize_t ml630q790_fw_version_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t ml630q790_fw_version_chk_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t ml630q790_fw_update_sq_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t ml630q790_fw_update_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t ml630q790_brmp_ctl_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t ml630q790_host_cmd_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t ml630q790_host_cmd_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t ml630q790_reg_rw_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t ml630q790_reg_rw_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );

static int ml630q790_probe(struct spi_device *client);
static int ml630q790_remove(struct spi_device *client);
static int ml630q790_suspend(struct device *dev);
static int ml630q790_resume(struct device *dev);
static void ml630q790_shutdown(struct spi_device *client);
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
static int debug_param = 0;
module_param(debug_param, int, S_IRUGO | S_IWUSR);
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
#define ENABLE_IRQ {                                                            \
    if((g_nIntIrqNo != -1) && (atomic_cmpxchg(&g_bIsIntIrqEnable, 0, 1) == 0)){ \
        enable_irq(g_nIntIrqNo);                                                \
    }                                                                           \
}
#define DISABLE_IRQ {                                                           \
    if((g_nIntIrqNo != -1) && (atomic_cmpxchg(&g_bIsIntIrqEnable, 1, 0) == 1)){ \
        disable_irq_nosync(g_nIntIrqNo);                                        \
    }                                                                           \
}

static int32_t sns_spi_write_proc(uint8_t adr, const uint8_t *data, uint8_t size)
{
    int32_t ret = 0;
    struct spi_message  message;
    struct spi_transfer transfer;
    uint8_t send_data[100];

    ML630Q790_D_LOG("start");

    if((data == NULL) || (size == 0)){
        ML630Q790_ERR_LOG("end return[ML630Q790_RC_ERR]");
        return ML630Q790_RC_ERR;
    }

    adr &= SSIO_MASK_WRITE;

    send_data[0] = adr;
    memcpy(&send_data[1], data, size);
    memset(&transfer, 0, sizeof(transfer));

    spi_message_init(&message);

    transfer.tx_buf = send_data;
    transfer.rx_buf = NULL;
    transfer.len    = 1 + size;
    spi_message_add_tail(&transfer, &message);

    ret = spi_sync(client_ic, &message);
    if(ret < 0){
        ML630Q790_ERR_LOG("falut spi_sync()-->ret[%d]",ret);
    }

    ML630Q790_D_LOG("end - return[%d]",ret);
    return ret;
}

static int32_t sns_spi_read_proc(uint8_t adr, uint8_t *data, uint16_t size)
{
    int32_t ret = 0;

    struct spi_message  message;
    struct spi_transfer transfer[2];

    ML630Q790_D_LOG("start");

    if( (data == NULL) || (size == 0)){
        ML630Q790_ERR_LOG("end return[ML630Q790_RC_ERR]");
        return ML630Q790_RC_ERR;
    }

    memset(&transfer, 0, sizeof(transfer));

    adr |= SSIO_MASK_READ;

    spi_message_init(&message);

    transfer[0].tx_buf = &adr;
    transfer[0].rx_buf = NULL;
    transfer[0].len    = 1;
    spi_message_add_tail(&transfer[0], &message);

    transfer[1].tx_buf = NULL;
    transfer[1].rx_buf = (void *)data;
    transfer[1].len    = size;
    spi_message_add_tail(&transfer[1], &message);

    ret = spi_sync(client_ic, &message);
    if(ret < 0){
        ML630Q790_ERR_LOG("falut spi_sync()-->ret[%d]",ret);
    }


    ML630Q790_D_LOG("end - return[%d]",ret);
    return ret;
}

#define SPI_RAM_WRITE_SPLIT_SIZE ((uint16_t)0x20)
int32_t sns_spi_ram_write_proc(uint8_t adr, const uint8_t *data, int32_t size)
{
    int32_t ret = 0;
    struct spi_message  message;
    struct spi_transfer transfer;
    uint8_t *send_data=  NULL;
    uint16_t remain = size;
    uint16_t write_size = size;
    uint8_t enomem_retry = 1;

    ML630Q790_D_LOG("start");

    if((data == NULL) || (size == 0)){
        ML630Q790_ERR_LOG("end return[ML630Q790_RC_ERR]");
        return ML630Q790_RC_ERR;
    }

    send_data = (uint8_t *)kmalloc( size + 1, GFP_KERNEL );
    if(send_data == NULL){
        ML630Q790_ERR_LOG("end return[ENOMEM]");
        return -ENOMEM;
    }

    adr &= SSIO_MASK_WRITE;

    do {
        send_data[0] = adr;
        memcpy(&send_data[1], data, write_size);
        memset(&transfer, 0, sizeof(transfer));

        spi_message_init(&message);

        transfer.tx_buf = send_data;
        transfer.rx_buf = NULL;
        transfer.len    = 1 + write_size;
        spi_message_add_tail(&transfer, &message);

        ret = spi_sync(client_ic, &message);

        if(ret < 0){
            ML630Q790_ERR_LOG("falut spi_sync()-->ret[%d]",ret);
            if(ret == -ENOMEM && enomem_retry > 0){
                ML630Q790_ERR_LOG("falut spi_sync()-->ENOMEM retry!");
                write_size = 0;
                enomem_retry = 0;
            }
            else {
                break;
            }
        }
        if (write_size == remain) {
            break;
        }
        data += write_size;
        remain -= write_size;
        write_size = min(SPI_RAM_WRITE_SPLIT_SIZE, remain);
    } while (1);

    kfree(send_data);

    ML630Q790_D_LOG("end - return[%d]",ret);
    return ret;
}

int32_t ml630q790_device_write(uint8_t adr, const uint8_t *data, uint8_t size)
{
    int32_t i;
    int32_t ret;

    ML630Q790_D_LOG("start");

    if(unlikely(spi_error)){
        ML630Q790_ERR_LOG("spi_setup NG");
        return -EINVAL;
    }

    mutex_lock(&s_tSpiMutex);

    ML630Q790_D_LOG("adr[0x%02x] data[%p] size[0x%02x]", adr, data, size);
    ML630Q790_D_DUMP("ml630q790_device_write:", DUMP_PREFIX_OFFSET, data, size);

    for(i=0; i<SPI_RESUME_RETRY_NUM; i++)
    {
        if( atomic_read(&g_bIsResume) == true )
        {
            break;
        }
        else
        {
            msleep(1);
        }
    }
    if( i == SPI_RESUME_RETRY_NUM )
    {
        ML630Q790_ERR_LOG("end return[EBUSY]");
        mutex_unlock(&s_tSpiMutex);
        return -EBUSY;
    }

    for(i=0; i<SPI_RETRY_NUM; i++)
    {
        ret = sns_spi_write_proc(adr, data, size);
        if(ret == 0){
            mutex_unlock(&s_tSpiMutex);
            ML630Q790_D_LOG("end - return[%d]",ret);
            return 0;
        }else if((ret == -EBUSY) || (ret == -ETIMEDOUT) || (ret == -EINPROGRESS) || ((ret == -EIO) && (i < SPI_RETRY_NUM - 1))){
            ML630Q790_ERR_LOG("fault sns_spi_write_proc()-->ret[%d] RETRY[%d]",ret ,i );
            msleep(100);
        }else{
            g_bDevIF_Error = true;
            ML630Q790_ERR_LOG("SPI write Other error (H/W Reset ON)");
            break;
        }
    }

    mutex_unlock(&s_tSpiMutex);

    ML630Q790_D_LOG("end - return[%d]",ret);
    return ret;
}

int32_t ml630q790_device_read(uint8_t adr, uint8_t *data, uint16_t size)
{
    int32_t i;
    int32_t ret;

    ML630Q790_D_LOG("start");

    if(unlikely(spi_error)){
        ML630Q790_ERR_LOG("spi_setup NG");
        return -EINVAL;
    }

    mutex_lock(&s_tSpiMutex);

    for(i=0; i<SPI_RESUME_RETRY_NUM; i++)
    {
        if( atomic_read(&g_bIsResume) == true )
        {
            break;
        }
        else
        {
            msleep(1);
        }
    }
    if( i == SPI_RESUME_RETRY_NUM )
    {
        ML630Q790_ERR_LOG("end return[EBUSY]");
        mutex_unlock(&s_tSpiMutex);
        return -EBUSY;
    }

    for(i=0; i<SPI_RETRY_NUM; i++)
    {
        ret = sns_spi_read_proc(adr, data, size);
        if(ret == 0){
            ML630Q790_D_LOG("adr[0x%02x] data[%p] size[0x%04x]", adr, data, size);
            ML630Q790_D_DUMP("ml630q790_device_read:", DUMP_PREFIX_OFFSET, data, size);
            mutex_unlock(&s_tSpiMutex);
            ML630Q790_D_LOG("end - return[%d]",ret);
            return 0;
        }else if((ret == -EBUSY) || (ret == -ETIMEDOUT) || (ret == -EINPROGRESS) || ((ret == -EIO) && (i < SPI_RETRY_NUM - 1))){
            ML630Q790_ERR_LOG("fault sns_spi_write_proc()-->ret[%d] RETRY[%d]",ret ,i );
            msleep(100);
        }else{
            g_bDevIF_Error = true;
            ML630Q790_ERR_LOG("SPI write Other error (H/W Reset ON)");
            break;
        }
    }

    mutex_unlock(&s_tSpiMutex);

    ML630Q790_D_LOG("end - return[%d]",ret);
    return ret;
}

int32_t sns_hostcmd(const HostCmd *prm, HostCmdRes *res, uint32_t res_size, uint8_t mode, uint8_t is_fwud)
{
    int32_t ret;
    uint8_t reg[19];
    int32_t i;

#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    s64 func_time_us, enter_us, exit_us;
    ktime_t enter, exit;
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
    ML630Q790_D_LOG("start");
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    if(debug_param ) {
        enter = ktime_get();
    }
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */

    if (g_micon_error) {
        ML630Q790_ERR_LOG("end - skip due to micon error");
        return ML630Q790_RC_ERR;
    }

    down(&s_tSnsrSema);

    ML630Q790_D_LOG("req [0x%04x] %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x", prm->cmd.udata16,
        prm->prm.ub_prm[0],
        prm->prm.ub_prm[1],
        prm->prm.ub_prm[2],
        prm->prm.ub_prm[3],
        prm->prm.ub_prm[4],
        prm->prm.ub_prm[5],
        prm->prm.ub_prm[6],
        prm->prm.ub_prm[7],
        prm->prm.ub_prm[8],
        prm->prm.ub_prm[9],
        prm->prm.ub_prm[10],
        prm->prm.ub_prm[11],
        prm->prm.ub_prm[12],
        prm->prm.ub_prm[13],
        prm->prm.ub_prm[14],
        prm->prm.ub_prm[15]
    );

    for(i = 0; i < 16; i++ ) {
        reg[i] = prm->prm.ub_prm[15-i];
    }
    reg[16] = prm->cmd.udata8[0];
    reg[17] = prm->cmd.udata8[1];
    reg[18] = 1;

    ML630Q790_D_LOG("reg0[%x] reg1[%x] reg2[%x] reg3[%x]",
                  reg[0] , reg[1] , reg[2] , reg[3] );
    ML630Q790_D_LOG("reg4[%x] reg5[%x] reg6[%x] reg7[%x]",
                  reg[4] , reg[5] , reg[6] , reg[7] );
    ML630Q790_D_LOG("reg8[%x] reg9[%x] reg10[%x] reg11[%x]",
                  reg[8] , reg[9] , reg[10] , reg[11] );
    ML630Q790_D_LOG("reg12[%x] reg13[%x] reg14[%x] reg15[%x]",
                  reg[12] , reg[13] , reg[14] , reg[15] );
    ML630Q790_D_LOG("reg16[%x] reg17[%x] reg18[%x] mode[%x]",
                  reg[16] , reg[17] , reg[18] , mode );

    mutex_lock(&s_tDataMutex);
    IntIrqFlg = 0;
    g_lastCmdError = 0;
    mutex_unlock(&s_tDataMutex);

    ret = ml630q790_device_write(PRM0F, reg, sizeof(reg));
    if(ret == ML630Q790_RC_OK){

        if((mode & EXE_HOST_WAIT) == EXE_HOST_WAIT){
            ret |= sns_waitcmd(INTREQ_HOST_CMD);
            if(ret != ML630Q790_RC_OK) {
                if( ((mode & EXE_HOST_EX_NO_RECOVER) == EXE_HOST_EX_NO_RECOVER) && 
                    (ret == ML630Q790_RC_ERR_TIMEOUT) ){
                    ML630Q790_ERR_LOG("SPI HostCmd error");
                    ML630Q790_ERR_LOG("mode:%x",mode);
                    up(&s_tSnsrSema);
                    return ML630Q790_RC_ERR_TIMEOUT;
                }
                sns_timeout_dump(reg);

                up(&s_tSnsrSema);
                g_bDevIF_Error = true;
                ML630Q790_ERR_LOG("SPI HostCmd error:%d mode:%x",ret ,mode);
                return ret;
            }
        }

        if((mode & EXE_HOST_RES) == EXE_HOST_RES){
            if(is_fwud == READ_FIFO){
                ret |= ml630q790_device_read(FIFO, res->res.ub_res, res_size);
                if(ret != ML630Q790_RC_OK) {
                    ML630Q790_ERR_LOG("SPI HostCmd error(FIFO)");
                }
            }
            else if(is_fwud == READ_RSLT){
                ret |= ml630q790_device_read(RSLT00, res->res.ub_res, res_size);
                if(ret != ML630Q790_RC_OK) {
                    ML630Q790_ERR_LOG("SPI HostCmd error(RSLT00)");
                }
            }else{
                ret |= ML630Q790_RC_ERR;
                if(ret != ML630Q790_RC_OK) {
                    ML630Q790_ERR_LOG("SPI HostCmd is_fwud error[%d]", is_fwud);
                }
            }
        }

        res->err.udata16 = g_lastCmdError;
    }else{
        ML630Q790_ERR_LOG("SPI HostCmd error ret[%d]", ret);
        g_bDevIF_Error = true;
    }

    ML630Q790_D_LOG("rsp [0x%04x] %s ret=%d,err=%d size=%u %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x...",
        prm->cmd.udata16,
        (ret == ML630Q790_RC_OK && res->err.udata16 == 0) ? "OK" : "NG",
        ret,
        res->err.udata16,
        res_size,
        res->res.ub_res[0],
        res->res.ub_res[1],
        res->res.ub_res[2],
        res->res.ub_res[3],
        res->res.ub_res[4],
        res->res.ub_res[5],
        res->res.ub_res[6],
        res->res.ub_res[7],
        res->res.ub_res[8],
        res->res.ub_res[9],
        res->res.ub_res[10],
        res->res.ub_res[11],
        res->res.ub_res[12],
        res->res.ub_res[13],
        res->res.ub_res[14],
        res->res.ub_res[15],
        res->res.ub_res[16],
        res->res.ub_res[17],
        res->res.ub_res[18],
        res->res.ub_res[19],
        res->res.ub_res[20],
        res->res.ub_res[21],
        res->res.ub_res[22],
        res->res.ub_res[23]
    );

    up(&s_tSnsrSema);

    ML630Q790_D_LOG("end - return[%d]",ret);
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    if(debug_param ){
        exit = ktime_get();
        func_time_us = ktime_to_us(ktime_sub(exit, enter));
        enter_us = ktime_to_us(enter);
        exit_us = ktime_to_us(exit);
        printk(KERN_NOTICE "[IT_TEST] %s: bgn:%lld end:%lld time:%lld\n",__func__,enter_us,exit_us,func_time_us);
    }
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
    return ret;
}

static int32_t sns_waitcmd(uint16_t intBit)
{
    int32_t ret = ML630Q790_RC_ERR_TIMEOUT;
    int32_t result = 0;
    long timeout;
    int32_t retry = 300;

    ML630Q790_D_LOG("start");

    timeout = msecs_to_jiffies(WAITEVENT_TIMEOUT);

    while(retry){
    
        result = wait_event_interruptible_timeout(s_tWaitInt, (IntIrqFlg & (INTREQ_HOST_CMD | INTREQ_ERROR)), timeout);
        mutex_lock(&s_tDataMutex);
        if( IntIrqFlg & INTREQ_ERROR ){
            IntIrqFlg &= ~INTREQ_ERROR;
            mutex_unlock(&s_tDataMutex);

            ML630Q790_D_LOG("INTREQ0/1 -Error- ");
            ret = ML630Q790_RC_ERR;
            break;

        }else if( IntIrqFlg & INTREQ_HOST_CMD ){
            IntIrqFlg &= ~INTREQ_HOST_CMD;
            mutex_unlock(&s_tDataMutex);

            ret = ML630Q790_RC_OK;
            ML630Q790_D_LOG("Wakeup Event... ");
            break;
        }
        mutex_unlock(&s_tDataMutex);

        if( result == -ERESTARTSYS ) {
            ML630Q790_D_LOG("wait event signal received. retry = [%d] IntIrqFlg = [%x}",
                         retry, IntIrqFlg);
            msleep(10);
        }

        if( result == 0 ){
            ret = ML630Q790_RC_ERR_TIMEOUT;
            ML630Q790_ERR_LOG("wait event timeout... [%x]", IntIrqFlg);
            break;
        }
        retry--;
    }

    ML630Q790_D_LOG("end - return[%d]",ret);
    return ret;
}

static void sns_timeout_dump(uint8_t *reg)
{
    unsigned char data[0x1f];
    int i;

    ML630Q790_D_LOG("start");

    ML630Q790_ERR_LOG("##### SNS TimeOut Error Log #####");
    ML630Q790_ERR_LOG("Gloval Value :");
    ML630Q790_ERR_LOG("Send Cmd :[%x][%x][%x][%x]", reg[0] ,reg[1] ,reg[2] ,reg[3]);
    ML630Q790_ERR_LOG("Send Cmd :[%x][%x][%x][%x]", reg[4] ,reg[5] ,reg[6] ,reg[7]);
    ML630Q790_ERR_LOG("Send Cmd :[%x][%x][%x][%x]", reg[8] ,reg[9] ,reg[10] ,reg[11]);
    ML630Q790_ERR_LOG("Send Cmd :[%x][%x][%x][%x]", reg[12] ,reg[13] ,reg[14] ,reg[15]);
    ML630Q790_ERR_LOG("Send Cmd :[%x][%x][%x]", reg[16] ,reg[17] ,reg[18]);
    ML630Q790_ERR_LOG("g_bIsIntIrqEnable[%04x]", (int)atomic_read(&g_bIsIntIrqEnable));
    ML630Q790_ERR_LOG("IntIrqFlg[%04x]", IntIrqFlg);
    if (gpio_is_valid(g_micon_bdata.int_gpio)) {
        ML630Q790_ERR_LOG("g_micon_bdata[%04x]", gpio_get_value(g_micon_bdata.int_gpio));
    }

    ml630q790_device_read(0x00, data, sizeof(data));

    ML630Q790_ERR_LOG("H/W Register Dump :");
    for(i=0; i<sizeof(data); i++){
        if((i % 16) == 0){
            ML630Q790_ERR_LOG("[%02x]", i);
        }
        ML630Q790_ERR_LOG("data[%02x]", data[i]);
    }
    ML630Q790_D_LOG("end");

}

static void ml630q790_drv_set_status(enum ml630q790_drv_status_e_type status)
{
    ML630Q790_D_LOG("start");
    atomic_set(&s_drvStatus, (int32_t)status);
    ML630Q790_D_LOG("end");
}

int32_t ml630q790_drv_get_status(void)
{
    ML630Q790_D_LOG("start");
    ML630Q790_D_LOG("end");
    return atomic_read(&s_drvStatus);
}

uint8_t ml630q790_get_shutdown_status(void)
{
    return (uint8_t)(ml630q790_drv_get_status() == ML630Q790_SHUTDOWN);
}

int32_t ml630q790_get_fw_version(uint8_t *fw_ver)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = ML630Q790_RC_OK;

    ML630Q790_D_LOG("start");

    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_MCU_GET_VERSION;

    ret = sns_hostcmd(&cmd, &res, 8, (EXE_HOST_ALL | EXE_HOST_EX_NO_RECOVER), READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((ML630Q790_RC_OK != ret) || (0 != res.err.udata16)) {
        ML630Q790_ERR_LOG("FW Version Get Error[%d][%x]",cmd.prm.ub_prm[0], res.err.udata16);
        return ML630Q790_RC_ERR;
    }

    fw_ver[0] = res.res.ub_res[0];
    fw_ver[1] = res.res.ub_res[1];
    fw_ver[2] = res.res.ub_res[2];
    fw_ver[3] = res.res.ub_res[3];

    ML630Q790_D_LOG("FW Version[%02x][%02x][%02x][%02x]",fw_ver[0], fw_ver[1], fw_ver[2], fw_ver[3]);

    ML630Q790_D_LOG("end - ML630Q790_RC_OK");

    return ret;
}

static void ml630q790_wq_init(void)
{
    int32_t i;
    bool temp_unlock = false;

    ML630Q790_D_LOG("start");
    for(i=0; i<WORK_QUEUE_NUM; i++){
        if(temp_unlock){
            //sensor_com_mutex_unlock();
        }
        cancel_work_sync(&s_tSnsWork[i].work);
        if(temp_unlock){
            //sensor_com_mutex_lock();
        }

        s_tSnsWork[i].status = false;
    }
    SnsWorkCnt = 0;

    ML630Q790_D_LOG("end");
}


static irqreturn_t ml630q790_irq_handler(int32_t irq, void *dev_id)
{

    ML630Q790_D_LOG("start");

    if( irq != g_nIntIrqNo ){
        return IRQ_NONE;
    }

    if( atomic_read(&g_InitializeState) == false ){
        ML630Q790_ERR_LOG("waiting Initialize");
        return IRQ_HANDLED;
    }

    DISABLE_IRQ;
    if( ml630q790_wq_create(sns_wq_int, ml630q790_int_work_func, WORK_OPT_NONE, WQ_TYPE_DUMMYVAL) != ML630Q790_RC_OK){
        ENABLE_IRQ;
    }else{
        ML630Q790_D_LOG("### --> s_tWork_Int");
    }

    ML630Q790_D_LOG("end - IRQ_HANDLED");

    return IRQ_HANDLED;
}

static void ml630q790_int_work_func(struct work_struct *work)
{
    Long lreg;

    u_int8_t sreg30, sreg31, sreg32, sreg33;
    u_int8_t sreg34, sreg35, sreg36, sreg37;

    ML630Q790_D_LOG("start");
    mutex_lock(&ml630q790_irq_mutex);

    ml630q790_device_read(ERROR0, lreg.udata8, 4);

    if(lreg.udata16[1] == 0){
        ml630q790_wq_delete(work) ;
        ENABLE_IRQ;
        goto done;
    }

    ML630Q790_D_LOG("### INTREQ0/1[%x],ERROR0/1[%x]",
                                    lreg.udata16[0],lreg.udata16[1]);

    g_lastCmdError = lreg.udata16[0];

    if(lreg.udata16[1] == 0xFFFF){
        ML630Q790_ERR_LOG("### sns_int_work_func MiconWDT Error[%x][%x]",
                                    lreg.udata16[0],lreg.udata16[1]);
        g_bDevIF_Error = true;
        ml630q790_wq_delete(work);

        DISABLE_IRQ;
        goto done;
    }

    if(lreg.udata16[1] == INTREQ_ERROR){
        ML630Q790_ERR_LOG("### sns_int_work_func Error[%x][%x]",
                                    lreg.udata16[0],lreg.udata16[1]);
        mutex_lock(&s_tDataMutex);
        IntIrqFlg |= INTREQ_ERROR;
        mutex_unlock(&s_tDataMutex);

        wake_up_interruptible(&s_tWaitInt);

        ml630q790_wq_delete(work);
        ENABLE_IRQ;
        goto done;
    }

    if(lreg.udata16[1] & INTREQ_HOST_CMD){
        if(!(IntIrqFlg & INTREQ_HOST_CMD)){
            mutex_lock(&s_tDataMutex);
            IntIrqFlg |= INTREQ_HOST_CMD;
            mutex_unlock(&s_tDataMutex);
            wake_up_interruptible(&s_tWaitInt);
        }
        ML630Q790_D_LOG("### INTREQ_HOST_CMD[%x]",IntIrqFlg);
    }

    if(lreg.udata16[1] & (INTREQ_ACC | INTREQ_MAG | INTREQ_GYRO | INTREQ_FUSION | INTREQ_KCLOG | INTREQ_LOGGING )){
        ML630Q790_D_LOG("### [%x] INTREQ_ACC | INTREQ_MAG | INTREQ_GYRO | INTREQ_FUSION | INTREQ_KCLOG | INTREQ_LOGGING", lreg.udata16[1]);
        if(lreg.udata16[1] & INTREQ_KCLOG){
            atomic_set(&g_MiconDebug, true);
        }
        ml630q790_wq_create(sns_wq, ml630q790_int_app_work_func, WORK_OPT_INT, WQ_TYPE_DUMMYVAL);
    }


    if(lreg.udata16[1]  & INTREQ_NMI){
        ML630Q790_ERR_LOG("### INTREQ_NMI");

        memset( &sreg30, 0x00, sizeof(sreg30));
        memset( &sreg31, 0x00, sizeof(sreg31));
        memset( &sreg32, 0x00, sizeof(sreg32));
        memset( &sreg33, 0x00, sizeof(sreg33));
        memset( &sreg34, 0x00, sizeof(sreg34));
        memset( &sreg35, 0x00, sizeof(sreg35));
        memset( &sreg36, 0x00, sizeof(sreg36));
        memset( &sreg37, 0x00, sizeof(sreg37));

        ml630q790_device_read(RSLT30, &sreg30, 1);
        ml630q790_device_read(RSLT31, &sreg31, 1);
        ml630q790_device_read(RSLT32, &sreg32, 1);
        ml630q790_device_read(RSLT33, &sreg33, 1);
        ml630q790_device_read(RSLT34, &sreg34, 1);
        ml630q790_device_read(RSLT35, &sreg35, 1);
        ml630q790_device_read(RSLT36, &sreg36, 1);
        ml630q790_device_read(RSLT37, &sreg37, 1);



        ML630Q790_ERR_LOG("### result reg 33-30(LR):[%02x][%02x][%02x][%02x]"
                                         ,sreg33, sreg32, sreg31, sreg30);
        ML630Q790_ERR_LOG("### result reg 34:[%x], 35:[%x], 36:[%x], 37:[%x]"
                                         ,sreg34, sreg35, sreg36, sreg37);
    }

    if(lreg.udata16[1]  & INTREQ_EXCP){
        ML630Q790_ERR_LOG("### INTREQ_EXCP");

        memset( &sreg30, 0x00, sizeof(sreg30));
        memset( &sreg31, 0x00, sizeof(sreg31));
        memset( &sreg32, 0x00, sizeof(sreg32));
        memset( &sreg33, 0x00, sizeof(sreg33));
        memset( &sreg34, 0x00, sizeof(sreg34));

        ml630q790_device_read(RSLT30, &sreg30, 1);
        ml630q790_device_read(RSLT31, &sreg31, 1);
        ml630q790_device_read(RSLT32, &sreg32, 1);
        ml630q790_device_read(RSLT33, &sreg33, 1);
        ml630q790_device_read(RSLT34, &sreg34, 1);

        ML630Q790_ERR_LOG("### result reg 33-30(LR):[%02x][%02x][%02x][%02x]"
                                         ,sreg33, sreg32, sreg31, sreg30);
        ML630Q790_ERR_LOG("### result reg 34:[%02x]",sreg34);
    }


    if(lreg.udata16[1]  & INTREQ_KCLOG){
        ML630Q790_ERR_LOG("### INTREQ_KCLOG");

        memset( &sreg30, 0x00, sizeof(sreg30));
        memset( &sreg31, 0x00, sizeof(sreg31));
        memset( &sreg32, 0x00, sizeof(sreg32));
        memset( &sreg33, 0x00, sizeof(sreg33));
        memset( &sreg34, 0x00, sizeof(sreg34));

        ml630q790_device_read(RSLT30, &sreg30, 1);
        ml630q790_device_read(RSLT31, &sreg31, 1);
        ml630q790_device_read(RSLT32, &sreg32, 1);
        ml630q790_device_read(RSLT33, &sreg33, 1);
        ml630q790_device_read(RSLT34, &sreg34, 1);

        ML630Q790_ERR_LOG("### result reg 33-30(LR):[%02x][%02x][%02x][%02x]"
                                         ,sreg33, sreg32, sreg31, sreg30);
        ML630Q790_ERR_LOG("### result reg 34:[%02x]",sreg34);


    }
    ml630q790_wq_delete(work) ;
    ENABLE_IRQ;

done:
    mutex_unlock(&ml630q790_irq_mutex);
    ML630Q790_D_LOG("end");

    return;
}

static void ml630q790_int_app_work_func(struct work_struct *work)
{
    SNS_WorkQueue      *sns_work = container_of(work, SNS_WorkQueue, work);
    int32_t            option = sns_work->option;
    int32_t            type = sns_work->type;

    ML630Q790_D_LOG("start");
    if(ml630q790_drv_get_status() == ML630Q790_SHUTDOWN){
        ML630Q790_ERR_LOG("### Shutdown return !!");
        return;
    }
    mutex_lock(&ml630q790_snsapp_mutex);


    if(option == WORK_OPT_INT) {
        sensor_irq_proc(INTREQ_DUMMY);
    } else if (option == WORK_OPT_FLUSH) {
    	sensor_exec_flush(type);
    }

    ml630q790_wq_delete(work) ;
    mutex_unlock(&ml630q790_snsapp_mutex);
    ML630Q790_D_LOG("end");

    return;
}

static int32_t ml630q790_wq_create( struct workqueue_struct *queue, void (*func)(struct work_struct *), int32_t option, int32_t type )
{
    int32_t ret = ML630Q790_RC_ERR;
    int32_t i;
    unsigned long flags;

    ML630Q790_D_LOG("start queue:%p func:%pf option:%d type:%d", queue, func, option, type);

    if((queue == NULL) || (func == NULL)){
        return ML630Q790_RC_ERR;
    }

    spin_lock_irqsave(&ml630q790_wq_lock, flags);

    ML630Q790_D_LOG("SnsWorkCnt[%d]status[%x]",SnsWorkCnt, s_tSnsWork[SnsWorkCnt].status);

    if(s_tSnsWork[SnsWorkCnt].status == false){

        INIT_WORK( &s_tSnsWork[SnsWorkCnt].work, func );
        s_tSnsWork[SnsWorkCnt].option = option;
        s_tSnsWork[SnsWorkCnt].type = type;

        ret = queue_work( queue, &s_tSnsWork[SnsWorkCnt].work );

        if (ret == 1) {
            s_tSnsWork[SnsWorkCnt].status = true;

            if(++SnsWorkCnt >= WORK_QUEUE_NUM){
                SnsWorkCnt = 0;
            }
            ret = ML630Q790_RC_OK;
        }else{
            ML630Q790_ERR_LOG("queue_work Non Create[%d]",ret);
        }
        g_workqueue_used = 0;
    }else{
        if(g_workqueue_used < WORK_QUEUE_NUM){
            ML630Q790_ERR_LOG("SNS queue_work[%d] used!!",SnsWorkCnt);
            for(i=0; i<WORK_QUEUE_NUM; i++){
                ML630Q790_D_LOG("SnsWorkCnt[%d]status[%x]",i,s_tSnsWork[i].status);
            }
            if(++SnsWorkCnt >= WORK_QUEUE_NUM){
                SnsWorkCnt = 0;
            }
            g_workqueue_used++;
        }
    }

    spin_unlock_irqrestore(&ml630q790_wq_lock, flags);

    ML630Q790_D_LOG("end - return[%d]",ret);

    return ret;
}

static void ml630q790_wq_delete(struct work_struct *work)
{
    int32_t i;
    unsigned long flags;

    ML630Q790_D_LOG("start");

    spin_lock_irqsave(&ml630q790_wq_lock, flags);

    for(i=0; i<WORK_QUEUE_NUM; i++){
        if(&s_tSnsWork[i].work == work){
            s_tSnsWork[i].status = false;
            ML630Q790_D_LOG("hit delete queue[%d]! work:[%p] [%p] ",
                        i, &s_tSnsWork[i].work, work);
            break;
        }
    }

    spin_unlock_irqrestore(&ml630q790_wq_lock, flags);

    ML630Q790_D_LOG("end");

    return ;
}

static int32_t ml630q790_parse_dt( struct spi_device *client )
{
	int value = 0;
	int ret = ML630Q790_RC_OK;
	struct device_node *np = client->dev.of_node;
    uint32_t tmp_fw_version_data = 0x00000000;

    ML630Q790_D_LOG("start");

	value = of_get_named_gpio_flags(np, "LAPIS,rst-gpio", 0, NULL);
	if(value < 0) {
		ML630Q790_ERR_LOG("LAPIS,rst-gpio property not found");
		ret = ML630Q790_RC_ERR;
	}
	g_micon_bdata.rst_gpio = value;
    ML630Q790_D_LOG("rst-gpio[%d]", g_micon_bdata.rst_gpio);

	value = of_get_named_gpio_flags(np, "LAPIS,int-gpio", 0, NULL);
	if(value < 0) {
		ML630Q790_ERR_LOG("LAPIS,int-gpio property not found");
		ret = ML630Q790_RC_ERR;
	}
	g_micon_bdata.int_gpio = value;
    ML630Q790_D_LOG("int-gpio[%d]", g_micon_bdata.int_gpio);

	value = of_get_named_gpio_flags(np, "LAPIS,brmp-gpio", 0, NULL);
	if(value < 0) {
		ML630Q790_ERR_LOG("LAPIS,brmp-gpio property not found");
		ret = ML630Q790_RC_ERR;
	}
	g_micon_bdata.brmp_gpio = value;
    ML630Q790_D_LOG("brmp-gpio[%d]", g_micon_bdata.brmp_gpio);

    ret = of_property_read_u32(np, "micon-fw-version", &tmp_fw_version_data);
    if(ret){
        ML630Q790_ERR_LOG("micon-fw-version read err[%x]", ret);
        return ret;
    }
    fw_version = tmp_fw_version_data; /* module_param */

    ML630Q790_D_LOG("end");

	return ret;
}

static int32_t ml630q790_gpio_init(struct spi_device *client )
{
    int32_t ret;

    ML630Q790_D_LOG("start");

	ret = ml630q790_parse_dt(client);
    if (ret < 0){
        ML630Q790_ERR_LOG("failed to get_gpio_info [%d]",ret);
        return ret;
    }

	ret = gpio_request(g_micon_bdata.rst_gpio, GPIO_RESET_NAME);
    if (ret < 0){
        ML630Q790_ERR_LOG("failed to gpio_request g_micon_bdata.rst_gpio[%d]",ret);
        return ret;
    }

    ret = gpio_direction_output(g_micon_bdata.rst_gpio, 1);
    if (ret < 0){
        ML630Q790_ERR_LOG("failed to gpio_direction_output g_micon_bdata.rst_gpio[%d]",ret);
        goto ERROR_RST;
    }

    ret = gpio_request(g_micon_bdata.brmp_gpio, GPIO_BRMP_NAME);
    if (ret < 0){
        ML630Q790_ERR_LOG("failed to gpio_request g_micon_bdata.brmp_gpio[%d]",ret);
        goto ERROR_RST;
    }

    ret = gpio_direction_output(g_micon_bdata.brmp_gpio, 0);
    if (ret < 0){
        ML630Q790_ERR_LOG("failed to gpio_direction_output g_micon_bdata.brmp_gpio[%d]",ret);
        goto ERROR_BRMP;
    }

    g_nIntIrqNo = gpio_to_irq(g_micon_bdata.int_gpio);
    atomic_set(&g_bIsIntIrqEnable, true);
    ret = gpio_request(g_micon_bdata.int_gpio, GPIO_INT_NAME);
    if (ret < 0){
        ML630Q790_ERR_LOG("failed to gpio_request g_micon_bdata.int_gpio[%d]",ret);
        goto ERROR_BRMP;
    }

    ret = gpio_direction_input(g_micon_bdata.int_gpio);
    if (ret < 0){
        ML630Q790_ERR_LOG("failed to gpio_direction_output g_micon_bdata.int_gpio[%d]",ret);
        goto ERROR_INT;
    }

    ret = request_any_context_irq(g_nIntIrqNo, ml630q790_irq_handler, IRQF_TRIGGER_LOW, GPIO_INT_NAME, NULL);
    if(ret < 0) {
        ML630Q790_ERR_LOG("Failed request_any_context_irq[%d]",ret);
        goto ERROR_INT;
    }

    ML630Q790_D_LOG("end - ML630Q790_RC_OK");

    return ML630Q790_RC_OK;

ERROR_INT:
    gpio_free(g_micon_bdata.int_gpio);
ERROR_BRMP:
    gpio_free(g_micon_bdata.brmp_gpio);
ERROR_RST:
    gpio_free(g_micon_bdata.rst_gpio);
    return ML630Q790_RC_ERR;
}


int32_t ml630q790_spi_setup(struct spi_device *client)
{
    int32_t ret = ML630Q790_RC_OK;
    ML630Q790_D_LOG("start");

    client->bits_per_word = 8;
    if(!client){
        ML630Q790_ERR_LOG("client is NULL");
        return -EINVAL;
    }

    ret = spi_setup(client);
    if(ret < 0) {
        ML630Q790_ERR_LOG("falut spi_setup()-->ret[%d]",ret);
    }

    ML630Q790_D_LOG("end");
    return ret;
}

static void ml630q790_g_arg_Init(void)
{
    spi_error = false;
    mutex_init(&s_tDataMutex);
    mutex_init(&micon_hcres_mutex);
    sema_init(&s_tSnsrSema, 1);
    atomic_set(&g_FWUpdateStatus,false);
    atomic_set(&g_MiconDebug,false);
	atomic_set(&g_bIsResume,true);
    atomic_set(&g_InitializeState,false);
    init_waitqueue_head(&s_tWaitInt);
}

static int32_t client_dev_initialize(void)
{
    int32_t ret = ML630Q790_RC_OK;
    ML630Q790_D_LOG("start");
    ret = sensor_dev_init(client_ic);
    ML630Q790_D_LOG("end");
    return ret;
}
static void ml630q790_BRMP_direction(void)
{
    int32_t    ret;

    ML630Q790_D_LOG("start");

    if (!gpio_is_valid(g_micon_bdata.brmp_gpio)) {
        ML630Q790_ERR_LOG("gpio is invalid");
        return;
    }
    ret = gpio_direction_input(g_micon_bdata.brmp_gpio);

    if(ret < 0 ){
        ML630Q790_ERR_LOG("Error BRMP CTL");
    }

    ML630Q790_D_LOG("end");
}

static void ml630q790_FW_BRMP_ctrl(void)
{
    ML630Q790_D_LOG("start");

    if (!gpio_is_valid(g_micon_bdata.brmp_gpio) || !gpio_is_valid(g_micon_bdata.rst_gpio)) {
        ML630Q790_ERR_LOG("gpio is invalid");
        return;
    }

    gpio_direction_output(g_micon_bdata.brmp_gpio, 1);

    gpio_direction_output(g_micon_bdata.rst_gpio, 0);

    usleep_range(510, 600);

    gpio_set_value(g_micon_bdata.rst_gpio, 1);

    msleep(20);

    gpio_set_value(g_micon_bdata.brmp_gpio, 0);

    g_micon_error = false;

    ML630Q790_D_LOG("end");
}


int32_t ml630q790_update_fw_exe(bool boot, uint8_t *arg_iData, uint32_t arg_iLen)
{
    uint8_t reg = 0xFF;
    int32_t i;
    int32_t ret;
    HostCmd cmd;
    HostCmdRes res;
    int32_t size;
    int32_t send_num = 0;
    uint32_t chksum = 0;
    uint8_t chksum_data[4] = {0x00, 0x00, 0x00, 0x00};
    uint8_t    fw_ver[4];
    int cnt = 170;

    ML630Q790_D_LOG("start");

    memset(&res, 0x00, sizeof(HostCmdRes));

    ML630Q790_D_LOG("boot[%d] Data[%p] Len[%d]", boot, arg_iData, arg_iLen);

    if((arg_iData == NULL) || (arg_iLen == 0)){
        ML630Q790_ERR_LOG("error arg_iData:0x%p arg_iLen:%d",arg_iData,arg_iLen);
        DISABLE_IRQ;
        return ML630Q790_RC_ERR;
    }

    ENABLE_IRQ;
    if(!boot){
        cmd.cmd.udata16 = HC_MCU_FUP_START;
        cmd.prm.ub_prm[0] = 0x55;
        cmd.prm.ub_prm[1] = 0xAA;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
        if(ret != ML630Q790_RC_OK) {
            ML630Q790_ERR_LOG("Communication Error!");
            DISABLE_IRQ;
            return ML630Q790_RC_ERR;
        }
        if(res.err.udata16 == ERROR_FUP_CERTIFICATION) {
            ML630Q790_ERR_LOG("Certification Error!");
            DISABLE_IRQ;
            return ML630Q790_RC_ERR;
        }
    }

    msleep(30);

    reg = 0x04;
    ml630q790_device_write(CFG, &reg, sizeof(reg));

    reg = 0x00;
    ml630q790_device_write(INTMASK0, &reg, sizeof(reg));
    ml630q790_device_write(INTMASK1, &reg, sizeof(reg));

    cmd.cmd.udata16 = HC_MCU_SET_PCON;
    cmd.prm.ub_prm[0] = 0x11;
    cmd.prm.ub_prm[1] = 0x11;
    cmd.prm.ub_prm[2] = 0x00;
    cmd.prm.ub_prm[3] = 0x00;
    cmd.prm.ub_prm[4] = 0x00;
    cmd.prm.ub_prm[5] = 0x00;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_RSLT);
    if((ML630Q790_RC_OK != ret) || (0 != res.err.udata16)) {
        ML630Q790_ERR_LOG("PortSettint err[%x]", res.err.udata16);
        DISABLE_IRQ;
        return ML630Q790_RC_ERR;
    }

    ML630Q790_D_LOG("Check Firmware Mode.");
    cmd.cmd.udata16 = HC_MCU_GET_VERSION;
    ret = sns_hostcmd(&cmd, &res, 8, EXE_HOST_ALL, READ_RSLT);
    if(ret != ML630Q790_RC_OK) {
        ML630Q790_ERR_LOG("Communication Error!");
        DISABLE_IRQ;
        return ML630Q790_RC_ERR;
    }

    if(res.res.ub_res[2] != 0x01){
        ML630Q790_ERR_LOG("Version check Error!");
        ML630Q790_D_LOG("FW Version[%02x] [%02x] [%02x] [%02x]",
                      res.res.ub_res[0], res.res.ub_res[1], res.res.ub_res[2], res.res.ub_res[3]);
        DISABLE_IRQ;
        return ML630Q790_RC_ERR;
    }

    ML630Q790_D_LOG("FW Version[%02x] [%02x] [%02x] [%02x]",
                  res.res.ub_res[0], res.res.ub_res[1], res.res.ub_res[2], res.res.ub_res[3]);

    ML630Q790_D_LOG("Flash Clear.");
    cmd.cmd.udata16 = HC_MCU_FUP_ERASE;
    cmd.prm.ub_prm[0] = 0xAA;
    cmd.prm.ub_prm[1] = 0x55;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_RSLT);
    if(ret != ML630Q790_RC_OK) {
        ML630Q790_ERR_LOG("Communication Error!");
        DISABLE_IRQ;
        return ML630Q790_RC_ERR;
    }

    if(res.err.udata16 == ERROR_FUP_CERTIFICATION) {
        ML630Q790_ERR_LOG("Certification Error!");
        DISABLE_IRQ;
        return ML630Q790_RC_ERR;
    }

    while(cnt > 0) {
        ml630q790_device_read(STATUS, &reg, sizeof(reg));
        if(reg == 0x00) {
            ML630Q790_D_LOG("STATUS OK!!");
            break;
        } else {
            msleep(10);
            cnt--;
        }
    }

    if(cnt <= 0){
        ML630Q790_ERR_LOG("Flash Clear STATUS Error!");
        DISABLE_IRQ;
        return ML630Q790_RC_ERR;
    }

    send_num = arg_iLen / FUP_MAX_RAMSIZE;
    send_num = (arg_iLen % FUP_MAX_RAMSIZE) ? send_num+1 : send_num;

    for(i=0; i < arg_iLen; i++){
        chksum += (uint32_t)arg_iData[i];
    }
    for(i=0; i < send_num; i++){
        if((arg_iLen - (FUP_MAX_RAMSIZE * i)) >= FUP_MAX_RAMSIZE){
            size = FUP_MAX_RAMSIZE;
        } else {
            size = arg_iLen % FUP_MAX_RAMSIZE;
        }
        ret = sns_spi_ram_write_proc(FIFO, &arg_iData[i * FUP_MAX_RAMSIZE], size);
        if(ret != ML630Q790_RC_OK) {
            ML630Q790_ERR_LOG("RAM Write Error!");
            DISABLE_IRQ;
            return ML630Q790_RC_ERR_RAMWRITE;
        }
        cmd.cmd.udata16 = HC_MCU_FUP_WRITE_FIFO;
        ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_RSLT);
        if((ret != ML630Q790_RC_OK) || (0 != res.err.udata16)) {
            ML630Q790_ERR_LOG("Communication Error! err[%x]",res.err.udata16);
            DISABLE_IRQ;
            return ML630Q790_RC_ERR;
        }
    }

    ML630Q790_D_LOG("Self Check.");
    chksum_data[3] = (uint8_t)((chksum >> 24) & 0x000000ff);
    chksum_data[2] = (uint8_t)((chksum >> 16) & 0x000000ff);
    chksum_data[1] = (uint8_t)((chksum >> 8) & 0x000000ff);
    chksum_data[0] = (uint8_t)(chksum & 0x000000ff);
    ret = ml630q790_device_write(FIFO, chksum_data, sizeof(chksum_data));
    if(ret != ML630Q790_RC_OK) {
        ML630Q790_ERR_LOG("chksum FIFO Write Error!");
        DISABLE_IRQ;
        return ML630Q790_RC_ERR_RAMWRITE;
    }
    cmd.cmd.udata16 = HC_MCU_FUP_WRITE_FIFO;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_RSLT);
    if(ret != ML630Q790_RC_OK) {
        ML630Q790_ERR_LOG("Communication Error!");
        DISABLE_IRQ;
        return ML630Q790_RC_ERR;
    }
    cmd.cmd.udata16 = HC_MCU_SELF_CHK_FW;
    ret = sns_hostcmd(&cmd, &res, 1, EXE_HOST_ALL | EXE_HOST_EX_NO_RECOVER, READ_RSLT);
    ML630Q790_D_LOG("HC_MCU_SELF_CHK_FW[%d] err res[%x] err[%x]",
                  ret, res.res.ub_res[0], res.err.udata16);
    ML630Q790_D_LOG("End Firmware Update.");

    cmd.cmd.udata16 = HC_MCU_FUP_END;
    sns_hostcmd(&cmd, &res, 0, EXE_HOST_ERR, READ_RSLT);
    msleep(300);

    reg = 0x04;
    ml630q790_device_write(CFG, &reg, sizeof(reg));

    reg = 0x00;
    ml630q790_device_write(INTMASK0, &reg, sizeof(reg));
    ml630q790_device_write(INTMASK1, &reg, sizeof(reg));

    ML630Q790_D_LOG("Check User program mode.");
    ret = ml630q790_get_fw_version(fw_ver);
    g_nFWVersion = ML630Q790_GET_FW_VER(fw_ver);
    if(ret != ML630Q790_RC_OK ){
        g_nFWVersion = ML630Q790_FW_VER_NONE;
        ML630Q790_ERR_LOG("Version not get.");
    }
    ML630Q790_D_LOG("Sensor FW Version.[%08x]",g_nFWVersion);
    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = HC_MCU_GET_VERSION;
    ret = sns_hostcmd(&cmd, &res, 8, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if(ret != ML630Q790_RC_OK) {
        ML630Q790_ERR_LOG("Communication Error!");
        DISABLE_IRQ;
        return ML630Q790_RC_ERR;
    }
    if(res.res.ub_res[2] != 0x00){
        ML630Q790_ERR_LOG("Version check Error!");
        DISABLE_IRQ;
        return ML630Q790_RC_ERR;
    }
    DISABLE_IRQ;
    ML630Q790_D_LOG("end");

    return ML630Q790_RC_OK;
}

int32_t ml630q790_update_fw(uint8_t *arg_iData, uint32_t arg_iLen)
{
    int32_t    ret = ML630Q790_RC_OK;
    uint32_t   i;

    ML630Q790_D_LOG("start");

    atomic_set(&g_FWUpdateStatus,true);

    ML630Q790_D_LOG("F/W Update Start...");
    DISABLE_IRQ;
    for(i = 0; i < SPI_RETRY_NUM; i++){
        ml630q790_FW_BRMP_ctrl();
        ret = ml630q790_update_fw_exe(true, arg_iData, arg_iLen);
        if(ret != ML630Q790_RC_ERR_RAMWRITE)
            break;
    }
    ML630Q790_D_LOG("F/W Initialize Start...");
    
    ret |= client_dev_initialize();
    if (gpio_is_valid(g_micon_bdata.brmp_gpio)) {
        ret |= gpio_direction_input(g_micon_bdata.brmp_gpio);
    }

    atomic_set(&g_FWUpdateStatus,false);

    if(ret != 0){
        ML630Q790_ERR_LOG("error(sns_update_fw_exe) : fw_update");
        return ML630Q790_RC_ERR;
    }

    ML630Q790_D_LOG("end - return[%d]",ret);
    return ret;
}
static int32_t ml630q790_get_fw_version_data(void)
{
    ML630Q790_D_LOG("fw_version[0x%08x]", fw_version);
    return fw_version;
}
int32_t ml630q790_update_fw_seq(uint8_t *arg_iData, uint32_t arg_iLen)
{
    int32_t    ret = ML630Q790_RC_OK;
    uint8_t    fw_ver[4];
    HostCmd    cmd;
    HostCmdRes res;
    int32_t    i;
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    s64 func_time_us, enter_us, exit_us;
    ktime_t enter, exit;
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */

    ML630Q790_D_LOG("start");

#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    enter = ktime_get();
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
    atomic_set(&g_FWUpdateStatus,true);
    
    ret = ml630q790_get_fw_version(fw_ver);
    g_nFWVersion = ML630Q790_GET_FW_VER(fw_ver);
    if(ret != ML630Q790_RC_OK){
        ML630Q790_ERR_LOG("Get Version Error!!");
        g_nFWVersion = ML630Q790_FW_VER_NONE;
    }

    ML630Q790_D_LOG("Now[%x] Base[%x]",g_nFWVersion, ml630q790_get_fw_version_data());

    if(g_nFWVersion != ml630q790_get_fw_version_data()){
        ML630Q790_D_LOG("Need to update F/W Version");

        mutex_lock(&micon_hcres_mutex);
        cmd.cmd.udata16 = HC_MCU_SELF_CHK_FW;
        ret = sns_hostcmd(&cmd, &res, 1, EXE_HOST_ALL | EXE_HOST_EX_NO_RECOVER, READ_FIFO);
        mutex_unlock(&micon_hcres_mutex);
        ML630Q790_D_LOG("HC_MCU_SELF_CHK_FW[%d] err res[%x] err[%x]", ret, res.res.ub_res[0], res.err.udata16);
        if(1) {
            ML630Q790_D_LOG("F/W Update Start...");
            DISABLE_IRQ;
            for(i = 0; i < SPI_RETRY_NUM; i++){
                ml630q790_FW_BRMP_ctrl();
                ret = ml630q790_update_fw_exe(true, arg_iData, arg_iLen);
                if(ret != ML630Q790_RC_ERR_RAMWRITE)
                    break;
            }
        }else if(0x00 == res.res.ub_res[0]){
            ML630Q790_D_LOG("HC_MCU_SELF_CHK_FW(-) OK!!");
        }
        ML630Q790_D_LOG("F/W Initialize Start...");
        ret |= client_dev_initialize();
    }else{
        ML630Q790_D_LOG("None update F/W Version");
    }
    ML630Q790_D_LOG("F/W Update Check Completed...");

    if (gpio_is_valid(g_micon_bdata.brmp_gpio)) {
        ret |= gpio_direction_input(g_micon_bdata.brmp_gpio);
    }

#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    exit = ktime_get();
    func_time_us = ktime_to_us(ktime_sub(exit, enter));
    enter_us = ktime_to_us(enter);
    exit_us = ktime_to_us(exit);
    printk(KERN_NOTICE "[IT_TEST] %s: bgn:%lld end:%lld time:%lld\n",__func__,enter_us,exit_us,func_time_us);
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
    atomic_set(&g_FWUpdateStatus,false);

    ML630Q790_D_LOG("end - return[%d]",ret);
    return ret;
}

static void get_DMcmd_host_cmd_res(uint32_t* res)
{
    uint32_t i;

    ML630Q790_D_LOG("start");

    for(i=0;i<128;i++){
        res[i] = (uint32_t)diag_host_cmd_res.res.ub_res[i];
    }

    ML630Q790_D_LOG("end");
}

int32_t set_DMcmd_host_cmd(int32_t* req_cmd, int32_t* req_param)
{
    int32_t    ret = ML630Q790_RC_OK;
    HostCmd cmd;
    HostCmdRes res;
    uint32_t i;

    ML630Q790_D_LOG("start");

    mutex_lock(&micon_hcres_mutex);
    cmd.cmd.udata16 = (uint16_t)((req_cmd[0] << 8) | req_cmd[1]);
    for(i=0;i<16;i++){
        cmd.prm.ub_prm[i] = (uint8_t)req_param[i];
    }

    ret = sns_hostcmd(&cmd, &res, 128, EXE_HOST_ALL, READ_FIFO);
    mutex_unlock(&micon_hcres_mutex);
    if((ML630Q790_RC_OK != ret) || (0 != res.err.udata16)) {
        ML630Q790_ERR_LOG("sns_acc_set_host_cmd[%x] err[%x]", cmd.cmd.udata16, res.err.udata16);
    }

    memcpy(&diag_host_cmd_res, &res, sizeof(HostCmdRes));

    ML630Q790_D_LOG("end - return[%d]",ret);

    return ret;
}

static ssize_t ml630q790_fw_version_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    uint8_t  fw_ver[4];
    uint32_t ret;
    uint32_t fw_version;

    ML630Q790_D_LOG("start");
    mutex_lock(&ml630q790_mutex);

    ret = ml630q790_get_fw_version(fw_ver);
    fw_version = ML630Q790_GET_FW_VER(fw_ver);
    ML630Q790_D_LOG("fw_version[%x]",(int)fw_version);
    if(ret != ML630Q790_RC_OK){
        fw_version = ML630Q790_FW_VER_NONE;
        ML630Q790_ERR_LOG("fail get fw ver-->set ver[%x]",(int)fw_version);
    }

    mutex_unlock(&ml630q790_mutex);
    ML630Q790_D_LOG("end - return[%d]",(int)fw_version);
    return sprintf(buf, "%x\n", fw_version);
}
static ssize_t ml630q790_fw_version_chk_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    uint8_t  fw_ver[4];
    uint32_t ret;
    uint32_t fw_version;
    uint32_t fw_chk = ML630Q790_FW_VER_CHK_NG;

    ML630Q790_D_LOG("start");
    mutex_lock(&ml630q790_mutex);

    ret = ml630q790_get_fw_version(fw_ver);
    fw_version = ML630Q790_GET_FW_VER(fw_ver);
    ML630Q790_D_LOG("fw_version[%x]",(int)fw_version);
    if(ret != ML630Q790_RC_OK){
        fw_version = ML630Q790_FW_VER_NONE;
        ML630Q790_ERR_LOG("fail get fw ver-->set ver[%x]",(int)fw_version);
    }

    if(fw_version == ML630Q790_FW_VER_DATA){
        fw_chk = ML630Q790_FW_VER_CHK_OK;
        ML630Q790_I_LOG("fw ver chk OK: fw_chk[%x]",(int)fw_chk);
    } else {
        fw_chk = ML630Q790_FW_VER_CHK_NG;
        ML630Q790_I_LOG("fw ver chk NG: fw_chk[%x]",(int)fw_chk);
    }

    mutex_unlock(&ml630q790_mutex);
    ML630Q790_D_LOG("end - return[%d]",(int)fw_chk);
    return sprintf(buf, "%x\n", fw_chk);
}
static ssize_t ml630q790_fw_update_sq_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    int32_t drv_status;
    int32_t data_length = 0;
    unsigned long data = 0;
    uint8_t *data_addr = NULL;
    uint32_t len;
    uint8_t *fw_data = NULL;
    uint32_t ret;

    ML630Q790_D_LOG("start");

    drv_status = ml630q790_drv_get_status();
    if( ML630Q790_NORMAL != drv_status ){
        ML630Q790_ERR_LOG("bad status-->drv_status[%d]",
                       drv_status );
        return count;
    }
    sscanf(buf, "%lx %d",&data, &data_length);
    data_addr = (uint8_t*)data;
    len = (uint32_t)data_length;
    if((data_addr == NULL) || (len == 0)){
        ML630Q790_ERR_LOG("bad param-->data_addr[0x%016llx] len[%d]",
                       (uint64_t)data_addr,len);
        return count;
    }
    if((len % 4) != 0){
        ML630Q790_ERR_LOG("bad param-->len[%d]:is not a multiple of 4",
                       (int)len);
        return count;
    }

    fw_data = (uint8_t *)kmalloc( len, GFP_KERNEL );

    if(fw_data == NULL){
        ML630Q790_ERR_LOG("fail kmalloc--> NULL");
        return -ENOMEM;
    }

    ret = copy_from_user( fw_data, data_addr, len );

    if( ret != 0 )
    {
        ML630Q790_ERR_LOG("fail copy_from_user-->ret[%d]",
                       (int)ret);
        kfree( fw_data );
        return count;
    }

    mutex_lock(&ml630q790_mutex);
    ml630q790_drv_set_status( ML630Q790_FW_UPDATING );
    ret = ml630q790_update_fw_seq(fw_data, len);
    ml630q790_drv_set_status( ML630Q790_NORMAL );
    mutex_unlock(&ml630q790_mutex);
    kfree( fw_data );

    if(ret != 0){
        ML630Q790_ERR_LOG("fail ml630q790_update_fw_seq-->ret[%d]",
                       (int)ret);
        return count;
    }

    ML630Q790_D_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t ml630q790_fw_update_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    int32_t drv_status;
    int32_t data_length = 0;
    unsigned long data = 0;
    uint8_t *data_addr = NULL;
    uint32_t len;
    uint8_t *fw_data = NULL;
    uint32_t ret;

    ML630Q790_D_LOG("start");

    drv_status = ml630q790_drv_get_status();
    if( ML630Q790_NORMAL != drv_status ){
        ML630Q790_ERR_LOG("bad status-->drv_status[%d]",
                       drv_status );
        return count;
    }
    sscanf(buf, "%lx %d",&data, &data_length);
    data_addr = (uint8_t*)data;
    len = (uint32_t)data_length;
    if((data_addr == NULL) || (len == 0)){
        ML630Q790_ERR_LOG("bad param-->data_addr[0x%016llx] len[%d]",
                       (uint64_t)data_addr,len);
        return count;
    }
    if((len % 4) != 0){
        ML630Q790_ERR_LOG("bad param-->len[%d]:is not a multiple of 4",
                       (int)len);
        return count;
    }

    fw_data = (uint8_t *)kmalloc( len, GFP_KERNEL );

    if(fw_data == NULL){
        ML630Q790_ERR_LOG("fail kmalloc--> NULL");
        return -ENOMEM;
    }

    ret = copy_from_user( fw_data, data_addr, len );

    if( ret != 0 )
    {
        ML630Q790_ERR_LOG("fail copy_from_user-->ret[%d]",
                       (int)ret);
        kfree( fw_data );
        return count;
    }

    mutex_lock(&ml630q790_mutex);
    ml630q790_drv_set_status( ML630Q790_FW_UPDATING );
    ret = ml630q790_update_fw_seq(fw_data, len);
    ml630q790_drv_set_status( ML630Q790_NORMAL );
    mutex_unlock(&ml630q790_mutex);
    kfree( fw_data );

    if(ret != 0){
        ML630Q790_ERR_LOG("fail ml630q790_update_fw_seq-->ret[%d]",
                       (int)ret);
        return count;
    }

    ML630Q790_D_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t ml630q790_brmp_ctl_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    unsigned long direction;
    int32_t ret;

    ML630Q790_D_LOG("start");

    ret = kstrtoul(buf, 10, &direction);
    if (ret < 0){
        ML630Q790_ERR_LOG("kstrtoul()-->ret[%d]",(int)ret);
        return count;
    }

    if(direction == 1){
        ML630Q790_D_LOG("BRMP port --> set input");
        ml630q790_BRMP_direction();
    }

    ML630Q790_D_LOG("end - return[%d]",(int)count);
    return count;}

static ssize_t ml630q790_host_cmd_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t ret = 0;
    uint32_t res[128];

    mutex_lock(&ml630q790_mutex);
    get_DMcmd_host_cmd_res(res);
    mutex_unlock(&ml630q790_mutex);

    ret = sprintf( buf, "%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n", 
                    res[0],res[1],res[2],res[3],
                    res[4],res[5],res[6],res[7],
                    res[8],res[9],res[10],res[11],
                    res[12],res[13],res[14],res[15],
                    res[16],res[17],res[18],res[19],
                    res[20],res[21],res[22],res[23],
                    res[24],res[25],res[26],res[27],
                    res[28],res[29],res[30],res[31],
                    res[32],res[33],res[34],res[35],
                    res[36],res[37],res[38],res[39],
                    res[40],res[41],res[42],res[43],
                    res[44],res[45],res[46],res[47],
                    res[48],res[49],res[50],res[51],
                    res[52],res[53],res[54],res[55],
                    res[56],res[57],res[58],res[59],
                    res[60],res[61],res[62],res[63],
                    res[64],res[65],res[66],res[67],
                    res[68],res[69],res[70],res[71],
                    res[72],res[73],res[74],res[75],
                    res[76],res[77],res[78],res[79],
                    res[80],res[81],res[82],res[83],
                    res[84],res[85],res[86],res[87],
                    res[88],res[89],res[90],res[91],
                    res[92],res[93],res[94],res[95],
                    res[96],res[97],res[98],res[99],
                    res[100],res[101],res[102],res[103],
                    res[104],res[105],res[106],res[107],
                    res[108],res[109],res[110],res[111],
                    res[112],res[113],res[114],res[115],
                    res[116],res[117],res[118],res[119],
                    res[120],res[121],res[122],res[123],
                    res[124],res[125],res[126],res[127] );

    return ret;
}

static ssize_t ml630q790_host_cmd_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    uint32_t ret;
    uint32_t req_cmd[2];
    uint32_t req_param[16];

    sscanf( buf, "%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",
            &req_cmd[0],
            &req_cmd[1],
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
            &req_param[15] );

    mutex_lock(&ml630q790_mutex);
    ret = set_DMcmd_host_cmd(req_cmd, req_param);
    mutex_unlock(&ml630q790_mutex);

    return count;
}

static ssize_t ml630q790_reg_rw_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t ret = 0;
    uint8_t *res = dmcmd_reg_rw_rslt;
    ML630Q790_D_LOG("start");

    ret = sprintf( buf, "%x"
        " %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x"
        " %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x"
        " %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x"
        " %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",
                    res[0],res[1],res[2],res[3],
                    res[4],res[5],res[6],res[7],
                    res[8],res[9],res[10],res[11],
                    res[12],res[13],res[14],res[15],
                    res[16],res[17],res[18],res[19],
                    res[20],res[21],res[22],res[23],
                    res[24],res[25],res[26],res[27],
                    res[28],res[29],res[30],res[31],
                    res[32],res[33],res[34],res[35],
                    res[36],res[37],res[38],res[39],
                    res[40],res[41],res[42],res[43],
                    res[44],res[45],res[46],res[47],
                    res[48],res[49],res[50],res[51],
                    res[52],res[53],res[54],res[55],
                    res[56],res[57],res[58],res[59],
                    res[60],res[61],res[62],res[63],
                    res[64] );

    ML630Q790_D_LOG("end");
    return ret;
}

static ssize_t ml630q790_reg_rw_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    uint32_t ret;
    uint32_t data[64] = {0};
    uint8_t  data_uint8[64];
    uint32_t addr = 0;
    uint32_t len = 0;
    int      i;
    ML630Q790_D_LOG("start");

    memset(dmcmd_reg_rw_rslt, 0x00, sizeof(dmcmd_reg_rw_rslt));
    dmcmd_reg_rw_rslt[0] = 0xFF;

    sscanf(buf, "%x %x"
        " %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x"
        " %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x"
        " %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x"
        " %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",
        &addr,
        &len,
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
        &data[63]);

    if ((len == 0) || (len > 0x40) || (addr >= 0x100) ||
        (((addr & 0x7F) + len) > 0x80)) {
        ML630Q790_ERR_LOG("bad parm addr[0x%02x] len[0x%02x]", addr, len);
        return count;
    }

    if (addr & 0x80) {
        mutex_lock(&ml630q790_mutex);
        ret  = ml630q790_device_read(addr & 0x7F, &dmcmd_reg_rw_rslt[0x01], len);
        mutex_unlock(&ml630q790_mutex);
        if (ret == ML630Q790_RC_OK) {
            ML630Q790_D_LOG("sns_device_read OK addr=0x%x len=0x%x", addr & 0x7F, len);
            dmcmd_reg_rw_rslt[0] = 0x00;
        } else {
            ML630Q790_ERR_LOG("sns_device_read NG");
        }
    } else {
        for (i = 0; i < len; i++)
            data_uint8[i] = (uint8_t)data[i];

        mutex_lock(&ml630q790_mutex);
        ret = ml630q790_device_write(addr, data_uint8, len);
        mutex_unlock(&ml630q790_mutex);
        if (ret == ML630Q790_RC_OK) {
            ML630Q790_D_LOG("sns_device_write OK");
            dmcmd_reg_rw_rslt[0] = 0x00;
        } else {
            ML630Q790_ERR_LOG("sns_device_write NG");
        }
    }

    ML630Q790_D_LOG("end");
    return count;
}

static DEVICE_ATTR(fw_version,
    S_IRUSR|S_IRGRP,
    ml630q790_fw_version_show,
    NULL
);
static DEVICE_ATTR(fw_version_chk,
    S_IRUSR|S_IRGRP,
    ml630q790_fw_version_chk_show,
    NULL
);
static DEVICE_ATTR(fw_update_sq,
    S_IWUSR|S_IWGRP,
    NULL,
    ml630q790_fw_update_sq_store
);
static DEVICE_ATTR(fw_update,
    S_IWUSR|S_IWGRP,
    NULL,
    ml630q790_fw_update_store
);
static DEVICE_ATTR(brmp_ctl,
    S_IWUSR|S_IWGRP,
    NULL,
    ml630q790_brmp_ctl_store
);
static DEVICE_ATTR(host_cmd,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    ml630q790_host_cmd_show,
    ml630q790_host_cmd_store
);
static DEVICE_ATTR(reg_rw,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    ml630q790_reg_rw_show,
    ml630q790_reg_rw_store
);


static struct attribute *ml630q790_attributes[] = {
	&dev_attr_fw_version.attr,
	&dev_attr_fw_version_chk.attr,
	&dev_attr_fw_update_sq.attr,
	&dev_attr_fw_update.attr,
	&dev_attr_brmp_ctl.attr,
	&dev_attr_host_cmd.attr,
	&dev_attr_reg_rw.attr,
	NULL,
};

static struct attribute_group attr_group = {
		    .attrs = ml630q790_attributes,
};

static struct of_device_id sensor_micon_match_table[] = {
    { .compatible = SENSOR_MICON_DRIVER_NAME,},
    { },
};

static SIMPLE_DEV_PM_OPS(ml630q790_pm, ml630q790_suspend, ml630q790_resume);

static struct spi_driver ml630q790_driver = {
    .probe       = ml630q790_probe,
    .driver = {
        .name    = SENSOR_MICON_DRIVER_NAME,
        .bus     = &spi_bus_type,
        .owner   = THIS_MODULE,
        .of_match_table = sensor_micon_match_table,
        .pm = &ml630q790_pm,
    },
    .remove      = ml630q790_remove,
//    .suspend     = ml630q790_suspend,
//    .resume      = ml630q790_resume,
    .shutdown    = ml630q790_shutdown,
};

static int ml630q790_probe(struct spi_device *client)
{
	int ret = ML630Q790_RC_OK;
    struct kobject *kobj;

	ml630q790_g_arg_Init();
	ret = ml630q790_gpio_init(client);
	if(ret != ML630Q790_RC_OK){
        spi_error = true;
		ML630Q790_ERR_LOG("Failed ml630q790_parse_dt[%d]",ret);
		return -ENODEV;
	}
	ml630q790_spi_setup(client);
	client_ic = client;
	kobj = kobject_create_and_add(ML630Q790_SYSFS_FOLDER, kernel_kobj);
    if(ret < 0){
		ML630Q790_ERR_LOG("Failure in creating %s sysfs dir", ML630Q790_SYSFS_FOLDER);
		goto err_kobj_create;
    }

	ML630Q790_D_LOG("  /sys/kernel/%s directory created \n",ML630Q790_SYSFS_FOLDER);

	ret = sysfs_create_group(kobj, &attr_group);
	if(ret < 0) {
		ML630Q790_ERR_LOG("Failure in creating sysfs entries");
		goto err_sysfs_create;
	}

	ret = sensor_dev_init(client);
    if(ret){
		ML630Q790_ERR_LOG("Failure in sensor driver init process.");
        goto err_sns_drv_init;
    }

	return ret;
err_sns_drv_init:
    sysfs_remove_group(kobj, &attr_group);
err_sysfs_create:
    kobject_put(kobj);
err_kobj_create:
    ML630Q790_ERR_LOG("probe failed. ret[%d]", ret);
    return ret;
}
static int ml630q790_remove(struct spi_device *client)
{
	return sensor_dev_remove(client);
}
static int ml630q790_client_suspend_proc(struct device *dev)
{
    int32_t ret = ML630Q790_RC_OK;
    ML630Q790_D_LOG("start");
	ret |= sensor_dev_suspend(dev);
    ml630q790_drv_set_status(ML630Q790_SUSPEND);
    ML630Q790_D_LOG("end");
    return ret;
}
static int ml630q790_suspend(struct device *dev)
{
	int ret = ML630Q790_RC_OK;
	ret = ml630q790_client_suspend_proc(dev);
	if(ret != ML630Q790_RC_OK){
		ML630Q790_ERR_LOG("Failed to sensor_micon_suspend[%d]",ret);
	}
    if(sns_wq_int != NULL){
        ML630Q790_D_LOG("sns_wq_int");
        flush_workqueue(sns_wq_int);
    }

    if(sns_wq != NULL){
        ML630Q790_D_LOG("sns_wq");
        flush_workqueue(sns_wq);
    }

    atomic_set(&g_bIsResume,false);
	return 0;
}

static int ml630q790_client_resume_proc(struct device *dev)
{
    int32_t ret = ML630Q790_RC_OK;
    ML630Q790_D_LOG("start");
	ret |= sensor_dev_resume(dev);
    ml630q790_drv_set_status(ML630Q790_NORMAL);
    ML630Q790_D_LOG("end");
    return ret;
}
static int ml630q790_resume(struct device *dev)
{
    atomic_set(&g_bIsResume,true);
	return ml630q790_client_resume_proc(dev);
}

static int ml630q790_client_shutdown_proc(struct spi_device *client)
{
    int32_t ret = ML630Q790_RC_OK;
    ML630Q790_D_LOG("start");
	sensor_dev_shutdown(client);
    ml630q790_drv_set_status(ML630Q790_SHUTDOWN);
    ML630Q790_D_LOG("end");
    return ret;
}

static void ml630q790_shutdown(struct spi_device *client)
{
    ml630q790_drv_set_status(ML630Q790_SHUTDOWN);
    if(ml630q790_drv_get_status() == ML630Q790_POWER_OFF){

        ml630q790_client_shutdown_proc(client);

        mutex_lock(&ml630q790_snsapp_mutex);
        mutex_lock(&ml630q790_irq_mutex);

        DISABLE_IRQ;
        ml630q790_wq_init();

        if(gpio_is_valid(g_micon_bdata.int_gpio)) {
            gpio_free(g_micon_bdata.int_gpio);
        }

        if(gpio_is_valid(g_micon_bdata.rst_gpio)) {
            gpio_free(g_micon_bdata.rst_gpio);
        }

        if(sns_wq_int != NULL){
            ML630Q790_D_LOG("sns_wq_int");
            destroy_workqueue(sns_wq_int);
            sns_wq_int = NULL;
        }

        if(sns_wq != NULL){
            ML630Q790_D_LOG("sns_wq");
            destroy_workqueue(sns_wq);
            sns_wq = NULL;
        }
        //wake_lock_destroy(&g_pedo_wakelock);

        mutex_unlock(&ml630q790_irq_mutex);
        mutex_unlock(&ml630q790_snsapp_mutex);
    }

    ml630q790_drv_set_status(ML630Q790_POWER_OFF);
}

void ml630q790_sns_set_flush(int32_t type){
    ML630Q790_D_LOG("start type[%d]", type);
    ml630q790_wq_create(sns_wq, ml630q790_int_app_work_func, WORK_OPT_FLUSH, type);
    ML630Q790_D_LOG("end");
}

void ml630q790_ope_wake_irq(bool enable)
{
    ML630Q790_D_LOG("start");
    if(enable == true){
        enable_irq_wake(g_nIntIrqNo);
        ML630Q790_D_LOG("enable_irq_wake");
    } else {
        disable_irq_wake(g_nIntIrqNo);
        ML630Q790_D_LOG("disable_irq_wake");
    }
    ML630Q790_D_LOG("end");
}

int32_t ml630q790_initialize_micondrv(void)
{
    HostCmd cmd;
    HostCmdRes res;
    int32_t ret = ML630Q790_RC_OK;
    bool  status_ok = false;
    uint8_t reg;
    int32_t cnt;
    int32_t cnt_hw_reset = 0;

    ML630Q790_D_LOG("start");

    DISABLE_IRQ;

    ml630q790_wq_init();

    if (!gpio_is_valid(g_micon_bdata.rst_gpio)) {
        ML630Q790_ERR_LOG("gpio is invalid");
        return ML630Q790_RC_ERR;
    }

    while (1) {
        gpio_set_value(g_micon_bdata.rst_gpio, 0);
        udelay(500);
        gpio_set_value(g_micon_bdata.rst_gpio, 1);

        msleep(20);

#ifdef CONFIG_USE_GPIOMUX
        //msm_gpiomux_install(msm_spi_sens_configs, ARRAY_SIZE(msm_spi_sens_configs));
#else
        //ml630q790_pinctrl_select(true); //cannnot operate pinctrl setting from client.
#endif
        atomic_set(&g_InitializeState,true);
        reg = 0xFF;
        cnt = 0;
        while (1) {
            ml630q790_device_read(STATUS, &reg, sizeof(reg));
            if (reg == 0x00) {
                status_ok = true;
                break;
            }
            if (reg == 0xFD && cnt >= STATUS_READ_RETRY_NUM_2) {
                break;
            }
            if (cnt >= STATUS_READ_RETRY_NUM) {
                ML630Q790_ERR_LOG("STATUS read TimeOut[%x]",reg);
                return ML630Q790_RC_ERR_TIMEOUT;
            }

            ++cnt;
            usleep_range(10000, 10100);
        }

        if (!status_ok) {
            cnt_hw_reset++;
            if (cnt_hw_reset > RESET_RETRY_NUM) {
                ML630Q790_ERR_LOG("STATUS read retry count over[%d]",cnt_hw_reset);
                return ML630Q790_RC_ERR_TIMEOUT;
            }
            ML630Q790_ERR_LOG("STATUS read fail reg[%x] 10ms_wait_cnt[%d] hwreset_cnt[%d]",reg,cnt,cnt_hw_reset);
            continue;
        }

        break;
    }

    reg = 0x04;
    ml630q790_device_write(CFG, &reg, sizeof(reg));
    {
        uint8_t data = 0;
        ml630q790_device_read(CFG, &data, 1);
        if (data != reg) {
            ML630Q790_ERR_LOG("CFG unable to write [%x]",data);
            return ML630Q790_RC_ERR;
        }
        ML630Q790_D_LOG("CFG read [%x]",data);
    }

    reg = 0x00;
    ml630q790_device_write(INTMASK0, &reg, sizeof(reg));
    ml630q790_device_write(INTMASK1, &reg, sizeof(reg));

    ENABLE_IRQ;

    cmd.cmd.udata16 = HC_MCU_SET_PDIR;
    cmd.prm.ub_prm[0] = 0x01;
    cmd.prm.ub_prm[1] = 0x01;
    cmd.prm.ub_prm[2] = 0x01;
    cmd.prm.ub_prm[3] = 0x01;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((ML630Q790_RC_OK != ret) || (0 != res.err.udata16)) {
        ML630Q790_ERR_LOG("end HC_MCU_SET_PDIR err[%x]",res.err.udata16);
        return ML630Q790_RC_ERR;
    }

    cmd.cmd.udata16 = HC_MCU_SET_PCON;
    cmd.prm.ub_prm[0] = 0x11;
    cmd.prm.ub_prm[1] = 0x11;
    ret = sns_hostcmd(&cmd, &res, 0, EXE_HOST_NO_RES, READ_FIFO);
    if((ML630Q790_RC_OK != ret) || (0 != res.err.udata16)) {
        ML630Q790_ERR_LOG("end HC_MCU_SET_PCON err[%x]",res.err.udata16);
        return ML630Q790_RC_ERR;
    }

    ML630Q790_D_LOG("end");
    return ret;
}

static int32_t __init ml630q790_init(void)
{
    int32_t ret = 0;
    sns_wq_int = create_singlethread_workqueue("sns_wq_int");
    if(!sns_wq_int)
    {
        ML630Q790_ERR_LOG("can't create interrupt queue-sns_wq_int");
        return ML630Q790_RC_ERR;
    }

    sns_wq = create_singlethread_workqueue("sns_wq");
    if(!sns_wq)
    {
        ML630Q790_ERR_LOG("can't create interrupt queue-sns_wq");
        goto REGIST_ERR;
    }

    ret = spi_register_driver(&ml630q790_driver);
    if(ret != 0){
        ML630Q790_ERR_LOG("fail:spi_register_driver()->ret[%d]",ret);
        goto REGIST_ERR;
    }
    return ML630Q790_RC_OK;
REGIST_ERR:
    if(sns_wq_int != NULL){
        flush_workqueue(sns_wq_int);
        destroy_workqueue(sns_wq_int);
        sns_wq_int = NULL;
    }

    if(sns_wq != NULL){
        flush_workqueue(sns_wq);
        destroy_workqueue(sns_wq);
        sns_wq = NULL;
    }
    ml630q790_drv_set_status( ML630Q790_NORMAL );
    
	return 0;

}

static void __exit ml630q790_exit(void)
{
    ML630Q790_D_LOG("start");
    ML630Q790_D_LOG("end");
}

module_init(ml630q790_init);
module_exit(ml630q790_exit);
