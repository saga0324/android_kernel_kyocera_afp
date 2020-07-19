/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
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
#include "sensor_driver.h"
#include "sensor_micon_driver.h"

#define SSIO_MASK_WRITE              (0x7f)
#define SSIO_MASK_READ               (0x80)
#define SPI_RESUME_RETRY_NUM         300
#define SPI_RETRY_NUM                5
#define WAITEVENT_TIMEOUT            (3000)

struct spi_device *client_sns;
struct semaphore s_tSnsrSema;
struct mutex s_tDataMutex;
struct mutex s_tSpiMutex;
bool g_bDevIF_Error = false;
bool g_micon_error = false;
bool g_spi_error = false;
int32_t g_nIntIrqFlg;
struct micon_bdata g_micon_bdata;
wait_queue_head_t s_tWaitInt;
atomic_t g_bIsResume;
atomic_t g_ResetStatus;
atomic_t g_ShutdownStatus;
atomic_t g_bIsIntIrqEnable;
uint16_t g_lastCmdError;

static int32_t sns_spi_write_proc(uint8_t adr, const uint8_t *data, uint8_t size);
static int32_t sns_spi_read_proc(uint8_t adr, uint8_t *data, uint16_t size);
static int32_t sns_waitcmd(uint16_t intBit);
static void sns_timeout_dump(uint8_t *reg);

static int32_t sns_spi_write_proc(uint8_t adr, const uint8_t *data, uint8_t size)
{
    int32_t ret = 0;
    struct spi_message  message;
    struct spi_transfer transfer;
    uint8_t send_data[100];

    SENSOR_N_LOG("start");

    if((data == NULL) || (size == 0)){
        SENSOR_ERR_LOG("end return[SNS_RC_ERR]");
        return SNS_RC_ERR;
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

    ret = spi_sync(client_sns, &message);
    if(ret < 0){
        SENSOR_ERR_LOG("falut spi_sync()-->ret[%d]",ret);
    }

    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

static int32_t sns_spi_read_proc(uint8_t adr, uint8_t *data, uint16_t size)
{
    int32_t ret = 0;

    struct spi_message  message;
    struct spi_transfer transfer[2];

    SENSOR_N_LOG("start");

    if( (data == NULL) || (size == 0)){
        SENSOR_ERR_LOG("end return[SNS_RC_ERR]");
        return SNS_RC_ERR;
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

    ret = spi_sync(client_sns, &message);
    if(ret < 0){
        SENSOR_ERR_LOG("falut spi_sync()-->ret[%d]",ret);
    }


    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

int32_t sns_spi_ram_write_proc(uint8_t adr, const uint8_t *data, int32_t size)
{
    int32_t ret = 0;
    struct spi_message  message;
    struct spi_transfer transfer;
    uint8_t *send_data=  NULL;

    SENSOR_N_LOG("start");

    if((data == NULL) || (size == 0)){
        SENSOR_ERR_LOG("end return[SNS_RC_ERR]");
        return SNS_RC_ERR;
    }

    send_data = (uint8_t *)kmalloc( size + 1, GFP_KERNEL );
    if(send_data == NULL){
        SENSOR_ERR_LOG("end return[ENOMEM]");
        return -ENOMEM;
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

    ret = spi_sync(client_sns, &message);
    kfree(send_data);

    if(ret < 0){
        SENSOR_ERR_LOG("falut spi_sync()-->ret[%d]",ret);
    }

    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

int32_t sns_device_write(uint8_t adr, const uint8_t *data, uint8_t size)
{
    int32_t i;
    int32_t ret;

    SENSOR_N_LOG("start");

    if(unlikely(g_spi_error)){
        SENSOR_ERR_LOG("spi_setup NG");
        return -EINVAL;
    }

    mutex_lock(&s_tSpiMutex);

    SENSOR_D_LOG("adr[0x%02x] data[%p] size[0x%02x]", adr, data, size);
    SENSOR_D_DUMP("sns_device_write:", DUMP_PREFIX_OFFSET, data, size);

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
        SENSOR_ERR_LOG("end return[EBUSY]");
        mutex_unlock(&s_tSpiMutex);
        return -EBUSY;
    }

    for(i=0; i<SPI_RETRY_NUM; i++)
    {
        ret = sns_spi_write_proc(adr, data, size);
        if(ret == 0){
            mutex_unlock(&s_tSpiMutex);
            SENSOR_N_LOG("end - return[%d]",ret);
            return 0;
        }else if((ret == -EBUSY) || ((ret == -EIO) && (i < SPI_RETRY_NUM - 1))){
            SENSOR_ERR_LOG("falut sns_spi_write_proc()-->ret[%d] RETRY[%d]",ret ,i );
            msleep(100);
        }else{
            g_bDevIF_Error = true;
            SENSOR_ERR_LOG("SPI write Other error (H/W Reset ON)");
            break;
        }
    }

    mutex_unlock(&s_tSpiMutex);

    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

int32_t sns_device_read(uint8_t adr, uint8_t *data, uint16_t size)
{
    int32_t i;
    int32_t ret;

    SENSOR_N_LOG("start");

    if(unlikely(g_spi_error)){
        SENSOR_ERR_LOG("spi_setup NG");
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
        SENSOR_ERR_LOG("end return[EBUSY]");
        mutex_unlock(&s_tSpiMutex);
        return -EBUSY;
    }

    for(i=0; i<SPI_RETRY_NUM; i++)
    {
        ret = sns_spi_read_proc(adr, data, size);
        if(ret == 0){
            SENSOR_D_LOG("adr[0x%02x] data[%p] size[0x%04x]", adr, data, size);
            SENSOR_D_DUMP("sns_device_read:", DUMP_PREFIX_OFFSET, data, size);
            mutex_unlock(&s_tSpiMutex);
            SENSOR_N_LOG("end - return[%d]",ret);
            return 0;
        }else if((ret == -EBUSY) || ((ret == -EIO) && (i < SPI_RETRY_NUM - 1))){
            SENSOR_ERR_LOG("falut sns_spi_write_proc()-->ret[%d] RETRY[%d]",ret ,i );
            msleep(100);
        }else{
            g_bDevIF_Error = true;
            SENSOR_ERR_LOG("SPI write Other error (H/W Reset ON)");
            break;
        }
    }

    mutex_unlock(&s_tSpiMutex);

    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

int32_t sns_hostcmd(const HostCmd *prm, HostCmdRes *res, uint32_t res_size, uint8_t mode, uint8_t is_fwud)
{
    int32_t ret;
    uint8_t reg[19];
    int32_t i;

    SENSOR_N_LOG("start");

    if (g_micon_error) {
        SENSOR_ERR_LOG("end - skip due to micon error");
        return SNS_RC_ERR;
    }

    down(&s_tSnsrSema);

    SENSOR_D_LOG("req [0x%04x] %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x", prm->cmd.udata16,
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

    SENSOR_N_LOG("reg0[%x] reg1[%x] reg2[%x] reg3[%x]",
                  reg[0] , reg[1] , reg[2] , reg[3] );
    SENSOR_N_LOG("reg4[%x] reg5[%x] reg6[%x] reg7[%x]",
                  reg[4] , reg[5] , reg[6] , reg[7] );
    SENSOR_N_LOG("reg8[%x] reg9[%x] reg10[%x] reg11[%x]",
                  reg[8] , reg[9] , reg[10] , reg[11] );
    SENSOR_N_LOG("reg12[%x] reg13[%x] reg14[%x] reg15[%x]",
                  reg[12] , reg[13] , reg[14] , reg[15] );
    SENSOR_N_LOG("reg16[%x] reg17[%x] reg18[%x] mode[%x]",
                  reg[16] , reg[17] , reg[18] , mode );

    mutex_lock(&s_tDataMutex);
    g_nIntIrqFlg = 0;
    g_lastCmdError = 0;
    mutex_unlock(&s_tDataMutex);

    ret = sns_device_write(PRM0F, reg, sizeof(reg));
    if(ret == SNS_RC_OK){

        if((mode & EXE_HOST_WAIT) == EXE_HOST_WAIT){
            ret |= sns_waitcmd(INTREQ_HOST_CMD);
            if(ret != SNS_RC_OK) {
                if( ((mode & EXE_HOST_EX_NO_RECOVER) == EXE_HOST_EX_NO_RECOVER) && 
                    (ret == SNS_RC_ERR_TIMEOUT) ){
                    SENSOR_ERR_LOG("SPI HostCmd error");
                    up(&s_tSnsrSema);
                    return SNS_RC_ERR_TIMEOUT;
                }
                sns_timeout_dump(reg);

                up(&s_tSnsrSema);
                g_bDevIF_Error = true;
                return ret;
            }
        }

        if((mode & EXE_HOST_RES) == EXE_HOST_RES){
            if(is_fwud == READ_FIFO){
                ret |= sns_device_read(FIFO, res->res.ub_res, res_size);
                if(ret != SNS_RC_OK) {
                    SENSOR_ERR_LOG("SPI HostCmd error(FIFO)");
                }
            }
            else if(is_fwud == READ_RSLT){
                ret |= sns_device_read(RSLT00, res->res.ub_res, res_size);
                if(ret != SNS_RC_OK) {
                    SENSOR_ERR_LOG("SPI HostCmd error(RSLT00)");
                }
            }else{
                ret |= SNS_RC_ERR;
                if(ret != SNS_RC_OK) {
                    SENSOR_ERR_LOG("SPI HostCmd is_fwud error[%d]", is_fwud);
                }
            }
        }

        res->err.udata16 = g_lastCmdError;
    }else{
        SENSOR_ERR_LOG("SPI HostCmd error ret[%d]", ret);
        g_bDevIF_Error = true;
    }

    SENSOR_D_LOG("rsp [0x%04x] %s ret=%d,err=%d size=%u %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x...",
        prm->cmd.udata16,
        (ret == SNS_RC_OK && res->err.udata16 == 0) ? "OK" : "NG",
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

    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

static int32_t sns_waitcmd(uint16_t intBit)
{
    int32_t ret = SNS_RC_ERR_TIMEOUT;
    int32_t result = 0;
    long timeout;
    int32_t retry = 300;

    SENSOR_N_LOG("start");

    timeout = msecs_to_jiffies(WAITEVENT_TIMEOUT);

    while(retry){
    
        result = wait_event_interruptible_timeout(s_tWaitInt, (g_nIntIrqFlg & (INTREQ_HOST_CMD | INTREQ_ERROR)), timeout);
        mutex_lock(&s_tDataMutex);
        if( g_nIntIrqFlg & INTREQ_ERROR ){
            g_nIntIrqFlg &= ~INTREQ_ERROR;
            mutex_unlock(&s_tDataMutex);

            SENSOR_N_LOG("INTREQ0/1 -Error- ");
            ret = SNS_RC_ERR;
            break;

        }else if( g_nIntIrqFlg & INTREQ_HOST_CMD ){
            g_nIntIrqFlg &= ~INTREQ_HOST_CMD;
            mutex_unlock(&s_tDataMutex);

            ret = SNS_RC_OK;
            SENSOR_N_LOG("Wakeup Event... ");
            break;
        }
        mutex_unlock(&s_tDataMutex);

        if( result == -ERESTARTSYS ) {
            SENSOR_N_LOG("wait event signal received. retry = [%d] g_nIntIrqFlg = [%x}",
                         retry, g_nIntIrqFlg);
            msleep(10);
        }

        if( result == 0 ){
            ret = SNS_RC_ERR_TIMEOUT;
            SENSOR_ERR_LOG("wait event timeout... [%x]", g_nIntIrqFlg);
            break;
        }
        retry--;
    }

    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

static void sns_timeout_dump(uint8_t *reg)
{
    unsigned char data[0x1f];
    int i;

    SENSOR_N_LOG("start");

    SENSOR_ERR_LOG("##### SNS TimeOut Error Log #####");
    SENSOR_ERR_LOG("Gloval Value :");
    SENSOR_ERR_LOG("Send Cmd :[%x][%x][%x][%x]", reg[0] ,reg[1] ,reg[2] ,reg[3]);
    SENSOR_ERR_LOG("Send Cmd :[%x][%x][%x][%x]", reg[4] ,reg[5] ,reg[6] ,reg[7]);
    SENSOR_ERR_LOG("Send Cmd :[%x][%x][%x][%x]", reg[8] ,reg[9] ,reg[10] ,reg[11]);
    SENSOR_ERR_LOG("Send Cmd :[%x][%x][%x][%x]", reg[12] ,reg[13] ,reg[14] ,reg[15]);
    SENSOR_ERR_LOG("Send Cmd :[%x][%x][%x]", reg[16] ,reg[17] ,reg[18]);
    SENSOR_ERR_LOG("g_bIsIntIrqEnable[%04x]", (int)atomic_read(&g_bIsIntIrqEnable));
    SENSOR_ERR_LOG("g_nIntIrqFlg[%04x]", g_nIntIrqFlg);
    if (gpio_is_valid(g_micon_bdata.int_gpio)) {
        SENSOR_ERR_LOG("g_micon_bdata[%04x]", gpio_get_value(g_micon_bdata.int_gpio));
    }

    sns_device_read(0x00, data, sizeof(data));

    SENSOR_ERR_LOG("H/W Register Dump :");
    for(i=0; i<sizeof(data); i++){
        if((i % 16) == 0){
            SENSOR_ERR_LOG("[%02x]", i);
        }
        SENSOR_ERR_LOG("data[%02x]", data[i]);
    }
    SENSOR_N_LOG("end");

}

int32_t sns_spi_probe(void)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");

    if(!client_sns){
        SENSOR_ERR_LOG("client_sns is NULL");
        return -EINVAL;
    }

    ret = spi_setup(client_sns);
    if(ret < 0) {
        SENSOR_ERR_LOG("falut spi_setup()-->ret[%d]",ret);
    }

    SENSOR_D_LOG("end");
    return ret;
}
