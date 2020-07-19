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

#include <linux/module.h>
#include <linux/kernel.h>

#include "sensor_driver.h"
#include "sensor_com.h"

#define SNESOR_COM_FW_VER_CHK_OK    (0x00)
#define SNESOR_COM_FW_VER_CHK_NG    (0xFF)
#define SENSOR_NOT_MICON_TYPE ( (1<<SENSOR_LIGHT)\
                                |(1<<SENSOR_PROX) )

#define SENSOR_NOT_OFF_SUSPEND_TYPE ( (1<<SENSOR_PROX)\
                                      | (1<<SENSOR_EXT_PEDO)\
                                      | (1<<SENSOR_EXT_VEHI)\
                                      | (1<<SENSOR_EXT_BARO)\
                                      | (1<<SENSOR_EXT_IWIFI)\
                                      | (1<<SENSOR_SGNFCNT_MTN)\
                                      | (1<<SENSOR_STEP_CNT)\
                                      | (1<<SENSOR_STEP_DTC)\
                                      | (1<<SENSOR_KC_MOTION_WALK_START)\
                                      | (1<<SENSOR_KC_MOTION_WALK_STOP)\
                                      | (1<<SENSOR_KC_MOTION_TRAIN)\
                                      | (1<<SENSOR_KC_MOTION_VEHICLE)\
                                      | (1<<SENSOR_KC_MOTION_BRINGUP) )

#if 0
#define SENSOR_NOT_SUPPORT_TYPE ( (1<<SENSOR_RH)\
                                  | (1<<SENSOR_TEMP)\
                                  | (1<<SENSOR_TEMP_AMBIENT)  )
#else
#define SENSOR_NOT_SUPPORT_TYPE (0)
#endif
#define MICON_FIFO_SIZE 3072

static ssize_t sensor_com_info_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t sensor_com_fw_update_sq_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count
);
static ssize_t sensor_com_fw_update_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t sensor_com_test0_ctl_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t sensor_com_fw_version_chk_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t sensor_com_fw_version_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static void sensor_com_set_input_params( struct input_dev *dev );
static uint32_t sensor_get_status_local(enum sensor_e_type type);
static void sensor_drv_set_status_local( enum sensor_drv_status_e_type next_status );
static enum sensor_drv_status_e_type sensor_drv_get_status_local(void);
static int32_t sensor_type_enable(enum sensor_e_type type,bool enable);
static int32_t sensor_application_enable(enum sensor_e_type type,bool enable);
static void sensor_suspend_ctl( void );
static void sensor_resume_ctl( void );
static void sensor_shutdown_ctl( void );
static void sensor_reset_ctl( void );
static enum sensor_chk_status_e_type sensor_check_status(
    enum sensor_e_type type,
    bool enable );

static DEVICE_ATTR(fw_update_sq,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    NULL,
    sensor_com_fw_update_sq_store
);
static DEVICE_ATTR(test0_ctl,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    NULL,
    sensor_com_test0_ctl_store
);
static DEVICE_ATTR(fw_version_chk,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    sensor_com_fw_version_chk_show,
    NULL
);
static DEVICE_ATTR(fw_version,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    sensor_com_fw_version_show,
    NULL
);
static DEVICE_ATTR(fw_update,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    NULL,
    sensor_com_fw_update_store
);
static DEVICE_ATTR(info,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    sensor_com_info_show,
    NULL
);

static struct attribute *sensor_com_attributes[] = {
    &dev_attr_fw_update_sq.attr,
    &dev_attr_test0_ctl.attr,
    &dev_attr_fw_version_chk.attr,
    &dev_attr_fw_version.attr,
    &dev_attr_fw_update.attr,
    &dev_attr_info.attr,
    NULL
};

static struct attribute_group sensor_com_attr_grp = {
    .attrs = sensor_com_attributes
};


struct sensor_input_info_str sensor_com_input_info =
{
    NULL,
    sensor_com_set_input_params,
    &sensor_com_attr_grp,
};
static struct sensor_poll_ctl_str sensor_poll_ctl_tbl[] = {
    { SENSOR_ACC, NULL },
    { SENSOR_ACC_LNR, NULL },
    { SENSOR_GAME_ROT_VCTR, NULL },
    { SENSOR_GRV, NULL },
    { SENSOR_GYRO, NULL },
    { SENSOR_GYRO_UNCAL, NULL },
    { SENSOR_MAG, NULL },
    { SENSOR_MAG_ROT_VCTR, NULL },
    { SENSOR_MAG_UNCAL, NULL },
    { SENSOR_PRESSURE, NULL },
    { SENSOR_ORTN, NULL },
    { SENSOR_ROT_VCTR, NULL },
    { SENSOR_STEP_CNT, NULL },
    { SENSOR_COM, NULL },
};
static uint32_t sensor_poll_ctl_tbl_size = sizeof(sensor_poll_ctl_tbl)
                                           / sizeof(struct sensor_poll_ctl_str);
static struct mutex sensor_com_mutex;
struct mutex sensor_batch_mutex;
static atomic_t sensor_type_status;
static enum sensor_drv_status_e_type sensor_drv_status = 0;

static ssize_t sensor_com_info_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int ret;

    ret = scnprintf(buf, PAGE_SIZE,
                    "type[0x%08X] batch[0x%08X] drv[0x%08X]\n",
                    (int)atomic_read(&sensor_type_status),
                    (int)sensor_get_batch_status(),
                    (int)sensor_drv_status);

    return ret;
}

static ssize_t sensor_com_fw_update_sq_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count
)
{
    uint32_t data[2];
    uint8_t *data_addr = NULL;
    uint32_t len;
    uint8_t *fw_data = NULL;
    uint32_t ret;

    SENSOR_N_LOG("start");

    if( SENSOR_RESUME != sensor_drv_get_status_local() ){
        SENSOR_ERR_LOG("bad status-->sensor_drv_status[%d]",
                       (int)sensor_drv_status );
        return count;
    }

    sscanf(buf, "%x %d", &data[0],&data[1]);
    data_addr = (uint8_t*)data[0];
    len = data[1];

    if((data_addr == NULL) || (len == 0)){
        SENSOR_ERR_LOG("bad param-->data_addr[%X] len[%d]",
                       (int)data_addr,
                       (int)len);
        return count;
    }
    if((len % 4) != 0){
        SENSOR_ERR_LOG("bad param-->len[%d]:is not a multiple of 4",
                       (int)len);
        return count;
    }

    fw_data = (uint8_t *)kmalloc( len, GFP_KERNEL );

    if(fw_data == NULL){
        SENSOR_ERR_LOG("fail kmalloc--> NULL");
        return -ENOMEM;
    }

    ret = copy_from_user( fw_data, data_addr, len );
    if( ret != 0 )
    {
        SENSOR_ERR_LOG("fail copy_from_user-->ret[%d]",
                       (int)ret);
        kfree( fw_data );
        return count;
    }

    mutex_lock(&sensor_com_mutex);
    sensor_drv_set_status_local( SENSOR_FW_UPDATE );
    ret = sns_update_fw_seq(fw_data, len);
    sensor_drv_set_status_local( SENSOR_RESUME );
    mutex_unlock(&sensor_com_mutex);
    kfree( fw_data );

    if(ret != 0){
        SENSOR_ERR_LOG("fail sns_update_fw_seq-->ret[%d]",
                       (int)ret);
        return count;
    }

    SENSOR_N_LOG("end - return[%d]",count);
    return count;
}

static ssize_t sensor_com_fw_update_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    uint8_t *fw_data = NULL;
    uint8_t *data_addr = NULL;
    uint32_t len;
    uint32_t data[2];
    uint32_t ret;

    SENSOR_N_LOG("start");

    if( SENSOR_RESUME != sensor_drv_get_status_local() ){
        SENSOR_ERR_LOG("bad status-->sensor_drv_status[%d]",
                       (int)sensor_drv_status );
        return count;
    }

    sscanf(buf, "%x %d", &data[0],&data[1]);
    data_addr = (uint8_t*)data[0];
    len = data[1];

    if((data_addr == NULL) || (len == 0)){
        SENSOR_ERR_LOG("bad param-->data_addr[%X] len[%d]",
                       (int)data_addr,
                       (int)len);
        return count;
    }
    if((len % 4) != 0){
        SENSOR_ERR_LOG("bad param-->len[%d]:is not a multiple of 4",
                       (int)len);
        return count;
    }

    fw_data = (uint8_t *)kmalloc( len, GFP_KERNEL );

    if(fw_data == NULL){
        SENSOR_ERR_LOG("fail kmalloc--> NULL");
        return -ENOMEM;
    }

    ret = copy_from_user( fw_data, data_addr, len );
    if( ret != 0 )
    {
        SENSOR_ERR_LOG("fail copy_from_user-->ret[%d]",
                       (int)ret);
        kfree( fw_data );
        return count;
    }

    mutex_lock(&sensor_com_mutex);
    sensor_drv_set_status_local( SENSOR_FW_UPDATE );
    ret = sns_update_fw(fw_data, len);
    sensor_drv_set_status_local( SENSOR_RESUME );
    mutex_unlock(&sensor_com_mutex);
    kfree( fw_data );

    if(ret != 0){
        SENSOR_ERR_LOG("fail sns_update_fw_seq-->ret[%d]",
                       (int)ret);
        return count;
    }

    SENSOR_N_LOG("end - return[%d]",count);
    return count;
}

static ssize_t sensor_com_test0_ctl_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    unsigned long direction;
    int32_t ret;

    SENSOR_N_LOG("start");

    ret = strict_strtoul(buf, 10, &direction);
    if (ret < 0){
        SENSOR_ERR_LOG("strict_strtoul()-->ret[%d]",(int)ret);
        return count;
    }

    if(direction == 1){
        SENSOR_A_LOG("BRMP port --> set input");
        sns_BRMP_direction();
    }

    SENSOR_N_LOG("end - return[%d]",count);
    return count;
}

static ssize_t sensor_com_fw_version_chk_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    uint8_t  fw_ver[4];
    uint32_t ret;
    uint32_t fw_version;
    uint32_t fw_chk = SNESOR_COM_FW_VER_CHK_NG;

    SENSOR_N_LOG("start");
    mutex_lock(&sensor_com_mutex);

    ret = sns_get_fw_version(fw_ver);
    fw_version = SNESOR_COM_GET_FW_VER(fw_ver);
    SENSOR_N_LOG("fw_version[%x]",(int)fw_version);
    if(ret != SNS_RC_OK){
        fw_version = SNESOR_COM_FW_VER_NONE;
        SENSOR_ERR_LOG("fail get fw ver-->set ver[%x]",(int)fw_version);
    }

    if(fw_version == SNESOR_COM_FW_VER_DATA){
        fw_chk = SNESOR_COM_FW_VER_CHK_OK;
        SENSOR_A_LOG("fw ver chk OK: fw_chk[%x]",(int)fw_chk);
    } else {
        fw_chk = SNESOR_COM_FW_VER_CHK_NG;
        SENSOR_A_LOG("fw ver chk NG: fw_chk[%x]",(int)fw_chk);
    }

    if( ret != 0 ) {
        sns_err_check();
    }

    mutex_unlock(&sensor_com_mutex);
    SENSOR_N_LOG("end - return[%d]",(int)fw_chk);
    return sprintf(buf, "%x\n", fw_chk);
}

static ssize_t sensor_com_fw_version_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    uint8_t  fw_ver[4];
    uint32_t ret;
    uint32_t fw_version;

    SENSOR_N_LOG("start");
    mutex_lock(&sensor_com_mutex);

    ret = sns_get_fw_version(fw_ver);
    fw_version = SNESOR_COM_GET_FW_VER(fw_ver);
    SENSOR_N_LOG("fw_version[%x]",(int)fw_version);
    if(ret != SNS_RC_OK){
        fw_version = SNESOR_COM_FW_VER_NONE;
        SENSOR_ERR_LOG("fail get fw ver-->set ver[%x]",(int)fw_version);
    }

    if( ret != 0 ) {
        sns_err_check();
    }

    mutex_unlock(&sensor_com_mutex);
    SENSOR_N_LOG("end - return[%d]",(int)fw_version);
    return sprintf(buf, "%x\n", fw_version);
}

static enum sensor_chk_status_e_type sensor_check_status(
    enum sensor_e_type type,
    bool enable )
{
    enum sensor_chk_status_e_type ret = SENSOR_CHK_NORMAL;

    if((SENSOR_ON<<type) & (SENSOR_NOT_SUPPORT_TYPE)){
        SENSOR_ERR_LOG("not supported!! type[%d]",(int)type);
        return SENSOR_CHK_SKIP_ALL;
    }

    if( true == enable ){
        if( SENSOR_ON == sensor_get_status_local(type) &&
            SENSOR_SGNFCNT_MTN != type) {
            ret = SENSOR_CHK_SKIP_ALL;
        } else if ( (SENSOR_ON<<type) & (~SENSOR_NOT_MICON_TYPE) ){
            ret = SENSOR_CHK_MICON_CTL;
        }
    } else {
        if( SENSOR_OFF == sensor_get_status_local(type) ){
            ret = SENSOR_CHK_SKIP_ALL;
        } else if ( (SENSOR_ON<<type) & (~SENSOR_NOT_MICON_TYPE) ){
            ret = SENSOR_CHK_MICON_CTL;
        }
    }

    return ret;
}

static void sensor_com_set_input_params( struct input_dev *dev )
{
    SENSOR_N_LOG("start");

    if(!dev){
        SENSOR_ERR_LOG("bad parm --> dev[%d]",(int)dev);
        return;
    }

    dev->name = "sensor_com";
    dev->id.bustype = BUS_SPI;

    SENSOR_N_LOG("end");
    return;
}

static int32_t sensor_type_enable(enum sensor_e_type type,bool enable)
{
    int32_t ret = 0;

    SENSOR_N_LOG("type [%d] enable[%d]",(int)type,(int)enable);

    switch (type) {
    case SENSOR_ACC:
        break;
    case SENSOR_GYRO:
        break;
    case SENSOR_MAG:
        break;
    case SENSOR_ACC_LNR:
        break;
    case SENSOR_GRV:
        break;
    case SENSOR_GYRO_UNCAL:
        break;
    case SENSOR_MAG_UNCAL:
        break;
    case SENSOR_ORTN:
        break;
    case SENSOR_ROT_VCTR:
        break;
    case SENSOR_GAME_ROT_VCTR:
        break;
    case SENSOR_MAG_ROT_VCTR:
        break;
    case SENSOR_LIGHT:
#ifdef CONFIG_INPUT_SENSOR_GP2AP030
        gp2ap_als_sensor_activate( enable );
#endif
        break;
    case SENSOR_PROX:
#ifdef CONFIG_INPUT_SENSOR_GP2AP030
        gp2ap_ps_sensor_activate( enable );
#endif
        break;
    case SENSOR_SGNFCNT_MTN:
        break;
    case SENSOR_STEP_CNT:
        break;
    case SENSOR_STEP_DTC:
        break;
    case SENSOR_EXT_PEDO:
        break;
    case SENSOR_EXT_VEHI:
        break;
    case SENSOR_EXT_BARO:
        break;
    case SENSOR_EXT_IWIFI:
        break;
    case SENSOR_EXT_VH:
        break;
    case SENSOR_KC_MOTION_WALK_START:
        break;
    case SENSOR_KC_MOTION_WALK_STOP:
        break;
    case SENSOR_KC_MOTION_TRAIN:
        break;
    case SENSOR_KC_MOTION_VEHICLE:
        break;
    case SENSOR_KC_MOTION_BRINGUP:
        break;
#if 0
    case SENSOR_RH:
    case SENSOR_TEMP:
    case SENSOR_TEMP_AMBIENT:
        break;
#endif
    case SENSOR_PRESSURE:
        break;
    case SENSOR_UNDERWATER_DETECT:
        break;
    default:
        break;
    }

    SENSOR_N_LOG("ret [%d]",(int)ret );

    return ret;
}
static int32_t sensor_application_enable(enum sensor_e_type type,bool enable)
{
    int32_t ret = 0;

    SENSOR_N_LOG("type [%d] enable[%d]",(int)type,(int)enable);

    switch (type) {
    case SENSOR_SGNFCNT_MTN:
        ret = sns_motion_start( enable );
        break;
    case SENSOR_EXT_PEDO:
        ret = sns_dailys_start( enable );
        break;
    case SENSOR_EXT_VEHI:
        ret = sns_dailys_start( enable );
        break;
    case SENSOR_EXT_BARO:
        ret = sns_baro_start( enable );
        break;
    case SENSOR_EXT_IWIFI:
        ret = sns_iwifi_start( enable );
        break;
    case SENSOR_EXT_VH:
        ret = sns_vhdetect_start( enable );
        break;
    case SENSOR_KC_MOTION_WALK_START:
        ret = sns_kc_motion_walk_start_start( enable );
        break;
    case SENSOR_KC_MOTION_WALK_STOP:
        ret = sns_kc_motion_walk_stop_start( enable );
        break;
    case SENSOR_KC_MOTION_TRAIN:
        ret = sns_kc_motion_train_start( enable );
        break;
    case SENSOR_KC_MOTION_VEHICLE:
        ret = sns_kc_motion_vehicle_start( enable );
        break;
    case SENSOR_KC_MOTION_BRINGUP:
        ret = sns_kc_motion_bringup_start( enable );
        break;
    case SENSOR_UNDERWATER_DETECT:
        ret = sns_uwater_start( enable );
        break;
    default:
        break;
    }

    SENSOR_N_LOG("ret [%d]",(int)ret );

    return ret;
}

uint32_t sensor_set_batch(
    enum sensor_e_type type,
    struct sensor_batch_info_str info )
{
    int32_t ret;

    mutex_lock(&sensor_com_mutex);

    ret = sns_set_batch(type, &info);
    if (ret != 0) {
        sns_err_check();
    }

    mutex_unlock(&sensor_com_mutex);

    return (uint32_t)ret;
}

uint32_t sensor_set_flush( enum sensor_e_type type , struct input_dev* dev)
{
    uint32_t ret = 0;

    SENSOR_N_LOG("start");

    if( SENSOR_RESUME != sensor_drv_get_status_local() ){
        SENSOR_ERR_LOG("bad status-->sensor_drv_status[%d]",
                       (int)sensor_drv_status );
        return -1;
    }

    mutex_lock(&sensor_com_mutex);

    if ((SENSOR_ON<<type) & (~SENSOR_NOT_MICON_TYPE)){
        SENSOR_N_LOG("test flush");
        sns_set_flush(type, dev);
    } else {
        SENSOR_N_LOG("test flush abs");
        input_report_abs(dev, ABS_MISC, -1);
        input_report_abs(dev, ABS_MISC, SENSOR_COMP_FLUSH_BF);
        input_sync(dev);
    }
    mutex_unlock(&sensor_com_mutex);
    SENSOR_N_LOG("end - return[%d]",ret);
    return ret;
}

void sensor_flush_event_buffer(void)
{
    int32_t ret = 0;

    SENSOR_N_LOG("start");
    mutex_lock(&sensor_com_mutex);

    ret = sns_flush_event_buffer();
    if(ret < 0){
        SENSOR_ERR_LOG("fail sns_flush_event_buffer ret[%d]",ret);
    }
    mutex_unlock(&sensor_com_mutex);
    SENSOR_N_LOG("end");

    return;
}

void sensor_com_mutex_lock(void)
{
    mutex_lock(&sensor_com_mutex);
}

void sensor_com_mutex_unlock(void)
{
    mutex_unlock(&sensor_com_mutex);
}

uint32_t sensor_get_status(enum sensor_e_type type)
{
    int32_t status = 0;
    SENSOR_N_LOG("start type=>%d", type);

    status = sensor_get_status_local(type);

    SENSOR_N_LOG("end");
    return status;
}

static uint32_t sensor_get_status_local(enum sensor_e_type type)
{
    int32_t status = 0;
    SENSOR_N_LOG("start");
    status = (uint32_t)((atomic_read(&sensor_type_status) & (SENSOR_ON << type)) >> type );
    SENSOR_N_LOG("end");
    return status;
}

enum sensor_drv_status_e_type sensor_drv_get_status( void )
{
   enum sensor_drv_status_e_type ret;

    SENSOR_N_LOG("start");
    mutex_lock(&sensor_com_mutex);

    ret = sensor_drv_get_status_local();

    mutex_unlock(&sensor_com_mutex);
    SENSOR_N_LOG("end");
    return ret;
}

static enum sensor_drv_status_e_type sensor_drv_get_status_local(void)
{
    enum sensor_drv_status_e_type ret;

    SENSOR_N_LOG("start");
    ret = sensor_drv_status;
    SENSOR_N_LOG("end");

    return ret;
}

void sensor_drv_set_status( enum sensor_drv_status_e_type next_status )
{
    SENSOR_N_LOG("start");
    mutex_lock(&sensor_com_mutex);

    sensor_drv_set_status_local(next_status);

    mutex_unlock(&sensor_com_mutex);
    SENSOR_N_LOG("end");
    return;
}

static void sensor_drv_set_status_local( enum sensor_drv_status_e_type next_status )
{
    SENSOR_N_LOG("start");
    sensor_drv_status = next_status;
    SENSOR_N_LOG("end");
    return;
}

void sensor_set_status(
    enum sensor_e_type type,
    enum sensor_type_status_e_type on
)
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("parm: type[%d] on[%d] - status[%X]",
                 (int)type,(int)on,(int)atomic_read(&sensor_type_status));

    if( on == SENSOR_ON) {
        atomic_set(&sensor_type_status, atomic_read(&sensor_type_status) | ( SENSOR_ON << type) );
    } else {
        atomic_set(&sensor_type_status, atomic_read(&sensor_type_status) & (~( SENSOR_ON << type)) );
    }

    SENSOR_N_LOG("status to [%X]",(int)atomic_read(&sensor_type_status));
    SENSOR_N_LOG("end");
    return;
}

uint32_t sensor_get_batch_status(void)
{
    uint32_t ret;

    SENSOR_N_LOG("start");
    ret = sns_get_logging_enable();
    SENSOR_N_LOG("end");

    return ret;
}

void sensor_poll_init(
    enum sensor_e_type type,
    struct sensor_poll_info_str* poll_info )
{
    SENSOR_N_LOG("start");

    if(!poll_info){
        return;
    }

    poll_info->poll_wq = create_singlethread_workqueue((poll_info->name));
    INIT_DELAYED_WORK(&(poll_info->poll_work), (poll_info->poll_func));

    sns_poll_init(type, poll_info);

    SENSOR_N_LOG("end");

    return;
}

int32_t sensor_type_get_data(
    enum sensor_e_type type,
    union sensor_read_data_u* read_data )
{
    int32_t ret = 0;

    mutex_lock(&sensor_com_mutex);

    if( SENSOR_RESUME != sensor_drv_get_status_local() ){
        SENSOR_ERR_LOG("bad status-->sensor_drv_status[%d]",
                       (int)sensor_drv_status );
        mutex_unlock(&sensor_com_mutex);
        return ret;
    }

    switch (type) {
    case SENSOR_ACC:
        ret = sns_acc_read_data( &(read_data->acc_data) );
        break;
    case SENSOR_GYRO:
        ret = sns_gyro_read_data( &(read_data->gyro_data) );
        break;
    case SENSOR_MAG:
        ret = sns_mag_read_data( &(read_data->mag_data) );
        break;
    case SENSOR_ACC_LNR:
        ret = sns_linacc_read_data( &(read_data->acc_lnr_data) );
        break;
    case SENSOR_GRV:
        ret = sns_gravity_read_data( &(read_data->grv_data) );
        break;
    case SENSOR_GYRO_UNCAL:
        ret = sns_gyrouncalib_read_data( &(read_data->gyro_uncal_data) );
        break;
    case SENSOR_MAG_UNCAL:
        ret = sns_maguncalib_read_data( &(read_data->mag_uncal_data) );
        break;
    case SENSOR_ORTN:
        ret = sns_ori_read_data( &(read_data->ortn_data) );
        break;
    case SENSOR_ROT_VCTR:
        ret = sns_rota_read_data( &(read_data->rot_vctr_data) );
        break;
    case SENSOR_GAME_ROT_VCTR:
        ret = sns_gamerota_read_data( &(read_data->game_rot_vctr_data) );
        break;
    case SENSOR_MAG_ROT_VCTR:
        ret = sns_magrota_read_data( &(read_data->mag_rot_vctr_data) );
        break;
    case SENSOR_LIGHT:
        break;
    case SENSOR_PROX:
        break;
    case SENSOR_SGNFCNT_MTN:
        break;
    case SENSOR_STEP_CNT:
        break;
    case SENSOR_STEP_DTC:
        break;
#if 0
    case SENSOR_RH:
    case SENSOR_TEMP:
    case SENSOR_TEMP_AMBIENT:
        break;
#endif
    case SENSOR_PRESSURE:
        ret = sns_pressure_read_data( &(read_data->pressure_data) );
        break;
    default:
        break;
    }

    if( ret != 0 ) {
        sns_err_check();
    }
    mutex_unlock(&sensor_com_mutex);

    return ret;
}

int32_t sensor_ext_get_data(
    enum sensor_e_type type,
    union sensor_ext_read_data_u* read_data )
{
    int32_t ret = 0;

    SENSOR_N_LOG("start");
    mutex_lock(&sensor_com_mutex);

    if( SENSOR_RESUME != sensor_drv_get_status_local() ){
        SENSOR_ERR_LOG("bad status-->sensor_drv_status[%d]",
                       (int)sensor_drv_status );
        mutex_unlock(&sensor_com_mutex);
        return ret;
    }

    switch (type) {
    case SENSOR_EXT_PEDO:
        ret = sns_get_pedo_data(&(read_data->pedm_data));
        break;
    case SENSOR_EXT_VEHI:
        ret = sns_get_vehi_data(&(read_data->vehi_data));
        break;
    case SENSOR_EXT_IWIFI:
        ret = sns_get_iwifi_data(&(read_data->iwifi_data));
        break;
    case SENSOR_EXT_VH:
        ret = sns_get_vhdetect_data(&(read_data->vh_data));
        break;
    default:
        break;
    }

    if( ret != 0 ) {
        sns_err_check();
    }
    mutex_unlock(&sensor_com_mutex);
    SENSOR_N_LOG("end");

    return ret;
}

int32_t sensor_ext_set_param(
    enum sensor_e_type type,
    DailysSetIWifiParam* iwifi_param,
    struct sensor_ext_pedom_param_str* pedom_param )
{
    int32_t ret = 0;

    SENSOR_N_LOG("start");
    mutex_lock(&sensor_com_mutex);

    if( SENSOR_RESUME != sensor_drv_get_status_local() ){
        SENSOR_ERR_LOG("bad status-->sensor_drv_status[%d]",
                       (int)sensor_drv_status );
        mutex_unlock(&sensor_com_mutex);
        return ret;
    }

    switch (type) {
    case SENSOR_EXT_PEDO:
        if(pedom_param){
            sns_set_pedo_param( pedom_param->weight,
                                pedom_param->step_wide,
                                pedom_param->vehi_type );
        }
        break;
    case SENSOR_EXT_IWIFI:
        if(iwifi_param){
            ret = sns_iwifi_set_info( 0, iwifi_param );
        }
        break;
    default:
        break;
    }

    if( ret != 0 ) {
        sns_err_check();
    }
    mutex_unlock(&sensor_com_mutex);
    SENSOR_N_LOG("end");

    return ret;
}

int32_t sensor_ext_clear( uint32_t clear_req )
{
    int32_t ret = 0;

    SENSOR_N_LOG("start");
    mutex_lock(&sensor_com_mutex);

    if( SENSOR_RESUME != sensor_drv_get_status_local() ){
        SENSOR_ERR_LOG("bad status-->sensor_drv_status[%d]",
                       (int)sensor_drv_status );
        mutex_unlock(&sensor_com_mutex);
        return ret;
    }

    ret = sns_pedom_clear( clear_req );
    if(ret < 0){
        SENSOR_ERR_LOG("fail sns_pedom_clear ret[%d]",ret);
    }

    if( ret != 0 ) {
        sns_err_check();
    }
    mutex_unlock(&sensor_com_mutex);
    SENSOR_N_LOG("end");

    return ret;
}

int32_t sensor_uwater_get_data(
    uwater_detect_info_t* read_data )
{
    int32_t ret = 0;

    SENSOR_N_LOG("start");
    mutex_lock(&sensor_com_mutex);

    if( SENSOR_RESUME != sensor_drv_get_status_local() ){
        SENSOR_ERR_LOG("bad status-->sensor_drv_status[%d]",
                       (int)sensor_drv_status );
        mutex_unlock(&sensor_com_mutex);
        return ret;
    }

    ret = sns_get_uwater_data(read_data);

    if( ret != 0 ) {
        sns_err_check();
    }
    mutex_unlock(&sensor_com_mutex);
    SENSOR_N_LOG("end");

    return ret;
}

int32_t sensor_uwater_clear( void )
{
    int32_t ret = 0;

    SENSOR_N_LOG("start");
    mutex_lock(&sensor_com_mutex);

    if( SENSOR_RESUME != sensor_drv_get_status_local() ){
        SENSOR_ERR_LOG("bad status-->sensor_drv_status[%d]",
                       (int)sensor_drv_status );
        mutex_unlock(&sensor_com_mutex);
        return ret;
    }

    ret = sns_uwater_clear();

    if( ret != 0 ) {
        sns_err_check();
    }
    mutex_unlock(&sensor_com_mutex);
    SENSOR_N_LOG("end");

    return ret;
}

void sensor_report_abs (
    struct input_dev* dev,
    int32_t x, int32_t y, int32_t z,
    int32_t last_x, int32_t last_y, int32_t last_z,
    int* rudder_cnt
)
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("report [x,y,z]:[%d,%d,%d]",(int)x,(int)y,(int)z );

    sensor_report_abs_current_time(dev);
    input_report_abs(dev, ABS_X, x);
    input_report_abs(dev, ABS_Y, y);
    input_report_abs(dev, ABS_Z, z);
    if( (last_x == x) && (last_y == y) && (last_z == z) ) {
        SENSOR_A_LOG("report grv - ABS_RUDDER: cnt[%d]",(int)(*rudder_cnt) );
        input_report_abs( dev,ABS_RUDDER,((*rudder_cnt)++) );
    }

    input_sync(dev);

    SENSOR_N_LOG("end");
    return;
}

void sensor_report_ac_abs (
    struct input_dev* dev,
    int32_t x, int32_t y, int32_t z,
    int32_t last_x, int32_t last_y, int32_t last_z,
    uint8_t accuracy,
    int* rudder_cnt
)
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("report [x,y,z]:[%d,%d,%d]",(int)x,(int)y,(int)z );

    sensor_report_abs_current_time(dev);
    input_report_abs(dev, ABS_X, x);
    input_report_abs(dev, ABS_Y, y);
    input_report_abs(dev, ABS_Z, z);
    input_report_abs(dev, ABS_STATUS, accuracy);
    if( (last_x == x) && (last_y == y) && (last_z == z) ) {
        SENSOR_A_LOG("report grv - ABS_RUDDER: cnt[%d]",(int)(*rudder_cnt) );
        input_report_abs( dev,ABS_RUDDER,((*rudder_cnt)++) );
    }

    input_sync(dev);

    SENSOR_N_LOG("end");
    return;
}

void sensor_report_abs_current_time(struct input_dev *dev)
{
    s64 time_ns = ktime_to_ns(ktime_get_boottime());
    sensor_report_abs_time(dev, ~time_ns);
    sensor_report_abs_time(dev, time_ns);
}

void sensor_report_abs_time(struct input_dev *dev, s64 time_ns)
{
    input_report_abs(dev, ABS_MISC+1, (time_ns&0xFFFFFFFF));
    input_report_abs(dev, ABS_MISC+2, ((time_ns>>32)&0xFFFFFFFF));
}

void sensor_enable(
    enum sensor_e_type type,
    struct sensor_poll_info_str* poll_info,
    bool enable
)
{
    int32_t ret = 0;
    int32_t ret_apl = 0;
    enum sensor_chk_status_e_type chk_state = SENSOR_CHK_NORMAL;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_com_mutex);
    SENSOR_N_LOG("parm: type[%d], poll_info[%d] enable[%d]",
                 (int)type, (int)poll_info, (int)enable);

    if (SENSOR_RESUME != sensor_drv_get_status_local()) {
        SENSOR_ERR_LOG("bad status-->sensor_drv_status[%d]",
                       (int)sensor_drv_status );
        mutex_unlock(&sensor_com_mutex);
        return;
    }

    chk_state = sensor_check_status( type, enable );
    if (SENSOR_CHK_SKIP_ALL == chk_state) {
        SENSOR_A_LOG("same status-->type[%d] sensor_type_status[0x%x]",
                      (int)type,
                      (int)atomic_read(&sensor_type_status));
        mutex_unlock(&sensor_com_mutex);
        return;
    }

    if (true == enable) {
        SENSOR_A_LOG("set ON");
        if (SENSOR_CHK_MICON_CTL == chk_state) {
            ret = sns_set_enable(type, enable);
            SENSOR_N_LOG("sen_set_enable() ret[%d]",ret);
            ret_apl = sensor_application_enable( type, enable );
            SENSOR_N_LOG("sensor_application_enable() ret_apl[%d]",ret_apl);
        } else {
            ret = sensor_type_enable(type, enable);
            SENSOR_N_LOG("sensor_type_enable() ret[%d]",ret);
        }
        sensor_set_status( type, SENSOR_ON);
    } else {
        SENSOR_A_LOG("set OFF");
        if (SENSOR_CHK_MICON_CTL == chk_state) {
            ret_apl = sensor_application_enable( type, enable );
            SENSOR_N_LOG("sensor_application_enable() ret_apl[%d]",ret_apl);
            ret = sns_set_enable(type, enable);
            SENSOR_N_LOG("sen_set_enable() ret[%d]",ret);
        } else {
            ret = sensor_type_enable(type, enable);
            SENSOR_N_LOG("sensor_type_enable() ret[%d]",ret);
        }
        sensor_set_status( type, SENSOR_OFF );
    }

    if ((ret != 0) || (ret_apl != 0)) {
        sns_err_check();
    }

    mutex_unlock(&sensor_com_mutex);

    SENSOR_N_LOG("end");
    return;
}

#ifdef CONFIG_INPUT_SENSOR_PROXIMITY
int sensor_set_ps_threshold( void )
{
    int ret = 0;
    SENSOR_N_LOG("start");
#ifdef CONFIG_INPUT_SENSOR_GP2AP030
    ret = gp2ap_set_ioctl_ps_threshold();
#endif
    SENSOR_N_LOG("end");
    return ret;
}
#endif /* CONFIG_INPUT_SENSOR_PROXIMITY */

void sensor_interrupt( enum sensor_e_type type, uint32_t data)
{
    SENSOR_N_LOG("start");

    switch (type) {
    case SENSOR_EXT_IWIFI:
#ifdef CONFIG_INPUT_SENSOR_EXT
        sensor_ext_iwifi_interrupt();
#endif
        break;
    case SENSOR_EXT_VH:
#ifdef CONFIG_INPUT_SENSOR_EXT
        sensor_ext_vh_interrupt();
#endif
        break;
    case SENSOR_EXT_PEDO:
#ifdef CONFIG_INPUT_SENSOR_EXT
        sensor_ext_pedo_interrupt();
#endif
        break;
    case SENSOR_EXT_VEHI:
#ifdef CONFIG_INPUT_SENSOR_EXT
        sensor_ext_vehi_interrupt();
#endif
        break;
    case SENSOR_PROX:
#ifdef CONFIG_INPUT_SENSOR_PROXIMITY
        prox_interrupt(data);
#endif
        break;
    case SENSOR_STEP_DTC:
#ifdef CONFIG_INPUT_SENSOR_STEP_DETECTOR
        step_dtc_interrupt();
#endif
        break;
    case SENSOR_SGNFCNT_MTN:
#ifdef CONFIG_INPUT_SENSOR_SIGNIFICANT_MOTION
        sgnfcnt_mtn_interrupt();
#endif
        break;
    case SENSOR_KC_MOTION_WALK_START:
#ifdef CONFIG_INPUT_KC_MOTION_SENSOR
        kc_motion_sensor_walk_start_interrupt();
#endif
        break;
    case SENSOR_KC_MOTION_WALK_STOP:
#ifdef CONFIG_INPUT_KC_MOTION_SENSOR
        kc_motion_sensor_walk_stop_interrupt();
#endif
        break;
    case SENSOR_KC_MOTION_TRAIN:
#ifdef CONFIG_INPUT_KC_MOTION_SENSOR
        kc_motion_sensor_train_interrupt();
#endif
        break;
    case SENSOR_KC_MOTION_VEHICLE:
#ifdef CONFIG_INPUT_KC_MOTION_SENSOR
        kc_motion_sensor_vehicle_interrupt();
#endif
        break;
    case SENSOR_KC_MOTION_BRINGUP:
#ifdef CONFIG_INPUT_KC_MOTION_SENSOR
        kc_motion_sensor_bringup_interrupt();
#endif
        break;
    case SENSOR_ACC_AUTO_CAL:
#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
        acc_auto_cal_interrupt();
#endif
        break;
    case SENSOR_UNDERWATER_DETECT:
#ifdef CONFIG_INPUT_SENSOR_UNDERWATER_DETECT
        underwater_detect_water_interrupt();
#endif
        break;
    default:
        break;
    }

    SENSOR_N_LOG("end");
    return;
}

void sensor_report_data( enum sensor_e_type type,  uint32_t data )
{
    SENSOR_N_LOG("start");

    switch (type) {
    case SENSOR_LIGHT:
#ifdef CONFIG_INPUT_SENSOR_LIGHT
        light_input_report(data);
#endif
        break;
    default:
        break;
    }

    SENSOR_N_LOG("end");
    return;
}

void sensor_report_event( enum sensor_e_type type, int64_t timestamp, void *event, uint32_t len )
{
    SENSOR_N_LOG("start");

    switch (type) {
    case SENSOR_EXT_PEDO:
        sensor_ext_input_report(timestamp, event, len);
        break;
    default:
        break;
    }

    SENSOR_N_LOG("end");
    return;
}

void sensor_set_poll_time(
    enum sensor_e_type type,
    struct sensor_poll_info_str* poll_info,
    int32_t polltime
)
{
    int32_t ret;
    struct sensor_batch_info_str batch_info = {
        0, polltime, 0
    };
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("parm: polltime[%d]",(int)polltime);

    mutex_lock(&sensor_com_mutex);

    ret = sns_set_batch(type, &batch_info);
    if (ret != 0) {
        sns_err_check();
    }

    mutex_unlock(&sensor_com_mutex);
    SENSOR_N_LOG("end");
    return;
};

int sensor_input_init( struct sensor_input_info_str* info )
{
    struct input_dev *dev;
    int err;

    SENSOR_N_LOG("start");
    SENSOR_N_LOG("in parms info[%d]",(int)info );

    if(!info){
        SENSOR_ERR_LOG("fail bad parm --> info[%d]",(int)info);
        SENSOR_ERR_LOG("end return[-1]");
        return -1;
    }
    if( (!(info->param_func)) || (!(info->attr_grp)) ){
        SENSOR_ERR_LOG("fail bad parm --> info.func[%d] info.attr_grp[%d]",
                        (int)(info->param_func), (int)(info->attr_grp));
        SENSOR_ERR_LOG("end return[-1]");
        return -1;
    }

    dev = input_allocate_device();
    if (!(dev)) {
        SENSOR_ERR_LOG("fail input_allocate_device()-->ret[%d]",(int)(dev));
        SENSOR_ERR_LOG("end return[%d]",-ENOMEM);
        return -ENOMEM;
    }

    (*(info->param_func))(dev);
    input_set_abs_params(dev, ABS_MISC+1, INT_MIN,INT_MAX, 0, 0);
    input_set_abs_params(dev, ABS_MISC+2, INT_MIN,INT_MAX, 0, 0);

    err = input_register_device(dev);
    if (err < 0) {
        SENSOR_ERR_LOG("falut input_register_device()-->ret[%d]",err);
        input_free_device(dev);

        SENSOR_ERR_LOG("end return[%d]",err);
        return err;
    }

    err = sysfs_create_group(&dev->dev.kobj, (info->attr_grp));
    if (err < 0) {
        SENSOR_ERR_LOG("fail sysfs_create_group()-->ret[%d]",err);
        SENSOR_ERR_LOG("end return[%d]",-ENODEV);
        return err;
    }

    (info->dev) = dev;

    SENSOR_N_LOG("end -return[0] info_dev[%d]",(int)(info->dev));
    return 0;
}

static void sensor_com_init(void)
{
    int ret = 0;

    SENSOR_N_LOG("start");

    ret = sensor_input_init( &sensor_com_input_info );
    SENSOR_N_LOG("sensor_input_init()-->ret[%d] sensor_com_dev[%d]",
                  ret, (int)sensor_com_input_info.dev );

    if( (0 != ret) || (NULL == (sensor_com_input_info.dev)) ) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        SENSOR_ERR_LOG("end return[%d]",-ENODEV);
        return;
    }

    mutex_init(&sensor_com_mutex);
    mutex_init(&sensor_batch_mutex);

    atomic_set(&sensor_type_status, 0);

    SENSOR_N_LOG("end");
    return;
}

static void sensor_suspend_ctl( void )
{
    uint32_t type;
    uint32_t state_app;
    int32_t ret = 0;

    SENSOR_N_LOG("start");

    for( type=0; type<SENSOR_MAX; type++){
        if( SENSOR_ON == sensor_get_status_local(type) ){
            state_app = (SENSOR_ON << type);
            SENSOR_N_LOG("SENSOR_ON state_app:%x",state_app);
            if((state_app & SENSOR_NOT_OFF_SUSPEND_TYPE) == 0){
                ret = sensor_application_enable( type, false );
                SENSOR_N_LOG("SUSPEND OFF state_app:%x",state_app);
            }
        }
    }

    sns_suspend();

    if( SENSOR_ON == sensor_get_status_local(SENSOR_LIGHT) ){
       ret = sensor_type_enable( SENSOR_LIGHT, false );
    }

    if(( SENSOR_ON == sensor_get_status_local(SENSOR_SGNFCNT_MTN) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_EXT_PEDO) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_EXT_VEHI) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_EXT_IWIFI) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_EXT_BARO) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_STEP_CNT) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_STEP_DTC) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_KC_MOTION_WALK_START) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_KC_MOTION_WALK_STOP) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_KC_MOTION_TRAIN) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_KC_MOTION_VEHICLE) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_KC_MOTION_BRINGUP) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_UNDERWATER_DETECT) ) ) {
        sns_enable_irq_wake_irq(true);
    }

    SENSOR_N_LOG("end");
    return;
}

static void sensor_resume_ctl( void )
{
    uint32_t type;
    uint32_t state_app;
    int32_t ret = 0;

    SENSOR_N_LOG("start");

    if(( SENSOR_ON == sensor_get_status_local(SENSOR_SGNFCNT_MTN) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_EXT_VEHI) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_EXT_IWIFI) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_STEP_CNT) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_STEP_DTC) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_KC_MOTION_WALK_START) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_KC_MOTION_WALK_STOP) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_KC_MOTION_TRAIN) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_KC_MOTION_VEHICLE) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_KC_MOTION_BRINGUP) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_UNDERWATER_DETECT) ) ) {
        sns_enable_irq_wake_irq(false);
    }

    if( SENSOR_ON == sensor_get_status_local(SENSOR_LIGHT) ){
       ret = sensor_type_enable( SENSOR_LIGHT, true );
    }

    sns_resume();

    for( type=0; type<SENSOR_MAX; type++){
        if( SENSOR_ON == sensor_get_status_local(type) ){
            state_app = (SENSOR_ON << type);
            SENSOR_N_LOG("SENSOR_ON state_app:%x",state_app);
            if((state_app & SENSOR_NOT_OFF_SUSPEND_TYPE) == 0){
                ret = sensor_application_enable( type, true );
                SENSOR_N_LOG("RESUME ON state_app:%x",state_app);
            }
        }
    }

    SENSOR_N_LOG("end");
    return;
}

static void sensor_shutdown_ctl( void )
{
    int i;
    uint32_t type;
    int32_t ret = 0;

    SENSOR_N_LOG("start");

    for(i=0; i<sensor_poll_ctl_tbl_size; i++){
        if( SENSOR_ON == sensor_get_status_local(sensor_poll_ctl_tbl[i].type)){
            if( NULL != sensor_poll_ctl_tbl[i].poll_info_p){
               mutex_unlock(&sensor_com_mutex);
               cancel_delayed_work_sync(&(sensor_poll_ctl_tbl[i].poll_info_p->poll_work));
               mutex_lock(&sensor_com_mutex);
            }
        }
    }

    for( type=0; type<SENSOR_MAX; type++){
        if( SENSOR_ON == sensor_get_status_local(type) ){
            ret = sensor_application_enable( type, false );
        }
    }

    for( type=0; type<SENSOR_MAX; type++){
        if( SENSOR_ON == sensor_get_status_local(type) ){
            ret = sensor_type_enable( type, false );
        }
    }
    atomic_set(&sensor_type_status, 0);

    ret = sns_shutdown();

    SENSOR_N_LOG("end");
    return;
}

static void sensor_reset_ctl( void )
{
    uint32_t type;
    int32_t ret = 0;

    SENSOR_ERR_LOG("start");

    for( type=0; type<SENSOR_MAX; type++){
        if( SENSOR_ON == sensor_get_status_local(type) ){
            ret = sensor_application_enable( type, false );
        }
    }
    SENSOR_ERR_LOG("end");
    return;
}

static void sensor_reset_resume_ctl( void )
{
    uint32_t type;
    int32_t ret = 0;

    SENSOR_ERR_LOG("start");

    for( type=0; type<SENSOR_MAX; type++){
        if( SENSOR_ON == sensor_get_status_local(type) ){
            ret = sensor_application_enable( type, true );
        }
    }

    SENSOR_ERR_LOG("end");
    return;
}

void sensor_suspend( void )
{
    uint32_t batch_status = 0;

    SENSOR_N_LOG("start");
    mutex_lock(&sensor_batch_mutex);
    mutex_lock(&sensor_com_mutex);

    sensor_suspend_ctl();

    sensor_drv_set_status_local( SENSOR_SUSPEND );

    batch_status = sensor_get_batch_status();
    if( 0 != batch_status) {
        sns_set_buff_int(false);
    }

    mutex_unlock(&sensor_com_mutex);
    mutex_unlock(&sensor_batch_mutex);

    SENSOR_N_LOG("end");
    return;
}

void sensor_resume( void )
{
    uint32_t batch_status = 0;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_batch_mutex);
    mutex_lock(&sensor_com_mutex);

    sensor_resume_ctl();

    sensor_drv_set_status_local( SENSOR_RESUME );

    batch_status = sensor_get_batch_status();
    if( 0 != batch_status) {
        sns_set_buff_int(true);
    }

    mutex_unlock(&sensor_com_mutex);
    mutex_unlock(&sensor_batch_mutex);

    SENSOR_N_LOG("end");
    return;
}

void sensor_shutdown( void )
{
    SENSOR_N_LOG("start");
    mutex_lock(&sensor_com_mutex);

    sensor_shutdown_ctl();

    sensor_drv_set_status_local( SENSOR_SHUTDOWN );
    mutex_unlock(&sensor_com_mutex);
    SENSOR_N_LOG("end");
    return;
}

void sensor_reset( void )
{
    SENSOR_N_LOG("start");

    sensor_reset_ctl();

    sensor_drv_set_status_local( SENSOR_RESET );
    SENSOR_N_LOG("end");
    return;
}

void sensor_reset_resume( void )
{
    SENSOR_N_LOG("start");

    sensor_reset_resume_ctl();

    sensor_drv_set_status_local( SENSOR_RESUME );
    SENSOR_N_LOG("end");
    return;
}

static int32_t __init sensor_init(void)
{
    SENSOR_N_LOG("start");

    sensor_micon_init();
#ifdef CONFIG_INPUT_SENSOR_GP2AP030
    gp2ap_init();
#endif
#ifdef CONFIG_SENSOR_COMMON
    sensor_com_init();
#endif

#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
    acc_driver_init();
#endif
#ifdef CONFIG_INPUT_SENSOR_LINEAR_ACCELERATION
    acc_lnr_driver_init();
#endif
#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
    gyro_driver_init();
#endif
#ifdef CONFIG_INPUT_SENSOR_UNCAL_GYROSCOPE
    gyro_uncal_driver_init();
#endif
#ifdef CONFIG_INPUT_SENSOR_GRAVITY
    grv_driver_init();
#endif
#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
    mag_driver_init();
#endif
#ifdef CONFIG_INPUT_SENSOR_UNCAL_MAGNETOMETER
    mag_uncal_driver_init();
#endif
#ifdef CONFIG_INPUT_SENSOR_RELATIVE_HUMIDITY
    relative_humidity_driver_init();
#endif
#ifdef CONFIG_INPUT_SENSOR_ROTATION_VECTOR
    rot_vctr_driver_init();
#endif
#ifdef CONFIG_INPUT_SENSOR_GAME_ROTATION_VECTOR
    game_rot_vctr_driver_init();
#endif
#ifdef CONFIG_INPUT_SENSOR_MAG_ROTATION_VECTOR
    mag_rot_vctr_driver_init();
#endif
#ifdef CONFIG_INPUT_SENSOR_ORIENTATION
    ortn_driver_init();
#endif
#ifdef CONFIG_INPUT_SENSOR_PRESSURE
    pressure_driver_init();
#endif
#ifdef CONFIG_INPUT_SENSOR_TEMPERATURE
    temperature_driver_init();
#endif
#ifdef CONFIG_INPUT_SENSOR_AMBIENT_TEMPERATURE
    temperature_ambient_driver_init();
#endif
#ifdef CONFIG_INPUT_SENSOR_SIGNIFICANT_MOTION
    sgnfcnt_mtn_driver_init();
#endif
#ifdef CONFIG_INPUT_SENSOR_STEP_COUNTER
    step_cnt_driver_init();
#endif
#ifdef CONFIG_INPUT_SENSOR_STEP_DETECTOR
    step_dtc_driver_init();
#endif
#ifdef CONFIG_INPUT_SENSOR_EXT
    sensor_ext_driver_init();
#endif
#ifdef CONFIG_INPUT_SENSOR_LIGHT
    light_driver_init();
#endif
#ifdef CONFIG_INPUT_SENSOR_PROXIMITY
    prox_driver_init();
#endif
#ifdef CONFIG_INPUT_SENSOR_EXT_VH
    sensor_ext_vh_driver_init();
#endif
#ifdef CONFIG_INPUT_KC_MOTION_SENSOR
    kc_motion_sensor_driver_init();
#endif
#ifdef CONFIG_INPUT_SENSOR_UNDERWATER_DETECT
    underwater_detect_driver_init();
#endif

    sensor_drv_set_status_local( SENSOR_RESUME );

    SENSOR_N_LOG("end");

    return 0;

}

static void __exit sensor_exit(void)
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end");
}

module_init(sensor_init);
module_exit(sensor_exit);

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("Sensor Common");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sensor_driver.c");

