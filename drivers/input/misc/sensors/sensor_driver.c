/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 * (C) 2017 KYOCERA Corporation
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

//#include <linux/module.h>
//#include <linux/kernel.h>
//#include <linux/android_alarm.h>
#include "sensor_driver.h"
#include "sensor_micon_driver.h"
#include "sensor_com.h"

#define MICON_FIFO_SIZE 3072

static ssize_t sensor_com_info_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t sensor_com_version_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t sensor_ts_nrfilt_prm_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t sensor_ts_nrfilt_prm_show(struct device *dev,
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
static DEVICE_ATTR(info,
    S_IRUSR|S_IRGRP,
    sensor_com_info_show,
    NULL
);

static DEVICE_ATTR(version,
    S_IRUSR|S_IRGRP,
    sensor_com_version_show,
    NULL
);

static DEVICE_ATTR(ts_nrfilt_prm,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    sensor_ts_nrfilt_prm_show,
    sensor_ts_nrfilt_prm_store
);

static struct attribute *sensor_com_attributes[] = {
    &dev_attr_info.attr,
    &dev_attr_version.attr,
    &dev_attr_ts_nrfilt_prm.attr,
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
static DECLARE_BITMAP(sensor_type_status, SENSOR_MAX);
static DECLARE_BITMAP(sensor_not_support_type, SENSOR_MAX);
static DECLARE_BITMAP(sensor_not_micon_type, SENSOR_MAX);
static DECLARE_BITMAP(sensor_not_off_suspend_type, SENSOR_MAX);
static enum sensor_drv_status_e_type sensor_drv_status = 0;

static ssize_t sensor_com_info_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int ret = 0;
    DECLARE_BITMAP(batch_status, SENSOR_MAX);

    ret += scnprintf(buf, PAGE_SIZE,
                    "type[");
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
    ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%*pb", SENSOR_MAX, sensor_type_status);
#else
    ret += bitmap_scnprintf(buf + ret, PAGE_SIZE - ret, sensor_type_status, SENSOR_MAX);
#endif /* LINUX_VERSION_CODE */
    ret += scnprintf(buf + ret, PAGE_SIZE - ret,
                    "] batch[");
    sensor_get_batch_status(batch_status);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
    ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%*pb", SENSOR_MAX, batch_status);
#else
    ret += bitmap_scnprintf(buf + ret, PAGE_SIZE - ret, batch_status, SENSOR_MAX);
#endif /* LINUX_VERSION_CODE */
    ret += scnprintf(buf + ret, PAGE_SIZE - ret,
                    "] drv[%08X]\n",
                    (int)sensor_drv_status);

    return ret;
}

static ssize_t sensor_com_version_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int ret = 0;
    ret += scnprintf(buf, PAGE_SIZE,"Ver.%04d %d\n", SOFT_VERSION, SOFT_BUILD_DATE);
    return ret;
}

static ssize_t sensor_ts_nrfilt_prm_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    struct nr_filt_prm param;
    SENSOR_D_LOG("start");
    sscanf(buf, "%d %d %d %d %d %d %d",
                &param.ts_th_us,
                &param.ts_rel_th,
                &param.ts_base_filt_coff,
                &param.ts_base_filt_acoff,
                &param.ts_filt_coff,
                &param.ts_ng_th,
                &param.ts_unaccept_num);
    SENSOR_D_LOG("ts_th_us:%d, ts_rel_th:%d, ts_base_filt_coff:%d, ts_base_filt_acoff:%d, ts_filt_coff:%d, ts_ng_th:%d, ts_unaccept_num:%d\n",
                (int)param.ts_th_us, (int)param.ts_rel_th, (int)param.ts_base_filt_coff,
                (int)param.ts_base_filt_acoff, (int)param.ts_filt_coff, (int)param.ts_ng_th,
                (int)param.ts_unaccept_num);
    sensor_set_nrfilt_param(NR_TIME, param);
    SENSOR_D_LOG("end");
    return count;
}

static ssize_t sensor_ts_nrfilt_prm_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t count = 0;
    struct nr_filt_prm param;
    SENSOR_D_LOG("start");
    sensor_get_nrfilt_param(NR_TIME, &param);
    count += scnprintf(buf + count, PAGE_SIZE - count,
                        "TS_NR_FILT_TH_US[%d]\n", param.ts_th_us);
    count += scnprintf(buf + count, PAGE_SIZE - count,
                        "TS_NR_REL_TH[%d]\n", param.ts_rel_th);
    count += scnprintf(buf + count, PAGE_SIZE - count,
                        "TS_NR_BASE_FILT_COFF[%d]\n", param.ts_base_filt_coff);
    count += scnprintf(buf + count, PAGE_SIZE - count,
                        "TS_NR_BASE_FILT_ACOFF[%d]\n", param.ts_base_filt_acoff);
    count += scnprintf(buf + count, PAGE_SIZE - count,
                        "TS_NR_FILT_COFF[%d]\n", param.ts_filt_coff);
    count += scnprintf(buf + count, PAGE_SIZE - count,
                        "TS_NR_NG_TH[%d]\n", param.ts_ng_th);
    count += scnprintf(buf + count, PAGE_SIZE - count,
                        "TS_NR_UNACCEPT_NUM[%d]\n", param.ts_unaccept_num);
    SENSOR_D_LOG("end");
    return count;
}

static enum sensor_chk_status_e_type sensor_check_status(
    enum sensor_e_type type,
    bool enable )
{
    enum sensor_chk_status_e_type ret = SENSOR_CHK_NORMAL;

    if(test_bit(type, sensor_not_support_type)){
        SENSOR_ERR_LOG("not supported!! type[%d]",(int)type);
        return SENSOR_CHK_SKIP_ALL;
    }

    if( true == enable ){
        if( SENSOR_ON == sensor_get_status_local(type) &&
            SENSOR_SGNFCNT_MTN != type) {
            ret = SENSOR_CHK_SKIP_ALL;
        } else if (!test_bit(type, sensor_not_micon_type)){
            ret = SENSOR_CHK_MICON_CTL;
        }
    } else {
        if( SENSOR_OFF == sensor_get_status_local(type) ){
            ret = SENSOR_CHK_SKIP_ALL;
        } else if (!test_bit(type, sensor_not_micon_type)){
            ret = SENSOR_CHK_MICON_CTL;
        }
    }

    return ret;
}

static void sensor_com_set_input_params( struct input_dev *dev )
{
    SENSOR_N_LOG("start");

    if(!dev){
        SENSOR_ERR_LOG("bad parm --> dev is NULL");
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
#if defined(CONFIG_INPUT_SENSOR_RPR0521) || defined(CONFIG_INPUT_SENSOR_APDS9960) || defined(CONFIG_INPUT_SENSOR_STK3338)
        ret = als_sensor_activate( enable );
#endif
        break;
    case SENSOR_PROX:
#ifdef CONFIG_INPUT_SENSOR_GP2AP030
        gp2ap_ps_sensor_activate( enable );
#endif
#if defined(CONFIG_INPUT_SENSOR_RPR0521) || defined(CONFIG_INPUT_SENSOR_APDS9960) || defined(CONFIG_INPUT_SENSOR_STK3338)
        ret = ps_sensor_activate( enable );
#endif
        break;
    case SENSOR_GESTURE:
#if defined(CONFIG_INPUT_SENSOR_APDS9960)
        ret = gs_sensor_activate( enable );
#endif
        break;
    case SENSOR_SGNFCNT_MTN:
        break;
    case SENSOR_STEP_CNT:
        break;
    case SENSOR_STEP_DTC:
        break;
    case SENSOR_DEVICE_ORIENTATION:
        break;
    case SENSOR_STATIONARY_DETECT:
        break;
    case SENSOR_MOTION_DETECT:
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
    case SENSOR_PIEZO_PRESS:
#ifdef CONFIG_INPUT_SENSOR_PIEZO_PRESS
        ret = msp430_piezo_press_sensor_activate(enable);
#endif
        break;
    case SENSOR_PIEZO_WAKEUP:
#ifdef CONFIG_INPUT_SENSOR_PIEZO_WAKEUP
        ret = msp430_piezo_wakeup_sensor_activate(enable);
#endif
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
        SENSOR_ERR_LOG("invalid type :%d",type);
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
        if (SENSOR_ON == sensor_get_status(SENSOR_DEVICE_ORIENTATION))
            break;
        ret = sns_vhdetect_start( enable );
        break;
    case SENSOR_DEVICE_ORIENTATION:
        if (SENSOR_ON == sensor_get_status(SENSOR_EXT_VH))
            break;
        ret = sns_vhdetect_start( enable );
        break;
    case SENSOR_MOTION_DETECT:
        ret = sns_motion_detect_start( enable );
        break;
    case SENSOR_STATIONARY_DETECT:
        ret = sns_stationary_detect_start( enable );
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
    case SENSOR_VOICE_TRIGGER:
        ret = sns_voice_trigger_start( enable );
        break;

    default:
        SENSOR_D_LOG("invalid type :%d",type);
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
    int32_t  err_chk = 0;

    mutex_lock(&sensor_com_mutex);

    ret = sns_set_batch(type, &info);
    if (ret != 0) {
        err_chk = sns_err_check();
        SENSOR_ERR_LOG("sns_err_check :%d",err_chk);
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

    if (!test_bit(type, sensor_not_micon_type)){
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
    status = (uint32_t)test_bit(type, sensor_type_status);
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
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
    SENSOR_N_LOG("start parm: type[%d] on[%d] - status[%*pb]",
                 (int)type, (int)on, SENSOR_MAX, sensor_type_status);
#else
    char buf[SENSOR_TYPE_STATUS_LOG_SIZE];
    bitmap_scnprintf(buf, SENSOR_TYPE_STATUS_LOG_SIZE, sensor_type_status, SENSOR_MAX);
    SENSOR_N_LOG("start parm: type[%d] on[%d] - status[%s]",
                 (int)type,(int)on,buf);
#endif /* LINUX_VERSION_CODE */

    if( on == SENSOR_ON) {
        set_bit(type, sensor_type_status);
    } else {
        clear_bit(type, sensor_type_status);
    }

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
    SENSOR_N_LOG("status to [%*pb]", SENSOR_MAX, sensor_type_status);
#else
    bitmap_scnprintf(buf, SENSOR_TYPE_STATUS_LOG_SIZE, sensor_type_status, SENSOR_MAX);
    SENSOR_N_LOG("status to [%s]",buf);
#endif /* LINUX_VERSION_CODE */
    SENSOR_N_LOG("end");
    return;
}

void sensor_get_batch_status(unsigned long *logging_enable_status)
{
    SENSOR_N_LOG("start");
    sns_get_logging_enable(logging_enable_status);
    SENSOR_N_LOG("end");

    return;
}

static void sensor_inittimer(struct hrtimer *timer, enum hrtimer_restart (*callback)(struct hrtimer *))
{
    hrtimer_init(timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
    timer->function = callback;
}

void sensor_starttimer(struct hrtimer *timer, int delay_ms)
{
    SENSOR_N_LOG("start");
    hrtimer_start(timer, ms_to_ktime(delay_ms), HRTIMER_MODE_REL);
    SENSOR_N_LOG("end");
}

void sensor_stoptimer(struct hrtimer *timer)
{
    SENSOR_N_LOG("start");
    hrtimer_cancel(timer);
    SENSOR_N_LOG("end");
}

static void sensor_inform_ts_noize_reduction_info(const char *wqname, int32_t delay_ms)
{
    SENSOR_D_LOG("start");
    SENSOR_D_LOG("workqueue name[%s] delay:%d msec",
                        wqname, delay_ms);
    if(!strcmp(wqname, "acc_poll_wq")){
        SENSOR_D_LOG("try to set acc ts_base_val.");
        sns_acc_set_ts_nrfilt_baseinfo(delay_ms);
    }
    else if(!strcmp(wqname, "gyro_poll_wq")) {
        SENSOR_D_LOG("try to set gyro ts_base_val.");
        sns_gyro_set_ts_nrfilt_baseinfo(delay_ms);
    }
    else if(!strcmp(wqname, "gyro_uncal_poll_wq")) {
        SENSOR_D_LOG("try to set gyro_uc ts_base_val.");
        sns_gyrouc_set_ts_nrfilt_baseinfo(delay_ms);
    }
    SENSOR_D_LOG("end");
}

enum hrtimer_restart sensor_poll(struct hrtimer *timer)
{
    struct sensor_poll_info_str *obj = (struct sensor_poll_info_str *)container_of(timer, struct sensor_poll_info_str, timer);
    int delay_ms;

    SENSOR_N_LOG("start");

    delay_ms = atomic_read(&(obj->poll_time));
    sensor_inform_ts_noize_reduction_info(obj->name, delay_ms);
    queue_work(obj->poll_wq, &obj->work);
    hrtimer_forward(timer, hrtimer_get_expires(timer), ms_to_ktime(delay_ms));

    SENSOR_N_LOG("end");

    return HRTIMER_RESTART;
}

void sensor_poll_init(
    enum sensor_e_type type,
    struct sensor_poll_info_str* poll_info )
{
    SENSOR_N_LOG("start");

    if(!poll_info){
        SENSOR_ERR_LOG("poll_info null");
        return;
    }

    poll_info->poll_wq = create_workqueue(poll_info->name);
    INIT_WORK(&(poll_info->work), (poll_info->poll_func));
    sensor_inittimer(&(poll_info->timer), sensor_poll);

    sns_poll_init(type, poll_info);

    SENSOR_N_LOG("end");

    return;
}

int32_t sensor_type_get_data(
    enum sensor_e_type type,
    union sensor_read_data_u* read_data )
{
    int32_t ret = 0;
    int32_t err_chk = 0;

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
    case SENSOR_GESTURE:
        break;
    case SENSOR_SGNFCNT_MTN:
        break;
    case SENSOR_STEP_CNT:
        break;
    case SENSOR_STEP_DTC:
        break;
    case SENSOR_DEVICE_ORIENTATION:
        break;
    case SENSOR_STATIONARY_DETECT:
        break;
    case SENSOR_MOTION_DETECT:
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
        SENSOR_ERR_LOG("invalid type:%d",type);
        break;
    }

    if( ret != 0 ) {
        err_chk = sns_err_check();
        SENSOR_ERR_LOG("sns_err_check :%d",err_chk);
    }
    mutex_unlock(&sensor_com_mutex);

    return ret;
}

int32_t sensor_get_initialize_state(enum sensor_e_type type)
{
    int32_t ret = 0;

    SENSOR_N_LOG("start type[%d]", type);
    switch (type) {
    case SENSOR_LIGHT:
#ifdef CONFIG_INPUT_SENSOR_LIGHT
        ret = als_get_initialize_state();
#ifdef CONFIG_INPUT_SENSOR_GP2AP030
        ret = gp2ap_get_initialize_state();
#endif /*CONFIG_INPUT_SENSOR_GP2AP030*/
#endif /*CONFIG_INPUT_SENSOR_LIGHT*/
        break;
    case SENSOR_PROX:
#ifdef CONFIG_INPUT_SENSOR_PROXIMITY
        ret = ps_get_initialize_state();
#ifdef CONFIG_INPUT_SENSOR_GP2AP030
        ret = gp2ap_get_initialize_state();
#endif /*CONFIG_INPUT_SENSOR_GP2AP030*/
#endif /*INPUT_SENSOR_PROXIMITY*/
        break;
    case SENSOR_GESTURE:
#ifdef CONFIG_INPUT_SENSOR_GESTURE
        ret = gs_get_initialize_state();
#endif /*INPUT_SENSOR_GESTURE*/
        break;

#ifdef CONFIG_INPUT_SENSOR_PIEZO_PRESS
    case SENSOR_PIEZO_PRESS:
        ret = msp430_piezo_press_get_initialize_state();
#endif /*CONFIG_INPUT_SENSOR_PIEZO_PRESS*/

#ifdef CONFIG_INPUT_SENSOR_PIEZO_WAKEUP
    case SENSOR_PIEZO_WAKEUP:
        ret = msp430_piezo_wakeup_get_initialize_state();
#endif /*CONFIG_INPUT_SENSOR_PIEZO_WAKEUP*/
    default:
        SENSOR_ERR_LOG("invalid type:%d",type);
        break;
    }
    SENSOR_N_LOG("End ret[%d]", ret);

    return ret;
}

int32_t sensor_ext_get_data(
    enum sensor_e_type type,
    union sensor_ext_read_data_u* read_data )
{
    int32_t ret = 0;
    int32_t err_chk = 0;

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
    case SENSOR_DEVICE_ORIENTATION:
        ret = sns_get_vhdetect_data(&(read_data->vh_data));
        break;
    default:
        SENSOR_ERR_LOG("invalid type:%d",type);
        break;
    }

    if( ret != 0 ) {
        err_chk = sns_err_check();
        SENSOR_ERR_LOG("sns_err_check :%d",err_chk);
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
    int32_t err_chk = 0;

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
        SENSOR_ERR_LOG("invalid type:%d",type);
        break;
    }

    if( ret != 0 ) {
        err_chk = sns_err_check();
        SENSOR_ERR_LOG("sns_err_check :%d",err_chk);
    }
    mutex_unlock(&sensor_com_mutex);
    SENSOR_N_LOG("end");

    return ret;
}

int32_t sensor_ext_clear( uint32_t clear_req )
{
    int32_t ret = 0;
    int32_t err_chk = 0;

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
        err_chk = sns_err_check();
        SENSOR_ERR_LOG("sns_err_check :%d",err_chk);
    }
    mutex_unlock(&sensor_com_mutex);
    SENSOR_N_LOG("end");

    return ret;
}

int32_t sensor_uwater_get_data(
    uwater_detect_info_t* read_data )
{
    int32_t ret = 0;
    int32_t err_chk = 0;

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
        err_chk = sns_err_check();
        SENSOR_ERR_LOG("sns_err_check :%d",err_chk);
    }
    mutex_unlock(&sensor_com_mutex);
    SENSOR_N_LOG("end");

    return ret;
}

int32_t sensor_uwater_clear( void )
{
    int32_t ret = 0;
    int32_t err_chk = 0;

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
        err_chk = sns_err_check();
        SENSOR_ERR_LOG("sns_err_check :%d",err_chk);
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
    int32_t  err_chk = 0;
    enum sensor_chk_status_e_type chk_state = SENSOR_CHK_NORMAL;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
    /* Do Nothing. */
#else
    char buf[SENSOR_TYPE_STATUS_LOG_SIZE];
#endif /* LINUX_VERSION_CODE */

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_com_mutex);
    SENSOR_N_LOG("parm: type[%d], poll_info[%p] enable[%d]",
                 (int)type, poll_info, (int)enable);

    if (SENSOR_RESUME != sensor_drv_get_status_local()) {
        SENSOR_ERR_LOG("bad status-->sensor_drv_status[%d]",
                       (int)sensor_drv_status );
        mutex_unlock(&sensor_com_mutex);
        return;
    }

    chk_state = sensor_check_status( type, enable );
    if (SENSOR_CHK_SKIP_ALL == chk_state) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
        SENSOR_A_LOG("same status-->type[%d] sensor_type_status[%*pb]",
                      (int)type, SENSOR_MAX, sensor_type_status);
#else
        bitmap_scnprintf(buf, SENSOR_TYPE_STATUS_LOG_SIZE, sensor_type_status, SENSOR_MAX);
        SENSOR_A_LOG("same status-->type[%d] sensor_type_status[%s]",
                      (int)type,
                      buf);
#endif /* LINUX_VERSION_CODE */
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
        err_chk = sns_err_check();
        SENSOR_ERR_LOG("sns_err_check :%d",err_chk);
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

void sensor_notify( enum sensor_e_type type, uint32_t data)
{
    SENSOR_N_LOG("start");

    switch (type) {
    case SENSOR_EXT_IWIFI:
#ifdef CONFIG_INPUT_SENSOR_EXT
        sensor_ext_iwifi_interrupt();
#endif
        break;
    case SENSOR_EXT_VH:
#ifdef CONFIG_INPUT_SENSOR_EXT_VH
        g_AppVhFirstReportDone = true;
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
    case SENSOR_GESTURE:
#ifdef CONFIG_INPUT_SENSOR_GESTURE
        gesture_interrupt(data);
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
    case SENSOR_DEVICE_ORIENTATION:
#ifdef CONFIG_INPUT_SENSOR_DEVICE_ORIENTATION
        g_AppVhFirstReportDone = true;
        device_orientation_interrupt();
#endif
        break;
    case SENSOR_STATIONARY_DETECT:
#ifdef CONFIG_INPUT_SENSOR_STATIONARY_DETECT
        stationary_detect_interrupt();
#endif
        break;
    case SENSOR_MOTION_DETECT:
#ifdef CONFIG_INPUT_SENSOR_MOTION_DETECT
        motion_detect_interrupt();
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
#ifdef CONFIG_INPUT_SENSOR_VT_WAKEUP
    case VT_WAKEUP:
        voice_detect_wakeup_interrupt();
        break;
#endif
    case SENSOR_VOICE_TRIGGER:
#ifdef CONFIG_INPUT_SENSOR_VOICE_TRIGGER
        voice_trigger_interrupt((uint8_t)data);
        break;
#endif
    default:
        SENSOR_ERR_LOG("invalid type:%d",type);
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
    case SENSOR_GESTURE:
#ifdef CONFIG_INPUT_SENSOR_GESTURE
        gesture_input_report(data);
#endif
        break;
    default:
        SENSOR_ERR_LOG("invalid type:%d",type);
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
        SENSOR_ERR_LOG("invalid type:%d",type);
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
    int32_t  err_chk = 0;
    struct sensor_batch_info_str batch_info = {
        0, polltime, 0
    };
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("parm: polltime[%d]",(int)polltime);

    mutex_lock(&sensor_com_mutex);

    ret = sns_set_batch(type, &batch_info);
    if (ret != 0) {
        err_chk = sns_err_check();
        SENSOR_ERR_LOG("sns_err_check :%d",err_chk);
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
    SENSOR_N_LOG("in parms info[%p]",info );
    if(!info){
        SENSOR_ERR_LOG("fail bad parm --> info is NULL");
        SENSOR_ERR_LOG("end return[-1]");
        return -1;
    }
    if( (!(info->param_func)) || (!(info->attr_grp)) ){
        SENSOR_ERR_LOG("fail bad parm --> info.func:0 AND info.attr_grp:0");
        SENSOR_ERR_LOG("end return[-1]");
        return -1;
    }

    dev = input_allocate_device();
    if (!(dev)) {
        SENSOR_ERR_LOG("fail input_allocate_device()-->ret[dev:NULL]");
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
        return -ENODEV;
    }

    (info->dev) = dev;

    SENSOR_N_LOG("end -return[0] info_dev[%p]",info->dev);
    return 0;
}

static void sensor_com_init(void)
{
    int ret = 0;

    SENSOR_N_LOG("start");

    ret = sensor_input_init( &sensor_com_input_info );
    SENSOR_N_LOG("sensor_input_init()-->ret[%d] sensor_com_dev[%p]",
                ret, sensor_com_input_info.dev );

    if( (0 != ret) || (NULL == (sensor_com_input_info.dev)) ) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        SENSOR_ERR_LOG("end return[%d]",-ENODEV);
        return;
    }

    mutex_init(&sensor_com_mutex);
    mutex_init(&sensor_batch_mutex);

    bitmap_zero(sensor_type_status, SENSOR_MAX);

    SENSOR_N_LOG("end");
    return;
}


void sensor_save_offset_value(enum sensor_e_type type, void *savedata)
{
    SENSOR_D_LOG("start");
    switch (type){
        case SENSOR_ACC:
#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
            acc_save_cal_ofs_val((struct acceleration*)savedata);
#endif
            break;
        case SENSOR_MAG:
#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
            mag_save_cal_ofs_val((struct geomagnetic*)savedata);
#endif
            break;
        case SENSOR_GYRO:
#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
            gyro_save_cal_ofs_val((struct gyroscope*)savedata);
#endif
            break;
        case SENSOR_PRESSURE:
#ifdef CONFIG_INPUT_SENSOR_PRESSURE
            pressure_save_cal_ofs_val((struct pressure*)savedata);
#endif
            break;
        default:
            SENSOR_ERR_LOG("Invalid sns type[%d]", type);
            break;
    }
    SENSOR_D_LOG("end");
}

void sensor_load_offset_value(enum sensor_e_type type, void *outdata)
{
    SENSOR_D_LOG("start");
    switch (type){
        case SENSOR_ACC:
#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
            acc_load_cal_ofs_val(outdata);
#endif
            break;
        case SENSOR_MAG:
#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
            mag_load_cal_ofs_val(outdata);
#endif
            break;
        case SENSOR_GYRO:
#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
            gyro_load_cal_ofs_val(outdata);
#endif
            break;
        case SENSOR_PRESSURE:
#ifdef CONFIG_INPUT_SENSOR_PRESSURE
            pressure_load_cal_ofs_val(outdata);
#endif
            break;
        default:
            SENSOR_ERR_LOG("Invalid sns type[%d]", type);
            break;
    }
    SENSOR_D_LOG("end");
}

int32_t sensor_set_offset_value(enum sensor_e_type type, struct sens_ofs set_ofs)
{
    int32_t ret = SNS_RC_OK;
#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
    struct acceleration acc_ofs = {0};
#endif
#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
    struct geomagnetic  mag_ofs = {0};
#endif
#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
    struct gyroscope    gyro_ofs = {0};
#endif
#ifdef CONFIG_INPUT_SENSOR_PRESSURE
    struct pressure     press_ofs = {0};
#endif
    SENSOR_D_LOG("start");
    switch (type){
#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
        case SENSOR_ACC:
            acc_ofs.nX = set_ofs.x;
            acc_ofs.nY = set_ofs.y;
            acc_ofs.nZ = set_ofs.z;
            ret = sns_acc_set_auto_cal_offset(acc_ofs);
            break;
#endif
#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
        case SENSOR_MAG:
            mag_ofs.x = set_ofs.x;
            mag_ofs.y = set_ofs.y;
            mag_ofs.z = set_ofs.z;
            mag_ofs.accuracy = set_ofs.accuracy;
            ret = sns_mag_set_offset(mag_ofs);
            break;
#endif
#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
        case SENSOR_GYRO:
            gyro_ofs.x = set_ofs.x;
            gyro_ofs.y = set_ofs.y;
            gyro_ofs.z = set_ofs.z;
            ret = sns_gyro_set_offset(gyro_ofs);
            break;
        case SENSOR_PRESSURE:
#endif
#ifdef CONFIG_INPUT_SENSOR_PRESSURE
            press_ofs.pressure = set_ofs.x;
            ret = sns_pressure_set_cal_ofs(press_ofs);
            break;
#endif
        default:
            SENSOR_ERR_LOG("Invalid sns type[%d]", type);
            break;
    }
    SENSOR_D_LOG("end");
    return ret;
}

int32_t sensor_get_offset_value(enum sensor_e_type type, void* output)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    switch (type){
#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
        case SENSOR_ACC:
            ret = sns_acc_get_auto_cal_offset((struct acceleration*)output);
            break;
#endif
#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
        case SENSOR_MAG:
            ret = sns_mag_get_offset((struct geomagnetic*)output);
            break;
#endif
#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
        case SENSOR_GYRO:
            ret = sns_gyro_get_offset((struct gyroscope*)output);
            break;
#endif
#ifdef CONFIG_INPUT_SENSOR_PRESSURE
        case SENSOR_PRESSURE:
            ret = sns_pressure_get_cal_ofs((struct pressure*)output);
            break;
#endif
        default:
            SENSOR_ERR_LOG("Invalid sns type[%d]", type);
            break;
    }
    SENSOR_D_LOG("end");
    return ret;
}

int32_t sensor_set_nrfilt_param(enum noizereduction_e_type type, struct nr_filt_prm prm)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    switch (type){
        case NR_ACC:
            ret = sns_acc_set_nrfilt_param(prm);
            break;
        case NR_GYRO:
            ret = sns_gyro_set_nrfilt_param(prm);
            break;
        case NR_TIME:
            ret = sns_ts_set_nrfilt_param(prm);
            break;
        default:
            SENSOR_ERR_LOG("Invalid type[%d]", type);
            break;
    }
    SENSOR_D_LOG("end");
    return ret;

}

int32_t sensor_get_nrfilt_param(enum noizereduction_e_type type, struct nr_filt_prm *prm)
{
    int32_t ret = SNS_RC_OK;
    SENSOR_D_LOG("start");
    switch (type){
        case NR_ACC:
            ret = sns_acc_get_nrfilt_param(prm);
            break;
        case NR_GYRO:
            ret = sns_gyro_get_nrfilt_param(prm);
            break;
        case NR_TIME:
            ret = sns_ts_get_nrfilt_param(prm);
            break;
        default:
            SENSOR_ERR_LOG("Invalid type[%d]", type);
            break;
    }
    SENSOR_D_LOG("end");
    return ret;

}

void sensor_restore_status(enum sensor_e_type type, void *restore_param)
{
    SENSOR_D_LOG("start");
    switch (type){
        case SENSOR_EXT_PEDO:
            sensor_ext_update_ofs_val();
            break;
        default:
            SENSOR_ERR_LOG("Invalid sns type[%d]", type);
            break;
    }
    SENSOR_D_LOG("end");
}

uint8_t sensor_get_device_id(enum sensor_e_type type)
{
    int32_t ret = SNS_RC_ERR;
    SENSOR_D_LOG("start");
    switch (type){
        case SENSOR_ACC:
            ret = sns_acc_get_deviceid();
            break;
        case SENSOR_MAG:
            ret = sns_mag_get_deviceid();
            break;
        case SENSOR_GYRO:
            ret = sns_gyro_get_deviceid();
            break;
        case SENSOR_PRESSURE:
            ret = sns_pressure_get_deviceid();
            break;
        default:
            SENSOR_ERR_LOG("Invalid sns type[%d]", type);
            break;
    }
    SENSOR_D_LOG("end");
    return ret;
}

int8_t sensor_set_dynamiccalib_enable(enum sensor_e_type sns_type,
                                    bool enable)
{
    int8_t ret = 0;
    SENSOR_D_LOG("start");
    switch (sns_type){
        case SENSOR_ACC:
            ret = sns_acc_set_dynamiccalib(enable);
            break;
        case SENSOR_MAG:
            ret = sns_mag_set_dynamiccalib(enable);
            break;
        case SENSOR_GYRO:
            ret = sns_gyro_set_dynamiccalib(enable);
            break;
        default:
            SENSOR_ERR_LOG("do not be equipped dynamic calib. snstype[%d]", sns_type);
            break;
    }
    SENSOR_D_LOG("end");
    return ret;

}

int8_t sensor_set_selfchk_config(enum sensor_e_type sns_type,
                                enum selfchk_test_e_type selfchk_type)
{
    int8_t ret;
    SENSOR_D_LOG("start");
    switch (sns_type){
        case SENSOR_ACC:
            ret = sns_acc_set_selfchk_config(selfchk_type);
            break;
        case SENSOR_GYRO:
            ret = sns_gyro_set_selfchk_config(selfchk_type);
            break;
        default:
            SENSOR_ERR_LOG("need not run self check test. snstype[%d]", sns_type);
            break;
    }
    SENSOR_D_LOG("end");
    return ret;
}

int8_t sensor_start_manual_calib(enum sensor_e_type sns_type,
                                uint8_t *cal_prm,
                                enum manual_calib_e_type cal_phase)
{
    int8_t ret;
    SENSOR_D_LOG("start");
    switch (sns_type){
        case SENSOR_GYRO:
            if(cal_phase == SETTING){
                ret = sns_gyro_start_manual_cal(cal_prm);
            }
            else if (cal_phase == WAIT){
                ret = sns_gyro_wait_cal_result();
            } else {
                //nop
            }
            break;
        default:
            SENSOR_ERR_LOG("no manual calib. snstype[%d]", sns_type);
            break;
    }
    SENSOR_D_LOG("end");
    return ret;
}

void sensor_get_event_markoff_timestamps(int64_t *kts, int32_t *micontaskts)
{
    SENSOR_N_LOG("start");
    sns_dailys_get_event_markoff_timestamps(kts, micontaskts);
    SENSOR_N_LOG("end");
}

void sensor_get_event_markoff_clrinfo(uint8_t *is_clr_exist, int32_t *clr_miconts)
{
    SENSOR_D_LOG("start");
    sns_dailys_get_event_markoff_clrinfo(is_clr_exist, clr_miconts);
    SENSOR_D_LOG("end");
}


static void sensor_suspend_ctl( void )
{
    uint32_t type;
    int32_t ret = 0;

    SENSOR_N_LOG("start");

    for( type=0; type<SENSOR_MAX; type++){
        if( SENSOR_ON == sensor_get_status_local(type) ){
            SENSOR_N_LOG("SENSOR_ON type:%d",type);
            if(test_bit(type, sensor_not_off_suspend_type) == 0){
                ret = sensor_application_enable( type, false );
                SENSOR_N_LOG("SUSPEND OFF type:%d",type);
            }
        }
    }

    sns_suspend();

    if( SENSOR_ON == sensor_get_status_local(SENSOR_LIGHT) ){
       ret = sensor_type_enable( SENSOR_LIGHT, false );
    }
    if( SENSOR_ON == sensor_get_status_local(SENSOR_GESTURE) ){
       ret = sensor_type_enable( SENSOR_GESTURE, false );
    }

    if(( SENSOR_ON == sensor_get_status_local(SENSOR_SGNFCNT_MTN) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_EXT_PEDO) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_EXT_VEHI) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_EXT_IWIFI) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_STEP_CNT) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_EXT_BARO) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_STEP_DTC) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_KC_MOTION_WALK_START) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_KC_MOTION_WALK_STOP) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_KC_MOTION_TRAIN) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_KC_MOTION_VEHICLE) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_KC_MOTION_BRINGUP) )||
       ( SENSOR_ON == sensor_get_status_local(SENSOR_UNDERWATER_DETECT) ) ||
         SENSOR_ON == sensor_get_status_local(SENSOR_VOICE_TRIGGER)) {
        sns_enable_irq_wake_irq(true);
    }

    SENSOR_N_LOG("end");
    return;
}

static void sensor_resume_ctl( void )
{
    uint32_t type;
    int32_t ret = 0;

    SENSOR_N_LOG("start");

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
       ( SENSOR_ON == sensor_get_status_local(SENSOR_UNDERWATER_DETECT) )||
         SENSOR_ON == sensor_get_status_local(SENSOR_VOICE_TRIGGER)) {
        sns_enable_irq_wake_irq(false);
    }

    if( SENSOR_ON == sensor_get_status_local(SENSOR_LIGHT) ){
       ret = sensor_type_enable( SENSOR_LIGHT, true );
    }
    if( SENSOR_ON == sensor_get_status_local(SENSOR_GESTURE) ){
       ret = sensor_type_enable( SENSOR_GESTURE, true );
    }

    sns_resume();

    for( type=0; type<SENSOR_MAX; type++){
        if( SENSOR_ON == sensor_get_status_local(type) ){
            SENSOR_N_LOG("SENSOR_ON type:%d",type);
            if(test_bit(type, sensor_not_off_suspend_type) == 0){
                ret = sensor_application_enable( type, true );
                SENSOR_N_LOG("RESUME ON type:%d",type);
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
                sensor_stoptimer(&(sensor_poll_ctl_tbl[i].poll_info_p->timer));
                mutex_unlock(&sensor_com_mutex);
                cancel_work_sync(&(sensor_poll_ctl_tbl[i].poll_info_p->work));
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
    bitmap_zero(sensor_type_status, SENSOR_MAX);

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
    DECLARE_BITMAP(batch_status, SENSOR_MAX);

    SENSOR_N_LOG("start");
    mutex_lock(&sensor_batch_mutex);
    mutex_lock(&sensor_com_mutex);

    sensor_suspend_ctl();

    sensor_drv_set_status_local( SENSOR_SUSPEND );

    sensor_get_batch_status(batch_status);
    if(!bitmap_empty(batch_status, SENSOR_MAX)) {
        sns_set_buff_int(false);
    }

    mutex_unlock(&sensor_com_mutex);
    mutex_unlock(&sensor_batch_mutex);

    SENSOR_N_LOG("end");
    return;
}

void sensor_resume( void )
{
    DECLARE_BITMAP(batch_status, SENSOR_MAX);

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_batch_mutex);
    mutex_lock(&sensor_com_mutex);

    sensor_resume_ctl();

    sensor_drv_set_status_local( SENSOR_RESUME );

    sensor_get_batch_status(batch_status);
    if(!bitmap_empty(batch_status, SENSOR_MAX)) {
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
static void sensor_set_not_micon_type_bitmap(void)
{
    SENSOR_N_LOG("start");
    bitmap_zero(sensor_not_micon_type, SENSOR_MAX);

    set_bit(SENSOR_LIGHT, sensor_not_micon_type);
    set_bit(SENSOR_PROX, sensor_not_micon_type);
    set_bit(SENSOR_GESTURE, sensor_not_micon_type);
    SENSOR_N_LOG("end");
    return;
}

static void sensor_set_not_off_suspend_type_bitmap(void)
{
    SENSOR_N_LOG("start");
    bitmap_zero(sensor_not_off_suspend_type, SENSOR_MAX);

    set_bit(SENSOR_PROX, sensor_not_off_suspend_type);
    set_bit(SENSOR_EXT_PEDO, sensor_not_off_suspend_type);
    set_bit(SENSOR_EXT_VEHI, sensor_not_off_suspend_type);
    set_bit(SENSOR_EXT_BARO, sensor_not_off_suspend_type);
    set_bit(SENSOR_EXT_IWIFI, sensor_not_off_suspend_type);
    set_bit(SENSOR_SGNFCNT_MTN, sensor_not_off_suspend_type);
    set_bit(SENSOR_STEP_CNT, sensor_not_off_suspend_type);
    set_bit(SENSOR_STEP_DTC, sensor_not_off_suspend_type);
    set_bit(SENSOR_KC_MOTION_WALK_START, sensor_not_off_suspend_type);
    set_bit(SENSOR_KC_MOTION_WALK_STOP, sensor_not_off_suspend_type);
    set_bit(SENSOR_KC_MOTION_TRAIN, sensor_not_off_suspend_type);
    set_bit(SENSOR_KC_MOTION_VEHICLE, sensor_not_off_suspend_type);
    set_bit(SENSOR_KC_MOTION_BRINGUP, sensor_not_off_suspend_type);
    SENSOR_N_LOG("end");
    return;
}

static void sensor_set_not_support_type_bitmap(void)
{
    SENSOR_N_LOG("start");
    bitmap_zero(sensor_not_support_type, SENSOR_MAX);

#if 0
    set_bit(SENSOR_RH, sensor_not_support_type);
    set_bit(SENSOR_TEMP, sensor_not_support_type);
    set_bit(SENSOR_TEMP_AMBIENT, sensor_not_support_type);
#else
#endif
    SENSOR_N_LOG("end");
    return;
}

extern void sensor_gs_ignore_detection(bool enable)
{
    SENSOR_D_LOG("start enable[%d]", enable);
#ifdef CONFIG_INPUT_SENSOR_APDS9960
    apds_gs_ignore_detection(enable);
#else
    SENSOR_D_LOG("No Effect. GestureSensor[APDS9960] is not equipped.");
#endif /* CONFIG_INPUT_SENSOR_APDS9960 */
    SENSOR_D_LOG("end");
}
EXPORT_SYMBOL(sensor_gs_ignore_detection);

static int32_t __init sensor_init(void)
{
    int32_t ret = 0;
    SENSOR_N_LOG("start");

    ret = sensor_micon_init();
    if (ret != 0) {
        SENSOR_ERR_LOG("fail sensor_micon_init");
        goto ERR_INIT;
    }
#ifdef CONFIG_INPUT_SENSOR_GP2AP030
    gp2ap_init();
#endif
#if defined(CONFIG_INPUT_SENSOR_RPR0521) || defined(CONFIG_INPUT_SENSOR_STK3338) 
    ps_als_init();
#elif defined(CONFIG_INPUT_SENSOR_APDS9960)
    apds_init();
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
#ifdef CONFIG_INPUT_SENSOR_DEVICE_ORIENTATION
    device_orientation_driver_init();
#endif
#ifdef CONFIG_INPUT_SENSOR_STATIONARY_DETECT
    stationary_detect_driver_init();
#endif
#ifdef CONFIG_INPUT_SENSOR_MOTION_DETECT
    motion_detect_driver_init();
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
#ifdef CONFIG_INPUT_SENSOR_GESTURE
    gesture_driver_init();
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
#ifdef CONFIG_INPUT_TI_MSP430
    ret = ti_msp430_driver_init();
    if (ret != 0) {
        SENSOR_ERR_LOG("fail ti_msp430_driver_init");
        goto ERR_INIT;
    }
#endif
#ifdef CONFIG_INPUT_SENSOR_PIEZO_PRESS
    piezo_press_driver_init();
#endif /*CONFIG_INPUT_SENSOR_PIEZO_PRESS*/
#ifdef CONFIG_INPUT_SENSOR_PIEZO_WAKEUP
    piezo_wakeup_driver_init();
#endif /*CONFIG_INPUT_SENSOR_PIEZO_WAKEUP*/
#ifdef CONFIG_INPUT_SENSOR_VOICE_TRIGGER
    voice_trigger_driver_init();
#endif /*CONFIG_INPUT_SENSOR_VOICE_TRIGGER*/

    sensor_drv_set_status_local( SENSOR_RESUME );

    sensor_set_not_support_type_bitmap();
    sensor_set_not_off_suspend_type_bitmap();
    sensor_set_not_micon_type_bitmap();

ERR_INIT:
    SENSOR_N_LOG("end");

    return 0;

}

static void __exit sensor_exit(void)
{
    SENSOR_N_LOG("start");
#ifdef CONFIG_INPUT_SENSOR_ACCELEROMETER
    acc_driver_exit();
#endif
#ifdef CONFIG_INPUT_SENSOR_MAGNETOMETER
    mag_driver_exit();
#endif
#ifdef CONFIG_INPUT_SENSOR_GYROSCOPE
    gyro_driver_exit();
#endif
#ifdef CONFIG_INPUT_SENSOR_PRESSURE
    pressure_driver_exit();
#endif
    SENSOR_N_LOG("end");
}

module_init(sensor_init);
module_exit(sensor_exit);

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("Sensor Common");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sensor_driver.c");

