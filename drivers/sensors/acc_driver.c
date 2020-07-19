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
#include "sensor_com.h"

#define DEFAULT_ACC_DELAY    (30)
#define GRAVITY_EARTH        9806550
#define ABSMAX_4G            (GRAVITY_EARTH * 4)
#define ABSMIN_4G            (-GRAVITY_EARTH * 4)
#define OFFSET_AVE_NUM               (1)
#define OFFSET_SUMPLE_NUM            (100)
#define OFFSET_ACC_HIGH_TH           (500)
#define OFFSET_ACC_LOW_TH           (-500)

#define ONESEC_MS                    (1000)
#define SETTING_0G                   (0)
#define WEIGHT_1G                    (1024)

#define ABS_THROTTLE_DUMMY_VAL            (-1)

enum KC_SENSOR_COMMON_OPERATION_CODE
{
    KC_SENSOR_COMMON_POWER_ON = 0x00000000,
    KC_SENSOR_COMMON_POWER_OFF,
    KC_SENSOR_COMMON_INIT,
    KC_SENSOR_COMMON_IMITATION_ON,
    KC_SENSOR_COMMON_IMITATION_OFF,
    KC_SENSOR_COMMON_MAX
};


enum {
    MODE_0 = 0,
    MODE_1,
    MODE_2,
    MODE_3,
    MODE_4,
    MODE_MAX
}MODE;

enum {
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z,
    AXIS_XYZ_MAX
} AXIS;

typedef struct tMovFilterWork {
    int32_t* m_pSamplWork[AXIS_XYZ_MAX];
    int32_t  m_unSum[AXIS_XYZ_MAX];
    int32_t  m_unCnt[AXIS_XYZ_MAX];
    uint8_t  m_ucAveN;
} acc_cal_movfilterwork;

typedef struct tCalibrationCtrl {
    bool     m_bFilterEnable;
    bool     m_bWaitSetting;
    bool     m_bComplete;
    bool     m_bRengeChkErr;
    uint16_t m_unSmpN;
    acc_cal_movfilterwork m_tFilterWork;
    int32_t  m_nCalX;
    int32_t  m_nCalY;
    int32_t  m_nCalZ;
    int32_t  m_nCurrentSampleNum;
    int32_t  m_nSummationX;
    int32_t  m_nSummationY;
    int32_t  m_nSummationZ;
    int32_t  m_nMode;
    int32_t  m_nHighTH;
    int32_t  m_nLowTH;
} acc_cal_ctrl_str;


static ssize_t acc_batch_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t acc_batch_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t acc_flush_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);

static ssize_t acc_ope_device_show(struct device *dev,
    struct device_attribute *attr,
     char *buf);
static ssize_t acc_ope_device_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
   size_t count );
static ssize_t accsns_cal_th_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t accsns_cal_th_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t accsns_cal_smp_n_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t accsns_cal_smp_n_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t acc_cal_wait_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static int32_t accsns_mov_acc_avg ( 
    acc_cal_movfilterwork* pCtrl,
    int32_t sample,
    int32_t axis );
static void accsns_calibration_range_chk(
    const struct acceleration* accData );
static void accsns_calibration_periodic(
    const struct acceleration* accData );
static void acc_calib_work_func(struct work_struct *work);
static ssize_t acc_start_calib_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t acc_calib_mode_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t acc_host_cmd_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t acc_host_cmd_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t acc_data_raw_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t acc_offset_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t acc_offset_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t acc_offset_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t acc_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t acc_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t acc_delay_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t acc_delay_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t acc_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t acc_auto_cal_offset_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t acc_auto_cal_offset_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t acc_reg_rw_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t acc_reg_rw_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );


static void acc_update_last_read_data(void);
static void acc_poll_work_func(struct work_struct *work);
static void acc_set_input_params( struct input_dev *dev );


static DEVICE_ATTR(enable,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    acc_enable_show,
    acc_enable_store
);
static DEVICE_ATTR(delay,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    acc_delay_show,
    acc_delay_store
);
static DEVICE_ATTR(data,
    S_IRUSR|S_IRGRP,
    acc_data_show,
    NULL
);
static DEVICE_ATTR(offset,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    acc_offset_show,
    acc_offset_store
);
static DEVICE_ATTR(data_raw,
    S_IRUSR|S_IRGRP,
    acc_data_raw_show,
    NULL
);
static DEVICE_ATTR(host_cmd,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    acc_host_cmd_show,
    acc_host_cmd_store
);
static DEVICE_ATTR(cal_mode,
    S_IWUSR|S_IWGRP,
    NULL,
    acc_calib_mode_store
);
static DEVICE_ATTR(start_cal,
    S_IWUSR|S_IWGRP,
    NULL,
    acc_start_calib_store
);
static DEVICE_ATTR(cal_wait,
    S_IRUSR|S_IRGRP,
    acc_cal_wait_show,
    NULL
);
static DEVICE_ATTR(cal_smp_n,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    accsns_cal_smp_n_show,
    accsns_cal_smp_n_store
);
static DEVICE_ATTR(cal_th,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    accsns_cal_th_show,
    accsns_cal_th_store
);
static DEVICE_ATTR(ope_device,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    acc_ope_device_show,
    acc_ope_device_store
);
static DEVICE_ATTR(batch,
    S_IWUSR|S_IWGRP,
    NULL,
    acc_batch_store
);
static DEVICE_ATTR(batch_data,
    S_IRUSR|S_IRGRP,
    acc_batch_data_show,
    NULL
);
static DEVICE_ATTR(flush,
    S_IWUSR|S_IWGRP,
    NULL,
    acc_flush_store
);
static DEVICE_ATTR(auto_cal_offset,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    acc_auto_cal_offset_show,
    acc_auto_cal_offset_store
);
static DEVICE_ATTR(reg_rw,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    acc_reg_rw_show,
    acc_reg_rw_store
);

static struct attribute *acc_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_delay.attr,
    &dev_attr_data.attr,
    &dev_attr_offset.attr,
    &dev_attr_data_raw.attr,
    &dev_attr_host_cmd.attr,
    &dev_attr_cal_mode.attr,
    &dev_attr_start_cal.attr,
    &dev_attr_cal_wait.attr,
    &dev_attr_cal_smp_n.attr,
    &dev_attr_cal_th.attr,
    &dev_attr_ope_device.attr,
    &dev_attr_batch.attr,
    &dev_attr_batch_data.attr,
    &dev_attr_flush.attr,
    &dev_attr_auto_cal_offset.attr,
    &dev_attr_reg_rw.attr,
    NULL
};

static struct attribute_group acc_attr_grp = {
    .attrs = acc_attributes
};

struct sensor_input_info_str acc_input_info =
{
    NULL,
    acc_set_input_params,
    &acc_attr_grp,
};

struct sensor_poll_info_str acc_poll_info = {
    .name       = "acc_poll_wq",
    .poll_time  = ATOMIC_INIT(DEFAULT_ACC_DELAY),
    .poll_func  = acc_poll_work_func,
};

static struct acceleration acc_last_read_data = {
    0,0,0,0,0,0,
};


static atomic_t g_nCalX;
static atomic_t g_nCalY;
static atomic_t g_nCalZ;
static acc_cal_ctrl_str acc_calib_ctrl;
struct sensor_poll_info_str acc_calib_wq_info = {
    .name       = "accsns_cal_wq",
    .poll_time  = ATOMIC_INIT(DEFAULT_ACC_DELAY),
    .poll_func  = acc_calib_work_func,
};
static struct acceleration acc_calib_read_data = {
    0,0,0,0,0,0,
};
static bool g_bIsAlreadyExistAccImitationXYZData       = false;

static struct sensor_batch_data_str acc_batch_data;
static uint32_t g_time_stamp_acc      = 0;
static uint32_t g_input_num_acc       = 0;
extern struct mutex sensor_batch_mutex;

static ssize_t acc_batch_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    uint32_t ret = 0;
    struct sensor_batch_info_str batch_info;

    SENSOR_N_LOG("start");

    sscanf( buf, "%d %d %d",
             &batch_info.flags,
             &batch_info.period_ns,
             &batch_info.timeout );
    SENSOR_N_LOG("parm - flags[%x] period_ns[%x] timeout[%x]",
                  (int)batch_info.flags,
                  (int)batch_info.period_ns,
                  (int)batch_info.timeout );

    ret = sensor_set_batch( SENSOR_ACC, batch_info);

    SENSOR_N_LOG("end - return[%d]",count);
    return count;
}

static ssize_t acc_batch_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret =0;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_batch_mutex);

    ret = sprintf(buf, "%d %d %d %d\n",acc_batch_data.payload_size,
                       acc_batch_data.recode_num, g_input_num_acc, g_time_stamp_acc);

    g_time_stamp_acc = 0;

    mutex_unlock(&sensor_batch_mutex);

    SENSOR_N_LOG("end");

    return ret;
}

void acc_ring_buffer_timestamp(
    uint32_t time_stamp_acc)
{
    SENSOR_N_LOG("start");
    g_time_stamp_acc = time_stamp_acc;
    SENSOR_N_LOG("end %d",g_time_stamp_acc);
}

void acc_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data )
{

    SENSOR_N_LOG("start");

    acc_batch_data = batch_data;

    sensor_report_batch( acc_input_info.dev,
                         repo_type,
                         acc_batch_data );

    SENSOR_N_LOG("end");
    return;
}

void acc_timestamp_report(void)
{
    SENSOR_N_LOG("start");

    sensor_report_batch( acc_input_info.dev,
                         SENSOR_COMP_TIME,
                         acc_batch_data );

    SENSOR_N_LOG("end");
    return;
}

static ssize_t acc_flush_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    uint32_t ret = 0;

    SENSOR_N_LOG("start");

    ret = sensor_set_flush(SENSOR_ACC, acc_input_info.dev);

    SENSOR_N_LOG("end - return[%d]",count);
    return count;
}

static ssize_t acc_ope_device_show(struct device *dev,
    struct device_attribute *attr,
     char *buf)
{
    return sprintf(buf, "not supported\n" );
}

static ssize_t acc_ope_device_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
   size_t count )
{
    int32_t ret = 0;
    uint32_t ope = 0;

    sscanf(buf, "%d", &ope);

    switch(ope)
    {
    case KC_SENSOR_COMMON_INIT:
        sensor_com_mutex_lock();
        ret = sns_initialize();
        ret |= sns_set_dev_param();
        sensor_com_mutex_unlock();
        break;
    case KC_SENSOR_COMMON_IMITATION_ON:
        g_bIsAlreadyExistAccImitationXYZData       = true;
        break;
    case KC_SENSOR_COMMON_IMITATION_OFF:
        g_bIsAlreadyExistAccImitationXYZData       = false;
        break;
    default:
        break;
    }
    return count;
}

static ssize_t accsns_cal_th_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret = 0;

    SENSOR_N_LOG("start");
    ret = sprintf(buf, "%d\n", acc_calib_ctrl.m_nHighTH);

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t accsns_cal_th_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    int32_t ret = 0;
    unsigned long arg_iCal_th;

    SENSOR_N_LOG("start");

    ret = kstrtoul(buf, 10, &arg_iCal_th);

    if (ret >= 0) {
        acc_calib_ctrl.m_nHighTH = arg_iCal_th;
        acc_calib_ctrl.m_nLowTH  = -(arg_iCal_th);
    }

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t accsns_cal_smp_n_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret = 0;
    SENSOR_N_LOG("start");

    ret = sprintf(buf, "%d\n", (acc_calib_ctrl.m_unSmpN));

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t accsns_cal_smp_n_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    int32_t ret = 0;
    unsigned long arg_iCal_smp_n;

    SENSOR_N_LOG("start");
    ret = kstrtoul(buf, 10, &arg_iCal_smp_n);
    if (ret < 0) {
        return count;
    }

    acc_calib_ctrl.m_unSmpN = arg_iCal_smp_n;

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t acc_cal_wait_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int32_t wait;
    ssize_t ret =0;

    SENSOR_N_LOG("start");

    wait = acc_calib_ctrl.m_bWaitSetting;
    if(wait == true && acc_calib_ctrl.m_nMode == MODE_4){
        wait = 2;
    }
    if(acc_calib_ctrl.m_bRengeChkErr == true){
        wait = 3;
    }

    ret = sprintf(buf, "%d\n", wait);

    SENSOR_N_LOG("end");
    return ret;
}

static int32_t accsns_mov_acc_avg ( 
    acc_cal_movfilterwork* pCtrl,
    int32_t sample,
    int32_t axis )
{
    int32_t ret =0;

    SENSOR_N_LOG("start");
    if(pCtrl->m_pSamplWork[axis] == NULL) {
        pCtrl->m_pSamplWork[axis] = kzalloc( sizeof(int32_t)
                                             * pCtrl->m_ucAveN,
                                            GFP_KERNEL );
        
        if(pCtrl->m_pSamplWork[axis] == NULL) {
            return -1;
        }
        memset( pCtrl->m_pSamplWork[axis],
                0x00,
                sizeof(int32_t) * pCtrl->m_ucAveN );
    }

    if(pCtrl->m_unCnt[axis]++ < pCtrl->m_ucAveN) {
        pCtrl->m_pSamplWork[axis][ (pCtrl->m_unCnt[axis]-1)
                                    % pCtrl->m_ucAveN  ]
            = sample;
        pCtrl->m_unSum[axis] += sample;
        return -1;

    } else {
        pCtrl->m_unSum[axis]
            -= pCtrl->m_pSamplWork[axis][ (pCtrl->m_unCnt[axis]-1)
                                          % pCtrl->m_ucAveN ];
        pCtrl->m_unSum[axis] += sample;
        pCtrl->m_pSamplWork[axis][ (pCtrl->m_unCnt[axis]-1)
                                    % pCtrl->m_ucAveN ]
            = sample;
    }


    ret = pCtrl->m_unSum[axis] / pCtrl->m_ucAveN;
    SENSOR_N_LOG("end");

    return ret;
}

static void accsns_calibration_range_chk(
    const struct acceleration* accData )
{
    int32_t ret = 0;

    SENSOR_N_LOG("start");

    switch (acc_calib_ctrl.m_nMode) {
        case MODE_0:
        case MODE_1:
        case MODE_3:
            if( acc_calib_ctrl.m_nLowTH > accData->nX
                || acc_calib_ctrl.m_nHighTH < accData->nX
                || acc_calib_ctrl.m_nLowTH > accData->nY
                || acc_calib_ctrl.m_nHighTH < accData->nY
                || acc_calib_ctrl.m_nLowTH > (accData->nZ - WEIGHT_1G)
                || acc_calib_ctrl.m_nHighTH < (accData->nZ - WEIGHT_1G)){
                ret = -1;
            }
        break;
        case MODE_2:
        case MODE_4:
            if( acc_calib_ctrl.m_nLowTH > accData->nX
                || acc_calib_ctrl.m_nHighTH < accData->nX
                || acc_calib_ctrl.m_nLowTH > accData->nY
                || acc_calib_ctrl.m_nHighTH < accData->nY
                || acc_calib_ctrl.m_nLowTH > (accData->nZ + WEIGHT_1G)
                || acc_calib_ctrl.m_nHighTH < (accData->nZ + WEIGHT_1G)){
                ret = -1;
            }
            break;
        default:
            break;
    }

    if(ret < 0){
        acc_calib_ctrl.m_bRengeChkErr = true;
    }

    SENSOR_N_LOG("end");

    return;
}

static void accsns_calibration_periodic(
    const struct acceleration* accData )
{
    int32_t ret = 0;

    SENSOR_N_LOG("start");

    if(acc_calib_ctrl.m_nCurrentSampleNum == acc_calib_ctrl.m_unSmpN) {
        acc_calib_ctrl.m_bComplete    = true;
        acc_calib_ctrl.m_bWaitSetting = true;

    } else {
        accsns_calibration_range_chk(accData);

        if(acc_calib_ctrl.m_bFilterEnable == true) {
            ret = accsns_mov_acc_avg( &(acc_calib_ctrl.m_tFilterWork),
                                      accData->nX,
                                      AXIS_X );
            if(ret != -1) {
                acc_calib_ctrl.m_nCurrentSampleNum++;
                acc_calib_ctrl.m_nSummationX += ret;
            }
            ret = accsns_mov_acc_avg( &(acc_calib_ctrl.m_tFilterWork),
                                      accData->nY,
                                      AXIS_Y );
            if(ret !=  -1) {
                acc_calib_ctrl.m_nSummationY += ret;
            }
            ret = accsns_mov_acc_avg( &(acc_calib_ctrl.m_tFilterWork),
                                      accData->nZ,
                                      AXIS_Z );
            if(ret !=  -1) {
                acc_calib_ctrl.m_nSummationZ += ret;
            }
            
        } else {
            acc_calib_ctrl.m_nSummationX += accData->nX;
            acc_calib_ctrl.m_nSummationY += accData->nY;
            acc_calib_ctrl.m_nSummationZ += accData->nZ;
            acc_calib_ctrl.m_nCurrentSampleNum++;
        }

        if(acc_calib_ctrl.m_nCurrentSampleNum == acc_calib_ctrl.m_unSmpN) {
            switch (acc_calib_ctrl.m_nMode) {
                case MODE_0:
                    acc_calib_ctrl.m_nCalX = ( acc_calib_ctrl.m_nSummationX
                                               / acc_calib_ctrl.m_unSmpN )
                                             - SETTING_0G;
                    acc_calib_ctrl.m_nCalY = ( acc_calib_ctrl.m_nSummationY
                                               / acc_calib_ctrl.m_unSmpN )
                                             - SETTING_0G;
                    acc_calib_ctrl.m_nCalZ = 0;
                    break;

                case MODE_1:
                    acc_calib_ctrl.m_nCalX = ( acc_calib_ctrl.m_nSummationX
                                               / acc_calib_ctrl.m_unSmpN )
                                              - SETTING_0G;
                    acc_calib_ctrl.m_nCalY = ( acc_calib_ctrl.m_nSummationY
                                              / acc_calib_ctrl.m_unSmpN )
                                              - SETTING_0G;
                    acc_calib_ctrl.m_nCalZ = ( acc_calib_ctrl.m_nSummationZ
                                               / acc_calib_ctrl.m_unSmpN)
                                              - WEIGHT_1G;
                    break;

                case MODE_2:
                    acc_calib_ctrl.m_nCalX = ( acc_calib_ctrl.m_nSummationX
                                               / acc_calib_ctrl.m_unSmpN )
                                             - SETTING_0G;
                    acc_calib_ctrl.m_nCalY = ( acc_calib_ctrl.m_nSummationY
                                               / acc_calib_ctrl.m_unSmpN )
                                             - SETTING_0G;
                    acc_calib_ctrl.m_nCalZ = ( acc_calib_ctrl.m_nSummationZ
                                               / acc_calib_ctrl.m_unSmpN )
                                             + WEIGHT_1G;
                    break;

                case MODE_3:
                    acc_calib_ctrl.m_nCalX = ( acc_calib_ctrl.m_nSummationX
                                               / acc_calib_ctrl.m_unSmpN );
                    acc_calib_ctrl.m_nCalY = ( acc_calib_ctrl.m_nSummationY
                                              / acc_calib_ctrl.m_unSmpN );
                    acc_calib_ctrl.m_nCalZ = ( acc_calib_ctrl.m_nSummationZ
                                               / acc_calib_ctrl.m_unSmpN );
                    break;

                case MODE_4:
                    acc_calib_ctrl.m_nCalX = ( acc_calib_ctrl.m_nCalX
                                               + ( acc_calib_ctrl.m_nSummationX
                                                   / acc_calib_ctrl.m_unSmpN ) )
                                             / 2;
                    acc_calib_ctrl.m_nCalY = ( acc_calib_ctrl.m_nCalY
                                               + ( acc_calib_ctrl.m_nSummationY
                                                   / acc_calib_ctrl.m_unSmpN ) )
                                             / 2;
                    acc_calib_ctrl.m_nCalZ = ( acc_calib_ctrl.m_nCalZ
                                               + ( acc_calib_ctrl.m_nSummationZ
                                                   / acc_calib_ctrl.m_unSmpN ) )
                                             / 2; 
                    break;

                default:
                    ret = -1;
                    break;
            }

            if(ret != -1 && acc_calib_ctrl.m_bRengeChkErr == false) {
                atomic_set(&g_nCalX, acc_calib_ctrl.m_nCalX);
                atomic_set(&g_nCalY, acc_calib_ctrl.m_nCalY);
                atomic_set(&g_nCalZ, acc_calib_ctrl.m_nCalZ);
                input_report_abs( (acc_input_info.dev),
                                   ABS_RX,
                                   acc_calib_ctrl.m_nCalX );
                input_report_abs( (acc_input_info.dev),
                                   ABS_RY,
                                   acc_calib_ctrl.m_nCalY );
                input_report_abs( (acc_input_info.dev),
                                   ABS_RZ,
                                   acc_calib_ctrl.m_nCalZ );
            }
            
            if(acc_calib_ctrl.m_tFilterWork.m_pSamplWork[AXIS_X] != NULL){
                kfree(acc_calib_ctrl.m_tFilterWork.m_pSamplWork[AXIS_X]);
            }
            if(acc_calib_ctrl.m_tFilterWork.m_pSamplWork[AXIS_Y] != NULL){
                kfree(acc_calib_ctrl.m_tFilterWork.m_pSamplWork[AXIS_Y]);
            }
            if(acc_calib_ctrl.m_tFilterWork.m_pSamplWork[AXIS_Z] != NULL){
                kfree(acc_calib_ctrl.m_tFilterWork.m_pSamplWork[AXIS_Z]);
            }
            acc_calib_ctrl.m_tFilterWork.m_pSamplWork[AXIS_X] = NULL;
            acc_calib_ctrl.m_tFilterWork.m_pSamplWork[AXIS_Y] = NULL;
            acc_calib_ctrl.m_tFilterWork.m_pSamplWork[AXIS_Z] = NULL;
        }
    }

    SENSOR_N_LOG("end");
    return;
}

static void acc_calib_work_func(struct work_struct *work)
{
    int32_t ret = 0;
    int32_t delay = 0;
    union sensor_read_data_u read_data;
    bool bCalibIdle = false;
    bool bCalibComp = false;
    int32_t offset[3];

    SENSOR_N_LOG("start");

    ret = sensor_type_get_data( SENSOR_ACC, &read_data);
    if( 0 != ret) {
        return;
    }
    memcpy(&acc_calib_read_data,&(read_data.acc_data),sizeof(read_data.acc_data));

    bCalibIdle = acc_calib_ctrl.m_bWaitSetting;
    bCalibComp = acc_calib_ctrl.m_bComplete;

    if((bCalibIdle == false) && (bCalibComp == false)){
        accsns_calibration_periodic(&acc_calib_read_data);

        bCalibComp = acc_calib_ctrl.m_bComplete;
        if(bCalibComp == false){
            delay = atomic_read(&(acc_poll_info.poll_time));
            queue_delayed_work( (acc_calib_wq_info.poll_wq),
                                &(acc_calib_wq_info.poll_work),
                                msecs_to_jiffies(delay) );
        }
        else {
            offset[0] = atomic_read(&g_nCalX);
            offset[1] = atomic_read(&g_nCalY);
            offset[2] = atomic_read(&g_nCalZ);
            sensor_com_mutex_lock();
            ret = sns_acc_set_offset(offset);
            sensor_com_mutex_unlock();
        }
    }

    SENSOR_N_LOG("end");

    return;
}

static ssize_t acc_start_calib_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    uint32_t mode;
    int32_t delay = 0;

    SENSOR_N_LOG("start");

    sscanf(buf, "%d", &mode);

    acc_calib_ctrl.m_bWaitSetting = false;
    acc_calib_ctrl.m_nMode = mode;

    if(acc_calib_ctrl.m_nMode == MODE_4){
        acc_calib_ctrl.m_bComplete  = false;
    }

    acc_calib_ctrl.m_nCurrentSampleNum    = 0;
    acc_calib_ctrl.m_nSummationX          = 0;
    acc_calib_ctrl.m_nSummationY          = 0;
    acc_calib_ctrl.m_nSummationZ          = 0;

    memset(acc_calib_ctrl.m_tFilterWork.m_unSum, 0x00, sizeof(int32_t) * AXIS_XYZ_MAX);
    memset(acc_calib_ctrl.m_tFilterWork.m_unCnt, 0x00, sizeof(int32_t) * AXIS_XYZ_MAX);

    delay = atomic_read(&(acc_poll_info.poll_time));
    queue_delayed_work( (acc_calib_wq_info.poll_wq),
                        &(acc_calib_wq_info.poll_work),
                        msecs_to_jiffies(delay) );

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t acc_calib_mode_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    int32_t i = 0;

    acc_calib_ctrl.m_bFilterEnable        = false;
    acc_calib_ctrl.m_bWaitSetting         = true;
    acc_calib_ctrl.m_bComplete            = false;
    acc_calib_ctrl.m_bRengeChkErr         = false;

    acc_calib_ctrl.m_tFilterWork.m_ucAveN = OFFSET_AVE_NUM;
    acc_calib_ctrl.m_nCalX                = 0;
    acc_calib_ctrl.m_nCalY                = 0;
    acc_calib_ctrl.m_nCalZ                = 0;

    acc_calib_ctrl.m_nCurrentSampleNum    = 0;
    acc_calib_ctrl.m_nSummationY          = 0;
    acc_calib_ctrl.m_nSummationZ          = 0;
    acc_calib_ctrl.m_nMode                = -1;

    for (i = 0; i < AXIS_XYZ_MAX; i++) {
        if(acc_calib_ctrl.m_tFilterWork.m_pSamplWork[i] != NULL) {
        } else {
            acc_calib_ctrl.m_tFilterWork.m_pSamplWork[i] = NULL;
        }
    }

    memset(acc_calib_ctrl.m_tFilterWork.m_unSum, 0x00, sizeof(int32_t) * AXIS_XYZ_MAX);
    memset(acc_calib_ctrl.m_tFilterWork.m_unCnt, 0x00, sizeof(int32_t) * AXIS_XYZ_MAX);

    return count;
}

static ssize_t acc_host_cmd_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t ret = 0;
    uint32_t res[128];

    sensor_com_mutex_lock();
    sns_acc_get_host_cmd(res);
    sensor_com_mutex_unlock();

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

static ssize_t acc_host_cmd_store(struct device *dev,
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

    sensor_com_mutex_lock();
    ret = sns_acc_set_host_cmd(req_cmd, req_param);
    sensor_com_mutex_unlock();

    return count;
}



static ssize_t acc_data_raw_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t ret = 0;
    int32_t raw[3];
    int32_t offsets[3];
    struct acceleration auto_cal_offset;

    SENSOR_N_LOG("start");

    sensor_com_mutex_lock();
    sns_acc_get_offset(offsets);
    sns_acc_get_auto_cal_offset(&auto_cal_offset);
    sensor_com_mutex_unlock();
    acc_update_last_read_data();

    offsets[0] *= (U2DH_GRAVITY_EARTH / U2DH_RESOLUTION);
    offsets[1] *= (U2DH_GRAVITY_EARTH / U2DH_RESOLUTION);
    offsets[2] *= (U2DH_GRAVITY_EARTH / U2DH_RESOLUTION);
    auto_cal_offset.nX *= (U2DH_GRAVITY_EARTH / U2DH_RESOLUTION);
    auto_cal_offset.nY *= (U2DH_GRAVITY_EARTH / U2DH_RESOLUTION);
    auto_cal_offset.nZ *= (U2DH_GRAVITY_EARTH / U2DH_RESOLUTION);

    raw[0] = acc_last_read_data.outX + offsets[0] + auto_cal_offset.nX;
    raw[1] = acc_last_read_data.outY + offsets[1] + auto_cal_offset.nY;
    raw[2] = acc_last_read_data.outZ + offsets[2] + auto_cal_offset.nZ;

    SENSOR_N_LOG("x_raw[%d], y_raw[%d], z_raw[%d]", raw[0], raw[1], raw[2]);
    SENSOR_N_LOG("x[%d], y[%d], z[%d]", acc_last_read_data.outX, acc_last_read_data.outY, acc_last_read_data.outZ);
    SENSOR_N_LOG("x_ofs[%d], y_ofs[%d], z_ofs[%d]", offsets[0], offsets[1], offsets[2]);
    SENSOR_N_LOG("x_aofs[%d], y_aofs[%d], z_aofs[%d]", auto_cal_offset.nX, auto_cal_offset.nY, auto_cal_offset.nZ);
    ret = sprintf( buf, "%d %d %d\n", raw[0], raw[1], raw[2] );

    SENSOR_N_LOG("end");

    return ret;
}

static ssize_t acc_offset_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret = 0;
    int32_t offsets[3];

    SENSOR_N_LOG("start");

    sensor_com_mutex_lock();
    ret = sns_acc_get_offset(offsets);
    sensor_com_mutex_unlock();

    if(ret == SNS_RC_OK) {
        ret = sprintf( buf, "%d %d %d\n",
                       offsets[0],
                       offsets[1],
                       offsets[2] );
    }

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t acc_offset_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    int32_t offset[3];

    SENSOR_N_LOG("start");
    sscanf(buf, "%d %d %d", &offset[0], &offset[1], &offset[2]);

    SENSOR_N_LOG("offset = %d %d %d",offset[0], offset[1], offset[2]);

    sensor_com_mutex_lock();
    sns_acc_set_offset(offset);
    sensor_com_mutex_unlock();

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t acc_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int enable = 0;
    SENSOR_N_LOG("start");

    enable = sensor_get_status(SENSOR_ACC);

    SENSOR_N_LOG("end ->enable[%d]",enable);
    return sprintf(buf, "%d\n", enable);
}

static ssize_t acc_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    unsigned long enable = 0;
    int ret = 0;

    SENSOR_N_LOG("start");

    ret = kstrtoul(buf, 10, &enable);
    SENSOR_N_LOG("kstrtoul() ret[%d]->enable[%d]",ret, (int)enable);

    sensor_enable( SENSOR_ACC, &acc_poll_info, (bool)enable );

    SENSOR_N_LOG("end - return[%d]",count);
    return count;
}

static ssize_t acc_delay_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int32_t delay = 0;
    SENSOR_N_LOG("start");
    delay = atomic_read(&(acc_poll_info.poll_time));
    SENSOR_N_LOG("end ->delay[%d]",(int)delay);
    return sprintf(buf, "%d\n", delay);
}

static ssize_t acc_delay_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    unsigned long delay = 0;
    int ret = 0;
    SENSOR_N_LOG("start");

    ret = kstrtoul(buf, 10, &delay);
    SENSOR_N_LOG("kstrtoul() ret[%d]->delay[%d]",ret, (int)delay);

    sensor_set_poll_time( SENSOR_ACC, &acc_poll_info, (int32_t)delay);

    SENSOR_N_LOG("end - return[%d]",count);
    return count;
}

static ssize_t acc_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    struct acceleration last_data;
    SENSOR_N_LOG("start");
    acc_update_last_read_data();
    last_data = acc_last_read_data;
    SENSOR_N_LOG("end - last_data[%d,%d,%d]",
                  last_data.outX,
                  last_data.outY,
                  last_data.outZ );

    return sprintf( buf, "%d %d %d\n",
                    last_data.outX,
                    last_data.outY,
                    last_data.outZ);
}

static ssize_t acc_auto_cal_offset_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    struct acceleration auto_cal_offset = {0};
    int cnt;

    SENSOR_N_LOG("start");
    cnt = sscanf(buf,"%d %d %d",
                &auto_cal_offset.nX,
                &auto_cal_offset.nY,
                &auto_cal_offset.nZ);
    if (cnt != 3) {
        SENSOR_ERR_LOG("sscanf failed cnt=%d", cnt);
        return count;
    }
    SENSOR_N_LOG("acc_auto_cal_offset : X:%d Y:%d Z:%d",
            auto_cal_offset.nX,
            auto_cal_offset.nY,
            auto_cal_offset.nZ);
    sensor_com_mutex_lock();
    sns_acc_set_auto_cal_offset(&auto_cal_offset);
    sensor_com_mutex_unlock();
    SENSOR_N_LOG("end");

    return count;
}

static ssize_t acc_auto_cal_offset_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret =0;
    struct acceleration auto_cal_offset = {0};

    SENSOR_N_LOG("start");
    sensor_com_mutex_lock();
    sns_acc_get_auto_cal_offset(&auto_cal_offset);
    sensor_com_mutex_unlock();
    ret = sprintf(buf,"%d %d %d\n",
                auto_cal_offset.nX,
                auto_cal_offset.nY,
                auto_cal_offset.nZ);
    SENSOR_N_LOG("buf");
    SENSOR_N_LOG("end");
    return ret;
}

static uint8_t acc_reg_rw_rslt[65] = {0};

static ssize_t acc_reg_rw_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t ret = 0;
    uint8_t *res = acc_reg_rw_rslt;
    SENSOR_N_LOG("start");

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

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t acc_reg_rw_store(struct device *dev,
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
    SENSOR_N_LOG("start");

    memset(acc_reg_rw_rslt, 0x00, sizeof(acc_reg_rw_rslt));
    acc_reg_rw_rslt[0] = 0xFF;

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
        SENSOR_ERR_LOG("bad parm addr[0x%02x] len[0x%02x]", addr, len);
        return count;
    }

    if (addr & 0x80) {
        sensor_com_mutex_lock();
        ret  = sns_device_read(addr & 0x7F, &acc_reg_rw_rslt[0x01], len);
        sensor_com_mutex_unlock();
        if (ret == SNS_RC_OK) {
            SENSOR_N_LOG("sns_device_read OK addr=0x%x len=0x%x", addr & 0x7F, len);
            acc_reg_rw_rslt[0] = 0x00;
        } else {
            SENSOR_ERR_LOG("sns_device_read NG");
        }
    } else {
        for (i = 0; i < len; i++)
            data_uint8[i] = (uint8_t)data[i];

        sensor_com_mutex_lock();
        ret = sns_device_write(addr, data_uint8, len);
        sensor_com_mutex_unlock();
        if (ret == SNS_RC_OK) {
            SENSOR_N_LOG("sns_device_write OK");
            acc_reg_rw_rslt[0] = 0x00;
        } else {
            SENSOR_ERR_LOG("sns_device_write NG");
        }
    }

    SENSOR_N_LOG("end");
    return count;
}

void acc_auto_cal_interrupt()
{
    SENSOR_N_LOG("start");
    input_report_abs((acc_input_info.dev), ABS_THROTTLE, ABS_THROTTLE_DUMMY_VAL);
    input_report_abs((acc_input_info.dev), ABS_THROTTLE, 1);
    input_sync((acc_input_info.dev));
    SENSOR_N_LOG("end");
}

static void acc_update_last_read_data(void)
{
    unsigned long enable = 0;
    int32_t ret = 0;
    union sensor_read_data_u read_data;

    enable = sensor_get_status(SENSOR_ACC);

    if (enable) {
        ret = sensor_type_get_data(SENSOR_ACC, &read_data);
        if (0 == ret) {
            memcpy(&acc_last_read_data,&(read_data.acc_data),sizeof(read_data.acc_data));
        }
    }
}

static void acc_poll_work_func(struct work_struct *work)
{
    int32_t polltime = 0;

    SENSOR_N_LOG("start");

    if(sns_get_reset_status() == false){
        sns_iio_report_event_now(SENSOR_ACC);
    }

    polltime = atomic_read(&(acc_poll_info.poll_time));
    if (polltime > 0) {
        queue_delayed_work( (acc_poll_info.poll_wq),
                            &(acc_poll_info.poll_work),
                            msecs_to_jiffies(polltime) );
        SENSOR_A_LOG("start delay work :polltime[%d]",
                     atomic_read(&(acc_poll_info.poll_time)) );
    } else {
        SENSOR_ERR_LOG("fail polltime[%d]",(int)polltime);
    }
    SENSOR_N_LOG("end");
    return;
}

static void acc_set_input_params( struct input_dev *dev )
{
    SENSOR_N_LOG("start");

    if(!dev){
        SENSOR_ERR_LOG("bad parm --> dev is NULL");
        return;
    }

    dev->name = "accelerometer";
    dev->id.bustype = BUS_SPI;

    input_set_capability(dev, EV_ABS, ABS_MISC);
    input_set_capability(dev, EV_ABS, ABS_RUDDER);
    input_set_capability(dev, EV_ABS, ABS_THROTTLE);
    input_set_abs_params(dev, ABS_X, ABSMIN_4G, ABSMAX_4G, 0, 0);
    input_set_abs_params(dev, ABS_Y, ABSMIN_4G, ABSMAX_4G, 0, 0);
    input_set_abs_params(dev, ABS_Z, ABSMIN_4G, ABSMAX_4G, 0, 0);
    input_set_abs_params(dev, ABS_RX, ABSMIN_4G, ABSMAX_4G, 0, 0);
    input_set_abs_params(dev, ABS_RY, ABSMIN_4G, ABSMAX_4G, 0, 0);
    input_set_abs_params(dev, ABS_RZ, ABSMIN_4G, ABSMAX_4G, 0, 0);
    input_set_abs_params(dev, ABS_MISC, INT_MIN,INT_MAX, 0, 0);
    input_set_abs_params(dev, ABS_THROTTLE, INT_MIN, INT_MAX, 0, 0);

    SENSOR_N_LOG("end");
    return;
}

void acc_driver_init( void )
{
    int ret = 0;
    int32_t i;

    SENSOR_N_LOG("start");

    ret = sensor_input_init( &acc_input_info );
    SENSOR_N_LOG("sensor_input_init()-->ret[%d] dev[%d]",
                  ret, (int)(acc_input_info.dev) );

    if( (0 != ret) || (NULL == (acc_input_info.dev))) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        SENSOR_ERR_LOG("end return[%d]",-ENODEV);
        return;
    }

    sensor_poll_init(SENSOR_ACC, &acc_poll_info);
    sensor_poll_init(SENSOR_COM,&acc_calib_wq_info);

    acc_calib_ctrl.m_bFilterEnable        = false;
    acc_calib_ctrl.m_bWaitSetting         = true;
    acc_calib_ctrl.m_bComplete            = true;
    acc_calib_ctrl.m_bRengeChkErr         = false;
    acc_calib_ctrl.m_unSmpN               = OFFSET_SUMPLE_NUM;

    acc_calib_ctrl.m_tFilterWork.m_ucAveN = OFFSET_AVE_NUM;
    acc_calib_ctrl.m_nCalX                = 0;
    acc_calib_ctrl.m_nCalY                = 0;
    acc_calib_ctrl.m_nCalZ                = 0;

    acc_calib_ctrl.m_nCurrentSampleNum    = 0;
    acc_calib_ctrl.m_nSummationX          = 0;
    acc_calib_ctrl.m_nSummationY          = 0;
    acc_calib_ctrl.m_nSummationZ          = 0;
    acc_calib_ctrl.m_nHighTH              = OFFSET_ACC_HIGH_TH;
    acc_calib_ctrl.m_nLowTH               = OFFSET_ACC_LOW_TH;

    acc_calib_ctrl.m_nMode                = -1; 

    for (i = 0; i < AXIS_XYZ_MAX; i++) {
        if(acc_calib_ctrl.m_tFilterWork.m_pSamplWork[i] != NULL) {
            SENSOR_ERR_LOG("Calib err occurd");
        } else {
            acc_calib_ctrl.m_tFilterWork.m_pSamplWork[i] = NULL;
        }
    }
    memset(acc_calib_ctrl.m_tFilterWork.m_unSum, 0x00, sizeof(int32_t) * AXIS_XYZ_MAX);
    memset(acc_calib_ctrl.m_tFilterWork.m_unCnt, 0x00, sizeof(int32_t) * AXIS_XYZ_MAX);

    SENSOR_N_LOG("end");
    return;
}

EXPORT_SYMBOL(acc_driver_init);

