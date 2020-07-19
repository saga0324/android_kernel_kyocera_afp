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

#define SELFCHK_DATANUM         (5)
#define SELFCHK_NORMAL_V_LLIMIT     (819)
#define SELFCHK_NORMAL_V_ULIMIT     (1229)
#define SELFCHK_DIFF_SELFP_LLIMIT   (90)
#define SELFCHK_DIFF_SELFP_ULIMIT   (1700)
#define SELFCHK_DIFF_SELFM_LLIMIT   (90)
#define SELFCHK_DIFF_SELFM_ULIMIT   (1700)
#define MSEC_2_USEC                 (1000)

enum ACC_SELFCHK_CONDITION
{
    NORMAL_V = 0,
    NORMAL_X_SELFP_X,
    NORMAL_Y_SELFP_Y,
    NORMAL_Z_SELFP_Z,
    NORMAL_X_SELFM_X,
    NORMAL_Y_SELFM_Y,
    NORMAL_Z_SELFM_Z,
    SELFCHK_CONDITION_MAX
};

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
    AXIS_XYZ_MAX,
    AXIS_V = 3,
    AXIS_XYZV_MAX,
} ACC_AXIS;

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
static ssize_t acc_device_id_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t acc_selfcheck_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t acc_nrfilt_prm_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t acc_nrfilt_prm_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);

static struct sens_ofs saved_ofs_val;

static void acc_update_last_read_data(void);
static void acc_poll_work_func(struct work_struct *work);
static void acc_set_input_params( struct input_dev *dev );
static int32_t acc_selfchk_advance_preparation(void);


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
static DEVICE_ATTR(device_id,
    S_IRUSR|S_IRGRP,
    acc_device_id_show,
    NULL
);
static DEVICE_ATTR(selfcheck,
    S_IRUSR|S_IRGRP,
    acc_selfcheck_show,
    NULL
);
static DEVICE_ATTR(nrfilt_prm,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    acc_nrfilt_prm_show,
    acc_nrfilt_prm_store
);

static struct attribute *acc_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_delay.attr,
    &dev_attr_data.attr,
    &dev_attr_offset.attr,
    &dev_attr_data_raw.attr,
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
    &dev_attr_device_id.attr,
    &dev_attr_selfcheck.attr,
    &dev_attr_nrfilt_prm.attr,
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

    SENSOR_N_LOG("end - return[%d]",(int)count);
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

    SENSOR_N_LOG("end - return[%d]",(int)count);
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
        SENSOR_ERR_LOG("invalid ope:%d",ope);
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
        SENSOR_ERR_LOG("kstrtoul failed ret=%d", ret);
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
            SENSOR_ERR_LOG("kzalloc failed axis:%d",axis);
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
            SENSOR_ERR_LOG("invalid m_nMode:%d",acc_calib_ctrl.m_nMode);
            break;
    }

    if(ret < 0){
        acc_calib_ctrl.m_bRengeChkErr = true;
        SENSOR_ERR_LOG("m_bRengeChkErr:%d ret:%d mode:%d", acc_calib_ctrl.m_bRengeChkErr, ret, acc_calib_ctrl.m_nMode);
        SENSOR_ERR_LOG("m_nLowTH:%d m_nHighTH:%d",acc_calib_ctrl.m_nLowTH, acc_calib_ctrl.m_nHighTH);
        SENSOR_ERR_LOG("Acc Data nX:%d nY:%d nZ:%d",accData->nX, accData->nY, accData->nZ);
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
                    SENSOR_ERR_LOG("Invalid mode:%d",acc_calib_ctrl.m_nMode);
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
    union sensor_read_data_u read_data;
    bool bCalibIdle = false;
    bool bCalibComp = false;
    int32_t offset[3];

    SENSOR_N_LOG("start");

    ret = sensor_type_get_data( SENSOR_ACC, &read_data);
    if( 0 != ret) {
        SENSOR_ERR_LOG("sensor_type_get_data err ret:%d",ret);
        return;
    }
    memcpy(&acc_calib_read_data,&(read_data.acc_data),sizeof(read_data.acc_data));

    bCalibIdle = acc_calib_ctrl.m_bWaitSetting;
    bCalibComp = acc_calib_ctrl.m_bComplete;

    if((bCalibIdle == false) && (bCalibComp == false)){
        accsns_calibration_periodic(&acc_calib_read_data);

        bCalibComp = acc_calib_ctrl.m_bComplete;
        if(bCalibComp == true){
            sensor_stoptimer(&(acc_calib_wq_info.timer));
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
    sensor_starttimer(&(acc_calib_wq_info.timer), delay);

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

    SENSOR_D_LOG("start");
    sscanf(buf, "%d %d %d", &offset[0], &offset[1], &offset[2]);

    SENSOR_D_LOG("offset = %d %d %d",offset[0], offset[1], offset[2]);

    sensor_com_mutex_lock();
    sns_acc_set_offset(offset);
    sensor_com_mutex_unlock();

    SENSOR_D_LOG("end");
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

    SENSOR_N_LOG("end - return[%d]",(int)count);
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

    SENSOR_N_LOG("end - return[%d]",(int)count);
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

    SENSOR_D_LOG("start");
    cnt = sscanf(buf,"%d %d %d",
                &auto_cal_offset.nX,
                &auto_cal_offset.nY,
                &auto_cal_offset.nZ);
    if (cnt != 3) {
        SENSOR_ERR_LOG("sscanf failed cnt=%d", cnt);
        return count;
    }
    SENSOR_D_LOG("acc_auto_cal_offset : X:%d Y:%d Z:%d",
            auto_cal_offset.nX,
            auto_cal_offset.nY,
            auto_cal_offset.nZ);
    auto_cal_offset.outX = 0;
    auto_cal_offset.outY = 0;
    auto_cal_offset.outZ = 0;
    acc_save_cal_ofs_val(&auto_cal_offset);
    sensor_com_mutex_lock();
    sensor_set_offset_value(SENSOR_ACC, saved_ofs_val);
    sensor_com_mutex_unlock();
    SENSOR_D_LOG("end");

    return count;
}

static ssize_t acc_auto_cal_offset_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret =0;
    struct acceleration* auto_cal_offset;

    SENSOR_N_LOG("start");
    auto_cal_offset = (struct acceleration*)kzalloc(sizeof(struct acceleration), GFP_KERNEL);
    sensor_com_mutex_lock();
    sensor_get_offset_value(SENSOR_ACC, auto_cal_offset);
    sensor_com_mutex_unlock();
    acc_save_cal_ofs_val(auto_cal_offset);
    ret = sprintf(buf,"%d %d %d\n",
                auto_cal_offset->nX,
                auto_cal_offset->nY,
                auto_cal_offset->nZ);
    kfree(auto_cal_offset);
    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t acc_nrfilt_prm_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    struct nr_filt_prm param;
    SENSOR_D_LOG("start");
    sscanf(buf, "%d %d %d %d %d %d %d %d %d",
                &param.th_l,
                &param.th_h,
                &param.coff,
                &param.rel_th,
                &param.base_filt_coff,
                &param.base_filt_acoff,
                &param.tolerance,
                &param.base_filt_hys,
                &param.filt_lmt_th);
    SENSOR_D_LOG("acc_th_l:%d, acc_th_h:%d, acc_coff:%d, acc_rel_th:%d, acc_base_filt_coff:%d, acc_base_filt_acoff:%d, acc_tolerance:%d, acc_base_filt_hys:%d, acc_filt_lmt_th:%d\n",
                (int)param.th_l, (int)param.th_h, (int)param.coff,
                (int)param.rel_th, (int)param.base_filt_coff, (int)param.base_filt_acoff,
                (int)param.tolerance, (int)param.base_filt_hys, (int)param.filt_lmt_th);
    sensor_set_nrfilt_param(NR_ACC, param);
    SENSOR_D_LOG("end");
    return count;
}

static ssize_t acc_nrfilt_prm_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t count = 0;
    struct nr_filt_prm param;
    SENSOR_D_LOG("start");
    sensor_get_nrfilt_param(NR_ACC, &param);
    count += scnprintf(buf + count, PAGE_SIZE - count,
                        "ACC_NR_FILT_TH_L[%d]\n", param.th_l);
    count += scnprintf(buf + count, PAGE_SIZE - count,
                        "ACC_NR_FILT_TH_H[%d]\n", param.th_h);
    count += scnprintf(buf + count, PAGE_SIZE - count,
                        "ACC_NR_FILT_COFF[%d]\n", param.coff);
    count += scnprintf(buf + count, PAGE_SIZE - count,
                        "ACC_NR_REL_TH[%d]\n", param.rel_th);
    count += scnprintf(buf + count, PAGE_SIZE - count,
                        "ACC_NR_BASE_FILT_COFF[%d]\n", param.base_filt_coff);
    count += scnprintf(buf + count, PAGE_SIZE - count,
                        "ACC_NR_BASE_FILT_ACOFF[%d]\n", param.base_filt_acoff);
    count += scnprintf(buf + count, PAGE_SIZE - count,
                        "ACC_NR_TOLERANCE[%d]\n", param.tolerance);
    count += scnprintf(buf + count, PAGE_SIZE - count,
                        "ACC_NR_BASE_FILT_HYS[%d]\n", param.base_filt_hys);
    count += scnprintf(buf + count, PAGE_SIZE - count,
                        "ACC_NR_FILT_LMT_TH[%d]\n", param.filt_lmt_th);
    SENSOR_D_LOG("end");
    return count;
}

static ssize_t acc_device_id_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret =0;
    uint8_t id;
    uint8_t restore_flg = false;

    SENSOR_D_LOG("start");
    if (SENSOR_ON == sensor_get_status(SENSOR_ACC)){
        sensor_enable( SENSOR_ACC, &acc_poll_info, false );
        restore_flg = true;
    }
    sensor_com_mutex_lock();
    id = sensor_get_device_id(SENSOR_ACC);
    sensor_com_mutex_unlock();
    ret = sprintf(buf,"0x%02x\n", id);
    if(restore_flg == true){
        sensor_enable( SENSOR_ACC, &acc_poll_info, true );
    }
    return ret;
}

static ssize_t acc_selfchk_make_output_alldata(
    int16_t normal_data[][AXIS_XYZV_MAX],
    int16_t self_p_data[][AXIS_XYZ_MAX],
    int16_t self_m_data[][AXIS_XYZ_MAX],
    char *buf
)
{
    int i;
    ssize_t count = 0;
    SENSOR_D_LOG("start");
    count +=  scnprintf(buf + count, PAGE_SIZE - count,
			"--- [Normal] ---\n");
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
    for(i = 0; i < SELFCHK_DATANUM; i++){
        count +=  scnprintf(buf + count, PAGE_SIZE - count,
                "X:%d Y:%d Z:%d V:%d\n",
                normal_data[i][AXIS_X], normal_data[i][AXIS_Y],
                normal_data[i][AXIS_Z], normal_data[i][AXIS_V]);
        if (count >= PAGE_SIZE) {
            SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
            goto fail_exit;
        }
    }

    count +=  scnprintf(buf + count, PAGE_SIZE - count,
			"--- [Self +] ---\n");
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
    for(i = 0; i < SELFCHK_DATANUM; i++){
        count +=  scnprintf(buf + count, PAGE_SIZE - count,
                "X:%d Y:%d Z:%d\n",
                self_p_data[i][AXIS_X],
                self_p_data[i][AXIS_Y],
                self_p_data[i][AXIS_Z]);
        if (count >= PAGE_SIZE) {
            SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
            goto fail_exit;
        }
    }

    count +=  scnprintf(buf + count, PAGE_SIZE - count,
			"--- [Self -] ---\n");
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
    for(i = 0; i < SELFCHK_DATANUM; i++){
        count +=  scnprintf(buf + count, PAGE_SIZE - count,
                "X:%d Y:%d Z:%d\n",
                self_m_data[i][AXIS_X],
                self_m_data[i][AXIS_Y],
                self_m_data[i][AXIS_Z]);
        if (count >= PAGE_SIZE) {
            SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
            goto fail_exit;
        }
    }

	SENSOR_D_LOG("end");
	return count;
fail_exit:
	SENSOR_D_LOG("end - fail");
	return PAGE_SIZE - 1;

}

static ssize_t acc_selfchk_make_output_result(
    bool *result,
    int16_t *normal_mean,
    int16_t *self_p_mean,
    int16_t *self_m_mean,
    char *buf
)
{
    ssize_t count = 0;
    SENSOR_D_LOG("start");
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"Normal  : %s [mean = %d]\n",
            (result[NORMAL_V] ? "OK":"NG"),
            normal_mean[AXIS_V]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"Self+ X : %s [mean = %d]\n",
            (result[NORMAL_X_SELFP_X] ? "OK":"NG"),
            self_p_mean[AXIS_X]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"Self+ Y : %s [mean = %d]\n",
            (result[NORMAL_Y_SELFP_Y] ? "OK":"NG"),
            self_p_mean[AXIS_Y]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"Self+ Z : %s [mean = %d]\n",
            (result[NORMAL_Z_SELFP_Z] ? "OK":"NG"),
            self_p_mean[AXIS_Z]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"Self- X : %s [mean = %d]\n",
            (result[NORMAL_X_SELFM_X] ? "OK":"NG"),
            self_m_mean[AXIS_X]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"Self- Y : %s [mean = %d]\n",
            (result[NORMAL_Y_SELFM_Y] ? "OK":"NG"),
            self_m_mean[AXIS_Y]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"Self- Z : %s [mean = %d]\n",
            (result[NORMAL_Z_SELFM_Z] ? "OK":"NG"),
            self_m_mean[AXIS_Z]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	SENSOR_D_LOG("end");
	return count;
fail_exit:
	SENSOR_D_LOG("end - fail");
	return PAGE_SIZE - 1;
}

static inline bool acc_judge_selfchk_result(const int16_t value, const int16_t l_limit, const int16_t u_limit)
{
    return ((l_limit <= value) && (value <= u_limit)) ? true : false;
}

static int32_t acc_culc_V(const int32_t X, const int32_t Y,const int32_t Z)
{
    int32_t pow_X = X * X;
    int32_t pow_Y = Y * Y;
    int32_t pow_Z = Z * Z;
    int32_t V;
    V = (int32_t)int_sqrt(pow_X+pow_Y+pow_Z);
    return V;
}

static void acc_get_selfchk_plusminus_data(const uint8_t data_num,
                                            int16_t data_set[][AXIS_XYZ_MAX],
                                            int16_t data_sum[])
{
    uint8_t i;
    int32_t wait_ms = atomic_read(&(acc_poll_info.poll_time));
    struct acceleration acc_data;

    SENSOR_D_LOG("start");

    for(i = 0; i < data_num; i++){
        usleep_range(wait_ms*MSEC_2_USEC, wait_ms*MSEC_2_USEC);
        acc_update_last_read_data();
        acc_data = acc_last_read_data;
        data_set[i][AXIS_X] = acc_data.outX;
        data_set[i][AXIS_Y] = acc_data.outY;
        data_set[i][AXIS_Z] = acc_data.outZ;
        SENSOR_D_LOG("X[%d], Y[%d], Z[%d]",
                        data_set[i][AXIS_X], data_set[i][AXIS_Y], data_set[i][AXIS_Z]);
        data_sum[AXIS_X] += data_set[i][AXIS_X];
        data_sum[AXIS_Y] += data_set[i][AXIS_Y];
        data_sum[AXIS_Z] += data_set[i][AXIS_Z];
    }
    SENSOR_D_LOG("sum_X[%d], sum_Y[%d], sum_Z[%d]",
                    data_sum[AXIS_X], data_sum[AXIS_Y], data_sum[AXIS_Z]);

    SENSOR_D_LOG("end");
}

static void acc_get_selfchk_normal_data(const uint8_t data_num,
                                        int16_t data_set[][AXIS_XYZV_MAX],
                                        int16_t data_sum[])
{
    uint8_t i;
    int32_t wait_ms = atomic_read(&(acc_poll_info.poll_time));
    struct acceleration acc_data;

    SENSOR_D_LOG("start");

    for(i = 0; i < data_num; i++){
        usleep_range(wait_ms*MSEC_2_USEC, wait_ms*MSEC_2_USEC);
        acc_update_last_read_data();
        acc_data = acc_last_read_data;
        data_set[i][AXIS_X] = acc_data.outX;
        data_set[i][AXIS_Y] = acc_data.outY;
        data_set[i][AXIS_Z] = acc_data.outZ;
        data_set[i][AXIS_V] = acc_culc_V(acc_data.outX, acc_data.outY, acc_data.outZ);
        SENSOR_D_LOG("X[%d], Y[%d], Z[%d], V[%d]",
                        data_set[i][AXIS_X], data_set[i][AXIS_Y],
                        data_set[i][AXIS_Z], data_set[i][AXIS_V]);
        data_sum[AXIS_X] += data_set[i][AXIS_X];
        data_sum[AXIS_Y] += data_set[i][AXIS_Y];
        data_sum[AXIS_Z] += data_set[i][AXIS_Z];
        data_sum[AXIS_V] += data_set[i][AXIS_V];
    }
    SENSOR_D_LOG("sum(X)[%d],sum(Y)[%d],sum(Z)[%d],sum(V)[%d]",
                    data_sum[AXIS_X],data_sum[AXIS_Y],
                    data_sum[AXIS_Z],data_sum[AXIS_V]);

    SENSOR_D_LOG("end");
}

static ssize_t acc_selfchk_exec(char* buf)
{
    ssize_t count = 0;
    int16_t normal_data[SELFCHK_DATANUM][AXIS_XYZV_MAX];
    int16_t normal_mean[AXIS_XYZV_MAX]  = {0};
    int16_t normal_sum[AXIS_XYZV_MAX]   = {0};
    int16_t self_p_data[SELFCHK_DATANUM][AXIS_XYZ_MAX];
    int16_t self_p_mean[AXIS_XYZ_MAX]  = {0};
    int16_t self_p_sum[AXIS_XYZ_MAX]   = {0};
    int16_t self_m_data[SELFCHK_DATANUM][AXIS_XYZ_MAX];
    int16_t self_m_mean[AXIS_XYZ_MAX]  = {0};
    int16_t self_m_sum[AXIS_XYZ_MAX]   = {0};
    int16_t tmp;
    bool result[SELFCHK_CONDITION_MAX] = {false};

    SENSOR_D_LOG("start");

    sensor_set_selfchk_config(SENSOR_ACC, SELFCHK_PRE);
    msleep(200);
    acc_get_selfchk_normal_data(SELFCHK_DATANUM, normal_data, normal_sum);
    normal_mean[AXIS_X] = normal_sum[AXIS_X] / SELFCHK_DATANUM;
    normal_mean[AXIS_Y] = normal_sum[AXIS_Y] / SELFCHK_DATANUM;
    normal_mean[AXIS_Z] = normal_sum[AXIS_Z] / SELFCHK_DATANUM;
    normal_mean[AXIS_V] = normal_sum[AXIS_V] / SELFCHK_DATANUM;

    sensor_enable( SENSOR_ACC, &acc_poll_info, false );
    sensor_set_selfchk_config(SENSOR_ACC, SELFCHK_PLUS);
    sensor_enable( SENSOR_ACC, &acc_poll_info, true );
    msleep(200);
    acc_get_selfchk_plusminus_data(SELFCHK_DATANUM, self_p_data, self_p_sum);
    self_p_mean[AXIS_X] = self_p_sum[AXIS_X] / SELFCHK_DATANUM;
    self_p_mean[AXIS_Y] = self_p_sum[AXIS_Y] / SELFCHK_DATANUM;
    self_p_mean[AXIS_Z] = self_p_sum[AXIS_Z] / SELFCHK_DATANUM;

    sensor_enable( SENSOR_ACC, &acc_poll_info, false );
    sensor_set_selfchk_config(SENSOR_ACC, SELFCHK_MINUS);
    sensor_enable( SENSOR_ACC, &acc_poll_info, true );
    msleep(200);
    acc_get_selfchk_plusminus_data(SELFCHK_DATANUM, self_m_data, self_m_sum);
    self_m_mean[AXIS_X] = self_m_sum[AXIS_X] / SELFCHK_DATANUM;
    self_m_mean[AXIS_Y] = self_m_sum[AXIS_Y] / SELFCHK_DATANUM;
    self_m_mean[AXIS_Z] = self_m_sum[AXIS_Z] / SELFCHK_DATANUM;
    sensor_enable( SENSOR_ACC, &acc_poll_info, false );

    result[NORMAL_V] = acc_judge_selfchk_result(normal_mean[AXIS_V],
                                                SELFCHK_NORMAL_V_LLIMIT,
                                                SELFCHK_NORMAL_V_ULIMIT);

    tmp = abs(normal_mean[AXIS_X] - self_p_mean[AXIS_X]);
    result[NORMAL_X_SELFP_X] = acc_judge_selfchk_result(tmp,
                                                        SELFCHK_DIFF_SELFP_LLIMIT,
                                                        SELFCHK_DIFF_SELFP_ULIMIT);

    tmp = abs(normal_mean[AXIS_Y] - self_p_mean[AXIS_Y]);
    result[NORMAL_Y_SELFP_Y] = acc_judge_selfchk_result(tmp,
                                                        SELFCHK_DIFF_SELFP_LLIMIT,
                                                        SELFCHK_DIFF_SELFP_ULIMIT);

    tmp = abs(normal_mean[AXIS_Z] - self_p_mean[AXIS_Z]);
    result[NORMAL_Z_SELFP_Z] = acc_judge_selfchk_result(tmp,
                                                        SELFCHK_DIFF_SELFP_LLIMIT,
                                                        SELFCHK_DIFF_SELFP_ULIMIT);

    tmp = abs(normal_mean[AXIS_X] - self_m_mean[AXIS_X]);
    result[NORMAL_X_SELFM_X] = acc_judge_selfchk_result(tmp,
                                                        SELFCHK_DIFF_SELFM_LLIMIT,
                                                        SELFCHK_DIFF_SELFM_ULIMIT);

    tmp = abs(normal_mean[AXIS_Y] - self_m_mean[AXIS_Y]);
    result[NORMAL_Y_SELFM_Y] = acc_judge_selfchk_result(tmp,
                                                        SELFCHK_DIFF_SELFM_LLIMIT,
                                                        SELFCHK_DIFF_SELFM_ULIMIT);

    tmp = abs(normal_mean[AXIS_Z] - self_m_mean[AXIS_Z]);
    result[NORMAL_Z_SELFM_Z] = acc_judge_selfchk_result(tmp,
                                                        SELFCHK_DIFF_SELFM_LLIMIT,
                                                        SELFCHK_DIFF_SELFM_ULIMIT);

    count += acc_selfchk_make_output_result(result, normal_mean, self_p_mean, self_m_mean, buf);
    count += acc_selfchk_make_output_alldata(normal_data, self_p_data, self_m_data, buf+count);

    SENSOR_D_LOG("end");
    return count;
}

static int32_t acc_selfchk_advance_preparation(void)
{
    int32_t ret = 0;
    SENSOR_D_LOG("start");
    if (SENSOR_ON == sensor_get_status(SENSOR_ACC)){
        sensor_enable( SENSOR_ACC, &acc_poll_info, false );
    }
    sensor_set_dynamiccalib_enable(SENSOR_ACC, false);
    sensor_set_poll_time( SENSOR_ACC, &acc_poll_info, 20);
    sensor_enable( SENSOR_ACC, &acc_poll_info, true );

    SENSOR_D_LOG("end");
    return ret;
}

static ssize_t acc_selfcheck_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int32_t ret = 0;
    ssize_t count = 0;
    SENSOR_D_LOG("start");

    ret = acc_selfchk_advance_preparation();
    if(ret != SNS_RC_OK){
        count += sprintf(buf, "self check advance preparation failed.\n" );
        goto exit;
    }

    count += acc_selfchk_exec(buf);
exit:
    SENSOR_D_LOG("end");
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

void acc_save_cal_ofs_val(struct acceleration *savedata)
{
    SENSOR_N_LOG("start");
    saved_ofs_val.x = savedata->nX;
    saved_ofs_val.y = savedata->nY;
    saved_ofs_val.z = savedata->nZ;
    SENSOR_N_LOG("end");
}

void acc_load_cal_ofs_val(struct acceleration* outdata)
{
    SENSOR_N_LOG("start");
    outdata->nX = saved_ofs_val.x;
    outdata->nY = saved_ofs_val.y;
    outdata->nZ = saved_ofs_val.z;
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
    SENSOR_N_LOG("start");

    if(sns_get_reset_status() == false){
        sns_iio_report_event_now(SENSOR_ACC);
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
    SENSOR_N_LOG("sensor_input_init()-->ret[%d] dev[%p]",
                  ret, acc_input_info.dev );

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

void acc_driver_exit( void )
{

}
EXPORT_SYMBOL(acc_driver_exit);
