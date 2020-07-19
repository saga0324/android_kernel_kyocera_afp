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
 

#include <linux/module.h>
#include <linux/kernel.h>
#include "sensor_driver.h"
#include "sensor_micon_driver.h"
#include "sensor_com.h"

#define DEFAULT_PRESSURE_DELAY  (45)
#define OFFSET_PRE_SUMPLE_NUM   (200)
#define OFFSET_PRE_TH           (200)
#define ABSMIN_PRESSURE         (-1)
#define ABSMAX_PRESSURE         (2000*100)
#define ABSMAX_CAL_PA           (200000)
#define ABSMIN_CAL_PA           (-200000)
#define PRESSURE_DUMMY_DATA     (-1)

enum SENSOR_CAL_MODE {
    MODE_0 = 0,
    MODE_1,
    MODE_2,
    MODE_3,
    MODE_4,
    MODE_MAX
};

typedef struct tCalibrationCtrl {
    bool     m_bWaitSetting;
    bool     m_bComplete;
    bool     m_bRengeChkErr;
    uint16_t m_unSmpN;
    int32_t  m_nP0;
    int32_t  m_nRefP;
    int32_t  m_nCalP;
    int32_t  m_nCurrentSampleNum;
    int32_t  m_nSummationSample;
    int32_t  m_nSummationPa;
    int32_t  m_nSummationPb;
    int32_t  m_nSummationPa_Sample;
    int32_t  m_nSummationPb_Sample;
    int32_t  m_nSummationSample_squ;
    int32_t  m_nPressureBase;
    int32_t  m_nSmpMaxP;
    int32_t  m_nSmpMinP;
    int32_t  m_nCalTH;
    int32_t  m_nMode;
    struct mutex m_tCalibMutex;
} pre_cal_ctrl_str;

static void pressure_update_last_read_data(void);
static void pressure_poll_work_func(struct work_struct *work);
static void pressure_set_input_params(struct input_dev *dev);

static ssize_t pressure_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t pressure_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t pressure_delay_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t pressure_delay_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t pressure_data_raw_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t pressure_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t pressure_wake_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t pressure_param_base_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t pressure_param_base_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t pressure_cal_mode_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t pressure_start_cal_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t pressure_start_cal_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t pressure_cal_wait_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t pressure_offset_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t pressure_offset_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t pressure_cal_smp_n_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t pressure_cal_smp_n_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t pressure_cal_th_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t pressure_cal_th_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t pressure_batch_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t pressure_batch_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t pressure_flush_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t pressure_device_id_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);

static DEVICE_ATTR(enable,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    pressure_enable_show,
    pressure_enable_store
);
static DEVICE_ATTR(delay,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    pressure_delay_show,
    pressure_delay_store
);
static DEVICE_ATTR(data_raw,
    S_IRUSR|S_IRGRP,
    pressure_data_raw_show,
    NULL
    );
static DEVICE_ATTR(data,
    S_IRUSR|S_IRGRP,
    pressure_data_show,
    NULL
);
static DEVICE_ATTR(wake,
    S_IWUSR|S_IWGRP,
    NULL,
    pressure_wake_store
);
static DEVICE_ATTR(param_base,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    pressure_param_base_show,
    pressure_param_base_store
);
static DEVICE_ATTR(cal_mode,
    S_IWUSR|S_IWGRP,
    NULL,
    pressure_cal_mode_store
);
static DEVICE_ATTR(start_cal,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    pressure_start_cal_show,
    pressure_start_cal_store
);

static DEVICE_ATTR(cal_wait,
    S_IRUSR|S_IRGRP,
    pressure_cal_wait_show,
    NULL
);
static DEVICE_ATTR(offset,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    pressure_offset_show,
    pressure_offset_store
);
static DEVICE_ATTR(cal_smp_n,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    pressure_cal_smp_n_show,
    pressure_cal_smp_n_store
);
static DEVICE_ATTR(cal_th,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    pressure_cal_th_show,
    pressure_cal_th_store
);
static DEVICE_ATTR(batch,
    S_IWUSR|S_IWGRP,
    NULL,
    pressure_batch_store
);
static DEVICE_ATTR(batch_data,
    S_IRUSR|S_IRGRP,
    pressure_batch_data_show,
    NULL
);
static DEVICE_ATTR(flush,
    S_IWUSR|S_IWGRP,
    NULL,
    pressure_flush_store
);
static DEVICE_ATTR(device_id,
    S_IRUSR|S_IRGRP,
    pressure_device_id_show,
    NULL
);

static struct attribute *pressure_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_delay.attr,
    &dev_attr_data_raw.attr,
    &dev_attr_data.attr,
    &dev_attr_wake.attr,
    &dev_attr_param_base.attr,
    &dev_attr_cal_mode.attr,
    &dev_attr_start_cal.attr,
    &dev_attr_cal_wait.attr,
    &dev_attr_offset.attr,
    &dev_attr_cal_smp_n.attr,
    &dev_attr_cal_th.attr,
    &dev_attr_batch.attr,
    &dev_attr_batch_data.attr,
    &dev_attr_flush.attr,
    &dev_attr_device_id.attr,
    NULL
};

static struct attribute_group pressure_attr_grp = {
    .attrs = pressure_attributes
};

struct sensor_input_info_str pressure_input_info =
{
    NULL,
    pressure_set_input_params,
    &pressure_attr_grp,
};

struct sensor_poll_info_str pressure_poll_info = {
    .name       = "pressure_poll_wq",
    .poll_time  = ATOMIC_INIT(DEFAULT_PRESSURE_DELAY),
    .poll_func  = pressure_poll_work_func,
};

static struct pressure pressure_last_read_data = {
    0
};

static struct sens_ofs saved_ofs_val = {0};
//static atomic_t g_nCalP = ATOMIC_INIT(0);
static atomic_t g_nRefP = ATOMIC_INIT(0);
static void pressure_calib_work_func(struct work_struct *work);
static void pressure_calibration_range_chk(const struct pressure* preData);
static pre_cal_ctrl_str pre_calib_ctrl;
struct sensor_poll_info_str pressure_calib_wq_info = {
    .name       = "pressure_cal_wq",
    .poll_time  = ATOMIC_INIT(DEFAULT_PRESSURE_DELAY),
    .poll_func  = pressure_calib_work_func,
};

static struct sensor_batch_data_str pressure_batch_data;
static uint32_t g_time_stamp_pressure      = 0;
static uint32_t g_input_num_pressure       = 0;
extern struct mutex sensor_batch_mutex;

static ssize_t pressure_batch_store(struct device *dev,
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

    ret = sensor_set_batch(SENSOR_PRESSURE, batch_info);

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t pressure_batch_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret =0;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_batch_mutex);

    ret = sprintf(buf, "%d %d %d %d\n",pressure_batch_data.payload_size,
                       pressure_batch_data.recode_num, g_input_num_pressure, g_time_stamp_pressure);

    g_time_stamp_pressure = 0;

    mutex_unlock(&sensor_batch_mutex);

    SENSOR_N_LOG("end");

    return ret;
}

void pressure_ring_buffer_timestamp(
    uint32_t time_stamp_pressure)
{
    SENSOR_N_LOG("start");
    g_time_stamp_pressure = time_stamp_pressure;
    SENSOR_N_LOG("end %d",g_time_stamp_pressure);
}

void pressure_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data )
{

    SENSOR_N_LOG("start");

    pressure_batch_data = batch_data;

    sensor_report_batch( pressure_input_info.dev,
                         repo_type,
                         pressure_batch_data );

    SENSOR_N_LOG("end");
    return;
}

void pressure_timestamp_report(void)
{
    SENSOR_N_LOG("start");

    sensor_report_batch( pressure_input_info.dev,
                         SENSOR_COMP_TIME,
                         pressure_batch_data );

    SENSOR_N_LOG("end");
    return;
}

static ssize_t pressure_flush_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    uint32_t ret = 0;

    SENSOR_N_LOG("start");

    ret = sensor_set_flush(SENSOR_PRESSURE, pressure_input_info.dev);

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t pressure_device_id_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret =0;
    uint8_t id;

    SENSOR_D_LOG("start");
    sensor_com_mutex_lock();
    id = sensor_get_device_id(SENSOR_PRESSURE);
    sensor_com_mutex_unlock();
    ret = sprintf(buf,"0x%02x\n", id);
    SENSOR_D_LOG("end");

    return ret;
}
static void pressure_set_offset(int32_t offset)
{
    struct pressure prs_ofs_val = {0};
    SENSOR_N_LOG("start");
    prs_ofs_val.pressure = offset;
    pressure_save_cal_ofs_val(&prs_ofs_val);
    sensor_com_mutex_lock();
    sensor_set_offset_value(SENSOR_PRESSURE, saved_ofs_val);
    sensor_com_mutex_unlock();
    SENSOR_N_LOG("end");
}

static int32_t pressure_get_offset(void)
{
    SENSOR_N_LOG("start/end");
    return saved_ofs_val.x;
}

static int32_t pressure_calibration_mode(void)
{
    SENSOR_N_LOG("start");

    pre_calib_ctrl.m_bWaitSetting         = true;
    pre_calib_ctrl.m_bComplete            = false;
    pre_calib_ctrl.m_bRengeChkErr         = false;

    pre_calib_ctrl.m_nP0                  = 0;
    pre_calib_ctrl.m_nRefP                = 0;
    pre_calib_ctrl.m_nCalP                = 0;

    pre_calib_ctrl.m_nCurrentSampleNum    = 0;
    pre_calib_ctrl.m_nSummationSample     = 0;
    pre_calib_ctrl.m_nSummationPa         = 0;
    pre_calib_ctrl.m_nSummationPb         = 0;
    pre_calib_ctrl.m_nSummationPa_Sample  = 0;
    pre_calib_ctrl.m_nSummationPb_Sample  = 0;
    pre_calib_ctrl.m_nSummationSample_squ = 0;
    pre_calib_ctrl.m_nMode = -1;

    SENSOR_N_LOG("end");
    return SNS_RC_OK;
}

static int32_t pressure_calibration_start(int32_t pres_mode, int32_t PressureBase)
{
    int32_t delay = 0;
    SENSOR_N_LOG("start");

    pre_calib_ctrl.m_bWaitSetting = false;
    pre_calib_ctrl.m_nMode = pres_mode;
    pre_calib_ctrl.m_nPressureBase = PressureBase;

    if (pre_calib_ctrl.m_nMode == MODE_2) {
        pre_calib_ctrl.m_bComplete = false;
    }

    pre_calib_ctrl.m_nP0                  = 0;
    pre_calib_ctrl.m_nCurrentSampleNum    = 0;
    pre_calib_ctrl.m_nSummationSample     = 0;
    pre_calib_ctrl.m_nSummationPa         = 0;
    pre_calib_ctrl.m_nSummationPb         = 0;
    pre_calib_ctrl.m_nSummationPa_Sample  = 0;
    pre_calib_ctrl.m_nSummationPb_Sample  = 0;
    pre_calib_ctrl.m_nSummationSample_squ = 0;
    pre_calib_ctrl.m_nSmpMaxP             = 0;
    pre_calib_ctrl.m_nSmpMinP             = 0;

    delay = atomic_read(&(pressure_poll_info.poll_time));
    sensor_starttimer(&(pressure_calib_wq_info.timer), delay);

    SENSOR_N_LOG("end");
    return SNS_RC_OK;
}

static int32_t pressure_calibration_is_wait(void)
{
    int32_t wait;
    SENSOR_N_LOG("start");

    wait = pre_calib_ctrl.m_bWaitSetting;
    if (wait == true && pre_calib_ctrl.m_nMode == MODE_2) {
        wait = 2;
    }
    if (pre_calib_ctrl.m_bRengeChkErr == true) {
        wait = 3;
    }

    SENSOR_N_LOG("end");
    return wait;
}

static void pressure_calibration_periodic(const struct pressure* preData)
{
    int32_t a_data,b_data;
    int32_t sample_num,summ_samp,summ_pa,summ_pa_samp,summ_pb,summ_pb_samp,summ_squ;
    SENSOR_N_LOG("start");

    if (pre_calib_ctrl.m_nCurrentSampleNum == pre_calib_ctrl.m_unSmpN) {
        pre_calib_ctrl.m_bComplete    = true;
        pre_calib_ctrl.m_bWaitSetting = true;
    } else {
        pressure_calibration_range_chk(preData);

        if (pre_calib_ctrl.m_nCurrentSampleNum == 0) {
            pre_calib_ctrl.m_nP0 = preData->pressure;
        }

        pre_calib_ctrl.m_nSummationSample += (pre_calib_ctrl.m_nCurrentSampleNum + 1);
        pre_calib_ctrl.m_nSummationPa += (preData->pressure - pre_calib_ctrl.m_nP0) * 100;
        pre_calib_ctrl.m_nSummationPb += (preData->pressure - pre_calib_ctrl.m_nP0);
        pre_calib_ctrl.m_nSummationPa_Sample += ((pre_calib_ctrl.m_nCurrentSampleNum + 1) * ((preData->pressure - pre_calib_ctrl.m_nP0) * 100));
        pre_calib_ctrl.m_nSummationPb_Sample += ((pre_calib_ctrl.m_nCurrentSampleNum + 1) * (preData->pressure - pre_calib_ctrl.m_nP0));
        pre_calib_ctrl.m_nSummationSample_squ += ((pre_calib_ctrl.m_nCurrentSampleNum + 1) * (pre_calib_ctrl.m_nCurrentSampleNum + 1));
        pre_calib_ctrl.m_nCurrentSampleNum++;

        if (pre_calib_ctrl.m_nCurrentSampleNum == pre_calib_ctrl.m_unSmpN &&
            pre_calib_ctrl.m_bRengeChkErr == false) {

            sample_num = pre_calib_ctrl.m_nCurrentSampleNum;
            summ_samp  = pre_calib_ctrl.m_nSummationSample;
            summ_pa     = pre_calib_ctrl.m_nSummationPa;
            summ_pb     = pre_calib_ctrl.m_nSummationPb;
            summ_pa_samp = pre_calib_ctrl.m_nSummationPa_Sample;
            summ_pb_samp = pre_calib_ctrl.m_nSummationPb_Sample;
            summ_squ   = pre_calib_ctrl.m_nSummationSample_squ;

            if (pre_calib_ctrl.m_nMode == MODE_0) {
                a_data = ((summ_pa_samp - (summ_pa / sample_num * summ_samp))) / (summ_squ - (summ_samp / sample_num * summ_samp));
                b_data = ((summ_squ / sample_num * summ_pb) - (summ_pb_samp / sample_num * summ_samp)) / (summ_squ - (summ_samp / sample_num * summ_samp));
                SENSOR_N_LOG("!!!!! PreCalib: a_data = %d b_data = %d",a_data,b_data);
                SENSOR_N_LOG("!!!!! PreCalib:A (summ_pa / sample_num * summ_samp) = %d (summ_squ - (summ_samp / sample_num * summ_samp)) = %d",
                        (summ_pa / sample_num * summ_samp),(summ_squ - (summ_samp / sample_num * summ_samp)));

                pre_calib_ctrl.m_nRefP = (a_data * sample_num / 100) + b_data + pre_calib_ctrl.m_nP0;
                atomic_set(&g_nRefP, pre_calib_ctrl.m_nRefP);
                SENSOR_N_LOG("PreCalib:Reference  m_nRefP = %d m_nP0 = %d sample_num = %d summ_samp=%d summ_pa=%d summ_pa_samp=%d summ_pb=%d summ_pb_samp=%d summ_squ=%d",
                    pre_calib_ctrl.m_nRefP,pre_calib_ctrl.m_nP0, sample_num, summ_samp, summ_pa, summ_pa_samp, summ_pb, summ_pb_samp, summ_squ);
            } else if (pre_calib_ctrl.m_nMode == MODE_2) {
                a_data = ((summ_pa_samp - (summ_pa / sample_num * summ_samp))) / (summ_squ - (summ_samp / sample_num * summ_samp));
                b_data = ((summ_squ / sample_num * summ_pb) - (summ_pb_samp / sample_num * summ_samp)) / (summ_squ - (summ_samp / sample_num * summ_samp));
                SENSOR_N_LOG("!!!!! PreCalib: a_data = %d b_data = %d",a_data,b_data);
                SENSOR_N_LOG("!!!!! PreCalib:A (summ_pa / sample_num * summ_samp) = %d (summ_squ - (summ_samp / sample_num * summ_samp)) = %d",
                        (summ_pa / sample_num * summ_samp),(summ_squ - (summ_samp / sample_num * summ_samp)));

                pre_calib_ctrl.m_nCalP = pre_calib_ctrl.m_nPressureBase - ((a_data / 100) + b_data + pre_calib_ctrl.m_nP0);
                pressure_set_offset(pre_calib_ctrl.m_nCalP);
                input_report_abs(pressure_input_info.dev, ABS_RX, pre_calib_ctrl.m_nCalP);
                SENSOR_N_LOG("PreCalib:complete calp = %d m_nP0 = %d sample_num = %d summ_samp=%d summ_pa=%d summ_pa_samp=%d summ_pb=%d summ_pb_samp=%d summ_squ=%d",
                    pre_calib_ctrl.m_nCalP,pre_calib_ctrl.m_nP0, sample_num, summ_samp, summ_pa, summ_pa_samp, summ_pb, summ_pb_samp, summ_squ);
            }
        }
    }

    SENSOR_N_LOG("end");
}

static void pressure_calib_work_func(struct work_struct *work)
{
    int32_t ret = SNS_RC_ERR;
    union sensor_read_data_u read_data;
    bool bCalibIdle = false;
    bool bCalibComp = false;
    SENSOR_N_LOG("start");

    ret = sensor_type_get_data(SENSOR_PRESSURE, &read_data);
    if (ret != SNS_RC_OK) {
        SENSOR_ERR_LOG("sensor_type_get_data Error ret[%d]", ret);
        return;
    }

    bCalibIdle = pre_calib_ctrl.m_bWaitSetting;
    bCalibComp = pre_calib_ctrl.m_bComplete;

    if ((bCalibIdle == false) && (bCalibComp == false)) {
        pressure_calibration_periodic(&read_data.pressure_data);
        bCalibComp = pre_calib_ctrl.m_bComplete;
        if (bCalibComp == true) {
            sensor_stoptimer(&(pressure_calib_wq_info.timer));
        }
    }

    SENSOR_N_LOG("end");
}

static void pressure_calibration_range_chk(const struct pressure* preData)
{
    int32_t presns_threshold = 0;

    SENSOR_N_LOG("start P:%d", preData->pressure);

    if (pre_calib_ctrl.m_nCurrentSampleNum == 0) {
        pre_calib_ctrl.m_nSmpMaxP = preData->pressure;
        pre_calib_ctrl.m_nSmpMinP = preData->pressure;
    }

    if (pre_calib_ctrl.m_nSmpMaxP < preData->pressure) {
        pre_calib_ctrl.m_nSmpMaxP = preData->pressure;
    } else if (pre_calib_ctrl.m_nSmpMinP > preData->pressure) {
        pre_calib_ctrl.m_nSmpMinP = preData->pressure;
    }

    presns_threshold = pre_calib_ctrl.m_nSmpMaxP - pre_calib_ctrl.m_nSmpMinP;

    if (presns_threshold >= pre_calib_ctrl.m_nCalTH) {
        pre_calib_ctrl.m_bRengeChkErr = true;
        SENSOR_ERR_LOG("Range Chk Err m_bRengeChkErr :%d", pre_calib_ctrl.m_bRengeChkErr);
    }

    SENSOR_N_LOG("end");
}

static ssize_t pressure_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int enable = 0;
    SENSOR_N_LOG("start");

    enable = sensor_get_status(SENSOR_PRESSURE);

    SENSOR_N_LOG("end ->enable[%d]",enable);
    return sprintf(buf, "%d\n", enable);
}

static ssize_t pressure_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    unsigned long enable = 0;
    int ret = 0;

    SENSOR_N_LOG("start");

    ret = kstrtoul(buf, 10, &enable);
    SENSOR_N_LOG("kstrtoul() ret[%d]->enable[%d]",ret, (int)enable);

    sensor_enable(SENSOR_PRESSURE, &pressure_poll_info, (bool)enable);

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t pressure_delay_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int32_t delay = 0;
    SENSOR_N_LOG("start");
    delay = atomic_read(&(pressure_poll_info.poll_time));
    SENSOR_N_LOG("end ->delay[%d]",(int)delay);
    return sprintf(buf, "%d\n", delay);
}

static ssize_t pressure_delay_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    unsigned long delay = 0;
    int ret = 0;
    SENSOR_N_LOG("start");

    ret = kstrtoul(buf, 10, &delay);
    SENSOR_N_LOG("kstrtoul() ret[%d]->delay[%d]",ret, (int)delay);

    sensor_set_poll_time( SENSOR_PRESSURE, &pressure_poll_info, (int32_t)delay);

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t pressure_data_raw_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    struct pressure last_data;
    SENSOR_N_LOG("start");
    pressure_update_last_read_data();
    last_data = pressure_last_read_data;
    SENSOR_N_LOG("end - last_data[%d]", last_data.pressure );

    return sprintf(buf, "%d %d %d %d\n", 0, 0, last_data.pressure, 0);
}

static ssize_t pressure_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int32_t pressure;

    SENSOR_N_LOG("start");
    pressure_update_last_read_data();
    pressure = pressure_last_read_data.pressure + pressure_get_offset();
    SENSOR_N_LOG("end - last_data[%d]", pressure);

    return sprintf(buf,"%d\n", pressure);
}

static ssize_t pressure_wake_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    static int cnt = 1;

    SENSOR_N_LOG("start");
    input_report_abs(pressure_input_info.dev, ABS_WAKE, cnt++);
    input_sync(pressure_input_info.dev);

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t pressure_param_base_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int32_t ret;
    DailysSetBaseParam DailysBaseParam;
    SENSOR_N_LOG("start");

    memset(&DailysBaseParam, 0x00, sizeof(DailysBaseParam));

    sensor_com_mutex_lock();
    ret = sns_pressure_base_val(false, &DailysBaseParam);
    sensor_com_mutex_unlock();

    if (ret < 0) {
        SENSOR_ERR_LOG("ERROR sns_pressure_base_val:ret[%d]", ret);
    }

    SENSOR_N_LOG("end");

    return sprintf(buf, "%d %d\n", DailysBaseParam.m_BasePress,DailysBaseParam.m_BaseHeight);
}

static ssize_t pressure_param_base_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    int32_t ret;
    DailysSetBaseParam DailysBaseParam;
    SENSOR_N_LOG("start");

    ret = sscanf(buf, "%d %d %d", &DailysBaseParam.m_SetMode,&DailysBaseParam.m_BasePress,
                                  &DailysBaseParam.m_BaseHeight);
    if (ret != 3) {
        SENSOR_ERR_LOG("ERROR sscanf:ret[%d]", ret);
        return count;
    }

    sensor_com_mutex_lock();
    ret = sns_pressure_base_val(true, &DailysBaseParam);
    sensor_com_mutex_unlock();
    if (ret < 0) {
        SENSOR_ERR_LOG("ERROR sns_pressure_base_val:ret[%d]", ret);
    }

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t pressure_cal_mode_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    int32_t ret = 0;
    SENSOR_N_LOG("start");

    ret = pressure_calibration_mode();
    if (ret < 0) {
        SENSOR_ERR_LOG("ERROR pressure_calibration_mode:ret[%d]", ret);
    }

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t pressure_start_cal_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int32_t nRefP;
    SENSOR_N_LOG("start");

    nRefP = atomic_read(&g_nRefP);

    SENSOR_N_LOG("end");
    return sprintf(buf, "%d\n", nRefP);
}

static ssize_t pressure_start_cal_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    int32_t ret = 0;
    uint32_t pres_mode = 0, pres_base = 0;
    SENSOR_N_LOG("start");

    ret = sscanf(buf, "%d %d", &pres_mode, &pres_base);
    SENSOR_N_LOG("ret = %d pres_mode = %d pres_base = %d", ret, pres_mode, pres_base);

    if (ret != 2) {
        SENSOR_ERR_LOG("ERROR sscanf:ret[%d]", ret);
        return count;
    }
    ret = pressure_calibration_start(pres_mode,pres_base);
    if (ret != 0) {
        SENSOR_ERR_LOG("ERROR pressure_calibration_start:ret[%d]", ret);
        return count;
    }

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t pressure_cal_wait_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    SENSOR_N_LOG("start/end");

    return sprintf(buf, "%d\n", pressure_calibration_is_wait());
}

static ssize_t pressure_offset_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    SENSOR_N_LOG("start/end");

    return sprintf(buf, "%d\n", pressure_get_offset());
}

static ssize_t pressure_offset_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    int32_t ret = 0;
    int32_t offset = 0;
    SENSOR_N_LOG("start");

    ret = sscanf(buf, "%d", &offset);
    if (ret != 1) {
        SENSOR_ERR_LOG("ERROR sscanf:ret[%d]", ret);
        return count;
    }
    pressure_set_offset(offset);

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t pressure_cal_smp_n_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int cal_smp_n;
    SENSOR_N_LOG("start");

    cal_smp_n = pre_calib_ctrl.m_unSmpN;

    SENSOR_N_LOG("end");
    return sprintf(buf, "%d\n", cal_smp_n);
}

static ssize_t pressure_cal_smp_n_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    int32_t ret = 0;
    unsigned long arg_iCal_smp_n;
    SENSOR_N_LOG("start");

    ret = kstrtoul(buf, 10, &arg_iCal_smp_n);
    if (ret < 0) {
        SENSOR_ERR_LOG("kstrtoul ERROR:%d count[%zd]",ret, count);
        return count;
    }
    SENSOR_N_LOG("Cal_smp_n = %ld\n", arg_iCal_smp_n);

    pre_calib_ctrl.m_unSmpN = arg_iCal_smp_n;

    SENSOR_N_LOG("end");
    return count;
}


static ssize_t pressure_cal_th_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int cal_th;
    SENSOR_N_LOG("start");

    cal_th = pre_calib_ctrl.m_nCalTH;

    SENSOR_N_LOG("end");
    return sprintf(buf, "%d\n", cal_th);
}

static ssize_t pressure_cal_th_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    int32_t ret = 0;
    unsigned long arg_iCal_th;

    SENSOR_N_LOG("start");

    ret = kstrtoul(buf, 10, &arg_iCal_th);
    if (ret < 0) {
        SENSOR_ERR_LOG("kstrtoul ERROR:%d count[%zd]",ret, count);
        return count;
    }
    SENSOR_N_LOG("Cal_th = %ld", arg_iCal_th);

    pre_calib_ctrl.m_nCalTH = arg_iCal_th;

    SENSOR_N_LOG("m_nCalTH = %d", pre_calib_ctrl.m_nCalTH);

    SENSOR_N_LOG("end");
    return count;
}

void pressure_save_cal_ofs_val(struct pressure* savedata)
{
    SENSOR_N_LOG("start");
    saved_ofs_val.x = savedata->pressure;
    SENSOR_N_LOG("end");
}

void pressure_load_cal_ofs_val(struct pressure* outdata)
{
    SENSOR_N_LOG("start");
    outdata->pressure = saved_ofs_val.x;
    SENSOR_N_LOG("end");
}

static void pressure_update_last_read_data(void)
{
    unsigned long enable = 0;
    int32_t ret = 0;
    union sensor_read_data_u read_data;

    enable = sensor_get_status(SENSOR_PRESSURE);

    if (enable) {
        ret = sensor_type_get_data(SENSOR_PRESSURE, &read_data);
        if (0 == ret) {
            pressure_last_read_data.pressure = read_data.pressure_data.pressure;
        }
    }
}

static void pressure_poll_work_func(struct work_struct *work)
{
    SENSOR_N_LOG("start");

    if (sns_get_reset_status() == false) {
        sns_iio_report_event_now(SENSOR_PRESSURE);
    }

    SENSOR_N_LOG("end");
    return;
}

static void pressure_set_input_params(struct input_dev *dev)
{
    SENSOR_N_LOG("start");

    if (!dev) {
        SENSOR_ERR_LOG("bad parm --> dev is NULL");
        return;
    }

    dev->name = "pressure";
    dev->id.bustype = BUS_SPI;

    set_bit(EV_ABS, dev->evbit);
    input_set_abs_params(dev, ABS_X, ABSMIN_PRESSURE, ABSMAX_PRESSURE, 0, 0);
    input_set_abs_params(dev, ABS_RX, ABSMIN_CAL_PA, ABSMAX_CAL_PA, 0, 0);
    input_set_abs_params(dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);

    SENSOR_N_LOG("end");
    return;
}

void pressure_driver_init(void)
{
    int ret = 0;

    SENSOR_N_LOG("start");

    ret = sensor_input_init(&pressure_input_info);
    SENSOR_N_LOG("sensor_input_init()-->ret[%d] dev[%p]",
                  ret, pressure_input_info.dev);

    if ((0 != ret) || (NULL == (pressure_input_info.dev))) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        SENSOR_ERR_LOG("end return[%d]",-ENODEV);
        return;
    }

    sensor_poll_init(SENSOR_PRESSURE, &pressure_poll_info);
    sensor_poll_init(SENSOR_MAX,&pressure_calib_wq_info);
    pre_calib_ctrl.m_bWaitSetting = true;
    pre_calib_ctrl.m_bComplete    = true;
    pre_calib_ctrl.m_bRengeChkErr = false;

    pre_calib_ctrl.m_unSmpN              = OFFSET_PRE_SUMPLE_NUM;
    pre_calib_ctrl.m_nP0                 = 0;
    pre_calib_ctrl.m_nRefP               = 0;
    pre_calib_ctrl.m_nCalP               = 0;
    pre_calib_ctrl.m_nCurrentSampleNum   = 0;
    pre_calib_ctrl.m_nSummationSample    = 0;
    pre_calib_ctrl.m_nSummationPa        = 0;
    pre_calib_ctrl.m_nSummationPa_Sample = 0;
    pre_calib_ctrl.m_nSummationPb        = 0;
    pre_calib_ctrl.m_nSummationPb_Sample = 0;
    pre_calib_ctrl.m_nSummationSample_squ= 0;
    pre_calib_ctrl.m_nPressureBase       = 0;
    pre_calib_ctrl.m_nSmpMaxP            = 0;
    pre_calib_ctrl.m_nSmpMinP            = 0;
    pre_calib_ctrl.m_nCalTH              = OFFSET_PRE_TH;
    pre_calib_ctrl.m_nMode               = -1;

    SENSOR_N_LOG("end");
}

EXPORT_SYMBOL(pressure_driver_init);

void pressure_driver_exit(void)
{
}

EXPORT_SYMBOL(pressure_driver_exit);