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

#define DEFAULT_GYRO_DELAY    (30)
#define ABSMAX_GYRO          (32000)
#define ABSMIN_GYRO          (-32000)
#define GYRO_DIAG_REQ_POWER_ON  (0)
#define GYRO_DIAG_REQ_POWER_OFF (1)
#define GYRO_DIAG_REQ_INIT      (2)

#define FASTEST_PERIOD              (5)
#define SELFCHK_DATANUM             (5)
#define SELFCHK_NORMAL_LLIMIT       (0)
#define SELFCHK_NORMAL_ULIMIT       (3277)
#define SELFCHK_DIFF_SELFP_LLIMIT   (2458)
#define SELFCHK_DIFF_SELFP_ULIMIT   (11468)
#define SELFCHK_DIFF_SELFM_LLIMIT   (2458)
#define SELFCHK_DIFF_SELFM_ULIMIT   (11468)
#define MSEC_2_USEC                 (1000)

#define GYROCAL_PRM_MAX             (14)

enum GYRO_SELFCHK_CONDITION
{
    NORMAL_X = 0,
    NORMAL_Y,
    NORMAL_Z,
    NORMAL_X_SELFP_X,
    NORMAL_Y_SELFP_Y,
    NORMAL_Z_SELFP_Z,
    NORMAL_X_SELFM_X,
    NORMAL_Y_SELFM_Y,
    NORMAL_Z_SELFM_Z,
    SELFCHK_CONDITION_MAX
};

enum {
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z,
    AXIS_XYZ_MAX,
    AXIS_V = 3,
    AXIS_XYZV_MAX,
} GYRO_AXIS;

enum {
    ERR_OTHER = -3,
    ERR_NOEXEC,
    ERR_TIMEOUT,
    SUCCESS
} GYRO_MANUALCALIB_RESULT;

static ssize_t gyro_batch_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t gyro_batch_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t gyro_flush_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t gyro_ope_device_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf, size_t count );
static ssize_t gyro_offset_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t gyro_offset_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t gyro_nrfilt_prm_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t gyro_nrfilt_prm_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t gyro_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t gyro_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t gyro_delay_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t gyro_delay_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t gyro_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t gyro_device_id_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t gyro_selfcheck_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t gyro_calib_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t gyro_calib_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static void gyro_update_last_read_data(void);
static void gyro_poll_work_func(struct work_struct *work);
static void gyro_set_input_params( struct input_dev *dev );


static DEVICE_ATTR(enable,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    gyro_enable_show,
    gyro_enable_store
);
static DEVICE_ATTR(delay,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    gyro_delay_show,
    gyro_delay_store
);
static DEVICE_ATTR(data,
    S_IRUSR|S_IRGRP,
    gyro_data_show,
    NULL
);
static DEVICE_ATTR(raw_data,
    S_IRUSR|S_IRGRP,
    gyro_data_show,
    NULL
);
static DEVICE_ATTR(offset,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    gyro_offset_show,
    gyro_offset_store
);
static DEVICE_ATTR(ope_device,
    S_IWUSR|S_IWGRP,
    NULL,
    gyro_ope_device_store
);
static DEVICE_ATTR(batch,
    S_IWUSR|S_IWGRP,
    NULL,
    gyro_batch_store
);
static DEVICE_ATTR(batch_data,
    S_IRUSR|S_IRGRP,
    gyro_batch_data_show,
    NULL
);
static DEVICE_ATTR(flush,
    S_IWUSR|S_IWGRP,
    NULL,
    gyro_flush_store
);
static DEVICE_ATTR(device_id,
    S_IRUSR|S_IRGRP,
    gyro_device_id_show,
    NULL
);
static DEVICE_ATTR(selfcheck,
    S_IRUSR|S_IRGRP,
    gyro_selfcheck_show,
    NULL
);
static DEVICE_ATTR(calib,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    gyro_calib_show,
    gyro_calib_store
);
static DEVICE_ATTR(nrfilt_prm,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    gyro_nrfilt_prm_show,
    gyro_nrfilt_prm_store
);

static struct attribute *gyro_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_delay.attr,
    &dev_attr_data.attr,
    &dev_attr_raw_data.attr,
    &dev_attr_offset.attr,
    &dev_attr_ope_device.attr,
    &dev_attr_batch.attr,
    &dev_attr_batch_data.attr,
    &dev_attr_flush.attr,
    &dev_attr_device_id.attr,
    &dev_attr_selfcheck.attr,
    &dev_attr_calib.attr,
    &dev_attr_nrfilt_prm.attr,
    NULL
};

static struct attribute_group gyro_attr_grp = {
    .attrs = gyro_attributes
};

struct sensor_input_info_str gyro_input_info =
{
    NULL,
    gyro_set_input_params,
    &gyro_attr_grp,
};

struct sensor_poll_info_str gyro_poll_info = {
    .name       = "gyro_poll_wq",
    .poll_time  = ATOMIC_INIT(DEFAULT_GYRO_DELAY),
    .poll_func  = gyro_poll_work_func,
};

static struct gyroscope gyro_last_read_data = {
    0,0,0,
};

static atomic_t man_cal_result = ATOMIC_INIT(ERR_NOEXEC);
static struct sens_ofs saved_ofs_val = {0};
static struct sensor_batch_data_str gyro_batch_data;
static uint32_t g_time_stamp_gyro      = 0;
static uint32_t g_input_num_gyro       = 0;
extern struct mutex sensor_batch_mutex;

static ssize_t gyro_batch_store(struct device *dev,
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

    ret = sensor_set_batch( SENSOR_GYRO, batch_info);

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t gyro_batch_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret =0;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_batch_mutex);

    ret = sprintf(buf, "%d %d %d %d\n",gyro_batch_data.payload_size,
                       gyro_batch_data.recode_num, g_input_num_gyro, g_time_stamp_gyro);

    g_time_stamp_gyro = 0;

    mutex_unlock(&sensor_batch_mutex);

    SENSOR_N_LOG("end");

    return ret;
}

void gyro_ring_buffer_timestamp(
    uint32_t time_stamp_gyro)
{
    SENSOR_N_LOG("start");
    g_time_stamp_gyro = time_stamp_gyro;
    SENSOR_N_LOG("end %d",g_time_stamp_gyro);
}

void gyro_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data )
{
    
    SENSOR_N_LOG("start");

    gyro_batch_data = batch_data;

    sensor_report_batch( gyro_input_info.dev,
                         repo_type,
                         gyro_batch_data );

    SENSOR_N_LOG("end");
    return;
}

void gyro_timestamp_report(void)
{
    SENSOR_N_LOG("start");

    sensor_report_batch( gyro_input_info.dev,
                         SENSOR_COMP_TIME,
                         gyro_batch_data );

    SENSOR_N_LOG("end");
    return;
}

static ssize_t gyro_flush_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    uint32_t ret = 0;

    SENSOR_N_LOG("start");

    ret = sensor_set_flush(SENSOR_GYRO, gyro_input_info.dev);

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t gyro_ope_device_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf, size_t count )
{
    int number = 0;
    int32_t ret = 0;

    sscanf(buf, "%d", &number);

    switch (number)
    {
    case GYRO_DIAG_REQ_POWER_ON:
        break;
    case GYRO_DIAG_REQ_POWER_OFF:
        break;
    case GYRO_DIAG_REQ_INIT:
        sensor_com_mutex_lock();
        ret = sns_initialize();
        ret |= sns_set_dev_param();
        sensor_com_mutex_unlock();
        break;
    default:
        break;
    }

    return count;
}

static ssize_t gyro_offset_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t ret = 0;
    struct gyroscope *offsets;

    offsets = (struct gyroscope*)kzalloc(sizeof(struct gyroscope), GFP_KERNEL);
    sensor_com_mutex_lock();
    ret = sensor_get_offset_value(SENSOR_GYRO, (void *)offsets);
    sensor_com_mutex_unlock();

    SENSOR_ERR_LOG("offset0[%d] offset1[%d] offset2[%d]",
                  offsets->x,offsets->y,offsets->z);

    if(ret == SNS_RC_OK) {
        ret = sprintf( buf, "%d %d %d\n",
                       offsets->x,
                       offsets->y,
                       offsets->z );
    }
    kfree(offsets);

    return ret;
}

static ssize_t gyro_nrfilt_prm_store(struct device *dev,
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
    SENSOR_D_LOG("gyro_th_l:%d, gyro_th_h:%d, gyro_coff:%d, gyro_rel_th:%d, "
                "gyro_base_filt_coff:%d, gyro_base_filt_acoff:%d, "
                "gyro_tolerance:%d, gyro_base_filt_hys:%d gyro_filt_lmt_th:%d\n",
                (int)param.th_l, (int)param.th_h, (int)param.coff,
                (int)param.rel_th, (int)param.base_filt_coff, (int)param.base_filt_acoff,
                (int)param.tolerance, (int)param.base_filt_hys, (int)param.filt_lmt_th);
    sensor_set_nrfilt_param(NR_GYRO, param);
    SENSOR_D_LOG("end");
    return count;
}

static ssize_t gyro_nrfilt_prm_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t count = 0;
    struct nr_filt_prm param;
    SENSOR_D_LOG("start");
    sensor_get_nrfilt_param(NR_GYRO, &param);
    count += scnprintf(buf + count, PAGE_SIZE - count,
                        "GYR_NR_FILT_TH_L[%d]\n", param.th_l);
    count += scnprintf(buf + count, PAGE_SIZE - count,
                        "GYR_NR_FILT_TH_H[%d]\n", param.th_h);
    count += scnprintf(buf + count, PAGE_SIZE - count,
                        "GYR_NR_FILT_COFF[%d]\n", param.coff);
    count += scnprintf(buf + count, PAGE_SIZE - count,
                        "GYR_NR_REL_TH[%d]\n", param.rel_th);
    count += scnprintf(buf + count, PAGE_SIZE - count,
                        "GYR_NR_BASE_FILT_COFF[%d]\n", param.base_filt_coff);
    count += scnprintf(buf + count, PAGE_SIZE - count,
                        "GYR_NR_BASE_FILT_ACOFF[%d]\n", param.base_filt_acoff);
    count += scnprintf(buf + count, PAGE_SIZE - count,
                        "GYR_NR_TOLERANCE[%d]\n", param.tolerance);
    count += scnprintf(buf + count, PAGE_SIZE - count,
                        "GYR_NR_BASE_FILT_HYS[%d]\n", param.base_filt_hys);
    count += scnprintf(buf + count, PAGE_SIZE - count,
                        "GYR_NR_FILT_LMT_TH[%d]\n", param.filt_lmt_th);
    SENSOR_D_LOG("end");
    return count;
}

static ssize_t gyro_offset_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    ssize_t ret = 0;
    struct gyroscope gyro_ofs = {0};

    SENSOR_N_LOG("start");

    sscanf(buf, "%d %d %d",
            &gyro_ofs.x,
            &gyro_ofs.y,
            &gyro_ofs.z);

    SENSOR_D_LOG("offset0[%d] offset1[%d] offset2[%d]",
                    gyro_ofs.x, gyro_ofs.y, gyro_ofs.z);
    gyro_save_cal_ofs_val(&gyro_ofs);
    sensor_com_mutex_lock();
    ret = sensor_set_offset_value(SENSOR_GYRO, saved_ofs_val);
    sensor_com_mutex_unlock();

    SENSOR_N_LOG("end - return[%d]",(int)count);

    return count;
}

static ssize_t gyro_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int enable = 0;
    SENSOR_N_LOG("start");

    enable = sensor_get_status(SENSOR_GYRO);

    SENSOR_N_LOG("end ->enable[%d]",enable);
    return sprintf(buf, "%d\n", enable);
}

static ssize_t gyro_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    unsigned long enable = 0;
    int ret = 0;
    struct sensor_batch_info_str batch_info;
    DECLARE_BITMAP(tmp_status, SENSOR_MAX);

    SENSOR_N_LOG("start");

    ret = kstrtoul(buf, 10, &enable);
    SENSOR_N_LOG("kstrtoul() ret[%d]->enable[%d]",ret, (int)enable);

    sensor_get_batch_status(tmp_status);
    if((enable == 0) && (test_bit(SENSOR_GYRO, tmp_status))){
        batch_info.flags = 0;
        batch_info.period_ns = 0;
        batch_info.timeout =0;

        sensor_set_batch( SENSOR_GYRO, batch_info);
    }

    sensor_enable( SENSOR_GYRO, &gyro_poll_info, (bool)enable );

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t gyro_delay_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int32_t delay = 0;
    SENSOR_N_LOG("start");
    delay = atomic_read(&(gyro_poll_info.poll_time));
    SENSOR_N_LOG("end ->delay[%d]",(int)delay);
    return sprintf(buf, "%d\n", delay);
}

static ssize_t gyro_delay_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    unsigned long delay = 0;
    int ret = 0;
    SENSOR_N_LOG("start");

    ret = kstrtoul(buf, 10, &delay);
    SENSOR_N_LOG("kstrtoul() ret[%d]->delay[%d]",ret, (int)delay);

    sensor_set_poll_time( SENSOR_GYRO, &gyro_poll_info, (int32_t)delay);

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t gyro_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    struct gyroscope last_data;

    SENSOR_N_LOG("start");
    gyro_update_last_read_data();
    last_data = gyro_last_read_data;
    SENSOR_N_LOG("end");

    return sprintf(buf, "%d %d %d\n",last_data.x, last_data.y, last_data.z);
}

static ssize_t gyro_device_id_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t ret =0;
    uint8_t id;

    SENSOR_D_LOG("start");
    sensor_com_mutex_lock();
    id = sensor_get_device_id(SENSOR_GYRO);
    sensor_com_mutex_unlock();
    ret = sprintf(buf,"0x%02x\n", id);
    SENSOR_D_LOG("end");

    return ret;
}

static ssize_t gyro_selfchk_make_output_alldata(
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

static ssize_t gyro_selfchk_make_output_result(
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
			"Normal X: %s [mean = %d]\n",
            (result[NORMAL_X] ? "OK":"NG"),
            normal_mean[AXIS_X]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"Normal Y: %s [mean = %d]\n",
            (result[NORMAL_Y] ? "OK":"NG"),
            normal_mean[AXIS_Y]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"Normal Z: %s [mean = %d]\n",
            (result[NORMAL_Z] ? "OK":"NG"),
            normal_mean[AXIS_Z]);
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

static inline bool gyro_judge_selfchk_result(const int16_t value, const int16_t l_limit, const int16_t u_limit)
{
    return ((l_limit <= value) && (value <= u_limit)) ? true : false;
}

static int32_t gyro_culc_V(const int32_t X, const int32_t Y,const int32_t Z)
{
    int32_t pow_X = X * X;
    int32_t pow_Y = Y * Y;
    int32_t pow_Z = Z * Z;
    int32_t V;
    V = (int32_t)int_sqrt(pow_X+pow_Y+pow_Z);
    return V;
}

static void gyro_get_selfchk_plusminus_data(const uint8_t data_num,
                                            int16_t data_set[][AXIS_XYZ_MAX],
                                            int16_t data_sum[])
{
    uint8_t i;
    int32_t wait_ms = atomic_read(&(gyro_poll_info.poll_time));
    struct gyroscope gyro_data;

    SENSOR_D_LOG("start");

    for(i = 0; i < data_num; i++){
        usleep_range(wait_ms*MSEC_2_USEC, wait_ms*MSEC_2_USEC);
        gyro_update_last_read_data();
        gyro_data = gyro_last_read_data;
        data_set[i][AXIS_X] = gyro_data.x;
        data_set[i][AXIS_Y] = gyro_data.y;
        data_set[i][AXIS_Z] = gyro_data.z;
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

static void gyro_get_selfchk_normal_data(const uint8_t data_num,
                                        int16_t data_set[][AXIS_XYZV_MAX],
                                        int16_t data_sum[])
{
    uint8_t i;
    int32_t wait_ms = atomic_read(&(gyro_poll_info.poll_time));
    struct gyroscope gyro_data;

    SENSOR_D_LOG("start");

    for(i = 0; i < data_num; i++){
        usleep_range(wait_ms*MSEC_2_USEC, wait_ms*MSEC_2_USEC);
        gyro_update_last_read_data();
        gyro_data = gyro_last_read_data;
        data_set[i][AXIS_X] = gyro_data.x;
        data_set[i][AXIS_Y] = gyro_data.y;
        data_set[i][AXIS_Z] = gyro_data.z;
        data_set[i][AXIS_V] = gyro_culc_V(gyro_data.x, gyro_data.y, gyro_data.z);
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

static ssize_t gyro_selfchk_exec(char* buf)
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

    sensor_set_selfchk_config(SENSOR_GYRO, SELFCHK_PRE);
    msleep(800);
    gyro_get_selfchk_normal_data(SELFCHK_DATANUM, normal_data, normal_sum);
    normal_mean[AXIS_X] = normal_sum[AXIS_X] / SELFCHK_DATANUM;
    normal_mean[AXIS_Y] = normal_sum[AXIS_Y] / SELFCHK_DATANUM;
    normal_mean[AXIS_Z] = normal_sum[AXIS_Z] / SELFCHK_DATANUM;
    normal_mean[AXIS_V] = normal_sum[AXIS_V] / SELFCHK_DATANUM;

    sensor_enable( SENSOR_GYRO, &gyro_poll_info, false );
    msleep(20);
    sensor_set_selfchk_config(SENSOR_GYRO, SELFCHK_PLUS);
    msleep(60);
    sensor_enable( SENSOR_GYRO, &gyro_poll_info, true );
    msleep(800);
    gyro_get_selfchk_plusminus_data(SELFCHK_DATANUM, self_p_data, self_p_sum);
    self_p_mean[AXIS_X] = self_p_sum[AXIS_X] / SELFCHK_DATANUM;
    self_p_mean[AXIS_Y] = self_p_sum[AXIS_Y] / SELFCHK_DATANUM;
    self_p_mean[AXIS_Z] = self_p_sum[AXIS_Z] / SELFCHK_DATANUM;

    sensor_enable( SENSOR_GYRO, &gyro_poll_info, false );
    msleep(20);
    sensor_set_selfchk_config(SENSOR_GYRO, SELFCHK_MINUS);
    msleep(60);
    sensor_enable( SENSOR_GYRO, &gyro_poll_info, true );
    msleep(800);
    gyro_get_selfchk_plusminus_data(SELFCHK_DATANUM, self_m_data, self_m_sum);
    self_m_mean[AXIS_X] = self_m_sum[AXIS_X] / SELFCHK_DATANUM;
    self_m_mean[AXIS_Y] = self_m_sum[AXIS_Y] / SELFCHK_DATANUM;
    self_m_mean[AXIS_Z] = self_m_sum[AXIS_Z] / SELFCHK_DATANUM;
    sensor_enable( SENSOR_GYRO, &gyro_poll_info, false );

    result[NORMAL_X] = gyro_judge_selfchk_result(abs(normal_mean[AXIS_X]),
                                                SELFCHK_NORMAL_LLIMIT,
                                                SELFCHK_NORMAL_ULIMIT);

    result[NORMAL_Y] = gyro_judge_selfchk_result(abs(normal_mean[AXIS_Y]),
                                                SELFCHK_NORMAL_LLIMIT,
                                                SELFCHK_NORMAL_ULIMIT);

    result[NORMAL_Z] = gyro_judge_selfchk_result(abs(normal_mean[AXIS_Z]),
                                                SELFCHK_NORMAL_LLIMIT,
                                                SELFCHK_NORMAL_ULIMIT);

    tmp = abs(normal_mean[AXIS_X] - self_p_mean[AXIS_X]);
    result[NORMAL_X_SELFP_X] = gyro_judge_selfchk_result(tmp,
                                                        SELFCHK_DIFF_SELFP_LLIMIT,
                                                        SELFCHK_DIFF_SELFP_ULIMIT);

    tmp = abs(normal_mean[AXIS_Y] - self_p_mean[AXIS_Y]);
    result[NORMAL_Y_SELFP_Y] = gyro_judge_selfchk_result(tmp,
                                                        SELFCHK_DIFF_SELFP_LLIMIT,
                                                        SELFCHK_DIFF_SELFP_ULIMIT);

    tmp = abs(normal_mean[AXIS_Z] - self_p_mean[AXIS_Z]);
    result[NORMAL_Z_SELFP_Z] = gyro_judge_selfchk_result(tmp,
                                                        SELFCHK_DIFF_SELFP_LLIMIT,
                                                        SELFCHK_DIFF_SELFP_ULIMIT);

    tmp = abs(normal_mean[AXIS_X] - self_m_mean[AXIS_X]);
    result[NORMAL_X_SELFM_X] = gyro_judge_selfchk_result(tmp,
                                                        SELFCHK_DIFF_SELFM_LLIMIT,
                                                        SELFCHK_DIFF_SELFM_ULIMIT);

    tmp = abs(normal_mean[AXIS_Y] - self_m_mean[AXIS_Y]);
    result[NORMAL_Y_SELFM_Y] = gyro_judge_selfchk_result(tmp,
                                                        SELFCHK_DIFF_SELFM_LLIMIT,
                                                        SELFCHK_DIFF_SELFM_ULIMIT);

    tmp = abs(normal_mean[AXIS_Z] - self_m_mean[AXIS_Z]);
    result[NORMAL_Z_SELFM_Z] = gyro_judge_selfchk_result(tmp,
                                                        SELFCHK_DIFF_SELFM_LLIMIT,
                                                        SELFCHK_DIFF_SELFM_ULIMIT);

    count += gyro_selfchk_make_output_result(result, normal_mean, self_p_mean, self_m_mean, buf);
    count += gyro_selfchk_make_output_alldata(normal_data, self_p_data, self_m_data, buf+count);

    SENSOR_D_LOG("end");
    return count;
}

static void gyro_selfchk_advance_preparation(void)
{
    SENSOR_D_LOG("start");
    if (SENSOR_ON == sensor_get_status(SENSOR_GYRO)){
        sensor_enable( SENSOR_GYRO, &gyro_poll_info, false );
    }
    sensor_set_dynamiccalib_enable(SENSOR_GYRO, false);
    sensor_set_poll_time( SENSOR_GYRO, &gyro_poll_info, 5);
    sensor_enable( SENSOR_GYRO, &gyro_poll_info, true );

    SENSOR_D_LOG("end");
}

static ssize_t gyro_selfcheck_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t count = 0;
    SENSOR_D_LOG("start");

    gyro_selfchk_advance_preparation();
    count += gyro_selfchk_exec(buf);
    SENSOR_D_LOG("end");
    return count;
}

static void gyro_calib_advance_preparation(void)
{
    SENSOR_D_LOG("start");
    if (SENSOR_ON == sensor_get_status(SENSOR_GYRO)){
        sensor_enable( SENSOR_GYRO, &gyro_poll_info, false );
    }
    sensor_set_poll_time( SENSOR_GYRO, &gyro_poll_info, FASTEST_PERIOD);
    SENSOR_D_LOG("end");
}

static ssize_t gyro_calib_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int32_t result = 0;
    ssize_t count = 0;
    struct gyroscope *offsets;
    SENSOR_D_LOG("start");
    result = atomic_read(&man_cal_result);
    offsets = (struct gyroscope*)kzalloc(sizeof(struct gyroscope), GFP_KERNEL);
    if(result == SUCCESS){
        sensor_com_mutex_lock();
        sensor_get_offset_value(SENSOR_GYRO, (void *)offsets);
        sensor_com_mutex_unlock();
        count += sprintf(buf, "[SUCCESS]\n");
        count += sprintf(buf+count, "%d %d %d\n",
                                    offsets->x,
                                    offsets->y,
                                    offsets->z);
    } else {
        count += sprintf(buf, "[FAILURE]\n");
        count += sprintf(buf+count, "%d\n", result);
    }
    SENSOR_D_LOG("end");
    return count;
}

static ssize_t gyro_calib_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    int8_t ret = SNS_RC_OK;
    int32_t tmp_param[GYROCAL_PRM_MAX] = {0};
    uint8_t  cal_param[GYROCAL_PRM_MAX] = {0};
    int8_t i;
    int8_t cal_result = 0;
    SENSOR_D_LOG("start");
    sscanf(buf, "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                &tmp_param[0],
                &tmp_param[1],
                &tmp_param[2],
                &tmp_param[3],
                &tmp_param[4],
                &tmp_param[5],
                &tmp_param[6],
                &tmp_param[7],
                &tmp_param[8],
                &tmp_param[9],
                &tmp_param[10],
                &tmp_param[11],
                &tmp_param[12],
                &tmp_param[13]
                );

    for(i = 0; i < GYROCAL_PRM_MAX; i++){
        cal_param[i] = (uint8_t)(tmp_param[i] & 0x000000FF);
        SENSOR_ERR_LOG("cal_param[%d]=0x%02x", i, cal_param[i]);
    }

    gyro_calib_advance_preparation();
    ret = sensor_start_manual_calib(SENSOR_GYRO, cal_param, SETTING);
    if(ret){
        SENSOR_ERR_LOG("manual calibration setting failed.");
        cal_result = ERR_OTHER;
        goto cal_end;
    }
    sensor_enable( SENSOR_GYRO, &gyro_poll_info, true );
    cal_result = sensor_start_manual_calib(SENSOR_GYRO, NULL, WAIT);
    if(cal_result == SUCCESS){
        sensor_enable( SENSOR_GYRO, &gyro_poll_info, false );
    }

cal_end:
    atomic_set(&man_cal_result, cal_result);
    SENSOR_D_LOG("end");
    return count;
}

void gyro_save_cal_ofs_val(struct gyroscope *savedata)
{
    SENSOR_N_LOG("start");
    saved_ofs_val.x = savedata->x;
    saved_ofs_val.y = savedata->y;
    saved_ofs_val.z = savedata->z;
    SENSOR_N_LOG("end");
}

void gyro_load_cal_ofs_val(struct gyroscope* outdata)
{
    SENSOR_N_LOG("start");
    outdata->x = saved_ofs_val.x;
    outdata->y = saved_ofs_val.y;
    outdata->z = saved_ofs_val.z;
    SENSOR_N_LOG("end");
}


static void gyro_update_last_read_data(void)
{
    unsigned long enable = 0;
    int32_t ret = 0;
    union sensor_read_data_u read_data;

    enable = sensor_get_status(SENSOR_GYRO);

    if (enable) {
        ret = sensor_type_get_data(SENSOR_GYRO, &read_data);
        if (0 == ret) {
            memcpy(&gyro_last_read_data,&(read_data.gyro_data),sizeof(read_data.gyro_data));
        }
    }
}

static void gyro_poll_work_func(struct work_struct *work)
{
    SENSOR_N_LOG("start");

    if(sns_get_reset_status() == false){
        sns_iio_report_event_now(SENSOR_GYRO);
    }

    SENSOR_N_LOG("end");
    return;
}

static void gyro_set_input_params( struct input_dev *dev )
{
    SENSOR_N_LOG("start");

    if(!dev){
        SENSOR_ERR_LOG("bad parm --> dev is NULL");
        return;
    }

    dev->name = "gyroscope";
    dev->id.bustype = BUS_SPI;

    set_bit(EV_ABS, dev->evbit);
    input_set_abs_params(dev, ABS_X, ABSMIN_GYRO, ABSMAX_GYRO, 0, 0);
    input_set_abs_params(dev, ABS_Y, ABSMIN_GYRO, ABSMAX_GYRO, 0, 0);
    input_set_abs_params(dev, ABS_Z, ABSMIN_GYRO, ABSMAX_GYRO, 0, 0);
    input_set_abs_params(dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);

    SENSOR_N_LOG("end");
    return;
}

void gyro_driver_init( void )
{
    int ret = 0;

    SENSOR_N_LOG("start");

    ret = sensor_input_init( &gyro_input_info );
    SENSOR_N_LOG("sensor_input_init()-->ret[%d] dev[%p]",
                  ret, gyro_input_info.dev );

    if( (0 != ret) || (NULL == (gyro_input_info.dev))) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        SENSOR_ERR_LOG("end return[%d]",-ENODEV);
        return;
    }

    sensor_poll_init(SENSOR_GYRO,&gyro_poll_info);

    SENSOR_N_LOG("end");
    return;
}

EXPORT_SYMBOL(gyro_driver_init);

void gyro_driver_exit( void )
{

}

EXPORT_SYMBOL(gyro_driver_exit);

