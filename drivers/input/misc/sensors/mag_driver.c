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

#define DEFAULT_MAG_DELAY   (30)
#define ABSMAX_MAG          (32767)
#define ABSMIN_MAG          (-32767)

struct mag_vector_str {
    int32_t v[3];
};

struct mag_offset_str {
    struct mag_vector_str calib_offset;
};

struct mag_matrix_str {
    int32_t matrix[9];
};

static ssize_t mag_batch_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t mag_batch_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t mag_flush_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t mag_get_raw_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t mag_raw_ellipsoid_mode_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t mag_raw_ellipsoid_mode_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t mag_raw_dynamic_matrix_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t mag_raw_dynamic_matrix_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t mag_raw_static_matrix_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t mag_raw_static_matrix_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t mag_raw_offsets_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t mag_raw_offsets_store(struct device *dev,
   struct device_attribute *attr,
   const char *buf,
   size_t count );
static ssize_t mag_raw_shape_show(struct device *dev,
   struct device_attribute *attr,
   char *buf );
static ssize_t mag_raw_shape_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t mag_raw_distortion_show(struct device *dev,
   struct device_attribute *attr,
   char *buf );
static ssize_t mag_raw_distortion_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t mag_raw_threshold_show(struct device *dev,
   struct device_attribute *attr,
   char *buf );
static ssize_t mag_raw_threshold_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t mag_filter_threshold_show(struct device *dev,
   struct device_attribute *attr,
   char *buf );
static ssize_t mag_filter_threshold_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t mag_position_show(struct device *dev,
   struct device_attribute *attr,
   char *buf );
static ssize_t mag_position_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t mag_filter_noise_show(struct device *dev,
   struct device_attribute *attr,
   char *buf );
static ssize_t mag_filter_noise_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t mag_filter_len_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t mag_filter_len_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t mag_filter_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t mag_filter_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf, size_t count );
static ssize_t mag_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t mag_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t mag_delay_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t mag_delay_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t mag_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t mag_status_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t mag_status_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t mag_accuracy_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t mag_accuracy_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t mag_device_id_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf );

static void mag_update_last_read_data(void);
static void mag_poll_work_func(struct work_struct *work);
static void mag_set_input_params( struct input_dev *dev );


static DEVICE_ATTR(enable,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    mag_enable_show,
    mag_enable_store
);
static DEVICE_ATTR(delay,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    mag_delay_show,
    mag_delay_store
);
static DEVICE_ATTR(status,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    mag_status_show,
    mag_status_store
);
static DEVICE_ATTR(data,
    S_IRUSR|S_IRGRP,
    mag_data_show,
    NULL
);
static DEVICE_ATTR(filter_enable,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    mag_filter_enable_show,
    mag_filter_enable_store
);
static DEVICE_ATTR(filter_len,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    mag_filter_len_show,
    mag_filter_len_store
);
static DEVICE_ATTR(filter_noise,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    mag_filter_noise_show,
    mag_filter_noise_store
);
static DEVICE_ATTR(position,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    mag_position_show,
    mag_position_store
);
static DEVICE_ATTR(filter_threshold,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    mag_filter_threshold_show,
    mag_filter_threshold_store
);
static DEVICE_ATTR(threshold,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    mag_raw_threshold_show,
    mag_raw_threshold_store
);
static DEVICE_ATTR(distortion,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    mag_raw_distortion_show,
    mag_raw_distortion_store
);
static DEVICE_ATTR(shape,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    mag_raw_shape_show,
    mag_raw_shape_store
);
static DEVICE_ATTR(offsets,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    mag_raw_offsets_show,
    mag_raw_offsets_store
);
static DEVICE_ATTR(static_matrix,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    mag_raw_static_matrix_show,
    mag_raw_static_matrix_store
);
static DEVICE_ATTR(dynamic_matrix,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    mag_raw_dynamic_matrix_show,
    mag_raw_dynamic_matrix_store
);
static DEVICE_ATTR(ellipsoid_mode,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    mag_raw_ellipsoid_mode_show,
    mag_raw_ellipsoid_mode_store
);
static DEVICE_ATTR(raw_data,
    S_IRUSR|S_IRGRP,
    mag_get_raw_data_show,
    NULL
);
static DEVICE_ATTR(batch,
    S_IWUSR|S_IWGRP,
    NULL,
    mag_batch_store
);
static DEVICE_ATTR(batch_data,
    S_IRUSR|S_IRGRP,
    mag_batch_data_show,
    NULL
);
static DEVICE_ATTR(flush,
    S_IWUSR|S_IWGRP,
    NULL,
    mag_flush_store
);
static DEVICE_ATTR(accuracy,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    mag_accuracy_show,
    mag_accuracy_store
);
static DEVICE_ATTR(device_id,
    S_IRUSR|S_IRGRP,
    mag_device_id_show,
    NULL
);

static struct attribute *mag_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_delay.attr,
    &dev_attr_status.attr,
    &dev_attr_data.attr,
    &dev_attr_filter_enable.attr,
    &dev_attr_filter_len.attr,
    &dev_attr_filter_noise.attr,
    &dev_attr_filter_threshold.attr,
    &dev_attr_position.attr,
    &dev_attr_threshold.attr,
    &dev_attr_distortion.attr,
    &dev_attr_shape.attr,
    &dev_attr_offsets.attr,
    &dev_attr_static_matrix.attr,
    &dev_attr_dynamic_matrix.attr,
    &dev_attr_ellipsoid_mode.attr,
    &dev_attr_raw_data.attr,
    &dev_attr_batch.attr,
    &dev_attr_batch_data.attr,
    &dev_attr_flush.attr,
    &dev_attr_accuracy.attr,
    &dev_attr_device_id.attr,
    NULL
};

static struct attribute_group mag_attr_grp = {
    .attrs = mag_attributes
};

struct sensor_input_info_str mag_input_info =
{
    NULL,
    mag_set_input_params,
    &mag_attr_grp,
};

struct sensor_poll_info_str mag_poll_info = {
    .name       = "mag_poll_wq",
    .poll_time  = ATOMIC_INIT(DEFAULT_MAG_DELAY),
    .poll_func  = mag_poll_work_func,
};

static struct geomagnetic mag_last_read_data = {
    0,0,0
};


static int mag_filter_enable =0;
static int mag_filter_len =0;
static int32_t mag_filter_noise[3];
static int mag_position = 0;
static int32_t mag_filter_threshold =0;
static int mag_threshold =0;
static int32_t mag_distortion[3];
static int mag_shape = 0;
static struct mag_offset_str mag_offset;
static int mag_last_status = 0;
static struct mag_matrix_str mag_static_matrix;
static struct mag_matrix_str mag_dynamic_matrix;
static int mag_ellipsoid_mode =0;

static struct sens_ofs saved_ofs_val = {0};
static struct sensor_batch_data_str mag_batch_data;
static uint32_t g_time_stamp_mag      = 0;
static uint32_t g_input_num_mag       = 0;
extern struct mutex sensor_batch_mutex;

static ssize_t mag_batch_store(struct device *dev,
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

    ret = sensor_set_batch( SENSOR_MAG, batch_info);

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t mag_batch_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret =0;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_batch_mutex);

    ret = sprintf(buf, "%d %d %d %d\n",mag_batch_data.payload_size,
                       mag_batch_data.recode_num, g_input_num_mag, g_time_stamp_mag);

    g_time_stamp_mag = 0;

    mutex_unlock(&sensor_batch_mutex);

    SENSOR_N_LOG("end");

    return ret;
}

void mag_ring_buffer_timestamp(
    uint32_t time_stamp_mag)
{
    SENSOR_N_LOG("start");
    g_time_stamp_mag = time_stamp_mag;
    SENSOR_N_LOG("end %d",g_time_stamp_mag);
}

void mag_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data )
{
    
    SENSOR_N_LOG("start");

    mag_batch_data = batch_data;

    sensor_report_batch( mag_input_info.dev,
                         repo_type,
                         mag_batch_data );

    SENSOR_N_LOG("end");
    return;
}

void mag_timestamp_report(void)
{
    SENSOR_N_LOG("start");

    sensor_report_batch( mag_input_info.dev,
                         SENSOR_COMP_TIME,
                         mag_batch_data );

    SENSOR_N_LOG("end");
    return;
}

static ssize_t mag_flush_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    uint32_t ret = 0;

    SENSOR_N_LOG("start");

    ret = sensor_set_flush(SENSOR_MAG, mag_input_info.dev);

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t mag_device_id_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t ret =0;
    uint8_t id;

    SENSOR_D_LOG("start");
    sensor_com_mutex_lock();
    id = sensor_get_device_id(SENSOR_MAG);
    sensor_com_mutex_unlock();
    ret = sprintf(buf,"0x%02x\n", id);
    SENSOR_D_LOG("end");

    return ret;
}

static ssize_t mag_accuracy_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int8_t accuracy;
    ssize_t ret;

    SENSOR_N_LOG("start");

    sensor_com_mutex_lock();
    ret = sns_mag_get_accuracy(&accuracy);
    sensor_com_mutex_unlock();

    SENSOR_N_LOG("sns_mag_get_accuracy ret[%d]",(int)ret);
    SENSOR_N_LOG("accuracy[%d]",accuracy);

    ret = sprintf( buf, "%d\n", accuracy );

    SENSOR_N_LOG("end");

    return ret;
}

static ssize_t mag_accuracy_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    int32_t accuracy;
    int32_t ret;

    SENSOR_N_LOG("start");

    sscanf(buf, "%d", &accuracy );
    SENSOR_N_LOG("accuracy[%d]",accuracy);

    if (0 <= accuracy && accuracy <= 3) {
        sensor_com_mutex_lock();
        ret = sns_mag_set_accuracy((int8_t)accuracy);
        sensor_com_mutex_unlock();
    }

    SENSOR_N_LOG("end");

    return count;
}

static ssize_t mag_get_raw_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int32_t gget_data_ret = 0;
    ssize_t ret;
    union sensor_read_data_u mag_read_data;
    union sensor_read_data_u uncal_mag_read_data;

    SENSOR_N_LOG("start");

    gget_data_ret = sensor_type_get_data( SENSOR_MAG, &mag_read_data);
    SENSOR_N_LOG("sensor_type_get_data() SENSOR_MAG ret[%d]",gget_data_ret);
    SENSOR_N_LOG("read_data: X[%d] Y[%d] Z[%d]",
                 (int)mag_read_data.mag_data.x,
                 (int)mag_read_data.mag_data.y,
                 (int)mag_read_data.mag_data.z);

    gget_data_ret = sensor_type_get_data( SENSOR_MAG_UNCAL, &uncal_mag_read_data);
    SENSOR_N_LOG("sensor_type_get_data() SENSOR_MAG_UNCAL ret[%d]",gget_data_ret);
    SENSOR_N_LOG("read_data: X[%d] Y[%d] Z[%d]",
                 (int)uncal_mag_read_data.mag_data.x,
                 (int)uncal_mag_read_data.mag_data.y,
                 (int)uncal_mag_read_data.mag_data.z);

    ret = sprintf( buf, "%d %d %d %d %d %d\n",
                   mag_read_data.mag_data.x,
                   mag_read_data.mag_data.y,
                   mag_read_data.mag_data.z,
                   uncal_mag_read_data.mag_data.x,
                   uncal_mag_read_data.mag_data.y,
                   uncal_mag_read_data.mag_data.z );

    SENSOR_N_LOG("end");

    return ret;
}

static ssize_t mag_raw_ellipsoid_mode_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t ret;
    SENSOR_N_LOG("start");

    ret = sprintf(buf, "%d\n", mag_ellipsoid_mode);

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t mag_raw_ellipsoid_mode_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    long value;

    SENSOR_N_LOG("start");

    if (kstrtol(buf, 10, &value) >= 0) {
        mag_ellipsoid_mode = value;
    }

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t mag_raw_dynamic_matrix_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t ret;
    SENSOR_N_LOG("start");

    ret = sprintf( buf, "%d %d %d %d %d %d %d %d %d\n",
                   mag_dynamic_matrix.matrix[0],
                   mag_dynamic_matrix.matrix[1],
                   mag_dynamic_matrix.matrix[2],
                   mag_dynamic_matrix.matrix[3],
                   mag_dynamic_matrix.matrix[4],
                   mag_dynamic_matrix.matrix[5],
                   mag_dynamic_matrix.matrix[6],
                   mag_dynamic_matrix.matrix[7],
                   mag_dynamic_matrix.matrix[8] );

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t mag_raw_dynamic_matrix_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    SENSOR_N_LOG("start");

    sscanf (buf, "%d %d %d %d %d %d %d %d %d",
            &mag_dynamic_matrix.matrix[0],
            &mag_dynamic_matrix.matrix[1],
            &mag_dynamic_matrix.matrix[2],
            &mag_dynamic_matrix.matrix[3],
            &mag_dynamic_matrix.matrix[4],
            &mag_dynamic_matrix.matrix[5],
            &mag_dynamic_matrix.matrix[6],
            &mag_dynamic_matrix.matrix[7],
            &mag_dynamic_matrix.matrix[8] );

    SENSOR_N_LOG("end");

	return count;
}

static ssize_t mag_raw_static_matrix_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t ret;
    int32_t matrix[9];

    SENSOR_N_LOG("start");

    sensor_com_mutex_lock();
    ret = sns_mag_get_static_matrix(matrix);
    sensor_com_mutex_unlock();
    if(ret == SNS_RC_OK) {
        mag_static_matrix.matrix[0] = matrix[0];
        mag_static_matrix.matrix[1] = matrix[1];
        mag_static_matrix.matrix[2] = matrix[2];
        mag_static_matrix.matrix[3] = matrix[3];
        mag_static_matrix.matrix[4] = matrix[4];
        mag_static_matrix.matrix[5] = matrix[5];
        mag_static_matrix.matrix[6] = matrix[6];
        mag_static_matrix.matrix[7] = matrix[7];
        mag_static_matrix.matrix[8] = matrix[8];
    }

    ret = sprintf( buf, "%d %d %d %d %d %d %d %d %d\n",
                   mag_static_matrix.matrix[0],
                   mag_static_matrix.matrix[1],
                   mag_static_matrix.matrix[2],
                   mag_static_matrix.matrix[3],
                   mag_static_matrix.matrix[4],
                   mag_static_matrix.matrix[5],
                   mag_static_matrix.matrix[6],
                   mag_static_matrix.matrix[7],
                   mag_static_matrix.matrix[8] );
    SENSOR_N_LOG("read_data: mat00[%d] mat01[%d] mat02[%d] mat03[%d] mat04[%d] mat05[%d] mat06[%d] mat07[%d] mat08[%d]",
                 mag_static_matrix.matrix[0],
                 mag_static_matrix.matrix[1],
                 mag_static_matrix.matrix[2],
                 mag_static_matrix.matrix[3],
                 mag_static_matrix.matrix[4],
                 mag_static_matrix.matrix[5],
                 mag_static_matrix.matrix[6],
                 mag_static_matrix.matrix[7],
                 mag_static_matrix.matrix[8] );

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t mag_raw_static_matrix_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    ssize_t ret;
    SENSOR_N_LOG("start");

    sscanf(buf, "%d %d %d %d %d %d %d %d %d",
            &mag_static_matrix.matrix[0],
            &mag_static_matrix.matrix[1],
            &mag_static_matrix.matrix[2],
            &mag_static_matrix.matrix[3],
            &mag_static_matrix.matrix[4],
            &mag_static_matrix.matrix[5],
            &mag_static_matrix.matrix[6],
            &mag_static_matrix.matrix[7],
            &mag_static_matrix.matrix[8] );
    sensor_com_mutex_lock();
    ret = sns_mag_set_static_matrix(mag_static_matrix.matrix);
    sensor_com_mutex_unlock();

    SENSOR_N_LOG("end");

    return count;
}

static ssize_t mag_raw_offsets_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t ret;
    struct geomagnetic *offsets;
    SENSOR_N_LOG("start");

    offsets = (struct geomagnetic *)kzalloc(sizeof(struct geomagnetic), GFP_KERNEL);
    sensor_com_mutex_lock();
    ret = sensor_get_offset_value(SENSOR_MAG, offsets);
    sensor_com_mutex_unlock();

    if(ret == SNS_RC_OK) {
        mag_offset.calib_offset.v[0] = offsets->x;
        mag_offset.calib_offset.v[1] = offsets->y;
        mag_offset.calib_offset.v[2] = offsets->z;
        mag_last_status = offsets->accuracy;
    }
    kfree(offsets);

    ret = sprintf( buf, "%d %d %d %d %d %d %d\n",
                   0, 0, 0,
                   mag_offset.calib_offset.v[0],
                   mag_offset.calib_offset.v[1],
                   mag_offset.calib_offset.v[2],
                   mag_last_status);

    SENSOR_N_LOG("read_data: CO1[%d] CO2[%d] CO3[%d] acc[%d]",
                 mag_offset.calib_offset.v[0],
                 mag_offset.calib_offset.v[1],
                 mag_offset.calib_offset.v[2],
                 mag_last_status);

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t mag_raw_offsets_store(struct device *dev,
   struct device_attribute *attr,
   const char *buf,
    size_t count )
{
    int32_t ret;
    int accuracy = -1;
    struct geomagnetic mag_cal_offset = {0};
    int32_t dummy;

    SENSOR_N_LOG("start");

    sscanf(buf, "%d %d %d %d %d %d %d",
            &dummy,&dummy,&dummy,
            &mag_offset.calib_offset.v[0],
            &mag_offset.calib_offset.v[1],
            &mag_offset.calib_offset.v[2],
            &accuracy);

    if (0 <= accuracy && accuracy <= 3) {
        mag_last_status = accuracy;
    }
    mag_cal_offset.x = mag_offset.calib_offset.v[0];
    mag_cal_offset.y = mag_offset.calib_offset.v[1];
    mag_cal_offset.z = mag_offset.calib_offset.v[2];
    mag_cal_offset.accuracy = accuracy;
    mag_save_cal_ofs_val(&mag_cal_offset);

    sensor_com_mutex_lock();
    ret = sensor_set_offset_value(SENSOR_MAG, saved_ofs_val);
    sensor_com_mutex_unlock();

    SENSOR_N_LOG("end");

    return count;
}

static ssize_t mag_raw_shape_show(struct device *dev,
   struct device_attribute *attr,
   char *buf )
{
    ssize_t ret;
    SENSOR_N_LOG("start");

    ret = sprintf(buf, "%d\n",mag_shape);

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t mag_raw_shape_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    long value;
    SENSOR_N_LOG("start");

    if (kstrtol(buf, 10, &value) >= 0) {
        mag_shape = value;
        SENSOR_A_LOG("ster mag_shape[%d]",(int)mag_shape);
    }

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t mag_raw_distortion_show(struct device *dev,
   struct device_attribute *attr,
   char *buf )
{
    ssize_t ret;
    SENSOR_N_LOG("start");

    ret = sprintf(buf, "%d %d %d\n",
                   mag_distortion[0],
                   mag_distortion[1],
                   mag_distortion[2] );

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t mag_raw_distortion_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    SENSOR_N_LOG("start");

    sscanf( buf, "%d %d %d",
             &mag_distortion[0],
             &mag_distortion[1],
             &mag_distortion[2] );

    SENSOR_N_LOG("end");
   return count;
}

static ssize_t mag_raw_threshold_show(struct device *dev,
   struct device_attribute *attr,
   char *buf )
{
    ssize_t ret;
    SENSOR_N_LOG("start");

    ret = sprintf(buf, "%d\n",mag_threshold);

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t mag_raw_threshold_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    long value;
    SENSOR_N_LOG("start");

    if (kstrtol(buf, 10, &value) >= 0) {
        mag_threshold = value;
        SENSOR_A_LOG("ster mag_threshold[%d]",(int)mag_threshold);
    }

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t mag_filter_threshold_show(struct device *dev,
   struct device_attribute *attr,
   char *buf )
{
    ssize_t ret;
    SENSOR_N_LOG("start");

    ret = sprintf(buf, "%d\n",mag_filter_threshold);

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t mag_filter_threshold_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    long value;
    SENSOR_N_LOG("start");

    if (kstrtol(buf, 10, &value) >= 0) {
        mag_filter_threshold = value;
        SENSOR_A_LOG("ster mag_filter_threshold[%d]",(int)mag_filter_threshold);
    }

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t mag_position_show(struct device *dev,
   struct device_attribute *attr,
   char *buf )
{
    ssize_t ret;
    SENSOR_N_LOG("start");

    ret = sprintf(buf, "%d\n",mag_position);

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t mag_position_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    long value;
    SENSOR_N_LOG("start");

    if (kstrtol(buf, 10, &value) >= 0) {
        mag_position = value;
        SENSOR_A_LOG("ster mag_position[%d]",(int)mag_position);
    }

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t mag_filter_noise_show(struct device *dev,
   struct device_attribute *attr,
   char *buf )
{
    ssize_t ret;
    SENSOR_N_LOG("start");

    ret = sprintf(buf, "%d %d %d\n",
                   mag_filter_noise[0],
                   mag_filter_noise[1],
                   mag_filter_noise[2] );

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t mag_filter_noise_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    SENSOR_N_LOG("start");

    sscanf( buf, "%d %d %d",
             &mag_filter_noise[0],
             &mag_filter_noise[1],
             &mag_filter_noise[2] );

    SENSOR_N_LOG("end");
   return count;
}

static ssize_t mag_filter_len_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t ret;
    SENSOR_N_LOG("start");

    ret = sprintf(buf, "%d\n", mag_filter_len);

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t mag_filter_len_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf, 
    size_t count)
{
    long value;
    SENSOR_N_LOG("start");

    if (kstrtol(buf, 10, &value) >= 0) {
        mag_filter_len = value;
        SENSOR_A_LOG("ster mag_filter_len[%d]",(int)mag_filter_len);
    }

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t mag_filter_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t ret;
    SENSOR_N_LOG("start");

    ret = sprintf(buf, "%d\n", mag_filter_enable);

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t mag_filter_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf, size_t count )
{
	long value;

    SENSOR_N_LOG("start");

    if (kstrtol(buf, 10, &value) >= 0) {
        mag_filter_enable = value;
        SENSOR_A_LOG("ster mag_filter_enable[%d]",(int)mag_filter_enable);
    }

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t mag_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int enable = 0;
    SENSOR_N_LOG("start");

    enable = sensor_get_status(SENSOR_MAG);

    SENSOR_N_LOG("end ->enable[%d]",enable);
    return sprintf(buf, "%d\n", enable);
}

static ssize_t mag_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    unsigned long enable = 0;
    int ret = 0;

    SENSOR_N_LOG("start");

    ret = kstrtoul(buf, 10, &enable);
    SENSOR_N_LOG("kstrtoul() ret[%d]->enable[%d]",ret, (int)enable);

    sensor_enable( SENSOR_MAG, &mag_poll_info, (bool)enable );

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t mag_delay_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int32_t delay = 0;
    SENSOR_N_LOG("start");
    delay = atomic_read(&(mag_poll_info.poll_time));
    SENSOR_N_LOG("end ->delay[%d]",(int)delay);
    return sprintf(buf, "%d\n", delay);
}

static ssize_t mag_delay_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    unsigned long delay = 0;
    int ret = 0;
    SENSOR_N_LOG("start");

    ret = kstrtoul(buf, 10, &delay);
    SENSOR_N_LOG("kstrtoul() ret[%d]->delay[%d]",ret, (int)delay);

    sensor_set_poll_time( SENSOR_MAG, &mag_poll_info, (int32_t)delay);

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t mag_status_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int rt = 0;
    SENSOR_N_LOG("start");

    rt = sensor_get_status(SENSOR_MAG);

    SENSOR_N_LOG("end");
    return sprintf(buf, "%d\n", rt);
}

static ssize_t mag_status_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    int ret = 0;
    unsigned long status = 0;
    SENSOR_N_LOG("start");

    ret = kstrtoul(buf, 10, &status);
    SENSOR_N_LOG("kstrtoul() ret[%d]->status[%d]",ret, (int)status);

    sensor_set_status(SENSOR_MAG, status);

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t mag_data_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret;
    SENSOR_N_LOG("start");
    mag_update_last_read_data();

    ret = sprintf( buf, "%d %d %d\n",
                    mag_last_read_data.x,
                    mag_last_read_data.y,
                    mag_last_read_data.z );

    SENSOR_N_LOG("read_data: X[%d] Y[%d] Z[%d]",
                    (int)mag_last_read_data.x,
                    (int)mag_last_read_data.y,
                    (int)mag_last_read_data.z );

    SENSOR_N_LOG("end");
    return ret;
}

void mag_save_cal_ofs_val(struct geomagnetic *savedata)
{
    SENSOR_N_LOG("start");
    saved_ofs_val.x = savedata->x;
    saved_ofs_val.y = savedata->y;
    saved_ofs_val.z = savedata->z;
    saved_ofs_val.accuracy = savedata->accuracy;
    SENSOR_N_LOG("end");
}

void mag_load_cal_ofs_val(struct geomagnetic* outdata)
{
    SENSOR_N_LOG("start");
    outdata->x = saved_ofs_val.x;
    outdata->y = saved_ofs_val.y;
    outdata->z = saved_ofs_val.z;
    outdata->accuracy = saved_ofs_val.accuracy;
    SENSOR_N_LOG("end");
}

static void mag_update_last_read_data(void)
{
    unsigned long enable = 0;
    int32_t ret = 0;
    union sensor_read_data_u read_data;

    enable = sensor_get_status(SENSOR_MAG);

    if (enable) {
        ret = sensor_type_get_data(SENSOR_MAG, &read_data);
        if (0 == ret) {
            memcpy(&mag_last_read_data,&(read_data.mag_data),sizeof(read_data.mag_data));
        }
    }
}

static void mag_poll_work_func(struct work_struct *work)
{
    SENSOR_N_LOG("start");

    if(sns_get_reset_status() == false){
        sns_iio_report_event_now(SENSOR_MAG);
    }

    SENSOR_N_LOG("end");
    return;
}

static void mag_set_input_params( struct input_dev *dev )
{
    SENSOR_N_LOG("start");

    if(!dev){
        SENSOR_ERR_LOG("bad parm --> dev is NULL");
        return;
    }

    dev->name = "geomagnetic";
    dev->id.bustype = BUS_SPI;

    set_bit(EV_ABS, dev->evbit);
    input_set_abs_params(dev, ABS_X, ABSMIN_MAG, ABSMAX_MAG, 0, 0);
    input_set_abs_params(dev, ABS_Y, ABSMIN_MAG, ABSMAX_MAG, 0, 0);
    input_set_abs_params(dev, ABS_Z, ABSMIN_MAG, ABSMAX_MAG, 0, 0);
    input_set_abs_params(dev, ABS_RUDDER, ABSMIN_MAG, ABSMAX_MAG, 0, 0);
    input_set_abs_params(dev, ABS_STATUS, 0, 3, 0, 0);
    input_set_abs_params(dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);

    SENSOR_N_LOG("end");
    return;
}

void mag_driver_init( void )
{
    int ret = 0;

    SENSOR_N_LOG("start");

    ret = sensor_input_init( &mag_input_info );
    SENSOR_N_LOG("sensor_input_init()-->ret[%d] dev[%p]",
                ret, mag_input_info.dev );

    if( (0 != ret) || (NULL == (mag_input_info.dev))) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        SENSOR_ERR_LOG("end return[%d]",-ENODEV);
        return;
    }

    sensor_poll_init(SENSOR_MAG,&mag_poll_info);

    SENSOR_N_LOG("end");
    return;
}

EXPORT_SYMBOL(mag_driver_init);

void mag_driver_exit( void )
{

}
EXPORT_SYMBOL(mag_driver_exit);
