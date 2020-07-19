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

#define DEFAULT_MAG_UNCAL_DELAY    (30)
#define ABSMAX_MAG                 (32767)
#define ABSMIN_MAG                 (-32767)

static ssize_t mag_uncal_batch_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t mag_uncal_batch_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t mag_uncal_flush_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t mag_uncal_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t mag_uncal_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t mag_uncal_delay_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t mag_uncal_delay_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t mag_uncal_status_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t mag_uncal_status_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t mag_uncal_data_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf);

static void mag_uncal_update_last_read_data(void);
static void mag_uncal_poll_work_func(struct work_struct *work);
static void mag_uncal_set_input_params( struct input_dev *dev );


static DEVICE_ATTR(enable,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    mag_uncal_enable_show,
    mag_uncal_enable_store
);
static DEVICE_ATTR(delay,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    mag_uncal_delay_show,
    mag_uncal_delay_store
);
static DEVICE_ATTR(status,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    mag_uncal_status_show,
    mag_uncal_status_store
);
static DEVICE_ATTR(data,
    S_IRUSR|S_IRGRP,
    mag_uncal_data_show,
    NULL
);
static DEVICE_ATTR(batch,
    S_IWUSR|S_IWGRP,
    NULL,
    mag_uncal_batch_store
);
static DEVICE_ATTR(batch_data,
    S_IRUSR|S_IRGRP,
    mag_uncal_batch_data_show,
    NULL
);
static DEVICE_ATTR(flush,
    S_IWUSR|S_IWGRP,
    NULL,
    mag_uncal_flush_store
);

static struct attribute *mag_uncal_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_delay.attr,
    &dev_attr_data.attr,
    &dev_attr_status.attr,
    &dev_attr_batch.attr,
    &dev_attr_batch_data.attr,
    &dev_attr_flush.attr,
    NULL
};

static struct attribute_group mag_uncal_attr_grp = {
    .attrs = mag_uncal_attributes
};

struct sensor_input_info_str mag_uncal_input_info =
{
    NULL,
    mag_uncal_set_input_params,
    &mag_uncal_attr_grp,
};

struct sensor_poll_info_str mag_uncal_poll_info = {
    .name       = "mag_uncal_poll_wq",
    .poll_time  = ATOMIC_INIT(DEFAULT_MAG_UNCAL_DELAY),
    .poll_func  = mag_uncal_poll_work_func,
};

static struct mag_uncalib mag_uncal_last_read_data = {
    0,0,0,0,0,0
};

static struct sensor_batch_data_str mag_uncal_batch_data;
static struct sensor_batch_data_str mag_uncal2_batch_data;
static uint32_t g_time_stamp_mag_uncal      = 0;
static uint32_t g_input_num_mag_uncal       = 0;
extern struct mutex sensor_batch_mutex;

static ssize_t mag_uncal_batch_store(struct device *dev,
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

    ret = sensor_set_batch( SENSOR_MAG_UNCAL, batch_info);

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t mag_uncal_batch_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret =0;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_batch_mutex);

    ret = sprintf(buf, "%d %d %d %d\n",mag_uncal_batch_data.payload_size,
                       mag_uncal_batch_data.recode_num,
                       g_input_num_mag_uncal, g_time_stamp_mag_uncal);

    g_time_stamp_mag_uncal = 0;

    mutex_unlock(&sensor_batch_mutex);

    SENSOR_N_LOG("end");

    return ret;
}

void mag_uncal_ring_buffer_timestamp(
    uint32_t time_stamp_mag_uncal)
{
    SENSOR_N_LOG("start");
    g_time_stamp_mag_uncal = time_stamp_mag_uncal;
    SENSOR_N_LOG("end %d",g_time_stamp_mag_uncal);
}

void mag_uncal_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data )
{
    SENSOR_N_LOG("start");

    mag_uncal_batch_data = batch_data;
    mag_uncal_batch_data.payload_size = 
             SENSOR_BATCH_REPORT_SIZE_MAG_UNCAL * mag_uncal_batch_data.recode_num;

    sensor_report_batch( mag_uncal_input_info.dev,
                         repo_type,
                         mag_uncal_batch_data );

    SENSOR_N_LOG("end");
    return;
}

void mag_uncal2_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data )
{
    SENSOR_N_LOG("start");

    mag_uncal2_batch_data = batch_data;

    SENSOR_N_LOG("end");
    return;
}

void mag_uncal_timestamp_report(void)
{
    SENSOR_N_LOG("start");

    sensor_report_batch( mag_uncal_input_info.dev,
                         SENSOR_COMP_TIME,
                         mag_uncal_batch_data );

    SENSOR_N_LOG("end");
    return;
}

static ssize_t mag_uncal_flush_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    uint32_t ret = 0;

    SENSOR_N_LOG("start");

    ret = sensor_set_flush(SENSOR_MAG_UNCAL, mag_uncal_input_info.dev);

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t mag_uncal_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int enable = 0;
    SENSOR_N_LOG("start");

    enable = sensor_get_status(SENSOR_MAG_UNCAL);

    SENSOR_N_LOG("end ->enable[%d]",enable);
    return sprintf(buf, "%d\n", enable);
}

static ssize_t mag_uncal_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    unsigned long enable = 0;
    int ret = 0;

    SENSOR_N_LOG("start");

    ret = kstrtoul(buf, 10, &enable);
    SENSOR_N_LOG("kstrtoul() ret[%d]->enable[%d]",ret, (int)enable);

    sensor_enable( SENSOR_MAG_UNCAL, &mag_uncal_poll_info, (bool)enable );

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t mag_uncal_delay_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int32_t delay = 0;
    SENSOR_N_LOG("start");
    delay = atomic_read(&(mag_uncal_poll_info.poll_time));
    SENSOR_N_LOG("end ->delay[%d]",(int)delay);
    return sprintf(buf, "%d\n", delay);
}

static ssize_t mag_uncal_delay_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    unsigned long delay = 0;
    int ret = 0;
    SENSOR_N_LOG("start");

    ret = kstrtoul(buf, 10, &delay);
    SENSOR_N_LOG("kstrtoul() ret[%d]->delay[%d]",ret, (int)delay);

    sensor_set_poll_time( SENSOR_MAG_UNCAL, &mag_uncal_poll_info, (int32_t)delay);

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t mag_uncal_status_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int rt = 0;
    SENSOR_N_LOG("start");

    rt = sensor_get_status(SENSOR_MAG_UNCAL);

    SENSOR_N_LOG("end");
    return sprintf(buf, "%d\n", rt);
}

static ssize_t mag_uncal_status_store(
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

    sensor_set_status(SENSOR_MAG_UNCAL, status);

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t mag_uncal_data_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret;
    SENSOR_N_LOG("start");
    mag_uncal_update_last_read_data();

    ret = sprintf( buf, "%d %d %d %d %d %d\n",
                   mag_uncal_last_read_data.x,
                   mag_uncal_last_read_data.y,
                   mag_uncal_last_read_data.z,
                   mag_uncal_last_read_data.cal_x,
                   mag_uncal_last_read_data.cal_y,
                   mag_uncal_last_read_data.cal_z );

    SENSOR_N_LOG("end");
    return ret;
}

static void mag_uncal_update_last_read_data(void)
{
    unsigned long enable = 0;
    int32_t ret = 0;
    union sensor_read_data_u read_data;

    enable = sensor_get_status(SENSOR_MAG_UNCAL);

    if (enable) {
        ret = sensor_type_get_data(SENSOR_MAG_UNCAL, &read_data);
        if (0 == ret) {
            memcpy(&mag_uncal_last_read_data,&(read_data.mag_uncal_data),
                   sizeof(read_data.mag_uncal_data));
        }
    }
}

static void mag_uncal_poll_work_func(struct work_struct *work)
{
    SENSOR_N_LOG("start");

    if(sns_get_reset_status() == false){
        sns_iio_report_event_now(SENSOR_MAG_UNCAL);
    }

    SENSOR_N_LOG("end");
    return;
}

static void mag_uncal_set_input_params( struct input_dev *dev )
{
    SENSOR_N_LOG("start");

    if(!dev){
        SENSOR_ERR_LOG("bad parm --> dev is NULL");
        return;
    }

    dev->name = "magnetic_field_uncalibrated";
    dev->id.bustype = BUS_SPI;

    set_bit(EV_ABS, dev->evbit);
    input_set_abs_params(dev, ABS_X, ABSMIN_MAG, ABSMAX_MAG, 0, 0);
    input_set_abs_params(dev, ABS_Y, ABSMIN_MAG, ABSMAX_MAG, 0, 0);
    input_set_abs_params(dev, ABS_Z, ABSMIN_MAG, ABSMAX_MAG, 0, 0);
    input_set_abs_params(dev, ABS_RX, ABSMIN_MAG, ABSMAX_MAG, 0, 0);
    input_set_abs_params(dev, ABS_RY, ABSMIN_MAG, ABSMAX_MAG, 0, 0);
    input_set_abs_params(dev, ABS_RZ, ABSMIN_MAG, ABSMAX_MAG, 0, 0);
    input_set_abs_params(dev, ABS_STATUS, 0, 3, 0, 0);
    input_set_abs_params(dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);

    SENSOR_N_LOG("end");
    return;
}

void mag_uncal_driver_init( void )
{
    int ret = 0;

    SENSOR_N_LOG("start");

    ret = sensor_input_init( &mag_uncal_input_info );
    SENSOR_N_LOG("sensor_input_init()-->ret[%d] dev[%p]",
                  ret, mag_uncal_input_info.dev );

    if( (0 != ret) || (NULL == (mag_uncal_input_info.dev))) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        SENSOR_ERR_LOG("end return[%d]",-ENODEV);
        return;
    }

    sensor_poll_init(SENSOR_MAG_UNCAL,&mag_uncal_poll_info);

    SENSOR_N_LOG("end");
    return;
}

EXPORT_SYMBOL(mag_uncal_driver_init);

