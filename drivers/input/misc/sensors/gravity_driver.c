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

#define DEFAULT_GRV_DELAY    (30)

static ssize_t grv_batch_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t grv_batch_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t grv_flush_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t grv_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t grv_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t grv_delay_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t grv_delay_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t grv_status_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t grv_status_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t grv_data_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf);

static void grv_poll_work_func(struct work_struct *work);
static void grv_set_input_params( struct input_dev *dev );


static DEVICE_ATTR(enable,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    grv_enable_show,
    grv_enable_store
);
static DEVICE_ATTR(delay,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    grv_delay_show,
    grv_delay_store
);
static DEVICE_ATTR(status,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    grv_status_show,
    grv_status_store
);
static DEVICE_ATTR(data,
    S_IRUSR|S_IRGRP,
    grv_data_show,
    NULL
);
static DEVICE_ATTR(batch,
    S_IWUSR|S_IWGRP,
    NULL,
    grv_batch_store
);
static DEVICE_ATTR(batch_data,
    S_IRUSR|S_IRGRP,
    grv_batch_data_show,
    NULL
);
static DEVICE_ATTR(flush,
    S_IWUSR|S_IWGRP,
    NULL,
    grv_flush_store
);

static struct attribute *grv_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_delay.attr,
    &dev_attr_status.attr,
    &dev_attr_data.attr,
    &dev_attr_batch.attr,
    &dev_attr_batch_data.attr,
    &dev_attr_flush.attr,
    NULL
};

static struct attribute_group grv_attr_grp = {
    .attrs = grv_attributes
};

struct sensor_input_info_str grv_input_info =
{
    NULL,
    grv_set_input_params,
    &grv_attr_grp,
};

struct sensor_poll_info_str grv_poll_info = {
    .name       = "grv_poll_wq",
    .poll_time  = ATOMIC_INIT(DEFAULT_GRV_DELAY),
    .poll_func  = grv_poll_work_func,
};

static struct gravity grv_last_read_data = {
    0,0,0
};


static struct sensor_batch_data_str grv_batch_data;
static uint32_t g_time_stamp_gravity  = 0;
static uint32_t g_input_num_grv       = 0;
extern struct mutex sensor_batch_mutex;

static ssize_t grv_batch_store(struct device *dev,
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

    ret = sensor_set_batch( SENSOR_GRV, batch_info);

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t grv_batch_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret =0;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_batch_mutex);

    ret = sprintf(buf, "%d %d %d %d\n",grv_batch_data.payload_size,
                       grv_batch_data.recode_num, g_input_num_grv, g_time_stamp_gravity);

    g_time_stamp_gravity = 0;

    mutex_unlock(&sensor_batch_mutex);

    SENSOR_N_LOG("end");

    return ret;
}

void grv_ring_buffer_timestamp(
    uint32_t time_stamp_gravity)
{
    SENSOR_N_LOG("start");
    g_time_stamp_gravity = time_stamp_gravity;
    SENSOR_N_LOG("end %d",g_time_stamp_gravity);
}

void grv_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data )
{
    
    SENSOR_N_LOG("start");

    grv_batch_data = batch_data;

    sensor_report_batch( grv_input_info.dev,
                         repo_type,
                         grv_batch_data );

    SENSOR_N_LOG("end");
    return;
}

void grv_timestamp_report(void)
{
    SENSOR_N_LOG("start");

    sensor_report_batch( grv_input_info.dev,
                         SENSOR_COMP_TIME,
                         grv_batch_data );

    SENSOR_N_LOG("end");
    return;
}

static ssize_t grv_flush_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    uint32_t ret = 0;

    SENSOR_N_LOG("start");

    ret = sensor_set_flush(SENSOR_GRV, grv_input_info.dev);

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t grv_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int enable = 0;
    SENSOR_N_LOG("start");

    enable = sensor_get_status(SENSOR_GRV);

    SENSOR_N_LOG("end ->enable[%d]",enable);
    return sprintf(buf, "%d\n", enable);
}

static ssize_t grv_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    unsigned long enable = 0;
    int ret = 0;

    SENSOR_N_LOG("start");

    ret = kstrtoul(buf, 10, &enable);
    SENSOR_N_LOG("kstrtoul() ret[%d]->enable[%d]",ret, (int)enable);

    sensor_enable( SENSOR_GRV, &grv_poll_info, (bool)enable );

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t grv_delay_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int32_t delay = 0;
    SENSOR_N_LOG("start");
    delay = atomic_read(&(grv_poll_info.poll_time));
    SENSOR_N_LOG("end ->delay[%d]",(int)delay);
    return sprintf(buf, "%d\n", delay);
}

static ssize_t grv_delay_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    unsigned long delay = 0;
    int ret = 0;
    SENSOR_N_LOG("start");

    ret = kstrtoul(buf, 10, &delay);
    SENSOR_N_LOG("kstrtoul() ret[%d]->delay[%d]",ret, (int)delay);

    sensor_set_poll_time( SENSOR_GRV, &grv_poll_info, (int32_t)delay);

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t grv_status_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int rt = 0;
    SENSOR_N_LOG("start");

    rt = sensor_get_status(SENSOR_GRV);

    SENSOR_N_LOG("end");
    return sprintf(buf, "%d\n", rt);
}

static ssize_t grv_status_store(
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

    sensor_set_status(SENSOR_GRV, status);

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t grv_data_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret;
    SENSOR_N_LOG("start");

    ret = sprintf( buf, "%d %d %d\n",
                   grv_last_read_data.x,
                   grv_last_read_data.y,
                   grv_last_read_data.z );

    SENSOR_N_LOG("end");
    return ret;
}


static void grv_poll_work_func(struct work_struct *work)
{
    SENSOR_N_LOG("start");

    if(sns_get_reset_status() == false){
        sns_iio_report_event_now(SENSOR_GRV);
    }

    SENSOR_N_LOG("end");
    return;
}

static void grv_set_input_params( struct input_dev *dev )
{
    SENSOR_N_LOG("start");

    if(!dev){
        SENSOR_ERR_LOG("bad parm --> dev is NULL");
        return;
    }

    dev->name = "gravity";
    dev->id.bustype = BUS_SPI;

    set_bit(EV_ABS, dev->evbit);
    input_set_abs_params(dev, ABS_RUDDER, INT_MIN, INT_MAX, 0, 0);
    input_set_abs_params(dev, ABS_STATUS, 0, 3, 0, 0);
    input_set_abs_params(dev, ABS_X, INT_MIN, INT_MAX, 0, 0);
    input_set_abs_params(dev, ABS_Y, INT_MIN, INT_MAX, 0, 0);
    input_set_abs_params(dev, ABS_Z, INT_MIN, INT_MAX, 0, 0);
    input_set_abs_params(dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);

    SENSOR_N_LOG("end");
    return;
}

void grv_driver_init( void )
{
    int ret = 0;

    SENSOR_N_LOG("start");

    ret = sensor_input_init( &grv_input_info );
    SENSOR_N_LOG("sensor_input_init()-->ret[%d] dev[%p]",
                  ret, grv_input_info.dev );

    if( (0 != ret) || (NULL == (grv_input_info.dev))) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        SENSOR_ERR_LOG("end return[%d]",-ENODEV);
        return;
    }

    sensor_poll_init(SENSOR_GRV,&grv_poll_info);

    SENSOR_N_LOG("end");
    return;
}

EXPORT_SYMBOL(grv_driver_init);

