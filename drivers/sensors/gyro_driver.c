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

#define DEFAULT_GYRO_DELAY    (30)
#define ABSMAX_GYRO          (32000)
#define ABSMIN_GYRO          (-32000)
#define GYRO_DIAG_REQ_POWER_ON  (0)
#define GYRO_DIAG_REQ_POWER_OFF (1)
#define GYRO_DIAG_REQ_INIT      (2)

static ssize_t gyro_batch_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t gyro_batch_data_store(struct device *dev,
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
   size_t count );
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
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
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
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    NULL,
    gyro_batch_store
);
static DEVICE_ATTR(batch_data,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    gyro_batch_data_show,
    gyro_batch_data_store
);
static DEVICE_ATTR(flush,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    NULL,
    gyro_flush_store
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

static struct sensor_batch_data_str gyro_batch_data;
static uint8_t* gyro_batch_hal_addr = NULL;
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

    SENSOR_N_LOG("end - return[%d]",count);
    return count;
}

static ssize_t gyro_batch_data_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    uint32_t data;
    uint8_t *data_addr = NULL;

    SENSOR_N_LOG("start");
    mutex_lock(&sensor_batch_mutex);

    sscanf(buf, "%x", &data);
    data_addr = (uint8_t*)data;

    if( NULL == data_addr){
        mutex_unlock(&sensor_batch_mutex);
        SENSOR_ERR_LOG("bad param-->data_addr[%x]",
                       (int)data_addr );
        return count;
    }

    gyro_batch_hal_addr = data_addr;

    SENSOR_N_LOG("gyro_batch_hal_addr[%x]", (int)gyro_batch_hal_addr);

    if(sensor_copy_batch_data(SENSOR_BATCH_TBL_GYRO,gyro_batch_data,gyro_batch_hal_addr)){
        mutex_unlock(&sensor_batch_mutex);
        SENSOR_ERR_LOG("miss copy batch data to user");
        return count;
    }

    mutex_unlock(&sensor_batch_mutex);
    SENSOR_N_LOG("end - return[%d]",count);
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

    SENSOR_N_LOG("end - return[%d]",count);
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
    int32_t offsets[3];

    sensor_com_mutex_lock();
    ret = sns_gyro_get_offset(offsets);
    sensor_com_mutex_unlock();

    SENSOR_N_LOG("offset0[%d] offset1[%d] offset2[%d]",
                  offsets[0],offsets[1],offsets[2]);

    if(ret == SNS_RC_OK) {
        ret = sprintf( buf, "%d %d %d\n",
                       offsets[0],
                       offsets[1],
                       offsets[2] );
    }

    return ret;
}

static ssize_t gyro_offset_store(struct device *dev,
   struct device_attribute *attr,
   const char *buf,
   size_t count )
{
    ssize_t ret = 0;
    int32_t offsets[3];

    SENSOR_N_LOG("start");

    sscanf(buf, "%d %d %d",
            &offsets[0],
            &offsets[1],
            &offsets[2]);

    SENSOR_N_LOG("offset0[%d] offset1[%d] offset2[%d]",
                  offsets[0],offsets[1],offsets[2]);

    sensor_com_mutex_lock();
    ret = sns_gyro_set_offset(&offsets[0]);
    sensor_com_mutex_unlock();

    SENSOR_N_LOG("end - return[%d]",count);

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
    uint32_t tmp_status = 0;

    SENSOR_N_LOG("start");

    ret = strict_strtoul(buf, 10, &enable);
    SENSOR_N_LOG("strict_strtoul() ret[%d]->enable[%d]",ret, (int)enable);

    tmp_status = sensor_get_batch_status();
    if((enable == 0) && (tmp_status & (BATCH_ON << SENSOR_GYRO))){
        batch_info.flags = 0;
        batch_info.period_ns = 0;
        batch_info.timeout =0;

        sensor_set_batch( SENSOR_GYRO, batch_info);
    }

    sensor_enable( SENSOR_GYRO, &gyro_poll_info, (bool)enable );

    SENSOR_N_LOG("end - return[%d]",count);
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

    ret = strict_strtoul(buf, 10, &delay);
    SENSOR_N_LOG("strict_strtoul() ret[%d]->delay[%d]",ret, (int)delay);

    sensor_set_poll_time( SENSOR_GYRO, &gyro_poll_info, (int32_t)delay);

    SENSOR_N_LOG("end - return[%d]",count);
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
    int32_t polltime = 0;

    SENSOR_N_LOG("start");

    if(sns_get_reset_status() == false){
        sns_iio_report_event_now(SENSOR_GYRO);
    }

    polltime = atomic_read(&(gyro_poll_info.poll_time));
    if (polltime > 0) {
        queue_delayed_work( (gyro_poll_info.poll_wq),
                            &(gyro_poll_info.poll_work),
                            msecs_to_jiffies(polltime) );
        SENSOR_A_LOG("start delay work :polltime[%d]",
                     atomic_read(&(gyro_poll_info.poll_time)) );
    } else {
        SENSOR_ERR_LOG("fail polltime[%d]",(int)polltime);
    }
    SENSOR_N_LOG("end");
    return;
}

static void gyro_set_input_params( struct input_dev *dev )
{
    SENSOR_N_LOG("start");

    if(!dev){
        SENSOR_ERR_LOG("bad parm --> dev[%d]",(int)dev);
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
    SENSOR_N_LOG("sensor_input_init()-->ret[%d] dev[%d]",
                  ret, (int)(gyro_input_info.dev) );

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

