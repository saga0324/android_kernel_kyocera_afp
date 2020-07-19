/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 * (C) 2017 KYOCERA Corporation
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


#define DEFAULT_TEMP_AMBIENT_DELAY   (50)

static void temperature_ambient_poll_work_func(struct work_struct *work);
static void temperature_ambient_set_input_params( struct input_dev *dev );

static ssize_t temperature_ambient_batch_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t temperature_ambient_flush_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t temperature_ambient_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t temperature_ambient_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t temperature_ambient_delay_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t temperature_ambient_delay_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);

static DEVICE_ATTR(enable,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    temperature_ambient_enable_show,
    temperature_ambient_enable_store
);
static DEVICE_ATTR(delay,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    temperature_ambient_delay_show,
    temperature_ambient_delay_store
);
static DEVICE_ATTR(batch,
    S_IWUSR|S_IWGRP,
    NULL,
    temperature_ambient_batch_store
);

static DEVICE_ATTR(flush,
    S_IWUSR|S_IWGRP,
    NULL,
    temperature_ambient_flush_store
);

static struct attribute *temperature_ambient_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_delay.attr,
    &dev_attr_batch.attr,
    &dev_attr_flush.attr,
    NULL
};

static struct attribute_group temperature_ambient_attr_grp = {
    .attrs = temperature_ambient_attributes
};

struct sensor_input_info_str temperature_ambient_input_info =
{
    NULL,
    temperature_ambient_set_input_params,
    &temperature_ambient_attr_grp,
};

struct sensor_poll_info_str temperature_ambient_poll_info = {
    .name       = "temp_ambient_poll_wq",
    .poll_time  = ATOMIC_INIT(DEFAULT_TEMP_AMBIENT_DELAY),
    .poll_func  = temperature_ambient_poll_work_func,
};


static ssize_t temperature_ambient_batch_store(struct device *dev,
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

    ret = sensor_set_batch( SENSOR_TEMP_AMBIENT, batch_info);

    SENSOR_N_LOG("end - return[%d]",count);
    return count;
}

static ssize_t temperature_ambient_flush_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    uint32_t ret = 0;

    SENSOR_N_LOG("start");

    ret = sensor_set_flush(SENSOR_TEMP_AMBIENT, temperature_ambient_input_info.dev);

    SENSOR_N_LOG("end - return[%d]",count);
    return count;
}

static ssize_t temperature_ambient_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int enable = 0;
    SENSOR_N_LOG("start");

    enable = sensor_get_status(SENSOR_TEMP_AMBIENT);

    SENSOR_N_LOG("end ->enable[%d]",enable);
    return sprintf(buf, "%d\n", enable);
}

static ssize_t temperature_ambient_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    unsigned long enable = 0;
    int ret = 0;

    SENSOR_N_LOG("start");

    ret = kstrtoul(buf, 10, &enable);
    SENSOR_N_LOG("kstrtoul() ret[%d]->enable[%d]",ret, (int)enable);

    sensor_enable( SENSOR_TEMP_AMBIENT, &temperature_ambient_poll_info, (bool)enable );

    SENSOR_N_LOG("end - return[%d]",count);
    return count;
}

static ssize_t temperature_ambient_delay_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int32_t delay = 0;
    SENSOR_N_LOG("start");
    delay = atomic_read(&(temperature_ambient_poll_info.poll_time));
    SENSOR_N_LOG("end ->delay[%d]",(int)delay);
    return sprintf(buf, "%d\n", delay);
}

static ssize_t temperature_ambient_delay_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    unsigned long delay = 0;
    int ret = 0;
    SENSOR_N_LOG("start");

    ret = kstrtoul(buf, 10, &delay);
    SENSOR_N_LOG("kstrtoul() ret[%d]->delay[%d]",ret, (int)delay);

    sensor_set_poll_time( SENSOR_TEMP_AMBIENT, &temperature_ambient_poll_info, (int32_t)delay);

    SENSOR_N_LOG("end - return[%d]",count);
    return count;
}


static void temperature_ambient_set_input_params(struct input_dev *dev)
{
    SENSOR_N_LOG("start");

    if (!dev) {
        SENSOR_ERR_LOG("bad parm --> dev is NULL");
        return;
    }

    dev->name = "temperature_ambient";
    dev->id.bustype = BUS_SPI;

    set_bit(EV_ABS, dev->evbit);
    input_set_abs_params(dev, ABS_X, INT_MIN, INT_MAX, 0, 0);

    SENSOR_N_LOG("end");
    return;
}

static void temperature_ambient_poll_work_func(struct work_struct *work)
{
    int32_t polltime = 0;

    SENSOR_D_LOG("start");

    if(sns_get_reset_status() == false){
        sensor_com_mutex_lock();
        sns_iio_report_event_now(SENSOR_TEMP_AMBIENT);
        sensor_com_mutex_unlock();
    }

    polltime = atomic_read(&(temperature_ambient_poll_info.poll_time));
    if (polltime > 0) {
        queue_delayed_work( (temperature_ambient_poll_info.poll_wq),
                            &(temperature_ambient_poll_info.poll_work),
                            msecs_to_jiffies(polltime) );
        SENSOR_A_LOG("start delay work :polltime[%d]",
                     atomic_read(&(temperature_ambient_poll_info.poll_time)) );
    } else {
        SENSOR_ERR_LOG("fail polltime[%d]",(int)polltime);
    }
    SENSOR_D_LOG("end");
    return;
}

void temperature_ambient_driver_init(void)
{
    int ret = 0;

    SENSOR_N_LOG("start");

    ret = sensor_input_init( &temperature_ambient_input_info );
    SENSOR_N_LOG("sensor_input_init()-->ret[%d] dev[%d]",
                  ret, (int)(temperature_ambient_input_info.dev) );

    if( (0 != ret) || (NULL == (temperature_ambient_input_info.dev))) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        SENSOR_ERR_LOG("end return[%d]",-ENODEV);
        return;
    }

    sensor_poll_init(SENSOR_TEMP_AMBIENT,&temperature_ambient_poll_info);

    SENSOR_N_LOG("end");
    return;
}

EXPORT_SYMBOL(temperature_ambient_driver_init);
