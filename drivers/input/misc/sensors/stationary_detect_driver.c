/*
 * This software is contributed or developed by KYOCERA Corporation.
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

#include "sensor_driver.h"
#include "sensor_micon_driver.h"
#include "sensor_com.h"

#define ABS_X_DUMMY_VAL            (-1)

static ssize_t stationary_detect_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t stationary_detect_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t stationary_detect_delay_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t stationary_detect_delay_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t stationary_detect_status_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t stationary_detect_status_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t stationary_detect_data_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf);
static void stationary_detect_set_input_params( struct input_dev *dev );


static DEVICE_ATTR(enable,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    stationary_detect_enable_show,
    stationary_detect_enable_store
);
static DEVICE_ATTR(delay,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    stationary_detect_delay_show,
    stationary_detect_delay_store
);
static DEVICE_ATTR(status,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    stationary_detect_status_show,
    stationary_detect_status_store
);
static DEVICE_ATTR(data,
    S_IRUSR|S_IRGRP,
    stationary_detect_data_show,
    NULL
);

static struct attribute *stationary_detect_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_delay.attr,
    &dev_attr_status.attr,
    &dev_attr_data.attr,
    NULL
};

static struct attribute_group stationary_detect_attr_grp = {
    .attrs = stationary_detect_attributes
};

struct sensor_input_info_str stationary_detect_input_info =
{
    NULL,
    stationary_detect_set_input_params,
    &stationary_detect_attr_grp,
};

static struct step_counter stationary_detect_last_read_data = {
    0
};

static uint32_t stationary_detect_delay_time = 8;
static bool stationary_detect_first_abs_report = false;

static ssize_t stationary_detect_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int enable = 0;
    SENSOR_N_LOG("start");

    enable = sensor_get_status(SENSOR_STATIONARY_DETECT);

    SENSOR_N_LOG("end ->enable[%d]", enable);
    return sprintf(buf, "%d\n", enable);
}

static ssize_t stationary_detect_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    unsigned long enable = 0;
    int ret = 0;
    int status = 0;

    SENSOR_N_LOG("start");

    ret = kstrtoul(buf, 10, &enable);
    SENSOR_N_LOG("kstrtoul() ret[%d]->enable[%d]",ret, (int)enable);

    status = sensor_get_status(SENSOR_STATIONARY_DETECT);

    sensor_enable( SENSOR_STATIONARY_DETECT, NULL, (bool)enable );

    if(false == status){
        stationary_detect_first_abs_report = true;
    }

    SENSOR_N_LOG("end - return[%ld]", count);
    return count;
}

static ssize_t stationary_detect_delay_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end");
    return sprintf(buf, "%d\n", stationary_detect_delay_time);
}

static ssize_t stationary_detect_delay_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    uint32_t delay_time = simple_strtoul(buf, NULL, 10);
    SENSOR_N_LOG("start delay_time[%u]", delay_time);

    stationary_detect_delay_time = delay_time;

    SENSOR_N_LOG("end - return[%ld]", count);
    return count;
}

static ssize_t stationary_detect_status_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int rt = 0;
    SENSOR_N_LOG("start");

    rt = sensor_get_status(SENSOR_STATIONARY_DETECT);

    SENSOR_N_LOG("end");
    return sprintf(buf, "%d\n", rt);
}

static ssize_t stationary_detect_status_store(
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

    sensor_set_status(SENSOR_STATIONARY_DETECT, status);

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t stationary_detect_data_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret;
    SENSOR_N_LOG("start");

    ret = sprintf( buf, "%d\n", stationary_detect_last_read_data.step );

    SENSOR_N_LOG("end");
    return ret;
}

static void stationary_detect_set_input_params( struct input_dev *dev )
{
    SENSOR_N_LOG("start");

    if(!dev){
        SENSOR_ERR_LOG("bad parm --> dev is NULL");
        return;
    }

    dev->name = "stationary_detect";
    dev->id.bustype = BUS_SPI;

    set_bit(EV_ABS, dev->evbit);
    input_set_abs_params(dev, ABS_X, INT_MIN, INT_MAX, 0, 0);
    input_set_abs_params(dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);

    SENSOR_N_LOG("end");
    return;
}

void stationary_detect_interrupt( void )
{
    SENSOR_N_LOG("start");
    if ( SENSOR_ON == sensor_get_status(SENSOR_STATIONARY_DETECT) ) {
        sensor_report_abs_current_time(stationary_detect_input_info.dev);
        input_report_abs( stationary_detect_input_info.dev, ABS_X, ABS_X_DUMMY_VAL );
        input_report_abs( stationary_detect_input_info.dev, ABS_X, 1 );
        input_sync( stationary_detect_input_info.dev );
    }
    SENSOR_N_LOG("end");
    return;
}

void stationary_detect_driver_init( void )
{
    int ret = 0;

    SENSOR_N_LOG("start");

    ret = sensor_input_init( &stationary_detect_input_info );
    SENSOR_N_LOG("sensor_input_init()-->ret[%d] dev[%p]",
                  ret, stationary_detect_input_info.dev );

    if( (0 != ret) || (NULL == (stationary_detect_input_info.dev))) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        SENSOR_ERR_LOG("end return[%d]",-ENODEV);
        return;
    }

    SENSOR_N_LOG("end");
    return;
}

EXPORT_SYMBOL(stationary_detect_driver_init);

