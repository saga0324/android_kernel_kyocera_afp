/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
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

#define ABS_X_DUMMY_VAL            (-1)

static ssize_t sgnfcnt_mtn_enable_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t sgnfcnt_mtn_enable_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t sgnfcnt_mtn_delay_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t sgnfcnt_mtn_delay_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t sgnfcnt_mtn_status_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t sgnfcnt_mtn_status_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static void sgnfcnt_mtn_set_input_params( struct input_dev *dev );

static DEVICE_ATTR( enable,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    sgnfcnt_mtn_enable_show,
    sgnfcnt_mtn_enable_store
);
static DEVICE_ATTR( delay,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    sgnfcnt_mtn_delay_show,
    sgnfcnt_mtn_delay_store
);
static DEVICE_ATTR(status,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    sgnfcnt_mtn_status_show,
    sgnfcnt_mtn_status_store
);

static struct attribute *sgnfcnt_mtn_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_delay.attr,
    &dev_attr_status.attr,
    NULL
};

static struct attribute_group sgnfcnt_mtn_attr_grp = {
    .attrs = sgnfcnt_mtn_attributes
};

struct sensor_input_info_str sgnfcnt_mtn_input_info =
{
    NULL,
    sgnfcnt_mtn_set_input_params,
    &sgnfcnt_mtn_attr_grp,
};

static uint32_t sgnfcnt_mtn_delay_time = 8;

static ssize_t sgnfcnt_mtn_delay_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end");
    return sprintf(buf, "%d\n", sgnfcnt_mtn_delay_time);
}

static ssize_t sgnfcnt_mtn_delay_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    uint32_t delay_time = simple_strtoul(buf, NULL, 10);
    SENSOR_N_LOG("start");

    sgnfcnt_mtn_delay_time = delay_time;

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t sgnfcnt_mtn_enable_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int enable = 0;
    SENSOR_N_LOG("start");
    enable = sensor_get_status(SENSOR_SGNFCNT_MTN);
    SENSOR_N_LOG("end ->enable[%d]",enable);
    return sprintf(buf, "%d\n", enable);
}

static ssize_t sgnfcnt_mtn_enable_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    unsigned long enable = simple_strtoul(buf, NULL, 10);
    SENSOR_N_LOG("start");

    sensor_enable(SENSOR_SGNFCNT_MTN, NULL ,(bool)enable);

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t sgnfcnt_mtn_status_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int rt = 0;
    SENSOR_N_LOG("start");

    rt = sensor_get_status(SENSOR_SGNFCNT_MTN);

    SENSOR_N_LOG("end");
    return sprintf(buf, "%d\n", rt);
}

static ssize_t sgnfcnt_mtn_status_store(
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

    sensor_set_status(SENSOR_SGNFCNT_MTN, status);

    SENSOR_N_LOG("end");
    return count;
}

void sgnfcnt_mtn_interrupt(void)
{
    SENSOR_N_LOG("start");

    sensor_report_abs_current_time(sgnfcnt_mtn_input_info.dev);
    input_report_abs( sgnfcnt_mtn_input_info.dev, ABS_X, ABS_X_DUMMY_VAL );
    input_report_abs( sgnfcnt_mtn_input_info.dev, ABS_X, 1 );
    input_sync( sgnfcnt_mtn_input_info.dev );

    SENSOR_N_LOG("end");
    return;
}

static void sgnfcnt_mtn_set_input_params( struct input_dev *dev )
{
    SENSOR_N_LOG("start");

    if(!dev){
        SENSOR_ERR_LOG("bad parm --> dev is NULL");
        return;
    }

    dev->name = "significant_motion";
    dev->id.bustype = BUS_SPI;

    set_bit(EV_ABS, dev->evbit);
    input_set_abs_params(dev, ABS_X, INT_MIN, INT_MAX, 0, 0);

    SENSOR_N_LOG("end");
    return;
}

void sgnfcnt_mtn_driver_init(void)
{
    int ret = 0;
    SENSOR_N_LOG("start");

    ret = sensor_input_init( &sgnfcnt_mtn_input_info );
    SENSOR_N_LOG("sensor_input_init()-->ret[%d] dev[%p]",
                  ret, sgnfcnt_mtn_input_info.dev );

    if( (0 != ret) || (NULL == (sgnfcnt_mtn_input_info.dev))) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        SENSOR_ERR_LOG("end return[%d]",-ENODEV);
        return;
    }

    SENSOR_N_LOG("end");
    return;
}

EXPORT_SYMBOL(sgnfcnt_mtn_driver_init);
