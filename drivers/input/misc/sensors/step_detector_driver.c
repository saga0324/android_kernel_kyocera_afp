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

static ssize_t step_dtc_batch_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t step_dtc_batch_data_status_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t step_dtc_batch_data_1_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t step_dtc_batch_data_2_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t step_dtc_batch_data_abs_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t step_dtc_flush_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t step_dtc_enable_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t step_dtc_enable_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t step_dtc_delay_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t step_dtc_delay_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t step_dtc_status_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t step_dtc_status_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );

static void step_dtc_set_input_params( struct input_dev *dev );

static DEVICE_ATTR( enable,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    step_dtc_enable_show,
    step_dtc_enable_store
);
static DEVICE_ATTR( delay,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    step_dtc_delay_show,
    step_dtc_delay_store
);
static DEVICE_ATTR(status,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    step_dtc_status_show,
    step_dtc_status_store
);
static DEVICE_ATTR(batch,
    S_IWUSR|S_IWGRP,
    NULL,
    step_dtc_batch_store
);
static DEVICE_ATTR(batch_data_status,
    S_IRUSR|S_IRGRP,
    step_dtc_batch_data_status_show,
    NULL
);
static DEVICE_ATTR(batch_data_1,
    S_IRUSR|S_IRGRP,
    step_dtc_batch_data_1_show,
    NULL
);
static DEVICE_ATTR(batch_data_2,
    S_IRUSR|S_IRGRP,
    step_dtc_batch_data_2_show,
    NULL
);
static DEVICE_ATTR(flush,
    S_IWUSR|S_IWGRP,
    NULL,
    step_dtc_flush_store
);
static DEVICE_ATTR(batch_data_abs,
    S_IRUSR|S_IRGRP,
    step_dtc_batch_data_abs_show,
    NULL
);


static struct attribute *step_dtc_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_delay.attr,
    &dev_attr_status.attr,
    &dev_attr_batch.attr,
    &dev_attr_batch_data_status.attr,
    &dev_attr_batch_data_1.attr,
    &dev_attr_batch_data_2.attr,
    &dev_attr_flush.attr,
    &dev_attr_batch_data_abs.attr,
    NULL
};

static struct attribute_group step_dtc_attr_grp = {
    .attrs = step_dtc_attributes
};

struct sensor_input_info_str step_dtc_input_info =
{
    NULL,
    step_dtc_set_input_params,
    &step_dtc_attr_grp,
};

static uint32_t step_dtc_delay_time = 8;

static struct sensor_batch_data_str step_dtc_step1_batch_data;
static struct sensor_batch_data_str step_dtc_step2_batch_data;
static struct sensor_batch_data_str step_dtc_step_abs_batch_data;
static bool step_dtc_step_abs_report = false;
static int32_t g_time_stamp_step_dtc1        = 0;
static int32_t g_time_stamp_step_dtc2        = 0;
static uint32_t g_input_num_dtc1       = 0;
static uint32_t g_input_num_dtc2       = 0;
static uint32_t g_step_detector              = 0;
static uint32_t step_dtc_step1_repo_type = 0;
extern struct mutex sensor_batch_mutex;

static ssize_t step_dtc_batch_store(struct device *dev,
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


    if( batch_info.period_ns > 30){
        batch_info.period_ns = 30;
    }
    step_dtc_delay_time = batch_info.period_ns;

    ret = sensor_set_batch( SENSOR_STEP_DTC, batch_info);

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t step_dtc_batch_data_status_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int status = false;
    SENSOR_N_LOG("start");
    mutex_lock(&sensor_batch_mutex);

    if(step_dtc_step1_batch_data.payload_size != 0) {
        status = true;
        sensor_report_batch( step_dtc_input_info.dev,
                             step_dtc_step1_repo_type,
                             step_dtc_step1_batch_data );
    }

    mutex_unlock(&sensor_batch_mutex);
    SENSOR_N_LOG("end ->status[%d]",status);
    return sprintf(buf, "%d\n", status);
}

static ssize_t step_dtc_batch_data_1_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret =0;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_batch_mutex);

    ret = sprintf(buf, "%d %d %d %d\n",step_dtc_step1_batch_data.payload_size,
                       step_dtc_step1_batch_data.recode_num, g_input_num_dtc1, g_time_stamp_step_dtc1);

    g_time_stamp_step_dtc1 = 0;

    mutex_unlock(&sensor_batch_mutex);

    SENSOR_N_LOG("end");

    return ret;
}

static ssize_t step_dtc_batch_data_2_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret =0;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_batch_mutex);

    ret = sprintf(buf, "%d %d %d %d %d\n",step_dtc_step2_batch_data.payload_size,
                       step_dtc_step2_batch_data.recode_num, g_input_num_dtc2, 
                       g_time_stamp_step_dtc2,g_step_detector);

    g_time_stamp_step_dtc2 = 0;
    g_step_detector = 0;

    mutex_unlock(&sensor_batch_mutex);

    SENSOR_N_LOG("end");

    return ret;
}

static ssize_t step_dtc_batch_data_abs_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int i = 0;
    uint8_t *load_ptr;
    uint8_t load_buf[8];
    uint32_t timestamp = 0;
    uint32_t step_abs  = 0;

    SENSOR_N_LOG("start");
    mutex_lock(&sensor_batch_mutex);

    if(step_dtc_step_abs_batch_data.recode_num != 0){
        load_ptr = (uint8_t*)step_dtc_step_abs_batch_data.payload_p;

        for(i=0; i<8; i++){
           load_buf[i] = *(load_ptr++);
           SENSOR_N_LOG("load_buf[%d]:%d\n", i, load_buf[i]);
        }

        timestamp = (uint32_t)((load_buf[3] & 0xFF) << 24) | ((load_buf[2] & 0xFF) << 16) | ((load_buf[1] & 0xFF) << 8) | (load_buf[0] & 0xFF);
        step_abs  = (uint32_t)((load_buf[7] & 0xFF) << 24) | ((load_buf[6] & 0xFF) << 16) | ((load_buf[5] & 0xFF) << 8) | (load_buf[4] & 0xFF);

    }else{
        timestamp = 0;
        step_abs  = 0;
    }

    mutex_unlock(&sensor_batch_mutex);
    SENSOR_N_LOG("timestamp[%d] step_abs[%d]\n",(int)timestamp, (int)step_abs);

    return sprintf(buf, "%d %d\n",timestamp, step_abs);
}

void step_dtc_batch_ring_buffer(uint32_t step_detector)
{
    SENSOR_N_LOG("start-step[%d]", step_detector);
    g_step_detector = step_detector;
    SENSOR_N_LOG("end-g_step_detector[%d]",g_step_detector);
}

void step_dtc_step1_ring_buffer_timestamp(
    int32_t time_stamp_step_dtc1)
{
    SENSOR_N_LOG("start");
    g_time_stamp_step_dtc1 = time_stamp_step_dtc1;
    SENSOR_N_LOG("end %d",g_time_stamp_step_dtc1);
}

void step_dtc_step2_ring_buffer_timestamp(
    int32_t time_stamp_step_dtc2)
{
    SENSOR_N_LOG("start");
    g_time_stamp_step_dtc2 = time_stamp_step_dtc2;
    SENSOR_N_LOG("end %d",g_time_stamp_step_dtc2);
}

void step_dtc_step1_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data )
{

    SENSOR_N_LOG("start");

    step_dtc_step1_repo_type = repo_type;
    step_dtc_step1_batch_data = batch_data;


    SENSOR_N_LOG("end");
    return;
}

void step_dtc_step2_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data )
{

    SENSOR_N_LOG("start");

    step_dtc_step2_batch_data = batch_data;

    if(step_dtc_step_abs_report == false){
    } else {
        step_dtc_step_abs_report = false;
    }

    sensor_report_batch( step_dtc_input_info.dev,
                         repo_type,
                         step_dtc_step2_batch_data );

    SENSOR_N_LOG("end");
    return;
}

void step_dtc_step_abs_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data )
{
    SENSOR_N_LOG("start");

    step_dtc_step_abs_batch_data = batch_data;

    step_dtc_step_abs_report = true;

    sensor_report_batch( step_dtc_input_info.dev,
                         SENSOR_COMP_ABS,
                         step_dtc_step_abs_batch_data );

    SENSOR_N_LOG("end");
    return;
}

void step_dtc_timestamp_report(void)
{
    SENSOR_N_LOG("start");

    sensor_report_batch( step_dtc_input_info.dev,
                         SENSOR_COMP_TIME,
                         step_dtc_step2_batch_data );

    SENSOR_N_LOG("end");
    return;
}

static ssize_t step_dtc_flush_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    uint32_t ret = 0;

    SENSOR_N_LOG("start");

    ret = sensor_set_flush(SENSOR_STEP_DTC, step_dtc_input_info.dev);

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}


static ssize_t step_dtc_delay_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end");
    return sprintf(buf, "%d\n", step_dtc_delay_time);
}

static ssize_t step_dtc_delay_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    uint32_t ret = 0;
    struct sensor_batch_info_str batch_info;
    uint32_t delay_time = simple_strtoul(buf, NULL, 10);
    SENSOR_N_LOG("start delay_time[%u]", delay_time);

    batch_info.flags = 0;
    batch_info.period_ns = delay_time;
    batch_info.timeout = 0;

    if( batch_info.period_ns > 30){
        batch_info.period_ns = 30;
    }
    step_dtc_delay_time = batch_info.period_ns;

    ret = sensor_set_batch( SENSOR_STEP_DTC, batch_info);

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t step_dtc_enable_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int enable = 0;
    SENSOR_N_LOG("start");
    enable = sensor_get_status(SENSOR_STEP_DTC);
    SENSOR_N_LOG("end ->enable[%d]",enable);
    return sprintf(buf, "%d\n", enable);
}

static ssize_t step_dtc_enable_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    unsigned long enable = simple_strtoul(buf, NULL, 10);
    SENSOR_N_LOG("start");

    sensor_enable( SENSOR_STEP_DTC, NULL, (bool)enable );
    SENSOR_N_LOG("end");
    return count;
}

static ssize_t step_dtc_status_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int rt = 0;
    SENSOR_N_LOG("start");

    rt = sensor_get_status(SENSOR_STEP_DTC);

    SENSOR_N_LOG("end");
    return sprintf(buf, "%d\n", rt);
}

static ssize_t step_dtc_status_store(
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

    sensor_set_status(SENSOR_STEP_DTC, status);

    SENSOR_N_LOG("end");
    return count;
}

void step_dtc_interrupt(void)
{
    SENSOR_N_LOG("start");

    sensor_report_abs_current_time(step_dtc_input_info.dev);
    input_report_abs( step_dtc_input_info.dev, ABS_X, 0 );
    input_sync( step_dtc_input_info.dev );

    SENSOR_N_LOG("end");
    return;
}

static void step_dtc_set_input_params( struct input_dev *dev )
{
    SENSOR_N_LOG("start");

    if(!dev){
        SENSOR_ERR_LOG("bad parm --> dev is NULL");
        return;
    }

    dev->name = "step_detector";
    dev->id.bustype = BUS_SPI;

    set_bit(EV_ABS, dev->evbit);
    input_set_abs_params(dev, ABS_X, INT_MIN, INT_MAX, 0, 0);
    input_set_abs_params(dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);

    SENSOR_N_LOG("end");
    return;
}

void step_dtc_driver_init(void)
{
    int ret = 0;
    SENSOR_N_LOG("start");

    ret = sensor_input_init( &step_dtc_input_info );
    SENSOR_N_LOG("sensor_input_init()-->ret[%d] dev[%p]",
                  ret, step_dtc_input_info.dev );

    if( (0 != ret) || (NULL == (step_dtc_input_info.dev))) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        SENSOR_ERR_LOG("end return[%d]",-ENODEV);
        return;
    }

    SENSOR_N_LOG("end");
    return;
}

EXPORT_SYMBOL(step_dtc_driver_init);
