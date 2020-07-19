/*
 * This software is contributed or developed by KYOCERA Corporation.
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
#include <linux/miscdevice.h>
#include "sensor_driver.h"
#include "sensor_micon_driver.h"
#include "sensor_com.h"

#define DEVICE_FILE_NAME        "piezo_press_sensor"

#define PIEZO_PRESS_DUMMY_DATA  (-1)

/* *************************************
 piezo_press_driver.c Grobal Argument
************************************* */
static int piezo_press_open(struct inode *inode_type, struct file *file);
static int piezo_press_release(struct inode *inode_type, struct file *file);
static ssize_t piezo_press_flush_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t piezo_press_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t piezo_press_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t piezo_press_delay_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t piezo_press_delay_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static void piezo_press_set_input_params( struct input_dev *dev );
static ssize_t piezo_press_value_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t piezo_press_imit_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t piezo_press_imit_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t piezo_press_status_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t piezo_press_properties_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t piezo_press_properties_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t piezo_press_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );

static DEVICE_ATTR(enable_piezo_press_sensor,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    piezo_press_enable_show,
    piezo_press_enable_store
);
static DEVICE_ATTR(poll_delay,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    piezo_press_delay_show,
    piezo_press_delay_store
);
static DEVICE_ATTR(value,
    S_IRUSR|S_IRGRP,
    piezo_press_value_show,
    NULL
);
static DEVICE_ATTR(imitation,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    piezo_press_imit_show,
    piezo_press_imit_store
);
static DEVICE_ATTR(flush,
    S_IWUSR|S_IWGRP,
    NULL,
    piezo_press_flush_store
);
static DEVICE_ATTR(status,
    S_IRUSR|S_IRGRP,
    piezo_press_status_show,
    NULL
);
static DEVICE_ATTR(prop,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    piezo_press_properties_show,
    piezo_press_properties_store
);
static DEVICE_ATTR( data,
    S_IRUSR|S_IRGRP,
    piezo_press_data_show,
    NULL
);

static struct attribute *piezo_press_attributes[] = {
    &dev_attr_enable_piezo_press_sensor.attr,
    &dev_attr_poll_delay.attr,
    &dev_attr_value.attr,
    &dev_attr_imitation.attr,
    &dev_attr_flush.attr,
    &dev_attr_status.attr,
    &dev_attr_prop.attr,
    &dev_attr_data.attr,
    NULL
};

static struct attribute_group piezo_press_attr_grp = {
    .attrs = piezo_press_attributes
};

struct sensor_input_info_str piezo_press_input_info =
{
    NULL,
    piezo_press_set_input_params,
    &piezo_press_attr_grp,
};

static struct file_operations piezo_press_fops = {
    .owner = THIS_MODULE,
    .open = piezo_press_open,
    .release = piezo_press_release,
};

static struct miscdevice piezo_press_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = DEVICE_FILE_NAME,
    .fops = &piezo_press_fops,
};

static uint8_t piezo_press_last_read_data[6] = {0};
static DEFINE_MUTEX(piezo_press_event_mutex);

static int piezo_press_open(struct inode *inode_type, struct file *file)
{
    SENSOR_D_LOG("start");
    SENSOR_D_LOG("end");
    return 0;
}

static int piezo_press_release(struct inode *inode_type, struct file *file)
{
    SENSOR_D_LOG("start");
    SENSOR_D_LOG("end");
    return 0;
}

static ssize_t piezo_press_flush_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    SENSOR_D_LOG("start");

    mutex_lock(&piezo_press_event_mutex);
    input_report_abs(piezo_press_input_info.dev, ABS_MISC+3, ~PIEZO_PRESS_DUMMY_DATA);
    input_report_abs(piezo_press_input_info.dev, ABS_MISC+3, PIEZO_PRESS_DUMMY_DATA);
    input_sync(piezo_press_input_info.dev);
    mutex_unlock(&piezo_press_event_mutex);

    SENSOR_D_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t piezo_press_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t count = 0;
    unsigned long enable = 0;
    int ret = 0;
    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_PIEZO_PRESS)  == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }

    count = msp430_piezo_press_poll_en_show(buf);
    ret = kstrtoul(buf, 10, &enable);
    SENSOR_D_LOG("kstrtoul() ret[%d]->enable[%d]",ret, (int)enable);

    SENSOR_D_LOG("end - return[%zu]",count);
    return count;
}

static ssize_t piezo_press_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    unsigned long enable = 0;
    int ret = 0;
    SENSOR_D_LOG("start");

    if(sensor_get_initialize_state(SENSOR_PIEZO_PRESS)  == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return count;
    }

    ret = kstrtoul(buf, 10, &enable);
    SENSOR_D_LOG("kstrtoul() ret[%d]->poll_enable[%d]",ret, (int)enable);
    msp430_piezo_press_poll_en_store((bool)enable);

    SENSOR_D_LOG("end - return[%zu]",count);
    return count;
}

static ssize_t piezo_press_delay_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t count = 0;
    unsigned long delay = 0;
    int ret = 0;
    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_PIEZO_PRESS)  == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }

    count = msp430_piezo_press_poll_delay_show(buf);
    ret = kstrtoul(buf, 10, &delay);
    SENSOR_D_LOG("kstrtoul() ret[%d]->delay[%d]",ret, (int)delay);

    SENSOR_D_LOG("end - return[%zu]",count);
    return count;
}

static ssize_t piezo_press_delay_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    unsigned long delay = 0;
    int ret = 0;
    SENSOR_D_LOG("start");

    if(sensor_get_initialize_state(SENSOR_PIEZO_PRESS)  == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return count;
    }

    ret = kstrtoul(buf, 10, &delay);
    SENSOR_D_LOG("kstrtoul() ret[%d]->delay[%d]",ret, (int)delay);
    msp430_piezo_press_poll_delay_store((unsigned int)delay);

    SENSOR_D_LOG("end - return[%zu]",count);
    return count;
}

void piezo_press_input_report( uint8_t *data )
{
    int i;

    SENSOR_D_LOG("start param[%d %d %d %d %d %d]",(int)data[0],(int)data[1],(int)data[2],(int)data[3],(int)data[4],(int)data[5]);

    for(i=0; i<6; i++) {
        piezo_press_last_read_data[i] = data[i];
    }
    mutex_lock(&piezo_press_event_mutex);
    sensor_report_abs_current_time(piezo_press_input_info.dev);
    input_report_abs( piezo_press_input_info.dev, ABS_X, data[0]);          //piezo_device0[Power]
    input_report_abs( piezo_press_input_info.dev, ABS_Y, data[1]);          //piezo_device1[Vol_Up]
    input_report_abs( piezo_press_input_info.dev, ABS_Z, data[2]);          //piezo_device2[Vol_Down]
    input_report_abs( piezo_press_input_info.dev, ABS_RX, data[3]);         //piezo_device3[App_L]
    input_report_abs( piezo_press_input_info.dev, ABS_RY, data[4]);         //piezo_device4[App_R]
    input_report_abs( piezo_press_input_info.dev, ABS_RZ, data[5]);         //piezo_device5[Display]

    input_sync(piezo_press_input_info.dev);
    mutex_unlock(&piezo_press_event_mutex);

    SENSOR_D_LOG("end");
    return;
}

static void piezo_press_set_input_params( struct input_dev *dev )
{
    SENSOR_D_LOG("start");

    if(!dev){
        SENSOR_ERR_LOG("bad parm --> dev is NULL");
        return;
    }

    dev->name = "piezo_press_sensor";
    dev->id.bustype = BUS_I2C;

    set_bit(EV_ABS, dev->evbit);
    input_set_capability(dev, EV_ABS, ABS_X);       //piezo_device0[Power]
    input_set_capability(dev, EV_ABS, ABS_MISC+3);
    input_set_capability(dev, EV_ABS, ABS_Y);       //piezo_device1[Vol_Up]
    input_set_capability(dev, EV_ABS, ABS_Z);       //piezo_device2[Vol_Down]
    input_set_capability(dev, EV_ABS, ABS_RX);      //piezo_device3[App_L]
    input_set_capability(dev, EV_ABS, ABS_RY);      //piezo_device4[App_R]
    input_set_capability(dev, EV_ABS, ABS_RZ);      //piezo_device5[Display]
    input_set_abs_params(dev, ABS_X,  INT_MIN, INT_MAX, 0, 0);
    input_set_abs_params(dev, ABS_MISC+3, INT_MIN, INT_MAX, 0, 0);
    input_set_abs_params(dev, ABS_Y,  INT_MIN, INT_MAX, 0, 0);
    input_set_abs_params(dev, ABS_Z,  INT_MIN, INT_MAX, 0, 0);
    input_set_abs_params(dev, ABS_RX, INT_MIN, INT_MAX, 0, 0);
    input_set_abs_params(dev, ABS_RY, INT_MIN, INT_MAX, 0, 0);
    input_set_abs_params(dev, ABS_RZ, INT_MIN, INT_MAX, 0, 0);

    SENSOR_D_LOG("end");
    return;
}

static ssize_t piezo_press_status_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t count = 0;
    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_PIEZO_PRESS)  == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
    count = msp430_piezo_press_status_show(buf);
    SENSOR_D_LOG("end");
    return count;
}

static ssize_t piezo_press_value_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t count = 0;
    int ret = 0;
    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_PIEZO_PRESS)  == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return ret;
    }
    count = msp430_piezo_press_val_show(buf);
    SENSOR_D_LOG("end -return[%zd]",count);

    return count;
}
static ssize_t piezo_press_imit_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t count = 0;

    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_PIEZO_PRESS)  == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
//    count = piezo_press_imit_show(buf);
    return count;
}

static ssize_t piezo_press_imit_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_PIEZO_PRESS)  == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return count;
    }

//    piezo_press_imit_store(buf);
    SENSOR_D_LOG("end");

    return count;
}

static ssize_t piezo_press_properties_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t count = 0;

    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_PIEZO_PRESS)  == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
//    count = piezo_press_properties_show(buf);
    return count;
}

static ssize_t piezo_press_properties_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{

    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_PIEZO_PRESS)  == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return count;
    }

//    piezo_press_properties_store(buf);
    SENSOR_D_LOG("end");

    return count;
}

static ssize_t piezo_press_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
	int32_t count=0;
    
	SENSOR_TI_LOG("piezo_press_data_show_start");
	count=ti_msp430_data(buf);
    SENSOR_TI_LOG("piezo_press_data_show_end");
    return count;
}

void piezo_press_driver_init( void )
{
    int ret = 0;

    SENSOR_D_LOG("start");

    ret = sensor_input_init( &piezo_press_input_info );
    SENSOR_D_LOG("sensor_input_init()-->ret[%d]", ret );

    if( (0 != ret) || (NULL == (piezo_press_input_info.dev))) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        SENSOR_ERR_LOG("end return[%d]",-ENODEV);
        return;
    }

    ret = misc_register(&piezo_press_device);
    if (ret) {
        SENSOR_ERR_LOG("fail misc_register()-->ret[%d]",(int)ret);
        return;
    }

    SENSOR_D_LOG("end");
    return;
}

EXPORT_SYMBOL(piezo_press_driver_init);
