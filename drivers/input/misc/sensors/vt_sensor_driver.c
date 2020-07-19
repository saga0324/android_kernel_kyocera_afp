/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2019 KYOCERA Corporation
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

#define DEVICE_FILE_NAME    "voice_trigger_sensor"
#define VOICE_TRIGGER_DUMMY_DATA                  (-1)

static int voice_trigger_open(struct inode *inode_type, struct file *file);
static int voice_trigger_release(struct inode *inode_type, struct file *file);

static ssize_t voice_trigger_enable_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t voice_trigger_enable_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t voice_trigger_flush_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);

static void voice_trigger_set_input_params( struct input_dev *dev );

static DEVICE_ATTR(enable,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    voice_trigger_enable_show,
    voice_trigger_enable_store
);
static DEVICE_ATTR( flush,
    S_IWUSR|S_IWGRP,
    NULL,
    voice_trigger_flush_store
);

static struct attribute *voice_trigger_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_flush.attr,
    NULL
};

static struct attribute_group voice_trigger_attr_grp = {
    .attrs = voice_trigger_attributes
};

struct sensor_input_info_str voice_trigger_input_info =
{
    NULL,
    voice_trigger_set_input_params,
    &voice_trigger_attr_grp,
};

static struct file_operations voice_trigger_fops = {
    .owner = THIS_MODULE,
    .open = voice_trigger_open,
    .release = voice_trigger_release,
};

static struct miscdevice voice_trigger_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = DEVICE_FILE_NAME,
    .fops = &voice_trigger_fops,
};

static DEFINE_MUTEX(voice_trigger_event_mutex);

static int voice_trigger_open(struct inode *inode_type, struct file *file)
{
    SENSOR_D_LOG("start");
    SENSOR_D_LOG("end");
    return 0;
}

static int voice_trigger_release(struct inode *inode_type, struct file *file)
{
    SENSOR_D_LOG("start");
    SENSOR_D_LOG("end");
    return 0;
}

static ssize_t voice_trigger_flush_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    SENSOR_D_LOG("start");

    mutex_lock(&voice_trigger_event_mutex);
    input_report_abs(voice_trigger_input_info.dev, ABS_MISC+3, ~VOICE_TRIGGER_DUMMY_DATA);
    input_report_abs(voice_trigger_input_info.dev, ABS_MISC+3, VOICE_TRIGGER_DUMMY_DATA);
    input_sync(voice_trigger_input_info.dev);
    mutex_unlock(&voice_trigger_event_mutex);

    SENSOR_D_LOG("end - return[%zd]",count);
    return count;
}

static ssize_t voice_trigger_enable_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int enable = 0;
    SENSOR_D_LOG("start");
    enable = sensor_get_status(SENSOR_VOICE_TRIGGER);
    SENSOR_D_LOG("end ->enable[%d]",enable);
    return sprintf(buf, "%d\n", enable);
}

static ssize_t voice_trigger_enable_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    unsigned long enable = simple_strtoul(buf, NULL, 10);
    SENSOR_D_LOG("start");
    sensor_enable(SENSOR_VOICE_TRIGGER, NULL ,(bool)enable);
    SENSOR_D_LOG("end");
    return count;
}

void voice_trigger_interrupt(uint8_t trigger_data)
{
    int enable = 0;
    SENSOR_D_LOG("start");

    enable = sensor_get_status(SENSOR_VOICE_TRIGGER);
    SENSOR_D_LOG("enable[%d]",enable);

    if(enable) {
        SENSOR_ERR_LOG("Voice Trigger occured! type[%d]",trigger_data);
        mutex_lock(&voice_trigger_event_mutex);
        sensor_report_abs_current_time(voice_trigger_input_info.dev);
        input_report_abs(voice_trigger_input_info.dev, ABS_MISC, VOICE_TRIGGER_DUMMY_DATA );
        input_report_abs(voice_trigger_input_info.dev, ABS_MISC, trigger_data);
        input_sync(voice_trigger_input_info.dev);
        mutex_unlock(&voice_trigger_event_mutex);
    }

    SENSOR_D_LOG("end");
    return;
}

static void voice_trigger_set_input_params( struct input_dev *dev )
{
    SENSOR_D_LOG("start");

    if(!dev){
        SENSOR_ERR_LOG("bad parm --> dev is NULL");
        return;
    }

    dev->name = "voice_trigger_sensor";
    dev->id.bustype = BUS_I2C;

    set_bit(EV_ABS, dev->evbit);
    set_bit(KEY_POWER, dev->keybit);
    input_set_capability(dev, EV_ABS, ABS_MISC);
    input_set_capability(dev, EV_ABS, ABS_MISC+4);
    input_set_abs_params(dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
    input_set_abs_params(dev, ABS_MISC+4, INT_MIN, INT_MAX, 0, 0);

    SENSOR_D_LOG("end");
    return;
}

void voice_trigger_driver_init(void)
{
    int ret = 0;
    SENSOR_D_LOG("start");

    ret = sensor_input_init( &voice_trigger_input_info );
    SENSOR_D_LOG("sensor_input_init()-->ret[%d]", ret);

    if( (0 != ret) || (NULL == (voice_trigger_input_info.dev))) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        SENSOR_ERR_LOG("end return[%d]",-ENODEV);
        return;
    }

    ret = misc_register(&voice_trigger_device);
    if (ret) {
        SENSOR_ERR_LOG("fail misc_register()-->ret[%d]",(int)ret);
        return;
    }

    SENSOR_D_LOG("end");
    return;
}
EXPORT_SYMBOL(voice_trigger_driver_init);
