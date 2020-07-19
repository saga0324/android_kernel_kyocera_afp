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

#define DEVICE_FILE_NAME    "piezo_wakeup_sensor"

#define PIEZO_WAKEUP_DEV_STATUS_SUSPEND_INT      0x00000002
#define PIEZO_WAKEUP_DUMMY_DATA                  (-1)

#define BIT0				0x01
#define BIT1				0x02
#define BIT2				0x04
#define BIT3				0x08
#define BIT4				0x10
#define BIT5				0x20
#define BIT6				0x40
#define BIT7				0x80
#define BIT_ALL				(BIT0|BIT1|BIT2|BIT3|BIT4|BIT5|BIT6|BIT7)
enum piezo_press_state {
	PIEZO_RELEASED	= 0,
	PIEZO_PRESSED,
};

static int piezo_wakeup_open(struct inode *inode_type, struct file *file);
static int piezo_wakeup_release(struct inode *inode_type, struct file *file);
static void piezo_wakeup_powkey_interrupt(void);
static void piezo_wakeup_lcdkey_interrupt(void);

static ssize_t piezo_wakeup_enable_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t piezo_wakeup_enable_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t piezo_wakeup_flush_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t piezo_wakeup_properties_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t piezo_wakeup_properties_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t piezo_wakeup_prop_powkey_th_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t piezo_wakeup_prop_powkey_th_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t piezo_wakeup_prop_lcdkey_th_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t piezo_wakeup_prop_lcdkey_th_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t piezo_wakeup_test_inputpwr_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t piezo_wakeup_test_inputlcd_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );

static void piezo_wakeup_set_input_params( struct input_dev *dev );

static DEVICE_ATTR( enable_piezo_wakeup_sensor,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    piezo_wakeup_enable_show,
    piezo_wakeup_enable_store
);
static DEVICE_ATTR( flush,
    S_IWUSR|S_IWGRP,
    NULL,
    piezo_wakeup_flush_store
);
static DEVICE_ATTR(prop,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    piezo_wakeup_properties_show,
    piezo_wakeup_properties_store
);
static DEVICE_ATTR( prop_powkey_th,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    piezo_wakeup_prop_powkey_th_show,
    piezo_wakeup_prop_powkey_th_store
);
static DEVICE_ATTR( prop_lcdkey_th,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    piezo_wakeup_prop_lcdkey_th_show,
    piezo_wakeup_prop_lcdkey_th_store
);
static DEVICE_ATTR( test_inputpwr,
    S_IWUSR|S_IWGRP,
    NULL,
    piezo_wakeup_test_inputpwr_store
);
static DEVICE_ATTR( test_inputlcd,
    S_IWUSR|S_IWGRP,
    NULL,
    piezo_wakeup_test_inputlcd_store
);

static struct attribute *piezo_wakeup_attributes[] = {
    &dev_attr_enable_piezo_wakeup_sensor.attr,
    &dev_attr_flush.attr,
    &dev_attr_prop.attr,
    &dev_attr_prop_powkey_th.attr,
    &dev_attr_prop_lcdkey_th.attr,
    &dev_attr_test_inputpwr.attr,
    &dev_attr_test_inputlcd.attr,
    NULL
};

static struct attribute_group piezo_wakeup_attr_grp = {
    .attrs = piezo_wakeup_attributes
};

struct sensor_input_info_str piezo_wakeup_input_info =
{
    NULL,
    piezo_wakeup_set_input_params,
    &piezo_wakeup_attr_grp,
};

static struct file_operations piezo_wakeup_fops = {
    .owner = THIS_MODULE,
    .open = piezo_wakeup_open,
    .release = piezo_wakeup_release,
};

static struct miscdevice piezo_wakeup_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = DEVICE_FILE_NAME,
    .fops = &piezo_wakeup_fops,
};

static DEFINE_MUTEX(piezo_wakeup_event_mutex);

static int piezo_wakeup_open(struct inode *inode_type, struct file *file)
{
    SENSOR_D_LOG("start");
    SENSOR_D_LOG("end");
    return 0;
}

static int piezo_wakeup_release(struct inode *inode_type, struct file *file)
{
    SENSOR_D_LOG("start");
    SENSOR_D_LOG("end");
    return 0;
}

static ssize_t piezo_wakeup_flush_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    SENSOR_D_LOG("start");

    mutex_lock(&piezo_wakeup_event_mutex);
    input_report_abs(piezo_wakeup_input_info.dev, ABS_MISC+3, ~PIEZO_WAKEUP_DUMMY_DATA);
    input_report_abs(piezo_wakeup_input_info.dev, ABS_MISC+3, PIEZO_WAKEUP_DUMMY_DATA);
    input_sync(piezo_wakeup_input_info.dev);
    mutex_unlock(&piezo_wakeup_event_mutex);

    SENSOR_D_LOG("end - return[%zd]",count);
    return count;
}

static ssize_t piezo_wakeup_properties_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t count = 0;

    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_PIEZO_WAKEUP)  == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
//    count = piezo_wakeup_properties_show(buf);
    return count;
}

static ssize_t piezo_wakeup_properties_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{

    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_PIEZO_WAKEUP)  == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return count;
    }

//    piezo_wakeup_properties_store(buf);
    SENSOR_D_LOG("end");

    return count;
}

static ssize_t piezo_wakeup_enable_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int enable = 0;
    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_PIEZO_WAKEUP) == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
    enable = sensor_get_status(SENSOR_PIEZO_WAKEUP);
    SENSOR_D_LOG("end ->enable[%d]",enable);
    return sprintf(buf, "%d\n", enable);
}

static ssize_t piezo_wakeup_enable_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    unsigned long enable = simple_strtoul(buf, NULL, 10);
    SENSOR_D_LOG("start");

    if(sensor_get_initialize_state(SENSOR_PIEZO_WAKEUP) == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return count;
    }

    sensor_enable(SENSOR_PIEZO_WAKEUP, NULL ,(bool)enable);

    SENSOR_D_LOG("end");
    return count;
}

static ssize_t piezo_wakeup_prop_powkey_th_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t count = 0;
    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_PIEZO_WAKEUP) == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
    count = msp430_piezo_wakeup_prop_powkey_th_show(buf);
    SENSOR_D_LOG("end");
    return count;
}
static ssize_t piezo_wakeup_prop_powkey_th_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    unsigned long thresh = 0;
    int ret = 0;
    SENSOR_D_LOG("start");

    if(sensor_get_initialize_state(SENSOR_PIEZO_WAKEUP) == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return count;
    }
    ret = kstrtoul(buf, 10, &thresh);
    SENSOR_D_LOG("kstrtoul() ret[%d]->thresh[%d]",ret, (int)thresh);
    msp430_piezo_wakeup_prop_powkey_th_store((unsigned int)thresh);
    SENSOR_D_LOG("end");
    return count;
}

static ssize_t piezo_wakeup_prop_lcdkey_th_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t count = 0;
    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_PIEZO_WAKEUP) == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
    count = msp430_piezo_wakeup_prop_lcdkey_th_show(buf);
    SENSOR_D_LOG("end");
    return count;
}
static ssize_t piezo_wakeup_prop_lcdkey_th_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    unsigned long thresh = 0;
    int ret = 0;
    SENSOR_D_LOG("start");

    if(sensor_get_initialize_state(SENSOR_PIEZO_WAKEUP) == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return count;
    }
    ret = kstrtoul(buf, 10, &thresh);
    SENSOR_D_LOG("kstrtoul() ret[%d]->thresh[%d]",ret, (int)thresh);
    msp430_piezo_wakeup_prop_lcdkey_th_store((unsigned int)thresh);
    SENSOR_D_LOG("end");
    return count;
}

static ssize_t piezo_wakeup_test_inputpwr_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    SENSOR_D_LOG("start");

    if(sensor_get_initialize_state(SENSOR_PIEZO_WAKEUP) == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return count;
    }
    piezo_wakeup_powkey_interrupt();
    SENSOR_D_LOG("end");
    return count;
}
static ssize_t piezo_wakeup_test_inputlcd_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    SENSOR_D_LOG("start");

    if(sensor_get_initialize_state(SENSOR_PIEZO_WAKEUP) == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return count;
    }
    piezo_wakeup_lcdkey_interrupt();
    SENSOR_D_LOG("end");
    return count;
}

static void piezo_wakeup_powkey_interrupt(void)
{
    SENSOR_D_LOG("start");

    SENSOR_TI_LOG("Report EdgeWakeUp [KEY_POWER_EVENT]");
    sensor_report_abs_current_time(piezo_wakeup_input_info.dev);
    input_report_abs(piezo_wakeup_input_info.dev, ABS_MISC, PIEZO_WAKEUP_DUMMY_DATA );
    input_report_abs(piezo_wakeup_input_info.dev, ABS_MISC, 1 );
    input_sync(piezo_wakeup_input_info.dev);

    SENSOR_D_LOG("end");
    return;
}

static void piezo_wakeup_lcdkey_interrupt(void)
{
    SENSOR_D_LOG("start");

    SENSOR_TI_LOG("Report EdgeWakeUp [KEY_LCD_EVENT]");
    sensor_report_abs_current_time(piezo_wakeup_input_info.dev);
    input_report_abs(piezo_wakeup_input_info.dev, ABS_MISC+4, PIEZO_WAKEUP_DUMMY_DATA );
    input_report_abs(piezo_wakeup_input_info.dev, ABS_MISC+4, 1 );
    input_sync(piezo_wakeup_input_info.dev);

    SENSOR_D_LOG("end");
    return;
}
void piezo_wake_input_report( uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4 ) //data1:VOL, data2:APP, data3:LCD data4:POW
{
    int enable = 0;
    SENSOR_D_LOG("start");

    enable = sensor_get_status(SENSOR_PIEZO_WAKEUP);
    SENSOR_D_LOG("enable[%d]",enable);

    if(enable) {
        mutex_lock(&piezo_wakeup_event_mutex);

        SENSOR_TI_LOG("V=%02x A=%02x L=%02x P=%02x", data1, data2, data3, data4);

        if(((data4 & BIT0) == BIT0) || ((data4 & BIT1) == BIT1)) //power key short press or long press
            piezo_wakeup_powkey_interrupt();

        if((data3 & BIT0) == BIT0) //lcd key press
            piezo_wakeup_lcdkey_interrupt();

        mutex_unlock(&piezo_wakeup_event_mutex);
    }

    SENSOR_D_LOG("end");
    return;
}

static void piezo_wakeup_set_input_params( struct input_dev *dev )
{
    SENSOR_D_LOG("start");

    if(!dev){
        SENSOR_ERR_LOG("bad parm --> dev is NULL");
        return;
    }

    dev->name = "piezo_wakeup_sensor";
    dev->id.bustype = BUS_I2C;

    set_bit(EV_ABS, dev->evbit);
    set_bit(KEY_POWER, dev->keybit);
    //set_bit(KEY_VOLUMEDOWN, dev->keybit);
    //set_bit(KEY_VOLUMEUP, dev->keybit);
    input_set_capability(dev, EV_ABS, ABS_MISC);
    input_set_capability(dev, EV_ABS, ABS_MISC+4);
    input_set_capability(dev, EV_KEY, KEY_POWER);
    //input_set_capability(dev, EV_KEY, KEY_VOLUMEDOWN);
    //input_set_capability(dev, EV_KEY, KEY_VOLUMEUP);
    input_set_abs_params(dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
    input_set_abs_params(dev, ABS_MISC+4, INT_MIN, INT_MAX, 0, 0);

    SENSOR_D_LOG("end");
    return;
}

void piezo_wakeup_driver_init(void)
{
    int ret = 0;
    SENSOR_D_LOG("start");

    ret = sensor_input_init( &piezo_wakeup_input_info );
    SENSOR_D_LOG("sensor_input_init()-->ret[%d]", ret);

    if( (0 != ret) || (NULL == (piezo_wakeup_input_info.dev))) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        SENSOR_ERR_LOG("end return[%d]",-ENODEV);
        return;
    }

    ret = misc_register(&piezo_wakeup_device);
    if (ret) {
        SENSOR_ERR_LOG("fail misc_register()-->ret[%d]",(int)ret);
        return;
    }

    SENSOR_D_LOG("end");
    return;
}

EXPORT_SYMBOL(piezo_wakeup_driver_init);
