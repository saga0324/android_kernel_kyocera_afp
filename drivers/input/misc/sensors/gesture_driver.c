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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/ioctl.h>
#include "sensor_driver.h"
#include "sensor_micon_driver.h"
#include "sensor_com.h"

#define DEVICE_FILE_NAME        "gs_sensor"
#define D_NV_DATA_MAX           (0x55)

typedef struct _t_gs_ioctl_mean_times
{
    unsigned long ulgs_mean_times;
}T_GS_IOCTL_MEAN_TIMES;

typedef struct _t_gs_ioctl_lux_ave
{
    unsigned long ulgs_lux_ave;
    long lcdata;
    long lirdata;
}T_GS_IOCTL_LUX_AVE;

typedef struct _t_gs_ioctl_nv
{
    unsigned long ulLength;
    unsigned char ucData[D_NV_DATA_MAX];
    unsigned long ulItem;
}T_GS_IOCTL_NV;

#define GS_ABS_MISC  (REL_MISC)
//#define GS_ABS_MISC  (ABS_MISC)

#define ABS_MISC_DUMMY_VAL (-1)

static int gesture_open(struct inode *inode_type, struct file *file);
static int gesture_release(struct inode *inode_type, struct file *file);
static ssize_t gesture_flush_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t gesture_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t gesture_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t gesture_delay_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t gesture_delay_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static void gesture_set_input_params( struct input_dev *dev );
static ssize_t gesture_val_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t gesture_ratio_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t gesture_d0d1_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t gesture_imit_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t gesture_imit_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t gesture_status_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t gesture_properties_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t gesture_properties_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t gs_calibration_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t gesture_ignore_time_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t gesture_ignore_time_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );

static DEVICE_ATTR(enable_gs_sensor,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    gesture_enable_show,
    gesture_enable_store
);
static DEVICE_ATTR(gs_poll_delay,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    gesture_delay_show,
    gesture_delay_store
);
static DEVICE_ATTR(gs_lux,
    S_IRUSR|S_IRGRP,
    gesture_val_show,
    NULL
);
static DEVICE_ATTR(gs_ratio,
    S_IRUSR|S_IRGRP,
    gesture_ratio_show,
    NULL
);
static DEVICE_ATTR(gs_d0d1,
    S_IRUSR|S_IRGRP,
    gesture_d0d1_show,
    NULL
);
static DEVICE_ATTR(gs_imitation,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    gesture_imit_show,
    gesture_imit_store
);
static DEVICE_ATTR( flush,
    S_IWUSR|S_IWGRP,
    NULL,
    gesture_flush_store
);
static DEVICE_ATTR(status,
    S_IRUSR|S_IRGRP,
    gesture_status_show,
    NULL
);
static DEVICE_ATTR( prop,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    gesture_properties_show,
    gesture_properties_store
);
static DEVICE_ATTR( gs_calibration,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    NULL,
    gs_calibration_store
);
static DEVICE_ATTR( gs_ignr_time,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    gesture_ignore_time_show,
    gesture_ignore_time_store
);

static struct attribute *gesture_attributes[] = {
    &dev_attr_enable_gs_sensor.attr,
    &dev_attr_gs_poll_delay.attr,
    &dev_attr_gs_lux.attr,
    &dev_attr_gs_ratio.attr,
    &dev_attr_gs_d0d1.attr,
    &dev_attr_gs_imitation.attr,
    &dev_attr_flush.attr,
    &dev_attr_status.attr,
    &dev_attr_prop.attr,
    &dev_attr_gs_calibration.attr,
    &dev_attr_gs_ignr_time.attr,
    NULL
};

static struct attribute_group gesture_attr_grp = {
    .attrs = gesture_attributes
};

struct sensor_input_info_str gesture_input_info =
{
    NULL,
    gesture_set_input_params,
    &gesture_attr_grp,
};

static struct file_operations gesture_fops = {
    .owner = THIS_MODULE,
    .open = gesture_open,
    .release = gesture_release,
};

static struct miscdevice gesture_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = DEVICE_FILE_NAME,
    .fops = &gesture_fops,
};

//static uint32_t gesture_last_read_data = 0;
static DEFINE_MUTEX(gesture_event_mutex);

static int gesture_open(struct inode *inode_type, struct file *file)
{
    SENSOR_D_LOG("start");
    SENSOR_D_LOG("end");
    return 0;
}

static int gesture_release(struct inode *inode_type, struct file *file)
{
    SENSOR_D_LOG("start");
    SENSOR_D_LOG("end");
    return 0;
}

static ssize_t gesture_flush_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    SENSOR_D_LOG("start");

/*
    mutex_lock(&gesture_event_mutex);
    input_report_abs(gesture_input_info.dev, ABS_MISC+3, ~(-1));
    input_report_abs(gesture_input_info.dev, ABS_MISC+3, (-1));
    input_sync(gesture_input_info.dev);
    mutex_unlock(&gesture_event_mutex);
*/
    SENSOR_D_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t gesture_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int enable = 0;
    SENSOR_D_LOG("start");

    if(sensor_get_initialize_state(SENSOR_GESTURE)  == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }

    enable = sensor_get_status(SENSOR_GESTURE);

    SENSOR_D_LOG("end ->enable[%d]",enable);
    return sprintf(buf, "%d\n", enable);
}

static ssize_t gesture_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    unsigned long enable = 0;
    int ret = 0;

    SENSOR_D_LOG("start");

    if(sensor_get_initialize_state(SENSOR_GESTURE)  == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return count;
    }

    ret = kstrtoul(buf, 10, &enable);
    SENSOR_D_LOG("kstrtoul() ret[%d]->enable[%d]",ret, (int)enable);

    sensor_enable( SENSOR_GESTURE, NULL, (bool)enable );

    SENSOR_D_LOG("end - return[%zd]",count);
    return count;
}

static ssize_t gesture_delay_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int32_t delay = 0;
    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_GESTURE)  == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }

    SENSOR_D_LOG("end ->delay[%d]",(int)delay);
    return sprintf(buf, "%d\n", delay);
}

static ssize_t gesture_delay_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    unsigned long delay = 0;
    int ret = 0;
    SENSOR_D_LOG("start");

    if(sensor_get_initialize_state(SENSOR_GESTURE)  == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return count;
    }

    ret = kstrtoul(buf, 10, &delay);
    SENSOR_D_LOG("kstrtoul() ret[%d]->delay[%d]",ret, (int)delay);

    SENSOR_D_LOG("end - return[%zu]",count);
    return count;
}


void gesture_interrupt(uint32_t data)
{
    SENSOR_D_LOG("start param[%08X]",(int)data);

    mutex_lock(&gesture_event_mutex);

    input_report_rel(gesture_input_info.dev, REL_MISC+1, data);
    input_sync(gesture_input_info.dev);
    mutex_unlock(&gesture_event_mutex);

    SENSOR_D_LOG("end");
    return;
}
void gesture_input_report(uint32_t data)
{
    SENSOR_D_LOG("start param[%08X]",(int)data);

    mutex_lock(&gesture_event_mutex);

    input_report_rel(gesture_input_info.dev, REL_MISC, data);
    input_sync(gesture_input_info.dev);
    mutex_unlock(&gesture_event_mutex);

    SENSOR_D_LOG("end");
    return;
}

static void gesture_set_input_params( struct input_dev *dev )
{
    SENSOR_D_LOG("start");

    if(!dev){
        SENSOR_ERR_LOG("bad parm --> dev is NULL");
        return;
    }

    dev->name = "gesture";
    dev->id.bustype = BUS_I2C;

    set_bit(EV_REL, dev->evbit);
    input_set_capability(dev, EV_REL, REL_MISC);
    input_set_capability(dev, EV_REL, REL_MISC+1);

    SENSOR_D_LOG("end");
    return;
}

static ssize_t gesture_status_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t count = 0;
    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_GESTURE)  == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
    SENSOR_D_LOG("end");
    return count;
}

static ssize_t gesture_val_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t count = 0;
	int ret = 0;
    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_GESTURE)  == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return ret;
    }
    SENSOR_D_LOG("end -return[%zd]",count);

    return count;
}
static ssize_t gesture_ratio_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
	ssize_t count = 0;
    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_GESTURE)  == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
    SENSOR_D_LOG("end");
    return count;
}
static ssize_t gesture_d0d1_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t count = 0;
    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_GESTURE)  == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
    SENSOR_D_LOG("end");
    return count;
}
static ssize_t gesture_imit_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t count = 0;

    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_GESTURE)  == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
    return count;
}

static ssize_t gesture_imit_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_GESTURE)  == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return count;
    }

    SENSOR_D_LOG("end");

    return count;
}

static ssize_t gesture_properties_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t count = 0;

    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_GESTURE)  == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
    return count;
}

static ssize_t gesture_properties_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{

    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_GESTURE)  == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return count;
    }

    SENSOR_D_LOG("end");

    return count;
}

static ssize_t gs_calibration_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    SENSOR_D_LOG("start");

    if(sensor_get_initialize_state(SENSOR_PROX) == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return count;
    }
    apds_gs_calibration();
    SENSOR_D_LOG("end");
    return count;
}

static ssize_t gesture_ignore_time_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    uint32_t ignr_time = 0;

    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_GESTURE)  == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
    ignr_time = gs_get_ignore_time();

    SENSOR_D_LOG("end ->ignr_time[%d]", ignr_time);
    return sprintf(buf, "%d\n", ignr_time);
}

static ssize_t gesture_ignore_time_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    unsigned long time_ms = 0;
    int ret = 0;
    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_GESTURE)  == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return count;
    }
    ret = kstrtoul(buf, 10, &time_ms);
    SENSOR_D_LOG("kstrtoul() ret[%d]->time_ms[%d]",ret, (uint32_t)time_ms);
    gs_set_ignore_time((uint32_t)time_ms);

    SENSOR_D_LOG("end");

    return count;
}

void gesture_driver_init( void )
{
    int ret = 0;

    SENSOR_ERR_LOG("start");

    ret = sensor_input_init( &gesture_input_info );
    SENSOR_D_LOG("sensor_input_init()-->ret[%d]", ret );

    if( (0 != ret) || (NULL == (gesture_input_info.dev))) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        SENSOR_ERR_LOG("end return[%d]",-ENODEV);
        return;
    }

    ret = misc_register(&gesture_device);
    if (ret) {
        SENSOR_ERR_LOG("fail misc_register()-->ret[%d]",(int)ret);
        return;
    }

    SENSOR_D_LOG("end");
    return;
}

EXPORT_SYMBOL(gesture_driver_init);

