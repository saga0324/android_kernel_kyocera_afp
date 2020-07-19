/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 * (C) 2017 KYOCERA Corporation
 * (C) 2018 KYOCERA Corporation
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
#include <linux/ioctl.h>
#include "sensor_driver.h"
#include "sensor_micon_driver.h"
#include "sensor_com.h"

#define DEVICE_FILE_NAME    "ps_sensor"

#define D_NV_DATA_MAX                           (0x55)
#define GP2AP_DEV_STATUS_SUSPEND_INT            0x00000002
#define PROXIMITY_DUMMY_DATA                    (-1)

/* KIND OF PS CAL */
#define PS_CAL_NORMAL       1

typedef struct _t_ps_ioctl_detection
{
    unsigned long ulps_detection;
}T_PS_IOCTL_DETECTION;

typedef struct _t_ps_ioctl_nv
{
    unsigned long ulLength;
    unsigned char ucData[D_NV_DATA_MAX];
    unsigned long ulItem;
}T_PS_IOCTL_NV;

typedef struct _t_ps_ioctl_report_thresh
{
    uint8_t set;
}T_PS_IOCTL_THRESHOLD;

#define PS_IO             'A'
#define IOCTL_PS_DETECTION_GET        _IOR(PS_IO, 0x01, T_PS_IOCTL_DETECTION)
#define IOCTL_PS_THRESHOLD_SET        _IOW(PS_IO, 0x02, T_PS_IOCTL_THRESHOLD)
#define IOCTL_PS_NV_DATA_SET          _IOW(PS_IO, 0x03, T_PS_IOCTL_NV)
#define IOCTL_PS_NV_DATA_GET          _IOR(PS_IO, 0x04, T_PS_IOCTL_NV)

static int prox_open(struct inode *inode_type, struct file *file);
static int prox_release(struct inode *inode_type, struct file *file);
static long prox_ioctl(struct file *file_type,
    unsigned int unCmd,
    unsigned long ulArg );

static ssize_t prox_enable_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t prox_enable_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t prox_count_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t prox_flush_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t prox_value_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t prox_status_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t prox_properties_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t prox_properties_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t prox_valid_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t prox_valid_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t prox_calib_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t prox_calib_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t prox_thresh_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t prox_calresult_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );

#ifdef CONFIG_INPUT_SENSOR_APDS9960
static ssize_t prox_err_monitor_enable_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t prox_err_monitor_enable_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t prox_irq_enable_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
#endif /*CONFIG_INPUT_SENSOR_APDS9960*/

static void prox_set_input_params( struct input_dev *dev );

static DEVICE_ATTR( enable,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    prox_enable_show,
    prox_enable_store
);
static DEVICE_ATTR( flush,
    S_IWUSR|S_IWGRP,
    NULL,
    prox_flush_store
);
static DEVICE_ATTR( ps_count,
    S_IRUSR|S_IRGRP,
    prox_count_show,
    NULL
);
static DEVICE_ATTR( value,
    S_IRUSR|S_IRGRP,
    prox_value_show,
    NULL
);
static DEVICE_ATTR( status,
    S_IRUSR|S_IRGRP,
    prox_status_show,
    NULL
);
static DEVICE_ATTR( prop,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    prox_properties_show,
    prox_properties_store
);
static DEVICE_ATTR( valid,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    prox_valid_show,
    prox_valid_store
);
static DEVICE_ATTR( calib,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    prox_calib_show,
    prox_calib_store
);
static DEVICE_ATTR( thresh,
    S_IRUSR|S_IRGRP,
    prox_thresh_show,
    NULL
);
static DEVICE_ATTR( calib_result,
    S_IRUSR|S_IWUSR,
    NULL,
    prox_calresult_store
);

#ifdef CONFIG_INPUT_SENSOR_APDS9960
static DEVICE_ATTR( err_monitor,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    prox_err_monitor_enable_show,
    prox_err_monitor_enable_store
);
static DEVICE_ATTR( irq_status,
    S_IRUSR|S_IWUSR,
    NULL,
    prox_irq_enable_store
);
#endif /*CONFIG_INPUT_SENSOR_APDS9960*/


static struct attribute *prox_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_ps_count.attr,
    &dev_attr_flush.attr,
    &dev_attr_value.attr,
    &dev_attr_status.attr,
    &dev_attr_prop.attr,
    &dev_attr_valid.attr,
    &dev_attr_calib.attr,
    &dev_attr_thresh.attr,
    &dev_attr_calib_result.attr,
#ifdef CONFIG_INPUT_SENSOR_APDS9960
    &dev_attr_err_monitor.attr,
    &dev_attr_irq_status.attr,
#endif /*CONFIG_INPUT_SENSOR_APDS9960*/
    NULL
};

static struct attribute_group prox_attr_grp = {
    .attrs = prox_attributes
};

struct sensor_input_info_str prox_input_info =
{
    NULL,
    prox_set_input_params,
    &prox_attr_grp,
};

static struct file_operations prox_fops = {
    .owner = THIS_MODULE,
    .open = prox_open,
    .release = prox_release,
    .unlocked_ioctl = prox_ioctl,
};

static struct miscdevice prox_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = DEVICE_FILE_NAME,
    .fops = &prox_fops,
};

static uint32_t prox_last_read_data = 0;
static DEFINE_MUTEX(prox_event_mutex);
static atomic_t g_prox_cal_result;
static uint8_t g_prox_valid = 1;

static int prox_open(struct inode *inode_type, struct file *file)
{
    SENSOR_D_LOG("start");
    SENSOR_D_LOG("end");
    return 0;
}

static int prox_release(struct inode *inode_type, struct file *file)
{
    SENSOR_D_LOG("start");
    SENSOR_D_LOG("end");
    return 0;
}


static long prox_ioctl(struct file *file_type,
    unsigned int unCmd,
    unsigned long ulArg )
{
    s32 nRet = -EINVAL;
    T_PS_IOCTL_DETECTION ps_detection_type;
    T_PS_IOCTL_NV sensor_nv_type;

    SENSOR_D_LOG("start");

    if(sensor_get_initialize_state(SENSOR_PROX) == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return -EFAULT;
    }

    memset((void*)&ps_detection_type, 0,
                        sizeof(T_PS_IOCTL_DETECTION));

   switch (unCmd)
   {
        case IOCTL_PS_DETECTION_GET:
            SENSOR_A_LOG("case IOCTL_PS_DETECTION_GET");
            nRet = copy_from_user(&ps_detection_type, 
                    (void __user *)ulArg, sizeof(T_PS_IOCTL_DETECTION));
            if (nRet)
            {
                SENSOR_ERR_LOG("copy_from_user err:%d", nRet);
                return -EFAULT;
            }
            ps_detection_type.ulps_detection = prox_last_read_data;
            nRet = copy_to_user((void *)(ulArg),
                     &ps_detection_type, sizeof(T_PS_IOCTL_DETECTION));
            if (nRet)
            {
                SENSOR_ERR_LOG("copy_to_user err:%d", nRet);
                return -EFAULT;
            }
            break;
        case IOCTL_PS_NV_DATA_SET:
            SENSOR_A_LOG("case IOCTL_PS_NV_DATA_SET");
            memset((void*)&sensor_nv_type, 0,sizeof(T_PS_IOCTL_NV));
            nRet = copy_from_user( &sensor_nv_type,
                                   (void __user *)ulArg,
                                   sizeof(T_PS_IOCTL_NV) );
            if (!nRet){
                //gp2ap_set_ioctl_sensor_nv((unsigned long)&sensor_nv_type);
                SENSOR_ERR_LOG("copy_from_user err:%d", nRet);
                nRet = 0;
            }
            break;
        case IOCTL_PS_THRESHOLD_SET:
            SENSOR_A_LOG("case IOCTL_PS_THRESHOLD_SET");
            sensor_set_ps_threshold();
            break;
        default:
            SENSOR_A_LOG("default");
            SENSOR_ERR_LOG("invalid unCmd:%d", unCmd);
            break;
    }

    SENSOR_D_LOG("end");
    return nRet;
}

static ssize_t prox_flush_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    SENSOR_D_LOG("start");

    mutex_lock(&prox_event_mutex);
    input_report_abs(prox_input_info.dev, ABS_MISC+3, ~PROXIMITY_DUMMY_DATA);
    input_report_abs(prox_input_info.dev, ABS_MISC+3, PROXIMITY_DUMMY_DATA);
    input_sync(prox_input_info.dev);
    mutex_unlock(&prox_event_mutex);

    SENSOR_D_LOG("end - return[%zd]",count);
    return count;
}

static ssize_t prox_enable_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int enable = 0;
    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_PROX) == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
    enable = sensor_get_status(SENSOR_PROX);
    SENSOR_D_LOG("end ->enable[%d]",enable);
    return sprintf(buf, "%d\n", enable);
}

static ssize_t prox_enable_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    unsigned long enable = simple_strtoul(buf, NULL, 10);
    SENSOR_D_LOG("start");

    if(sensor_get_initialize_state(SENSOR_PROX) == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return count;
    }

    sensor_enable(SENSOR_PROX, NULL ,(bool)enable);
    if( enable == 0 && prox_last_read_data != 0 ) {
        prox_interrupt(0);
    }

    SENSOR_D_LOG("end");
    return count;
}

static ssize_t prox_count_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int enable = 0;
    uint32_t ps_count = 0;
    SENSOR_D_LOG("start");

    if(sensor_get_initialize_state(SENSOR_PROX) == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
    enable = sensor_get_status(SENSOR_PROX);
    if(!enable)
    {
        SENSOR_ERR_LOG("end:ProximitySensor is disable!!!");
        return sprintf(buf, "ProximitySensor is disable.\n" );
    }
#if defined(CONFIG_INPUT_SENSOR_RPR0521) || defined(CONFIG_INPUT_SENSOR_STK3338) || defined(CONFIG_INPUT_SENSOR_APDS9960) || defined(CONFIG_INPUT_SENSOR_GP2AP)
    ps_count = ps_sensor_get_count();
#else
    SENSOR_D_LOG("Not Supported!");
#endif

    SENSOR_D_LOG("end");
    return sprintf(buf, "0x%04x\n", ps_count);
}

static ssize_t prox_value_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t count = 0;
    SENSOR_D_LOG("start");

    if(sensor_get_initialize_state(SENSOR_PROX) == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
#if defined(CONFIG_INPUT_SENSOR_RPR0521) || defined(CONFIG_INPUT_SENSOR_STK3338) || defined(CONFIG_INPUT_SENSOR_APDS9960) || defined(CONFIG_INPUT_SENSOR_GP2AP)
    count = ps_val_show(buf);
#else
    count = sprintf(buf, "Not Supported!\n" );
#endif
    SENSOR_D_LOG("end");

    return count;
}

static ssize_t prox_status_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t count = 0;
    SENSOR_D_LOG("start");

    if(sensor_get_initialize_state(SENSOR_PROX) == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
#if defined(CONFIG_INPUT_SENSOR_RPR0521) || defined(CONFIG_INPUT_SENSOR_STK3338) || defined(CONFIG_INPUT_SENSOR_APDS9960)
    count = ps_status_show(buf);
#else
    count = sprintf(buf, "Not Supported!\n" );
#endif
    SENSOR_D_LOG("end");

    return count;
}
static ssize_t prox_properties_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t count = 0;
    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_PROX) == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
#if defined(CONFIG_INPUT_SENSOR_RPR0521) || defined(CONFIG_INPUT_SENSOR_STK3338) || defined(CONFIG_INPUT_SENSOR_APDS9960) || defined(CONFIG_INPUT_SENSOR_GP2AP)
    count = ps_properties_show(buf);
#else
    count = sprintf(buf, "Not Supported!\n" );
#endif
    SENSOR_D_LOG("end");
    return count;
}

static ssize_t prox_properties_store(
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
#if defined(CONFIG_INPUT_SENSOR_RPR0521) || defined(CONFIG_INPUT_SENSOR_STK3338) || defined(CONFIG_INPUT_SENSOR_APDS9960) || defined(CONFIG_INPUT_SENSOR_GP2AP)
    ps_properties_store(buf);
#else
    SENSOR_D_LOG("Not Supported!");
#endif
    SENSOR_D_LOG("end");
    return count;
}
static ssize_t prox_valid_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t count = 0;

    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_PROX) == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
#if defined(CONFIG_INPUT_SENSOR_RPR0521) || defined(CONFIG_INPUT_SENSOR_STK3338) || defined(CONFIG_INPUT_SENSOR_APDS9960)
    count = ps_valid_show(buf);
#else
    count += scnprintf(buf, PAGE_SIZE - count,
             "%u\n", g_prox_valid);
#endif
    SENSOR_D_LOG("end");
    return count;
}

static ssize_t prox_valid_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    int32_t ret;
    unsigned long value;

    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_PROX) == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return count;
    }
    ret = kstrtoul(buf, 10, &value);
    SENSOR_D_LOG("kstrtoul() ret[%d]->value[%d]",ret, (int)value);
#if defined(CONFIG_INPUT_SENSOR_RPR0521) || defined(CONFIG_INPUT_SENSOR_STK3338) || defined(CONFIG_INPUT_SENSOR_APDS9960) || defined(CONFIG_INPUT_SENSOR_GP2AP)
    ps_valid_store(value);
#else
    if (!value && g_prox_valid) {
        prox_interrupt(0);
        g_prox_valid = value;
    }
    else if (value && !g_prox_valid) {
        g_prox_valid = value;
        prox_interrupt(prox_last_read_data);
    }
#endif
    if (ret < 0){
        SENSOR_ERR_LOG("kstrtoul()-->ret[%d]",(int)ret);
    }
    return count;
}
static ssize_t prox_calib_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t count = 0;
    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_PROX) == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
    count = sprintf(buf, "%d\n", atomic_read(&g_prox_cal_result));

    SENSOR_D_LOG("end");
    return count;
}

static ssize_t prox_calib_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    unsigned long calib_kind = PS_CAL_NORMAL;

    SENSOR_D_LOG("start");

    if(sensor_get_initialize_state(SENSOR_PROX) == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        atomic_set(&g_prox_cal_result, PS_CAL_ERR_OTHER);
        return count;
    }
    atomic_set(&g_prox_cal_result, PS_CALIBRATING);
    switch(calib_kind){
        case PS_CAL_NORMAL :
#if defined(CONFIG_INPUT_SENSOR_RPR0521) || defined(CONFIG_INPUT_SENSOR_STK3338)
            ps_calib_start();
#else
            SENSOR_D_LOG("This function is implemented only RPR0521");
#endif
            break;
        default:
            break;
    }
    SENSOR_D_LOG("end [%d]", atomic_read(&g_prox_cal_result));
    return count;
}

static ssize_t prox_thresh_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t count = 0;
#if defined(CONFIG_INPUT_SENSOR_RPR0521) || defined(CONFIG_INPUT_SENSOR_STK3338)
    uint16_t th_detect = 0;
    uint16_t th_no_detect = 0;
    uint16_t distance_th = 0;
#endif

    SENSOR_D_LOG("start");
#if defined(CONFIG_INPUT_SENSOR_RPR0521) || defined(CONFIG_INPUT_SENSOR_STK3338)
    if(sensor_get_initialize_state(SENSOR_PROX) == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
    ps_get_threshold(&th_detect, &th_no_detect, &distance_th);
    count = sprintf(buf, "%x %x %x\n", th_detect, th_no_detect, distance_th);
#else
    count = sprintf(buf, "Not Supported.");
#endif
    SENSOR_D_LOG("end");
    return count;
}
static ssize_t prox_calresult_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    unsigned long dbg_res;
    int32_t ret;

    SENSOR_D_LOG("start");
    ret = kstrtoul(buf, 10, &dbg_res);
    atomic_set(&g_prox_cal_result, (int32_t)dbg_res);
    SENSOR_D_LOG("end");

    return count;
}

#ifdef CONFIG_INPUT_SENSOR_APDS9960
static ssize_t prox_err_monitor_enable_show(
    struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    ssize_t count = 0;

    SENSOR_D_LOG("start");
    if(sensor_get_initialize_state(SENSOR_PROX) == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
    count = apds_err_monitor_enable_show(buf);
    SENSOR_D_LOG("end");
    return count;
}

static ssize_t prox_err_monitor_enable_store(
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
    apds_err_monitor_enable_store(buf);
    SENSOR_D_LOG("end");
    return count;
}

static ssize_t prox_irq_enable_store(
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
    apds_irq_enable_store(buf);
    SENSOR_D_LOG("end");
    return count;
}
#endif /*CONFIG_INPUT_SENSOR_APDS9960*/

static void prox_cal_send_uevent(struct input_dev *dev)
{
    char *envp[] = {"PS_CALIB_UEVENT", NULL };

    SENSOR_D_LOG("start");
    if (!dev) {
        SENSOR_ERR_LOG("dev NULL");
        return;
    }

    kobject_uevent_env(&dev->dev.kobj, KOBJ_CHANGE, envp);
    SENSOR_D_LOG("dev = 0x%lx, &dev->dev.kobj = 0x%lx"
                    ,(unsigned long)dev, (unsigned long)&dev->dev.kobj);

    SENSOR_D_LOG("end");
}

void prox_cal_result_set(int8_t result)
{
    SENSOR_D_LOG("start");
    atomic_set(&g_prox_cal_result, result);
    SENSOR_D_LOG("end");
}

void prox_cal_interrupt(int8_t cal_result)
{
    SENSOR_D_LOG("start");

    mutex_lock(&prox_event_mutex);
    if(cal_result == PS_CAL_OK){
        prox_cal_send_uevent(prox_input_info.dev);
    }
    mutex_unlock(&prox_event_mutex);

    SENSOR_D_LOG("end");
}

void prox_interrupt(uint32_t data)
{
    SENSOR_D_LOG("start");

    if (g_prox_valid) {
        mutex_lock(&prox_event_mutex);
        sensor_report_abs_current_time(prox_input_info.dev);
        input_report_abs(prox_input_info.dev, ABS_DISTANCE, PROXIMITY_DUMMY_DATA);
        input_report_abs(prox_input_info.dev, ABS_DISTANCE, data);
        input_sync(prox_input_info.dev);
        mutex_unlock(&prox_event_mutex);

        SENSOR_ERR_LOG("Report Proximity info [%d]", data);
    }
    else {
        SENSOR_ERR_LOG("Skip Report Proximity");
    }

    prox_last_read_data = data;

    SENSOR_D_LOG("end");
    return;
}

static void prox_set_input_params( struct input_dev *dev )
{
    SENSOR_D_LOG("start");

    if(!dev){
        SENSOR_ERR_LOG("bad parm --> dev is NULL");
        return;
    }

    dev->name = "ps_sensor";
    dev->id.bustype = BUS_I2C;

    set_bit(EV_ABS, dev->evbit);
    input_set_capability(dev, EV_ABS, ABS_DISTANCE);
    input_set_capability(dev, EV_ABS, ABS_MISC+3);
    input_set_abs_params(dev, ABS_DISTANCE, 0, 1, 0, 0);
    input_set_abs_params(dev, ABS_MISC+3, INT_MIN, INT_MAX, 0, 0);

    SENSOR_D_LOG("end");
    return;
}

void prox_driver_init(void)
{
    int ret = 0;
    SENSOR_D_LOG("start");

    atomic_set(&g_prox_cal_result, PS_CAL_OK);
    ret = sensor_input_init( &prox_input_info );
    SENSOR_D_LOG("sensor_input_init()-->ret[%d]", ret);

    if( (0 != ret) || (NULL == (prox_input_info.dev))) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        SENSOR_ERR_LOG("end return[%d]",-ENODEV);
        return;
    }

    ret = misc_register(&prox_device);
    if (ret) {
        SENSOR_ERR_LOG("fail misc_register()-->ret[%d]",(int)ret);
        return;
    }

    SENSOR_D_LOG("end");
    return;
}

EXPORT_SYMBOL(prox_driver_init);
