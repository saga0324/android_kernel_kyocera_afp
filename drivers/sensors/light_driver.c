/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
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

#define DEVICE_FILE_NAME        "als_sensor"
#define D_NV_DATA_MAX           (0x55)

typedef struct _t_als_ioctl_mean_times
{
    unsigned long ulals_mean_times;
}T_ALS_IOCTL_MEAN_TIMES;

typedef struct _t_als_ioctl_lux_ave
{
    unsigned long ulals_lux_ave;
    long lcdata;
    long lirdata;
}T_ALS_IOCTL_LUX_AVE;

typedef struct _t_als_ioctl_nv
{
    unsigned long ulLength;
    unsigned char ucData[D_NV_DATA_MAX];
    unsigned long ulItem;
}T_ALS_IOCTL_NV;

#define ALS_IO             'A'
#define IOCTL_ALS_MEAN_TIMES_SET      _IOW(ALS_IO, 0x01, T_ALS_IOCTL_MEAN_TIMES)
#define IOCTL_ALS_LUX_AVE_GET         _IOR(ALS_IO, 0x02, T_ALS_IOCTL_LUX_AVE)
#define IOCTL_ALS_NV_DATA_SET         _IOW(ALS_IO, 0x03, T_ALS_IOCTL_NV)
#define IOCTL_ALS_NV_DATA_GET         _IOR(ALS_IO, 0x04, T_ALS_IOCTL_NV)


static int light_open(struct inode *inode_type, struct file *file);
static int light_release(struct inode *inode_type, struct file *file);
static long light_ioctl(struct file *file_type,
    unsigned int unCmd,
    unsigned long ulArg );
static ssize_t light_flush_store(
    struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t light_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t light_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t light_delay_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t light_delay_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static void light_set_input_params( struct input_dev *dev );
static ssize_t light_val_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t light_ratio_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t light_d0d1_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t light_imit_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t light_imit_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
#ifdef CONFIG_KC_CORRECT_LUXVALUE
static ssize_t light_mode_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
#endif /* CONFIG_KC_CORRECT_LUXVALUE */

static DEVICE_ATTR(enable_als_sensor,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    light_enable_show,
    light_enable_store
);
static DEVICE_ATTR(als_poll_delay,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    light_delay_show,
    light_delay_store
);
static DEVICE_ATTR(als_lux,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    light_val_show,
    NULL
);
static DEVICE_ATTR(als_ratio,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    light_ratio_show,
    NULL
);
static DEVICE_ATTR(als_d0d1,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    light_d0d1_show,
    NULL
);
static DEVICE_ATTR(als_imitation,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    light_imit_show,
    light_imit_store
);
static DEVICE_ATTR( flush,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    NULL,
    light_flush_store
);
#ifdef CONFIG_KC_CORRECT_LUXVALUE
static DEVICE_ATTR(als_mode,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    light_mode_show,
    NULL
);
#endif /* CONFIG_KC_CORRECT_LUXVALUE */

static struct attribute *light_attributes[] = {
    &dev_attr_enable_als_sensor.attr,
    &dev_attr_als_poll_delay.attr,
    &dev_attr_als_lux.attr,
    &dev_attr_als_ratio.attr,
    &dev_attr_als_d0d1.attr,
    &dev_attr_als_imitation.attr,
#ifdef CONFIG_KC_CORRECT_LUXVALUE
    &dev_attr_als_mode.attr,
#endif /* CONFIG_KC_CORRECT_LUXVALUE */
    &dev_attr_flush.attr,
    NULL
};

static struct attribute_group light_attr_grp = {
    .attrs = light_attributes
};

struct sensor_input_info_str light_input_info =
{
    NULL,
    light_set_input_params,
    &light_attr_grp,
};

static struct file_operations light_fops = {
    .owner = THIS_MODULE,
    .open = light_open,
    .release = light_release,
    .unlocked_ioctl = light_ioctl,
};

static struct miscdevice light_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = DEVICE_FILE_NAME,
    .fops = &light_fops,
};

static uint32_t light_last_read_data = 0;
static DEFINE_MUTEX(light_event_mutex);

static int light_open(struct inode *inode_type, struct file *file)
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end");
    return 0;
}

static int light_release(struct inode *inode_type, struct file *file)
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end");
    return 0;
}

static long light_ioctl(struct file *file_type,
    unsigned int unCmd,
    unsigned long ulArg )
{
    u32 als_lux_ave = 0;
    s32 cdata = 0;
    s32 irdata = 0;
    s32 nRet = -EINVAL;
    T_ALS_IOCTL_NV sensor_nv_type;
    T_ALS_IOCTL_MEAN_TIMES als_mean_times_type;
    T_ALS_IOCTL_LUX_AVE als_lux_ave_type;

    SENSOR_N_LOG("start");

    if(gp2ap_get_initialize_state() == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return -EFAULT;
    }

    memset((void*)&als_mean_times_type, 0,
            sizeof(T_ALS_IOCTL_MEAN_TIMES));
    memset((void*)&als_lux_ave_type, 0,
            sizeof(T_ALS_IOCTL_LUX_AVE));

    switch (unCmd)
    {
        case IOCTL_ALS_NV_DATA_SET:
            SENSOR_A_LOG("case IOCTL_ALS_NV_DATA_SET");
            memset((void*)&sensor_nv_type, 0,sizeof(T_ALS_IOCTL_NV));
            nRet = copy_from_user( &sensor_nv_type,
                                   (void __user *)ulArg,
                                   sizeof(T_ALS_IOCTL_NV) );
            if (!nRet){
                gp2ap_set_ioctl_sensor_nv((unsigned long)&sensor_nv_type);
                nRet = 0;
            }
            break;
        case IOCTL_ALS_MEAN_TIMES_SET:
            SENSOR_A_LOG("case IOCTL_ALS_MEAN_TIMES_SET");
            nRet = copy_from_user(&als_mean_times_type, 
                    (void __user *)ulArg, sizeof(T_ALS_IOCTL_MEAN_TIMES));
            if (nRet){
                return -EFAULT;
            }
            gp2ap_set_ioctl_als_mean_times(als_mean_times_type.ulals_mean_times);
            nRet = copy_to_user((void *)(ulArg),
                     &als_mean_times_type, sizeof(T_ALS_IOCTL_MEAN_TIMES));
            if (nRet){
                return -EFAULT;
            }
            break;
        case IOCTL_ALS_LUX_AVE_GET:
            SENSOR_A_LOG("case IOCTL_ALS_LUX_AVE_GET");
            nRet = copy_from_user(&als_lux_ave_type, 
                    (void __user *)ulArg, sizeof(T_ALS_IOCTL_LUX_AVE));
            if (nRet) {
                return -EFAULT;
            }
            gp2ap_get_ioctl_lux_ave(&als_lux_ave, &cdata, &irdata);

            als_lux_ave_type.ulals_lux_ave = als_lux_ave;
            als_lux_ave_type.lcdata = cdata;
            als_lux_ave_type.lirdata = irdata;

            nRet = copy_to_user((void *)(ulArg),
                     &als_lux_ave_type, sizeof(T_ALS_IOCTL_LUX_AVE));
            if (nRet) {
                return -EFAULT;
            }
            break;
        default:
            SENSOR_A_LOG("default");
            break;
    }

    SENSOR_N_LOG("end");
    return nRet;
}

static ssize_t light_flush_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    SENSOR_N_LOG("start");

    mutex_lock(&light_event_mutex);
    input_report_abs(light_input_info.dev, ABS_MISC+3, ~(-1));
    input_report_abs(light_input_info.dev, ABS_MISC+3, (-1));
    input_sync(light_input_info.dev);
    mutex_unlock(&light_event_mutex);

    SENSOR_N_LOG("end - return[%d]",count);
    return count;
}

static ssize_t light_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int enable = 0;
    SENSOR_N_LOG("start");

    if(gp2ap_get_initialize_state() == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }

    enable = sensor_get_status(SENSOR_LIGHT);

    SENSOR_N_LOG("end ->enable[%d]",enable);
    return sprintf(buf, "%d\n", enable);
}

static ssize_t light_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    unsigned long enable = 0;
    int ret = 0;

    SENSOR_N_LOG("start");

    if(gp2ap_get_initialize_state() == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return count;
    }

    ret = strict_strtoul(buf, 10, &enable);
    SENSOR_N_LOG("strict_strtoul() ret[%d]->enable[%d]",ret, (int)enable);

    sensor_enable( SENSOR_LIGHT, NULL, (bool)enable );

    SENSOR_N_LOG("end - return[%d]",count);
    return count;
}

static ssize_t light_delay_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int32_t delay = 0;
    SENSOR_N_LOG("start");
    if(gp2ap_get_initialize_state() == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }

    SENSOR_N_LOG("end ->delay[%d]",(int)delay);
    return sprintf(buf, "%d\n", delay);
}

static ssize_t light_delay_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    unsigned long delay = 0;
    int ret = 0;
    SENSOR_N_LOG("start");

    if(gp2ap_get_initialize_state() == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return count;
    }

    ret = strict_strtoul(buf, 10, &delay);
    SENSOR_N_LOG("strict_strtoul() ret[%d]->delay[%d]",ret, (int)delay);

    gp2ap_set_als_poll_delay(delay);

    SENSOR_N_LOG("end - return[%d]",count);
    return count;
}


void light_input_report( uint32_t data )
{
    SENSOR_N_LOG("start param[%d]",(int)data);

    light_last_read_data = data;
    mutex_lock(&light_event_mutex);
    sensor_report_abs_current_time(light_input_info.dev);
    input_report_abs( light_input_info.dev, ABS_MISC, data);
    input_sync(light_input_info.dev);
    mutex_unlock(&light_event_mutex);

    SENSOR_N_LOG("end");
    return;
}

static void light_set_input_params( struct input_dev *dev )
{
    SENSOR_N_LOG("start");

    if(!dev){
        SENSOR_ERR_LOG("bad parm --> dev[%d]",(int)dev);
        return;
    }

    dev->name = "light";
    dev->id.bustype = BUS_I2C;

    set_bit(EV_ABS, dev->evbit);
    input_set_capability(dev, EV_ABS, ABS_MISC);
    input_set_capability(dev, EV_ABS, ABS_MISC+3);
    input_set_abs_params(dev, ABS_MISC, 0,GP2AP_LUXVALUE_MAX, 0, 0);
    input_set_abs_params(dev, ABS_MISC+3, INT_MIN, INT_MAX, 0, 0);

    SENSOR_N_LOG("end");
    return;
}

#ifdef CONFIG_KC_CORRECT_LUXVALUE
static ssize_t light_mode_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int mode = 0;
    SENSOR_N_LOG("start");
    if(gp2ap_get_initialize_state() == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
    mode = gp2ap_get_als_mode();
    SENSOR_N_LOG("end");
    return sprintf(buf, "%d\n", mode);
}
#endif /* CONFIG_KC_CORRECT_LUXVALUE */

static ssize_t light_val_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    u32 lux = 0;
    s32 cdata = 0;
    s32 irdata = 0;
    SENSOR_N_LOG("start");
    if(gp2ap_get_initialize_state() == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
    gp2ap_get_ioctl_lux_ave(&lux, &cdata, &irdata);

    SENSOR_N_LOG("end");
    return sprintf(buf, "%d\n", lux);
}
static ssize_t light_ratio_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    u32 lux = 0;
    s32 cdata = 0;
    s32 irdata = 0;
    s32 ratio = 0;
    SENSOR_N_LOG("start");
    if(gp2ap_get_initialize_state() == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
    gp2ap_get_ioctl_lux_ave(&lux, &cdata, &irdata);
    ratio = ((cdata == 0)? 100 : (irdata * 100) / cdata);
    SENSOR_N_LOG("end");
    return sprintf(buf, "%d\n", ratio);
}
static ssize_t light_d0d1_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    u32 lux = 0;
    s32 cdata = 0;
    s32 irdata = 0;
    SENSOR_N_LOG("start");
    if(gp2ap_get_initialize_state() == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
    gp2ap_get_ioctl_lux_ave(&lux, &cdata, &irdata);

    SENSOR_N_LOG("end");
    return sprintf(buf, "%d %d\n", cdata, irdata);
}
static ssize_t light_imit_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int imit_flg = 0;
    int imit_d0 = 0;
    int imit_d1 = 0;
    SENSOR_N_LOG("start");
    if(gp2ap_get_initialize_state() == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return 0;
    }
    gp2ap_als_get_imitation_info(&imit_flg, &imit_d0, &imit_d1);
    SENSOR_N_LOG("end ->imit_flg[%d], imit_d0[%d], imit_d1[%d]", imit_flg, imit_d0, imit_d1);
    return sprintf(buf, "%d %d %d\n", imit_flg, imit_d0, imit_d1);
}
static ssize_t light_imit_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    int imit_flg = 0;
    int imit_d0 = 0;
    int imit_d1 = 0;
    SENSOR_N_LOG("start");
    if(gp2ap_get_initialize_state() == 0)
    {
        SENSOR_ERR_LOG("end:Not Initialize Device!!!");
        return count;
    }
    sscanf(buf, "%d %d %d", &imit_flg, &imit_d0, &imit_d1);
    imit_flg = !!imit_flg;
    if( (imit_d0 >= 0) && (imit_d1 >= 0) ){
        gp2ap_als_set_imitation_info(imit_flg, imit_d0, imit_d1);
    } else {
        SENSOR_ERR_LOG("end:input Data is INVALID!!!");
        return count;
    }
    SENSOR_N_LOG("imit_data flg[%d], d0(cdata)[%d], d1(irdata)[%d]", imit_flg, imit_d0, imit_d1);
    SENSOR_N_LOG("end");

    return count;
}


void light_driver_init( void )
{
    int ret = 0;

    SENSOR_N_LOG("start");

    ret = sensor_input_init( &light_input_info );
    SENSOR_N_LOG("sensor_input_init()-->ret[%d] dev[%d]",
                  ret, (int)(light_input_info.dev) );

    if( (0 != ret) || (NULL == (light_input_info.dev))) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        SENSOR_ERR_LOG("end return[%d]",-ENODEV);
        return;
    }

    ret = misc_register(&light_device);
    if (ret) {
        SENSOR_ERR_LOG("fail misc_register()-->ret[%d]",(int)ret);
        return;
    }

    SENSOR_N_LOG("end");
    return;
}

EXPORT_SYMBOL(light_driver_init);

