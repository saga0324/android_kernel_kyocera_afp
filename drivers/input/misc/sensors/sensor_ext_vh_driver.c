/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
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
#include <linux/poll.h>
#include "sensor_driver.h"
#include "sensor_micon_driver.h"
#include "sensor_com.h"

#define EXT_FUNC_VH_KIND 0x01

#define UNDEFINED_VALUE 0x01
#define UPSIDE_UP 0x02
#define LEFT_HORIZONTAL 0x10
#define UPSIDE_DOWN 0x20
#define RIGHT_HORIZONTAL 0x40

static atomic_t sns_ext_vh_interrupt_kind;
static atomic_t sns_ext_vh_interrupt_flag;

static ssize_t sensor_ext_vh_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t sensor_ext_vh_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t sensor_ext_vh_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t sensor_ext_vh_imit_data_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static int32_t sensor_ext_vh_open( struct inode* inode, struct file* filp );
static int32_t sensor_ext_vh_release( struct inode* inode, struct file* filp );
static unsigned int sensor_ext_vh_poll(struct file *fp, poll_table *wait);
static void sensor_ext_vh_set_input_params( struct input_dev *dev );

static DEVICE_ATTR( vhdetect_enable,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    sensor_ext_vh_enable_show,
    sensor_ext_vh_enable_store
);
static DEVICE_ATTR(vhdetect_data,
    S_IRUSR|S_IRGRP,
    sensor_ext_vh_data_show,
    NULL
);

static DEVICE_ATTR( vhdetect_imit_data,
    S_IWUSR|S_IWGRP,
    NULL,
    sensor_ext_vh_imit_data_store
);

static struct attribute *sensor_ext_vh_attributes[] = {
    &dev_attr_vhdetect_enable.attr,
    &dev_attr_vhdetect_data.attr,
    &dev_attr_vhdetect_imit_data.attr,
    NULL
};

static struct attribute_group sensor_ext_vh_attr_grp = {
    .attrs = sensor_ext_vh_attributes
};

struct sensor_input_info_str sensor_ext_vh_input_info =
{
    NULL,
    sensor_ext_vh_set_input_params,
    &sensor_ext_vh_attr_grp,
};

static struct file_operations sensor_ext_vh_fops = {
  .owner   = THIS_MODULE,
  .open    = sensor_ext_vh_open,
  .release = sensor_ext_vh_release,
  .poll    = sensor_ext_vh_poll,
};
static struct miscdevice sensor_ext_vh_device = {
  .minor = MISC_DYNAMIC_MINOR,
  .name  = "vhdetect_io",
  .fops  = &sensor_ext_vh_fops,
};

static wait_queue_head_t sensor_ext_vh_p_queue;

static int32_t gPSet = 0;
static int32_t gVhState = 0;

static ssize_t sensor_ext_vh_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret = 0;
    int enable = 0;

    SENSOR_N_LOG("start");
    enable = sensor_get_status(SENSOR_EXT_VH);
    ret = sprintf(buf, "%x\n",enable);
    SENSOR_N_LOG("end");

    return ret;
}

static ssize_t sensor_ext_vh_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    int32_t enable = 0;
    SENSOR_N_LOG("start");

    sscanf(buf, "%x", &enable);
    sensor_enable( SENSOR_EXT_VH, NULL, (bool)enable );
    if (enable && sns_is_vhdetect_first_report_done()) {
        sensor_ext_vh_interrupt();
    }

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t sensor_ext_vh_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int get_ret = 0;
    ssize_t ret = 0;
    union sensor_ext_read_data_u read_data;

    SENSOR_N_LOG("start");
    memset(&read_data, 0, sizeof(read_data));

    if(gPSet) {
        read_data.vh_data.usVhStatus = gVhState;
    } else {
        get_ret = sensor_ext_get_data( SENSOR_EXT_VH, &read_data );
    }

    if (  0 == get_ret ) {
        ret = sprintf( buf, "%d\n",
                       read_data.vh_data.usVhStatus );
    }

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t sensor_ext_vh_imit_data_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    int32_t pseudo_set = 0;
    int32_t vh_state = 0;
    SENSOR_N_LOG("start");

    sscanf(buf, "%d %x", &pseudo_set, &vh_state);
    gPSet = pseudo_set;
    switch (vh_state){
        case UPSIDE_UP:
            gVhState = 1;
            break;
        case LEFT_HORIZONTAL:
            gVhState = 2;
            break;
        case UPSIDE_DOWN:
            gVhState = 3;
            break;
        case RIGHT_HORIZONTAL:
            gVhState = 4;
            break;
        case UNDEFINED_VALUE:
        default:
            gVhState = 0;
            SENSOR_ERR_LOG("UNDEFINED vh_state:%d",vh_state);
            break;
    }

    sensor_ext_vh_interrupt( );

    SENSOR_N_LOG("end");
    return count;
}

static int32_t sensor_ext_vh_open( struct inode* inode, struct file* filp )
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end");
    return 0;
}

static int32_t sensor_ext_vh_release( struct inode* inode, struct file* filp )
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end");
    return 0;
}

static unsigned int sensor_ext_vh_poll(struct file *fp, poll_table *wait)
{
    unsigned int ret = 0;
    unsigned int int_flag = 0;
    SENSOR_N_LOG("start");

    poll_wait(fp, &sensor_ext_vh_p_queue, wait);
    int_flag = atomic_read(&sns_ext_vh_interrupt_flag);
    if(int_flag)
    {
        ret = POLLIN | POLLPRI;
        atomic_set(&sns_ext_vh_interrupt_kind, 0);
        atomic_set(&sns_ext_vh_interrupt_flag, 0);
    }

    SENSOR_N_LOG("end");
    return ret;
}

static void sensor_ext_vh_set_input_params( struct input_dev *dev )
{
    SENSOR_N_LOG("start");

    if(!dev){
        SENSOR_ERR_LOG("bad parm --> dev is NULL");
        return;
    }

    dev->name = "vhdetect";
    dev->id.bustype = BUS_SPI;

    SENSOR_N_LOG("end");
    return;
}

void sensor_ext_vh_interrupt( void )
{
    uint32_t tmp_kind = 0;

    SENSOR_N_LOG("start");
    if ( SENSOR_ON == sensor_get_status(SENSOR_EXT_VH) ) {
        tmp_kind = atomic_read(&sns_ext_vh_interrupt_kind);
        tmp_kind |= EXT_FUNC_VH_KIND;
        atomic_set(&sns_ext_vh_interrupt_kind,tmp_kind);
        atomic_set(&sns_ext_vh_interrupt_flag,1);
        SENSOR_A_LOG("wake up VH");
        wake_up_interruptible(&sensor_ext_vh_p_queue);
    }
    SENSOR_N_LOG("end");
    return;
}

void sensor_ext_vh_driver_init(void)
{
    int32_t ret;

    SENSOR_N_LOG("start");

    atomic_set(&sns_ext_vh_interrupt_kind,0);
    atomic_set(&sns_ext_vh_interrupt_flag,0);

    ret = sensor_input_init( &sensor_ext_vh_input_info );
    SENSOR_N_LOG("sensor_input_init()-->ret[%d] dev[%p]",
                  ret, sensor_ext_vh_input_info.dev );

    if( (0 != ret) || (NULL == (sensor_ext_vh_input_info.dev))) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        return;
    }

    ret = misc_register(&sensor_ext_vh_device);
    if (ret) {
        SENSOR_ERR_LOG("fail misc_register()-->ret[%d]",(int)ret);
        return;
    }

    init_waitqueue_head(&sensor_ext_vh_p_queue);

    SENSOR_N_LOG("end");
    return;
}

EXPORT_SYMBOL(sensor_ext_vh_driver_init);

