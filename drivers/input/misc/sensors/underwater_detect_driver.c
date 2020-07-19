/*
 * This software is contributed or developed by KYOCERA Corporation.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include "sensor_driver.h"
#include "sensor_com.h"

#define UNDERWATER_DETECT_FUNC_WATER            (0x0002)

static DEFINE_MUTEX(interrupt_kind_mutex);
uint32_t underwater_detect_interrupt_kind;

static ssize_t underwater_detect_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t underwater_detect_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t underwater_detect_interrupt_kind_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t underwater_detect_clear_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );

static int32_t underwater_detect_open( struct inode* inode, struct file* filp );
static int32_t underwater_detect_release( struct inode* inode, struct file* filp );
static unsigned int underwater_detect_poll(struct file *fp, poll_table *wait);
static void underwater_detect_set_input_params( struct input_dev *dev );

static DEVICE_ATTR( enable,
    S_IWUSR|S_IWGRP,
    NULL,
    underwater_detect_enable_store
);
static DEVICE_ATTR(data,
    S_IRUSR|S_IRGRP,
    underwater_detect_data_show,
    NULL
);
static DEVICE_ATTR(interrupt_kind,
    S_IRUSR|S_IRGRP,
    underwater_detect_interrupt_kind_show,
    NULL
);
static DEVICE_ATTR(clear,
    S_IWUSR|S_IWGRP,
    NULL,
    underwater_detect_clear_store
);


static struct attribute *underwater_detect_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_data.attr,
    &dev_attr_interrupt_kind.attr,
    &dev_attr_clear.attr,
    NULL
};

static struct attribute_group underwater_detect_attr_grp = {
    .attrs = underwater_detect_attributes
};

struct sensor_input_info_str underwater_detect_input_info =
{
    NULL,
    underwater_detect_set_input_params,
    &underwater_detect_attr_grp,
};

static struct file_operations underwater_detect_fops = {
  .owner   = THIS_MODULE,
  .open    = underwater_detect_open,
  .release = underwater_detect_release,
  .poll    = underwater_detect_poll,
};
static struct miscdevice underwater_detect_device = {
  .minor = MISC_DYNAMIC_MINOR,
  .name  = "underwater_detect",
  .fops  = &underwater_detect_fops,
};

static wait_queue_head_t underwater_detect_p_queue;

static ssize_t underwater_detect_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    int32_t enable = 0;
    SENSOR_N_LOG("start");

    sscanf(buf, "%x", &enable);
    SENSOR_N_LOG("enable[%d]", (int)enable);

    sensor_enable( SENSOR_UNDERWATER_DETECT, NULL, (bool)enable );

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t underwater_detect_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int get_ret = 0;
    ssize_t ret = 0;
    uwater_detect_info_t read_data;

    SENSOR_N_LOG("start");
    memset(&read_data, 0, sizeof(read_data));

    get_ret = sensor_uwater_get_data(&read_data);
    if ( 0 == get_ret ) {
        ret = sprintf( buf, "%d\n", read_data.uwater_detected );
    }

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t underwater_detect_interrupt_kind_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret = 0;
    uint32_t tmp_kind;

    SENSOR_N_LOG("start");

    mutex_lock(&interrupt_kind_mutex);
    tmp_kind = underwater_detect_interrupt_kind;
    underwater_detect_interrupt_kind = 0;
    mutex_unlock(&interrupt_kind_mutex);

    ret = sprintf( buf, "%x\n", tmp_kind);

    SENSOR_N_LOG("end. interrupt kind = %x", tmp_kind);
    return ret;
}

static ssize_t underwater_detect_clear_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    SENSOR_N_LOG("start");

    sensor_uwater_clear();

    SENSOR_N_LOG("end");
    return count;
}

static int32_t underwater_detect_open( struct inode* inode, struct file* filp )
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end");
    return 0;
}

static int32_t underwater_detect_release( struct inode* inode, struct file* filp )
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end");
    return 0;
}

static unsigned int underwater_detect_poll(struct file *fp, poll_table *wait)
{
    unsigned int ret = 0;
    unsigned int int_flag = 0;
    SENSOR_N_LOG("start");

    poll_wait(fp, &underwater_detect_p_queue, wait);
    mutex_lock(&interrupt_kind_mutex);
    int_flag = underwater_detect_interrupt_kind;
    if( int_flag ) {
        ret = POLLIN | POLLPRI;
    }
    mutex_unlock(&interrupt_kind_mutex);

    SENSOR_N_LOG("end int_flag = %d", int_flag);
    return ret;
}

static void underwater_detect_set_input_params( struct input_dev *dev )
{
    SENSOR_N_LOG("start");

    if(!dev){
        SENSOR_ERR_LOG("bad param --> dev is NULL");
        return;
    }

    dev->name = "underwater_detect";
    dev->id.bustype = BUS_SPI;

    SENSOR_N_LOG("end");
    return;
}

void underwater_detect_water_interrupt( void )
{
    SENSOR_N_LOG("start");

    if ( SENSOR_ON == sensor_get_status(SENSOR_UNDERWATER_DETECT) ) {
        mutex_lock(&interrupt_kind_mutex);
        underwater_detect_interrupt_kind |= UNDERWATER_DETECT_FUNC_WATER;
        mutex_unlock(&interrupt_kind_mutex);
        wake_up_interruptible(&underwater_detect_p_queue);
    }

    SENSOR_N_LOG("end");
    return;
}

void underwater_detect_driver_init(void)
{
    int32_t ret;

    SENSOR_N_LOG("start");

    mutex_lock(&interrupt_kind_mutex);
    underwater_detect_interrupt_kind = 0;
    mutex_unlock(&interrupt_kind_mutex);

    ret = sensor_input_init( &underwater_detect_input_info );
    SENSOR_N_LOG("sensor_input_init()-->ret[%d] dev[%p]",
                  ret, underwater_detect_input_info.dev );

    if( (0 != ret) || (NULL == (underwater_detect_input_info.dev))) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        return;
    }

    ret = misc_register(&underwater_detect_device);
    if (ret) {
        SENSOR_ERR_LOG("fail misc_register()-->ret[%d]",(int)ret);
        return;
    }

    init_waitqueue_head(&underwater_detect_p_queue);

    SENSOR_N_LOG("end");
    return;
}

EXPORT_SYMBOL(underwater_detect_driver_init);

