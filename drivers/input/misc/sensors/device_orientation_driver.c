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

#include "sensor_driver.h"
#include "sensor_micon_driver.h"
#include "sensor_com.h"

#define DEVICE_ORIENTATION_ROTATE_UP        (0)
#define DEVICE_ORIENTATION_ROTATE_LEFT      (1)
#define DEVICE_ORIENTATION_ROTATE_DOWN      (2)
#define DEVICE_ORIENTATION_ROTATE_RIGHT     (3)
#define DEVICE_ORIENTATION_ROTATE_VH_UP     (1)
#define DEVICE_ORIENTATION_ROTATE_VH_LEFT   (2)
#define DEVICE_ORIENTATION_ROTATE_VH_DOWN   (3)
#define DEVICE_ORIENTATION_ROTATE_VH_RIGHT  (4)

#define DEVICE_ORIENTATION_DUMMY_DATA      (-1)
#ifdef CONFIG_USE_DEVICE_ORIENTATION_1ST_REPORT_DELAY
#define DEVICE_ORIENTATION_1ST_REPORT_TIME_MS (350)
#endif

#define UNDEFINED_VALUE 0x01
#define UPSIDE_UP 0x02
#define LEFT_HORIZONTAL 0x10
#define UPSIDE_DOWN 0x20
#define RIGHT_HORIZONTAL 0x40

static int32_t gPSet = 0;
static int32_t gState = 0;
#ifdef CONFIG_USE_DEVICE_ORIENTATION_1ST_REPORT_DELAY
static struct workqueue_struct *device_orientation_wq;
struct delayed_work	device_orientation_dwork;
#endif


static ssize_t device_orientation_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t device_orientation_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t device_orientation_delay_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t device_orientation_delay_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t device_orientation_flush_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t device_orientation_imit_data_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static void device_orientation_set_input_params( struct input_dev *dev );
static void device_orientation_input_report( void );
static int32_t device_orientation_convert_orientation_via( int32_t orientation );
#ifdef CONFIG_USE_DEVICE_ORIENTATION_1ST_REPORT_DELAY
static void device_orientation_1st_report_func(struct work_struct *work);
#endif


static DEVICE_ATTR(enable,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    device_orientation_enable_show,
    device_orientation_enable_store
);
static DEVICE_ATTR(delay,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    device_orientation_delay_show,
    device_orientation_delay_store
);
static DEVICE_ATTR(flush,
    S_IWUSR|S_IWGRP,
    NULL,
    device_orientation_flush_store
);
static DEVICE_ATTR(imit_data,
    S_IWUSR|S_IWGRP,
    NULL,
    device_orientation_imit_data_store
);

static struct attribute *device_orientation_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_delay.attr,
    &dev_attr_flush.attr,
    &dev_attr_imit_data.attr,
    NULL
};

static struct attribute_group device_orientation_attr_grp = {
    .attrs = device_orientation_attributes
};

struct sensor_input_info_str device_orientation_input_info =
{
    NULL,
    device_orientation_set_input_params,
    &device_orientation_attr_grp,
};

static uint32_t device_orientation_delay_time = 8;
static DEFINE_MUTEX(device_orientation_event_mutex);

static ssize_t device_orientation_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int enable = 0;
    SENSOR_N_LOG("start");

    enable = sensor_get_status(SENSOR_DEVICE_ORIENTATION);

    SENSOR_N_LOG("end ->enable[%d]", enable);
    return sprintf(buf, "%d\n", enable);
}

static ssize_t device_orientation_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    unsigned long enable = 0;
    int ret = 0;

    SENSOR_N_LOG("start");

    ret = kstrtoul(buf, 10, &enable);
    SENSOR_N_LOG("kstrtoul() ret[%d]->enable[%d]",ret, (int)enable);

    sensor_enable( SENSOR_DEVICE_ORIENTATION, NULL, (bool)enable );
#ifdef CONFIG_USE_DEVICE_ORIENTATION_1ST_REPORT_DELAY
    if (enable) {
        SENSOR_D_LOG("Scheduling 1st report");
        queue_delayed_work(device_orientation_wq, &device_orientation_dwork,
                            msecs_to_jiffies(DEVICE_ORIENTATION_1ST_REPORT_TIME_MS));
    } else {
        cancel_delayed_work(&device_orientation_dwork);
    }
#else
    if (enable && sns_is_vhdetect_first_report_done()) {
        device_orientation_input_report();
    }
#endif

    SENSOR_N_LOG("end - return[%ld]", count);
    return count;
}

static ssize_t device_orientation_delay_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end");
    return sprintf(buf, "%d\n", device_orientation_delay_time);
}

static ssize_t device_orientation_delay_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    uint32_t delay_time = simple_strtoul(buf, NULL, 10);
    SENSOR_N_LOG("start delay_time[%u]", delay_time);

    device_orientation_delay_time = delay_time;

    SENSOR_N_LOG("end - return[%ld]", count);
    return count;
}

static ssize_t device_orientation_flush_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    SENSOR_D_LOG("start");

    mutex_lock(&device_orientation_event_mutex);
    input_report_abs(device_orientation_input_info.dev, ABS_MISC+3, ~(-1));
    input_report_abs(device_orientation_input_info.dev, ABS_MISC+3, (-1));
    input_sync(device_orientation_input_info.dev);
    mutex_unlock(&device_orientation_event_mutex);

    SENSOR_D_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t device_orientation_imit_data_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    int32_t pseudo_set = 0;
    int32_t state = 0;
    SENSOR_D_LOG("start");

    sscanf(buf, "%d %x", &pseudo_set, &state);
    gPSet = pseudo_set;
    switch (state){
        case UPSIDE_UP:
            gState = 1;
            break;
        case LEFT_HORIZONTAL:
            gState = 2;
            break;
        case UPSIDE_DOWN:
            gState = 3;
            break;
        case RIGHT_HORIZONTAL:
            gState = 4;
            break;
        case UNDEFINED_VALUE:
        default:
            gState = 0;
            SENSOR_ERR_LOG("UNDEFINED state:%d:",state);
            break;
    }

    device_orientation_interrupt( );

    SENSOR_D_LOG("end - return[%d]",(int)count);
    return count;
}

static void device_orientation_set_input_params( struct input_dev *dev )
{
    SENSOR_N_LOG("start");

    if(!dev){
        SENSOR_ERR_LOG("bad parm --> dev is NULL");
        return;
    }

    dev->name = "device_orientation";
    dev->id.bustype = BUS_SPI;

    set_bit(EV_ABS, dev->evbit);
    input_set_capability(dev, EV_ABS, ABS_MISC+3);
    input_set_abs_params(dev, ABS_X, INT_MIN, INT_MAX, 0, 0);
    input_set_abs_params(dev, ABS_MISC+3, INT_MIN, INT_MAX, 0, 0);

    SENSOR_N_LOG("end");
    return;
}

int32_t device_orientation_convert_orientation( int32_t orientation )
{
    int32_t ret = 0;

    SENSOR_N_LOG("start [%d]", orientation);

    switch (orientation) {
    case DEVICE_ORIENTATION_ROTATE_VH_UP:
        ret = DEVICE_ORIENTATION_ROTATE_UP;
        break;
    case DEVICE_ORIENTATION_ROTATE_VH_LEFT:
        ret = DEVICE_ORIENTATION_ROTATE_LEFT;
        break;
    case DEVICE_ORIENTATION_ROTATE_VH_DOWN:
        ret = DEVICE_ORIENTATION_ROTATE_DOWN;
        break;
    case DEVICE_ORIENTATION_ROTATE_VH_RIGHT:
        ret = DEVICE_ORIENTATION_ROTATE_RIGHT;
        break;
    default:
        SENSOR_ERR_LOG("invalid orientation [%d]", orientation);
        break;
    }

    SENSOR_N_LOG("end [%d]", ret);

    return ret;
}

static int32_t device_orientation_convert_orientation_via( int32_t orientation )
{
    int32_t orien_local = orientation;
    int32_t ret = 0;

    SENSOR_D_LOG("start");
    if(gPSet){
        orien_local = gState;
        SENSOR_D_LOG("Because imitation function ran, device orientation value is chenged to [%d]", orien_local);
    }
    ret = device_orientation_convert_orientation(orien_local);

    SENSOR_D_LOG("end ret[%d]", ret);

    return ret;
}

#ifdef CONFIG_USE_DEVICE_ORIENTATION_1ST_REPORT_DELAY
static void device_orientation_1st_report_func(struct work_struct *work)
{
    SENSOR_D_LOG("start");
    if(!sns_is_vhdetect_first_report_done()){
        device_orientation_input_report();
    } else {
        SENSOR_ERR_LOG("Already reported via irq_handler");
    }
    SENSOR_D_LOG("end");
}
#endif

static void device_orientation_input_report( void )
{
    int get_ret = 0;
    union sensor_ext_read_data_u read_data;
    int32_t report_data;

    SENSOR_N_LOG("start");

    get_ret = sensor_ext_get_data(SENSOR_DEVICE_ORIENTATION, &read_data);
    if (  0 == get_ret ) {
        report_data = device_orientation_convert_orientation_via(read_data.vh_data.usVhStatus);
        mutex_lock(&device_orientation_event_mutex);
        sensor_report_abs_current_time(device_orientation_input_info.dev);
        input_report_abs(device_orientation_input_info.dev, ABS_X, DEVICE_ORIENTATION_DUMMY_DATA);
        input_report_abs(device_orientation_input_info.dev, ABS_X, report_data);
        input_sync(device_orientation_input_info.dev);
        mutex_unlock(&device_orientation_event_mutex);
    }

    SENSOR_N_LOG("end");

    return;
}

void device_orientation_interrupt( void )
{
    SENSOR_N_LOG("start");
    if ( SENSOR_ON == sensor_get_status(SENSOR_DEVICE_ORIENTATION) ) {
        device_orientation_input_report();
    }
    SENSOR_N_LOG("end");
    return;
}

void device_orientation_driver_init( void )
{
    int ret = 0;

    SENSOR_N_LOG("start");

#ifdef CONFIG_USE_DEVICE_ORIENTATION_1ST_REPORT_DELAY
    device_orientation_wq = create_singlethread_workqueue("device_orientation_workqueue");
    INIT_DELAYED_WORK(&device_orientation_dwork, device_orientation_1st_report_func);
#endif
    ret = sensor_input_init( &device_orientation_input_info );
    SENSOR_N_LOG("sensor_input_init()-->ret[%d] dev[%p]",
                  ret, device_orientation_input_info.dev );

    if( (0 != ret) || (NULL == (device_orientation_input_info.dev))) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        SENSOR_ERR_LOG("end return[%d]",-ENODEV);
        return;
    }

    SENSOR_N_LOG("end");
    return;
}

EXPORT_SYMBOL(device_orientation_driver_init);

