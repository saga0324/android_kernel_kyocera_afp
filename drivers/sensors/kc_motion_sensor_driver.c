/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
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

#define KCMOTION_FUNC_WALK_START       (0x0001)
#define KCMOTION_FUNC_WALK_STOP        (0x0002)
#define KCMOTION_FUNC_TRAIN            (0x0004)
#define KCMOTION_FUNC_VEHICLE          (0x0008)
#define KCMOTION_FUNC_BRINGUP          (0x0010)

static atomic_t kc_motion_interrupt_kind;
static atomic_t kc_motion_interrupt_flag;

static ssize_t kc_motion_sensor_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t kc_motion_sensor_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t kc_motion_sensor_walk_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t kc_motion_sensor_vehi_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t kc_motion_sensor_bringup_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t kc_motion_sensor_interrupt_kind_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);

static int32_t kc_motion_sensor_get_data( enum sensor_e_type type, union sensor_motion_read_data_u *read_data );
static int32_t kc_motion_sensor_open( struct inode* inode, struct file* filp );
static int32_t kc_motion_sensor_release( struct inode* inode, struct file* filp );
static unsigned int kc_motion_sensor_poll(struct file *fp, poll_table *wait);
static void kc_motion_sensor_set_input_params( struct input_dev *dev );

static DEVICE_ATTR( kc_motion_enable,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    kc_motion_sensor_enable_show,
    kc_motion_sensor_enable_store
);
static DEVICE_ATTR(kc_motion_walk_data,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    kc_motion_sensor_walk_data_show,
    NULL
);
static DEVICE_ATTR(kc_motion_vehi_data,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    kc_motion_sensor_vehi_data_show,
    NULL
);
static DEVICE_ATTR(kc_motion_bringup_data,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    kc_motion_sensor_bringup_data_show,
    NULL
);
static DEVICE_ATTR(kc_motion_interrupt_kind,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    kc_motion_sensor_interrupt_kind_show,
    NULL
);

static struct attribute *kc_motion_sensor_attributes[] = {
    &dev_attr_kc_motion_enable.attr,
    &dev_attr_kc_motion_walk_data.attr,
    &dev_attr_kc_motion_vehi_data.attr,
    &dev_attr_kc_motion_bringup_data.attr,
    &dev_attr_kc_motion_interrupt_kind.attr,
    NULL
};

static struct attribute_group kc_motion_sensor_attr_grp = {
    .attrs = kc_motion_sensor_attributes
};

struct sensor_input_info_str kc_motion_sensor_input_info =
{
    NULL,
    kc_motion_sensor_set_input_params,
    &kc_motion_sensor_attr_grp,
};

static struct file_operations kc_motion_sensor_fops = {
  .owner   = THIS_MODULE,
  .open    = kc_motion_sensor_open,
  .release = kc_motion_sensor_release,
  .poll    = kc_motion_sensor_poll,
};
static struct miscdevice kc_motion_sensor_device = {
  .minor = MISC_DYNAMIC_MINOR,
  .name  = "kc_motion_sensor_io",
  .fops  = &kc_motion_sensor_fops,
};

static wait_queue_head_t kc_motion_sensor_p_queue;
static struct kc_motion_train stTrain_old = {0};

static ssize_t kc_motion_sensor_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret = 0;
    int enable = 0;

    SENSOR_N_LOG("start");
    if( SENSOR_ON == sensor_get_status(SENSOR_KC_MOTION_WALK_START)) {
        enable |= KCMOTION_FUNC_WALK_START;
    }
    if( SENSOR_ON == sensor_get_status(SENSOR_KC_MOTION_WALK_STOP)) {
        enable |= KCMOTION_FUNC_WALK_STOP;
    }
    if( SENSOR_ON == sensor_get_status(SENSOR_KC_MOTION_TRAIN)) {
        enable |= KCMOTION_FUNC_TRAIN;
    }
    if( SENSOR_ON == sensor_get_status(SENSOR_KC_MOTION_VEHICLE)) {
        enable |= KCMOTION_FUNC_VEHICLE;
    }
    if( SENSOR_ON == sensor_get_status(SENSOR_KC_MOTION_BRINGUP)) {
        enable |= KCMOTION_FUNC_BRINGUP;
    }
    ret = sprintf(buf, "%x\n",enable);
    SENSOR_N_LOG("enable = %x",enable);
    SENSOR_N_LOG("end");

    return ret;
}

static ssize_t kc_motion_sensor_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    int32_t enable = 0;
    int32_t type = 0;
    SENSOR_N_LOG("start");

    sscanf(buf, "%x %x", &type, &enable);
    SENSOR_N_LOG("type[%d] enable[%d]",(int)type, (int)enable);

    if( (type & KCMOTION_FUNC_WALK_START) == KCMOTION_FUNC_WALK_START ){
        sensor_enable( SENSOR_KC_MOTION_WALK_START, NULL, (bool)enable );
    }

    if ((type & KCMOTION_FUNC_WALK_STOP) == KCMOTION_FUNC_WALK_STOP ){
        sensor_enable( SENSOR_KC_MOTION_WALK_STOP, NULL, (bool)enable );
    }

    if ((type & KCMOTION_FUNC_TRAIN) == KCMOTION_FUNC_TRAIN ){
        sensor_enable( SENSOR_KC_MOTION_TRAIN, NULL, (bool)enable );
    }

    if ((type & KCMOTION_FUNC_VEHICLE) == KCMOTION_FUNC_VEHICLE ){
        sensor_enable( SENSOR_KC_MOTION_VEHICLE, NULL, (bool)enable );
    }

    if ((type & KCMOTION_FUNC_BRINGUP) == KCMOTION_FUNC_BRINGUP ){
        sensor_enable( SENSOR_KC_MOTION_BRINGUP, NULL, (bool)enable );
    }

    SENSOR_N_LOG("end");
    return count;
}

static ssize_t kc_motion_sensor_walk_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int get_ret = 0;
    ssize_t ret = 0;
    union sensor_motion_read_data_u read_data;

    SENSOR_N_LOG("start");
    memset(&read_data, 0, sizeof(read_data));

    get_ret = kc_motion_sensor_get_data( SENSOR_KC_MOTION_WALK_START, &read_data );
    if (  0 == get_ret ) {
        ret = sprintf( buf, "%d\n",
                       read_data.walk_data.status );
        SENSOR_N_LOG("walk status = %d",read_data.walk_data.status);
    }

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t kc_motion_sensor_vehi_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int get_ret = 0;
    ssize_t ret = 0;
    union sensor_motion_read_data_u read_data;

    SENSOR_N_LOG("start");
    memset(&read_data, 0, sizeof(read_data));

    get_ret = kc_motion_sensor_get_data( SENSOR_KC_MOTION_VEHICLE, &read_data );
    if (  0 == get_ret ) {
        ret = sprintf( buf, "%d %d\n",
                       read_data.vehicle_data.status, read_data.vehicle_data.kind );
        SENSOR_N_LOG("vehicle status = %d,kind = %d",read_data.vehicle_data.status, read_data.vehicle_data.kind);
    }

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t kc_motion_sensor_bringup_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int get_ret = 0;
    ssize_t ret = 0;
    union sensor_motion_read_data_u read_data;

    SENSOR_N_LOG("start");
    memset(&read_data, 0, sizeof(read_data));

    get_ret = kc_motion_sensor_get_data( SENSOR_KC_MOTION_BRINGUP, &read_data );
    if (  0 == get_ret ) {
        ret = sprintf( buf, "%d\n",
                       read_data.bringup_data.status );
        SENSOR_N_LOG("bringup status = %d",read_data.bringup_data.status);
    }

    SENSOR_N_LOG("end");
    return ret;
}

static ssize_t kc_motion_sensor_interrupt_kind_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret = 0;

    SENSOR_N_LOG("start");

    ret = sprintf( buf, "%x\n",
                   atomic_read(&kc_motion_interrupt_kind) );
    SENSOR_N_LOG("interrupt kind = %x",atomic_read(&kc_motion_interrupt_kind));
    atomic_set(&kc_motion_interrupt_kind, 0);

    SENSOR_N_LOG("end");
    return ret;
}

static int32_t kc_motion_sensor_get_data( enum sensor_e_type type, union sensor_motion_read_data_u *read_data )
{
    int32_t ret = 0;
    struct kc_motion_walk_data stWalk;
    struct kc_motion_train stTrain;
    struct kc_motion_bringup_data stBringup;

    SENSOR_N_LOG("start type = %d",type);

    switch(type) {
        case SENSOR_KC_MOTION_WALK_START:
        case SENSOR_KC_MOTION_WALK_STOP:
            sensor_com_mutex_lock();
            sns_get_kc_motion_walk_data(&stWalk);
            sensor_com_mutex_unlock();
            read_data->walk_data.status = stWalk.status;
            break;
        case SENSOR_KC_MOTION_TRAIN:
        case SENSOR_KC_MOTION_VEHICLE:
            sensor_com_mutex_lock();
            sns_get_kc_motion_train_data(&stTrain);
            sensor_com_mutex_unlock();
            read_data->vehicle_data.status = (stTrain.usTrFstDetect | stTrain.usTrOtherFstDetect);
            if( stTrain_old.usTrFstDetect != stTrain.usTrFstDetect ) {
                read_data->vehicle_data.kind = 1;
            }
            else if( stTrain_old.usTrOtherFstDetect != stTrain.usTrOtherFstDetect ) {
                read_data->vehicle_data.kind = 0;
            }
            else {
                read_data->vehicle_data.kind = 0;
            }
            stTrain_old.usTrFstDetect = stTrain.usTrFstDetect;
            stTrain_old.usTrOtherFstDetect = stTrain.usTrOtherFstDetect;
            break;
        case SENSOR_KC_MOTION_BRINGUP:
            sensor_com_mutex_lock();
            sns_get_kc_motion_bringup_data(&stBringup);
            sensor_com_mutex_unlock();
            read_data->bringup_data.status = stBringup.status;
            break;
        default:
            break;
    }

    SENSOR_N_LOG("end");

    return ret;
}


static int32_t kc_motion_sensor_open( struct inode* inode, struct file* filp )
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end");
    return 0;
}

static int32_t kc_motion_sensor_release( struct inode* inode, struct file* filp )
{
    SENSOR_N_LOG("start");
    SENSOR_N_LOG("end");
    return 0;
}

static unsigned int kc_motion_sensor_poll(struct file *fp, poll_table *wait)
{
    unsigned int ret = 0;
    unsigned int int_flag = 0;
    SENSOR_N_LOG("start");

    poll_wait(fp, &kc_motion_sensor_p_queue, wait);
    int_flag = atomic_read(&kc_motion_interrupt_flag);
    if( int_flag ) {
        ret = POLLIN | POLLPRI;
        atomic_set(&kc_motion_interrupt_flag,0);
    }

    SENSOR_N_LOG("end ret = %d", ret);
    return ret;
}

static void kc_motion_sensor_set_input_params( struct input_dev *dev )
{
    SENSOR_N_LOG("start");

    if(!dev){
        SENSOR_ERR_LOG("bad parm --> dev[%d]",(int)dev);
        return;
    }

    dev->name = "kc_motion_sensor";
    dev->id.bustype = BUS_SPI;

    SENSOR_N_LOG("end");
    return;
}

void kc_motion_sensor_walk_start_interrupt( void )
{
    uint32_t tmp_kind = 0;

    SENSOR_N_LOG("start");
    if ( SENSOR_ON == sensor_get_status(SENSOR_KC_MOTION_WALK_START) ) {
        tmp_kind = atomic_read(&kc_motion_interrupt_kind);
        tmp_kind |= KCMOTION_FUNC_WALK_START;
        atomic_set(&kc_motion_interrupt_kind,tmp_kind);
        atomic_set(&kc_motion_interrupt_flag,1);
        wake_up_interruptible(&kc_motion_sensor_p_queue);
    }
    SENSOR_N_LOG("end");
    return;
}

void kc_motion_sensor_walk_stop_interrupt( void )
{
    uint32_t tmp_kind = 0;

    SENSOR_N_LOG("start");
    if ( SENSOR_ON == sensor_get_status(SENSOR_KC_MOTION_WALK_STOP) ) {
        tmp_kind = atomic_read(&kc_motion_interrupt_kind);
        tmp_kind |= KCMOTION_FUNC_WALK_STOP;
        atomic_set(&kc_motion_interrupt_kind,tmp_kind);
        atomic_set(&kc_motion_interrupt_flag,1);
        wake_up_interruptible(&kc_motion_sensor_p_queue);
    }
    SENSOR_N_LOG("end");
    return;
}

void kc_motion_sensor_train_interrupt( void )
{
    uint32_t tmp_kind = 0;

    SENSOR_N_LOG("start");
    if ( SENSOR_ON == sensor_get_status(SENSOR_KC_MOTION_TRAIN) ) {
        tmp_kind = atomic_read(&kc_motion_interrupt_kind);
        tmp_kind |= KCMOTION_FUNC_TRAIN;
        atomic_set(&kc_motion_interrupt_kind,tmp_kind);
        atomic_set(&kc_motion_interrupt_flag,1);
        wake_up_interruptible(&kc_motion_sensor_p_queue);
    }
    SENSOR_N_LOG("end");
    return;
}

void kc_motion_sensor_vehicle_interrupt( void )
{
    uint32_t tmp_kind = 0;

    SENSOR_N_LOG("start");
    if ( SENSOR_ON == sensor_get_status(SENSOR_KC_MOTION_VEHICLE) ) {
        tmp_kind = atomic_read(&kc_motion_interrupt_kind);
        tmp_kind |= KCMOTION_FUNC_VEHICLE;
        atomic_set(&kc_motion_interrupt_kind,tmp_kind);
        atomic_set(&kc_motion_interrupt_flag,1);
        wake_up_interruptible(&kc_motion_sensor_p_queue);
    }
    SENSOR_N_LOG("end");
    return;
}

void kc_motion_sensor_bringup_interrupt( void )
{
    uint32_t tmp_kind = 0;

    SENSOR_N_LOG("start");
    if ( SENSOR_ON == sensor_get_status(SENSOR_KC_MOTION_BRINGUP) ) {
        tmp_kind = atomic_read(&kc_motion_interrupt_kind);
        tmp_kind |= KCMOTION_FUNC_BRINGUP;
        atomic_set(&kc_motion_interrupt_kind,tmp_kind);
        atomic_set(&kc_motion_interrupt_flag,1);
        wake_up_interruptible(&kc_motion_sensor_p_queue);
    }
    SENSOR_N_LOG("end");
    return;
}

void kc_motion_sensor_driver_init(void)
{
    int32_t ret;

    SENSOR_N_LOG("start");

    atomic_set(&kc_motion_interrupt_kind,0);
    atomic_set(&kc_motion_interrupt_flag,0);

    ret = sensor_input_init( &kc_motion_sensor_input_info );
    SENSOR_N_LOG("sensor_input_init()-->ret[%d] dev[%d]",
                  ret, (int)(kc_motion_sensor_input_info.dev) );

    if( (0 != ret) || (NULL == (kc_motion_sensor_input_info.dev))) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        return;
    }

    ret = misc_register(&kc_motion_sensor_device);
    if (ret) {
        SENSOR_ERR_LOG("fail misc_register()-->ret[%d]",(int)ret);
        return;
    }

    init_waitqueue_head(&kc_motion_sensor_p_queue);

    SENSOR_N_LOG("end");
    return;
}

EXPORT_SYMBOL(kc_motion_sensor_driver_init);

