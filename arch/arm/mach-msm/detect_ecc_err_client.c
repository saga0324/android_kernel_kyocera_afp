/*
 * =========================================================================
 *                                                                          
 * This software is contributed or developed by KYOCERA Corporation.        
 * (C) 2016 KYOCERA Corporation                                             
 *                                                                          
 * =========================================================================
 */

#include<linux/module.h>
#include<linux/fs.h>
#include<linux/cdev.h>
#include<linux/device.h>
#include<linux/wait.h>
#include<linux/sched.h>
#include<linux/miscdevice.h>
#include<linux/workqueue.h>
#include<linux/uaccess.h>
#include<linux/mutex.h>
#include<linux/mmc/host.h>
#include<linux/mmc/mmc.h>

struct ecc_err_save_device_t {
    struct miscdevice misc;
};

/* ioctl structure */
struct ecc_err_ioctl_info {
    unsigned int err_addr;
    unsigned int device_status;
    unsigned int err_log_count;
};

#define IOC_MAGIC 0xF9
#define DETECT_ECC_ERROR_WAIT_FOR_REQ   _IOR(IOC_MAGIC, 1, struct ecc_err_ioctl_info)
#define DETECT_ECC_ERROR_END            _IOR(IOC_MAGIC, 2, struct ecc_err_ioctl_info)

//#define SAVE_SECTOR_UNIT    0x4000  /* 8MB */

#define DETECT_ENABLE       0
#define DETECT_DISABLE      1

#define ERR_SAVE_COUNT 16

wait_queue_head_t event_q;

atomic_t detect_flag;
atomic_t detect_err_log_count;

unsigned char detect_ecc_err_sts = DETECT_ENABLE;

unsigned int ecc_err_addr[ERR_SAVE_COUNT];
unsigned int dev_status[ERR_SAVE_COUNT];
unsigned int ioctl_count = 0;
unsigned int error_count = 0;



static long detect_ecc_err_ioctl( struct file *file,
                                  unsigned int cmd,
                                  unsigned long arg )
{
    struct ecc_err_ioctl_info info;
    unsigned long result = -EFAULT;
    int ret;
    
    switch(cmd)
    {
        case DETECT_ECC_ERROR_WAIT_FOR_REQ:
            ret = wait_event_interruptible(event_q, atomic_read(&detect_flag) != 0);
            if(ret < 0)
            {
                return ret;
            }
            
            info.err_addr      = ecc_err_addr[ioctl_count];
            info.device_status = dev_status[ioctl_count];
            ioctl_count++;
            if(ioctl_count >= ERR_SAVE_COUNT)
            {
                ioctl_count = 0;
            }
            
            info.err_log_count = atomic_read(&detect_err_log_count);
            if(info.err_log_count != 0)
            {
                atomic_set(&detect_err_log_count, 0);
            }
            
            result = copy_to_user((struct ecc_err_ioctl_info __user *)arg, &info, sizeof(info));
            if(atomic_read(&detect_flag) > 0)
            {
                atomic_dec(&detect_flag);
            }
            break;
        
        case DETECT_ECC_ERROR_END:
            detect_ecc_err_sts = DETECT_DISABLE;
            break;
        
        default:
            break;
    }
    return 0;
}

void detect_ecc_error(unsigned int err_addr, unsigned int device_status)
{
    if(detect_ecc_err_sts)
    {
        return;
    }
    
    if((error_count == ioctl_count) && (atomic_read(&detect_flag) != 0))
    {
        atomic_inc(&detect_err_log_count);
        return;
    }
    
    ecc_err_addr[error_count] = err_addr;
    dev_status[error_count]   = device_status;
    error_count++;
    if(error_count >= ERR_SAVE_COUNT)
    {
        error_count = 0;
    }
    
    atomic_inc(&detect_flag);
    
    wake_up_interruptible(&event_q);
    
    return;
}


static int detect_ecc_err_open(struct inode *ip, struct file *fp)
{
    return nonseekable_open(ip, fp);
}

static int detect_ecc_err_release(struct inode *ip, struct file *fp)
{
    return 0;
}

static const struct file_operations detect_ecc_err_fops = {
    .owner = THIS_MODULE,
    .open  = detect_ecc_err_open,
    .unlocked_ioctl = detect_ecc_err_ioctl,
    .release = detect_ecc_err_release,
};

static struct ecc_err_save_device_t detect_ecc_err_device = {
    .misc = {
            .minor = MISC_DYNAMIC_MINOR,
            .name  = "ecc_err_info",
            .fops  = &detect_ecc_err_fops,
    }
};

static void __exit detect_ecc_err_exit(void)
{
    misc_deregister(&detect_ecc_err_device.misc);
}

static int __init detect_ecc_err_init(void)
{
    int ret;
    
    init_waitqueue_head(&event_q);
    
    atomic_set(&detect_flag, 0);
    atomic_set(&detect_err_log_count, 0);
    
    ret = misc_register(&detect_ecc_err_device.misc);
    return ret;
}

module_init(detect_ecc_err_init);
module_exit(detect_ecc_err_exit);

MODULE_DESCRIPTION("detect_ecc_err Driver");
MODULE_LICENSE("GPL v2");
