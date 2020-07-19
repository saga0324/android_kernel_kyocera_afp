/* This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifdef CONFIG_ARCH_GOLDFISH
#define KERR_SMEM_DEBUG
#endif /* CONFIG_ARCH_GOLDFISH */

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <soc/qcom/smem.h>

#ifndef KERR_SMEM_DEBUG
#include "smd_private.h"
#endif

MODULE_DESCRIPTION("KC Error SMEM Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.1");

#define KERR_SMEM_POLLING_TIME    500 /* ms */
#define KERR_SMEM_LOG_SIZE        KCC_SMEM_KERR_LOG_SIZE

/* SMEM Log type */
enum _kerrsmem_log_t {
    KSMEM_LOG_TYPE_NONE                      = 0x00,
    KSMEM_LOG_TYPE_AMSS_CRASH                = 0x01,
    KSMEM_LOG_TYPE_ANDROID_CRASH             = 0x02, /* Not used */
    KSMEM_LOG_TYPE_QA                        = 0x03,
};

typedef struct {
    /* SMEM Status - for synchronizing between apps & modem.
     *  0: Idle
     *  1: MODEM is now writing
     *  2: APPS is now reading */
    char   state;

    /* Data */
    char   log[KERR_SMEM_LOG_SIZE - 1];
} __attribute__((packed)) kerrsmem_log_t;

struct kerrsmem_dev {
    /* State for the char kerr_devp */
    unsigned int             major;
    unsigned int             minor_start;
    int                      num;
    struct cdev*             cdev;
    char*                    name;
    struct class*            kerr_class;
    int                      ref_count;
    struct mutex             kerrsmem_mutex;
    kerrsmem_log_t* __iomem  smem;

    #ifdef KERR_BLOCK_READ
    wait_queue_head_t        wait_q;
    struct workqueue_struct* smem_wq;
    struct work_struct       smem_read_work;
    #endif /* KERR_BLOCK_READ */
};

static struct kerrsmem_dev *kerr_devp = NULL ;

#ifdef KERR_BLOCK_READ
void kerrsmem_read_work_fn(struct work_struct *work)
{
    printk(KERN_INFO "kerrsmem_read_work_fn\n");

    for( ;; ) {
        printk(KERN_DEBUG "Check SMEM for log!\n");

        if (kerr_devp->smem->log_type != 0x00) {
            printk(KERN_DEBUG "Got log - type: %d\n", kerr_devp->smem->log_type);
            printk(KERN_DEBUG "Wake up reading process...\n");
            wake_up_interruptible(&kerr_devp->wait_q);
            break;
        }

        msleep(KERR_SMEM_POLLING_TIME);
    }
}
#endif /* KERR_BLOCK_READ */

static int kerrsmem_open(struct inode *inode, struct file *file)
{
    printk(KERN_INFO "kerrsmem_open\n");

    if (kerr_devp) {
        mutex_lock(&kerr_devp->kerrsmem_mutex);
        kerr_devp->ref_count++;
        mutex_unlock(&kerr_devp->kerrsmem_mutex);

        return 0;
    }

    return -ENOMEM;
}

static int kerrsmem_close(struct inode *inode, struct file *file)
{
    printk(KERN_INFO "kerrsmem_close\n");

    if (kerr_devp) {
        mutex_lock(&kerr_devp->kerrsmem_mutex);
        kerr_devp->ref_count--;
        mutex_unlock(&kerr_devp->kerrsmem_mutex);

        return 0;
    }

    return -ENOMEM;
}

static ssize_t kerrsmem_read(struct file *file, char __user *buf, size_t count,
              loff_t *ppos)
{
    int ret = 0;
    int retry_cnt = 0;

    printk(KERN_INFO "kerrsmem_read\n");

    #ifdef KERR_BLOCK_READ
    wait_event_interruptible(kerr_devp->wait_q, kerr_devp->smem->log_type);
    printk(KERN_DEBUG "Reading process woke up!\n");
    #endif /* KERR_BLOCK_READ */

    mutex_lock(&kerr_devp->kerrsmem_mutex);

    /* Check if SMEM is being written by MODEM side.
     * If it is the case, sleep 1 ms and then check again.
     * Retry for 5 times. */
    while (kerr_devp->smem->state == 1) {
        if( retry_cnt++ >= 5 ) {
            printk(KERN_DEBUG "SMEM is busy writing --> leave for next time!\n");
            mutex_unlock(&kerr_devp->kerrsmem_mutex);
            return -1;
        }

        msleep(1);
    }

    /* Set state flag to indicate that APPS is in reading */
    kerr_devp->smem->state = 2;

    printk(KERN_DEBUG "Log num: %03d %03d %03d %03d %03d %03d %03d %03d %03d\n", 
                kerr_devp->smem->log[0], kerr_devp->smem->log[1],
                kerr_devp->smem->log[2], kerr_devp->smem->log[3],
                kerr_devp->smem->log[4], kerr_devp->smem->log[5],
                kerr_devp->smem->log[6], kerr_devp->smem->log[7],
                kerr_devp->smem->log[8]);

    /* Copy log data from SMEM to user space */
    if (count > sizeof(kerrsmem_log_t)) {
        count = sizeof(kerrsmem_log_t);
    }
    ret = count;
    if (copy_to_user(buf, kerr_devp->smem->log, ret)) {
        printk(KERN_ERR "><Copy log to user buffer failed!\n");
        ret = -1;
    }
    /* Clear log buffer and state flag */
    memset(kerr_devp->smem->log, 0x00, count);
    kerr_devp->smem->state = 0;

    mutex_unlock(&kerr_devp->kerrsmem_mutex);

    #ifdef KERR_BLOCK_READ
    /* Queue another work */
    queue_work(kerr_devp->smem_wq, &kerr_devp->smem_read_work);
    #endif /* KERR_BLOCK_READ */

    return ret;
}

static ssize_t kerrsmem_write(struct file *file, const char __user *buf,
                  size_t count, loff_t *ppos)
{
    printk(KERN_INFO "kerrsmem_write\n");
    printk(KERN_INFO "Unsupported operation!\n");

    return 0;
}

static long kerrsmem_ioctl(struct file *filp,
               unsigned int iocmd, unsigned long ioarg)
{
    printk(KERN_INFO "kerrsmem_ioctl\n");
    printk(KERN_INFO "Unsupported operation!\n");

    return 0;
}

static const struct file_operations kerrsmem_fops = {
    .owner          = THIS_MODULE,
    .read           = kerrsmem_read,
    .write          = kerrsmem_write,
    .unlocked_ioctl = kerrsmem_ioctl,
    .open           = kerrsmem_open,
    .release        = kerrsmem_close
};

static int kerrsmem_setup_cdev(dev_t devno)
{
    int err;

    printk(KERN_INFO "kerrsmem_setup_cdev\n");

    cdev_init(kerr_devp->cdev, &kerrsmem_fops);

    kerr_devp->cdev->owner = THIS_MODULE;
    kerr_devp->cdev->ops = &kerrsmem_fops;

    err = cdev_add(kerr_devp->cdev, devno, 1);
    if (err) {
        printk(KERN_ERR "kerrsmem cdev registration failed !\n\n");
        return -1;
    }

    kerr_devp->kerr_class = class_create(THIS_MODULE, "kerr");
    if (IS_ERR(kerr_devp->kerr_class)) {
        printk(KERN_ERR "Error creating kerrsmem class.\n");
        return -1;
    }

    device_create(kerr_devp->kerr_class, NULL, devno,
                  (void *)kerr_devp, "kclog_smem");

    return 0;
}

static int kerrsmem_cleanup(void)
{
    printk(KERN_INFO "kerrsmem_cleanup\n");

    if (kerr_devp) {
        /* Destroy device */
        if (kerr_devp->cdev) {
            device_destroy(kerr_devp->kerr_class, 
                            MKDEV(kerr_devp->major, kerr_devp->minor_start));
            cdev_del(kerr_devp->cdev);
        }

        /* Destroy class */
        if (!IS_ERR(kerr_devp->kerr_class)) {
            class_destroy(kerr_devp->kerr_class);
        }

        #ifdef KERR_BLOCK_READ
        if (kerr_devp->smem_wq) {
            destroy_workqueue(kerr_devp->smem_wq);
        }
        #endif /* KERR_BLOCK_READ */

        kfree(kerr_devp);
    }

    return 0;
}

static int __init kerrsmem_init(void)
{
    dev_t dev;
    int error;

    printk(KERN_INFO "kerrsmem_init\n");

    kerr_devp = kzalloc(sizeof(struct kerrsmem_dev) + 11, GFP_KERNEL);
    if (kerr_devp) {
        mutex_init(&kerr_devp->kerrsmem_mutex);

        #ifdef KERR_BLOCK_READ
        init_waitqueue_head(&kerr_devp->wait_q);
        INIT_WORK(&kerr_devp->smem_read_work, kerrsmem_read_work_fn);

        kerr_devp->smem_wq = create_singlethread_workqueue("smem_wq");
        #endif /* KERR_BLOCK_READ */

        kerr_devp->num = 1;
        kerr_devp->name = ((void *)kerr_devp) + sizeof(struct kerrsmem_dev);
        strlcpy(kerr_devp->name, "kclog_smem", 11);

        /* Get major number from kernel and initialize */
        error = alloc_chrdev_region(&dev, kerr_devp->minor_start,
                        kerr_devp->num, kerr_devp->name);
        if (!error) {
            kerr_devp->major = MAJOR(dev);
            kerr_devp->minor_start = MINOR(dev);
        } else {
            printk(KERN_ERR "Major number not allocated\n");
            goto fail;
        }

        kerr_devp->cdev = cdev_alloc();
        error = kerrsmem_setup_cdev(dev);
        if (error) {
            goto fail;
        }

        #ifndef KERR_SMEM_DEBUG
        kerr_devp->smem = (kerrsmem_log_t*)kc_smem_alloc(SMEM_KERR_LOG, 
                                            sizeof(kerrsmem_log_t));
        #else
        kerr_devp->smem = kmalloc(sizeof(kerrsmem_log_t), GFP_KERNEL);
        #endif /* KERR_SMEM_DEBUG */
        if (kerr_devp->smem == NULL) {
            printk(KERN_ERR "smem_alloc() failed!\n");
            goto fail;
        }

        #ifdef KERR_BLOCK_READ
        queue_work(kerr_devp->smem_wq, &kerr_devp->smem_read_work);
        #endif /* KERR_BLOCK_READ */
    } else {
        printk(KERN_ERR "kzalloc failed!\n");
        goto fail;
    }

    printk(KERN_INFO "kerrsmem initialized!\n");
    return 0;

fail:
    kerrsmem_cleanup();
    return -1;
}

static void __exit kerrsmem_exit(void)
{
    printk(KERN_INFO "kerrsmem_exit...\n");
    kerrsmem_cleanup();
    printk(KERN_INFO "kerrsmem_exit done!\n");
}

module_init(kerrsmem_init);
module_exit(kerrsmem_exit);
