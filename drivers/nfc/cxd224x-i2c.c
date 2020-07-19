/*
 *  cxd224x-i2c.c - cxd224x NFC i2c driver
 *
 * Copyright (C) 2013-2016 Sony Corporation.
 * Copyright (C) 2012 Broadcom Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 * (C) 2018 KYOCERA Corporation
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/poll.h>
#include <linux/version.h>
#include <linux/of_gpio.h>

#include <linux/nfc/cxd224x.h>
#include <linux/wakelock.h>

#include <linux/cdev.h>

/************************************************************************/
/*  DEFINITION                                                          */
/************************************************************************/
#define CXD224X_WAKE_LOCK_TIMEOUT       10        /* wake lock timeout for HOSTINT (sec) */
#define CXD224X_WAKE_LOCK_NAME          "cxd224x-i2c"        /* wake lock for HOSTINT */
#define CXD224X_WAKE_LOCK_TIMEOUT_LP    3        /* wake lock timeout for low-power-mode (sec) */
#define CXD224X_WAKE_LOCK_NAME_LP       "cxd224x-i2c-lp"    /* wake lock for low-power-mode */

/* do not change below */
#define MAX_BUFFER_SIZE                 780

/* Read data */
#define PACKET_HEADER_SIZE_NCI          (3)
#define PACKET_HEADER_SIZE_HCI          (3)
#define PACKET_TYPE_NCI                 (16)
#define PACKET_TYPE_HCIEV               (4)
#define MAX_PACKET_SIZE                 (PACKET_HEADER_SIZE_NCI + 255)

/* RESET */
#define RESET_ASSERT_MS                 (10)

#undef CONFIG_OF

/* KC Custom */
/* CXD224X device */
#define CXD224X_DEV_NAME                "cxd224x-i2c"
#define CXD224X_DEV_COUNT               1

/* RFS  device */
#define RFS_DEV_COUNT                   1
#define RFS_DEV_NAME                    "felica_rfs"
#define RFS_GPIO_VAL_H                  1
#define RFS_GPIO_VAL_L                  0
#define RFS_RET_STS_INACTIVE            0
#define RFS_RET_STS_ACTIVE              1

/* USB device */
#define VBUS_DEV_COUNT                  1
#define VBUS_DEV_NAME                   "knfcvbus"

/* DEBUG_LOG */
#if 0
#define DEBUG_KNFC_DRIVER
#endif

#ifdef DEBUG_KNFC_DRIVER
#define KNFC_LOG_D(fmt, args...) printk(KERN_INFO "[KNFC][%s]" fmt"\n", __func__, ## args)
#else
#define KNFC_LOG_D(fmt, args...)
#endif

/* INFO_LOG */
#define KNFC_LOG_I(fmt, args...) printk(KERN_INFO "[KNFC][%s]" fmt"\n", __func__, ## args)

/* ERROR_LOG */
#define KNFC_LOG_E(fmt, args...) printk(KERN_ERR "[KNFC][%s]ERR " fmt"\n", __func__, ## args)

#define REGISTER_AS_MISC_DRIVER         1
#define DEBUG_READ_BUF_SIZE             255

#ifdef DEBUG_KNFC_DRIVER
#define CXDNFC_DEBUG_SKIP_RESPONSE_CTL  _IO(CXDNFC_MAGIC, 0xF0)
#endif

/************************************************************************/
/*  GLOBAL                                                              */
/************************************************************************/
static struct class*        knfc_class = NULL;
static struct cdev          rfs_cdev;
static struct cdev          knfcvbus_cdev;

static int                  g_nfc_rfs_gpio = -1;

#ifdef DEBUG_KNFC_DRIVER
static char                 g_debug_read_string_buf[DEBUG_READ_BUF_SIZE * 2];
#endif

#ifdef DEBUG_KNFC_DRIVER
static int                  g_debug_skip_response;
#endif

/************************************************************************/
/* PROTOTYPE                                                            */
/************************************************************************/
#if !REGISTER_AS_MISC_DRIVER
static void     cxd224x_exit(struct cdev *device);
#endif
static int      cxd224x_dev_init(void);
static void     cxd224x_dev_exit(void);

static ssize_t  rfs_read(struct file *file, char __user * buf, size_t len, loff_t * ppos);
static ssize_t  rfs_write(struct file *file, const char __user *data, size_t len, loff_t *ppos);
static int      rfs_open(struct inode *inode, struct file *file);
static int      rfs_release(struct inode *inode, struct file *file);
static int      rfs_init(void);
static void     rfs_exit(void);

static ssize_t  knfcvbus_read(struct file *file, char __user * buf, size_t len, loff_t * ppos);
static int      knfcvbus_open(struct inode *inode, struct file *file);
static int      knfcvbus_release(struct inode *inode, struct file *file);

#ifdef DEBUG_KNFC_DRIVER
static char     *print_byte_array(unsigned char *buf, int size);
#endif

extern bool     is_vbus_active(void);

/************************************************************************/
/* STRUCT                                                               */
/************************************************************************/
struct cxd224x_dev {
    wait_queue_head_t read_wq;
    struct mutex read_mutex;
    struct i2c_client *client;
#if REGISTER_AS_MISC_DRIVER
    struct miscdevice cxd224x_device;
#else
    struct cdev cxd224x_device;
#endif
    struct cxd224x_platform_data *gpio;
    bool irq_enabled;
    struct mutex lock;
    spinlock_t irq_enabled_lock;
    unsigned int users;
    unsigned int count_irq;
    struct wake_lock wakelock;    /* wake lock for HOSTINT */
    struct wake_lock wakelock_lp;    /* wake lock for low-power-mode */
    /* Driver message queue */
    struct workqueue_struct    *wqueue;
    struct work_struct qmsg;
};

/************************************************************************/
/* FUNCTION                                                             */
/************************************************************************/
#if defined(CONFIG_CXD224X_NFC_RST) || defined(CONFIG_CXD224X_NFC_RST_MODULE)
static void cxd224x_workqueue(struct work_struct *work)
{
    struct cxd224x_dev *cxd224x_dev = container_of(work, struct cxd224x_dev, qmsg);
    unsigned long flags;

    KNFC_LOG_D("START");

    dev_info(&cxd224x_dev->client->dev, "%s, xrst assert\n", __func__);
    spin_lock_irqsave(&cxd224x_dev->irq_enabled_lock, flags);
    gpio_set_value(cxd224x_dev->gpio->rst_gpio, CXDNFC_RST_ACTIVE);
    cxd224x_dev->count_irq=0; /* clear irq */
    spin_unlock_irqrestore(&cxd224x_dev->irq_enabled_lock, flags);

    msleep(RESET_ASSERT_MS);
    dev_info(&cxd224x_dev->client->dev, "%s, xrst deassert\n", __func__);
    gpio_set_value(cxd224x_dev->gpio->rst_gpio, ~CXDNFC_RST_ACTIVE & 0x1);

    KNFC_LOG_D("END");
}

static int __init init_wqueue(struct cxd224x_dev *cxd224x_dev)
{
    KNFC_LOG_D("START");
    INIT_WORK(&cxd224x_dev->qmsg, cxd224x_workqueue);
    cxd224x_dev->wqueue = create_workqueue("cxd224x-i2c_wrokq");
    if (cxd224x_dev->wqueue == NULL)
        return -EBUSY;
    KNFC_LOG_D("END");
    return 0;
}
#endif /* CONFIG_CXD224X_NFC_RST */

static void cxd224x_init_stat(struct cxd224x_dev *cxd224x_dev)
{
    cxd224x_dev->count_irq = 0;
}

static void cxd224x_disable_irq(struct cxd224x_dev *cxd224x_dev)
{
    unsigned long flags;
    KNFC_LOG_D("START");
    spin_lock_irqsave(&cxd224x_dev->irq_enabled_lock, flags);
    if (cxd224x_dev->irq_enabled) {
        disable_irq_nosync(cxd224x_dev->client->irq);
        cxd224x_dev->irq_enabled = false;
    }
    spin_unlock_irqrestore(&cxd224x_dev->irq_enabled_lock, flags);
    KNFC_LOG_D("END");
}

static void cxd224x_enable_irq(struct cxd224x_dev *cxd224x_dev)
{
    unsigned long flags;
    KNFC_LOG_D("START");
    spin_lock_irqsave(&cxd224x_dev->irq_enabled_lock, flags);
    if (!cxd224x_dev->irq_enabled) {
        cxd224x_dev->irq_enabled = true;
        enable_irq(cxd224x_dev->client->irq);
    }
    spin_unlock_irqrestore(&cxd224x_dev->irq_enabled_lock, flags);
    KNFC_LOG_D("END");
}

static irqreturn_t cxd224x_dev_irq_handler(int irq, void *dev_id)
{
    struct cxd224x_dev *cxd224x_dev = dev_id;
    unsigned long flags;

    KNFC_LOG_D("START");
    spin_lock_irqsave(&cxd224x_dev->irq_enabled_lock, flags);
    cxd224x_dev->count_irq++;
    spin_unlock_irqrestore(&cxd224x_dev->irq_enabled_lock, flags);
    wake_up(&cxd224x_dev->read_wq);
    KNFC_LOG_D("END");

    return IRQ_HANDLED;
}

static unsigned int cxd224x_dev_poll(struct file *filp, poll_table *wait)
{
    struct cxd224x_dev *cxd224x_dev = filp->private_data;
    unsigned int mask = 0;
    unsigned long flags;

    KNFC_LOG_D("START");
    poll_wait(filp, &cxd224x_dev->read_wq, wait);

    spin_lock_irqsave(&cxd224x_dev->irq_enabled_lock, flags);
    if (cxd224x_dev->count_irq > 0)
    {
        cxd224x_dev->count_irq--;
        mask |= POLLIN | POLLRDNORM;
    }
    spin_unlock_irqrestore(&cxd224x_dev->irq_enabled_lock, flags);

    if(mask) 
        wake_lock_timeout(&cxd224x_dev->wakelock, CXD224X_WAKE_LOCK_TIMEOUT*HZ);

    KNFC_LOG_D("END");

    return mask;
}

static ssize_t cxd224x_dev_read(struct file *filp, char __user *buf,
        size_t count, loff_t *offset)
{
    struct cxd224x_dev *cxd224x_dev = filp->private_data;
    unsigned char tmp[MAX_BUFFER_SIZE];
    int total, len, ret;

    KNFC_LOG_D("START");

    total = 0;
    len = 0;

    if (count > MAX_BUFFER_SIZE)
        count = MAX_BUFFER_SIZE;

    mutex_lock(&cxd224x_dev->read_mutex);

    ret = i2c_master_recv(cxd224x_dev->client, tmp, 3);
    if (ret == 3 && (tmp[0] != 0xff)) {
        total = ret;

        len = tmp[PACKET_HEADER_SIZE_NCI-1];

        /** make sure full packet fits in the buffer
         **/
        if (len > 0 && (len + total) <= count) {
            /** read the remainder of the packet.
             **/
            ret = i2c_master_recv(cxd224x_dev->client, tmp+total, len);
            if (ret == len)
                total += len;
        }
    } 

    mutex_unlock(&cxd224x_dev->read_mutex);

    if ( total > 0 ) {
        KNFC_LOG_D("read:%s", print_byte_array(tmp, total));
    }

#ifdef DEBUG_KNFC_DRIVER
    if ( g_debug_skip_response > 0 ) {
        KNFC_LOG_D("debug no response");
        total = 3;
        g_debug_skip_response--;
    } else
#endif
    if (total > count || copy_to_user(buf, tmp, total)) {
        dev_err(&cxd224x_dev->client->dev,
                "failed to copy to user space, total = %d\n", total);
        total = -EFAULT;
    }

    KNFC_LOG_D("END");

    return total;
}

static ssize_t cxd224x_dev_write(struct file *filp, const char __user *buf,
        size_t count, loff_t *offset)
{
    struct cxd224x_dev *cxd224x_dev = filp->private_data;
    char tmp[MAX_BUFFER_SIZE];
    int ret;

    KNFC_LOG_D("START");

    if (count > MAX_BUFFER_SIZE) {
        dev_err(&cxd224x_dev->client->dev, "out of memory\n");
        return -ENOMEM;
    }

    if (copy_from_user(tmp, buf, count)) {
        dev_err(&cxd224x_dev->client->dev,
                "failed to copy from user space\n");
        return -EFAULT;
    }

    mutex_lock(&cxd224x_dev->read_mutex);
    /* Write data */

    KNFC_LOG_D("write:%s", print_byte_array(tmp, count));

    ret = i2c_master_send(cxd224x_dev->client, tmp, count);
    if (ret != count) {
        dev_err(&cxd224x_dev->client->dev,
                "failed to write %d\n", ret);
        ret = -EIO;
    }
    mutex_unlock(&cxd224x_dev->read_mutex);

    KNFC_LOG_D("END");

    return ret;
}

static int cxd224x_dev_open(struct inode *inode, struct file *filp)
{
    int ret = 0;
    int call_enable = 0;
    struct cxd224x_dev *cxd224x_dev = container_of(filp->private_data,
            struct cxd224x_dev,
            cxd224x_device);

    KNFC_LOG_D("START");

    filp->private_data = cxd224x_dev;
    mutex_lock(&cxd224x_dev->lock);
    if (!cxd224x_dev->users)
    {
        cxd224x_init_stat(cxd224x_dev);
        call_enable = 1;
    }
    cxd224x_dev->users++;
    mutex_unlock(&cxd224x_dev->lock);
    if (call_enable)
        cxd224x_enable_irq(cxd224x_dev);

    dev_info(&cxd224x_dev->client->dev,
            "open %d,%d users=%d\n", imajor(inode), iminor(inode), cxd224x_dev->users);

    KNFC_LOG_D("END");

    return ret;
}

static int cxd224x_dev_release(struct inode *inode, struct file *filp)
{
    int ret = 0;
    int call_disable = 0;
    struct cxd224x_dev *cxd224x_dev = filp->private_data;

    KNFC_LOG_D("START");

    mutex_lock(&cxd224x_dev->lock);
    cxd224x_dev->users--;
    if (!cxd224x_dev->users)
    {
        call_disable = 1;
    }
    mutex_unlock(&cxd224x_dev->lock);
    if (call_disable)
        cxd224x_disable_irq(cxd224x_dev);

    dev_info(&cxd224x_dev->client->dev,
            "release %d,%d users=%d\n", imajor(inode), iminor(inode), cxd224x_dev->users);

    KNFC_LOG_D("END");

    return ret;
}

static long cxd224x_dev_unlocked_ioctl(struct file *filp,
        unsigned int cmd, unsigned long arg)
{
    struct cxd224x_dev *cxd224x_dev = filp->private_data;

    KNFC_LOG_D("START cmd:0x%x, arg:0x%lx\n", cmd, arg);

    switch (cmd) {
    case CXDNFC_RST_CTL:
#ifdef DEBUG_KNFC_DRIVER
        g_debug_skip_response = 0;
#endif
#if defined(CONFIG_CXD224X_NFC_RST) || defined(CONFIG_CXD224X_NFC_RST_MODULE)
        KNFC_LOG_I("START CXDNFC_RST_CTL cmd:0x%x, arg:0x%lx\n", cmd, arg);
        dev_info(&cxd224x_dev->client->dev, "%s, rst arg=%d\n", __func__, (int)arg);
        return (queue_work(cxd224x_dev->wqueue, &cxd224x_dev->qmsg) ? 0 : 1);
#endif
        break;
    case CXDNFC_POWER_CTL:
#if defined(CONFIG_CXD224X_NFC_VEN) || defined(CONFIG_CXD224X_NFC_VEN_MODULE)
        KNFC_LOG_I("START CXDNFC_POWER_CTL cmd:0x%x, arg:0x%lx\n", cmd, arg);
        if (arg == 0) {
            gpio_set_value(cxd224x_dev->en_gpio, 1);
        } else if (arg == 1) {
            gpio_set_value(cxd224x_dev->en_gpio, 0);  
        } else {
            /* do nothing */
        }
#else
        return 1; /* not support */
#endif
        break;
    case CXDNFC_WAKE_CTL:
        KNFC_LOG_I("START CXDNFC_WAKE_CTL cmd:0x%x, arg:0x%lx\n", cmd, arg);
        if (arg == 0) {
            wake_lock_timeout(&cxd224x_dev->wakelock_lp, CXD224X_WAKE_LOCK_TIMEOUT_LP*HZ);
            /* PON HIGH (normal power mode)*/
            gpio_set_value(cxd224x_dev->gpio->wake_gpio, 1);
        } else if (arg == 1) {
            /* PON LOW (low power mode) */
            gpio_set_value(cxd224x_dev->gpio->wake_gpio, 0);
            wake_unlock(&cxd224x_dev->wakelock_lp);
        } else {
            /* do nothing */
        }
        break;
#ifdef DEBUG_KNFC_DRIVER
    case CXDNFC_DEBUG_SKIP_RESPONSE_CTL:
        KNFC_LOG_I("START CXDNFC_DEBUG_SKIP_RESPONSE_CTL cmd:0x%x, arg:0x%lx\n", cmd, arg);
        g_debug_skip_response = (int)arg;
        break;
#endif
    default:
        dev_err(&cxd224x_dev->client->dev,
                "%s, unknown cmd (%x, %lx)\n", __func__, cmd, arg);
        return 0;
    }

    KNFC_LOG_D("END");

    return 0;
}

static const struct file_operations cxd224x_dev_fops = {
    .owner = THIS_MODULE,
    .llseek = no_llseek,
    .poll = cxd224x_dev_poll,
    .read = cxd224x_dev_read,
    .write = cxd224x_dev_write,
    .open = cxd224x_dev_open,
    .release = cxd224x_dev_release,
    .unlocked_ioctl = cxd224x_dev_unlocked_ioctl
};

#if defined(CONFIG_OF)
static int cxd224x_parse_dt(struct device *dev,
        struct cxd224x_platform_data *pdata)
{
    int ret=0;

    /*nfc_int*/
    pdata->irq_gpio =  of_get_named_gpio_flags(dev->of_node, "sony,nfc_int", 0,NULL);
    if (pdata->irq_gpio < 0) {
        pr_err( "failed to get \"nfc_int\"\n");
        goto dt_err;
    }

#if defined(CONFIG_CXD224X_NFC_VEN) || defined(CONFIG_CXD224X_NFC_VEN_MODULE)
    pdata->en_gpio = of_get_named_gpio_flags(dev->of_node, "sony,nfc_ven", 0,NULL);
    if (pdata->en_gpio< 0) {
        pr_err( "failed to get \"nfc_ven\"\n");
        goto dt_err;
    }
#endif

#if defined(CONFIG_CXD224X_NFC_RST) || defined(CONFIG_CXD224X_NFC_RST_MODULE)
    pdata->rst_gpio = of_get_named_gpio_flags(dev->of_node, "sony,nfc_rst", 0,NULL);
    if (pdata->rst_gpio< 0) {
        pr_err( "failed to get \"nfc_rst\"\n");
        goto dt_err;
    }
#endif

    pdata->wake_gpio = of_get_named_gpio_flags(dev->of_node, "sony,nfc_wake", 0,NULL);
    if (pdata->wake_gpio< 0) {
        pr_err( "failed to get \"nfc_wake\"\n");
        goto dt_err;
    }

    pdata->rfs_gpio = of_get_named_gpio_flags(dev->of_node, "sony,nfc_rfs", 0,NULL);
    if (pdata->rfs_gpio< 0) {
        pr_err( "failed to get \"nfc_rfs\"\n");
        goto dt_err;
    }

    return 0;

dt_err:
    return ret;        
}
#endif

static int cxd224x_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    int ret;
    struct cxd224x_platform_data *platform_data=NULL;
    struct cxd224x_dev *cxd224x_dev;
    int irq_gpio_ok  = 0;
#if defined(CONFIG_CXD224X_NFC_VEN) || defined(CONFIG_CXD224X_NFC_VEN_MODULE)
    int en_gpio_ok   = 0;
#endif
#if defined(CONFIG_CXD224X_NFC_RST) || defined(CONFIG_CXD224X_NFC_RST_MODULE)
    int rst_gpio_ok = 0;
#endif
    int wake_gpio_ok = 0;
    int rfs_gpio_ok = 0;

#if !REGISTER_AS_MISC_DRIVER
    struct device *class_dev;
    dev_t dev = MKDEV(MISC_MAJOR, 0);
#endif

    KNFC_LOG_I("START");

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "need I2C_FUNC_I2C\n");
        return -ENODEV;
    }

#if defined(CONFIG_OF)
    platform_data = kzalloc(sizeof(struct cxd224x_platform_data),
            GFP_KERNEL);
    if (platform_data == NULL) {
        dev_err(&client->dev, "failed to allocate memory\n");
        return -ENOMEM;
    }
    ret = cxd224x_parse_dt(&client->dev, platform_data);
    if (ret) {
        dev_err(&client->dev, "failed to parse device tree\n");
        kfree(platform_data);
        return -ENODEV;
    }
#else
    platform_data = client->dev.platform_data;

    dev_info(&client->dev, "%s, probing cxd224x driver flags = %x\n", __func__, client->flags);
    if (platform_data == NULL) {
        dev_err(&client->dev, "nfc probe fail\n");
        return -ENODEV;
    }
#endif
    dev_info(&client->dev, "%s, rst_gpio(%d)\n", __func__, platform_data->rst_gpio);
    dev_info(&client->dev, "%s, ven_gpio(%d)\n", __func__, platform_data->en_gpio);
    dev_info(&client->dev, "%s, irq_gpio(%d)\n", __func__, platform_data->irq_gpio);
    dev_info(&client->dev, "%s, wake_gpio(%d)\n", __func__, platform_data->wake_gpio);
    dev_info(&client->dev, "%s, rfs_gpio(%d)\n", __func__, platform_data->rfs_gpio);

    irq_gpio_ok=1;
    client->irq = gpio_to_irq(platform_data->irq_gpio);
    if (client->irq<0)
    {
        dev_err(&client->dev, "%s, failed to allocate irq=%d\n", __func__, client->irq);
        return -ENODEV;
    }
    dev_info(&client->dev, "%s, irq(%d)\n", __func__, client->irq);

#if defined(CONFIG_CXD224X_NFC_VEN) || defined(CONFIG_CXD224X_NFC_VEN_MODULE)
    ret = gpio_request_one(platform_data->en_gpio, GPIOF_OUT_INIT_LOW, "nfc_cen");
    if (ret)
        goto err_exit;
    en_gpio_ok=1;
    ret = gpio_direction_output(platform_data->en_gpio, 0);
    if (ret)
        return -ENODEV;
#endif

#if defined(CONFIG_CXD224X_NFC_RST) || defined(CONFIG_CXD224X_NFC_RST_MODULE)
    ret = gpio_request_one(platform_data->rst_gpio, GPIOF_OUT_INIT_LOW, "nfc_rst");
    if (ret)
        goto err_exit;
    rst_gpio_ok=1;
    ret = gpio_direction_output(platform_data->rst_gpio, ~CXDNFC_RST_ACTIVE & 0x1);
    if (ret)
        return -ENODEV;
    dev_info(&client->dev, "%s, xrst deassert\n", __func__);
#endif

    ret = gpio_request_one(platform_data->wake_gpio, GPIOF_OUT_INIT_LOW, "nfc_wake");
    if (ret)
        goto err_exit;
    wake_gpio_ok=1;
    ret = gpio_direction_output(platform_data->wake_gpio, 0);

    ret = gpio_request_one(platform_data->rfs_gpio, GPIOF_IN, "nfc_rfs");
    if (ret)
        goto err_exit;
    rfs_gpio_ok=1;
    g_nfc_rfs_gpio = platform_data->rfs_gpio;

    cxd224x_dev = kzalloc(sizeof(*cxd224x_dev), GFP_KERNEL);
    if (cxd224x_dev == NULL) {
        dev_err(&client->dev,
                "failed to allocate memory for module data\n");
        ret = -ENOMEM;
        goto err_exit;
    }

    cxd224x_dev->client = client;
    cxd224x_dev->gpio = platform_data;
    wake_lock_init(&cxd224x_dev->wakelock, WAKE_LOCK_SUSPEND, CXD224X_WAKE_LOCK_NAME);
    wake_lock_init(&cxd224x_dev->wakelock_lp, WAKE_LOCK_SUSPEND, CXD224X_WAKE_LOCK_NAME_LP);
    cxd224x_dev->users =0;

    /* init mutex and queues */
    init_waitqueue_head(&cxd224x_dev->read_wq);
    mutex_init(&cxd224x_dev->read_mutex);
    mutex_init(&cxd224x_dev->lock);
    spin_lock_init(&cxd224x_dev->irq_enabled_lock);

#if defined(CONFIG_CXD224X_NFC_RST) || defined(CONFIG_CXD224X_NFC_RST_MODULE)
    if (init_wqueue(cxd224x_dev) != 0) {
        dev_err(&client->dev, "init workqueue failed\n");
        goto err_exit;
    }
#endif

#if REGISTER_AS_MISC_DRIVER
    cxd224x_dev->cxd224x_device.minor = MISC_DYNAMIC_MINOR;
    cxd224x_dev->cxd224x_device.name = CXD224X_DEV_NAME;
    cxd224x_dev->cxd224x_device.fops = &cxd224x_dev_fops;

    ret = misc_register(&cxd224x_dev->cxd224x_device);
    if (ret) {
        dev_err(&client->dev, "misc_register failed\n");
        goto err_misc_register;
    }
#else
    ret = alloc_chrdev_region(&dev , 0 , CXD224X_DEV_COUNT, CXD224X_DEV_NAME);
    if (ret) {
        KNFC_LOG_E("alloc_chrdev_region ret = %d", ret);
        goto err_device_create;
    }

    cdev_init(&cxd224x_dev->cxd224x_device, &cxd224x_dev_fops);
    cxd224x_dev->cxd224x_device.owner = THIS_MODULE;
    cxd224x_dev->cxd224x_device.ops = &cxd224x_dev_fops;

    ret = cdev_add(&cxd224x_dev->cxd224x_device, dev, CXD224X_DEV_COUNT);
    if (ret) {
        unregister_chrdev_region(dev, CXD224X_DEV_COUNT);
        KNFC_LOG_E("cdev_add ret = %d", ret);
        goto err_device_create;
    }

    class_dev = device_create(knfc_class, NULL, dev, NULL, CXD224X_DEV_NAME);
    if (IS_ERR(class_dev)) {
        cdev_del(&cxd224x_dev->cxd224x_device);
        unregister_chrdev_region(dev, CXD224X_DEV_COUNT);
        ret = PTR_ERR(class_dev);
        KNFC_LOG_E("device_create ret = %d", ret);
        goto err_device_create;
    }
#endif

    /* request irq.  the irq is set whenever the chip has data available
     * for reading.  it is cleared when all data has been read.
     */
    dev_info(&client->dev, "requesting IRQ %d\n", client->irq);
    cxd224x_dev->irq_enabled = true;
    ret = request_irq(client->irq, cxd224x_dev_irq_handler,
            IRQF_TRIGGER_FALLING, client->name, cxd224x_dev);
    if (ret) {
        dev_err(&client->dev, "request_irq failed\n");
        goto err_request_irq_failed;
    }
    cxd224x_disable_irq(cxd224x_dev);
    i2c_set_clientdata(client, cxd224x_dev);
    dev_info(&client->dev,
            "%s, probing cxd224x driver exited successfully\n",
            __func__);
    return 0;

err_request_irq_failed:
#if REGISTER_AS_MISC_DRIVER
    misc_deregister(&cxd224x_dev->cxd224x_device);
err_misc_register:
#else
    cxd224x_exit(&cxd224x_dev->cxd224x_device);
err_device_create:
#endif
    mutex_destroy(&cxd224x_dev->read_mutex);
    kfree(cxd224x_dev);
err_exit:
    if(irq_gpio_ok)
        gpio_free(platform_data->irq_gpio);
#if defined(CONFIG_CXD224X_NFC_VEN) || defined(CONFIG_CXD224X_NFC_VEN_MODULE)
    if(en_gpio_ok)
        gpio_free(platform_data->en_gpio);
#endif
#if defined(CONFIG_CXD224X_NFC_RST) || defined(CONFIG_CXD224X_NFC_RST_MODULE)
    if(rst_gpio_ok)
        gpio_free(platform_data->rst_gpio);
#endif
    if(wake_gpio_ok)
        gpio_free(platform_data->wake_gpio);

    if(rfs_gpio_ok)
        gpio_free(platform_data->rfs_gpio);

#if defined(CONFIG_OF)
    if(platform_data)
        kfree(platform_data);
#endif

    KNFC_LOG_I("END");

    return ret;
}

static int cxd224x_remove(struct i2c_client *client)
{
    struct cxd224x_dev *cxd224x_dev;

    KNFC_LOG_D("START");

    cxd224x_dev = i2c_get_clientdata(client);
    wake_lock_destroy(&cxd224x_dev->wakelock);
    wake_lock_destroy(&cxd224x_dev->wakelock_lp);
    free_irq(client->irq, cxd224x_dev);
#if REGISTER_AS_MISC_DRIVER
    misc_deregister(&cxd224x_dev->cxd224x_device);
#else
    cxd224x_exit(&cxd224x_dev->cxd224x_device);
#endif
    mutex_destroy(&cxd224x_dev->read_mutex);
    if(cxd224x_dev->gpio)
    {
        gpio_free(cxd224x_dev->gpio->irq_gpio);
        gpio_free(cxd224x_dev->gpio->wake_gpio);

#if defined(CONFIG_CXD224X_NFC_VEN) || defined(CONFIG_CXD224X_NFC_VEN_MODULE)
        gpio_free(cxd224x_dev->gpio->en_gpio);
#endif
#if defined(CONFIG_CXD224X_NFC_RST) || defined(CONFIG_CXD224X_NFC_RST_MODULE)
        gpio_free(cxd224x_dev->gpio->rst_gpio);
#endif

        gpio_free(cxd224x_dev->gpio->rfs_gpio);

#if defined(CONFIG_OF)

        kfree(cxd224x_dev->gpio);
#endif
    }
    kfree(cxd224x_dev);

    KNFC_LOG_D("END");

    return 0;
}

#if !REGISTER_AS_MISC_DRIVER
static void cxd224x_exit(struct cdev *device)
{
    dev_t dev = MKDEV(MISC_MAJOR, 0);

    KNFC_LOG_I("START");

    device_destroy(knfc_class, dev);

    cdev_del(device);
    unregister_chrdev_region(dev, CXD224X_DEV_COUNT);

    KNFC_LOG_I("END");

}
#endif

#ifdef CONFIG_PM_SLEEP
static int cxd224x_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct cxd224x_dev *cxd224x_dev = i2c_get_clientdata(client);

    KNFC_LOG_D("START");

/* Do not enable IRQ in the state without HalOpen */
/*  if (device_may_wakeup(&client->dev)) { */
    if (cxd224x_dev->users > 0 && device_may_wakeup(&client->dev)) {
        int irq = gpio_to_irq(cxd224x_dev->gpio->irq_gpio);
        enable_irq_wake(irq);
    }

    KNFC_LOG_D("END");

    return 0;
}

static int cxd224x_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct cxd224x_dev *cxd224x_dev = i2c_get_clientdata(client);

    KNFC_LOG_D("START");

    if (device_may_wakeup(&client->dev)) {
        int irq = gpio_to_irq(cxd224x_dev->gpio->irq_gpio);
        disable_irq_wake(irq);
    }

    KNFC_LOG_D("END");

    return 0;
}

static const struct dev_pm_ops cxd224x_pm_ops = {
    .suspend    = cxd224x_suspend,
    .resume        = cxd224x_resume,
};
#endif


static const struct file_operations rfs_fileops = {
    .owner   = THIS_MODULE,
    .read    = rfs_read,
    .write    = rfs_write,
    .open    = rfs_open,
    .release = rfs_release,
};

static const struct file_operations knfcvbus_fileops = {
    .owner   = THIS_MODULE,
    .read    = knfcvbus_read,
    .open    = knfcvbus_open,
    .release = knfcvbus_release,
};

/*
 * function_rfs
 */
static ssize_t rfs_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
    int ret;
    char on;

    KNFC_LOG_D("START");

    if ( len < 1 ) {
        KNFC_LOG_E("length check len = %d", (int)len);
        return -EIO;
    }

    ret = gpio_get_value_cansleep( g_nfc_rfs_gpio );
    if (ret < 0) {
        KNFC_LOG_E("gpio_get_value ret = %d", ret);
        return ret;
    }
    if( RFS_GPIO_VAL_H == ret ){
        on = RFS_RET_STS_INACTIVE;
    }else{
        on = RFS_RET_STS_ACTIVE;
    }
    len = 1;

    if (copy_to_user(buf, &on, len)) {
        KNFC_LOG_E("copy_to_user");
        return -EFAULT;
    }

    KNFC_LOG_D("END on = %d, len = %d", on, (int)len);

    return len;
}

static ssize_t rfs_write(struct file *file, const char __user *data, size_t len, loff_t *ppos)
{
    KNFC_LOG_D("START");
    KNFC_LOG_D("END");

    return len;
}

static int rfs_open(struct inode *inode, struct file *file)
{
    KNFC_LOG_D("");
    return 0;
}

static int rfs_release(struct inode *inode, struct file *file)
{
    KNFC_LOG_D("");
    return 0;
}

static int rfs_init(void)
{
    int sdResult = 0;
    struct device *class_dev;

    dev_t dev = MKDEV(MISC_MAJOR, 0);

    KNFC_LOG_I("START");

    sdResult = alloc_chrdev_region(&dev , 0 , RFS_DEV_COUNT, RFS_DEV_NAME);
    if (sdResult) {
        KNFC_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
        return sdResult;
    }

    cdev_init(&rfs_cdev, &rfs_fileops);
    rfs_cdev.owner = THIS_MODULE;

    sdResult = cdev_add(&rfs_cdev, dev, RFS_DEV_COUNT);
    if (sdResult) {
        unregister_chrdev_region(dev, RFS_DEV_COUNT);
        KNFC_LOG_E("cdev_add sdResult = %d", sdResult);
        return sdResult;
    }

    class_dev = device_create(knfc_class, NULL, dev, NULL, RFS_DEV_NAME);
    if (IS_ERR(class_dev)) {
        cdev_del(&rfs_cdev);
        unregister_chrdev_region(dev, RFS_DEV_COUNT);
        sdResult = PTR_ERR(class_dev);
        KNFC_LOG_E("device_create sdResult = %d", sdResult);
        return sdResult;
    }

    KNFC_LOG_I("END");

    return sdResult;
}

static void rfs_exit(void)
{
    dev_t dev = MKDEV(MISC_MAJOR, 0);

    KNFC_LOG_I("START");

    cdev_del(&rfs_cdev);
    unregister_chrdev_region(dev, RFS_DEV_COUNT);

    KNFC_LOG_I("END");
}

/*
 * USB device
 */
static ssize_t knfcvbus_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
    bool vbus_val = 0;

    KNFC_LOG_D("START");

    if( 1 > len ){
        KNFC_LOG_E("length check len = %d", (int)len);
        return -EIO;
    }
    vbus_val = is_vbus_active();

    if (copy_to_user(buf, &vbus_val, 1)) {
        KNFC_LOG_E("copy_to_user");
        return -EFAULT;
    }

    KNFC_LOG_D("END rate_val = %d, len = %d", (int)vbus_val, (int)len);

    return 1;
}

static int knfcvbus_open(struct inode *inode, struct file *file)
{
    KNFC_LOG_D("START");
    KNFC_LOG_D("END");
    return 0;
}

static int knfcvbus_release(struct inode *inode, struct file *file)
{
    KNFC_LOG_D("START");
    KNFC_LOG_D("END");
    return 0;
}

static int knfcvbus_init(void)
{
    int sdResult = 0;
    struct device *class_dev;

    dev_t dev = MKDEV(MISC_MAJOR, 0);

    KNFC_LOG_I("START");

    sdResult = alloc_chrdev_region(&dev , 0 , VBUS_DEV_COUNT, VBUS_DEV_NAME);
    if (sdResult) {
        KNFC_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
        return sdResult;
    }

    cdev_init(&knfcvbus_cdev, &knfcvbus_fileops);
    knfcvbus_cdev.owner = THIS_MODULE;

    sdResult = cdev_add(&knfcvbus_cdev, dev, VBUS_DEV_COUNT);
    if (sdResult) {
        unregister_chrdev_region(dev, VBUS_DEV_COUNT);
        KNFC_LOG_E("cdev_add sdResult = %d", sdResult);
        return sdResult;
    }

    class_dev = device_create(knfc_class, NULL, dev, NULL, VBUS_DEV_NAME);
    if (IS_ERR(class_dev)) {
        cdev_del(&knfcvbus_cdev);
        unregister_chrdev_region(dev, VBUS_DEV_COUNT);
        sdResult = PTR_ERR(class_dev);
        KNFC_LOG_E("device_create sdResult = %d", sdResult);
        return sdResult;
    }

    KNFC_LOG_I("END");

    return sdResult;
}

static void knfcvbus_exit(void)
{
    dev_t dev = MKDEV(MISC_MAJOR, 0);

    KNFC_LOG_I("START");

    device_destroy(knfc_class, dev);

    cdev_del(&knfcvbus_cdev);
    unregister_chrdev_region(dev, VBUS_DEV_COUNT);

    KNFC_LOG_I("END");
}

#ifdef DEBUG_KNFC_DRIVER
static char *print_byte_array(unsigned char *buf, int size)
{
    int cnt = 0;

    memset(g_debug_read_string_buf, '\0', DEBUG_READ_BUF_SIZE * 2);

    for ( cnt = 0; cnt < size; cnt++ ) {
        sprintf(&g_debug_read_string_buf[cnt * 2], "%02x" ,*buf);
        buf++;
    }

    return g_debug_read_string_buf;
}
#endif

#ifdef CONFIG_OF
static struct of_device_id cxd224x_dt_match[] = {
    { .compatible = "sony,cxd224x-i2c", },
    {},
};
MODULE_DEVICE_TABLE(of, cxd224x_dt_match);
#endif


static const struct i2c_device_id cxd224x_id[] = {
    {CXD224X_DEV_NAME, 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, cxd224x_id);

static struct i2c_driver cxd224x_driver = {
    .id_table = cxd224x_id,
    .probe = cxd224x_probe,
    .remove = cxd224x_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = CXD224X_DEV_NAME,
#ifdef CONFIG_PM
        .pm    = &cxd224x_pm_ops,
#endif
    },
};

static int cxd224x_dev_init(void)
{
    return i2c_add_driver(&cxd224x_driver);
}

static void cxd224x_dev_exit(void)
{
    i2c_del_driver(&cxd224x_driver);
}

/*
 * module load/unload record keeping
 */
/*
 *  * knfc_init
 *   */
static int __init knfc_init(void)
{
    int ret;

    KNFC_LOG_I("START");
    knfc_class = class_create(THIS_MODULE, "knfc");

    if (IS_ERR(knfc_class)) {
        return PTR_ERR(knfc_class);
    }

    ret = cxd224x_dev_init();
    if( ret < 0 ){
        KNFC_LOG_E("cxd224x_dev_init err\n");
        return ret;
    }

    ret = rfs_init();
    if( ret < 0 ){
        KNFC_LOG_E("rfs init err\n");
        return ret;
    }

    ret = knfcvbus_init();
    if( ret < 0 ){
        KNFC_LOG_E("knfcvbus_init err = %d\n",ret);
    }

    KNFC_LOG_I("END");
    return 0;
}
module_init(knfc_init);

/*
 * knfc_exit
 */
static void  __exit knfc_exit(void)
{
    KNFC_LOG_I("START");

    class_destroy( knfc_class );

    rfs_exit();

    knfcvbus_exit();

    cxd224x_dev_exit();

    KNFC_LOG_I("END");

    return;
}
module_exit(knfc_exit);

MODULE_AUTHOR("Sony");
MODULE_DESCRIPTION("NFC cxd224x driver");
MODULE_LICENSE("GPL");
