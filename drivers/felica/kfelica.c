/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
* drivers/felica/kfelica.c  (kfelica driver)
*
* This software is contributed or developed by KYOCERA Corporation.
* (C) 2012 KYOCERA Corporation
* (C) 2013 KYOCERA Corporation
* (C) 2014 KYOCERA Corporation
* (C) 2015 KYOCERA Corporation
* (C) 2016 KYOCERA Corporation
* (C) 2017 KYOCERA Corporation
*
*   This program is free software; you can redistribute it and/or
*   modify it under the terms of the GNU General Public License
*   as published by the Free Software Foundation; only version 2.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program; if not, write to the Free Software
*   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/

/***************header***************/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/unistd.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/major.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/i2c-dev.h>
#include <linux/platform_device.h>
//#include <linux/mfd/pmic8058.h>
#include <linux/serial_core.h>
#include <linux/sem.h>
#include <linux/spinlock.h>
#include <linux/termios.h>

#include <asm/uaccess.h>
#include <asm/current.h>
//#include <asm/mach/mmc.h>

//#include <mach/irqs-8974.h>
#include <linux/of_gpio.h>

//#include <mach/msm_smem.h>

//extern bool is_vbus_active(void);

#if 1
#define ENABLED_FELICA_VFC_RST_DEV
#endif
/************************************************************************/
/*	Macro																*/
/************************************************************************/
/* CEN device		*/
#define MFD_LOCK_MASK					0x01
#define I2C_FAILURE						-1
#define I2C_SUCCESS						0
#define CEN_MAGIC						0xFC
#define I2C_WRITE_COMMAND				_IOWR(CEN_MAGIC, 6, struct ioctl_cmd)
#define SET_I2C_READ_COMMAND			_IOWR(CEN_MAGIC, 7, struct ioctl_cmd)
#define CEN_DEV_COUNT					1
#define CEN_DEV_NAME					"felica_cen"
#define CEN_IC_NAME						"S_7780A"
/* INTU device	*/
#define INTU_POLL_DEV_COUNT				1
#define INTU_POLL_DEV_NAME				"intu_poll"
#define INTU_POLL_DELAY					3
#define INTU_POLL_DEV_LOW				0
#define INTU_POLL_DEV_HIGH				1
/* AVAILABLE device		*/
#define AVAILABLE_POLL_DEV_COUNT		1
#define AVAILABLE_POLL_DEV_NAME			"available_poll"
#define AVAILABLE_POLL_DEV_MAGIC		0xFC
#define AVAILABLE_POLL_DEV_SET_PRE_LOCK	_IO(AVAILABLE_POLL_DEV_MAGIC, 8)
/* FELICA device				*/
#define FELICA_DEV_COUNT				1
#define FELICA_DEV_NAME					"felica"
#define FELICA_DEV_SIZE_MASK			0x00FF
/* FELICA POLL device			*/
#define FELICADEV_POLL_DEV_COUNT		1
#define FELICADEV_POLL_DEV_NAME			"felicadev_poll"
/* PON	device			*/
#define I2C_BUS_NUMBER					1
#define I2C_MFD_SLAVE_ADDR				(0x80 >> 1)
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)	(pm_gpio + NR_GPIO_IRQS)
#define PON_DEV_COUNT					1
#define PON_DEV_NAME					"felica_pon"
#define PON_DEV_LOW						0
#define PON_DEV_HIGH					1
/* FELICA_VFC_RST device */
#ifdef ENABLED_FELICA_VFC_RST_DEV
#define FELICA_VFC_RST_DEV_COUNT			1
#define FELICA_VFC_RST_DEV_NAME			"vfc_rst"
#define FELICA_VFC_RST_DEV_LOW				0
#define FELICA_VFC_RST_DEV_HIGH			1
#endif

/* RATE device			*/
#define RATE_MAGIC						0xFC
#define RATE_REQ_RATE_115				_IO(RATE_MAGIC, 1)
#define RATE_REQ_RATE_230				_IO(RATE_MAGIC, 2)
#define RATE_REQ_RATE_460				_IO(RATE_MAGIC, 3)
#define RATE_DEV_COUNT					1
#define RATE_DEV_NAME					"felica_rate"
/* RWS	device			*/
#define RWS_DEV_COUNT					1
#define RWS_DEV_NAME					"felica_rws"
/* RFS	device			*/
#define RFS_DEV_COUNT					1
#define RFS_DEV_NAME					"felica_rfs"
#define	RFS_GPIO_VAL_H					1
#define RFS_GPIO_VAL_L					0
#define RFS_RET_STS_LOW					1
#define RFS_RET_STS_HIGH				0
/* INT POLL device		*/
#define INT_POLL_DEV_COUNT				1
#define INT_POLL_DEV_NAME				"int_poll"
#define INT_POLL_DELAY					3
#define INT_POLL_DEV_LOW				0
#define INT_POLL_DEV_HIGH				1
#define INT_POLL_RET_STS_LOW			0
#define INT_POLL_RET_STS_HIGH			1

/* RFS POLL device		*/
#define RFS_POLL_DEV_COUNT				1
#define RFS_POLL_DEV_NAME				"rfs_poll"
#define RFS_POLL_RETRY_CNT				3
#define RFS_POLL_SLEEP					1
#define RFS_POLL_DELAY					3
#define RFS_POLL_DEV_LOW				0
#define RFS_POLL_DEV_HIGH				1
#define RFS_POLL_RET_STS_LOW			0
#define RFS_POLL_RET_STS_HIGH			1

/* USB device	*/
#define VBUS_DEV_COUNT					1
#define VBUS_DEV_NAME					"kfelicavbus"

#define PM_FELICA_NAME						"msm8909-felica-driver"

/* UartExclusion	device			*/
#define UE_DEV_COUNT					1
#define UE_DEV_NAME					"felica_ue"

/* ioctl				*/
struct ioctl_cmd
{
	unsigned int val;
	unsigned int debug;
	unsigned int debug2;
};
struct cen_data{
	struct input_dev *input_dev;
};

struct icc_poll_data {
	wait_queue_head_t read_wait;
	wait_queue_head_t rsp_wait;
	wait_queue_head_t dummy_wait;

	int handler_done;
	int rsp_done;
	struct delayed_work work;
	int device_status;
	int read_error;
	int open_flag;
	int	available_flag;
};
struct poll_data {
	wait_queue_head_t read_wait;
	int irq_handler_done;
	struct delayed_work work;
	int device_status;
	int read_error;
	int open_flag;
};

/* Status */
#define STATUS_MAGIC			0xFC
#define STATUS_0				_IO(STATUS_MAGIC, 10)
#define STATUS_1				_IO(STATUS_MAGIC, 11)
#define STATUS_2				_IO(STATUS_MAGIC, 12)
#define STATUS_3				_IO(STATUS_MAGIC, 13)
#define STATUS_4				_IO(STATUS_MAGIC, 14)
#define STATUS_DEV_COUNT					1
#define STATUS_DEV_NAME					"felica_status"

/************************************************************************/
/*	global																*/
/************************************************************************/
static struct class*		kfelica_class = NULL;
static struct cdev			rfs_cdev;
static struct cdev			pon_cdev;
#ifdef ENABLED_FELICA_VFC_RST_DEV
static struct cdev			felica_vfc_rst_cdev;
#endif
static struct cdev			cen_cdev;
static struct cdev			int_poll_cdev;
static struct cdev			rfs_poll_cdev;
static struct cdev			kfelicavbus_cdev;
static char					g_readCmd = 0xFF;
static unsigned int			g_unrate = 0;
static unsigned int			g_unbaud = 0;
static struct cdev			rate_cdev;
static struct cdev			rws_cdev;
static char					g_rws_data = -1;
static struct semaphore		chardev_sem;
static struct cdev			available_poll_cdev;
static struct cdev			felicadev_cdev;
static struct cdev			felicadev_poll_cdev;
static struct i2c_client*	this_client;
static struct icc_poll_data	g_felicadev_data;
static struct icc_poll_data	g_felicadev_poll_data;
static struct icc_poll_data	g_available_data;	
static int					g_flcsts		= 0;
static int					g_startkind	= 255;
static struct poll_data		g_int_data;
static struct poll_data*	g_int_d = &g_int_data;
static struct poll_data		g_rfs_data;
static struct poll_data*	g_rfs_d = &g_rfs_data;
static char					g_functype = 0;
static short				g_sdatasize = 0;
static char					g_carbuf[4101];	
static int					g_felicadev_open_cnt = 0;
static struct cdev			ue_cdev;
/* available 	*/
static int	g_cen_sts = 0;
static int	g_rfs_sts = 0;
static int	g_read_value = 0;

static spinlock_t rfs_spin_lock;

static int					g_felica_rfs_gpio = -1;
static int					g_felica_int_gpio = -1;

static int					g_felica_pon_gpio = -1;

#ifdef ENABLED_FELICA_VFC_RST_DEV
static int					g_felica_vfc_gpio = -1;
#endif

/* Status */
static char					g_status = 0x00;
static struct cdev			status_cdev;

/* UART Exclusion */
static char g_ue_data = 0x00;
static char g_felica_open = 0x00;

/* DEBUG_LOG */
#if 0
#define DEBUG_KFELICA_DRIVER
#endif

#ifdef DEBUG_KFELICA_DRIVER
#define KFELICA_LOG_D(fmt, args...) printk(KERN_INFO "[KFELICA][%s]" fmt"\n", __func__, ## args)
#else
#define KFELICA_LOG_D(fmt, args...)
#endif

/* ERROR_LOG */
#define KFELICA_LOG_E(fmt, args...) printk(KERN_ERR "[KFELICA][%s]ERR " fmt"\n", __func__, ## args)


/*
 * prototype
*/
static __init int	kfelica_init(void);
static void __exit	kfelica_exit(void);

static ssize_t rws_read(struct file *file, char __user * buf, size_t len, loff_t * ppos);
static ssize_t rws_write(struct file *file, const char __user *data, size_t len, loff_t *ppos);
static int rws_open(struct inode *inode, struct file *file);
static int rws_release(struct inode *inode, struct file *file);
static int rws_init(void);
static void rws_exit(void);
static ssize_t rate_read(struct file *file, char __user * buf, size_t len, loff_t * ppos);
static long rate_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static int rate_open(struct inode *inode, struct file *file);
static int rate_release(struct inode *inode, struct file *file);
static int rate_init(void);
static void rate_exit(void);
static ssize_t rfs_read(struct file *file, char __user * buf, size_t len, loff_t * ppos);
static ssize_t rfs_write(struct file *file, const char __user *data, size_t len, loff_t *ppos);
static int rfs_open(struct inode *inode, struct file *file);
static int rfs_release(struct inode *inode, struct file *file);
static int rfs_init(void);
static void rfs_exit(void);
static ssize_t pon_read(struct file *file, char __user * buf, size_t len, loff_t *ppos);
static ssize_t pon_write(struct file *file, const char __user *data, size_t len, loff_t *ppos);
static int pon_open(struct inode *inode, struct file *file);
static int pon_release(struct inode *inode, struct file *file);
static int pon_init(void);
static void pon_exit(void);
#ifdef ENABLED_FELICA_VFC_RST_DEV
static ssize_t felica_vfc_rst_write(struct file *file, const char __user *data, size_t len, loff_t *ppos);
static int felica_vfc_rst_open(struct inode *inode, struct file *file);
static int felica_vfc_rst_release(struct inode *inode, struct file *file);
static int felica_vfc_rst_init(void);
static void felica_vfc_rst_exit(void);
#endif
static irqreturn_t int_poll_irq_handler(int irq, void *dev_id);
static void int_poll_work_func(struct work_struct *work);
static unsigned int int_poll_poll(struct file *file, poll_table *wait);
static ssize_t int_poll_read(struct file *file, char __user * buf, size_t len, loff_t * ppos);
static int int_poll_open(struct inode *inode, struct file *file);
static int int_poll_release(struct inode *inode, struct file *file);
static int int_poll_init(void);
static void int_poll_exit(void);

static irqreturn_t rfs_poll_irq_handler(int irq, void *dev_id);
static void rfs_poll_work_func(struct work_struct *work);
static unsigned int rfs_poll_poll(struct file *file, poll_table *wait);
static ssize_t rfs_poll_read(struct file *file, char __user * buf, size_t len, loff_t * ppos);
static int rfs_poll_open(struct inode *inode, struct file *file);
static int rfs_poll_release(struct inode *inode, struct file *file);
static int rfs_poll_init(void);
static void rfs_poll_exit(void);

static int cen_open( struct inode *inode, struct file *filp );
static int cen_release( struct inode *inode, struct file *filp );
static ssize_t cen_read(struct file *file, char __user *buf,size_t count, loff_t *pos);
static long cen_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static int cen_remove(struct i2c_client *client);
static long available_poll_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static long available_poll_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static ssize_t available_poll_read(struct file *file, char __user * buf, size_t len, loff_t * ppos);
static int available_poll_open(struct inode *inode, struct file *file);
static int available_poll_release(struct inode *inode, struct file *file);
static int available_poll_init(void);
static void available_poll_exit(void);
static void felicadev_poll_kick( void );
static int felicadev_chk_conflict(void);
static ssize_t felicadev_read(struct file *file, char __user * buf, size_t len, loff_t * ppos);
static ssize_t felicadev_write(struct file *file, const char __user *data, size_t len, loff_t *ppos);
static long felicadev_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static int felicadev_open(struct inode *inode, struct file *file);
static int felicadev_release(struct inode *inode, struct file *file);

static int ue_init(void);
static int ue_open(struct inode *inode, struct file *file);
static ssize_t ue_write(struct file *file, const char __user *data, size_t len, loff_t *ppos);
static ssize_t ue_read(struct file *file, char __user * buf, size_t len, loff_t * ppos);
static int ue_release(struct inode *inode, struct file *file);
static void ue_exit(void);

static int felicadev_fsync(struct file *file, loff_t start, loff_t end, int datasync);

static int felicadev_init(void);
static void felicadev_exit(void);
static void felicadev_poll_kick( void );
static ssize_t felicadev_poll_read(struct file *file, char __user * buf, size_t len, loff_t * ppos);
static ssize_t felicadev_poll_write(struct file *file, const char __user *data, size_t len, loff_t *ppos);
static unsigned int felicadev_poll_poll(struct file *file, poll_table *wait);
static int felicadev_poll_open(struct inode *inode, struct file *file);
static int felicadev_poll_release(struct inode *inode, struct file *file);
static int felicadev_poll_init(void);
static void felicadev_poll_exit(void);

static ssize_t kfelicavbus_read(struct file *file, char __user * buf, size_t len, loff_t * ppos);
static int kfelicavbus_open(struct inode *inode, struct file *file);
static int kfelicavbus_release(struct inode *inode, struct file *file);


static struct platform_driver msm8909_felica_driver;
static int pm_felica_probe(struct platform_device *pdev);
static int pm_felica_remove(struct platform_device *pdev);
static int pm_felica_suspend (struct platform_device *pdev, pm_message_t state);
static int pm_felica_resume (struct platform_device *pdev);

static ssize_t status_read(struct file *file, char __user * buf, size_t len, loff_t * ppos);
static long status_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static int status_open(struct inode *inode, struct file *file);
static int status_release(struct inode *inode, struct file *file);
static int status_init(void);
static void status_exit(void);

static const struct file_operations kfelicavbus_fileops = {
	.owner   = THIS_MODULE,
	.read    = kfelicavbus_read,
	.open    = kfelicavbus_open,
	.release = kfelicavbus_release,
};
static const struct file_operations rws_fileops = {
	.owner   = THIS_MODULE,
	.read    = rws_read,
	.write   = rws_write,
	.open    = rws_open,
	.release = rws_release,
};
static const struct file_operations rate_fileops = {
	.owner   = THIS_MODULE,
	.read    = rate_read,
	.unlocked_ioctl   = rate_ioctl,
	.open    = rate_open,
	.release = rate_release,
};
static const struct file_operations rfs_fileops = {
	.owner   = THIS_MODULE,
	.read    = rfs_read,
	.write    = rfs_write,
	.open    = rfs_open,
	.release = rfs_release,
};
static const struct file_operations pon_fileops = {
	.owner   = THIS_MODULE,
	.read    = pon_read,
	.write   = pon_write,
	.open    = pon_open,
	.release = pon_release,
};
#ifdef ENABLED_FELICA_VFC_RST_DEV
static const struct file_operations felica_vfc_rst_fileops = {
	.owner   = THIS_MODULE,
	.write   = felica_vfc_rst_write,
	.open    = felica_vfc_rst_open,
	.release = felica_vfc_rst_release,
};
#endif
static const struct file_operations int_poll_fileops = {
	.owner   = THIS_MODULE,
	.read    = int_poll_read,
	.open    = int_poll_open,
	.release = int_poll_release,
	.poll    = int_poll_poll,
};
static const struct file_operations rfs_poll_fileops = {
	.owner   = THIS_MODULE,
	.read    = rfs_poll_read,
	.open    = rfs_poll_open,
	.release = rfs_poll_release,
	.poll    = rfs_poll_poll,
};
static struct file_operations cen_fileops = {
	.owner	 = THIS_MODULE,
	.open	 = cen_open,
	.release = cen_release,
	.read	 = cen_read,
	.unlocked_ioctl	 = cen_ioctl, 
};
static const struct file_operations available_poll_fileops = {
	.owner   = THIS_MODULE,
	.unlocked_ioctl   = available_poll_ioctl,
	.compat_ioctl = available_poll_compat_ioctl,
	.read    = available_poll_read,
	.open    = available_poll_open,
	.release = available_poll_release,
};
static const struct file_operations felicadev_fileops = {
	.owner   = THIS_MODULE,
	.read    = felicadev_read,
	.write   = felicadev_write,
	.unlocked_ioctl   = felicadev_ioctl,
	.open    = felicadev_open,
	.release = felicadev_release,
	.fsync   = felicadev_fsync,
};
static const struct file_operations felicadev_poll_fileops = {
	.owner   = THIS_MODULE,
	.read    = felicadev_poll_read,
	.write   = felicadev_poll_write,
	.open    = felicadev_poll_open,
	.release = felicadev_poll_release,
	.poll    = felicadev_poll_poll,
};

static const struct file_operations status_fileops = {
	.owner   = THIS_MODULE,
	.read    = status_read,
	.unlocked_ioctl   = status_ioctl,
	.open    = status_open,
	.release = status_release,
};
static const struct file_operations ue_fileops = {
	.owner   = THIS_MODULE,
	.read    = ue_read,
	.write   = ue_write,
	.open    = ue_open,
	.release = ue_release,
};

static const struct of_device_id msm8909_felica_of_match[] = {
	{ .compatible = "kc,msm8909-felica-driver", },
	{},
};

static struct platform_driver msm8909_felica_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= PM_FELICA_NAME,
		.of_match_table = msm8909_felica_of_match,
	},
	.probe	 = pm_felica_probe,
	.remove	 = pm_felica_remove,
	.suspend	= pm_felica_suspend,
	.resume		= pm_felica_resume

};


/* rws device	*/
static ssize_t rws_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
	char rws_val;
	
	KFELICA_LOG_D("START");

	if ( NULL == buf ) {
		KFELICA_LOG_E("rws_read param err");
		return -EIO;
	}

	if ( 1 > len ) {
		KFELICA_LOG_E("length check len = %d", (int)len);
		return -EIO;
	}

	rws_val = g_rws_data;

	if (copy_to_user(buf, &rws_val, 1)) {
		KFELICA_LOG_E("copy_to_user");
		return -EFAULT;
	}

	KFELICA_LOG_D("END rws_val = %d, len = %d", rws_val, (int)len);

	return 1;
}

static ssize_t rws_write(struct file *file, const char __user *data, size_t len, loff_t *ppos)
{
	char rws_val;

	KFELICA_LOG_D("START");

	if( NULL == data ){
		KFELICA_LOG_E("rws_write param err");
		return -EIO;
	}

	if ( 1 > len ) {
		KFELICA_LOG_E("length check len = %d", (int)len);
		return -EIO;
	}
	
	if (copy_from_user(&rws_val, data, 1)) {
		KFELICA_LOG_E("copy_from_user");
		return -EFAULT;
	}

	g_rws_data = rws_val;

	KFELICA_LOG_D("END rws_val = %d, g_rws_data = %d", rws_val, g_rws_data);

	return 1;
}
static int rws_open(struct inode *inode, struct file *file)
{
	KFELICA_LOG_D("START");
	KFELICA_LOG_D("END");
	return 0;
}

static int rws_release(struct inode *inode, struct file *file)
{
	KFELICA_LOG_D("START");
	KFELICA_LOG_D("END");
	return 0;
}

static int rws_init(void)
{
	int				sdResult = 0;
	struct device*	class_dev;

	dev_t dev = MKDEV(MISC_MAJOR, 0);

	KFELICA_LOG_D("START");

	sdResult = alloc_chrdev_region(&dev , 0 , RWS_DEV_COUNT, RWS_DEV_NAME);
	if( 0 != sdResult ) {
		KFELICA_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&rws_cdev, &rws_fileops);
	rws_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&rws_cdev, dev, RWS_DEV_COUNT);
	if( 0 != sdResult) {
		unregister_chrdev_region(dev, RWS_DEV_COUNT);
		KFELICA_LOG_E("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}

	class_dev = device_create(kfelica_class, NULL, dev, NULL, RWS_DEV_NAME);
	if ( 0 != IS_ERR(class_dev)) {
		cdev_del(&rws_cdev);
		unregister_chrdev_region(dev, RWS_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KFELICA_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}
	g_rws_data = 0;
	KFELICA_LOG_D("END");

	return sdResult;
}

static void rws_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KFELICA_LOG_D("START");

	device_destroy(kfelica_class, dev);

	cdev_del(&rws_cdev);
	unregister_chrdev_region(dev, RWS_DEV_COUNT);

	KFELICA_LOG_D("END");
}

static ssize_t rate_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
	int rate_val;
	
	KFELICA_LOG_D("START");
	
	if( 4 > len ){
		KFELICA_LOG_E("length check len = %d", (int)len);
		return -EIO;
	}

	rate_val = g_unrate;
	if (copy_to_user(buf, &rate_val, 4)) {
		KFELICA_LOG_E("copy_to_user");
		return -EFAULT;
	}

	KFELICA_LOG_D("END rate_val = %d, len = %d", rate_val, (int)len);

	return 4;
}
static long rate_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long				nRet	= 0;

	KFELICA_LOG_D("START");
	KFELICA_LOG_D("cmd = %x\n",cmd);
	switch( cmd ){
		case RATE_REQ_RATE_115:
			g_unrate = 115200;
			g_unbaud = B115200;
			break;
		case RATE_REQ_RATE_230:
			g_unrate = 230400;
			g_unbaud = B230400;
			break;
		case RATE_REQ_RATE_460:
			g_unrate = 460800;
			g_unbaud = B460800;
			break;
		default:
			nRet = -EIO;
			break;
	}
	KFELICA_LOG_D("END");
	return nRet;
}

static int rate_open(struct inode *inode, struct file *file)
{
	KFELICA_LOG_D("START");
	KFELICA_LOG_D("END");
	return 0;
}

static int rate_release(struct inode *inode, struct file *file)
{
	KFELICA_LOG_D("START");
	KFELICA_LOG_D("END");
	return 0;
}

static int rate_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KFELICA_LOG_D("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , RATE_DEV_COUNT, RATE_DEV_NAME);
	if (sdResult) {
		KFELICA_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&rate_cdev, &rate_fileops);
	rate_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&rate_cdev, dev, RATE_DEV_COUNT);
	if (sdResult) {
		unregister_chrdev_region(dev, RATE_DEV_COUNT);
		KFELICA_LOG_E("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}
	
	class_dev = device_create(kfelica_class, NULL, dev, NULL, RATE_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&rate_cdev);
		unregister_chrdev_region(dev, RATE_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KFELICA_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}
	
	KFELICA_LOG_D("END");
	
	return sdResult;
}

static void rate_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KFELICA_LOG_D("START");

	device_destroy(kfelica_class, dev);

	cdev_del(&rate_cdev);
	unregister_chrdev_region(dev, RATE_DEV_COUNT);

	KFELICA_LOG_D("END");
}

/*
 * function_rfs
 */
static ssize_t rfs_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
	int ret;
	char on;
	
	KFELICA_LOG_D("START");
	KFELICA_LOG_D("g_felica_pon_gpio [%d]",g_felica_pon_gpio);
	KFELICA_LOG_D("g_felica_rfs_gpio [%d]",g_felica_rfs_gpio);
	KFELICA_LOG_D("g_felica_int_gpio [%d]",g_felica_int_gpio);
	
	if ( len < 1 ) {
		KFELICA_LOG_E("length check len = %d", (int)len);
		return -EIO;
	}

	ret = gpio_get_value_cansleep( g_felica_rfs_gpio );
	if (ret < 0) {
		KFELICA_LOG_E("gpio_get_value ret = %d", ret);
		return ret;
	}
	if( RFS_GPIO_VAL_H == ret ){
		on = RFS_RET_STS_HIGH;
	}else{
		on = RFS_RET_STS_LOW;
	}
	len = 1;

	if (copy_to_user(buf, &on, len)) {
		KFELICA_LOG_E("copy_to_user");
		return -EFAULT;
	}
	
	KFELICA_LOG_D("END on = %d, len = %d", on, (int)len);

	return len;
}

static ssize_t rfs_write(struct file *file, const char __user *data, size_t len, loff_t *ppos)
{
#if 0
	unsigned char *smem;

    KFELICA_LOG_D("START");

    smem = (unsigned char *)kc_smem_alloc(SMEM_felica_RFS_SIG_STATE, 1);
    if (smem == NULL) {
        KFELICA_LOG_E("smem_alloc");
        return -EFAULT;
    }

#ifdef DEBUG_KFELICA_DRIVER
    KFELICA_LOG_E("rfs=%d", *data);
    KFELICA_LOG_E("old_smem=%d", *smem);
#endif
    if (copy_from_user(smem, data, 1)) {
        KFELICA_LOG_E("copy_from_user");
        return -EFAULT;
    }
#ifdef DEBUG_KFELICA_DRIVER
    KFELICA_LOG_E("new_smem=%d", *smem);
#endif

    KFELICA_LOG_D("END");
#endif
    return len;
}

static int rfs_open(struct inode *inode, struct file *file)
{
	KFELICA_LOG_D("");
	return 0;
}

static int rfs_release(struct inode *inode, struct file *file)
{
	KFELICA_LOG_D("");
	return 0;
}



static int rfs_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KFELICA_LOG_D("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , RFS_DEV_COUNT, RFS_DEV_NAME);
	if (sdResult) {
		KFELICA_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&rfs_cdev, &rfs_fileops);
	rfs_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&rfs_cdev, dev, RFS_DEV_COUNT);
	if (sdResult) {
		unregister_chrdev_region(dev, RFS_DEV_COUNT);
		KFELICA_LOG_E("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}
	
	class_dev = device_create(kfelica_class, NULL, dev, NULL, RFS_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&rfs_cdev);
		unregister_chrdev_region(dev, RFS_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KFELICA_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}
	
	KFELICA_LOG_D("END");
	
	return sdResult;
}

static void rfs_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KFELICA_LOG_D("START");
	
	cdev_del(&rfs_cdev);
	unregister_chrdev_region(dev, RFS_DEV_COUNT);
	
	KFELICA_LOG_D("END");
}


/*
 * function_pon
 */
static ssize_t pon_read(struct file *file, char __user * buf, size_t len, loff_t *ppos)
{
	char on;
	int  ret = 0;
	

	KFELICA_LOG_D("START");
	
	if ( NULL == buf ) {
		KFELICA_LOG_E("pon_read param err");
		return -EIO;
	}

	if ( 1 > len ) {
		KFELICA_LOG_E("length check len = %d", (int)len);
		return -EIO;
	}

	ret = gpio_get_value_cansleep( g_felica_pon_gpio );
	if( ret < 0 ) {
		KFELICA_LOG_E("gpio_get_value ret = %d", ret);
		return ret;
	}
	if( PON_DEV_HIGH == ret ){
		on = PON_DEV_HIGH;
	}else{
		on = PON_DEV_LOW;
	}
	len = 1;

	if( copy_to_user(buf, &on, len) ) {
		KFELICA_LOG_E("copy_to_user");
		return -EFAULT;
	}

	KFELICA_LOG_D("END on = %d, len = %d", on, (int)len);

	return len;
}
static ssize_t pon_write(struct file *file, const char __user *data, size_t len, loff_t *ppos)
{
	char on;
	int seton;
	
	KFELICA_LOG_D("START");
	
	if ( len < 1 ) {
		KFELICA_LOG_E("length check len = %d", (int)len);
		return -EIO;
	}
	
	if (copy_from_user(&on, data, 1)) {
		KFELICA_LOG_E("copy_from_user");
		return -EFAULT;
	}

	if (on == PON_DEV_HIGH){
		seton = PON_DEV_HIGH;
		KFELICA_LOG_D("pon high.\n");
	}else if (on == PON_DEV_LOW){
		KFELICA_LOG_D("pon low.\n");
		seton = PON_DEV_LOW;
	}else {
		KFELICA_LOG_D("pon err value = %x \n",on );
		return -EFAULT;
	}

	gpio_set_value(g_felica_pon_gpio , seton );

	KFELICA_LOG_D("END on = %d, seton = %d", on, seton);
	
	return len;
}
static int pon_open(struct inode *inode, struct file *file)
{
	int				nRet = 0;

	KFELICA_LOG_D("START");

	KFELICA_LOG_D("END");
	return nRet;
}

static int pon_release(struct inode *inode, struct file *file)
{
	KFELICA_LOG_D("START");

	gpio_set_value(g_felica_pon_gpio , PON_DEV_LOW);

	KFELICA_LOG_D("END");

	return 0;
}



static int pon_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KFELICA_LOG_D("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , PON_DEV_COUNT, PON_DEV_NAME);
	if (sdResult) {
		KFELICA_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&pon_cdev, &pon_fileops);
	pon_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&pon_cdev, dev, PON_DEV_COUNT);
	if (sdResult) {
		unregister_chrdev_region(dev, PON_DEV_COUNT);
		KFELICA_LOG_E("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}
	
	class_dev = device_create(kfelica_class, NULL, dev, NULL, PON_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&pon_cdev);
		unregister_chrdev_region(dev, PON_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KFELICA_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}

	KFELICA_LOG_D("END");
	
	return sdResult;
}

static void pon_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KFELICA_LOG_D("START");

	device_destroy(kfelica_class, dev);

	cdev_del(&pon_cdev);
	unregister_chrdev_region(dev, PON_DEV_COUNT);

	KFELICA_LOG_D("END");
}


/*
 * function_felica_vfc_rst
 */
#ifdef ENABLED_FELICA_VFC_RST_DEV
static ssize_t felica_vfc_rst_write(struct file *file, const char __user *data, size_t len, loff_t *ppos)
{
	char on;
	int seton;
	
	KFELICA_LOG_D("START");
	
	if ( len < 1 ) {
		KFELICA_LOG_E("length check len = %d", (int)len);
		return -EIO;
	}
	
	if (copy_from_user(&on, data, 1)) {
		KFELICA_LOG_E("copy_from_user");
		return -EFAULT;
	}

	if (on == FELICA_VFC_RST_DEV_HIGH){
		seton = FELICA_VFC_RST_DEV_HIGH;
		KFELICA_LOG_D("felica_vfc_rst high.\n");
	}else if (on == FELICA_VFC_RST_DEV_LOW){
		KFELICA_LOG_D("felica_vfc_rst low.\n");
		seton = FELICA_VFC_RST_DEV_LOW;
	}else {
		KFELICA_LOG_D("felica_vfc_rst err value = %x \n",on );
		return -EFAULT;
	}

	gpio_set_value(g_felica_vfc_gpio , seton );

	KFELICA_LOG_D("END on = %d, seton = %d", on, seton);
	
	return len;
}
static int felica_vfc_rst_open(struct inode *inode, struct file *file)
{
	int				nRet = 0;

	KFELICA_LOG_D("START");

	KFELICA_LOG_D("END");
	return nRet;
}

static int felica_vfc_rst_release(struct inode *inode, struct file *file)
{
	KFELICA_LOG_D("START");

	gpio_set_value(g_felica_vfc_gpio , FELICA_VFC_RST_DEV_LOW);

	KFELICA_LOG_D("END");

	return 0;
}


static int felica_vfc_rst_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KFELICA_LOG_D("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , FELICA_VFC_RST_DEV_COUNT, FELICA_VFC_RST_DEV_NAME);
	if (sdResult) {
		KFELICA_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&felica_vfc_rst_cdev, &felica_vfc_rst_fileops);
	felica_vfc_rst_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&felica_vfc_rst_cdev, dev, FELICA_VFC_RST_DEV_COUNT);
	if (sdResult) {
		unregister_chrdev_region(dev, FELICA_VFC_RST_DEV_COUNT);
		KFELICA_LOG_E("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}
	
	class_dev = device_create(kfelica_class, NULL, dev, NULL, FELICA_VFC_RST_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&felica_vfc_rst_cdev);
		unregister_chrdev_region(dev, FELICA_VFC_RST_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KFELICA_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}

	KFELICA_LOG_D("END");
	
	return sdResult;
}

static void felica_vfc_rst_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KFELICA_LOG_D("START");

	device_destroy(kfelica_class, dev);

	cdev_del(&felica_vfc_rst_cdev);
	unregister_chrdev_region(dev, FELICA_VFC_RST_DEV_COUNT);

	KFELICA_LOG_D("END");
}
#endif

/*
 * function_int_poll
 */
static void int_poll_work_func(struct work_struct *work)
{
	struct poll_data *int_d				= g_int_d;
	int				read_value			= 0;
	int				old_value			= 0;
	unsigned long	irqflag				= 0;
	
	KFELICA_LOG_D("START");
	old_value = int_d->device_status;
	read_value = gpio_get_value_cansleep(g_felica_int_gpio);

	KFELICA_LOG_D("read_value = %d old_value = %d", read_value, old_value);
	
	if (read_value < 0) {
		int_d->read_error = read_value;
	} else if (read_value != old_value) {
		int_d->device_status = read_value;
		int_d->read_error = 0;
		
		if (int_d->device_status == INT_POLL_DEV_LOW){
			irqflag = IRQF_TRIGGER_HIGH | IRQF_SHARED;
		}else{
			irqflag = IRQF_TRIGGER_LOW | IRQF_SHARED;
		}
		if (irq_set_irq_type(gpio_to_irq(g_felica_int_gpio), irqflag)){
			KFELICA_LOG_E("set_irq_type irqflag = %ld", irqflag);
		}
	}
	
	enable_irq(gpio_to_irq(g_felica_int_gpio));
	
	if (read_value != old_value || int_d->read_error) {
		int_d->irq_handler_done = 1;
		wake_up_interruptible(&int_d->read_wait);
	}
	
	KFELICA_LOG_D("END read_value = %d, old_value = %d, int_d->read_error = %d"
					, read_value, old_value, int_d->read_error);
}

static irqreturn_t int_poll_irq_handler(int irq, void *dev_id)
{
	struct poll_data *int_d = g_int_d;
	
	KFELICA_LOG_D("START irq = %d", irq);
	
	disable_irq_nosync(gpio_to_irq(g_felica_int_gpio));

	schedule_delayed_work(&int_d->work, msecs_to_jiffies(INT_POLL_DELAY));
	
	KFELICA_LOG_D("END");
	
	return IRQ_HANDLED;
}

static unsigned int int_poll_poll(struct file *file, poll_table *wait)
{
	struct poll_data *int_d = g_int_d;
	unsigned int mask = 0;
	
	KFELICA_LOG_D("START");
	
	poll_wait(file, &int_d->read_wait, wait);
	if (int_d->irq_handler_done){
		mask = POLLIN | POLLRDNORM;
	}
	KFELICA_LOG_D("END mask = %d", mask);
	
	return (mask);
}

static ssize_t int_poll_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
	struct poll_data *int_d = g_int_d;
	int ret;
	char cret;
	
	KFELICA_LOG_D("START");
	
	if ( len < 1 ) {
		KFELICA_LOG_E("length check len = %d", (int)len);
		return -EIO;
	}
	
	if (!int_d->irq_handler_done) {
		if (file->f_flags & O_NONBLOCK) {
			KFELICA_LOG_E("NONBLOCK");
			return -EAGAIN;
		}
		KFELICA_LOG_D("FeliCa int_poll wait irq");
		ret = wait_event_interruptible(int_d->read_wait, int_d->irq_handler_done == 1);
		if (-ERESTARTSYS == ret) {
			KFELICA_LOG_D("wait_event_interruptible ret = %d", ret);
			return -EINTR;
		}
	}
	
	if (int_d->read_error) {
		int_d->irq_handler_done = 0;
		int_d->read_error = 0;
		KFELICA_LOG_E("int_d->read_error = %d", int_d->read_error);
		return -EIO;
	}
	
	if (int_d->device_status == INT_POLL_DEV_HIGH){
		cret = INT_POLL_RET_STS_HIGH;
	}else{
		cret = INT_POLL_RET_STS_LOW;
	}

	len = 1;

	if (copy_to_user(buf, &cret, len)) {
		KFELICA_LOG_E("copy_to_user");
		return -EFAULT;
	}
	int_d->irq_handler_done = 0;
	
	KFELICA_LOG_D("END len = %d, cret = %d", (int)len, cret);
	
	return len;
}

static int int_poll_open(struct inode *inode, struct file *file)
{
	struct poll_data *int_d = g_int_d;
	unsigned long irqflag = 0;
	int ret = 0;
	
	KFELICA_LOG_D("START");
	
	if (int_d->open_flag) {
		KFELICA_LOG_E("only one time");
		return -EBUSY;
	}
	int_d->open_flag = 1;
	
	ret = gpio_get_value_cansleep(g_felica_int_gpio);
	if (ret < 0) {
		int_d->open_flag = 0;
		KFELICA_LOG_E("gpio_get_value ret = %d", ret);
		return -EIO;
	}
	int_d->device_status = ret;
	
	if (int_d->device_status == INT_POLL_DEV_LOW){
		irqflag = IRQF_TRIGGER_HIGH | IRQF_SHARED;
	}else{
		irqflag = IRQF_TRIGGER_LOW | IRQF_SHARED;
	}

	if (request_any_context_irq(gpio_to_irq(g_felica_int_gpio), int_poll_irq_handler, irqflag, INT_POLL_DEV_NAME, (void*)int_d)) {
		int_d->open_flag = 0;
		KFELICA_LOG_E("request_irq irqflag = %ld", irqflag);
		return -EIO;
	}
	
	if (enable_irq_wake(gpio_to_irq(g_felica_int_gpio))){
		
		KFELICA_LOG_E("enable_irq_wake");
		
		free_irq(gpio_to_irq(g_felica_int_gpio), (void *)int_d);
		
		return -EIO;
	}

	int_d->irq_handler_done = 0;
	
	KFELICA_LOG_D("END");
	
	return 0;
}

static int int_poll_release(struct inode *inode, struct file *file)
{
	struct poll_data *int_d = g_int_d;

	KFELICA_LOG_D("START");
	
	cancel_delayed_work(&int_d->work);

	if (disable_irq_wake(gpio_to_irq(g_felica_int_gpio))){
		KFELICA_LOG_E("disable_irq_wake");
	}
	free_irq(gpio_to_irq(g_felica_int_gpio), (void *)int_d);
	
	int_d->open_flag = 0;

	KFELICA_LOG_D("END");
	
	return 0;
}


static int int_poll_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KFELICA_LOG_D("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , INT_POLL_DEV_COUNT, INT_POLL_DEV_NAME);
	if (sdResult) {
		KFELICA_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&int_poll_cdev, &int_poll_fileops);
	int_poll_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&int_poll_cdev, dev, INT_POLL_DEV_COUNT);
	if (sdResult) {
		unregister_chrdev_region(dev, INT_POLL_DEV_COUNT);
		KFELICA_LOG_E("cdev_add sdResult = %d",sdResult);
		return sdResult;
	}
	
	class_dev = device_create(kfelica_class, NULL, dev, NULL, INT_POLL_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&int_poll_cdev);
		unregister_chrdev_region(dev, INT_POLL_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KFELICA_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}
	
	memset(g_int_d, 0x00, sizeof(struct poll_data));

	INIT_DELAYED_WORK(&g_int_d->work, int_poll_work_func);

	init_waitqueue_head(&g_int_d->read_wait);
	
	g_int_d->open_flag = 0;
	
	KFELICA_LOG_D("END");
	
	return sdResult;
}

static void int_poll_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	KFELICA_LOG_D("START");
	
	cdev_del(&int_poll_cdev);
	unregister_chrdev_region(dev, INT_POLL_DEV_COUNT);
	KFELICA_LOG_D("END");
}
/* rfs poll func 
	to available_read()
*/
static void rfs_poll_work_func(struct work_struct *work)
{
	struct poll_data *rfs_d = g_rfs_d;
	int old_value	= 0;
	int local_value	= 0;
	int gpio_read_value	= 0;
	unsigned long irqflag = 0;
	unsigned long spin_lock_irqflag = 0;
	struct icc_poll_data* available_d = &g_available_data;
	
	KFELICA_LOG_D("START");
	
	old_value = rfs_d->device_status;

	spin_lock_irqsave( &rfs_spin_lock, spin_lock_irqflag);

	KFELICA_LOG_D("g_read_value = %d old_value = %d", g_read_value, old_value);
	
	if (g_read_value < 0) {
		local_value = rfs_d->read_error = g_read_value;
		spin_unlock_irqrestore( &rfs_spin_lock, spin_lock_irqflag);
		KFELICA_LOG_E("gpio_get_value err g_read_value = %d", g_read_value);
	}else {
		local_value = g_read_value;
		spin_unlock_irqrestore( &rfs_spin_lock, spin_lock_irqflag);

		gpio_read_value = gpio_get_value_cansleep(g_felica_rfs_gpio);

		if(gpio_read_value < 0){
			rfs_d->read_error = gpio_read_value;
			KFELICA_LOG_E("gpio_get_value_cansleep err gpio_read_value = %d", gpio_read_value);
		}else{
			g_rfs_sts = gpio_read_value;
			rfs_d->device_status = gpio_read_value;
			rfs_d->read_error = 0;

			if (rfs_d->device_status == RFS_POLL_DEV_LOW){
				irqflag = IRQF_TRIGGER_HIGH | IRQF_SHARED;
			}else{
				irqflag = IRQF_TRIGGER_LOW | IRQF_SHARED;
			}
			if (irq_set_irq_type(gpio_to_irq(g_felica_rfs_gpio), irqflag)){
				KFELICA_LOG_E("set_irq_type irqflag = %ld", irqflag);
			}
		}

		if( 1 == available_d->available_flag ){
			if( 0 != g_cen_sts ){
				if( RFS_GPIO_VAL_L != g_rfs_sts ){
					if( 0 == g_flcsts ){
						KFELICA_LOG_D("wake up available");
						available_d->rsp_done = 1;
						wake_up_interruptible(&available_d->read_wait);
					}
				}
			}
		}
	}
	
	enable_irq(gpio_to_irq(g_felica_rfs_gpio));
	
	if (local_value != old_value || gpio_read_value != old_value || rfs_d->read_error) {
		rfs_d->irq_handler_done = 1;
		wake_up_interruptible(&rfs_d->read_wait);
	}

	KFELICA_LOG_D("END local_value = %d, old_value = %d, rfs_d->read_error = %d", local_value, old_value, rfs_d->read_error);
}

static irqreturn_t rfs_poll_irq_handler(int irq, void *dev_id)
{
	struct poll_data *rfs_d = g_rfs_d;

	KFELICA_LOG_D("START irq = %d", irq);

	disable_irq_nosync(gpio_to_irq(g_felica_rfs_gpio));

	spin_lock( &rfs_spin_lock );
	g_read_value = gpio_get_value(g_felica_rfs_gpio);
	spin_unlock( &rfs_spin_lock );

	schedule_delayed_work(&rfs_d->work, msecs_to_jiffies(RFS_POLL_DELAY));
	
	KFELICA_LOG_D("END");
	
	return IRQ_HANDLED;
}

static unsigned int rfs_poll_poll(struct file *file, poll_table *wait)
{
	struct poll_data *rfs_d = g_rfs_d;
	unsigned int mask = 0;
	
	KFELICA_LOG_D("START");
	
	poll_wait(file, &rfs_d->read_wait, wait);
	if (rfs_d->irq_handler_done){
		mask = POLLIN | POLLRDNORM;
	}
	KFELICA_LOG_D("END mask = %d", mask);
	
	return (mask);
}

static ssize_t rfs_poll_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
	struct poll_data *rfs_d = g_rfs_d;
	int ret;
	char cret;
	
	KFELICA_LOG_D("START");
	
	if ( len < 1 ) {
		KFELICA_LOG_E("length check len = %d", (int)len);
		return -EIO;
	}
	
	if (!rfs_d->irq_handler_done) {
		if (file->f_flags & O_NONBLOCK) {
			KFELICA_LOG_E("NONBLOCK");
			return -EAGAIN;
		}
		KFELICA_LOG_D("FeliCa rfs_poll wait irq");
		ret = wait_event_interruptible(rfs_d->read_wait, rfs_d->irq_handler_done == 1);
		if (-ERESTARTSYS == ret) {
			KFELICA_LOG_D("wait_event_interruptible ret = %d", ret);
			return -EINTR;
		}
	}
	
	if (rfs_d->read_error) {
		rfs_d->irq_handler_done = 0;
		rfs_d->read_error = 0;
		KFELICA_LOG_E("rfs_d->read_error = %d", rfs_d->read_error);
		return -EIO;
	}
	
	if (rfs_d->device_status == RFS_POLL_DEV_HIGH){
		cret = RFS_POLL_RET_STS_HIGH;
	}else{
		cret = RFS_POLL_RET_STS_LOW;
	}

	len = 1;

	if (copy_to_user(buf, &cret, len)) {
		KFELICA_LOG_E("copy_to_user");
		return -EFAULT;
	}
	rfs_d->irq_handler_done = 0;
	
	KFELICA_LOG_D("END len = %d, cret = %d", (int)len, cret);
	
	return len;
}

static int rfs_poll_open(struct inode *inode, struct file *file)
{
	struct poll_data *rfs_d = g_rfs_d;
	unsigned long irqflag = 0;
	int ret = 0;
	
	KFELICA_LOG_D("START");

	if (rfs_d->open_flag) {
		KFELICA_LOG_E("only one time");
		return -EBUSY;
	}
	rfs_d->open_flag = 1;
	
	ret = gpio_get_value_cansleep(g_felica_rfs_gpio);
	if (ret < 0) {
		rfs_d->open_flag = 0;
		KFELICA_LOG_E("gpio_get_value ret = %d", ret);
		return -EIO;
	}
	rfs_d->device_status = ret;
	g_rfs_sts = ret;

	if (rfs_d->device_status == RFS_POLL_DEV_LOW){
		irqflag = IRQF_TRIGGER_HIGH | IRQF_SHARED;
	}else{
		irqflag = IRQF_TRIGGER_LOW | IRQF_SHARED;
	}

	if (request_any_context_irq(gpio_to_irq(g_felica_rfs_gpio), rfs_poll_irq_handler, irqflag, RFS_POLL_DEV_NAME, (void*)rfs_d)) {
		rfs_d->open_flag = 0;
		KFELICA_LOG_E("request_irq irqflag = %ld", irqflag);
		return -EIO;
	}
	
	if (enable_irq_wake(gpio_to_irq(g_felica_rfs_gpio))){
		
		KFELICA_LOG_E("enable_irq_wake");
		
		free_irq(gpio_to_irq(g_felica_rfs_gpio), (void *)rfs_d);
		
		return -EIO;
	}

	rfs_d->irq_handler_done = 0;
	
	KFELICA_LOG_D("END");
	
	return 0;
}

static int rfs_poll_release(struct inode *inode, struct file *file)
{
	struct poll_data *rfs_d = g_rfs_d;

	KFELICA_LOG_D("START");
	
	cancel_delayed_work(&rfs_d->work);

	if (disable_irq_wake(gpio_to_irq(g_felica_rfs_gpio))){
		KFELICA_LOG_E("disable_irq_wake");
	}
	free_irq(gpio_to_irq(g_felica_rfs_gpio), (void *)rfs_d);

	rfs_d->open_flag = 0;
	
	KFELICA_LOG_D("END");
	
	return 0;
}


static int rfs_poll_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KFELICA_LOG_D("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , RFS_POLL_DEV_COUNT, RFS_POLL_DEV_NAME);
	if (sdResult) {
		KFELICA_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&rfs_poll_cdev, &rfs_poll_fileops);
	rfs_poll_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&rfs_poll_cdev, dev, RFS_POLL_DEV_COUNT);
	if (sdResult) {
		unregister_chrdev_region(dev, RFS_POLL_DEV_COUNT);
		KFELICA_LOG_E("cdev_add sdResult = %d",sdResult);
		return sdResult;
	}
	
	class_dev = device_create(kfelica_class, NULL, dev, NULL, RFS_POLL_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&rfs_poll_cdev);
		unregister_chrdev_region(dev, RFS_POLL_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KFELICA_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}
	
	memset(g_rfs_d, 0x00, sizeof(struct poll_data));

	spin_lock_init( &rfs_spin_lock );

	INIT_DELAYED_WORK(&g_rfs_d->work, rfs_poll_work_func);

	init_waitqueue_head(&g_rfs_d->read_wait);
	
	g_rfs_d->open_flag = 0;
	
	KFELICA_LOG_D("END");
	
	return sdResult;
}

static void rfs_poll_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	KFELICA_LOG_D("START");
	
	cdev_del(&rfs_poll_cdev);
	unregister_chrdev_region(dev, RFS_POLL_DEV_COUNT);
	KFELICA_LOG_D("END");
}



static int cen_open( struct inode *inode, struct file *filp )
{
	KFELICA_LOG_D("");
	return 0;
}

static int cen_release( struct inode *inode, struct file *filp )
{
	KFELICA_LOG_D("");
	return 0;
}

static ssize_t cen_read(struct file *file, char __user *buf,size_t count, loff_t *pos)
{
	int				i2c_ret				= -1;
	unsigned char	cmd					= g_readCmd;
	unsigned char	read_buff			= 0;
	int retry_cnt = 0;
	struct i2c_msg read_msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &cmd,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,	
			.len	= 1,
			.buf	= &read_buff,	
		},
	};

	KFELICA_LOG_D("cen_read start\n");
	KFELICA_LOG_D("cen_read cmd = %x\n", cmd);

	if (buf == NULL) {
		KFELICA_LOG_D("cen_read param err\n");
		return -EFAULT;
	}

	for( retry_cnt = 0; retry_cnt<2; retry_cnt++ ){
		i2c_ret  =i2c_transfer(this_client->adapter, read_msgs, 2);
		KFELICA_LOG_D("write_felica_cen ret = %d val = %d\n", i2c_ret,g_cen_sts);
		if (I2C_FAILURE >= i2c_ret) {
			usleep_range(10000,11000);
		} else {
			break;
		}
	}

	//KFELICA_LOG_D("cen_read i2c_ret = %d \n",i2c_ret);
	if (I2C_FAILURE >= i2c_ret) {
		return -EIO;
	}

	read_buff &= MFD_LOCK_MASK;

	if (copy_to_user(buf, &read_buff, sizeof(read_buff))) {
		KFELICA_LOG_D("cen_read copy_to_user err \n");
		return -EFAULT;
	}
	KFELICA_LOG_D("cen_read end\n");
	return (sizeof(read_buff));

}

static long cen_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	unsigned char	lock_status[2];	
	int				i2c_ret		= -1;
	int				retry_cnt	= 0;
	unsigned char	read_cmd	= 0;
	unsigned char	lock_temp	= 0;
	struct icc_poll_data* available_d = &g_available_data;
	struct i2c_msg write_msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 2,
		},
	};

	KFELICA_LOG_D("START");
	
	switch( cmd ){
		case SET_I2C_READ_COMMAND:
			KFELICA_LOG_D("START SET_I2C_READ_COMMAND");
			if (copy_from_user(&read_cmd, (void __user *)arg, sizeof(read_cmd))) {
				KFELICA_LOG_E("copy_from_user set_COMMAND = %x", read_cmd);
				return -EFAULT;
			}
			KFELICA_LOG_D("copy_from_user set_COMMAND = %x", read_cmd);
			if( 0xFF == g_readCmd ){
				g_readCmd = read_cmd;
				return 0;
			}else{
				return -1;
			}
			break;
		case I2C_WRITE_COMMAND:
			KFELICA_LOG_D("START I2C_WRITE_COMMAND");
			if (copy_from_user(&lock_status, (void __user *)arg, 2)) {
				KFELICA_LOG_E("copy_from_user write_E2PROM");
				return -EFAULT;
			}
			write_msg[0].buf = lock_status;
			lock_temp = g_cen_sts;
			g_cen_sts = lock_status[1];
			for( retry_cnt = 0; retry_cnt<2; retry_cnt++ ){
				i2c_ret = i2c_transfer(this_client->adapter, write_msg, 1);
				KFELICA_LOG_D("write_felica_cen ret = %d val = %d\n", i2c_ret,g_cen_sts);
				if( I2C_FAILURE < i2c_ret ){
					break;
				}else{
					usleep_range(10000,11000);
				}
			}

			if (I2C_FAILURE >= i2c_ret) {
				g_cen_sts = lock_temp;
				return -EIO;
			}else{
				KFELICA_LOG_D("write_felica_cen SUCCESS \n");
				if( 1 == available_d->available_flag ){
					if( 0 != g_cen_sts ){
						if( RFS_GPIO_VAL_L != g_rfs_sts ){
							if( 0 == g_flcsts ){
								available_d->rsp_done = 1;
								KFELICA_LOG_D("wake up available");
								wake_up_interruptible(&available_d->read_wait);
							}
						}
					}
				}
				return 1;
			}
		default:
			KFELICA_LOG_E("cmd = %d", cmd);
			return -EINVAL;
	}
	return 0;
}


static int cen_probe(struct i2c_client *client, const struct i2c_device_id * devid)
{
	struct cen_data *cen;
	int alloc_ret = 0;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KFELICA_LOG_D("START");
	
	cen = kzalloc(sizeof(struct cen_data), GFP_KERNEL);
	if (!cen) {
		KFELICA_LOG_E("kzalloc");
		return -ENOMEM;	
	}
	this_client = client;

	i2c_set_clientdata(client, cen);

	cen->input_dev = input_allocate_device();
	
	alloc_ret = alloc_chrdev_region(&dev , 0 , CEN_DEV_COUNT, CEN_DEV_NAME);
	if (alloc_ret) {
		KFELICA_LOG_E("alloc_chrdev_region");
		return alloc_ret;
	}
	
	cdev_init(&cen_cdev, &cen_fileops);
	cen_cdev.owner = THIS_MODULE;
	
	cdev_add(&cen_cdev, dev, CEN_DEV_COUNT);
	
	device_create(kfelica_class, NULL, dev, NULL, CEN_DEV_NAME);

	g_readCmd = 0xFF;

	KFELICA_LOG_D("END");
	
	return (0);
}

static int cen_remove(struct i2c_client *client)
{

	dev_t dev;

	struct cen_data *cen = i2c_get_clientdata(client);
	
	KFELICA_LOG_D("START");

	dev  = MKDEV(MISC_MAJOR, 0);

	device_destroy(kfelica_class, dev);

	cdev_del(&cen_cdev);

	unregister_chrdev_region(dev, 1);

	input_unregister_device(cen->input_dev);
	
	kfree(cen);
	
	KFELICA_LOG_D("END");
	
	return (0);
}

/********************/
/*available poll	*/
static long available_poll_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long			nRet	= -EIO;

	KFELICA_LOG_D("available_poll_ioctl START cmd = %x",cmd);
	switch(cmd){
		case AVAILABLE_POLL_DEV_SET_PRE_LOCK:
			g_cen_sts = 0;
			nRet = 0;
			break;
		default:
			break;
	}
	KFELICA_LOG_D("available_poll_ioctl END cmd = %x",cmd);
	return nRet;
}
static long available_poll_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long			nRet	= -EIO;

	KFELICA_LOG_D("available_poll_compat_ioctl START cmd = %x",cmd);
	switch(cmd){
		case AVAILABLE_POLL_DEV_SET_PRE_LOCK:
			g_cen_sts = 0;
			nRet = 0;
			break;
		default:
			break;
	}
	KFELICA_LOG_D("available_poll_compat_ioctl END cmd = %x",cmd);
	return nRet;
}

static ssize_t available_poll_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
	char	type = 0;
	int		nRet = -1;
	struct icc_poll_data* available_d = &g_available_data;

	KFELICA_LOG_D("available poll read START");

	if( ( 0 != g_flcsts )	||
		( 0 == g_cen_sts )	||
		( RFS_GPIO_VAL_L == g_rfs_sts )){

		KFELICA_LOG_D(" g_flcsts = %d, g_cen_sts = %d, g_rfs_sts = %d", g_flcsts, g_cen_sts, g_rfs_sts);

		available_d->available_flag = 1;
		nRet = wait_event_interruptible(available_d->read_wait,available_d->rsp_done == 1);
		if( nRet < 0 ){
			KFELICA_LOG_E("available_poll_read() wait_event_interruptible ERROR = %d ",nRet);
			return -EIO;
		}
		type = 0x01;
		available_d->rsp_done		= 0;
		available_d->available_flag	= 0;
	}else{
		type = 0x01;
	}

	if (copy_to_user(buf, &type, len)) {
		KFELICA_LOG_E("copy_to_user");
		return -EFAULT;
	}

	KFELICA_LOG_D("available poll read END read size = %d ", (int)len);

	return len;
}

static int available_poll_open(struct inode *inode, struct file *file)
{
	KFELICA_LOG_D("available poll open START");
	
	KFELICA_LOG_D("available poll open END");
	return 0;
}

static int available_poll_release(struct inode *inode, struct file *file)
{
	KFELICA_LOG_D("available poll close START");
	if( g_available_data.available_flag == 1 ){
		g_available_data.rsp_done = 1;
		wake_up_interruptible(&g_available_data.read_wait);
	}
	g_available_data.rsp_done = 0;
	g_available_data.available_flag = 0;
	KFELICA_LOG_D("available poll close END");
	return 0;
}

static int available_poll_init(void)
{
	int           sdResult      = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KFELICA_LOG_D("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , AVAILABLE_POLL_DEV_COUNT, AVAILABLE_POLL_DEV_NAME);
	if (sdResult) {
		KFELICA_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&available_poll_cdev, &available_poll_fileops);
	available_poll_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&available_poll_cdev, dev, AVAILABLE_POLL_DEV_COUNT);
	if (sdResult) {
		unregister_chrdev_region(dev, AVAILABLE_POLL_DEV_COUNT);
		KFELICA_LOG_E("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}

	class_dev = device_create(kfelica_class, NULL, dev, NULL, AVAILABLE_POLL_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&available_poll_cdev);
		unregister_chrdev_region(dev, AVAILABLE_POLL_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KFELICA_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}

	memset((void*)&g_available_data, 0x00, sizeof(struct icc_poll_data));

	init_waitqueue_head(&g_available_data.read_wait);

	KFELICA_LOG_D("END");

	return sdResult;
}

static void available_poll_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KFELICA_LOG_D("START");

	device_destroy(kfelica_class, dev);

	cdev_del(&available_poll_cdev);
	unregister_chrdev_region(dev, AVAILABLE_POLL_DEV_COUNT);

	KFELICA_LOG_D("END");
}

/********************/
/* felica	*/

static int felicadev_chk_conflict(void)
{
	return 0;
}

static ssize_t felicadev_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
	short slen = 0;
	int		nRet = -1;
	struct icc_poll_data *felica_d = &g_felicadev_data;

	KFELICA_LOG_D("felicadev read START len = %d",(int)len);
	nRet = down_interruptible( &chardev_sem );
	if( 0 > nRet ){
		return -ERESTARTSYS;
	}
	g_functype = 1;
	g_sdatasize = len;

	g_carbuf[0] = (char)g_functype;
	g_carbuf[1] = 0xFA;
	g_carbuf[2] = ((g_sdatasize >> 8) & FELICA_DEV_SIZE_MASK);
	g_carbuf[3] = (g_sdatasize & FELICA_DEV_SIZE_MASK);

	felicadev_poll_kick();
	
	nRet = wait_event_interruptible(felica_d->rsp_wait,felica_d->rsp_done == 1);
	if( nRet < 0 ){
		KFELICA_LOG_E("felicadev_ioctl() wait_event_interruptible ERROR = %d ",nRet);
		up( &chardev_sem );
		return -EIO;
	}
	felica_d->rsp_done = 0;

	slen = g_carbuf[2];
	slen = slen << 8;
	slen = slen + g_carbuf[3];
	if( len != slen ){
		if( len < slen ){
			KFELICA_LOG_E("size unmatch len = %d slen = %d",(int)len,slen);
			up( &chardev_sem );
			return -EFAULT;
		}
	}
	if (copy_to_user(buf, &g_carbuf[4], slen)){
		KFELICA_LOG_E("copy_to_user");
		up( &chardev_sem );
		return -EFAULT;
	}

	g_functype = -1;
	up( &chardev_sem );
	KFELICA_LOG_D("felicadev read END param size = %d ret size = %d ", (int)len, slen);

	return slen;
}
static ssize_t felicadev_write(struct file *file, const char __user *data, size_t len, loff_t *ppos)
{
	int		nRet = -1;
	struct icc_poll_data *felica_d = &g_felicadev_data;

	KFELICA_LOG_D("felicadev write START");
	nRet = down_interruptible( &chardev_sem );
	if( 0 > nRet ){
		return -ERESTARTSYS;
	}
	g_functype = 2;
	g_sdatasize = len;

	g_carbuf[0] = (char)g_functype;
	g_carbuf[2] = ((g_sdatasize >> 8) & FELICA_DEV_SIZE_MASK);
	g_carbuf[3] = (g_sdatasize & FELICA_DEV_SIZE_MASK);
	if (copy_from_user(&g_carbuf[4], data, len)) {
		KFELICA_LOG_E("copy_from_user");
		up( &chardev_sem );
		return -EFAULT;
	}

	felicadev_poll_kick();

	nRet = wait_event_interruptible(felica_d->rsp_wait,felica_d->rsp_done == 1);
	if( nRet < 0 ){
		KFELICA_LOG_E("felicadev_ioctl() wait_event_interruptible ERROR = %d ",nRet);
		up( &chardev_sem );
		return -EIO;
	}
	felica_d->rsp_done = 0;

	nRet = g_carbuf[2];
	nRet = nRet << 8;
	nRet = nRet + g_carbuf[3];

	g_functype = -1;
	up( &chardev_sem );
	KFELICA_LOG_D("felicadev write END retsize = %d paramsize = %d",nRet,(int)len);
	return nRet;
}
static long felicadev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long			nRet	= 0;
	short			sstructsize = 0;
	int				nSize	= 0;
	struct icc_poll_data *felica_d = &g_felicadev_data;

	KFELICA_LOG_D("felicadev ioctl START cmd = %x",cmd);
	nRet = down_interruptible( &chardev_sem );
	if( 0 > nRet ){
		KFELICA_LOG_E("felicadev_ioctl down_interruptible ERROR cmd = %d",cmd);
		return -ERESTARTSYS;
	}
	switch(cmd){
		case TCGETS:
			KFELICA_LOG_E("felicadev_ioctl un use cmd = %d",cmd);
			nRet = -EIO;
			break;
		case TCSANOW:
		case TCSETS:
			g_functype = 4;
			g_carbuf[0] = (char)g_functype;

			sstructsize = sizeof(struct termios);
			KFELICA_LOG_D("felicadev ioctl termios size = %d",sstructsize);

			g_carbuf[2] = (( sstructsize >> 8 )& FELICA_DEV_SIZE_MASK);
			g_carbuf[3] = (sstructsize & FELICA_DEV_SIZE_MASK);

			if (copy_from_user(&g_carbuf[4], (void __user *)arg, sizeof(struct termios))) {
				KFELICA_LOG_E("copy_from_user");
				up( &chardev_sem );
				return -EFAULT;
			}

			felicadev_poll_kick();
			nRet = wait_event_interruptible(felica_d->rsp_wait,felica_d->rsp_done == 1);
			if( nRet < 0 ){
				KFELICA_LOG_E("felicadev_ioctl() wait_event_interruptible ERROR = %d ",(int)nRet);
				up( &chardev_sem );
				return -EIO;
			}
			felica_d->rsp_done = 0;
			break;
		case FIONREAD:
			g_functype = 5;
			g_carbuf[0] = (char)g_functype;
			felicadev_poll_kick();

			nRet = wait_event_interruptible(felica_d->rsp_wait,felica_d->rsp_done == 1);
			if( nRet < 0 ){
				KFELICA_LOG_E("felicadev_ioctl() wait_event_interruptible ERROR = %d ",(int)nRet);
				up( &chardev_sem );
				return -EIO;
			}
			felica_d->rsp_done = 0;
			if( g_carbuf[1] == 0 ){
				nRet = 0;
				nSize = g_carbuf[2];
				nSize = (nSize << 8);
				nSize = nSize + g_carbuf[3];
				if (copy_to_user((int __user*)arg, &nSize, 4)){
					KFELICA_LOG_E("copy_to_user");
					nRet = -EFAULT;
				}
			}else{
				nRet = -EIO;
			}
			KFELICA_LOG_D("felicadev ioctl END ret = %d cmd = %x avail = %d",(int)nRet, cmd, nSize);
			break;
		default:
			KFELICA_LOG_E("felicadev ioctl un use cmd = %d",cmd);
			nRet = -EIO;
			break;
	}
	g_functype = -1;
	up( &chardev_sem );

	return nRet;
}

static int felicadev_open(struct inode *inode, struct file *file)
{
	struct icc_poll_data *felica_d = &g_felicadev_data;
	int				nRet	= 0;

	nRet = down_interruptible( &chardev_sem );
	if( 0 > nRet ){
		return -ERESTARTSYS;
	}
	KFELICA_LOG_D("felicadev open START");

	nRet = felicadev_chk_conflict();

	if( g_ue_data != 0x00 ){
		up( &chardev_sem );
		KFELICA_LOG_D("felicadev open fail. UART alredy used. \n");
		return -EIO;
	}
	g_felica_open = 0x01;

	if( 0 == nRet ){
		if( 0 == g_felicadev_open_cnt ){
			g_flcsts = 1;
			switch( g_startkind ){
				case 255:
					/* idle status */
					g_flcsts++;
					break;
				case 1:
					/* poll		*/
				{
					up( &chardev_sem );
					nRet = wait_event_interruptible(felica_d->rsp_wait,felica_d->rsp_done == 1);
					if( nRet < 0 ){
						g_flcsts = 0;
						KFELICA_LOG_E("felicadev_open() wait_event_interruptible ERROR = %d ",nRet);
						return -EIO;
					}
					nRet = down_interruptible( &chardev_sem );
					if( 0 > nRet ){
					KFELICA_LOG_E("felicadev_open() down_interruptible ERR ");
						return -ERESTARTSYS;
					}
					felica_d->rsp_done = 0;
					g_flcsts = 2;
				}
					break;
				default:
					nRet = -EIO;
					g_flcsts = 0;
					KFELICA_LOG_E("felicadev_open() unknown kind = %d",g_startkind);
					break;
			}
		}
		if( 0 == nRet ){
			g_functype = 0;
			g_carbuf[0] = (char)g_functype;

			felicadev_poll_kick();

			nRet = wait_event_interruptible(felica_d->rsp_wait,felica_d->rsp_done == 1);
			if( nRet < 0 ){
				up( &chardev_sem );
				KFELICA_LOG_E("felicadev_open() wait_event_interruptible ERROR = %d ",nRet);
				return -EIO;
			}
			felica_d->rsp_done = 0;
			if( 0x00 != g_carbuf[1] ){
				g_functype = -1;
				up( &chardev_sem );
				KFELICA_LOG_E("felicadev_open() 0x00 != g_carbuf[1] rsp = %d ",g_carbuf[1]);
				return -EIO;
			}
		}else{
			g_functype = -1;
			up( &chardev_sem );
			return nRet;
		}
		g_felicadev_open_cnt++;
	}
	g_functype = -1;
	up( &chardev_sem );
	KFELICA_LOG_D("felicadev open SUCCESS END = %d",g_felicadev_open_cnt);
	return 0;
}

static int felicadev_release(struct inode *inode, struct file *file)
{
	struct icc_poll_data* available_d = &g_available_data;
	struct icc_poll_data *felica_d = &g_felicadev_data;
	int		nRet = -1;

	nRet = down_interruptible( &chardev_sem );
	if( 0 > nRet ){
		KFELICA_LOG_E("felicadev_close() down_interruptible ERROR");
		return -ERESTARTSYS;
	}
	KFELICA_LOG_D("felicadev close START");

	if( 0 < g_felicadev_open_cnt ){
		g_felicadev_open_cnt--;
		g_functype = 3;
		g_carbuf[0] = (char)g_functype;

		felicadev_poll_kick();

		nRet = wait_event_interruptible(felica_d->rsp_wait,felica_d->rsp_done == 1);
		if( nRet < 0 ){
			KFELICA_LOG_E("felicadev_close() wait_event_interruptible ERROR = %d ",nRet);

			/* 1 time retry */
			usleep_range(10000,11000); /* 10ms */

			nRet = wait_event_interruptible(felica_d->rsp_wait,felica_d->rsp_done == 1);
			if( nRet < 0 ){
				KFELICA_LOG_E("felicadev_close() wait_event_interruptible retry ERROR = %d ",nRet);
			up( &chardev_sem );
			return -EIO;
		}
		}
		felica_d->rsp_done = 0;
		/*  all close only */
		if( 0 == g_felicadev_open_cnt ){
			g_flcsts = 0;
			if( 1 == available_d->available_flag ){
				if( 0 != g_cen_sts ){
					if( RFS_GPIO_VAL_L != g_rfs_sts ){
						available_d->rsp_done = 1;
					KFELICA_LOG_D("wake up available");
						wake_up_interruptible(&available_d->read_wait);
					}
				}
			}
		}
	}else{
		KFELICA_LOG_E("felicadev close cnt ERROR = %d",g_felicadev_open_cnt);
		up( &chardev_sem );
		return -EIO;
	}
	g_functype = -1;
	up( &chardev_sem );
	KFELICA_LOG_D("felicadev close SUCCESS END = %d available = %d ",g_felicadev_open_cnt,available_d->available_flag);

	g_felica_open = 0x00;

	return 0;
}

static int felicadev_fsync(struct file *file, loff_t start, loff_t end, int datasync)
{
	KFELICA_LOG_D("felicadev fsync START");

	KFELICA_LOG_D("felicadev fsync END");
	return 0;
}

static int felicadev_init(void)
{
	int				sdResult	= 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KFELICA_LOG_D("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , FELICA_DEV_COUNT, FELICA_DEV_NAME);
	if (sdResult) {
		KFELICA_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&felicadev_cdev, &felicadev_fileops);
	felicadev_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&felicadev_cdev, dev, FELICA_DEV_COUNT);
	if (sdResult) {
		unregister_chrdev_region(dev, FELICA_DEV_COUNT);
		KFELICA_LOG_E("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}

	class_dev = device_create(kfelica_class, NULL, dev, NULL, FELICA_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&felicadev_cdev);
		unregister_chrdev_region(dev, FELICA_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KFELICA_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}

	memset((void*)&g_felicadev_data, 0x00, sizeof(struct icc_poll_data));

	sema_init( &chardev_sem, 1 );

	init_waitqueue_head(&g_felicadev_data.rsp_wait);
	
	g_felicadev_data.open_flag = 0;
	g_felicadev_data.rsp_done = 0;

	KFELICA_LOG_D("END");

	return sdResult;
}

static void felicadev_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KFELICA_LOG_D("START");

	device_destroy(kfelica_class, dev);

	cdev_del(&felicadev_cdev);
	unregister_chrdev_region(dev, FELICA_DEV_COUNT);

	KFELICA_LOG_D("END");
}
static void felicadev_poll_kick( void )
{
	struct icc_poll_data *felicadev_poll_d = &g_felicadev_poll_data;

	felicadev_poll_d->handler_done = 1;
	wake_up_interruptible(&felicadev_poll_d->read_wait);

}
static ssize_t felicadev_poll_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
	struct icc_poll_data *felicadev_poll_d = &g_felicadev_poll_data;

	int		ret;

	if (0 == felicadev_poll_d->handler_done) {
		if (file->f_flags & O_NONBLOCK) {
			KFELICA_LOG_E("NONBLOCK");
			return -EAGAIN;
		}
		KFELICA_LOG_D("felicadev_poll_read wait data!");
		ret = wait_event_interruptible(felicadev_poll_d->read_wait, felicadev_poll_d->handler_done == 1);
		if (-ERESTARTSYS == ret) {
			KFELICA_LOG_D("wait_event_interruptible ret = %d", ret);
			return -EINTR;
		}
	}

	switch( g_functype ){
		case 0:
			len = 1;
			break;
		case 1:
			len = 4;
			break;
		case 2:
			len = g_carbuf[2];
			len = len << 8;
			len = len + g_carbuf[3];
			len = len +4;
			break;
		case 3:
			len = 1;
			break;
		case 4:
			len = sizeof(struct termios) + 4;
			break;
		case 5:
			len = 1;
			break;
		default:
			KFELICA_LOG_E("felicadev_poll_read UNKNOWN functype");
			felicadev_poll_d->handler_done = 0;
			return -EFAULT;
	}
	if (copy_to_user(buf, g_carbuf, len)){
		KFELICA_LOG_E("copy_to_user");
		return -EFAULT;
	}
	felicadev_poll_d->handler_done = 0;

	return len;
}
static ssize_t felicadev_poll_write(struct file *file, const char __user *data, size_t len, loff_t *ppos)
{
	struct icc_poll_data *felica_d = &g_felicadev_data;

	g_carbuf[0] = 0x00;
	g_carbuf[1] = 0x00;
	g_carbuf[2] = 0x00;
	g_carbuf[3] = 0x00;

	if (copy_from_user(g_carbuf, data, len)) {
		KFELICA_LOG_E("copy_from_user");
		return -EFAULT;
	}

	felica_d->rsp_done = 1;
	wake_up_interruptible(&felica_d->rsp_wait);

	return 0;
}
static unsigned int felicadev_poll_poll(struct file *file, poll_table *wait)
{
	struct icc_poll_data *felicadev_poll_d = &g_felicadev_poll_data;
	unsigned int mask = 0;
	
	poll_wait(file, &felicadev_poll_d->read_wait, wait);
	if (0 != felicadev_poll_d->handler_done){
		mask = POLLIN | POLLRDNORM;
	}
	
	return (mask);
}

static int felicadev_poll_open(struct inode *inode, struct file *file)
{
	struct icc_poll_data *felicadev_poll_d = &g_felicadev_poll_data;
	KFELICA_LOG_D("felicadev_poll_open START");

	felicadev_poll_d->handler_done = 0;
	felicadev_poll_d->rsp_done = 0;

	KFELICA_LOG_D("felicadev_poll_open END");
	return 0;
}

static int felicadev_poll_release(struct inode *inode, struct file *file)
{
	KFELICA_LOG_D("felicadev_poll_close START");

	KFELICA_LOG_D("felicadev_poll_close END");
	return 0;
}


static int felicadev_poll_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KFELICA_LOG_D("felicadev_poll_init() START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , FELICADEV_POLL_DEV_COUNT, FELICADEV_POLL_DEV_NAME);
	if (sdResult) {
		KFELICA_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&felicadev_poll_cdev, &felicadev_poll_fileops);
	felicadev_poll_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&felicadev_poll_cdev, dev, FELICADEV_POLL_DEV_COUNT);
	if (sdResult) {
		unregister_chrdev_region(dev, FELICADEV_POLL_DEV_COUNT);
		KFELICA_LOG_E("cdev_add sdResult = %d",sdResult);
		return sdResult;
	}
	
	class_dev = device_create(kfelica_class, NULL, dev, NULL, FELICADEV_POLL_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&felicadev_poll_cdev);
		unregister_chrdev_region(dev, FELICADEV_POLL_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KFELICA_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}
	
	memset((void*)&g_felicadev_poll_data, 0x00, sizeof(struct icc_poll_data));

	init_waitqueue_head(&g_felicadev_poll_data.read_wait);

	init_waitqueue_head(&g_felicadev_poll_data.rsp_wait);

	g_felicadev_poll_data.open_flag = 0;

	g_felicadev_poll_data.handler_done = 0;
	g_felicadev_poll_data.rsp_done = 0;

	KFELICA_LOG_D("felicadev_poll_init() END");
	
	return sdResult;
}

static void felicadev_poll_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	KFELICA_LOG_D("felicadev_poll_exit() START");
	
	cdev_del(&felicadev_poll_cdev);
	unregister_chrdev_region(dev, FELICADEV_POLL_DEV_COUNT);
	KFELICA_LOG_D("END");
}

static ssize_t kfelicavbus_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
	bool vbus_val = 0;

	KFELICA_LOG_D("START");
	
	if( 1 > len ){
		KFELICA_LOG_E("length check len = %d", (int)len);
		return -EIO;
	}
	//vbus_val = is_vbus_active();
	vbus_val = 0;

	if (copy_to_user(buf, &vbus_val, 1)) {
		KFELICA_LOG_E("copy_to_user");
		return -EFAULT;
	}

	KFELICA_LOG_D("END rate_val = %d, len = %d", (int)vbus_val, (int)len);

	return 1;
}
static int kfelicavbus_open(struct inode *inode, struct file *file)
{
	KFELICA_LOG_D("START");
	KFELICA_LOG_D("END");
	return 0;
}

static int kfelicavbus_release(struct inode *inode, struct file *file)
{
	KFELICA_LOG_D("START");
	KFELICA_LOG_D("END");
	return 0;
}

static int kfelicavbus_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KFELICA_LOG_D("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , VBUS_DEV_COUNT, VBUS_DEV_NAME);
	if (sdResult) {
		KFELICA_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&kfelicavbus_cdev, &kfelicavbus_fileops);
	kfelicavbus_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&kfelicavbus_cdev, dev, VBUS_DEV_COUNT);
	if (sdResult) {
		unregister_chrdev_region(dev, VBUS_DEV_COUNT);
		KFELICA_LOG_E("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}
	
	class_dev = device_create(kfelica_class, NULL, dev, NULL, VBUS_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&kfelicavbus_cdev);
		unregister_chrdev_region(dev, VBUS_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KFELICA_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}
	
	KFELICA_LOG_D("END");
	
	return sdResult;
}

static void kfelicavbus_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KFELICA_LOG_D("START");

	device_destroy(kfelica_class, dev);

	cdev_del(&kfelicavbus_cdev);
	unregister_chrdev_region(dev, VBUS_DEV_COUNT);

	KFELICA_LOG_D("END");
}
static ssize_t status_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
	KFELICA_LOG_D("START");
	
	if( 1 > len ){
		KFELICA_LOG_E("length check len = %d", (int)len);
		return -EIO;
	}

	if (copy_to_user(buf, &g_status, 1)) {
		KFELICA_LOG_E("copy_to_user");
		return -EFAULT;
	}

	KFELICA_LOG_D("END status = %d, len = %d", g_status, (int)len);

	return 1;
}
static long status_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long				nRet	= 0;

	KFELICA_LOG_D("START");
	KFELICA_LOG_D("cmd = %x\n",cmd);
	switch( cmd ){
		case STATUS_0:
		KFELICA_LOG_D("STATUS_0 = %x\n",STATUS_0);
			g_status = 0;
			break;
		case STATUS_1:
		KFELICA_LOG_D("STATUS_1 = %x\n",STATUS_1);
			g_status = 1;
			break;
		case STATUS_2:
		KFELICA_LOG_D("STATUS_2 = %x\n",STATUS_2);
			g_status = 2;
			break;
		case STATUS_3:
		KFELICA_LOG_D("STATUS_3 = %x\n",STATUS_3);
			g_status = 3;
			break;
		case STATUS_4:
		KFELICA_LOG_D("STATUS_4 = %x\n",STATUS_4);
			g_status = 4;
			break;
		default:
			nRet = -EIO;
			break;
	}
	KFELICA_LOG_D("END status = %d", g_status);
	KFELICA_LOG_D("END");
	return nRet;
}

static int status_open(struct inode *inode, struct file *file)
{
	KFELICA_LOG_D("START");
	KFELICA_LOG_D("END");
	return 0;
}

static int status_release(struct inode *inode, struct file *file)
{
	KFELICA_LOG_D("START");
	KFELICA_LOG_D("END");
	return 0;
}

static int status_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KFELICA_LOG_D("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , STATUS_DEV_COUNT, STATUS_DEV_NAME);
	if (sdResult) {
		KFELICA_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&status_cdev, &status_fileops);
	status_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&status_cdev, dev, STATUS_DEV_COUNT);
	if (sdResult) {
		unregister_chrdev_region(dev, STATUS_DEV_COUNT);
		KFELICA_LOG_E("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}
	
	class_dev = device_create(kfelica_class, NULL, dev, NULL, STATUS_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&status_cdev);
		unregister_chrdev_region(dev, STATUS_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KFELICA_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}
	
	KFELICA_LOG_D("END");
	
	return sdResult;
}

static void status_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KFELICA_LOG_D("START");

	device_destroy(kfelica_class, dev);

	cdev_del(&status_cdev);
	unregister_chrdev_region(dev, STATUS_DEV_COUNT);

	KFELICA_LOG_D("END");
}

#ifdef CONFIG_OF
static struct of_device_id i2c_felica_table[] = {
	{ .compatible = "kc,felica_i2c"},
	{ },
};
#else
#define i2c_felica_table NULL
#endif
static const struct i2c_device_id cen_id[] = {
	{ "felica_cen", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, cen_id);

static struct i2c_driver s7780a_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= CEN_DEV_NAME,
		.of_match_table = i2c_felica_table,
	},
	.probe	 = cen_probe,
	.id_table = cen_id,
	.remove	 = cen_remove,
};

/*
 * kfelica_init
 */
static __init int kfelica_init(void)
{
	int ret;

	KFELICA_LOG_D("START");
	kfelica_class = class_create(THIS_MODULE, "kfelica");
	
	if (IS_ERR(kfelica_class)) {
		return PTR_ERR(kfelica_class);
	}
	
	ret = platform_driver_register(&msm8909_felica_driver);
	if( ret < 0 ){
		KFELICA_LOG_E("platform_driver_register err\n");
		return ret;
	}

	ret = rate_init();
	if( ret < 0 ){
		KFELICA_LOG_E("rate init err\n");
		return ret;
	}

	ret = pon_init();
	if( ret < 0 ){
		KFELICA_LOG_E("pon init err\n");
		return ret;
	}

#ifdef ENABLED_FELICA_VFC_RST_DEV
	ret = felica_vfc_rst_init();
	if( ret < 0 ){
		KFELICA_LOG_E("felica_vfc_rst init err\n");
		return ret;
	}
#endif

	ret = rfs_init();
	if( ret < 0 ){
		KFELICA_LOG_E("rfs init err\n");
		return ret;
	}

	ret = i2c_add_driver(&s7780a_driver);
	if( ret < 0 ){
		KFELICA_LOG_E("i2c add driver err\n");
		return ret;
	}

	ret = int_poll_init();
	if( ret < 0 ){
		KFELICA_LOG_E("int_poll_init err = %d\n",ret);
		return ret;
	}
	ret = rws_init();
	if( ret < 0 ){
		KFELICA_LOG_E("rws_init err = %d\n",ret);
		return ret;
	}

	ret = available_poll_init();
	if( ret < 0 ){
		KFELICA_LOG_E("available_poll_init err = %d\n",ret);
		return ret;
	}
	
	ret = felicadev_init();
	if( ret < 0 ){
		KFELICA_LOG_E("felicadev_init err = %d\n",ret);
		return ret;
	}
	
	ret = felicadev_poll_init();
	if( ret < 0 ){
		KFELICA_LOG_E("felicadev_poll_init err = %d\n",ret);
		return ret;
	}

	ret = rfs_poll_init();
	if( ret < 0 ){
		KFELICA_LOG_E("rfs_poll_init err = %d\n",ret);
		return ret;
	}

	ret = kfelicavbus_init();
	if( ret < 0 ){
		KFELICA_LOG_E("kfelicavbus_init err = %d\n",ret);
	}

	ret = status_init();
	if( ret < 0 ){
		KFELICA_LOG_E("status_init err = %d\n",ret);
	}

	ret = ue_init();
	if( ret < 0 ){
		KFELICA_LOG_E("ue_inir err = %d\n",ret);
	}


	KFELICA_LOG_D("END");
	return 0;
}

/*
 * kfelica_exit
 */
static void __exit kfelica_exit(void)
{

	class_destroy( kfelica_class );

	platform_driver_unregister(&msm8909_felica_driver);

	rate_exit();

	pon_exit();

#ifdef ENABLED_FELICA_VFC_RST_DEV
	felica_vfc_rst_exit();
#endif

	rfs_exit();

	i2c_del_driver(&s7780a_driver);

	int_poll_exit();

	rws_exit();

	available_poll_exit();

	felicadev_exit();

	felicadev_poll_exit();

	rfs_poll_exit();

	kfelicavbus_exit();

	status_exit();

	ue_exit();

	return;
}


static int pm_felica_probe(struct platform_device *pdev)
{

	int rc = 0;

	KFELICA_LOG_D("START");

	/* PON GPIO No Get */
	g_felica_pon_gpio = of_get_named_gpio(pdev->dev.of_node, "felica_pon-gpio", 0);
	if (g_felica_pon_gpio < 0) {
		pr_err("%s: of_get_named_gpio failed.\n", __func__);
		return -EINVAL;
	}

	rc = gpio_request(g_felica_pon_gpio, "felica_pon-gpio");
	if (rc) {
		pr_err("%s: gpio_request failed.\n", __func__);
		return -EINVAL;
	}

#ifdef ENABLED_FELICA_VFC_RST_DEV
	g_felica_vfc_gpio = of_get_named_gpio(pdev->dev.of_node, "felica_vfc-gpio", 0);
	if (g_felica_vfc_gpio < 0) {
		pr_err("%s: of_get_named_gpio failed.\n", __func__);
		return -EINVAL;
	}

	rc = gpio_request(g_felica_vfc_gpio, "felica_vfc-gpio");
	if (rc) {
		pr_err("%s: gpio_request failed.\n", __func__);
		return -EINVAL;
	}
#endif

	/* RFS GPIO No Get */
	g_felica_rfs_gpio = of_get_named_gpio(pdev->dev.of_node, "felica_rfs-gpio", 0);
	if (g_felica_rfs_gpio < 0) {
		pr_err("%s: of_get_named_gpio failed.\n", __func__);
		return -EINVAL;
	}

	rc = gpio_request(g_felica_rfs_gpio, "felica_rfs-gpio");
	if (rc) {
		pr_err("%s: gpio_request failed.\n", __func__);
		return -EINVAL;
	}

	/* INT GPIO No Get */
	g_felica_int_gpio = of_get_named_gpio(pdev->dev.of_node, "felica_int-gpio", 0);
	if (g_felica_int_gpio < 0) {
		pr_err("%s: of_get_named_gpio failed.\n", __func__);
		return -EINVAL;
	}

	rc = gpio_request(g_felica_int_gpio, "felica_int-gpio");
	if (rc) {
		pr_err("%s: gpio_request failed.\n", __func__);
		return -EINVAL;
	}

	KFELICA_LOG_D("END");
	return 0;
}

static int pm_felica_remove(struct platform_device *pdev)
{

#ifdef ENABLED_FELICA_VFC_RST_DEV
	gpio_free(g_felica_vfc_gpio);
#endif
	return 0;
}

static int pm_felica_suspend (struct platform_device *pdev, pm_message_t state)
{
	KFELICA_LOG_D("pm_felica_suspend Start");

	KFELICA_LOG_D("pm_felica_suspend End");
	return 0;
}

static int pm_felica_resume (struct platform_device *pdev)
{
	KFELICA_LOG_D("pm_felica_resume Start");

	KFELICA_LOG_D("pm_felica_resume End");
	return 0;
}

/* UartExclusion device	 */
static ssize_t ue_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
	char ue_val;
	
	KFELICA_LOG_D("START");
	
	if ( NULL == buf ) {
		KFELICA_LOG_E("ue_read param err");
		return -EIO;
	}
	
	if ( 1 > len ) {
		KFELICA_LOG_E("length check len = %d", (int)len);
		return -EIO;
	}
	
	ue_val = g_felica_open;
	
	if (copy_to_user(buf, &ue_val, 1)) {
		KFELICA_LOG_E("copy_to_user");
		return -EFAULT;
	}
	
	KFELICA_LOG_D("END ue_val = %d, len = %d", ue_val, (int)len);
	
	return 1;
}
static ssize_t ue_write(struct file *file, const char __user *data, size_t len, loff_t *ppos)
{
	char ue_val;
	
	KFELICA_LOG_D("START");
	
	if( NULL == data ){
		KFELICA_LOG_E("ue_write param err");
		return -EIO;
	}
	
	if ( 1 > len ) {
		KFELICA_LOG_E("length check len = %d", (int)len);
		return -EIO;
	}
	
	if (copy_from_user(&ue_val, data, 1)) {
		KFELICA_LOG_E("copy_from_user");
		return -EFAULT;
	}
	
	g_ue_data = ue_val;
	
	KFELICA_LOG_D("END ue_val = %d / len = %d / g_ue_data = %d", ue_val, (int)len, g_ue_data);
	
	return 1;
}
static int ue_open(struct inode *inode, struct file *file)
{
	int				nRet = 0;
	
	KFELICA_LOG_D("START");

	KFELICA_LOG_D("END");
	return nRet;
}
static int ue_release(struct inode *inode, struct file *file)
{
	KFELICA_LOG_D("START");
	
	KFELICA_LOG_D("END");
	return 0;
}
static int ue_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KFELICA_LOG_D("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , UE_DEV_COUNT, UE_DEV_NAME);
	if (sdResult) {
		KFELICA_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&ue_cdev, &ue_fileops);
	ue_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&ue_cdev, dev, UE_DEV_COUNT);
	if (sdResult) {
		unregister_chrdev_region(dev, UE_DEV_COUNT);
		KFELICA_LOG_E("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}
	
	class_dev = device_create(kfelica_class, NULL, dev, NULL, UE_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&ue_cdev);
		unregister_chrdev_region(dev, UE_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KFELICA_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}

	KFELICA_LOG_D("END");
	
	return sdResult;
}
static void ue_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KFELICA_LOG_D("START");

	device_destroy(kfelica_class, dev);

	cdev_del(&ue_cdev);
	unregister_chrdev_region(dev, UE_DEV_COUNT);

	KFELICA_LOG_D("END");
}

MODULE_LICENSE("GPL v2");

module_init(kfelica_init);
module_exit(kfelica_exit);

