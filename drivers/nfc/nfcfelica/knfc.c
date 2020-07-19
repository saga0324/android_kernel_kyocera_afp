/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
* drivers/nfc/nfcfelica/knfc.c  (knfc driver)
*
* This software is contributed or developed by KYOCERA Corporation.
* (C) 2012 KYOCERA Corporation
* (C) 2013 KYOCERA Corporation
* (C) 2014 KYOCERA Corporation
* (C) 2015 KYOCERA Corporation
* (C) 2016 KYOCERA Corporation
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
#include <linux/irq.h>
#include <linux/mfd/pmic8058.h>
#include <linux/serial_core.h>
#include <linux/sem.h>
#include <linux/spinlock.h>
#include <linux/termios.h>

//#include <asm/uaccess.h>
//#include <asm/current.h>
//#include <asm/mach/mmc.h>

//#include <mach/irqs-8974.h>
#include <linux/of_gpio.h>

#include <soc/qcom/smem.h>

extern bool is_vbus_active(void);

#if 1
#define ENABLED_NFC_VFC_RST_DEV
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
/* HSEL device	*/
#define HSEL_DEV_COUNT					1
#define HSEL_DEV_NAME					"nfc_hsel"
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
/* CXD2235 device				*/
#define CXD2235_DEV_COUNT				1
#define CXD2235_DEV_NAME				"cxd2235"
#define CXD2235_MAGIC					0xFC
#define CXD2235_DBG_POLLING				_IOW(CXD2235_MAGIC, 5,int)
#define CXD2235_SIZE_MASK				0x00FF
/* CXD2235 POLL device		*/
#define CXD2235_POLL_DEV_COUNT			1
#define CXD2235_POLL_DEV_NAME			"cxd2235_poll"
/* UARTSWITCH device		*/
#define UART_SWITCH_DEV_COUNT			1
#define UART_SWITCH_DEV_NAME			"uart_switch"
#define UART_MAGIC						0xFC
#define UART_SWITCH_SET_FLC_STS			_IOW(UART_MAGIC, 1,int)
#define UART_SWITCH_SET_START_KIND		_IOW(UART_MAGIC, 2,int)
#define UART_SWITCH_GET_FLC_STS			_IOR(UART_MAGIC, 3,int)
#define UART_SWTICH_GET_START_KIND		_IOR(UART_MAGIC, 4,int)
/* PON	device			*/
#define I2C_BUS_NUMBER					1
#define I2C_MFD_SLAVE_ADDR				(0x80 >> 1)
#define MFD_RFS_PM_GPIO_NUM				7
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)	(pm_gpio + NR_GPIO_IRQS)
#define PON_DEV_COUNT					1
#define PON_DEV_NAME					"felica_pon"
#define PON_DEV_LOW						0
#define PON_DEV_HIGH					1
/* NFC_VFC_RST device */
#ifdef ENABLED_NFC_VFC_RST_DEV
#define NFC_VFC_RST_DEV_COUNT			1
#define NFC_VFC_RST_DEV_NAME			"vfc_rst"
#define NFC_VFC_RST_DEV_LOW				0
#define NFC_VFC_RST_DEV_HIGH			1
#endif
/* CXD2235POWER device	*/
#define CXD2235_POWER_DEV_NAME			"cxd2235_power"
#define CXD2235_POWER_DEV_COUNT			1

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
#define VBUS_DEV_NAME					"knfcvbus"

#define PM_NFC_NAME						"msm8974-nfc-driver"


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
/************************************************************************/
/*	global																*/
/************************************************************************/
static struct class*		knfc_class = NULL;
static struct cdev			rfs_cdev;
static struct cdev			pon_cdev;
#ifdef ENABLED_NFC_VFC_RST_DEV
static struct cdev			nfc_vfc_rst_cdev;
#endif
static struct cdev			cen_cdev;
static struct cdev			int_poll_cdev;
static struct cdev			rfs_poll_cdev;
static struct cdev			knfcvbus_cdev;
static char					g_readCmd = 0xFF;
static unsigned int			g_unrate = 0;
static unsigned int			g_unbaud = 0;
static struct cdev			rate_cdev;
static struct cdev			rws_cdev;
static char					g_rws_data = -1;
static struct semaphore		chardev_sem;
static struct cdev			intu_poll_cdev;
static struct cdev			hsel_cdev;
static struct cdev			available_poll_cdev;
static struct semaphore		switch_sem;
static struct cdev			felicadev_cdev;
static struct cdev			felicadev_poll_cdev;
static struct cdev			cxd2235_cdev;
static struct cdev			cxd2235_poll_cdev;
static struct cdev			uart_switch_cdev;
static struct i2c_client*	this_client;
static struct icc_poll_data	g_felicadev_data;
static struct icc_poll_data	g_felicadev_poll_data;
static struct icc_poll_data	g_cxd2235_data;	
static struct icc_poll_data	g_cxd2235_poll_data;	
static struct icc_poll_data	g_available_data;	
static int					g_flcsts		= 0;
static int					g_startkind	= 255;
static struct poll_data		g_int_data;
static struct poll_data*	g_int_d = &g_int_data;
static struct poll_data		g_rfs_data;
static struct poll_data*	g_rfs_d = &g_rfs_data;
static struct poll_data		g_intu_data;
static struct poll_data*	g_intu_d = &g_intu_data;
static char					g_functype = 0;
static short				g_sdatasize = 0;
static char					g_carbuf[4101];	
static int					g_felicadev_open_cnt = 0;
static char					g_cxd_buf[500];	
static char					g_cxd_functype = 0;
static struct cdev			cxd2235power_cdev;
/* available 	*/
static int	g_cen_sts = 0;
static int	g_rfs_sts = 0;
static int	g_read_value = 0;

static spinlock_t rfs_spin_lock;

static int					g_nfc_rfs_gpio = -1;
static int					g_nfc_int_gpio = -1;

static int					g_nfc_pon_gpio = -1;
static int					g_nfc_hsel_gpio = -1;
static int					g_nfc_intu_gpio = -1;
#ifdef ENABLED_NFC_VFC_RST_DEV
static int					g_nfc_vfc_gpio = -1;
#endif

/* DEBUG_LOG */
#if 0
#define DEBUG_KNFC_DRIVER
#endif

#ifdef DEBUG_KNFC_DRIVER
#define KNFC_LOG_D(fmt, args...) printk(KERN_INFO "[KNFC][%s]" fmt"\n", __func__, ## args)
#else
#define KNFC_LOG_D(fmt, args...)
#endif

/* ERROR_LOG */
#define KNFC_LOG_E(fmt, args...) printk(KERN_ERR "[KNFC][%s]ERR " fmt"\n", __func__, ## args)


/*
 * prototype
*/
static __init int	knfc_init(void);
static void __exit	knfc_exit(void);

static ssize_t hsel_read(struct file *file, char __user * buf, size_t len, loff_t * ppos);
static ssize_t hsel_write(struct file *file, const char __user *data, size_t len, loff_t *ppos);
static int hsel_open(struct inode *inode, struct file *file);
static int hsel_release(struct inode *inode, struct file *file);
static int hsel_init(void);
static void hsel_exit(void);

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
static ssize_t pon_write(struct file *file, const char __user *data, size_t len, loff_t *ppos);
static int pon_open(struct inode *inode, struct file *file);
static int pon_release(struct inode *inode, struct file *file);
static int pon_init(void);
static void pon_exit(void);
#ifdef ENABLED_NFC_VFC_RST_DEV
static ssize_t nfc_vfc_rst_write(struct file *file, const char __user *data, size_t len, loff_t *ppos);
static int nfc_vfc_rst_open(struct inode *inode, struct file *file);
static int nfc_vfc_rst_release(struct inode *inode, struct file *file);
static int nfc_vfc_rst_init(void);
static void nfc_vfc_rst_exit(void);
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
static irqreturn_t intu_poll_irq_handler(int irq, void *dev_id);
static void intu_poll_work_func(struct work_struct *work);
static ssize_t intu_poll_read(struct file *file, char __user * buf, size_t len, loff_t * ppos);
static int intu_poll_open(struct inode *inode, struct file *file);
static int intu_poll_release(struct inode *inode, struct file *file);
static int intu_poll_init(void);
static void intu_poll_exit(void);
static long available_poll_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
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
static void cxd2235_poll_kick( void );
static int cxd2235_chk_conflict(void);
static int cxd2235_open(struct inode *inode, struct file *file);
static int cxd2235_release(struct inode *inode, struct file *file);
static ssize_t cxd2235_read(struct file *file, char __user * buf, size_t len, loff_t * ppos);
static ssize_t cxd2235_write(struct file *file, const char __user *data, size_t len, loff_t *ppos);
static long cxd2235_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static unsigned int cxd2235_poll(struct file *file, poll_table *wait);
static int cxd2235_init(void);
static void cxd2235_exit(void);
static ssize_t cxd2235_poll_read(struct file *file, char __user * buf, size_t len, loff_t * ppos);
static ssize_t cxd2235_poll_write(struct file *file, const char __user *data, size_t len, loff_t *ppos);
static unsigned int cxd2235_poll_poll(struct file *file, poll_table *wait);
static int cxd2235_poll_open(struct inode *inode, struct file *file);
static int cxd2235_poll_release(struct inode *inode, struct file *file);
static int cxd2235_poll_init(void);
static void cxd2235_poll_exit(void);
static long uart_switch_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static int uart_switch_open(struct inode *inode, struct file *file);
static int uart_switch_release(struct inode *inode, struct file *file);
static int uart_switch_init(void);
static void uart_switch_exit(void);

static ssize_t cxd2235power_write(struct file *file, const char __user *data, size_t len, loff_t *ppos);
static int cxd2235power_open(struct inode *inode, struct file *file);
static int cxd2235power_release(struct inode *inode, struct file *file);
static int cxd2235power_init(void);
static void cxd2235power_exit(void);

static ssize_t knfcvbus_read(struct file *file, char __user * buf, size_t len, loff_t * ppos);
static int knfcvbus_open(struct inode *inode, struct file *file);
static int knfcvbus_release(struct inode *inode, struct file *file);


static struct platform_driver msm8974_nfc_driver;
static int pm_nfc_probe(struct platform_device *pdev);
static int pm_nfc_remove(struct platform_device *pdev);
static int pm_nfc_suspend (struct platform_device *pdev, pm_message_t state);
static int pm_nfc_resume (struct platform_device *pdev);


static const struct file_operations knfcvbus_fileops = {
	.owner   = THIS_MODULE,
	.read    = knfcvbus_read,
	.open    = knfcvbus_open,
	.release = knfcvbus_release,
};

static const struct file_operations hsel_fileops = {
	.owner   = THIS_MODULE,
	.read    = hsel_read,
	.write   = hsel_write,
	.open    = hsel_open,
	.release = hsel_release,
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
	.write   = pon_write,
	.open    = pon_open,
	.release = pon_release,
};
#ifdef ENABLED_NFC_VFC_RST_DEV
static const struct file_operations nfc_vfc_rst_fileops = {
	.owner   = THIS_MODULE,
	.write   = nfc_vfc_rst_write,
	.open    = nfc_vfc_rst_open,
	.release = nfc_vfc_rst_release,
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

static const struct file_operations intu_poll_fileops = {
	.owner   = THIS_MODULE,
	.read    = intu_poll_read,
	.open    = intu_poll_open,
	.release = intu_poll_release,
};
static const struct file_operations available_poll_fileops = {
	.owner   = THIS_MODULE,
	.unlocked_ioctl   = available_poll_ioctl,
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
static const struct file_operations cxd2235_fileops = {
	.owner   = THIS_MODULE,
	.read    = cxd2235_read,
	.write   = cxd2235_write,
	.unlocked_ioctl   = cxd2235_ioctl,
	.open    = cxd2235_open,
	.release = cxd2235_release,
	.poll    = cxd2235_poll,
};
static const struct file_operations cxd2235_poll_fileops = {
	.owner   = THIS_MODULE,
	.read    = cxd2235_poll_read,
	.write   = cxd2235_poll_write,
	.open    = cxd2235_poll_open,
	.release = cxd2235_poll_release,
	.poll    = cxd2235_poll_poll,
};
static const struct file_operations uart_switch_fileops = {
	.owner   = THIS_MODULE,
	.unlocked_ioctl   = uart_switch_ioctl,
	.open    = uart_switch_open,
	.release = uart_switch_release,
};
static const struct file_operations cxd2235power_fileops = {
	.owner   = THIS_MODULE,
	.write   = cxd2235power_write,
	.open    = cxd2235power_open,
	.release = cxd2235power_release,
};

static const struct of_device_id msm8974_nfc_of_match[] = {
	{ .compatible = "kc,msm8974-nfc-driver", },
	{},
};

static struct platform_driver msm8974_nfc_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= PM_NFC_NAME,
		.of_match_table = msm8974_nfc_of_match,
	},
	.probe	 = pm_nfc_probe,
	.remove	 = pm_nfc_remove,
	.suspend	= pm_nfc_suspend,
	.resume		= pm_nfc_resume
};

static ssize_t hsel_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
	char	hsel_val;
	int		ret;

	KNFC_LOG_D("START");

	if (NULL == buf) {
		KNFC_LOG_E("hsel_read param err");
		return -EIO;
	}

	if ( 1 > len ) {
		KNFC_LOG_E("length check len = %d", (int)len);
		return -EIO;
	}
	ret = gpio_get_value( g_nfc_hsel_gpio );
	if ( 0 > ret ) {
		KNFC_LOG_E("gpio_get_value ret = %d", ret);
		return ret;
	}
	if ( 0 == ret){
		hsel_val = 0;
	}else{
		hsel_val = 1;
	}
	if (copy_to_user(buf, &hsel_val, 1)) {
		KNFC_LOG_E("copy_to_user");
		return -EFAULT;
	}

	KNFC_LOG_D("END hsel_val = %d, len = %d", hsel_val, (int)len);

	return 1;
}
static ssize_t hsel_write(struct file *file, const char __user *data, size_t len, loff_t *ppos)
{
	char	hsel_val;

	KNFC_LOG_D("START");

	if (NULL == data) {
		KNFC_LOG_E("hsel_write param err");
		return -EIO;
	}

	if ( 1 > len ) {
		KNFC_LOG_E("length check len = %d", (int)len);
		return -EIO;
	}
	
	if (copy_from_user(&hsel_val, data, 1)) {
		KNFC_LOG_E("copy_from_user");
		return -EFAULT;
	}
	if( 0 == hsel_val ){
		KNFC_LOG_D("HSEL = Low");
	}else if( 1 == hsel_val ){
		KNFC_LOG_D("HSEL = High");
	}else{
		KNFC_LOG_E("param err = %d", hsel_val);
		return -EIO;
	}

	gpio_set_value(g_nfc_hsel_gpio , hsel_val );

	KNFC_LOG_D("END  hsel_val = %d\n",hsel_val);
	return 1;
}
static int hsel_open(struct inode *inode, struct file *file)
{
	KNFC_LOG_D("START");
	KNFC_LOG_D("END");
	return 0;
}

static int hsel_release(struct inode *inode, struct file *file)
{
	KNFC_LOG_D("START");
	KNFC_LOG_D("END");
	return 0;
}

static int hsel_init(void)
{
	int				sdResult = 0;
	struct device*	class_dev;
	dev_t			dev = MKDEV(MISC_MAJOR, 0);
	
	KNFC_LOG_D("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , HSEL_DEV_COUNT, HSEL_DEV_NAME);
	if ( 0 != sdResult) {
		KNFC_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}

	cdev_init(&hsel_cdev, &hsel_fileops);
	hsel_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&hsel_cdev, dev, HSEL_DEV_COUNT);
	if ( 0 != sdResult) {
		unregister_chrdev_region(dev, HSEL_DEV_COUNT);
		KNFC_LOG_E("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}

	class_dev = device_create(knfc_class, NULL, dev, NULL, HSEL_DEV_NAME);
	if ( 0 != IS_ERR(class_dev)) {
		cdev_del(&hsel_cdev);
		unregister_chrdev_region(dev, HSEL_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KNFC_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}

	KNFC_LOG_D("END");

	return sdResult;
}

static void hsel_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KNFC_LOG_D("START");

	device_destroy(knfc_class, dev);

	cdev_del(&hsel_cdev);
	unregister_chrdev_region(dev, HSEL_DEV_COUNT);

	KNFC_LOG_D("END");
}

/* rws device	*/
static ssize_t rws_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
	char rws_val;
	
	KNFC_LOG_D("START");

	if ( NULL == buf ) {
		KNFC_LOG_E("rws_read param err");
		return -EIO;
	}

	if ( 1 > len ) {
		KNFC_LOG_E("length check len = %d", (int)len);
		return -EIO;
	}

	rws_val = g_rws_data;

	if (copy_to_user(buf, &rws_val, 1)) {
		KNFC_LOG_E("copy_to_user");
		return -EFAULT;
	}

	KNFC_LOG_D("END rws_val = %d, len = %d", rws_val, (int)len);

	return 1;
}

static ssize_t rws_write(struct file *file, const char __user *data, size_t len, loff_t *ppos)
{
	char rws_val;

	KNFC_LOG_D("START");

	if( NULL == data ){
		KNFC_LOG_E("rws_write param err");
		return -EIO;
	}

	if ( 1 > len ) {
		KNFC_LOG_E("length check len = %d", (int)len);
		return -EIO;
	}
	
	if (copy_from_user(&rws_val, data, 1)) {
		KNFC_LOG_E("copy_from_user");
		return -EFAULT;
	}

	g_rws_data = rws_val;

	KNFC_LOG_D("END rws_val = %d, g_rws_data = %d", rws_val, g_rws_data);

	return 1;
}
static int rws_open(struct inode *inode, struct file *file)
{
	KNFC_LOG_D("START");
	KNFC_LOG_D("END");
	return 0;
}

static int rws_release(struct inode *inode, struct file *file)
{
	KNFC_LOG_D("START");
	KNFC_LOG_D("END");
	return 0;
}

static int rws_init(void)
{
	int				sdResult = 0;
	struct device*	class_dev;

	dev_t dev = MKDEV(MISC_MAJOR, 0);

	KNFC_LOG_D("START");

	sdResult = alloc_chrdev_region(&dev , 0 , RWS_DEV_COUNT, RWS_DEV_NAME);
	if( 0 != sdResult ) {
		KNFC_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&rws_cdev, &rws_fileops);
	rws_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&rws_cdev, dev, RWS_DEV_COUNT);
	if( 0 != sdResult) {
		unregister_chrdev_region(dev, RWS_DEV_COUNT);
		KNFC_LOG_E("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}

	class_dev = device_create(knfc_class, NULL, dev, NULL, RWS_DEV_NAME);
	if ( 0 != IS_ERR(class_dev)) {
		cdev_del(&rws_cdev);
		unregister_chrdev_region(dev, RWS_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KNFC_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}
	g_rws_data = 0;
	KNFC_LOG_D("END");

	return sdResult;
}

static void rws_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KNFC_LOG_D("START");

	device_destroy(knfc_class, dev);

	cdev_del(&rws_cdev);
	unregister_chrdev_region(dev, RWS_DEV_COUNT);

	KNFC_LOG_D("END");
}

static ssize_t rate_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
	int rate_val;
	
	KNFC_LOG_D("START");
	
	if( 4 > len ){
		KNFC_LOG_E("length check len = %d", (int)len);
		return -EIO;
	}

	rate_val = g_unrate;
	if (copy_to_user(buf, &rate_val, 4)) {
		KNFC_LOG_E("copy_to_user");
		return -EFAULT;
	}

	KNFC_LOG_D("END rate_val = %d, len = %d", rate_val, (int)len);

	return 4;
}
static long rate_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long				nRet	= 0;

	KNFC_LOG_D("START");
	KNFC_LOG_D("cmd = %x\n",cmd);
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
	KNFC_LOG_D("END");
	return nRet;
}

static int rate_open(struct inode *inode, struct file *file)
{
	KNFC_LOG_D("START");
	KNFC_LOG_D("END");
	return 0;
}

static int rate_release(struct inode *inode, struct file *file)
{
	KNFC_LOG_D("START");
	KNFC_LOG_D("END");
	return 0;
}

static int rate_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KNFC_LOG_D("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , RATE_DEV_COUNT, RATE_DEV_NAME);
	if (sdResult) {
		KNFC_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&rate_cdev, &rate_fileops);
	rate_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&rate_cdev, dev, RATE_DEV_COUNT);
	if (sdResult) {
		unregister_chrdev_region(dev, RATE_DEV_COUNT);
		KNFC_LOG_E("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}
	
	class_dev = device_create(knfc_class, NULL, dev, NULL, RATE_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&rate_cdev);
		unregister_chrdev_region(dev, RATE_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KNFC_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}
	
	KNFC_LOG_D("END");
	
	return sdResult;
}

static void rate_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KNFC_LOG_D("START");

	device_destroy(knfc_class, dev);

	cdev_del(&rate_cdev);
	unregister_chrdev_region(dev, RATE_DEV_COUNT);

	KNFC_LOG_D("END");
}

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
		on = RFS_RET_STS_HIGH;
	}else{
		on = RFS_RET_STS_LOW;
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
	unsigned char *smem;

    KNFC_LOG_D("START");

    smem = (unsigned char *)kc_smem_alloc(SMEM_NFC_RFS_SIG_STATE, 1);
    if (smem == NULL) {
        KNFC_LOG_E("smem_alloc");
        return -EFAULT;
    }

#ifdef DEBUG_KNFC_DRIVER
    KNFC_LOG_E("rfs=%d", *data);
    KNFC_LOG_E("old_smem=%d", *smem);
#endif
    if (copy_from_user(smem, data, 1)) {
        KNFC_LOG_E("copy_from_user");
        return -EFAULT;
    }
#ifdef DEBUG_KNFC_DRIVER
    KNFC_LOG_E("new_smem=%d", *smem);
#endif

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
	
	KNFC_LOG_D("START");
	
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
	
	KNFC_LOG_D("END");
	
	return sdResult;
}

static void rfs_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KNFC_LOG_D("START");
	
	cdev_del(&rfs_cdev);
	unregister_chrdev_region(dev, RFS_DEV_COUNT);
	
	KNFC_LOG_D("END");
}


/*
 * function_pon
 */
static ssize_t pon_write(struct file *file, const char __user *data, size_t len, loff_t *ppos)
{
	char on;
	int seton;
	
	KNFC_LOG_D("START");
	
	if ( len < 1 ) {
		KNFC_LOG_E("length check len = %d", (int)len);
		return -EIO;
	}
	
	if (copy_from_user(&on, data, 1)) {
		KNFC_LOG_E("copy_from_user");
		return -EFAULT;
	}

	if (on == PON_DEV_HIGH){
		seton = PON_DEV_HIGH;
		KNFC_LOG_D("pon high.\n");
	}else if (on == PON_DEV_LOW){
		KNFC_LOG_D("pon low.\n");
		seton = PON_DEV_LOW;
	}else {
		KNFC_LOG_D("pon err value = %x \n",on );
		return -EFAULT;
	}

	gpio_set_value(g_nfc_pon_gpio , seton );

	KNFC_LOG_D("END on = %d, seton = %d", on, seton);
	
	return len;
}
static int pon_open(struct inode *inode, struct file *file)
{
	int				nRet = 0;

	KNFC_LOG_D("START");

	KNFC_LOG_D("END");
	return nRet;
}

static int pon_release(struct inode *inode, struct file *file)
{
	KNFC_LOG_D("START");

	gpio_set_value(g_nfc_pon_gpio , PON_DEV_LOW);

	KNFC_LOG_D("END");

	return 0;
}



static int pon_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KNFC_LOG_D("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , PON_DEV_COUNT, PON_DEV_NAME);
	if (sdResult) {
		KNFC_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&pon_cdev, &pon_fileops);
	pon_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&pon_cdev, dev, PON_DEV_COUNT);
	if (sdResult) {
		unregister_chrdev_region(dev, PON_DEV_COUNT);
		KNFC_LOG_E("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}
	
	class_dev = device_create(knfc_class, NULL, dev, NULL, PON_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&pon_cdev);
		unregister_chrdev_region(dev, PON_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KNFC_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}

	KNFC_LOG_D("END");
	
	return sdResult;
}

static void pon_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KNFC_LOG_D("START");

	device_destroy(knfc_class, dev);

	cdev_del(&pon_cdev);
	unregister_chrdev_region(dev, PON_DEV_COUNT);

	KNFC_LOG_D("END");
}


/*
 * function_nfc_vfc_rst
 */
#ifdef ENABLED_NFC_VFC_RST_DEV
static ssize_t nfc_vfc_rst_write(struct file *file, const char __user *data, size_t len, loff_t *ppos)
{
	char on;
	int seton;
	
	KNFC_LOG_D("START");
	
	if ( len < 1 ) {
		KNFC_LOG_E("length check len = %d", (int)len);
		return -EIO;
	}
	
	if (copy_from_user(&on, data, 1)) {
		KNFC_LOG_E("copy_from_user");
		return -EFAULT;
	}

	if (on == NFC_VFC_RST_DEV_HIGH){
		seton = NFC_VFC_RST_DEV_HIGH;
		KNFC_LOG_D("nfc_vfc_rst high.\n");
	}else if (on == NFC_VFC_RST_DEV_LOW){
		KNFC_LOG_D("nfc_vfc_rst low.\n");
		seton = NFC_VFC_RST_DEV_LOW;
	}else {
		KNFC_LOG_D("nfc_vfc_rst err value = %x \n",on );
		return -EFAULT;
	}

	gpio_set_value(g_nfc_vfc_gpio , seton );

	KNFC_LOG_D("END on = %d, seton = %d", on, seton);
	
	return len;
}
static int nfc_vfc_rst_open(struct inode *inode, struct file *file)
{
	int				nRet = 0;

	KNFC_LOG_D("START");

	KNFC_LOG_D("END");
	return nRet;
}

static int nfc_vfc_rst_release(struct inode *inode, struct file *file)
{
	KNFC_LOG_D("START");

	gpio_set_value(g_nfc_vfc_gpio , NFC_VFC_RST_DEV_LOW);

	KNFC_LOG_D("END");

	return 0;
}


static int nfc_vfc_rst_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KNFC_LOG_D("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , NFC_VFC_RST_DEV_COUNT, NFC_VFC_RST_DEV_NAME);
	if (sdResult) {
		KNFC_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&nfc_vfc_rst_cdev, &nfc_vfc_rst_fileops);
	nfc_vfc_rst_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&nfc_vfc_rst_cdev, dev, NFC_VFC_RST_DEV_COUNT);
	if (sdResult) {
		unregister_chrdev_region(dev, NFC_VFC_RST_DEV_COUNT);
		KNFC_LOG_E("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}
	
	class_dev = device_create(knfc_class, NULL, dev, NULL, NFC_VFC_RST_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&nfc_vfc_rst_cdev);
		unregister_chrdev_region(dev, NFC_VFC_RST_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KNFC_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}

	KNFC_LOG_D("END");
	
	return sdResult;
}

static void nfc_vfc_rst_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KNFC_LOG_D("START");

	device_destroy(knfc_class, dev);

	cdev_del(&nfc_vfc_rst_cdev);
	unregister_chrdev_region(dev, NFC_VFC_RST_DEV_COUNT);

	KNFC_LOG_D("END");
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
	
	KNFC_LOG_D("START");
	old_value = int_d->device_status;
	read_value = gpio_get_value_cansleep(g_nfc_int_gpio);

	KNFC_LOG_D("read_value = %d old_value = %d", read_value, old_value);
	
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
		if (irq_set_irq_type(gpio_to_irq(g_nfc_int_gpio), irqflag)){
			KNFC_LOG_E("set_irq_type irqflag = %ld", irqflag);
		}
	}
	
	enable_irq(gpio_to_irq(g_nfc_int_gpio));
	
	if (read_value != old_value || int_d->read_error) {
		int_d->irq_handler_done = 1;
		wake_up_interruptible(&int_d->read_wait);
	}
	
	KNFC_LOG_D("END read_value = %d, old_value = %d, int_d->read_error = %d"
					, read_value, old_value, int_d->read_error);
}

static irqreturn_t int_poll_irq_handler(int irq, void *dev_id)
{
	struct poll_data *int_d = g_int_d;
	
	KNFC_LOG_D("START irq = %d", irq);
	
	disable_irq_nosync(gpio_to_irq(g_nfc_int_gpio));

	schedule_delayed_work(&int_d->work, msecs_to_jiffies(INT_POLL_DELAY));
	
	KNFC_LOG_D("END");
	
	return IRQ_HANDLED;
}

static unsigned int int_poll_poll(struct file *file, poll_table *wait)
{
	struct poll_data *int_d = g_int_d;
	unsigned int mask = 0;
	
	KNFC_LOG_D("START");
	
	poll_wait(file, &int_d->read_wait, wait);
	if (int_d->irq_handler_done){
		mask = POLLIN | POLLRDNORM;
	}
	KNFC_LOG_D("END mask = %d", mask);
	
	return (mask);
}

static ssize_t int_poll_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
	struct poll_data *int_d = g_int_d;
	int ret;
	char cret;
	
	KNFC_LOG_D("START");
	
	if ( len < 1 ) {
		KNFC_LOG_E("length check len = %d", (int)len);
		return -EIO;
	}
	
	if (!int_d->irq_handler_done) {
		if (file->f_flags & O_NONBLOCK) {
			KNFC_LOG_E("NONBLOCK");
			return -EAGAIN;
		}
		KNFC_LOG_D("FeliCa int_poll wait irq");
		ret = wait_event_interruptible(int_d->read_wait, int_d->irq_handler_done == 1);
		if (-ERESTARTSYS == ret) {
			KNFC_LOG_D("wait_event_interruptible ret = %d", ret);
			return -EINTR;
		}
	}
	
	if (int_d->read_error) {
		int_d->irq_handler_done = 0;
		int_d->read_error = 0;
		KNFC_LOG_E("int_d->read_error = %d", int_d->read_error);
		return -EIO;
	}
	
	if (int_d->device_status == INT_POLL_DEV_HIGH){
		cret = INT_POLL_RET_STS_HIGH;
	}else{
		cret = INT_POLL_RET_STS_LOW;
	}

	len = 1;

	if (copy_to_user(buf, &cret, len)) {
		KNFC_LOG_E("copy_to_user");
		return -EFAULT;
	}
	int_d->irq_handler_done = 0;
	
	KNFC_LOG_D("END len = %d, cret = %d", (int)len, cret);
	
	return len;
}

static int int_poll_open(struct inode *inode, struct file *file)
{
	struct poll_data *int_d = g_int_d;
	unsigned long irqflag = 0;
	int ret = 0;
	
	KNFC_LOG_D("START");
	
	if (int_d->open_flag) {
		KNFC_LOG_E("only one time");
		return -EBUSY;
	}
	int_d->open_flag = 1;
	
	ret = gpio_get_value_cansleep(g_nfc_int_gpio);
	if (ret < 0) {
		int_d->open_flag = 0;
		KNFC_LOG_E("gpio_get_value ret = %d", ret);
		return -EIO;
	}
	int_d->device_status = ret;
	
	if (int_d->device_status == INT_POLL_DEV_LOW){
		irqflag = IRQF_TRIGGER_HIGH | IRQF_SHARED;
	}else{
		irqflag = IRQF_TRIGGER_LOW | IRQF_SHARED;
	}

	if (request_any_context_irq(gpio_to_irq(g_nfc_int_gpio), int_poll_irq_handler, irqflag, INT_POLL_DEV_NAME, (void*)int_d)) {
		int_d->open_flag = 0;
		KNFC_LOG_E("request_irq irqflag = %ld", irqflag);
		return -EIO;
	}
	
	if (enable_irq_wake(gpio_to_irq(g_nfc_int_gpio))){
		
		KNFC_LOG_E("enable_irq_wake");
		
		free_irq(gpio_to_irq(g_nfc_int_gpio), (void *)int_d);
		
		return -EIO;
	}
	
	int_d->irq_handler_done = 0;
	
	KNFC_LOG_D("END");
	
	return 0;
}

static int int_poll_release(struct inode *inode, struct file *file)
{
	struct poll_data *int_d = g_int_d;
	
	KNFC_LOG_D("START");
	
	cancel_delayed_work(&int_d->work);
	
	if (disable_irq_wake(gpio_to_irq(g_nfc_int_gpio))){
		KNFC_LOG_E("disable_irq_wake");
	}
	free_irq(gpio_to_irq(g_nfc_int_gpio), (void *)int_d);
	
	int_d->open_flag = 0;
	
	KNFC_LOG_D("END");
	
	return 0;
}


static int int_poll_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KNFC_LOG_D("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , INT_POLL_DEV_COUNT, INT_POLL_DEV_NAME);
	if (sdResult) {
		KNFC_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&int_poll_cdev, &int_poll_fileops);
	int_poll_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&int_poll_cdev, dev, INT_POLL_DEV_COUNT);
	if (sdResult) {
		unregister_chrdev_region(dev, INT_POLL_DEV_COUNT);
		KNFC_LOG_E("cdev_add sdResult = %d",sdResult);
		return sdResult;
	}
	
	class_dev = device_create(knfc_class, NULL, dev, NULL, INT_POLL_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&int_poll_cdev);
		unregister_chrdev_region(dev, INT_POLL_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KNFC_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}
	
	memset(g_int_d, 0x00, sizeof(struct poll_data));

	INIT_DELAYED_WORK(&g_int_d->work, int_poll_work_func);

	init_waitqueue_head(&g_int_d->read_wait);
	
	g_int_d->open_flag = 0;
	
	KNFC_LOG_D("END");
	
	return sdResult;
}

static void int_poll_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	KNFC_LOG_D("START");
	
	cdev_del(&int_poll_cdev);
	unregister_chrdev_region(dev, INT_POLL_DEV_COUNT);
	KNFC_LOG_D("END");
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
	
	KNFC_LOG_D("START");
	
	old_value = rfs_d->device_status;

	spin_lock_irqsave( &rfs_spin_lock, spin_lock_irqflag);

	KNFC_LOG_D("g_read_value = %d old_value = %d", g_read_value, old_value);
	
	if (g_read_value < 0) {
		local_value = rfs_d->read_error = g_read_value;
		spin_unlock_irqrestore( &rfs_spin_lock, spin_lock_irqflag);
		KNFC_LOG_E("gpio_get_value err g_read_value = %d", g_read_value);
	}else {
		local_value = g_read_value;
		spin_unlock_irqrestore( &rfs_spin_lock, spin_lock_irqflag);

		gpio_read_value = gpio_get_value_cansleep(g_nfc_rfs_gpio);

		if(gpio_read_value < 0){
			rfs_d->read_error = gpio_read_value;
			KNFC_LOG_E("gpio_get_value_cansleep err gpio_read_value = %d", gpio_read_value);
		}else{
			g_rfs_sts = gpio_read_value;
			rfs_d->device_status = gpio_read_value;
			rfs_d->read_error = 0;

			if (rfs_d->device_status == RFS_POLL_DEV_LOW){
				irqflag = IRQF_TRIGGER_HIGH | IRQF_SHARED;
			}else{
				irqflag = IRQF_TRIGGER_LOW | IRQF_SHARED;
			}
			if (irq_set_irq_type(gpio_to_irq(g_nfc_rfs_gpio), irqflag)){
				KNFC_LOG_E("set_irq_type irqflag = %ld", irqflag);
			}
		}

		if( 1 == available_d->available_flag ){
			if( 0 != g_cen_sts ){
				if( RFS_GPIO_VAL_L != g_rfs_sts ){
					if( 0 == g_flcsts ){
						KNFC_LOG_D("wake up available");
						available_d->rsp_done = 1;
						wake_up_interruptible(&available_d->read_wait);
					}
				}
			}
		}
	}
	
	enable_irq(gpio_to_irq(g_nfc_rfs_gpio));
	
	if (local_value != old_value || gpio_read_value != old_value || rfs_d->read_error) {
		rfs_d->irq_handler_done = 1;
		wake_up_interruptible(&rfs_d->read_wait);
	}

	KNFC_LOG_D("END local_value = %d, old_value = %d, rfs_d->read_error = %d", local_value, old_value, rfs_d->read_error);
}

static irqreturn_t rfs_poll_irq_handler(int irq, void *dev_id)
{
	struct poll_data *rfs_d = g_rfs_d;

	KNFC_LOG_D("START irq = %d", irq);

	disable_irq_nosync(gpio_to_irq(g_nfc_rfs_gpio));

	spin_lock( &rfs_spin_lock );
	g_read_value = gpio_get_value(g_nfc_rfs_gpio);
	spin_unlock( &rfs_spin_lock );

	schedule_delayed_work(&rfs_d->work, msecs_to_jiffies(RFS_POLL_DELAY));
	
	KNFC_LOG_D("END");
	
	return IRQ_HANDLED;
}

static unsigned int rfs_poll_poll(struct file *file, poll_table *wait)
{
	struct poll_data *rfs_d = g_rfs_d;
	unsigned int mask = 0;
	
	KNFC_LOG_D("START");
	
	poll_wait(file, &rfs_d->read_wait, wait);
	if (rfs_d->irq_handler_done){
		mask = POLLIN | POLLRDNORM;
	}
	KNFC_LOG_D("END mask = %d", mask);
	
	return (mask);
}

static ssize_t rfs_poll_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
	struct poll_data *rfs_d = g_rfs_d;
	int ret;
	char cret;
	
	KNFC_LOG_D("START");
	
	if ( len < 1 ) {
		KNFC_LOG_E("length check len = %d", (int)len);
		return -EIO;
	}
	
	if (!rfs_d->irq_handler_done) {
		if (file->f_flags & O_NONBLOCK) {
			KNFC_LOG_E("NONBLOCK");
			return -EAGAIN;
		}
		KNFC_LOG_D("FeliCa rfs_poll wait irq");
		ret = wait_event_interruptible(rfs_d->read_wait, rfs_d->irq_handler_done == 1);
		if (-ERESTARTSYS == ret) {
			KNFC_LOG_D("wait_event_interruptible ret = %d", ret);
			return -EINTR;
		}
	}
	
	if (rfs_d->read_error) {
		rfs_d->irq_handler_done = 0;
		rfs_d->read_error = 0;
		KNFC_LOG_E("rfs_d->read_error = %d", rfs_d->read_error);
		return -EIO;
	}
	
	if (rfs_d->device_status == RFS_POLL_DEV_HIGH){
		cret = RFS_POLL_RET_STS_HIGH;
	}else{
		cret = RFS_POLL_RET_STS_LOW;
	}

	len = 1;

	if (copy_to_user(buf, &cret, len)) {
		KNFC_LOG_E("copy_to_user");
		return -EFAULT;
	}
	rfs_d->irq_handler_done = 0;
	
	KNFC_LOG_D("END len = %d, cret = %d", (int)len, cret);
	
	return len;
}

static int rfs_poll_open(struct inode *inode, struct file *file)
{
	struct poll_data *rfs_d = g_rfs_d;
	unsigned long irqflag = 0;
	int ret = 0;
	
	KNFC_LOG_D("START");
	
	if (rfs_d->open_flag) {
		KNFC_LOG_E("only one time");
		return -EBUSY;
	}
	rfs_d->open_flag = 1;
	
	ret = gpio_get_value_cansleep(g_nfc_rfs_gpio);
	if (ret < 0) {
		rfs_d->open_flag = 0;
		KNFC_LOG_E("gpio_get_value ret = %d", ret);
		return -EIO;
	}
	rfs_d->device_status = ret;
	g_rfs_sts = ret;

	if (rfs_d->device_status == RFS_POLL_DEV_LOW){
		irqflag = IRQF_TRIGGER_HIGH | IRQF_SHARED;
	}else{
		irqflag = IRQF_TRIGGER_LOW | IRQF_SHARED;
	}

	if (request_any_context_irq(gpio_to_irq(g_nfc_rfs_gpio), rfs_poll_irq_handler, irqflag, RFS_POLL_DEV_NAME, (void*)rfs_d)) {
		rfs_d->open_flag = 0;
		KNFC_LOG_E("request_irq irqflag = %ld", irqflag);
		return -EIO;
	}
	
	if (enable_irq_wake(gpio_to_irq(g_nfc_rfs_gpio))){
		
		KNFC_LOG_E("enable_irq_wake");
		
		free_irq(gpio_to_irq(g_nfc_rfs_gpio), (void *)rfs_d);
		
		return -EIO;
	}
	
	rfs_d->irq_handler_done = 0;
	
	KNFC_LOG_D("END");
	
	return 0;
}

static int rfs_poll_release(struct inode *inode, struct file *file)
{
	struct poll_data *rfs_d = g_rfs_d;
	
	KNFC_LOG_D("START");
	
	cancel_delayed_work(&rfs_d->work);
	
	if (disable_irq_wake(gpio_to_irq(g_nfc_rfs_gpio))){
		KNFC_LOG_E("disable_irq_wake");
	}
	free_irq(gpio_to_irq(g_nfc_rfs_gpio), (void *)rfs_d);
	
	rfs_d->open_flag = 0;
	
	KNFC_LOG_D("END");
	
	return 0;
}


static int rfs_poll_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KNFC_LOG_D("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , RFS_POLL_DEV_COUNT, RFS_POLL_DEV_NAME);
	if (sdResult) {
		KNFC_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&rfs_poll_cdev, &rfs_poll_fileops);
	rfs_poll_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&rfs_poll_cdev, dev, RFS_POLL_DEV_COUNT);
	if (sdResult) {
		unregister_chrdev_region(dev, RFS_POLL_DEV_COUNT);
		KNFC_LOG_E("cdev_add sdResult = %d",sdResult);
		return sdResult;
	}
	
	class_dev = device_create(knfc_class, NULL, dev, NULL, RFS_POLL_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&rfs_poll_cdev);
		unregister_chrdev_region(dev, RFS_POLL_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KNFC_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}
	
	memset(g_rfs_d, 0x00, sizeof(struct poll_data));

	spin_lock_init( &rfs_spin_lock );

	INIT_DELAYED_WORK(&g_rfs_d->work, rfs_poll_work_func);

	init_waitqueue_head(&g_rfs_d->read_wait);
	
	g_rfs_d->open_flag = 0;
	
	KNFC_LOG_D("END");
	
	return sdResult;
}

static void rfs_poll_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	KNFC_LOG_D("START");
	
	cdev_del(&rfs_poll_cdev);
	unregister_chrdev_region(dev, RFS_POLL_DEV_COUNT);
	KNFC_LOG_D("END");
}



static int cen_open( struct inode *inode, struct file *filp )
{
	KNFC_LOG_D("");
	return 0;
}

static int cen_release( struct inode *inode, struct file *filp )
{
	KNFC_LOG_D("");
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

	KNFC_LOG_D("cen_read start\n");
	KNFC_LOG_D("cen_read cmd = %x\n", cmd);

	if (buf == NULL) {
		KNFC_LOG_D("cen_read param err\n");
		return -EFAULT;
	}
	
	for( retry_cnt = 0; retry_cnt<2; retry_cnt++ ){
	i2c_ret  =i2c_transfer(this_client->adapter, read_msgs, 2);
		KNFC_LOG_D("write_felica_cen ret = %d val = %d\n", i2c_ret,g_cen_sts);
		if (I2C_FAILURE >= i2c_ret) {
			usleep( 10000 );
		} else {
			break;
		}
	}

	//KNFC_LOG_D("cen_read i2c_ret = %d \n",i2c_ret);
	if (I2C_FAILURE >= i2c_ret) {
		return -EIO;
	}

	read_buff &= MFD_LOCK_MASK;

	if (copy_to_user(buf, &read_buff, sizeof(read_buff))) {
		KNFC_LOG_D("cen_read copy_to_user err \n");
		return -EFAULT;
	}
	KNFC_LOG_D("cen_read end\n");
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

	KNFC_LOG_D("START");
	
	switch( cmd ){
		case SET_I2C_READ_COMMAND:
			KNFC_LOG_D("START SET_I2C_READ_COMMAND");
			if (copy_from_user(&read_cmd, (void __user *)arg, sizeof(read_cmd))) {
				KNFC_LOG_E("copy_from_user set_COMMAND = %x", read_cmd);
				return -EFAULT;
			}
			KNFC_LOG_D("copy_from_user set_COMMAND = %x", read_cmd);
			if( 0xFF == g_readCmd ){
				g_readCmd = read_cmd;
				return 0;
			}else{
				return -1;
			}
			break;
		case I2C_WRITE_COMMAND:
			KNFC_LOG_D("START I2C_WRITE_COMMAND");
			if (copy_from_user(&lock_status, (void __user *)arg, 2)) {
				KNFC_LOG_E("copy_from_user write_E2PROM");
				return -EFAULT;
			}
			write_msg[0].buf = lock_status;
			lock_temp = g_cen_sts;
			g_cen_sts = lock_status[1];
			for( retry_cnt = 0; retry_cnt<2; retry_cnt++ ){
				i2c_ret = i2c_transfer(this_client->adapter, write_msg, 1);
				KNFC_LOG_D("write_felica_cen ret = %d val = %d\n", i2c_ret,g_cen_sts);
				if( I2C_FAILURE < i2c_ret ){
					break;
				}else{
					usleep( 10000 );
				}
			}

			if (I2C_FAILURE >= i2c_ret) {
				g_cen_sts = lock_temp;
				return -EIO;
			}else{
				KNFC_LOG_D("write_felica_cen SUCCESS \n");
				if( 1 == available_d->available_flag ){
					if( 0 != g_cen_sts ){
						if( RFS_GPIO_VAL_L != g_rfs_sts ){
							if( 0 == g_flcsts ){
								available_d->rsp_done = 1;
								KNFC_LOG_D("wake up available");
								wake_up_interruptible(&available_d->read_wait);
							}
						}
					}
				}
				return 1;
			}
		default:
			KNFC_LOG_E("cmd = %d", cmd);
			return -EINVAL;
	}
	return 0;
}


static int cen_probe(struct i2c_client *client, const struct i2c_device_id * devid)
{
	struct cen_data *cen;
	int alloc_ret = 0;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KNFC_LOG_D("START");
	
	cen = kzalloc(sizeof(struct cen_data), GFP_KERNEL);
	if (!cen) {
		KNFC_LOG_E("kzalloc");
		return -ENOMEM;	
	}
	this_client = client;

	i2c_set_clientdata(client, cen);

	cen->input_dev = input_allocate_device();
	
	alloc_ret = alloc_chrdev_region(&dev , 0 , CEN_DEV_COUNT, CEN_DEV_NAME);
	if (alloc_ret) {
		KNFC_LOG_E("alloc_chrdev_region");
		return alloc_ret;
	}
	
	cdev_init(&cen_cdev, &cen_fileops);
	cen_cdev.owner = THIS_MODULE;
	
	cdev_add(&cen_cdev, dev, CEN_DEV_COUNT);
	
	device_create(knfc_class, NULL, dev, NULL, CEN_DEV_NAME);

	g_readCmd = 0xFF;

	KNFC_LOG_D("END");
	
	return (0);
}

static int cen_remove(struct i2c_client *client)
{

	dev_t dev;

	struct cen_data *cen = i2c_get_clientdata(client);
	
	KNFC_LOG_D("START");

	dev  = MKDEV(MISC_MAJOR, 0);

	device_destroy(knfc_class, dev);

	cdev_del(&cen_cdev);

	unregister_chrdev_region(dev, 1);

	input_unregister_device(cen->input_dev);
	
	kfree(cen);
	
	KNFC_LOG_D("END");
	
	return (0);
}

/*
 * function_intu_poll
 */

static void intu_poll_work_func(struct work_struct *work)
{
	struct poll_data *intu_d = g_intu_d;
	int read_value			= 0;
	int old_value			= 0;
	unsigned long irqflag	= 0;
	
	KNFC_LOG_D("START");

	old_value = intu_d->device_status;
	read_value = gpio_get_value(g_nfc_intu_gpio);

	KNFC_LOG_D("read_value = %d old_value = %d", read_value, old_value);
	
	if (read_value < 0) {
		intu_d->read_error = read_value;
	} else if (read_value != old_value) {
		intu_d->device_status = read_value;
		intu_d->read_error = 0;
		
		if (intu_d->device_status == INTU_POLL_DEV_LOW){
			irqflag = IRQF_TRIGGER_HIGH | IRQF_SHARED;
		}else{
			irqflag = IRQF_TRIGGER_LOW | IRQF_SHARED;
		}
		if (irq_set_irq_type(gpio_to_irq(g_nfc_intu_gpio), irqflag)){
			KNFC_LOG_E("set_irq_type irqflag = %ld", irqflag);
		}
	}
	
	enable_irq(gpio_to_irq(g_nfc_intu_gpio));
	
	if (read_value != old_value || intu_d->read_error) {
		intu_d->irq_handler_done = 1;
		wake_up_interruptible(&intu_d->read_wait);
	}
	KNFC_LOG_D("END read_value = %d, old_value = %d, intu_d->read_error = %d"
					, read_value, old_value, intu_d->read_error);
}

static irqreturn_t intu_poll_irq_handler(int irq, void *dev_id)
{
	struct poll_data *intu_d = g_intu_d;
	
	KNFC_LOG_D("START irq = %d", irq);

	disable_irq_nosync(gpio_to_irq(g_nfc_intu_gpio));

	schedule_delayed_work(&intu_d->work, msecs_to_jiffies(INTU_POLL_DELAY));
	
	KNFC_LOG_D("END");
	
	return IRQ_HANDLED;
}

static ssize_t intu_poll_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
	struct poll_data *intu_d = g_intu_d;
	int ret;
	char cret;
	
	KNFC_LOG_D("START");
	
	if ( len < 1 ) {
		KNFC_LOG_E("length check len = %d", (int)len);
		return -EIO;
	}
	
	if (!intu_d->irq_handler_done) {
		if (file->f_flags & O_NONBLOCK) {
			KNFC_LOG_E("NONBLOCK");
			return -EAGAIN;
		}
		KNFC_LOG_D("NFC intu_poll wait irq");
		ret = wait_event_interruptible(intu_d->read_wait, intu_d->irq_handler_done == 1);
		if (-ERESTARTSYS == ret) {
			KNFC_LOG_D("wait_event_interruptible ret = %d", ret);
			return -EINTR;
		}
	}
	
	if (intu_d->read_error) {
		intu_d->irq_handler_done = 0;
		intu_d->read_error = 0;
		KNFC_LOG_E("intu_d->read_error = %d", intu_d->read_error);
		return -EIO;
	}
	
	if (intu_d->device_status == INTU_POLL_DEV_HIGH){
		cret = 1;
	}else{
		cret = 0;
	}
	len = 1;

	if (copy_to_user(buf, &cret, len)) {
		KNFC_LOG_E("copy_to_user");
		return -EFAULT;
	}
	intu_d->irq_handler_done = 0;

	KNFC_LOG_D("END len = %d, cret = %d", (int)len, cret);
	
	return len;
}

static int intu_poll_open(struct inode *inode, struct file *file)
{
	struct poll_data *intu_d = g_intu_d;
	unsigned long irqflag = 0;
	int ret = 0;
	
	KNFC_LOG_D("START");
	
	if (intu_d->open_flag) {
		KNFC_LOG_E("only one time");
		return 0;
	}
	intu_d->open_flag = 1;
	
	ret = gpio_get_value(g_nfc_intu_gpio);
	if (ret < 0) {
		intu_d->open_flag = 0;
		KNFC_LOG_E("gpio_get_value ret = %d", ret);
		return -EIO;
	}
	intu_d->device_status = ret;
	
	if (intu_d->device_status == INTU_POLL_DEV_LOW){
		irqflag = IRQF_TRIGGER_HIGH | IRQF_SHARED;
	}else{
		irqflag = IRQF_TRIGGER_LOW | IRQF_SHARED;
	}

	ret = request_irq(gpio_to_irq(g_nfc_intu_gpio), intu_poll_irq_handler, irqflag, INTU_POLL_DEV_NAME, (void*)intu_d);
	if (ret) {
		intu_d->open_flag = 0;
		KNFC_LOG_E("request_irq irqflag = %ld  ret = %d", irqflag, ret);
		return -EIO;
	}
	
	if (enable_irq_wake(gpio_to_irq(g_nfc_intu_gpio))){
		
		KNFC_LOG_E("enable_irq_wake");
		
		free_irq(gpio_to_irq(g_nfc_intu_gpio), (void *)intu_d);
		
		return -EIO;
	}
	
	intu_d->irq_handler_done = 0;
	
	KNFC_LOG_D("END");
	
	return 0;
}

static int intu_poll_release(struct inode *inode, struct file *file)
{
	struct poll_data *intu_d = g_intu_d;
	
	KNFC_LOG_D("START");
	
	cancel_delayed_work(&intu_d->work);
	
	if (disable_irq_wake(gpio_to_irq(g_nfc_intu_gpio))){
		KNFC_LOG_E("disable_irq_wake");
	}
	free_irq(gpio_to_irq(g_nfc_intu_gpio), (void *)intu_d);
	
	intu_d->open_flag = 0;
	
	KNFC_LOG_D("END");
	
	return 0;
}



static int intu_poll_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KNFC_LOG_D("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , INTU_POLL_DEV_COUNT, INTU_POLL_DEV_NAME);
	if (sdResult) {
		KNFC_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&intu_poll_cdev, &intu_poll_fileops);
	intu_poll_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&intu_poll_cdev, dev, INTU_POLL_DEV_COUNT);
	if (sdResult) {
		unregister_chrdev_region(dev, INTU_POLL_DEV_COUNT);
		KNFC_LOG_E("cdev_add sdResult = %d",sdResult);
		return sdResult;
	}
	
	class_dev = device_create(knfc_class, NULL, dev, NULL, INTU_POLL_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&intu_poll_cdev);
		unregister_chrdev_region(dev, INTU_POLL_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KNFC_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}
	
	memset(g_intu_d, 0x00, sizeof(struct poll_data));

	INIT_DELAYED_WORK(&g_intu_d->work, intu_poll_work_func);

	init_waitqueue_head(&g_intu_d->read_wait);
	
	g_intu_d->open_flag = 0;
	
	KNFC_LOG_D("END");
	
	return sdResult;
}

static void intu_poll_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	KNFC_LOG_D("START");
	
	cdev_del(&intu_poll_cdev);
	unregister_chrdev_region(dev, INTU_POLL_DEV_COUNT);
	KNFC_LOG_D("END");
}
/********************/
/*available poll	*/
static long available_poll_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long			nRet	= -EIO;

	KNFC_LOG_D("available_poll_ioctl START cmd = %x",cmd);
	switch(cmd){
		case AVAILABLE_POLL_DEV_SET_PRE_LOCK:
			g_cen_sts = 0;
			nRet = 0;
			break;
		default:
			break;
	}
	KNFC_LOG_D("available_poll_ioctl END cmd = %x",cmd);
	return nRet;
}

static ssize_t available_poll_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
	char	type = 0;
	int		nRet = -1;
	struct icc_poll_data* available_d = &g_available_data;

	KNFC_LOG_D("available poll read START");

	if( ( 0 != g_flcsts )	||
		( 0 == g_cen_sts )	||
		( RFS_GPIO_VAL_L == g_rfs_sts )){

		KNFC_LOG_D(" g_flcsts = %d, g_cen_sts = %d, g_rfs_sts = %d", g_flcsts, g_cen_sts, g_rfs_sts);

		available_d->available_flag = 1;
		nRet = wait_event_interruptible(available_d->read_wait,available_d->rsp_done == 1);
		if( nRet < 0 ){
			KNFC_LOG_E("available_poll_read() wait_event_interruptible ERROR = %d ",nRet);
			return -EIO;
		}
		type = 0x01;
		available_d->rsp_done		= 0;
		available_d->available_flag	= 0;
	}else{
		type = 0x01;
	}

	if (copy_to_user(buf, &type, len)) {
		KNFC_LOG_E("copy_to_user");
		return -EFAULT;
	}

	KNFC_LOG_D("available poll read END read size = %d ", (int)len);

	return len;
}

static int available_poll_open(struct inode *inode, struct file *file)
{
	KNFC_LOG_D("available poll open START");
	
	KNFC_LOG_D("available poll open END");
	return 0;
}

static int available_poll_release(struct inode *inode, struct file *file)
{
	KNFC_LOG_D("available poll close START");
	if( g_available_data.available_flag == 1 ){
		g_available_data.rsp_done = 1;
		wake_up_interruptible(&g_available_data.read_wait);
	}
	g_available_data.rsp_done = 0;
	g_available_data.available_flag = 0;
	KNFC_LOG_D("available poll close END");
	return 0;
}

static int available_poll_init(void)
{
	int           sdResult      = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KNFC_LOG_D("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , AVAILABLE_POLL_DEV_COUNT, AVAILABLE_POLL_DEV_NAME);
	if (sdResult) {
		KNFC_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&available_poll_cdev, &available_poll_fileops);
	available_poll_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&available_poll_cdev, dev, AVAILABLE_POLL_DEV_COUNT);
	if (sdResult) {
		unregister_chrdev_region(dev, AVAILABLE_POLL_DEV_COUNT);
		KNFC_LOG_E("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}

	class_dev = device_create(knfc_class, NULL, dev, NULL, AVAILABLE_POLL_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&available_poll_cdev);
		unregister_chrdev_region(dev, AVAILABLE_POLL_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KNFC_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}

	memset((void*)&g_available_data, 0x00, sizeof(struct icc_poll_data));

	init_waitqueue_head(&g_available_data.read_wait);

	KNFC_LOG_D("END");

	return sdResult;
}

static void available_poll_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KNFC_LOG_D("START");

	device_destroy(knfc_class, dev);

	cdev_del(&available_poll_cdev);
	unregister_chrdev_region(dev, AVAILABLE_POLL_DEV_COUNT);

	KNFC_LOG_D("END");
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

	KNFC_LOG_D("felicadev read START len = %d",(int)len);
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
		KNFC_LOG_E("felicadev_ioctl() wait_event_interruptible ERROR = %d ",nRet);
		up( &chardev_sem );
		return -EIO;
	}
	felica_d->rsp_done = 0;

	slen = g_carbuf[2];
	slen = slen << 8;
	slen = slen + g_carbuf[3];
	if( len != slen ){
		if( len < slen ){
			KNFC_LOG_E("size unmatch len = %d slen = %d",(int)len,slen);
			up( &chardev_sem );
			return -EFAULT;
		}
	}
	if (copy_to_user(buf, &g_carbuf[4], slen)){
		KNFC_LOG_E("copy_to_user");
		up( &chardev_sem );
		return -EFAULT;
	}

	g_functype = -1;
	up( &chardev_sem );
	KNFC_LOG_D("felicadev read END param size = %d ret size = %d ", (int)len, slen);

	return slen;
}
static ssize_t felicadev_write(struct file *file, const char __user *data, size_t len, loff_t *ppos)
{
	int		nRet = -1;
	struct icc_poll_data *felica_d = &g_felicadev_data;

	KNFC_LOG_D("felicadev write START");
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
		KNFC_LOG_E("copy_from_user");
		up( &chardev_sem );
		return -EFAULT;
	}

	felicadev_poll_kick();

	nRet = wait_event_interruptible(felica_d->rsp_wait,felica_d->rsp_done == 1);
	if( nRet < 0 ){
		KNFC_LOG_E("felicadev_ioctl() wait_event_interruptible ERROR = %d ",nRet);
		up( &chardev_sem );
		return -EIO;
	}
	felica_d->rsp_done = 0;

	nRet = g_carbuf[2];
	nRet = nRet << 8;
	nRet = nRet + g_carbuf[3];

	g_functype = -1;
	up( &chardev_sem );
	KNFC_LOG_D("felicadev write END retsize = %d paramsize = %d",nRet,(int)len);
	return nRet;
}
static long felicadev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long			nRet	= 0;
	short			sstructsize = 0;
	int				nSize	= 0;
	struct icc_poll_data *felica_d = &g_felicadev_data;

	KNFC_LOG_D("felicadev ioctl START cmd = %x",cmd);
	nRet = down_interruptible( &chardev_sem );
	if( 0 > nRet ){
		KNFC_LOG_E("felicadev_ioctl down_interruptible ERROR cmd = %d",cmd);
		return -ERESTARTSYS;
	}
	switch(cmd){
		case TCGETS:
			KNFC_LOG_E("felicadev_ioctl un use cmd = %d",cmd);
			nRet = -EIO;
			break;
		case TCSANOW:
		case TCSETS:
			g_functype = 4;
			g_carbuf[0] = (char)g_functype;

			sstructsize = sizeof(struct termios);
			KNFC_LOG_D("felicadev ioctl termios size = %d",sstructsize);

			g_carbuf[2] = (( sstructsize >> 8 )& FELICA_DEV_SIZE_MASK);
			g_carbuf[3] = (sstructsize & FELICA_DEV_SIZE_MASK);

			if (copy_from_user(&g_carbuf[4], (void __user *)arg, sizeof(struct termios))) {
				KNFC_LOG_E("copy_from_user");
				up( &chardev_sem );
				return -EFAULT;
			}

			felicadev_poll_kick();
			nRet = wait_event_interruptible(felica_d->rsp_wait,felica_d->rsp_done == 1);
			if( nRet < 0 ){
				KNFC_LOG_E("felicadev_ioctl() wait_event_interruptible ERROR = %d ",(int)nRet);
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
				KNFC_LOG_E("felicadev_ioctl() wait_event_interruptible ERROR = %d ",(int)nRet);
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
					KNFC_LOG_E("copy_to_user");
					nRet = -EFAULT;
				}
			}else{
				nRet = -EIO;
			}
			KNFC_LOG_D("felicadev ioctl END ret = %d cmd = %x avail = %d",(int)nRet, cmd, nSize);
			break;
		default:
			KNFC_LOG_E("felicadev ioctl un use cmd = %d",cmd);
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
	KNFC_LOG_D("felicadev open START");

	nRet = felicadev_chk_conflict();
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
						KNFC_LOG_E("felicadev_open() wait_event_interruptible ERROR = %d ",nRet);
						return -EIO;
					}
					nRet = down_interruptible( &chardev_sem );
					if( 0 > nRet ){
					KNFC_LOG_E("felicadev_open() down_interruptible ERR ");
						return -ERESTARTSYS;
					}
					felica_d->rsp_done = 0;
					g_flcsts = 2;
				}
					break;
				default:
					nRet = -EIO;
					g_flcsts = 0;
					KNFC_LOG_E("felicadev_open() unknown kind = %d",g_startkind);
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
				KNFC_LOG_E("felicadev_open() wait_event_interruptible ERROR = %d ",nRet);
				return -EIO;
			}
			felica_d->rsp_done = 0;
			if( 0x00 != g_carbuf[1] ){
				g_functype = -1;
				up( &chardev_sem );
				KNFC_LOG_E("felicadev_open() 0x00 != g_carbuf[1] rsp = %d ",g_carbuf[1]);
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
	KNFC_LOG_D("felicadev open SUCCESS END = %d",g_felicadev_open_cnt);
	return 0;
}

static int felicadev_release(struct inode *inode, struct file *file)
{
	struct icc_poll_data* available_d = &g_available_data;
	struct icc_poll_data *felica_d = &g_felicadev_data;
	int		nRet = -1;

	nRet = down_interruptible( &chardev_sem );
	if( 0 > nRet ){
		KNFC_LOG_E("felicadev_close() down_interruptible ERROR");
		return -ERESTARTSYS;
	}
	KNFC_LOG_D("felicadev close START");

	if( 0 < g_felicadev_open_cnt ){
		g_felicadev_open_cnt--;
		g_functype = 3;
		g_carbuf[0] = (char)g_functype;

		felicadev_poll_kick();

		nRet = wait_event_interruptible(felica_d->rsp_wait,felica_d->rsp_done == 1);
		if( nRet < 0 ){
			KNFC_LOG_E("felicadev_close() wait_event_interruptible ERROR = %d ",nRet);

			/* 1 time retry */
			usleep( 10000 ); /* 10ms */

			nRet = wait_event_interruptible(felica_d->rsp_wait,felica_d->rsp_done == 1);
			if( nRet < 0 ){
				KNFC_LOG_E("felicadev_close() wait_event_interruptible retry ERROR = %d ",nRet);
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
					KNFC_LOG_D("wake up available");
						wake_up_interruptible(&available_d->read_wait);
					}
				}
			}
		}
	}else{
		KNFC_LOG_E("felicadev close cnt ERROR = %d",g_felicadev_open_cnt);
		up( &chardev_sem );
		return -EIO;
	}
	g_functype = -1;
	up( &chardev_sem );
	KNFC_LOG_D("felicadev close SUCCESS END = %d available = %d ",g_felicadev_open_cnt,available_d->available_flag);
	return 0;
}

static int felicadev_fsync(struct file *file, loff_t start, loff_t end, int datasync)
{
	KNFC_LOG_D("felicadev fsync START");

	KNFC_LOG_D("felicadev fsync END");
	return 0;
}

static int felicadev_init(void)
{
	int				sdResult	= 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KNFC_LOG_D("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , FELICA_DEV_COUNT, FELICA_DEV_NAME);
	if (sdResult) {
		KNFC_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&felicadev_cdev, &felicadev_fileops);
	felicadev_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&felicadev_cdev, dev, FELICA_DEV_COUNT);
	if (sdResult) {
		unregister_chrdev_region(dev, FELICA_DEV_COUNT);
		KNFC_LOG_E("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}

	class_dev = device_create(knfc_class, NULL, dev, NULL, FELICA_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&felicadev_cdev);
		unregister_chrdev_region(dev, FELICA_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KNFC_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}

	memset((void*)&g_felicadev_data, 0x00, sizeof(struct icc_poll_data));

	init_waitqueue_head(&g_felicadev_data.rsp_wait);
	
	g_felicadev_data.open_flag = 0;
	g_felicadev_data.rsp_done = 0;

	KNFC_LOG_D("END");

	return sdResult;
}

static void felicadev_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KNFC_LOG_D("START");

	device_destroy(knfc_class, dev);

	cdev_del(&felicadev_cdev);
	unregister_chrdev_region(dev, FELICA_DEV_COUNT);

	KNFC_LOG_D("END");
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
			KNFC_LOG_E("NONBLOCK");
			return -EAGAIN;
		}
		KNFC_LOG_D("felicadev_poll_read wait data!");
		ret = wait_event_interruptible(felicadev_poll_d->read_wait, felicadev_poll_d->handler_done == 1);
		if (-ERESTARTSYS == ret) {
			KNFC_LOG_D("wait_event_interruptible ret = %d", ret);
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
			KNFC_LOG_E("felicadev_poll_read UNKNOWN functype");
			felicadev_poll_d->handler_done = 0;
			return -EFAULT;
	}
	if (copy_to_user(buf, g_carbuf, len)){
		KNFC_LOG_E("copy_to_user");
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
		KNFC_LOG_E("copy_from_user");
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
	KNFC_LOG_D("felicadev_poll_open START");

	felicadev_poll_d->handler_done = 0;
	felicadev_poll_d->rsp_done = 0;

	KNFC_LOG_D("felicadev_poll_open END");
	return 0;
}

static int felicadev_poll_release(struct inode *inode, struct file *file)
{
	KNFC_LOG_D("felicadev_poll_close START");

	KNFC_LOG_D("felicadev_poll_close END");
	return 0;
}


static int felicadev_poll_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KNFC_LOG_D("felicadev_poll_init() START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , FELICADEV_POLL_DEV_COUNT, FELICADEV_POLL_DEV_NAME);
	if (sdResult) {
		KNFC_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&felicadev_poll_cdev, &felicadev_poll_fileops);
	felicadev_poll_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&felicadev_poll_cdev, dev, FELICADEV_POLL_DEV_COUNT);
	if (sdResult) {
		unregister_chrdev_region(dev, FELICADEV_POLL_DEV_COUNT);
		KNFC_LOG_E("cdev_add sdResult = %d",sdResult);
		return sdResult;
	}
	
	class_dev = device_create(knfc_class, NULL, dev, NULL, FELICADEV_POLL_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&felicadev_poll_cdev);
		unregister_chrdev_region(dev, FELICADEV_POLL_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KNFC_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}
	
	memset((void*)&g_felicadev_poll_data, 0x00, sizeof(struct icc_poll_data));

	init_waitqueue_head(&g_felicadev_poll_data.read_wait);

	init_waitqueue_head(&g_felicadev_poll_data.rsp_wait);

	g_felicadev_poll_data.open_flag = 0;

	g_felicadev_poll_data.handler_done = 0;
	g_felicadev_poll_data.rsp_done = 0;

	KNFC_LOG_D("felicadev_poll_init() END");
	
	return sdResult;
}

static void felicadev_poll_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	KNFC_LOG_D("felicadev_poll_exit() START");
	
	cdev_del(&felicadev_poll_cdev);
	unregister_chrdev_region(dev, FELICADEV_POLL_DEV_COUNT);
	KNFC_LOG_D("END");
}
/************************/
/* NFC Driver cxd2235	*/
/************************/
static void cxd2235_poll_kick( void )
{
	struct icc_poll_data *cxd_poll_d = &g_cxd2235_poll_data;

	cxd_poll_d->handler_done = 1;
	wake_up_interruptible(&cxd_poll_d->read_wait);
}
static int cxd2235_chk_conflict(void)
{
	return 0;
}
static int cxd2235_open(struct inode *inode, struct file *file)
{
	int		nRet = -1;
	struct icc_poll_data *cxd_d = &g_cxd2235_data;

	KNFC_LOG_D("cxd2235_open START");

	nRet = down_interruptible( &chardev_sem );
	if( 0 > nRet ){
		return -ERESTARTSYS;
	}

	nRet = cxd2235_chk_conflict();
	if( 0 == nRet ){
		g_cxd_functype = 6;
		g_cxd_buf[0] = (char)g_cxd_functype;
		cxd2235_poll_kick();
		nRet = wait_event_interruptible(cxd_d->rsp_wait,cxd_d->rsp_done == 1);
		if( nRet < 0 ){
			KNFC_LOG_E("cxd2235_open() wait_event_interruptible ERROR = %d ",nRet);
			up( &chardev_sem );
			return -EIO;
		}
		cxd_d->rsp_done = 0;
	}
	up( &chardev_sem );
	KNFC_LOG_D("cxd2235_open END");
	return nRet;
}
static int cxd2235_release(struct inode *inode, struct file *file)
{
	int			nRet = -1;
	struct icc_poll_data *cxd_d = &g_cxd2235_data;
	KNFC_LOG_D("cxd2235_release START");

	nRet = down_interruptible( &chardev_sem );
	if( 0 > nRet ){
		return -ERESTARTSYS;
	}
	g_cxd_functype = 7;
	g_cxd_buf[0] = (char)g_cxd_functype;

	cxd2235_poll_kick();
	nRet = wait_event_interruptible(cxd_d->rsp_wait,cxd_d->rsp_done == 1);
	if( nRet < 0 ){
		KNFC_LOG_E("cxd2235_release() wait_event_interruptible ERROR = %d ",nRet);
		up( &chardev_sem );
		return -EIO;
	}

	cxd_d->rsp_done = 0;

	up( &chardev_sem );
	KNFC_LOG_D("cxd2235_release END");
	return nRet;
}
static ssize_t cxd2235_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
	int			nRet	= -1;
	short		ssize	= 0;
	struct icc_poll_data *cxd_d = &g_cxd2235_data;

	KNFC_LOG_D("cxd2235_read START");

	nRet = down_interruptible( &chardev_sem );
	if( 0 > nRet ){
		return -ERESTARTSYS;
	}
	g_cxd_functype = 8;
	ssize	= len;

	g_cxd_buf[0] = (char)g_cxd_functype;
	g_cxd_buf[1] = 0xFA;
	g_cxd_buf[2] = ((ssize >> 8) & CXD2235_SIZE_MASK);
	g_cxd_buf[3] = (ssize & CXD2235_SIZE_MASK);

	cxd2235_poll_kick();

	nRet = wait_event_interruptible(cxd_d->rsp_wait,cxd_d->rsp_done == 1);
	if( nRet < 0 ){
		KNFC_LOG_E("cxd2235_read() wait_event_interruptible ERROR = %d ",nRet);
		up( &chardev_sem );
		return -EIO;
	}

	cxd_d->rsp_done = 0;

	if( 0x00 == g_cxd_buf[1] ){
		ssize = g_cxd_buf[2];
		ssize = ssize << 8;
		ssize = ssize + g_cxd_buf[3];
		if( len != ssize ){
			KNFC_LOG_E("size unmatch len = %d ssize = %d",(int)len,ssize);
			if( len < ssize ){
				up( &chardev_sem );
				return -EFAULT;
			}
		}
		if (copy_to_user(buf, &g_cxd_buf[4], ssize)){
			KNFC_LOG_E("copy_to_user");
			up( &chardev_sem );
			return -EFAULT;
		}
		nRet = ssize;
	}else{
		nRet = -EIO;
	}
	up( &chardev_sem );
	KNFC_LOG_D("cxd2235_read END");
	return nRet;
}
static ssize_t cxd2235_write(struct file *file, const char __user *data, size_t len, loff_t *ppos)
{
	int			nRet	= -1;
	short		ssize	= 0;
	short		sRetSize = 0;
	struct icc_poll_data *cxd_d = &g_cxd2235_data;

	KNFC_LOG_D("cxd2235_write START");

	nRet = down_interruptible( &chardev_sem );
	if( 0 > nRet ){
		return -ERESTARTSYS;
	}

	g_cxd_functype	= 9;
	ssize			= len;

	g_cxd_buf[0] = (char)g_cxd_functype;
	g_cxd_buf[2] = ((ssize >> 8) & FELICA_DEV_SIZE_MASK);
	g_cxd_buf[3] = (ssize & FELICA_DEV_SIZE_MASK);
	if (copy_from_user(&g_cxd_buf[4], data, ssize)) {
		KNFC_LOG_E("copy_from_user");
		up( &chardev_sem );
		return -EFAULT;
	}

	cxd2235_poll_kick();
	nRet = wait_event_interruptible(cxd_d->rsp_wait,cxd_d->rsp_done == 1);
	if( nRet < 0 ){
		KNFC_LOG_E("cxd2235_read() wait_event_interruptible ERROR = %d ",nRet);
		up( &chardev_sem );
		return -EIO;
	}

	cxd_d->rsp_done = 0;

	if( 0x00 == g_cxd_buf[1] ){
		sRetSize = g_cxd_buf[2];
		sRetSize = sRetSize << 8;
		sRetSize = g_cxd_buf[3];
	}
	up( &chardev_sem );
	KNFC_LOG_D("cxd2235_write END size = %d",sRetSize);
	return sRetSize;
}
static long cxd2235_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long			nRet		= -1;
	short			sstructsize	= 0;
	struct icc_poll_data *cxd_d	= &g_cxd2235_data;

	KNFC_LOG_D("cxd2235_ioctl START arg = 0x%lx",arg);

	nRet = down_interruptible( &chardev_sem );
	if( 0 > nRet ){
		return -ERESTARTSYS;
	}
	switch( cmd ){
		case TCFLSH:
			g_cxd_functype = 13;
			g_cxd_buf[0] = (char)g_cxd_functype;
			g_cxd_buf[2] =0;
			g_cxd_buf[3] =1;
			g_cxd_buf[4] =(unsigned char)arg;
			cxd2235_poll_kick();
			nRet = wait_event_interruptible(cxd_d->rsp_wait,cxd_d->rsp_done == 1);
			if( nRet < 0 ){
				KNFC_LOG_E("cxd2235_ioctl() wait_event_interruptible ERROR = %d ",(int)nRet);
				up( &chardev_sem );
				return -EIO;
			}
			cxd_d->rsp_done = 0;
			if( 0x00 == g_cxd_buf[1] ){
				nRet = 0;
			}else{
				nRet = -1;
			}
			break;
		case TCSANOW:
		case TCSETS:
			g_cxd_functype = 10;
			g_cxd_buf[0] = (char)g_cxd_functype;
			
			sstructsize = sizeof(struct termios);
			KNFC_LOG_D("cxd2235_ioctl termios size = %d",sstructsize);

			g_cxd_buf[2] = (( sstructsize >> 8 )& CXD2235_SIZE_MASK);
			g_cxd_buf[3] = ( sstructsize & CXD2235_SIZE_MASK );

			if (copy_from_user(&g_cxd_buf[4], (void __user *)arg, sizeof(struct termios))) {
				KNFC_LOG_E("copy_from_user");
				up( &chardev_sem );
				return -EFAULT;
			}
			cxd2235_poll_kick();

			nRet = wait_event_interruptible(cxd_d->rsp_wait,cxd_d->rsp_done == 1);
			if( nRet < 0 ){
				KNFC_LOG_E("cxd2235_ioctl() wait_event_interruptible ERROR = %d ",(int)nRet);
				up( &chardev_sem );
				return -EIO;
			}

			cxd_d->rsp_done = 0;

			break;
		case CXD2235_DBG_POLLING:
			g_cxd_functype = 12;
			g_cxd_buf[0] = (char)g_cxd_functype;
			g_cxd_buf[2] =0;
			g_cxd_buf[3] =1;
			g_cxd_buf[4] =(unsigned char)arg;
			
			cxd2235_poll_kick();
			nRet = wait_event_interruptible(cxd_d->rsp_wait,cxd_d->rsp_done == 1);
			if( nRet < 0 ){
				KNFC_LOG_E("cxd2235_ioctl() wait_event_interruptible ERROR = %d ",(int)nRet);
				up( &chardev_sem );
				return -EIO;
			}
			cxd_d->rsp_done = 0;

			if( g_cxd_buf[1] == 0x00 ){
				nRet = 0;
			}else{
				switch( g_cxd_buf[1] ){
					case 1:
						nRet = 1;
						break;
					case 2:
						nRet = 2;
						break;
					case 3:
						nRet = 3;
						break;
					case 4:
						nRet = 4;
						break;
					default:
						nRet = -EIO;
						break;
				}
			}
			break;
		default:
			nRet = -EINVAL;
			break;
	}
	up( &chardev_sem );
	KNFC_LOG_D("cxd2235_ioctl END");
	return nRet;
}
static unsigned int cxd2235_poll(struct file *file, poll_table *wait)
{
	unsigned int			mask = 0;
	int						nRet = 0;

	struct icc_poll_data *cxd_d	= &g_cxd2235_data;

	nRet = down_interruptible( &chardev_sem );
	if( 0 > nRet ){
		return -ERESTARTSYS;
	}
	g_cxd_functype = 11;
	g_cxd_buf[0] = (char)g_cxd_functype;

	cxd2235_poll_kick();
	nRet = wait_event_interruptible(cxd_d->rsp_wait,cxd_d->rsp_done == 1);
	if( nRet < 0 ){
		KNFC_LOG_E("cxd2235_ioctl() wait_event_interruptible ERROR = %d ",nRet);
		up( &chardev_sem );
		return -EIO;
	}
	cxd_d->rsp_done = 0;

	poll_wait(file, &cxd_d->dummy_wait, wait);
	if( 0x00 == g_cxd_buf[1] ){
		mask = POLLIN | POLLRDNORM;
	}else{
	}
	up( &chardev_sem );
	return mask;
}

static int cxd2235_init(void)
{
	int           	sdResult      = 0;
	struct device*	class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KNFC_LOG_D("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , CXD2235_DEV_COUNT, CXD2235_DEV_NAME);
	if (sdResult) {
		KNFC_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&cxd2235_cdev, &cxd2235_fileops);
	cxd2235_cdev.owner = THIS_MODULE;

	sdResult = cdev_add(&cxd2235_cdev, dev, CXD2235_DEV_COUNT);
	if (sdResult) {
		unregister_chrdev_region(dev, CXD2235_DEV_COUNT);
		KNFC_LOG_E("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}

	class_dev = device_create(knfc_class, NULL, dev, NULL, CXD2235_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&cxd2235_cdev);
		unregister_chrdev_region(dev, CXD2235_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KNFC_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}

	memset((void*)&g_cxd2235_data, 0x00, sizeof(struct icc_poll_data));

	sema_init( &chardev_sem, 1 );

	init_waitqueue_head(&g_cxd2235_data.rsp_wait);

	init_waitqueue_head(&g_cxd2235_data.dummy_wait);
	
	g_cxd2235_data.open_flag = 0;

	KNFC_LOG_D("END");

	return sdResult;
}

static void cxd2235_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KNFC_LOG_D("START");

	device_destroy(knfc_class, dev);

	cdev_del(&cxd2235_cdev);
	unregister_chrdev_region(dev, CXD2235_DEV_COUNT);

	KNFC_LOG_D("END");
}
/********************/
/*	cxd2235_poll	*/
/********************/
static ssize_t cxd2235_poll_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
	struct icc_poll_data *cxd_poll_d = &g_cxd2235_poll_data;
	int		ret;

	if (0 == cxd_poll_d->handler_done) {
		if (file->f_flags & O_NONBLOCK) {
			KNFC_LOG_E("NONBLOCK");
			return -EAGAIN;
		}
		KNFC_LOG_D("cxd2235_poll_read wait data!");
		ret = wait_event_interruptible(cxd_poll_d->read_wait, cxd_poll_d->handler_done == 1);
		if (-ERESTARTSYS == ret) {
			KNFC_LOG_D("wait_event_interruptible ret = %d", ret);
			return -EINTR;
		}
	}
	switch( g_cxd_functype ){
		case 6:
			len = 1;
			break;
		case 7:
			len = 1;
			break;
		case 8:
			len = 4;
			break;
		case 9:
			len = g_cxd_buf[2];
			len = len << 8;
			len = len + g_cxd_buf[3];
			len = len +4;
			break;
		case 10:
			len = sizeof(struct termios) + 4;
			break;
		case 11:
			len = 1;
			break;
		case 12:
			len = 5;
			break;
		default:
			KNFC_LOG_E("cxd2235_poll_read UNKNOWN functype");
			return -EFAULT;
	}
	if (copy_to_user(buf, g_cxd_buf, len)){
		KNFC_LOG_E("copy_to_user");
		return -EFAULT;
	}
	cxd_poll_d->handler_done = 0;
	
	return len;
}
static ssize_t cxd2235_poll_write(struct file *file, const char __user *data, size_t len, loff_t *ppos)
{
	struct icc_poll_data *cxd_d = &g_cxd2235_data;

	g_cxd_buf[0] = 0x00;
	g_cxd_buf[1] = 0x00;
	g_cxd_buf[2] = 0x00;
	g_cxd_buf[3] = 0x00;

	if (copy_from_user(g_cxd_buf, data, len)) {
		KNFC_LOG_E("copy_from_user");
		return -EFAULT;
	}
	cxd_d->rsp_done = 1;
	wake_up_interruptible_sync(&cxd_d->rsp_wait);

	return 0;
}
static unsigned int cxd2235_poll_poll(struct file *file, poll_table *wait)
{
	struct icc_poll_data *cxd_poll_d = &g_cxd2235_poll_data;
	unsigned int mask = 0;

	poll_wait(file, &cxd_poll_d->read_wait, wait);
	if (0 != cxd_poll_d->handler_done){
		mask = POLLIN | POLLRDNORM;
	}

	return (mask);
}
static int cxd2235_poll_open(struct inode *inode, struct file *file)
{
	struct icc_poll_data *cxd_poll_d = &g_cxd2235_poll_data;
	KNFC_LOG_D("cxd2235_poll_open START");

	cxd_poll_d->handler_done	= 0;
	cxd_poll_d->rsp_done		= 0;

	KNFC_LOG_D("cxd2235_poll_open END");
	return 0;
}
static int cxd2235_poll_release(struct inode *inode, struct file *file)
{
	KNFC_LOG_D("cxd2235_poll_release START");
	
	KNFC_LOG_D("cxd2235_poll_release END");
	return 0;
}


static int cxd2235_poll_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KNFC_LOG_D("cxd2235_poll_init() START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , CXD2235_POLL_DEV_COUNT, CXD2235_POLL_DEV_NAME);
	if (sdResult) {
		KNFC_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}

	cdev_init(&cxd2235_poll_cdev, &cxd2235_poll_fileops);
	cxd2235_poll_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&cxd2235_poll_cdev, dev, CXD2235_POLL_DEV_COUNT);
	if (sdResult) {
		unregister_chrdev_region(dev, CXD2235_POLL_DEV_COUNT);
		KNFC_LOG_E("cdev_add sdResult = %d",sdResult);
		return sdResult;
	}
	
	class_dev = device_create(knfc_class, NULL, dev, NULL, CXD2235_POLL_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&cxd2235_poll_cdev);
		unregister_chrdev_region(dev, CXD2235_POLL_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KNFC_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}
	
	memset((void*)&g_cxd2235_poll_data, 0x00, sizeof(struct icc_poll_data));

	init_waitqueue_head(&g_cxd2235_poll_data.read_wait);

	g_cxd2235_poll_data.open_flag = 0;
	
	KNFC_LOG_D("cxd2235_poll_init() END");
	
	return sdResult;
}

static void cxd2235_poll_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	KNFC_LOG_D("cxd2235_poll_exit() START");
	
	cdev_del(&cxd2235_poll_cdev);
	unregister_chrdev_region(dev, CXD2235_POLL_DEV_COUNT);
	KNFC_LOG_D("cxd2235_poll_exit() END");
}
/* uart switch */
static long uart_switch_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long					nRet	= 0;
	int						buff	= 0;
	struct icc_poll_data *felica_d = &g_felicadev_data;

	KNFC_LOG_D("uart_switch_ioctl() START  " );

	switch( cmd ){
		case UART_SWITCH_SET_FLC_STS:
			KNFC_LOG_D("uart_switch_ioctl() UART_SWITCH_SET_FLC_STS  " );
			break;
		case UART_SWITCH_SET_START_KIND:
			nRet = down_interruptible( &switch_sem );
			if( 0 > nRet ){
				return -ERESTARTSYS;
			}
			KNFC_LOG_D("uart_switch_ioctl() UART_SWITCH_SET_START_KIND  " );
			if (copy_from_user(&buff, (void __user *)arg, sizeof(buff))) {
				KNFC_LOG_E("uart_switch_ioctl() copy_from_user err");
				up( &switch_sem );
				return -EFAULT;
			}
			g_startkind = buff;
			if( 255 == buff ){
				if( 1 == g_flcsts ){
					felica_d->rsp_done = 1;
					wake_up_interruptible(&felica_d->rsp_wait);
				}
			}
			up( &switch_sem );
			break;
		case UART_SWITCH_GET_FLC_STS:
			nRet = down_interruptible( &switch_sem );
			if( 0 > nRet ){
				return -ERESTARTSYS;
			}
			KNFC_LOG_D("uart_switch_ioctl() UART_SWITCH_GET_FLC_STS  " );
			buff = g_flcsts;
			if (copy_to_user((int __user*)arg, &buff, sizeof(buff))) {
				KNFC_LOG_E("uart_switch_ioctl() copy_to_user err");
				up( &switch_sem );
				return -EFAULT;
			}
			up( &switch_sem );
			break;
		case UART_SWTICH_GET_START_KIND:
			KNFC_LOG_D("uart_switch_ioctl() UART_SWTICH_GET_START_KIND  " );
			break;
		default:
			KNFC_LOG_D("uart_switch_ioctl() OTHER  " );
			nRet = -EIO;
			break;
	}
	KNFC_LOG_D("uart_switch_ioctl() END");

	return nRet;
}

static int uart_switch_open(struct inode *inode, struct file *file)
{
	int				nRet = 0;

	KNFC_LOG_D("uart_switch_open() START");


	KNFC_LOG_D("uart_switch_open() END");
	return nRet;
}

static int uart_switch_release(struct inode *inode, struct file *file)
{
	KNFC_LOG_D("uart_switch_release() START");

	KNFC_LOG_D("uart_switch_release() END");

	return 0;
}



static int uart_switch_init(void)
{
	int sdResult = 0;
	struct device *class_dev;
	
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KNFC_LOG_D("uart_switch_init() START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , UART_SWITCH_DEV_COUNT, UART_SWITCH_DEV_NAME);
	if (sdResult) {
		KNFC_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}

	cdev_init(&uart_switch_cdev, &uart_switch_fileops);
	uart_switch_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&uart_switch_cdev, dev, UART_SWITCH_DEV_COUNT);
	if (sdResult) {
		unregister_chrdev_region(dev, UART_SWITCH_DEV_COUNT);
		KNFC_LOG_E("cdev_add sdResult = %d",sdResult);
		return sdResult;
	}
	
	class_dev = device_create(knfc_class, NULL, dev, NULL, UART_SWITCH_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&uart_switch_cdev);
		unregister_chrdev_region(dev, UART_SWITCH_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KNFC_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}

	sema_init( &switch_sem, 1 );

	KNFC_LOG_D("cxd2235_poll_init() END");
	
	return sdResult;
}

static void uart_switch_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	KNFC_LOG_D("uart_switch_exit() START");
	
	cdev_del(&uart_switch_cdev);
	unregister_chrdev_region(dev, UART_SWITCH_DEV_COUNT);
	KNFC_LOG_D("uart_switch_exit() END");
}



/*
 * cxd2235 power
 */
static ssize_t cxd2235power_write(struct file *file, const char __user *data, size_t len, loff_t *ppos)
{
	char on;

	KNFC_LOG_D("START");

	if ( len < 1 ) {
		KNFC_LOG_E("length check len = %d", (int)len);
		return -EIO;
	}
	len = 1;
	if (copy_from_user(&on, data, len)) {
		KNFC_LOG_E("copy_from_user");
		return -EFAULT;
	}

	gpio_set_value(g_nfc_pon_gpio , on );

	KNFC_LOG_D("END on = %d", on);
	
	return len;
}
static int cxd2235power_open(struct inode *inode, struct file *file)
{
	int				nRet = 0;

	KNFC_LOG_D("START");

	KNFC_LOG_D("END");
	return nRet;
}

static int cxd2235power_release(struct inode *inode, struct file *file)
{
	KNFC_LOG_D("START");

	gpio_set_value(g_nfc_pon_gpio , PON_DEV_LOW);

	KNFC_LOG_D("END");

	return 0;
}

static int cxd2235power_init(void)
{
	int				sdResult = 0;
	struct device*	class_dev;

	dev_t dev = MKDEV(MISC_MAJOR, 0);

	KNFC_LOG_D("START");
	
	sdResult = alloc_chrdev_region(&dev , 0 , CXD2235_POWER_DEV_COUNT, CXD2235_POWER_DEV_NAME);
	if (sdResult) {
		KNFC_LOG_E("alloc_chrdev_region sdResult = %d", sdResult);
		return sdResult;
	}
	
	cdev_init(&cxd2235power_cdev, &cxd2235power_fileops);
	cxd2235power_cdev.owner = THIS_MODULE;
	
	sdResult = cdev_add(&cxd2235power_cdev, dev, CXD2235_POWER_DEV_COUNT);
	if (sdResult) {
		unregister_chrdev_region(dev, CXD2235_POWER_DEV_COUNT);
		KNFC_LOG_E("cdev_add sdResult = %d", sdResult);
		return sdResult;
	}
	
	class_dev = device_create(knfc_class, NULL, dev, NULL, CXD2235_POWER_DEV_NAME);
	if (IS_ERR(class_dev)) {
		cdev_del(&cxd2235power_cdev);
		unregister_chrdev_region(dev, CXD2235_POWER_DEV_COUNT);
		sdResult = PTR_ERR(class_dev);
		KNFC_LOG_E("device_create sdResult = %d", sdResult);
		return sdResult;
	}

	KNFC_LOG_D("END");
	
	return sdResult;
}

static void cxd2235power_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KNFC_LOG_D("START");

	device_destroy(knfc_class, dev);

	cdev_del(&cxd2235power_cdev);
	unregister_chrdev_region(dev, CXD2235_POWER_DEV_COUNT);

	KNFC_LOG_D("END");
}

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
	
	KNFC_LOG_D("START");
	
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
	
	KNFC_LOG_D("END");
	
	return sdResult;
}

static void knfcvbus_exit(void)
{
	dev_t dev = MKDEV(MISC_MAJOR, 0);
	
	KNFC_LOG_D("START");

	device_destroy(knfc_class, dev);

	cdev_del(&knfcvbus_cdev);
	unregister_chrdev_region(dev, VBUS_DEV_COUNT);

	KNFC_LOG_D("END");
}

#ifdef CONFIG_OF
static struct of_device_id i2c_nfc_table[] = {
	{ .compatible = "kc,nfc_i2c"},
	{ },
};
#else
#define i2c_nfc_table NULL
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
		.of_match_table = i2c_nfc_table,
	},
	.probe	 = cen_probe,
	.id_table = cen_id,
	.remove	 = cen_remove,
};

/*
 * knfc_init
 */
static __init int knfc_init(void)
{
	int ret;

	KNFC_LOG_D("START");
	knfc_class = class_create(THIS_MODULE, "knfc");
	
	if (IS_ERR(knfc_class)) {
		return PTR_ERR(knfc_class);
	}

	ret = platform_driver_register(&msm8974_nfc_driver);
	if( ret < 0 ){
		KNFC_LOG_E("platform_driver_register err\n");
		return ret;
	}

	ret = rate_init();
	if( ret < 0 ){
		KNFC_LOG_E("rate init err\n");
		return ret;
	}

	ret = pon_init();
	if( ret < 0 ){
		KNFC_LOG_E("pon init err\n");
		return ret;
	}

#ifdef ENABLED_NFC_VFC_RST_DEV
	ret = nfc_vfc_rst_init();
	if( ret < 0 ){
		KNFC_LOG_E("nfc_vfc_rst init err\n");
		return ret;
	}
#endif

	ret = rfs_init();
	if( ret < 0 ){
		KNFC_LOG_E("rfs init err\n");
		return ret;
	}

	ret = i2c_add_driver(&s7780a_driver);
	if( ret < 0 ){
		KNFC_LOG_E("i2c add driver err\n");
		return ret;
	}

	ret = int_poll_init();
	if( ret < 0 ){
		KNFC_LOG_E("int_poll_init err = %d\n",ret);
		return ret;
	}
	ret = rws_init();
	if( ret < 0 ){
		KNFC_LOG_E("rws_init err = %d\n",ret);
		return ret;
	}

	ret = intu_poll_init();
	if( ret < 0 ){
		KNFC_LOG_E("intu_poll_init = %d\n",ret);
		return ret;
	}
	ret = hsel_init();
	if( ret < 0 ){
		KNFC_LOG_E("hsel_init err = %d\n",ret);
		return ret;
	}
	ret = available_poll_init();
	if( ret < 0 ){
		KNFC_LOG_E("available_poll_init err = %d\n",ret);
		return ret;
	}
	
	ret = felicadev_init();
	if( ret < 0 ){
		KNFC_LOG_E("felicadev_init err = %d\n",ret);
		return ret;
	}
	
	ret = felicadev_poll_init();
	if( ret < 0 ){
		KNFC_LOG_E("felicadev_poll_init err = %d\n",ret);
		return ret;
	}
	ret = cxd2235_init();
	if( ret < 0 ){
		KNFC_LOG_E("cxd2235_init err = %d\n",ret);
		return ret;
	}
	ret = cxd2235_poll_init();
	if( ret < 0 ){
		KNFC_LOG_E("cxd2235_poll_init err = %d\n",ret);
		return ret;
	}
	ret = uart_switch_init();
	if( ret < 0 ){
		KNFC_LOG_E("uart_switch_init err = %d\n",ret);
		return ret;
	}
	ret = cxd2235power_init();
	if( ret < 0 ){
		KNFC_LOG_E("cxd2235power_init err = %d\n",ret);
		return ret;
	}
	ret = rfs_poll_init();
	if( ret < 0 ){
		KNFC_LOG_E("rfs_poll_init err = %d\n",ret);
		return ret;
	}

	ret = knfcvbus_init();
	if( ret < 0 ){
		KNFC_LOG_E("knfcvbus_init err = %d\n",ret);
	}

	KNFC_LOG_D("END");
	return 0;
}

/*
 * knfc_exit
 */
static void __exit knfc_exit(void)
{

	class_destroy( knfc_class );

	platform_driver_unregister(&msm8974_nfc_driver);

	rate_exit();

	pon_exit();

#ifdef ENABLED_NFC_VFC_RST_DEV
	nfc_vfc_rst_exit();
#endif

	rfs_exit();

	i2c_del_driver(&s7780a_driver);

	int_poll_exit();

	rws_exit();

	intu_poll_exit();

	hsel_exit();
	
	available_poll_exit();
	
	felicadev_exit();
	
	felicadev_poll_exit();
	
	cxd2235_exit();
	
	cxd2235_poll_exit();

	uart_switch_exit();

	cxd2235power_exit();

	rfs_poll_exit();

	knfcvbus_exit();

	return;
}


static int pm_nfc_probe(struct platform_device *pdev)
{
	
	int rc = 0;
	
	KNFC_LOG_D("START");

	if (!pdev->dev.of_node) {
		pr_err("No platform supplied from device tree.\n");
		return -EINVAL;
	}


	g_nfc_pon_gpio = of_get_named_gpio(pdev->dev.of_node, "nfc_pon-gpio", 0);
	if (g_nfc_pon_gpio < 0) {
		pr_err("%s: of_get_named_gpio failed.\n", __func__);
		return -EINVAL;
	}

	rc = gpio_request(g_nfc_pon_gpio, "nfc_pon-gpio");
	if (rc) {
		pr_err("%s: gpio_request failed.\n", __func__);
		return -EINVAL;
	}
	
	g_nfc_hsel_gpio = of_get_named_gpio(pdev->dev.of_node, "nfc_hsel-gpio", 0);
	if (g_nfc_hsel_gpio < 0) {
		pr_err("%s: of_get_named_gpio failed.\n", __func__);
		return -EINVAL;
	}

	rc = gpio_request(g_nfc_hsel_gpio, "nfc_hsel-gpio");
	if (rc) {
		pr_err("%s: gpio_request failed.\n", __func__);
		return -EINVAL;
	}
	
	g_nfc_intu_gpio = of_get_named_gpio(pdev->dev.of_node, "nfc_intu-gpio", 0);
	if (g_nfc_intu_gpio < 0) {
		pr_err("%s: of_get_named_gpio failed.\n", __func__);
		return -EINVAL;
	}

	rc = gpio_request(g_nfc_intu_gpio, "nfc_intu-gpio");
	if (rc) {
		pr_err("%s: gpio_request failed.\n", __func__);
		return -EINVAL;
	}
#ifdef ENABLED_NFC_VFC_RST_DEV
	g_nfc_vfc_gpio = of_get_named_gpio(pdev->dev.of_node, "nfc_vfc-gpio", 0);
	if (g_nfc_vfc_gpio < 0) {
		pr_err("%s: of_get_named_gpio failed.\n", __func__);
		return -EINVAL;
	}

	rc = gpio_request(g_nfc_vfc_gpio, "nfc_vfc-gpio");
	if (rc) {
		pr_err("%s: gpio_request failed.\n", __func__);
		return -EINVAL;
	}
#endif
	g_nfc_rfs_gpio = of_get_named_gpio(pdev->dev.of_node, "nfc_rfs-gpio", 0);
	if (g_nfc_rfs_gpio < 0) {
		pr_err("%s: of_get_named_gpio failed.\n", __func__);
		return -EINVAL;
	}

	rc = gpio_request(g_nfc_rfs_gpio, "nfc_rfs-gpio");
	if (rc) {
		pr_err("%s: gpio_request failed.\n", __func__);
		return -EINVAL;
	}

	g_nfc_int_gpio = of_get_named_gpio(pdev->dev.of_node, "nfc_int-gpio", 0);
	if (g_nfc_int_gpio < 0) {
		pr_err("%s: of_get_named_gpio failed.\n", __func__);
		return -EINVAL;
	}

	rc = gpio_request(g_nfc_int_gpio, "nfc_int-gpio");
	if (rc) {
		pr_err("%s: gpio_request failed.\n", __func__);
		return -EINVAL;
	}

	KNFC_LOG_D("END");
	return 0;
}

static int pm_nfc_remove(struct platform_device *pdev)
{
	gpio_free(g_nfc_rfs_gpio);
	gpio_free(g_nfc_int_gpio);

	gpio_free(g_nfc_pon_gpio);
	gpio_free(g_nfc_hsel_gpio);
	gpio_free(g_nfc_intu_gpio);

#ifdef ENABLED_NFC_VFC_RST_DEV
	gpio_free(g_nfc_vfc_gpio);
#endif
	return 0;
}
static int pm_nfc_suspend (struct platform_device *pdev, pm_message_t state)
{
	KNFC_LOG_D("pm_nfc_suspend Start");
//	gpio_tlmm_config(GPIO_CFG(MFD_PON_GPIO_NUM, 0, GPIO_CFG_OUTPUT,GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	KNFC_LOG_D("pm_nfc_suspend End");
	return 0;
}

static int pm_nfc_resume (struct platform_device *pdev)
{
	KNFC_LOG_D("pm_nfc_resume Start");
//	gpio_tlmm_config(GPIO_CFG(MFD_PON_GPIO_NUM, 0, GPIO_CFG_OUTPUT,GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	KNFC_LOG_D("pm_nfc_resume End");
	return 0;
}

MODULE_LICENSE("GPL v2");

module_init(knfc_init);
module_exit(knfc_exit);

