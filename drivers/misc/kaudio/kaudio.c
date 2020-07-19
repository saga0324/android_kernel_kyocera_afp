/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 * (C) 2017 KYOCERA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <../sound/soc/codecs/msm8x16_wcd_registers.h>
#include <../sound/soc/codecs/msm8x16-wcd.h>
#include <sound/soc.h>
#include <linux/kaudio_api.h>
#include "kaudio.h"

#define MINOR_NUMBER 1
#define KAUDIO_DEFAULT_SIDETONE_VOLUME (-84)

int wcd_mbhc_get_jack_status( void );
void wcd9xxx_spmi_set_SYSTEM_RESUME_TIMEOUT( unsigned int );

typedef struct {
	struct cdev*	cdev;
	struct device*	dev;
	int				major;
} kaudio_device_t;

static kaudio_device_t*	g_kaudio_dev;
static struct class*	g_kaudio_class;
static dev_t			g_kaudio_device_num;
static struct work_struct g_kaudio_sidetone_mute;
static struct work_struct g_kaudio_sidetone_unmute;
static struct snd_soc_codec *g_kaudio_codec = NULL;
static int g_kaudio_sidetone_volume = KAUDIO_DEFAULT_SIDETONE_VOLUME;
static int g_kaudio_sidetone_mixer_val = 0;
static bool g_kaudio_is_close = false;
static int g_kaudio_reg;
static struct mutex kaudio_lock;
bool klog_is_enable[KLOG_FILE_NO_MAX] = {0};

#ifdef pr_debug
#undef pr_debug
bool klog_is_enable[];
#define pr_debug(fmt, ...) \
({	\
	if(klog_is_enable[1])\
		printk(fmt, ##__VA_ARGS__);\
	else\
		dynamic_pr_debug(fmt, ##__VA_ARGS__);\
})
#endif

/*===========================================================================

FUNCTION : kaudio_set_sidetone

===========================================================================*/
static void kaudio_set_sidetone_mute(struct work_struct *work)
{
	pr_debug("%s: start\n", __func__);

	mutex_lock(&kaudio_lock);
	g_kaudio_is_close = true;

	if (!g_kaudio_codec) {
		pr_err("%s: NULL codec pointer\n", __func__);
		mutex_unlock(&kaudio_lock);
		return ;
	}

	if (g_kaudio_sidetone_volume != KAUDIO_DEFAULT_SIDETONE_VOLUME) {
		pr_debug("%s: set   mute\n", __func__);
		snd_soc_write(g_kaudio_codec, g_kaudio_reg, (unsigned int)KAUDIO_DEFAULT_SIDETONE_VOLUME);
	}
	mutex_unlock(&kaudio_lock);

	pr_debug("%s: leave\n", __func__);
}

static void kaudio_set_sidetone_unmute(struct work_struct *work)
{
	pr_debug("%s: start\n", __func__);

	mutex_lock(&kaudio_lock);
	g_kaudio_is_close = false;

	if (!g_kaudio_codec) {
		pr_err("%s: NULL codec pointer\n", __func__);
		mutex_unlock(&kaudio_lock);
		return ;
	}

	if (g_kaudio_sidetone_volume != KAUDIO_DEFAULT_SIDETONE_VOLUME) {
		pr_debug("%s: set unmute\n", __func__);
		snd_soc_write(g_kaudio_codec, g_kaudio_reg, (unsigned int)g_kaudio_sidetone_volume);
	}
	mutex_unlock(&kaudio_lock);

	pr_debug("%s: leave\n", __func__);
}

/*===========================================================================

FUNCTION : kaudio_wcd_sidetone_unmute_get

===========================================================================*/
static int kaudio_wcd_sidetone_unmute_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = g_kaudio_sidetone_mixer_val;

	pr_debug("%s: value 0x%x\n", __func__, g_kaudio_sidetone_mixer_val);
	return 0;
}

/*===========================================================================

FUNCTION : kaudio_wcd_sidetone_unmute_put

===========================================================================*/
static int kaudio_wcd_sidetone_unmute_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.integer.value[0];

	pr_debug("%s: value 0x%x\n", __func__, value);

	mutex_lock(&kaudio_lock);
	g_kaudio_reg = ((struct soc_multi_mixer_control *) kcontrol->private_value)->reg;
	g_kaudio_sidetone_mixer_val = value;
	g_kaudio_sidetone_volume = g_kaudio_sidetone_mixer_val + KAUDIO_DEFAULT_SIDETONE_VOLUME;
	if (!g_kaudio_is_close || 
		(g_kaudio_sidetone_volume == KAUDIO_DEFAULT_SIDETONE_VOLUME)) {
		snd_soc_write(g_kaudio_codec, g_kaudio_reg, (unsigned int)g_kaudio_sidetone_volume);
	}
	mutex_unlock(&kaudio_lock);

	return 0;
}

static const struct snd_kcontrol_new kaudio_wcd_controls[] = {
	SOC_SINGLE_EXT("Sidetone Volume", MSM8X16_WCD_A_CDC_IIR1_GAIN_B1_CTL, 0, 40, 0,
	kaudio_wcd_sidetone_unmute_get,
	kaudio_wcd_sidetone_unmute_put),
};

/*===========================================================================

FUNCTION : kaudio_wcd_init

===========================================================================*/
int kaudio_wcd_init(struct snd_soc_codec *codec)
{
	g_kaudio_codec = codec;
	pr_debug("%s \n", __func__);
	snd_soc_add_codec_controls(codec, kaudio_wcd_controls,
				   ARRAY_SIZE(kaudio_wcd_controls));

	mutex_init(&kaudio_lock);
	INIT_WORK(&g_kaudio_sidetone_mute, kaudio_set_sidetone_mute);
	INIT_WORK(&g_kaudio_sidetone_unmute, kaudio_set_sidetone_unmute);

	return 0;
}

/*===========================================================================

FUNCTION : kaudio_codec_sidetone_mute

===========================================================================*/
int kaudio_codec_sidetone_mute( int mute )
{
	int ret = 0;
	pr_debug("%s: Sidetone Mute val = %d\n", __func__, mute);

	switch (mute){
	case KAUDIO_SIDETONE_UNMUTE:
		pr_debug("%s: scheduling g_kaudio_sidetone_unmute\n", __func__);
		schedule_work(&g_kaudio_sidetone_unmute);
		break;

	case KAUDIO_SIDETONE_MUTE:
		pr_debug("%s: scheduling g_kaudio_sidetone_mute\n", __func__);
		schedule_work(&g_kaudio_sidetone_mute);
		break;

	default:
		pr_err("%s: Invalid arguments: %d\n", __func__, mute);
		ret = -EINVAL;
		break;
	}

	return ret;
}

EXPORT_SYMBOL(kaudio_codec_sidetone_mute);

/*===========================================================================

FUNCTION : kaudio_open

===========================================================================*/
static int kaudio_open( struct inode *inode, struct file *file )
{
	int							w_rc;
/**/
	pr_debug("%s\n", __func__);

	w_rc = 0;

	return w_rc;
}

static int kaudio_min_bytes_needed(unsigned long val)
{
	int c = 0;
	int i;

	for (i = (sizeof val * 8) - 1; i >= 0; --i, ++c)
		if (val & (1UL << i))
			break;
	c = (sizeof val * 8) - c;
	if (!c || (c % 8))
		c = (c + 8) / 8;
	else
		c /= 8;
	return c;
}

static ssize_t kaudio_soc_codec_reg_show(struct snd_soc_codec *codec)
{
	int i, step = 1;
	int wordsize = kaudio_min_bytes_needed(codec->driver->reg_cache_size) * 2;
	int regsize = codec->driver->reg_word_size * 2;
	int len = wordsize + regsize + 2 + 1;
	int ret = 0;
	size_t total = 0;
	size_t count = PAGE_SIZE;
	char tmpbuf[len + 1];
	char regbuf[regsize + 1];

	if (!codec->driver->reg_cache_size)
		return 0;

	if (codec->driver->reg_cache_step)
		step = codec->driver->reg_cache_step;

	for (i = 0; i < codec->driver->reg_cache_size; i += step) {
		if (!snd_soc_codec_readable_register(codec, i))
			continue;
		if (total + len >= count - 1)
			break;
		ret = snd_soc_read(codec, i);
		if (ret < 0) {
			memset(regbuf, 'X', regsize);
			regbuf[regsize] = '\0';
		} else {
			snprintf(regbuf, regsize + 1, "%.*x", regsize, ret);
		}
		/* prepare the buffer */
		snprintf(tmpbuf, len + 1, "%.*x: %s", wordsize, i, regbuf);
		pr_err("%s: DM_codec_reg: %s\n", __func__, tmpbuf);
		total += len;
	}

	total = min(total, count - 1);

	return total;
}

/*===========================================================================

FUNCTION : kaudio_ioctl

===========================================================================*/
static long kaudio_ioctl( struct file * in_file_p, unsigned int cmd, unsigned long arg )
{

#ifdef CONFIG_COMPAT
  void __user*	w_argp = compat_ptr( arg );
#else
  void __user*  w_argp = (void __user *)arg;
#endif
	kaudio_ioctl_t	w_kaudio_ioctl;
	long			w_rc;
	size_t total = 0;

/**/
	pr_debug("%s\n", __func__);

	w_rc = 0;
	memset( &w_kaudio_ioctl, (-1), sizeof( w_kaudio_ioctl ) );

	switch( cmd ){
	case KAUDIO_IOCTL_GET_JACK_STATE:
		w_kaudio_ioctl.jack_state.value = wcd_mbhc_get_jack_status();
		if( copy_to_user( w_argp, &w_kaudio_ioctl, sizeof( w_kaudio_ioctl ) ) ){
			w_rc = (-1);
			pr_err("failed copy_to_user()\n");
			goto err;
		}
		pr_info("%s value=%x", __func__, w_kaudio_ioctl.jack_state.value);
		break;

	case KAUDIO_IOCTL_SET_SYSTEM_RESUME_TIMEOUT:
		if( copy_from_user( (void*)&w_kaudio_ioctl, (void*)w_argp, sizeof( w_kaudio_ioctl ) ) ){
			w_rc = (-1);
			pr_err("failed copy_from_user()\n");
			goto err;
		}

		wcd9xxx_spmi_set_SYSTEM_RESUME_TIMEOUT( w_kaudio_ioctl.system_resume_timeout.value );
		pr_info("%s value=%x", __func__, w_kaudio_ioctl.system_resume_timeout.value);
		break;

	case KAUDIO_IOCTL_SET_REG_VAL:
		if (copy_from_user((void*)&w_kaudio_ioctl, (void*)w_argp, sizeof(w_kaudio_ioctl))) {
			w_rc = (-1);
			pr_err("failed copy_from_user()\n");
			goto err;
		}

		if (w_kaudio_ioctl.reg_val.addr > MSM8X16_WCD_MAX_REGISTER) {
			w_rc = (-1);
			pr_err("%s [%x] is invalid register address\n", __func__, w_kaudio_ioctl.reg_val.addr);
			goto err;
		}
		if (msm8x16_wcd_reg_readonly[w_kaudio_ioctl.reg_val.addr]) {
			w_rc = (-1);
			pr_err("%s [%x] is read-only register address\n", __func__, w_kaudio_ioctl.reg_val.addr);
			goto err;
		}

		w_rc = snd_soc_write(g_kaudio_codec, w_kaudio_ioctl.reg_val.addr, w_kaudio_ioctl.reg_val.val);
		pr_info("%s addr=%x val=%x\n", __func__, w_kaudio_ioctl.reg_val.addr, w_kaudio_ioctl.reg_val.val);
		break;

	case KAUDIO_IOCTL_GET_REG_VAL:
		total = kaudio_soc_codec_reg_show(g_kaudio_codec);
		if (total <= 0) {
			w_rc = (-1);
			pr_err("failed kaudio_soc_codec_reg_show() total:%d\n", total);
			goto err;
		}
		break;

	case KAUDIO_IOCTL_SET_LOG_STATE:
		if (copy_from_user((void*)&w_kaudio_ioctl, (void*)w_argp, sizeof(w_kaudio_ioctl))) {
			w_rc = (-1);
			pr_err("failed copy_from_user()\n");
			goto err;
		}
		pr_info("%s file_no=%d on_off=%d\n", __func__, w_kaudio_ioctl.log_state.file_no, w_kaudio_ioctl.log_state.on_off);
		if (w_kaudio_ioctl.log_state.file_no == 0) {
			int i;
			for(i = 0; i < sizeof(klog_is_enable); i++)
				klog_is_enable[i] = w_kaudio_ioctl.log_state.on_off;
		} else {
			klog_is_enable[w_kaudio_ioctl.log_state.file_no] = w_kaudio_ioctl.log_state.on_off;
		}
		break;

	default:
		w_rc = (-1);
		pr_err("%s invalid cmd=%x\n", __func__, cmd);
		goto err;
	}

err:
	pr_debug("w_rc=%x cmd=%x", (int)w_rc, cmd);
	return w_rc;
}

/*===========================================================================

FUNCTION : kaudio_release

===========================================================================*/
static int kaudio_release( struct inode *inode, struct file *file )
{
	int							w_rc;
/**/
	pr_debug("%s\n", __func__);

	w_rc = 0;

	return w_rc;
}

/*===========================================================================

FUNCTION : kaudio_probe

===========================================================================*/
static const struct file_operations kaudio_fops = {
	.owner =                THIS_MODULE,
	.open =                 kaudio_open,
#ifdef CONFIG_COMPAT
	.compat_ioctl =			kaudio_ioctl,
#else
	.unlocked_ioctl =		kaudio_ioctl,
#endif
	.release =              kaudio_release,
};


static int kaudio_probe( struct platform_device* pdev )
{
	int							w_rc;
/**/
	pr_debug("%s\n", __func__);

	w_rc = 0;

	g_kaudio_dev = devm_kzalloc( &pdev->dev, sizeof(kaudio_device_t), GFP_KERNEL );
	if ( !g_kaudio_dev ){
		pr_err("%s: kzalloc failed\n", __func__);
		w_rc = (-ENOMEM);
		goto err0;
	}

	w_rc = alloc_chrdev_region( &g_kaudio_device_num, 0, MINOR_NUMBER, KAUDIO_DRIVER_NAME );
	if ( w_rc ){
		pr_err("%s: Failed to alloc chrdev\n", __func__);
		w_rc = (-ENODEV);
		goto err1;
	}

	g_kaudio_dev->major = MAJOR( g_kaudio_device_num );
	g_kaudio_class = class_create( THIS_MODULE, KAUDIO_DRIVER_NAME );
	if ( IS_ERR( g_kaudio_class ) ){
		w_rc = PTR_ERR(g_kaudio_class);
		pr_err("%s: Failed to create class; err = %d\n", __func__, w_rc);
		goto err2;
	}

	g_kaudio_dev->dev = device_create( g_kaudio_class, NULL, g_kaudio_device_num, NULL, KAUDIO_DRIVER_NAME );
	if ( IS_ERR( g_kaudio_dev->dev ) ){
		w_rc = PTR_ERR(g_kaudio_dev->dev);
		pr_err("%s: Failed to create device; err = %d\n", __func__, w_rc);
		goto err3;
	}

	g_kaudio_dev->cdev = cdev_alloc();
	if ( !g_kaudio_dev->cdev ){
		pr_err("%s: Failed to alloc cdev\n", __func__);
		w_rc = (-ENOMEM);
		goto err4;
	}

	cdev_init( g_kaudio_dev->cdev, &kaudio_fops );
	w_rc = cdev_add( g_kaudio_dev->cdev, g_kaudio_device_num, MINOR_NUMBER );
	if ( w_rc ){
		pr_err("%s: Failed to register chrdev; err = %d\n", __func__, w_rc);
		goto err5;
	}

	pr_debug("%s: Device created\n", __func__);
end:
	return w_rc;

/**/
err5:
	cdev_del( g_kaudio_dev->cdev );
err4:
	device_destroy( g_kaudio_class, g_kaudio_device_num );
err3:
	class_destroy( g_kaudio_class );
err2:
	unregister_chrdev_region( 0, MINOR_NUMBER );
err1:
	kfree( g_kaudio_dev );
err0:
	goto end;
}

/*===========================================================================

FUNCTION : kaudio_remove

===========================================================================*/
static int kaudio_remove( struct platform_device* pdev )
{
	pr_debug("%s\n", __func__);

	cdev_del(g_kaudio_dev->cdev);
	kfree(g_kaudio_dev->cdev);
	mutex_destroy(&kaudio_lock);
	device_destroy(g_kaudio_class, g_kaudio_device_num);
	class_destroy(g_kaudio_class);
	unregister_chrdev_region(0, MINOR_NUMBER);
	kfree(g_kaudio_dev);

	return 0;
}

/*===========================================================================

FUNCTION : kaudio_init

===========================================================================*/
static struct of_device_id kaudio_of_match[] = {
	{.compatible = "kc,kaudio"},
	{ }
};
MODULE_DEVICE_TABLE(of, kaudio_of_match);

static struct platform_driver kaudio_driver = {
	.probe          = kaudio_probe,
	.remove         = kaudio_remove,
	.driver         = {
		.name   = "kaudio",
		.owner  = THIS_MODULE,
		.of_match_table = kaudio_of_match,
	},
};

static int __init kaudio_init(void)
{
	pr_debug("%s\n", __func__);

	return platform_driver_register( &kaudio_driver );
}

/*===========================================================================

FUNCTION : kaudio_exit

===========================================================================*/
static void __exit kaudio_exit(void)
{
	pr_debug("%s\n", __func__);

	platform_driver_unregister( &kaudio_driver );
}

module_init(kaudio_init);
module_exit(kaudio_exit);

