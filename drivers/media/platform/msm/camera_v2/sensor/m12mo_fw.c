/* This program is free software; you can redistribute it and/or
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
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2017 KYOCERA Corporation
 */
//#include "msm_sensor.h"
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/of_gpio.h>

#define M12MOFW_NAME "m12mofw"
#define m12moFW_obj m12mofw_##obj


#define M12MO_FW_ADDR_VERSION_HI (0x18)
#define M12MO_FW_ADDR_VERSION_LO (0x19)

/*#define CONFIG_MSMB_CAMERA_DEBUG*/
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_info(fmt, ##args)
#else
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#endif

void m12mo_fw_get_version(uint8_t *ver_hi, uint8_t *ver_lo);
int m12mo_spi_transfer(void);
int m12mo_fw_update(void *data);

struct spi_device *m12mofw_dev;

#include "m12mo_fw.inc"
static uint8_t *g_fw_update = NULL;

void m12mo_fw_get_version(uint8_t *ver_hi, uint8_t *ver_lo)
{
    const uint8_t *fw;

    if (g_fw_update) {
        CDBG("%s:%d get firmware version from requested image.\n", __func__, __LINE__);
        fw = g_fw_update;
    } else {
        CDBG("%s:%d get firmware version from rom image.\n", __func__, __LINE__);
        fw = firmware;
    }
    *ver_hi = fw[M12MO_FW_ADDR_VERSION_HI];
    *ver_lo = fw[M12MO_FW_ADDR_VERSION_LO];
    CDBG("%s:%d firmware version is 0x%02x%02x.\n", __func__, __LINE__,
        *ver_hi, *ver_lo);
}

int m12mo_spi_transfer(void)
{
    int rc;
    struct spi_message spi_msg;
    struct spi_transfer spi_xfer;

    memset(&spi_xfer, 0, sizeof(spi_xfer));
    rc = spi_setup(m12mofw_dev);
    if (rc < 0){
        pr_err("%s:%d spi setup failed. rc=%d\n", __func__, __LINE__, rc);
    }
    spi_message_init(&spi_msg);
    spi_xfer.rx_buf = NULL;
    spi_xfer.len = sizeof(firmware);
    spi_xfer.bits_per_word = 8;
    spi_xfer.speed_hz = m12mofw_dev->max_speed_hz;
    if (g_fw_update) {
        CDBG("%s:%d write firmware of requested image.\n", __func__, __LINE__);
        spi_xfer.tx_buf = g_fw_update;
    } else {
        CDBG("%s:%d write firmware of rom image.\n", __func__, __LINE__);
        spi_xfer.tx_buf = firmware;
    }
    spi_message_add_tail(&spi_xfer, &spi_msg);

    rc = spi_sync(m12mofw_dev, &spi_msg);
    if (rc < 0) {
        pr_err("%s:%d firmware write failed.\n", __func__, __LINE__);
    } else {
        CDBG("%s:%d firmware write succeed.\n", __func__, __LINE__);
    }

    return rc;
}

static int m12mo_fw_spi_probe(struct spi_device *spi)
{
    int irq;
    int cs;
    int cpha,cpol,cs_high;
    u32 max_speed;
    
    m12mofw_dev = spi;

    irq = spi->irq;
    cs = spi->chip_select;
    cpha = ( spi->mode & SPI_CPHA ) ? 1:0;
    cpol = ( spi->mode & SPI_CPOL ) ? 1:0;
    cs_high = ( spi->mode & SPI_CS_HIGH ) ? 1:0;
    max_speed = spi->max_speed_hz;
    pr_info("%s: irq [%d] cs [%x] CPHA [%x] CPOL [%x] CS_HIGH [%x]\n",
            __func__, irq, cs, cpha, cpol, cs_high);
    pr_info("Max_speed [%d]\n", max_speed );

    return 0;
}

int m12mo_fw_update(void* data)
{
    long int fw_size = sizeof(firmware);

    if (!g_fw_update) {
        g_fw_update = kzalloc(fw_size, GFP_KERNEL);
        if (!g_fw_update){
            pr_err("%s:%d memory allocation failed.\n", __func__, __LINE__);
            return -ENOMEM;
        }
    }

    if (copy_from_user(g_fw_update, (void *)data, fw_size)) {
        pr_err("%s:%d memory copy from user failed. size=%ld\n",
            __func__, __LINE__, fw_size);
        kfree(g_fw_update);
        g_fw_update = NULL;
        return -EFAULT;
    }

    return 0;
}

static const struct of_device_id m12mo_spi_table[] = {
    {.compatible = "qcom,m12mofw",},
    {}
};

//SPI Driver Info
static struct spi_driver m12mo_spi_driver = {
    .driver = {
    .name = M12MOFW_NAME,
    .owner = THIS_MODULE,
    .of_match_table = m12mo_spi_table,
    },
    .probe = m12mo_fw_spi_probe,
};

static int __init m12mofw_init_module(void)
{
    int rc;
    rc = spi_register_driver(&m12mo_spi_driver);
    return rc;
}

static void __exit m12mofw_exit_module(void)
{
    if (g_fw_update) {
        kfree(g_fw_update);
        g_fw_update = NULL;
    }
    spi_unregister_driver(&m12mo_spi_driver);
}


module_init(m12mofw_init_module);
module_exit(m12mofw_exit_module);
MODULE_DESCRIPTION("m12mo fw download by SPI");
MODULE_LICENSE("GPL v2");
