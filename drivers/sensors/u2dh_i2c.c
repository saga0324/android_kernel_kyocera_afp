/* This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 */
/* drivers/input/misc/u2dh_i2c.c
 *
 * Accelerometer device driver for I2C
 *
 * Copyright (C) 2011-2014 ALPS ELECTRIC CO., LTD. All Rights Reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifdef ALPS_ACC_DEBUG
#define DEBUG 1
#endif

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>

#define I2C_RETRIES		5

#define ACCSNS_DRIVER_NAME	"u2dh"
#define ACCSNS_LOG_TAG		"[U2DH], "

/* Register Name for accsns */
#define ACCSNS_OUT_X_L		0x28
#define ACCSNS_OUT_X_H		0x29
#define ACCSNS_OUT_Y_L		0x2A
#define ACCSNS_OUT_Y_H		0x2B
#define ACCSNS_OUT_Z_L		0x2C
#define ACCSNS_OUT_Z_H		0x2D
#define ACCSNS_REG0F		0x0F
#define ACCSNS_REG1F		0x1F
#define ACCSNS_REG20		0x20
#define ACCSNS_REG21		0x21
#define ACCSNS_REG22		0x22
#define ACCSNS_REG23		0x23
#define ACCSNS_REG24		0x24

#define ACCSNS_DATA_ACCESS_NUM	6
#define ACCSNS_3AXIS_NUM	3
#define ACCSNS_INITIALL_DELAY	10
#define ACCSNS_HRM_RANGE_4G  2

#define ACCSNS_DELAY(us)	usleep_range(us, us)

static struct i2c_client *client_accsns;
static atomic_t flgEna;
static atomic_t delay;
static atomic_t flgSuspend;

struct accsns_regulator_data {
    struct regulator* vdd_reg;
    uint32_t min_uV;
    uint32_t max_uV;
    uint32_t on_load_uA;
    uint32_t off_load_uA;
};

static struct accsns_regulator_data reg_data;

struct accsns_odr {
    uint32_t delay;
    uint8_t  odr;
};

static const struct accsns_odr accsns_odr_tbl[] = {
    { 1,    0x67},
    { 10,   0x57},
    { 20,   0x47},
    { 40,   0x37},
    { 100,  0x27}
};

/*--------------------------------------------------------------------------
 * i2c read/write function
 *--------------------------------------------------------------------------*/
static int accsns_i2c_read(u8 *rxData, int length)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
			.addr	= client_accsns->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxData,
		},
		{
			.addr	= client_accsns->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxData,
		},
	};

	do {
		err = i2c_transfer(client_accsns->adapter,
			msgs, ARRAY_SIZE(msgs));
	} while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

	if (err != ARRAY_SIZE(msgs)) {
		dev_err(&client_accsns->adapter->dev,
			"read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int accsns_i2c_write(u8 *txData, int length)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
			.addr	= client_accsns->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txData,
		},
	};

	do {
		err = i2c_transfer(client_accsns->adapter,
			msgs, ARRAY_SIZE(msgs));
	} while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

	if (err != ARRAY_SIZE(msgs)) {
		dev_err(&client_accsns->adapter->dev,
			"write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}


/*--------------------------------------------------------------------------
 * accsns function
 *--------------------------------------------------------------------------*/
int accsns_get_acceleration_data(int *xyz)
{
	int err = -1;
    u8 buf[6];

	if (atomic_read(&flgSuspend) == 1)
		return err;

    buf[0] = (ACCSNS_OUT_X_L | 0x80);
    err = accsns_i2c_read(buf, sizeof(buf));
    xyz[0] = (((s16) ((buf[1] << 8) | buf[0])) >> 4) * ACCSNS_HRM_RANGE_4G;
    xyz[1] = (((s16) ((buf[3] << 8) | buf[2])) >> 4) * ACCSNS_HRM_RANGE_4G;
    xyz[2] = (((s16) ((buf[5] << 8) | buf[4])) >> 4) * ACCSNS_HRM_RANGE_4G;

    dev_dbg(&client_accsns->adapter->dev,
		ACCSNS_LOG_TAG "Org: x:%d,y:%d,z:%d\n", xyz[0], xyz[1], xyz[2]);
    xyz[0] = -1 * xyz[0];
    xyz[1] = 1 * xyz[1];
    xyz[2] = -1 * xyz[2];


	dev_dbg(&client_accsns->adapter->dev,
		ACCSNS_LOG_TAG "x:%d,y:%d,z:%d\n", xyz[0], xyz[1], xyz[2]);

	return err;
}
EXPORT_SYMBOL(accsns_get_acceleration_data);

void accsns_activate(int flgatm, int flg, int dtime)
{
    int i;
	int err = -1;
	u8 buf[2];

	if (flg != 0)
		flg = 1;

    if (flg == 1) {
        err = regulator_set_optimum_mode(reg_data.vdd_reg, reg_data.on_load_uA);
        if( err < 0 ) {
            dev_err(&client_accsns->adapter->dev, "regulator_set_optimum_mode fail. err=%d\n", err);
            return;
        }
        usleep_range(1000,1000);
    }

	buf[0] = ACCSNS_REG20;/* CTRL_REG1 */
	if (flg == 0)
		buf[1] = 0x07;/* power down mode */
	else {
        buf[1] = 0x57;
        for(i = 0; i < sizeof(accsns_odr_tbl)/sizeof(struct accsns_odr); i++) {
            if( dtime >= accsns_odr_tbl[i].delay ) {
                buf[1] = accsns_odr_tbl[i].odr;
            }
        }
    }
	err = accsns_i2c_write(buf, 2);
	if (err < 0)
		dev_info(&client_accsns->adapter->dev,
			ACCSNS_LOG_TAG "%s:Err6n", __func__);

	ACCSNS_DELAY(2000);
	if (flgatm) {
		atomic_set(&flgEna, flg);
        atomic_set(&delay, dtime);
    }

    if (flg == 0) {
        usleep_range(1000,1000);
        err = regulator_set_optimum_mode(reg_data.vdd_reg, reg_data.off_load_uA);
        if( err < 0 ) {
            dev_err(&client_accsns->adapter->dev, "regulator_set_optimum_mode fail. err=%d\n", err);
            return;
        }
    }
}
EXPORT_SYMBOL(accsns_activate);

static int accsns_register_init(void)
{
	int ret = 0;
	u8  buf[2];

	dev_dbg(&client_accsns->adapter->dev,
		ACCSNS_LOG_TAG "%s\n", __func__);

    buf[0] = ACCSNS_REG24;
    buf[1] = 0x80;
    accsns_i2c_write(buf, 2);

    msleep(10);

    buf[0] = ACCSNS_REG20;
    buf[1] = 0x07;
    accsns_i2c_write(buf, 2);

    buf[0] = ACCSNS_REG23;
    buf[1] = 0x98;
    accsns_i2c_write(buf, 2);

    usleep_range(1000,1000);
    ret = regulator_set_optimum_mode(reg_data.vdd_reg, reg_data.off_load_uA);
    if( ret < 0 ) {
        dev_err(&client_accsns->adapter->dev, "regulator_set_optimum_mode fail. ret=%d\n", ret);
        return ret;
    }

	return 0;
}


/*--------------------------------------------------------------------------
 * suspend/resume function
 *--------------------------------------------------------------------------*/
static int accsns_suspend(struct i2c_client *client, pm_message_t mesg)
{
	dev_dbg(&client->adapter->dev,
		ACCSNS_LOG_TAG "%s\n", __func__);
	atomic_set(&flgSuspend, 1);
	accsns_activate(0, 0, atomic_read(&delay));
	return 0;
}

static int accsns_resume(struct i2c_client *client)
{
	dev_dbg(&client->adapter->dev,
		ACCSNS_LOG_TAG "%s\n", __func__);
	atomic_set(&flgSuspend, 0);
	accsns_activate(0, atomic_read(&flgEna), atomic_read(&delay));
	return 0;
}



/*--------------------------------------------------------------------------
 * i2c device
 *--------------------------------------------------------------------------*/
static int accsns_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
    int err = 0;

	dev_dbg(&client->adapter->dev,
		ACCSNS_LOG_TAG "%s\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->adapter->dev, "client not i2c capable\n");
		return -ENODEV;
	}

	client_accsns = client;


	atomic_set(&flgEna, 0);
	atomic_set(&delay, ACCSNS_INITIALL_DELAY);
	atomic_set(&flgSuspend, 0);

    of_property_read_u32(client->dev.of_node, "acc-vdd-min-voltage", &reg_data.min_uV);
    of_property_read_u32(client->dev.of_node, "acc-vdd-max-voltage", &reg_data.max_uV);
    of_property_read_u32(client->dev.of_node, "acc-vdd-on-load-current", &reg_data.on_load_uA);
    of_property_read_u32(client->dev.of_node, "acc-vdd-off-load-current", &reg_data.off_load_uA);
    dev_info(&client->adapter->dev, "regulator min_uV = %d, max_uV = %d, on_load_uA = %d, off_load_uA = %d\n",
        reg_data.min_uV, reg_data.max_uV, reg_data.on_load_uA, reg_data.off_load_uA);

    reg_data.vdd_reg = regulator_get(&client->dev, "acc-vdd");
    if( IS_ERR(reg_data.vdd_reg) ) {
        dev_err(&client->adapter->dev, "failed regulator_get \n");
        return -EIO;
    }

    err = regulator_set_voltage(reg_data.vdd_reg, reg_data.min_uV, reg_data.max_uV);
    if( err ) {
        dev_err(&client->adapter->dev, "regulator_set_voltage fail. err=%d\n", err);
        return -EIO;
    }

    err = regulator_set_optimum_mode(reg_data.vdd_reg, reg_data.on_load_uA);
    if( err < 0 ) {
        dev_err(&client->adapter->dev, "regulator_set_optimum_mode fail. err=%d\n", err);
        return -EIO;
    }

    err = regulator_enable(reg_data.vdd_reg);
    if( err ) {
        dev_err(&client->adapter->dev, "regulator_enable fail. err=%d\n", err);
        return -EIO;
    }

    usleep_range(1000,1000);

	if (accsns_register_init()) {
		dev_err(&client->adapter->dev,
			"failed to initialize sensor\n");
		return -EIO;
	}

	dev_info(&client->adapter->dev,
		"detected " ACCSNS_DRIVER_NAME "accelerometer\n");

	return 0;
}

static int accsns_remove(struct i2c_client *client)
{
	dev_dbg(&client->adapter->dev,
		ACCSNS_LOG_TAG "%s\n", __func__);
	accsns_activate(0, 0, atomic_read(&delay));
	client_accsns = NULL;
	return 0;
}


/*--------------------------------------------------------------------------
 * module
 *--------------------------------------------------------------------------*/
static const struct i2c_device_id accsns_id[] = {
	{ ACCSNS_DRIVER_NAME, 0 },
	{ }
};

static struct i2c_driver accsns_driver = {
	.probe		= accsns_probe,
	.remove		= accsns_remove,
	.id_table	= accsns_id,
	.driver		= {
	.name		= ACCSNS_DRIVER_NAME,
	},
	.suspend	= accsns_suspend,
	.resume		= accsns_resume,
};

static int __init accsns_init(void)
{
	pr_debug(ACCSNS_LOG_TAG "%s\n", __func__);
	return i2c_add_driver(&accsns_driver);
}

static void __exit accsns_exit(void)
{
	pr_debug(ACCSNS_LOG_TAG "%s\n", __func__);
	i2c_del_driver(&accsns_driver);
}

module_init(accsns_init);
module_exit(accsns_exit);

MODULE_DESCRIPTION("Alps Accelerometer Device");
MODULE_AUTHOR("ALPS ELECTRIC CO., LTD.");
MODULE_LICENSE("GPL v2");
