/*
 * This software is contributed or developed by KYOCERA Corporation.
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#include <linux/fb.h>
#include "disp_ext_i2c.h"
#include "disp_ext_sub_dbg.h"

#define I2C_RETRY_TIMES 2


static struct i2c_client *disp_ext_i2c_client;

void disp_ext_i2c_clk_enable(void)
{
	int ret;

	pr_debug("%s start\n",__func__);

	if (!disp_ext_i2c_client) {
		pr_err("%s : disp_ext_i2c_client NULL!\n",__func__);
		return;
	}

	ret = i2c_clk_prepare_enable(disp_ext_i2c_client->adapter);
	if (ret) {
		pr_err("%s : i2c_clk_prepare_enable error:%d\n",__func__, ret);
		return;
	}

	pr_debug("%s end\n",__func__);
}

void disp_ext_i2c_clk_disable(void)
{
	pr_debug("%s start\n",__func__);

	if (!disp_ext_i2c_client) {
		pr_err("%s : disp_ext_i2c_client NULL!\n",__func__);
		return;
	}

	i2c_clk_disable_unprepare(disp_ext_i2c_client->adapter);

	pr_debug("%s end\n",__func__);
}


int32_t disp_ext_i2c_write(uint8_t *data_p, uint32_t size)
{
	int32_t ret = 0;
	struct i2c_msg msg;
	int retry;

	pr_debug("%s start size=%d\n",__func__,size);
	if (!disp_ext_i2c_client) {
		pr_err("%s : disp_ext_i2c_client NULL!\n",__func__);
		return -ENODEV;
	}
	if (!data_p){
		pr_err("%s : data NULL!\n",__func__);
		return -ENODEV;
	}
	if (!size){
		pr_err("%s : data size zero!\n",__func__);
		return -ENODEV;
	}

	disp_ext_sub_seq_log(data_p, size);

	msg.addr = disp_ext_i2c_client->addr;
	msg.flags = 0;
	msg.len = size;
	msg.buf = data_p;

	for (retry = 0; retry < I2C_RETRY_TIMES; retry++) {
		ret = i2c_transfer(disp_ext_i2c_client->adapter, &msg, 1);
		if (ret == 1)
			break;
		pr_err("i2c_transfer ret:%d, retry after 20ms\n", ret);
		msleep(20);
	}

	if (ret == 1) {
		ret = 0;
	} else {
		pr_err("i2c_transfer error\n");
		ret = -1;
	}

	pr_debug("%s end ret=%d\n",__func__, ret);
	return ret;
}

static int32_t disp_ext_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	pr_debug("%s start\n",__func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality err\n");
		return -ENODEV;
	}

	disp_ext_i2c_client = client;

	pr_debug("%s end\n",__func__);

	return 0;
}

static int disp_ext_i2c_remove(struct i2c_client *client)
{
	pr_debug("%s start\n",__func__);

	pr_debug("%s end\n",__func__);

	return 0;
}

#define DISP_EXT_I2C_NAME "ld7032"
#define DISP_EXT_I2C_DTS_NAME "ldt,ld7032"

static const struct i2c_device_id ld7032_id[] = {
	{ DISP_EXT_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ld7032_id);

static struct of_device_id ld7032_match_table[] = {
	{ .compatible = DISP_EXT_I2C_DTS_NAME,},
	{ },
};

static struct i2c_driver ld7032_driver = {
	.driver = {
		.name   = DISP_EXT_I2C_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = ld7032_match_table,
	},
	.probe  = disp_ext_i2c_probe,
	.remove = disp_ext_i2c_remove,
	.id_table = ld7032_id,
};

int disp_ext_i2c_init( void )
{
	int32_t rc = 0;

	pr_debug("%s start\n",__func__);

	rc = i2c_add_driver(&ld7032_driver);
	if (rc != 0) {
		pr_err("can't add i2c driver\n");
		rc = -ENOTSUPP;
	}

	pr_debug("%s end\n",__func__);

	return 0; //rc;
}

static void __exit disp_ext_i2c_exit(void)
{
	pr_debug("%s start\n",__func__);

	i2c_del_driver(&ld7032_driver);

	pr_debug("%s end\n",__func__);
}

module_exit(disp_ext_i2c_exit);
