/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
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
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 */

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/spi/spi.h>
#include <asm/mach/arch.h>
#include <soc/qcom/socinfo.h>
#include <mach/board.h>
#include <mach/msm_memtypes.h>
#include <soc/qcom/rpm-smd.h>
#include <soc/qcom/smd.h>
#include <soc/qcom/smem.h>
#include <soc/qcom/spm.h>
#include <soc/qcom/pm.h>
#include "board-dt.h"
#include "platsmp.h"

#include <linux/nfc/cxd224x.h>
#include <linux/i2c.h>
#include <linux/gpio.h>

#define MSM_8909_BLSP1_QUP1_I2C_BUS_ID   1

#define MSM_GPIO_NUM_OFFSET   911
#define MSM_GPIO(n) (n+MSM_GPIO_NUM_OFFSET)

#define NFC_GPIO_IRQ           31   /* HOSTINT*/
#if defined(CONFIG_CXD224X_NFC_VEN) || defined(CONFIG_CXD224X_NFC_VEN_MODULE)
#define NFC_GPIO_VEN          136   /* not used */
#endif /* defined(CONFIG_CXD224X_NFC_VEN) || defined(CONFIG_CXD224X_NFC_VEN_MODULE) */
#define NFC_GPIO_FIRM          32   /* PON */
#define NFC_GPIO_RST           38   /* RST(active hi) */
#define NFC_GPIO_RFS          110   /* RFS */


static void __init msm8909_dt_reserve(void)
{
	of_scan_flat_dt(dt_scan_for_memory_reserve, NULL);
}

static void __init msm8909_map_io(void)
{
	msm_map_msm8909_io();
}

static struct of_dev_auxdata msm8909_auxdata_lookup[] __initdata = {
	{}
};

static struct spi_board_info spi_board_info[] __initdata = {
	{
		.modalias = "spidev",
		.bus_num = 0,
		.chip_select = 0,
		.max_speed_hz = 12.5*1000*1000,
		.mode = SPI_MODE_0
	},
};

static struct cxd224x_platform_data cxd224x_pdata = {
    .irq_gpio = MSM_GPIO(NFC_GPIO_IRQ),
#if defined(CONFIG_CXD224X_NFC_VEN) || defined(CONFIG_CXD224X_NFC_VEN_MODULE)
    .en_gpio = MSM_GPIO(NFC_GPIO_VEN),
#endif
    .wake_gpio = MSM_GPIO(NFC_GPIO_FIRM),
    .rst_gpio = MSM_GPIO(NFC_GPIO_RST),
    .rfs_gpio = MSM_GPIO(NFC_GPIO_RFS),
};

static struct i2c_board_info i2c_cxd224x[] __initdata = {
    {
        I2C_BOARD_INFO("cxd224x-i2c", 0x28),
        .flags = I2C_CLIENT_WAKE,
        .platform_data = &cxd224x_pdata
    },
};

static void __init i2c_devices_init(void) {
    i2c_cxd224x[0].irq = gpio_to_irq(MSM_GPIO(NFC_GPIO_IRQ));
    i2c_register_board_info( MSM_8909_BLSP1_QUP1_I2C_BUS_ID, i2c_cxd224x, ARRAY_SIZE(i2c_cxd224x) );
}

static void __init spi_devices_init(void) {
	spi_register_board_info( spi_board_info, ARRAY_SIZE(spi_board_info) );
}

/*
 * Used to satisfy dependencies for devices that need to be
 * run early or in a particular order. Most likely your device doesn't fall
 * into this category, and thus the driver should not be added here. The
 * EPROBE_DEFER can satisfy most dependency problems.
 */
void __init msm8909_add_drivers(void)
{
	msm_smd_init();
	msm_rpm_driver_init();
	msm_spm_device_init();
	msm_pm_sleep_status_init();
}

static void __init msm8909_init(void)
{
	struct of_dev_auxdata *adata = msm8909_auxdata_lookup;

	/*
	 * populate devices from DT first so smem probe will get called as part
	 * of msm_smem_init.  socinfo_init needs smem support so call
	 * msm_smem_init before it.
	 */
	of_platform_populate(NULL, of_default_bus_match_table, adata, NULL);
	msm_smem_init();

	if (socinfo_init() < 0)
		pr_err("%s: socinfo_init() failed\n", __func__);

	msm8909_add_drivers();
	
	spi_devices_init();

	i2c_devices_init();
}

static const char *msm8909_dt_match[] __initconst = {
	"qcom,msm8909",
	NULL
};

DT_MACHINE_START(MSM8909_DT,
	"Qualcomm Technologies, Inc. MSM 8909 (Flattened Device Tree)")
	.map_io = msm8909_map_io,
	.init_machine = msm8909_init,
	.dt_compat = msm8909_dt_match,
	.reserve = msm8909_dt_reserve,
	.smp = &msm8916_smp_ops,
MACHINE_END
