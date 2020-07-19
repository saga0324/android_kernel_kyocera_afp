/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2019 KYOCERA Corporation
 */

#ifndef __SENSOR_MICON_COMMON_H__
#define __SENSOR_MICON_COMMON_H__

#include <linux/spi/spi.h>
//#include <linux/ql/qleos_s3.h>
#include <linux/device.h>

int sensor_dev_init(struct spi_device *client);
int sensor_dev_remove(struct spi_device *client);
int sensor_dev_suspend(struct device *dev);
int sensor_dev_resume(struct device *dev);
void sensor_dev_shutdown(struct spi_device *client);
void sensor_irq_proc(uint64_t result);
void sensor_exec_flush(int32_t type);
int32_t sensor_dev_save_param(void);
int32_t sensor_dev_load_param(void);

#endif /*__SENSOR_MICON_COMMON_H__*/
