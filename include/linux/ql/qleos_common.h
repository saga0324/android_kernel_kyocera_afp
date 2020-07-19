/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2019 KYOCERA Corporation
 */

#ifndef __QL_EOS_COMMON_H__
#define __QL_EOS_COMMON_H__

#include<linux/sensor_micon_common.h>

int sensor_ql_micon_probe(struct spi_device *client);
int sensor_ql_micon_remove(struct spi_device *client);
int sensor_ql_micon_suspend(struct device *dev);
int sensor_ql_micon_resume(struct device *dev);
void sensor_ql_micon_shutdown(struct spi_device *client);
void sensor_ql_micon_interrupt(uint64_t result);
void sensor_ql_micon_flush(int32_t type);
int32_t sensor_ql_micon_save_param(void);
int32_t sensor_ql_micon_load_param(void);
#endif /*__QL_EOS_COMMON_H__*/
