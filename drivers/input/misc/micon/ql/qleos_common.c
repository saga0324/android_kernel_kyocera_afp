/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2019 KYOCERA Corporation
 */

#include<linux/sensor_micon_common.h>

int sensor_ql_micon_probe(struct spi_device *client)
{
    return sensor_dev_init(client);
}

int sensor_ql_micon_remove(struct spi_device *client)
{
    return sensor_dev_remove(client);
}

int sensor_ql_micon_suspend(struct device *dev)
{
    return sensor_dev_suspend(dev);
}

int sensor_ql_micon_resume(struct device *dev)
{
    return sensor_dev_resume(dev);
}

void sensor_ql_micon_shutdown(struct spi_device *client)
{
    sensor_dev_shutdown(client);
}

void sensor_ql_micon_interrupt(uint64_t result)
{
    sensor_irq_proc(result);
}

void sensor_ql_micon_flush(int32_t type)
{
	sensor_exec_flush(type);
}

int32_t sensor_ql_micon_save_param(void)
{
	return sensor_dev_save_param();
}

int32_t sensor_ql_micon_load_param(void)
{
	return sensor_dev_load_param();
}
