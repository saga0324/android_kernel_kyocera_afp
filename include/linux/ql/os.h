#ifndef __QL_SPI_OS_H__
#define __QL_SPI_OS_H__

//Linux OS specifics

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/delay.h>
#include<linux/slab.h>


#include"qlspi_linux.h"

#define os_malloc(size)		kzalloc(size,GFP_KERNEL)
#define os_free(ptr)		kfree(ptr)

#define OS_MSLEEP
#ifdef OS_MSLEEP
#define os_msleep(ms)		msleep(ms)
#endif
#define		OS_ASSERT()		return QL_STATUS_ERROR;

int tlc_reg_write(unsigned int addr, u8 *writeBuf, unsigned int length);

#endif /*__QL_SPI_OS_H__*/
