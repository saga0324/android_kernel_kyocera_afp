 /*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2018 KYOCERA Corporation
 * (C) 2019 KYOCERA Corporation
 */
/*
 * This program is free software; you can redistribute it and/or
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/syscalls.h>
#include <linux/gpio.h>

#include <linux/ql/qlspi.h>
#include <linux/ql/qlmcu.h>
#include <linux/ql/qltrace.h>

#include <linux/ql/qlspi_linux.h>

#define EN_SYSKERNEL_ENTRY

#undef RESET_S3_USING_GPIO

//#define SYSCLASS_ENTRY

void dump_reg_ldr(uint32_t addr);

#ifdef READ_CPU_PROC
extern char *cpu_freq_filename[];
extern struct file *cpu_freq_file_handle[NO_OF_CPUFREQ_PROCS];
#endif
#define QL_LDR_DRV_DMESG_TAG        " ql_spi : loader "

#undef LDR_DEBUG_EN

#ifdef LDR_DEBUG_EN
#define LDR_ERR(fmt, args...)            pr_err(QL_LDR_DRV_DMESG_TAG"%s %d ERROR: "fmt, __FUNCTION__, __LINE__, ##args)
#define LDR_WARN(fmt, args...)           pr_notice(QL_LDR_DRV_DMESG_TAG"%s %d WARN: "fmt, __FUNCTION__, __LINE__, ##args)
#define LDR_LOG(fmt, args...)            pr_info(QL_LDR_DRV_DMESG_TAG"%s %d INFO: "fmt, __FUNCTION__, __LINE__, ##args)
#define LDR_DBG(fmt, args...)            pr_debug(QL_LDR_DRV_DMESG_TAG"%s %d DEBUG: "fmt, __FUNCTION__, __LINE__, ##args)
#else
#define LDR_ERR(fmt, args...)            pr_err(KERN_ERR QL_LDR_DRV_DMESG_TAG"%s %d ERROR: "fmt, __FUNCTION__, __LINE__, ##args)
#define LDR_WARN(fmt, args...)
#define LDR_LOG(fmt, args...)
#define LDR_DBG(fmt, args...)
#endif


struct kobject *loader_kobj;

#ifdef SYSCLASS_ENTRY
struct class *sensor_class;
struct device *sensor_dev;
#endif

#ifdef FABRIC_LOADER

	#define FABRIC_PROG_FW_FILENAME				"/vendor/firmware/Fabric_PRG.bin"			//fabric programmer binary OR combined binary

	#define FABRIC_FW_FILENAME					"/vendor/firmware/Fabric.bin"				//actual fabric binary , no progammer

#endif



#ifdef FFE_LOADER

	#define FFE_FW_FILENAME				"/vendor/firmware/FFE.bin"

#endif

#define FILENAME_SZ (200)

#define M4_FW_FILENAME					"/vendor/firmware/Shub.bin"

#define LOADER_DRV_SYSFS_FOLDER		"fw_loader"			// This will be at  /sys/kernel/fw_loader

static ssize_t s3_fw_download(struct device *dev, struct device_attribute *attr,const char *buf, size_t count);
static ssize_t s3_fw_download_sts(struct device *dev, struct device_attribute *attr,char *buf);






#ifdef EN_SYSKERNEL_ENTRY

static DEVICE_ATTR(fw_download, 0660, NULL, s3_fw_download);
static DEVICE_ATTR(fw_download_sts, 0660 , s3_fw_download_sts , NULL);


static struct attribute *loader_attributes[] = {

	&dev_attr_fw_download_sts.attr,
	&dev_attr_fw_download.attr,
	NULL,
};

static struct attribute_group loader_attr_group = {
		    .attrs = loader_attributes,
};

#endif

#ifdef RESET_S3_USING_GPIO
int reset_slave_mcu(void);



/*	To Reset the S3 we need to make the SYS_RESET_IN pin to LOW
*	GPIO24 is connected to SYS_RESET_IN
*/
QL_Status reset_s3_hold(void)
{
	struct SPIDEV_DATA *spi_priv=spi_get_drvdata(spi_slave_dev);
	LDR_DBG("Resetting S3 using GPIO [HOLD]\n");
	usleep_range(1000,2000);
	gpio_set_value(spi_priv->rst_gpio, 0);
	usleep_range(3000,4000);
	LDR_DBG("End\n");
	return QL_STATUS_OK;
}
//EXPORT_SYMBOL_GPL(reset_s3_hold);


/*	To Release S3 from Reset we need to make the SYS_RESET_IN pin to HIGH
*	GPIO24 is connected to SYS_RESET_IN
*/
QL_Status reset_s3_release(void)
{
	struct SPIDEV_DATA *spi_priv=spi_get_drvdata(spi_slave_dev);
	LDR_DBG("Resetting S3 using GPIO [UN HOLD]\n");
	gpio_set_value(spi_priv->rst_gpio, 1);
	usleep_range(3000,4000);
	LDR_DBG("End\n");
	return QL_STATUS_OK;
}
//EXPORT_SYMBOL_GPL(reset_s3_release);


/*	S3:
*	IO20 - Chip Select
*		1 - Smart Phone
*		0 - Host
*	IO19 - MOSI
*		1 - Debugger
*		0 - FW Load SPI
*
*For BootStrap CS Pin Should be HIGH and MOSI Pin should be LOW
*/
QL_Status configure_bootstraps(void)
{
	return QL_STATUS_OK;
}
//EXPORT_SYMBOL_GPL(configure_bootstraps);

#endif

struct file *openFile(char *path,int flag)
{
    struct file *fp=NULL;

	LDR_DBG("Start\n");

    fp = filp_open(path, flag, 0);

    if (IS_ERR_OR_NULL(fp))
    {
 	    LDR_ERR("ERR = %ld :  Not able to open the firmware file %s\n", PTR_ERR(fp),path);
 	    return NULL;
    }
//	LDR_DBG(" Opened the firmware file %p \n",fp);

	LDR_DBG("End\n");
	return fp;


}

int get_file_size(struct file* fp)
{
	int size=0;

	LDR_DBG("Start\n");

	fp->f_op->llseek(fp, 0, 0);

	size=fp->f_pos;

	fp->f_op->llseek(fp, 0, 2);

	size=fp->f_pos-(size);

	fp->f_op->llseek(fp, 0, 0);

	LDR_DBG("End\n");
	return size;
}

int readFile(struct file *fp,char *buf,int readlen)
{
	LDR_DBG("Start\n");

  	//LDR_LOG(" Reading the firmware file \n");

	LDR_DBG("End\n");
    return vfs_read(fp, buf, readlen, &fp->f_pos);
}

void closeFile(struct file *fp)
{
	LDR_DBG("Start\n");

    filp_close(fp, NULL);

	LDR_DBG(" Closed the firmware file \n");

}

int load_fw_to_arr(uint8_t** fw_image_buf,uint32_t* fw_image_size,const char* name)
{
    struct file *fw_fd = NULL;

    mm_segment_t oldfs;

    char fw_name[FILENAME_SZ] = {0};

	LDR_DBG("Start\n");

    strncpy(fw_name, name , FILENAME_SZ);

//	LDR_DBG(" file name %s ,  filenamesize %d \n",name,strlen(name));

    oldfs = get_fs();

    set_fs(KERNEL_DS);

	fw_fd = openFile(fw_name, O_RDONLY);

//	LDR_DBG(" returned ptr % p \n",fw_fd);


	if(NULL==fw_fd)
	{
		LDR_ERR(" Firware file opening FAILED \n");
		goto err;
	}

	*fw_image_size=fw_fd->f_pos;

	fw_fd->f_op->llseek(fw_fd, 0, 2);



	*fw_image_size=fw_fd->f_pos-(*fw_image_size);

	LDR_DBG(" Firmware file size %d bytes \n",*fw_image_size);

//rewind

	fw_fd->f_op->llseek(fw_fd, 0, 0);


	*fw_image_buf= kzalloc( (sizeof(uint8_t) * (*fw_image_size)), GFP_KERNEL);

	if (*fw_image_buf == NULL)
	{
		LDR_ERR(" Firware buff alloc FAILED %p \n",*fw_image_buf);
		goto err_buff_alloc;
	}

//	LDR_LOG(" Firmware buf allocated \n");


	if(readFile(fw_fd,*fw_image_buf,*fw_image_size) < 0)
    {
        LDR_LOG(" readFile failed !");

		goto err_readfile;
    }

	closeFile(fw_fd);

	set_fs(oldfs);


	LDR_DBG("End\n");

return  QL_STATUS_OK;


err_readfile:
	kfree(*fw_image_buf);
	*fw_image_buf=NULL;

err_buff_alloc:

	closeFile(fw_fd);

err:

	set_fs(oldfs);

	return QL_STATUS_ERROR;


}
EXPORT_SYMBOL_GPL(load_fw_to_arr);

void dump_reg_ldr(uint32_t addr)
{
	uint32_t buff=0;
	QL_Status sts;

	LDR_DBG("Start\n");

	buff=0;

	sts=QLSPI_Read_S3_Mem(addr, (uint8_t*)&buff, 4);
	if(sts!=QL_STATUS_OK)
	{
		LDR_ERR(" Reg 0x%x FAILED\n",addr);
		return;
	}

	LDR_DBG(" Reg 0x%x  = 0x%08x \n",addr,buff);
}


static ssize_t s3_fw_download_sts(struct device *dev, struct device_attribute *attr,char *buf)
{
	LDR_DBG("Start\n");
	if(qlspi_get_boot_mode())
	{
		return sprintf(buf,"0\n");
	}

	LDR_DBG("End\n");
	return sprintf(buf,"1\n");
}


//This API is to trigger fw download using echo/write (on boot)

static ssize_t s3_fw_download(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
		SLAVE_DEV_FW_LOAD_T  slave_dev_fw_load_info;

		uint8_t *fw_image_buf=NULL;

		uint32_t fw_image_size=0;


#ifdef READ_CPU_PROC
	int i=0;
	mm_segment_t oldfs;
#endif

#ifdef FABRIC_LOADER

		uint8_t *fabric_image_buf=0;
		uint32_t fabric_image_size=0;

#endif

#ifdef FFE_LOADER
		uint8_t *ffe_image_buf=0;
		uint32_t ffe_image_size=0;

#endif

		LDR_DBG(" FW download starting ");

#ifdef READ_CPU_PROC

			oldfs = get_fs();
			set_fs(KERNEL_DS);

			for(i=0;i<NO_OF_CPUFREQ_PROCS;i++)
			{
				cpu_freq_file_handle[i]=openFile(cpu_freq_filename[i],O_RDONLY);
				if(NULL==cpu_freq_file_handle[i])
				{
					QLSPI_ERR(" %s open FAILED \n",cpu_freq_filename[i]);
				}
			}

			set_fs(oldfs);
#endif
		if(read_devid()!=EOSS3_SPI_DEVID)
		{
			LDR_ERR(" Device not present \n");
			return -1;
		}



		if(!qlspi_get_boot_mode())
		{
			qlspi_set_boot_mode(MCU_MODE_BOOT);
		}

		memset(&slave_dev_fw_load_info,0,sizeof(SLAVE_DEV_FW_LOAD_T));

	//read fw into a buffer

		if(load_fw_to_arr(&fw_image_buf,&fw_image_size,M4_FW_FILENAME)!=QL_STATUS_OK)
		{
			QLSPI_ERR(" Reading FW file FAILED \n");
			goto end;
		}


		slave_dev_fw_load_info.m4_fw_addr=fw_image_buf;
		slave_dev_fw_load_info.m4_fw_size=fw_image_size;
		slave_dev_fw_load_info.m4_fw_dest_addr=APP_START_ADDR;



#ifdef FABRIC_LOADER

	if(load_fw_to_arr(&fabric_image_buf,&fabric_image_size,FABRIC_FW_FILENAME)!=QL_STATUS_OK)
	{
		QLSPI_ERR(" Reading Fabric FW file FAILED \n");
		goto free_m4_buf;
	}

	slave_dev_fw_load_info.fab_fw_addr = fabric_image_buf;
	slave_dev_fw_load_info.fab_fw_size = fabric_image_size;
	slave_dev_fw_load_info.fab_fw_dest_addr=FABRIC_FW_DEST_ADDR;


	if(load_fw_to_arr(&fabric_image_buf,&fabric_image_size,FABRIC_PROG_FW_FILENAME)!=QL_STATUS_OK)
	{
		QLSPI_ERR(" Reading Fabric Programmer file FAILED \n");
		goto free_m4_buf;
	}

	slave_dev_fw_load_info.fab_prg_fw_addr = fabric_image_buf;
	slave_dev_fw_load_info.fab_prg_fw_size = fabric_image_size;
	slave_dev_fw_load_info.fab_prg_fw_dest_addr=FABRIC_PROG_FW_DEST_ADDR;

#endif


#ifdef FFE_LOADER

		if(load_fw_to_arr(&ffe_image_buf,&ffe_image_size,FFE_FW_FILENAME)!=QL_STATUS_OK)
		{
			QLSPI_ERR(" Reading FFE FW file FAILED \n");
			goto free_fab_buf;
		}

		slave_dev_fw_load_info.ffe_fw_addr = ffe_image_buf;
		slave_dev_fw_load_info.ffe_fw_size = ffe_image_size;
		slave_dev_fw_load_info.ffe_fw_dest_addr=APP_START_ADDR;




#endif

#ifdef RESET_S3_USING_GPIO

		reset_slave_mcu();		//reset m4
		reset_s3_hold();		//reset s3 using gpio (HOLD)
		configure_bootstraps();	//make CS and MISO to Define State
		reset_s3_release();		//release the s3 reset
#if 0
		if(read_devid()!=EOSS3_SPI_DEVID)
		{
			LDR_ERR(" Device not present \n");
			return -1;
		}
		aucBuffer = 0x3;

		QL_TRACE_MCU_ERROR(" Writing ACCESS CTRL \n");

		if(tlc_reg_write( SPITLC_AHB_ACCESS_CTL, &aucBuffer, ONE_BYTE) !=QL_STATUS_OK) {	// prevent automatic trigger when mem addr	is written for write operations
			QL_TRACE_MCU_ERROR("Failed to write SPITLC_AHB_ACCESS_CTL\n");
			return QL_STATUS_ERROR;

		}
		QL_TRACE_MCU_DEBUG(" Writing ACCESS CTRL DONE \n");


		reset_slave_mcu();		//reset m4
#endif
#endif
		//qlspi_change_freq(high_spi_freq);

		if(QLMCU_Fw_Download(&slave_dev_fw_load_info)!=QL_STATUS_OK)
		{
			QLSPI_ERR(" Downloading FW to Device FAILED\n");
			goto free_ffe_buf;

		}

		qlspi_change_freq(SPI_FREQ_LOW_ID);



		qlspi_set_boot_mode(MCU_MODE_NORMAL);

		LDR_DBG(" FW download completed \n");


#ifdef	QLSPI_LINUX_TEST_CODE

		dump_reg_ldr(SW_INTR_1_REG);
		dump_reg_ldr(SW_INTR_1_EN_AP_REG);
		dump_reg_ldr(SW_INTR_1_EN_M4);

		dump_reg_ldr(SW_INTR_2_REG);
		dump_reg_ldr(SW_INTR_2_EN_AP_REG);
		dump_reg_ldr(SW_INTR_2_EN_M4_REG);

#endif

#ifdef FABRIC_LOADER

		QLSPI_DBG(" Fabric INTR CTRL regs \n");

		dump_reg_ldr(FABRIC_INTR_STS_REG);
		dump_reg_ldr(FABRIC_INTR_STS_REG_RAW);
		dump_reg_ldr(FABRIC_INTR_TYPE);
		dump_reg_ldr(FABRIC_INTR_POL);
		dump_reg_ldr(FABRIC_INTR_EN_AP);

#endif



#ifdef FFE_LOADER

		if(ffe_image_buf)
		{
			kfree(ffe_image_buf);
		}

#endif


#ifdef FABRIC_LOADER

		kfree(fabric_image_buf);

#endif

		kfree(fw_image_buf);

		return count;

free_ffe_buf:

#ifdef FFE_LOADER

	if(ffe_image_buf)
	{
			kfree(ffe_image_buf);
	}
free_fab_buf:
#endif




#ifdef FABRIC_LOADER
		kfree(fabric_image_buf);
free_m4_buf:
#endif


		kfree(fw_image_buf);

end:

		return -1;

}


QL_Status mcu_fw_download(void)
{
	LDR_DBG(" FW download starting : mcu_fw_download ");
	set_APtoS3_value(GPIO_LOW);
	s3_fw_download(NULL,NULL,NULL,0);

	if(qlspi_get_boot_mode())
	{
		return QL_STATUS_ERROR;
	}
	LDR_DBG("End\n");
	return QL_STATUS_OK;
}
EXPORT_SYMBOL_GPL(mcu_fw_download);


#ifdef SYSCLASS_ENTRY

static struct device_attribute attributes[] = {

	    __ATTR(fw_download, 0660 ,  NULL, s3_fw_download),

};

#endif

static int __init loader_drv_init(void){


#ifdef RESET_S3_USING_GPIO
	//int error_chk = 0;
#endif

#ifdef EN_SYSKERNEL_ENTRY

	int retval=0;

	LDR_DBG("Start\n");

	loader_kobj = kobject_create_and_add(LOADER_DRV_SYSFS_FOLDER, kernel_kobj);
	if (!loader_kobj) {
		LDR_ERR("Failure in creating loader kobj\n");
		retval = -EFAULT;
		goto err_kobj_create;
	}

	LDR_DBG(" Loader sysfs folder /sys/kernel/%s created \n",LOADER_DRV_SYSFS_FOLDER);

	retval = sysfs_create_group(loader_kobj, &loader_attr_group);
	if(retval < 0) {
		LDR_ERR("Failure in creating sys group\n");
		goto err_sysfs_create;
	}
#endif

#ifdef RESET_S3_USING_GPIO
#if 0
	error_chk = gpio_request(24, "reset_gpio");
	if (error_chk >= 0) {
		gpio_direction_output(24, 1);
		LDR_DBG("Reset GPIO (24) Registered\n");
	} else {
		LDR_ERR("Reset GPIO (24) req failed %d\n", error_chk);
	}
#endif
#endif

#ifdef SYSCLASS_ENTRY

	int i=0;

	sensor_class = class_create(THIS_MODULE, "sensorhub");
	if (IS_ERR(sensor_class))
	{
		LDR_ERR(" Error creating the sensorhub class \n");
		goto err_class_create;
	}

	sensor_dev = device_create(sensor_class, NULL, 0, "%s", "eoss3");
	if (IS_ERR(sensor_dev))
	{
		LDR_ERR(" Error creating the eoss3 device \n");
		goto err_device_create;
	}


	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(sensor_dev, attributes + i))
			goto error;

#endif


	LDR_DBG(" Loader init done \n");



	return 0;

#ifdef SYSCLASS_ENTRY

	error:
		while (--i >= 0)
			device_remove_file(sensor_dev, attributes + i);
//	err_set_drvdata:
		put_device(sensor_dev);
		device_unregister(sensor_dev);

	err_device_create:
		class_destroy(sensor_class);


	err_class_create:

#endif

#ifdef EN_SYSKERNEL_ENTRY


		sysfs_remove_group(loader_kobj,&loader_attr_group);

	err_sysfs_create:
		kobject_put(loader_kobj);

	err_kobj_create:
#endif

		return -1;
}

static void __exit loader_drv_exit(void){




	LDR_DBG(" Exiting loader driver \n");

#ifdef SYSCLASS_ENTRY

	int i=0;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
	device_remove_file(sensor_dev, attributes + i);

	put_device(sensor_dev);
	device_unregister(sensor_dev);

	class_destroy(sensor_class);

#endif

#ifdef EN_SYSKERNEL_ENTRY



	sysfs_remove_group(loader_kobj,&loader_attr_group);

	sysfs_remove_group(loader_kobj,&loader_attr_group);

	LDR_DBG(" Removed sysfs group\n");

	kobject_put(loader_kobj);

	LDR_DBG(" Removed loader kobj\n");

#endif

}


late_initcall(loader_drv_init);
module_exit(loader_drv_exit);

MODULE_DESCRIPTION("Loader driver");
MODULE_AUTHOR("QuickLogic Private Limited");
MODULE_LICENSE("GPL");

