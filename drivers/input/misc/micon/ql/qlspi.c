/*	qlspi.c
 *	Copyrighted (C) 2016 QuickLogic Corp
 *
 *	This file contains source of QLSPI Interface.
 *  This file consists of definition of QLSPI API
 *  which are exported.
 *  The features of the QLSPI framework are mentioned
 *  below:
 *      - OS Independent
 *		- SPI HCD Independent
 *      - Provides Interfaces to initialize platform related configuration like
		  GPIO, etc.
 *      - Provides interfaces to perform read/write from/to SPI Slave device
 *      - Provides interfaces to trigger/receive interrupts to/from SPI Slave
	      device
 */
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2019 KYOCERA Corporation
 */

#include <linux/ql/os.h>
//#include <linux/ql/qlspi.h>
//#include <linux/ql/qlspi_tlc.h>
#include <linux/ql/qlspi_linux.h>
#include <linux/ql/qltrace.h>
#include <linux/ql/qlmcu.h>
#include <linux/ql/qleos_s3.h>

#define QL_SPI_MAX_DEV	2 // Max two devices using SPI Bus-Voice and Sensor

#ifdef DMA_FIFO_UNDERFLOW_EMULATE
extern int spi_underflow_flag;
#endif

QLSPI_Isr_Handler spidev_handler[QL_SPI_MAX_DEV];

static int store_spi_dev_isr (QLSPI_Isr_Handler handler);
QL_Status trigger_isr_handler (void *data);

/* SPI slave driver APIs used by QLSPI */

QL_Status ql_spi_init (struct QLSPI_Platform *spi_plat);
int ql_spi_read_s3_mem(uint32_t addr, uint8_t *data, unsigned int len);
QL_Status ql_spi_trigger_intr(void);
int shub_ahb_write( uint32_t addr, uint8_t *buf, unsigned int sz);
int shub_ahb_read_dma( uint32_t addr, uint8_t *buf, unsigned int sz);
void handle_shub_ahb_status( int status );

QL_Status mcu_fw_download(void);

static int store_spi_dev_isr (QLSPI_Isr_Handler handler)
{
	int i;

	QLSPI_DBG("Start\n");
	for (i = 0; i < QL_SPI_MAX_DEV; i++)
	{
		if (spidev_handler[i] == NULL)
		{
			spidev_handler[i] = handler;
			return QL_STATUS_OK;
		}
	}

	QLSPI_DBG("End\n");
	return QL_STATUS_ERROR;
}

QL_Status trigger_isr_handler (void *data)
{
	int i ;

	QLSPI_DBG("Start\n");
	for (i= 0; i < QL_SPI_MAX_DEV; i++)
    {
	if (spidev_handler[i] != NULL)
		spidev_handler[i](data);
    }

	QLSPI_DBG("End\n");
	return QL_STATUS_OK;
}

int shub_read_dma_status(uint8_t* buf)
{
	QLSPI_DBG("Start\n");
	if(tlc_reg_read(SPITLC_DMA_STATUS, buf, 1) != SPI_STATUS_OK)
	{
		QLSPI_ERR("Read DMA status FAIL !\n");
		return SPI_STATUS_ERR;
	}

	QLSPI_DBG("End\n");
	return SPI_STATUS_OK;
}



int read_ahb_status(uint8_t *buf)
{
	QLSPI_DBG("Start\n");
	if(tlc_reg_read (SPITLC_AHB_STATUS,  buf, ONE_BYTE)!=SPI_STATUS_OK)
	{
		QLSPI_ERR(" Read AHB sts reg FAILED \n");
		return SPI_STATUS_ERR;
	}

	QLSPI_DBG("End\n");
	return SPI_STATUS_OK;
}

void handle_shub_ahb_status( int status )
{
}

int shub_ahb_write( uint32_t addr, uint8_t *buf, unsigned int sz)
{
	uint8_t aucBuffer[10] = {0};
	int err = SPI_STATUS_OK;
	int counter = 0;
	uint8_t *addrPtr;
	int rty_count = 2;

	uint8_t* padbuf;
	unsigned int padsize;

	QLSPI_DBG("Start\n");
	do {
		err = read_ahb_status(&aucBuffer[0]);
		if (err != SPI_STATUS_OK) {
			QLSPI_ERR(" Read AHB sts reg FAILED in ahb write \n");
			return SPI_STATUS_ERR;
		}

		if(counter++ >= MAX_WAIT)
			break;

	} while((aucBuffer[0] & 0x01) != 0);

	if (counter++ >= MAX_WAIT) {
		err = SPI_STATUS_ERR_AHB;
		QLSPI_ERR(" Failed to write data, AHB busy\n");
		//hw_reboot_exec(REBOOT_TYPE_HW);

		if(err < 0)
			return err;
		
		
	}

RETRY:


	/*Make last two bits 1*/
	addr |= 0x03; 	// prevent automatic trigger whe mem addr 1 is written

	addrPtr = (uint8_t *)(&addr);

//					tlc_dump();


	if (sz % 4) {
		QLSPI_DBG(" Write size is unaligned %d b \n",sz);

		padsize = sz + (4 - (sz % 4));

		QLSPI_DBG("  Aligned %d b \n",padsize);
		padsize++;
		padbuf=os_malloc(padsize + 8);
		if (NULL == padbuf) {
			QLSPI_ERR("SPI Write to S3 Address %x Failed\n",addr);
			return QL_STATUS_ERROR;
		}
		memset(padbuf,0,padsize+8);
		memcpy(padbuf,(uint8_t *)&addr,4);
		memcpy(padbuf+8,buf,sz);

		err = tlc_reg_write( SPITLC_MEM_ADDR_BYTE_0, padbuf, padsize+8);
		//err = tlc_reg_write( SPITLC_MEM_DATA_BYTE_0, padbuf, padsize);
		os_free(padbuf);
	} else
		{

		/* If size is aligned to 4B
		 * then two extra clock cycles are required so send one extra byte */
			//QLSPI_DBG(" Aligned %d b \n",sz);
			sz++;

			padbuf=os_malloc(sz+8);

			if(NULL==padbuf)
			{
				QLSPI_ERR("SPI Write to S3 Address %x Failed\n",addr);
				return QL_STATUS_ERROR;
			}

			memset(padbuf,0,sz+8);
			memcpy(padbuf,(uint8_t *)&addr,4);
			memcpy(padbuf+8,buf,sz);

			err = tlc_reg_write( SPITLC_MEM_ADDR_BYTE_0, padbuf, sz+8);

			os_free(padbuf);
		}
//			tlc_dump();

		if(err < 0){
			if(rty_count-- > 0)
				goto RETRY;

			QLSPI_ERR("Failed to write SPITLC_MEM_DATA_BYTE_0\n");

			err = SPI_STATUS_ERR_AHB;	

			return err;
		}
    
		QLSPI_DBG("End\n");

		return err;
}


int shub_ahb_read(  unsigned int addr, u8 *readBuf, unsigned int size)
{
    int err=SPI_STATUS_OK;
    uint32_t counter;
	uint8_t aucBuffer[10] = {0};
	uint8_t sts=0;

	QLSPI_DBG("Start\n");

    if (size > MAX_AHB_READ_SIZE || size < MAX_AHB_READ_SIZE || size==0 )
    {
		QLSPI_ERR("Requested DMA read size is invalid , %u B \n",size);
		return SPI_STATUS_ERR;
    }

counter=0;
do
{
	err = read_ahb_status(&sts);
	if(err!=SPI_STATUS_OK)
	{
		QLSPI_ERR(" Read AHB sts reg FAILED in ahb normal read \n");
		return SPI_STATUS_ERR;
	}

	if(counter++ >= MAX_WAIT)
		break;

} while(sts & 0x01 );



if (counter++ >= MAX_WAIT)
{
	err = SPI_STATUS_ERR_AHB;
	QLSPI_ERR(" AHB busy for read\n");
	//hw_reboot_exec(REBOOT_TYPE_HW);

	if(err < 0)
		return err;
}


	memcpy(&aucBuffer[0],(u8*)&addr,4);

	err = tlc_reg_write(SPITLC_MEM_ADDR_BYTE_2, (u8*)&aucBuffer[2], 2);
	if(err < 0)
	{
		QLSPI_ERR(" Error writing memory address to TLC for normal AHB read , err %d \n",err);
		return err;
	}
	err = tlc_reg_write(SPITLC_MEM_ADDR_BYTE_0, (u8*)&aucBuffer[0], 2);
	if(err < 0)
	{
		QLSPI_ERR(" Error writing memory address to TLC for normal AHB read , err %d \n",err);
		return err;
	}

	counter=0;
	do
	{
		err = read_ahb_status(&sts);
		if(err!=SPI_STATUS_OK)
		{
			QLSPI_ERR(" Read AHB sts reg FAILED in ahb normal read \n");
			return SPI_STATUS_ERR;
		}

		if(counter++ >= MAX_WAIT)
			break;

	} while( !(sts & 0x02) );

	if (counter++ >= MAX_WAIT)
	{
		err = SPI_STATUS_ERR_AHB;
		QLSPI_ERR(" AHB busy for read, sts %x \n",sts);
		//hw_reboot_exec(REBOOT_TYPE_HW);

		if(err < 0)
			return err;
	}

	err = tlc_reg_read(SPITLC_MEM_DATA_BYTE_0, (u8*)readBuf, size);
	if(err < 0)
	{
		QLSPI_ERR(" Error read memory address to TLC for normal AHB read , err %d \n",err);
		return err;
	}


#ifdef OPT_AHB_WR

	//prev_op_ahb_w=0;

#endif

	QLSPI_DBG("End\n");
    return err;

}
int shub_ahb_read_dma(  unsigned int addr, u8 *readBuf, unsigned int size)
{
    int err=SPI_STATUS_OK;
    uint8_t dmaStatus=0;

#ifndef OPT_AHB_READ
    uint8_t *addrPtr;
#endif

    uint32_t dmaReadCount=0, counter=0;

#ifdef OPT_AHB_READ

	uint8_t buf[8]={0};

#endif

#ifdef QLSPI_PADDING_EN

	uint8_t* padbuf=NULL;
	unsigned int padsize=0;

#endif

	QLSPI_DBG("Start\n");

    if (size > MAX_AHB_BURST_READ_SIZE || size == 0 )
    {
		QLSPI_ERR("Requested DMA read size is too big to fit in RX buffer\n");
		return SPI_STATUS_ERR;
    }

#ifndef OPT_AHB_READ

    addrPtr = (uint8_t *)(&addr);

    //Send 4 bytes of address to DMA

    err = tlc_reg_write(SPITLC_DMA_ADD0, addrPtr, 4);
    if(err < 0)
         return err;


    /*DMA Read size is in Bytes- 4 : eg to read 8 bytes we need to write 4*/

	dmaReadCount = size - 4;

    //Write size of the bytes to read using DMA

	addrPtr = (uint8_t *)(&dmaReadCount);


	//Send "size of data" DMA need to read

	#ifdef OPT_READ_BURST_SIZE
	err = tlc_reg_write( SPITLC_DMA_BRUST_SIZE0, addrPtr, 2); // 4B , as two dummy bytes cycles required
	#else
	err = tlc_reg_write( SPITLC_DMA_BRUST_SIZE0, addrPtr, 4);
	#endif

    if(err < 0)
     {
 		QLSPI_ERR("\n Failed to write DMA burst size 0\n");
 		return err;
   	}

#else	//OPT_AHB_READ

#ifdef QLSPI_PADDING_EN
	if (size % 4) {
		//QLSPI_DBG("Read op size is unaligned %d b \n",size);

		padsize = size + (4 - (size % 4));

		//QLSPI_DBG("Aligned %d b \n",padsize);

		padbuf=os_malloc(padsize);

		if(NULL==padbuf)
		{
			QLSPI_ERR(" RD padbuff alloc failed \n");
			return SPI_STATUS_ERR;
		}
		memset(padbuf, 0, padsize);

		dmaReadCount = padsize - 4;
	} else {
		padsize = size;
		dmaReadCount = size - 4;
//		QLSPI_DBG(" Aligned %d b \n",padsize);
	}
#else
		dmaReadCount = size - 4;
#endif

	memcpy(buf,&addr,4);

#ifdef OPT_READ_BURST_SIZE
	memcpy((buf+4),&dmaReadCount,2);
#else
	memcpy((buf+4),&dmaReadCount,4);
#endif

	//Write addr and data

#ifdef OPT_READ_BURST_SIZE
	err = tlc_reg_write( SPITLC_DMA_ADD0, buf, 6);
#else
	err = tlc_reg_write( SPITLC_DMA_ADD0, buf, 8);
#endif

	if (err < 0) {
		QLSPI_ERR("\n Failed to write DMA burst size 0\n");
		return err;
	}

#endif

	do{
		err = shub_read_dma_status(&dmaStatus);

		if (err != SPI_STATUS_OK) {
			QLSPI_ERR("Failed to read DMA status reg \n");
			return err;
		}

		if(counter >= MAX_WAIT)
			break;

		//Wait untill data is ready
	} while(((dmaStatus & 0x01) == 1) && (counter++ < MAX_WAIT));

	if (counter >= MAX_WAIT) {
		err = SPI_STATUS_ERR_DMA;
		QLSPI_ERR(" Timeout waiting for DMA busy clear\n");
		//hw_reboot_exec(REBOOT_TYPE_HW);
	}


	/* Read DMA data */
	if(err == SPI_STATUS_OK) {
#ifdef QLSPI_PADDING_EN
		if (size % 4) {

		//QLSPI_DBG("Ash underflow spi_underflow_flag = %d\n", spi_underflow_flag);
#ifdef DMA_FIFO_UNDERFLOW_EMULATE
			if (spi_underflow_flag == 1) {
				QLSPI_DBG("Reading bytes %d b \n",padsize + 4);
				err = tlc_reg_read(SPITLC_DMA_READ_DATA, padbuf,
					padsize + 4);
			} else if (spi_underflow_flag == 2) {
				QLSPI_DBG("Reading bytes %d b \n",padsize - 4);
				err = tlc_reg_read(SPITLC_DMA_READ_DATA, padbuf,
					padsize - 4);
			} else
#endif
			{
				err = tlc_reg_read(SPITLC_DMA_READ_DATA, padbuf,
					padsize);
			}

#ifdef DMA_FIFO_UNDERFLOW_CHECK
			/* Checking for DMA underflow */
			err = shub_read_dma_status(&dmaStatus);
			if(err != SPI_STATUS_OK) {
				QLSPI_ERR("Failed to read DMA status reg \n");
				return err;
			}

			if (dmaStatus & DMA_FIFO_UNDERFLOW) {
				QLSPI_ERR("DMA FIFO hit underflow\n");
			//	return SPI_STATUS_ERR;
			}
#endif
			memcpy(readBuf, padbuf, size);

			os_free(padbuf);
		} else
#endif
		{
#ifdef DMA_FIFO_UNDERFLOW_EMULATE
			if (spi_underflow_flag == 1) {
				QLSPI_DBG(" Reading bytes %d b \n",size + 4);
				err = tlc_reg_read(SPITLC_DMA_READ_DATA,
					readBuf, size + 4);
			} else if (spi_underflow_flag == 2) {
				QLSPI_DBG(" Reading bytes %d b \n",size - 4);
				err = tlc_reg_read(SPITLC_DMA_READ_DATA,
					readBuf, size - 4);
			} else
#endif
			{
				err = tlc_reg_read(SPITLC_DMA_READ_DATA,
					readBuf, size);
			}

#ifdef DMA_FIFO_UNDERFLOW_CHECK
			/* Checking for DMA underflow */
			err = shub_read_dma_status(&dmaStatus);
			if(err != SPI_STATUS_OK) {
				QLSPI_ERR("Failed to read DMA status reg \n");
				return err;
			}

			if (dmaStatus & DMA_FIFO_UNDERFLOW) {
				QLSPI_ERR("DMA FIFO hit underflow condition\n");
				//return SPI_STATUS_ERR;
			}
#endif

		}
	}


#ifdef OPT_AHB_WR

	//prev_op_ahb_w=0;

#endif

	QLSPI_DBG("End\n");

    return err;

}

void clear_intr_sts_s3(void)
{
    unsigned int reg_clr_val = 0;
    int status = SPI_STATUS_OK;

	QLSPI_DBG("Start\n");

    status = shub_ahb_write(SW_INTR_2_REG, (unsigned char *)&reg_clr_val, 4); 
    if(status < SPI_STATUS_OK){
        QLSPI_ERR("Clearing M4 INTR STS FAILED\n");
		handle_shub_ahb_status(status);	
    }

	QLSPI_DBG("Clearing M4 INTR done\n");

}

#ifdef FABRIC_LOADER


void clear_intr_sts_fabric(void)
{
    unsigned int reg_clr_val = 1;
    int status = SPI_STATUS_OK;

	QLSPI_DBG("Start\n");
    status = shub_ahb_write(FABRIC_INTR_STS_REG, (unsigned char *)&reg_clr_val, 4); 
    if(status < SPI_STATUS_OK) {
        QLSPI_ERR("Clearing Fabric INTR STS FAILED\n");
	handle_shub_ahb_status(status);	    
    }
	QLSPI_DBG("End\n");
}
EXPORT_SYMBOL_GPL(clear_intr_sts_fabric);

#endif

#ifndef OPT_INTR

void dis_intr_from_s3()
{
    unsigned int reg_clr_val = 0;
    int status = SPI_STATUS_OK;

	QLSPI_DBG("Start\n");
    status = shub_ahb_write(SW_INTR_2_EN_AP_REG, (unsigned char *)&reg_clr_val, 4); 
    if(status < SPI_STATUS_OK) {
        QLSPI_ERR(" Disabling intr from S3 FAILED \n");
		handle_shub_ahb_status(status);	
    }
	QLSPI_DBG("End\n");
}

#endif

void en_intr_to_s3()
{

    unsigned int reg_clr_val = 1;
    int status = SPI_STATUS_OK;

	QLSPI_DBG("Start\n");
    status = shub_ahb_write(SW_INTR_1_EN_M4, (unsigned char *)&reg_clr_val, 4); 
    if(status < SPI_STATUS_OK) {
        QLSPI_ERR(" Enabling intr from S3 FAILED \n");
		handle_shub_ahb_status(status);
    }
	QLSPI_DBG("End\n");
}

#ifndef OPT_INTR

void en_intr_from_s3()
{

    unsigned int reg_clr_val = 1;

    int status = SPI_STATUS_OK;

	QLSPI_DBG("Start\n");
    status = shub_ahb_write(SW_INTR_2_EN_AP_REG, (unsigned char *)&reg_clr_val, 4); 
    if(status < SPI_STATUS_OK) {
        QLSPI_ERR(" Enabling intr from S3 FAILED \n");
		handle_shub_ahb_status(status);

    }
	QLSPI_DBG("End\n");
}

#endif

 #ifdef FABRIC_LOADER

void en_intr_from_fabric(void)
{

    unsigned int reg_clr_val = 1;
	int status = SPI_STATUS_OK;
	reg_clr_val = 0;

	QLSPI_DBG("Start\n");
    status = shub_ahb_write(FABRIC_INTR_TYPE, (unsigned char *)&reg_clr_val, 4); 
    if(status < SPI_STATUS_OK) {
        QLSPI_ERR(" Write FAILED \n");
		handle_shub_ahb_status(status);
    }

	reg_clr_val = 1;

    status = shub_ahb_write(FABRIC_INTR_POL, (unsigned char *)&reg_clr_val, 4); 
    if(status < SPI_STATUS_OK) {
        QLSPI_ERR(" Write FAILED \n");
	handle_shub_ahb_status(status);
    }
    status = shub_ahb_write(FABRIC_INTR_EN_AP, (unsigned char *)&reg_clr_val, 4); 
    if(status < SPI_STATUS_OK) {
        QLSPI_ERR(" Write FAILED \n");
	handle_shub_ahb_status(status);
    }
	QLSPI_DBG(" Fabric regs \n");
}
#endif
unsigned char read_devid(void)
{
	unsigned char devid=0;

	QLSPI_DBG("Start\n");
	if(tlc_reg_read(SPITLC_DEVICE_ID_BYTE ,&devid,1)!=SPI_STATUS_OK)
	{
		return 0;
	}

	QLSPI_DBG("  Device ID 0x%x \n",devid);

	if(devid!=EOSS3_SPI_DEVID)
	{
		QLSPI_ERR(" Device ID read FAILED , ret 0x%x \n",devid);

		return 0;
	}

	QLSPI_DBG("End\n");
	return devid;
}
EXPORT_SYMBOL_GPL(read_devid);


/* SPI Initialization routine which initializes SPI related configured required
   to perform operations on SPI Bus using SPI HCD as Master and SPI Slave. This
   routine also setup the platform related configurations required such as
   GPIO, Chip Select, IRQ, etc.
 */
QL_Status QLSPI_Init (struct QLSPI_Platform *spi_plat)
{
	int ql_status = QL_STATUS_OK;

	QLSPI_DBG("Start\n");
	if (spi_plat == NULL)
	{
		QL_TRACE_SPI_ERROR("Bad Parameter: spi_plat is Null \n");
		return QL_STATUS_ERROR_BAD_PARAMETER;
	}

	/* Initializing/Creating mutex lock*/
	ql_status = ql_mutex_init();
	if (ql_status != QL_STATUS_OK)
	{
		QL_TRACE_SPI_ERROR("Initializing/creating mutex Failed\n");
		return QL_STATUS_ERROR;
	}

	ql_mutex_lock();

	/* Calling OS specific routine to do the SPI Initialization */
	ql_status = ql_spi_init(spi_plat);
	if (ql_status != QL_STATUS_OK)
	{
		ql_mutex_unlock();
		QL_TRACE_SPI_ERROR("SPI Initialization Failed \n");
		return QL_STATUS_ERROR;
	}

	ql_mutex_unlock();

	QLSPI_DBG("End\n");
	return QL_STATUS_OK;
}

/*
   This routine should perform read on SPI Bus. Reads data from S3 Memory using
   SPI Bus
 */
QL_Status QLSPI_Read_S3_Mem (uint32_t addr, uint8_t *data, uint32_t len)
{
	int ql_status = QL_STATUS_OK;
	uint32_t data_len = len;
	uint32_t curr_addr = 0;
	uint32_t next_boundary = 0;
	uint32_t bytes_to_read = 0;

	QLSPI_DBG("Start\n");
	if(ql_spi_get_eoss3_status() == DEAD){
		QL_TRACE_SPI_ERROR("EOSS3 is dead. \n");
		return -1;
	}

	if (len == 0)
	{
		QL_TRACE_SPI_ERROR("Bad Parameter: len is Null,len = %d \n",len);
		return QL_STATUS_OK;
	}

	if (data == NULL)
	{
		QL_TRACE_SPI_ERROR("Bad Parameter: data is Null \n");
		return QL_STATUS_ERROR_BAD_PARAMETER;
	}


	if(addr&0x3)
	{
		QL_TRACE_SPI_ERROR("Bad Parameter: Addr not dword(32bit) aligned\n");
		return QL_STATUS_ERROR_BAD_PARAMETER;
	}

	curr_addr = addr;

	do {
		next_boundary = ((curr_addr + 0x10000) & 0xFFFF0000);

		if((data_len + curr_addr) <= next_boundary)
			bytes_to_read = data_len;
		else
			bytes_to_read = next_boundary - curr_addr;

		ql_mutex_lock();
		/* Calling OS Specific routine to read to S3 Memory using SPI HCD */
		ql_status = shub_ahb_read_dma(curr_addr, data, bytes_to_read);
		if (ql_status != QL_STATUS_OK) {
			ql_mutex_unlock();
			QL_TRACE_SPI_ERROR("SPI Read to S3 Address %x Failed [%d]\n",
			                    addr, ql_status);
			return ql_status;
		}
		ql_mutex_unlock();

		curr_addr += bytes_to_read;
		data_len -= bytes_to_read;
		data += bytes_to_read;

	}while(data_len);

	QLSPI_DBG("End\n");
	return QL_STATUS_OK;
}

/*
   This routine should perform write on SPI Bus. Write data to S3 Memory using
   SPI Bus
 */
QL_Status QLSPI_Write_S3_Mem (uint32_t addr, uint8_t *data, uint32_t len)
{
	int ql_status = QL_STATUS_OK;
	uint32_t data_len = len;
	uint32_t curr_addr = 0;
	uint32_t next_boundary = 0;
	uint32_t bytes_to_write = 0;
	uint8_t *data_ptr = data;

	QLSPI_DBG("Start\n");
	if(ql_spi_get_eoss3_status() == DEAD){
		QL_TRACE_SPI_ERROR("EOSS3 is dead. \n");
		return -1;
	}
#ifdef WRITE_VERIFY

	uint8_t * rx_data;
	int i=0;
#endif

	if (len == 0)
	{
		QL_TRACE_SPI_ERROR("Bad Parameter: len is Null,len = %d \n",len);
		return QL_STATUS_OK;
	}
	if (data == NULL)
	{
		QL_TRACE_SPI_ERROR("Bad Parameter: data is Null \n");
		return QL_STATUS_ERROR_BAD_PARAMETER;
	}

	if(addr&0x3)
	{
		QL_TRACE_SPI_ERROR("Bad Parameter: Addr not dword(32bit) aligned\n");
		return QL_STATUS_ERROR_BAD_PARAMETER;
	}

	curr_addr = addr;

	do {
		next_boundary = ((curr_addr + 0x10000) & 0xFFFF0000);

		if((data_len + curr_addr) <= next_boundary)
			bytes_to_write = data_len;
		else
			bytes_to_write = next_boundary - curr_addr;

		ql_mutex_lock();
		/* Calling OS Specific routine to write to S3 Memory using SPI HCD */
		ql_status = shub_ahb_write(curr_addr, data_ptr, bytes_to_write);
		if (ql_status != SPI_STATUS_OK) {
			ql_mutex_unlock();
			QL_TRACE_SPI_ERROR("SPI Write to S3 Address %x Failed [%d]\n",
			                        addr, ql_status);
			//handle_shub_ahb_status(ql_status);
			return ql_status;
		}
		ql_mutex_unlock();

		curr_addr += bytes_to_write;
		data_len -= bytes_to_write;
		data_ptr += bytes_to_write;

	}while(data_len);


#ifdef WRITE_VERIFY

		QL_TRACE_SPI_DEBUG(" Verifing write \n");

		rx_data=os_malloc(len);

		if(NULL==rx_data)
		{
			QL_TRACE_SPI_ERROR(" Verify buf alloc failed \n");
			goto END;
		}

		ql_status = QLSPI_Read_S3_Mem(addr, rx_data, len);
		if (ql_status != QL_STATUS_OK)
		{
			QL_TRACE_SPI_ERROR(" verify read failed addr = 0x%x, len = %d\n", addr, len);
		os_free(rx_data);
			goto END;
		}
		for(i=0;i<len;i++)
		{
			if(data[i]!=rx_data[i])
			{
				QL_TRACE_SPI_ERROR(" verify failed at %d , data %x , rx_data %x\n",i,data[i],rx_data[i]);
				os_free(rx_data);
				goto END;
			}
		}

		os_free(rx_data);

		QL_TRACE_SPI_DEBUG(" Verifing write DONE\n");

END:

#endif


	//ql_mutex_unlock();

	QLSPI_DBG("End\n");
	return QL_STATUS_OK;
}

/*  This routine registers the user defined ISR to QLSPI.
	This routine need to be called during initialization to register the
	ISR handler. This handler will be called when interrupt is generated
	from EOSS3 M4
 */
QL_Status QLSPI_Register_Isr (QLSPI_Isr_Handler handler)
{
	int ql_status = QL_STATUS_OK;

	QLSPI_DBG("Start\n");
	if (handler == NULL)
	{
		QL_TRACE_SPI_ERROR("Bad Parameter: handler is Null \n");
		return QL_STATUS_ERROR_BAD_PARAMETER;
	}

	ql_mutex_lock();

	/* Save the id and user defined ISR handler */
	ql_status = store_spi_dev_isr(handler);
	if (ql_status != QL_STATUS_OK)
	{
		ql_mutex_unlock();
		QL_TRACE_SPI_ERROR("Registering User defined ISR Failed \n");
		return QL_STATUS_ERROR;
	}

	ql_mutex_unlock();

	QLSPI_DBG("End\n");
	return QL_STATUS_OK;
}

/*
 * This routine generate an interrupt to EOSS3 M4
 */
QL_Status QLSPI_Trigger_Intr (void)
{
	int ql_status = QL_STATUS_OK;

	QLSPI_DBG("Start\n");
	ql_mutex_lock();

	/* Calling OS Specific routine to generate interrupt to EOSS3 M4
	   by writing to SPITLC registers using SPI HCD
	 */
	ql_status = ql_spi_trigger_intr();
	if (ql_status != QL_STATUS_OK)
	{
		ql_mutex_unlock();
		QL_TRACE_SPI_ERROR("Interrupt Trigger to EOSS3 M4 Failed\n");
		return QL_STATUS_ERROR;
	}

	ql_mutex_unlock();

	QLSPI_DBG("End\n");
	return QL_STATUS_OK;
}
/*
 * This routine clear interrupt status from s3(M4 to AP) interrupt
 */
QL_Status QLSPI_Clear_S3_Intr (void)
{
	int ql_status = QL_STATUS_OK;
	unsigned int reg_clr_val = 0;

	QLSPI_DBG("Start\n");
	ql_mutex_lock();
	
	ql_status = shub_ahb_write(SW_INTR_2_REG, (unsigned char *)&reg_clr_val, 4);
	if(ql_status < SPI_STATUS_OK) {
		ql_mutex_unlock();
     		QLSPI_ERR("Clearing S3 interrupt status failed\n");
		handle_shub_ahb_status(ql_status);
		ql_status=QL_STATUS_ERROR;
		return ql_status;
	}

	ql_mutex_unlock();

	QLSPI_DBG("End\n");
	return ql_status;
}
