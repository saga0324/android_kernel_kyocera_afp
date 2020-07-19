 /*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2019 KYOCERA Corporation
 */
 /*  qlmcu.c
  *  Copyrighted (C) 2016 QuickLogic Corp
  *
  *  This file contains source to download the firmware
  *  into S3 Memory. It provides an API exposed to application
  *  which will download the firmware into S3 Memory.
  *  This file provides API which uses QLSPI API to download
  *  the firmware.
  *  This API does the below things required to download the
  *  firmware into S3 Memory.
  *   	- Reset S3 MCU
  *     - Load firmware into S3 Memory
  *     - Release S3 MCU from reset
  */

#include <linux/ql/os.h>
#include <linux/ql/qlmcu.h>
#include <linux/ql/qlspi_tlc.h>
#include <linux/ql/qltrace.h>
#include <linux/crc32.h>



#define VALIDATE_FFE_MEM_WRITE  0

#if (VALIDATE_FFE_MEM_WRITE == 1)
#define READBACK_FFE_CFGMEM	1
#define READBACK_FFE_DMMEM	1
#define READBACK_FFE_SM0MEM	1
#define READBACK_FFE_SM1MEM	1
#define READBACK_FFE_CMMEM	1
#else
#define READBACK_FFE_CFGMEM	0
#define READBACK_FFE_DMMEM	0
#define READBACK_FFE_SM0MEM	0
#define READBACK_FFE_SM1MEM	0
#define READBACK_FFE_CMMEM	0
#endif  /* VALIDATE_FFE_MEM_WRITE */

//static uint8_t ucRxBuf[10];
//volatile uint32_t *FFE_CFG = (volatile uint32_t*)0x2007FE80;
//uint32_t * const FFE_DM0 = (uint32_t*)0x40040000;
//uint32_t * const FFE_CM  = (uint32_t*)0x40050000;
//uint32_t * const FFE_SM0  = (uint32_t*)0x40044000;
//uint32_t * const FFE_SM1  = (uint32_t*)0x40048000;
uint32_t FFE_CFG = 0x2007FE80;
uint32_t FFE_DM0 = 0x40040000;
uint32_t FFE_CM  = 0x40050000;
uint32_t FFE_SM0 = 0x40044000;
uint32_t FFE_SM1 = 0x40048000;
//#ifdef CRC_CHECK
//QL_Status read_s3_mem(uint32_t addr, void *data, uint32_t len);

#if CRC_CHECK_VERSION == 2
#ifdef FFE_LOADER
static bool ffe_has_crc_header = false;
#endif
static bool m4_has_crc_header = false;

uint32_t m4_crc = 0;
uint32_t ffe_crc = 0;
uint32_t m4_bin_size = 0;
#ifdef FABRIC_LOADER
uint32_t fab_bin_size = 0;
#endif
uint8_t ffe_data[16384];
uint32_t ffe_data_bytes = 0;
//#else
#endif  //CRC_CHECK_VERSION
uint8_t bin_array[MAX_SECTION_SIZE] = {0};

/*Function Declarations*/
int release_slave_mcu(void);
int reset_slave_mcu(void);


#if defined (FABRIC_LOADER) &&  (CRC_CHECK_VERSION == 2)
static atomic_t is_fabric_load;
#endif
/*Function Definitions*/

int32_t disp_reset_reg(uint32_t* slave_por_addr)
{
	QL_Status ql_status;

	QL_TRACE_MCU_DEBUG("Start\n");
	*slave_por_addr =0;

	ql_status = QLSPI_Read_S3_Mem(MISC_POR_0_ADDR,(uint8_t*)slave_por_addr, 4);

	QL_TRACE_MCU_DEBUG(" *** Reset reg %x \n",*slave_por_addr);

	if (ql_status != QL_STATUS_OK)
	{
		QL_TRACE_MCU_ERROR(" ### Failure reading RESET reg \n");
		return -1;
	}

	QL_TRACE_MCU_DEBUG("End\n");
	return 0;
}

#define AIP_OSC_OK_LOCK_READ_MAX_RETRY			1500

#ifdef CMOS_CLK_INPUT
/* Change reg CMOS CLK Mode */
/* spi speed must be low when use this function. */
int initialize_s3_host_interface(void)
{
#ifndef CONFIG_QL_MICON_USE_EXTERNAL_SLEEPCLK
	int ql_status = QL_STATUS_OK;
	uint32_t aip_rtc_reg_read = 0x00UL;

	uint32_t aip_rtc_reg_val = 0x00UL; /* for AIP registers */
#endif /* CONFIG_QL_MICON_USE_EXTERNAL_SLEEPCLK */

	uint32_t retry_count = 0;
	uint32_t osc_lock_data_1 = 0x00UL;
	uint32_t osc_lock_data_2 = 0x00UL;

	QL_TRACE_MCU_DEBUG(" Before Initialize S3 Host Interface \n");
#ifndef CONFIG_QL_MICON_USE_EXTERNAL_SLEEPCLK

	// for CMOS-clock based HW configuration, ensure that
	// the AIP registers are set correctly
	aip_rtc_reg_val = 0x5; 

	ql_status = QLSPI_Write_S3_Mem(AIP_RTC_CTRL_2 ,(uint8_t *)&aip_rtc_reg_val, 4);
	if (ql_status != QL_STATUS_OK)
	{
		QL_TRACE_MCU_ERROR("Failure in writing AIP_RTC_CTRL_2 \n");
		return QL_STATUS_ERROR;
	}
	QLSPI_Read_S3_Mem(AIP_RTC_CTRL_2,(uint8_t*)&aip_rtc_reg_read, 4);
	QL_TRACE_MCU_DEBUG(" *** AIP_RTC_CTRL_2 reg %x \n",aip_rtc_reg_read);

	aip_rtc_reg_val = 0x1A;
	ql_status = QLSPI_Write_S3_Mem(AIP_RTC_CTRL_7 ,(uint8_t *)&aip_rtc_reg_val, 4);
	if (ql_status != QL_STATUS_OK)
	{
		QL_TRACE_MCU_ERROR("Failure in writing AIP_RTC_CTRL_7 \n");
		return QL_STATUS_ERROR;
	}
	QLSPI_Read_S3_Mem(AIP_RTC_CTRL_7,(uint8_t*)&aip_rtc_reg_read, 4);
	QL_TRACE_MCU_DEBUG(" *** AIP_RTC_CTRL_7 reg %x \n",aip_rtc_reg_read);

	aip_rtc_reg_val = 0x18;
	ql_status = QLSPI_Write_S3_Mem(AIP_RTC_CTRL_7 ,(uint8_t *)&aip_rtc_reg_val, 4);
	if (ql_status != QL_STATUS_OK)
	{
		QL_TRACE_MCU_ERROR("Failure in writing AIP_RTC_CTRL_7 \n");
		return QL_STATUS_ERROR;
	}
	QLSPI_Read_S3_Mem(AIP_RTC_CTRL_7,(uint8_t*)&aip_rtc_reg_read, 4);
	QL_TRACE_MCU_DEBUG(" *** AIP_RTC_CTRL_7 reg %x \n",aip_rtc_reg_read);

#endif /* CONFIG_QL_MICON_USE_EXTERNAL_SLEEPCLK */

	do
	{
		/* Requires two consecutive read to confirm register value. */
		QLSPI_Read_S3_Mem(AIP_OSC_STA_0, (uint8_t*)&osc_lock_data_1, 4); // read first time
		QLSPI_Read_S3_Mem(AIP_OSC_STA_0, (uint8_t*)&osc_lock_data_2, 4); // read second time
		usleep_range(10000, 10000); // 10msec gap
		retry_count += 10; // use same retry_count variable here due to 10msec gap everytime
		QL_TRACE_MCU_ERROR(" OSC LOCK retry %x\n", retry_count);
	}while(!((osc_lock_data_1 & AIP_OSC_STA_HOSCLOCK) && (osc_lock_data_2 & AIP_OSC_STA_HOSCLOCK)) && (retry_count < AIP_OSC_OK_LOCK_READ_MAX_RETRY)); // both reads should have BIT0 set to 1 and timeout

	if(retry_count >= AIP_OSC_OK_LOCK_READ_MAX_RETRY)
	{
		// abort.
		QL_TRACE_MCU_ERROR(" OSC LOCK status is unlocked %x, %x\n", osc_lock_data_1, osc_lock_data_2);
		return QL_STATUS_ERROR;
	}

	QL_TRACE_MCU_DEBUG(" After Initialize S3 Host Interface \n");

	return QL_STATUS_OK;
}
#endif

int reset_slave_mcu(void)
{
	int ql_status = QL_STATUS_OK;
	uint32_t slave_por_addr = 0x00UL; /* Keep the MCU in reset mode */

	int32_t sts;
#ifdef ACCESS_CTRL_ONCE
		uint8_t aucBuffer;
		int rty_count = 2;
#endif

	QL_TRACE_MCU_DEBUG(" Reset reg before RESET \n");

	sts=disp_reset_reg(&slave_por_addr);

	if(sts<0)
	{
		QL_TRACE_MCU_ERROR(" Failure reading reset reg MISC_POR_0_ADDR \n");
		return QL_STATUS_ERROR;
	}

	if(slave_por_addr)
	{
		QL_TRACE_MCU_DEBUG(" Resetting M4 \n");

		slave_por_addr = 0x00UL;

		ql_status = QLSPI_Write_S3_Mem(MISC_POR_0_ADDR,(uint8_t *)&slave_por_addr, 4);
		if (ql_status != QL_STATUS_OK)
		{
			QL_TRACE_MCU_ERROR("Failure in resetting mcu \n");
			return QL_STATUS_ERROR;
		}

		QL_TRACE_MCU_DEBUG(" Reset reg after RESET \n");

		slave_por_addr=0;

		disp_reset_reg(&slave_por_addr);

		return QL_STATUS_OK;
	}

	QL_TRACE_MCU_DEBUG(" M4 already in reset state %x \n",slave_por_addr);
#ifdef ACCESS_CTRL_ONCE

		RETRY:
		aucBuffer = 0x3;

		QL_TRACE_MCU_DEBUG(" Writing ACCESS CTRL \n");

		ql_status = tlc_reg_write( SPITLC_AHB_ACCESS_CTL, &aucBuffer, ONE_BYTE);	// prevent automatic trigger when mem addr	is written for write operations

		if(ql_status < 0){

			if(rty_count-- > 0)
				goto RETRY;

			QL_TRACE_MCU_ERROR("Failed to write SPITLC_AHB_ACCESS_CTL\n");
			return QL_STATUS_ERROR;

		}
		QL_TRACE_MCU_DEBUG(" Writing ACCESS CTRL DONE \n");

#endif


	return QL_STATUS_OK;
}
EXPORT_SYMBOL_GPL(reset_slave_mcu);


int release_slave_mcu(void)
{
	int ql_status = QL_STATUS_OK;
	uint32_t slave_por_addr = 0x01UL; /* Keep the MCU in reset mode */

	int32_t sts;

	slave_por_addr=0;

	QL_TRACE_MCU_DEBUG(" Reset reg before release \n");

	sts=disp_reset_reg(&slave_por_addr);

	if(sts<0)
	{
		QL_TRACE_MCU_ERROR(" Failure reading reset reg MISC_POR_0_ADDR \n");
		return QL_STATUS_ERROR;
	}

	if(!slave_por_addr)
	{
		slave_por_addr=1;

		ql_status = QLSPI_Write_S3_Mem(MISC_POR_0_ADDR,(uint8_t *)&slave_por_addr, 4);
		if (ql_status != QL_STATUS_OK)
		{
			QL_TRACE_MCU_ERROR("Failure in releasing mcu from reset \n");
			return QL_STATUS_ERROR;
		}

#ifdef OS_MSLEEP
//		os_msleep(200);	//for DMA status timeout
#endif
		QL_TRACE_MCU_DEBUG(" Reset reg after release \n");

		slave_por_addr=0;

		disp_reset_reg(&slave_por_addr);

	}
	else
	{
		QL_TRACE_MCU_DEBUG(" M4 RESET already released\n");
	}

	QL_TRACE_MCU_DEBUG("End\n");

	return QL_STATUS_OK;
}
EXPORT_SYMBOL_GPL(release_slave_mcu);




uint32_t calculate_verify_crc(section_info *section)
{
	uint32_t crc;
	/* Reading Section from S3 Memory  and add it into array*/
	if (QL_STATUS_ERROR == QLSPI_Read_S3_Mem(section->addr, bin_array, section->size)) {
		QL_TRACE_MCU_ERROR(" Read back S3 mem for CRC failed\n");
		return QL_STATUS_ERROR;
	}
#if 0
	if(section->size < 0x129) {
		int o=0;
		for(o=0; o<section->size; o++)
			QL_TRACE_MCU_DEBUG("0x%x\n",bin_array[o]);
	}
#endif
	/*CRC-32 does a pre- and post-exclusive-or with 0xffffffff.
	But Linux CRC doesn't do that. We need to do manually.
	* Here, we are skipping post-exclusive-or with 0xffffffff
	and using big endian crc function since CRC build tool is using BE*/
	crc = crc32_be(0^ 0xffffffff, bin_array, section->size);

	QL_TRACE_MCU_DEBUG(" Calculated CRC = 0x%x, Org CRC = 0x%x\n", crc, section->crc);

	if(crc != section->crc) {
		QL_TRACE_MCU_ERROR(" CRC Verify Failed...\n");
		QL_TRACE_MCU_ERROR(" Calculated CRC = 0x%x, Org CRC = 0x%x\n", crc, section->crc);
		return QL_STATUS_ERROR;
	} else {
			QL_TRACE_MCU_DEBUG(" CRC Verify Done!!!\n");
	}

	return QL_STATUS_OK;
}

#if CRC_CHECK_VERSION == 2
uint32_t give_me(uint8_t what, uint8_t *source, uint32_t section)
{
	uint32_t ret = 0, i;

	QL_TRACE_MCU_DEBUG("Start\n");
	switch(what) {
		case SEC_ADDRESS:
			source += 4;				//skip first 4bytes.bcz it is binary CRC
			for(i=0; i < ((section-1)*8); i++){
				source++;
			}
			ret = *(uint32_t*)source;
			break;
		case SEC_SIZE:
			for(i=0; i < ((section*4)-4); i++){
				source++;
			}
			ret = *(uint32_t*)source;
			break;
		case SEC_CRC:
			source += 8;		//skip first 8bytes.bcz it is binary CRC, First Sections address
			for(i=0; i < ((section-1)*8); i++){
				source++;
			}
			ret = *(uint32_t*)source;
			break;

		default:
			QL_TRACE_MCU_DEBUG("That Field is not Available in Section!!!\n");
			break;
	}
	QL_TRACE_MCU_DEBUG("End\n");
	return ret;
}

#ifdef FABRIC_LOADER
QL_Status QLFAB_FW_Write (unsigned char *fw_raw_data, uint32_t size)
{
	int ql_status = QL_STATUS_OK;
	uint8_t *raw_addr = NULL;
	uint8_t buf[10];
	uint32_t fw_size = 0, num_sections = 0;
	section_info section;
	int i;
	/*This Fab binary may have Header or not.
	*
	* 1st byte = '#', then it has CRC Header.
	*
	*
	* If you found '#' then take the binary size in Words. CRC will be stored after that many bytes.
	*
	*
	*/
	raw_addr = (uint8_t *)fw_raw_data;

	if (*raw_addr == '#') {
		QL_TRACE_MCU_DEBUG("Fab Binary has CRC Header ('#' Detected)\n");
	} else {
		QL_TRACE_MCU_DEBUG("Fab Binary Doesn't have CRC Header('#' Not Detected)\n");
		return QL_STATUS_ERROR;
	}
	while(*(raw_addr++) != '#');	// raw_addr is now at character after #

	/* Reading Binary Size */
	for(i=0; *(raw_addr) !=' '; raw_addr++) {
		buf[i++] = *raw_addr;
	}
	buf[i] = '\0';
		/* convert string to uint32 */
	if (kstrtouint(buf, 0, &fw_size)) {
		QL_TRACE_MCU_ERROR("Invalid data\n");
		return QL_STATUS_ERROR;
	}
	fw_size *= 4;		//converting word into bytes
	fab_bin_size = fw_size;
	/* Reading Number of Section */
	raw_addr++;
	memset(buf, 0, sizeof(buf));
	for(i=0; *(raw_addr) !=' '; raw_addr++) {
		buf[i++] = *raw_addr;
	}
	buf[i] = '\0';

	/* convert string to uint32 */
	if (kstrtouint(buf, 0, &num_sections)) {
		QL_TRACE_MCU_ERROR("Invalid data\n");
		return QL_STATUS_ERROR;
	}
	QL_TRACE_MCU_DEBUG("Number of Sections in Fab Binary = %d \n", num_sections);
	while(*(raw_addr++) != '\n');	// raw_addr is now at character after \n

	ql_status = QLSPI_Write_S3_Mem(FABRIC_FW_DEST_ADDR, raw_addr, fw_size);
	if (ql_status != QL_STATUS_OK)
	{
		QL_TRACE_MCU_ERROR("Load of Firwmare in S3 Memory Failed \n");
		return QL_STATUS_ERROR;
	}

	section.addr = FABRIC_FW_DEST_ADDR;
	section.size = fw_size;
	section.crc = *(uint32_t *)(raw_addr + fw_size);

	//QL_TRACE_MCU_DEBUG("Section %d wants to do CRC verification\n", i+1);

	ql_status = calculate_verify_crc(&section);
	if (ql_status != QL_STATUS_OK)
	{
		QL_TRACE_MCU_ERROR(" CRC Verify of Fabric Bin Failed...\n");
		return QL_STATUS_ERROR;
	}

	return QL_STATUS_OK;
}
#endif //FABRIC_LOADER
QL_Status QLM4_Fw_Download (unsigned char *m4_fw_raw_data, uint32_t size)
{
	uint8_t *raw_addr = NULL;
	uint8_t buf[10];
	uint32_t fw_size = 0, num_sections = 0;
	uint8_t header[10*4];
	uint8_t footer[(10*8)+4];
	uint8_t container[10*12];
	uint8_t *dummy;
	int i;
#ifdef CRC_CHECK
	int ql_status = QL_STATUS_OK;
	section_info section;
#endif

	QL_TRACE_MCU_DEBUG("Start\n");

	/*This M4 binary may have Header or not.
	*
	* 1st byte = '#', then it has CRC Header.
	*
	*
	* If you found '#' then take the binary size in Words. CRC will be stored after that many bytes.
	*
	*
	*/
	raw_addr = (uint8_t *)m4_fw_raw_data;

	if (*raw_addr == '#') {
		QL_TRACE_MCU_DEBUG("M4 has CRC Header ('#' Detected)\n");
		m4_has_crc_header = true;
	} else {
		QL_TRACE_MCU_DEBUG("M4 Doesn't have CRC Header('#' Not Detected)\n");
		m4_has_crc_header = false;
	}

	if(m4_has_crc_header == true) {
		while(*(raw_addr++) != '#');	// raw_addr is now at character after #

		/* Reading Binary Size */
		for(i=0; *(raw_addr) !=' '; raw_addr++) {
			buf[i++] = *raw_addr;
		}
		buf[i] = '\0';

		/* convert string to uint32 */
		if (kstrtouint(buf, 0, &fw_size)) {
			QL_TRACE_MCU_ERROR("Invalid data\n");
			return QL_STATUS_ERROR;
		}

		fw_size *= 4;		//converting word into bytes

		m4_bin_size = fw_size;

		/* Reading Number of Section */
		raw_addr++;
		memset(buf, 0, sizeof(buf));
		for(i=0; *(raw_addr) !=' '; raw_addr++) {
			buf[i++] = *raw_addr;
		}
		buf[i] = '\0';

		/* convert string to uint32 */
		if (kstrtouint(buf, 0, &num_sections)) {
			QL_TRACE_MCU_ERROR("Invalid data\n");
			return QL_STATUS_ERROR;
		}
		QL_TRACE_MCU_DEBUG("Number of Sections in M4 = %d \n", num_sections);

		while(*(raw_addr++) != '\n');	// raw_addr is now at character after \n

		/* Copy Each Sections Size to header array */
		memcpy(header, raw_addr, num_sections*4);
		//raw_addr += (num_sections*4);

		/* Copy Binary's CRC, Each Section's CRC and Address to Footer array */
		memcpy(footer, (raw_addr + fw_size), 4+(8*num_sections));
	} else {

		fw_size = size;
		m4_bin_size = size;
	}

#ifdef FABRIC_LOADER
	if (atomic_read(&is_fabric_load) == true) {
		
		/*Flash the FPGA Programmer firmware to S3 mem*/
		if(QL_STATUS_OK != QLSPI_Write_S3_Mem( FABRIC_PROG_FW_DEST_ADDR, (uint8_t *)raw_addr + (num_sections*4), fw_size)) {
			QL_TRACE_MCU_ERROR("Load of FPGA Prog Firwmare in S3 Memory Failed \n");
			return QL_STATUS_ERROR;
		}

	} else 
#endif	//#ifdef FABRIC_LOADER
	{

		/*Flash the M4 firmware to S3 mem*/
		if(QL_STATUS_OK != QLSPI_Write_S3_Mem( BOOTL1_START_ADDR, (uint8_t *)raw_addr + (num_sections*4), fw_size)) {
			QL_TRACE_MCU_ERROR("Load of M4 Firwmare in S3 Memory Failed \n");
			return QL_STATUS_ERROR;
		}
	}

#if 0
	QL_TRACE_MCU_DEBUG(" 1st Section's Address = %p\n",give_me(SEC_ADDRESS, footer, 1));
	QL_TRACE_MCU_DEBUG(" 1st Section's CRC = %p\n",give_me(SEC_CRC, footer, 1));
	QL_TRACE_MCU_DEBUG(" 1st Section's Size = %p\n",give_me(SEC_SIZE, header, 1));
	QL_TRACE_MCU_DEBUG(" 2nd Section's Address = %p\n",give_me(SEC_ADDRESS, footer, 2));
	QL_TRACE_MCU_DEBUG(" 2nd Section's CRC = %p\n",give_me(SEC_CRC, footer, 2));
	QL_TRACE_MCU_DEBUG(" 2nd Section's Size = %p\n",give_me(SEC_SIZE, header, 2));
#endif
	dummy = container;

	*(uint32_t*) (dummy) = num_sections;
	dummy += 4;

	for(i=0; i<num_sections; i++) {
		*(uint32_t*)(dummy + 0) = give_me(SEC_ADDRESS, footer, i+1);
		*(uint32_t*)(dummy + 4) = give_me(SEC_SIZE, header, i+1);
		*(uint32_t*)(dummy + 8) = give_me(SEC_CRC, footer, i+1);
#if 1		
		QL_TRACE_MCU_DEBUG(" Section %d Address = %d\n", i+1, *(uint32_t*)(dummy + 0));
		QL_TRACE_MCU_DEBUG(" Section %d Size = %d\n", i+1, *(uint32_t*)(dummy + 4));
		QL_TRACE_MCU_DEBUG(" Section %d CRC = %d\n", i+1, *(uint32_t*)(dummy + 8));
#endif

#ifdef CRC_CHECK
		section.addr = *(uint32_t*)(dummy + 0);
		section.size = *(uint32_t*)(dummy + 4);
		section.crc = *(uint32_t*)(dummy + 8);

		if (section.size != 0){
			//ql_status = calculate_verify_crc(&section);
			if (ql_status != QL_STATUS_OK)
			{
#ifdef FABRIC_LOADER
				if (atomic_read(&is_fabric_load) == true) {
					QL_TRACE_MCU_ERROR(" CRC Verify of Fabric Prg (M4) Bin Failed...\n");
				}
				else
#endif
				{
					QL_TRACE_MCU_ERROR(" CRC Verify of M4 Bin Failed...\n");
				}
				return QL_STATUS_ERROR;
			}
		}
#endif
		dummy += 12;

	}

#ifdef FABRIC_LOADER
	if (atomic_read(&is_fabric_load) != true)
#endif	//#ifdef FABRIC_LOADER
	{
		if((QLSPI_Write_S3_Mem(M4_CRC_ADDR, container, 4+(num_sections*12))) != QL_STATUS_OK) {
			QL_TRACE_MCU_ERROR("Failure in Writing CRC to M4 \n");
			return QL_STATUS_ERROR;
		}
	}

	QL_TRACE_MCU_DEBUG("End\n");

	return QL_STATUS_OK;
}

#ifdef FFE_LOADER
QL_Status QLFFE_Fw_Download (unsigned char *ffe_fw_raw_data, uint32_t size)
{
	int ql_status = QL_STATUS_OK;
	uint32_t *buf;
#ifdef CRC_CHECK
	uint8_t buffer[10];
	uint32_t fw_size = 0, num_sections = 0;
	uint8_t header[10*4];
	uint8_t footer[(10*8)+4];
	uint8_t container[10*12];
	uint8_t *dummy;
	int i;
	section_info section;
#endif



	volatile uint32_t *exportVariableTab;
	volatile uint32_t *dmTab;
	volatile uint32_t *sm0Tab;
#ifdef FFE_IMAGE_HAS_HEADER_AND_SM1_SUPPORT
	volatile uint32_t *sm1Tab;
#endif // FFE_IMAGE_HAS_HEADER_AND_SM1_SUPPORT
	volatile uint32_t *cmTab;
	uint32_t exportVariableTabSzInWord = 0;
	uint32_t dmTabSzInWord = 0;
	uint32_t sm0TabSzInWord = 0;
#ifdef FFE_IMAGE_HAS_HEADER_AND_SM1_SUPPORT
	uint32_t sm1TabSzInWord = 0;
#endif // FFE_IMAGE_HAS_HEADER_AND_SM1_SUPPORT
	uint32_t cmTabSzInWord = 0;
#ifdef FFE_IMAGE_HAS_HEADER_AND_SM1_SUPPORT
	uint8_t* raw_data_ptr;
#endif // FFE_IMAGE_HAS_HEADER_AND_SM1_SUPPORT

	QL_TRACE_MCU_DEBUG("  FFE Fw size %d bytes \n",size);


#ifndef CRC_CHECK
#ifdef FFE_IMAGE_HAS_HEADER_AND_SM1_SUPPORT
	raw_data_ptr = (uint8_t *)ffe_fw_raw_data;
	while(*(raw_data_ptr++) != '$');	// raw_data_ptr is now at character after $
	while(*(raw_data_ptr++) != '\n');	// raw_data_ptr is now at character after \n, header parsing code is TODO

	buf = (uint32_t *)raw_data_ptr;
#endif // FFE_IMAGE_HAS_HEADER_AND_SM1_SUPPORT

#else //CRC_CHECK

#ifdef FFE_IMAGE_HAS_HEADER_AND_SM1_SUPPORT

	/*This FFE binary has either one of the Header below.
	* 1. Header added by CRC generating Application.
	* 2. Header added by FFE itself.
	*
	* 1st byte = '#', then it should be CRC Header.
	*                         (or)
	* 1st byte = '$', then it should be Normal FFE Header.
	*
	*
	* If you found '#' then take the binary size in Words. CRC will be stored after that many bytes.
	* If you found '$' Just skip the CRC veryfication for FFE.
	*
	*
	*/
	raw_data_ptr = (uint8_t *)ffe_fw_raw_data;

	if (*raw_data_ptr == '#') {
		QL_TRACE_MCU_DEBUG("FFE has CRC Header ('#' Detected)\n");
		ffe_has_crc_header = true;
	} else {
		QL_TRACE_MCU_DEBUG("FFE Doesn't have CRC Header ('$' Detected)\n");
		ffe_has_crc_header = false;
	}

	if(ffe_has_crc_header == true) {
		while(*(raw_data_ptr++) != '#');	// raw_data_ptr is now at character after #
	} else {
		while(*(raw_data_ptr++) != '$');	// raw_data_ptr is now at character after $
	}

	/*Reading Firmware Size */
	for(i=0; *(raw_data_ptr) !=' '; raw_data_ptr++) {
		buffer[i++] = *raw_data_ptr;
	}
	buffer[i] = '\0';

	/* convert string to uint32 */
	if (kstrtouint(buffer, 0, &fw_size)) {
		QL_TRACE_MCU_ERROR("Invalid data\n");
		return QL_STATUS_ERROR;
	}

	fw_size *= 4;		//converting word into bytes

	/* Reading Number of Section */
	raw_data_ptr++;
	memset(buffer, 0, sizeof(buffer));
	for(i=0; *(raw_data_ptr) !=' '; raw_data_ptr++) {
		buffer[i++] = *raw_data_ptr;
	}
	buffer[i] = '\0';
	/* convert string to uint32 */
	if (kstrtouint(buffer, 0, &num_sections)) {
		QL_TRACE_MCU_ERROR("Invalid data\n");
		return QL_STATUS_ERROR;
	}
	QL_TRACE_MCU_DEBUG("Number of Sections in FFE = %d \n", num_sections);

	while(*(raw_data_ptr++) != '\n');	// raw_data_ptr is now at character after \n, header parsing code is TODO

	/* Copy Each Sections Size to header array */
	memcpy(header, raw_data_ptr, num_sections*4);
	//raw_data_ptr += (num_sections*4);

	/* Copy Binary's CRC, Each Section's CRC and Address to Footer array */
	memcpy(footer, (raw_data_ptr + fw_size), 4+(8*num_sections));

	//QL_TRACE_MCU_DEBUG("Extra_bytes = %d", (raw_data_ptr - ffe_fw_raw_data) + 20);

	buf = (uint32_t *)raw_data_ptr;



#endif // FFE_IMAGE_HAS_HEADER_AND_SM1_SUPPORT
#endif //CRC_CHECK


	exportVariableTabSzInWord = buf[0] >> 2;
	dmTabSzInWord = buf[1] >> 2;
	sm0TabSzInWord = buf[2] >> 2;
#ifdef FFE_IMAGE_HAS_HEADER_AND_SM1_SUPPORT
	sm1TabSzInWord = buf[3] >> 2;
	cmTabSzInWord = buf[4] >> 2;
#else // NOT FFE_IMAGE_HAS_HEADER_AND_SM1_SUPPORT
	cmTabSzInWord = buf[3] >> 2;
#endif // FFE_IMAGE_HAS_HEADER_AND_SM1_SUPPORT

#ifdef FFE_IMAGE_HAS_HEADER_AND_SM1_SUPPORT
	exportVariableTab = &buf[5];
#else // NOT FFE_IMAGE_HAS_HEADER_AND_SM1_SUPPORT
	exportVariableTab = &buf[4];
#endif // FFE_IMAGE_HAS_HEADER_AND_SM1_SUPPORT
	dmTab = exportVariableTab + exportVariableTabSzInWord;
	sm0Tab = dmTab + dmTabSzInWord;
#ifdef FFE_IMAGE_HAS_HEADER_AND_SM1_SUPPORT
	sm1Tab = sm0Tab + sm0TabSzInWord;
	cmTab = sm1Tab + sm1TabSzInWord;
#else // NOT FFE_IMAGE_HAS_HEADER_AND_SM1_SUPPORT
	cmTab = sm0Tab + sm0TabSzInWord;
#endif // FFE_IMAGE_HAS_HEADER_AND_SM1_SUPPORT
#if 0
	QL_TRACE_MCU_DEBUG("\nValidating FFE image data and sizes have been proceed correctly or not");
	QL_TRACE_MCU_DEBUG("\nExport section size = %d, address = 0x%08x, starting data = 0x%08x", exportVariableTabSzInWord << 2, (uint8_t *)exportVariableTab - (uint8_t *)raw_data_ptr, *exportVariableTab);
	QL_TRACE_MCU_DEBUG("\nData Memory section size = %d, address = 0x%08x, starting data = 0x%08x", dmTabSzInWord << 2, (uint8_t *)dmTab- (uint8_t *)raw_data_ptr, *dmTab);
	QL_TRACE_MCU_DEBUG("\nSection Memory0 section size = %d, address = 0x%08x, starting data = 0x%08x", sm0TabSzInWord << 2, (uint8_t *)sm0Tab-(uint8_t *)raw_data_ptr, *sm0Tab);
	QL_TRACE_MCU_DEBUG("\nSection Memory1 section size = %d, address = 0x%08x, starting data = 0x%08x", sm1TabSzInWord << 2, (uint8_t *)sm1Tab-(uint8_t *)raw_data_ptr, *sm0Tab);
	QL_TRACE_MCU_DEBUG("\nCode Memory section size = %d, address = 0x%08x, starting data = 0x%08x", cmTabSzInWord << 2, (uint8_t *)cmTab-(uint8_t *)raw_data_ptr, *cmTab);
#endif

	/* Load FFE Export Variable Memory */
	if (exportVariableTabSzInWord) {
		ql_status = QLSPI_Write_S3_Mem((uint32_t)FFE_CFG, (uint8_t *)exportVariableTab, exportVariableTabSzInWord * 4);
		if (ql_status != QL_STATUS_OK)
		{
			QL_TRACE_MCU_ERROR("Load of Firwmare in S3 Memory Failed \n");
			return QL_STATUS_ERROR;
		}
	}

	/* Load FFE Data Memory */
	if (dmTabSzInWord) {
		ql_status = QLSPI_Write_S3_Mem((uint32_t)FFE_DM0, (uint8_t *)dmTab, dmTabSzInWord* 4);
		if (ql_status != QL_STATUS_OK)
		{
			QL_TRACE_MCU_ERROR("Load of Firwmare in S3 Memory Failed \n");
			return QL_STATUS_ERROR;
		}
	}

	/* Load FFE sensor SM0 Memory */
	if (sm0TabSzInWord) {
		ql_status = QLSPI_Write_S3_Mem((uint32_t)FFE_SM0, (uint8_t *)sm0Tab, sm0TabSzInWord* 4);
		if (ql_status != QL_STATUS_OK)
		{
			QL_TRACE_MCU_ERROR("Load of Firwmare in S3 Memory Failed \n");
			return QL_STATUS_ERROR;
		}
	}


	/* Load FFE code Memory */
	if (cmTabSzInWord) {
		ql_status = QLSPI_Write_S3_Mem((uint32_t)FFE_CM, (uint8_t *)cmTab, cmTabSzInWord* 4);
		if (ql_status != QL_STATUS_OK)
		{
			QL_TRACE_MCU_ERROR("Load of Firwmare in S3 Memory Failed \n");
			return QL_STATUS_ERROR;
		}
	}

#ifdef CRC_CHECK
#if 0
		if(ffe_has_crc_header == true) {

			/* Reading FFE sensor SM0 Memory  and add it into array*/
			if (QL_STATUS_ERROR == QLSPI_Read_S3_Mem((uint32_t)FFE_SM0, ffe_data, sm0TabSzInWord* 4)) {
				QL_TRACE_MCU_ERROR(" Read S3 mem failed\n");
				return QL_STATUS_ERROR;
			}
			ffe_crc = give_me(SEC_CRC, footer, 3);
			/*CRC-32 does a pre- and post-exclusive-or with 0xffffffff. But Linux CRC doesn't do that. We need to do manually.
			* Here, we are skipping post-exclusive-or with 0xffffffff and using big endian crc function since CRC build tool is using BE*/
			crc_check_ffe = crc32_be(0^ 0xffffffff, ffe_data, sm0TabSzInWord* 4);
			QL_TRACE_MCU_DEBUG("Calculated FFE SM0 CRC = 0X%X \n", crc_check_ffe);

			if(ffe_crc != crc_check_ffe) {
				QL_TRACE_MCU_ERROR(" CRC Verify Failed...\n");
				QL_TRACE_MCU_ERROR(" Calculated CRC = 0x%x, Org CRC = 0x%x\n",crc_check_ffe,ffe_crc);
			} else {
				QL_TRACE_MCU_INFO(" CRC Verify Done!!!\n");
			}
		}
	QL_TRACE_MCU_DEBUG(" 1st Section's Address = %p\n",give_me(SEC_ADDRESS, footer, 1));
	QL_TRACE_MCU_DEBUG(" 1st Section's CRC = %p\n",give_me(SEC_CRC, footer, 1));
	QL_TRACE_MCU_DEBUG(" 1st Section's Size = %p\n",give_me(SEC_SIZE, header, 1));
	QL_TRACE_MCU_DEBUG(" 2nd Section's Address = %p\n",give_me(SEC_ADDRESS, footer, 2));
	QL_TRACE_MCU_DEBUG(" 2nd Section's CRC = %p\n",give_me(SEC_CRC, footer, 2));
	QL_TRACE_MCU_DEBUG(" 2nd Section's Size = %p\n",give_me(SEC_SIZE, header, 2));
	QL_TRACE_MCU_DEBUG(" 3st Section's Address = %p\n",give_me(SEC_ADDRESS, footer, 3));
	QL_TRACE_MCU_DEBUG(" 3st Section's CRC = %p\n",give_me(SEC_CRC, footer, 3));
	QL_TRACE_MCU_DEBUG(" 3st Section's Size = %p\n",give_me(SEC_SIZE, header, 3));
	QL_TRACE_MCU_DEBUG(" 4nd Section's Address = %p\n",give_me(SEC_ADDRESS, footer, 4));
	QL_TRACE_MCU_DEBUG(" 4nd Section's CRC = %p\n",give_me(SEC_CRC, footer, 4));
	QL_TRACE_MCU_DEBUG(" 4nd Section's Size = %p\n",give_me(SEC_SIZE, header, 4));
	QL_TRACE_MCU_DEBUG(" 5nd Section's Address = %p\n",give_me(SEC_ADDRESS, footer, 5));
	QL_TRACE_MCU_DEBUG(" 5nd Section's CRC = %p\n",give_me(SEC_CRC, footer, 5));
	QL_TRACE_MCU_DEBUG(" 5nd Section's Size = %p\n",give_me(SEC_SIZE, header, 5));
#endif



		dummy = container;

		*(uint32_t*) (dummy) = num_sections;
		dummy += 4;

		for(i=0; i<num_sections; i++) {
			*(uint32_t*)(dummy + 0) = give_me(SEC_ADDRESS, footer, i+1);
			*(uint32_t*)(dummy + 4) = give_me(SEC_SIZE, header, i+1);
			*(uint32_t*)(dummy + 8) = give_me(SEC_CRC, footer, i+1);
#if 1
			QL_TRACE_MCU_DEBUG(" Section %d Address = %d\n", i+1, *(uint32_t*)(dummy + 0));
			QL_TRACE_MCU_DEBUG(" Section %d Size = %d\n", i+1, *(uint32_t*)(dummy + 4));
			QL_TRACE_MCU_DEBUG(" Section %d CRC = %d\n", i+1, *(uint32_t*)(dummy + 8));
#endif
			section.addr = *(uint32_t*)(dummy + 0);
			section.size = *(uint32_t*)(dummy + 4);
			section.crc = *(uint32_t*)(dummy + 8);

			if (section.size != 0){
				//ql_status = calculate_verify_crc(&section);
				if (ql_status != QL_STATUS_OK)
				{
					QL_TRACE_MCU_ERROR(" CRC Verify of FFE Bin Failed...\n");
					return QL_STATUS_ERROR;
				}
			}
			dummy += 12;
		}

		if((QLSPI_Write_S3_Mem(FFE_CRC_ADDR, container, 4+(num_sections*12))) != QL_STATUS_OK) {
			QL_TRACE_MCU_ERROR("Failure in Writing CRC to M4 \n");
			return QL_STATUS_ERROR;
		}
#endif

	QL_TRACE_MCU_DEBUG("End\n");

	return QL_STATUS_OK;

}
#endif	//FFE_LOADER
#else
QL_Status QL_Fw_Download (unsigned char *fw_raw_data, uint32_t size)
{
	uint8_t *raw_addr = NULL;
	uint8_t buf[10];
	uint32_t fw_size = 0, num_sections = 0;
	uint8_t container[CRC_FOOTER_SIZE];
	uint32_t temp_size = 0;
	section_info *section;
	int i;
	//uint32_t crc_mask = 0;
	uint32_t crc_address = 0;

	QL_TRACE_MCU_DEBUG("Start\n");

	raw_addr = (uint8_t *)fw_raw_data;

	while(*(raw_addr++) != '#');	// raw_addr is now at character after #

	/* Reading Binary Size */
	for(i=0; *(raw_addr) !=' '; raw_addr++) {
		buf[i++] = *raw_addr;
	}
	buf[i] = '\0';

	/* convert string to uint32 */
	if (kstrtouint(buf, 0, &fw_size)) {
		QL_TRACE_MCU_ERROR("Invalid data\n");
		return QL_STATUS_ERROR;
	}
	fw_size *= 4;		//converting word into bytes
	while(*(raw_addr++) != '\n');	// raw_addr is now at character after \n

	num_sections = *(uint32_t*) (raw_addr + fw_size);
	QL_TRACE_MCU_DEBUG("Number of Sections = %d\n", num_sections);

	temp_size = (NUM_SEC_SIZE + (num_sections * sizeof(section_info)));
	memcpy(container, (raw_addr + fw_size), temp_size);

	/* get the address where need to write the CRC part into the S3 */
	crc_address = *(uint32_t*) (raw_addr + fw_size + temp_size);
	QL_TRACE_MCU_DEBUG("crc_address = 0x%08X\n", crc_address);

	if((QLSPI_Write_S3_Mem(crc_address, container, temp_size)) != QL_STATUS_OK) {
		QL_TRACE_MCU_ERROR("Failure to write CRC array to S3\n");
		return QL_STATUS_ERROR;
	}
	QL_TRACE_MCU_DEBUG("Write CRC array to S3 .... DONE!\n");
#if 0
	//QL_TRACE_MCU_DEBUG(" Loading FW size %d , destaddr 0x%x \n", size,DestAddr);

	ql_status = read_s3_mem(MISC_POR_0_ADDR,&ucRxBuf[0],4);
	if (ql_status != QL_STATUS_OK)
	{
		QL_TRACE_MCU_ERROR("Read_S3_Mem Failure \n");
		return QL_STATUS_ERROR;
	}

	//QL_TRACE_MCU_DEBUG("POR content after reset - 0x%08x \n", *(uint32_t *)ucRxBuf);
#endif
	/*Flash the firmware to S3 mem, section by section*/
	for(i=0; i<num_sections; i++) {
		section = (section_info*) (container + (NUM_SEC_SIZE + (i* sizeof(section_info))));
		QL_TRACE_MCU_DEBUG("[Section %d] : Addr = %08X, Size = %d, Type = %d\n"
						,i, section->addr, section->size, section->type);


		if(QL_STATUS_OK != QLSPI_Write_S3_Mem(section->addr, (uint8_t *)raw_addr, section->size)) {
			QL_TRACE_MCU_ERROR("Load Firwmare in S3 Memory Failed for section %d \n", i);
			return QL_STATUS_ERROR;
		}
		raw_addr += section->size;
		QL_TRACE_MCU_DEBUG("Flashing Section %d DONE!!!\n", i);
	}

#if 0
	/* If CRC is enabled any of the section, do CRC Verification */
	crc_mask = *(uint32_t*) (container + NUM_SEC_SIZE + (num_sections*12));
	QL_TRACE_MCU_DEBUG("CRC_MASK = 0x%08X \n", crc_mask);
	for(i=0; i<MAX_NUM_SECTIONS; i++) {
		if(crc_mask >> i) {
			QL_TRACE_MCU_DEBUG("Section %d wants to do CRC verification\n", i);
			section = (section_info*) (container + (NUM_SEC_SIZE + (i*12)));
			/* Reading Section from S3 Memory  and add it into array*/
			if (QL_STATUS_ERROR == QLSPI_Read_S3_Mem(section->addr, bin_array, section->size)) {
				QL_TRACE_MCU_ERROR(" Read back S3 mem for CRC failed\n");
				return QL_STATUS_ERROR;
			}
			/*CRC-32 does a pre- and post-exclusive-or with 0xffffffff.
			But Linux CRC doesn't do that. We need to do manually.
			* Here, we are skipping post-exclusive-or with 0xffffffff
			and using big endian crc function since CRC build tool is using BE*/
			crc = crc32_be(0^ 0xffffffff, bin_array, section->size);
			QL_TRACE_MCU_DEBUG("Calculated CRC for Section %d = 0X%X \n", i, crc);

			if(crc != section->crc) {
				QL_TRACE_MCU_ERROR(" CRC Verify Failed...\n");
				QL_TRACE_MCU_ERROR(" Calculated CRC = 0x%x, Org CRC = 0x%x\n", crc, section->crc);
			} else {
				QL_TRACE_MCU_ERROR(" CRC Verify Done!!!\n");
			}

		}
	}
#endif
	/*Check each section's type. If it is
		CODE_SEC      		-> Dont do CRC Verification.
		CODE_MUTABLE_SEC	-> Do CRC Verification.
		DATA_SEC		-> Do CRC Verification if CRC_VERIFY_DATA_SEC flag enabled.
		AUDIO_MODEL_SEC	-> Do CRC Verification.
	*/
	for(i=0; i<num_sections; i++) {
		section = (section_info*) (container + (NUM_SEC_SIZE + (i*sizeof(section_info))));
		if((section->type == CODE_MUTABLE_SEC) || (section->type == AUDIO_MODEL_SEC)) {
			QL_TRACE_MCU_DEBUG("Section %d wants to do CRC verification\n", i);

			//crc = calculate_verify_crc(section);
			//QL_TRACE_MCU_DEBUG("Calculated CRC for Section %d = 0X%X \n", i, crc);
		} else if((section->type == DATA_SEC)) {
#ifdef CRC_VERIFY_DATA_SEC
			QL_TRACE_MCU_DEBUG("Data Section %d wants to do CRC verification\n", i);

			//crc = calculate_verify_crc(section);
			//QL_TRACE_MCU_DEBUG("Calculated CRC for Section %d = 0X%X \n", i, crc);
#else
			QL_TRACE_MCU_DEBUG("Section %d wants to do CRC verification.But FLAG Disabled\n", i);
#endif

		}
	}

	QL_TRACE_MCU_DEBUG("End\n");

	return QL_STATUS_OK;
}

#endif //CRC_CHECK_VERSION

#ifdef FABRIC_LOADER

QL_Status QLFAB_Fw_Download (unsigned char *fab_fw_raw_data, uint32_t size,unsigned char *fab_prg_fw_raw_data, uint32_t prg_size)
{
	int ql_status = QL_STATUS_OK;
	volatile uint32_t prog_sts;

	uint32_t counter=0;

	struct FABRIC_HEADER hdr;

	fab_bin_size = size;

	/* do and hold the mcu in reset mode */

	ql_status = reset_slave_mcu();

	if (ql_status != QL_STATUS_OK)
	{
		QL_TRACE_MCU_ERROR(" Reset Slave MCU for FPGA programmer Failed \n");

		return QL_STATUS_ERROR;
	}

//	QL_TRACE_MCU_DEBUG("Reset Slave MCU for FPGA programmer DONE\n");

	prog_sts=0x00000000;

	QLSPI_Write_S3_Mem(FABRIC_PROGRAM_STS,(uint8_t*)&prog_sts,4);

	QLSPI_Write_S3_Mem(FABRIC_HEADER_ADDR,(uint8_t*)&prog_sts,4);

	if (fab_fw_raw_data && size && fab_prg_fw_raw_data && prg_size)
	{

		QL_TRACE_MCU_DEBUG(" Loading Fabric image  \n");


#if CRC_CHECK_VERSION == 2
		ql_status = QLFAB_FW_Write(fab_fw_raw_data, size);
#else
		ql_status = QLSPI_Write_S3_Mem(FABRIC_FW_DEST_ADDR, fab_fw_raw_data, size);
#endif //#if CRC_CHECK_VERSION == 2

		if (ql_status != QL_STATUS_OK)
		{
			QL_TRACE_MCU_ERROR("Loading Fabric image FAILED\n");

			return QL_STATUS_ERROR;
		}


		QL_TRACE_MCU_DEBUG(" Writing Fabric header \n");

		hdr.size=fab_bin_size;

		QLSPI_Write_S3_Mem(FABRIC_HEADER_ADDR, (unsigned char*)&hdr, sizeof(struct FABRIC_HEADER));

		QL_TRACE_MCU_DEBUG(" Loading Fabric programmer \n");

#if CRC_CHECK_VERSION == 2
		atomic_set(&is_fabric_load, true);
		ql_status = QLM4_Fw_Download(fab_prg_fw_raw_data, prg_size);
		atomic_set(&is_fabric_load, false);
#else
		ql_status = QLSPI_Write_S3_Mem(FABRIC_PROG_FW_DEST_ADDR, fab_prg_fw_raw_data, prg_size);
#endif //#if CRC_CHECK_VERSION == 2

		if (ql_status != QL_STATUS_OK)
		{
			QL_TRACE_MCU_ERROR("Loading Fabric programmer FAILED\n");
			return QL_STATUS_ERROR;
		}

		QL_TRACE_MCU_DEBUG(" Loading Fabric programmer DONE \n");
	}
	else
	{
		QL_TRACE_MCU_ERROR("Loading Fabric programmer FAILED\n");
		QL_TRACE_MCU_ERROR("Fabric bin or Fabric Programmer bin or size of those bin is NULL\n");
	}

	/* Release MCU from reset state */

	ql_status = release_slave_mcu();

	if (ql_status != QL_STATUS_OK)
	{
		QL_TRACE_MCU_ERROR("Releasing RESET to FPGA programmer FAILED\n");
		return QL_STATUS_ERROR;
	}

	QL_TRACE_MCU_DEBUG("Releasing RESET to FPGA programmer DONE\n");

	QL_TRACE_MCU_DEBUG(" Waiting for program completion \n ");
	msleep(200);

	counter=0;

	do
	{
		prog_sts=0;
		QLSPI_Read_S3_Mem(FABRIC_PROGRAM_STS,(uint8_t*)&prog_sts,4);


		if(counter++>2000)
		{
			QL_TRACE_MCU_DEBUG(" Timeout waiting for fabric program completion \n ");
			return QL_STATUS_ERROR;
		}


	}while(prog_sts!=1);

	QL_TRACE_MCU_DEBUG(" Fabric FW download completed %d ",prog_sts);
	return ql_status;

}

#endif //#ifdef FABRIC_LOADER

QL_Status pre_fw_load__hw_sequence(int fw_type)
{
	int ql_status = QL_STATUS_OK;
	uint32_t S3_Reg_Addr = 0x0UL;
	volatile uint32_t Reg_Value = 0x00UL;

	QL_TRACE_MCU_DEBUG("Start\n");

	switch(fw_type) {

	case FFE_FW:

		//enable FFE power & clock domain
		S3_Reg_Addr = 0x40004494; /* FFE_PWR_MODE_CFG */
		ql_status = QLSPI_Write_S3_Mem(S3_Reg_Addr, (uint8_t *)&Reg_Value, 4);
		if (ql_status != QL_STATUS_OK)
		{
			QL_TRACE_MCU_ERROR("write_s3_mem Failure \n");
			return QL_STATUS_ERROR;
		}

		S3_Reg_Addr = 0x40004498; /* FFE_PD_SRC_MASK_N */
		ql_status = QLSPI_Write_S3_Mem(S3_Reg_Addr, (uint8_t *)&Reg_Value, 4);
		if (ql_status != QL_STATUS_OK)
		{
			QL_TRACE_MCU_ERROR("write_s3_mem Failure \n");
			return QL_STATUS_ERROR;
		}

		S3_Reg_Addr = 0x4000449C; /* FFE_WU_SRC_MASK_N */
		ql_status = QLSPI_Write_S3_Mem(S3_Reg_Addr, (uint8_t *)&Reg_Value, 4);
		if (ql_status != QL_STATUS_OK)
		{
			QL_TRACE_MCU_ERROR("write_s3_mem Failure \n");
			return QL_STATUS_ERROR;
		}

		/* Wake up FFE */
		S3_Reg_Addr = 0x40004610; /* FFE_FB_PF_SW_WU */
		Reg_Value = 0x01UL;
		ql_status = QLSPI_Write_S3_Mem(S3_Reg_Addr, (uint8_t *)&Reg_Value, 4);
		if (ql_status != QL_STATUS_OK)
		{
			QL_TRACE_MCU_ERROR("write_s3_mem Failure \n");
			return QL_STATUS_ERROR;
		}


		/* check if FFE is in Active mode */
		S3_Reg_Addr = 0x40004490; /* FFE_STATUS */
		do {
			ql_status =  QLSPI_Read_S3_Mem(S3_Reg_Addr, (uint8_t *)&Reg_Value, 4);
			if (ql_status != QL_STATUS_OK)
			{
				QL_TRACE_MCU_ERROR("read_s3_mem Failure \n");
				Reg_Value = 0x00; /* Don't return continue in loop */
			}
		} while(!(Reg_Value& 0x1));

		/* Enable C08-X4 clock */
		S3_Reg_Addr = 0x40004010;	/* CLK_CTRL_C_0 */
		Reg_Value = 0x204;
		ql_status = QLSPI_Write_S3_Mem(S3_Reg_Addr, (uint8_t *)&Reg_Value, 4);
		if (ql_status != QL_STATUS_OK)
		{
			QL_TRACE_MCU_ERROR("write_s3_mem Failure \n");
			return QL_STATUS_ERROR;
		}

		S3_Reg_Addr = 0x40004134;  /* CLK_SWITCH_FOR_C */
		Reg_Value = 0x00UL;
		ql_status = QLSPI_Write_S3_Mem(S3_Reg_Addr, (uint8_t *)&Reg_Value, 4);
		if (ql_status != QL_STATUS_OK)
		{
			QL_TRACE_MCU_ERROR("write_s3_mem Failure \n");
			return QL_STATUS_ERROR;
		}

		S3_Reg_Addr = 0x40004040;	/* C01_CLK_GATE */
		Reg_Value = 0x29F;
		ql_status = QLSPI_Write_S3_Mem(S3_Reg_Addr, (uint8_t *)&Reg_Value, 4);
		if (ql_status != QL_STATUS_OK)
		{
			QL_TRACE_MCU_ERROR("write_s3_mem Failure \n");
			return QL_STATUS_ERROR;
		}


		/* Enable C08-X4 clock */
		S3_Reg_Addr =0x40004048; /* C08_X4_CLK_GATE */
		Reg_Value = 0x01;
		ql_status = QLSPI_Write_S3_Mem(S3_Reg_Addr, (uint8_t *)&Reg_Value, 4);
		if (ql_status != QL_STATUS_OK)
		{
			QL_TRACE_MCU_ERROR("write_s3_mem Failure \n");
			return QL_STATUS_ERROR;
		}

		/* Enable C08-X1 clock */
		S3_Reg_Addr =0x4000404C; /* C08_X1_CLK_GATE */
		Reg_Value = 0x0F;
		ql_status = QLSPI_Write_S3_Mem(S3_Reg_Addr, (uint8_t *)&Reg_Value, 4);
		if (ql_status != QL_STATUS_OK)
		{
			QL_TRACE_MCU_ERROR("write_s3_mem Failure \n");
			return QL_STATUS_ERROR;
		}

		S3_Reg_Addr =0x40004084; /* FFE_SW_RESET */
		Reg_Value = 0x03;
		ql_status = QLSPI_Write_S3_Mem(S3_Reg_Addr, (uint8_t *)&Reg_Value, 4);
		if (ql_status != QL_STATUS_OK)
		{
			QL_TRACE_MCU_ERROR("write_s3_mem Failure \n");
			return QL_STATUS_ERROR;
		}

		S3_Reg_Addr =0x40004084; /* FFE_SW_RESET */
		Reg_Value = 0x00;
		ql_status = QLSPI_Write_S3_Mem(S3_Reg_Addr, (uint8_t *)&Reg_Value, 4);
		if (ql_status != QL_STATUS_OK)
		{
			QL_TRACE_MCU_ERROR("write_s3_mem Failure \n");
			return QL_STATUS_ERROR;
		}
		break;
	case M4_FW:

		break;
	default :

		break;
	};

	QL_TRACE_MCU_DEBUG("End\n");

	return ql_status;
}

QL_Status post_fw_load__hw_sequence (int fw_type)
{
	QL_TRACE_MCU_DEBUG("Start\n");

	switch(fw_type) {

	case FFE_FW:

		break;
	case M4_FW:

		break;
	default:

		break;

	};

	QL_TRACE_MCU_DEBUG("End\n");

	return QL_STATUS_OK;
}


#if 0
/* Download the Firmware using QLSPI */
QL_Status QLMCU_Fw_Download (SLAVE_DEV_FW_LOAD_T *slave_dev_fw_load_info)
{
	int ql_status = QL_STATUS_OK;

	QL_TRACE_MCU_DEBUG("Start\n");

#ifdef FABRIC_LOADER

	QL_TRACE_MCU_DEBUG(" Fabric programmer  size %d bytes \n",slave_dev_fw_load_info->fab_prg_fw_size);

	QL_TRACE_MCU_DEBUG(" Fabric size %d bytes \n",slave_dev_fw_load_info->fab_fw_size);

	if (slave_dev_fw_load_info->fab_prg_fw_addr && slave_dev_fw_load_info->fab_prg_fw_size) {

		ql_status = QLFAB_Fw_Download((uint8_t *)slave_dev_fw_load_info->fab_fw_addr, slave_dev_fw_load_info->fab_fw_size,(uint8_t *)slave_dev_fw_load_info->fab_prg_fw_addr, slave_dev_fw_load_info->fab_prg_fw_size);

		if (ql_status != QL_STATUS_OK)
		{
			QL_TRACE_MCU_ERROR("Load Fab Firwmare in S3 Memory Failed \n");
			return QL_STATUS_ERROR;
		}
	}

#endif

	/* Do and Hold the mcu in reset mode */
	ql_status = reset_slave_mcu();
	if (ql_status != QL_STATUS_OK)
	{
		QL_TRACE_MCU_ERROR("Reset Slave MCU Failed \n");
		return QL_STATUS_ERROR;
	}
#ifdef FFE_LOADER
	pre_fw_load__hw_sequence(FFE_FW);


	if (slave_dev_fw_load_info->ffe_fw_addr && slave_dev_fw_load_info->ffe_fw_size) {
#if CRC_CHECK_VERSION == 2
		/* Load FFE firmware into FFE Memory.
		*  And also read back the FFE firmware from FFE Memory (RAM) and store into the array if CRC_CHECK enabled.
		*  Take the CRC value from the Binary and put it into variable.
		*/
		ql_status = QLFFE_Fw_Download((uint8_t *)slave_dev_fw_load_info->ffe_fw_addr, slave_dev_fw_load_info->ffe_fw_size);

#else
		/* Load FFE firmware into FFE Memory*/
		ql_status = QL_Fw_Download((uint8_t *)slave_dev_fw_load_info->ffe_fw_addr, slave_dev_fw_load_info->ffe_fw_size);
#endif //CRC_CHECK_VERSION
		if (ql_status != QL_STATUS_OK)
		{
			QL_TRACE_MCU_ERROR("Load of FFE Firwmare in S3 Memory Failed \n");
			return QL_STATUS_ERROR;
		}
	}

	post_fw_load__hw_sequence(FFE_FW);
#endif	//FFE_LOADER
	pre_fw_load__hw_sequence(M4_FW);

	if (slave_dev_fw_load_info->m4_fw_addr && slave_dev_fw_load_info->m4_fw_size) {
#if CRC_CHECK_VERSION == 2
		/* Load M4 firmware into M4 Memory.
		*  Take the CRC value from the Binary and put it into variable if CRC_CHECK enabled.
		*/
		ql_status = QLM4_Fw_Download((uint8_t *)slave_dev_fw_load_info->m4_fw_addr, slave_dev_fw_load_info->m4_fw_size);
#else
		/* Load M4 firmware into S3 Memory*/
		ql_status = QL_Fw_Download((uint8_t *)slave_dev_fw_load_info->m4_fw_addr, slave_dev_fw_load_info->m4_fw_size);
#endif

		if (ql_status != QL_STATUS_OK)
		{
			QL_TRACE_MCU_ERROR("Load of M4 Firwmare in S3 Memory Failed \n");
			return QL_STATUS_ERROR;
		}
	}

	post_fw_load__hw_sequence(M4_FW);



	/* Release MCU from reset state */
	ql_status = release_slave_mcu();
	if (ql_status != QL_STATUS_OK)
	{
		QL_TRACE_MCU_ERROR("Load of Firwmare in S3 Memory Failed \n");
		return QL_STATUS_ERROR;
	}

	QLSPI_Read_S3_Mem(MISC_POR_0_ADDR,&ucRxBuf[0],4 );
	QL_TRACE_MCU_DEBUG("POR content after release - 0x%08x \n", *(uint32_t *)ucRxBuf);

	QL_TRACE_MCU_DEBUG("End\n");

	return QL_STATUS_OK;
}
#endif


static QL_Status QLMCU_release_mcu_resetmode(void)
{
	int ql_status = QL_STATUS_OK;

	QL_TRACE_MCU_DEBUG("Start\n");
	ql_status = release_slave_mcu();
	if (ql_status != QL_STATUS_OK)
	{
		QL_TRACE_MCU_ERROR("Load of Firwmare in S3 Memory Failed \n");
		return QL_STATUS_ERROR;
	}

	//QLSPI_Read_S3_Mem(MISC_POR_0_ADDR,&ucRxBuf[0],4 );
	//QL_TRACE_MCU_DEBUG("POR content after release - 0x%08x \n", *(uint32_t *)ucRxBuf);
	QL_TRACE_MCU_DEBUG("End\n");
	return ql_status;
}

static QL_Status QLMCU_M4_Fw_Download(SLAVE_DEV_FW_LOAD_T *slave_dev_fw_load_info)
{
	int ql_status = QL_STATUS_OK;

	QL_TRACE_MCU_DEBUG("Start\n");

	pre_fw_load__hw_sequence(M4_FW);

	if (slave_dev_fw_load_info->m4_fw_addr && slave_dev_fw_load_info->m4_fw_size) {
		/* Load M4 firmware into M4 Memory.
		*  Take the CRC value from the Binary and put it into variable if CRC_CHECK enabled.
		*/
		ql_status = QLM4_Fw_Download((uint8_t *)slave_dev_fw_load_info->m4_fw_addr, slave_dev_fw_load_info->m4_fw_size);
		if (ql_status != QL_STATUS_OK)
		{
			QL_TRACE_MCU_ERROR("Load of M4 Firwmare in S3 Memory Failed \n");
			return QL_STATUS_ERROR;
		}
	}

	post_fw_load__hw_sequence(M4_FW);

	QL_TRACE_MCU_DEBUG("End\n");
	return QL_STATUS_OK;
}

#ifdef FFE_LOADER
static QL_Status QLMCU_FFE_Fw_Download(SLAVE_DEV_FW_LOAD_T *slave_dev_fw_load_info)
{
	int ql_status = QL_STATUS_OK;
	QL_TRACE_MCU_DEBUG("Start\n");
	pre_fw_load__hw_sequence(FFE_FW);
	if (slave_dev_fw_load_info->ffe_fw_addr && slave_dev_fw_load_info->ffe_fw_size) {
		/* Load FFE firmware into FFE Memory.
		*  And also read back the FFE firmware from FFE Memory (RAM) and store into the array if CRC_CHECK enabled.
		*  Take the CRC value from the Binary and put it into variable.
		*/
		ql_status = QLFFE_Fw_Download((uint8_t *)slave_dev_fw_load_info->ffe_fw_addr, slave_dev_fw_load_info->ffe_fw_size);
		if (ql_status != QL_STATUS_OK)
		{
			QL_TRACE_MCU_ERROR("Load of FFE Firwmare in S3 Memory Failed \n");
			return QL_STATUS_ERROR;
		}
	}

	post_fw_load__hw_sequence(FFE_FW);

	QL_TRACE_MCU_DEBUG("End\n");
    return QL_STATUS_OK;
}
#endif /* FFE_LOADER */

static QL_Status QLMCU_request_mcu_resetmode(void)
{
	int ql_status = QL_STATUS_OK;

	QL_TRACE_MCU_DEBUG("Start\n");
	/* Do and Hold the mcu in reset mode */
	ql_status = reset_slave_mcu();
	if (ql_status != QL_STATUS_OK)
	{
		QL_TRACE_MCU_ERROR("Reset Slave MCU Failed \n");
		return QL_STATUS_ERROR;
	}
	QL_TRACE_MCU_DEBUG("End\n");
	return ql_status;
}

#ifdef FABRIC_LOADER
static QL_Status QLMCU_Fab_Fw_Download(SLAVE_DEV_FW_LOAD_T *slave_dev_fw_load_info)
{
	int ql_status = QL_STATUS_OK;
	QL_TRACE_MCU_DEBUG("Start\n");
	QL_TRACE_MCU_DEBUG(" Fabric programmer  size %d bytes \n",slave_dev_fw_load_info->fab_prg_fw_size);

	QL_TRACE_MCU_DEBUG(" Fabric size %d bytes \n",slave_dev_fw_load_info->fab_fw_size);

	if (slave_dev_fw_load_info->fab_prg_fw_addr && slave_dev_fw_load_info->fab_prg_fw_size) {

		ql_status = QLFAB_Fw_Download((uint8_t *)slave_dev_fw_load_info->fab_fw_addr, slave_dev_fw_load_info->fab_fw_size,(uint8_t *)slave_dev_fw_load_info->fab_prg_fw_addr, slave_dev_fw_load_info->fab_prg_fw_size);

		if (ql_status != QL_STATUS_OK)
		{
			QL_TRACE_MCU_ERROR("Load Fab Firwmare in S3 Memory Failed \n");
			return QL_STATUS_ERROR;
		}
	}

	QL_TRACE_MCU_DEBUG("End\n");
    return QL_STATUS_OK;
}
#endif /* FABRIC_LOADER */

/* Download the Firmware using QLSPI (KC customize: divide per module.)*/
QL_Status QLMCU_Fw_Download(SLAVE_DEV_FW_LOAD_T *slave_dev_fw_load_info)
{
	int ql_status = QL_STATUS_OK;
	QL_TRACE_MCU_DEBUG("Start\n");

#ifdef FABRIC_LOADER
    /* Download Fabric area binary. */
	if(qlspi_get_fab_dl_errcount() < ALLOW_FABDL_COUNT){
		ql_status = QLMCU_Fab_Fw_Download(slave_dev_fw_load_info);
		if(ql_status != QL_STATUS_OK){
			qlspi_fab_dl_errcount_inc();
			QL_TRACE_MCU_ERROR("Fw_Download Error[Fab]\n");
			return ql_status;
		}
	}
#endif

	/* preprocess of Doing and Holding the mcu in reset mode */
	ql_status = QLMCU_request_mcu_resetmode();
	if(ql_status != QL_STATUS_OK){
		QL_TRACE_MCU_ERROR("Reset MCU Failed.\n");
		return ql_status;
	}

#ifdef FFE_LOADER
    /* Download FFE area binary. */
	ql_status = QLMCU_FFE_Fw_Download(slave_dev_fw_load_info);
	if(ql_status != QL_STATUS_OK){
		QL_TRACE_MCU_ERROR("Fw_Download Error[FFE]\n");
		return ql_status;
	}
#endif

	/* Download M4 area binary. */
	ql_status = QLMCU_M4_Fw_Download(slave_dev_fw_load_info);
	if(ql_status != QL_STATUS_OK){
		QL_TRACE_MCU_ERROR("Fw_Download Error[M4]\n");
		return ql_status;
	}

	/* postprocess of Release the mcu from reset mode */
	ql_status = QLMCU_release_mcu_resetmode();
	if(ql_status != QL_STATUS_OK){
		QL_TRACE_MCU_ERROR("Release MCU Failed.\n");
		return ql_status;
	}

	QL_TRACE_MCU_DEBUG("End\n");
	return QL_STATUS_OK;
}
