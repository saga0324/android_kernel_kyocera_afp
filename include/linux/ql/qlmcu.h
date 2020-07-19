#ifndef __QL_MCU_H__
#define __QL_MCU_H__
#include "qlspi.h"
#include "qlspi_tlc.h"
#include "qlmemorymap.h"

/*
* This flag enables the firmware reload option when error occurs
*/
#define RELOAD_FW_IN_ERROR

/*
* This flag will enable the CRC Check for FFE and M4 *
* Get the CRC from binary and store it into variable *
* Then read the RAM, calculate the CRC & verify it   *
*/
#define CRC_CHECK

#ifdef CRC_CHECK

/* CRC_CHECK_VERSION
 * 1 -> M4 Will do CRC Verification
 * 2 -> HOST will do CRC Verification Only for SM0(FFE). Other section CRC verification will be done by M4.
 * 3 -> Host will check the type of the each section. Based on the type, it decides weather do CRC verification or Not. M4 also does like HOST.
 */
#define CRC_CHECK_VERSION	2
//#define CRC_VERIFY_DATA_SEC

#define FFE_FW			0
#define M4_FW			1


#if CRC_CHECK_VERSION == 2
//#define M4_CRC_ADDR		0x2006c000
//#define FFE_CRC_ADDR		0x2006c000 + 128
#define M4_CRC_ADDR		CRC32_M4_SECTION_ADDR
#define FFE_CRC_ADDR		CRC32_FFE_SECTION_ADDR

#define	SEC_ADDRESS		0
#define	SEC_SIZE		1
#define	SEC_CRC			2
#endif

#define MAX_SECTION_SIZE		512*1024
#define MAX_NUM_SECTIONS 	CRC32_MAX_NUM_SECTIONS//10
#define NUM_SEC_SIZE		4
#define CRC_MASK_SIZE		8
#define BINARY_CRC_SIZE		4
#define CRC_FOOTER_SIZE  	NUM_SEC_SIZE + (MAX_NUM_SECTIONS*12) + \
			 	CRC_MASK_SIZE + BINARY_CRC_SIZE

typedef struct {
	uint32_t addr;
	uint32_t size;
	uint32_t type;
	uint32_t crc;
}section_info;

enum sec_type {
	CODE_SEC = 0,
	CODE_MUTABLE_SEC,
	DATA_SEC,
	AUDIO_MODEL_SEC,
};

#endif

extern uint32_t low_spi_freq;
extern uint32_t high_spi_freq;


typedef struct {

	volatile unsigned char* m4_fw_addr;		//should be a pointer , unsigned char*

	volatile uint32_t m4_fw_size;
	volatile uint32_t m4_fw_dest_addr;
	volatile unsigned char* ffe_fw_addr;
	volatile uint32_t ffe_fw_size;
	volatile uint32_t ffe_fw_dest_addr;
	volatile unsigned char* fab_fw_addr;
	volatile uint32_t fab_fw_size;
	volatile uint32_t fab_fw_dest_addr;


	volatile unsigned char* fab_prg_fw_addr;
	volatile uint32_t fab_prg_fw_size;
	volatile uint32_t fab_prg_fw_dest_addr;


} SLAVE_DEV_FW_LOAD_T;

QL_Status QLMCU_Fw_Download (SLAVE_DEV_FW_LOAD_T * slave_dev_fw_load_info);

#define FFE_LOADER

#ifdef FFE_LOADER
#define FFE_IMAGE_HAS_HEADER_AND_SM1_SUPPORT			// define this if FFE Image has signature header in the binary as well as SM1 support (Celeris 3.2.7 onwards)
#endif

//#define FABRIC_LOADER

#ifdef FABRIC_LOADER

	#define	IR_FABRIC_DEV

	struct FABRIC_HEADER
	{
		uint32_t size;
	};

	/* Fabric_PRG.bin (FPGA programmer) address */

	#define FABRIC_PROG_FW_DEST_ADDR		(0x20000000)

	#define FABRIC_HEADER_ADDR				(FABRIC_PROG_FW_DEST_ADDR+(32*1024))

	/* Fabric program completion status written at this address*/

	#define FABRIC_PROGRAM_STS				(FABRIC_HEADER_ADDR+sizeof(struct FABRIC_HEADER))	//this is 4B

	/* Fabric.bin copied to this location */

	#define FABRIC_FW_DEST_ADDR			(0x20010000)
void clear_intr_sts_fabric(void);
#endif

#ifdef CMOS_CLK_INPUT
int initialize_s3_host_interface(void);
#endif

#endif	/* __QL_MCU_H__ */
