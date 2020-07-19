#ifndef __QL_SPI_H__
#define __QL_SPI_H__
#include "os.h"

#define QL_STATUS_BASE 0	// QL Status starting number

#define QL_SPI_READ_MAX_LEN (64*1024)
#define QL_SPI_WRITE_MAX_LEN (64*1024)  //128 - when using chandalar host, //(64*1024)- Linux host

#define	QLSPI_PADDING_EN

/* QLSPI optimizations */

/* Access ctrl write only once B0 03 optimization*/
#define ACCESS_CTRL_ONCE

// enable if the HW uses CMOS CLK for S3 Input
#define CMOS_CLK_INPUT

/* Enable verify for qlspi write operation */

//#define	WRITE_VERIFY


typedef enum {
				QL_STATUS_OK = QL_STATUS_BASE,		/*0*/
				QL_STATUS_ERROR,					/*1 Error*/
				QL_STATUS_ERROR_BAD_PARAMETER,		/*2 Bad parameter passed*/
				QL_STATUS_ERROR_TIMEOUT,			/*3 Timeout occured*/
				QL_STATUS_ERROR_DEVICE_BUSY			/*4 Device is currently busy*/
			 } QL_Status;

typedef enum {
    FAB_DEAD   = -1,
    FAB_NORMAL = 0,
}FAB_Status;
#define ALLOW_FABDL_COUNT 2

typedef QL_Status (*QLSPI_Isr_Handler)(void *data);		/*User Defined ISR to be registered with QLSPI. It is invoked by QLSPI upon interrupt from EOSS3 M4*/

struct QLSPI_Platform {
	uint32_t irq_gpio;		/*GPIO number to be configured as IRQ to receive interrupt from EOSS3 M4 */
	uint32_t cs_gpio;		/*GPIO number to be configured as Chip Select or Slave Select Pin */
	uint32_t	irq_out_gpio;
};



/* QLSPI API Prototypes*/
QL_Status QLSPI_Init (struct QLSPI_Platform *spi_plat) ;		/*This API does SPI as well as platform related configurations*/
QL_Status QLSPI_Read_S3_Mem (uint32_t addr, uint8_t *data, uint32_t len);		/*This API reads from S3 Memory using SPI Bus*/
QL_Status QLSPI_Write_S3_Mem (uint32_t addr, uint8_t *data, uint32_t len);		/*This API writes to S3 Memory using SPI Bus*/
QL_Status QLSPI_Register_Isr (QLSPI_Isr_Handler handler);		/*This API registers user defined ISR handler*/
QL_Status QLSPI_Trigger_Intr (void);		/*This API triggers interrupt to S3 MCU (SPI slave device)*/
QL_Status QLSPI_Clear_S3_Intr (void);	/* This API clears the interrupt status of interrupt received from S3 */
void qlspi_enable_irq(void);
void qlspi_disable_irq(void);
int qlspi_is_spi_intr_set_zero(void);
int qlspi_check_for_gpio_if_low(void);
int qlspi_reset_dma(void);
int read_reg_status(uint8_t reg, uint8_t *buf);

#endif	/* __QL_SPI_H__ */
