#ifndef __QL_SPI_TLC_H__
#define __QL_SPI_TLC_H__

#define MISC_CTRL_BASE	0x40005000
#define INTR_CTRL_BASE	0x40004800

#define MAX_WAIT	50

#define QLULPSH_CMD_READ	0x0

#define BOOTL1_START_ADDR                                         (0x20000000)
#define APP_START_ADDR                                            (0x20000000)

#define APP_START_ADDR_2                                            (0x20020000)
#define APP_START_ADDR_3                                            (0x20040000)

#define MISC_POR_0_ADDR                                           (0x40004400)
#define MISC_POR_2_ADDR                                           (0x40004408)
#define MISC_M4_STATUS                                            (0x40004480)
#define SW_MB_1_ADDR                                              (0x40005110)
#define SW_MB_2_ADDR                                              (0x40005114)

#define AIP_RTC_CTRL_2                                              (0x40005408)
#define AIP_RTC_CTRL_7                                              (0x4000541C)

#define AIP_RTC_STA_0                                               (0x40005420)
#define AIP_RTC_STA_OSCOK                                           (0x1<<3)

#define AIP_OSC_STA_0                                               (0x400054A0)
#define AIP_OSC_STA_HOSCLOCK                                        (0x1<<0)

#define MISC_POR_0_RELEASE_CPU_RESET                              (0x01)

#define FIFO0	0
#define FIFO1	1

// QL Register Define
#define SPITLC_CM_FIFO_0_DATA                                     (0x09)
#define SPITLC_CM_FIFO_1_DATA                                     (0x0A)
#define SPITLC_CM_FIFO_2_DATA                                     (0x0B)
#define SPITLC_CM_FIFO_8K_DATA                                    (0x0C)
#define SPITLC_MEM_ADDR_BYTE_0                                    (0x20)
#define SPITLC_MEM_ADDR_BYTE_1                                    (0x21)
#define SPITLC_MEM_ADDR_BYTE_2                                    (0x22)
#define SPITLC_MEM_ADDR_BYTE_3                                    (0x23)
#define SPITLC_MEM_DATA_BYTE_0                                    (0x28)
#define SPITLC_MEM_DATA_BYTE_1                                    (0x29)
#define SPITLC_MEM_DATA_BYTE_2                                    (0x2A)
#define SPITLC_MEM_DATA_BYTE_3                                    (0x2B)
#define SPITLC_AHB_STATUS                                         (0x2F)
#define SPITLC_AHB_ACCESS_CTL                                     (0x30)
#define SPITLC_SCRATCH_BYTE                                       (0x31)
#define SPITLC_TAMAR_STATUS                                       (0x32)
#define SPITLC_DMA_DEBUG_CTL0                                     (0x36)
#define SPITLC_DMA_DEBUG_CTL1                                     (0x37)
#define SPITLC_DMA_ADDR0                                          (0x38)
#define SPITLC_DMA_ADDR1                                          (0x39)
#define SPITLC_DMA_ADDR2                                          (0x3A)
#define SPITLC_DMA_ADDR3                                          (0x3B)
#define SPITLC_DMA_BURST_SZ0                                      (0x3C)
#define SPITLC_DMA_BURST_SZ1                                      (0x3D)
#define SPITLC_DMA_STATUS                                         (0x3F)
#define SPITLC_DMA_RD_DATA                                        (0x40)
#define SPITLC_DEVICE_ID_BYTE                                     (0x7F)

#define EOSS3_SPI_DEVID			(0x21)

#define SPITLC_DMA_DBG_CTRL0		0x36
#define SPITLC_DMA_DBG_CTRL1		0x37
#define SPITLC_DMA_ADD0		0x38
#define SPITLC_DMA_BRUST_SIZE0		0x3C
#define SPITLC_DMA_BRUST_SIZE1		0x3D
//#define SPITLC_DMA_STATUS			0x3F
#define SPITLC_DMA_READ_DATA		0x40

#define SW_INTR_1_EN_AP_REG		(INTR_CTRL_BASE+0x44)
#define SW_INTR_1_EN_M4		(INTR_CTRL_BASE+0x48)



#define SW_INTR_2_EN_AP_REG		(INTR_CTRL_BASE+0x54)
#define SW_INTR_2_EN_M4_REG		(INTR_CTRL_BASE+0x58)

/* Fabric related Interrupt Control Registers */

#define FABRIC_INTR_STS_REG		(INTR_CTRL_BASE+0x80)		//For edge interrupt
#define FABRIC_INTR_STS_REG_RAW	(INTR_CTRL_BASE+0x84)		//For level interrupt
#define FABRIC_INTR_TYPE		(INTR_CTRL_BASE+0x88)
#define FABRIC_INTR_POL			(INTR_CTRL_BASE+0x8C)
#define FABRIC_INTR_EN_AP		(INTR_CTRL_BASE+0x90)


#define FABRIC_DEV_0_BASE		(0x40020000+(0x800))//new IR fabric file	//(0x40020000)

#define IR_DEV_ID_REG		 (FABRIC_DEV_0_BASE + (0x50<<2))
#define IR_CMD_REG			 (FABRIC_DEV_0_BASE + (0x54<<2))
#define IR_STS_CTRL_REG		 (FABRIC_DEV_0_BASE + (0x55<<2))
#define IR_CODE_INDEX_REG	 (FABRIC_DEV_0_BASE + (0x5F<<2))

#define IR_DEVICE_ID			(0x2)

typedef enum {
 	
	//SW_INTR_1               = ((uint32_t) (0x00000001)),
	SW_INTR_1_REG			= (INTR_CTRL_BASE+0x040),
	SW_INTR_2_REG			= (INTR_CTRL_BASE+0x050),
	QLULPSH_ENABLE			= 1,
	QLULPSH_DISABLE			= 0,
	QLULPSH_CMD_WRITE		= 0x1,
	OPER_BIT_LOC			= 7,
	ADDR_MASK				= 0x7F,
	ONE_BYTE				= 1,
	MISC_MAILBOX_REG_0	 	= MISC_CTRL_BASE + 0x110,
	MISC_MAILBOX_REG_1	 	= MISC_CTRL_BASE + 0x114,
} QL_DEF;

#endif /* __QLSPI_TLC_H__ */
