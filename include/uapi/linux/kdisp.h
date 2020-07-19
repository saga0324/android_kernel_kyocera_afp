/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2018 KYOCERA Corporation
 */

#ifndef _KDISP_H
#define _KDISP_H

#include <linux/msm_mdp.h>

#define KDISP_MIPI_REG_WRITE  _IOWR(MSMFB_IOCTL_MAGIC, 180, \
						struct disp_diag_mipi_write_reg_type)
#define KDISP_MIPI_REG_READ   _IOWR(MSMFB_IOCTL_MAGIC, 181, \
						struct disp_diag_mipi_read_reg_type)
#define KDISP_DISPLAY_STANDBY _IOW(MSMFB_IOCTL_MAGIC, 182, unsigned int)
#define KDISP_DISP_DEBUG    _IOW(MSMFB_IOCTL_MAGIC, 184, \
											struct disp_diag_debug)
#define KDISP_OTP_READ         _IOWR(MSMFB_IOCTL_MAGIC, 191, unsigned int)
#define KDISP_DISP_DET_GET    _IOWR(MSMFB_IOCTL_MAGIC, 192, int)

#define KDISP_DIAGVALID    _IOWR(MSMFB_IOCTL_MAGIC, 193, int)
#define KDISP_DIAGINVALID    _IOWR(MSMFB_IOCTL_MAGIC, 194, int)

#define KDISP_RESUME_SW_REFRESHER    _IOWR(MSMFB_IOCTL_MAGIC, 195, int)
#define KDISP_SUSPEND_SW_REFRESHER    _IOWR(MSMFB_IOCTL_MAGIC, 196, int)

/*
struct disp_diag_mipi_write_reg_type {
	uint8_t type;
	uint8_t speed;
	uint8_t bta;
	uint8_t wait;
	uint8_t len;
	uint8_t dummy;
	uint8_t data[25];

	char ack_err_status[2];
};
*/

/*
struct disp_diag_mipi_read_reg_type {
	uint8_t type;
	uint8_t speed;
	uint8_t wait;
	uint8_t len;
	uint8_t rlen;
	uint8_t data[25];
};

*/
struct disp_diag_debug {
	uint8_t type;
	uint8_t level;
};

#endif /* _KDISP_H */
