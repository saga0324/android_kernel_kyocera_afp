/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 *
 * drivers/video/msm/mdss/disp_ext_sub_panel.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
*/
#ifndef DISP_EXT_SUB_PANEL_H
#define DISP_EXT_SUB_PANEL_H

#include <linux/mdss_io_util.h>
#include <linux/fb.h>
#include "disp_ext_spi.h"

#define DISP_EXT_SUB_PANEL_IMAGE_SIZE (96*96*2/8)

typedef enum {
	DISP_SUB_STATE_PWR_OFF,
	DISP_SUB_STATE_PWR_ON,
	DISP_SUB_STATE_OFF,
	DISP_SUB_STATE_ON,
	DISP_SUB_STATE_ALLWAYSON
} disp_ext_sub_state_type;

typedef enum {
	DISP_SUB_MODEM_STATE_ON,
	DISP_SUB_MODEM_STATE_OFF,
	DISP_SUB_MODEM_STATE_MAX
} disp_ext_sub_modem_state_type;

typedef enum {
	DISP_SUB_CTRL_CMD =  0x01,
	DISP_SUB_CTRL_WAIT = 0x10,
	DISP_SUB_CTRL_SIG =  0x20,
	DISP_SUB_CTRL_MAX
} disp_ext_sub_ctrl_kind_type;

typedef enum {
	DISP_SUB_SIG_RESET = 0,
	DISP_SUB_SIG_SPI_CS,
	DISP_SUB_SIG_SPI_CLK,
	DISP_SUB_SIG_SPI_DATA,
	DISP_SUB_SIG_SPI_RS,
	DISP_SUB_SIG_MAX
} disp_ext_sub_sig_kind_type;

struct disp_ext_sub_cmd_hdr {
	unsigned char ctrl_kind;
	unsigned char payload_len;
};

struct  disp_ext_sub_cmd_detail {
	struct  disp_ext_sub_cmd_hdr cmd_hdr;
	void *  payload_p;
};

struct disp_ext_sub_cmds {
	char *  buf_p;
	unsigned int  blen;
	unsigned int  cmd_cnt;
	struct disp_ext_sub_cmd_detail* cmd_p;
};

enum {
	DISP_EXT_SUB_MDMREQ_DISPCTL = 0,
	DISP_EXT_SUB_MDMREQ_IMAGE_UPDATE,
	DISP_EXT_SUB_MDMREQ_DRVINFO,
	DISP_EXT_SUB_MDMREQ_SVCINFO,
	DISP_EXT_SUB_MDMREQ_SET_TIMERADJ,
	DISP_EXT_SUB_MDMREQ_SET_SSR,
	DISP_EXT_SUB_MDMREQ_DUMP_SVCIMAGE,
	DISP_EXT_SUB_MDMREQ_DUMP_DRVIMAGE,
	DISP_EXT_SUB_MDMREQ_DUMP_IMGPARTS,
	DISP_EXT_SUB_MDMREQ_DUMP_TIMEHIST,
	DISP_EXT_SUB_MDMREQ_SET_SVCDBGLVL,
	DISP_EXT_SUB_MDMREQ_SET_DRVDBGLVL,
	DISP_EXT_SUB_MDMREQ_MAX,
};

enum {
	DISP_EXT_SUB_MDMREQ_DEBUG_DISPCTL_RESET = 0,
	DISP_EXT_SUB_MDMREQ_DEBUG_MAX,
};

enum {
	DISP_EXT_SUB_MDMREQ_LOGLVL_TYPE_L = 0,
	DISP_EXT_SUB_MDMREQ_LOGLVL_TYPE_M,
	DISP_EXT_SUB_MDMREQ_LOGLVL_TYPE_H,
};

enum {
	DISP_EXT_SUB_MDMREQ_DISPCRL_TYPE_START = 0,
	DISP_EXT_SUB_MDMREQ_DISPCRL_TYPE_STOP,
	DISP_EXT_SUB_MDMREQ_DISPCRL_TYPE_MAX
};

#define DISP_EXT_SUB_MDMRES_TYPE_SUCCESS 0
#define DISP_EXT_SUB_MDMRES_TYPE_ERROR   1

typedef struct
{
	uint32_t result;
} disp_ext_sub_mdmres_type;

typedef struct
{
	uint32_t seq;
	uint32_t request;
	union {
		disp_ext_sub_mdmres_type  resp;
	};
} disp_ext_sub_linux2mdm_rx_type;

typedef struct
{
	uint32_t mode;
	struct fb_var_subdispinfo disp_info;
} disp_ext_sub_dispctl_type;

typedef struct
{
	uint32_t timeadj;
} disp_ext_sub_timeradj_type;

typedef struct
{
	uint32_t level;
} disp_ext_sub_loglvl_type;

typedef struct
{
	uint32_t seq;
	uint32_t request;
	union {
		disp_ext_sub_dispctl_type  dispctl;
		disp_ext_sub_timeradj_type timardj;
		disp_ext_sub_loglvl_type loglvl;
		uint32_t param;
		uint8_t buf[56];
	};
} disp_ext_sub_linux2mdm_tx_type;

struct disp_ext_sub_smd_data {
	int                     request_updated;
	disp_ext_sub_linux2mdm_tx_type tx;
	disp_ext_sub_linux2mdm_rx_type rx;

	const char*             port_name;
	struct smd_channel*     smd_ch;
	bool                    smd_opened;
	void (*cb_func)(void *, unsigned);
	struct completion       completion;
	struct mutex            mutex;
};

typedef struct
{
	uint32_t seq;
	uint32_t request;
} disp_ext_sub_mdm2linux_rx_type;

typedef struct
{
	uint32_t seq;
	uint32_t request;
	uint32_t result;
	uint32_t padding;
} disp_ext_sub_mdm2linux_tx_type;

struct disp_ext_sub_smd_m2l_data {
	struct work_struct      work;

	disp_ext_sub_mdm2linux_tx_type tx;
	disp_ext_sub_mdm2linux_rx_type rx;

	const char*             port_name;
	struct smd_channel*     smd_ch;
	bool                    smd_opened;
	void (*cb_func)(void *, unsigned);
};

struct disp_ext_sub_mdm_notify_data {
	struct notifier_block  nb;
	struct work_struct     work;
	struct completion      completion;
};

struct disp_ext_sub_pdata {
	u32  xres;
	u32  yres;
	u32  bpp;
	int  rst_gpio;
	struct disp_ext_spi_data  spi_data;
	struct disp_ext_sub_cmds  pwron_cmd;
	struct disp_ext_sub_cmds  on_cmd;
	struct disp_ext_sub_cmds  post_on_cmd;
	struct disp_ext_sub_cmds  off_cmd;
	struct disp_ext_sub_cmds  pwroff_cmd;
	struct disp_ext_sub_cmds  ram_wr_cmd;
	disp_ext_sub_state_type state;
	disp_ext_sub_modem_state_type modem_state;
	struct pinctrl            *pinctrl;
	bool first_update;
	struct disp_ext_sub_smd_data  smd_data;
	struct disp_ext_sub_smd_m2l_data  smd_m2l_data;
	struct disp_ext_sub_mdm_notify_data  mdm_notif;
	struct fb_var_subdispinfo subdispinfo;
	int  current_device_elec_vol;
};

int disp_ext_sub_panel_set_status(disp_ext_sub_state_type next_status, struct disp_ext_sub_pdata *pdata);
int disp_ext_sub_panel_update(struct fb_var_screeninfo *var, struct fb_info *info, uint8_t* apps_img_p);
int disp_ext_sub_set_cmd(void * payload_p, unsigned char payload_len, struct disp_ext_sub_pdata *pdata);
int disp_ext_sub_set_cmd2(void * payload_p, unsigned char payload_len, struct disp_ext_sub_pdata *pdata);
int disp_ext_sub_panel_bus_init(struct disp_ext_sub_pdata *pdata);
int disp_ext_sub_panel_signal_init(struct disp_ext_sub_pdata *pdata);
int disp_ext_sub_get_panel_dt(struct device_node * np, struct disp_ext_sub_pdata *pdata);
int disp_ext_sub_get_signal_dt(struct device_node * np, struct disp_ext_sub_pdata *pdata);
int disp_ext_sub_get_seq_dt(struct device_node * np, struct disp_ext_sub_pdata *pdata);
int disp_ext_sub_mdm_init( struct disp_ext_sub_pdata *pdata );
int disp_ext_sub_smd_send_request( struct disp_ext_sub_pdata *pdata, disp_ext_sub_linux2mdm_tx_type *request);
int disp_ext_sub_smd_send_command(struct disp_ext_sub_pdata *pdata, int cmd, int param);
int disp_ext_sub_smd_send_raw_data(struct disp_ext_sub_pdata *pdata, uint8_t *buf, uint32_t len);

int disp_ext_sub_set_subdispinfo(struct disp_ext_sub_pdata *pdata, struct fb_var_subdispinfo *psubdispinfo);
int disp_ext_sub_subdispinfo_init(struct disp_ext_sub_pdata *pdata);
int disp_ext_sub_get_subdispinfo(struct disp_ext_sub_pdata *pdata);
int disp_ext_sub_set_data(void * payload_p, unsigned char payload_len, struct disp_ext_sub_pdata *pdata);
int disp_ext_sub_get_battery_temp(void);
int disp_ext_sub_set_contrast(struct disp_ext_sub_pdata *pdata, bool force_set);
int disp_ext_sub_set_user_contrast(struct disp_ext_sub_pdata *pdata, int value, bool force_set);


#endif /* DISP_EXT_SUB_PANEL_H */
