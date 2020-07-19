/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __KAUDIO_H__
#define __KAUDIO_H__

#include <linux/types.h>
#include <linux/ioctl.h>

#define KAUDIO_DRIVER_NAME "kaudio"

#define kaudio_MVM_STR "MVM"
#define kaudio_CVS_STR "CVS"
#define MAX_APR_SERVICE_NAME_LEN  64

#define MSG_REGISTER 0x1
#define MSG_REQUEST  0x2
#define MSG_RESPONSE 0x3

struct kaudio_write_msg {
	__u32 msg_type;
	__u8 payload[0];
};

struct kaudio_register {
	char svc_name[MAX_APR_SERVICE_NAME_LEN];
	__u32 src_port;
	__u8 reg_flag;
};

struct kaudio_cmd_response {
	__u32 src_port;
	__u32 dest_port;
	__u32 token;
	__u32 opcode;
	__u32 payload_size;
	__u8 payload[0];
};

struct kaudio_cmd_request {
	char svc_name[MAX_APR_SERVICE_NAME_LEN];
	__u32 src_port;
	__u32 dest_port;
	__u32 token;
	__u32 opcode;
	__u32 payload_size;
	__u8 payload[0];
};

#endif
