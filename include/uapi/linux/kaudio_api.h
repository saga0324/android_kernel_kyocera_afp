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

#ifndef __KAUDIO_API_H__
#define __KAUDIO_API_H__

#define KAUDIO_DEVICE_NAME "/dev/kaudio"

#define KAUDIO_SIDETONE_UNMUTE			0
#define KAUDIO_SIDETONE_MUTE			  1

enum {
	KAUDIO_IOCTL_GET_JACK_STATE = 1,
	KAUDIO_IOCTL_SET_SYSTEM_RESUME_TIMEOUT,
};

typedef struct {
	unsigned int	value;
} kaudio_jack_state_t;

typedef struct {
	unsigned int	value;
} kaudio_system_resume_timeout_t;

typedef union {
	kaudio_jack_state_t				jack_state;
	kaudio_system_resume_timeout_t	system_resume_timeout;
} kaudio_ioctl_t ;

extern int kaudio_codec_sidetone_mute( int mute );

#endif
