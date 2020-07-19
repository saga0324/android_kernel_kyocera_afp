/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
*/

#ifndef __LEDS_TRICOLOR_H_INCLUDED
#define __LEDS_TRICOLOR_H_INCLUDED

#include <linux/types.h>

#define TRICOLOR_RGB_MAX_BRIGHT_VAL      (0xFFFFFFFFu)
#define TRICOLOR_RGB_GET_R(color)        (((color) & 0x00FF0000u) >> 16)
#define TRICOLOR_RGB_GET_G(color)        (((color) & 0x0000FF00u) >> 8 )
#define TRICOLOR_RGB_GET_B(color)        (((color) & 0x000000FFu) >> 0 )
#define TRICOLOR_RGB_MASK                (0x00FFFFFFu)
#define TRICOLOR_RGB_OFF                 (0x00000000u)

enum tricolor_blink_control_enum {
	TRICOLOR_NO_BLINK_REQUEST = 0,
	TRICOLOR_BLINK_REQUEST,
};

struct tricolor_led_request {
	uint32_t    color;
	uint32_t    mode;
	uint32_t    on_time;
	uint32_t    off_time;
	uint32_t    off_color;
};

#endif	/* __LEDS_TRICOLOR_H_INCLUDED */
