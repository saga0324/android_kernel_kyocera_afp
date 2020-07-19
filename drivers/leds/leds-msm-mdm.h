/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
*/

#ifndef __LEDS_MDM_H_INCLUDED
#define __LEDS_MDM_H_INCLUDED

#include <linux/types.h>
#include "leds-tricolor.h"

struct mdm_led;

typedef int (*send_request_func_t)(struct mdm_led *,
	const struct tricolor_led_request* request);

struct mdm_led {
	send_request_func_t   send_request_fn;
};

struct mdm_led *mdm_led_create(void);
void mdm_led_destroy(struct mdm_led *mdm_led);

#endif	/* __LEDS_MDM_H_INCLUDED */
