#ifndef __LEDS_GPIO_OEM_H
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2017 KYOCERA Corporation
 */

enum {
	SPAMP_ID_SOUND	= 0x0001,	/* */
	SPAMP_ID_SUBBL	= 0x0002,	/* sub-lcd-backlight */
	SPAMP_ID_KEYBL	= 0x0004,	/* button-backlight(key-backlight) */
};
int spamp_gpio_enable(int id, int value);

#endif/* __LEDS_GPIO_OEM_H */
