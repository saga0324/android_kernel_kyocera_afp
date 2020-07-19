/*
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
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 * (C) 2018 KYOCERA Corporation
 * (C) 2019 KYOCERA Corporation
 */

#ifndef SWIC_H
#define SWIC_H

typedef enum {
	SWIC_UNINITIALIZE		= 0x00,
	SWIC_NO_ACCESSORY,
	SWIC_USB_MODE,
	SWIC_AUDIO_MODE,
} swic_accessory_enum;

typedef enum {
	SWIC_MODE_PLUGOUT_TO_AUDIO	= 0x00,
	SWIC_MODE_AUDIO_TO_PLUGOUT,
	SWIC_MODE_PLUGOUT_TO_USB,
	SWIC_MODE_USB_TO_PLUGOUT,
} swic_state_enum;

typedef void (*swic_event_func)(swic_accessory_enum accessoryinfo);

struct swic_event_callback {
    swic_event_func fn;
};

extern int32_t swic_reg_cbfunc(struct swic_event_callback* cb);
extern int32_t swic_unreg_cbfunc(struct swic_event_callback* cb);
extern void swic_set_mic_exist(bool mic);
extern swic_accessory_enum swic_get_accessory(void);
extern void swic_notify_from_cclogic(swic_state_enum en_state);
extern u8 swic_set_dminfo_from_cclogic(u8 ccic_reg01, u8 ccic_reg02, u8 ccic_reg03, u8 ccic_reg04);
extern void swic_detect_audio(bool is_audio);

static inline bool swic_is_fix_accessory(u8 value)
{
	bool ret = false;
	switch (value) {
	case SWIC_USB_MODE :
	case SWIC_AUDIO_MODE :
	case SWIC_NO_ACCESSORY :
		ret = true;
		break;
	case SWIC_UNINITIALIZE :
	default :
		break;
	}
	return ret;
}

#endif
