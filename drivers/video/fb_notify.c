/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 */
/*
 *  linux/drivers/video/fb_notify.c
 *
 *  Copyright (C) 2006 Antonino Daplas <adaplas@pol.net>
 *
 *	2001 - Documented with DocBook
 *	- Brad Douglas <brad@neruo.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 */
#include <linux/fb.h>
#include <linux/notifier.h>
#include <linux/export.h>

static BLOCKING_NOTIFIER_HEAD(fb_notifier_list);

/**
 *	fb_register_client - register a client notifier
 *	@nb: notifier block to callback on events
 */
int fb_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&fb_notifier_list, nb);
}
EXPORT_SYMBOL(fb_register_client);

/**
 *	fb_unregister_client - unregister a client notifier
 *	@nb: notifier block to callback on events
 */
int fb_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&fb_notifier_list, nb);
}
EXPORT_SYMBOL(fb_unregister_client);

static int is_disp_ext_sub(struct fb_info *fb_info)
{
	return fb_info->node == 1;
}

/**
 * fb_notifier_call_chain - notify clients of fb_events
 *
 */
int fb_notifier_call_chain(unsigned long val, void *v)
{
	if (is_disp_ext_sub(((struct fb_event *)v)->info)) {
		return 0;
	}
	return blocking_notifier_call_chain(&fb_notifier_list, val, v);
}
EXPORT_SYMBOL_GPL(fb_notifier_call_chain);
