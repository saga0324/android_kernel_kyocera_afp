/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 */

#ifndef LOGCAPTURE_H_
#define LOGCAPTURE_H_

#include <linux/input.h>

#ifdef CONFIG_LOGCAPTURE

void check_logcapture(struct input_dev* dev,
			unsigned int code, int value);

void __init logcapture_init(void);

void __exit logcapture_exit(void);

#else

static inline void check_logcapture(struct input_dev* dev,
			unsigned int code, int value)
{ /* nothing to do */ }

static inline void logcapture_init(void)
{ /* nothing to do */ }

static inline void logcapture_exit(void)
{ /* nothing to do */ }

#endif /* CONFIG_LOGCAPTURE */

#endif /* LOGCAPTURE_H_ */
