/*****************************************************************************
*
* Filename:      irda_sd.h
* Version:       0.1
* Description:   IrDA driver
* Status:        Experimental
* Author:        KYOCERA Corporation
*
*   This software is contributed or developed by KYOCERA Corporation.
*	(C) 2016 KYOCERA Corporation
*
*   This program is free software; you can redistribute it and/or
*   modify it under the terms of the GNU General Public License
*   as published by the Free Software Foundation; only version 2.
*
*	This program is distributed in the hope that it will be useful,
*	but WITHOUT ANY WARRANTY; without even the implied warranty of
*	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*	GNU General Public License for more details.
*
*	You should have received a copy of the GNU General Public License
*	along with this program; if not, write to the Free Software
*   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*
*****************************************************************************/
#if !defined(__irda_sd_h__)
#define	__irda_sd_h__

#include <linux/spinlock.h>
#include <linux/skbuff.h>
#include <linux/module.h>
#include <linux/cdev.h>
MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("IrDASD driver work");
MODULE_LICENSE("GPL");


#define MODULE_NAME "irdasd"


struct irda_sd_info {
	dev_t dev_num;
	struct device *dev;
	struct cdev *cdev;
};



#endif
