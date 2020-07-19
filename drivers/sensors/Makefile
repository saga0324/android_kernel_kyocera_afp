# This software is contributed or developed by KYOCERA Corporation.
# (C) 2015 KYOCERA Corporation
############################################################################
#
# Makefile
#
# Copyright (C) 2011-2012 ALPS ELECTRIC CO., LTD. All Rights Reserved.
#
# This software is licensed under the terms of the GNU General Public
# License version 2, as published by the Free Software Foundation, and
# may be copied, distributed, and modified under those terms.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
############################################################################

# Debug Option
#CFLAGS_MODULE += -DALPS_IDEV_DEBUG
#CFLAGS_MODULE += -DALPS_MAG_DEBUG
#CFLAGS_MODULE += -DALPS_ACC_DEBUG

#ccflags-y := -mfloat-abi=softfp -mfpu=vfp

#ccflags-y += -Iinclude/linux
ccflags-y := -Iinclude/linux

obj-$(CONFIG_INPUT_SENSOR_PROXIMITY) += gp2ap030a.o
obj-$(CONFIG_INPUT_ALPS_INPUT) += alps-input.o
obj-$(CONFIG_INPUT_HSCDTD) += hscdtd007a_i2c.o
obj-$(CONFIG_INPUT_SENSOR_ACCELEROMETER) += u2dh_i2c.o

obj-$(CONFIG_SENSORS)		+= sensors_class.o
obj-$(CONFIG_SENSORS_SSC)	+= sensors_ssc.o
