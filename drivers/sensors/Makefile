#
# This software is contributed or developed by KYOCERA Corporation.
# (C) 2012 KYOCERA Corporation
# (C) 2013 KYOCERA Corporation
# (C) 2014 KYOCERA Corporation
# (C) 2015 KYOCERA Corporation
# (C) 2016 KYOCERA Corporation
# (C) 2017 KYOCERA Corporation
#
# Copyright (c) 2010 Yamaha Corporation
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
# USA.
#
ccflags-y := -mfloat-abi=softfp -mfpu=vfp

obj-$(CONFIG_SENSOR_POWER) += kc_sensor_power.o

obj-$(CONFIG_SENSOR_COMMON) += sensor_driver.o

obj-$(CONFIG_INPUT_SENSOR_ACCELEROMETER) += acc_driver.o
obj-$(CONFIG_INPUT_SENSOR_LINEAR_ACCELERATION) += acc_linear_driver.o
obj-$(CONFIG_INPUT_SENSOR_GAME_ROTATION_VECTOR) += game_rotation_vector_driver.o
obj-$(CONFIG_INPUT_SENSOR_GRAVITY) +=  gravity_driver.o
obj-$(CONFIG_INPUT_SENSOR_GYROSCOPE) += gyro_driver.o
obj-$(CONFIG_INPUT_SENSOR_UNCAL_GYROSCOPE) += gyro_uncal_driver.o
obj-$(CONFIG_INPUT_SENSOR_MAGNETOMETER) += mag_driver.o
obj-$(CONFIG_INPUT_SENSOR_MAG_ROTATION_VECTOR) += mag_rotation_vector_driver.o
obj-$(CONFIG_INPUT_SENSOR_UNCAL_MAGNETOMETER) += mag_uncal_driver.o
obj-$(CONFIG_INPUT_SENSOR_ORIENTATION) += orientation_driver.o
obj-$(CONFIG_INPUT_SENSOR_PRESSURE) +=  pressure_driver.o
obj-$(CONFIG_INPUT_SENSOR_PROXIMITY) += prox_driver.o
obj-$(CONFIG_INPUT_SENSOR_RELATIVE_HUMIDITY) += relative_humidity_driver.o
obj-$(CONFIG_INPUT_SENSOR_ROTATION_VECTOR) += rotation_vector_driver.o
obj-$(CONFIG_INPUT_SENSOR_EXT) += sensor_ext_driver.o
obj-$(CONFIG_INPUT_SENSOR_EXT_VH) += sensor_ext_vh_driver.o
obj-$(CONFIG_INPUT_SENSOR_STEP_COUNTER) += step_counter_driver.o
obj-$(CONFIG_INPUT_SENSOR_STEP_DETECTOR) += step_detector_driver.o
obj-$(CONFIG_INPUT_SENSOR_AMBIENT_TEMPERATURE) += temperature_ambient_driver.o
obj-$(CONFIG_INPUT_SENSOR_TEMPERATURE) += temperature_driver.o
obj-$(CONFIG_INPUT_SENSOR_SIGNIFICANT_MOTION) += significant_motion_driver.o
obj-$(CONFIG_INPUT_SENSOR_LIGHT) += light_driver.o
obj-$(CONFIG_INPUT_SENSOR_UNDERWATER_DETECT) += underwater_detect_driver.o

obj-$(CONFIG_INPUT_SENSOR_MICON) += sensor_micon_driver.o

obj-$(CONFIG_INPUT_SENSOR_ML630Q790) += ml630q790.o
obj-$(CONFIG_INPUT_SENSOR_GP2AP030) += gp2ap030a.o
obj-$(CONFIG_INPUT_SENSOR_RPR0521) += rohm_rpr0521_i2c.o
obj-$(CONFIG_INPUT_KC_MOTION_SENSOR) += kc_motion_sensor_driver.o

obj-$(CONFIG_SENSORS)		+= sensors_class.o
#obj-$(CONFIG_SENSORS_SSC)	+= sensors_ssc.o
