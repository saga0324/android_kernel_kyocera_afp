# This software is contributed or developed by KYOCERA Corporation.
# (C) 2015 KYOCERA Corporation
#

obj-$(CONFIG_MSM_PM)		+= msm-pm.o pm-data.o
obj-$(CONFIG_MSM_IDLE_STATS)	+= pm-stats.o
obj-$(CONFIG_MSM_IDLE_STATS)	+= lpm-stats.o
obj-$(CONFIG_MSM_NOPM)		+= no-pm.o
obj-$(CONFIG_PM)		+= pm-boot.o
obj-$(CONFIG_APSS_CORE_EA)	+= msm-core.o debug_core.o

ifeq ($(KC_DEBUGTOOL_ENABLED_EXPORT), enable)
CFLAGS_msm-pm.o += -DKC_BATTERY_LOG_ENABLED
endif
