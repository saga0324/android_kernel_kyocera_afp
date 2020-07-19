#
# This software is contributed or developed by KYOCERA Corporation.
# (C) 2019 KYOCERA Corporation
#

ccflags-y += -I$(src)			# needed for trace events

obj-y					+= ion/

obj-$(CONFIG_ANDROID_BINDER_IPC)	+= binder.o binder_alloc.o
obj-$(CONFIG_ASHMEM)			+= ashmem.o
obj-$(CONFIG_ANDROID_LOGGER)           += logger.o
obj-$(CONFIG_ANDROID_TIMED_OUTPUT)	+= timed_output.o
obj-$(CONFIG_ANDROID_TIMED_GPIO)	+= timed_gpio.o
obj-$(CONFIG_ANDROID_LOW_MEMORY_KILLER)	+= lowmemorykiller.o
obj-$(CONFIG_SYNC)			+= sync.o
obj-$(CONFIG_SW_SYNC)			+= sw_sync.o
obj-$(CONFIG_ONESHOT_SYNC)		+= oneshot_sync.o
obj-$(CONFIG_ANDROID_CRASH_REASON)		+= crash_reason.o

obj-$(CONFIG_ANDROID_KCJLOG)		+= kcjlog.o
CFLAGS_kcjlog.o += -DBUILD_DISPLAY_ID='"$(RESETLOG_BUILD_DISPLAY_ID)"'
CFLAGS_kcjlog.o += -DPRODUCT_MODEL_NAME='"$(RESETLOG_PRODUCT_MODEL_NAME)"'
