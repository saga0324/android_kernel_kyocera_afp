ccflags-y += -Idrivers/media/platform/msm/camera_v2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/msm_vb2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/camera
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/io
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/cci
obj-$(CONFIG_MSMB_CAMERA) += cci/ io/ csiphy/ csid/ actuator/ flash/ eeprom/ ois/
obj-$(CONFIG_MSM_CAMERA_SENSOR) += msm_sensor_init.o msm_sensor_driver.o msm_sensor.o
obj-$(CONFIG_MT9M114) += mt9m114.o
obj-$(CONFIG_OV5645) += ov5645.o
obj-$(CONFIG_OV5645) += ov7695.o
obj-$(CONFIG_GC0310) += gc0310.o
obj-$(CONFIG_M12MO) += m12mo_fw.o
obj-$(CONFIG_M12MO) += m12mo.o

clean-files := m12mo_fw.inc m12mo_fwup.inc
$(obj)/m12mo.o:	$(obj)/m12mo_fwup.inc
$(obj)/m12mo_fw.o:	$(obj)/m12mo_fw.inc

ifeq ($(CONFIG_M12MO),y)
quiet_cmd_build_m12mo_fwup = BUILD FW Updater $@
      cmd_build_m12mo_fwup = \
	od -v -tx1 -An $(srctree)/$(src)/m12mo_fwup.bin | \
	sed 's/ \([0-9a-f]\{2\}\)/0x\1,/g;1s/0x  ,//g;1s/^/static const uint8_t updater[]={\n/;$$s/,$$/\n};\n/' \
	> $(obj)/m12mo_fwup.inc
quiet_cmd_build_m12mo_fw = BUILD Firmware $@
      cmd_build_m12mo_fw = \
	od -v -tx1 -An $(srctree)/$(src)/m12mo_fw.bin | \
	sed 's/ \([0-9a-f]\{2\}\)/0x\1,/g;1s/0x  ,//g;1s/^/static const uint8_t firmware[]={\n/;$$s/,$$/\n};\n/' \
	> $(obj)/m12mo_fw.inc

$(obj)/m12mo_fwup.inc:	$(src)/m12mo_fwup.bin
	$(call if_changed,build_m12mo_fwup)
$(obj)/m12mo_fw.inc:	$(src)/m12mo_fw.bin
	$(call if_changed,build_m12mo_fw)
endif

