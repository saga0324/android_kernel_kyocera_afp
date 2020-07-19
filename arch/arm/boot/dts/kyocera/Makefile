dtb-$(CONFIG_ARCH_MSM8909) += msm8909-NATSUME01-rev00.dtb \
                              msm8909-NATSUME01-rev10.dtb \
                              msm8909-NATSUME01-rev20.dtb \
                              msm8909-NATSUME01_1G-rev00.dtb \
                              msm8909-NATSUME01_1G-rev10.dtb \
                              msm8909-NATSUME01_1G-rev20.dtb \
                              msm8909-NATSUME02-rev00.dtb \
                              msm8909-NATSUME02-rev10.dtb \
                              msm8909-NATSUME03-rev00.dtb

ifeq ($(CONFIG_ARM64),y)
always          := $(dtb-y)
subdir-y        := $(dts-dirs)
else
targets += dtbs
targets += $(addprefix ../, $(dtb-y))

$(obj)/../%.dtb: $(src)/%.dts FORCE
        $(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/,$(dtb-y))
endif
clean-files := *.dtb
