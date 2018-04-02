#
# Makefile for misc devices that really don't fit anywhere else.
#
include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-y += mt65xx_lcm_list.o

LCM_LISTS := $(addsuffix /, $(shell echo $(CONFIG_CUSTOM_KERNEL_LCM)))
obj-y += $(LCM_LISTS)