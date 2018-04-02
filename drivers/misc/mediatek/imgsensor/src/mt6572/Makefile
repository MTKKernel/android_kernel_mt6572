#
# Makefile for misc devices that really don't fit anywhere else.
#
include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-y				+= kd_sensorlist.o
obj-y                           += camera_hw/
custom_kernel_imgsensor := $(addsuffix /, $(shell echo $(CONFIG_CUSTOM_KERNEL_IMGSENSOR)))
obj-y += $(custom_kernel_imgsensor)