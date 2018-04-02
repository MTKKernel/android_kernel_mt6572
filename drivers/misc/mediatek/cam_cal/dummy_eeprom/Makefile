#
# Makefile for misc devices that really don't fit anywhere else.
#
include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-y				+= dummy_cam_cal.o

MTK_CUSTOM_PATH  := $(srctree)/drivers/misc/mediatek

subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/imgsensor/src/$(CONFIG_MTK_PLATFORM)/camera_hw