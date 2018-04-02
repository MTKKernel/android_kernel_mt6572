#
# Makefile for misc devices that really don't fit anywhere else.
#
include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-y				+= kd_flashlightlist.o
obj-y				+= main_strobe2.o
obj-y				+= strobe_part_id.o
obj-y				+= sub_strobe2.o
obj-y				+= sub_strobe.o

MTK_CUSTOM_PATH  := $(srctree)/drivers/misc/mediatek

subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/imgsensor/src/$(CONFIG_MTK_PLATFORM)/camera_hw