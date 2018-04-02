include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifeq ($(CONFIG_ARCH_MT6572), y)
  obj-$(CONFIG_MTK_SENSOR_SUPPORT) += mt6572/ 
endif

MTK_CUSTOM_PATH  := $(srctree)/drivers/misc/mediatek

subdir-ccflags-y += -I$(MTK_CUSTOM_PATH)/imgsensor/inc \
                    -I$(MTK_CUSTOM_PATH)/imgsensor/src/$(CONFIG_MTK_PLATFORM)/camera_hw