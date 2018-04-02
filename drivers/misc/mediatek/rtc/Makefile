include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-$(CONFIG_MTK_RTC) += mtk_rtc_common.o

# LGE_CHANGE_S : 2014-01-06 yeonsoo7.kim@lge.com, LG Secure Clock
obj-$(CONFIG_RTC_INTF_SECCLK) += secclk.o
# LGE_CHANGE_E : 2014-01-06 yeonsoo7.kim@lge.com, LG Secure Clock

ifeq ($(CONFIG_ARCH_MT6572), y)
  obj-y += mt6572/
endif
ifeq ($(CONFIG_ARCH_MT6582), y)
  obj-y += mt6582/
endif
obj-$(CONFIG_MTK_ACCDET) += $(subst ",,$(CONFIG_MTK_PLATFORM))/

