include $(srctree)/drivers/misc/mediatek/Makefile.custom

#obj-y	:=  yusu_android_speaker.o

ifeq ($(CONFIG_LGE_L50_REVA), y)
	obj-$(CONFIG_MTK_SOUND) += amp_tpa6136a2/
else
	obj-$(CONFIG_MTK_SOUND) += amp_tpa2058d3/
endif
