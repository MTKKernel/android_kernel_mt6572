include $(srctree)/drivers/misc/mediatek/Makefile.custom


# In case the platform does NOT support this type of sensors
obj-y   +=  mtk_tpd.o
obj-y   +=  tpd_button.o
obj-y   +=  tpd_calibrate.o
obj-y   +=  tpd_debug.o
obj-y   +=  tpd_default.o
obj-y   +=  tpd_init.o
obj-y   +=  tpd_misc.o
obj-y   +=  tpd_setting.o

#ifeq ($(CONFIG_MTK_GT9XX),y)
#obj-y	+=  GT9XX/
#endif

ifeq ($(CONFIG_MTK_MSG2133),y)
obj-y	+=  MSG2133/
endif

# [LGE_BSP_COMMON] CHANGE_S: 2014-01-17 junmo.kang@lge.com  Synaptics Touch porting
#ifeq ($(CONFIG_MTK_LU201X), y)
#obj-y += lu201x/
#endif

#ifeq ($(CONFIG_MTK_S2200), y)
#obj-y += synaptics_2200/
#endif
# [LGE_BSP_COMMON] CHANGE_E: 2014-01-17 junmo.kang@lge.com  Synaptics Touch porting

#ifeq ($(CONFIG_TOUCHSCREEN_SYNAPTICS_S2202), y)
#obj-y   += synaptics_2202/
#endif

#ifeq ($(CONFIG_MTK_FT5406),y)
#obj-y	+=  ft5406/
#endif
