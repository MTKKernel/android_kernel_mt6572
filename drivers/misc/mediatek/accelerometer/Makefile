include $(srctree)/drivers/misc/mediatek/Makefile.custom

# In case the platform does NOT support this type of sensors
obj-y   +=  accelerometer_dummy.o

#ifeq ($(CONFIG_MTK_KXTJ2_1009),y)
#obj-y	+=  KXTJ2_1009/
#endif

ifeq ($(CONFIG_MTK_KXTIK1004_AUTO),y)
obj-y   +=  KXTIK1004_AUTO/
endif

ifeq ($(CONFIG_MTK_MMA8452Q_AUTO),y)
obj-y	+=  MMA8452Q_AUTO/
endif

#ifeq ($(CONFIG_LGE_L50_REVA),y)
#obj-$(CONFIG_STM_K2DH) += k2dh/
#else
#obj-$(CONFIG_MTK_BMA255) += bma255_l50/
#endif

# [LGE_BSP_COMMON] CHANGE_S: 2014-01-16 jongwoo82.lee@lge.com  Accel Driver porting
#ifeq ($(CONFIG_LGE_L30_REVA),y)
#obj-$(CONFIG_STM_K2DH) += k2dh/
#else
#obj-$(CONFIG_BOSCH_BMA255) += bma255/
#endif
# [LGE_BSP_COMMON] CHANGE_E: 2014-01-16 jongwoo82.lee@lge.com  Accel Driver porting

