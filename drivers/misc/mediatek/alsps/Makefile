include $(srctree)/drivers/misc/mediatek/Makefile.custom

# In case the platform does NOT support this type of sensors
obj-y   +=  alsps_dummy.o

#ifeq ($(CONFIG_MTK_GP2AP002S00F),y)
#obj-y	+=  gp2ap002s00f/
#endif

#ifeq ($(CONFIG_MTK_CM36283),y)
#obj-y	+=  cm36283/
#endif

#ifeq ($(CONFIG_LGE_L50_REVA),y)
#obj-$(CONFIG_MTK_APDS9190) += apds9190/
#endif

ifeq ($(CONFIG_MTK_STK3171),y)
obj-y	+=  stk3171/
endif

