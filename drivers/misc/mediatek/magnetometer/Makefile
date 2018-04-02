include $(srctree)/drivers/misc/mediatek/Makefile.custom

# In case the platform does NOT support this type of sensors
obj-y   +=  magnetmeter_dummy.o

ifeq ($(CONFIG_MTK_HSCDTD006),y)
obj-y	+=  hscdtd006/
endif

ifeq ($(CONFIG_MTK_AKM8963),y)
obj-y	+=  akm8963/
endif

#ifeq ($(CONFIG_MTK_BMM050-NEW),y)
#obj-y   +=  bmm050-new/
#endif

ifeq ($(CONFIG_LGE_L50_REVA),y)
obj-$(CONFIG_ALPS_HSCDTD008) +=  hscdtd008/
else
obj-$(CONFIG_BOSCH_BMM050) +=  bmm050/
endif

