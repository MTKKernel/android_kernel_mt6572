include $(srctree)/drivers/misc/mediatek/Makefile.custom

# In case the platform does NOT support this type of sensors
obj-y   +=  hallic_dummy.o

ifeq ($(CONFIG_LGE_HALLIC),y)
obj-y	+=  bu52031nvx/
endif
