include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(MTK_PROJECT_PATH_ROOT)/touchpanel/MSG2133/

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/MSG2133/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/


ifeq ($(CONFIG_MTK_PLATFORM),"mt6572")
    ccflags-y += -DMT6572
endif

obj-y	+=  msg2133_driver.o

