#include <linux/module.h>
#include "partition_define.h"
struct excel_info PartInfo[PART_NUM]={
			{"preloader",12582912,0x0, EMMC, 0,BOOT_1},
			{"mbr",524288,0xc00000, EMMC, 0,USER},
			{"ebr1",524288,0xc80000, EMMC, 1,USER},
			{"misc2",8388608,0xd00000, EMMC, 0,USER},
			{"ftm",8388608,0x1500000, EMMC, 0,USER},
			{"_ext_pl",524288,0x1d00000, EMMC, 0,USER},
			{"pro_info",3145728,0x1d80000, EMMC, 0,USER},
			{"nvram",5242880,0x2080000, EMMC, 0,USER},
			{"protect_f",10485760,0x2580000, EMMC, 2,USER},
			{"protect_s",10485760,0x2f80000, EMMC, 3,USER},
			{"seccfg",524288,0x3980000, EMMC, 0,USER},
			{"uboot",524288,0x3a00000, EMMC, 0,USER},
			{"bootimg",10485760,0x3a80000, EMMC, 0,USER},
			{"recovery",10485760,0x4480000, EMMC, 0,USER},
			{"sec_ro",524288,0x4e80000, EMMC, 0,USER},
			{"misc",524288,0x4f00000, EMMC, 0,USER},
			{"logo",3145728,0x4f80000, EMMC, 0,USER},
			{"ebr2",524288,0x5280000, EMMC, 0,USER},
			{"ebr3",524288,0x5300000, EMMC, 0,USER},
			{"expdb",10485760,0x5380000, EMMC, 0,USER},
			{"persist_lg",8388608,0x5d80000, EMMC, 4,USER},
			{"persist",6291456,0x6580000, EMMC, 5,USER},
			{"mpt",23068672,0x6b80000, EMMC, 6,USER},
			{"lgfota",10485760,0x8180000, EMMC, 0,USER},
			{"cust",104857600,0x8b80000, EMMC, 7,USER},
			{"rct",1048576,0xef80000, EMMC, 0,USER},
			{"factory",20971520,0xf080000, EMMC, 0,USER},
			{"android",1572864000,0x10480000, EMMC, 8,USER},
			{"cache",358612992,0x6e080000, EMMC, 9,USER},
			{"usrdata",1677721600,0x83680000, EMMC, 10,USER},
			{"fat",0,0xe7680000, EMMC, 11,USER},
			{"bmtpool",4194304,0xFFFF0020, EMMC, 0,USER},
 };
EXPORT_SYMBOL(PartInfo);

#if defined(MTK_EMMC_SUPPORT) || defined(CONFIG_MTK_EMMC_SUPPORT)
struct MBR_EBR_struct MBR_EBR_px[MBR_COUNT]={
	{"mbr", {1, 2, 3, 4, }},
	{"ebr1", {5, 6, 7, }},
	{"ebr2", {8, 9, 10, }},
	{"ebr3", {11, }},
};

EXPORT_SYMBOL(MBR_EBR_px);
#endif

