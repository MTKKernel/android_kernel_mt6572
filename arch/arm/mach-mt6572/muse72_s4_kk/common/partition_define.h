
#ifndef __PARTITION_DEFINE_H__
#define __PARTITION_DEFINE_H__




#define KB  (1024)
#define MB  (1024 * KB)
#define GB  (1024 * MB)

#define PART_PRELOADER "PRELOADER" 
#define PART_MBR "MBR" 
#define PART_EBR1 "EBR1" 
#define PART_MISC2 "MISC2" 
#define PART_FTM "FTM" 
#define PART__EXT_PL "_EXT_PL" 
#define PART_PRO_INFO "PRO_INFO" 
#define PART_NVRAM "NVRAM" 
#define PART_PROTECT_F "PROTECT_F" 
#define PART_PROTECT_S "PROTECT_S" 
#define PART_SECCFG "SECCFG" 
#define PART_UBOOT "UBOOT" 
#define PART_BOOTIMG "BOOTIMG" 
#define PART_RECOVERY "RECOVERY" 
#define PART_SEC_RO "SEC_RO" 
#define PART_MISC "MISC" 
#define PART_LOGO "LOGO" 
#define PART_EBR2 "EBR2" 
#define PART_EBR3 "EBR3" 
#define PART_EXPDB "EXPDB" 
#define PART_PERSIST_LG "PERSIST_LG" 
#define PART_PERSIST "PERSIST" 
#define PART_MPT "MPT" 
#define PART_LGFOTA "LGFOTA" 
#define PART_CUST "CUST" 
#define PART_RCT "RCT" 
#define PART_FACTORY "FACTORY" 
#define PART_ANDROID "ANDROID" 
#define PART_CACHE "CACHE" 
#define PART_USRDATA "USRDATA" 
#define PART_FAT "FAT" 
#define PART_BMTPOOL "BMTPOOL" 
/*preloader re-name*/
#define PART_SECURE "SECURE" 
#define PART_SECSTATIC "SECSTATIC" 
#define PART_ANDSYSIMG "ANDSYSIMG" 
#define PART_USER "USER" 
/*Uboot re-name*/
#define PART_APANIC "APANIC" 

#define PART_FLAG_NONE              0 
#define PART_FLAG_LEFT             0x1 
#define PART_FLAG_END              0x2 
#define PART_MAGIC              0x58881688 

#define PART_SIZE_PRELOADER			(12288LL*KB)
#define PART_SIZE_MBR			(512LL*KB)
#define PART_SIZE_EBR1			(512LL*KB)
#define PART_SIZE_MISC2			(8192LL*KB)
#define PART_SIZE_FTM			(8192LL*KB)
#define PART_SIZE__EXT_PL			(512LL*KB)
#define PART_SIZE_PRO_INFO			(3072LL*KB)
#define PART_SIZE_NVRAM			(5120LL*KB)
#define PART_SIZE_PROTECT_F			(10240LL*KB)
#define PART_SIZE_PROTECT_S			(10240LL*KB)
#define PART_SIZE_SECCFG			(512LL*KB)
#define PART_OFFSET_SECCFG			(0x3980000)
#define PART_SIZE_UBOOT			(512LL*KB)
#define PART_SIZE_BOOTIMG			(10240LL*KB)
#define PART_SIZE_RECOVERY			(10240LL*KB)
#define PART_SIZE_SEC_RO			(512LL*KB)
#define PART_OFFSET_SEC_RO			(0x4e80000)
#define PART_SIZE_MISC			(512LL*KB)
#define PART_SIZE_LOGO			(3072LL*KB)
#define PART_SIZE_EBR2			(512LL*KB)
#define PART_SIZE_EBR3			(512LL*KB)
#define PART_SIZE_EXPDB			(10240LL*KB)
#define PART_SIZE_PERSIST_LG			(8192LL*KB)
#define PART_SIZE_PERSIST			(6144LL*KB)
#define PART_SIZE_MPT			(22528LL*KB)
#define PART_SIZE_LGFOTA			(10240LL*KB)
#define PART_SIZE_CUST			(102400LL*KB)
#define PART_SIZE_RCT			(1024LL*KB)
#define PART_SIZE_FACTORY			(20480LL*KB)
#define PART_SIZE_ANDROID			(1536000LL*KB)
#define PART_SIZE_CACHE			(350208LL*KB)
#define PART_SIZE_USRDATA			(1638400LL*KB)
#define PART_SIZE_FAT			(0LL*KB)
#define PART_SIZE_BMTPOOL			(0x20)


#define PART_NUM			32



#define PART_MAX_COUNT			 40

#define MBR_START_ADDRESS_BYTE			(12288*KB)

#define WRITE_SIZE_Byte		512
typedef enum  {
	EMMC = 1,
	NAND = 2,
} dev_type;

typedef enum {
	USER = 0,
	BOOT_1,
	BOOT_2,
	RPMB,
	GP_1,
	GP_2,
	GP_3,
	GP_4,
} Region;


struct excel_info{
	char * name;
	unsigned long long size;
	unsigned long long start_address;
	dev_type type ;
	unsigned int partition_idx;
	Region region;
};
#if defined(MTK_EMMC_SUPPORT) || defined(CONFIG_MTK_EMMC_SUPPORT)
/*MBR or EBR struct*/
#define SLOT_PER_MBR 4
#define MBR_COUNT 8

struct MBR_EBR_struct{
	char part_name[8];
	int part_index[SLOT_PER_MBR];
};

extern struct MBR_EBR_struct MBR_EBR_px[MBR_COUNT];
#endif
extern struct excel_info PartInfo[PART_NUM];


#endif
