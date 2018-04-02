//#include <mach/mt6577_clock_manager.h>
#include "mt6577_clock_manager.h"

#include "mt6577_smi.h"
#include "sync_write.h"

unsigned long u4SMILarbBaseAddr[SMI_LARB_NUMBER];
unsigned long u4SMICommBaseAddr[SMI_COMM_NUMBER];

void SMI_Init(void)
{
    u4SMILarbBaseAddr[0] = SMI_LARB0_BASE;
    u4SMILarbBaseAddr[1] = SMI_LARB1_BASE;
    u4SMILarbBaseAddr[2] = SMI_LARB2_BASE;
    u4SMILarbBaseAddr[3] = SMI_LARB3_BASE;

    u4SMICommBaseAddr[0] = SMI_COMMON_BASE;
}

/*
typedef struct {
	unsigned long u4Master;   //SMI larbno 0~3
    unsigned long u4PortNo;
    unsigned long bBusType : 1;//0 for GMC, 1 for AXI
    unsigned long bDestType : 2;//0 for EMI+internal mem, 1 for EMI, 3 for internal mem
    unsigned long bRWType : 2;//0 for R+W, 1 for read, 2 for write
}SMIBMCfg;
*/

void SMI_SetSMIBMCfg(unsigned long larbno, unsigned long portno,
                                         unsigned long desttype, unsigned long rwtype)
{
    mt65xx_reg_sync_writel(portno , MT6575SMI_MON_PORT(u4SMILarbBaseAddr[larbno]));
    mt65xx_reg_sync_writel( ((rwtype<< 2) | desttype) , MT6575SMI_MON_TYPE(u4SMILarbBaseAddr[larbno]));
}

void SMI_SetMonitorControl (SMIBMCfg_Ext *cfg_ex) {

    int i;
    unsigned long u4RegVal;

    if(cfg_ex != NULL)
    {
        u4RegVal = (((unsigned long)cfg_ex->uStarvationTime << 8) | ((unsigned long)cfg_ex->bStarvationEn << 6) |
        ((unsigned long)cfg_ex->bMaxPhaseSelection << 5) | ((unsigned long)cfg_ex->bDPSelection << 4) |
        ((unsigned long)cfg_ex->uIdleOutStandingThresh << 1) | cfg_ex->bIdleSelection);
//        printk("Ex configuration %lx\n", u4RegVal);
    }
    else // default
    {
    	  u4RegVal = (((unsigned long)8 << 8) | ((unsigned long)1 << 6) |
        ((unsigned long)1 << 5) | ((unsigned long)1 << 4) |
        ((unsigned long)3 << 1) | 1);
//        printk("default configuration %lx\n", u4RegVal);
    }

    for (i=0; i<SMI_LARB_NUMBER; i++) {
        mt65xx_reg_sync_writel(u4RegVal , MT6575SMI_MON_CON(u4SMILarbBaseAddr[i]));
    }

}

void SMI_Disable(unsigned long larbno) {
        mt65xx_reg_sync_writel(0 , MT6575SMI_MON_ENA(u4SMILarbBaseAddr[larbno]));//G2D
}

void SMI_Enable(unsigned long larbno, unsigned long bustype) {
//Resume Counter
    //if(cfg->bBusType == 0) //GMC
    if( bustype ==0 ) //GMC
    {
        mt65xx_reg_sync_writel(0x1 , MT6575SMI_MON_ENA(u4SMILarbBaseAddr[larbno]));//G2D
    }
    else //AXI
    {
        mt65xx_reg_sync_writel(0x3 , MT6575SMI_MON_ENA(u4SMILarbBaseAddr[larbno]));//GPU
    }
    // Cross Page Prevent , HW designer suggests add.
    *((volatile unsigned int *)0xF000882C) &= ~0x01;
    *((volatile unsigned int *)0xF000882C) |= 0x01;

}

void SMI_Comm_Disable(unsigned long commonno) {
	mt65xx_reg_sync_writel(0 , SMI_COMM_MON_ENA(u4SMICommBaseAddr[commonno]));
}

void SMI_Comm_Enable(unsigned long commonno) {
	mt65xx_reg_sync_writel(0x1 , SMI_COMM_MON_ENA(u4SMICommBaseAddr[commonno]));
}

void SMI_Comm_Clear(int commonno)
{
//Clear Counter
	mt65xx_reg_sync_writel(1 , SMI_COMM_MON_CLR(u4SMICommBaseAddr[commonno]));
	mt65xx_reg_sync_writel(0 , SMI_COMM_MON_CLR(u4SMICommBaseAddr[commonno]));
}

void SMI_SetCommBMCfg(unsigned long commonno, unsigned long portno,
                      unsigned long desttype, unsigned long rwtype)
{
	mt65xx_reg_sync_writel( ((rwtype<< 2) | desttype) , SMI_COMM_MON_TYPE(u4SMICommBaseAddr[commonno]));
}

void SMI_Pause(int larbno)
{
//Pause Counter
    mt65xx_reg_sync_writel(0 , MT6575SMI_MON_ENA(u4SMILarbBaseAddr[larbno]));
}

void SMI_Clear(int larbno)
{
//Clear Counter
        mt65xx_reg_sync_writel(1 , MT6575SMI_MON_CLR(u4SMILarbBaseAddr[larbno]));
        mt65xx_reg_sync_writel(0 , MT6575SMI_MON_CLR(u4SMILarbBaseAddr[larbno]));
}

//Get SMI result
/* result->u4ActiveCnt = ioread32(MT6575SMI_MON_ACT_CNT(u4SMIBaseAddr));
    result->u4RequestCnt = ioread32(MT6575SMI_MON_REQ_CNT(u4SMIBaseAddr));
    result->u4IdleCnt = ioread32(MT6575SMI_MON_IDL_CNT(u4SMIBaseAddr));
    result->u4BeatCnt = ioread32(MT6575SMI_MON_BEA_CNT(u4SMIBaseAddr));
    result->u4ByteCnt = ioread32(MT6575SMI_MON_BYT_CNT(u4SMIBaseAddr));
    result->u4CommPhaseAccum = ioread32(MT6575SMI_MON_CP_CNT(u4SMIBaseAddr));
    result->u4DataPhaseAccum = ioread32(MT6575SMI_MON_DP_CNT(u4SMIBaseAddr));
    result->u4MaxCommOrDataPhase = ioread32(MT6575SMI_MON_CDP_MAX(u4SMIBaseAddr));
    result->u4MaxOutTransaction = ioread32(MT6575SMI_MON_COS_MAX(u4SMIBaseAddr));
*/

int SMI_GetCount (void *addr) {
    unsigned int iCount;
    iCount = ioread32(addr);
    return iCount;
}

int SMI_GetPortNo(int larbno) {
    //return SMI_GetCount((void *)(MT6575SMI_MON_PORT(u4SMILarbBaseAddr[i])));
    return ioread32(MT6575SMI_MON_PORT(u4SMILarbBaseAddr[larbno]));
}

/* === get counter === */

int SMI_GetActiveCnt(int larbno) {
    //return SMI_GetCount(MT6575SMI_MON_ACT_CNT(u4SMILarbBaseAddr[i]));
    return ioread32(MT6575SMI_MON_ACT_CNT(u4SMILarbBaseAddr[larbno]));
}

int SMI_GetRequestCnt(int larbno) {
    //return SMI_GetCount(MT6575SMI_MON_REQ_CNT(u4SMILarbBaseAddr[i]));
    return ioread32(MT6575SMI_MON_REQ_CNT(u4SMILarbBaseAddr[larbno]));
}

int SMI_GetIdleCnt(int larbno) {
    //return SMI_GetCount(MT6575SMI_MON_IDL_CNT(u4SMILarbBaseAddr[i]));
    return ioread32(MT6575SMI_MON_IDL_CNT(u4SMILarbBaseAddr[larbno]));
}

int SMI_GetBeatCnt(int larbno) {
    //return SMI_GetCount(MT6575SMI_MON_BEA_CNT(u4SMILarbBaseAddr[i]));
    return ioread32(MT6575SMI_MON_BEA_CNT(u4SMILarbBaseAddr[larbno]));
}

int SMI_GetByteCnt(int larbno) {
    //return SMI_GetCount(MT6575SMI_MON_BYT_CNT(u4SMILarbBaseAddr[i]));
    return ioread32(MT6575SMI_MON_BYT_CNT(u4SMILarbBaseAddr[larbno]));
}

//get common counter
int SMI_Comm_GetActiveCnt(int commonno) {
	return ioread32(SMI_COMM_MON_ACT_CNT(u4SMICommBaseAddr[commonno]));
}

int SMI_Comm_GetRequestCnt(int commonno) {
	return ioread32(SMI_COMM_MON_REQ_CNT(u4SMICommBaseAddr[commonno]));
}

int SMI_Comm_GetIdleCnt(int commonno) {
	return ioread32(SMI_COMM_MON_IDL_CNT(u4SMICommBaseAddr[commonno]));
}

int SMI_Comm_GetBeatCnt(int commonno) {
	return ioread32(SMI_COMM_MON_BEA_CNT(u4SMICommBaseAddr[commonno]));
}

int SMI_Comm_GetByteCnt(int commonno) {
	return ioread32(SMI_COMM_MON_BYT_CNT(u4SMICommBaseAddr[commonno]));
}

void SMI_PowerOn(void)
{
    if(enable_clock(MT65XX_PDN_MM_SMI_LARB0, "SMI_LARB0")){printk("Enable SMI_LARB0 clock failed!\n");}
    if(enable_clock(MT65XX_PDN_MM_SMI_LARB0_EMI, "SMI_LARB0_EMI")){printk("Enable SMI_LARB0_EMI clock failed!\n");}

    if(enable_clock(MT65XX_PDN_MM_SMI_LARB1, "SMI_LARB1")){printk("Enable SMI_LARB1 clock failed!\n");}
    if(enable_clock(MT65XX_PDN_MM_SMI_LARB1_EMI, "SMI_LARB1_EMI")){printk("Enable SMI_LARB1_EMI clock failed!\n");}

    if(enable_clock(MT65XX_PDN_MM_SMI_LARB2_ACP_BUS, "SMI_LARB2_ACP_BUS")){printk("Enable SMI_LARB2_ACP_BUS clock failed!\n");}
    if(enable_clock(MT65XX_PDN_MM_SMI_LARB2_260MHZ, "SMI_LARB2_260MHZ")){printk("Enable SMI_LARB2_260MHZ clock failed!\n");}
    if(enable_clock(MT65XX_PDN_MM_SMI_LARB2_EMI, "SMI_LARB2_EMI")){printk("Enable SMI_LARB2_EMI clock failed!\n");}
    if(enable_clock(MT65XX_PDN_MM_SMI_LARB2_ACP_BUS_EMI, "SMI_LARB2_ACP_BUS_EMI")){printk("Enable SMI_LARB2_ACP_BUS_EMI clock failed!\n");}
    if(enable_clock(MT65XX_PDN_MM_SMI_LARB2, "SMI_LARB2")){printk("Enable SMI_LARB2 clock failed!\n");}

    if(enable_clock(MT65XX_PDN_MM_SMI_LARB3_FULL, "SMI_LARB3_FULL")){printk("Enable SMI_LARB3_FULL clock failed!\n");}
    if(enable_clock(MT65XX_PDN_MM_SMI_LARB3_HALF, "MSMI_LARB3_HALF")){printk("Enable SMI_LARB3_HALF clock failed!\n");}

//printk("SMI clock on\n");
}

void SMI_PowerOff(void)
{
    if(disable_clock(MT65XX_PDN_MM_SMI_LARB3_HALF, "MSMI_LARB3_HALF")){printk("Enable SMI_LARB3_HALF clock failed!\n");}
    if(disable_clock(MT65XX_PDN_MM_SMI_LARB3_FULL, "SMI_LARB3_FULL")){printk("Enable SMI_LARB3_FULL clock failed!\n");}

    if(disable_clock(MT65XX_PDN_MM_SMI_LARB2, "SMI_LARB2")){printk("Enable SMI_LARB2 clock failed!\n");}
    if(disable_clock(MT65XX_PDN_MM_SMI_LARB2_ACP_BUS_EMI, "SMI_LARB2_ACP_BUS_EMI")){printk("Enable SMI_LARB2_ACP_BUS_EMI clock failed!\n");}
    if(disable_clock(MT65XX_PDN_MM_SMI_LARB2_EMI, "SMI_LARB2_EMI")){printk("Enable SMI_LARB2_EMI clock failed!\n");}
    if(disable_clock(MT65XX_PDN_MM_SMI_LARB2_260MHZ, "SMI_LARB2_260MHZ")){printk("Enable SMI_LARB2_260MHZ clock failed!\n");}
    if(disable_clock(MT65XX_PDN_MM_SMI_LARB2_ACP_BUS, "SMI_LARB2_ACP_BUS")){printk("Enable SMI_LARB2_ACP_BUS clock failed!\n");}

    if(disable_clock(MT65XX_PDN_MM_SMI_LARB1_EMI, "SMI_LARB1_EMI")){printk("Enable SMI_LARB1_EMI clock failed!\n");}
    if(disable_clock(MT65XX_PDN_MM_SMI_LARB1, "SMI_LARB1")){printk("Enable SMI_LARB1 clock failed!\n");}

    if(disable_clock(MT65XX_PDN_MM_SMI_LARB0_EMI, "SMI_LARB0_EMI")){printk("Enable SMI_LARB0_EMI clock failed!\n");}
    if(disable_clock(MT65XX_PDN_MM_SMI_LARB0, "SMI_LARB0")){printk("Enable SMI_LARB0 clock failed!\n");}

//printk("SMI clock off\n");
}