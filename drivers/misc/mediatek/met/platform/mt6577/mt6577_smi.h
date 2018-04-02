#ifndef __MT6577_SMI_H__
#define __MT6577_SMI_H__


#define SMI_LARB_NUMBER 4
#define SMI_COMM_NUMBER 1

#define SMI_REQ_OK           (0)
#define SMI_ERR_WRONG_REQ    (-1)
#define SMI_ERR_OVERRUN      (-2)

#define MT6575SMI_MON_ENA(i) (i + 0x400)
#define MT6575SMI_MON_CLR(i) (i + 0x404)
#define MT6575SMI_MON_PORT(i) (i + 0x408)
#define MT6575SMI_MON_TYPE(i) (i + 0x40C)
#define MT6575SMI_MON_CON(i) (i + 0x410)
#define MT6575SMI_MON_ACT_CNT(i) (i + 0x420)
#define MT6575SMI_MON_REQ_CNT(i) (i + 0x424)
#define MT6575SMI_MON_IDL_CNT(i) (i + 0x428)
#define MT6575SMI_MON_BEA_CNT(i) (i + 0x42C)
#define MT6575SMI_MON_BYT_CNT(i) (i + 0x430)
#define MT6575SMI_MON_CP_CNT(i) (i + 0x434)
#define MT6575SMI_MON_DP_CNT(i) (i + 0x438)
#define MT6575SMI_MON_CDP_MAX(i) (i + 0x43C)
#define MT6575SMI_MON_COS_MAX(i) (i + 0x440)
#define MT6575SMI_MON_BUS_REQ0(i) (i + 0x450)

#define SMI_COMM_MON_ENA(i) (i + 0x1A0)
#define SMI_COMM_MON_CLR(i) (i + 0x1A4)
//#define SMI_COMM_MON_PORT(i) (i + 0x408)
#define SMI_COMM_MON_TYPE(i) (i + 0x1AC)
#define SMI_COMM_MON_CON(i) (i + 0x1B0)
#define SMI_COMM_MON_ACT_CNT(i) (i + 0x1C0)
#define SMI_COMM_MON_REQ_CNT(i) (i + 0x1C4)
#define SMI_COMM_MON_IDL_CNT(i) (i + 0x1C8)
#define SMI_COMM_MON_BEA_CNT(i) (i + 0x1CC)
#define SMI_COMM_MON_BYT_CNT(i) (i + 0x1D0)
#define SMI_COMM_MON_CP_CNT(i) (i + 0x1D4)
#define SMI_COMM_MON_DP_CNT(i) (i + 0x1D8)
#define SMI_COMM_MON_CDP_MAX(i) (i + 0x1DC)
#define SMI_COMM_MON_COS_MAX(i) (i + 0x1E0)

typedef struct {
    unsigned long bIdleSelection : 1; // 0 : idle count increase when no request, and outstanding request is less than , 1 : idle count increase when there is no request and read/write data transfer.
    unsigned long uIdleOutStandingThresh : 3;
    unsigned long bDPSelection : 1; // 0 : data phase incresae 1 when any outstanding transaction waits for data transfer. 1 : data phase increase N when N out standing transaction are waiting.
    unsigned long bMaxPhaseSelection : 1;// 0 : Command pahse , 1 : Data phase.
    unsigned long bStarvationEn : 1; // 0 : disable , 1 : Enable
    unsigned long uStarvationTime : 8;
    unsigned long u2Reserved : 12; //Reserved
}SMIBMCfg_Ext;


void SMI_Init(void);
//void SMI_SetPort(unsigned long master, unsigned long portno);
void SMI_SetSMIBMCfg(unsigned long master, unsigned long portno, unsigned long desttype, unsigned long rwtype);
void SMI_SetMonitorControl (SMIBMCfg_Ext *cfg_ex);
void SMI_Enable(unsigned long master, unsigned long bustype);
void SMI_Disable(unsigned long master);
void SMI_Pause(int master);
void SMI_Clear(int master);
void SMI_PowerOn(void);
void SMI_PowerOff(void);

int SMI_GetCount(void *addr);
int SMI_GetPortNo(int master);
int SMI_GetActiveCnt(int master);
int SMI_GetRequestCnt(int master);
int SMI_GetIdleCnt(int master);
int SMI_GetBeatCnt(int master);
int SMI_GetByteCnt(int master);

//common
void SMI_Comm_Init(void);
void SMI_SetCommBMCfg(unsigned long commonno, unsigned long portno, unsigned long desttype, unsigned long rwtype);
void SMI_Comm_Enable(unsigned long commonno);
void SMI_Comm_Disable(unsigned long commonno);
void SMI_Pause(int commonno);
void SMI_Comm_Clear(int commonno);

int SMI_Comm_GetPortNo(int commonno);
int SMI_Comm_GetActiveCnt(int commonno);
int SMI_Comm_GetRequestCnt(int commonno);
int SMI_Comm_GetIdleCnt(int commonno);
int SMI_Comm_GetBeatCnt(int commonno);
int SMI_Comm_GetByteCnt(int commonno);
#endif  /* !__MT6577_SMI_H__ */