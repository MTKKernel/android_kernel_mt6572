#ifndef _SG_COMMON_H
#define _SG_COMMON_H
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	(Software Power Gauge CT개발실 1PM 3PL)
	
	Copyright(c) 2010 LG전자
	 
	Date		Name		Version		Description
	----------	------		----------	-----------------------------------------------------------
	2011.03.10	kensin		 v1.1		Convert adc to Voltage for current estimation
	2010.10.22	kensin		 v1.05		Calibration for P350, C550
	2010.10.20  EJJ			 v1.04		Switching Deviation Protection code added
	2010.10.08	kensin		 v1.03		Android porting and calibration
	2010.07.17	kensin		 v1.02		Charger on battery swap code added
	2010.06.24	EJJ		 	 v1.01		KU380 calibration
	2010.03.31  kensin		 v1.00		Modify
	2009.12.16	kensin	 	 v0.90		Create
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*===================================================================================================
    이파일은 CT개발실 1PM3PL이 사용하기 위해 만든 파일입니다. 
    문제가 생길 경우 Custlge_common.h 에서 FEATURE_POWERAPI_SG를 주석처리해 주시기 바랍니다. 
===================================================================================================*/

//#define FEATURE_POWERAPI_SG_NV_USE
#include <linux/kernel.h>

//===== constants for hid device =====//
#define SPG_MODULE_PATH 	"/system/lib/modules/sg_common.ko"
#define SPG_DEV_NAME 		"lg-spg"
#define SPG_DEV_PATH 		"/dev/lg-spg"
#define SPG_DEV_MAJOR		257
#define SPG_DEV_MINOR		1

#define DEBUG_MSG(f, a...)   printk(f, ## a)
//static int spg_test(void);
#define FEATURE_POWERAPI_SG_NV_USE

//##########################################################################################################################################################
//structure
//##########################################################################################################################################################
typedef struct	
{
/*	float					fSocView;//SPG test structure
	float					fSocMeas;
	float					fIcv;
	float					fCurrent;
	float					fAdcAvg;
	float					fVoltage;
	float					fTemp1;
	unsigned long int		lTimeTick[50];
	unsigned long int		lLastTick;
	unsigned long int		lTemp1;
	char					sLogMsg[50];*/
	int 					nAdcValue[50];
}SG_Info;

//##########################################################################################################################################################
//SG function define
//##########################################################################################################################################################

void	SG_ICV_Init(void);
void 	SG_ICV_cb(void);
void 	SG_AdcLog(void);
void	SG_AdcLog_Init(void);
void 	SG_AdcLog_cb(unsigned long data);
//void 	SG_AdcLog_cb(void);

void	SG_ChargerOnBatterySwap(int nAdc, bool bCobsOn);
void 	SG_AdoptSDP(kal_bool bCurrCharger);	
long	SG_SocMeasure(void);
//long 	SG_GetSoc(void);


#ifdef FEATURE_POWERAPI_SG_ANDROID_TEST //EJJ_TEST 101020 PowerAPI RPC SPG item
int		SG_GetIcv(void);
int		SG_GetAdc(void);
int		SG_GetCurrent(void);
BOOL 	SG_GetVoltage(unsigned int *pVoltage);

#endif
void	SPG_FileLog(long time, char* msg);

#if defined(TARGET_S4)
//##########################################################################################################################################################
//SG parameter define 
//##########################################################################################################################################################
//온도보정 전에 SG_THERMAL_EFFECT define을 열어 놓으면 안됨
#define		SG_THERMAL_EFFECT

//battery 용량에 의해 달라지는 값
#define		ICV_CURRENT					250 //부팅 시 평균 소모전류 240mA
#define		BATTERY_CAPABILTY				1850  //1500mAh
#define		LOW_BATTERY_ADC				3399 //1%
#define		ICV_DELAY_TIME					HZ/10 //0초
#define		NV_SAVE_SOC_INTERVAL			1//1000 = 1% 주기로 NV writing
#ifdef 		SG_THERMAL_EFFECT
#define		THERMAL_GRADIENT				-13
#define		THERMAL_INTERCEPT				170
#define		THERMAL_0DEGREE				0
#endif

#define		PARAM_SCALE					1000
#define		CPD_1000						3
#define		BATTERY_1000mA					1000
//#define		CURRENT_PER_DROP				(CPD_1000*(BATTERY_CAPABILTY/BATTERY_1000mA))
#define		CURRENT_PER_DROP				CPD_1000
#define     CHARGING_CURRENT_PER_DROP       6
//#define		LOW_BATTERY_DEFENCE		//ADC 값이 일정 이하일 경우 SOC를 10% under로 내림
#define		LOW_BATTERY_SOC				1*PARAM_SCALE
#define		HOUR_TO_SEC					3600//1h = 3600s
#define		REAL_BATTERY_CAPABILITY		(BATTERY_CAPABILTY*97/100)//4.16V 를 100%로 잡는 경우
#define		BATTERY_CAP_TO_SEC				(REAL_BATTERY_CAPABILITY*HOUR_TO_SEC) //mAsec
#define		ADC_READ_INTERVAL  			HZ*2//100ms interval
#define		ADC_AVG_COUNT 					5
#define		ADC_AVG_QUEUE_LENGTH			5
#define		END_OF_ADC_LIST				-1111
//#define		SEC_PER_TICK					0.000030517578
#define		SOC_VIEW_MAX					101500
#define		SOC_MEAS_MAX					101500
#define		SOC_ONE_PERCENT				1*PARAM_SCALE
#define		ICV_LOG_COUNT 					5//10개
#define		ICV_TIMER_INTERVAL 			HZ/10//100ms
#define		SG_CALIBRATING					-1
#define		SLEEP_CURRENT					3//5mA
#define		VOLTAGE_STEP					3//5mV

#define		COMPENSATION_VOLTAGE_TA			25 //-
#define		COMPENSATION_VOLTAGE_USB		30  //-
#define		COMPENSATION_VOLTAGE_NORMAL		-25 //+
#define		CALL_SLEEP_CURRENT				50//70mA


//##########################################################################################################################################################
//SG global variable
//##########################################################################################################################################################

//##########################################################################################################################################################
//V2S, ICV, Charger on ICV table
//##########################################################################################################################################################
typedef enum
{
	SG_CHARGING_UNKNOWN,
	SG_CHARGING_NONE,
	SG_CHARGING_NA_TA,
	SG_CHARGING_LG_TA,
	SG_CHARGING_TA_1A,
	SG_CHARGING_INVALID_CHG,
	SG_CHARGING_USB,
	SG_CHARGING_FACTORY
}spg_muic_chg_st;

typedef enum
{
	SOC_LOW	=	0,
	SOC_ICV,
	SOC_CHR, //charger on ICV 
	SOC_USB, //USB on ICV
	SOC_HIGH
}spg_chg_st;

typedef enum
{
	ADC_LOW	=	0,
	ADC_ICV,
	ADC_CHR, //charger on ICV 
	ADC_USB, //USB on ICV
	ADC_HIGH
}spg_chg_st_adc;

typedef enum
{
	SLEEP_LOW = 0,
	SLEEP_DISCHARGING, //no charger
	SLEEP_CHANGE,//charger
	SLEEP_CHARGING, //charger, charger
	SLEEP_CANCEL,
	SLEEP_HIGH
}spg_long_term_st;

typedef enum
{
	BETWEEN_MAX = 0,
	BETWEEN_MAX_SEC,
	BETWEEN_SEC_THR,
	BETWEEN_THR_FTH,
	BETWEEN_FTH_FIF,
	BETWEEN_FIF_SIX,
	BETWEEN_SIX_SEV,
	BETWEEN_SEV_CUT
}spg_state;

#define 	SOC_MAX_ICV 				101500
#define 	SOC_SEC_ICV 				82*PARAM_SCALE
#define    	SOC_THR_ICV				45*PARAM_SCALE
#define		SOC_FTH_ICV				20*PARAM_SCALE
#define		SOC_FIF_ICV				11*PARAM_SCALE
#define		SOC_SIX_ICV				6*PARAM_SCALE
#define		SOC_SEV_ICV				2*PARAM_SCALE
#define		SOC_CUT_ICV				0
	
#define 	ADC_MAX_ICV				4250
#define		ADC_SEC_ICV				4130
#define 	ADC_THR_ICV				3850
#define		ADC_FTH_ICV				3700
#define		ADC_FIF_ICV				3643
#define		ADC_SIX_ICV				3600
#define		ADC_SEV_ICV				3400
#define		ADC_CUT_ICV				3399

#define 	SOC_MAX_CHR 				100500
#define 	SOC_SEC_CHR 				92*PARAM_SCALE
#define    	SOC_THR_CHR				56*PARAM_SCALE
#define		SOC_FTH_CHR				38*PARAM_SCALE
#define		SOC_FIF_CHR				13*PARAM_SCALE
#define		SOC_SIX_CHR				5*PARAM_SCALE
#define		SOC_SEV_CHR				2*PARAM_SCALE
#define		SOC_CUT_CHR				0

#define		ADC_MAX_CHR				4330
#define		ADC_SEC_CHR				4295
#define 	ADC_THR_CHR				4050
#define		ADC_FTH_CHR				3960
#define		ADC_FIF_CHR				3890
#define		ADC_SIX_CHR				3820
#define		ADC_SEV_CHR				3520
#define		ADC_CUT_CHR				3399

#define 	SOC_MAX_USB 				SOC_MAX_CHR
#define 	SOC_SEC_USB 				SOC_SEC_CHR
#define    	SOC_THR_USB				SOC_THR_CHR
#define		SOC_FTH_USB				SOC_FTH_CHR
#define		SOC_FIF_USB				SOC_FIF_CHR
#define		SOC_SIX_USB				SOC_SIX_CHR
#define		SOC_SEV_USB				SOC_SEV_CHR
#define		SOC_CUT_USB				SOC_CUT_CHR

#define		ADC_MAX_USB				4320
#define		ADC_SEC_USB				4220
#define 	ADC_THR_USB				3970
#define		ADC_FTH_USB				3900
#define		ADC_FIF_USB				3840
#define		ADC_SIX_USB				3740
#define		ADC_SEV_USB				3520
#define		ADC_CUT_USB				3399

#define 	SOC_MAX_SCV 				SOC_MAX_ICV
#define 	SOC_SEC_SCV 				SOC_SEC_ICV
#define    	SOC_THR_SCV				SOC_THR_ICV
#define		SOC_FTH_SCV				SOC_FTH_ICV
#define		SOC_FIF_SCV				SOC_FIF_ICV
#define		SOC_SIX_SCV				SOC_SIX_ICV
#define		SOC_SEV_SCV				SOC_SEV_ICV
#define		SOC_CUT_SCV				SOC_CUT_ICV

/*
#define 	ADC_MAX_SCV				ADC_MAX_ICV
#define		ADC_SEC_SCV				ADC_SEC_ICV
#define 	ADC_THR_SCV				ADC_THR_ICV
#define		ADC_FTH_SCV				ADC_FTH_ICV
#define		ADC_FIF_SCV				ADC_FIF_ICV
#define		ADC_SIX_SCV				ADC_SIX_ICV
#define		ADC_SEV_SCV				ADC_SEV_ICV
#define		ADC_CUT_SCV				ADC_CUT_ICV
*/

#define 	ADC_MAX_SCV				(ADC_MAX_ICV+((ICV_CURRENT)/CURRENT_PER_DROP))
#define		ADC_SEC_SCV				(ADC_SEC_ICV+((ICV_CURRENT)/CURRENT_PER_DROP))
#define 	ADC_THR_SCV				(ADC_THR_ICV+((ICV_CURRENT)/CURRENT_PER_DROP))
#define		ADC_FTH_SCV				(ADC_FTH_ICV+((ICV_CURRENT)/CURRENT_PER_DROP))
#define		ADC_FIF_SCV				(ADC_FIF_ICV+((ICV_CURRENT)/CURRENT_PER_DROP))
#define		ADC_SIX_SCV				(ADC_SIX_ICV+((ICV_CURRENT)/CURRENT_PER_DROP))
#define		ADC_SEV_SCV				(ADC_SEV_ICV+((ICV_CURRENT)/CURRENT_PER_DROP))
#define		ADC_CUT_SCV				(ADC_CUT_ICV+((ICV_CURRENT)/CURRENT_PER_DROP))


//Switching Deviation Protection
#define 	SDP_LOWER_CUTOFF		 	4
#define 	SDP_UPPER_CUTOFF 			96
#define		SDP_MARGINE 				10
#define		SDP_LOWER_BLOCK    		SDP_MARGINE*2
#define		SDP_UPPER_BLOCK 			100-SDP_LOWER_BLOCK
//Valid ADC Filter 
#define		ADC_LOWER_LIMIT			ADC_CUT_ICV - 200
#define		ADC_UPPER_LIMIT			ADC_MAX_CHR + 200
#define		isValid(nAdc)				((nAdc<ADC_LOWER_LIMIT)? 0:((nAdc>ADC_UPPER_LIMIT)?0:1))
#define		ADC_SWING_MARGIN			(ADC_UPPER_LIMIT-ADC_LOWER_LIMIT)/17

//##########################################################################################################################################################
//V2S, ICV, Charger on ICV macro
//##########################################################################################################################################################

#define 	GetSOC_B1_ICV(ICV)		SOC_MAX_ICV-(((ADC_MAX_ICV-ICV)*(SOC_MAX_ICV-SOC_SEC_ICV))/(ADC_MAX_ICV-ADC_SEC_ICV))
#define 	GetSOC_B2_ICV(ICV) 	SOC_SEC_ICV-(((ADC_SEC_ICV-ICV)*(SOC_SEC_ICV-SOC_THR_ICV))/(ADC_SEC_ICV-ADC_THR_ICV))
#define 	GetSOC_B3_ICV(ICV) 	SOC_THR_ICV-(((ADC_THR_ICV-ICV)*(SOC_THR_ICV-SOC_FTH_ICV))/(ADC_THR_ICV-ADC_FTH_ICV))
#define 	GetSOC_B4_ICV(ICV) 	SOC_FTH_ICV-(((ADC_FTH_ICV-ICV)*(SOC_FTH_ICV-SOC_FIF_ICV))/(ADC_FTH_ICV-ADC_FIF_ICV))
#define 	GetSOC_B5_ICV(ICV) 	SOC_FIF_ICV-(((ADC_FIF_ICV-ICV)*(SOC_FIF_ICV-SOC_SIX_ICV))/(ADC_FIF_ICV-ADC_SIX_ICV))
#define 	GetSOC_B6_ICV(ICV) 	SOC_SIX_ICV-(((ADC_SIX_ICV-ICV)*(SOC_SIX_ICV-SOC_SEV_ICV))/(ADC_SIX_ICV-ADC_SEV_ICV))
#define 	GetSOC_B7_ICV(ICV) 	SOC_SEV_ICV-(((ADC_SEV_ICV-ICV)*(SOC_SEV_ICV-SOC_CUT_ICV))/(ADC_SEV_ICV-ADC_CUT_ICV))

#define		GetSOC_B1_CHR(CHR)		SOC_MAX_CHR-(((ADC_MAX_CHR-CHR)*(SOC_MAX_CHR-SOC_SEC_CHR))/(ADC_MAX_CHR-ADC_SEC_CHR))
#define 	GetSOC_B2_CHR(CHR) 	SOC_SEC_CHR-(((ADC_SEC_CHR-CHR)*(SOC_SEC_CHR-SOC_THR_CHR))/(ADC_SEC_CHR-ADC_THR_CHR))
#define 	GetSOC_B3_CHR(CHR) 	SOC_THR_CHR-(((ADC_THR_CHR-CHR)*(SOC_THR_CHR-SOC_FTH_CHR))/(ADC_THR_CHR-ADC_FTH_CHR))
#define 	GetSOC_B4_CHR(CHR) 	SOC_FTH_CHR-(((ADC_FTH_CHR-CHR)*(SOC_FTH_CHR-SOC_FIF_CHR))/(ADC_FTH_CHR-ADC_FIF_CHR))
#define 	GetSOC_B5_CHR(CHR) 	SOC_FIF_CHR-(((ADC_FIF_CHR-CHR)*(SOC_FIF_CHR-SOC_SIX_CHR))/(ADC_FIF_CHR-ADC_SIX_CHR))
#define 	GetSOC_B6_CHR(CHR) 	SOC_SIX_CHR-(((ADC_SIX_CHR-CHR)*(SOC_SIX_CHR-SOC_SEV_CHR))/(ADC_SIX_CHR-ADC_SEV_CHR))
#define 	GetSOC_B7_CHR(CHR) 	SOC_SEV_CHR-(((ADC_SEV_CHR-CHR)*(SOC_SEV_CHR-SOC_CUT_CHR))/(ADC_SEV_CHR-ADC_CUT_CHR))

#define		GetSOC_B1_USB(USB)		SOC_MAX_USB-(((ADC_MAX_USB-USB)*(SOC_MAX_USB-SOC_SEC_USB))/(ADC_MAX_USB-ADC_SEC_USB))
#define 	GetSOC_B2_USB(USB) 	SOC_SEC_USB-(((ADC_SEC_USB-USB)*(SOC_SEC_USB-SOC_THR_USB))/(ADC_SEC_USB-ADC_THR_USB))
#define 	GetSOC_B3_USB(USB) 	SOC_THR_USB-(((ADC_THR_USB-USB)*(SOC_THR_USB-SOC_FTH_USB))/(ADC_THR_USB-ADC_FTH_USB))
#define 	GetSOC_B4_USB(USB) 	SOC_FTH_USB-(((ADC_FTH_USB-USB)*(SOC_FTH_USB-SOC_FIF_USB))/(ADC_FTH_USB-ADC_FIF_USB))
#define 	GetSOC_B5_USB(USB) 	SOC_FIF_USB-(((ADC_FIF_USB-USB)*(SOC_FIF_USB-SOC_SIX_USB))/(ADC_FIF_USB-ADC_SIX_USB))
#define 	GetSOC_B6_USB(USB) 	SOC_SIX_USB-(((ADC_SIX_USB-USB)*(SOC_SIX_USB-SOC_SEV_USB))/(ADC_SIX_USB-ADC_SEV_USB))
#define 	GetSOC_B7_USB(USB) 	SOC_SEV_USB-(((ADC_SEV_USB-USB)*(SOC_SEV_USB-SOC_CUT_USB))/(ADC_SEV_USB-ADC_CUT_USB))

#define 	GetADC_B1_SCV(SOC)		ADC_MAX_SCV-(((SOC_MAX_SCV-SOC)*(ADC_MAX_SCV-ADC_SEC_SCV))/(SOC_MAX_SCV-SOC_SEC_SCV))
#define 	GetADC_B2_SCV(SOC) 	ADC_SEC_SCV-(((SOC_SEC_SCV-SOC)*(ADC_SEC_SCV-ADC_THR_SCV))/(SOC_SEC_SCV-SOC_THR_SCV))
#define 	GetADC_B3_SCV(SOC) 	ADC_THR_SCV-(((SOC_THR_SCV-SOC)*(ADC_THR_SCV-ADC_FTH_SCV))/(SOC_THR_SCV-SOC_FTH_SCV))
#define 	GetADC_B4_SCV(SOC) 	ADC_FTH_SCV-(((SOC_FTH_SCV-SOC)*(ADC_FTH_SCV-ADC_FIF_SCV))/(SOC_FTH_SCV-SOC_FIF_SCV))
#define 	GetADC_B5_SCV(SOC) 	ADC_FIF_SCV-(((SOC_FIF_SCV-SOC)*(ADC_FIF_SCV-ADC_SIX_SCV))/(SOC_FIF_SCV-SOC_SIX_SCV))
#define 	GetADC_B6_SCV(SOC) 	ADC_SIX_SCV-(((SOC_SIX_SCV-SOC)*(ADC_SIX_SCV-ADC_SEV_SCV))/(SOC_SIX_SCV-SOC_SEV_SCV))
#define 	GetADC_B7_SCV(SOC) 	ADC_SEV_SCV-(((SOC_SEV_SCV-SOC)*(ADC_SEV_SCV-ADC_CUT_SCV))/(SOC_SEV_SCV-SOC_CUT_SCV))

#define GetSOC_ICV(ICV_VAL,RESULT) \
	if	    ( ICV_VAL <= 0)				 	RESULT = -1;\
	else if ( ICV_VAL >= ADC_MAX_ICV )		RESULT = SOC_MAX_ICV;\
	else if ( ICV_VAL >= ADC_SEC_ICV ) 		RESULT = GetSOC_B1_ICV(ICV_VAL);\
	else if ( ICV_VAL >= ADC_THR_ICV ) 		RESULT = GetSOC_B2_ICV(ICV_VAL);\
	else if ( ICV_VAL >= ADC_FTH_ICV ) 		RESULT = GetSOC_B3_ICV(ICV_VAL);\
	else if ( ICV_VAL >= ADC_FIF_ICV ) 		RESULT = GetSOC_B4_ICV(ICV_VAL);\
	else if ( ICV_VAL >= ADC_SIX_ICV ) 		RESULT = GetSOC_B5_ICV(ICV_VAL);\
	else if ( ICV_VAL >= ADC_SEV_ICV ) 		RESULT = GetSOC_B6_ICV(ICV_VAL);\
	else if ( ICV_VAL >= ADC_CUT_ICV ) 		RESULT = GetSOC_B7_ICV(ICV_VAL);\
	else 									RESULT = 0;

#define GetSOC_CHR(CHR_VAL,RESULT) \
	if	    ( CHR_VAL <= 0)			 		RESULT = -1;\
	else if ( CHR_VAL >= ADC_MAX_CHR )		RESULT = SOC_MAX_CHR;\
	else if ( CHR_VAL >= ADC_SEC_CHR ) 		RESULT = GetSOC_B1_CHR(CHR_VAL);\
	else if ( CHR_VAL >= ADC_THR_CHR ) 		RESULT = GetSOC_B2_CHR(CHR_VAL);\
	else if ( CHR_VAL >= ADC_FTH_CHR ) 		RESULT = GetSOC_B3_CHR(CHR_VAL);\
	else if ( CHR_VAL >= ADC_FIF_CHR ) 		RESULT = GetSOC_B4_CHR(CHR_VAL);\
	else if ( CHR_VAL >= ADC_SIX_CHR )		RESULT = GetSOC_B5_CHR(CHR_VAL);\
	else if ( CHR_VAL >= ADC_SEV_CHR ) 		RESULT = GetSOC_B6_CHR(CHR_VAL);\
	else if ( CHR_VAL >= ADC_CUT_CHR ) 		RESULT = GetSOC_B7_CHR(CHR_VAL);\
	else 									RESULT = 0;

#define GetSOC_USB(USB_VAL,RESULT) \
	if	    (USB_VAL <= 0)			 		RESULT = -1;\
	else if (USB_VAL >= ADC_MAX_USB)		RESULT = SOC_MAX_USB;\
	else if (USB_VAL >= ADC_SEC_USB) 		RESULT = GetSOC_B1_USB(USB_VAL);\
	else if (USB_VAL >= ADC_THR_USB) 		RESULT = GetSOC_B2_USB(USB_VAL);\
	else if (USB_VAL >= ADC_FTH_USB) 		RESULT = GetSOC_B3_USB(USB_VAL);\
	else if (USB_VAL >= ADC_FIF_USB) 		RESULT = GetSOC_B4_USB(USB_VAL);\
	else if (USB_VAL >= ADC_SIX_USB)		RESULT = GetSOC_B5_USB(USB_VAL);\
	else if (USB_VAL >= ADC_SEV_USB) 		RESULT = GetSOC_B6_USB(USB_VAL);\
	else if (USB_VAL >= ADC_CUT_USB) 		RESULT = GetSOC_B7_USB(USB_VAL);\
	else 									RESULT = 0;

//voltage vector scaling
#define GetADC_SCV(SOC_VAL,RESULT) \
	if		(SOC_VAL  < SOC_CUT_SCV)		RESULT = -1;\
	else if (SOC_VAL >= SOC_MAX_SCV)		RESULT = ADC_MAX_SCV;\
	else if (SOC_VAL >= SOC_SEC_SCV)		RESULT = GetADC_B1_SCV(SOC_VAL);\
	else if (SOC_VAL >= SOC_THR_SCV)		RESULT = GetADC_B2_SCV(SOC_VAL);\
	else if (SOC_VAL >= SOC_FTH_SCV)		RESULT = GetADC_B3_SCV(SOC_VAL);\
	else if (SOC_VAL >= SOC_FIF_SCV)		RESULT = GetADC_B4_SCV(SOC_VAL);\
	else if (SOC_VAL >= SOC_SIX_SCV)		RESULT = GetADC_B5_SCV(SOC_VAL);\
	else if (SOC_VAL >= SOC_SEV_SCV)		RESULT = GetADC_B6_SCV(SOC_VAL);\
	else if (SOC_VAL >= SOC_CUT_SCV)		RESULT = GetADC_B7_SCV(SOC_VAL);\
	else									RESULT = 0;

//debuging
#define 	GetADC_B1_ICV(SOC)  ADC_MAX_ICV-(((SOC_MAX_ICV-SOC)*(ADC_MAX_ICV-ADC_SEC_ICV))/(SOC_MAX_ICV-SOC_SEC_ICV))
#define 	GetADC_B2_ICV(SOC)  ADC_SEC_ICV-(((SOC_SEC_ICV-SOC)*(ADC_SEC_ICV-ADC_THR_ICV))/(SOC_SEC_ICV-SOC_THR_ICV))
#define 	GetADC_B3_ICV(SOC)  ADC_THR_ICV-(((SOC_THR_ICV-SOC)*(ADC_THR_ICV-ADC_FTH_ICV))/(SOC_THR_ICV-SOC_FTH_ICV))
#define 	GetADC_B4_ICV(SOC)  ADC_FTH_ICV-(((SOC_FTH_ICV-SOC)*(ADC_FTH_ICV-ADC_FIF_ICV))/(SOC_FTH_ICV-SOC_FIF_ICV))
#define 	GetADC_B5_ICV(SOC)  ADC_FIF_ICV-(((SOC_FIF_ICV-SOC)*(ADC_FIF_ICV-ADC_SIX_ICV))/(SOC_FIF_ICV-SOC_SIX_ICV))
#define 	GetADC_B6_ICV(SOC)  ADC_SIX_ICV-(((SOC_SIX_ICV-SOC)*(ADC_SIX_ICV-ADC_SEV_ICV))/(SOC_SIX_ICV-SOC_SEV_ICV))
#define 	GetADC_B7_ICV(SOC)  ADC_SEV_ICV-(((SOC_SEV_ICV-SOC)*(ADC_SEV_ICV-ADC_CUT_ICV))/(SOC_SEV_ICV-SOC_CUT_ICV))

#define 	GetADC_B1_USB(SOC)  ADC_MAX_USB-(((SOC_MAX_USB-SOC)*(ADC_MAX_USB-ADC_SEC_USB))/(SOC_MAX_USB-SOC_SEC_USB))
#define 	GetADC_B2_USB(SOC)  ADC_SEC_USB-(((SOC_SEC_USB-SOC)*(ADC_SEC_USB-ADC_THR_USB))/(SOC_SEC_USB-SOC_THR_USB))
#define 	GetADC_B3_USB(SOC)  ADC_THR_USB-(((SOC_THR_USB-SOC)*(ADC_THR_USB-ADC_FTH_USB))/(SOC_THR_USB-SOC_FTH_USB))
#define 	GetADC_B4_USB(SOC)  ADC_FTH_USB-(((SOC_FTH_USB-SOC)*(ADC_FTH_USB-ADC_FIF_USB))/(SOC_FTH_USB-SOC_FIF_USB))
#define 	GetADC_B5_USB(SOC)  ADC_FIF_USB-(((SOC_FIF_USB-SOC)*(ADC_FIF_USB-ADC_SIX_USB))/(SOC_FIF_USB-SOC_SIX_USB))
#define 	GetADC_B6_USB(SOC)  ADC_SIX_USB-(((SOC_SIX_USB-SOC)*(ADC_SIX_USB-ADC_SEV_USB))/(SOC_SIX_USB-SOC_SEV_USB))
#define 	GetADC_B7_USB(SOC)  ADC_SEV_USB-(((SOC_SEV_USB-SOC)*(ADC_SEV_USB-ADC_CUT_USB))/(SOC_SEV_USB-SOC_CUT_USB))

#define 	GetADC_B1_CHR(SOC)  ADC_MAX_CHR-(((SOC_MAX_CHR-SOC)*(ADC_MAX_CHR-ADC_SEC_CHR))/(SOC_MAX_CHR-SOC_SEC_CHR))
#define 	GetADC_B2_CHR(SOC)  ADC_SEC_CHR-(((SOC_SEC_CHR-SOC)*(ADC_SEC_CHR-ADC_THR_CHR))/(SOC_SEC_CHR-SOC_THR_CHR))
#define 	GetADC_B3_CHR(SOC)  ADC_THR_CHR-(((SOC_THR_CHR-SOC)*(ADC_THR_CHR-ADC_FTH_CHR))/(SOC_THR_CHR-SOC_FTH_CHR))
#define 	GetADC_B4_CHR(SOC)  ADC_FTH_CHR-(((SOC_FTH_CHR-SOC)*(ADC_FTH_CHR-ADC_FIF_CHR))/(SOC_FTH_CHR-SOC_FIF_CHR))
#define 	GetADC_B5_CHR(SOC)  ADC_FIF_CHR-(((SOC_FIF_CHR-SOC)*(ADC_FIF_CHR-ADC_SIX_CHR))/(SOC_FIF_CHR-SOC_SIX_CHR))
#define 	GetADC_B6_CHR(SOC)  ADC_SIX_CHR-(((SOC_SIX_CHR-SOC)*(ADC_SIX_CHR-ADC_SEV_CHR))/(SOC_SIX_CHR-SOC_SEV_CHR))
#define 	GetADC_B7_CHR(SOC)  ADC_SEV_CHR-(((SOC_SEV_CHR-SOC)*(ADC_SEV_CHR-ADC_CUT_CHR))/(SOC_SEV_CHR-SOC_CUT_CHR))



#define GetADC_ICV(SOC_VAL,RESULT) \
	if		(SOC_VAL  < SOC_CUT_ICV)		RESULT = -1;\
	else if (SOC_VAL >= SOC_MAX_ICV)		RESULT = ADC_MAX_ICV;\
	else if (SOC_VAL >= SOC_SEC_ICV)		RESULT = GetADC_B1_ICV(SOC_VAL);\
	else if (SOC_VAL >= SOC_THR_ICV)		RESULT = GetADC_B2_ICV(SOC_VAL);\
	else if (SOC_VAL >= SOC_FTH_ICV)		RESULT = GetADC_B3_ICV(SOC_VAL);\
	else if (SOC_VAL >= SOC_FIF_ICV)		RESULT = GetADC_B4_ICV(SOC_VAL);\
	else if (SOC_VAL >= SOC_SIX_ICV)		RESULT = GetADC_B5_ICV(SOC_VAL);\
	else if (SOC_VAL >= SOC_SEV_ICV)		RESULT = GetADC_B6_ICV(SOC_VAL);\
	else if (SOC_VAL >= SOC_CUT_ICV)		RESULT = GetADC_B7_ICV(SOC_VAL);\
	else									RESULT = 0;

#define GetADC_USB(SOC_VAL,RESULT) \
	if		(SOC_VAL  < SOC_CUT_USB)		RESULT = -1;\
	else if (SOC_VAL >= SOC_MAX_USB)		RESULT = ADC_MAX_USB;\
	else if (SOC_VAL >= SOC_SEC_USB)		RESULT = GetADC_B1_USB(SOC_VAL);\
	else if (SOC_VAL >= SOC_THR_USB)		RESULT = GetADC_B2_USB(SOC_VAL);\
	else if (SOC_VAL >= SOC_FTH_USB)		RESULT = GetADC_B3_USB(SOC_VAL);\
	else if (SOC_VAL >= SOC_FIF_USB)		RESULT = GetADC_B4_USB(SOC_VAL);\
	else if (SOC_VAL >= SOC_SIX_USB)		RESULT = GetADC_B5_USB(SOC_VAL);\
	else if (SOC_VAL >= SOC_SEV_USB)		RESULT = GetADC_B6_USB(SOC_VAL);\
	else if (SOC_VAL >= SOC_CUT_USB)		RESULT = GetADC_B7_USB(SOC_VAL);\
	else									RESULT = 0;

#define GetADC_CHR(SOC_VAL,RESULT) \
	if		(SOC_VAL  < SOC_CUT_CHR)		RESULT = -1;\
	else if (SOC_VAL >= SOC_MAX_CHR)		RESULT = ADC_MAX_CHR;\
	else if (SOC_VAL >= SOC_SEC_CHR)		RESULT = GetADC_B1_CHR(SOC_VAL);\
	else if (SOC_VAL >= SOC_THR_CHR)		RESULT = GetADC_B2_CHR(SOC_VAL);\
	else if (SOC_VAL >= SOC_FTH_CHR)		RESULT = GetADC_B3_CHR(SOC_VAL);\
	else if (SOC_VAL >= SOC_FIF_CHR)		RESULT = GetADC_B4_CHR(SOC_VAL);\
	else if (SOC_VAL >= SOC_SIX_CHR)		RESULT = GetADC_B5_CHR(SOC_VAL);\
	else if (SOC_VAL >= SOC_SEV_CHR)		RESULT = GetADC_B6_CHR(SOC_VAL);\
	else if (SOC_VAL >= SOC_CUT_CHR)		RESULT = GetADC_B7_CHR(SOC_VAL);\
	else									RESULT = 0;

#define GetADC(which,SOC_VAL,RESULT)	\
	switch(which)\
	{	\
		case SOC_ICV : GetADC_ICV(SOC_VAL,RESULT) 		break; \
		case SOC_CHR : GetADC_CHR(SOC_VAL,RESULT) 		break; \
		case SOC_USB : GetADC_USB(SOC_VAL,RESULT) 		break; \
    }

#define GetSOC(which,xCV_VAL,RESULT)	\
	switch(which)\
	{	\
		case SOC_ICV : GetSOC_ICV(xCV_VAL,RESULT) 		break; \
		case SOC_CHR : GetSOC_CHR(xCV_VAL,RESULT) 		break; \
		case SOC_USB : GetSOC_USB(xCV_VAL,RESULT) 		break; \
	}	

//yunhang.heo
#define BETWEEN_ADC_ICV(ADC_VAL,RESULT) \
	if	    ( ADC_VAL <= 0)				 	RESULT = -1;\
	else if ( ADC_VAL >= ADC_MAX_ICV )		RESULT = BETWEEN_MAX;\
	else if ( ADC_VAL >= ADC_SEC_ICV ) 		RESULT = BETWEEN_MAX_SEC;\
	else if ( ADC_VAL >= ADC_THR_ICV ) 		RESULT = BETWEEN_SEC_THR;\
	else if ( ADC_VAL >= ADC_FTH_ICV ) 		RESULT = BETWEEN_THR_FTH;\
	else if ( ADC_VAL >= ADC_FIF_ICV ) 		RESULT = BETWEEN_FTH_FIF;\
	else if ( ADC_VAL >= ADC_SIX_ICV ) 		RESULT = BETWEEN_FIF_SIX;\
	else if ( ADC_VAL >= ADC_SEV_ICV ) 		RESULT = BETWEEN_SIX_SEV;\
	else if ( ADC_VAL >= ADC_CUT_ICV ) 		RESULT = BETWEEN_SEV_CUT;\
	else 									RESULT = -2;

#define BETWEEN_ADC_CHR(ADC_VAL,RESULT) \
	if	    ( ADC_VAL <= 0)				 	RESULT = -1;\
	else if ( ADC_VAL >= ADC_MAX_CHR )		RESULT = BETWEEN_MAX;\
	else if ( ADC_VAL >= ADC_SEC_CHR ) 		RESULT = BETWEEN_MAX_SEC;\
	else if ( ADC_VAL >= ADC_THR_CHR ) 		RESULT = BETWEEN_SEC_THR;\
	else if ( ADC_VAL >= ADC_FTH_CHR ) 		RESULT = BETWEEN_THR_FTH;\
	else if ( ADC_VAL >= ADC_FIF_CHR ) 		RESULT = BETWEEN_FTH_FIF;\
	else if ( ADC_VAL >= ADC_SIX_CHR ) 		RESULT = BETWEEN_FIF_SIX;\
	else if ( ADC_VAL >= ADC_SEV_CHR ) 		RESULT = BETWEEN_SIX_SEV;\
	else if ( ADC_VAL >= ADC_CUT_CHR ) 		RESULT = BETWEEN_SEV_CUT;\
	else 									RESULT = -2;

#define BETWEEN_ADC_USB(ADC_VAL,RESULT) \
	if	    ( ADC_VAL <= 0)				 	RESULT = -1;\
	else if ( ADC_VAL >= ADC_MAX_USB )		RESULT = BETWEEN_MAX;\
	else if ( ADC_VAL >= ADC_SEC_USB ) 		RESULT = BETWEEN_MAX_SEC;\
	else if ( ADC_VAL >= ADC_THR_USB ) 		RESULT = BETWEEN_SEC_THR;\
	else if ( ADC_VAL >= ADC_FTH_USB ) 		RESULT = BETWEEN_THR_FTH;\
	else if ( ADC_VAL >= ADC_FIF_USB ) 		RESULT = BETWEEN_FTH_FIF;\
	else if ( ADC_VAL >= ADC_SIX_USB ) 		RESULT = BETWEEN_FIF_SIX;\
	else if ( ADC_VAL >= ADC_SEV_USB ) 		RESULT = BETWEEN_SIX_SEV;\
	else if ( ADC_VAL >= ADC_CUT_USB ) 		RESULT = BETWEEN_SEV_CUT;\
	else 									RESULT = -2;

#define GET_BETWEEN_ADC(which,xCV_VAL,RESULT)	\
	switch(which)\
	{	\
		case ADC_ICV : BETWEEN_ADC_ICV(xCV_VAL,RESULT) 		break; \
		case ADC_CHR : BETWEEN_ADC_CHR(xCV_VAL,RESULT) 		break; \
		case ADC_USB : BETWEEN_ADC_USB(xCV_VAL,RESULT) 		break; \
	}


//Switching Deviation Protection
#define GetSDP_WT_NORMAL_CHRON(psoc, nsoc,result)\
	if	  (nsoc >= 	SOC_MAX_CHR)	{result = 1; 	DEBUG_MSG("..........NORMAL_CHRON SOC_MAX_CHR %ld\n",SOC_MAX_CHR,0,0);}\
	else if(nsoc >=	SOC_SEC_CHR)	{result = 1;    	DEBUG_MSG("..........NORMAL_CHRON SOC_SEC_CHR %ld\n",SOC_SEC_CHR,0,0);}\
	else if(nsoc >=	SOC_THR_CHR)	{result = 2;	    DEBUG_MSG("..........NORMAL_CHRON SOC_THR_CHR %ld\n",SOC_THR_CHR,0,0);}\
	else if(nsoc >=	SOC_FTH_CHR)	{result = 3;	    DEBUG_MSG("..........NORMAL_CHRON SOC_FTH_CHR %ld\n",SOC_FTH_CHR,0,0);}\
	else if(nsoc >=	SOC_FIF_CHR)	{result = 3;	    DEBUG_MSG("..........NORMAL_CHRON SOC_FIF_CHR %ld\n",SOC_FIF_CHR,0,0);}\
	else if(nsoc >=	SOC_SIX_CHR)	{result = 3;	    DEBUG_MSG("..........NORMAL_CHRON SOC_SIX_CHR %ld\n",SOC_SIX_CHR,0,0);}\
	else if(nsoc >=	SOC_SEV_CHR)	{result = 1;	    DEBUG_MSG("..........NORMAL_CHRON SOC_SEV_CHR %ld\n",SOC_SEV_CHR,0,0);}\
	else if(nsoc >=	SOC_CUT_CHR)	{result = 1;	    DEBUG_MSG("..........NORMAL_CHRON SOC_CUT_CHR %ld\n",SOC_CUT_CHR,0,0);}
	
#define GetSDP_WT_NORMAL_CHROFF(psoc, nsoc,result)\
	if	  (nsoc >= 	SOC_MAX_ICV)	{result = 1; 	DEBUG_MSG("..........NORMAL_CHROFF SOC_MAX_ICV %ld\n",SOC_MAX_ICV,0,0);}\
	else if(nsoc >=	SOC_SEC_ICV)	{result = 1;    	DEBUG_MSG("..........NORMAL_CHROFF SOC_SEC_ICV %ld\n",SOC_SEC_ICV,0,0);}\
	else if(nsoc >=	SOC_THR_ICV)	{result = 2;	    DEBUG_MSG("..........NORMAL_CHROFF SOC_THR_ICV %ld\n",SOC_THR_ICV,0,0);}\
	else if(nsoc >=	SOC_FTH_ICV)	{result = 1;	    DEBUG_MSG("..........NORMAL_CHROFF SOC_FTH_ICV %ld\n",SOC_FTH_ICV,0,0);}\
	else if(nsoc >=	SOC_FIF_ICV)	{result = 1;	    DEBUG_MSG("..........NORMAL_CHROFF SOC_FIF_ICV %ld\n",SOC_FIF_ICV,0,0);}\
	else if(nsoc >=	SOC_SIX_ICV)	{result = 1;	    DEBUG_MSG("..........NORMAL_CHROFF SOC_SIX_ICV %ld\n",SOC_SIX_ICV,0,0);}\
	else if(nsoc >=	SOC_SEV_ICV)	{result = 1;	    DEBUG_MSG("..........NORMAL_CHROFF SOC_SEV_ICV %ld\n",SOC_SEV_ICV,0,0);}\
	else if(nsoc >=	SOC_CUT_ICV)	{result = 1;	    DEBUG_MSG("..........NORMAL_CHROFF SOC_CUT_ICV %ld\n",SOC_CUT_ICV,0,0);}

#define GetSDP_WT_CHRSW_CHRON(psoc, nsoc,result)\
	if	 (nsoc >= 	SOC_MAX_CHR)	{result = 1; 	DEBUG_MSG("..........CHRSW_CHRON MAX_CHR %ld\n" ,SOC_MAX_CHR,0,0);}\
	else if(nsoc >=	SOC_SEC_CHR)	{result = 1;	    DEBUG_MSG("..........CHRSW_CHRON SEC_CHR %ld\n" ,SOC_SEC_CHR,0,0);}\
	else if(nsoc >=	SOC_THR_CHR)	{result = 2;	    DEBUG_MSG("..........CHRSW_CHRON THR_CHR %ld\n" ,SOC_THR_CHR,0,0);}\
	else if(nsoc >=	SOC_FTH_CHR)	{result = 2;	    DEBUG_MSG("..........CHRSW_CHRON FTH_CHR %ld\n" ,SOC_FTH_CHR,0,0);}\
	else if(nsoc >=	SOC_FIF_CHR)	{result = 2;	    DEBUG_MSG("..........CHRSW_CHRON FIF_CHR %ld\n" ,SOC_FIF_CHR,0,0);}\
	else if(nsoc >=	SOC_SIX_CHR)	{result = 2;	    DEBUG_MSG("..........CHRSW_CHRON SIX_CHR %ld\n" ,SOC_SIX_CHR,0,0);}\
	else if(nsoc >=	SOC_SEV_CHR)	{result = 2;	    DEBUG_MSG("..........CHRSW_CHRON SEV_CHR %ld\n" ,SOC_SEV_CHR,0,0);}\
	else if(nsoc >=	SOC_CUT_CHR)	{result = 1;	    DEBUG_MSG("..........CHRSW_CHRON CUT_CHR %ld\n" ,SOC_CUT_CHR,0,0);}

#define GetSDP_WT_CHRSW_CHROFF(psoc, nsoc,result)\
	if	  (nsoc >= 	SOC_MAX_ICV)	{result = 1; 	DEBUG_MSG("..........CHRSW_CHROFF SOC_MAX_ICV %ld\n",SOC_MAX_ICV,0,0);}\
	else if(nsoc >=	SOC_SEC_ICV)	{result = 2;	    DEBUG_MSG("..........CHRSW_CHROFF SOC_SEC_ICV %ld\n",SOC_SEC_ICV,0,0);}\
	else if(nsoc >=	SOC_THR_ICV)	{result = 2;	    DEBUG_MSG("..........CHRSW_CHROFF SOC_THR_ICV %ld\n",SOC_THR_ICV,0,0);}\
	else if(nsoc >=	SOC_FTH_ICV)	{result = 2;	    DEBUG_MSG("..........CHRSW_CHROFF SOC_FTH_ICV %ld\n",SOC_FTH_ICV,0,0);}\
	else if(nsoc >=	SOC_FIF_ICV)	{result = 2;	    DEBUG_MSG("..........CHRSW_CHROFF SOC_FIF_ICV %ld\n",SOC_FIF_ICV,0,0);}\
	else if(nsoc >=	SOC_SIX_ICV)	{result = 1;	    DEBUG_MSG("..........CHRSW_CHROFF SOC_SIX_ICV %ld\n",SOC_SIX_ICV,0,0);}\
	else if(nsoc >=	SOC_SEV_ICV)	{result = 1;	    DEBUG_MSG("..........CHRSW_CHROFF SOC_SEV_ICV %ld\n",SOC_SEV_ICV,0,0);}\
	else if(nsoc >=	SOC_CUT_ICV)	{result = 1;	    DEBUG_MSG("..........CHRSW_CHROFF SOC_CUT_ICV %ld\n",SOC_CUT_ICV,0,0);}

#define GetSDP_WT_CHRSW(P_SOC,N_SOC,bCHR,RESULT)\
	if(bCHR){\
		GetSDP_WT_CHRSW_CHRON(P_SOC,N_SOC,RESULT)\
	}else{\
		GetSDP_WT_CHRSW_CHROFF(P_SOC,N_SOC,RESULT)\
	}\

#define GetSDP_WT_NORMAL(P_SOC,N_SOC,bCHR,RESULT)\
	if(bCHR){\
		GetSDP_WT_NORMAL_CHRON(P_SOC,N_SOC,RESULT)\
	}else{\
		GetSDP_WT_NORMAL_CHROFF(P_SOC,N_SOC,RESULT)\
	}\
	
#define GetSDP_WT(PREV_SOC, NOW_SOC, NOW_CHR, PREV_CHR, RESULT)\
	if	(NOW_CHR != PREV_CHR){\
		GetSDP_WT_CHRSW(PREV_SOC,NOW_SOC,NOW_CHR,RESULT)\
	}else{\
		GetSDP_WT_NORMAL(PREV_SOC,NOW_SOC,NOW_CHR,RESULT)\
	}\
	
#define		ABS(x)					((x) < 0 ? -(x) : (x))
#define		VOL_STEP(x, y)			((x) >= (y) ? (x) : (x + VOLTAGE_STEP))
#define 	MAX(x,y) 				((x)>(y) ? (x):(y))

#else

//##########################################################################################################################################################
//SG parameter define 
//##########################################################################################################################################################
//온도보정 전에 SG_THERMAL_EFFECT define을 열어 놓으면 안됨
#define		SG_THERMAL_EFFECT

//battery 용량에 의해 달라지는 값
#define		ICV_CURRENT					250 //부팅 시 평균 소모전류 240mA
#define		BATTERY_CAPABILTY				1500  //1500mAh
#define		LOW_BATTERY_ADC				3300 //1%
#define		ICV_DELAY_TIME					HZ/10 //0초
#define		NV_SAVE_SOC_INTERVAL			1// 1% 주기로 NV writing
#ifdef 		SG_THERMAL_EFFECT
#define		THERMAL_GRADIENT				-10
#define		THERMAL_INTERCEPT				220
#define		THERMAL_0DEGREE					0
#endif

#define		PARAM_SCALE					1000
#define		CPD_1000						3
#define		BATTERY_1000mA					1000
//#define		CURRENT_PER_DROP				(CPD_1000*(BATTERY_CAPABILTY/BATTERY_1000mA))
#define		CURRENT_PER_DROP				CPD_1000
#define     CHARGING_CURRENT_PER_DROP       6
//#define		LOW_BATTERY_DEFENCE		//ADC 값이 일정 이하일 경우 SOC를 10% under로 내림
#define		LOW_BATTERY_SOC				1*PARAM_SCALE
#define		HOUR_TO_SEC					3600//1h = 3600s
#define		REAL_BATTERY_CAPABILITY		(BATTERY_CAPABILTY*97/100)//4.16V 를 100%로 잡는 경우
#define		BATTERY_CAP_TO_SEC				(REAL_BATTERY_CAPABILITY*HOUR_TO_SEC) //mAsec
#define		ADC_READ_INTERVAL  			HZ*2//100ms interval
#define		ADC_AVG_COUNT 					5
#define		ADC_AVG_QUEUE_LENGTH			5
#define		END_OF_ADC_LIST				-1111
//#define		SEC_PER_TICK					0.000030517578
#define		SOC_VIEW_MAX					101500
#define		SOC_MEAS_MAX					101500
#define		SOC_ONE_PERCENT				1*PARAM_SCALE
#define		ICV_LOG_COUNT 					5//10개
#define		ICV_TIMER_INTERVAL 			HZ/10//100ms
#define		SG_CALIBRATING					-1
#define		SLEEP_CURRENT					3//5mA
#define		VOLTAGE_STEP					3//5mV

#define		COMPENSATION_VOLTAGE_TA			25
#define		COMPENSATION_VOLTAGE_USB		30
#define		COMPENSATION_VOLTAGE_NORMAL		-25
#define		CALL_SLEEP_CURRENT				70//70mA

//##########################################################################################################################################################
//SG global variable
//##########################################################################################################################################################

//##########################################################################################################################################################
//V2S, ICV, Charger on ICV table
//##########################################################################################################################################################
typedef enum
{
	SG_CHARGING_UNKNOWN,
	SG_CHARGING_NONE,
	SG_CHARGING_NA_TA,
	SG_CHARGING_LG_TA,
	SG_CHARGING_TA_1A,
	SG_CHARGING_INVALID_CHG,
	SG_CHARGING_USB,
	SG_CHARGING_FACTORY
}spg_muic_chg_st;

typedef enum
{
	SOC_LOW	=	0,
	SOC_ICV,
	SOC_CHR, //charger on ICV 
	SOC_USB, //USB on ICV
	SOC_HIGH
}spg_chg_st;

typedef enum
{
	ADC_LOW	=	0,
	ADC_ICV,
	ADC_CHR, //charger on ICV 
	ADC_USB, //USB on ICV
	ADC_HIGH
}spg_chg_st_adc;

typedef enum
{
	SLEEP_LOW = 0,
	SLEEP_DISCHARGING, //no charger
	SLEEP_CHANGE,//charger
	SLEEP_CHARGING, //charger, charger
	SLEEP_CANCEL,
	SLEEP_HIGH
}spg_long_term_st;

typedef enum
{
	BETWEEN_MAX = 0,
	BETWEEN_MAX_SEC,
	BETWEEN_SEC_THR,
	BETWEEN_THR_FTH,
	BETWEEN_FTH_FIF,
	BETWEEN_FIF_SIX,
	BETWEEN_SIX_SEV,
	BETWEEN_SEV_CUT
}spg_state;

#define 	SOC_MAX_ICV 				101500
#define 	SOC_SEC_ICV 				78*PARAM_SCALE
#define    	SOC_THR_ICV				58*PARAM_SCALE
#define		SOC_FTH_ICV				40*PARAM_SCALE
#define		SOC_FIF_ICV				19*PARAM_SCALE
#define		SOC_SIX_ICV				5*PARAM_SCALE
#define		SOC_SEV_ICV				2*PARAM_SCALE
#define		SOC_CUT_ICV				0
	
#define 	ADC_MAX_ICV				4118
#define		ADC_SEC_ICV				3886
#define 	ADC_THR_ICV				3760
#define		ADC_FTH_ICV				3700
#define		ADC_FIF_ICV				3659
#define		ADC_SIX_ICV				3562
#define		ADC_SEV_ICV				3373
#define		ADC_CUT_ICV				3300

#define 	SOC_MAX_CHR 				101500
#define 	SOC_SEC_CHR 				93*PARAM_SCALE
#define    	SOC_THR_CHR				65*PARAM_SCALE
#define		SOC_FTH_CHR				46*PARAM_SCALE
#define		SOC_FIF_CHR				10*PARAM_SCALE
#define		SOC_SIX_CHR				4*PARAM_SCALE
#define		SOC_SEV_CHR				2*PARAM_SCALE
#define		SOC_CUT_CHR				0

#define		ADC_MAX_CHR				4211
#define		ADC_SEC_CHR				4150
#define 	ADC_THR_CHR				3910
#define		ADC_FTH_CHR				3850
#define		ADC_FIF_CHR				3750
#define		ADC_SIX_CHR				3600
#define		ADC_SEV_CHR				3500
#define		ADC_CUT_CHR				3380

#define 	SOC_MAX_USB 				SOC_MAX_CHR
#define 	SOC_SEC_USB 				SOC_SEC_CHR
#define    	SOC_THR_USB				SOC_THR_CHR
#define		SOC_FTH_USB				SOC_FTH_CHR
#define		SOC_FIF_USB				SOC_FIF_CHR
#define		SOC_SIX_USB				SOC_SIX_CHR
#define		SOC_SEV_USB				SOC_SEV_CHR
#define		SOC_CUT_USB				SOC_CUT_CHR

#define		ADC_MAX_USB				4200
#define		ADC_SEC_USB				4120
#define 	ADC_THR_USB				3880
#define		ADC_FTH_USB				3800
#define		ADC_FIF_USB				3720
#define		ADC_SIX_USB				3570
#define		ADC_SEV_USB				3480
#define		ADC_CUT_USB				3350

#define 	SOC_MAX_SCV 				SOC_MAX_ICV
#define 	SOC_SEC_SCV 				SOC_SEC_ICV
#define    	SOC_THR_SCV				SOC_THR_ICV
#define		SOC_FTH_SCV				SOC_FTH_ICV
#define		SOC_FIF_SCV				SOC_FIF_ICV
#define		SOC_SIX_SCV				SOC_SIX_ICV
#define		SOC_SEV_SCV				SOC_SEV_ICV
#define		SOC_CUT_SCV				SOC_CUT_ICV

/*
#define 	ADC_MAX_SCV				ADC_MAX_ICV
#define		ADC_SEC_SCV				ADC_SEC_ICV
#define 	ADC_THR_SCV				ADC_THR_ICV
#define		ADC_FTH_SCV				ADC_FTH_ICV
#define		ADC_FIF_SCV				ADC_FIF_ICV
#define		ADC_SIX_SCV				ADC_SIX_ICV
#define		ADC_SEV_SCV				ADC_SEV_ICV
#define		ADC_CUT_SCV				ADC_CUT_ICV
*/

#define 	ADC_MAX_SCV				(ADC_MAX_ICV+((ICV_CURRENT)/CURRENT_PER_DROP))
#define		ADC_SEC_SCV				(ADC_SEC_ICV+((ICV_CURRENT)/CURRENT_PER_DROP))
#define 	ADC_THR_SCV				(ADC_THR_ICV+((ICV_CURRENT)/CURRENT_PER_DROP))
#define		ADC_FTH_SCV				(ADC_FTH_ICV+((ICV_CURRENT)/CURRENT_PER_DROP))
#define		ADC_FIF_SCV				(ADC_FIF_ICV+((ICV_CURRENT)/CURRENT_PER_DROP))
#define		ADC_SIX_SCV				(ADC_SIX_ICV+((ICV_CURRENT)/CURRENT_PER_DROP))
#define		ADC_SEV_SCV				(ADC_SEV_ICV+((ICV_CURRENT)/CURRENT_PER_DROP))
#define		ADC_CUT_SCV				(ADC_CUT_ICV+((ICV_CURRENT)/CURRENT_PER_DROP))


//Switching Deviation Protection
#define 	SDP_LOWER_CUTOFF		 	4
#define 	SDP_UPPER_CUTOFF 			96
#define		SDP_MARGINE 				10
#define		SDP_LOWER_BLOCK    		SDP_MARGINE*2
#define		SDP_UPPER_BLOCK 			100-SDP_LOWER_BLOCK
//Valid ADC Filter 
#define		ADC_LOWER_LIMIT			ADC_CUT_ICV - 200
#define		ADC_UPPER_LIMIT			ADC_MAX_CHR + 200
#define		isValid(nAdc)				((nAdc<ADC_LOWER_LIMIT)? 0:((nAdc>ADC_UPPER_LIMIT)?0:1))
#define		ADC_SWING_MARGIN			(ADC_UPPER_LIMIT-ADC_LOWER_LIMIT)/17

//##########################################################################################################################################################
//V2S, ICV, Charger on ICV macro
//##########################################################################################################################################################

#define 	GetSOC_B1_ICV(ICV)		SOC_MAX_ICV-(((ADC_MAX_ICV-ICV)*(SOC_MAX_ICV-SOC_SEC_ICV))/(ADC_MAX_ICV-ADC_SEC_ICV))
#define 	GetSOC_B2_ICV(ICV) 	SOC_SEC_ICV-(((ADC_SEC_ICV-ICV)*(SOC_SEC_ICV-SOC_THR_ICV))/(ADC_SEC_ICV-ADC_THR_ICV))
#define 	GetSOC_B3_ICV(ICV) 	SOC_THR_ICV-(((ADC_THR_ICV-ICV)*(SOC_THR_ICV-SOC_FTH_ICV))/(ADC_THR_ICV-ADC_FTH_ICV))
#define 	GetSOC_B4_ICV(ICV) 	SOC_FTH_ICV-(((ADC_FTH_ICV-ICV)*(SOC_FTH_ICV-SOC_FIF_ICV))/(ADC_FTH_ICV-ADC_FIF_ICV))
#define 	GetSOC_B5_ICV(ICV) 	SOC_FIF_ICV-(((ADC_FIF_ICV-ICV)*(SOC_FIF_ICV-SOC_SIX_ICV))/(ADC_FIF_ICV-ADC_SIX_ICV))
#define 	GetSOC_B6_ICV(ICV) 	SOC_SIX_ICV-(((ADC_SIX_ICV-ICV)*(SOC_SIX_ICV-SOC_SEV_ICV))/(ADC_SIX_ICV-ADC_SEV_ICV))
#define 	GetSOC_B7_ICV(ICV) 	SOC_SEV_ICV-(((ADC_SEV_ICV-ICV)*(SOC_SEV_ICV-SOC_CUT_ICV))/(ADC_SEV_ICV-ADC_CUT_ICV))

#define		GetSOC_B1_CHR(CHR)		SOC_MAX_CHR-(((ADC_MAX_CHR-CHR)*(SOC_MAX_CHR-SOC_SEC_CHR))/(ADC_MAX_CHR-ADC_SEC_CHR))
#define 	GetSOC_B2_CHR(CHR) 	SOC_SEC_CHR-(((ADC_SEC_CHR-CHR)*(SOC_SEC_CHR-SOC_THR_CHR))/(ADC_SEC_CHR-ADC_THR_CHR))
#define 	GetSOC_B3_CHR(CHR) 	SOC_THR_CHR-(((ADC_THR_CHR-CHR)*(SOC_THR_CHR-SOC_FTH_CHR))/(ADC_THR_CHR-ADC_FTH_CHR))
#define 	GetSOC_B4_CHR(CHR) 	SOC_FTH_CHR-(((ADC_FTH_CHR-CHR)*(SOC_FTH_CHR-SOC_FIF_CHR))/(ADC_FTH_CHR-ADC_FIF_CHR))
#define 	GetSOC_B5_CHR(CHR) 	SOC_FIF_CHR-(((ADC_FIF_CHR-CHR)*(SOC_FIF_CHR-SOC_SIX_CHR))/(ADC_FIF_CHR-ADC_SIX_CHR))
#define 	GetSOC_B6_CHR(CHR) 	SOC_SIX_CHR-(((ADC_SIX_CHR-CHR)*(SOC_SIX_CHR-SOC_SEV_CHR))/(ADC_SIX_CHR-ADC_SEV_CHR))
#define 	GetSOC_B7_CHR(CHR) 	SOC_SEV_CHR-(((ADC_SEV_CHR-CHR)*(SOC_SEV_CHR-SOC_CUT_CHR))/(ADC_SEV_CHR-ADC_CUT_CHR))

#define		GetSOC_B1_USB(USB)		SOC_MAX_USB-(((ADC_MAX_USB-USB)*(SOC_MAX_USB-SOC_SEC_USB))/(ADC_MAX_USB-ADC_SEC_USB))
#define 	GetSOC_B2_USB(USB) 	SOC_SEC_USB-(((ADC_SEC_USB-USB)*(SOC_SEC_USB-SOC_THR_USB))/(ADC_SEC_USB-ADC_THR_USB))
#define 	GetSOC_B3_USB(USB) 	SOC_THR_USB-(((ADC_THR_USB-USB)*(SOC_THR_USB-SOC_FTH_USB))/(ADC_THR_USB-ADC_FTH_USB))
#define 	GetSOC_B4_USB(USB) 	SOC_FTH_USB-(((ADC_FTH_USB-USB)*(SOC_FTH_USB-SOC_FIF_USB))/(ADC_FTH_USB-ADC_FIF_USB))
#define 	GetSOC_B5_USB(USB) 	SOC_FIF_USB-(((ADC_FIF_USB-USB)*(SOC_FIF_USB-SOC_SIX_USB))/(ADC_FIF_USB-ADC_SIX_USB))
#define 	GetSOC_B6_USB(USB) 	SOC_SIX_USB-(((ADC_SIX_USB-USB)*(SOC_SIX_USB-SOC_SEV_USB))/(ADC_SIX_USB-ADC_SEV_USB))
#define 	GetSOC_B7_USB(USB) 	SOC_SEV_USB-(((ADC_SEV_USB-USB)*(SOC_SEV_USB-SOC_CUT_USB))/(ADC_SEV_USB-ADC_CUT_USB))

#define 	GetADC_B1_SCV(SOC)		ADC_MAX_SCV-(((SOC_MAX_SCV-SOC)*(ADC_MAX_SCV-ADC_SEC_SCV))/(SOC_MAX_SCV-SOC_SEC_SCV))
#define 	GetADC_B2_SCV(SOC) 	ADC_SEC_SCV-(((SOC_SEC_SCV-SOC)*(ADC_SEC_SCV-ADC_THR_SCV))/(SOC_SEC_SCV-SOC_THR_SCV))
#define 	GetADC_B3_SCV(SOC) 	ADC_THR_SCV-(((SOC_THR_SCV-SOC)*(ADC_THR_SCV-ADC_FTH_SCV))/(SOC_THR_SCV-SOC_FTH_SCV))
#define 	GetADC_B4_SCV(SOC) 	ADC_FTH_SCV-(((SOC_FTH_SCV-SOC)*(ADC_FTH_SCV-ADC_FIF_SCV))/(SOC_FTH_SCV-SOC_FIF_SCV))
#define 	GetADC_B5_SCV(SOC) 	ADC_FIF_SCV-(((SOC_FIF_SCV-SOC)*(ADC_FIF_SCV-ADC_SIX_SCV))/(SOC_FIF_SCV-SOC_SIX_SCV))
#define 	GetADC_B6_SCV(SOC) 	ADC_SIX_SCV-(((SOC_SIX_SCV-SOC)*(ADC_SIX_SCV-ADC_SEV_SCV))/(SOC_SIX_SCV-SOC_SEV_SCV))
#define 	GetADC_B7_SCV(SOC) 	ADC_SEV_SCV-(((SOC_SEV_SCV-SOC)*(ADC_SEV_SCV-ADC_CUT_SCV))/(SOC_SEV_SCV-SOC_CUT_SCV))

#define GetSOC_ICV(ICV_VAL,RESULT) \
	if	    ( ICV_VAL <= 0)				 	RESULT = -1;\
	else if ( ICV_VAL >= ADC_MAX_ICV )		RESULT = SOC_MAX_ICV;\
	else if ( ICV_VAL >= ADC_SEC_ICV ) 		RESULT = GetSOC_B1_ICV(ICV_VAL);\
	else if ( ICV_VAL >= ADC_THR_ICV ) 		RESULT = GetSOC_B2_ICV(ICV_VAL);\
	else if ( ICV_VAL >= ADC_FTH_ICV ) 		RESULT = GetSOC_B3_ICV(ICV_VAL);\
	else if ( ICV_VAL >= ADC_FIF_ICV ) 		RESULT = GetSOC_B4_ICV(ICV_VAL);\
	else if ( ICV_VAL >= ADC_SIX_ICV ) 		RESULT = GetSOC_B5_ICV(ICV_VAL);\
	else if ( ICV_VAL >= ADC_SEV_ICV ) 		RESULT = GetSOC_B6_ICV(ICV_VAL);\
	else if ( ICV_VAL >= ADC_CUT_ICV ) 		RESULT = GetSOC_B7_ICV(ICV_VAL);\
	else 									RESULT = 0;

#define GetSOC_CHR(CHR_VAL,RESULT) \
	if	    ( CHR_VAL <= 0)			 		RESULT = -1;\
	else if ( CHR_VAL >= ADC_MAX_CHR )		RESULT = SOC_MAX_CHR;\
	else if ( CHR_VAL >= ADC_SEC_CHR ) 		RESULT = GetSOC_B1_CHR(CHR_VAL);\
	else if ( CHR_VAL >= ADC_THR_CHR ) 		RESULT = GetSOC_B2_CHR(CHR_VAL);\
	else if ( CHR_VAL >= ADC_FTH_CHR ) 		RESULT = GetSOC_B3_CHR(CHR_VAL);\
	else if ( CHR_VAL >= ADC_FIF_CHR ) 		RESULT = GetSOC_B4_CHR(CHR_VAL);\
	else if ( CHR_VAL >= ADC_SIX_CHR )		RESULT = GetSOC_B5_CHR(CHR_VAL);\
	else if ( CHR_VAL >= ADC_SEV_CHR ) 		RESULT = GetSOC_B6_CHR(CHR_VAL);\
	else if ( CHR_VAL >= ADC_CUT_CHR ) 		RESULT = GetSOC_B7_CHR(CHR_VAL);\
	else 									RESULT = 0;

#define GetSOC_USB(USB_VAL,RESULT) \
	if	    (USB_VAL <= 0)			 		RESULT = -1;\
	else if (USB_VAL >= ADC_MAX_USB)		RESULT = SOC_MAX_USB;\
	else if (USB_VAL >= ADC_SEC_USB) 		RESULT = GetSOC_B1_USB(USB_VAL);\
	else if (USB_VAL >= ADC_THR_USB) 		RESULT = GetSOC_B2_USB(USB_VAL);\
	else if (USB_VAL >= ADC_FTH_USB) 		RESULT = GetSOC_B3_USB(USB_VAL);\
	else if (USB_VAL >= ADC_FIF_USB) 		RESULT = GetSOC_B4_USB(USB_VAL);\
	else if (USB_VAL >= ADC_SIX_USB)		RESULT = GetSOC_B5_USB(USB_VAL);\
	else if (USB_VAL >= ADC_SEV_USB) 		RESULT = GetSOC_B6_USB(USB_VAL);\
	else if (USB_VAL >= ADC_CUT_USB) 		RESULT = GetSOC_B7_USB(USB_VAL);\
	else 									RESULT = 0;

//voltage vector scaling
#define GetADC_SCV(SOC_VAL,RESULT) \
	if		(SOC_VAL  < SOC_CUT_SCV)		RESULT = -1;\
	else if (SOC_VAL >= SOC_MAX_SCV)		RESULT = ADC_MAX_SCV;\
	else if (SOC_VAL >= SOC_SEC_SCV)		RESULT = GetADC_B1_SCV(SOC_VAL);\
	else if (SOC_VAL >= SOC_THR_SCV)		RESULT = GetADC_B2_SCV(SOC_VAL);\
	else if (SOC_VAL >= SOC_FTH_SCV)		RESULT = GetADC_B3_SCV(SOC_VAL);\
	else if (SOC_VAL >= SOC_FIF_SCV)		RESULT = GetADC_B4_SCV(SOC_VAL);\
	else if (SOC_VAL >= SOC_SIX_SCV)		RESULT = GetADC_B5_SCV(SOC_VAL);\
	else if (SOC_VAL >= SOC_SEV_SCV)		RESULT = GetADC_B6_SCV(SOC_VAL);\
	else if (SOC_VAL >= SOC_CUT_SCV)		RESULT = GetADC_B7_SCV(SOC_VAL);\
	else									RESULT = 0;

//debuging
#define 	GetADC_B1_ICV(SOC)  ADC_MAX_ICV-(((SOC_MAX_ICV-SOC)*(ADC_MAX_ICV-ADC_SEC_ICV))/(SOC_MAX_ICV-SOC_SEC_ICV))
#define 	GetADC_B2_ICV(SOC)  ADC_SEC_ICV-(((SOC_SEC_ICV-SOC)*(ADC_SEC_ICV-ADC_THR_ICV))/(SOC_SEC_ICV-SOC_THR_ICV))
#define 	GetADC_B3_ICV(SOC)  ADC_THR_ICV-(((SOC_THR_ICV-SOC)*(ADC_THR_ICV-ADC_FTH_ICV))/(SOC_THR_ICV-SOC_FTH_ICV))
#define 	GetADC_B4_ICV(SOC)  ADC_FTH_ICV-(((SOC_FTH_ICV-SOC)*(ADC_FTH_ICV-ADC_FIF_ICV))/(SOC_FTH_ICV-SOC_FIF_ICV))
#define 	GetADC_B5_ICV(SOC)  ADC_FIF_ICV-(((SOC_FIF_ICV-SOC)*(ADC_FIF_ICV-ADC_SIX_ICV))/(SOC_FIF_ICV-SOC_SIX_ICV))
#define 	GetADC_B6_ICV(SOC)  ADC_SIX_ICV-(((SOC_SIX_ICV-SOC)*(ADC_SIX_ICV-ADC_SEV_ICV))/(SOC_SIX_ICV-SOC_SEV_ICV))
#define 	GetADC_B7_ICV(SOC)  ADC_SEV_ICV-(((SOC_SEV_ICV-SOC)*(ADC_SEV_ICV-ADC_CUT_ICV))/(SOC_SEV_ICV-SOC_CUT_ICV))

#define 	GetADC_B1_USB(SOC)  ADC_MAX_USB-(((SOC_MAX_USB-SOC)*(ADC_MAX_USB-ADC_SEC_USB))/(SOC_MAX_USB-SOC_SEC_USB))
#define 	GetADC_B2_USB(SOC)  ADC_SEC_USB-(((SOC_SEC_USB-SOC)*(ADC_SEC_USB-ADC_THR_USB))/(SOC_SEC_USB-SOC_THR_USB))
#define 	GetADC_B3_USB(SOC)  ADC_THR_USB-(((SOC_THR_USB-SOC)*(ADC_THR_USB-ADC_FTH_USB))/(SOC_THR_USB-SOC_FTH_USB))
#define 	GetADC_B4_USB(SOC)  ADC_FTH_USB-(((SOC_FTH_USB-SOC)*(ADC_FTH_USB-ADC_FIF_USB))/(SOC_FTH_USB-SOC_FIF_USB))
#define 	GetADC_B5_USB(SOC)  ADC_FIF_USB-(((SOC_FIF_USB-SOC)*(ADC_FIF_USB-ADC_SIX_USB))/(SOC_FIF_USB-SOC_SIX_USB))
#define 	GetADC_B6_USB(SOC)  ADC_SIX_USB-(((SOC_SIX_USB-SOC)*(ADC_SIX_USB-ADC_SEV_USB))/(SOC_SIX_USB-SOC_SEV_USB))
#define 	GetADC_B7_USB(SOC)  ADC_SEV_USB-(((SOC_SEV_USB-SOC)*(ADC_SEV_USB-ADC_CUT_USB))/(SOC_SEV_USB-SOC_CUT_USB))

#define 	GetADC_B1_CHR(SOC)  ADC_MAX_CHR-(((SOC_MAX_CHR-SOC)*(ADC_MAX_CHR-ADC_SEC_CHR))/(SOC_MAX_CHR-SOC_SEC_CHR))
#define 	GetADC_B2_CHR(SOC)  ADC_SEC_CHR-(((SOC_SEC_CHR-SOC)*(ADC_SEC_CHR-ADC_THR_CHR))/(SOC_SEC_CHR-SOC_THR_CHR))
#define 	GetADC_B3_CHR(SOC)  ADC_THR_CHR-(((SOC_THR_CHR-SOC)*(ADC_THR_CHR-ADC_FTH_CHR))/(SOC_THR_CHR-SOC_FTH_CHR))
#define 	GetADC_B4_CHR(SOC)  ADC_FTH_CHR-(((SOC_FTH_CHR-SOC)*(ADC_FTH_CHR-ADC_FIF_CHR))/(SOC_FTH_CHR-SOC_FIF_CHR))
#define 	GetADC_B5_CHR(SOC)  ADC_FIF_CHR-(((SOC_FIF_CHR-SOC)*(ADC_FIF_CHR-ADC_SIX_CHR))/(SOC_FIF_CHR-SOC_SIX_CHR))
#define 	GetADC_B6_CHR(SOC)  ADC_SIX_CHR-(((SOC_SIX_CHR-SOC)*(ADC_SIX_CHR-ADC_SEV_CHR))/(SOC_SIX_CHR-SOC_SEV_CHR))
#define 	GetADC_B7_CHR(SOC)  ADC_SEV_CHR-(((SOC_SEV_CHR-SOC)*(ADC_SEV_CHR-ADC_CUT_CHR))/(SOC_SEV_CHR-SOC_CUT_CHR))



#define GetADC_ICV(SOC_VAL,RESULT) \
	if		(SOC_VAL  < SOC_CUT_ICV)		RESULT = -1;\
	else if (SOC_VAL >= SOC_MAX_ICV)		RESULT = ADC_MAX_ICV;\
	else if (SOC_VAL >= SOC_SEC_ICV)		RESULT = GetADC_B1_ICV(SOC_VAL);\
	else if (SOC_VAL >= SOC_THR_ICV)		RESULT = GetADC_B2_ICV(SOC_VAL);\
	else if (SOC_VAL >= SOC_FTH_ICV)		RESULT = GetADC_B3_ICV(SOC_VAL);\
	else if (SOC_VAL >= SOC_FIF_ICV)		RESULT = GetADC_B4_ICV(SOC_VAL);\
	else if (SOC_VAL >= SOC_SIX_ICV)		RESULT = GetADC_B5_ICV(SOC_VAL);\
	else if (SOC_VAL >= SOC_SEV_ICV)		RESULT = GetADC_B6_ICV(SOC_VAL);\
	else if (SOC_VAL >= SOC_CUT_ICV)		RESULT = GetADC_B7_ICV(SOC_VAL);\
	else									RESULT = 0;

#define GetADC_USB(SOC_VAL,RESULT) \
	if		(SOC_VAL  < SOC_CUT_USB)		RESULT = -1;\
	else if (SOC_VAL >= SOC_MAX_USB)		RESULT = ADC_MAX_USB;\
	else if (SOC_VAL >= SOC_SEC_USB)		RESULT = GetADC_B1_USB(SOC_VAL);\
	else if (SOC_VAL >= SOC_THR_USB)		RESULT = GetADC_B2_USB(SOC_VAL);\
	else if (SOC_VAL >= SOC_FTH_USB)		RESULT = GetADC_B3_USB(SOC_VAL);\
	else if (SOC_VAL >= SOC_FIF_USB)		RESULT = GetADC_B4_USB(SOC_VAL);\
	else if (SOC_VAL >= SOC_SIX_USB)		RESULT = GetADC_B5_USB(SOC_VAL);\
	else if (SOC_VAL >= SOC_SEV_USB)		RESULT = GetADC_B6_USB(SOC_VAL);\
	else if (SOC_VAL >= SOC_CUT_USB)		RESULT = GetADC_B7_USB(SOC_VAL);\
	else									RESULT = 0;

#define GetADC_CHR(SOC_VAL,RESULT) \
	if		(SOC_VAL  < SOC_CUT_CHR)		RESULT = -1;\
	else if (SOC_VAL >= SOC_MAX_CHR)		RESULT = ADC_MAX_CHR;\
	else if (SOC_VAL >= SOC_SEC_CHR)		RESULT = GetADC_B1_CHR(SOC_VAL);\
	else if (SOC_VAL >= SOC_THR_CHR)		RESULT = GetADC_B2_CHR(SOC_VAL);\
	else if (SOC_VAL >= SOC_FTH_CHR)		RESULT = GetADC_B3_CHR(SOC_VAL);\
	else if (SOC_VAL >= SOC_FIF_CHR)		RESULT = GetADC_B4_CHR(SOC_VAL);\
	else if (SOC_VAL >= SOC_SIX_CHR)		RESULT = GetADC_B5_CHR(SOC_VAL);\
	else if (SOC_VAL >= SOC_SEV_CHR)		RESULT = GetADC_B6_CHR(SOC_VAL);\
	else if (SOC_VAL >= SOC_CUT_CHR)		RESULT = GetADC_B7_CHR(SOC_VAL);\
	else									RESULT = 0;

#define GetADC(which,SOC_VAL,RESULT)	\
	switch(which)\
	{	\
		case SOC_ICV : GetADC_ICV(SOC_VAL,RESULT) 		break; \
		case SOC_CHR : GetADC_CHR(SOC_VAL,RESULT) 		break; \
		case SOC_USB : GetADC_USB(SOC_VAL,RESULT) 		break; \
    }

#define GetSOC(which,xCV_VAL,RESULT)	\
	switch(which)\
	{	\
		case SOC_ICV : GetSOC_ICV(xCV_VAL,RESULT) 		break; \
		case SOC_CHR : GetSOC_CHR(xCV_VAL,RESULT) 		break; \
		case SOC_USB : GetSOC_USB(xCV_VAL,RESULT) 		break; \
	}	

//yunhang.heo
#define BETWEEN_ADC_ICV(ADC_VAL,RESULT) \
	if	    ( ADC_VAL <= 0)				 	RESULT = -1;\
	else if ( ADC_VAL >= ADC_MAX_ICV )		RESULT = BETWEEN_MAX;\
	else if ( ADC_VAL >= ADC_SEC_ICV ) 		RESULT = BETWEEN_MAX_SEC;\
	else if ( ADC_VAL >= ADC_THR_ICV ) 		RESULT = BETWEEN_SEC_THR;\
	else if ( ADC_VAL >= ADC_FTH_ICV ) 		RESULT = BETWEEN_THR_FTH;\
	else if ( ADC_VAL >= ADC_FIF_ICV ) 		RESULT = BETWEEN_FTH_FIF;\
	else if ( ADC_VAL >= ADC_SIX_ICV ) 		RESULT = BETWEEN_FIF_SIX;\
	else if ( ADC_VAL >= ADC_SEV_ICV ) 		RESULT = BETWEEN_SIX_SEV;\
	else if ( ADC_VAL >= ADC_CUT_ICV ) 		RESULT = BETWEEN_SEV_CUT;\
	else 									RESULT = -2;

#define BETWEEN_ADC_CHR(ADC_VAL,RESULT) \
	if	    ( ADC_VAL <= 0)				 	RESULT = -1;\
	else if ( ADC_VAL >= ADC_MAX_CHR )		RESULT = BETWEEN_MAX;\
	else if ( ADC_VAL >= ADC_SEC_CHR ) 		RESULT = BETWEEN_MAX_SEC;\
	else if ( ADC_VAL >= ADC_THR_CHR ) 		RESULT = BETWEEN_SEC_THR;\
	else if ( ADC_VAL >= ADC_FTH_CHR ) 		RESULT = BETWEEN_THR_FTH;\
	else if ( ADC_VAL >= ADC_FIF_CHR ) 		RESULT = BETWEEN_FTH_FIF;\
	else if ( ADC_VAL >= ADC_SIX_CHR ) 		RESULT = BETWEEN_FIF_SIX;\
	else if ( ADC_VAL >= ADC_SEV_CHR ) 		RESULT = BETWEEN_SIX_SEV;\
	else if ( ADC_VAL >= ADC_CUT_CHR ) 		RESULT = BETWEEN_SEV_CUT;\
	else 									RESULT = -2;

#define BETWEEN_ADC_USB(ADC_VAL,RESULT) \
	if	    ( ADC_VAL <= 0)				 	RESULT = -1;\
	else if ( ADC_VAL >= ADC_MAX_USB )		RESULT = BETWEEN_MAX;\
	else if ( ADC_VAL >= ADC_SEC_USB ) 		RESULT = BETWEEN_MAX_SEC;\
	else if ( ADC_VAL >= ADC_THR_USB ) 		RESULT = BETWEEN_SEC_THR;\
	else if ( ADC_VAL >= ADC_FTH_USB ) 		RESULT = BETWEEN_THR_FTH;\
	else if ( ADC_VAL >= ADC_FIF_USB ) 		RESULT = BETWEEN_FTH_FIF;\
	else if ( ADC_VAL >= ADC_SIX_USB ) 		RESULT = BETWEEN_FIF_SIX;\
	else if ( ADC_VAL >= ADC_SEV_USB ) 		RESULT = BETWEEN_SIX_SEV;\
	else if ( ADC_VAL >= ADC_CUT_USB ) 		RESULT = BETWEEN_SEV_CUT;\
	else 									RESULT = -2;

#define GET_BETWEEN_ADC(which,xCV_VAL,RESULT)	\
	switch(which)\
	{	\
		case ADC_ICV : BETWEEN_ADC_ICV(xCV_VAL,RESULT) 		break; \
		case ADC_CHR : BETWEEN_ADC_CHR(xCV_VAL,RESULT) 		break; \
		case ADC_USB : BETWEEN_ADC_USB(xCV_VAL,RESULT) 		break; \
	}


//Switching Deviation Protection
#define GetSDP_WT_NORMAL_CHRON(psoc, nsoc,result)\
	if	  (nsoc >= 	SOC_MAX_CHR)	{result = 1; 	DEBUG_MSG("..........NORMAL_CHRON SOC_MAX_CHR %ld\n",SOC_MAX_CHR,0,0);}\
	else if(nsoc >=	SOC_SEC_CHR)	{result = 1;    	DEBUG_MSG("..........NORMAL_CHRON SOC_SEC_CHR %ld\n",SOC_SEC_CHR,0,0);}\
	else if(nsoc >=	SOC_THR_CHR)	{result = 2;	    DEBUG_MSG("..........NORMAL_CHRON SOC_THR_CHR %ld\n",SOC_THR_CHR,0,0);}\
	else if(nsoc >=	SOC_FTH_CHR)	{result = 3;	    DEBUG_MSG("..........NORMAL_CHRON SOC_FTH_CHR %ld\n",SOC_FTH_CHR,0,0);}\
	else if(nsoc >=	SOC_FIF_CHR)	{result = 3;	    DEBUG_MSG("..........NORMAL_CHRON SOC_FIF_CHR %ld\n",SOC_FIF_CHR,0,0);}\
	else if(nsoc >=	SOC_SIX_CHR)	{result = 3;	    DEBUG_MSG("..........NORMAL_CHRON SOC_SIX_CHR %ld\n",SOC_SIX_CHR,0,0);}\
	else if(nsoc >=	SOC_SEV_CHR)	{result = 1;	    DEBUG_MSG("..........NORMAL_CHRON SOC_SEV_CHR %ld\n",SOC_SEV_CHR,0,0);}\
	else if(nsoc >=	SOC_CUT_CHR)	{result = 1;	    DEBUG_MSG("..........NORMAL_CHRON SOC_CUT_CHR %ld\n",SOC_CUT_CHR,0,0);}
	
#define GetSDP_WT_NORMAL_CHROFF(psoc, nsoc,result)\
	if	  (nsoc >= 	SOC_MAX_ICV)	{result = 1; 	DEBUG_MSG("..........NORMAL_CHROFF SOC_MAX_ICV %ld\n",SOC_MAX_ICV,0,0);}\
	else if(nsoc >=	SOC_SEC_ICV)	{result = 1;    	DEBUG_MSG("..........NORMAL_CHROFF SOC_SEC_ICV %ld\n",SOC_SEC_ICV,0,0);}\
	else if(nsoc >=	SOC_THR_ICV)	{result = 2;	    DEBUG_MSG("..........NORMAL_CHROFF SOC_THR_ICV %ld\n",SOC_THR_ICV,0,0);}\
	else if(nsoc >=	SOC_FTH_ICV)	{result = 1;	    DEBUG_MSG("..........NORMAL_CHROFF SOC_FTH_ICV %ld\n",SOC_FTH_ICV,0,0);}\
	else if(nsoc >=	SOC_FIF_ICV)	{result = 1;	    DEBUG_MSG("..........NORMAL_CHROFF SOC_FIF_ICV %ld\n",SOC_FIF_ICV,0,0);}\
	else if(nsoc >=	SOC_SIX_ICV)	{result = 1;	    DEBUG_MSG("..........NORMAL_CHROFF SOC_SIX_ICV %ld\n",SOC_SIX_ICV,0,0);}\
	else if(nsoc >=	SOC_SEV_ICV)	{result = 1;	    DEBUG_MSG("..........NORMAL_CHROFF SOC_SEV_ICV %ld\n",SOC_SEV_ICV,0,0);}\
	else if(nsoc >=	SOC_CUT_ICV)	{result = 1;	    DEBUG_MSG("..........NORMAL_CHROFF SOC_CUT_ICV %ld\n",SOC_CUT_ICV,0,0);}

#define GetSDP_WT_CHRSW_CHRON(psoc, nsoc,result)\
	if	 (nsoc >= 	SOC_MAX_CHR)	{result = 1; 	DEBUG_MSG("..........CHRSW_CHRON MAX_CHR %ld\n" ,SOC_MAX_CHR,0,0);}\
	else if(nsoc >=	SOC_SEC_CHR)	{result = 1;	    DEBUG_MSG("..........CHRSW_CHRON SEC_CHR %ld\n" ,SOC_SEC_CHR,0,0);}\
	else if(nsoc >=	SOC_THR_CHR)	{result = 2;	    DEBUG_MSG("..........CHRSW_CHRON THR_CHR %ld\n" ,SOC_THR_CHR,0,0);}\
	else if(nsoc >=	SOC_FTH_CHR)	{result = 2;	    DEBUG_MSG("..........CHRSW_CHRON FTH_CHR %ld\n" ,SOC_FTH_CHR,0,0);}\
	else if(nsoc >=	SOC_FIF_CHR)	{result = 2;	    DEBUG_MSG("..........CHRSW_CHRON FIF_CHR %ld\n" ,SOC_FIF_CHR,0,0);}\
	else if(nsoc >=	SOC_SIX_CHR)	{result = 2;	    DEBUG_MSG("..........CHRSW_CHRON SIX_CHR %ld\n" ,SOC_SIX_CHR,0,0);}\
	else if(nsoc >=	SOC_SEV_CHR)	{result = 2;	    DEBUG_MSG("..........CHRSW_CHRON SEV_CHR %ld\n" ,SOC_SEV_CHR,0,0);}\
	else if(nsoc >=	SOC_CUT_CHR)	{result = 1;	    DEBUG_MSG("..........CHRSW_CHRON CUT_CHR %ld\n" ,SOC_CUT_CHR,0,0);}

#define GetSDP_WT_CHRSW_CHROFF(psoc, nsoc,result)\
	if	  (nsoc >= 	SOC_MAX_ICV)	{result = 1; 	DEBUG_MSG("..........CHRSW_CHROFF SOC_MAX_ICV %ld\n",SOC_MAX_ICV,0,0);}\
	else if(nsoc >=	SOC_SEC_ICV)	{result = 2;	    DEBUG_MSG("..........CHRSW_CHROFF SOC_SEC_ICV %ld\n",SOC_SEC_ICV,0,0);}\
	else if(nsoc >=	SOC_THR_ICV)	{result = 2;	    DEBUG_MSG("..........CHRSW_CHROFF SOC_THR_ICV %ld\n",SOC_THR_ICV,0,0);}\
	else if(nsoc >=	SOC_FTH_ICV)	{result = 2;	    DEBUG_MSG("..........CHRSW_CHROFF SOC_FTH_ICV %ld\n",SOC_FTH_ICV,0,0);}\
	else if(nsoc >=	SOC_FIF_ICV)	{result = 2;	    DEBUG_MSG("..........CHRSW_CHROFF SOC_FIF_ICV %ld\n",SOC_FIF_ICV,0,0);}\
	else if(nsoc >=	SOC_SIX_ICV)	{result = 1;	    DEBUG_MSG("..........CHRSW_CHROFF SOC_SIX_ICV %ld\n",SOC_SIX_ICV,0,0);}\
	else if(nsoc >=	SOC_SEV_ICV)	{result = 1;	    DEBUG_MSG("..........CHRSW_CHROFF SOC_SEV_ICV %ld\n",SOC_SEV_ICV,0,0);}\
	else if(nsoc >=	SOC_CUT_ICV)	{result = 1;	    DEBUG_MSG("..........CHRSW_CHROFF SOC_CUT_ICV %ld\n",SOC_CUT_ICV,0,0);}

#define GetSDP_WT_CHRSW(P_SOC,N_SOC,bCHR,RESULT)\
	if(bCHR){\
		GetSDP_WT_CHRSW_CHRON(P_SOC,N_SOC,RESULT)\
	}else{\
		GetSDP_WT_CHRSW_CHROFF(P_SOC,N_SOC,RESULT)\
	}\

#define GetSDP_WT_NORMAL(P_SOC,N_SOC,bCHR,RESULT)\
	if(bCHR){\
		GetSDP_WT_NORMAL_CHRON(P_SOC,N_SOC,RESULT)\
	}else{\
		GetSDP_WT_NORMAL_CHROFF(P_SOC,N_SOC,RESULT)\
	}\
	
#define GetSDP_WT(PREV_SOC, NOW_SOC, NOW_CHR, PREV_CHR, RESULT)\
	if	(NOW_CHR != PREV_CHR){\
		GetSDP_WT_CHRSW(PREV_SOC,NOW_SOC,NOW_CHR,RESULT)\
	}else{\
		GetSDP_WT_NORMAL(PREV_SOC,NOW_SOC,NOW_CHR,RESULT)\
	}\
	
#define		ABS(x)					((x) < 0 ? -(x) : (x))
#define		VOL_STEP(x, y)			((x) >= (y) ? (x) : (x + VOLTAGE_STEP))
#define 	MAX(x,y) 				((x)>(y) ? (x):(y))

#endif

#endif
