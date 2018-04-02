/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    (Software Power Gauge CT개발실 1PM 3PL)

    Copyright(c) 2010 LG전자

    Date        Name        Version     Description
    ----------  ------      ----------  -----------------------------------------------------------
    2011.03.10  kensin       v1.1       Convert adc to Voltage for current estimation
    2010.10.22  kensin       v1.04      Calibration for P350, C550
    2010.10.19  EJJ          v1.03      Switching Deviation Protection & PowerApi RPC SPG item code added
    2010.07.17  kensin       v1.02      Charger on battery swap code added
    2010.06.24  EJJ          v1.01      KU380 calibration
    2010.03.31  kensin       v1.00      Modify
    2009.12.09  kensin       v0.90      Create
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/*===================================================================================================
  이파일은 CT개발실 1PM 3PL이 사용하기 위해 만든 파일입니다.
  문제가 생길 경우 Custlge_common.h 에서 FEATURE_POWERAPI_SG를 주석처리해 주시기 바랍니다.
===================================================================================================*/
#include <linux/init.h>        /* For init/exit macros */
#include <linux/module.h>      /* For MODULE_ marcros  */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/xlog.h>
#include <linux/seq_file.h>
#include <linux/namei.h>
#include <linux/syscalls.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <mach/system.h>
#include <mach/mt_sleep.h>

#include <linux/file.h>
#include <linux/fcntl.h>

#include <mach/mt_typedefs.h>
#include <mach/battery_common.h>
#include <linux/sg_common.h>  
#include <mach/charging.h>


#define FEATURE_POWERAPI_SG_ANDROID_LOG

//###############################################################################################
// For ICV & CAL
//###############################################################################################
long g_lSocByICV = SG_CALIBRATING;
int g_nICV = SG_CALIBRATING;
int g_nIcvLog[ICV_LOG_COUNT] = {0};
int g_nCurrent = 0;
int g_nAdc = 0;
kal_bool g_bLongTermFlag = FALSE;
unsigned long g_nIcvTick[ICV_LOG_COUNT] = {0};
unsigned long g_lInterval_checker[ADC_AVG_COUNT];
SG_Info g_stSGInfo;
int g_nPowerOnCause;
int g_nRebootCause;
int g_sgAdcFirst = 1;
int g_is_call_state=0;
struct timespec spg_time_before_sleep;
unsigned long fTime   = 0;

extern int LGBM_ReadBatVoltAdc(void);
//extern int LGBM_GetBatVoltAdc(void);
extern bool LGE_FacWriteSocForSPG(unsigned char *socAddr);
extern bool LGE_FacWriteChargerForSPG(unsigned char *chargerAddr);
extern int LGBM_is_call_state(void);

extern long g_lSocMeas;
extern int g_lgbmChargerType;
extern int g_nFirstStart;
extern long g_lSocView;
extern long g_batICV;
extern int g_lgbmbatSoc;
extern int g_lgbmcharger;
extern int g_nBatteryTemp;
extern int lk_batt_voltage;

///////////////////////////////////////////////////////////////////////////////////////////
//// Thread related 
///////////////////////////////////////////////////////////////////////////////////////////
#define BAT_MS_TO_NS(x) (x * 1000 * 1000)
static kal_bool bat_thread_timeout = KAL_FALSE;
static DECLARE_WAIT_QUEUE_HEAD(spg_thread_wq);
static struct hrtimer spg_kthread_timer;
static kal_bool spg_thread_timeout = KAL_FALSE;

kal_bool writeChargingStatus( int charger )
{
    #if 1
    struct file *fd = NULL;
    int retVal = 0;
    kal_bool result = KAL_TRUE;

    fd = filp_open("/data/battery_charger", O_CREAT|O_TRUNC|O_WRONLY , 0666);
    if (IS_ERR(fd))
    {
        retVal = PTR_ERR(fd);
        DEBUG_MSG("Charger Write ( File Open ) Fail ( errno = %d )\n", retVal);
        result = KAL_FALSE;
    }
    else
    {
        retVal = fd->f_op->write(fd, (unsigned char *)&charger, 1, &(fd->f_pos));
        if( retVal < 0 )
        {
            DEBUG_MSG("Charger Write ( File Write ) Fail ( errno = %d )\n", retVal);
            result = KAL_FALSE;
        }
        else
        {
            DEBUG_MSG("Charger Write Success ( batSoc = %d )\n", charger);
        }

        filp_close(fd, NULL);
    }

    return result;
    #else
    LGE_FacWriteChargerForSPG(&charger);
    DEBUG_MSG("[LGSPG] writeChargingStatus1 = %d\n", charger);
    #endif

}

int readChargingStatus( void )
{
    #if 1
    struct file *fd = NULL;
    int retVal = 0;
    int charger=0;
    int waitCount = 0;

    do {
    fd = filp_open("/data/battery_charger", O_RDONLY , 0666);
    if(!IS_ERR(fd))
    {
        retVal = fd->f_op->read(fd, (unsigned char *)&charger, 1, &(fd->f_pos));
		filp_close(fd, NULL);
        DEBUG_MSG("Charger File Open Success!\n");
            break;
    }
    else
    {
            waitCount++;
        retVal = PTR_ERR(fd);
            DEBUG_MSG("Wait for mount /data/ file system: %dtime, retVal: %d\n", waitCount, retVal);
            msleep(200);
    }
    }while(waitCount < 5);

    if( retVal < 0 )
    {
        DEBUG_MSG("Charger Read ( File Read ) Fail ( errno = %d )\n", retVal);
        charger = -1;
    }
    else
    {
        DEBUG_MSG("Charger Read Success ( charger = %d )\n", charger);
    }

    return charger;
    #else
    DEBUG_MSG("[LGSPG] readChargingStatus = %d\n", g_lgbmcharger);
    return g_lgbmcharger;
    #endif

}

kal_bool writeSocStatus( int batSoc )
{
    #if 1
    struct file *fd = NULL;
    int retVal = 0;
    kal_bool result = KAL_TRUE;

    fd = filp_open("/data/battery_soc", O_CREAT|O_WRONLY|O_SYNC , 0666);
    if (IS_ERR(fd))
    {
        retVal = PTR_ERR(fd);
        DEBUG_MSG("Soc Write ( File Open ) Fail ( errno = %d )\n", retVal);
        result = KAL_FALSE;
    }
    else
    {
        retVal = fd->f_op->write(fd, (unsigned char *)&batSoc, 1, &(fd->f_pos));
        if( retVal < 0 )
        {
            DEBUG_MSG("Soc Write ( File Write ) Fail ( errno = %d )\n", retVal);
            result = KAL_FALSE;
        }
        else
        {
            DEBUG_MSG("Soc Write Success ( batSoc = %d )\n", batSoc);
        }

        filp_close(fd, NULL);
    }

    return result;
    #else
    LGE_FacWriteSocForSPG(&batSoc);
    DEBUG_MSG("[LGSPG] writeSocStatus = %d\n", batSoc);
    #endif

}

int readSocStatus( void )
{
    #if 1
    struct file *fd = NULL;
    int retVal = 0;
    int batSoc=0;
    int waitCount = 0;

    do {
        fd = filp_open("/data/battery_soc", O_RDONLY , 0666);
        if(!IS_ERR(fd))
        {
            retVal = fd->f_op->read(fd, (unsigned char *)&batSoc, 1, &(fd->f_pos));
			filp_close(fd, NULL);
            DEBUG_MSG("Soc File Open Success!\n");
            break;
        }
        else
        {
            waitCount++;
            retVal = PTR_ERR(fd);
            DEBUG_MSG("Wait for mount /data/ file system: %dtime, retVal: %d\n", waitCount, retVal);
            msleep(1000);
        }
    }while(waitCount < 5);

    if( retVal < 0 )
    {
        DEBUG_MSG("Soc Read ( File Read ) Fail ( errno = %d )\n", retVal);
        batSoc = -1;
    }
    else
    {
        DEBUG_MSG("Soc Read Success ( batSoc = %d )\n", batSoc);
    }

    return batSoc;
    #else
    DEBUG_MSG("[LGSPG] readSocStatus = %ld\n",g_lgbmbatSoc);
    return g_lgbmbatSoc;
    #endif

}

kal_bool chg_is_charging( void )
{
    kal_bool result = KAL_TRUE;

    if ( g_lgbmChargerType == CHARGER_UNKNOWN )
    {
        result = KAL_FALSE;
    }
    
    return result;
}

#ifdef FEATURE_POWERAPI_SG_ANDROID_LOG

void SPG_FileLog(long time, char* msg)
{
    struct file *fd = NULL;
	int		retVal = 0;
    char    filename[30];
    char    cMsg[200];
    int     nSec            = 0;
    int     nMinTmp         = 0;
    int     nMin            = 0;
    int     nHour           = 0;

    nSec    = (int)(time/PARAM_SCALE)%60;
    nMinTmp = (int)(time/PARAM_SCALE)/60;
    nMin    = nMinTmp%60;
    nHour   = nMinTmp/60;

    sprintf(filename,"/data/spg.log");
    sprintf(cMsg, "%02d:%02d:%02d: %s\n", nHour, nMin, nSec, msg);

	/*fd = filp_open(filename, O_CREAT|O_TRUNC|O_WRONLY , 0666);
	if (IS_ERR(fd))
    {
        DEBUG_MSG("Write ( File Open ) Fail ( errno = %d )\n", retVal);
    }
    else
    {
        retVal = fd->f_op->write(fd, cMsg, 1, &(fd->f_pos));
        if( retVal < 0 )
        {
            DEBUG_MSG("Write ( File Write ) Fail ( errno = %d )\n", retVal);
        }
        else
        {
            DEBUG_MSG("Write Success\n");
        }

        filp_close(fd, NULL);
    }*/
	DEBUG_MSG("SPGLOG:%s",cMsg);
    return;

}
#endif

long SG_SocMeasure(void)
{
    long lP1 = 0;
    long lA4 = 0;
    static unsigned long lT4 = 0;
    static long lA1 = 0;
    static long lA2 = 0;
    static long lC1 = 0;
	long lA3 = 0;
	static unsigned long lT3 = 0;
    static int  nT1 = 0;
    int i   = 0;
	static spg_state temp1 = -2;
	static spg_state temp2 = -2;
	
#ifdef FEATURE_POWERAPI_SG_ANDROID_LOG//file log를 위한 code이므로 porting 하지 않음
    static unsigned long fTime   = 0;
    char sg_msg[200];
#endif
    while(g_stSGInfo.nAdcValue[i] != END_OF_ADC_LIST)
    {
        if(!g_stSGInfo.nAdcValue[i])
        {
            return 0;
        }
		 lA1 += g_stSGInfo.nAdcValue[i];
		 nT1++;
		 lT4 += g_lInterval_checker[i];

		if((++lT3 % ADC_AVG_QUEUE_LENGTH) == 0) //1.5초
		{
			lA2 = lA1/nT1;
		
#ifdef SG_THERMAL_EFFECT
            if(g_nBatteryTemp < THERMAL_0DEGREE)
            {
                lA3 = lA3 + (THERMAL_GRADIENT * g_nBatteryTemp + THERMAL_INTERCEPT);
                lA2 = lA2 + (THERMAL_GRADIENT * g_nBatteryTemp + THERMAL_INTERCEPT);
            }
#endif
			GetADC_SCV(g_lSocMeas, lA3);

			lA4 = lA3 - lA2;
			
			if( g_lgbmChargerType != CHARGER_UNKNOWN && lA4 < 0 )		
			{
			    lC1 = lA4 * CHARGING_CURRENT_PER_DROP;
			}
			else
			{
			    lC1 = lA4 * CURRENT_PER_DROP;
			}
			
			if(lT4 < 0)
            {
                lT4 = 6000;
            }
            lP1 = lC1 * lT4;
            lP1 = lP1 / (BATTERY_CAP_TO_SEC / 100);
#ifdef FEATURE_POWERAPI_SG_ANDROID_LOG
            sprintf(sg_msg, "SM: %ld, lA2: %ld, T: %ld, V: %d, Tmp2: %d Fuel_SOC: %d Current: %d\n",g_lSocMeas, (long)lA2, (long)lT4, LGBM_ReadBatVoltAdc(), 77,g_lSocView,lC1);
            SPG_FileLog(fTime, sg_msg);
            fTime += lT4;
#endif
            if(g_lSocMeas >= SOC_ONE_PERCENT)
            {
                if((lC1 > BATTERY_CAPABILTY * 8 / 10) && g_lgbmChargerType == CHARGER_UNKNOWN)
                {
                    g_lSocMeas -= lP1 * 2;
                }
#if defined(TARGET_S4)
				else if( g_lgbmChargerType != CHARGER_UNKNOWN)
				{
					if(lA2 < 3900)
						g_lSocMeas -= lP1 * 3;
					else
						g_lSocMeas -= lP1 * 2;
				}
#else
				else if( g_lgbmChargerType != CHARGER_UNKNOWN && lA2 < 3850 && lA2 > 3600 )
				{
					g_lSocMeas -= lP1 * 2;
				}
#endif				
                else
                {
                    g_lSocMeas -= lP1;
                }
            }
#ifdef LOW_BATTERY_DEFENCE
            else if((g_lSocMeas > LOW_BATTERY_SOC) && (lA2 < LOW_BATTERY_ADC))
            {	
                g_lSocMeas = LOW_BATTERY_SOC;
            }
#endif
            else
            {
                if((g_lSocMeas < SOC_ONE_PERCENT) && (lA2 < ADC_CUT_CHR) && g_lgbmChargerType != CHARGER_UNKNOWN)
                {
                    g_lSocMeas -= 0;
                }
                else if(g_lSocMeas < SOC_ONE_PERCENT)
                {
                    g_lSocMeas -= lP1 / 2;
                }
                else
                {
                    g_lSocMeas = 0;
                }
            }			
            if(g_lSocMeas < 1*PARAM_SCALE)
            {
                g_lSocMeas = 1*PARAM_SCALE;
            }
            else if(g_lSocMeas >= SOC_MEAS_MAX)
            {
                g_lSocMeas = SOC_MEAS_MAX;
            }
            if(((g_lSocMeas < g_lSocView) && (g_lSocView > 0)) || (g_lSocMeas >= g_lSocView + 5 * PARAM_SCALE) ||
                (g_lgbmChargerType != CHARGER_UNKNOWN && (g_lSocMeas > g_lSocView) && (g_lSocView < SOC_VIEW_MAX)))
            {
                g_lSocView = g_lSocMeas;
            }
            lA1 = 0;
            nT1 = 0;
            lT3 = 0;
            lT4 = 0;
            g_nFirstStart = 0;
#ifdef FEATURE_POWERAPI_SG_ANDROID_LOG
            g_nCurrent = lC1;
#endif
        }
        i++;
    }
    return lC1;
}

#ifdef FEATURE_POWERAPI_SG_NV_USE
void SG_AdoptSDP(kal_bool bCurrCharger) //EJJ_TEST 101020 To protect the ICV_SOC swing at system on/off charger connection whether or not
{
    int readSoc = 0;
    int initSoc = 0;
	int initSocGap = 0;
    int nNvWriteSoc = 0;
    int bNvWriteChr = 0;
    kal_bool status = KAL_FALSE;

    readSoc = readSocStatus();
    initSoc = g_lSocByICV / PARAM_SCALE;
    if( readSoc > 0 && readSoc <= 101 && initSoc > 5 )
    {
        if( readSoc > initSoc )
        {
            initSocGap = readSoc - initSoc;
            if( initSocGap > 5 )
            {
                readSoc = readSoc - 1;
            }
        }
        else
        {
            initSocGap = initSoc - readSoc;
        }
        if( initSocGap < 15 )
        {
            initSoc = readSoc;
        }
    }
    if( bCurrCharger == KAL_TRUE )
	{
        g_lSocMeas = initSoc * PARAM_SCALE;
    }
	else
	{
        g_lSocMeas = (initSoc * PARAM_SCALE) + PARAM_SCALE;
	}

    (g_lSocByICV > SOC_VIEW_MAX) ? (g_lSocView = SOC_VIEW_MAX) : (g_lSocView = g_lSocMeas);

    //EJJ_TEST 101020 Switching Deviation Protection
    nNvWriteSoc = (int)(g_lSocView / PARAM_SCALE);
    bNvWriteChr = chg_is_charging();

    status = writeSocStatus(nNvWriteSoc);
    status = writeChargingStatus(bNvWriteChr);

    /*status = nvio_write_item(NV_POWERAPI_SG_PARAM1_I,0,&nNvWriteSoc,nvim_op_get_size(NV_POWERAPI_SG_PARAM1_I),0);
    if(status != NV_DONE_S)
        DEBUG_MSG("[LGSPG] Fail to write Nv ITEM param1111 %d , %d",nNvWriteSoc,0,0);

    status = nvio_write_item(NV_POWERAPI_SG_PARAM2_I,0,&bNvWriteChr,nvim_op_get_size(NV_POWERAPI_SG_PARAM2_I),0);
    if(status != NV_DONE_S)
        DEBUG_MSG("[LGSPG] Fail to write Nv ITEM param222 %d , %d",bNvWriteChr,0,0);

    DEBUG_MSG("[LGSPG] ******* NV Write SocView*100= %d , ChagerOnOff=%d , Time=%d\n", nNvWriteSoc, chg_is_charging(), 0);
*/
    DEBUG_MSG("[LGSPG] ******* NV Write SocView = %ld , ChagerOnOff=%d , Time=%d\n", nNvWriteSoc, bNvWriteChr, 0);
}//EJJ_END
#endif

void SG_AdcLog(void)
{
    int nBattAdc = 9999;
    static bool bFirst = true;
    static int nLogCount = 0;
    static long lSumAdc = 0;
    static unsigned long old_sec = 0;
    static CHARGER_TYPE nLongTermStat = 0;

	struct timespec xts, tom, spg_time;
	
#ifdef FEATURE_POWERAPI_SG_NV_USE
    static int bNvWriteChr = 0;
    static long nNvWriteSoc = 0;
    int status = 0;
#endif

#ifdef FEATURE_POWERAPI_SG_ANDROID_LOG
    char sg_msg[300];
#endif
    if(g_lSocByICV == SG_CALIBRATING)
    {
        bFirst      = true;
        nLogCount   = 0;
        lSumAdc     = 0;
    }
    if (nLogCount < ADC_AVG_COUNT)
    { 
        nBattAdc = LGBM_ReadBatVoltAdc();
        
        if(old_sec == 0)
        {
			old_sec = jiffies/HZ;
			return;
        }
        g_lInterval_checker[nLogCount] = (((jiffies/HZ) - old_sec) * PARAM_SCALE);
		
		old_sec = jiffies/HZ;

        g_stSGInfo.nAdcValue[nLogCount] = nBattAdc;
        lSumAdc += nBattAdc;
        nLogCount++;
        if(bFirst)
        {
            bFirst=false;
#ifdef SG_THERMAL_EFFECT
            if(g_nBatteryTemp < THERMAL_0DEGREE)
            {
                g_nICV = g_nICV + (THERMAL_GRADIENT * g_nBatteryTemp + THERMAL_INTERCEPT);
            }
#endif
            switch (g_lgbmChargerType){
                case NONSTANDARD_CHARGER:
                case STANDARD_CHARGER:
                case FACTORY_CHARGER:
					lk_batt_voltage=lk_batt_voltage-COMPENSATION_VOLTAGE_TA;  //voltage compensation in case of TA boot				
					GetSOC(SOC_ICV, lk_batt_voltage, g_lSocByICV);
					DEBUG_MSG("[LGSPG] Charger on SOC_CHR lk_batt_voltage = %d,g_lSocByICV = %ld  \n",lk_batt_voltage,g_lSocByICV);
					g_nPowerOnCause = SOC_CHR;
#ifdef FEATURE_POWERAPI_SG_NV_USE
                    SG_AdoptSDP(KAL_TRUE);
#else
                    g_lSocMeas = g_lSocByICV;
#endif
                    break;
                case STANDARD_HOST:
                case CHARGING_HOST:
					lk_batt_voltage=lk_batt_voltage-COMPENSATION_VOLTAGE_USB;  //voltage compensation in case of usb boot
                    GetSOC(SOC_ICV, lk_batt_voltage, g_lSocByICV);
                    DEBUG_MSG("[LGSPG] Charger on SOC_USB lk_batt_voltage = %d,g_lSocByICV = %ld  \n",lk_batt_voltage,g_lSocByICV);
                    g_nPowerOnCause = SOC_USB;
#ifdef FEATURE_POWERAPI_SG_NV_USE
                    SG_AdoptSDP(KAL_TRUE);
#else
                    g_lSocMeas = g_lSocByICV;
#endif
                    break;
                default:
					lk_batt_voltage=lk_batt_voltage+COMPENSATION_VOLTAGE_NORMAL;  //voltage compensation in case of normal boot
                    GetSOC(SOC_ICV, lk_batt_voltage, g_lSocByICV);
                    DEBUG_MSG("[LGSPG] Charger on SOC_ICV lk_batt_voltage = %d,g_lSocByICV = %ld  \n",lk_batt_voltage,g_lSocByICV);
                    g_nPowerOnCause = SOC_ICV;
#ifdef FEATURE_POWERAPI_SG_NV_USE
                    SG_AdoptSDP(KAL_FALSE);
#else
                    g_lSocMeas = g_lSocByICV;
#endif
                    break;
                }
            g_stSGInfo.nAdcValue[nLogCount]= END_OF_ADC_LIST;
            DEBUG_MSG("[LGSPG] init SOC g_lSocMeas = %ld \n",g_lSocMeas);			
        }
    }
    else if(nLogCount >= ADC_AVG_COUNT)
    {
        g_stSGInfo.nAdcValue[nLogCount]= END_OF_ADC_LIST;
        nLogCount = 0;
        lSumAdc = 0;
#ifdef FEATURE_POWERAPI_SG_NV_USE
        if(((ABS(nNvWriteSoc - (int)(g_lSocView/PARAM_SCALE))) >= NV_SAVE_SOC_INTERVAL) || bNvWriteChr != chg_is_charging())
            //&& chg_is_battery_valid())
        {

        nNvWriteSoc = (int)(g_lSocView/PARAM_SCALE);
        bNvWriteChr = chg_is_charging();

        status = writeSocStatus(nNvWriteSoc);
        
        status = writeChargingStatus(bNvWriteChr);
        /*status = nvio_write_item(NV_POWERAPI_SG_PARAM1_I,0,&nNvWriteSoc,nvim_op_get_size(NV_POWERAPI_SG_PARAM1_I),0);
        if(status != NV_DONE_S)
            DEBUG_MSG("[LGSPG] Fail to write Nv ITEM param1111 %d , %d",nNvWriteSoc,0,0);

        status = nvio_write_item(NV_POWERAPI_SG_PARAM2_I,0,&bNvWriteChr,nvim_op_get_size(NV_POWERAPI_SG_PARAM2_I),0);
        if(status != NV_DONE_S)
            DEBUG_MSG("[LGSPG] Fail to write Nv ITEM param222 %d , %d",bNvWriteChr,0,0);*/

        DEBUG_MSG("[LGSPG] ******* NV Write SocView = %d , ChagerOnOff=%d , Time=%d\n", nNvWriteSoc, bNvWriteChr, 0);
        }
#endif
        SG_SocMeasure();
    }
}


void SG_ICV_cb(void)
{
    static int nBattAdc = 9999;
    static int nLogCount = 0;
    static long fSumBattAdc = 0;
    int i = 0;
    int fDiff = 0;
    int nValidCount = 0;
    static int flag = 0;
#ifdef FEATURE_POWERAPI_SG_ANDROID_LOG
    char sg_msg[300];
#endif
    long fAvgSequence = 0;

    if (nLogCount < ICV_LOG_COUNT)
    {
        nBattAdc = LGBM_ReadBatVoltAdc();
        if(isValid(nBattAdc))
		{
            fSumBattAdc         += nBattAdc ;
            g_nIcvLog[nLogCount] = nBattAdc;
            g_nIcvTick[nLogCount]= jiffies;
            nLogCount++;
        }
#ifdef FEATURE_POWERAPI_SG_ANDROID_LOG
        DEBUG_MSG("nBattAdc:%d, fSumBattAdc:%d, nLogCount:%d, nValidCount:%d\n",nBattAdc, fSumBattAdc, nLogCount, nValidCount);
#endif
    }
    else
    {
        fSumBattAdc     = 0;
        nValidCount     = 1;
        fAvgSequence    = 0;

        for(i = 1; i < ICV_LOG_COUNT; i++)
        {
            fAvgSequence = ((fSumBattAdc + g_nIcvLog[i -1]) * 100) / (nValidCount);
            fDiff = g_nIcvLog[i] - (fAvgSequence / 100);
            if(fDiff <= ADC_SWING_MARGIN)
            {
                nValidCount++;
                fSumBattAdc += g_nIcvLog[i - 1];
            }
            else
            {
                if(i < (ICV_LOG_COUNT - 6)) //Minimum 6 samples.
                {
                    nValidCount = 1;
                    fSumBattAdc = 0;
                }
                else
                {
                    i = ICV_LOG_COUNT;
                }
            }
        }
        g_nICV = (int)(fAvgSequence / 100);
        g_batICV = g_nICV;
        flag = 1;
        nBattAdc    = 9999;
        nLogCount   = 0;
        fSumBattAdc = 0;
    }
}


int spg_thread_kthread( void *x )
{
    g_lSocMeas  = SG_CALIBRATING;
    g_lSocByICV = SG_CALIBRATING;
    g_nICV      = SG_CALIBRATING;
    
    ktime_t ktime = ktime_set(0, BAT_MS_TO_NS(100));
    
    while (1)
    {
        //DEBUG_MSG("[SG_SPG] wait event \n");
        wait_event(spg_thread_wq, (spg_thread_timeout == KAL_TRUE));
        spg_thread_timeout = KAL_FALSE;
        if( g_nICV == SG_CALIBRATING )
        {
            SG_ICV_cb();
        }
        else
        {
            SG_AdcLog();
        }
        
        hrtimer_start(&spg_kthread_timer, ktime, HRTIMER_MODE_REL);
        if( g_nFirstStart == 0 )
        {
            ktime = ktime_set(1, 0);
        }
    }
    return 0;
}

void spg_thread_wakeup(void)
{
    //DEBUG_MSG("******** spg : spg_thread_wakeup  ********\n");
    
    spg_thread_timeout = KAL_TRUE;
    
    wake_up(&spg_thread_wq);    
}

enum hrtimer_restart spg_kthread_hrtimer_func(struct hrtimer *timer)
{
    spg_thread_wakeup(); 
    
    return HRTIMER_NORESTART;
}

void spg_kthread_hrtimer_init(void)
{
    
    ktime_t ktime;
    
    ktime = ktime_set(3, 0);    // 3s, 10* 1000 ms
    hrtimer_init(&spg_kthread_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    spg_kthread_timer.function = spg_kthread_hrtimer_func;
    hrtimer_start(&spg_kthread_timer, ktime, HRTIMER_MODE_REL);

    DEBUG_MSG("battery_kthread_hrtimer_init : done\n" );
}

static int sg_spg_probe(struct platform_device *dev)
{
	spg_kthread_hrtimer_init();
	
    kthread_run(spg_thread_kthread, NULL, "spg_thread_kthread"); 
    DEBUG_MSG("[spg_probe] spg_thread_kthread Done\n");
    return 0;
}

static int sg_spg_remove(struct platform_device *dev)
{
    DEBUG_MSG("[sg_spg_remove]\n");
    return 0;
}

static void sg_spg_shutdown(struct platform_device *dev)
{
    DEBUG_MSG("[sg_spg_shutdown]\n");
}

static int sg_spg_suspend(struct platform_device *dev, pm_message_t state)
{	
	return 0;
}

static int sg_spg_resume(struct platform_device *dev)
{
    return 0;
}

struct platform_device sg_spg_device = {
        .name                = "sg_spg",
        .id                  = -1,
};

static struct platform_driver sg_spg_driver = {
    .probe        = sg_spg_probe,
    .remove       = sg_spg_remove,
    .shutdown     = sg_spg_shutdown,
//    .suspend      = sg_spg_suspend,
//    .resume       = sg_spg_resume,
    .driver       = {
        .name = "sg_spg",
    },
};

static int __init SG_SPG_Init(void)
{
    DEBUG_MSG("******** SPG driver Init!! ********\n");

	int ret;

    ret = platform_device_register(&sg_spg_device);
    if (ret) {
        DEBUG_MSG("[sg_spg_device] Unable to device register(%d)\n", ret);
        return ret;
    }

    ret = platform_driver_register(&sg_spg_driver);
    if (ret) {
        DEBUG_MSG("[sg_spg_driver] Unable to register driver (%d)\n", ret);
        return ret;
    }

    DEBUG_MSG("[sg_spg_driver] Initialization : DONE \n");

    return 0;
}

static void __exit SG_SPG_exit(void)
{
    g_lSocMeas  = SG_CALIBRATING;
    g_lSocByICV = SG_CALIBRATING;
    g_nICV = SG_CALIBRATING;
    return;
}
module_init(SG_SPG_Init);
module_exit(SG_SPG_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Software Power Gauge");
MODULE_AUTHOR("LG Electornics");
