/*****************************************************************************
 *
 * Filename:
 * ---------
 *    battery_common.c
 *
 * Project:
 * --------
 *   Android_Software
 *
 * Description:
 * ------------
 *   This Module defines functions of mt6323 Battery charging algorithm 
 *   and the Anroid Battery service for updating the battery status
 *
 * Author:
 * -------
 * Oscar Liu
 *
 ****************************************************************************/
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

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <mach/system.h>
#include <mach/mt_sleep.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpt.h>
#include <mach/mt_boot.h>

#include <cust_charging.h>  
#include <mach/upmu_common.h>
#include <mach/upmu_hw.h>
#include <mach/charging.h>
#include <mach/battery_common.h>
#include <mach/battery_meter.h>
#include "cust_battery_meter.h"
#include <mach/mt_boot.h>
#include "mach/mtk_rtc.h"

//                                                                          
#if defined( LGE_BSP_LGBM )
////////////////////////////////////////////////////////////////////////////////
// LG Battery Management
////////////////////////////////////////////////////////////////////////////////
#include <mach/mt_gpio.h>
#include <linux/charger_bq25040.h>
#include <linux/sg_common.h>

//                                                                               
#if defined(CONFIG_LGE_MINIABB)
#include <mach/lge_miniabb.h>
#endif
//                                                                               

extern int IMM_auxadc_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
extern int PMIC_IMM_GetOneChannelValue(int dwChannel, int deCount, int trimd);
extern long SG_GetSoc(void);

#define LGBM_TAG "LGBM"
#if defined (TARGET_S4)
#define LGBM_BAT_REMOVE_ADC_TH (1118)
#define LGBM_LOW_VOLTAGE	(3450)
#else
#define LGBM_BAT_REMOVE_ADC_TH (1100)
#define LGBM_LOW_VOLTAGE	(3400)
#endif
#define RBAT_PULL_UP_R             61900	
#define RBAT_PULL_DOWN_R		  100000	
#define RBAT_PULL_UP_VOLT          	1800

#define LGBM_OTP_STOP_MIN_TEMP          (-10)
#define LGBM_OTP_STOP_MAX_TEMP          (56)
#define LGBM_OTP_DECREASE_TEMP          (45)

#define LGBM_OTP_STOP_TO_NORMAL_TEMP     (-5)
#define LGBM_OTP_DECREASE_TO_NORMAL_TEMP (42)
#define LGBM_OTP_STOP_TO_DECREASE_TEMP   (52)

#define LGBM_OTP_STOP_VOLTAGE       (4000)  // STOP CHARGING WHEN TEMPERATURE IS 45 C ~ 55C AND VOLTAGE IS OVER 4.0V.
#define LGBM_OTP_HYST_VOLTAGE       (3900)  // STOP TO DECREASE CHARGINHG VOLTAGE WHEN TEMPERATURE IS 45 C ~ 55C.

#if defined(CONFIG_LGE_MINIABB)
#define LGBM_SW_EOC_VOLTAGE_TH (4300)
#define LGBM_CHARGING_FULL_TO_RECHARGING    (4250)
#define LGBM_CUT_OFF_VOLTAGE (3400)
#define LGBM_CHARGING_FULL_TO_CHARGING_VOLTAGE  (4200)
#else
#define LGBM_SW_EOC_VOLTAGE_TH (4100)
#define LGBM_CHARGING_FULL_TO_RECHARGING    (4150)
#define LGBM_CUT_OFF_VOLTAGE (3300)
#define LGBM_CHARGING_FULL_TO_CHARGING_VOLTAGE  (3700)
#endif


/* ADC Channel Number */
#define VBAT_CHANNEL_NUMBER      7
#define ISENSE_CHANNEL_NUMBER	 6
#define VCHARGER_CHANNEL_NUMBER  4
#define VBATTEMP_CHANNEL_NUMBER  5

#define AUXADC_USB_ID_CHANNEL	 0

#define LGBM_CABLE_ID_56K_ADC_MIN	(45)
#define LGBM_CABLE_ID_56K_ADC		(60)
#define LGBM_CABLE_ID_56K_ADC_MAX	(71)
#if defined (TARGET_S4)
#define LGBM_CABLE_ID_130K_ADC_MIN	(72)
#define LGBM_CABLE_ID_130K_ADC		(91)
#define LGBM_CABLE_ID_130K_ADC_MAX	(106)
#define LGBM_CABLE_ID_180K_ADC_MIN	(107)
#define LGBM_CABLE_ID_180K_ADC		(111)
#define LGBM_CABLE_ID_180K_ADC_MAX	(115)
#else
#define LGBM_CABLE_ID_130K_ADC_MIN	(72)
#define LGBM_CABLE_ID_130K_ADC		(91)
#define LGBM_CABLE_ID_130K_ADC_MAX	(96)
#define LGBM_CABLE_ID_180K_ADC_MIN	(97)
#define LGBM_CABLE_ID_180K_ADC		(102)
#define LGBM_CABLE_ID_180K_ADC_MAX	(115)
#endif
#define LGBM_CABLE_ID_910K_ADC_MIN	(116)
#define LGBM_CABLE_ID_910K_ADC		(136)
#define LGBM_CABLE_ID_910K_ADC_MAX	(141)
#define LGBM_CABLE_ID_OPEN_ADC_MIN	(143)
#define LGBM_CABLE_ID_OPEN_ADC		(150)

#define LGBM_LOG(fmt, args...)  	printk("[LGBM] "fmt, ##args)
#define LGBM_ERR(fmt, args...)  printk("[LGBM] [ERROR] %s() line=%d : "fmt, __FUNCTION__, __LINE__, ##args)
#if defined (TARGET_S4)
BATT_TEMPERATURE lgbmBatTempTbl[] = {  // at 25C, Rntc, 25C = 68000 ohm
	{-20,95880},
	{-15,93200},
	{-10,88834},
	{ -5,83446},
	{  0,77378},
	{  5,69371},
	{ 10,62709},
	{ 15,56585},
	{ 20,48382},
	{ 25,44318},
	{ 30,37816},
	{ 35,32976},
	{ 40,28539},
	{ 45,24046},
	{ 50,20390},
	{ 55,17357},
	{ 60,14804},
	{ 65,11669}
};
#else
BATT_TEMPERATURE lgbmBatTempTbl[] = {  // at 25C, Rntc, 25C = 68000 ohm
    {-20,87408},
    {-15,83870},
    {-10,79708},
    { -5,74947},
    {  0,69661},
    {  5,63970},
    { 10,58032},
    { 15,52023},
    { 20,46120},
    { 25,40476},
    { 30,35213},
    { 35,30410},
    { 40,26110},
    { 45,22318},
    { 50,19017},
    { 55,16171},
    { 60,13738},
    { 65,11669}
};
#endif

typedef enum LGBmCableIdTag
{
	USB_CABLE_ID_NONE = 0,
	USB_CABLE_ID_OPEN,
	USB_CABLE_ID_56K,
	USB_CABLE_ID_130K,
	USB_CABLE_ID_180K,
	USB_CABLE_ID_910K,
	USB_CABLE_ID_UNKNOWN,	

	USB_CABLE_ID_MAX
}
LGBmCableId;


typedef struct LGBmVitalTag
{
    kal_int32 batTempAdc;
    kal_int32 batVoltAdc;
    kal_int32 chgVoltAdc;
    kal_int32 batTemp;
    kal_int32 batVolt;
    kal_int32 chgVolt;
} LGBmVital;

typedef enum LGBmChargingCurrentTag
{
    LGBM_CC_OFF = 0,
    LGBM_CC_USB_100,
    LGBM_CC_USB_500,
    LGBM_CC_I_SET,
    LGBM_CC_FACTORY,
    LGBM_CC_UNKNOWN,
} LGBmChargingCurrent;

typedef enum LGBmTempStateTag
{
    LGBM_TEMP_LOW = 0,
    LGBM_TEMP_MIDDLE,
    LGBM_TEMP_HIGH,
    LGBM_TEMP_ULTRA_HIGH,
    LGBM_TEMP_UNKNOWN
} LGBmTempState;

typedef enum {
    LGBM_TS_INIT = 0,
    LGBM_TS_NO_CHARGER,
    LGBM_TS_CHARGING,
    LGBM_TS_CHARGING_FULL,
    LGBM_TS_FACTORY,
    LGBM_TS_INSERT_BATTERY,
    LGBM_TS_UNKNOWN
} LGBmState;

typedef enum {
    LGBM_OTP_NORMAL_CHARGING = 0,
    LGBM_OTP_DECREASE_CHARGING,
    LGBM_OTP_STOP_CHARGING,
    LGBM_OTP_UNKNOWN,
} LGBmOTPState;

typedef enum LGBmSocTrackStateTag
{
    LGBM_SOC_UN_TRACKED = 0,
    LGBM_SOC_ON_TRACKING,
    LGBM_SOC_TRACKED
} LGBmSocTrackState;

typedef struct LGBmDataTag
{
    kal_int32 initSoc;
    kal_int32 curSoc;
    kal_int32 batExist;
    CHARGER_TYPE charger;
    LGBmState bmState;
    LGBmOTPState bmOTPState;
    //kal_bool  bmOTPChanged;
    LGBmSocTrackState chargingSocTrackState;
    LGBmSocTrackState nochargerSocTrackState;
    LGBmTempState tempState;
} LGBmData;

kal_int32 g_batVoltAdc = 0;	//For LG_SPG
//kal_int32 g_lgbmInitSoc = 0;	//For LG_SPG
CHARGER_TYPE g_lgbmChargerType = CHARGER_UNKNOWN;	//For LG_SPG
EXPORT_SYMBOL(g_lgbmChargerType);
int g_nFirstStart = 1; //For LG_SPG
EXPORT_SYMBOL(g_nFirstStart);
long g_lSocView = SG_CALIBRATING; //For LG_SPG
EXPORT_SYMBOL(g_lSocView);
long g_lSocMeas = SG_CALIBRATING;   //For LG_SPG
EXPORT_SYMBOL(g_lSocMeas);
int g_batICV = 0;   //For LG_SPG
EXPORT_SYMBOL(g_batICV);
int g_nBatteryTemp = 25;   //For LG_SPG
EXPORT_SYMBOL(g_nBatteryTemp);
int g_updateBatStateFlag = 0;
int g_low_bat_wakelock_enable = 0;

extern LGBmCableId g_lgbmBootUsbCableId;

#if defined (TARGET_S4)
extern char g_batt_id_info[];
#endif

LGBmVital *g_pAtCmdBatVital = NULL; /* bad design but for easy implementation, it's read only variable for atci_service process */
LGBmData *g_pAtCmdBmData = NULL; /* bad design but for easy implementation, it's read only variable for atci_service process */
kal_int32 g_AtCmdChargeMode = 0; //For at%charge
kal_int32 g_AtCmdBatSocUI = 0;
kal_int32 g_AtCmdUsbId = 0;  //for at%usbidadc
kal_int32 g_AtCmdUsbAdc = 0;  //for at%usbidadc

int g_AtCmdBatFullUI = 0;
kal_int32 g_AtCmdChargingModeOff = 0;
void LGBM_ReadBatVital(LGBmVital *pVital);
int LGBM_ReadBatVoltAdc ( void );
LGBmCableId LGBM_ReadUsbCableId( void );



static int fake_batt_mode = 0;
#endif
//                                                                          

////////////////////////////////////////////////////////////////////////////////
// Battery Logging Entry
////////////////////////////////////////////////////////////////////////////////
int Enable_BATDRV_LOG = BAT_LOG_CRTI;
//static struct proc_dir_entry *proc_entry;
char proc_bat_data[32];  

///////////////////////////////////////////////////////////////////////////////////////////
//// Smart Battery Structure
///////////////////////////////////////////////////////////////////////////////////////////
PMU_ChargerStruct BMT_status;


///////////////////////////////////////////////////////////////////////////////////////////
//// Thermal related flags
///////////////////////////////////////////////////////////////////////////////////////////
int g_battery_thermal_throttling_flag=1; // 0:nothing, 1:enable batTT&chrTimer, 2:disable batTT&chrTimer, 3:enable batTT, disable chrTimer
int battery_cmd_thermal_test_mode=0;
int battery_cmd_thermal_test_mode_value=0;
int g_battery_tt_check_flag=0; // 0:default enable check batteryTT, 1:default disable check batteryTT


///////////////////////////////////////////////////////////////////////////////////////////
//// Global Variable
///////////////////////////////////////////////////////////////////////////////////////////
struct wake_lock battery_suspend_lock; 
CHARGING_CONTROL battery_charging_control;
unsigned int g_BatteryNotifyCode=0x0000;
unsigned int g_BN_TestMode=0x0000;
kal_bool g_bat_init_flag = 0;
kal_bool g_call_state = CALL_IDLE;
kal_bool g_charging_full_reset_bat_meter = KAL_FALSE;
int g_platform_boot_mode = 0;
struct timespec g_bat_time_before_sleep;
int g_smartbook_update = 0;

#if defined(MTK_TEMPERATURE_RECHARGE_SUPPORT)
kal_uint32 g_batt_temp_status = TEMP_POS_NORMAL;
#endif

kal_bool g_battery_soc_ready = KAL_FALSE;
////////////////////////////////////////////////////////////////////////////////
// Integrate with NVRAM 
////////////////////////////////////////////////////////////////////////////////
#define ADC_CALI_DEVNAME "MT_pmic_adc_cali"
#define TEST_ADC_CALI_PRINT _IO('k', 0)
#define SET_ADC_CALI_Slop _IOW('k', 1, int)
#define SET_ADC_CALI_Offset _IOW('k', 2, int)
#define SET_ADC_CALI_Cal _IOW('k', 3, int)
#define ADC_CHANNEL_READ _IOW('k', 4, int)
#define BAT_STATUS_READ _IOW('k', 5, int)
#define Set_Charger_Current _IOW('k', 6, int)
//add for meta tool-----------------------------------------
#define Get_META_BAT_VOL _IOW('k', 10, int) 
#define Get_META_BAT_SOC _IOW('k', 11, int) 
//add for meta tool-----------------------------------------

static struct class *adc_cali_class = NULL;
static int adc_cali_major = 0;
static dev_t adc_cali_devno;
static struct cdev *adc_cali_cdev;

int adc_cali_slop[14] = {1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000};
int adc_cali_offset[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int adc_cali_cal[1] = {0};
int battery_in_data[1] = {0};
int battery_out_data[1] = {0};    
int charging_level_data[1] = {0};
kal_bool g_ADC_Cali = KAL_FALSE;
kal_bool g_ftm_battery_flag = KAL_FALSE;
static int g_wireless_state = 0;

///////////////////////////////////////////////////////////////////////////////////////////
//// Thread related 
///////////////////////////////////////////////////////////////////////////////////////////
#define BAT_MS_TO_NS(x) (x * 1000 * 1000)
static kal_bool bat_thread_timeout = KAL_FALSE;
static kal_bool chr_wake_up_bat = KAL_FALSE;	// charger in/out to wake up battery thread
static kal_bool bat_meter_timeout = KAL_FALSE;
static DEFINE_MUTEX(bat_mutex);
static DEFINE_MUTEX(charger_type_mutex);
static DECLARE_WAIT_QUEUE_HEAD(bat_thread_wq);
static struct hrtimer charger_hv_detect_timer;
static struct task_struct *charger_hv_detect_thread = NULL;
static kal_bool charger_hv_detect_flag = KAL_FALSE;
static DECLARE_WAIT_QUEUE_HEAD(charger_hv_detect_waiter);
static struct hrtimer battery_kthread_timer;


////////////////////////////////////////////////////////////////////////////////
// FOR ANDROID BATTERY SERVICE
////////////////////////////////////////////////////////////////////////////////
/* Dual battery */
int g_status_2nd = POWER_SUPPLY_STATUS_NOT_CHARGING;
int g_capacity_2nd = 50;
int g_present_2nd = 0;

struct wireless_data {
    struct power_supply psy;
    int WIRELESS_ONLINE;    
};

struct ac_data {
    struct power_supply psy;
    int AC_ONLINE;    
};

struct usb_data {
    struct power_supply psy;
    int USB_ONLINE;    
};

struct battery_data {
    struct power_supply psy;
    int BAT_STATUS;
    int BAT_HEALTH;
    int BAT_PRESENT;
    int BAT_TECHNOLOGY;
    int BAT_CAPACITY;
    /* Add for Battery Service*/
    int BAT_batt_vol;
    int BAT_batt_temp;
    /* Add for EM */
    int BAT_TemperatureR;
    int BAT_TempBattVoltage;
    int BAT_InstatVolt;
    int BAT_BatteryAverageCurrent;
    int BAT_BatterySenseVoltage;
    int BAT_ISenseVoltage;
    int BAT_ChargerVoltage;
    /* Dual battery */
    int status_2nd;
    int capacity_2nd;
    int present_2nd;
};

static enum power_supply_property wireless_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property ac_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property usb_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property battery_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_TECHNOLOGY,
    POWER_SUPPLY_PROP_CAPACITY,
    /* Add for Battery Service */
    POWER_SUPPLY_PROP_batt_vol,
    POWER_SUPPLY_PROP_batt_temp,    
    /* Add for EM */
    POWER_SUPPLY_PROP_TemperatureR,
    POWER_SUPPLY_PROP_TempBattVoltage,
    POWER_SUPPLY_PROP_InstatVolt,
    POWER_SUPPLY_PROP_BatteryAverageCurrent,
    POWER_SUPPLY_PROP_BatterySenseVoltage,
    POWER_SUPPLY_PROP_ISenseVoltage,
    POWER_SUPPLY_PROP_ChargerVoltage,
    /* Dual battery */
    POWER_SUPPLY_PROP_status_2nd,
    POWER_SUPPLY_PROP_capacity_2nd,
    POWER_SUPPLY_PROP_present_2nd,
};



///////////////////////////////////////////////////////////////////////////////////////////
//// extern function
///////////////////////////////////////////////////////////////////////////////////////////
//extern void mt_power_off(void);
extern bool mt_usb_is_device(void);
extern void mt_usb_connect(void);
extern void mt_usb_disconnect(void);
//extern int set_rtc_spare_fg_value(int val);

int LGBM_is_call_state( void )
{
    struct file *fd = NULL;
    int retVal = 0;
    int call_state = 0;
	kal_bool state = KAL_FALSE;
 
    fd = filp_open("/sys/devices/platform/mtk-kpd/driver/kpd_call_state", O_RDONLY , 0666);

	if(!IS_ERR(fd))
	{
		retVal = fd->f_op->read(fd,&call_state, 1, &(fd->f_pos));
	    if( retVal < 0 )
	    {
    	    DEBUG_MSG("File open fail ret val = %d\n", retVal);
			state = KAL_FALSE;
	    }
    	else
	    {
    	    DEBUG_MSG("File open success call sate = %d \n", call_state);
			if( call_state == '1' || call_state == '2' )
			{
				state = KAL_TRUE;
			}
			else
			{
				state = KAL_FALSE;
			}
	    }
		filp_close(fd, NULL);
	}
	else
	{
		DEBUG_MSG("File open fail\n");
		state = KAL_FALSE;
	}
	return state;
}
EXPORT_SYMBOL(LGBM_is_call_state);

int read_tbat_value(void)
{
    return BMT_status.temperature;
}

int get_charger_detect_status(void)
{
	kal_bool chr_status;

	battery_charging_control(CHARGING_CMD_GET_CHARGER_DET_STATUS,&chr_status);
    return chr_status;
}


///////////////////////////////////////////////////////////////////////////////////////////
//// PMIC PCHR Related APIs
///////////////////////////////////////////////////////////////////////////////////////////
kal_bool upmu_is_chr_det(void)
{
#if defined(CONFIG_POWER_EXT)
    //return KAL_TRUE;
    return get_charger_detect_status();
#else	
    kal_uint32 tmp32;
    tmp32=get_charger_detect_status();
    if(tmp32 == 0)
    {
    	battery_xlog_printk(BAT_LOG_CRTI, "[upmu_is_chr_det] Charger doesn't exist\n");
        return KAL_FALSE;
    }
    else
    {
    #if 0   //when charger is detected, this function (mt_usb_is_device) is not necessary
        if( mt_usb_is_device() )
        {
        	battery_xlog_printk(BAT_LOG_CRTI, "[upmu_is_chr_det] Charger exist and USB is not host\n");

            return KAL_TRUE;
        }
        else
        {
            battery_xlog_printk(BAT_LOG_CRTI, "[upmu_is_chr_det] Charger exist but USB is host\n");

            return KAL_FALSE;
        }
    #else
        battery_xlog_printk(BAT_LOG_CRTI, "[upmu_is_chr_det] Charger exist\n");
        return KAL_TRUE;
    #endif
    }
#endif	
}
EXPORT_SYMBOL(upmu_is_chr_det);


void wake_up_bat (void)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] wake_up_bat. \r\n");
    
    chr_wake_up_bat = KAL_TRUE;    
    bat_thread_timeout = KAL_TRUE;
	
    wake_up(&bat_thread_wq);
}
EXPORT_SYMBOL(wake_up_bat);


static ssize_t bat_log_write( struct file *filp, const char __user *buff,
                        size_t len, loff_t *data )
{
    if (copy_from_user( &proc_bat_data, buff, len )) {
        battery_xlog_printk(BAT_LOG_FULL, "bat_log_write error.\n");
        return -EFAULT;
    }

    if (proc_bat_data[0] == '1') {
        battery_xlog_printk(BAT_LOG_CRTI, "enable battery driver log system\n");
        Enable_BATDRV_LOG = 1;
    } else if (proc_bat_data[0] == '2') {
        battery_xlog_printk(BAT_LOG_CRTI, "enable battery driver log system:2\n");
        Enable_BATDRV_LOG = 2;    
    } else {
        battery_xlog_printk(BAT_LOG_CRTI, "Disable battery driver log system\n");
        Enable_BATDRV_LOG = 0;
    }
    
    return len;
}

static const struct file_operations bat_proc_fops = { 
    .write = bat_log_write,
};

int init_proc_log(void)
{
    int ret=0;

#if 1
    proc_create("batdrv_log", 0644, NULL, &bat_proc_fops);
    battery_xlog_printk(BAT_LOG_CRTI, "proc_create bat_proc_fops\n");
#else    
    proc_entry = create_proc_entry( "batdrv_log", 0644, NULL );
    
    if (proc_entry == NULL) {
        ret = -ENOMEM;
        battery_xlog_printk(BAT_LOG_FULL, "init_proc_log: Couldn't create proc entry\n");
    } else {
        proc_entry->write_proc = bat_log_write;       
        battery_xlog_printk(BAT_LOG_CRTI, "init_proc_log loaded.\n");
    }
#endif
  
    return ret;
}


static int wireless_get_property(struct power_supply *psy,
    enum power_supply_property psp,
    union power_supply_propval *val)
{
    int ret = 0;
    struct wireless_data *data = container_of(psy, struct wireless_data, psy);    
    battery_xlog_printk(BAT_LOG_CRTI, "[wireless_get_property] start\n");    
    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:                           
        val->intval = data->WIRELESS_ONLINE;
        battery_xlog_printk(BAT_LOG_CRTI, "[wireless_get_property] data->WIRELESS_ONLINE %d\n", data->WIRELESS_ONLINE);    
        break;
    default:
        ret = -EINVAL;
        break;
    }
    return ret;
}

static int ac_get_property(struct power_supply *psy,
    enum power_supply_property psp,
    union power_supply_propval *val)
{
    int ret = 0;
    struct ac_data *data = container_of(psy, struct ac_data, psy);    

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:                           
        val->intval = data->AC_ONLINE;
        break;
    default:
        ret = -EINVAL;
        break;
    }
    return ret;
}

static int usb_get_property(struct power_supply *psy,
    enum power_supply_property psp,
    union power_supply_propval *val)
{
    int ret = 0;
    struct usb_data *data = container_of(psy, struct usb_data, psy);    

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:     
        #if defined(CONFIG_POWER_EXT)
        //#if 0
        data->USB_ONLINE = 1;
        val->intval = data->USB_ONLINE;
        #else
        val->intval = data->USB_ONLINE;
        #endif        
        break;
    default:
        ret = -EINVAL;
        break;
    }
    return ret;
}

static int battery_get_property(struct power_supply *psy,
    enum power_supply_property psp,
    union power_supply_propval *val)
{
    int ret = 0;     
    struct battery_data *data = container_of(psy, struct battery_data, psy);

    switch (psp) {
    case POWER_SUPPLY_PROP_STATUS:
        val->intval = data->BAT_STATUS;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        val->intval = data->BAT_HEALTH;
        break;
    case POWER_SUPPLY_PROP_PRESENT:
        val->intval = data->BAT_PRESENT;
        break;
    case POWER_SUPPLY_PROP_TECHNOLOGY:
        val->intval = data->BAT_TECHNOLOGY;
        break;
    case POWER_SUPPLY_PROP_CAPACITY:
        val->intval = data->BAT_CAPACITY;
        break;        
    case POWER_SUPPLY_PROP_batt_vol:
        val->intval = data->BAT_batt_vol;
        break;
    case POWER_SUPPLY_PROP_batt_temp:
        val->intval = data->BAT_batt_temp;
        break;
    case POWER_SUPPLY_PROP_TemperatureR:
        val->intval = data->BAT_TemperatureR;
        break;    
    case POWER_SUPPLY_PROP_TempBattVoltage:        
        val->intval = data->BAT_TempBattVoltage;
        break;    
    case POWER_SUPPLY_PROP_InstatVolt:
        val->intval = data->BAT_InstatVolt;
        break;    
    case POWER_SUPPLY_PROP_BatteryAverageCurrent:
        val->intval = data->BAT_BatteryAverageCurrent;
        break;    
    case POWER_SUPPLY_PROP_BatterySenseVoltage:
        val->intval = data->BAT_BatterySenseVoltage;
        break;    
    case POWER_SUPPLY_PROP_ISenseVoltage:
        val->intval = data->BAT_ISenseVoltage;
        break;    
    case POWER_SUPPLY_PROP_ChargerVoltage:
        val->intval = data->BAT_ChargerVoltage;
        break;
    /* Dual battery */
    case POWER_SUPPLY_PROP_status_2nd :
        val->intval = data->status_2nd;
        break;
    case POWER_SUPPLY_PROP_capacity_2nd :
        val->intval = data->capacity_2nd;
        break;
    case POWER_SUPPLY_PROP_present_2nd :
        val->intval = data->present_2nd;
        break;

    default:
        ret = -EINVAL;
        break;
    }

    return ret;
}

#if !defined(CONFIG_LGE_MINIABB)
/* wireless_data initialization */
static struct wireless_data wireless_main = {
    .psy = {
    .name = "wireless",
    .type = POWER_SUPPLY_TYPE_WIRELESS,
    .properties = wireless_props,
    .num_properties = ARRAY_SIZE(wireless_props),
    .get_property = wireless_get_property,                
    },
    .WIRELESS_ONLINE = 0,
};
#endif

/* ac_data initialization */
static struct ac_data ac_main = {
    .psy = {
    .name = "ac",
    .type = POWER_SUPPLY_TYPE_MAINS,
    .properties = ac_props,
    .num_properties = ARRAY_SIZE(ac_props),
    .get_property = ac_get_property,                
},
    .AC_ONLINE = 0,
};

/* usb_data initialization */
static struct usb_data usb_main = {
    .psy = {
    .name = "usb",
    .type = POWER_SUPPLY_TYPE_USB,
    .properties = usb_props,
    .num_properties = ARRAY_SIZE(usb_props),
    .get_property = usb_get_property,                
    },
    .USB_ONLINE = 0,
};

/* battery_data initialization */
static struct battery_data battery_main = {
    .psy = {
    .name = "battery",
    .type = POWER_SUPPLY_TYPE_BATTERY,
    .properties = battery_props,
    .num_properties = ARRAY_SIZE(battery_props),
    .get_property = battery_get_property,                
    },
/* CC: modify to have a full power supply status */
#if defined(CONFIG_POWER_EXT)
    .BAT_STATUS = POWER_SUPPLY_STATUS_FULL,    
    .BAT_HEALTH = POWER_SUPPLY_HEALTH_GOOD,
    .BAT_PRESENT = 1,
    .BAT_TECHNOLOGY = POWER_SUPPLY_TECHNOLOGY_LION,
    .BAT_CAPACITY = 100,
    .BAT_batt_vol = 4200,
    .BAT_batt_temp = 22,
    /* Dual battery */
    .status_2nd = POWER_SUPPLY_STATUS_NOT_CHARGING,
    .capacity_2nd = 50,
    .present_2nd = 0,
#else
    .BAT_STATUS = POWER_SUPPLY_STATUS_NOT_CHARGING,    
    .BAT_HEALTH = POWER_SUPPLY_HEALTH_GOOD,
    .BAT_PRESENT = 1,
    .BAT_TECHNOLOGY = POWER_SUPPLY_TECHNOLOGY_LION,
    .BAT_CAPACITY = 50,
    .BAT_batt_vol = 0,
    .BAT_batt_temp = 0,
    /* Dual battery */
    .status_2nd = POWER_SUPPLY_STATUS_NOT_CHARGING,
    .capacity_2nd = 50,
    .present_2nd = 0,
#endif
};


#if !defined(CONFIG_POWER_EXT)
///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Charger_Voltage
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Charger_Voltage(struct device *dev,struct device_attribute *attr, char *buf)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] show_ADC_Charger_Voltage : %d\n", BMT_status.charger_vol);
    return sprintf(buf, "%d\n", BMT_status.charger_vol);
}
static ssize_t store_ADC_Charger_Voltage(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Charger_Voltage, 0664, show_ADC_Charger_Voltage, store_ADC_Charger_Voltage);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_0_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_0_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+0));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_0_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_0_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_0_Slope, 0664, show_ADC_Channel_0_Slope, store_ADC_Channel_0_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_1_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_1_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+1));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_1_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_1_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_1_Slope, 0664, show_ADC_Channel_1_Slope, store_ADC_Channel_1_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_2_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_2_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+2));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_2_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_2_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_2_Slope, 0664, show_ADC_Channel_2_Slope, store_ADC_Channel_2_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_3_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_3_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+3));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_3_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_3_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_3_Slope, 0664, show_ADC_Channel_3_Slope, store_ADC_Channel_3_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_4_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_4_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+4));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_4_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_4_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_4_Slope, 0664, show_ADC_Channel_4_Slope, store_ADC_Channel_4_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_5_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_5_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+5));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_5_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_5_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_5_Slope, 0664, show_ADC_Channel_5_Slope, store_ADC_Channel_5_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_6_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_6_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+6));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_6_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_6_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_6_Slope, 0664, show_ADC_Channel_6_Slope, store_ADC_Channel_6_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_7_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_7_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+7));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_7_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_7_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_7_Slope, 0664, show_ADC_Channel_7_Slope, store_ADC_Channel_7_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_8_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_8_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+8));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_8_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_8_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_8_Slope, 0664, show_ADC_Channel_8_Slope, store_ADC_Channel_8_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_9_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_9_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+9));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_9_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_9_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_9_Slope, 0664, show_ADC_Channel_9_Slope, store_ADC_Channel_9_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_10_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_10_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+10));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_10_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_10_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_10_Slope, 0664, show_ADC_Channel_10_Slope, store_ADC_Channel_10_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_11_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_11_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+11));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_11_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_11_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_11_Slope, 0664, show_ADC_Channel_11_Slope, store_ADC_Channel_11_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_12_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_12_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+12));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_12_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_12_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_12_Slope, 0664, show_ADC_Channel_12_Slope, store_ADC_Channel_12_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_13_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_13_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+13));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_13_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_13_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_13_Slope, 0664, show_ADC_Channel_13_Slope, store_ADC_Channel_13_Slope);


///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_0_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_0_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+0));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_0_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_0_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_0_Offset, 0664, show_ADC_Channel_0_Offset, store_ADC_Channel_0_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_1_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_1_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+1));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_1_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_1_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_1_Offset, 0664, show_ADC_Channel_1_Offset, store_ADC_Channel_1_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_2_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_2_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+2));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_2_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_2_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_2_Offset, 0664, show_ADC_Channel_2_Offset, store_ADC_Channel_2_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_3_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_3_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+3));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_3_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_3_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_3_Offset, 0664, show_ADC_Channel_3_Offset, store_ADC_Channel_3_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_4_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_4_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+4));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_4_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_4_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_4_Offset, 0664, show_ADC_Channel_4_Offset, store_ADC_Channel_4_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_5_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_5_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+5));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_5_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_5_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_5_Offset, 0664, show_ADC_Channel_5_Offset, store_ADC_Channel_5_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_6_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_6_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+6));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_6_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_6_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_6_Offset, 0664, show_ADC_Channel_6_Offset, store_ADC_Channel_6_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_7_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_7_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+7));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_7_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_7_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_7_Offset, 0664, show_ADC_Channel_7_Offset, store_ADC_Channel_7_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_8_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_8_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+8));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_8_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_8_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_8_Offset, 0664, show_ADC_Channel_8_Offset, store_ADC_Channel_8_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_9_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_9_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+9));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_9_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_9_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_9_Offset, 0664, show_ADC_Channel_9_Offset, store_ADC_Channel_9_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_10_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_10_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+10));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_10_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_10_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_10_Offset, 0664, show_ADC_Channel_10_Offset, store_ADC_Channel_10_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_11_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_11_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+11));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_11_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_11_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_11_Offset, 0664, show_ADC_Channel_11_Offset, store_ADC_Channel_11_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_12_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_12_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+12));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_12_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_12_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_12_Offset, 0664, show_ADC_Channel_12_Offset, store_ADC_Channel_12_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_13_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_13_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+13));
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_13_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_13_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_13_Offset, 0664, show_ADC_Channel_13_Offset, store_ADC_Channel_13_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_Is_Calibration
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_Is_Calibration(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=2;
    ret_value = g_ADC_Cali;
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] ADC_Channel_Is_Calibration : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_Is_Calibration(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_Is_Calibration, 0664, show_ADC_Channel_Is_Calibration, store_ADC_Channel_Is_Calibration);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : Power_On_Voltage
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_Power_On_Voltage(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = 3400;
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Power_On_Voltage : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_Power_On_Voltage(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(Power_On_Voltage, 0664, show_Power_On_Voltage, store_Power_On_Voltage);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : Power_Off_Voltage
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_Power_Off_Voltage(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = 3400;
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Power_Off_Voltage : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_Power_Off_Voltage(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(Power_Off_Voltage, 0664, show_Power_Off_Voltage, store_Power_Off_Voltage);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : Charger_TopOff_Value
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_Charger_TopOff_Value(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = 4110;
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Charger_TopOff_Value : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_Charger_TopOff_Value(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(Charger_TopOff_Value, 0664, show_Charger_TopOff_Value, store_Charger_TopOff_Value);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : FG_Battery_CurrentConsumption
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_FG_Battery_CurrentConsumption(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=8888;
    ret_value = battery_meter_get_battery_current();    
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] FG_Battery_CurrentConsumption : %d/10 mA\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_FG_Battery_CurrentConsumption(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(FG_Battery_CurrentConsumption, 0664, show_FG_Battery_CurrentConsumption, store_FG_Battery_CurrentConsumption);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : FG_SW_CoulombCounter
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_FG_SW_CoulombCounter(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_int32 ret_value=7777;
    ret_value = battery_meter_get_car();
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] FG_SW_CoulombCounter : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_FG_SW_CoulombCounter(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(FG_SW_CoulombCounter, 0664, show_FG_SW_CoulombCounter, store_FG_SW_CoulombCounter);


static ssize_t show_Charging_CallState(struct device *dev,struct device_attribute *attr, char *buf)
{
    battery_xlog_printk(BAT_LOG_CRTI, "call state = %d\n",g_call_state);    
    return sprintf(buf, "%u\n", g_call_state);
}
static ssize_t store_Charging_CallState(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	sscanf(buf, "%u", &g_call_state);
    battery_xlog_printk(BAT_LOG_CRTI, "call state = %d\n",g_call_state);    
    return size;
}
static DEVICE_ATTR(Charging_CallState, 0664, show_Charging_CallState, store_Charging_CallState);

#if defined ( LGE_BSP_LGBM )
/***********          LGBM ATCMD                   ****************/

static ssize_t show_LGBM_AtCmdUsbidadc(struct device *dev,struct device_attribute *attr, char *buf)
{
	LGBM_ReadUsbCableId();
	return sprintf(buf, "%d,%d",g_AtCmdUsbAdc,g_AtCmdUsbId);
}

static DEVICE_ATTR(LGBM_AtCmdUsbidadc, 0664, show_LGBM_AtCmdUsbidadc, NULL);

static ssize_t show_LGBM_AtCmdCharge(struct device *dev,struct device_attribute *attr, char *buf)
{
    /* return factory charge mode */
    return sprintf(buf, "%d\n", g_AtCmdChargeMode);
}
static ssize_t store_LGBM_AtCmdCharge(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    /* set factory charge mode */
    if(buf != NULL && size != 0)
    {
        if(buf[0] == '1')
        {
            g_AtCmdChargeMode = 1;
        }
        else
        {
            g_AtCmdChargeMode = 0;
        }
    }
    return size;
}
static DEVICE_ATTR(LGBM_AtCmdCharge, 0664, show_LGBM_AtCmdCharge, store_LGBM_AtCmdCharge);

static ssize_t show_LGBM_AtCmdBatl(struct device *dev,struct device_attribute *attr, char *buf)
{
    /* return battery voltage */

    int batVolt = 0;

    if( g_AtCmdChargeMode == 1 )
    {
        batVolt = LGBM_ReadBatVoltAdc();
    }
    else
    {
		batVolt = g_pAtCmdBatVital->batVolt;
    }

    return sprintf(buf, "%d\n", batVolt);
}
static DEVICE_ATTR(LGBM_AtCmdBatl, 0664, show_LGBM_AtCmdBatl, NULL);

static ssize_t show_LGBM_AtCmdBatmp(struct device *dev,struct device_attribute *attr, char *buf)
{
    /* return battery temperature */
    kal_int32 batTemp = 0;

    LGBmVital batVital = {0};

    if( g_AtCmdChargeMode == 1 )
    {
        LGBM_ReadBatVital(&batVital);
        batTemp = batVital.batTemp;

    }
    else
    {
        batTemp = g_pAtCmdBatVital->batTemp;
    }

    return sprintf(buf, "%d\n", batTemp);

}
static DEVICE_ATTR(LGBM_AtCmdBatmp, 0664, show_LGBM_AtCmdBatmp, NULL);

static ssize_t show_LGBM_AtCmdChcomp(struct device *dev,struct device_attribute *attr, char *buf)
{
    int isChargeComplete = 0;
    int batVolt = {0};

    /* return charge complete state */
    if( g_AtCmdChargeMode ==1 )
    {
        batVolt = LGBM_ReadBatVoltAdc();
        if( batVolt > 4100 ) //temporary value
        {
            isChargeComplete = 1;
        }
    }
	
    else
    {
        if( g_AtCmdBatFullUI == POWER_SUPPLY_STATUS_FULL )
        {
            isChargeComplete = 1;
        }
    }

    return sprintf(buf, "%d\n", isChargeComplete);

}
static DEVICE_ATTR(LGBM_AtCmdChcomp, 0664, show_LGBM_AtCmdChcomp, NULL);

static ssize_t show_LGBM_AtCmdFuelval(struct device *dev,struct device_attribute *attr, char *buf)
{
    /* return battery soc */
    int batSoc = 0;
#if defined(TARGET_S4)	
	int nVbatt;
    long ret;

    nVbatt = LGBM_ReadBatVoltAdc();
    GetSOC(SOC_CHR, nVbatt, ret);
	batSoc=(int)(ret/1000);
	if(batSoc>=100)
		batSoc=100;
    LGBM_LOG("show_LGBM_AtCmdFuelval  = %d \n",batSoc);
#endif	
/*              // using fuelval path  "/sys/class/power_supply/battery/capacity" it is temporary path,  it will be change

    LGBmVital batVital = {0};



    if( g_AtCmdChargeMode == 1 )
    {
        LGBM_ReadBatVital(&batVital);
        batSoc = LGBM_GetSocByOcv(batt_id_check, batVital.batTemp, batVital.batVolt);
    }
    else
    {
        batSoc = g_pAtCmdBmData->curSoc;
    }
*/
    return sprintf(buf, "%d\n", batSoc);
}
static DEVICE_ATTR(LGBM_AtCmdFuelval, 0664, show_LGBM_AtCmdFuelval, NULL);

static ssize_t show_LGBM_UpdateBatStateFlag(struct device *dev,struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_updateBatStateFlag);
}
static DEVICE_ATTR(LGBM_UpdateBatStateFlag, 0664, show_LGBM_UpdateBatStateFlag, NULL);

#if defined (TARGET_S4)
static ssize_t show_LGBM_BatteryID(struct device *dev,struct device_attribute *attr, char *buf)
{
    if( strcmp(g_batt_id_info, "unknown") == 0 )
    {
        return sprintf(buf, "%d\n", 0);
    }
    else
    {
        return sprintf(buf, "%d\n", 1);
    }
}
static DEVICE_ATTR(LGBM_BatteryID, 0664, show_LGBM_BatteryID, NULL);
#endif

#endif

static void mt_battery_update_EM(struct battery_data *bat_data)
{
	bat_data->BAT_CAPACITY = BMT_status.UI_SOC;
    bat_data->BAT_TemperatureR=BMT_status.temperatureR;	//API
    bat_data->BAT_TempBattVoltage=BMT_status.temperatureV; // API
    bat_data->BAT_InstatVolt=BMT_status.bat_vol;	//VBAT
    bat_data->BAT_BatteryAverageCurrent=BMT_status.ICharging;	
    bat_data->BAT_BatterySenseVoltage=BMT_status.bat_vol;	
    bat_data->BAT_ISenseVoltage=BMT_status.Vsense;	// API
    bat_data->BAT_ChargerVoltage=BMT_status.charger_vol;	
    /* Dual battery */
    bat_data->status_2nd = g_status_2nd;
    bat_data->capacity_2nd = g_capacity_2nd;
    bat_data->present_2nd = g_present_2nd;
	battery_xlog_printk(BAT_LOG_FULL, "Power/Battery", "status_2nd = %d, capacity_2nd = %d, present_2nd = %d\n", bat_data->status_2nd, bat_data->capacity_2nd, bat_data->present_2nd);
	if((BMT_status.UI_SOC == 100) && (BMT_status.charger_exist == KAL_TRUE))
		 bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_FULL;

	#ifdef CONFIG_MTK_DISABLE_POWER_ON_OFF_VOLTAGE_LIMITATION
	if(bat_data->BAT_CAPACITY <=0)
		bat_data->BAT_CAPACITY = 1;
	
	battery_xlog_printk(BAT_LOG_CRTI, "BAT_CAPACITY=1, due to define CONFIG_MTK_DISABLE_POWER_ON_OFF_VOLTAGE_LIMITATION\r\n");
	#endif
}


static kal_bool mt_battery_100Percent_tracking_check(void)
{
	kal_bool resetBatteryMeter = KAL_FALSE;
	
#if defined(MTK_JEITA_STANDARD_SUPPORT)
	kal_uint32 cust_sync_time = CUST_SOC_JEITA_SYNC_TIME;
	static kal_uint32 timer_counter = (CUST_SOC_JEITA_SYNC_TIME/BAT_TASK_PERIOD);
#else
	kal_uint32 cust_sync_time = ONEHUNDRED_PERCENT_TRACKING_TIME;
	static kal_uint32 timer_counter = (ONEHUNDRED_PERCENT_TRACKING_TIME/BAT_TASK_PERIOD);
#endif

	 if(BMT_status.bat_full == KAL_TRUE)	// charging full first, UI tracking to 100%
	 {
	 	if(BMT_status.UI_SOC >= 100)
		{
			BMT_status.UI_SOC = 100;
			
			if((g_charging_full_reset_bat_meter == KAL_TRUE) && (BMT_status.bat_charging_state == CHR_BATFULL))
			{
				resetBatteryMeter = KAL_TRUE;
				g_charging_full_reset_bat_meter = KAL_FALSE;
			}
			else
			{
				resetBatteryMeter = KAL_FALSE;
			}	
		}
		else
		{
		        //increase UI percentage every xxs
                        if(timer_counter >= (cust_sync_time/BAT_TASK_PERIOD))
                        {
                                timer_counter=1;
                                BMT_status.UI_SOC++;				   
                        }
                        else
                        {
                                timer_counter++;

                                return resetBatteryMeter;
                         }	
	 	
                         resetBatteryMeter = KAL_TRUE;
	        }
		
		battery_xlog_printk(BAT_LOG_CRTI, "[Battery] mt_battery_100percent_tracking(), Charging full first UI(%d), reset(%d) \r\n",
			BMT_status.UI_SOC,resetBatteryMeter);
	 }
	 else	
	 {
	 	// charging is not full,  UI keep 99% if reaching 100%,  
				
		if(BMT_status.UI_SOC>=99)
        {
            BMT_status.UI_SOC=99;
			resetBatteryMeter = KAL_FALSE;
       		
			battery_xlog_printk(BAT_LOG_CRTI, "[Battery] mt_battery_100percent_tracking(), UI full first, keep (%d) \r\n", BMT_status.UI_SOC);
 		}

		timer_counter = (cust_sync_time/BAT_TASK_PERIOD);

	 }
	 
	 return resetBatteryMeter;	 
}


static kal_bool mt_battery_nPercent_tracking_check(void)
{
	kal_bool resetBatteryMeter = KAL_FALSE;
#if defined(SOC_BY_HW_FG)
	static kal_uint32 timer_counter = (NPERCENT_TRACKING_TIME/BAT_TASK_PERIOD);	

    if (BMT_status.nPrecent_UI_SOC_check_point == 0)
        return KAL_FALSE;
			
	// fuel gauge ZCV < 15%, but UI > 15%,  15% can be customized 
	if ( (BMT_status.ZCV <= BMT_status.nPercent_ZCV) &&(BMT_status.UI_SOC > BMT_status.nPrecent_UI_SOC_check_point) )
	{
		  if(timer_counter == (NPERCENT_TRACKING_TIME/BAT_TASK_PERIOD))	// every x sec decrease UI percentage
		  {
		  	 BMT_status.UI_SOC--;
			 timer_counter=1;
		  }
		  else
		  {
			 timer_counter++;
			 return resetBatteryMeter;
		  }
			  
  		  resetBatteryMeter = KAL_TRUE;

		   battery_xlog_printk(BAT_LOG_CRTI, "[Battery]mt_battery_nPercent_tracking_check(), ZCV(%d) <= BMT_status.nPercent_ZCV(%d), UI_SOC=%d., tracking UI_SOC=%d \r\n", 
                BMT_status.ZCV, BMT_status.nPercent_ZCV, BMT_status.UI_SOC, BMT_status.nPrecent_UI_SOC_check_point);
	}
	else if ( (BMT_status.ZCV > BMT_status.nPercent_ZCV)&&(BMT_status.UI_SOC==BMT_status.nPrecent_UI_SOC_check_point) )
    {
    	//UI less than 15 , but fuel gague is more than 15, hold UI 15%
    	timer_counter=(NPERCENT_TRACKING_TIME/BAT_TASK_PERIOD);
		resetBatteryMeter = KAL_TRUE;
		
        battery_xlog_printk(BAT_LOG_CRTI, "[Battery]mt_battery_nPercent_tracking_check() ZCV(%d) > BMT_status.nPercent_ZCV(%d) and UI SOC (%d), then keep %d. \r\n", 
            BMT_status.ZCV, BMT_status.nPercent_ZCV, BMT_status.UI_SOC, BMT_status.nPrecent_UI_SOC_check_point);
 	}
	else
	{
		timer_counter=(NPERCENT_TRACKING_TIME/BAT_TASK_PERIOD);
	}
#endif	
	return resetBatteryMeter;

}

static kal_bool mt_battery_0Percent_tracking_check(void)
{
	kal_bool resetBatteryMeter = KAL_TRUE;
	 
	if(BMT_status.UI_SOC <= 0)
	{
		BMT_status.UI_SOC=0;
	}	
    else
    {
        if (BMT_status.bat_vol > SYSTEM_OFF_VOLTAGE && BMT_status.UI_SOC > 1) {
            BMT_status.UI_SOC--;
        } else if (BMT_status.bat_vol <= SYSTEM_OFF_VOLTAGE) {
    	BMT_status.UI_SOC--;
    }
    }
	
    battery_xlog_printk(BAT_LOG_CRTI, "[Battery] mt_battery_0Percent_tracking_check(), VBAT < %d UI_SOC = (%d)\r\n", SYSTEM_OFF_VOLTAGE, BMT_status.UI_SOC);                

	return resetBatteryMeter;	
}


static void mt_battery_Sync_UI_Percentage_to_Real(void)
{
	static kal_uint32 timer_counter = 0;	
	
	if( (BMT_status.UI_SOC > BMT_status.SOC) && ((BMT_status.UI_SOC!=1)) )
    {   
        //reduce after xxs
		if(timer_counter == (SYNC_TO_REAL_TRACKING_TIME/BAT_TASK_PERIOD))
		{
			BMT_status.UI_SOC--;
			timer_counter = 0;
		}
		else
		{
			timer_counter ++;
		}

		battery_xlog_printk(BAT_LOG_CRTI, "Sync UI percentage to Real one, BMT_status.UI_SOC=%d, BMT_status.SOC=%d, counter = %d\r\n", 
                      BMT_status.UI_SOC, BMT_status.SOC, timer_counter);
	}
	else
	{
		timer_counter = 0;
	    BMT_status.UI_SOC = BMT_status.SOC;
    }

	if(BMT_status.UI_SOC <= 0 )
	{
		BMT_status.UI_SOC=1;
		battery_xlog_printk(BAT_LOG_CRTI, "[Battery]UI_SOC get 0 first (%d)\r\n", BMT_status.UI_SOC);
	}
}

static void battery_update(struct battery_data *bat_data)
{
    struct power_supply *bat_psy = &bat_data->psy;
	kal_bool resetBatteryMeter = KAL_FALSE;

    bat_data->BAT_TECHNOLOGY = POWER_SUPPLY_TECHNOLOGY_LION;
    bat_data->BAT_HEALTH = POWER_SUPPLY_HEALTH_GOOD;
    bat_data->BAT_batt_vol = BMT_status.bat_vol;
    bat_data->BAT_batt_temp= BMT_status.temperature * 10;
	bat_data->BAT_PRESENT = BMT_status.bat_exist;

    if( (BMT_status.charger_exist == KAL_TRUE) && (BMT_status.bat_charging_state != CHR_ERROR) )
    {     
        if ( BMT_status.bat_exist )  /* charging */               
        {
            if (BMT_status.bat_vol <= V_0PERCENT_TRACKING) {
                resetBatteryMeter = mt_battery_0Percent_tracking_check();
            } else {
        	resetBatteryMeter = mt_battery_100Percent_tracking_check();
            }
           	
			bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_CHARGING;  
        }
        else	 /* No Battery, Only Charger */
        {
            bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_UNKNOWN;
           BMT_status.UI_SOC = 0;
        }
        
    }
    else	/* Only Battery */
    {
        bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_NOT_CHARGING;
        if (BMT_status.bat_vol <= V_0PERCENT_TRACKING)
    		resetBatteryMeter = mt_battery_0Percent_tracking_check();
		else 
			resetBatteryMeter = mt_battery_nPercent_tracking_check();
    }    

	if(resetBatteryMeter == KAL_TRUE)
  	{
		battery_meter_reset();
  	}
    else
    {
    	if(bat_is_recharging_phase() == KAL_TRUE)
		{
			BMT_status.UI_SOC = 100;
			battery_xlog_printk(BAT_LOG_CRTI, "[Battery_Recharging_phase] Keep UI as 100. BMT_status.UI_SOC=%d, BMT_status.SOC=%ld\r\n", 
                        BMT_status.UI_SOC, BMT_status.SOC);
		}
		else
		{
        	mt_battery_Sync_UI_Percentage_to_Real();
		}	
    }

	battery_xlog_printk(BAT_LOG_CRTI, "UI_SOC=(%d), resetBatteryMeter=(%d)\n", BMT_status.UI_SOC,resetBatteryMeter);	

	//restore battery UI capacity to rtc
	if (BMT_status.UI_SOC <= 1) {
		set_rtc_spare_fg_value(1);
	}
	else {
		set_rtc_spare_fg_value(BMT_status.UI_SOC);
	}
	
	mt_battery_update_EM(bat_data);
		
    power_supply_changed(bat_psy);    
}

void update_charger_info(int wireless_state)
{
    #if defined(CONFIG_POWER_VERIFY)
    battery_xlog_printk(BAT_LOG_CRTI, "[update_charger_info] no support\n");
    #else
    g_wireless_state = wireless_state;
    battery_xlog_printk(BAT_LOG_CRTI, "[update_charger_info] get wireless_state=%d\n",
        wireless_state);

    wake_up_bat();
    #endif
}

static void wireless_update(struct wireless_data *wireless_data)
{
    struct power_supply *wireless_psy = &wireless_data->psy;

    if( BMT_status.charger_exist == KAL_TRUE || g_wireless_state)
    {         
        if ( (BMT_status.charger_type == WIRELESS_CHARGER) || g_wireless_state)
        {
            wireless_data->WIRELESS_ONLINE = 1;        
            wireless_psy->type = POWER_SUPPLY_TYPE_WIRELESS;
            battery_xlog_printk(BAT_LOG_CRTI, "[wireless_update]\n");
        }
    }
    else
    {
        wireless_data->WIRELESS_ONLINE = 0;        
    }
    
    power_supply_changed(wireless_psy);    
}

static void ac_update(struct ac_data *ac_data)
{
    struct power_supply *ac_psy = &ac_data->psy;

    if( BMT_status.charger_exist == KAL_TRUE )
    {         
        if ( (BMT_status.charger_type == NONSTANDARD_CHARGER) || 
             (BMT_status.charger_type == STANDARD_CHARGER)        )
        {
            ac_data->AC_ONLINE = 1;        
            ac_psy->type = POWER_SUPPLY_TYPE_MAINS;
        }
    }
    else
    {
        ac_data->AC_ONLINE = 0;        
    }

    power_supply_changed(ac_psy);    
}

static void usb_update(struct usb_data *usb_data)
{
    struct power_supply *usb_psy = &usb_data->psy;

    if( BMT_status.charger_exist == KAL_TRUE )        
    {
        if ( (BMT_status.charger_type == STANDARD_HOST) ||
             (BMT_status.charger_type == CHARGING_HOST)        )
        {
            usb_data->USB_ONLINE = 1;            
            usb_psy->type = POWER_SUPPLY_TYPE_USB;            
        }
    }
    else
    {
        usb_data->USB_ONLINE = 0;
    }   

    power_supply_changed(usb_psy); 
}

#endif

//                                                         
#if defined ( LGE_BSP_LGBM ) 
static ssize_t show_usb_cable(struct device *dev,struct device_attribute *attr, char *buf)
{
    LGBM_LOG("SHOW_USB_CABLE : g_lgbmBootUsbCableId = %d\n", g_lgbmBootUsbCableId);
    return sprintf(buf, "%d\n", g_lgbmBootUsbCableId);
}
static DEVICE_ATTR(usb_cable, S_IRUGO, show_usb_cable, NULL);

//                                                                             
static ssize_t show_pseudo_batt(struct device *dev,struct device_attribute *attr, char *buf)
{
#if defined ( LGE_BSP_LGBM ) //                                              
    LGBM_LOG("Show Fake Battery Mode = %d\n", fake_batt_mode);
#endif
    return sprintf(buf, "%d\n", fake_batt_mode);
}

static ssize_t store_pseudo_batt(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    if(buf != NULL && size != 0)
    {
        if(buf[0] == '1')
        {
            fake_batt_mode = 1 ;
        }
        else
        {
            fake_batt_mode = 0 ;
        }
    }
#if defined ( LGE_BSP_LGBM ) //                                              
    LGBM_LOG("Store Fake Battery Mode = %d\n", fake_batt_mode);
#endif
     return size;
}

static DEVICE_ATTR(pseudo_batt, 0664, show_pseudo_batt, store_pseudo_batt);
//                                                                             

static ssize_t show_LGBM_AtCmdChargingModeOff(struct device *dev,struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", g_AtCmdChargingModeOff);
}
static ssize_t store_LGBM_AtCmdChargingModeOff(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    if(buf != NULL && size != 0)
    {
        if(buf[0] == '1')
        {
            g_AtCmdChargingModeOff = 1;
		#if defined(CONFIG_LGE_MINIABB)
			set_charger_stop_mode();
		#else
			BQ25040_SetChargingMode(BQ25040_CM_OFF);
		#endif
        }
    }
    return size;
}
static DEVICE_ATTR(LGBM_AtCmdChargingModeOff, 0664, show_LGBM_AtCmdChargingModeOff, store_LGBM_AtCmdChargingModeOff);
static ssize_t show_LGBM_AtCmdBattExist(struct device *dev,struct device_attribute *attr, char *buf)
{
    int batExist = 0;
    batExist = g_pAtCmdBmData->batExist;
    return sprintf(buf, "%d\n", batExist);
}
static DEVICE_ATTR(LGBM_AtCmdBattExist, 0664, show_LGBM_AtCmdBattExist, NULL);
#endif
//                                                         
///////////////////////////////////////////////////////////////////////////////////////////
//// Battery Temprature Parameters and functions
///////////////////////////////////////////////////////////////////////////////////////////
kal_bool pmic_chrdet_status(void)
{
    if( upmu_is_chr_det() == KAL_TRUE )    
    {
        return KAL_TRUE;
    }
    else
    {
        battery_xlog_printk(BAT_LOG_CRTI, "[pmic_chrdet_status] No charger\r\n");
        return KAL_FALSE;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
//// Pulse Charging Algorithm 
///////////////////////////////////////////////////////////////////////////////////////////
kal_bool bat_is_charger_exist(void)
{
	return get_charger_detect_status();
}


kal_bool bat_is_charging_full(void)
{
	if((BMT_status.bat_full == KAL_TRUE) && (BMT_status.bat_in_recharging_state == KAL_FALSE))
		return KAL_TRUE;
	else
		return KAL_FALSE;
}


kal_uint32 bat_get_ui_percentage(void)
{
	//  for plugging out charger in recharge phase, using SOC as UI_SOC
	if(chr_wake_up_bat == KAL_TRUE)
		return BMT_status.SOC;
	else
		return BMT_status.UI_SOC;
}

/* Full state --> recharge voltage --> full state */
kal_uint32 bat_is_recharging_phase(void)
{
	return (BMT_status.bat_in_recharging_state || BMT_status.bat_full == KAL_TRUE);
}


int get_bat_charging_current_level(void)
{
    CHR_CURRENT_ENUM charging_current;

	battery_charging_control(CHARGING_CMD_GET_CURRENT,&charging_current);

	return charging_current;
}

#if defined(MTK_TEMPERATURE_RECHARGE_SUPPORT)
PMU_STATUS do_batt_temp_state_machine(void)
{
	if (BMT_status.temperature == ERR_CHARGE_TEMPERATURE)
	{
		 return PMU_STATUS_FAIL;
	}
#ifdef BAT_LOW_TEMP_PROTECT_ENABLE
	if (BMT_status.temperature < MIN_CHARGE_TEMPERATURE)
	{
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Battery Under Temperature or NTC fail !!\n\r");
		g_batt_temp_status = TEMP_POS_LOW;
		return PMU_STATUS_FAIL;       
	}
	else if(g_batt_temp_status == TEMP_POS_LOW)
	{
		if (BMT_status.temperature >= MIN_CHARGE_TEMPERATURE_PLUS_X_DEGREE) {
			battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Battery Temperature raise from %d to %d(%d), allow charging!!\n\r", 
				MIN_CHARGE_TEMPERATURE,BMT_status.temperature, MIN_CHARGE_TEMPERATURE_PLUS_X_DEGREE); 
			g_batt_temp_status = TEMP_POS_NORMAL;
			BMT_status.bat_charging_state=CHR_PRE;
			return PMU_STATUS_OK;
		} else {
			return PMU_STATUS_FAIL;
		}
	}
	else
#endif
	if (BMT_status.temperature >= MAX_CHARGE_TEMPERATURE) 
	{
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Battery Over Temperature !!\n\r"); 
		g_batt_temp_status =TEMP_POS_HIGH;
		return PMU_STATUS_FAIL; 
	}
	else if(g_batt_temp_status == TEMP_POS_HIGH)
	{
		if (BMT_status.temperature < MAX_CHARGE_TEMPERATURE_MINUS_X_DEGREE) {             
			battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Battery Temperature down from %d to %d(%d), allow charging!!\n\r", 
				MAX_CHARGE_TEMPERATURE,BMT_status.temperature,MAX_CHARGE_TEMPERATURE_MINUS_X_DEGREE); 
			g_batt_temp_status = TEMP_POS_NORMAL;
			BMT_status.bat_charging_state=CHR_PRE;
			return PMU_STATUS_OK;
		} else {
			return PMU_STATUS_FAIL;
		}
	}
	else
	{
		g_batt_temp_status = TEMP_POS_NORMAL;
	}
	return PMU_STATUS_OK;
}
#endif

unsigned long BAT_Get_Battery_Voltage(int polling_mode)
{
    unsigned long ret_val = 0;

#if defined(CONFIG_POWER_EXT)
	ret_val = 4000;
#else	
    ret_val=battery_meter_get_battery_voltage();
#endif    

    return ret_val;
}


static void mt_battery_average_method_init(kal_uint32 *bufferdata, kal_uint32 data, kal_int32 *sum)
{
	kal_uint32 i;
	static kal_bool batteryBufferFirst = KAL_TRUE;
	static kal_bool previous_charger_exist = KAL_FALSE;
	static kal_bool previous_in_recharge_state = KAL_FALSE;
	static kal_uint8 index=0;

	/* reset charging current window while plug in/out {*/
	if(BMT_status.charger_exist == KAL_TRUE)
	{
		if(previous_charger_exist == KAL_FALSE)
		{
			batteryBufferFirst = KAL_TRUE;
			previous_charger_exist = KAL_TRUE;
			if (BMT_status.charger_type == STANDARD_CHARGER) {
				data = AC_CHARGER_CURRENT / 100;
			} else if (BMT_status.charger_type == CHARGING_HOST) {
				data = CHARGING_HOST_CHARGER_CURRENT / 100;
			} else if (BMT_status.charger_type == NONSTANDARD_CHARGER)
				data = NON_STD_AC_CHARGER_CURRENT / 100;		//mA
			else	// USB
				data = USB_CHARGER_CURRENT / 100;		//mA
		}		
		else if((previous_in_recharge_state == KAL_FALSE) && (BMT_status.bat_in_recharging_state == KAL_TRUE))
		{
			batteryBufferFirst = KAL_TRUE;
			if (BMT_status.charger_type == STANDARD_CHARGER) {
				data = AC_CHARGER_CURRENT / 100;
			} else if (BMT_status.charger_type == CHARGING_HOST) {
				data = CHARGING_HOST_CHARGER_CURRENT / 100;
			} else if (BMT_status.charger_type == NONSTANDARD_CHARGER)
				data = NON_STD_AC_CHARGER_CURRENT / 100;		//mA
			else	// USB
				data = USB_CHARGER_CURRENT / 100;		//mA
		}

		previous_in_recharge_state = BMT_status.bat_in_recharging_state;
	}
	else
	{
		if(previous_charger_exist == KAL_TRUE)
		{
			batteryBufferFirst = KAL_TRUE;
			previous_charger_exist = KAL_FALSE;
			data = 0;
		}
	}
	/* reset charging current window while plug in/out }*/

	battery_xlog_printk(BAT_LOG_FULL, "batteryBufferFirst =%d, data= (%d) \n", batteryBufferFirst, data);
	
	if(batteryBufferFirst == KAL_TRUE)
	{
		for (i=0; i<BATTERY_AVERAGE_SIZE; i++)
		{
            bufferdata[i] = data;            
		}

		*sum = data * BATTERY_AVERAGE_SIZE;
	}

	index++;
	if(index >= BATTERY_AVERAGE_DATA_NUMBER)
	{
		index = BATTERY_AVERAGE_DATA_NUMBER;
       	batteryBufferFirst = KAL_FALSE;	
	}	
}


static kal_uint32 mt_battery_average_method(kal_uint32 *bufferdata, kal_uint32 data, kal_int32 *sum, kal_uint8 batteryIndex)
{
	kal_uint32 avgdata;

	mt_battery_average_method_init(bufferdata, data, sum);

	*sum -=	bufferdata[batteryIndex];
	*sum +=  data;
	bufferdata[batteryIndex] = data;
    avgdata = (*sum)/BATTERY_AVERAGE_SIZE;

	battery_xlog_printk(BAT_LOG_FULL, "bufferdata[%d]= (%d) \n", batteryIndex,bufferdata[batteryIndex]);
	return avgdata;
}

void mt_battery_GetBatteryData(void)
{ 
	kal_uint32 bat_vol, charger_vol, Vsense, ZCV; 
	kal_int32 ICharging, temperature, temperatureR, temperatureV, SOC;
	static kal_int32 bat_sum, icharging_sum, temperature_sum;
	static kal_int32 batteryVoltageBuffer[BATTERY_AVERAGE_SIZE];
	static kal_int32 batteryCurrentBuffer[BATTERY_AVERAGE_SIZE];
	static kal_int32 batteryTempBuffer[BATTERY_AVERAGE_SIZE];
	static kal_uint8 batteryIndex = 0;
	static kal_int32 previous_SOC = -1;
	
	bat_vol = battery_meter_get_battery_voltage();
	Vsense = battery_meter_get_VSense();
	ICharging = battery_meter_get_charging_current();
	charger_vol = battery_meter_get_charger_voltage();
	temperature = battery_meter_get_battery_temperature();
	temperatureV = battery_meter_get_tempV();
	temperatureR = battery_meter_get_tempR(temperatureV);
			
	if(bat_meter_timeout == KAL_TRUE)
	{
		SOC = battery_meter_get_battery_percentage();
		bat_meter_timeout = KAL_FALSE;
	}
	else
	{
		if (previous_SOC == -1)
			SOC = battery_meter_get_battery_percentage();
		else
			SOC = previous_SOC;		
	}
    
	ZCV = battery_meter_get_battery_zcv();

	BMT_status.ICharging = mt_battery_average_method(&batteryCurrentBuffer[0],ICharging, &icharging_sum, batteryIndex);	
	if(previous_SOC == -1 && bat_vol <= SYSTEM_OFF_VOLTAGE)
		BMT_status.bat_vol = mt_battery_average_method(&batteryVoltageBuffer[0],ZCV, &bat_sum, batteryIndex);
	else
	BMT_status.bat_vol = mt_battery_average_method(&batteryVoltageBuffer[0],bat_vol, &bat_sum, batteryIndex);
	BMT_status.temperature = mt_battery_average_method(&batteryTempBuffer[0],temperature, &temperature_sum, batteryIndex);
	BMT_status.Vsense = Vsense;
	BMT_status.charger_vol = charger_vol;	
	BMT_status.temperatureV = temperatureV;
	BMT_status.temperatureR = temperatureR;
	BMT_status.SOC = SOC;	
	BMT_status.ZCV = ZCV;
	
	if(BMT_status.charger_exist == KAL_FALSE)
	{
		if(BMT_status.SOC > previous_SOC && previous_SOC >= 0)
			BMT_status.SOC = previous_SOC;
	}

	previous_SOC = BMT_status.SOC;
	
	batteryIndex++;
    if (batteryIndex >= BATTERY_AVERAGE_SIZE)
        batteryIndex = 0;
	
	g_battery_soc_ready = KAL_TRUE;
	
	battery_xlog_printk(BAT_LOG_CRTI, "AvgVbat=(%d),bat_vol=(%d),AvgI=(%d),I=(%d),VChr=(%d),AvgT=(%d),T=(%d),pre_SOC=(%d),SOC=(%d),ZCV=(%d)\n",
		BMT_status.bat_vol,bat_vol,BMT_status.ICharging,ICharging,BMT_status.charger_vol,BMT_status.temperature,temperature,previous_SOC,BMT_status.SOC,BMT_status.ZCV);	


}


static PMU_STATUS mt_battery_CheckBatteryTemp(void)
{	
	PMU_STATUS status = PMU_STATUS_OK;
	
#if defined(MTK_JEITA_STANDARD_SUPPORT)

    battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] support JEITA, temperature=%d\n", BMT_status.temperature);            

    if( do_jeita_state_machine() == PMU_STATUS_FAIL)
    {
        battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] JEITA : fail\n");
        status = PMU_STATUS_FAIL;
    }

#else

#if defined(MTK_TEMPERATURE_RECHARGE_SUPPORT)
	if (do_batt_temp_state_machine() == PMU_STATUS_FAIL)
	{
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Batt temp check : fail\n");
		status = PMU_STATUS_FAIL;
	}
#else
    #ifdef BAT_LOW_TEMP_PROTECT_ENABLE
    if ((BMT_status.temperature < MIN_CHARGE_TEMPERATURE) || (BMT_status.temperature == ERR_CHARGE_TEMPERATURE))
    {
        battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Battery Under Temperature or NTC fail !!\n\r");                
        status = PMU_STATUS_FAIL;       
    }
    #endif                
    if (BMT_status.temperature >= MAX_CHARGE_TEMPERATURE)
    {
        battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Battery Over Temperature !!\n\r");                
        status = PMU_STATUS_FAIL;       
    }    
#endif

#endif

	return status;
}


static PMU_STATUS mt_battery_CheckChargerVoltage(void)
{
	PMU_STATUS status = PMU_STATUS_OK;
	
	if( BMT_status.charger_exist == KAL_TRUE)
    {
        #if (V_CHARGER_ENABLE == 1)
        if (BMT_status.charger_vol <= V_CHARGER_MIN )
        {
           battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY]Charger under voltage!!\r\n");                    
            BMT_status.bat_charging_state = CHR_ERROR;
            status = PMU_STATUS_FAIL;        
        }
        #endif        
        if ( BMT_status.charger_vol >= V_CHARGER_MAX )
        {
            battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY]Charger over voltage !!\r\n");                    
            BMT_status.charger_protect_status = charger_OVER_VOL;
            BMT_status.bat_charging_state = CHR_ERROR;
            status = PMU_STATUS_FAIL;        
        }            
    }

	return status;
}


static PMU_STATUS mt_battery_CheckChargingTime(void)
{
    PMU_STATUS status = PMU_STATUS_OK;

    if( (g_battery_thermal_throttling_flag==2) || (g_battery_thermal_throttling_flag==3) )
    {
		battery_xlog_printk(BAT_LOG_CRTI, "[TestMode] Disable Safty Timer. bat_tt_enable=%d, bat_thr_test_mode=%d, bat_thr_test_value=%d\n", 
			g_battery_thermal_throttling_flag, battery_cmd_thermal_test_mode, battery_cmd_thermal_test_mode_value);
		
    }
    else
    {    
        /* Charging OT */
        if(BMT_status.total_charging_time >= MAX_CHARGING_TIME)
        {
   		    battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Charging Over Time. \n");
			   			
            status = PMU_STATUS_FAIL;
        }
    }    

	return status;
        
}

#if defined(STOP_CHARGING_IN_TAKLING)
static PMU_STATUS mt_battery_CheckCallState(void)
{
	PMU_STATUS status = PMU_STATUS_OK;
	
	if((g_call_state == CALL_ACTIVE) && (BMT_status.bat_vol > V_CC2TOPOFF_THRES))
		status = PMU_STATUS_FAIL;

	return status;
}
#endif

static void mt_battery_CheckBatteryStatus(void)
{
    if(mt_battery_CheckBatteryTemp() != PMU_STATUS_OK)
    {
        BMT_status.bat_charging_state = CHR_ERROR;
        return;                  
    }	  

    if(mt_battery_CheckChargerVoltage() != PMU_STATUS_OK)
    {
        BMT_status.bat_charging_state = CHR_ERROR;
        return;                  
    }

    #if defined(STOP_CHARGING_IN_TAKLING)
    if(mt_battery_CheckCallState() != PMU_STATUS_OK)
    {        
        BMT_status.bat_charging_state = CHR_HOLD;
        return;                          
    }
    #endif

    if(mt_battery_CheckChargingTime() != PMU_STATUS_OK)
    {
        BMT_status.bat_charging_state = CHR_ERROR;
        return;                  
    }	
}


static void mt_battery_notify_TatalChargingTime_check(void)
{
#if defined(BATTERY_NOTIFY_CASE_0005_TOTAL_CHARGINGTIME)
    if( (g_battery_thermal_throttling_flag==2) || (g_battery_thermal_throttling_flag==3) )
    {
        battery_xlog_printk(BAT_LOG_CRTI, "[TestMode] Disable Safty Timer : no UI display\n");
    }
    else
    {
        if(BMT_status.total_charging_time >= MAX_CHARGING_TIME)
        //if(BMT_status.total_charging_time >= 60) //test
        {
            g_BatteryNotifyCode |= 0x0010;
            battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Charging Over Time\n");
        }
        else
        {
            g_BatteryNotifyCode &= ~(0x0010);
        }
    }
    
    battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] BATTERY_NOTIFY_CASE_0005_TOTAL_CHARGINGTIME (%x)\n", g_BatteryNotifyCode);
#endif
}


static void mt_battery_notify_VBat_check(void)
{
#if defined(BATTERY_NOTIFY_CASE_0004_VBAT)
    if(BMT_status.bat_vol > 4350)
    //if(BMT_status.bat_vol > 3800) //test
    {
        g_BatteryNotifyCode |= 0x0008;
        battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] bat_vlot(%ld) > 4350mV\n", BMT_status.bat_vol);
    }
    else
    {
        g_BatteryNotifyCode &= ~(0x0008);
    }

    battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] BATTERY_NOTIFY_CASE_0004_VBAT (%x)\n", g_BatteryNotifyCode);

#endif
}


static void mt_battery_notify_ICharging_check(void)
{
#if defined(BATTERY_NOTIFY_CASE_0003_ICHARGING)
    if( (BMT_status.ICharging > 1000) &&
        (BMT_status.total_charging_time > 300))
    {
        g_BatteryNotifyCode |= 0x0004;
        battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] I_charging(%ld) > 1000mA\n", BMT_status.ICharging);
    }
    else
    {
        g_BatteryNotifyCode &= ~(0x0004);
    }

    battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] BATTERY_NOTIFY_CASE_0003_ICHARGING (%x)\n", g_BatteryNotifyCode);
        
#endif
}


static void mt_battery_notify_VBatTemp_check(void)
{
#if defined(BATTERY_NOTIFY_CASE_0002_VBATTEMP)

	if(BMT_status.temperature >= MAX_CHARGE_TEMPERATURE)
    {
        g_BatteryNotifyCode |= 0x0002;
        battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] bat_temp(%d) out of range(too high)\n", BMT_status.temperature);		
    }
#if defined(MTK_JEITA_STANDARD_SUPPORT)
	else if (BMT_status.temperature < TEMP_NEG_10_THRESHOLD)
	{
        g_BatteryNotifyCode |= 0x0020;
        battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] bat_temp(%d) out of range(too low)\n", BMT_status.temperature);
	}
#else
#ifdef BAT_LOW_TEMP_PROTECT_ENABLE
	else if (BMT_status.temperature < MIN_CHARGE_TEMPERATURE)
    {
        g_BatteryNotifyCode |= 0x0020;
        battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] bat_temp(%d) out of range(too low)\n", BMT_status.temperature);
    }
#endif
#endif

    battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] BATTERY_NOTIFY_CASE_0002_VBATTEMP (%x)\n", g_BatteryNotifyCode);
        
#endif
}


static void mt_battery_notify_VCharger_check(void)
{
#if defined(BATTERY_NOTIFY_CASE_0001_VCHARGER)
	if(BMT_status.charger_vol > V_CHARGER_MAX)
    {
        g_BatteryNotifyCode |= 0x0001;
        battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] BMT_status.charger_vol(%ld) > %d mV\n", BMT_status.charger_vol, V_CHARGER_MAX);
    }
    else
    {
        g_BatteryNotifyCode &= ~(0x0001);
    }

    battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] BATTERY_NOTIFY_CASE_0001_VCHARGER (%x)\n", g_BatteryNotifyCode);
#endif	
}


static void mt_battery_notify_UI_test(void)
{
	if(g_BN_TestMode == 0x0001)
    {
        g_BatteryNotifyCode = 0x0001;
        battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0001_VCHARGER\n");
    }
    else if(g_BN_TestMode == 0x0002)
    {
        g_BatteryNotifyCode = 0x0002;
        battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0002_VBATTEMP\n");
    }
    else if(g_BN_TestMode == 0x0003)
    {
        g_BatteryNotifyCode = 0x0004;
        battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0003_ICHARGING\n");
    }
    else if(g_BN_TestMode == 0x0004)
    {
        g_BatteryNotifyCode = 0x0008;
        battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0004_VBAT\n");
    }
    else if(g_BN_TestMode == 0x0005)
    {
        g_BatteryNotifyCode = 0x0010;
        battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0005_TOTAL_CHARGINGTIME\n");
    }
    else
    {
        battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Unknown BN_TestMode Code : %x\n", g_BN_TestMode);
    }
}


void mt_battery_notify_check(void)
{
    g_BatteryNotifyCode = 0x0000;

	if(g_BN_TestMode == 0x0000)	/* for normal case */
    {
        battery_xlog_printk(BAT_LOG_FULL, "[BATTERY] mt_battery_notify_check\n");

	    mt_battery_notify_VCharger_check();

		mt_battery_notify_VBatTemp_check();

		mt_battery_notify_ICharging_check();

		mt_battery_notify_VBat_check();

		mt_battery_notify_TatalChargingTime_check();
    }	
	else  /* for UI test */
	{
		mt_battery_notify_UI_test();
	}
}

static void mt_battery_thermal_check(void)
{
	if( (g_battery_thermal_throttling_flag==1) || (g_battery_thermal_throttling_flag==3) )
    {
        if(battery_cmd_thermal_test_mode == 1){
            BMT_status.temperature = battery_cmd_thermal_test_mode_value;
            battery_xlog_printk(BAT_LOG_FULL, "[Battery] In thermal_test_mode , Tbat=%d\n", BMT_status.temperature);
        }
    
#if defined(MTK_JEITA_STANDARD_SUPPORT)
        //ignore default rule
#else    
		if(BMT_status.temperature >= 60)
        {
            #if defined(CONFIG_POWER_EXT)
            battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] CONFIG_POWER_EXT, no update battery update power down.\n");
            #else
            {
                if( (g_platform_boot_mode==META_BOOT) || (g_platform_boot_mode==ADVMETA_BOOT) || (g_platform_boot_mode==ATE_FACTORY_BOOT) )
                {
                    battery_xlog_printk(BAT_LOG_FULL, "[BATTERY] boot mode = %d, bypass temperature check\n", g_platform_boot_mode);
                }
                else
                {
                    struct battery_data *bat_data = &battery_main;
                    struct power_supply *bat_psy = &bat_data->psy;

                    battery_xlog_printk(BAT_LOG_CRTI, "[Battery] Tbat(%d)>=60, system need power down.\n", BMT_status.temperature);

                    bat_data->BAT_CAPACITY = 0;

                    power_supply_changed(bat_psy); 

                    if( BMT_status.charger_exist == KAL_TRUE )
                    {
                        // can not power down due to charger exist, so need reset system
                        battery_charging_control(CHARGING_CMD_SET_PLATFORM_RESET,NULL);
                    }
                    //avoid SW no feedback
                    battery_charging_control(CHARGING_CMD_SET_POWER_OFF,NULL);
                    //mt_power_off();
                }
            }
            #endif
        }
#endif
        
    }

}


CHARGER_TYPE mt_charger_type_detection(void)
{
	//                                                         
	#if defined( LGE_BSP_LGBM )
	CHARGER_TYPE CHR_Type_num = CHARGER_UNKNOWN;

	mutex_lock(&charger_type_mutex);
	battery_charging_control(CHARGING_CMD_GET_CHARGER_TYPE,&CHR_Type_num);
	mutex_unlock(&charger_type_mutex);
	
	return CHR_Type_num;

	#else
    CHARGER_TYPE CHR_Type_num = CHARGER_UNKNOWN;

    mutex_lock(&charger_type_mutex);
    
#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
    battery_charging_control(CHARGING_CMD_GET_CHARGER_TYPE,&CHR_Type_num);
    BMT_status.charger_type = CHR_Type_num;	
#else
    if(BMT_status.charger_type == CHARGER_UNKNOWN)
    {
        battery_charging_control(CHARGING_CMD_GET_CHARGER_TYPE,&CHR_Type_num);
        BMT_status.charger_type = CHR_Type_num;	
    }	
#endif

    mutex_unlock(&charger_type_mutex);
	
    return BMT_status.charger_type;
	#endif
	//                                                         
}


static void mt_battery_charger_detect_check(void)
{
    if( upmu_is_chr_det() == KAL_TRUE )
    {
        wake_lock(&battery_suspend_lock);
		
		BMT_status.charger_exist = KAL_TRUE;
	
#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
        mt_charger_type_detection();

        if((BMT_status.charger_type==STANDARD_HOST) || (BMT_status.charger_type==CHARGING_HOST) )
        {
            mt_usb_connect();
        }
#else        
		if(BMT_status.charger_type == CHARGER_UNKNOWN)
		{
			mt_charger_type_detection();

			if((BMT_status.charger_type==STANDARD_HOST) || (BMT_status.charger_type==CHARGING_HOST) )
	        	{
	           		mt_usb_connect();
			}
		}
#endif        

		battery_xlog_printk(BAT_LOG_CRTI, "[BAT_thread]Cable in, CHR_Type_num=%d\r\n", BMT_status.charger_type);
		
    }
    else 
    {
        wake_unlock(&battery_suspend_lock);

		BMT_status.charger_exist = KAL_FALSE;
		BMT_status.charger_type = CHARGER_UNKNOWN;
        BMT_status.bat_full = KAL_FALSE;
		BMT_status.bat_in_recharging_state = KAL_FALSE;
		BMT_status.bat_charging_state = CHR_PRE;
		BMT_status.total_charging_time = 0;
		BMT_status.PRE_charging_time = 0;
		BMT_status.CC_charging_time = 0;
		BMT_status.TOPOFF_charging_time = 0;
		BMT_status.POSTFULL_charging_time = 0;

		battery_xlog_printk(BAT_LOG_CRTI, "[BAT_thread]Cable out \r\n");
				
        mt_usb_disconnect();          
    }
}


static void mt_battery_update_status(void)
{
#if defined(CONFIG_POWER_EXT)
    battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] CONFIG_POWER_EXT, no update Android.\n");
#else
	{
	#if !defined(CONFIG_LGE_MINIABB)
		wireless_update(&wireless_main);
	#endif
		battery_update(&battery_main);			
		ac_update(&ac_main);
		usb_update(&usb_main);
	}

#endif	
}

void update_battery_2nd_info(int status_2nd, int capacity_2nd, int present_2nd)
{
    #if defined(CONFIG_POWER_VERIFY)
    battery_xlog_printk(BAT_LOG_CRTI, "Power/Battery", "[update_battery_2nd_info] no support\n");
    #else
    g_status_2nd = status_2nd;
    g_capacity_2nd = capacity_2nd;
    g_present_2nd = present_2nd;
    battery_xlog_printk(BAT_LOG_CRTI, "Power/Battery", "[update_battery_2nd_info] get status_2nd=%d,capacity_2nd=%d,present_2nd=%d\n",
        status_2nd, capacity_2nd, present_2nd);

    wake_up_bat();
    g_smartbook_update = 1;
    #endif
}

#if 0 //                                             
static void mt_kpoc_power_off_check(void)
{
	kal_int32 charger_vol = battery_meter_get_charger_voltage();

	battery_xlog_printk(BAT_LOG_CRTI, "[mt_kpoc_power_off_check] , chr_vol=%d, boot_mode=%d\r\n",charger_vol,g_platform_boot_mode);
	if(g_platform_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT || g_platform_boot_mode == LOW_POWER_OFF_CHARGING_BOOT)
	{
		if( (upmu_is_chr_det() == KAL_FALSE)  && (BMT_status.charger_vol < 2500))	//vbus < 2.5V
		{
			battery_xlog_printk(BAT_LOG_CRTI, "[bat_thread_kthread] Unplug Charger/USB In Kernel Power Off Charging Mode!  Shutdown OS!\r\n");
            battery_charging_control(CHARGING_CMD_SET_POWER_OFF,NULL);
		}
	}
}
#endif //                                             

void do_chrdet_int_task(void)
{
    if(g_bat_init_flag == KAL_TRUE)
    {
    	//                                                         
		#if !defined( LGE_BSP_LGBM )
        if( upmu_is_chr_det() == KAL_TRUE )
        {
            battery_xlog_printk(BAT_LOG_CRTI, "[do_chrdet_int_task] charger exist!\n");
            BMT_status.charger_exist = KAL_TRUE;

            wake_lock(&battery_suspend_lock);

#if defined(CONFIG_POWER_EXT)
            mt_usb_connect();
     	      battery_xlog_printk(BAT_LOG_CRTI, "[do_chrdet_int_task] call mt_usb_connect() in EVB\n");
#endif
        }
        else
        {
       	    battery_xlog_printk(BAT_LOG_CRTI, "[do_chrdet_int_task] charger NOT exist!\n");
            BMT_status.charger_exist = KAL_FALSE;
    		
#if 0 //                                             
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
            if(g_platform_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT || g_platform_boot_mode == LOW_POWER_OFF_CHARGING_BOOT)
            {
                battery_xlog_printk(BAT_LOG_CRTI, "[pmic_thread_kthread] Unplug Charger/USB In Kernel Power Off Charging Mode!  Shutdown OS!\r\n");
    			battery_charging_control(CHARGING_CMD_SET_POWER_OFF,NULL);
    			//mt_power_off();
            }
#endif
#endif //                                             

            wake_unlock(&battery_suspend_lock);

#if defined(CONFIG_POWER_EXT)
            mt_usb_disconnect();
            battery_xlog_printk(BAT_LOG_CRTI, "[do_chrdet_int_task] call mt_usb_disconnect() in EVB\n");
#endif
        }
         
		//Place charger detection and battery update here is used to speed up charging icon display.
			
		mt_battery_charger_detect_check();
		if (BMT_status.UI_SOC == 100 && BMT_status.charger_exist == KAL_TRUE)
		{
			BMT_status.bat_charging_state = CHR_BATFULL;
			BMT_status.bat_full = KAL_TRUE;
			g_charging_full_reset_bat_meter = KAL_TRUE;
		}

		if(g_battery_soc_ready == KAL_FALSE)
			BMT_status.SOC = battery_meter_get_battery_percentage();

        if (BMT_status.bat_vol > 0)
        {
            mt_battery_update_status();
        }
		#endif
		//                                                         
        wake_up_bat();
    }    
    else
   	{
        battery_xlog_printk(BAT_LOG_CRTI, "[do_chrdet_int_task] battery thread not ready, will do after bettery init.\n");    
   	}

}
void bat_thread_wakeup(void)
{
    battery_xlog_printk(BAT_LOG_CRTI, "******** battery : bat_thread_wakeup  ********\n" );
    
    bat_thread_timeout = KAL_TRUE;
    bat_meter_timeout = KAL_TRUE;
    
    wake_up(&bat_thread_wq);    
}

//                                                                          
#if defined( LGE_BSP_LGBM )

////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////LG Function
////////////////////////////////////////////////////////////////////////////////////////////////////////////
LGBmTempState LGBM_GetTempState( LGBmTempState prevState, kal_int32 batTemp )
{
    LGBmTempState newState = LGBM_TEMP_UNKNOWN;

    kal_int32 lowToMid = 0;
    kal_int32 midToHigh = 0;
    kal_int32 highToUltra = 0;

    if( prevState == LGBM_TEMP_UNKNOWN || prevState == LGBM_TEMP_MIDDLE )
    {
        lowToMid = LGBM_OTP_STOP_MIN_TEMP;
        midToHigh = LGBM_OTP_DECREASE_TEMP;
        highToUltra = LGBM_OTP_STOP_MAX_TEMP;
    }
    else if( prevState == LGBM_TEMP_LOW )
    {
        lowToMid = LGBM_OTP_STOP_TO_NORMAL_TEMP;
        midToHigh = LGBM_OTP_DECREASE_TEMP;
        highToUltra = LGBM_OTP_STOP_MAX_TEMP;
    }
    else if( prevState == LGBM_TEMP_HIGH )
    {
        lowToMid = LGBM_OTP_STOP_MIN_TEMP;
        midToHigh = LGBM_OTP_DECREASE_TO_NORMAL_TEMP;
        highToUltra = LGBM_OTP_STOP_MAX_TEMP;
    }
    else /* prevState == LGBM_TEMP_ULTRA_HIGH */
    {
        lowToMid = LGBM_OTP_STOP_MIN_TEMP;
        midToHigh = LGBM_OTP_DECREASE_TEMP;
        highToUltra = LGBM_OTP_STOP_TO_DECREASE_TEMP;
    }

    if( batTemp < lowToMid )
    {
        newState = LGBM_TEMP_LOW;
    }
    else if( batTemp < midToHigh )
    {
        newState = LGBM_TEMP_MIDDLE;
    }
    else if( batTemp < highToUltra )
    {
        newState = LGBM_TEMP_HIGH;
    }
    else
    {
        newState = LGBM_TEMP_ULTRA_HIGH;
    }

    if( prevState != newState )
    {
        switch( newState )
        {
            case LGBM_TEMP_LOW:
                LGBM_LOG("TEMP = LGBM_TEMP_LOW\n");
                break;
            case LGBM_TEMP_MIDDLE:
                LGBM_LOG("TEMP = LGBM_TEMP_MIDDLE\n");
                break;
            case LGBM_TEMP_HIGH:
                LGBM_LOG("TEMP = LGBM_TEMP_HIGH\n");
                break;
            case LGBM_TEMP_ULTRA_HIGH:
                LGBM_LOG("TEMP = LGBM_TEMP_ULTRA_HIGH\n");
                break;
            default:
                LGBM_LOG("Invalid Temp State ( %d )\n", newState);
                break;
        }

    }

    return newState;

}

int LGBM_GetLinearInterpolation(int x1, int x2, int y1, int y2, int x )
{
    int retVal = 0;
    if( x1 == x2 )
    {
        if( y1 == y2 )
        {
            retVal = y1;
        }
        else
        {
            LGBM_ERR("Can't interpolate ( x1=%d, x2=%d, y1=%d, y2=%d )\n", x1, x2, y1, y2 );
        }
    }
    else
    {
        retVal = (y2-y1)*(x-x1)/(x2-x1)+y1;
    }
    return retVal;
}

int LGBM_GetBatTemp (int batTempAdc)
{
    int tmpAdc = 0;
    int batTempVolt = 0;
    int batTemp = 0;

    batTempVolt = batTempAdc;

    if( batTempVolt < 0 )
    {
        batTempVolt = 0;
    }

    BATT_TEMPERATURE *pTable = NULL;
    int tableSize = 0;
    int index = 0;
    int thermistorValue = 0;
    int x1=0, x2=0, y1=0, y2=0;

    pTable = lgbmBatTempTbl;
    tableSize = sizeof(lgbmBatTempTbl)/sizeof(BATT_TEMPERATURE);
        
    thermistorValue = ( batTempVolt * RBAT_PULL_UP_R ) / ( RBAT_PULL_UP_VOLT - batTempVolt ) ;

    if( thermistorValue > pTable[0].TemperatureR )
    {
        batTemp = pTable[0].BatteryTemp;
    }
    else
    {
        for ( index = 1 ; index < tableSize ; index++ )
        {
            if( thermistorValue > pTable[index].TemperatureR )
            {
                x1 = pTable[index].TemperatureR;
                x2 = pTable[index-1].TemperatureR;
                y1 = pTable[index].BatteryTemp;
                y2 = pTable[index-1].BatteryTemp;
                batTemp = LGBM_GetLinearInterpolation(x1, x2, y1, y2, thermistorValue);
                break;
            }
        }

        if ( index == tableSize )
        {
            batTemp = pTable[tableSize-1].BatteryTemp;
        }
    }

    return batTemp;

}


int LGBM_ReadBatTempAdc ( void )
{
    static int lastAdcValue = -99;
    int adcValue = 0;

    adcValue = PMIC_IMM_GetOneChannelValue(VBATTEMP_CHANNEL_NUMBER, 5, 1); /* unit : mV */

    //LGBM_LOG("Battery Temp ADC = %d[mV]\n", adcValue);

    if( adcValue <= 0 )
    {
        LGBM_LOG("Battery Temperature ADC is out of range ( %d ), so use previous value\n", adcValue);
        adcValue = lastAdcValue;
    }
    else
    {
        lastAdcValue = adcValue;
		g_nBatteryTemp = adcValue;
    }
	/*
	if( fake_batt_mode == 1)
	{
		adcValue = 710;	// force to set the temperature to 25 degree celsius 
		lastAdcValue = adcValue;
	}
	*/
    return adcValue;

}

int LGBM_ReadChgVoltAdc ( void )
{
    static int lastAdcValue = -99;
    int adcValue = 0;

    adcValue = PMIC_IMM_GetOneChannelValue(VCHARGER_CHANNEL_NUMBER, 5, 1); /* unit : mV */

    //LGBM_LOG("Charger ADC = %d[mV]\n", adcValue);

    if( adcValue <= 0 )
    {
        LGBM_LOG("Battery ADC is out of range ( %d ), so use previous value\n", adcValue);
        adcValue = lastAdcValue;
    }
    else
    {
        lastAdcValue = adcValue;
    }

    return adcValue;

}
int LGBM_GetBatVoltAdc(void)
{
	return g_batVoltAdc;
}
EXPORT_SYMBOL(LGBM_GetBatVoltAdc);
int LGBM_ReadBatVoltAdc ( void )
{
    static int lastAdcValue = -99;
    int adcValue = 0;

    adcValue = PMIC_IMM_GetOneChannelValue(VBAT_CHANNEL_NUMBER, 5, 1); /* unit : mV */

    //LGBM_LOG("Battery ADC = %d[mV]\n", adcValue);

    if( adcValue <= 0 )
    {
        LGBM_LOG("Battery ADC is out of range ( %d ), so use previous value\n", adcValue);
        adcValue = lastAdcValue;
    }
    else
    {
        lastAdcValue = adcValue;
    }

    return adcValue;

}
EXPORT_SYMBOL(LGBM_ReadBatVoltAdc);

int LGBM_ReadBatExistance( void )
{
    int batExist = 1;
    int batTempAdc = 0;
    int loopCount = 0;

    batTempAdc = LGBM_ReadBatTempAdc();
    if( batTempAdc > LGBM_BAT_REMOVE_ADC_TH )
    {
        for( loopCount=0 ; loopCount < 5 ; loopCount++ )
        {
            mdelay(100);
            batTempAdc = LGBM_ReadBatTempAdc();
            if( batTempAdc < LGBM_BAT_REMOVE_ADC_TH )
            {
                break;
            }
        }

        if( loopCount >= 5 )
        {
            batExist = 0;
        }
    }

    return batExist;

}


void LGBM_ReadBatVital(LGBmVital *pVital)
{
    kal_int32 batTempAdc = 0;
    kal_int32 batVoltAdc = 0;
    kal_int32 chgVoltAdc = 0;
    kal_int32 batTemp = 0;
    kal_int32 batVolt = 0;
    kal_int32 chgVolt = 0;

    /* read battery ADC */
    batVoltAdc = LGBM_ReadBatVoltAdc();
	g_batVoltAdc = batVoltAdc;
	
    /* read charger ADC */
    chgVoltAdc = LGBM_ReadChgVoltAdc();
    chgVolt = chgVoltAdc*5000/670;

    /* read battery temperature ADC */
    batTempAdc = LGBM_ReadBatTempAdc();
    if( batTempAdc > LGBM_BAT_REMOVE_ADC_TH )
    {
        /* impossible temperature so set to default embient temperature */
        /* it is needed to process battery removal, if not high temperature protection will be processed */
        batTempAdc = 710; /* 710[mV] means 25 degree celsius */
    }

    /* read battery temperature */
    batTemp = LGBM_GetBatTemp(batTempAdc);

    /* read battery voltage */
    batVolt = batVoltAdc;		//need coding yunhang.heo

	LGBM_LOG("[Vital] batVoltAdc=%d[mV], batTempAdc=%d[mV], chgVoltAdc=%d[mV]\n", batVoltAdc, batTempAdc, chgVoltAdc);
    LGBM_LOG("[Vital] batTemp=%d[C], batVolt=%d[mV], chgVolt=%d[mV]\n", batTemp, batVolt, chgVolt);

    pVital->batTempAdc = batTempAdc;
    pVital->batVoltAdc = batVoltAdc;
    pVital->chgVoltAdc = chgVoltAdc;
    pVital->batTemp = batTemp;
    pVital->batVolt = batVolt;
    pVital->chgVolt = chgVolt;
	if(fake_batt_mode==1)
	{
		pVital->batTempAdc = 710;
		pVital->batTemp = 29;
	}

}

int LGBM_GetNewSoc( void )
{
	int getSoc = 0;
	int nVbatt;
    long ret;

	if(g_nFirstStart)
    {
        nVbatt = LGBM_ReadBatVoltAdc();
        switch (g_lgbmChargerType)
        {
            case NONSTANDARD_CHARGER:
            case STANDARD_CHARGER:
            case FACTORY_CHARGER:
                DEBUG_MSG("[LGBM] NONSTANDARD_CHARGER or STANDARD_CHARGER\n");
                GetSOC(SOC_CHR, nVbatt, ret);
                break;
            case STANDARD_HOST:
            case CHARGING_HOST:
                DEBUG_MSG("[LGBM] STANDARD_HOST or CHARGING_HOST\n");
                GetSOC(SOC_USB, nVbatt, ret);
                break;
            default:
                DEBUG_MSG("[LGBM] default!!!\n");
                GetSOC(SOC_ICV, nVbatt, ret);
                break;
            if(ret <= PARAM_SCALE)
                ret = PARAM_SCALE;
		}
        getSoc = (int)(ret / 1000);
    }
    else
    {
        getSoc = (int)(g_lSocView/1000);
    }

	LGBM_LOG("newSoc=%d[%]\n", getSoc);
	
	return getSoc;
	
}

CHARGER_TYPE LGBM_ReadChargerType ( void )
{
    CHARGER_TYPE charger = CHARGER_UNKNOWN;

    charger = mt_charger_type_detection();

    switch ( charger )
    {
        case STANDARD_HOST:
            LGBM_LOG("STANDARD_HOST was detected\n");
            break;
        case CHARGING_HOST:
            LGBM_LOG("CHARGING_HOST was detected\n");
            break;
        case NONSTANDARD_CHARGER:
            LGBM_LOG("NONSTANDARD_CHARGER was detected\n");
            break;
        case STANDARD_CHARGER:
            LGBM_LOG("STANDARD_CHARGER was detected\n");
            break;
        default:
            LGBM_LOG("Unknown type of charger ( %d ) was detected, and treat is as STANDARD_CHARGER\n", charger);
            charger = STANDARD_CHARGER;
            break;
    }

    return charger ;

}

LGBmCableId LGBM_ReadUsbCableId( void )
{
	LGBmCableId cableId = USB_CABLE_ID_NONE;
	int resVal = 0;
	int rawdata = 0;
	int adcVolt = 0;
	int data[4] = {0, 0, 0, 0};
	
	resVal = IMM_auxadc_GetOneChannelValue(AUXADC_USB_ID_CHANNEL, data, &rawdata);
	if( resVal < 0 )
    {
        LGBM_LOG("ADC Read Fail on AUXADC_USB_ID_CHANNEL ( error = %d )\n", resVal);
    }
    else
    {
        adcVolt = g_AtCmdUsbAdc = rawdata*150/4096;

        LGBM_LOG("ReadUsbCableId ( rawdata = %d, adcVolt = %d.%d%d[mV] )\n", rawdata, adcVolt/100, (adcVolt%100)/10, (adcVolt%100)%10);
		if(upmu_is_chr_det() == KAL_TRUE)
        {
	        if( LGBM_CABLE_ID_56K_ADC_MIN <= adcVolt && adcVolt <= LGBM_CABLE_ID_56K_ADC_MAX )
	        {
	            cableId = USB_CABLE_ID_56K;
				g_AtCmdUsbId = 56;
	            LGBM_LOG("cableId = LGBM_CABLE_ID_56K\n");
	        }
	        else if( LGBM_CABLE_ID_130K_ADC_MIN <= adcVolt && adcVolt <= LGBM_CABLE_ID_130K_ADC_MAX )
	        {
	            cableId = USB_CABLE_ID_130K;
				g_AtCmdUsbId=130;
	            LGBM_LOG("cableId = LGBM_CABLE_ID_130K\n");
	        }
	        else if( LGBM_CABLE_ID_180K_ADC_MIN <= adcVolt && adcVolt <= LGBM_CABLE_ID_180K_ADC_MAX )
	        {
	            cableId = USB_CABLE_ID_180K;
				g_AtCmdUsbId=180;
	            LGBM_LOG("cableId = LGBM_CABLE_ID_180K\n");
	        }
	        else if( LGBM_CABLE_ID_910K_ADC_MIN <= adcVolt && adcVolt <= LGBM_CABLE_ID_910K_ADC_MAX )
	        {
	            cableId = USB_CABLE_ID_910K;
				g_AtCmdUsbId=910;
	            LGBM_LOG("cableId = LGBM_CABLE_ID_910K\n");
	        }
	        else if( LGBM_CABLE_ID_OPEN_ADC_MIN <= adcVolt )
	        {
	            cableId = USB_CABLE_ID_OPEN;
				g_AtCmdUsbId=0;
	            LGBM_LOG("cableId = LGBM_CABLE_ID_OPEN\n");
	        }
	        else
	        {
        		cableId = USB_CABLE_ID_UNKNOWN;
				g_AtCmdUsbId=0;
            	LGBM_LOG("cableId = LGBM_CABLE_ID_UNKNOWN ( rawdata = %d, adcVolt = %d[mV] )\n", rawdata, adcVolt);
			}
		}
		else
		{
			cableId = USB_CABLE_ID_NONE;
           	LGBM_LOG("cableId = LGBM_CABLE_ID_NONE ( rawdata = %d, adcVolt = %d[mV] )\n", rawdata, adcVolt);
		}

    }
	return cableId;
}
CHARGER_TYPE LGBM_GetCharger ( void )
{
    LGBmCableId cableId = USB_CABLE_ID_NONE;

    CHARGER_TYPE charger = CHARGER_UNKNOWN;

    cableId = LGBM_ReadUsbCableId();	
    if( cableId == USB_CABLE_ID_56K || cableId == USB_CABLE_ID_130K || cableId == USB_CABLE_ID_910K )
    {
        charger = FACTORY_CHARGER;
    }
    else
    {
        charger = LGBM_ReadChargerType();
    }

    return charger;

}

kal_bool LGBM_ReadChargerExistance(void)
{
    kal_bool chargerExist = KAL_FALSE;

    chargerExist = upmu_is_chr_det();

    LGBM_LOG("CHARGER = %d\n", chargerExist);

    return chargerExist;
}
void LGBM_SetUsbConnection( kal_bool isConnect )
{
    if( isConnect == KAL_TRUE )
    {
		mt_usb_connect();
        LGBM_LOG("USB CONNECTION = 1\n");
    }
    else
    {
    	mt_usb_disconnect();
        LGBM_LOG("USB CONNECTION = 0\n");
    }
}
void LGBM_UpdateUsbStatus( kal_bool isConnect )
{
    if( isConnect == KAL_TRUE )
    {
    	usb_main.USB_ONLINE = 1;
        power_supply_changed(&usb_main);
    }
    else
    {
        usb_main.USB_ONLINE = 0;
        power_supply_changed(&usb_main);
    }

    LGBM_LOG("UI_UPDATE ( USB =  %d )\n", usb_main.USB_ONLINE);
}
void LGBM_UpdateAcStatus( kal_bool isConnect )
{
    if( isConnect == KAL_TRUE )
    {
        ac_main.AC_ONLINE = 1;
        power_supply_changed(&ac_main);
    }
    else
    {
        ac_main.AC_ONLINE = 0;
        power_supply_changed(&ac_main);
    }

    LGBM_LOG("UI_UPDATE ( AC =  %d )\n", ac_main.AC_ONLINE);
}

void LGBM_PrintBmStateChange ( LGBmState prevState, LGBmState newState )
{
    char strPrevState[30] = {0};
    char strNewState[30] = {0};

    switch ( prevState )
    {
        case LGBM_TS_INIT:
            strcpy( strPrevState, "LGBM_TS_INIT" );
            break;

        case LGBM_TS_NO_CHARGER:
            strcpy( strPrevState, "LGBM_TS_NO_CHARGER" );
            break;

        case LGBM_TS_CHARGING:
            strcpy( strPrevState, "LGBM_TS_CHARGING" );
            break;

        case LGBM_TS_CHARGING_FULL:
            strcpy( strPrevState, "LGBM_TS_CHARGING_FULL" );
            break;

        case LGBM_TS_FACTORY:
            strcpy( strPrevState, "LGBM_TS_FACTORY" );
            break;

        case LGBM_TS_INSERT_BATTERY:
            strcpy( strPrevState, "LGBM_TS_INSERT_BATTERY" );
            break;

        case LGBM_TS_UNKNOWN:
            strcpy( strPrevState, "LGBM_TS_UNKNOWN" );
            break;

        default :
            strcpy( strPrevState, "LGBM_TS_INVALID" );
            break;

    }

    switch ( newState )
    {
        case LGBM_TS_INIT:
            strcpy( strNewState, "LGBM_TS_INIT" );
            break;

        case LGBM_TS_NO_CHARGER:
            strcpy( strNewState, "LGBM_TS_NO_CHARGER" );
            break;

        case LGBM_TS_CHARGING:
            strcpy( strNewState, "LGBM_TS_CHARGING" );
            break;

        case LGBM_TS_CHARGING_FULL:
            strcpy( strNewState, "LGBM_TS_CHARGING_FULL" );
            break;

        case LGBM_TS_FACTORY:
            strcpy( strNewState, "LGBM_TS_FACTORY" );
            break;

        case LGBM_TS_INSERT_BATTERY:
            strcpy( strNewState, "LGBM_TS_INSERT_BATTERY" );
            break;

        case LGBM_TS_UNKNOWN:
            strcpy( strNewState, "LGBM_TS_UNKNOWN" );
            break;

        default :
            strcpy( strNewState, "LGBM_TS_INVALID" );
            break;

    }

    if( prevState != newState )
    {
        LGBM_LOG("BM_STATE : %s => %s\n", strPrevState, strNewState);
    }
    else
    {
        LGBM_LOG("BM_STATE : %s => %s\n", strPrevState, strNewState);
    }

}

void LGBM_SetChargingCurrent( LGBmChargingCurrent chgCurrent )
{
    static LGBmChargingCurrent prevChgCurrent = LGBM_CC_UNKNOWN;
	//                                                                               
	#if defined(CONFIG_LGE_MINIABB)
	#else
    BQ25040_ChargingMode mode = BQ25040_CM_UNKNOWN;
	#endif

	
    if( (g_AtCmdChargingModeOff == 0 ) && ( prevChgCurrent != chgCurrent ) )
    {
        prevChgCurrent = chgCurrent;

        switch ( chgCurrent )
        {

		#if defined(CONFIG_LGE_MINIABB)
		     case LGBM_CC_OFF:
                LGBM_LOG("Charging Current = LGBM_CC_OFF\n");
				set_charger_stop_mode();
                break;
            case LGBM_CC_USB_100:
                LGBM_LOG("Charging Current = LGBM_CC_USB_100\n");
				set_charger_start_mode(CHG_100);
                break;
            case LGBM_CC_USB_500:
                LGBM_LOG("Charging Current = LGBM_CC_USB_500\n");
				set_charger_start_mode(CHG_500);
                break;
            case LGBM_CC_I_SET:
                LGBM_LOG("Charging Current = LGBM_CC_I_SET\n");
				set_charger_start_mode(CHG_TA);
                break;
            case LGBM_CC_FACTORY:
                LGBM_LOG("Charging Current = LGBM_CC_FACTORY\n");
				set_charger_start_mode(CHG_TA);
                break;
            default:
                LGBM_LOG("Invalid Charging Current ( %d )\n", chgCurrent);
                break;
		#else		
            case LGBM_CC_OFF:
                LGBM_LOG("Charging Current = LGBM_CC_OFF\n");
                mode = BQ25040_CM_OFF;
                break;
            case LGBM_CC_USB_100:
                LGBM_LOG("Charging Current = LGBM_CC_USB_100\n");
                mode = BQ25040_CM_USB_100;
                break;
            case LGBM_CC_USB_500:
                LGBM_LOG("Charging Current = LGBM_CC_USB_500\n");
                mode = BQ25040_CM_USB_500;
                break;
            case LGBM_CC_I_SET:
                LGBM_LOG("Charging Current = LGBM_CC_I_SET\n");
                mode = BQ25040_CM_I_SET;
                break;
            case LGBM_CC_FACTORY:
                LGBM_LOG("Charging Current = LGBM_CC_FACTORY\n");
                mode = BQ25040_CM_FACTORY;
                break;
            default:
                LGBM_LOG("Invalid Charging Current ( %d )\n", chgCurrent);
                break;
		#endif				
        }

		
		#if defined(CONFIG_LGE_MINIABB)
		#else
        if( mode != BQ25040_CM_UNKNOWN )
        {
            BQ25040_SetChargingMode(mode);
        }
		#endif
		//                                                                                 

    }

}

void LGBM_ChangeState( LGBmData *pBmData, LGBmState newState )
{
    kal_bool result = KAL_TRUE;

    LGBmState prevState = LGBM_TS_UNKNOWN;

    prevState = pBmData->bmState;

    if( prevState != newState )
    {
        if( prevState == LGBM_TS_INIT || prevState == LGBM_TS_NO_CHARGER )
        {
            if( newState == LGBM_TS_CHARGING || newState == LGBM_TS_CHARGING_FULL )
            {
                pBmData->bmOTPState = LGBM_OTP_UNKNOWN;

                if( newState == LGBM_TS_CHARGING )
                {
                    if( pBmData->charger == NONSTANDARD_CHARGER || pBmData->charger == STANDARD_CHARGER )
                    {
                        LGBM_SetChargingCurrent(LGBM_CC_I_SET);
                    }
                    else
                    {
                        LGBM_SetChargingCurrent(LGBM_CC_USB_500);
                    }
                }
            }
            else if( newState == LGBM_TS_FACTORY )
            {
                if( prevState == LGBM_TS_NO_CHARGER )
                {
                    LGBM_SetChargingCurrent(LGBM_CC_FACTORY);
                }
            }
            else if( newState == LGBM_TS_NO_CHARGER )
            {
                LGBM_SetChargingCurrent(LGBM_CC_OFF);
            }
            else if( newState == LGBM_TS_INSERT_BATTERY )
            {
                // do nothing
            }
            else
            {
                result = KAL_FALSE;
            }
        }
        else if( prevState == LGBM_TS_CHARGING || prevState == LGBM_TS_CHARGING_FULL )
        {
            if( newState == LGBM_TS_NO_CHARGER )
            {
                LGBM_SetChargingCurrent(LGBM_CC_OFF);
            }
            else if( newState == LGBM_TS_CHARGING || newState == LGBM_TS_CHARGING_FULL )
            {
                pBmData->bmOTPState = LGBM_OTP_UNKNOWN;

                if( newState == LGBM_TS_CHARGING_FULL )
                {
                    LGBM_SetChargingCurrent(LGBM_CC_OFF);
                }
            }
            else
            {
                result = KAL_FALSE;
            }
        }
        else if( prevState == LGBM_TS_FACTORY )
        {
            if( newState == LGBM_TS_NO_CHARGER )
            {
                LGBM_SetChargingCurrent(LGBM_CC_OFF);
            }
            else
            {
                result = KAL_FALSE;
            }
        }
        else
        {
            result = KAL_FALSE;
        }

    }

    if( result == KAL_TRUE )
    {
        if( pBmData->bmState != newState )
        {
            LGBM_PrintBmStateChange( prevState, newState );

            pBmData->bmState = newState ;

            switch( newState )
            {
                case LGBM_TS_NO_CHARGER:
                    LGBM_LOG("BM_STATE = LGBM_TS_NO_CHARGER\n");
                    break;
                case LGBM_TS_CHARGING:
                    LGBM_LOG("BM_STATE = LGBM_TS_CHARGING\n");
                    break;
                case LGBM_TS_CHARGING_FULL:
                    LGBM_LOG("BM_STATE = LGBM_TS_CHARGING_FULL\n");
                    break;
                case LGBM_TS_FACTORY:
                    LGBM_LOG("BM_STATE = LGBM_TS_FACTORY\n");
                    break;
                case LGBM_TS_INSERT_BATTERY:
                    LGBM_LOG("BM_STATE = LGBM_TS_INSERT_BATTERY\n");
                    break;
                default:
                    LGBM_LOG("Invalid BM_STATE ( %d )\n", newState);
                    break;
            }
        }
        else
        {
            LGBM_LOG("No State Change\n");
        }
    }
    else
    {
        LGBM_LOG("Invalid State Change ( %d => %d )\n", prevState, newState);
    }

}

void LGBM_UpdateBatStatus( LGBmData *pBmData, LGBmVital *pVital, struct battery_data *bat_data )
{
	struct power_supply *bat_psy = &bat_data->psy;

	if( pBmData->tempState == LGBM_TEMP_LOW )
	{
		bat_data->BAT_HEALTH = POWER_SUPPLY_HEALTH_COLD;
	}
	else if( pBmData->tempState == LGBM_TEMP_HIGH ||pBmData->tempState == LGBM_TEMP_ULTRA_HIGH )
	{
		bat_data->BAT_HEALTH = POWER_SUPPLY_HEALTH_OVERHEAT;
	}
	else
	{
		bat_data->BAT_HEALTH = POWER_SUPPLY_HEALTH_GOOD;
	}

	bat_data->BAT_batt_vol = pVital->batVolt ;
	bat_data->BAT_batt_temp = pVital->batTemp * 10 ;
	bat_data->BAT_PRESENT = pBmData->batExist ;
	bat_data->BAT_CAPACITY = pBmData->curSoc ;
	bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_UNKNOWN;

	if( pBmData->bmState == LGBM_TS_NO_CHARGER )
	{
		bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_DISCHARGING;
		if( bat_data->BAT_CAPACITY > 100 )
		{
			bat_data->BAT_CAPACITY = 100;
		}
		else if( bat_data->BAT_CAPACITY < 1 )
		{
			bat_data->BAT_CAPACITY = 1;
		}
	}
	else if( pBmData->bmState == LGBM_TS_CHARGING )
	{
		if( pBmData->tempState == LGBM_TEMP_ULTRA_HIGH )
		{
			bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
		else
		{
			bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_CHARGING;
		}

		if( bat_data->BAT_CAPACITY > 99 )
		{
			bat_data->BAT_CAPACITY = 100;
		}
		else if( bat_data->BAT_CAPACITY < 1 )
		{
			bat_data->BAT_CAPACITY = 1;
		}
	}
	else if( pBmData->bmState == LGBM_TS_CHARGING_FULL )
	{
		if( pBmData->tempState == LGBM_TEMP_ULTRA_HIGH )
		{
			bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
		else
		{
			bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_FULL;
		}

		bat_data->BAT_CAPACITY = 100;
	}
	else if( pBmData->bmState == LGBM_TS_FACTORY )
	{
		bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_CHARGING;
		if( bat_data->BAT_CAPACITY > 100 )
		{
			bat_data->BAT_CAPACITY = 100;
		}
		else if( bat_data->BAT_CAPACITY < 1 )
		{
			bat_data->BAT_CAPACITY = 1;
		}
	}
	
	bat_data->BAT_TemperatureR = 0;
	bat_data->BAT_TempBattVoltage = pVital->batTempAdc;
	bat_data->BAT_InstatVolt = pVital->batVolt ;
	bat_data->BAT_BatteryAverageCurrent = 0;
	bat_data->BAT_BatterySenseVoltage = pVital->batVoltAdc ;
	bat_data->BAT_ISenseVoltage = 0 ;
	bat_data->BAT_ChargerVoltage = pVital->chgVolt;

    /* for facotry AT command */
    g_AtCmdBatSocUI = bat_data->BAT_CAPACITY;
    g_AtCmdBatFullUI = bat_data->BAT_STATUS;
	
	power_supply_changed(bat_psy);

	LGBM_LOG("UI_UPDATE ( BATTERY ) : batVolt=%d, batTemp=%d, batCap=%d, batSoc=%d, driverSoc=%d\n", bat_data->BAT_batt_vol, bat_data->BAT_batt_temp, bat_data->BAT_CAPACITY, pBmData->curSoc, LGBM_GetNewSoc());
	//LGBM_LOG("UI_UPDATE ( BATTERY ) : batVolt=%d, batTemp=%d, batSoc=%d, driverSoc=%d\n", bat_data->BAT_batt_vol, bat_data->BAT_batt_temp, pBmData->curSoc, LGBM_GetNewSoc());
	//LGBM_LOG("UI_UPDATE ( BATTERY ) : bat_data->BAT_TempBattVoltage = %d\n", bat_data->BAT_TempBattVoltage);

}
int LGBM_ReadEocState ( LGBmVital *pVital )
{
    kal_int32 hwEocState = 0;
    kal_int32 eocState = 0;

    int decisionCount = 3; /* 30ms duration */

#if defined(CONFIG_LGE_MINIABB)
    hwEocState = check_EOC_status();

    while ( hwEocState == 1 && decisionCount > 0 )
    {
        msleep(10);
        hwEocState = check_EOC_status();
        decisionCount--;
    }
#else
    hwEocState = mt_get_gpio_in(CHG_EOC_N);

    while ( hwEocState == 1 && decisionCount > 0 )
    {
        msleep(10);
        hwEocState = mt_get_gpio_in(CHG_EOC_N);
        decisionCount--;
    }
#endif
	
    if( hwEocState == 1 )
    {
        if( pVital->batVoltAdc > LGBM_SW_EOC_VOLTAGE_TH )
        {
            eocState = 1;
            LGBM_LOG("EOC = 1 ( HW = %d, batVoltAdc = %d[mV] )\n", hwEocState, pVital->batVoltAdc);
        }
        else
        {
            LGBM_LOG("EOC = 0 ( HW = %d, batVoltAdc = %d[mV] )\n", hwEocState, pVital->batVoltAdc);
        }
    }

    return eocState;

}

void LGBM_StateInit( LGBmData *pBmData, LGBmVital *pVital )
{
	//do nothing
}
void LGBM_StateNoCharger( LGBmData *pBmData, LGBmVital *pVital )
{
    kal_int32 newSoc = 0;
    kal_int32 batVolt = pVital->batVolt;
    kal_int32 soc15Volt = 0;
    long tempSoc = 0;
    static kal_int32 trackingMethod = 0; /* 0=No tracking, 1=Running, 2=Keeping */
    static kal_int32 trackingVolt = 0;
    static kal_int32 trackingSoc = 0;
    static kal_int32 trackingCount = 0;
	static bool is_first_called = KAL_TRUE;
    
    pBmData->chargingSocTrackState = LGBM_SOC_UN_TRACKED;

    newSoc = LGBM_GetNewSoc();
	if(is_first_called)
	{
		pBmData->curSoc = newSoc;
		is_first_called = KAL_FALSE;
	}
	
	if( batVolt < LGBM_LOW_VOLTAGE )
	{
		wake_lock(&battery_suspend_lock);
		g_low_bat_wakelock_enable = KAL_TRUE;
	}
	else
	{
		g_low_bat_wakelock_enable = KAL_FALSE;
	}

    if( pBmData->nochargerSocTrackState == LGBM_SOC_UN_TRACKED )
    {
        trackingMethod = 0;
        trackingVolt = 0;
        trackingSoc = 0;
        trackingCount = 0;

        pBmData->nochargerSocTrackState = LGBM_SOC_ON_TRACKING;
    }

    if( trackingMethod == 0 ) /* No tracking */
    {
        if( newSoc >= 1 && batVolt < LGBM_CUT_OFF_VOLTAGE )
        {
            trackingMethod = 1;
            trackingVolt = 0;
            trackingSoc = 1;
        }
        else if( newSoc == 1 && batVolt > LGBM_CUT_OFF_VOLTAGE && pBmData->curSoc > 2 )
        {
            trackingMethod = 1;
            trackingVolt = 0;
            trackingSoc = 1;
        }
        else if( newSoc == 1 && batVolt > LGBM_CUT_OFF_VOLTAGE && pBmData->curSoc <= 2 )
        {
            trackingMethod = 2;
            trackingVolt = 0;
            trackingSoc = 1;
        }
        else if( newSoc >= 100 && pBmData->curSoc == 99 )
        {
            trackingMethod = 2;
            trackingSoc = 99;
        }
        else
        {
            pBmData->curSoc = newSoc;
        }

        if( trackingMethod != 0 )
        {
            trackingCount = 0;
            LGBM_LOG("SOC TRACKING :  Method = %d, trackingSoc = %d, trackingVolt = %d\n", trackingMethod, trackingSoc, trackingVolt);
        }

    }
    else if( trackingMethod == 1 ) /* Running */
    {
        LGBM_LOG("SOC TRACKING :  Method = %d, trackingSoc = %d\n", trackingMethod, trackingSoc);
        trackingCount++;
        if( trackingCount == 6 ) /* WakeLock ==> 6 = 6*10sec, WakeUnlock ==> 6 = ?? ( TBD )  */
        {
            trackingCount = 0;

            pBmData->curSoc--;

            pBmData->nochargerSocTrackState = LGBM_SOC_UN_TRACKED;
            LGBM_LOG("SOC TRACKING : Tracking Done ( curSoc = %d )\n", pBmData->curSoc);

        }
    }
    else if( trackingMethod == 2 ) /* Keeping */
    {
        LGBM_LOG("SOC TRACKING :  Method = %d, trackingSoc = %d\n", trackingMethod, trackingSoc);

        if( batVolt < LGBM_CUT_OFF_VOLTAGE )
        {
            pBmData->curSoc--;
            pBmData->nochargerSocTrackState = LGBM_SOC_UN_TRACKED;
            LGBM_LOG("SOC TRACKING : Tracking Done ( curSoc = %d )\n", pBmData->curSoc);
        }
        else if( newSoc == trackingSoc )
        {
            pBmData->nochargerSocTrackState = LGBM_SOC_UN_TRACKED;
            LGBM_LOG("SOC TRACKING : Tracking Done ( curSoc = %d )\n", pBmData->curSoc);
        }
    }
    else
    {
        LGBM_ERR("Invalid Tracking Method ( %d )\n", trackingMethod);
    }
}

void LGBM_StateCharging( LGBmData *pBmData, LGBmVital *pVital )
{
    LGBmOTPState newOtpState = LGBM_OTP_UNKNOWN;
    kal_int32 stopChargingVolt = 0;
    kal_int32 batVolt = pVital->batVolt;
    kal_int32 newSoc = 0;
    kal_int32 eocState = 0;
	
    static kal_int32 trackingMethod = 0; /* 0=No tracking, 1=Running, 2=Keeping */
    static kal_int32 trackingSoc = 0;
    static kal_int32 trackingCount = 0;

    pBmData->nochargerSocTrackState = LGBM_SOC_UN_TRACKED;
    newSoc = LGBM_GetNewSoc();

    if( pBmData->bmOTPState == LGBM_OTP_NORMAL_CHARGING )
    {
        eocState = LGBM_ReadEocState(pVital);
    }

    if( pBmData->chargingSocTrackState == LGBM_SOC_UN_TRACKED )
    {
        trackingMethod = 0;
        trackingSoc = 0;
        trackingCount = 0;

        pBmData->chargingSocTrackState = LGBM_SOC_ON_TRACKING;
    }

    if( trackingMethod == 0 ) /* No tracking */
    {
        if( eocState == 1 )
        {
            trackingMethod = 1;
            trackingSoc = 100;
        }
        else if( eocState == 0 && newSoc >= 100 && pBmData->curSoc == 99 )
        {
            trackingMethod = 2;
            trackingSoc = 100;
        }
        else
        {
            pBmData->curSoc = newSoc;
        }

        if( trackingMethod != 0 )
        {
            trackingCount = 0;
            LGBM_LOG("SOC TRACKING :  Method = %d, trackingSoc = %d\n", trackingMethod, trackingSoc);
        }

    }
    else if( trackingMethod == 1 ) /* Running */
    {
        trackingCount++;
        LGBM_LOG("SOC TRACKING :  Method = %d, trackingSoc = %d\n", trackingMethod, trackingSoc);
        if( trackingCount == 6 ) /* WakeLock ==> 6 = 6*10sec, WakeUnlock ==> 6 = ?? ( TBD )  */
        {
            trackingCount = 0;

            pBmData->curSoc++;
		    g_lSocMeas = ( pBmData->curSoc * PARAM_SCALE ) + ( PARAM_SCALE / 2 );

	    if( pBmData->curSoc >= trackingSoc )
            {
                pBmData->chargingSocTrackState = LGBM_SOC_UN_TRACKED;
                LGBM_LOG("SOC TRACKING : Tracking Done ( curSoc = %d )\n", pBmData->curSoc);
            }
        }
    }
    else if( trackingMethod == 2 ) /* Keeping */
    {
        LGBM_LOG("SOC TRACKING :  Method = %d, trackingSoc = %d\n", trackingMethod, trackingSoc);
        if( trackingSoc == 100 )
        {
            if( eocState == 1 )
            {
                pBmData->chargingSocTrackState = LGBM_SOC_UN_TRACKED;
                pBmData->curSoc++;
                LGBM_LOG("SOC TRACKING : Tracking Done ( curSoc = %d )\n", pBmData->curSoc);
            }
        }
        else
        {
            LGBM_ERR("Invalid Tracking Target ( %d )\n", trackingSoc);
        }
    }
    else
    {
        LGBM_ERR("Invalid Tracking Method ( %d )\n", trackingMethod);
    }

    if( pBmData->tempState == LGBM_TEMP_LOW || pBmData->tempState == LGBM_TEMP_ULTRA_HIGH )
    {
        newOtpState = LGBM_OTP_STOP_CHARGING;
    }
    else if( pBmData->tempState == LGBM_TEMP_HIGH )
    {
        if( pBmData->bmOTPState == LGBM_OTP_DECREASE_CHARGING )
        {
            stopChargingVolt = LGBM_OTP_STOP_VOLTAGE;
        }
        else
        {
            stopChargingVolt = LGBM_OTP_HYST_VOLTAGE;
        }

        if( pVital->batVolt > stopChargingVolt )
        {
            newOtpState = LGBM_OTP_STOP_CHARGING;
        }
        else
        {
            newOtpState = LGBM_OTP_DECREASE_CHARGING;
        }
    }
    else
    {
        newOtpState = LGBM_OTP_NORMAL_CHARGING;
    }

    if( pBmData->bmOTPState != newOtpState )
    {
        pBmData->bmOTPState = newOtpState;

        if( newOtpState == LGBM_OTP_NORMAL_CHARGING )
        {
            if( pBmData->charger == NONSTANDARD_CHARGER || pBmData->charger == STANDARD_CHARGER )
            {
                LGBM_SetChargingCurrent(LGBM_CC_I_SET);
            }
            else
            {
                LGBM_SetChargingCurrent(LGBM_CC_USB_500);
            }
        }
        else if ( newOtpState == LGBM_OTP_DECREASE_CHARGING )
        {
            LGBM_SetChargingCurrent(LGBM_CC_USB_500);
        }
        else if( newOtpState == LGBM_OTP_STOP_CHARGING )
        {
            LGBM_SetChargingCurrent(LGBM_CC_OFF);
        }
        else
        {
            LGBM_LOG("Invalid OtpState ( %d )\n", newOtpState);
        }

        switch( newOtpState )
        {
            case LGBM_OTP_NORMAL_CHARGING:
                LGBM_LOG("OPT_STATE = LGBM_OTP_NORMAL_CHARGING\n");
                break;
            case LGBM_OTP_DECREASE_CHARGING:
                LGBM_LOG("OPT_STATE = LGBM_OTP_DECREASE_CHARGING\n");
                break;
            case LGBM_OTP_STOP_CHARGING:
                LGBM_LOG("OPT_STATE = LGBM_OTP_STOP_CHARGING\n");
                break;
            default:
                LGBM_LOG("Invalid OPT_STATE ( %d )\n", newOtpState);
                break;
        }

    }
    else
    {
        if( pBmData->bmOTPState == LGBM_OTP_NORMAL_CHARGING )
        {
            if( eocState == 1 && pBmData->curSoc >= 100 )
            {
                LGBM_ChangeState( pBmData, LGBM_TS_CHARGING_FULL );
				pBmData->curSoc = 102;
                g_lSocMeas = 101500;
            }
        }
    }

}

void LGBM_StateChargingFull( LGBmData *pBmData, LGBmVital *pVital )
{
    LGBmOTPState newOtpState = LGBM_OTP_UNKNOWN;
    kal_int32 stopChargingVolt = 0;
    pBmData->chargingSocTrackState = LGBM_SOC_UN_TRACKED;
    pBmData->nochargerSocTrackState = LGBM_SOC_UN_TRACKED;

    pBmData->curSoc = LGBM_GetNewSoc();
    
    if( pBmData->tempState == LGBM_TEMP_LOW || pBmData->tempState == LGBM_TEMP_ULTRA_HIGH )
    {
        newOtpState = LGBM_OTP_STOP_CHARGING;
    }
    else if( pBmData->tempState == LGBM_TEMP_HIGH )
    {
        if( pBmData->bmOTPState == LGBM_OTP_DECREASE_CHARGING )
        {
            stopChargingVolt = LGBM_OTP_STOP_VOLTAGE;
        }
        else
        {
            stopChargingVolt = LGBM_OTP_HYST_VOLTAGE;
        }

        if( pVital->batVolt > stopChargingVolt )
        {
            newOtpState = LGBM_OTP_STOP_CHARGING;
        }
        else
        {
            newOtpState = LGBM_OTP_DECREASE_CHARGING;
        }
    }
    else /* pBmData->tempState == LGBM_TEMP_MIDDLE */
    {
        if( pBmData->bmOTPState == LGBM_OTP_NORMAL_CHARGING )
        {
            if( LGBM_ReadEocState(pVital) == 1 )
            {
                newOtpState = LGBM_OTP_STOP_CHARGING;
            }
            else
            {
                newOtpState = LGBM_OTP_NORMAL_CHARGING;
            }
        }
        else
        {
            if( pVital->batVolt >= LGBM_CHARGING_FULL_TO_RECHARGING ) //For setting Recharging Volt
            {
                newOtpState = LGBM_OTP_STOP_CHARGING;
            }
            else
            {
                newOtpState = LGBM_OTP_NORMAL_CHARGING;
            }
        }

		pBmData->curSoc = 102;
        g_lSocMeas = 101500;		

	}

    if( pBmData->bmOTPState != newOtpState )
    {
        pBmData->bmOTPState = newOtpState;

        if( newOtpState == LGBM_OTP_NORMAL_CHARGING )
        {
            if( pBmData->charger == NONSTANDARD_CHARGER || pBmData->charger == STANDARD_CHARGER )
            {
                LGBM_SetChargingCurrent(LGBM_CC_I_SET);
            }
            else
            {
                LGBM_SetChargingCurrent(LGBM_CC_USB_500);
            }
        }
        else if ( pBmData->bmOTPState == LGBM_OTP_DECREASE_CHARGING )
        {
            LGBM_SetChargingCurrent(LGBM_CC_USB_500);
        }
        else if( pBmData->bmOTPState == LGBM_OTP_STOP_CHARGING )
        {
            LGBM_SetChargingCurrent(LGBM_CC_OFF);
        }
        else
        {
            LGBM_LOG("Invalid OtpState ( %d )\n", newOtpState);
        }

        switch( newOtpState )
        {
            case LGBM_OTP_NORMAL_CHARGING:
                LGBM_LOG("OPT_STATE = LGBM_OTP_NORMAL_CHARGING\n");
                break;
            case LGBM_OTP_DECREASE_CHARGING:
                LGBM_LOG("OPT_STATE = LGBM_OTP_DECREASE_CHARGING\n");
                break;
            case LGBM_OTP_STOP_CHARGING:
                LGBM_LOG("OPT_STATE = LGBM_OTP_STOP_CHARGING\n");
                break;
            default:
                LGBM_LOG("Invalid OPT_STATE ( %d )\n", newOtpState);
                break;
        }

    }

    if( pVital->batVolt < LGBM_CHARGING_FULL_TO_CHARGING_VOLTAGE )
    {
        LGBM_ChangeState( pBmData, LGBM_TS_CHARGING );
    }

}
void LGBM_StateFactory( LGBmData *pBmData, LGBmVital *pVital )
{
    kal_int32 batExist = 0;
    kal_int32 newSoc = 0;
    pBmData->chargingSocTrackState = LGBM_SOC_UN_TRACKED;
    pBmData->nochargerSocTrackState = LGBM_SOC_UN_TRACKED;

    batExist = LGBM_ReadBatExistance();
    if( pBmData->batExist != batExist )
    {
        pBmData->batExist = batExist;

        LGBM_LOG("BATTERY = %d\n", batExist);

        if( batExist == 1 )
        {
            pBmData->curSoc = LGBM_GetNewSoc();
        }
        else
        {
            pBmData->curSoc = 100;
        }

        LGBM_UpdateBatStatus( pBmData, pVital, &battery_main );

    }
    else
    {
        if( batExist == 1 )
        {
			pBmData->curSoc = LGBM_GetNewSoc();
        }
    }

}

void LGBM_StateInsertBattery( LGBmData *pBmData, LGBmVital *pVital )
{
    kal_int32 batExist = 0;
    
    pBmData->chargingSocTrackState = LGBM_SOC_UN_TRACKED;
    pBmData->nochargerSocTrackState = LGBM_SOC_UN_TRACKED;

    batExist = LGBM_ReadBatExistance();
    if( batExist == 1 )
    {
        LGBM_LOG("BATTERY = %d so do power down\n", batExist);

        msleep(100);
		battery_charging_control(CHARGING_CMD_SET_POWER_OFF,NULL);
    }
    else
    {
        pBmData->curSoc = 0;
    }
}

#endif

//                                                                          

void BAT_thread(void)
{
    static kal_bool  battery_meter_initilized = KAL_FALSE;
    if(battery_meter_initilized == KAL_FALSE)
    {
        battery_meter_initial();	//move from battery_probe() to decrease booting time
       	BMT_status.nPercent_ZCV = battery_meter_get_battery_nPercent_zcv();
        battery_meter_initilized = KAL_TRUE;
    }

    mt_battery_charger_detect_check();
    mt_battery_GetBatteryData();
    mt_battery_thermal_check();
    mt_battery_notify_check();    

    if( BMT_status.charger_exist == KAL_TRUE )
    {
        mt_battery_CheckBatteryStatus();	    
        mt_battery_charging_algorithm();
    }
	
    mt_battery_update_status();
#if 0 //                                             
	mt_kpoc_power_off_check();
#endif //                                             
}

///////////////////////////////////////////////////////////////////////////////////////////
//// Internal API
///////////////////////////////////////////////////////////////////////////////////////////
int bat_thread_kthread(void *x)
{

//                                                                        
	#if defined( LGE_BSP_LGBM )
	kal_bool delayedInitDone = KAL_FALSE;
	static kal_bool isFirst = KAL_TRUE;
	LGBmVital batVital = { 0, 0, 0, 0, 0, 0 };
	LGBmState newBmState = LGBM_TS_UNKNOWN;
	LGBmTempState newTempState = LGBM_TEMP_UNKNOWN;
	LGBmData bmData;	
    CHARGER_TYPE prevCharger = CHARGER_UNKNOWN;
#if defined (TARGET_S4)	
	int ovp_check=0;
#endif	
	
    /* for facotry AT command */
    g_pAtCmdBatVital = &batVital;
    g_pAtCmdBmData = &bmData;

	bmData.curSoc = 50;
	bmData.batExist = 1;
	bmData.charger = CHARGER_UNKNOWN;
	bmData.bmState = LGBM_TS_INIT;
	bmData.bmOTPState = LGBM_OTP_NORMAL_CHARGING;
    bmData.chargingSocTrackState = LGBM_SOC_UN_TRACKED;
    bmData.nochargerSocTrackState = LGBM_SOC_UN_TRACKED;
	bmData.tempState = LGBM_TEMP_UNKNOWN;	

    ktime_t ktime = ktime_set(7, 0);  // 10s, 10* 1000 ms

    /* Run on a process content */  
    while (1) {

		//LGBM_LOG("wait event \n");
		wait_event(bat_thread_wq, (bat_thread_timeout == KAL_TRUE));

        mutex_lock(&bat_mutex);

        bat_thread_timeout = KAL_FALSE;

		if(delayedInitDone == KAL_FALSE)
		{

			if(chr_wake_up_bat == KAL_TRUE)
			{
				chr_wake_up_bat=KAL_FALSE;
				LGBM_LOG("Charger Event Happened before delayed init processing\n");

			}
            LGBM_LOG("Delayed init processing was started\n");

			LGBM_ReadBatVital(&batVital);
			newTempState = LGBM_GetTempState(bmData.tempState, batVital.batTemp);
			bmData.tempState = newTempState;

			bmData.batExist = LGBM_ReadBatExistance();
			LGBM_LOG("BATTERY = %d\n", bmData.batExist);

			if( LGBM_ReadChargerExistance() == KAL_TRUE )
			{
				wake_lock(&battery_suspend_lock);
				bmData.charger = LGBM_GetCharger();
			}
			else
			{
				wake_unlock(&battery_suspend_lock);
				bmData.charger = CHARGER_UNKNOWN;
			}

            g_lgbmChargerType = bmData.charger;
            
			if( bmData.charger == FACTORY_CHARGER )
			{
				newBmState = LGBM_TS_FACTORY;
				LGBM_SetUsbConnection(KAL_TRUE);
				LGBM_UpdateUsbStatus(KAL_TRUE);
			}
			else if( bmData.charger == CHARGER_UNKNOWN )
			{
				newBmState = LGBM_TS_NO_CHARGER;
				LGBM_SetUsbConnection(KAL_FALSE);
				LGBM_UpdateUsbStatus(KAL_FALSE);
				LGBM_UpdateAcStatus(KAL_FALSE);
			}
			else
			{
				if( bmData.batExist == 1 )
				{
					newBmState = LGBM_TS_CHARGING;
				}
				else
				{
					newBmState = LGBM_TS_INSERT_BATTERY;
				}
				
				if( bmData.charger == NONSTANDARD_CHARGER || bmData.charger == STANDARD_CHARGER )
				{
					LGBM_UpdateAcStatus(KAL_TRUE);
				}
				
				else
				{
					LGBM_SetUsbConnection(KAL_TRUE);
					LGBM_UpdateUsbStatus(KAL_TRUE);
				}
			}
			
			LGBM_ChangeState( &bmData, newBmState );
            LGBM_LOG("Delayed init processing was done\n");
			delayedInitDone=KAL_TRUE;			
		}
		else
		{	
			
			if(chr_wake_up_bat == KAL_TRUE)
			{
				g_AtCmdChargingModeOff = 0;
			
				chr_wake_up_bat = 0;

                LGBM_LOG("Battery Thread Was Called By Charger Event ( PMIC or USB )\n");

                prevCharger = bmData.charger;
				
                if( LGBM_ReadChargerExistance() == KAL_TRUE )
                {
                	wake_lock(&battery_suspend_lock);
                    if( prevCharger == CHARGER_UNKNOWN )
                    {
                        bmData.charger = LGBM_GetCharger();
                    }
                }
				else
				{
					wake_unlock(&battery_suspend_lock);
                    bmData.charger = CHARGER_UNKNOWN;
				}

				g_lgbmChargerType = bmData.charger;

                if( prevCharger != bmData.charger )
               	{
					LGBM_ReadBatVital(&batVital);
					newTempState = LGBM_GetTempState(bmData.tempState, batVital.batTemp);
					bmData.tempState = newTempState;

				
					if( bmData.charger == FACTORY_CHARGER )
					{
						newBmState = LGBM_TS_FACTORY;
						LGBM_SetUsbConnection(KAL_TRUE);
						LGBM_UpdateUsbStatus(KAL_TRUE);
					}
					else if( bmData.charger == CHARGER_UNKNOWN )
					{
						newBmState = LGBM_TS_NO_CHARGER;
						LGBM_SetUsbConnection(KAL_FALSE);
						LGBM_UpdateUsbStatus(KAL_FALSE);
						LGBM_UpdateAcStatus(KAL_FALSE);
					}
#if defined (TARGET_S4)
					else if(check_miniabb_OVP())
					{
						
						newBmState = LGBM_TS_NO_CHARGER;
						LGBM_SetUsbConnection(KAL_FALSE);
						LGBM_UpdateUsbStatus(KAL_FALSE);
						LGBM_UpdateAcStatus(KAL_FALSE);
                        bmData.charger = CHARGER_UNKNOWN;
						ovp_check=KAL_TRUE;				
					}
#endif					
					else
					{
						if( bmData.batExist == 1 )
						{
							if( bmData.curSoc < 100 )
							{
								newBmState = LGBM_TS_CHARGING;
							}
							else
							{
								newBmState = LGBM_TS_CHARGING_FULL;
							}
					
						}
						else
						{
							newBmState = LGBM_TS_INSERT_BATTERY;
							bmData.curSoc = 0;
						}
						if( bmData.charger == NONSTANDARD_CHARGER || bmData.charger == STANDARD_CHARGER )
						{
							LGBM_UpdateAcStatus(KAL_TRUE);
						}
						
						else
						{
							LGBM_SetUsbConnection(KAL_TRUE);
							LGBM_UpdateUsbStatus(KAL_TRUE);
						}
					}
               	}
				
				LGBM_ChangeState( &bmData, newBmState );
				LGBM_UpdateBatStatus( &bmData, &batVital, &battery_main );				
				
			}
			else
			{

                LGBM_LOG("Battery Thread Was Called By Timer\n");
				
				LGBM_ReadBatVital(&batVital);
				newTempState = LGBM_GetTempState(bmData.tempState, batVital.batTemp);
				bmData.tempState = newTempState;

				//bmData.curSoc = LGBM_GetNewSoc();
#if defined (TARGET_S4)
				if(ovp_check==KAL_TRUE)
				{
						if(check_miniabb_OVP()==KAL_FALSE)
						{
							chr_wake_up_bat=KAL_TRUE;
							ovp_check=KAL_FALSE;
							wake_up_bat();
						}
				}
#endif
				if( bmData.bmState == LGBM_TS_CHARGING || bmData.bmState == LGBM_TS_CHARGING_FULL )
				{
					if( bmData.bmOTPState == LGBM_OTP_NORMAL_CHARGING || bmData.bmOTPState == LGBM_OTP_DECREASE_CHARGING )
					{
						if( LGBM_ReadBatExistance() == 0 )
						{
							/* turn off charging to power shut down in case of battery removal */
							LGBM_SetChargingCurrent(LGBM_CC_OFF);
						}
#if defined (TARGET_S4)
						if(check_miniabb_OVP()==KAL_TRUE)
						{
							chr_wake_up_bat=KAL_TRUE;
							ovp_check=KAL_TRUE;
	                        bmData.charger = CHARGER_UNKNOWN;
							wake_up_bat();
						}
#endif						
					}

				}
				switch( bmData.bmState )
				{
					case LGBM_TS_INIT:
						LGBM_LOG("BM_STATE = LGBM_TS_INIT\n");
						LGBM_StateInit( &bmData, &batVital );
						break;

					case LGBM_TS_NO_CHARGER:
						LGBM_LOG("BM_STATE = LGBM_TS_NO_CHARGER\n");
						/* CAUTION : fix sleep current problem after set USB connection type, it can save about 1 ~ 2mA */
						if(wake_lock_active(&battery_suspend_lock))
						{
							if(g_low_bat_wakelock_enable != KAL_TRUE)
								wake_unlock(&battery_suspend_lock);
						}
						mt_usb_disconnect();
						LGBM_StateNoCharger( &bmData, &batVital );
						break;

					case LGBM_TS_CHARGING:
						LGBM_LOG("BM_STATE = LGBM_TS_CHARGING\n");
						LGBM_StateCharging( &bmData, &batVital );
						break;

					case LGBM_TS_CHARGING_FULL:
						LGBM_LOG("BM_STATE = LGBM_TS_CHARGING_FULL\n");
						LGBM_StateChargingFull( &bmData, &batVital );
						break;

					case LGBM_TS_FACTORY:
						LGBM_LOG("BM_STATE = LGBM_TS_FACTORY\n");
						LGBM_StateFactory( &bmData, &batVital );
						break;

					case LGBM_TS_INSERT_BATTERY:
						LGBM_LOG("BM_STATE = LGBM_TS_INSERT_BATTERY\n");
						LGBM_StateInsertBattery( &bmData, &batVital );
						break;

					default:
						LGBM_LOG("Invalid BM_STATE ( %d )\n", bmData.bmState);
						break;

				}
				LGBM_UpdateBatStatus( &bmData, &batVital, &battery_main );
				if(g_updateBatStateFlag == 0)
				{		
					g_updateBatStateFlag = 1;
				}
			}
		}

        mutex_unlock(&bat_mutex);
        hrtimer_start(&battery_kthread_timer, ktime, HRTIMER_MODE_REL);
        ktime = ktime_set(BAT_TASK_PERIOD, 0);  // 10s, 10* 1000 ms
    }
	#else
    ktime_t ktime = ktime_set(3, 0);  // 10s, 10* 1000 ms	

    /* Run on a process content */  
    while (1) {               
        mutex_lock(&bat_mutex);
          
		if(chargin_hw_init_done == KAL_TRUE)  
	        BAT_thread();                      

        mutex_unlock(&bat_mutex);
    
        battery_xlog_printk(BAT_LOG_FULL, "wait event \n" );

		wait_event(bat_thread_wq, (bat_thread_timeout == KAL_TRUE));
	
        bat_thread_timeout = KAL_FALSE;
        hrtimer_start(&battery_kthread_timer, ktime, HRTIMER_MODE_REL);   
		if(ktime_1sec_count < ktime_1sec_max_count)		// make sure the charger type detection is done before IPOD timeout
		{
			ktime_1sec_count++;
			ktime = ktime_set(1, 0);  // 10s, 10* 1000 ms
		}
		else
		{
    	    ktime = ktime_set(BAT_TASK_PERIOD, 0);  // 10s, 10* 1000 ms
		}
        if( chr_wake_up_bat == KAL_TRUE && g_smartbook_update != 1)	// for charger plug in/ out
        {
                g_smartbook_update = 0;
           	battery_meter_reset();
			chr_wake_up_bat = KAL_FALSE;
			            
            battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Charger plug in/out, Call battery_meter_reset. (%d)\n", BMT_status.UI_SOC);
        }
        
    }
	#endif
	//                                                                        

    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////
//// fop API 
///////////////////////////////////////////////////////////////////////////////////////////
static long adc_cali_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int *user_data_addr;
    int *naram_data_addr;
    int i = 0;
    int ret = 0;
	int adc_in_data[2] = {1,1};
	int adc_out_data[2] = {1,1};

    mutex_lock(&bat_mutex);

    switch(cmd)
    {
        case TEST_ADC_CALI_PRINT :
            g_ADC_Cali = KAL_FALSE;
            break;
        
        case SET_ADC_CALI_Slop:            
            naram_data_addr = (int *)arg;
            ret = copy_from_user(adc_cali_slop, naram_data_addr, 36);
            g_ADC_Cali = KAL_FALSE; /* enable calibration after setting ADC_CALI_Cal */            
            /* Protection */
            for (i=0;i<14;i++) 
            { 
                if ( (*(adc_cali_slop+i) == 0) || (*(adc_cali_slop+i) == 1) ) {
                    *(adc_cali_slop+i) = 1000;
                }
            }
            for (i=0;i<14;i++) battery_xlog_printk(BAT_LOG_CRTI, "adc_cali_slop[%d] = %d\n",i , *(adc_cali_slop+i));
            battery_xlog_printk(BAT_LOG_FULL, "**** unlocked_ioctl : SET_ADC_CALI_Slop Done!\n");            
            break;    
            
        case SET_ADC_CALI_Offset:            
            naram_data_addr = (int *)arg;
            ret = copy_from_user(adc_cali_offset, naram_data_addr, 36);
            g_ADC_Cali = KAL_FALSE; /* enable calibration after setting ADC_CALI_Cal */
            for (i=0;i<14;i++) battery_xlog_printk(BAT_LOG_CRTI, "adc_cali_offset[%d] = %d\n",i , *(adc_cali_offset+i));
            battery_xlog_printk(BAT_LOG_FULL, "**** unlocked_ioctl : SET_ADC_CALI_Offset Done!\n");            
            break;
            
        case SET_ADC_CALI_Cal :            
            naram_data_addr = (int *)arg;
            ret = copy_from_user(adc_cali_cal, naram_data_addr, 4);
            g_ADC_Cali = KAL_TRUE;
            if ( adc_cali_cal[0] == 1 ) {
                g_ADC_Cali = KAL_TRUE;
            } else {
                g_ADC_Cali = KAL_FALSE;
            }            
            for (i=0;i<1;i++) battery_xlog_printk(BAT_LOG_CRTI, "adc_cali_cal[%d] = %d\n",i , *(adc_cali_cal+i));
            battery_xlog_printk(BAT_LOG_FULL, "**** unlocked_ioctl : SET_ADC_CALI_Cal Done!\n");            
            break;    

        case ADC_CHANNEL_READ:            
            //g_ADC_Cali = KAL_FALSE; /* 20100508 Infinity */
            user_data_addr = (int *)arg;
            ret = copy_from_user(adc_in_data, user_data_addr, 8); /* 2*int = 2*4 */
          
            if( adc_in_data[0] == 0 ) // I_SENSE
            {
                adc_out_data[0] = battery_meter_get_VSense() * adc_in_data[1];
            }
            else if( adc_in_data[0] == 1 ) // BAT_SENSE
            {
                adc_out_data[0] = battery_meter_get_battery_voltage() * adc_in_data[1];
            }
            else if( adc_in_data[0] == 3 ) // V_Charger
            {
                adc_out_data[0] = battery_meter_get_charger_voltage() * adc_in_data[1];
                //adc_out_data[0] = adc_out_data[0] / 100;
            }    
            else if( adc_in_data[0] == 30 ) // V_Bat_temp magic number
            {                
                adc_out_data[0] = battery_meter_get_battery_temperature() * adc_in_data[1];                
            }
            else if( adc_in_data[0] == 66 ) 
            {
                adc_out_data[0] = (battery_meter_get_battery_current())/10;
                
                if (battery_meter_get_battery_current_sign() == KAL_TRUE) 
                {                    
                    adc_out_data[0] = 0 - adc_out_data[0]; //charging
                }                                
            }
            else
            {
                battery_xlog_printk(BAT_LOG_FULL, "unknown channel(%d,%d)%d\n", adc_in_data[0], adc_in_data[1]);
            }
            
            if (adc_out_data[0]<0)
                adc_out_data[1]=1; /* failed */
            else
                adc_out_data[1]=0; /* success */

            if( adc_in_data[0] == 30 )
                adc_out_data[1]=0; /* success */

            if( adc_in_data[0] == 66 )
                adc_out_data[1]=0; /* success */
                
            ret = copy_to_user(user_data_addr, adc_out_data, 8);
            battery_xlog_printk(BAT_LOG_CRTI, "**** unlocked_ioctl : Channel %d * %d times = %d\n", adc_in_data[0], adc_in_data[1], adc_out_data[0]);            
            break;

        case BAT_STATUS_READ:            
            user_data_addr = (int *)arg;
            ret = copy_from_user(battery_in_data, user_data_addr, 4); 
            /* [0] is_CAL */
            if (g_ADC_Cali) {
                battery_out_data[0] = 1;
            } else {
                battery_out_data[0] = 0;
            }
            ret = copy_to_user(user_data_addr, battery_out_data, 4); 
            battery_xlog_printk(BAT_LOG_CRTI, "**** unlocked_ioctl : CAL:%d\n", battery_out_data[0]);                        
            break;        

        case Set_Charger_Current: /* For Factory Mode*/
            user_data_addr = (int *)arg;
            ret = copy_from_user(charging_level_data, user_data_addr, 4);
            g_ftm_battery_flag = KAL_TRUE;            
            if( charging_level_data[0] == 0 ) {             charging_level_data[0] = CHARGE_CURRENT_70_00_MA;
            } else if ( charging_level_data[0] == 1  ) {    charging_level_data[0] = CHARGE_CURRENT_200_00_MA;
            } else if ( charging_level_data[0] == 2  ) {    charging_level_data[0] = CHARGE_CURRENT_400_00_MA;
            } else if ( charging_level_data[0] == 3  ) {    charging_level_data[0] = CHARGE_CURRENT_450_00_MA;
            } else if ( charging_level_data[0] == 4  ) {    charging_level_data[0] = CHARGE_CURRENT_550_00_MA;
            } else if ( charging_level_data[0] == 5  ) {    charging_level_data[0] = CHARGE_CURRENT_650_00_MA;
            } else if ( charging_level_data[0] == 6  ) {    charging_level_data[0] = CHARGE_CURRENT_700_00_MA;
            } else if ( charging_level_data[0] == 7  ) {    charging_level_data[0] = CHARGE_CURRENT_800_00_MA;
            } else if ( charging_level_data[0] == 8  ) {    charging_level_data[0] = CHARGE_CURRENT_900_00_MA;
            } else if ( charging_level_data[0] == 9  ) {    charging_level_data[0] = CHARGE_CURRENT_1000_00_MA;
            } else if ( charging_level_data[0] == 10 ) {    charging_level_data[0] = CHARGE_CURRENT_1100_00_MA;
            } else if ( charging_level_data[0] == 11 ) {    charging_level_data[0] = CHARGE_CURRENT_1200_00_MA;
            } else if ( charging_level_data[0] == 12 ) {    charging_level_data[0] = CHARGE_CURRENT_1300_00_MA;
            } else if ( charging_level_data[0] == 13 ) {    charging_level_data[0] = CHARGE_CURRENT_1400_00_MA;
            } else if ( charging_level_data[0] == 14 ) {    charging_level_data[0] = CHARGE_CURRENT_1500_00_MA;
            } else if ( charging_level_data[0] == 15 ) {    charging_level_data[0] = CHARGE_CURRENT_1600_00_MA;
            } else { 
                charging_level_data[0] = CHARGE_CURRENT_450_00_MA;
            }
            wake_up_bat();
            battery_xlog_printk(BAT_LOG_CRTI, "**** unlocked_ioctl : set_Charger_Current:%d\n", charging_level_data[0]);
            break;
		//add for meta tool-------------------------------
		case Get_META_BAT_VOL:
			user_data_addr = (int *)arg;
            ret = copy_from_user(adc_in_data, user_data_addr, 8);
			adc_out_data[0] = BMT_status.bat_vol;
			ret = copy_to_user(user_data_addr, adc_out_data, 8); 
            //xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "**** unlocked_ioctl : BAT_VOL:%d\n", adc_out_data[0]);   
			break;
		case Get_META_BAT_SOC:
			user_data_addr = (int *)arg;
            ret = copy_from_user(adc_in_data, user_data_addr, 8);
			adc_out_data[0] = BMT_status.UI_SOC;
			ret = copy_to_user(user_data_addr, adc_out_data, 8); 
            //xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "**** unlocked_ioctl : SOC:%d\n", adc_out_data[0]);   
			break;
		//add bing meta tool-------------------------------
          
        default:
            g_ADC_Cali = KAL_FALSE;
            break;
    }

    mutex_unlock(&bat_mutex);
    
    return 0;
}

static int adc_cali_open(struct inode *inode, struct file *file)
{ 
   return 0;
}

static int adc_cali_release(struct inode *inode, struct file *file)
{
    return 0;
}


static struct file_operations adc_cali_fops = {
    .owner        = THIS_MODULE,
    .unlocked_ioctl    = adc_cali_ioctl,
    .open        = adc_cali_open,
    .release    = adc_cali_release,    
};


void check_battery_exist(void)
{
#if defined(CONFIG_DIS_CHECK_BATTERY)
    battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Disable check battery exist.\n");
#else
    kal_uint32 baton_count = 0;
	kal_uint32 charging_enable = KAL_FALSE;
	kal_uint32 battery_status;
	kal_uint32 i;

	for(i=0;i<3;i++)
	{
		battery_charging_control(CHARGING_CMD_GET_BATTERY_STATUS,&battery_status);
		baton_count += battery_status;

	}
       
    if( baton_count >= 3)
    {
        if( (g_platform_boot_mode==META_BOOT) || (g_platform_boot_mode==ADVMETA_BOOT) || (g_platform_boot_mode==ATE_FACTORY_BOOT) )
        {
            battery_xlog_printk(BAT_LOG_FULL, "[BATTERY] boot mode = %d, bypass battery check\n", g_platform_boot_mode);
        }
        else
        {
            battery_xlog_printk(BAT_LOG_FULL, "[BATTERY] Battery is not exist, power off FAN5405 and system (%d)\n", baton_count);
            
			battery_charging_control(CHARGING_CMD_ENABLE,&charging_enable);
            battery_charging_control(CHARGING_CMD_SET_PLATFORM_RESET,NULL);    
        }
    }    
#endif
}


int charger_hv_detect_sw_thread_handler(void *unused)
{
    ktime_t ktime;
	kal_uint32 charging_enable;
	kal_uint32 hv_voltage = BATTERY_VOLT_07_000000_V;
	kal_bool hv_status;	


    do
    {
        ktime = ktime_set(0, BAT_MS_TO_NS(2000));       

		if(chargin_hw_init_done)
			battery_charging_control(CHARGING_CMD_SET_HV_THRESHOLD,&hv_voltage);
            
        wait_event_interruptible(charger_hv_detect_waiter, (charger_hv_detect_flag == KAL_TRUE));
    
       	if ((upmu_is_chr_det() == KAL_TRUE))
        {
            check_battery_exist();
        }
		
	 	charger_hv_detect_flag = KAL_FALSE;

		if(chargin_hw_init_done)
			battery_charging_control(CHARGING_CMD_GET_HV_STATUS,&hv_status);

		if(hv_status == KAL_TRUE)
        {
            battery_xlog_printk(BAT_LOG_CRTI, "[charger_hv_detect_sw_thread_handler] charger hv\n");    
            
			charging_enable = KAL_FALSE;
			if(chargin_hw_init_done)
				battery_charging_control(CHARGING_CMD_ENABLE,&charging_enable);
        }
        else
        {
            battery_xlog_printk(BAT_LOG_FULL, "[charger_hv_detect_sw_thread_handler] upmu_chr_get_vcdt_hv_det() != 1\n");    
        }

		if(chargin_hw_init_done)
			battery_charging_control(CHARGING_CMD_RESET_WATCH_DOG_TIMER,NULL);
       
        hrtimer_start(&charger_hv_detect_timer, ktime, HRTIMER_MODE_REL);    
        
    } while (!kthread_should_stop());
    
    return 0;
}

enum hrtimer_restart charger_hv_detect_sw_workaround(struct hrtimer *timer)
{
    charger_hv_detect_flag = KAL_TRUE; 
    wake_up_interruptible(&charger_hv_detect_waiter);

    battery_xlog_printk(BAT_LOG_FULL, "[charger_hv_detect_sw_workaround] \n");
    
    return HRTIMER_NORESTART;
}

void charger_hv_detect_sw_workaround_init(void)
{
    ktime_t ktime;

    ktime = ktime_set(0, BAT_MS_TO_NS(2000));
    hrtimer_init(&charger_hv_detect_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    charger_hv_detect_timer.function = charger_hv_detect_sw_workaround;    
    hrtimer_start(&charger_hv_detect_timer, ktime, HRTIMER_MODE_REL);

    charger_hv_detect_thread = kthread_run(charger_hv_detect_sw_thread_handler, 0, "mtk charger_hv_detect_sw_workaround");
    if (IS_ERR(charger_hv_detect_thread))
    {
        battery_xlog_printk(BAT_LOG_FULL, "[%s]: failed to create charger_hv_detect_sw_workaround thread\n", __FUNCTION__);
    }

    battery_xlog_printk(BAT_LOG_CRTI, "charger_hv_detect_sw_workaround_init : done\n" );
}


enum hrtimer_restart battery_kthread_hrtimer_func(struct hrtimer *timer)
{
    bat_thread_wakeup(); 
    
    return HRTIMER_NORESTART;
}

void battery_kthread_hrtimer_init(void)
{
    ktime_t ktime;

    ktime = ktime_set(1, 0);	// 3s, 10* 1000 ms
    hrtimer_init(&battery_kthread_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    battery_kthread_timer.function = battery_kthread_hrtimer_func;    
    hrtimer_start(&battery_kthread_timer, ktime, HRTIMER_MODE_REL);

    battery_xlog_printk(BAT_LOG_CRTI, "battery_kthread_hrtimer_init : done\n" );
}


static void get_charging_control(void)
{
	battery_charging_control = chr_control_interface;
}


static int battery_probe(struct platform_device *dev)    
{
    struct class_device *class_dev = NULL;
    int ret=0;

    battery_xlog_printk(BAT_LOG_CRTI, "******** battery driver probe!! ********\n" );

    /* Integrate with NVRAM */
    ret = alloc_chrdev_region(&adc_cali_devno, 0, 1, ADC_CALI_DEVNAME);
    if (ret) 
       battery_xlog_printk(BAT_LOG_CRTI, "Error: Can't Get Major number for adc_cali \n");
    adc_cali_cdev = cdev_alloc();
    adc_cali_cdev->owner = THIS_MODULE;
    adc_cali_cdev->ops = &adc_cali_fops;
    ret = cdev_add(adc_cali_cdev, adc_cali_devno, 1);
    if(ret)
       battery_xlog_printk(BAT_LOG_CRTI, "adc_cali Error: cdev_add\n");
    adc_cali_major = MAJOR(adc_cali_devno);
    adc_cali_class = class_create(THIS_MODULE, ADC_CALI_DEVNAME);
    class_dev = (struct class_device *)device_create(adc_cali_class, 
                                                   NULL, 
                                                   adc_cali_devno, 
                                                   NULL, 
                                                   ADC_CALI_DEVNAME);
    battery_xlog_printk(BAT_LOG_CRTI, "[BAT_probe] adc_cali prepare : done !!\n ");

	get_charging_control();

    battery_charging_control(CHARGING_CMD_GET_PLATFORM_BOOT_MODE, &g_platform_boot_mode);
    battery_xlog_printk(BAT_LOG_CRTI, "[BAT_probe] g_platform_boot_mode = %d\n ", g_platform_boot_mode);

	wake_lock_init(&battery_suspend_lock, WAKE_LOCK_SUSPEND, "battery suspend wakelock");    

    /* Integrate with Android Battery Service */
    ret = power_supply_register(&(dev->dev), &ac_main.psy);
    if (ret)
    {            
        battery_xlog_printk(BAT_LOG_CRTI, "[BAT_probe] power_supply_register AC Fail !!\n");                    
        return ret;
    }             
    battery_xlog_printk(BAT_LOG_CRTI, "[BAT_probe] power_supply_register AC Success !!\n");

    ret = power_supply_register(&(dev->dev), &usb_main.psy);
    if (ret)
    {            
        battery_xlog_printk(BAT_LOG_CRTI, "[BAT_probe] power_supply_register USB Fail !!\n");                    
        return ret;
    }             
    battery_xlog_printk(BAT_LOG_CRTI, "[BAT_probe] power_supply_register USB Success !!\n");

    #if !defined(CONFIG_LGE_MINIABB)
    ret = power_supply_register(&(dev->dev), &wireless_main.psy);
    if (ret)
    {            
        battery_xlog_printk(BAT_LOG_CRTI, "[BAT_probe] power_supply_register WIRELESS Fail !!\n");                    
        return ret;
    }             
    battery_xlog_printk(BAT_LOG_CRTI, "[BAT_probe] power_supply_register WIRELESS Success !!\n");
    #endif
    
    ret = power_supply_register(&(dev->dev), &battery_main.psy);
    if (ret)
    {
        battery_xlog_printk(BAT_LOG_CRTI, "[BAT_probe] power_supply_register Battery Fail !!\n");
        return ret;
    }
    battery_xlog_printk(BAT_LOG_CRTI, "[BAT_probe] power_supply_register Battery Success !!\n");

#if !defined(CONFIG_POWER_EXT)
    /* For EM */
	{
	    int ret_device_file=0;
		
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Charger_Voltage);
	    
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_0_Slope);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_1_Slope);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_2_Slope);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_3_Slope);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_4_Slope);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_5_Slope);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_6_Slope);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_7_Slope);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_8_Slope);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_9_Slope);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_10_Slope);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_11_Slope);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_12_Slope);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_13_Slope);

	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_0_Offset);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_1_Offset);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_2_Offset);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_3_Offset);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_4_Offset);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_5_Offset);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_6_Offset);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_7_Offset);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_8_Offset);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_9_Offset);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_10_Offset);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_11_Offset);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_12_Offset);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_13_Offset);

	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_Is_Calibration);

	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_Power_On_Voltage);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_Power_Off_Voltage);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_Charger_TopOff_Value);
	    
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_Battery_CurrentConsumption);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_SW_CoulombCounter);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_Charging_CallState);
	//                                                          
		#if defined ( LGE_BSP_LGBM )
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_usb_cable); /* Using */
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_pseudo_batt);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdChargingModeOff);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdBattExist);
	//                                                          

		
   //                                                           
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdCharge);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdBatl);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdBatmp);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdChcomp);
	    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdFuelval);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_AtCmdUsbidadc);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_UpdateBatStateFlag);
#if defined (TARGET_S4)
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_LGBM_BatteryID);
#endif
   //                                                             
		#endif

	}
	
	//battery_meter_initial();	//move to mt_battery_GetBatteryData() to decrease booting time
	
	/* Initialization BMT Struct */
    BMT_status.bat_exist = KAL_TRUE;       /* phone must have battery */
    BMT_status.charger_exist = KAL_FALSE;     /* for default, no charger */
    BMT_status.bat_vol = 0;
    BMT_status.ICharging = 0;
    BMT_status.temperature = 0;
    BMT_status.charger_vol = 0;
    BMT_status.total_charging_time = 0;
    BMT_status.PRE_charging_time = 0;
    BMT_status.CC_charging_time = 0;
    BMT_status.TOPOFF_charging_time = 0;
    BMT_status.POSTFULL_charging_time = 0;
	BMT_status.SOC = 0;
	BMT_status.UI_SOC = 0;

    BMT_status.bat_charging_state = CHR_PRE;
	BMT_status.bat_in_recharging_state = KAL_FALSE;
	BMT_status.bat_full= KAL_FALSE;
	BMT_status.nPercent_ZCV = 0;
	BMT_status.nPrecent_UI_SOC_check_point= battery_meter_get_battery_nPercent_UI_SOC();


    //battery kernel thread for 10s check and charger in/out event
    /* Replace GPT timer by hrtime */
    battery_kthread_hrtimer_init();
	
    kthread_run(bat_thread_kthread, NULL, "bat_thread_kthread"); 
    battery_xlog_printk(BAT_LOG_CRTI, "[battery_probe] bat_thread_kthread Done\n");    
	//                                                          
		#if defined ( LGE_BSP_LGBM )
    //charger_hv_detect_sw_workaround_init();
	#endif
	//                                                          

    /*LOG System Set*/
    init_proc_log();

#endif   
	g_bat_init_flag = KAL_TRUE;
	
    return 0;
	
}


static int battery_remove(struct platform_device *dev)    
{
    battery_xlog_printk(BAT_LOG_CRTI, "******** battery driver remove!! ********\n" );

    return 0;
}

static void battery_shutdown(struct platform_device *dev)    
{
    battery_xlog_printk(BAT_LOG_CRTI, "******** battery driver shutdown!! ********\n" );

}

static int battery_suspend(struct platform_device *dev, pm_message_t state)    
{
	//                                                          
	#if defined ( LGE_BSP_LGBM )
	battery_xlog_printk(BAT_LOG_CRTI, "******** battery driver suspend!! ********\n" );
	#else
    struct timespec xts, tom;

    battery_xlog_printk(BAT_LOG_CRTI, "******** battery driver suspend!! ********\n" );

    get_xtime_and_monotonic_and_sleep_offset(&xts, &tom, &g_bat_time_before_sleep);
	#endif
	//                                                          
    return 0;
}


static int battery_resume(struct platform_device *dev)
{
#ifdef CONFIG_POWER_EXT
#else
	//                                                          
	#if defined ( LGE_BSP_LGBM )
    battery_xlog_printk(BAT_LOG_CRTI, "******** battery driver resume!! ********\n" );
    bat_thread_wakeup();
    #else
	kal_bool is_pcm_timer_trigger = KAL_FALSE;
	struct timespec xts, tom, bat_time_after_sleep;

	get_xtime_and_monotonic_and_sleep_offset(&xts, &tom, &bat_time_after_sleep);
	battery_charging_control(CHARGING_CMD_GET_IS_PCM_TIMER_TRIGGER,&is_pcm_timer_trigger);

	if(is_pcm_timer_trigger == KAL_TRUE)
	{
		battery_xlog_printk(BAT_LOG_CRTI, "battery resume by pcm timer!!\n" );
		mutex_lock(&bat_mutex);
		BAT_thread();
		mutex_unlock(&bat_mutex);
	}
	else
	{
		battery_xlog_printk(BAT_LOG_CRTI, "battery resume NOT by pcm timer!!\n" );
	}

	if(g_call_state == CALL_ACTIVE && (bat_time_after_sleep.tv_sec - g_bat_time_before_sleep.tv_sec >= TALKING_SYNC_TIME))	// phone call last than x min
	{
		BMT_status.UI_SOC = battery_meter_get_battery_percentage();
		battery_xlog_printk(BAT_LOG_CRTI, "Sync UI SOC to SOC immediately\n" );
	}	
	#endif
	//                                                          
#endif
    return 0;
}


///////////////////////////////////////////////////////////////////////////////////////////
//// Battery Notify API 
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_BatteryNotify(struct device *dev,struct device_attribute *attr, char *buf)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[Battery] show_BatteryNotify : %x\n", g_BatteryNotifyCode);
    
    return sprintf(buf, "%u\n", g_BatteryNotifyCode);
}
static ssize_t store_BatteryNotify(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    char *pvalue = NULL;
    unsigned int reg_BatteryNotifyCode = 0;
    battery_xlog_printk(BAT_LOG_CRTI, "[Battery] store_BatteryNotify\n");
    if(buf != NULL && size != 0)
    {
        battery_xlog_printk(BAT_LOG_CRTI, "[Battery] buf is %s and size is %d \n",buf,size);
        reg_BatteryNotifyCode = simple_strtoul(buf,&pvalue,16);
        g_BatteryNotifyCode = reg_BatteryNotifyCode;
        battery_xlog_printk(BAT_LOG_CRTI, "[Battery] store code : %x \n",g_BatteryNotifyCode);        
    }        
    return size;
}
static DEVICE_ATTR(BatteryNotify, 0664, show_BatteryNotify, store_BatteryNotify);

static ssize_t show_BN_TestMode(struct device *dev,struct device_attribute *attr, char *buf)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[Battery] show_BN_TestMode : %x\n", g_BN_TestMode);
    return sprintf(buf, "%u\n", g_BN_TestMode);
}
static ssize_t store_BN_TestMode(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    char *pvalue = NULL;
    unsigned int reg_BN_TestMode = 0;
    battery_xlog_printk(BAT_LOG_CRTI, "[Battery] store_BN_TestMode\n");
    if(buf != NULL && size != 0)
    {
        battery_xlog_printk(BAT_LOG_CRTI, "[Battery] buf is %s and size is %d \n",buf,size);
        reg_BN_TestMode = simple_strtoul(buf,&pvalue,16);
        g_BN_TestMode = reg_BN_TestMode;
        battery_xlog_printk(BAT_LOG_CRTI, "[Battery] store g_BN_TestMode : %x \n",g_BN_TestMode);        
    }        
    return size;
}
static DEVICE_ATTR(BN_TestMode, 0664, show_BN_TestMode, store_BN_TestMode);


///////////////////////////////////////////////////////////////////////////////////////////
//// platform_driver API 
///////////////////////////////////////////////////////////////////////////////////////////
#if 0
static int battery_cmd_read(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
    int len = 0;
    char *p = buf;
    
    p += sprintf(p, "g_battery_thermal_throttling_flag=%d,\nbattery_cmd_thermal_test_mode=%d,\nbattery_cmd_thermal_test_mode_value=%d\n", 
        g_battery_thermal_throttling_flag, battery_cmd_thermal_test_mode, battery_cmd_thermal_test_mode_value);
    
    *start = buf + off;
    
    len = p - buf;
    if (len > off)
        len -= off;
    else
        len = 0;
    
    return len < count ? len  : count;
}
#endif

static ssize_t battery_cmd_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
    int len = 0, bat_tt_enable=0, bat_thr_test_mode=0, bat_thr_test_value=0;
    char desc[32];
    
    len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
    if (copy_from_user(desc, buffer, len))
    {
        return 0;
    }
    desc[len] = '\0';
    
    if (sscanf(desc, "%d %d %d", &bat_tt_enable, &bat_thr_test_mode, &bat_thr_test_value) == 3)
    {
        g_battery_thermal_throttling_flag = bat_tt_enable;
        battery_cmd_thermal_test_mode = bat_thr_test_mode;
        battery_cmd_thermal_test_mode_value = bat_thr_test_value;
        
        battery_xlog_printk(BAT_LOG_CRTI, "bat_tt_enable=%d, bat_thr_test_mode=%d, bat_thr_test_value=%d\n", 
            g_battery_thermal_throttling_flag, battery_cmd_thermal_test_mode, battery_cmd_thermal_test_mode_value);
        
        return count;
    }
    else
    {
        battery_xlog_printk(BAT_LOG_CRTI, "  bad argument, echo [bat_tt_enable] [bat_thr_test_mode] [bat_thr_test_value] > battery_cmd\n");
    }
    
    return -EINVAL;
}

static int proc_utilization_show(struct seq_file *m, void *v)
{
    seq_printf(m, "=> g_battery_thermal_throttling_flag=%d,\nbattery_cmd_thermal_test_mode=%d,\nbattery_cmd_thermal_test_mode_value=%d\n", 
        g_battery_thermal_throttling_flag, battery_cmd_thermal_test_mode, battery_cmd_thermal_test_mode_value);
    return 0;
}

static int proc_utilization_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_utilization_show, NULL);
}

static const struct file_operations battery_cmd_proc_fops = { 
    .open  = proc_utilization_open, 
    .read  = seq_read,
    .write = battery_cmd_write,
};

static int mt_batteryNotify_probe(struct platform_device *dev)    
{
    int ret_device_file = 0;
    //struct proc_dir_entry *entry = NULL;
    struct proc_dir_entry *battery_dir = NULL;

    battery_xlog_printk(BAT_LOG_CRTI, "******** mt_batteryNotify_probe!! ********\n" );


    ret_device_file = device_create_file(&(dev->dev), &dev_attr_BatteryNotify);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_BN_TestMode);
    
    battery_dir = proc_mkdir("mtk_battery_cmd", NULL);
    if (!battery_dir)
    {
        pr_err("[%s]: mkdir /proc/mtk_battery_cmd failed\n", __FUNCTION__);
    }
    else
    {
        #if 1
        proc_create("battery_cmd", S_IRUGO | S_IWUSR, battery_dir, &battery_cmd_proc_fops);
        battery_xlog_printk(BAT_LOG_CRTI, "proc_create battery_cmd_proc_fops\n");
        #else
        entry = create_proc_entry("battery_cmd", S_IRUGO | S_IWUSR, battery_dir);
        if (entry)
        {
            entry->read_proc = battery_cmd_read;
            entry->write_proc = battery_cmd_write;
        }
        #endif
    }

    battery_xlog_printk(BAT_LOG_CRTI, "******** mtk_battery_cmd!! ********\n" );    
		
    return 0;

}

struct platform_device battery_device = {
    .name   = "battery",
    .id        = -1,
};

static struct platform_driver battery_driver = {
    .probe         = battery_probe,
    .remove        = battery_remove,
    .shutdown      = battery_shutdown,
    //#ifdef CONFIG_PM
    .suspend       = battery_suspend,
    .resume        = battery_resume,
    //#endif
    .driver        = {
        .name = "battery",
    },
};

struct platform_device MT_batteryNotify_device = {
    .name   = "mt-battery",
    .id        = -1,
};

static struct platform_driver mt_batteryNotify_driver = {
    .probe        = mt_batteryNotify_probe,
    .driver     = {
        .name = "mt-battery",
    },
};

static int __init battery_init(void)
{
    int ret;

    ret = platform_device_register(&battery_device);
    if (ret) {
        battery_xlog_printk(BAT_LOG_CRTI, "****[battery_driver] Unable to device register(%d)\n", ret);
    return ret;
    }
    
    ret = platform_driver_register(&battery_driver);
    if (ret) {
        battery_xlog_printk(BAT_LOG_CRTI, "****[battery_driver] Unable to register driver (%d)\n", ret);
    return ret;
    }

    // battery notofy UI
    ret = platform_device_register(&MT_batteryNotify_device);
    if (ret) {
        battery_xlog_printk(BAT_LOG_CRTI, "****[mt_batteryNotify] Unable to device register(%d)\n", ret);
        return ret;
    }    
    ret = platform_driver_register(&mt_batteryNotify_driver);
    if (ret) {
        battery_xlog_printk(BAT_LOG_CRTI, "****[mt_batteryNotify] Unable to register driver (%d)\n", ret);
        return ret;
    }

    battery_xlog_printk(BAT_LOG_CRTI, "****[battery_driver] Initialization : DONE !!\n");
    return 0;
}

static void __exit battery_exit (void)
{
}

module_init(battery_init);
module_exit(battery_exit);

MODULE_AUTHOR("Oscar Liu");
MODULE_DESCRIPTION("Battery Device Driver");
MODULE_LICENSE("GPL");

