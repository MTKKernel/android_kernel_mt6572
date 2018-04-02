/*
 * Copyright (C) 2013 LG Electironics, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/****************************************************************************
* Include Files
****************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif 
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/rtpm_prio.h>
#include <linux/proc_fs.h>
#include <linux/jiffies.h>
#include <linux/spinlock.h>
#include <linux/wakelock.h>
#include <linux/dma-mapping.h>
#include <linux/input/mt.h>
#include <linux/file.h>		//for file access
#include <linux/syscalls.h> //for file access
#include <linux/uaccess.h>  //for file access

#include <mach/wd_api.h>
#include <mach/eint.h>
#include <mach/mt_wdt.h>
#include <mach/mt_gpt.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

#include <asm/uaccess.h>
#include <cust_eint.h>

#include "tpd.h"
#include "lu201x_driver.h"

#ifdef TARGET_S2
#include "lu201x_firmware_l30.h"
#else
#include "lu201x_firmware_l20.h"
#endif


/****************************************************************************
* Constants / Definitions
****************************************************************************/
#define LGE_TOUCH_NAME				"lge_touch"

#define TPD_DEV_NAME				"lu201x"
#define TPD_I2C_ADDRESS				0x0E
#define I2C_DEVICE_ADDRESS_LEN		2
#define MAX_TRANSACTION_LENGTH		8
#define MAX_I2C_TRANSFER_SIZE		(MAX_TRANSACTION_LENGTH - I2C_DEVICE_ADDRESS_LEN)


#define FW_STATUS_REG				0x0000
#define FW_VERSION_REG				0x0080

#define KNOCK_TAP_COUNT				0x0082
#define KNOCK_TAP_POINT				0x0083
#define KNOCK_STATUS				0x00C0
#define KNOCK_TAP_THON				0x00C1
#define KNOCK_EXCEPT_PALM_ONCH		0x00C5
#define KNOCK_WAKEUP_INTERVAL		0x00C6
#define KNOCK_TAPOFF_TIMEOUT		0x00C9
#define KNOCK_TAP_MAXCOUNT			0x00CB

#define LU201x_MODE_REG				0x00E0
#define LU201x_CMDACK_REG			0x00ED
#define LU201x_DEVICEID_ADDR		0x10FD
#define LU201x_I2C_DONE_REG			0x10FF
#define LU201x_CMDReply_REG			0x0100

#define CMD_I2C_DONE                0x01
#define CMD_LU201x_CHANGEMODE       0xA3
#define CMD_LU201x_DEVICEINFO       0xAA
#define CMD_LU201x_CHCAPTEST		0xB6

#define FWSTATUS_NORMAL 		    0x00
#define FWSTATUS_CHFAIL 		   	0x01
#define FWSTATUS_CALFAIL			0x02

#define CMD_LU201x_NORMODE          0x00
#define CMD_LU201x_PDN              0x01
#define CMD_LU201x_DEBUG            0x02
#define CMD_LU201x_IDLE_DOUBLETAB	0x11
#define CMD_LU201x_IDLE_MULTITAB	0x11

#define EVENT_NONE					0x00
#define EVENT_ABS					0x01
#define EVENT_KEY					0x02
#define EVENT_GEST					0x04
#define EVENT_MOUSE					0x08

#define TYPE_PRESS					0x01
#define TYPE_RELEASE				0x03

#define TOUCH_PRESSED				1
#define TOUCH_RELEASED				0
#define CANCEL_KEY					0xFF


#define MAX_FINGER_NUM				2
#define MAX_POINT_SIZE_FOR_LPWG		12

#define TPD_HAVE_BUTTON
#ifdef TPD_HAVE_BUTTON
#ifdef LGE_USE_DOME_KEY
#define TPD_KEY_COUNT	2
static int tpd_keys_local[TPD_KEY_COUNT] = { KEY_BACK , KEY_MENU };
#else
#define TPD_KEY_COUNT	4
static int tpd_keys_local[TPD_KEY_COUNT] = { KEY_BACK, KEY_HOMEPAGE, KEY_MENU, KEY_SSK };
#endif
#endif


/****************************************************************************
* Type Definitions
****************************************************************************/
typedef struct {
	u8 FWStatus;		// 0x0000
	u8 EventType;		// 0x0001
	u8 VPCount; 		// 0x0002
	u8 KeyData[2];		// 0x0003
	u8 Point[8];		// 0x0005 X0 position
} lu201x_tpd;

typedef struct {
	u8 status[MAX_FINGER_NUM];
	u8 id[MAX_FINGER_NUM];
	u16 x[MAX_FINGER_NUM];
	u16 y[MAX_FINGER_NUM];
} touch_info;

enum {
	LPWG_READ = 1,
	LPWG_ENABLE,
	LPWG_LCD_X,
	LPWG_LCD_Y,
	LPWG_ACTIVE_AREA_X1,
	LPWG_ACTIVE_AREA_X2,
	LPWG_ACTIVE_AREA_Y1,
	LPWG_ACTIVE_AREA_Y2,
	LPWG_TAP_COUNT,
	LPWG_REPLY,
};

enum {
	LPWG_NONE = 0,
	LPWG_DOUBLE_TAP,
	LPWG_MULTI_TAP,
};

struct point {
    int x;
    int y;
};

struct foo_obj{
	struct kobject kobj;
	int interrupt;
};


/****************************************************************************
* Variables
****************************************************************************/
static int touch_pressed_check = 0;
static int suspend_status = 0;
static int pressed_key = 0;
static bool key_lock_status;

static char knock_on_type;
static int knock_on_enable = 0;
static u8 double_tap_enable = 0;
static u8 multi_tap_enable = 0;
static u8 multi_tap_count;
static u8 lpwg_mode = 0;
static u8 KnockCode_over_check = 0;
static u8 gesture_property[MAX_POINT_SIZE_FOR_LPWG*4] = {0};
static struct point lpwg_data[MAX_POINT_SIZE_FOR_LPWG+1];
static struct foo_obj *foo_obj;
static struct wake_lock knock_code_lock;
static char *lpwg_uevent[2][2] = { { "TOUCH_GESTURE_WAKEUP=WAKEUP", NULL },	{ "TOUCH_GESTURE_WAKEUP=PASSWORD", NULL } };


struct device *touch_fw_dev;
struct class *touch_class;

extern struct tpd_device *tpd;
struct i2c_client *LU201x_i2c_client = NULL;
static int tpd_flag = 0;


static DEFINE_MUTEX(i2c_access);
static DECLARE_WAIT_QUEUE_HEAD(tpd_waiter);


/****************************************************************************
* Extern Function Prototypes
****************************************************************************/
extern int mtk_wdt_enable ( enum wk_wdt_en en );
extern SEL_SYSFS;


/****************************************************************************
* Local Function Prototypes
****************************************************************************/
static void touch_eint_interrupt_handler ( void );
static int LU201x_do_update_bin ( char *path );


/****************************************************************************
* Platform(AP) dependent functions
****************************************************************************/
void LU201x_setup_eint ( void )
{
	TPD_FUN ();

	/* Configure GPIO settings for external interrupt pin  */
	mt_set_gpio_dir ( GPIO_CTP_EINT_PIN, GPIO_DIR_IN );
	mt_set_gpio_mode ( GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT );
	mt_set_gpio_pull_enable ( GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE );
	mt_set_gpio_pull_select ( GPIO_CTP_EINT_PIN, GPIO_PULL_UP );

	msleep(50);

	/* Configure external interrupt settings for external interrupt pin */
	mt65xx_eint_set_sens ( CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE );
	mt65xx_eint_set_hw_debounce ( CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN );
	mt65xx_eint_registration ( CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_POLARITY_LOW, touch_eint_interrupt_handler, 1 );

	/* unmask external interrupt */
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
}

void LU201x_reset ( unsigned int on )
{
	TPD_FUN ();

	if ( on )
	{
		msleep ( 10 );
		TPD_LOG ( "Reset pin is set high\n" );
		mt_set_gpio_out ( GPIO_TOUCH_RESET_PIN, 1 );
		msleep ( 100 );
	}
	else
	{
		TPD_LOG ( "Reset pin is set low\n" );
		mt_set_gpio_out ( GPIO_TOUCH_RESET_PIN, 0 );
	}
}

void LU201x_power ( unsigned int on )
{
	TPD_FUN ();

	mt_set_gpio_mode ( GPIO_TOUCH_RESET_PIN, GPIO_TOUCH_RESET_PIN_M_GPIO );
	mt_set_gpio_dir ( GPIO_TOUCH_RESET_PIN, GPIO_DIR_OUT );

	if ( on )
	{
		hwPowerOn ( MT6323_POWER_LDO_VGP2, VOL_3000, "TP" );
		TPD_LOG ( "turned on the power ( VGP2 )\n" );
		LU201x_reset ( 1 );
		msleep ( 300 );
	}
	else
	{
		LU201x_reset ( 0 );
		hwPowerDown ( MT6323_POWER_LDO_VGP2, "TP" );
		TPD_LOG ( "turned off the power ( VGP2 )\n" );
	}
}

/****************************************************************************
* LU201x  I2C  Read / Write Funtions
****************************************************************************/
static int LU201x_i2c_read_bytes ( struct i2c_client *client, u16 addr, u8 *rxbuf, int len )
{
	u8 buffer[I2C_DEVICE_ADDRESS_LEN];
	u8 retry;
	u16 left = len;
	u16 offset = 0;

	struct i2c_msg msg[2] =
	{
		{
			.addr = client->addr,
			.flags = 0,
			.len = I2C_DEVICE_ADDRESS_LEN,
			.buf = buffer,
			//.timing = 400
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			//.timing = 400
		},
	};

	if ( rxbuf == NULL )
		return -1;

	while ( left > 0 )
	{
		buffer[0] = ( ( addr + offset ) >> 8 ) & 0xFF;
		buffer[1] = ( addr + offset ) & 0xFF;

		msg[1].buf = &rxbuf[offset];

		if ( left > MAX_TRANSACTION_LENGTH )
		{
			msg[1].len = MAX_TRANSACTION_LENGTH;
			left -= MAX_TRANSACTION_LENGTH;
			offset += MAX_TRANSACTION_LENGTH;
		}
		else
		{
			msg[1].len = left;
			left = 0;
		}

		retry = 0;

		while ( i2c_transfer ( client->adapter, &msg[0], 2 ) != 2 )
		{
			retry++;

			if ( retry == 3 )
			{
				TPD_ERR ( "I2C read 0x%X length=%d failed\n", addr + offset, len );
				return -1;
			}
		}
	}

	return 0;
}

static int LU201x_i2c_write_bytes ( struct i2c_client *client, u16 addr, u8 *txbuf, int len )
{
	u8 buffer[MAX_TRANSACTION_LENGTH];
	u16 left = len;
	u16 offset = 0;
	u8 retry = 0;

	struct i2c_msg msg = 
	{
		.addr = client->addr,
		.flags = 0,
		.buf = buffer,
		.len = len
	};

	if ( txbuf == NULL )
		return -1;

	while ( left > 0 )
	{
		retry = 0;

		buffer[0] = ( ( addr + offset ) >> 8 ) & 0xFF;
		buffer[1] = ( addr+offset ) & 0xFF;

		if ( left > MAX_I2C_TRANSFER_SIZE )
		{
			memcpy ( &buffer[I2C_DEVICE_ADDRESS_LEN], &txbuf[offset], MAX_I2C_TRANSFER_SIZE );
			msg.len = MAX_TRANSACTION_LENGTH;
			left -= MAX_I2C_TRANSFER_SIZE;
			offset += MAX_I2C_TRANSFER_SIZE;
		}
		else
		{
			memcpy ( &buffer[I2C_DEVICE_ADDRESS_LEN], &txbuf[offset], left );
			msg.len = left + I2C_DEVICE_ADDRESS_LEN;
			left = 0;
		}

		while ( i2c_transfer ( client->adapter, &msg, 1 ) != 1 )
		{
			retry++;

			if ( retry == 3 )
			{
				TPD_ERR ( "I2C write 0x%X%X length=%d failed\n", buffer[0], buffer[1], len );
				return -1;
			}
			else
				TPD_ERR ( "I2C write retry %d addr 0x%X%X\n", retry, buffer[0], buffer[1] );
		}
	}

	return 0;
}

static void LU201x_i2c_done ( void )
{
	u8 i2c_done = CMD_I2C_DONE;

	LU201x_i2c_write_bytes ( LU201x_i2c_client, LU201x_I2C_DONE_REG, &i2c_done, 1 );
}

static int LU201x_intPort_check ( void )
{
	return mt_get_gpio_in ( GPIO_CTP_EINT_PIN );
}

static void LU201x_set_i2c_mode ( void )
{
	mt_set_gpio_mode ( GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_GPIO );
	mt_set_gpio_dir ( GPIO_CTP_EINT_PIN, GPIO_DIR_OUT );
	mt_set_gpio_out ( GPIO_CTP_EINT_PIN, GPIO_OUT_ZERO );
	msleep ( 1 );
	mt_set_gpio_out ( GPIO_CTP_EINT_PIN, GPIO_OUT_ONE );
	mt_set_gpio_dir ( GPIO_CTP_EINT_PIN, GPIO_DIR_IN );
	msleep ( 1 );
}

static int LU201x_wait_ready_i2c ( void )
{
	int ret = 0, dlycnt = 500;

  	// wait till interrup pin goes low
	while ( LU201x_intPort_check() && dlycnt )	// wait if inperrupt pin is high
	{
		msleep ( 1 );
		dlycnt--;
	}

	if ( dlycnt == 0 )
		ret = -1;

	return ret;
}

static int LU201x_read_register ( u16 addr,u8 *buf,u8 size )
{
	int ret = 1;

	LU201x_set_i2c_mode ();
	ret = LU201x_wait_ready_i2c ();

	if ( ret == -1 )
	{
		TPD_ERR ( "Register Read failure\n" );
	}
	else
	{
		ret = LU201x_i2c_read_bytes ( LU201x_i2c_client, addr, buf, size );
		LU201x_i2c_done ();
	}

	mt_set_gpio_mode ( GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT );

	return ret;
}

static int LU201x_set_mode ( u8 mode )
{
	u8 buf[3] = { CMD_LU201x_CHANGEMODE, 0, 0 };
	u8 temp = 0;
	u16 tab_dlytime = 0;
	int ret = -1;

	TPD_LOG ( "set_mode = 0x%x\n", mode );
	buf[1] = mode;

 	LU201x_set_i2c_mode ();

	if ( LU201x_wait_ready_i2c() == -1 )
	{
		TPD_ERR ( "Set mode Int ready error\n" );
		goto exit;
	}

	LU201x_i2c_write_bytes ( LU201x_i2c_client, LU201x_MODE_REG, buf, 3 );

	if ( lpwg_mode == LPWG_DOUBLE_TAP )
	{
		temp = 1;
		LU201x_i2c_write_bytes ( LU201x_i2c_client, KNOCK_STATUS, &temp, 1 );
		temp = 150;
		LU201x_i2c_write_bytes ( LU201x_i2c_client, KNOCK_TAP_THON, &temp, 1 );
		temp = 8;
		LU201x_i2c_write_bytes ( LU201x_i2c_client, KNOCK_EXCEPT_PALM_ONCH, &temp, 1 );
		temp = 70;
		LU201x_i2c_write_bytes ( LU201x_i2c_client, KNOCK_WAKEUP_INTERVAL, &temp, 1 );
		multi_tap_count = 2;
		LU201x_i2c_write_bytes ( LU201x_i2c_client, KNOCK_TAP_MAXCOUNT, &multi_tap_count, 1 );
	}
	else if ( lpwg_mode == LPWG_MULTI_TAP )
	{
		temp = 2;
		LU201x_i2c_write_bytes ( LU201x_i2c_client, KNOCK_STATUS, &temp, 1 );
		temp = 150;
		LU201x_i2c_write_bytes ( LU201x_i2c_client, KNOCK_TAP_THON, &temp, 1 );
		temp = 8;
		LU201x_i2c_write_bytes ( LU201x_i2c_client, KNOCK_EXCEPT_PALM_ONCH, &temp, 1 );
		temp = 70;
		LU201x_i2c_write_bytes ( LU201x_i2c_client, KNOCK_WAKEUP_INTERVAL, &temp, 1 );
		tab_dlytime = 200;
		LU201x_i2c_write_bytes ( LU201x_i2c_client, KNOCK_TAPOFF_TIMEOUT, &tab_dlytime, 2 );
		temp = multi_tap_count;
		LU201x_i2c_write_bytes ( LU201x_i2c_client, KNOCK_TAP_MAXCOUNT, &temp, 1 );
	}
	LU201x_i2c_done ();
	msleep ( 2 );

	if ( LU201x_wait_ready_i2c() == -1 )
	{
		TPD_ERR ( "Set mode read ready error\n" );
		goto exit;
	}

	LU201x_i2c_read_bytes ( LU201x_i2c_client, LU201x_CMDACK_REG, buf, 3 );
	LU201x_i2c_done ();

	if ( buf[2] != CMD_LU201x_CHANGEMODE )
	{
		TPD_ERR ( "write reg failed! %#x ret: %d", buf[0], ret );
		goto exit;
	}
	ret = buf[2];

exit:
	mt_set_gpio_mode ( GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT );

	return ret;
}

static void LU201x_release_all_finger ( void )
{
	TPD_FUN ();

	input_report_key ( tpd->dev, BTN_TOUCH, 0 );
	input_mt_sync ( tpd->dev );
	input_sync ( tpd->dev );
}

void touch_keylock_enable ( int key_lock )
{
	TPD_FUN ();

	if ( !key_lock )
	{
		mt65xx_eint_unmask ( CUST_EINT_TOUCH_PANEL_NUM );
		key_lock_status = 0;
	}
	else
	{
		mt65xx_eint_mask ( CUST_EINT_TOUCH_PANEL_NUM );
		key_lock_status = 1;
	}
}
EXPORT_SYMBOL ( touch_keylock_enable );

/****************************************************************************
* Touch Knock_On Funtions
****************************************************************************/
static int touch_knock_check ( u8 tap )
{
	TPD_FUN ();
	int ret;

	TPD_LOG ( "Knock Tap Count = %d\n", tap );

	ret = LU201x_i2c_read_bytes ( LU201x_i2c_client, KNOCK_TAP_POINT, gesture_property, multi_tap_count * 4 );
	if ( ret != 0 )
	{
		LU201x_i2c_done();
		TPD_ERR ( "KNOCK_TAP_POINT read fail\n" );
		return -1;
	}
	LU201x_i2c_done();

	if ( double_tap_enable && ( tap == 2 ) )
	{
		TPD_LOG ( "Knock On occured!!\n" );
		kobject_uevent_env ( &foo_obj->kobj, KOBJ_CHANGE, lpwg_uevent[LPWG_DOUBLE_TAP-1] );
	}
	else if ( multi_tap_enable )
	{
		if ( multi_tap_count != tap )
		{
			KnockCode_over_check = 1;
		}

		TPD_LOG ( "Knock Code occured!!\n" );
		kobject_uevent_env ( &foo_obj->kobj, KOBJ_CHANGE, lpwg_uevent[LPWG_MULTI_TAP-1] );
	}

	return 0;
}

static int touch_knock_lpwg ( struct i2c_client *client, u32 code, u32 value, struct point *data )
{
	u8 buf = 0;
	int i = 0, ret = 0;

	switch ( code )
	{
		case LPWG_READ:
			if ( multi_tap_enable )
			{
				if ( KnockCode_over_check )
				{
					data[0].x = 1;
					data[0].y = 1;
					data[1].x = -1;
					data[1].y = -1;
					KnockCode_over_check = 0;
					break;
				}

				for ( i = 0 ; i < multi_tap_count ; i++ )
				{
					data[i].x = ( gesture_property[4*i+1] << 8 | gesture_property[4*i] );
					data[i].y = ( gesture_property[4*i+3] << 8 | gesture_property[4*i+2] );
					TPD_LOG ( "TAP Position x[%3d], y[%3d]\n", data[i].x, data[i].y );
					// '-1' should be assinged to the last data.
					// Each data should be converted to LCD-resolution.
				}
				data[i].x = -1;
				data[i].y = -1;
			}
			break;

		case LPWG_ENABLE:
			lpwg_mode = value;
			TPD_LOG ( "lpwg_mode=%d, double_tap_enable=%d, multi_tap_enable=%d", lpwg_mode, double_tap_enable, multi_tap_enable );

			// The 'lpwg_mode' is changed to 'value' but it is applied in suspend-state.
			break;

		case LPWG_LCD_X:
			break;

		case LPWG_LCD_Y:
			break;

		case LPWG_ACTIVE_AREA_X1:
		case LPWG_ACTIVE_AREA_X2:
		case LPWG_ACTIVE_AREA_Y1:
		case LPWG_ACTIVE_AREA_Y2:
			// Quick Cover Area
			break;

		case LPWG_TAP_COUNT:
			if ( value )
			{
				multi_tap_count = value;
			}
			break;

		case LPWG_REPLY:
			break;

		default:
			break;
	}

	return 0;
}

/****************************************************************************
* Touch Interrupt Service Routines
****************************************************************************/
static void touch_eint_interrupt_handler ( void )
{
	mt65xx_eint_mask ( CUST_EINT_TOUCH_PANEL_NUM );
	tpd_flag = 1;
	wake_up_interruptible ( &tpd_waiter );
}

static int LU201x_touch_event_handler ( void *unused )
{
	int i, ret = 0;
	int press_count;
	u8 tap_count = 0;
	lu201x_tpd touch_data;
	touch_info info;
	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD }; 

	sched_setscheduler ( current, SCHED_RR, &param );

	do
	{
		set_current_state ( TASK_INTERRUPTIBLE );
		wait_event_interruptible ( tpd_waiter, tpd_flag != 0 );

		tpd_flag = 0;
		set_current_state ( TASK_RUNNING );
		mutex_lock ( &i2c_access );

		/* Knock On or Knock Code Check */
		if ( suspend_status && knock_on_enable )
		{
			ret = LU201x_i2c_read_bytes ( LU201x_i2c_client, KNOCK_TAP_COUNT, &tap_count, 1 );
			if ( ( ret != 0 ) || ( tap_count < 2 ) )
			{
				LU201x_i2c_done ();
				TPD_ERR ( "KNOCK_TAP_COUNT error\n" );
				goto exit_work;
			}

			wake_lock ( &knock_code_lock );
			ret = touch_knock_check ( tap_count );
			if ( ret != 0 )
			{
				TPD_ERR ( "touch_knock_check fail\n" );
				wake_unlock ( &knock_code_lock );
			}

			goto exit_work;
		}

		press_count = 0;
		memset(&info, 0x0, sizeof(touch_info));
		memset(&touch_data, 0x0, sizeof(lu201x_tpd));

		/* read touch data */
		if ( LU201x_i2c_read_bytes ( LU201x_i2c_client, FW_STATUS_REG, (u8*)&touch_data , sizeof(touch_data) ) == 0 )
		{
			LU201x_i2c_done ();
		}
		else
		{
			LU201x_i2c_done ();
			TPD_ERR ( " LU201x I2C Communication error\n" );

			LU201x_reset ( 0 );
			LU201x_reset ( 1 );
			LU201x_release_all_finger ();

			goto exit_work;
		}


		/* Event Processing */
		if ( touch_data.EventType == EVENT_ABS )
		{
			for ( i = 0 ; i < touch_data.VPCount ; i++ )
			{
				info.x[i] = ( ( touch_data.Point[i*4+1] & 0x07 ) << 8 ) | touch_data.Point[i*4];
				info.y[i] = ( ( touch_data.Point[i*4+2] & 0x38 ) << 5 ) | ( ( touch_data.Point[i*4+2] & 0x07 ) << 5 ) | ( ( touch_data.Point[i*4+1] >> 3 ) & 0x1f );
				info.id[i] = ( ( touch_data.Point[i*4+3] & 0x07 ) << 3 ) | ( ( touch_data.Point[i*4+2] >> 6 ) & 0x03 );
				info.status[i] = ( touch_data.Point[i*4+3] >> 3 ) & 0x03;

				if ( info.status[i] == TYPE_PRESS )
				{
					input_report_key(tpd->dev, BTN_TOUCH, 1);
					input_report_abs ( tpd->dev, ABS_MT_TRACKING_ID, info.id[i] );
					input_report_abs ( tpd->dev, ABS_MT_POSITION_X, info.x[i] );
					input_report_abs ( tpd->dev, ABS_MT_POSITION_Y, info.y[i] );
					input_report_abs ( tpd->dev, ABS_MT_TOUCH_MAJOR, 20 );
					input_report_abs ( tpd->dev, ABS_MT_PRESSURE, 20 );
					input_mt_sync ( tpd->dev );

					press_count++;
					if ( touch_pressed_check == 0 )
					{
						TPD_LOG ( "[Jun] Touch_pressed START!\n" );
						touch_pressed_check = 1;
					}

					if ( pressed_key != 0 )
					{
						input_report_key ( tpd->dev, tpd_keys_local[pressed_key-1], CANCEL_KEY );
						TPD_LOG ( "Touch Key[%d] is canceled!\n", pressed_key );
						pressed_key = 0;
					}
				}
			}

			if ( press_count == 0 )
			{
				input_report_key ( tpd->dev, BTN_TOUCH, 0 );
				input_mt_sync ( tpd->dev );
				TPD_LOG ( "[Jun] Touch_pressed END!\n" );
				touch_pressed_check = 0;
			}

			input_sync ( tpd->dev );
		}
		else if ( touch_data.EventType == EVENT_KEY )
		{
			if ( touch_data.KeyData[0] == 0 )
			{
				input_report_key ( tpd->dev, tpd_keys_local[pressed_key-1], TOUCH_RELEASED );
				TPD_LOG ( "Touch Key[%d] is released\n", pressed_key );
				pressed_key = 0;
			}
			else
			{
				pressed_key = touch_data.KeyData[0];
				input_report_key ( tpd->dev, tpd_keys_local[pressed_key-1], TOUCH_PRESSED );
				TPD_LOG ( "Touch Key[%d] is pressed\n", pressed_key );
			}

			input_sync ( tpd->dev );
		}

exit_work:
		mutex_unlock ( &i2c_access );
		mt65xx_eint_unmask ( CUST_EINT_TOUCH_PANEL_NUM );
	} while ( !kthread_should_stop() );

	return 0;
}

/****************************************************************************
* SYSFS function for Touch
****************************************************************************/
static ssize_t LU201x_store_firmware ( struct device *dev, struct device_attribute *attr, const char *buf, size_t size )
{
	int ret;

	ret = LU201x_do_update_bin ( "/mnt/sdcard/LU201xRom.bin" );
	if ( ret == 0 )
	{
		TPD_LOG ( "update tsp success\n" );
	}
	else
	{
		TPD_ERR ( "update tsp fail\n" );
	}

	LU201x_reset ( 0 );
	LU201x_reset ( 1 );

	return size;
}
static DEVICE_ATTR ( firmware, 0664, NULL, LU201x_store_firmware );

static ssize_t LU201x_show_version ( struct device *dev, struct device_attribute *attr, char *buf )
{
	u8 fw_ver[2] = { 0, };
	int ret;

	ret = LU201x_read_register ( FW_VERSION_REG, &fw_ver[0], 2 );
	if ( ret < 0 )
	{
		TPD_LOG ( "FW_VERSION_REG read fail\n" );
	}

	return sprintf ( buf, "%d.%d\n", fw_ver[0], fw_ver[1] );
}
static DEVICE_ATTR ( version, 0664, LU201x_show_version, NULL );

static ssize_t show_knock_on_type ( struct device *dev, struct device_attribute *attr, char *buf )
{
	int ret = 0;

	ret = sprintf ( buf, "%d\n", knock_on_type );

	return ret;
}
static DEVICE_ATTR ( knock_on_type, 0664, show_knock_on_type, NULL );

static ssize_t show_lpwg_data ( struct device *dev, struct device_attribute *attr, char *buf )
{
	TPD_FUN ();
	int i = 0, ret = 0;

	memset(lpwg_data, 0, sizeof(struct point)*MAX_POINT_SIZE_FOR_LPWG);

	touch_knock_lpwg ( LU201x_i2c_client, LPWG_READ, 0, lpwg_data );
	for ( i = 0 ; i < MAX_POINT_SIZE_FOR_LPWG ; i++ )
	{
		if ( lpwg_data[i].x == -1 && lpwg_data[i].y == -1 )
		{
			break;
		}
		ret += sprintf ( buf+ret, "%d %d\n", lpwg_data[i].x, lpwg_data[i].y );
	}

	return ret;
}

static ssize_t store_lpwg_data ( struct device *dev, struct device_attribute *attr, const char *buf, size_t size )
{
	TPD_FUN ();
	int reply = 0;

	sscanf ( buf, "%d", &reply );
	TPD_LOG ( "LPWG RESULT = %d ", reply );

	touch_knock_lpwg ( LU201x_i2c_client, LPWG_REPLY, reply, NULL );

	wake_unlock ( &knock_code_lock );

	return size;
}
static DEVICE_ATTR ( lpwg_data, 0664, show_lpwg_data, store_lpwg_data );

static ssize_t store_lpwg_notify ( struct device *dev, struct device_attribute *attr, const char *buf, size_t size )
{
	static int suspend = 0;
	int type = 0;
	int value[4] = {0};

	sscanf ( buf, "%d %d %d %d %d", &type, &value[0], &value[1], &value[2], &value[3] );
	TPD_LOG ( "touch notify type = %d , value[0] = %d, value[1] = %d, valeu[2] = %d, value[3] = %d ", type, value[0], value[1], value[2], value[3] );

	switch ( type )
	{
		case 1:
			touch_knock_lpwg ( LU201x_i2c_client, LPWG_ENABLE, value[0], NULL );

			if ( value[0] )
			{
				knock_on_enable = 1;
			}
			else
			{
				knock_on_enable = 0;
			}
			break;

        case 2:
			touch_knock_lpwg ( LU201x_i2c_client, LPWG_LCD_X, value[0], NULL );
			touch_knock_lpwg ( LU201x_i2c_client, LPWG_LCD_Y, value[1], NULL );
			break;

        case 3:
			touch_knock_lpwg ( LU201x_i2c_client, LPWG_ACTIVE_AREA_X1, value[0], NULL );
			touch_knock_lpwg ( LU201x_i2c_client, LPWG_ACTIVE_AREA_X2, value[1], NULL );
			touch_knock_lpwg ( LU201x_i2c_client, LPWG_ACTIVE_AREA_Y1, value[2], NULL );
			touch_knock_lpwg ( LU201x_i2c_client, LPWG_ACTIVE_AREA_Y2, value[3], NULL );
			break;

        case 4:
			touch_knock_lpwg ( LU201x_i2c_client, LPWG_TAP_COUNT, value[0], NULL );
			break;

		case 6:
			break;

		default:
			break;
		}

	return size;
}
static DEVICE_ATTR ( lpwg_notify, 0664, NULL, store_lpwg_notify );

static void sd_write_log ( char *data )
{
	int fd;
	char *fname = "/mnt/sdcard/touch_self_test.txt";

	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND, 0644);

	if (fd >= 0)
	{
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	}

	set_fs(old_fs);
}

static void LU201x_ChCapTest ( void )
{
	u8 buf[3] = { CMD_LU201x_CHCAPTEST, 0, 0 };
	u8 reply[2] = { 0, };
	u8 temp = 0, size = 0;
	u8 capreadvalue[80] = { 0, };
	u16 tab_dlytime = 0;
	u16 ChCapData[40] = { 0, };
	int ret = -1;

	LU201x_set_i2c_mode ();

	if ( LU201x_wait_ready_i2c() == -1 )
	{
		TPD_ERR ( "Set mode Int ready error\n" );
		goto exit;
	}

	LU201x_i2c_write_bytes ( LU201x_i2c_client, LU201x_MODE_REG, buf, 3 );
	LU201x_i2c_done ();

	msleep ( 2 );

	if ( LU201x_wait_ready_i2c() == -1 )
	{
		TPD_ERR ( "Set mode read ready error\n" );
		goto exit;
	}

	LU201x_i2c_read_bytes ( LU201x_i2c_client, LU201x_CMDACK_REG, buf, 3 );
	if ( buf[2] != CMD_LU201x_CHCAPTEST )
	{
		TPD_ERR ( "Read reg failed! %#x\n", buf[0] );
		goto exit;
	}
	ret = buf[2];

	LU201x_i2c_read_bytes ( LU201x_i2c_client, LU201x_CMDReply_REG, reply, 2 );
	if ( reply[0] != 0xC1 )
	{
		TPD_ERR ( "Captest reply error! %#x\n", reply[0] );
		goto exit;
	}
	size = reply[1]-6;

	LU201x_i2c_read_bytes ( LU201x_i2c_client, LU201x_CMDReply_REG+2, capreadvalue, size / 2 );
	LU201x_i2c_done ();

	for ( temp = 0 ; temp < size / 4 ; temp++ )
	{
		ChCapData[temp] = capreadvalue[temp*2] | (capreadvalue[temp*2+1] << 8 );
		TPD_LOG ( "Ch[%d] Cap data = %d\n", temp, ChCapData[temp] );
	}
exit:
	mt_set_gpio_mode ( GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT );
}

static ssize_t show_sd ( struct device *dev, struct device_attribute *attr, char *buf )
{
	int ret = 0;
	int err = 0;
	u8 check = 1;
	u8 fw_ver[2] = { 0, };

	ret += sprintf ( buf+ret, "\n\n" );
	sd_write_log ( buf );
	msleep ( 30 );

	ret = LU201x_read_register ( FW_VERSION_REG, &fw_ver[0], 2 );
	if ( ret < 0 )
	{
		TPD_ERR ( "FW_VERSION_REG read fail\n" );
	}

	msleep ( 5 );

	ret = LU201x_read_register ( FW_STATUS_REG, &check, 1 );
	if ( ret < 0 )
	{
		TPD_ERR ( "FW_STATUS_REG read fail\n" );
	}
	TPD_LOG ( "FW_STATUS_REG value = 0x%x\n", check );

	LU201x_ChCapTest ();

	ret += sprintf ( buf+ret, "FW_Version : %d.%d\n", fw_ver[0], fw_ver[1] );
	ret += sprintf ( buf+ret, "FW_Status : 0x%x\n", check );

	sd_write_log ( buf );
	msleep ( 100 );

	ret += sprintf ( buf+ret, "=======RESULT========\n" );
	ret += sprintf ( buf+ret, "Channel Status : %s\n", ( check & FWSTATUS_CHFAIL ) ? "FAIL" : "PASS" );
	ret += sprintf ( buf+ret, "Raw Data : %s\n", ( check & FWSTATUS_CALFAIL ) ? "FAIL" : "PASS" );

	LU201x_reset ( 0 );
	LU201x_reset ( 1 );

	return ret;
}
static DEVICE_ATTR ( sd, 0664, show_sd, NULL );

static ssize_t show_CapTest ( struct device *dev, struct device_attribute *attr, char *buf )
{
	int ret = 0;

	LU201x_ChCapTest ();

	return ret;
}
static DEVICE_ATTR ( captest, 0664, show_CapTest, NULL );

static ssize_t show_reset ( struct device *dev, struct device_attribute *attr, char *buf )
{
	int ret = 0;

	TPD_LOG ( "Touch Reset!!\n" );
	LU201x_reset ( 0 );
	LU201x_reset ( 1 );

	return ret;
}
static DEVICE_ATTR ( reset, 0664, show_reset, NULL );

static struct attribute *lge_touch_attrs[] = {
	&dev_attr_knock_on_type.attr,
	&dev_attr_lpwg_data.attr,
	&dev_attr_lpwg_notify.attr,
	&dev_attr_sd.attr,
	&dev_attr_captest.attr,
	&dev_attr_reset.attr,
	NULL,
};

static struct attribute_group lge_touch_group = {
	.name = LGE_TOUCH_NAME,
	.attrs = lge_touch_attrs,
};

/****************************************************************************
* Touch Firmware Update Function
****************************************************************************/
static int LU201x_readFW ( unsigned char *pBuf, int addr, int size )
{
	TPD_FUN ();
	unsigned char temp[64] = { 0, };
	int i;
	u8 Cmd[2] = { 0, 0 };

	//Set Read Command
	Cmd[0] = 0x84;
	if ( LU201x_i2c_write_bytes ( LU201x_i2c_client, 0x1000, Cmd, 1 ) == -1 )
	{
		TPD_ERR ( "Read Cmd operation failed\n" );
		return -1;
	}

	for ( i = 0 ; i < size ; i += 64 )
	{
		if ( LU201x_i2c_read_bytes ( LU201x_i2c_client, addr+i, temp, 64 ) == -1 )
		{
			TPD_ERR ( "bin write operation failed %d\n", i );
			return -1;
		}
		memcpy ( &pBuf[i], temp, 64 );
	}

	return 1;
}

static int LU201x_programFW ( unsigned char *pBuf, int addr, int size )
{
	TPD_FUN ();
	u8 Cmd[2] = { 0, 0 };
	u8 Status = 0;
	int i;

	for ( i = 0 ; i < size ; i += 64 )
	{
		Cmd[0] = 0x83;
		if ( LU201x_i2c_write_bytes ( LU201x_i2c_client, 0x1000, Cmd, 1 ) == -1 )
		{
			TPD_ERR ( "Set load bit operation failed\n" );
			return -1;
		}
		if ( LU201x_i2c_write_bytes ( LU201x_i2c_client, addr+i, (pBuf+i), 64 ) == -1 )
		{
			TPD_ERR ( "bin write operation failed %d\n", i );
			return -1;
		}
		Cmd[0] = 0x82;
		if ( LU201x_i2c_write_bytes ( LU201x_i2c_client, 0x1000, Cmd, 1 ) == -1 )
		{
			TPD_ERR ( "Reset load bit operation failed\n" );
			return -1;
		}
		while ( 1 )
		{
			if ( LU201x_i2c_read_bytes ( LU201x_i2c_client, 0x1000, &Status, 1 ) == -1 )
			{
				TPD_ERR ( "Read Status operation failed\n" );
				return -1;
			}

			if ( ( Status & 0x40 ) == 0x40 )
			{
				break;
			}
		}
	}

	return 0;
}

static int LU201x_erase ( void )
{
	TPD_FUN ();
	u8 Cmd[2] = { 0, 0 };
	u8 Status = 0;
	u8 *pBuf;

	pBuf = kmalloc ( 1024, GFP_KERNEL );

	//Read Cal Data
	if ( LU201x_readFW ( pBuf, 0xFC00, 1024 ) == -1 )
	{
		TPD_ERR ( "Read Cal Data failed\n" );
		goto ERASE_FAIL;
	}

	//Erase
	Cmd[0] = 0x80;
	if ( LU201x_i2c_write_bytes ( LU201x_i2c_client, 0x1000, Cmd, 1 ) == -1 )
	{
		TPD_ERR ( "Set mode command operation failed\n" );
		goto ERASE_FAIL;
	}

	Cmd[0] = 0x04;
	if ( LU201x_i2c_write_bytes ( LU201x_i2c_client, 0x1005, Cmd, 1 ) == -1 )
	{
		TPD_ERR ( "Erase command operation failed\n" );
		goto ERASE_FAIL;
	}

	while ( 1 )
	{
		if ( LU201x_i2c_read_bytes ( LU201x_i2c_client, 0x1000, &Status, 1 ) == -1 )
		{
			TPD_ERR ( "Read Status operation failed\n" );
			goto ERASE_FAIL;
		}

		if ( ( Status & 0x40 ) == 0x40 )
		{
			break;
		}
	}
	msleep ( 100 );
	if ( LU201x_programFW ( pBuf, 0xFC00, 1024 ) == -1 )
	{
		TPD_ERR ( "Read Cal Data failed\n" );
		goto ERASE_FAIL;
	}
	//Write Cal Data
	kfree ( pBuf );
	return 0;

ERASE_FAIL:
	kfree ( pBuf );
	return -1;
}

static int LU201x_update_setmode ( int mode )
{
	TPD_FUN ();
	u8 Cmd[2] = { 0, 0 };

	if ( mode ) //FW Upgrade mode
	{
		mt65xx_eint_mask ( CUST_EINT_TOUCH_PANEL_NUM );

		Cmd[0] = 0x80;
		if ( LU201x_i2c_write_bytes ( LU201x_i2c_client, 0x1000, Cmd, 1 ) == -1 )
		{
			TPD_ERR ( "Set mode operation failed\n" );
			return -1;
		}

		Cmd[0] = 0x75;
		Cmd[1] = 0x6C;
		if ( LU201x_i2c_write_bytes ( LU201x_i2c_client, 0x1003, Cmd, 2 ) == -1 )
		{
			TPD_ERR ( "Set password operation failed\n" );
			return -1;
		}
	}
	else
	{
		mt_set_gpio_mode ( GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT );
	    mt_set_gpio_dir ( GPIO_CTP_EINT_PIN, GPIO_DIR_IN );
		mt65xx_eint_unmask ( CUST_EINT_TOUCH_PANEL_NUM );
	}

	TPD_LOG("LU201x_update_setmode is success\n");

	return 0;
}

static int LU201x_do_update ( void )
{
	TPD_FUN ();
	u8 *fw_start;
	int err;

	fw_start = (unsigned char *) &rawData[0];

	mtk_wdt_enable ( WK_WDT_DIS );
	TPD_LOG ( "Watchdog disable\n" );

	//Set mode
	if ( LU201x_update_setmode ( 1 ) == -1 )
	{
		TPD_ERR ( "Set Mode failed\n" );
		goto UPDATE_FAIL;
	}

	//Erase
	if ( LU201x_erase () == -1 )
	{
		TPD_ERR ( "Erase failed\n" );
		goto UPDATE_FAIL;
	}

	//Program
	if ( LU201x_programFW ( fw_start, 0x8000, (31*1024) ) == -1 )
	{
		TPD_ERR ( "Bin program failed\n" );
		goto UPDATE_FAIL;
	}

	LU201x_update_setmode ( 0 );
	LU201x_reset ( 0 );
	LU201x_reset ( 1 );

	mtk_wdt_enable ( WK_WDT_EN );
	TPD_LOG ( "Watchdog enable\n" );

	return 0;

UPDATE_FAIL:
	LU201x_update_setmode ( 0 );
	return -1;
}

static int LU201x_firmware_check ( void )
{
	TPD_FUN ();
	int err = 0;
	u8 fw_ver[4] = { 0, };
	u8 info[8] = { 0, };
	u8 Cmd[2] = { 0, };

	msleep ( 5 );

	/* read Product ID */
	//User mode enter
	Cmd[0] = 0x75;
	Cmd[1] = 0x6C;
	if ( LU201x_i2c_write_bytes ( LU201x_i2c_client, 0x1003, Cmd, 2 ) == -1 )
	{
		TPD_ERR ( "Set password operation failed\n" );
		return -1;
	}
	Cmd[0] = 0x88;
	if ( LU201x_i2c_write_bytes ( LU201x_i2c_client, 0x1000, Cmd, 1 ) == -1 )
	{
		TPD_ERR ( "User mode go operation failed\n" );
		return -1;
	}
	//Get information
	Cmd[0] = 0x8C;
	if ( LU201x_i2c_write_bytes ( LU201x_i2c_client, 0x1000, Cmd, 1 ) == -1 )
	{
		TPD_ERR ( "User Area-Read Cmd operation failed\n" );
		return -1;
	}
	if ( LU201x_i2c_read_bytes ( LU201x_i2c_client, 0x0038, info, 8 ) == -1 )
	{
		TPD_ERR ( "User Area-Read  failed\n" );
		return -1;
	}
	//User mode exit
	Cmd[0] = 0x00;
	Cmd[1] = 0x00;
	if ( LU201x_i2c_write_bytes ( LU201x_i2c_client, 0x1003, Cmd, 2 ) == -1 )
	{
		TPD_ERR ( "User mode exit failed\n" );
		return -1;
	}
	TPD_LOG ( "Touch Product Infomation [Vendor: %x, Version: %x]\n", info[2], info[3] );

	LU201x_reset ( 0 );
	LU201x_reset ( 1 );

	/* check Firmware version */
	err = LU201x_read_register ( FW_VERSION_REG-2, &fw_ver[0], 4 );
	if ( err < 0 )
	{
		TPD_ERR ( "FW_VERSION_REG read fail\n" );
	}
	else
	{
		TPD_LOG ( "Touch IC: FW_Version [%d.%d], Release Version [%d.%d]\n", fw_ver[0], fw_ver[1], fw_ver[2], fw_ver[3] );
		TPD_LOG ( "Binary: FW_Version [%d.%d], Release Version [%d.%d]\n", rawData[0x79FC], rawData[0x79FD], rawData[0x79FE], rawData[0x79FF] );

		if ( fw_ver[0] != rawData[0x79FC] || fw_ver[1] != rawData[0x79FD] || fw_ver[2] != rawData[0x79FE] || fw_ver[3] < rawData[0x79FF] )
		{
			TPD_LOG ( "Touch FW Update!! [%d.%d ==> %d.%d]\n", fw_ver[2], fw_ver[3], rawData[0x79FE], rawData[0x79FF] );
		}
		else
		{
			TPD_LOG ( "don't have to update FW\n" );
			return 0;
		}
	}

	err = LU201x_do_update ();
	if ( err < 0 )
	{
		TPD_ERR ( "update tsp fail\n" );
		return -1;
	}
	else
	{
		TPD_LOG ( "update tsp success\n" );
	}

	return 0;
}

static struct file* LU201x_file_open ( char *path, mm_segment_t *old_fs_p )
{
	struct file *filp = NULL;
	int errno = -1;

	*old_fs_p = get_fs();
	set_fs(KERNEL_DS);

	filp = filp_open(path, O_RDWR, S_IRUSR|S_IWUSR);

	if ( !filp || IS_ERR(filp) )
	{
		TPD_ERR ( "file open error!\n" );
		if( !filp )
		{
			errno = -ENOENT;
		}
		else
		{
			errno = PTR_ERR(filp);
		}

		return NULL;
	}

	filp->f_op->llseek(filp,0,0);
	return filp;
}

static void LU201x_file_close ( struct file *filp, mm_segment_t old_fs )
{
	set_fs(old_fs);
	if ( filp )
	{
		filp_close(filp, NULL);
	}
}

static int LU201x_do_update_bin ( char *path )
{
	TPD_FUN ();
	unsigned char *buf;
	struct file *file_ck = NULL;
	mm_segment_t old_fs;
	int ret, length;

	if ( path == NULL )
	{
		TPD_ERR ( "LU201x_do_update_bin path is NULL\n" );
		return -1;
	}

	file_ck = LU201x_file_open(path, &old_fs);
	TPD_LOG ( "File Path : %s\n", path );

	if ( file_ck == NULL )
	{
		TPD_ERR ( "File Open failed\n" );
		return -1;
	}

	length = file_ck->f_op->llseek(file_ck, 0, SEEK_END);
	if ( length != (31*1024) )
	{
		TPD_ERR	( "File Length failed\n" );
		LU201x_file_close ( file_ck, old_fs );
		return -1;
	}

	//set file pointer to the begining of the file
	file_ck->f_op->llseek(file_ck, 0, SEEK_SET);

	//memory Allocation
	buf = kmalloc(length, GFP_KERNEL);
	ret = file_ck->f_op->read(file_ck, buf, length, &file_ck->f_pos);

	LU201x_file_close(file_ck, old_fs);

	if ( ret != length )
	{
		TPD_ERR ( "File Length of contents mismatch %d\n", ret );
		goto UPDATE_FAIL;
	}
	if ( buf[0] != 0x80 )
	{
		TPD_ERR ( "Upgrade Already done %d\n", ret );
		goto UPDATE_FAIL;
	}

	//Set mode
	if ( LU201x_update_setmode ( 1 ) == -1 )
	{
		TPD_ERR ( "Set Mode failed\n" );
		goto UPDATE_FAIL;
	}

	//Erase
	if ( LU201x_erase () == -1 )
	{
		TPD_ERR ( "Erase failed\n" );
		goto UPDATE_FAIL;
	}

	//Program
	if ( LU201x_programFW ( buf, 0x8000, (31*1024) ) == -1 )
	{
		TPD_ERR ( "Bin program failed\n" );
		goto UPDATE_FAIL;
	}

	//If Update succeed, remove file header
	file_ck = LU201x_file_open ( path, &old_fs );
	if ( file_ck == NULL )
	{
		TPD_ERR ( "File Open failed \n" );
		return -1;
	}

	//set file pointer to the begining of the file
	file_ck->f_op->llseek(file_ck, 0, SEEK_SET);
	LU201x_file_close(file_ck, old_fs);

	kfree ( buf );
	LU201x_update_setmode ( 0 );

	return 0;
UPDATE_FAIL:
	kfree ( buf );
	LU201x_update_setmode ( 0 );
	return -1;
}

void LU201x_init_sysfs ( void )
{
	TPD_FUN ();
	int err;

	touch_class = class_create ( THIS_MODULE, "touch" );

	touch_fw_dev = device_create ( touch_class, NULL, 0, NULL, "firmware" );
	err = device_create_file ( touch_fw_dev, &dev_attr_firmware );
	if ( err )
	{
		TPD_ERR ( "Touchscreen : [firmware] touch device_create_file Fail\n" );
		device_remove_file ( touch_fw_dev, &dev_attr_firmware );
	}

	err = device_create_file ( touch_fw_dev, &dev_attr_version );
	if ( err )
	{
		TPD_ERR ( "Touchscreen : [version] touch device_create_file Fail\n" );
		device_remove_file ( touch_fw_dev, &dev_attr_version );
	}

	sysfs_create_group ( tpd->dev->dev.kobj.parent, &lge_touch_group );
}
EXPORT_SYMBOL ( LU201x_init_sysfs );

#define to_foo_obj(x) container_of(x, struct foo_obj, kobj)
struct foo_attribute {
	struct attribute attr;
	ssize_t (*show)(struct foo_obj *foo, struct foo_attribute *attr, char *buf);
	ssize_t (*store)(struct foo_obj *foo, struct foo_attribute *attr, const char *buf, size_t count);
};
#define to_foo_attr(x) container_of(x, struct foo_attribute, attr)
static ssize_t foo_attr_show(struct kobject *kobj,
			     struct attribute *attr,
			     char *buf){
		struct foo_attribute *attribute;
	struct foo_obj *foo;

	attribute = to_foo_attr(attr);
	foo = to_foo_obj(kobj);

	if (!attribute->show)
		return -EIO;

	return attribute->show(foo, attribute, buf);
}
static ssize_t foo_attr_store(struct kobject *kobj,
			      struct attribute *attr,
			      const char *buf, size_t len)
{
	struct foo_attribute *attribute;
	struct foo_obj *foo;

	attribute = to_foo_attr(attr);
	foo = to_foo_obj(kobj);

	if (!attribute->store)
		return -EIO;

	return attribute->store(foo, attribute, buf, len);
}
static const struct sysfs_ops foo_sysfs_ops = {
	.show = foo_attr_show,
	.store = foo_attr_store,
};
static void foo_release(struct kobject *kobj)
{
	struct foo_obj *foo;

	foo = to_foo_obj(kobj);
	kfree(foo);
}

static ssize_t foo_show(struct foo_obj *foo_obj, struct foo_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", foo_obj->interrupt);
}
static ssize_t foo_store(struct foo_obj *foo_obj, struct foo_attribute *attr,
			 const char *buf, size_t count)
{
	sscanf(buf, "%du", &foo_obj->interrupt);
	return count;
}
static struct foo_attribute foo_attribute =__ATTR(interrupt, 0664, foo_show, foo_store);
static struct attribute *foo_default_attrs[] = {
	&foo_attribute.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};
static struct kobj_type foo_ktype = {
	.sysfs_ops = &foo_sysfs_ops,
	.release = foo_release,
	.default_attrs = foo_default_attrs,
};
static struct kset *example_kset;

static struct foo_obj *create_foo_obj(const char *name){
	struct foo_obj *foo;
	int retval;
	foo = kzalloc(sizeof(*foo), GFP_KERNEL);
	if (!foo)
		return NULL;
	foo->kobj.kset = example_kset;
	retval = kobject_init_and_add(&foo->kobj, &foo_ktype, NULL, "%s", name);
	if (retval) {
		kobject_put(&foo->kobj);
		return NULL;
	}
	kobject_uevent(&foo->kobj, KOBJ_ADD);
	return foo;
}


/****************************************************************************
* I2C BUS Related Functions
****************************************************************************/
static int LU201x_i2c_probe ( struct i2c_client *client, const struct i2c_device_id *id )
{
	TPD_FUN ();
	int i, err = 0;
	u8 DeviceID[2] = { 0, };
	struct task_struct *thread = NULL;
	struct foo_obj *foo;

	knock_on_type = 1;

	LU201x_i2c_client = client;

	/* Turn on the power for TOUCH */
	LU201x_power ( 0 );
	LU201x_power ( 1 );

	err = LU201x_i2c_read_bytes ( LU201x_i2c_client, LU201x_DEVICEID_ADDR, DeviceID, 2 );
	if ( err < 0 )
	{
		TPD_LOG ( "Touch IC is not LeadingUI\n" );
		return -1;
	}
	else if ( ( DeviceID[1] != 0x20 ) || ( DeviceID[0] != 0x10 ) )
	{
		TPD_LOG ( "Device ID is fail (%x%x)\n", DeviceID[1], DeviceID[0] );
		return -1;
	}

	SEL_SYSFS = 2;

	/* Touch Firmware Update */
	for ( i = 0 ; i < 3 ; i++ )
	{
		err = LU201x_firmware_check ();
		if ( err == 0 || i == 2 )
		{
			break;
		}
		else
		{
			LU201x_reset ( 0 );
			LU201x_reset ( 1 );
			TPD_ERR ( "Firmware check Retry %d\n", i+1 );
		}
	}

	example_kset = kset_create_and_add("lge", NULL, kernel_kobj);
	foo_obj = create_foo_obj(LGE_TOUCH_NAME);

	wake_lock_init ( &knock_code_lock, WAKE_LOCK_SUSPEND, "knock_code" );

#ifdef TPD_HAVE_BUTTON
	for ( i = 0 ; i < TPD_KEY_COUNT ; i++ )
	{
		input_set_capability ( tpd->dev, EV_KEY, tpd_keys_local[i] );
	}
#endif

	thread = kthread_run ( LU201x_touch_event_handler, 0, TPD_DEVICE );	
	if ( IS_ERR ( thread ) )
	{
		TPD_ERR ( "failed to create kernel thread\n" );
	}

	/* Configure external ( GPIO ) interrupt */
	LU201x_setup_eint ();

	TPD_LOG ( "Probe is done\n" );
	tpd_load_status = 1;

	LU201x_reset ( 0 );
	LU201x_reset ( 1 );

	return 0;
}

static int __devexit LU201x_i2c_remove ( struct i2c_client *client )
{
	TPD_FUN ();
	return 0;
}

static int LU201x_i2c_detect ( struct i2c_client *client, struct i2c_board_info *info )
{
	TPD_FUN ();
	strcpy ( info->type, "mtk-tpd" );
	return 0;
}

static const struct i2c_device_id tpd_i2c_id[] = { { TPD_DEV_NAME, 0 }, {} };

static struct i2c_driver tpd_i2c_driver = {
	.driver.name = "mtk-tpd",
	.probe = LU201x_i2c_probe,
	.remove = LU201x_i2c_remove,
	.detect = LU201x_i2c_detect,
	.id_table = tpd_i2c_id,
};

/****************************************************************************
* Linux Device Driver Related Functions
****************************************************************************/
static int LU201x_local_init ( void )
{
	TPD_FUN ();

	if ( i2c_add_driver ( &tpd_i2c_driver ) != 0 )
	{
		TPD_ERR ( "i2c_add_driver failed\n" );
		return -1;
	}

	if ( tpd_load_status == 0 )
	{
		TPD_ERR ( "touch driver probing failed\n" );
		i2c_del_driver ( &tpd_i2c_driver );
		return -1;
	}

	tpd_type_cap = 1;

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void LU201x_suspend ( struct early_suspend *h )
{
	TPD_FUN ();
	int ret;
	u8 retry = 0;

	LU201x_release_all_finger ();

	if ( !suspend_status )
	{
		if ( knock_on_enable )
		{
			switch ( lpwg_mode )
			{
				case LPWG_DOUBLE_TAP:
					do
					{
						ret = LU201x_set_mode ( CMD_LU201x_IDLE_DOUBLETAB );
						if ( ret == -1 )
						{
							LU201x_reset ( 0 );
							LU201x_reset ( 1 );
							retry++;
							TPD_ERR ( "set_mode retry %d\n", retry );
						}
					} while ( ret == -1 && retry < 3 );
					double_tap_enable = 1;
					break;

				case LPWG_MULTI_TAP:
					do
					{
						ret = LU201x_set_mode ( CMD_LU201x_IDLE_MULTITAB );
						if ( ret == -1 )
						{
							LU201x_reset ( 0 );
							LU201x_reset ( 1 );
							retry++;
							TPD_ERR ( "set_mode retry %d\n", retry );
						}
					} while ( ret == -1 && retry < 3 );
					multi_tap_enable = 1;
					break;

				default:
					break;
			}

			TPD_LOG ( "knock_suspend lpwg_mode : %d, %d, %d\n", lpwg_mode, double_tap_enable, multi_tap_enable );
		}
		else
		{
			mt65xx_eint_mask ( CUST_EINT_TOUCH_PANEL_NUM );
			ret = LU201x_set_mode ( CMD_LU201x_PDN );
		}
	}

	suspend_status = 1;
}

static void LU201x_resume ( struct early_suspend *h )
{
	TPD_FUN ();

	LU201x_release_all_finger ();

	if ( key_lock_status == 0 )
	{
		mt65xx_eint_unmask ( CUST_EINT_TOUCH_PANEL_NUM );
	}
	else
	{
		mt65xx_eint_mask ( CUST_EINT_TOUCH_PANEL_NUM );
	}

	LU201x_reset ( 0 );
	LU201x_reset ( 1 );

	suspend_status = 0;
	double_tap_enable = 0;
	multi_tap_enable = 0;
}
#endif

static struct i2c_board_info __initdata i2c_tpd = {	I2C_BOARD_INFO ( TPD_DEV_NAME, TPD_I2C_ADDRESS ) };

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = TPD_DEV_NAME,
	.tpd_local_init = LU201x_local_init,
#ifdef CONFIG_HAS_EARLYSUSPEND
	.suspend = LU201x_suspend,
	.resume = LU201x_resume,
#endif
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif
};

static int __init LU201x_driver_init ( void )
{
	TPD_FUN ();

	i2c_register_board_info ( 1, &i2c_tpd, 1 );
	if ( tpd_driver_add ( &tpd_device_driver ) < 0 )
	{
		TPD_ERR ( "tpd_driver_add failed\n" );
	}

	return 0;
}

static void __exit LU201x_driver_exit ( void )
{
	TPD_FUN ();

	tpd_driver_remove ( &tpd_device_driver );
}


module_init ( LU201x_driver_init );
module_exit ( LU201x_driver_exit );

MODULE_DESCRIPTION ( "LU201x Touchscreen Driver for MTK platform" );
MODULE_AUTHOR ( "Junmo Kang <junmo.kang@lge.com>" );
MODULE_LICENSE ( "GPL" );
