/*
 * Charging IC driver (BQ25040)
 *
 * Copyright (C) 2010 LGE, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/proc_fs.h>      // might need to get fuel gauge info
#include <linux/power_supply.h>     // might need to get fuel gauge info
//#include "../staging/android//timed_output.h"
#include <mach/mt_gpio.h>
#include <linux/charger_bq25040.h>

static DEFINE_SPINLOCK(bq25040_spin);

void BQ25040_WriteCommand( BQ25040_ChargingMode mode )
{    
	int pulseCount = 0;    
	int i= 0;
	 
	switch ( mode )
	{
		case BQ25040_CM_OFF:
			printk("[LGE] CHARGING_MODE = BQ25040_CM_OFF\n");
			break;
		case BQ25040_CM_USB_100:
			printk("[LGE] CHARGING_MODE = BQ25040_CM_USB_100\n");
			pulseCount = 2;
			break;
		case BQ25040_CM_USB_500:
			printk("[LGE] CHARGING_MODE = BQ25040_CM_USB_500\n");
			pulseCount = 0;
			break;
		case BQ25040_CM_I_SET:
			printk("[LGE] CHARGING_MODE = BQ25040_CM_I_SET\n");
			pulseCount = 1;
			break;
		case BQ25040_CM_FACTORY:
			printk("[LGE] CHARGING_MODE = BQ25040_CM_FACTORY\n");
			pulseCount = 3;
			break;
		default:
			printk("[LGE] Invalid Charging Mode ( %d )\n", mode);
			break;
	}
	if( mode != BQ25040_CM_OFF && mode != BQ25040_CM_UNKNOWN )
	{
		printk("[LGE] Pulse Count ( %d )\n", pulseCount);
		spin_lock(&bq25040_spin);
		mt_set_gpio_out(CHG_EN_SET_N, GPIO_OUT_ZERO);
		mdelay(35);
		for( i=0; i<pulseCount ; i++)
		{
			mt_set_gpio_out(CHG_EN_SET_N, GPIO_OUT_ONE);
			udelay(150);
			mt_set_gpio_out(CHG_EN_SET_N, GPIO_OUT_ZERO);
			if( i < ( pulseCount - 1 ) )
			{
				udelay(150);
			}
		}
		udelay(1800);
		spin_unlock(&bq25040_spin);
	}
	else if( mode == BQ25040_CM_OFF )
	{
		spin_lock(&bq25040_spin);
		mt_set_gpio_out(CHG_EN_SET_N, GPIO_OUT_ONE);
		udelay(1800);
		spin_unlock(&bq25040_spin);
	}
	else
	{
		printk("[LGE] Invalid Charging Mode ( %d )\n", mode);
	}
}
void BQ25040_SetChargingMode( BQ25040_ChargingMode newMode )
{
	static BQ25040_ChargingMode prevMode = BQ25040_CM_UNKNOWN;
	if( prevMode != newMode )
	{
		if( newMode == BQ25040_CM_OFF )
		{
			BQ25040_WriteCommand(newMode);
		}
		else
		{
			if( prevMode != BQ25040_CM_OFF )
			{
				BQ25040_WriteCommand(BQ25040_CM_OFF);
			}
			BQ25040_WriteCommand(newMode);
		}
		prevMode = newMode;
	}
}
EXPORT_SYMBOL(BQ25040_SetChargingMode);
