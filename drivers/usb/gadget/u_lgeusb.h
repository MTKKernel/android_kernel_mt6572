/* linux/drivers/usb/gadget/u_lgeusb.h
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (C) 2011 LGE.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __U_LGEUSB_H__
#define __U_LGEUSB_H__

#define FACTORY_PID		0x6000
#define LGE_FACTORY_CABLE_TYPE 1
#define MAX_IMEI_LEN 19
#define LGE_PIF_CABLE 2
#define LGE_130K_CABLE 3

#if 1 //                                                                         
typedef enum LGBmCableIdTag
{
	USB_CABLE_ID_NONE  = 0,
	USB_CABLE_ID_OPEN,
	USB_CABLE_ID_56K,
	USB_CABLE_ID_130K,
	USB_CABLE_ID_180K,
	USB_CABLE_ID_910K,
	USB_CABLE_ID_UNKNOWN,

	USB_CABLE_ID_MAX
}
LGBmCableId;
#else
typedef enum {
  DEVICE_NONE,   // 0
  DEVICE_OPEN_CABLE,   // 1
  DEVICE_FACTORY_UART_CABLE,   // 130K
  DEVICE_FACTORY_USB_CABLE,  // 56K
  DEVICE_FACTORY_DOWNLOAD_CABLE,  // 910K
  DEVIDE_MAX
} USB_ID_TYPE;

extern USB_ID_TYPE readUSB_ID_Value(); 
#endif //                                              
int android_set_factory_mode(void);
bool android_get_factory_mode(void);
void android_factory_desc(int enable, char usb_desc);

#endif /*                */
