/* include/linux/ami304.h - HSCDTD008 compass driver
 *
 * Copyright (C) 2009 AMIT Technology Inc.
 * Author: Kyle Chen <sw-support@amit-inc.com>
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

/*
 * Definitions for ami304 compass chip.
 */
#ifndef HSCDTD008_H
#define HSCDTD008_H

#include <linux/ioctl.h>
//#include <asm-arm/arch/regs-gpio.h>

#define HSCDTD008_I2C_ADDRESS 			0x0C  

/* HSCDTD008 Internal Register Address  (Please refer to HSCDTD008 Specifications) */

#define HSCDTD008_REG_STB         0x0C
#define HSCDTD008_REG_XOUT        0x10
#define HSCDTD008_REG_YOUT        0x12
#define HSCDTD008_REG_ZOUT        0x14
#define HSCDTD008_REG_DATAXH	  0x11
#define HSCDTD008_REG_DATAXL	  0x10
#define HSCDTD008_REG_DATAYH	  0x13
#define HSCDTD008_REG_DATAYL	  0x12
#define HSCDTD008_REG_DATAZH	  0x15
#define HSCDTD008_REG_DATAZL	  0x14

#define HSCDTD008_REG_STATUS      0x18

#define HSCDTD008_REG_CTRL1		  0x1B
#define HSCDTD008_REG_CTRL2		  0x1C
#define HSCDTD008_REG_CTRL3		  0x1D
#define HSCDTD008_REG_CTRL4		  0x1E

/* HSCDTD008 Control Bit  (Please refer to HSCDTD008 Specifications) */
#define HSCDTD008_CTRL1_PC1			0x80   //power mode  1 active mode
#define HSCDTD008_CTRL1_FS1_NORMAL			0x08 //Normal 10Hz  000 01 000
#define HSCDTD008_CTRL1_FS1_FORCE			0x02 //Force
#define HSCDTD008_CTRL1_FS1_NORMAL100		0x18 //Normal 100Hz  000 11 000
#define HSCDTD008_CTRL1_FS1_NORMAL20		0x10 //Normal 100Hz  000 10 000
#define HSCDTD008_CTRL2_DREN			0x08
#define HSCDTD008_CTRL2_DRP			0x04
#define HSCDTD008_CTRL3_NOFORCE_BIT		0x00
#define HSCDTD008_CTRL3_FORCE_BIT			0x40
#define HSCDTD008_CTRL3_B0_LO_CLR			0x00
#define HSCDTD008_CTRL3_SRST			0x80  //SW reset
#define HSCDTD008_CTRL4_RS				0x10  //15bit mode

#define HSCDTD008_BUFSIZE				256
#define HSCDTD008_NORMAL_MODE			0
#define HSCDTD008_FORCE_MODE			1
#define HSCDTD008_IRQ				IRQ_EINT9

// conversion of magnetic data to nT units
#define CONVERT_M                       1//25
#define ORIENTATION_ACCURACY_RATE                   100


enum {
    ACTIVE_SS_NUL = 0x00 ,
    ACTIVE_SS_ACC = 0x01 ,
    ACTIVE_SS_MAG = 0x02 ,
    ACTIVE_SS_ORI = 0x04 ,
};

struct TAIFD_HW_DATA {
    int activate;
    int delay;
    int acc[4];
    int mag[4];
};

struct TAIFD_SW_DATA {
    int acc[5];
    int mag[5];
    int ori[5];
};



#endif
