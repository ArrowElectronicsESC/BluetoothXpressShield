/************************************************************************
File: ArrowBGXShield.h
Version: V1.0
Release Date: February 4, 2019
Authors: Eric Beppler, Andrew Reisdorph, Arrow Electronics

Description: The defines in this file are provided for the 
ArrowBGXShield library to ease development

Contact ESC@Arrow.com for all inquiries with the subject SLASH1000


This file is distributed by ARrow Electronics under the MIT License. 
See License.txt for more details.
************************************************************************/


#ifndef ArrowBGXDefines_h
#define ArrowBGXDefines_h

#include "Arduino.h" 

#define SINGLE_READ 1

#define SEND_STOP true

#define OFF_BRIGHTNESS				0x00
#define HALF_BRIGHTNESS				0x88
#define QUARTER_BRIGHTNESS  		0x44
#define THREE_QUARTER_BRIGHTNESS 	0xCC
#define MAX_BRIGHTNESS				0xFF


#define MODE1				(0x00) //mode register 1
#define MODE2				(0x01) //mode register 2
#define PWM0				(0x02) //Brightness control led0 - White
#define PWM1				(0x03) //Brightness control led1 - Blue
#define PWM2				(0x04) //Brightness control led2 - Red
#define PWM3				(0x05) //Brightness control led3 - Green
#define GRPPWM				(0x06) //group duty cycle control
#define GRPFREQ				(0x07) //group frequency
#define LEDOUT				(0x08) //LED output state
#define SUBADR1				(0x09) //i2c bus subaddress 1
#define SUBADR2				(0x0A) //i2c bus subaddress 2
#define SUBADR3				(0x0B) //i2c bus subaddress 3
#define ALLCALLADR			(0x0C) //LED All Call i2c address

//Mode 1 Reg Definitions
#define NO_AUTO_INC 		(B000 << 5)
#define INC_ALL_REG 		(B100 << 5)
#define INC_INDV_REG 		(B101 << 5)
#define INC_CTRL_REG 		(B110 << 5)
#define INC_INDV_AND_CTRL 	(B111 << 5)
#define SLEEP_ENABLE		(B1 << 4)
#define SLEEP_DISABLE		(B0 << 4)
#define ALT_ADDR_DISABLE	(B0000 << 0)


//Mode 2 Reg Definitions
#define GROUP_DIMMING		(B0 << 5)
#define GROUP_BLINKING		(B1 << 5)
#define INVRT 				(B1 << 4)
#define NON_INVRT			(B0 << 4)
#define CHANGE_ON_STOP		(B0 << 3)
#define CHANGE_ON_ACK		(B1 << 3)
#define TOTEM_CONFIG		(B1 << 2)
#define OPEN_DRAIN_CONFIG	(B0 << 2)
#define OUTNE				(B01 << 0)

//LEDOUT reg definitions
#define LED_OFF(x) 		(B00 << x)
#define LED_ON(x)		(B01 << x)
#define LED_INDV(x)		(B10 << x)
#define LED_INDV_GRP(x)	(B11 << x)
#define LED0 			0 
#define LED1 			2 
#define LED2 			4 
#define LED3 			6 



#define START_TEMP		0xF3

#define CONF_REF		0x00
#define ALS_REG 		0x04
#define WHITE_REG		0x05

#endif