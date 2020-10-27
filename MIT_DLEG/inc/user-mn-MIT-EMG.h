/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-manage' Mid-level computing, and networking
	Copyright (C) 2017 Dephy, Inc. <http://dephy.com/>
*****************************************************************************
	[Lead developper] Jean-Francois (JF) Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] emg: external emg Processor
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2018-03-06 | syeon | Initial release
	*
****************************************************************************/

// how to use this library

// include in i2c fsm
// include in i2c2 RxCplt callback

#include "user-mn.h"
#ifdef USE_MIT_EMG_I2C

#ifndef INC_MIT_EMG_H
#define INC_MIT_EMG_H

//****************************************************************************
// Definitions
//****************************************************************************

//****************************************************************************
// Include(s)
//****************************************************************************
#include "main.h"
#include "i2c.h"
#include <i2c.h>
#include "flexsea_global_structs.h"
#include "flexsea_user_structs.h"
#include "flexsea.h"
#include <stdlib.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "rigid.h"

//****************************************************************************
// Shared variable(s)
//****************************************************************************

//variables contain the data
extern int16_t emgData[16]; //16 ch emg data
extern int16_t emgMisc[3]; // array of int16_t reserved for various usage
extern volatile uint8_t emgOnFlag; //this flag decide whether use EMG or not
extern uint16_t emgTimestamp;
extern volatile uint16_t emgErrorCnt;
//****************************************************************************
// Prototype(s):
//****************************************************************************
void mitEmgUpdateStatus(void);
void mitEmgDecode(void);

void mitEmgDecode16ch(void);
void mitEmgRead(void);
void mitEmgI2CErrorCallback(I2C_HandleTypeDef *hi2c);
void mitEmgI2CRxCpltCallback(I2C_HandleTypeDef *hi2c); //attach this function on i2c.c
void mitEmgI2c2Fsm(void);

uint8_t mitEmgGetState(void); //read value when only 1 is returned
void mitEmgChangeState(uint8_t); //activate / deactive the EMG peripheral


void scaleEMGMultipacket(void);

//****************************************************************************
// Definition(s):
//****************************************************************************

#define I2C_SLAVE_ADDR_EMG		16	//8 bits (7 = 0x66)
#define I2C_EMG_BUFSIZE 40

#define EMG_PERIPH_READY 0
#define EMG_PERIPH_RECEIVE_WAIT 1
#define EMG_PERIPH_RECEIVE_COMPLETE 2

#define EMG_LINE_THRESHOLD 3475 // 2.8V/3.3V *4096
#define EMG_TIMER_THRESHOLD 250 //250ms
#define EMG_TIMER_PRESCALER 2 // comm freq = 1kHz/presc

#ifdef PROJECT_MIT_DLEG
//#define EMG_LINE_READY 1 //AIN1-CON
#endif

#ifdef PROJECT_POCKET_2XDC //pocket AIN-0/1 Swapped
//#define EMG_LINE_READY 1 //AIN0 - CON
#endif

#define EMG_COMM_HEADER 0xAB;
#define EMG_COMM_TAIL 0x45;

#define EMG_DATA_RAW 0
#define EMG_DATA_BPF 1
#define EMG_DATA_EVF 2
#define EMG_DATA_MAC 3
#define EMG_DATA_MISC 4

#define EMG_COMM_COMMAND_OFF 0
#define EMG_COMM_COMMAND_STREAM 1
#define EMG_COMM_COMMAND_WRITE 3

#define EMG_STATE_DISABLE 0
#define EMG_STATE_INACTIVE 1
#define EMG_STATE_WAIT 2
#define EMG_STATE_READ 4

#define EMG_STATE_DEINIT 10
#define EMG_STATE_RECOVER 11

#define EMG_DEINIT_PERIOD 4 //4ms
#define EMG_I2C_RESET_PERIOD 1000 //1s
//****************************************************************************
// Structure(s):
//****************************************************************************

#endif	//INC_MIT_EMG_H
#endif 	//USE_MIT_EMG_I2C
