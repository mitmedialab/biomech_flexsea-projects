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
  * 2018-03-23 | syeon | Add muscle activation calculation and calibration
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

#define MIT_EMG_MUSCLE_CH_LG 6
#define MIT_EMG_MUSCLE_CH_TA 3
#define MIT_EMG_MUSCLE_CH_PL 5
#define MIT_EMG_MUSCLE_CH_TP 0
#define MIT_EMG_MUSCLE_ALL 8

#define MIT_EMG_MUSCLE_MIN 0
#define MIT_EMG_MUSCLE_MAX 1
#define MIT_EMG_ACTIVATION_MAX 1 //allow 200% of muscle activation

#define MIT_EMG_BASELINE_THRESHOLD 5 //add 5uV in calibration routine
// Jim, 0315, 4pm test
#define USER_MIN_CH1 6
#define USER_MIN_CH2 100
#define USER_MIN_CH3 100
#define USER_MIN_CH4 6
#define USER_MIN_CH5 100
#define USER_MIN_CH6 6
#define USER_MIN_CH7 6
#define USER_MIN_CH8 100

#define USER_MAX_CH1 28
#define USER_MAX_CH2 1000
#define USER_MAX_CH3 1000
#define USER_MAX_CH4 156
#define USER_MAX_CH5 1000
#define USER_MAX_CH6 103
#define USER_MAX_CH7 63
#define USER_MAX_CH8 1000

//LG TA TL TP
//****************************************************************************
// Include(s)
//****************************************************************************
#include "main.h"
#include "flexsea_global_structs.h"
#include <stdlib.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "rigid.h"

//****************************************************************************
// Shared variable(s)
//****************************************************************************

//variables contain the data
extern int16_t emg_data[8]; //8 ch emg data
extern int16_t emg_misc[3]; // array of int16_t reserved for various usage
extern volatile uint8_t emg_on_flag; //this flag decide whether use EMG or not
extern uint16_t emg_timestamp;
extern volatile uint16_t emg_error_cnt;
extern int16_t emg_processed_data[8];
extern float emg_threshold[8][2];
//****************************************************************************
// Prototype(s):
//****************************************************************************
void MIT_EMG_update_status(void);
void MIT_EMG_decode(void);
void MIT_EMG_read(void);
void MIT_EMG_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
void MIT_EMG_I2C_RxCpltCallback(I2C_HandleTypeDef *hi2c); //attach this function on i2c.c
void MIT_EMG_i2c2_fsm(void);

uint8_t MIT_EMG_getState(void); //read value when only 1 is returned
void MIT_EMG_changeState(uint8_t); //activate / deactive the EMG peripheral


void MIT_EMG_start_calibration_min(void);
void MIT_EMG_start_calibration_max(uint8_t muscle_ch);
void MIT_EMG_stop_calibration(void);
void MIT_EMG_calibration_fsm(void);
void MIT_EMG_calibrate_activation_max(void);
void MIT_EMG_calibrate_activation_min(void);
void MIT_EMG_calculate_activation(void);

//****************************************************************************
// Definition(s):
//****************************************************************************

#define I2C_SLAVE_ADDR_EMG		16	//8 bits (7 = 0x66)

#define EMG_PERIPH_READY 0
#define EMG_PERIPH_RECEIVE_WAIT 1
#define EMG_PERIPH_RECEIVE_COMPLETE 2

#define EMG_LINE_THRESHOLD 3475 // 2.8V/3.3V *4096
#define EMG_TIMER_THRESHOLD 250 //250mV
#define EMG_TIMER_PRESCALER 1 // comm freq = 1kHz/presc

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
#define EMG_STATE_CALIBRATION 5

#define EMG_STATE_DEINIT 10
#define EMG_STATE_RECOVER 11

#define EMG_CALSTATE_OFF 0
#define EMG_CALSTATE_MIN 1
#define EMG_CALSTATE_MAX 2

#define EMG_DEINIT_PERIOD 4 //4ms
#define EMG_I2C_RESET_PERIOD 1000 //1s
//****************************************************************************
// Structure(s):
//****************************************************************************

#endif	//INC_MIT_EMG_H
#endif 	//USE_MIT_EMG_I2C
