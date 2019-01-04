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

#ifndef INC_MIT_FIR_H
#define INC_MIT_FIR_H

//****************************************************************************
// Definitions
//****************************************************************************

//****************************************************************************
// Include(s)
//****************************************************************************
#include "main.h"
#include "flexsea_global_structs.h"
#include <stdlib.h>
#include "stm32f4xx.h"
#include "rigid.h"

//****************************************************************************
// Shared variable(s)
//****************************************************************************

//****************************************************************************
// Prototype(s):
//****************************************************************************
void  init_MIT_FIR(void);
float MIT_FIR_filter1kHz(float input);
void  MIT_FIR_latchInput(float n);

//****************************************************************************
// Definition(s):
//****************************************************************************
//#define LPF1 // Passband 100Hz, Stopband 200Hz
//#define LPF2 // Passband 50Hz, Stopband 100Hz
#define LPF3 // Passband 50Hz, Stopband 70Hz
//#define LPF4 // Passband 35Hz, Stopband 70Hz


//****************************************************************************
// Structure(s):
//****************************************************************************

#endif	//INC_MIT_FIR_H
