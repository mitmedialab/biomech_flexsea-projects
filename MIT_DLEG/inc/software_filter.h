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

//****************************************************************************
// Include(s)
//****************************************************************************
#include "main.h"
#include "flexsea_global_structs.h"
#include "flexsea_user_structs.h"
#include "flexsea.h"
#include <stdlib.h>
#include "stm32f4xx.h"
#include "rigid.h"
#include "user-mn.h"
//****************************************************************************
// Definitions
//****************************************************************************
#ifndef INC_MIT_FIR_H
#define INC_MIT_FIR_H

//****************************************************************************
// Shared variable(s)
//****************************************************************************

//****************************************************************************
// Prototype(s):
//****************************************************************************
void  initMitFir(void);
float mitFirFilter1kHz(float input);
void  mitFirLatchInput(float n);

// Matt's Filter
void initSoftFIRFilt(void);
float runSoftFirFilt(float inputVal);
void initCircularSoftFIRFilt(void);
float runCircularSoftFirFilt(float inputVal);

float filterButterworth30Hz(float inputVal);
float filterButterworth50Hz(float inputVal);
float filterTorqueButterworth(float inputVal);
float filterEncoderTorqueButterworth(float inputVal);
float filterTorqueDerivativeButterworth(float inputVal, int8_t reset);
float filterJointAngleButterworth(float inputVal, int8_t reset);
float filterJointAngleLimitOutputButterworth(float inputVal);
float filterJointVelocityButterworth(float inputVal, int8_t reset);
float filterJointAngleOutputButterworth(float inputVal);

float filterLowPass30(float *u, float *y, float refTorque);


float filterTorqueEncButterworth(float inputVal);
float filterMotorCommandButterworth(float inputVal);

float testingFilter(float input);
float testingVelFilter(float input);

float filterAccelButterworth15Hz(float value);

//****************************************************************************
// Definition(s):
//****************************************************************************
//#define LPF1 // Passband 100Hz, Stopband 200Hz
//#define LPF2 // Passband 50Hz, Stopband 100Hz
//#define LPF3 // Passband 50Hz, Stopband 70Hz
//#define LPF4 // Passband 35Hz, Stopband 70Hz


//****************************************************************************
// Structure(s):
//****************************************************************************

#endif	//INC_MIT_FIR_H
