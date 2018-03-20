/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-projects' User projects
	Copyright (C) 2018 Dephy, Inc. <http://dephy.com/>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*****************************************************************************
	[Lead developer] Jean-Francois Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] user--mn-MIT-PocketClim: Demo state machine for Pocket 2x DC
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2018-03-02 | jfduval | New release
****************************************************************************/

#ifdef INCLUDE_UPROJ_MIT_POCKET_CLIMB

#include "main.h"

#ifdef BOARD_TYPE_FLEXSEA_MANAGE

#ifndef INC_MIT_POCKET_CLIMB_H
#define INC_MIT_POCKET_CLIMB_H
#define PID_CONTROLLER
//****************************************************************************
// Include(s)
//****************************************************************************


//****************************************************************************
// Shared variable(s)
//****************************************************************************



//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void init_MIT_PocketClimb(void);
void MIT_PocketClimb_fsm_1(void);
void MIT_PocketClimb_fsm_2(void);
void MIT_PocketClimb_fsm_3(void);

//****************************************************************************
// Definition(s):
//****************************************************************************


//Constants for tuning the controller

#define PFTORQUEGAIN			20
#define DFTORQUEGAIN			20
#define PFDFSTIFFGAIN			400
#define DPONTHRESH				2000

#define INTORQUEGAIN			20
#define EVTORQUEGAIN			20
#define INEVSTIFFGAIN			400
#define IEONTHRESH				2000

#define COCONTHRESH				0.5

#define LGGAIN 					1
#define TPGAIN					1
#define TAGAIN 					1
#define PLGAIN 					1
#define RANGELIMIT				.5

#define VIRTUALK 				0.85
#define VIRTUALJ				0.0025
#define VIRTUALB				0.1

//MANUAL CALIBRATION
#define MANMIN_LG				949
#define MANMAX_LG 				32862

#define MANMIN_TP				578
#define MANMAX_TP			 	10135

#define MANMIN_TA				1175
#define MANMAX_TA			    101771

#define MANMIN_PL				1173
#define MANMAX_PL				63669

#define FRAMELENGTH				.1635
#define FOOTLENGTH				.0467
#define THETAFRAME				14.1809
#define FRAMEHEIGHT				.1601
#define FOOTWIDTH				.0175
#define THETAFRAMEWIDTH			83.7620

//EMILY CONSTS
#define REST_ACT_LEN 	 		.176
#define ACTMAXLENGTH			.191
#define ACTMINLENGTH			.164
#define NEUTRALDF				20
#define MOTOR_0					0
#define MOTOR_1					1
#define OPEN_PWM_DEMO_HIGH		200	//Range: -500 to +500 (PWM duty cycle)
#define FORCE_GAIN				10	//pwm = ticks / gain => 5000 = max PWM
#define MAX_INTEGRAL_ERROR 		0
#define MAX_PWM 				500 //3276 //12V = 4095, 4095*0.85
#define MIN_PWM 				60 //819//12V = 4095, 4095*0.2 = 819
#define PID_KI 					0//
#define PID_KP 					.15 //.05 (not getting back to neutral)
#define PID_KD 					0 //.00001 //0.05 //0.05//0.005//0.5
#define DEBUG_TIMER 			4000
#define PWM_MOTORINIT			100
#define THRESH_MOTORINIT		10
#define DAMPINGTHRESH			10
#define ENCODERMIDPOINT			-4500


//Force Sensor
#define FORCE_DIR			-1		// Direction of positive force, Plantar Flexion is (+)
#define FORCE_STRAIN_GAIN 	500		// Gain on pocket
#define FORCE_STRAIN_BIAS	2.5		// Strain measurement Bias
#define FORCE_EXCIT			5		// Excitation Voltage
#define	FORCE_RATED_OUTPUT	0.002	// 2mV/V, Rated Output
#define FORCE_MAX			2224	// Newtons, for LCM200 load cell
#define FORCE_MAX_TICKS		(FORCE_STRAIN_GAIN * FORCE_EXCIT * FORCE_RATED_OUTPUT + FORCE_STRAIN_BIAS)/5 * 65536	// max ticks expected
#define FORCE_MIN_TICKS		(FORCE_STRAIN_BIAS - FORCE_STRAIN_GAIN * FORCE_EXCIT * FORCE_RATED_OUTPUT)/5 * 65536	// min ticks expected
#define FORCE_PER_TICK		2*FORCE_MAX / (FORCE_MAX_TICKS - FORCE_MIN_TICKS)	// Newtons/Tick
#define FORCE_THRESHOLD		8
#define UNLOAD_THRESHOLD	5

//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void interpret_EMG (float k, float b, float J);
void RK4_SIMPLE(float dtheta, float domega, float* cur_state);

// define global variables

//****************************************************************************
// Structure(s)
//****************************************************************************

#endif	//INC_MIT_POCKET_CLIMB_H

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
#endif //INCLUDE_UPROJ_MIT_POCKET_CLIMB
