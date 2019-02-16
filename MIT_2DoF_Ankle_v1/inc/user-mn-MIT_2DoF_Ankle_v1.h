/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-manage' Mid-level computing, and networking
	Copyright (C) 2016 Dephy, Inc. <http://dephy.com/>

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
	[Lead developper] Luke Mooney, lmooney at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] user_ankle_2dof: 2-DoF Ankle Functions
****************************************************************************/

#ifdef INCLUDE_UPROJ_MIT_A2DOF
#include "main.h"

#ifdef BOARD_TYPE_FLEXSEA_MANAGE

#ifndef INC_ANKLE_2DOF_H
#define INC_ANKLE_2DOF_H

//****************************************************************************
// Include(s)
//****************************************************************************

//Constants for tuning the controller
#define AUTOCAL					0
#define LEFTY					1

#define VIRTUALFIELD			0
#define DFDAMPING				5 //Keep below 6
#define IESPRING				5
#define SAFEZONE				100

#define PFTORQUEGAIN			50
#define DFTORQUEGAIN			50
#define PFDFSTIFFGAIN			100
#define DPONTHRESH				0.1

#define INTORQUEGAIN			40
#define EVTORQUEGAIN			40
#define INEVSTIFFGAIN			10
#define IEONTHRESH				0.1

#define COCONTHRESH				0.5

//STIMULATION THRESHOLDS
#define ISON_STIM				0
#define BLANKWINDOW				0
#define DPSTIM_ONTHRESH			1500
#define IESTIM_ONTHRESH			500
#define STIMSTIFFGAIN			3 //Keep at 5 or lower


//MANUAL CALIBRATION
#define MANMIN_LG				949
#define MANMAX_LG 				32862

#define MANMIN_TP				578
#define MANMAX_TP			 	10135

#define MANMIN_TA				1175
#define MANMAX_TA			    101771

#define MANMIN_PL				1173
#define MANMAX_PL				63669

// Other Constants
#define REST_MOTOR_ANGLE 	1000

//****************************************************************************
// Shared variable(s)
//****************************************************************************

extern int16_t glob_var_1;
extern int16_t glob_var_2;
extern int16_t glob_var_3;

//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void init_ankle_2dof(void);
void ankle_2dof_fsm_1(void);
void ankle_2dof_fsm_2(void);
int16_t get_ankle_ang(double);
int16_t get_ankle_trans(double);
void set_ankle_torque_1(int32_t);
void set_ankle_torque_2(int32_t);

//****************************************************************************
// Definition(s):
//****************************************************************************

//Constants used by get_ankle_ang():
#define A0 				(202.2+1140.0)
#define A1 				1302.0
#define A2				-39.06
#define B1 				14.76
#define B2 				-7.874
#define W				0.00223

//****************************************************************************
// Structure(s)
//****************************************************************************

#endif	//INC_ANKLE_2DOF_H

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
#endif //INCLUDE_UPROJ_MIT_A2DOF
