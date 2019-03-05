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

#ifndef INC_ANKLE_2DOF_STMJOINT_H
#define INC_ANKLE_2DOF_STMJOINT_H

//****************************************************************************
// Include(s)
//****************************************************************************
//****************************************************************************
// EASY ACCESS
//****************************************************************************

//Constants for tuning the controller
#define CTRL_IMPEDANCE          0
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

// #define MANMIN_LG				1000
// #define MANMAX_LG 				10000
//
// #define MANMIN_TP				1000
// #define MANMAX_TP			 	10000
//
// #define MANMIN_TA				1000
// #define MANMAX_TA			    10000
//
// #define MANMIN_PL				1000
// #define MANMAX_PL				10000
//

//****************************************************************************
// Include(s)
//****************************************************************************


//****************************************************************************
// Shared variable(s)
//****************************************************************************



// extern int16_t per_PFDF;
// extern int16_t per_INEV;
// extern uint16_t stiff_PFDF;
// extern int16_t DP_torque_trans;
// extern int16_t IE_torque_trans;
// extern int32_t EMGavgs[4];

//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

uint8_t ankle_2dof_STMjoint_EMG_fsm(void);
uint8_t ankle_2dof_STMjoint_calibration_fsm(void);
uint8_t ankle_2dof_STMjoint_init(void);
void interpret_EMG (float k, float b, float J);

float ankle_2dof_get_PFDF(void);
float ankle_2dof_get_INEV(void);

//****************************************************************************
// Definition(s):
//****************************************************************************

//****************************************************************************
// Structure(s)
//****************************************************************************

#endif	//INC_ANKLE_2DOF_H

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
#endif //INCLUDE_UPROJ_MIT_A2DOF
