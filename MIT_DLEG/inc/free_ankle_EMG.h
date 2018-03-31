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
	[Lead developer] Luke Mooney, lmooney at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] user_ankle_2dof: EMG FSM functions
****************************************************************************/

#include "main.h"
#include "state_variables.h"

#ifdef BOARD_TYPE_FLEXSEA_MANAGE

#ifndef INC_EMG_FREE_H
#define INC_EMG_FREE_H

//****************************************************************************
// EASY ACCESS
//****************************************************************************

//HANDLE EMG FROM SEONG
#define GAIN_LG					1.0
#define GAIN_TA					1.0
#define EMG_IN_MAX				10000

//Constants for tuning the controller
#define PF_TORQUE_GAIN			100
#define DF_TORQUE_GAIN			30
#define PFDF_STIFF_GAIN			1.0
#define DP_ON_THRESH			0.1

#define COCON_THRESH			0.6 //co-contraction threshold (high for Hugh)


//VIRTUAL DYNAMIC JOINT PARAMS
#define VIRTUAL_K				1.0
#define VIRTUAL_B				0.085
#define VIRTUAL_J				0.0025


//****************************************************************************
// Include(s)
//****************************************************************************


//****************************************************************************
// Shared variable(s)
//****************************************************************************
extern int32_t EMGavgs[2];
extern float PFDF_state[3];
extern float equilibriumAngle;

//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void updateVirtualJoint(GainParams* pgains);
void get_EMG(void);
void interpret_EMG (float k, float b, float J);
void RK4_SIMPLE(float dtheta, float domega, float* cur_state);
int16_t windowSmoothEMG0(int16_t val);
int16_t windowSmoothEMG1(int16_t val);

//****************************************************************************
// Definition(s):
//****************************************************************************


//****************************************************************************
// Structure(s)
//****************************************************************************


#endif	//INC_EMG_FSM_H

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
