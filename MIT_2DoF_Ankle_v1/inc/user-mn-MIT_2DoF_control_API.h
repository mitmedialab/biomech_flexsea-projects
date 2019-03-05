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

#ifndef INC_ANKLE_2DOF_CONTROL_H
#define INC_ANKLE_2DOF_CONTROL_H

//****************************************************************************
// Include(s)
//****************************************************************************
//****************************************************************************
// EASY ACCESS
//****************************************************************************


//****************************************************************************
// Include(s)
//****************************************************************************


//****************************************************************************
// Shared variable(s)
//****************************************************************************

//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

uint8_t ankle_2dof_control_init_impedance(void);
uint8_t ankle_2dof_control_init_position(void);
void ankle_2dof_control_update(void);
void ankle_2dof_control_update_target(float pfdf, float inev);
void ankle_2dof_control_update_position(float pfdf, float inev, float kp, float ki, float kd);
void ankle_2dof_control_update_impedance(float pfdf, float inev, float kp, float ki, float kd);

uint8_t ankle_2dof_control_demo_position_fsm(void);
uint8_t ankle_2dof_control_demo_impedance_fsm(void);


//****************************************************************************
// Definition(s):
//****************************************************************************
// #define MAX_VOLTAGE 15000
// #define MAX_CURRENT 20000

//****************************************************************************
// Structure(s)
//****************************************************************************

#endif	//INC_ANKLE_2DOF_H

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
#endif //INCLUDE_UPROJ_MIT_A2DOF
