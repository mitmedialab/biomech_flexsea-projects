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

#ifndef INC_EMG_STAND_H
#define INC_EMG_STAND_H

//****************************************************************************
// EASY ACCESS
//****************************************************************************


//****************************************************************************
// Include(s)
//****************************************************************************


//****************************************************************************
// Shared variable(s)
//****************************************************************************
extern float stand_state;

//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void  updateStandJoint(GainParams* pgains);
void  get_stand_EMG(void);
float interpret_stand_EMG (void);
void  reset_EMG_stand(float resetAngle);

//****************************************************************************
// Definition(s):
//****************************************************************************

#define MOVEMENT_TIME    		500 //stand time in ms
#define COMEDOWN_TIME	 		500 //come down time in ms
#define STAND_HOLD_ANGLE 		15 //angle to hold when on tiptoes
#define EMG_STAND_WINDOW_SIZE 	500 //consider N ms of svm classification data



//****************************************************************************
// Structure(s)
//****************************************************************************


#endif	//INC_EMG_STAND_H

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
