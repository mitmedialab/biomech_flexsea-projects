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
	[Lead developer] Jean-Francois Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors] Matthew Carney, mcarney at mit dot edu, Tony Shu, tonyshu at mit dot edu
*****************************************************************************
	[This file] MIT DARPA Leg Main FSM
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2018-02-24 | jfduval | New release
****************************************************************************/

#if defined INCLUDE_UPROJ_MIT_DLEG || defined BOARD_TYPE_FLEXSEA_PLAN
#if defined BOARD_TYPE_FLEXSEA_MANAGE || defined BOARD_TYPE_FLEXSEA_PLAN

#ifndef INC_MIT_DLEG_H
#define INC_MIT_DLEG_H

//****************************************************************************
// Include(s)
//****************************************************************************
#include "global-config.h"
#include "main.h"
//#include "user-mn-MIT-EMG.h"
#include "actuator_functions.h"
//#include "walking_state_machine.h"
#include "walking_knee_ankle_state_machine.h"
#include "state_variables.h"
#include "cmd-ActPack.h"


//****************************************************************************
// Shared variable(s)
//****************************************************************************

extern struct act_s act1, act2;	//define actuator structure shared
extern int8_t isEnabledUpdateSensors;

extern uint8_t calibrationFlags, calibrationNew;
extern int8_t zeroLoadCell; 		// used for zeroing the load cell.


//****************************************************************************
// Structure(s)
//****************************************************************************

//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void initMITDLeg(void);
void MITDLegFsm1(void);
void MITDLegFsm2(void);

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************

//User writes/reads
void updateUserWrites(Act_s *actx, WalkParams *wParams, ActTestSettings *act1TestSet, TorqueRep *torqueRep);
void initializeUserWrites(Act_s *actx, WalkParams *wParams, ActTestSettings *act1TestSet, TorqueRep *torqueRep);
void updateGenVarOutputs(Act_s *actx, WalkParams *wParams, ActTestSettings *act1TestSet);

//****************************************************************************
// Definition(s):
//****************************************************************************

#endif	//INC_MIT_DLEG

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE || BOARD_TYPE_FLEXSEA_MANAGE
#endif //INCLUDE_UPROJ_MIT_DLEG || BOARD_TYPE_FLEXSEA_MANAGE
