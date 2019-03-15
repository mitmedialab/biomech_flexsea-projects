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
	[Lead developers] Luke Mooney, lmooney at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors] Matthew Carney, mcarney at mit dot edu, Tony Shu, tonyshu at mit dot edu
*****************************************************************************
	[This file] MIT DARPA Leg Main FSM
****************************************************************************/

#if defined INCLUDE_UPROJ_MIT_DLEG || defined BOARD_TYPE_FLEXSEA_PLAN
#if defined BOARD_TYPE_FLEXSEA_MANAGE || defined BOARD_TYPE_FLEXSEA_PLAN

#ifndef INC_MIT_DLEG
#define INC_MIT_DLEG

//****************************************************************************
// Include(s)
//****************************************************************************
#include "global-config.h"

//#include "user-mn-MIT-EMG.h"
#include "actuator_functions.h"
#include "walking_state_machine.h"
#include "state_variables.h"
#include "run_main_user_application.h"




//****************************************************************************
// Shared variable(s)
//****************************************************************************
extern struct act_s act1;	//define actuator structure shared
extern int8_t isEnabledUpdateSensors;

//****************************************************************************
// Structure(s)
//****************************************************************************

//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************
void initMITDLeg(void);
void MITDLegFsm1(void);
void MITDLegFsm2(void);


enum Gui_Modes {
    GUI_MODE_FL_CONTROL = 0,
    GUI_MODE_UR_CONTROL = 1,
    GUI_MODE_DR_CONTROL = 2,
    GUI_MODE_US_CONTROL = 3,
    GUI_MODE_DS_CONTROL = 4,
    GUI_MODE_NOM_CONTROL = 5,
    GUI_MODE_SW_CONTROL = 6,
    GUI_MODE_ADAPTIVE_CONTROL = 7,
    GUI_MODE_GAIT_EVENTS = 8,
    GUI_MODE_KINEMATICS = 9,
    GUI_MODE_LEARNING = 10,
    GUI_MODE_FEATURES = 11,
};

#endif	//INC_MIT_DLEG

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE || BOARD_TYPE_FLEXSEA_MANAGE
#endif //INCLUDE_UPROJ_MIT_DLEG || BOARD_TYPE_FLEXSEA_MANAGE
