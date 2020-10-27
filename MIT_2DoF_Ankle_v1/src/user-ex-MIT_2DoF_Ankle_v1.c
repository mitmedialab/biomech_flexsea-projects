/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-user' User projects
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
	[This file] user-ex-MIT_2DoF_Ankle_v1: User code running on Execute
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-10-28 | jfduval | New release
	*
****************************************************************************/

#ifdef INCLUDE_UPROJ_MIT_A2DOF
#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

//****************************************************************************
// Include(s)
//****************************************************************************

#include <flexsea_board.h>
#include "../inc/user-ex-MIT_2DoF_Ankle_v1.h"
#include "user-ex.h"
#include "control.h"
#include "motor.h"
#include "flexsea_sys_def.h"

//****************************************************************************
// Variable(s)
//****************************************************************************


//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************

static void ankle_refresh_values(void);

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Call this function once in main.c, just before the while()
void init_ankle_2dof(void)
{
	setBoardID(SLAVE_ID);

    //Controller setup:
    ctrl.active_ctrl = CTRL_OPEN;   //Position controller
    motor_open_speed_1(0);              //0% PWM
	#if(MOTOR_COMMUT == COMMUT_BLOCK)
    Coast_Brake_Write(1);               //Brake (regen)
	#endif

    //Position PID gains - initially 0
    ctrl.position.gain.P_KP = 0;
    ctrl.position.gain.P_KI = 0;
}

//Knee Finite State Machine.
//Call this function in one of the main while time slots.
void ankle_fsm(void)
{
    static int state = 0;

    ankle_refresh_values();

    switch (state)
    {
        case 0:
			//...
            break;
	}

	//Code does nothing, everything is happening on Manage
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

//Here's an example function:
static void ankle_refresh_values(void)
{
	//...
}

#endif //BOARD_TYPE_FLEXSEA_EXECUTE
#endif //INCLUDE_UPROJ_MIT_A2DOF
