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
	[Contributors] Luke Mooney, Elliott Rouse
*****************************************************************************
	[This file] knee: knee functions
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

#ifdef INCLUDE_UPROJ_RICNU_KNEE_V1
#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

//****************************************************************************
// Include(s)
//****************************************************************************

#include "../inc/user-ex-RICNU_Knee_v1.h"
#include <flexsea_board.h>
#include "user-ex.h"
#include "control.h"
#include "motor.h"
#include "trapez.h"
#include "main_fsm.h"
#include "flexsea_sys_def.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************

static void ricnu_knee_refresh_values(void);

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Call this function once in main.c, just before the while()
void init_ricnu_knee(void)
{
	setBoardID(SLAVE_ID);

	//FlexSEA-Execute setup:
	//Example:	ctrl.active_ctrl = CTRL_OPEN;	//Open controller
	//Example: motor_open_speed_1(0);			//0% PWM
	ctrl.active_ctrl = CTRL_OPEN;	//Position controller
	motor_open_speed_1(0);			//0% PWM
	#if(MOTOR_COMMUT == COMMUT_BLOCK)
	Coast_Brake_Write(1);			//Brake (rather than Coast)
	#endif

	//Position PID gains:
	//ctrl.position.gain.P_KP = 22;
	//ctrl.position.gain.P_KI = 5;

	//Starts off:
	LED_R_Write(1);
	LED_G_Write(1);
	LED_B_Write(1);
}

//Knee Finite State Machine.
//Call this function in one of the main while time slots (demo only)
void ricnu_knee_fsm(void)
{
	static uint16 time = 0;
	static uint8_t state = 0;
	static uint16 tmp_posi = 0;
	static uint16 tmp_posf = 0;
	static uint16 tmp_spdm = 0;
	static uint16 tmp_acc = 0;

	//Increment time
	time++;

	//Before going to a state we refresh values:
	ricnu_knee_refresh_values();

	if (time == 0)
	{
		state = 0;
		tmp_posi = *exec1.enc_ang;
		tmp_posf = 1000;
		tmp_spdm = 10000;
		tmp_acc = 10000;
	}

	if (time == 2000)
	{
		state = 1;
		tmp_posi = *exec1.enc_ang;
		tmp_posf = -2000;
		tmp_spdm = 20000;
		tmp_acc = 20000;
	}

	switch(state)
	{
		case 0:
			//Put some code here...
			//Example: motor_open_speed_1(85);			//0% PWM
			ctrl.position.posi = tmp_posi;
			ctrl.position.posf = tmp_posf;
			ctrl.position.spdm = tmp_spdm;
			ctrl.position.acc = tmp_acc;
			steps = trapez_gen_motion_1(tmp_posi, tmp_posf, tmp_spdm, tmp_acc);
			state = 2;
			break;
		case 1:
			ctrl.position.posi = tmp_posi;
			ctrl.position.posf = tmp_posf;
			ctrl.position.spdm = tmp_spdm;
			ctrl.position.acc = tmp_acc;
			steps = trapez_gen_motion_1(tmp_posi, tmp_posf, tmp_spdm, tmp_acc);

			time = -1000;
			state = 2;
			break;
		case 2:
			break;
		default:
			//Handle exceptions here
			break;
	}
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

//Note: 'static' makes them private; they can only called from functions in this
//file. It's safer than making everything global.

//Here's an example function:
static void ricnu_knee_refresh_values(void)
{
	//...
}
//That function can be called from the FSM.

#endif //BOARD_TYPE_FLEXSEA_EXECUTE
#endif //INCLUDE_UPROJ_RICNU_KNEE_V1
