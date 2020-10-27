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
	[Contributors] Jean-Francois Duval, Elliott Rouse
*****************************************************************************
	[This file] knee: knee functions
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	* 2016-11-21 | jfduval | Updated to mimic MIT_2DoF_Ankle_v1
	*
****************************************************************************/

#ifdef INCLUDE_UPROJ_RICNU_KNEE_V1
#ifdef BOARD_TYPE_FLEXSEA_MANAGE

//****************************************************************************
// Include(s)
//****************************************************************************

#include "user-mn-RICNU_Knee_v1.h"
#include "cmd-RICNU_Knee_v1.h"
#include <math.h>
#include "flexsea_system.h"
#include "flexsea_sys_def.h"
#include "flexsea.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

//Use this to share info between the two FSM:
uint8_t my_ricnu_control = CTRL_NONE;
int16_t my_ricnu_pwm = 0;
int16_t my_ricnu_cur = 0;
uint8_t offset = 0;

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
	//Default init state:
	my_ricnu_control = CTRL_NONE;
	my_ricnu_pwm = 0;

	//Data structures:
	ricnu_1.ex = &exec1;
	ricnu_1.st = &strain1;
}

//Knee Finite State Machine.
//Call this function in one of the main while time slots.
//Note: this is a dumb example to showcase how to use the code, modify for
//your application!
void ricnu_knee_fsm_1(void)
{
	#if(ACTIVE_PROJECT == PROJECT_RICNU_KNEE)

    static uint32_t time = 0;
    static int8_t state = 0;

    //Increment time (1 tick = 1ms)
    time++;

	//Before going to a state we refresh values:
	ricnu_knee_refresh_values();

	switch(state)
	{
		case 0:	//Wait for 10 seconds to let everything load

			my_ricnu_control = CTRL_OPEN;
			my_ricnu_pwm = 0;

			if (time >= 10000)
			{
				time = 0;
				state = 1;
			}

			break;

		case 1:	//PWM = 100 for 5s

			my_ricnu_pwm = 100;

			if (time >= 5000)
			{
				time = 0;
				state = 2;
			}

			break;

		case 2:	//PWM = 0 for 5s

			my_ricnu_pwm = 0;

			if (time >= 5000)
			{
				time = 0;
				state = 1;
			}

			break;

		default:
			//Handle exceptions here
			break;
	}

	#endif 	//(ACTIVE_PROJECT == PROJECT_RICNU_KNEE)
}

//Second state machine for the RIC/NU Knee project:
//Deals with the communication between Manage and 1x Execute
//This function is called at 1kHz. We divide it down to 250Hz for now (ToDo: test faster)
void ricnu_knee_fsm_2(void)
{
	#if(ACTIVE_PROJECT == PROJECT_RICNU_KNEE)

	#ifdef BOARD_SUBTYPE_RIGID

	//ActPack_fsm_2();	//WiP, use at your own risk

	#else

	static uint8_t ex_refresh_fsm_state = 0;
	static uint32_t timer = 0;
	uint8_t info[2] = {PORT_RS485_1, PORT_RS485_1};

	//This FSM talks to the slaves at 250Hz each
	switch(ex_refresh_fsm_state)
	{
		case 0:		//Power-up

			if(timer < 7000)
			{
				//We wait 7s before sending the first commands
				timer++;
			}
			else
			{
				//Ready to start transmitting
				ex_refresh_fsm_state = 1;
			}

			break;

		case 1:	//Communicating with Execute #1, offset = 0

			info[0] = PORT_RS485_1;
			offset++;
			offset %= 2;
			tx_cmd_ricnu_rw(TX_N_DEFAULT, offset, my_ricnu_control, my_ricnu_pwm, KEEP, 0, 0, 0, 0);
			packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_1, info, SEND_TO_SLAVE);
			ex_refresh_fsm_state++;

			break;

		case 2:	//

			//Skipping one cycle
			ex_refresh_fsm_state++;

			break;

		case 3:	//Communicating with Execute #1, offset = 1

			info[0] = PORT_RS485_1;
			tx_cmd_ricnu_rw(TX_N_DEFAULT, 1, my_ricnu_control, my_ricnu_pwm, KEEP, 0, 0, 0, 0);
			packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_1, info, SEND_TO_SLAVE);
			ex_refresh_fsm_state++;

			break;

		case 4:	//

			//Skipping one cycle
			ex_refresh_fsm_state = 1;

			break;
	}

	#endif

	#endif	//ACTIVE_PROJECT == PROJECT_RICNU_KNEE
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

//Note: 'static' makes them private; they can only called from functions in this
//file. It's safer than making everything global.

//Here's an example function:
static void ricnu_knee_refresh_values(void)
{
	//(compute/update values here)
}
//That function can be called from the FSM.

#endif //BOARD_TYPE_FLEXSEA_MANAGE
#endif //INCLUDE_UPROJ_RICNU_KNEE_V1
