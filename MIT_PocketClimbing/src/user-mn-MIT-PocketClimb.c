/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-projects' User projects
	Copyright (C) 2018 Dephy, Inc. <http://dephy.com/>

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
	[Contributors]
*****************************************************************************
	[This file] user--mn-MIT-PocketClim: Demo state machine for Pocket 2x DC
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2018-03-02 | jfduval | New release
****************************************************************************/

#ifdef INCLUDE_UPROJ_MIT_POCKET_CLIMB
#ifdef BOARD_TYPE_FLEXSEA_MANAGE

//****************************************************************************
// Include(s)
//****************************************************************************

#include "user-mn.h"
#include "user-mn-MIT-PocketClimb.h"
#include "user-mn-ActPack.h"
#include <flexsea_comm.h>
#include <math.h>
#include "flexsea_sys_def.h"
#include "flexsea_system.h"
#include "flexsea_cmd_calibration.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

uint8_t mitClimbInfo[2] = {PORT_RS485_2, PORT_RS485_2};

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************

static void dualOpenSpeed(void);
static void forceFeedback(void);

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Call this function once in main.c, just before the while()
void init_MIT_PocketClimb(void)
{

}

//MIT DLeg Finite State Machine.
//Call this function in one of the main while time slots.
void MIT_PocketClimb_fsm_1(void)
{
	#if(ACTIVE_PROJECT == PROJECT_POCKET_2XDC)

	static uint32_t time = 0, state = 0;

	//Increment time (1 tick = 1ms)
	time++;

	switch(state)
	{
		case 0:
			//Same power-on delay as FSM2 (+ some time to refresh values)
			if(time >= (AP_FSM2_POWER_ON_DELAY + 25))
			{
				state = 1;
				time = 0;
			}

			break;

		case 1:
			//dualOpenSpeed();
			forceFeedback();
			break;

		default:
			//Handle exceptions here
			break;
	}

	#endif	//ACTIVE_PROJECT == PROJECT_POCKET_2XDC
}


//Second state machine for the DLeg project
void MIT_PocketClimb_fsm_2(void)
{
	#if(ACTIVE_PROJECT == PROJECT_POCKET_2XDC)

		//Currently unused - we use ActPack's FSM2 for comm

	#endif	//ACTIVE_PROJECT == PROJECT_POCKET_2XDC
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

static void dualOpenSpeed(void)
{
	static uint32_t deltaT = 0;
	static uint8_t fsm1State = 0;

	switch(fsm1State)
	{
		case 0:

			//Set controllers:
			setControlMode(CTRL_OPEN, LEFT_MOTOR);
			setMotorVoltage(0, LEFT_MOTOR);
			setControlMode(CTRL_OPEN, RIGHT_MOTOR);
			setMotorVoltage(0, RIGHT_MOTOR);
			fsm1State = 1;
			deltaT = 0;
			break;

		case 1:

			//1s clockwise:
			setMotorVoltage(OPEN_PWM_DEMO_HIGH, LEFT_MOTOR);
			setMotorVoltage(OPEN_PWM_DEMO_HIGH, RIGHT_MOTOR);

			deltaT++;
			if(deltaT > 1000)
			{
				deltaT = 0;
				fsm1State = 2;
			}

			break;

		case 2:

			//1s static:
			setMotorVoltage(0, LEFT_MOTOR);
			setMotorVoltage(0, RIGHT_MOTOR);

			deltaT++;
			if(deltaT > 1000)
			{
				deltaT = 0;
				fsm1State = 3;
			}

			break;

		case 3:

			//1s counter-clockwise:
			setMotorVoltage(-OPEN_PWM_DEMO_HIGH, LEFT_MOTOR);
			setMotorVoltage(-OPEN_PWM_DEMO_HIGH, RIGHT_MOTOR);

			deltaT++;
			if(deltaT > 1000)
			{
				deltaT = 0;
				fsm1State = 4;
			}

			break;

		case 4:

			//5s static:
			setMotorVoltage(0, LEFT_MOTOR);
			setMotorVoltage(0, RIGHT_MOTOR);

			deltaT++;
			if(deltaT > 5000)
			{
				deltaT = 0;
				fsm1State = 1;
			}

			break;
	}
}

uint16_t zf[2] = {0,0};
int32_t deltaF[2] = {0,0};
int16_t pwm[2] = {0,0};

static void forceFeedback(void)
{
	static uint8_t firstTime = 1;


	//The first time we run this function we save the SG values
	if(firstTime)
	{
		//Measure zeros:
		zf[LEFT_MOTOR] = pocket1.ex[LEFT_MOTOR].strain;
		zf[RIGHT_MOTOR] = pocket1.ex[RIGHT_MOTOR].strain;

		//Set controllers:
		setControlMode(CTRL_OPEN, LEFT_MOTOR);
		setMotorVoltage(0, LEFT_MOTOR);
		setControlMode(CTRL_OPEN, RIGHT_MOTOR);
		setMotorVoltage(0, RIGHT_MOTOR);

		firstTime = 0;
	}

	//Measure difference between no-load and current value:
	deltaF[LEFT_MOTOR] = (int32_t)pocket1.ex[LEFT_MOTOR].strain - (int32_t)zf[LEFT_MOTOR];
	deltaF[RIGHT_MOTOR] = (int32_t)pocket1.ex[RIGHT_MOTOR].strain - (int32_t)zf[RIGHT_MOTOR];

	//Calculate PWM and send it:
	pwm[LEFT_MOTOR] = (int16_t)deltaF[LEFT_MOTOR] / FORCE_GAIN;
	pwm[RIGHT_MOTOR] = (int16_t)deltaF[RIGHT_MOTOR] / FORCE_GAIN;
	setMotorVoltage(pwm[LEFT_MOTOR], LEFT_MOTOR);
	setMotorVoltage(pwm[RIGHT_MOTOR], RIGHT_MOTOR);
}

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
#endif //INCLUDE_UPROJ_MIT_POCKET_CLIMB
