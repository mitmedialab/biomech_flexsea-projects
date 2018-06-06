/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'user/ActPack' DDephy's Actuator Package (ActPack)
	Copyright (C) 2017 Dephy, Inc. <http://dephy.com/>
*****************************************************************************
	[Lead developper] Jean-Francois Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] user-mn-ActPack: User code running on Mn
****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2017-09-27 | jfduval | Initial release
	*
****************************************************************************/

#ifdef INCLUDE_UPROJ_ACTPACK
#ifdef BOARD_TYPE_FLEXSEA_MANAGE

#include "user-mn.h"

#if (ACTIVE_PROJECT == PROJECT_ACTPACK) || defined (CO_ENABLE_ACTPACK)

//Un-comment the next line to enable manual control from the GUI:
//#define MANUAL_GUI_CONTROL

//****************************************************************************
// Include(s)
//****************************************************************************

#include "user-mn-ActPack.h"
#include "flexsea_sys_def.h"
#include "flexsea_user_structs.h"
#include "flexsea_global_structs.h"
#include <flexsea_system.h>
#include <flexsea_comm.h>
#include "flexsea_board.h"
#include "user-mn-Rigid.h"
#include "cmd-ActPack.h"
#include "cmd-Rigid.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

//Comm & FSM2:
uint8_t apInfo[2] = {PORT_RS485_2, PORT_RS485_2};
uint8_t enableAPfsm2 = 0, apFSM2ready = 0, apCalibrated = 0;
uint8_t ActPackCoFSM = APC_FSM2_ENABLED;
int32_t apSetpoint = 0;
uint32_t apTimer1 = 0;
uint8_t apCalibFlag = 0;
uint8_t apSeriousError = 0;

struct ctrl_s ctrl;

writeEx_s writeEx = {.ctrl = CTRL_NONE, .setpoint = 0, .setGains = KEEP, \
						.offset = 0, .g[0] = 0, .g[1] = 0, .g[2] = 0, .g[3] = 0};

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Prepare the system:
void init_ActPack(void)
{
	//Rigid's pointers:
	init_rigid();
}

//Logic Finite State Machine.
void ActPack_fsm_1(void)
{
	static uint32_t timer = 0, deltaT = 0;
	static uint8_t fsm1State = 0;

	#ifdef CO_ENABLE_ACTPACK
		//In co-enabled scenarios we use the active project's FSM, not this one.
		return;	//Exiting here
	#endif

	//Wait X seconds before communicating
	if(timer < 2500)
	{
		timer++;
		return;
	}

	switch(fsm1State)
	{
		case 0:
			setControlMode(CTRL_OPEN);
			setMotorVoltage(0);
			fsm1State = 1;
			deltaT = 0;
			break;
		case 1:
			deltaT++;
			if(deltaT > 3000)
			{
				deltaT = 0;
				fsm1State = 2;
			}
			setMotorVoltage(0);
			break;
		case 2:
			deltaT++;
			if(deltaT > 3000)
			{
				deltaT = 0;
				fsm1State = 1;
			}
			setMotorVoltage(1000);
			break;
	}
}

//Second state machine for the Actuator package project
//Deals with the communication between Manage and Execute
void ActPack_fsm_2(void)
{
	static uint32_t timer = 0;
	static int8_t foundPoles = 0;

	//Wait X seconds before communicating
	if(timer < AP_FSM2_POWER_ON_DELAY)
	{
		apFSM2ready = 0;
		timer++;
		return;
	}

	apFSM2ready = 1;

	if (!foundPoles) {
		if (findPolesActpack()) {
			foundPoles = 1;
		}
		return;
	}

	//External controller can fully disable the comm:
	if(ActPackSys == SYS_NORMAL && ActPackCoFSM == APC_FSM2_ENABLED){enableAPfsm2 = 1;}
	else {enableAPfsm2 = 0;}

	//FSM1 can disable this one:
	if(enableAPfsm2)
	{
		#ifndef MANUAL_GUI_CONTROL

			//Normal mode - we use the dpeb31 command:

			//Special Data Collection mode, cancels any setpoint and controller:
			#ifdef DATA_COLLECT_NO_MOTOR
				writeEx.ctrl = CTRL_NONE;
				writeEx.setpoint = 0;
			#endif

			tx_cmd_actpack_rw(TX_N_DEFAULT, writeEx.offset, writeEx.ctrl, writeEx.setpoint, \
											writeEx.setGains, writeEx.g[0], writeEx.g[1], \
											writeEx.g[2], writeEx.g[3], 0);
			packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_1, apInfo, SEND_TO_SLAVE);

			//Reset KEEP/CHANGE once set:
			if(writeEx.setGains == CHANGE)
			{
				writeEx.setGains = KEEP;
			}

		#else

			//Development mode - we use the Read all command:
			tx_cmd_rigid_r(TX_N_DEFAULT, 0);
			packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_1, apInfo, SEND_TO_SLAVE);

		#endif
	}
}

//This should be static, but exo-angles needs it. (ToDo)
void init_current_controller(void)
{
	ctrl.active_ctrl = CTRL_CURRENT;
	ctrl.current.gain.g0 = CTRL_I_KP;
	ctrl.current.gain.g1 = CTRL_I_KI;
	ctrl.current.setpoint_val = 0;

	//Prep for comm:
	writeEx.ctrl = CTRL_CURRENT;
	writeEx.g[0] = CTRL_I_KP;
	writeEx.g[1] = CTRL_I_KI;
	writeEx.setpoint = 0;
	writeEx.setGains = CHANGE;
}

void setMotorVoltage(int32_t v)
{
	if(writeEx.ctrl == CTRL_OPEN)
	{
		writeEx.setpoint = v;
	}
}

void setMotorCurrent(int32_t i)
{
	ctrl.current.setpoint_val =  i;

	//Prep for comm:
	if(writeEx.ctrl == CTRL_CURRENT)
	{
		writeEx.setpoint = i;
	}
}

void setMotorPosition(int32_t i)
{
	if(writeEx.ctrl == CTRL_POSITION)
	{
		writeEx.setpoint = i;
		ctrl.position.setp =  i;
	}
	else if(writeEx.ctrl == CTRL_IMPEDANCE)
	{
		writeEx.setpoint = i;
		ctrl.impedance.setpoint_val = i;
	}
}

void setControlMode(uint8_t m)
{
	ctrl.active_ctrl = m;
	writeEx.ctrl = m;
}

void setControlGains(int16_t g0, int16_t g1, int16_t g2, int16_t g3)
{
	writeEx.g[0] = g0;
	writeEx.g[1] = g1;
	writeEx.g[2] = g2;
	writeEx.g[3] = g3;
	writeEx.setGains = CHANGE;
}

int8_t findPolesActpack(void) {
	static uint32_t timer = 0;
	static int8_t polesState = 0;

	timer++;

	switch(polesState) {
		case 0:
			//Disable FSM2:
			disableActPackFSM2();
			if(timer > 100)
			{
				polesState = 1;
			}

			return 0;

		case 1:
			//Send Find Poles command:

			tx_cmd_calibration_mode_rw(TX_N_DEFAULT, CALIBRATION_FIND_POLES);
			packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_1, apInfo, SEND_TO_SLAVE);
			polesState = 2;
			timer = 0;

			return 0;

		case 2:

			if(timer >= 44*1000)
			{
				//Enable FSM2, position controller
				enableActPackFSM2();
				return 1;
			}
			return 0;


		default:

			return 0;

	}

	return 0;
}

void enableActPackFSM2(void){ActPackCoFSM = APC_FSM2_ENABLED;}
void disableActPackFSM2(void){ActPackCoFSM = APC_FSM2_DISABLED;}

//****************************************************************************
// Private Function(s)
//****************************************************************************

#endif	//(ACTIVE_PROJECT == PROJECT_ACTPACK)
#endif 	//BOARD_TYPE_FLEXSEA_MANAGE

#endif 	//INCLUDE_UPROJ_ACTPACK
