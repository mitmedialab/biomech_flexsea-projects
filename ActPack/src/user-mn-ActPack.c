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
#include "cmd-Pocket.h"
#include "mn-MotorControl.h"

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
	init_pocket();

	initWriteEx(0);
	initWriteEx(1);
}

//Logic Finite State Machine.
void ActPack_fsm_1(void)
{
	static uint32_t timer = 0, deltaT = 0;
	static uint8_t fsm1State = 0;
	uint8_t ch = 0;

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
			setControlMode(CTRL_OPEN, ch);
			setMotorVoltage(0, ch);
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
			setMotorVoltage(0, ch);
			break;
		case 2:
			deltaT++;
			if(deltaT > 3000)
			{
				deltaT = 0;
				fsm1State = 1;
			}
			setMotorVoltage(1000, ch);
			break;
	}
}

//Second state machine for the Actuator package project
//Deals with the communication between Manage and Execute
void ActPack_fsm_2(void)
{
	static uint32_t timer = 0;
	static uint8_t dualOffs = 0;

	//Wait X seconds before communicating
	if(timer < AP_FSM2_POWER_ON_DELAY)
	{
		apFSM2ready = 0;
		timer++;
		return;
	}

	apFSM2ready = 1;

	//External controller can fully disable the comm:
	if(ActPackSys == SYS_NORMAL && ActPackCoFSM == APC_FSM2_ENABLED){enableAPfsm2 = 1;}
	else {enableAPfsm2 = 0;}

	//FSM1 can disable this one:
	if(enableAPfsm2)
	{
		#ifndef MANUAL_GUI_CONTROL

			//Special Data Collection mode, cancels any setpoint and controller:
			#ifdef DATA_COLLECT_NO_MOTOR
				writeEx.ctrl = CTRL_NONE;
				writeEx.setpoint = 0;
			#endif

			#ifndef BOARD_SUBTYPE_POCKET
			tx_cmd_actpack_rw(TX_N_DEFAULT, writeEx[0].offset, writeEx[0].ctrl, writeEx[0].setpoint, \
											writeEx[0].setGains, writeEx[0].g[0], writeEx[0].g[1], \
											writeEx[0].g[2], writeEx[0].g[3], 0);
			(void)dualOffs;
			#else
			dualOffs ^= 1;	//Toggle between the two channels
			tx_cmd_pocket_rw(TX_N_DEFAULT, dualOffs, writeEx[0].ctrl, writeEx[0].setpoint, \
														writeEx[0].setGains, writeEx[0].g[0], writeEx[0].g[1], \
														writeEx[0].g[2], writeEx[0].g[3], writeEx[1].ctrl, \
														writeEx[1].setpoint, writeEx[1].setGains, writeEx[1].g[0], \
														writeEx[1].g[1], writeEx[1].g[2], writeEx[1].g[3], 0);
			#endif
			packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_1, apInfo, SEND_TO_SLAVE);

			//Reset KEEP/CHANGE once set:
			if(writeEx[0].setGains == CHANGE){writeEx[0].setGains = KEEP;}
			if(writeEx[1].setGains == CHANGE){writeEx[1].setGains = KEEP;}

		#else

			//Development mode - we use the Read all command:
			tx_cmd_rigid_r(TX_N_DEFAULT, 0);
			packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_1, apInfo, SEND_TO_SLAVE);

		#endif
	}
}

void enableActPackFSM2(void){ActPackCoFSM = APC_FSM2_ENABLED;}
void disableActPackFSM2(void){ActPackCoFSM = APC_FSM2_DISABLED;}

//****************************************************************************
// Private Function(s)
//****************************************************************************

#endif	//(ACTIVE_PROJECT == PROJECT_ACTPACK)
#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
