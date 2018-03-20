/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-user' User projects
	Copyright (C) 2017 Dephy, Inc. <http://dephy.com/>
*****************************************************************************
	[Lead developper] Luke Mooney, lmooney at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors] Jean-Francois Duval, Elliott Rouse
*****************************************************************************
	[This file] user-mn-Rigid: FlexSEA-Rigid Manage user code
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2017-05-24 | jfduval | New file
	*
****************************************************************************/

#ifdef BOARD_TYPE_FLEXSEA_MANAGE

//****************************************************************************
// Include(s)
//****************************************************************************

#include "user-mn-Rigid.h"
#include "cmd-Rigid.h"
#include "flexsea_user_structs.h"
#include "dynamic_user_structs.h"
#include <flexsea_system.h>
#include <flexsea_comm.h>
#include "flexsea_board.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

uint8_t riInfo[2] = {PORT_RS485_2, PORT_RS485_2};
//int32_t enc_ang = 0, enc_ang_vel = 0;
//int16_t joint_ang = 0, joint_ang_vel = 0;
uint8_t enableRigidfsm2 = 0, rigidFSM2ready = 0;
//int16_t ctrl_ank_ang_deg = 0, ctrl_ank_vel = 0, ctrl_ank_ang_from_mot = 0;

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Logic Finite State Machine.
void rigid_fsm_1(void)
{
	static uint16_t t = 0;
	t++;
	dynamicUserData.time = t;
}

//Second state machine for the Exo project
//Deals with the communication between Manage and Execute
void rigid_fsm_2(void)
{
	static uint32_t timer = 0;

	//Wait X seconds before communicating
	if(timer < RIGID_FSM2_POWER_ON_DELAY)
	{
		rigidFSM2ready = 0;
		timer++;
		return;
	}

	rigidFSM2ready = 1;

	//FSM1 can disable this one:
	if(!enableRigidfsm2)
	{
		//Read all:
		tx_cmd_rigid_r(TX_N_DEFAULT, 0);
		packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_1, riInfo, SEND_TO_SLAVE);
	}
}

#endif //BOARD_TYPE_FLEXSEA_MANAGE
