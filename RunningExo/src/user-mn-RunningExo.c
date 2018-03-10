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

#if (ACTIVE_PROJECT == PROJECT_RUNNING_EXO)

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
#include "user-mn-RunningExo.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

//Comm & FSM2:

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Prepare the system:
void init_runningExo(void)
//TODO
{

}

//Running Exo state machine
void RunningExo_fsm_1(void)
//TODO
{

}

//Second state machine for the Running Exo
void RunningExo_fsm_2(void)
//TODO

{

}

//****************************************************************************
// Private Function(s)
//****************************************************************************

#endif	//(ACTIVE_PROJECT == PROJECT_ACTPACK)
#endif 	//BOARD_TYPE_FLEXSEA_MANAGE

#endif 	//INCLUDE_UPROJ_ACTPACK
