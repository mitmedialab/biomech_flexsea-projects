/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-user' System commands & functions specific to
	user projects
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
	[Lead developper] Jean-Francois (JF) Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] flexsea_cmd_user: Interface to the user functions
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-10-27 | jfduval | Initial release
	*
****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

//****************************************************************************
// Include(s)
//****************************************************************************

#include <flexsea.h>
#include <flexsea_cmd_user.h>
#include <dynamic_user_structs.h>
#include "cmd-ActPack.h"

#ifdef BOARD_TYPE_FLEXSEA_EXECUTE
	#if(!defined BOARD_SUBTYPE_RIGID && !defined BOARD_SUBTYPE_POCKET)
		#include "../inc/user-ex.h"
	#else
		#include "user-ex-rigid.h"
	#endif
#endif

#ifdef BOARD_TYPE_FLEXSEA_PLAN
#include "user-plan.h"
#endif

#ifdef BOARD_TYPE_FLEXSEA_MANAGE
#include "user-mn.h"
#endif

#ifdef TEST_PC
#include "user-testPC.h"
#include "../Rigid/inc/cmd-Rigid.h"
#include "../Rigid/inc/cmd-Pocket.h"
#else
	#include "../MIT_2DoF_Ankle_v1/inc/cmd-MIT_2DoF_Ankle_v1.h"
	#include "../RICNU_Knee_v1/inc/cmd-RICNU_Knee_v1.h"
#endif	//TEST_PC

#ifndef TEST_PC
#include "../Rigid/inc/cmd-Rigid.h"
#include "../Rigid/inc/cmd-Pocket.h"
#endif	//TEST_PC

#ifdef DEPHY
#include "flexsea_cmd_dephy.h"
#endif

//****************************************************************************
// Variable(s)
//****************************************************************************


//****************************************************************************
// Function(s)
//****************************************************************************

//This gets called by flexsea_system's init_flexsea_payload_ptr(). Map all
//functions from this file to the array here. Failure to do so will send all
//commands to flexsea_payload_catchall().
void init_flexsea_payload_ptr_user(void)
{
	#if((ACTIVE_PROJECT == PROJECT_ANKLE_2DOF) && defined INCLUDE_UPROJ_MIT_A2DOF)
	//MIT 2-dof Ankle:
	flexsea_payload_ptr[CMD_A2DOF][RX_PTYPE_READ] = &rx_cmd_ankle2dof_rw;
	flexsea_payload_ptr[CMD_A2DOF][RX_PTYPE_REPLY] = &rx_cmd_ankle2dof_rr;
	#endif	//(ACTIVE_PROJECT == PROJECT_ANKLE2DOF)

	#if((ACTIVE_PROJECT == PROJECT_RICNU_KNEE) && defined INCLUDE_UPROJ_RICNU_KNEE_V1)
	//RIC/NU Knee:
	flexsea_payload_ptr[CMD_RICNU][RX_PTYPE_READ] = &rx_cmd_ricnu_rw;
	flexsea_payload_ptr[CMD_RICNU][RX_PTYPE_WRITE] = &rx_cmd_ricnu_w;
	flexsea_payload_ptr[CMD_RICNU][RX_PTYPE_REPLY] = &rx_cmd_ricnu_rr;
	#endif //(ACTIVE_SUBPROJECT == PROJECT_RICNU_KNEE)

	//Dephy's Actuator Package
	flexsea_payload_ptr[CMD_ACTPACK][RX_PTYPE_READ] = &rx_cmd_actpack_rw;
	//flexsea_payload_ptr[CMD_ACTPACK][RX_PTYPE_WRITE] = &rx_cmd_actpack_w;
	flexsea_payload_ptr[CMD_ACTPACK][RX_PTYPE_REPLY] = &rx_cmd_actpack_rr;

	flexsea_multipayload_ptr[CMD_ACTPACK][RX_PTYPE_READ] = &rx_multi_cmd_actpack_rw;
	flexsea_multipayload_ptr[CMD_ACTPACK][RX_PTYPE_REPLY] = &rx_multi_cmd_actpack_rr;

	//Rigid:
	flexsea_payload_ptr[CMD_READ_ALL_RIGID][RX_PTYPE_READ] = &rx_cmd_rigid_rw;
	flexsea_payload_ptr[CMD_READ_ALL_RIGID][RX_PTYPE_REPLY] = &rx_cmd_rigid_rr;

	//Pocket:
	flexsea_payload_ptr[CMD_READ_ALL_POCKET][RX_PTYPE_READ] = &rx_cmd_pocket_rw;
	flexsea_payload_ptr[CMD_READ_ALL_POCKET][RX_PTYPE_REPLY] = &rx_cmd_pocket_rr;

	#ifndef TEST_PC

	//Dynamic & Gait:
	init_flexsea_payload_ptr_dynamic();

	#endif //TEST_PC

	#ifdef DEPHY
	init_flexsea_payload_ptr_dephy();
	#endif
}

#ifdef __cplusplus
}
#endif
