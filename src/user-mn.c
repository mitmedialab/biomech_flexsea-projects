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
	[Lead developper] Jean-Francois (JF) Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] user: User Projects & Functions
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-23 | jfduval | Initial GPL-3.0 release
	*
****************************************************************************/

#ifdef BOARD_TYPE_FLEXSEA_MANAGE

//****************************************************************************
// Include(s)
//****************************************************************************

#include "user-mn.h"
#include "flexsea_user_structs.h"

//Barebone Rigid:
#if(ACTIVE_PROJECT == PROJECT_BB_RIGID)
#include "cmd-Rigid.h"
#include "user-mn-Rigid.h"
#endif	//PROJECT_BB_RIGID

//University of Michigan's Knee:
#if(ACTIVE_PROJECT == PROJECT_UMICH_KNEE)
#include "user-mn-Rigid.h"
//ToDo: switch to their include
#endif	//PROJECT_UMICH_KNEE

#if(ACTIVE_PROJECT == PROJECT_MIT_DLEG)
#include "user-mn-MIT-DLeg.h"
#include "user-mn-MIT-EMG.h"
#include <software_filter.h>
#endif

#if(ACTIVE_PROJECT == PROJECT_POCKET_2XDC)
#include "user-mn-MIT-PocketClimb.h"
#endif

//Dephy's Actuator Package (ActPack)
#if((ACTIVE_PROJECT == PROJECT_ACTPACK) || defined CO_ENABLE_ACTPACK)
#include "user-mn-ActPack.h"
#endif	//PROJECT_ACTPACK

#ifdef DEPHY
#include "dephy-mn.h"
#endif

//****************************************************************************
// Variable(s)
//****************************************************************************

//MIT Ankle 2-DoF:
#if(ACTIVE_PROJECT == PROJECT_ANKLE_2DOF)

struct ankle2dof_s ankle2dof_left, ankle2dof_right;

#endif	//PROJECT_ANKLE_2DOF

//Dev
#if(ACTIVE_PROJECT == PROJECT_DEV)
#include "dev.h"
#endif	//PROJECT_MOTORTB

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************

static void init_user_common(void);
inline static void user_fsm2_common(void);

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Initialization function - call once in main.c, before while()
void init_user(void)
{
	//Common to all projects:
	init_user_common();

	//RIC/NU Knee:
	#if(ACTIVE_PROJECT == PROJECT_RICNU_KNEE)
	init_ricnu_knee();
	#endif	//PROJECT_RICNU_KNEE

	//MIT Ankle 2-DoF:
	#if(ACTIVE_PROJECT == PROJECT_ANKLE_2DOF)
	init_ankle_2dof();
	#endif	//PROJECT_ANKLE_2DOF

	//Rigid:
	#if(ACTIVE_PROJECT == PROJECT_RIGID)
	init_rigid();
	#endif	//PROJECT_RIGID

	//Barebone Rigid:
	#if(ACTIVE_PROJECT == PROJECT_BB_RIGID)
	init_rigid();
	#endif	//PROJECT_BB_RIGID

	//University of Michigan's Knee:
	#if(ACTIVE_PROJECT == PROJECT_UMICH_KNEE)
	init_rigid();
	//ToDo: switch to their init
	#endif	//PROJECT_UMICH_KNEE

	//MIT D-Leg:
	#if(ACTIVE_PROJECT == PROJECT_MIT_DLEG)
	init_MIT_DLeg();
	#endif	//PROJECT_MIT_DLEG

	//MIT Pocket 2xDC / PocketClimb:
	#if(ACTIVE_PROJECT == PROJECT_POCKET_2XDC)
	init_MIT_PocketClimb();
	#endif	//PROJECT_POCKET_2XDC

	//Dephy's Actuator Package (ActPack)
	#if((ACTIVE_PROJECT == PROJECT_ACTPACK) || defined CO_ENABLE_ACTPACK)
	init_ActPack();
	#endif	//PROJECT_ACTPACK

	#ifdef DEPHY
	init_dephy();
	#endif
}

//Call this function in one of the main while time slots.
void user_fsm_1(void)
{
	#if(RUNTIME_FSM1 == ENABLED)

		//MIT Ankle 2-DoF:
		#if(ACTIVE_PROJECT == PROJECT_ANKLE_2DOF)
		ankle_2dof_fsm_1();
		#endif	//PROJECT_ANKLE_2DOF

		//RIC/NU Knee code
		#if(ACTIVE_PROJECT == PROJECT_RICNU_KNEE)
		ricnu_knee_fsm_1();
		#endif	//PROJECT_RICNU_KNEE

		//Dev
		#if(ACTIVE_PROJECT == PROJECT_DEV)
		dev_fsm_1();
		#endif	//PROJECT_DEV

		//Barebone Rigid:
		#if(ACTIVE_PROJECT == PROJECT_BB_RIGID)
		rigid_fsm_1();
		#endif	//PROJECT_BB_RIGID

		//University of Michigan's Knee:
		#if(ACTIVE_PROJECT == PROJECT_UMICH_KNEE)
		rigid_fsm_1();
		//ToDo: switch to their fsm1
		#endif	//PROJECT_UMICH_KNEE

		//Biomechatronics' DLeg:
		#if(ACTIVE_PROJECT == PROJECT_MIT_DLEG)
		MIT_DLeg_fsm_1();
		#endif	//PROJECT_MIT_DLEG

		//MIT Pocket 2xDC / PocketClimb:
		#if(ACTIVE_PROJECT == PROJECT_POCKET_2XDC)
		MIT_PocketClimb_fsm_1();
		#endif	//PROJECT_POCKET_2XDC

		//Dephy's Actuator Package (ActPack)
		#if((ACTIVE_PROJECT == PROJECT_ACTPACK) || defined CO_ENABLE_ACTPACK)
		ActPack_fsm_1();
		#endif	//PROJECT_ACTPACK

		#ifdef DEPHY
		dephy_fsm_1();
		#endif

	#endif	//(RUNTIME_FSM1 == ENABLED)
}

//Optional second FSM
//Call this function in one of the main while time slots.
void user_fsm_2(void)
{
	//Common:
	user_fsm2_common();

	#if(RUNTIME_FSM2 == ENABLED)

		//MIT Ankle 2-DoF:
		#if(ACTIVE_PROJECT == PROJECT_ANKLE_2DOF)
		ankle_2dof_fsm_2();
		#endif	//PROJECT_ANKLE_2DOF

		//RIC/NU Knee code
		#if(ACTIVE_PROJECT == PROJECT_RICNU_KNEE)
		ricnu_knee_fsm_2();
		#endif	//PROJECT_RICNU_KNEE

		//Barebone Rigid:
		#if(ACTIVE_PROJECT == PROJECT_BB_RIGID)
		rigid_fsm_2();
		#endif	//PROJECT_BB_RIGID

		//University of Michigan's Knee:
		#if(ACTIVE_PROJECT == PROJECT_UMICH_KNEE)
		rigid_fsm_2();
		//ToDo: switch to their fsm2
		#endif	//PROJECT_UMICH_KNEE

		//MIT D-Leg:
		#if(ACTIVE_PROJECT == PROJECT_MIT_DLEG)
//		MIT_DLeg_fsm_1();
		ActPack_fsm_2();	// used for communication
		#endif	//PROJECT_MIT_DLEG

		//MIT Pocket 2xDC / PocketClimb:
		#if(ACTIVE_PROJECT == PROJECT_POCKET_2XDC)
		MIT_PocketClimb_fsm_2();
		#endif	//PROJECT_POCKET_2XDC

		//Dephy's Actuator Package (ActPack)
		#if((ACTIVE_PROJECT == PROJECT_ACTPACK) || defined CO_ENABLE_ACTPACK && (ACTIVE_PROJECT != PROJECT_MIT_DLEG) )
		ActPack_fsm_2();
		#endif	//PROJECT_ACTPACK

		#ifdef DEPHY
		dephy_fsm_2();
		#endif

	#endif	//(RUNTIME_FSM2 == ENABLED)
}

void reset_user_code(void)
{
	#ifdef DEPHY
	reset_dephy();
	#endif
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

static void init_user_common(void)
{
	rigid1.ctrl.timestamp = 0;
}

inline static void user_fsm2_common(void)
{
	rigid1.ctrl.timestamp++;
}

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
