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
	[Lead developper] Jean-Francois Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] user-ex: User Projects & Functions, FlexSEA-Execute
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-10-30 | jfduval | New release
	*
****************************************************************************/

#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

/*Important: we reached a point where we couldn't support all configurations
  without changing the TopDesign (we ran out of ressources). You might have
  to select a different TopDesign file than the one included by default (check
  the folded, there is more than one included) */

//****************************************************************************
// Include(s)
//****************************************************************************

#include "../inc/user-ex.h"
#include "user-ex-DpEb21.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************

static void init_barebone(void);

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Initialization function - call once in main.c, before while()
void init_user(void)
{
	//Barebone:
	#if(ACTIVE_PROJECT == PROJECT_BAREBONE)
	init_barebone();
	#endif	//PROJECT_BAREBONE

	//MIT Ankle 2-DoF:
	#if(ACTIVE_PROJECT == PROJECT_ANKLE_2DOF)
	init_ankle_2dof();
	#endif	//PROJECT_ANKLE_2DOF

	//RIC/NU Knee:
	#if(ACTIVE_PROJECT == PROJECT_RICNU_KNEE)
	init_ricnu_knee();
	#endif	//PROJECT_RICNU_KNEE

	//DpEb21
	#if(ACTIVE_PROJECT == PROJECT_DPEB21)
	initDpEb21();
	#endif	//PROJECT_MOTORTB
}

//Call this function in one of the main while time slots.
void user_fsm(void)
{
	//DpEB2.1:
	#if(ACTIVE_PROJECT == PROJECT_DPEB21)
		DpEb21_refresh_values();
		#if(RUNTIME_FSM == ENABLED)
			DpEb21_fsm();
		#endif
	#endif	//PROJECT_DPEB21
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

static void init_barebone(void)
{
	//Barebone:
	#if(ACTIVE_PROJECT == PROJECT_BAREBONE)
	setBoardID(SLAVE_ID);
	#endif	//PROJECT_BAREBONE
}

#endif //BOARD_TYPE_FLEXSEA_EXECUTE
