/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-user' User projects
	Copyright (C) 2017 Dephy, Inc. <http://dephy.com/>
*****************************************************************************
	[Lead developper] Jean-Francois Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] user-ex: User Projects & Functions, FlexSEA-Rigid Ex
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2017-04-24 | jfduval | New release
	*
****************************************************************************/

#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

//****************************************************************************
// Include(s)
//****************************************************************************

#include "../inc/user-ex-rigid.h"

#if((ACTIVE_PROJECT == PROJECT_ACTPACK) || defined CO_ENABLE_ACTPACK)
#include "user-ex-ActPack.h"
#endif	//PROJECT_ACTPACK

#ifdef DEPHY
#include "dephy-ex.h"
#endif

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
	
	//ActPack:
	#if(ACTIVE_PROJECT == PROJECT_ACTPACK)
	initActPack();
	#endif	//PROJECT_ACTPACK
	
	#ifdef DEPHY
	init_dephy();
	#endif
}

//Call this function in one of the main while time slots.
void user_fsm(void)
{
	//ActPack:
	//(Note: Biomech's projects use ActPack)
	#if((ACTIVE_PROJECT == PROJECT_ACTPACK) || defined CO_ENABLE_ACTPACK)

		#if(RUNTIME_FSM == ENABLED)
			ActPack_fsm();
		#endif

	#endif	//PROJECT_ACTPACK

	#ifdef DEPHY
	dephy_fsm();
	#endif
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
