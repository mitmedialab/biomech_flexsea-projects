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
#include "flexsea_user_structs.h"
#include "flexsea_sys_def.h"
#include "filters.h"
#include "control.h"
#include "motor.h"

#if((ACTIVE_PROJECT == PROJECT_ACTPACK) || defined CO_ENABLE_ACTPACK)
#include "user-ex-ActPack.h"
#endif	//PROJECT_ACTPACK

#ifdef DEPHY
#include "dephy-ex.h"
#endif

//****************************************************************************
// Variable(s)
//****************************************************************************

struct diffarr_s mot_ang_clks;
int32_t mot_ang = 0;
int32_t mot_ang_offset = 0; //initialized offset to match with calibration
int32_t mot_vel = 0;
int32_t mot_acc = 0;

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************

static void refresh_values(void);
static void init_project(void);

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Initialization function - call once in main.c, before while()
void init_user(void)
{
	//Common setup:
	init_project();
	
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
	//Refresh sensor values - common across projects:
	refresh_values();
	
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

//Common setup
static void init_project(void)
{
	setBoardID(SLAVE_ID);
	
	//Controller setup:
	ctrl[0].active_ctrl = CTRL_NONE;	//No controller at boot
	setMotorVoltage(0, 0);				//0% PWM
}

//Refresh sensor values - common across projects
static void refresh_values(void)
{	
	mot_ang = *rigid1.ex.enc_ang - mot_ang_offset;
	mot_vel = *exec1.enc_ang_vel; //cpms
	
	update_diffarr(&mot_ang_clks, mot_ang, 10);
	
	mot_acc = get_accl_1k_5samples_downsampled(&mot_ang_clks)/2609; //rad/s^2
	rigid1.ex.mot_acc = mot_acc;
	
	#ifdef BOARD_SUBTYPE_POCKET
	//ToDo second channel
	pocket1.ex[0].mot_acc = rigid1.ex.mot_acc;
	pocket1.ex[1].mot_acc = rigid1.ex.mot_acc;
	#endif
}

#endif //BOARD_TYPE_FLEXSEA_EXECUTE
