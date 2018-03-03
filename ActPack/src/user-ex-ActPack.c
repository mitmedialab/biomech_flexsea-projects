/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'user/ActPack' Dephy's Actuator Package (ActPack)
	Copyright (C) 2017 Dephy, Inc. <http://dephy.com/>
*****************************************************************************
	[Lead developper] Luke Mooney, lmooney at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] user-ex-ActPack: User code running on Ex
****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2017-09-27 | jfduval | Initial release
	*
****************************************************************************/

#ifdef INCLUDE_UPROJ_ACTPACK

#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

//****************************************************************************
// Include(s)
//****************************************************************************
 
#include "main.h"
#include "user-ex-ActPack.h"
#include "user-ex-rigid.h"
#include "flexsea_user_structs.h"
#include "flexsea_sys_def.h"
#include "filters.h"
#include "flexsea_board.h"
#include "control.h"
#include "mem_angle.h"
#include "motor.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

struct diffarr_s mot_ang_clks;

int32_t mot_ang = 0;
int32_t mot_ang_offset = 0; //initialized offset to match with calibration
int32_t mot_vel = 0;
int32_t mot_acc = 0;

int16_t ank_to_mot[320];

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************  


//****************************************************************************
// Public Function(s)
//****************************************************************************

//Call this function once in main.c, just before the while()
void initActPack(void)
{
	setBoardID(SLAVE_ID);
	
	//Controller setup:
	ctrl[0].active_ctrl = CTRL_NONE;		//No controller at boot
	setMotorVoltage(0,0);				//0% PWM
}

//Knee Finite State Machine.
//Call this function in one of the main while time slots.
void ActPack_fsm(void)
{
	ActPack_refresh_values();
}

//Here's an example function:
void ActPack_refresh_values(void)
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

//****************************************************************************
// Private Function(s)
//****************************************************************************

#endif //BOARD_TYPE_FLEXSEA_EXECUTE

#endif 	//INCLUDE_UPROJ_ACTPACK
