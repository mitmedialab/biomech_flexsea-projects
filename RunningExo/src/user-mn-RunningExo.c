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
int16_t fsm1State;								//state in fsm_1()
uint16_t controlAction;							//
#if (CONTROL_STRATEGY == TORQUE_TRACKING)
	int32_t heelStrikeTime;						//for keeping track of time
	int32_t toeOffTime;
	uint32_t stancePhaseDuration;
	int32_t count;
	int8_t	bodyWeight = 70;					//user body weight (kg)
#endif
//Comm & FSM2:


//****************************************************************************
// Trajectory lookup table(s)
//****************************************************************************
#if (CONTROL_STRATEGY == TORQUE_TRACKING)
static const float torqueProfile[TABLE_SIZE]=
{0,0.02549246813,0.1162200143,0.3286161584,0.6523014719,
0.9534528241,1.176622864,1.332684105,1.478365918,
1.624047731,1.747688666,1.794137225,1.774167304,
1.654372427,1.467271517,1.225413575,1.082591652,
0.9400655802,0.7647247356,0.600157787,0.4463647346,
0.2707280392,0.1725795715,0.07482557136,0.05436256503};
#endif	//(CONTROL_STRATEGY == TORQUE_TRACKING)

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************
void stateTransition(void);
void trajectoryTracking(void);
//****************************************************************************
// Public Function(s)
//****************************************************************************

//Prepare the system:
void init_runningExo(void)
//TODO
{
	fsm1State = 1;							//Initialize state
	heelStrikeTime = -1;					//clear timers
	toeOffTime = -1;						//clear timers
	controlAction = 0;						//clear control action
	stancePhaseDuration = 0;				//stance phase
	//TODO:
	count = 0;
}

//Running Exo state machine
void RunningExo_fsm_1(void)
//TODO
{
	#if (CONTROL_STRATEGY == TORQUE_TRACKING)
	//Torque Trajectory Tracking
	//Update states
	stateTransition();
	//Perform actions
	switch (fsm1State)
	{
		case 1:
			//Gait cycle is between heel strike and toe off
			trajectoryTracking();
			break;
		case 2:
			//Swing phase
			controlAction = 0;					//no force output
			break;
	}
	count++;

	//Output stuff for debugging
	rigid1.mn.genVar[0] = fsm1State;			//outputs state
	rigid1.mn.genVar[1] = controlAction;		//outputs control action
	rigid1.mn.genVar[2] = count;
	rigid1.mn.genVar[3] = stancePhaseDuration;
	#endif
}

//Second state machine for the Running Exo
void RunningExo_fsm_2(void)
//TODO

{

}

//****************************************************************************
// Private Function(s)
//****************************************************************************
void stateTransition(void)
//Computes the current state
{
	//TODO: Add debounce filtering
	if(fsm1State==2 && rigid1.mn.analog[FOOTSWITCH_HEEL]>FOOTSWITCH_THRESHOLD)
	{
		//Heel Strike
		fsm1State=1;
		//TODO:
		heelStrikeTime = count;
	}
	else if (fsm1State==1 && (rigid1.mn.analog[FOOTSWITCH_TOE]<FOOTSWITCH_THRESHOLD &&
			rigid1.mn.analog[FOOTSWITCH_LATERAL]<FOOTSWITCH_THRESHOLD &&
			rigid1.mn.analog[FOOTSWITCH_MEDIAL]<FOOTSWITCH_THRESHOLD)&&
			rigid1.mn.analog[FOOTSWITCH_HEEL]<FOOTSWITCH_THRESHOLD
			)
	{
		//Toe off
		fsm1State=2;
		//TODO:
		toeOffTime = count;
		stancePhaseDuration = toeOffTime-heelStrikeTime;
		//stancePhaseDuration=10000;
	}
}

void trajectoryTracking(void)
//sets control output based on trajectory tracking
{
	if (stancePhaseDuration==0)
		//No reference duration
		return;

	float percentStance = (float)(count-heelStrikeTime)/(float)stancePhaseDuration>1 ? 1:(float)(count-heelStrikeTime)/(float)stancePhaseDuration;//range=[0,1]
	//TODO: interpolation and perhaps other fancy stuff
	float torqueValue = torqueProfile[(int)(percentStance*(float)TABLE_SIZE)]*bodyWeight;

	//Debug
	rigid1.mn.genVar[5]=(int)(percentStance*(float)TABLE_SIZE);
	rigid1.mn.genVar[6]=torqueValue;

	//Set motor output
	//TODO: Unit conversion and casting
	controlAction = (int)torqueValue;
}

#endif	//(ACTIVE_PROJECT == PROJECT_ACTPACK)
#endif 	//BOARD_TYPE_FLEXSEA_MANAGE

#endif 	//INCLUDE_UPROJ_ACTPACK
