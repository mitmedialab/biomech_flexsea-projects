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


#if defined INCLUDE_UPROJ_RUNNINGEXO || defined BOARD_TYPE_FLEXSEA_PLAN
#if defined BOARD_TYPE_FLEXSEA_MANAGE || defined BOARD_TYPE_FLEXSEA_PLAN



//Un-comment the next line to enable manual control from the GUI:
//#define MANUAL_GUI_CONTROL

//****************************************************************************
// Include(s)
//****************************************************************************
#include "stdbool.h"
#include <stdio.h>
#include <math.h>
#include "user-mn.h"
#include "user-mn-ActPack.h"
#include "flexsea_sys_def.h"
#include "flexsea_user_structs.h"
//#include "flexsea_global_structs.h"
#include <flexsea_system.h>
#include "flexsea_cmd_calibration.h"
#include <flexsea_comm.h>
//#include "flexsea_board.h"
//#include "user-mn-Rigid.h"
//#include "cmd-ActPack.h"
//#include "cmd-Rigid.h"
#include "user-mn-RunningExo.h"

//----------------------------------------------------------------------------
// Variable(s)
//----------------------------------------------------------------------------
int16_t fsm1State;
uint16_t controlAction;							////state in fsm_1()

#if ((CONTROL_STRATEGY == RUN_TORQUE_TRACKING) || (CONTROL_STRATEGY == WALK_TORQUE_TRACKING))
	int32_t heelStrikeTime;						//for keeping track of time
	int32_t toeOffTime;
	uint32_t stancePhaseDuration;
	int32_t count;
 	int8_t	bodyWeight = BODY_WEIGHT;					//user body weight (kg)
#endif


//Comm & FSM2:


//Gait transition variables
typedef struct{
	bool    	heelStrikeFlag;
	uint32_t 	stanceTimer;
	uint32_t 	stancePeriod;
	bool    	stancePhaseFlag;
	bool    	toeOffFlag;
	uint32_t 	swingTimer;
	uint32_t 	swingPeriod;
	bool    	swingPhaseFlag;
	bool    	groundTouchFlag;
	bool    	footFlatFlag;
	float   	gaitCycle;
	bool    	normalWalkingFlag;
	uint32_t 	ForefootPressure_1;
	uint32_t 	ForefootPressure_2;
	int32_t		dForefootPressure;
	uint32_t 	RearfootPressure_1;
	uint32_t 	RearfootPressure_2;
	int32_t		dRearfootPressure;
	uint32_t	stepCounter;
	bool		newStepFlag;
	bool		postHeelStrikeFlag;
	bool		postToeOffFlag;

} walkingStateMachine;
walkingStateMachine stateMachine = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


//save actual command torque profile
float commandTorqueProfile[TABLE_SIZE];

//Power parameters
float currentTorqueValue;



//****************************************************************************
// Trajectory lookup table(s)
//****************************************************************************

/*Albert definition
#if (CONTROL_STRATEGY == RUN_TORQUE_TRACKING)
static const float torqueProfile[TABLE_SIZE]=
{0,0.02549246813,0.1162200143,0.3286161584,0.6523014719,
0.9534528241,1.176622864,1.332684105,1.478365918,
1.624047731,1.747688666,1.794137225,1.774167304,
1.654372427,1.467271517,1.225413575,1.082591652,
0.9400655802,0.7647247356,0.600157787,0.4463647346,
0.2707280392,0.1725795715,0.07482557136,0.05436256503};
#endif	//(CONTROL_STRATEGY == TORQUE_TRACKING)
*/

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************
void calCommandTorqueProfile(void);
void stateTransition(void);
// void trajectoryTracking(void);  //Albert function
void getCurrentTorqueCommand(void);
//****************************************************************************
// Public Function(s)
//****************************************************************************

/*Albert function
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
*/

//Prepare the system:
void init_runningExo(void)
//TODO
{
	fsm1State = 1;							//Initialize state
	controlAction = 0;						//clear control action

	//TODO:
	calCommandTorqueProfile();




}


/*Albert function
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
*/

//Running Exo state machine
void RunningExo_fsm_1(void)
//TODO
{
	#if ((CONTROL_STRATEGY == RUN_TORQUE_TRACKING) || (CONTROL_STRATEGY == WALK_TORQUE_TRACKING))
	//Torque Trajectory Tracking
	//Update states
	stateTransition();
	//Perform actions
/*
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
*/

	getCurrentTorqueCommand();

	controlAction = currentTorqueValue*100;

	//save current Footswitch data
	stateMachine.ForefootPressure_1 = stateMachine.ForefootPressure_2;
	stateMachine.RearfootPressure_1 = stateMachine.RearfootPressure_2;

	rigid1.mn.genVar[0] = stateMachine.ForefootPressure_1;			//outputs state
	rigid1.mn.genVar[1] = stateMachine.RearfootPressure_1;		//outputs control action
	rigid1.mn.genVar[2] = controlAction;
	rigid1.mn.genVar[3] = stateMachine.swingPhaseFlag;
	rigid1.mn.genVar[4] = stateMachine.gaitCycle*10000;
	rigid1.mn.genVar[5] = stateMachine.stancePeriod;
	rigid1.mn.genVar[6] = stateMachine.swingPeriod;
	rigid1.mn.genVar[7] = stateMachine.swingTimer;
	rigid1.mn.genVar[8] = stateMachine.stanceTimer;
	rigid1.mn.genVar[9] = stateMachine.stepCounter;

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

/*Albert function
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
*/

//calculate actual command torque profile
void calCommandTorqueProfile(void)
{
	int16_t i = 0;
	while (i < TABLE_SIZE)
	{
		commandTorqueProfile[i]= COMMAND_TORQUE_GAIN*BODY_WEIGHT*unitCommandTorqueProfile[i];
		i++;
	}

}



// new gait detection and transition function
void stateTransition(void)
//Computes the current state
{
	//TODO: Add debounce filtering

	//-------------------------------detect gait transition-----------------------------------
	//update current Footswitch data
	stateMachine.RearfootPressure_2 = rigid1.mn.analog[FOOTSWITCH_HEEL]; //tick
	stateMachine.ForefootPressure_2 = rigid1.mn.analog[FOOTSWITCH_MEDIAL];
	stateMachine.dRearfootPressure = stateMachine.RearfootPressure_2 - stateMachine.RearfootPressure_1;
	stateMachine.dForefootPressure = stateMachine.ForefootPressure_2 - stateMachine.ForefootPressure_1;

    //detect heel-strike moment: transition from a swing phase to a stance phase, i.e.,  swingPhaseFlag = 1 -> 0
	if (stateMachine.swingPhaseFlag == 1 && stateMachine.RearfootPressure_2 > HEEL_STRIKE_THRESHOLD && stateMachine.dRearfootPressure > dHEEL_STRIKE_THRESHOLD)
	{
		stateMachine.heelStrikeFlag = 1;
		stateMachine.toeOffFlag = 0;
		stateMachine.footFlatFlag = 0;
		stateMachine.stancePhaseFlag = 1;
		stateMachine.swingPhaseFlag = 0;
	    stateMachine.swingPeriod = stateMachine.swingTimer; //save the result of the counter in the previous swing phase
	    stateMachine.stanceTimer = 0;
	    stateMachine.swingTimer = 0; //reset the swing timer
	}

	//detect toe-off moment: transition from a stance phase to a swing phase, i.e., stancePhaseFlat(footNo) = 1 -> 0
	else if (stateMachine.stancePhaseFlag == 1  && stateMachine.ForefootPressure_2 < TOE_OFF_THRESHOLD && stateMachine.dForefootPressure < dTOE_OFF_THRESHOLD && stateMachine.footFlatFlag == 1)
	{
		stateMachine.toeOffFlag = 1;
		stateMachine.heelStrikeFlag = 0;
		stateMachine.footFlatFlag = 0;
		stateMachine.stancePhaseFlag = 0;
		stateMachine.swingPhaseFlag = 1;
		stateMachine.stancePeriod = stateMachine.stanceTimer; //save the result of the counter in the previous stance phase
		stateMachine.swingTimer = 0;
		stateMachine.stanceTimer = 0; //reset the stance timer

	}

	// detect swinging (can detect late or early swing?)
	else if (stateMachine.toeOffFlag == 1  && stateMachine.ForefootPressure_2 < TOE_OFF_THRESHOLD &&  stateMachine.RearfootPressure_2 < HEEL_STRIKE_THRESHOLD)
	{
		stateMachine.swingPhaseFlag = 1;
		stateMachine.stancePhaseFlag = 0;
		stateMachine.footFlatFlag = 0;
	}

	// detect stance - foot flat (can distinguish true foot-flat or false Push-off?)
	else if (stateMachine.ForefootPressure_2 > TOE_OFF_THRESHOLD && stateMachine.RearfootPressure_2 > HEEL_STRIKE_THRESHOLD)
	{
		stateMachine.stancePhaseFlag = 1;
		stateMachine.swingPhaseFlag = 0;
		stateMachine.footFlatFlag = 1;
	}
	//---------------------------- end of  gait transition detection-----------------------------------

	//-----------------------------------------stance phase--------------------------------------------
	if (stateMachine.stancePhaseFlag == 1)
	{
		//update the current state and run the counter
		stateMachine.groundTouchFlag = 1;
		stateMachine.stanceTimer = stateMachine.stanceTimer + 1;

		//check  whether the prior recorded stance period is reasonable,  swing and stance phases have to be shorter than 2 sec
		if (stateMachine.swingPeriod > GAIT_PERIOD_THRESHOLD_FLOOR && stateMachine.swingPeriod < GAIT_PERIOD_THRESHOLD_CEILING && stateMachine.stancePeriod > GAIT_PERIOD_THRESHOLD_FLOOR && stateMachine.stancePeriod < GAIT_PERIOD_THRESHOLD_CEILING)
		{
			stateMachine.normalWalkingFlag = 1;

			//find out the number of the walking steps when the controller fires
			if (stateMachine.heelStrikeFlag == 1)
			{
				stateMachine.stepCounter = stateMachine.stepCounter + 1;
				stateMachine.newStepFlag = 1;
				stateMachine.heelStrikeFlag = 0;
			}
			else
			{
				stateMachine.postHeelStrikeFlag = 1;
				stateMachine.postToeOffFlag = 0;
			}

			stateMachine.gaitCycle = (float)stateMachine.stanceTimer/(float)(stateMachine.stancePeriod + stateMachine.swingPeriod) > 1 ? 1:(float)stateMachine.stanceTimer/(float)(stateMachine.stancePeriod + stateMachine.swingPeriod);
		}   //if (stateMachine.swingPeriod > GAIT_PERIOD_THRESHOLD_FLOOR && stateMachine.swingPeriod < GAIT_PERIOD_THRESHOLD_CEILING && stateMachine.stancePeriod > GAIT_PERIOD_THRESHOLD_FLOOR && stateMachine.stancePeriod < GAIT_PERIOD_THRESHOLD_CEILING)

		else
		{
	        stateMachine.normalWalkingFlag = 0;
	        stateMachine.gaitCycle = 0;
	        currentTorqueValue = 0;
			stateMachine.stepCounter = 0;     // reset the step counter
		}

	} // if (stateMachine.stancePhaseFlag == 1)

	else if (stateMachine.swingPhaseFlag == 1)
	{
		//powerAssistProfile = 0;
		stateMachine.groundTouchFlag = 0;
		stateMachine.swingTimer = stateMachine.swingTimer + 1;

		//check whether the prior recorded swing period is reasonable
		if (stateMachine.stancePeriod > GAIT_PERIOD_THRESHOLD_FLOOR && stateMachine.stancePeriod < GAIT_PERIOD_THRESHOLD_CEILING && stateMachine.swingPeriod > SWING_PERIOD_THRESHOLD_FLOOR && stateMachine.stepCounter >0)
		{
			stateMachine.normalWalkingFlag = 1;

			if (stateMachine.toeOffFlag == 1)
			{
				stateMachine.newStepFlag = 0;

			}
			else
			{
				stateMachine.postToeOffFlag = 1;
				stateMachine.postHeelStrikeFlag = 0;
			}

			stateMachine.gaitCycle = (float)(stateMachine.swingTimer + stateMachine.stancePeriod)/(float)(stateMachine.stancePeriod + stateMachine.swingPeriod) > 1 ? 1:(float)(stateMachine.swingTimer + stateMachine.stancePeriod)/(float)(stateMachine.stancePeriod + stateMachine.swingPeriod);
		}

		else
		{
			stateMachine.normalWalkingFlag = 0;
			stateMachine.gaitCycle = 0;
		}

	}

	else
	{
		stateMachine.normalWalkingFlag = 0;
		stateMachine.gaitCycle = 0;
		stateMachine.stanceTimer = 0;
		stateMachine.swingTimer = 0;

	}






}

/* Albert function
void trajectoryTracking(void)
//sets control output based on trajectory tracking
{
	if (stancePhaseDuration==0)
		//No reference duration
		return;

	float percentStance = (float)(count-heelStrikeTime)/(float)stancePhaseDuration>1 ? 1:(float)(count-heelStrikeTime)/(float)stancePhaseDuration;//range=[0,1]
	//TODO: interpolation and perhaps other fancy stuff
	float torqueValue = commandTorqueProfile[(int)(percentStance*(float)TABLE_SIZE)]*bodyWeight*commandTorqueGain;

	//Debug
	rigid1.mn.genVar[5]=(int)(percentStance*(float)TABLE_SIZE);
	rigid1.mn.genVar[6]=torqueValue;

	//Set motor output
	//TODO: Unit conversion and casting
	controlAction = (int)torqueValue;
}
*/

void getCurrentTorqueCommand(void)
//get the specific torque value at the current point, current torque command
//used to set motor output after some transformations
{
	if ((stateMachine.stancePhaseFlag == 1) || (stateMachine.swingPhaseFlag == 1) )
	{
		//current ankle torque command in N.m
		//rigid1.mn.genVar[3] = commandTorqueProfile[(int)floor(stateMachine.gaitCycle*TABLE_SIZE)]*100;
		currentTorqueValue = stateMachine.normalWalkingFlag*commandTorqueProfile[(int)floor(stateMachine.gaitCycle*TABLE_SIZE)];
	}
	else
	{
		currentTorqueValue = 0;
	}

}



#endif 	//defined BOARD_TYPE_FLEXSEA_MANAGE || defined BOARD_TYPE_FLEXSEA_PLAN

#endif 	//defined INCLUDE_UPROJ_RUNNINGEXO || defined BOARD_TYPE_FLEXSEA_PLAN


