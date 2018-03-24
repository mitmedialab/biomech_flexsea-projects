/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'user/ActPack' Dephy's Actuator Package (ActPack)
	Copyright (C) 2017 Dephy, Inc. <http://dephy.com/>
*****************************************************************************
	[Lead developper] Jean-Francois Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] user-mn-RunningExo: Running Exoskeleton Project
****************************************************************************/

#if defined INCLUDE_UPROJ_RUNNINGEXO || defined BOARD_TYPE_FLEXSEA_PLAN
#if defined BOARD_TYPE_FLEXSEA_MANAGE || defined BOARD_TYPE_FLEXSEA_PLAN

#ifndef INC_RUNNINGEXO_MN_H
#define INC_RUNNINGEXO_MN_H

//****************************************************************************
// Include(s)
//****************************************************************************
#include "main.h"

//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void init_runningExo(void);
void RunningExo_fsm_1(void);
void RunningExo_fsm_2(void);
void stateTransition(void);

//****************************************************************************
// Accessor(s)
//****************************************************************************


//****************************************************************************
// Constant Definition(s):
//****************************************************************************
#define CONTROL_STRATEGY 1
/* Controlling Strategy Options
 * 0: No action
 * 1: Torque Tracking
 *
 */
#define TORQUE_TRACKING 1

//Angle Limit
//TODO:
#define ENC_POS_MAX 16384
#define ENC_POS_MIN 0

//Velocity Limit
//TODO:
#define ENC_VEL_MAX 1000
#define ENC_VEL_MIN -1000

//Human parameters
#define LEFT_ANKLE 0
#define Right_ANKLE 1
#define DEFAULT_BODY_WEIGHT 75 // kg, subject's body weight
#define ACTIVE_LEG 0
#define DISABLED_STEPS 5

//Walking and Running modes
#define RUNNING_TORQUE_TRACKING
#define WALKING_TORQUE_TRACKING

//Foot switch
#define FOOTSWITCH_TOE 0                	// 0th analog channel
#define FOOTSWITCH_LATERAL 1            	// 1st analog channel
#define FOOTSWITCH_HEEL 2               	// 2nd analog channel
#define FOOTSWITCH_MEDIAL 3             	// 3rd analog channel

#if ACTIVE_LEG == LEFT_ANKLE

//#define FOOTSWITCH_THRESHOLD 500			//threshold for heel strike
#define FOOT_FLAT_THRESHOLD 500
#define TOE_OFF_THRESHOLD 30		    	//threshold for toe off detection
#define dTOE_OFF_THRESHOLD -30		    	//differential threshold for toe off detection
#define HEEL_STRIKE_THRESHOLD 1500			//threshold for heel strike detection
#define dHEEL_STRIKE_THRESHOLD 30	    	//differential threshold for heel strike detection

//Gait sanity checks
#define MIN_GAIT_PERIOD 200  	//msec, floor time for a normal gait cycle period
#define MAX_GAIT_PERIOD 2000  //msec, ceiling time for  a normal gait cycle period
#define MAX_STANCE_PERIOD 2000  	//msec, floor time for a normal swing period of a gait cycle

//#elif ACTIVE_LEG == RIGHT_ANKLE
//
//#define FOOTSWITCH_THRESHOLD 500			//tick, threshold for heel strike
//#define TOE_OFF_THRESHOLD 30		    	//tick, threshold for toe off detection
//#define dTOE_OFF_THRESHOLD -30		    	//tick, differential threshold for toe off detection
//#define HEEL_STRIKE_THRESHOLD 1500			//tick,threshold for heel strike detection
//#define dHEEL_STRIKE_THRESHOLD 30	    	//tick, differential threshold for heel strike detection
//#define MIN_GAIT_PERIOD 200  	//msec, floor time for a normal gait cycle period
//#define MAX_GAIT_PERIOD 2000  //msec, ceiling time for  a normal gait cycle period
//#define MIN_STANCE_PERIOD 30  	//msec, floor time for a normal swing period of a gait cycle

#endif //ACTIVE_LEG == LEFT_ANKLE

//Lookup Table
#if (CONTROL_STRATEGY == TORQUE_TRACKING)
	#define TABLE_SIZE 1001
	#define DEFAULT_TORQUE_PROFILE_GAIN 0.1    	//percentage of biological torque applied to the subject
	//State Definition
	#define DEACTIVATED 0
	#define HEEL_STRIKE 1						//between heel strike and foot flat
	#define FOOT_FLAT 2
	#define SWING_PHASE 3

#endif //(CONTROL_STRATEGY == TORQUE_TRACKING)

//****************************************************************************
// Structure(s)
//****************************************************************************


//****************************************************************************
// Shared variable(s)
//****************************************************************************


//****************************************************************************
// Torque command trajectory lookup table(s)
//****************************************************************************




#endif	//INC_RUNNINGEXO_MN_H

#endif //defined BOARD_TYPE_FLEXSEA_MANAGE || defined BOARD_TYPE_FLEXSEA_PLAN
#endif //defined INCLUDE_UPROJ_RUNNINGEXO || defined BOARD_TYPE_FLEXSEA_PLAN
