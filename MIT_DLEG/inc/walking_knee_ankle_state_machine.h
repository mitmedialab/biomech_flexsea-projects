#ifdef __cplusplus
extern "C" {
#endif

#if (defined INCLUDE_UPROJ_MIT_DLEG && defined BOARD_TYPE_FLEXSEA_MANAGE) || defined BOARD_TYPE_FLEXSEA_PLAN

#ifndef BIOM_STATE_MACHINE
#define BIOM_STATE_MACHINE

//****************************************************************************
// Include(s)
//****************************************************************************
#include "main.h"
#include <math.h>
#include "flexsea_user_structs.h"
#include "flexsea_system.h"
#include "flexsea.h"
#include "state_variables.h"

//****************************************************************************
// Definition(s):
//****************************************************************************

#define LST_TO_ESW_DELAY              			 100  	//
#define ESW_TO_LSW_DELAY              			 100    // Transition time: 2->3 in ms (orig 200)
#define LSW_TO_EST_DELAY						 100	// transition to early stance

#define LSW_TO_EMG_DELAY						 1000	// transition into EMG freespace

//      THRESHOLD / LIMIT NAME                    VALUE          	UNITS           BRIEF DESCRIPTION                             			TRANSITION(S)
#define JNT_ORIENT								-JOINT_ANGLE_DIR	// 				JOINT_ANGLE_DIR is defined in user-mn-MIT-DLeg-2dof

#define K_VIRTUAL_HARDSTOP_NM_P_DEG				 7.13				// Nm/deg		Stiffness of virtual hardstop mimicking BiOM physical hardstop
#define HARD_HEELSTRIKE_TORQUE_THRESH           -5 					//  Nm       Foot-strike detector
#define GENTLE_HEELSTRIKE_TORQUE_THRESH         -3 				//  Nm       Foot-strike detector
#define HARD_HEELSTRIKE_TORQ_RATE_THRESH        -600 * JNT_ORIENT	// Nm/sec       Foot-strike detector                          			3->4
#define GENTLE_TOESTRIKE_TORQUE_THRESH			 3
#define HARD_TOESTRIKE_TORQUE_THRESH			 40.0				// [Nm] TODO: needs to be update for each user
#define HARD_TOESTRIKE_ANGLE_THRESH              140/K_VIRTUAL_HARDSTOP_NM_P_DEG * JNT_ORIENT // Degree          Toe-strike detector
#define HARD_TOESTRIKE_VEL_THRESH_DEG			 100.0				//[Deg/s] TODO: Find a decent value for this
#define	GENTLE_HEELSTRIKE_TORQ_RATE_THRESH       60     			// Nm/sec       Gentle foot-strike detector
#define ANKLE_UNLOADED_TORQUE_THRESH              3.0            	// Nm           Foot unloaded threshold                       			5->2
#define EARLYSTANCE_DECAY_CONSTANT                0.994987437  		//was 0.99           Decay constant for early stance
#define ANGLE_VIRTUAL_HARDSTOP_NM_P_DEG	0.0 	//Virtual hardstop engagement angle


//****************************************************************************
// Shared Variable(s):
//****************************************************************************
extern GainParams ankleGainsEsw;
extern GainParams ankleGainsLsw;
extern GainParams ankleGainsEst;
extern GainParams ankleGainsLst;

extern WalkParams *ankleWalkParams;
extern WalkingStateMachine kneeAnkleStateMachine;
extern Act_s act1, act2;

//extern int16_t experimentTask; // used to determine what state machine we're running.

//****************************************************************************
// Prototype(s):
//****************************************************************************

void setKneeAnkleFlatGroundFSM(Act_s *actx, WalkParams *ankleWalkParamx);
void setTorqueAnklePassive(Act_s *actx, WalkParams *wParams);
void setTorqueQuasiPassive(Act_s *act1, WalkParams *wParams);
void setSimpleAnkleFlatGroundFSM(Act_s *actx, WalkParams *ankleWalkParamx);
void setAnkleTorqueReplay(Act_s *actx, WalkParams *ankleWalkParamx);
void setAnkleNonLinearStiff(Act_s *actx, WalkParams *ankleWalkParamx);

//****************************************************************************
// Static Functions:
//****************************************************************************

void updateImpedanceParams(Act_s *actx, WalkParams *wParams);
void updateAnkleVirtualHardstopTorque(Act_s *actx, WalkParams *wParams);
void updateStiffnessRampDTheta(Act_s *actx, RampParam *rampParamx);
void updateStiffnessRampDt(RampParam *rampParamx);



#endif //BIOM_STATE_MACHINE
#endif //(INCLUDE_UPROJ_MIT_DLEG && BOARD_TYPE_FLEXSEA_MANAGE) || BOARD_TYPE_FLEXSEA_PLAN
#ifdef __cplusplus
}
#endif
