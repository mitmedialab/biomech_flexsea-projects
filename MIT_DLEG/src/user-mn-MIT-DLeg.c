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
	[Lead developer] Luke Mooney, lmooney at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors] Tony Shu, tony shu at mit dot edu, Matthew Carney mcarney at mit dot edu
*****************************************************************************
	[This file] user-mn-MIT_DLeg_2dof: User code running on Manage
*****************************************************************************/

#if defined INCLUDE_UPROJ_MIT_DLEG || defined BOARD_TYPE_FLEXSEA_PLAN
#if defined BOARD_TYPE_FLEXSEA_MANAGE || defined BOARD_TYPE_FLEXSEA_PLAN

//****************************************************************************
// Include(s)
//****************************************************************************
#include <stdint.h>
#include "main.h"
#include "user-mn.h"
#include "user-mn-ActPack.h"
#include "flexsea_sys_def.h"
#include "flexsea_system.h"
#include "flexsea_cmd_calibration.h"
#include "flexsea_user_structs.h"
#include <math.h>
#include "timer.h"
#include "dio.h"
#include "calibration_tools.h"

#include <user-mn-MIT-DLeg.h>
#include "actuator_functions.h"
#include "walking_state_machine.h"	// Included to allow UserWrites to update walking machine controller.


//****************************************************************************
// Variable(s)
//****************************************************************************

float freq_input = 0;
float freq_rad = 0;
float torq_input = 0;

// EXTERNS
extern uint8_t calibrationFlags, calibrationNew;

//****************************************************************************
// Public Function(s)
//****************************************************************************

//MIT DLeg Finite State Machine.

void init_MIT_DLeg(void) {


}

//Call this function in one of the main while fsm_time slots.
void MIT_DLeg_fsm_1(void)
{
	#if(ACTIVE_PROJECT == PROJECT_MIT_DLEG)

    static uint32_t fsm_time = 0;
    static uint16_t mainFSMLoopTimer = 0;
    static uint16_t mainFSMLoopTimerPrev = 0;
    static uint16_t deltaTimer = 0;

    //Increment fsm_time (1 tick = 1ms nominally; need to confirm)
    fsm_time++;

    //begin main FSM
	switch(fsm1State)
	{
		case -2:
			stateMachine.current_state = STATE_IDLE;
			//Same power-on delay as FSM2:
			if(fsm_time >= AP_FSM2_POWER_ON_DELAY ) {
				fsm1State = -1;
				fsm_time = 0;

				// USE these to to TURN OFF FIND POLES set these = 0 for OFF, or =1 for ON
				calibrationFlags = 1, calibrationNew = 1;
			}

			break;

		case -1:
			stateMachine.current_state = STATE_INIT;

			// Check if FindPoles has completed, if so then go ahead. This is done in calibration_tools.c
			if ( (calibrationFlags == 0) && (calibrationNew == 0) ){
//				mit_init_current_controller();		//initialize Current Controller with gains
				fsm1State = 0;
				fsm_time = 0;

				setControlMode(CTRL_OPEN, 0);

			}


			break;

		case 0:
			//sensor update happens in mainFSM2(void) in main_fsm.c
			isEnabledUpdateSensors = 1;

			/*reserve for additional initialization*/

			//Set usewrites to initial values
			walkParams.initializedStateMachineVariables = 0;
			if (!walkParams.initializedStateMachineVariables){
				initializeUserWrites(&act1, &walkParams);

			}

			act1.safetyTorqueScalar = 1.0;

			fsm1State = 1;
			fsm_time = 0;

			break;

		case 1:
			{

				//populate rigid1.mn.genVars to send to Plan
//				packRigidVars(&act1);

				//begin safety check
			    if (safetyShutoff()) {
			    	/*motor behavior changes based on failure mode.
			    	  Bypasses the switch statement if return true
			    	  but sensors check still runs and has a chance
			    	  to allow code to move past this block.
			    	  Only update the walking FSM, but don't output torque.
			    	*/
			    	stateMachine.current_state = STATE_EARLY_STANCE;

			    } else {
			    	mainFSMLoopTimer = readTimer6();

			    	deltaTimer = mainFSMLoopTimer - mainFSMLoopTimerPrev;
			    	mainFSMLoopTimerPrev = mainFSMLoopTimer;

			        updateUserWrites(&act1, &walkParams);

//			    	runFlatGroundFSM(&act1);





//			    	act1.tauDes = biomCalcImpedance(user_data_1.w[0]/100., user_data_1.w[1]/100., user_data_1.w[2]/100.);
//			    	act1.tauDes = biomCalcImpedance(.5, .1, 0);
			    	freq_rad = ANG_UNIT * freq_input;

			    	act1.tauDes = torq_input * frequencySweep(freq_rad,  ( ( (float) fsm_time ) / SECONDS )  );


			    	// Check that torques are within specified safety range.
			    	if (act1.tauDes > act1.safetyTorqueScalar * ABS_TORQUE_LIMIT_INIT ) {
			    		act1.tauDes = act1.safetyTorqueScalar * ABS_TORQUE_LIMIT_INIT;
			    	} else if (act1.tauDes < -act1.safetyTorqueScalar * ABS_TORQUE_LIMIT_INIT ) {
			    		act1.tauDes = - act1.safetyTorqueScalar * ABS_TORQUE_LIMIT_INIT;
			    	}

//			    	setMotorTorqueOpenLoop(&act1, act1.tauDes);
			    	setMotorTorqueOpenLoopVolts(&act1, act1.tauDes);

//			    	setMotorTorque(&act1, act1.tauDes);

			    	/* Output variables live here. Use this as the main reference
			    	 * NOTE: the communication Offsets are defined in /Rigid/src/cmd-rigid.c
			    	 */
			        rigid1.mn.genVar[0] = (int16_t) (act1.linkageMomentArm *1000.0); //startedOverLimit;
					rigid1.mn.genVar[1] = (int16_t) (act1.jointAngleDegrees*100.0); //deg
					rigid1.mn.genVar[2] = (int16_t)  walkParams.transition_id;
 					rigid1.mn.genVar[3] = (int16_t) (act1.jointVel * 100.0); 	// rad/s
					rigid1.mn.genVar[4] = (int16_t) (act1.jointTorqueRate*100.0);
					rigid1.mn.genVar[5] = (int16_t) (act1.jointTorque*100.0); //Nm
					rigid1.mn.genVar[6] = (int16_t) rigid1.ex.mot_current; // LG
					rigid1.mn.genVar[7] = (int16_t) rigid1.ex.mot_volt;// ( ( fsm_time ) % SECONDS ) ; //rigid1.ex.mot_volt; // TA
					rigid1.mn.genVar[8] = (int16_t) (act1.safetyFlag) ; //stateMachine.current_state;
					rigid1.mn.genVar[9] = (int16_t) act1.tauDes*100;




			    }

				break;
			}

        	default:
			//Handle exceptions here
			break;
	}

	#endif	//ACTIVE_PROJECT == PROJECT_ANKLE_2DOF

}




//Second state machine for the DLeg project
void MIT_DLeg_fsm_2(void)
{
	#if(ACTIVE_PROJECT == PROJECT_MIT_DLEG)

		//Currently unused - we use ActPack's FSM2 for comm

	#endif	//ACTIVE_PROJECT == PROJECT_MIT_DLEG
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

/*UserWrites are inputs from Plan. They are initailized to teh values shown below.
 * Their values are then used by udpateUserWrites to set function values.
 * These can be updated as necessary.
 * Do keep care to initialize and upate correctly.
 * Also note, the initial values will not show up in Plan, that must be manually entered*/

void updateUserWrites(Act_s *actx, WalkParams *wParams){

	actx->safetyTorqueScalar 				= ( (float) user_data_1.w[0] ) /100.0;	// Reduce overall torque limit.
	freq_input				 				= ( (float) user_data_1.w[1] ) /100.0;	// Reduce overall torque limit.
	torq_input				 				= ( (float) user_data_1.w[2] ) /100.0;	// Reduce overall torque limit.
//	wParams->virtualHardstopEngagementAngle = ( (float) user_data_1.w[1] ) /100.0;	// [Deg]
//	wParams->virtualHardstopK 				= ( (float) user_data_1.w[2] ) /100.0;	// [Nm/deg]
//	wParams->lspEngagementTorque 			= ( (float) user_data_1.w[3] ) /100.0; 	// [Nm] Late stance power, torque threshhold
//	wParams->lstPGDelTics 					= ( (float) user_data_1.w[4] ); 		// ramping rate
//	lstPowerGains.k1						= ( (float) user_data_1.w[5] ) / 100.0;	// [Nm/deg]
//	lstPowerGains.thetaDes 					= ( (float) user_data_1.w[6] ) / 100.0;	// [Deg]
//	lstPowerGains.b		 					= ( (float) user_data_1.w[7] ) / 100.0;	// [Nm/s]
//	estGains.k1			 					= ( (float) user_data_1.w[8] ) / 100.0;	// [Nm/deg]
//	estGains.b			 					= ( (float) user_data_1.w[9] ) / 100.0;	// [Nm/s]


}

void initializeUserWrites(Act_s *actx, WalkParams *wParams){

//	wParams->earlyStanceK0 = 6.23;
//	wParams->earlyStanceKF = 0.1;
//	wParams->earlyStanceDecayConstant = EARLYSTANCE_DECAY_CONSTANT;

	actx->safetyTorqueScalar 				= 1.0; 	//user_data_1.w[0] = 100
	freq_input 								= 0.0;
	torq_input								= 0.0;
//	wParams->virtualHardstopEngagementAngle = 0.0;	//user_data_1.w[1] = 0	  [deg]
//	wParams->virtualHardstopK				= 3.5;	//user_data_1.w[2] = 350 [Nm/deg] NOTE: Everett liked this high, Others prefer more like 6.0
//	wParams->lspEngagementTorque 			= 74.0;	//user_data_1.w[3] = 7400 [Nm]
//	wParams->lstPGDelTics 					= 70.0;	//user_data_1.w[4] = 30
//	lstPowerGains.k1						= 4.0;	//user_data_1.w[5] = 400 [Nm/deg]
//	lstPowerGains.thetaDes 					= 18;	//user_data_1.w[6] = 1800 [Deg]
//	lstPowerGains.b		 					= 0.20;	//user_data_1.w[7] = 30   [Nm/s]
//	estGains.k1			 					= 1.50;	//user_data_1.w[8] = 150  [Nm/deg]
//	estGains.b			 					= 0.30;	//user_data_1.w[9] = 32  [Nm/s]

	//USER WRITE INITIALIZATION GOES HERE//////////////

	user_data_1.w[0] =  (int32_t) ( actx->safetyTorqueScalar*100 ); 	// torque scalar
	user_data_1.w[1] =  (int32_t) ( freq_input*100 ); 	// frequency set point for freq test
	user_data_1.w[2] =  (int32_t) ( torq_input*100 ); 	// torque input for freq test

	//	user_data_1.w[1] =  (int32_t) ( wParams->virtualHardstopEngagementAngle*100 ); 	// Hardstop Engagement angle
//	user_data_1.w[2] =  (int32_t) ( wParams->virtualHardstopK*100 ); 				// Hardstop spring constant
//	user_data_1.w[3] =  (int32_t) ( wParams->lspEngagementTorque*100 ); 			// Late stance power, torque threshhold
//	user_data_1.w[4] =  (int32_t) ( wParams->lstPGDelTics ); 		// ramping rate
//	user_data_1.w[5] =  (int32_t) ( lstPowerGains.k1 * 100 );		// 4.5 x 100
//	user_data_1.w[6] =  (int32_t) ( lstPowerGains.thetaDes * 100 ); // 14 x 100
//	user_data_1.w[7] =  (int32_t) ( lstPowerGains.b * 100 ); // 0.1 x 100
//	user_data_1.w[8] =  (int32_t) ( estGains.k1 * 100 ); // 0.1 x 100
//	user_data_1.w[9] =  (int32_t) ( estGains.b * 100 ); // 0.1 x 100

	///////////////////////////////////////////////////

	wParams->initializedStateMachineVariables = 1;	// set flag that we initialized variables
}








#endif 	//BOARD_TYPE_FLEXSEA_MANAGE || defined BOARD_TYPE_FLEXSEA_PLAN
#endif //INCLUDE_UPROJ_MIT_DLEG || defined BOARD_TYPE_FLEXSEA_PLAN
