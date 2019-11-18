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

	[Lead developer] Jean-Francois Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors] Tony Shu, tony shu at mit dot edu, Matthew Carney mcarney at mit dot edu
*****************************************************************************
	[This file] user-mn-MIT_DLeg_2dof: User code running on Manage
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2018-02-24 | jfduval | New release
****************************************************************************/

#if defined INCLUDE_UPROJ_MIT_DLEG || defined BOARD_TYPE_FLEXSEA_PLAN
#if defined BOARD_TYPE_FLEXSEA_MANAGE || defined BOARD_TYPE_FLEXSEA_PLAN

//****************************************************************************
// Include(s)
//****************************************************************************

#include "user-mn-MIT-DLeg.h"
#include "actuator_functions.h"
#include "safety_functions.h"
#include "walking_knee_ankle_state_machine.h"
#include "run_main_user_application.h"	// This is where user application functions live
#include "ui.h"
#include "subject_walking_params.h"

#include "median_filter.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

uint8_t mitDlegInfo[2] = {PORT_RS485_1, PORT_RS485_1};
uint8_t enableMITfsm2 = 0, mitFSM2ready = 0, mitCalibrated = 0;

// Initiate some variables that may be used for testing
#if defined(IS_ACTUATOR_TESTING)
	float freqInput 	= 0.0;
	float freqRad 		= 0.0;
	float inputTheta 	= 0.0;
	float inputK 		= 0.0;
	float inputB 		= 0.0;
	float inputTorq 	= 0.0;
	int8_t currentOrVoltage = 0;
#elif defined(IS_SWEEP_TEST) || defined(IS_SWEEP_CHIRP_TEST)
	float freq									= 0.0;
	float amplitude								= 0.0;
	float dcBias								= 0.0;
	float noiseAmp								= 0.0;
	float freqFinal								= 0.0;
	int16_t chirpType							= 0;
	int16_t begin 								= 0;
	float freqSweepTime							= 0.0;
#endif

int8_t onEntry = 0;
Act_s act1, act2;
ActTestSettings act1TestInput;
TorqueRep torqueRep;

// EXTERNS
#if defined(IS_ANKLE)

	extern WalkParams subjectAnkleWalkParams;

#elif defined(IS_KNEE)
	extern WalkParams subjectKneeWalkParams;
#endif

int8_t isEnabledUpdateSensors = 0;

int16_t experimentTask = EXP_BARE_BONES;  // used to determine what state machine we're running.
int16_t userWriteMode = USER_INPUT_ANKLE_NOMINAL;

//static int gui_mode = GUI_MODE_NOM_CONTROL_PARAMS;
//static int gui_sub_mode = 0;
//static int gui_mode_prev = GUI_MODE_NOM_CONTROL_PARAMS;

//****************************************************************************
// Macro(s)
//****************************************************************************

#define FINDPOLES_DONE (calibrationFlags == 0) && (calibrationNew == 0)
//****************************************************************************
// Public Function(s)
//****************************************************************************

//MIT DLeg Finite State Machine.

/*
 *  Initialize the Finite State Machine
 */
void initMITDLeg(void)
{
}

/*
 *  Finite State Machine with multiple states.
 *  Call this function in one of the main while fsmTime slots.
 *  The possible states are as follows:
 *  	STATE_POWER_ON
 *  	STATE_INITIALIZE_SENSORS
 *  	STATE_FIND_POLES
 *  	STATE_INIT_USER_WRITES
 *  	STATE_MAIN
 *  	STATE_DEBUG
 */
void MITDLegFsm1(void)
{
	#if(ACTIVE_PROJECT == PROJECT_MIT_DLEG)

    static uint32_t fsmTime = 0;

    //Increment fsm_time (1 tick = 1ms nominally)
    fsmTime++;

    // Send genVars values to the GUI
    updateGenVarOutputs(&act1, ankleWalkParams, &act1TestInput);

    //Force a reset if E-stop was pushed while in a normal mode.
    if(act1.eStop)
    {
    	fsm1State = STATE_SAFETY;
    }

    //begin main FSM
	switch(fsm1State)
	{
		case STATE_SAFETY: //-1
		{ // Do nothing, until manually reset
			updateUserWrites(&act1, ankleWalkParams, &act1TestInput, &torqueRep);

			if (experimentTask == EXP_RESET_DEVICE )
			{
				if (act1.eStop == 0)
				{ // E-stop was just released, start over again. Only after manually changing experimentTask

					fsm1State = STATE_INITIALIZE_SENSORS;
					fsmTime = 0;
				}
			}

			break;
		}

		//this state is always reached
		case STATE_POWER_ON:
		{	//Same power-on delay as FSM2:
			if(fsmTime >= AP_FSM2_POWER_ON_DELAY)
			{
				//sensor update happens in mainFSM2(void) in main_fsm.c
				isEnabledUpdateSensors = 0;
				onEntry = 1;
				fsmTime = 0;
				fsm1State = STATE_INITIALIZE_SETTINGS;
			}

			break;
		}
		case STATE_INITIALIZE_SETTINGS:
		{
			if(fsmTime >= (AP_FSM2_POWER_ON_DELAY + 3000))
			{	// +3000 to allow regulate to read battery voltage correctly

				#if defined(NO_DEVICE)
					fsm1State = STATE_DEBUG;				// Skip over All states and sit in debug mode
				#elif defined(NO_ACTUATOR) || defined(NO_POWER)
					fsm1State = STATE_INITIALIZE_SENSORS;		// Still run controls, but skip Find Poles
					calibrationFlags = 0, calibrationNew = 0;
					isEnabledUpdateSensors = 1;
					onEntry = 1;
				#else
					fsm1State = STATE_FIND_POLES;
				#endif

				onEntry = 1;

				// If Master, enable Comm's, if Slave Do not need to turn it on.
				#if (ACTIVE_SUBPROJECT == SUBPROJECT_A)
					enableMITfsm2 = 1;	// Turn on communication to to SLAVE
				#else
					enableMITfsm2 = 0;	// If it's slave, then don't bother turning on comms?
				#endif
			}

			break;
		}
		case STATE_FIND_POLES:
		{
			if (rigid1.re.vb < UVLO_BIOMECH)
			{	// Skip FindPoles if no motor power attached.
				calibrationFlags = 0;
				calibrationNew = 0;
				fsm1State = STATE_INITIALIZE_SENSORS;
				fsmTime = 0;
				isEnabledUpdateSensors = 1;
				onEntry = 1;
			}
			else
			{
				if (!actuatorIsCorrect(&act1))
				{
					setLEDStatus(0,0,1); //flash red; needs reset
				}
				else
				{
					if (onEntry)
					{
						// USE these to to TURN OFF FIND POLES set these = 0 for OFF, or =1 for ON
						calibrationFlags = 1;
						calibrationNew = 1;

						isEnabledUpdateSensors = 0;
						onEntry = 0;
					}

					// Check if FindPoles has completed, if so then go ahead. This is done in calibration_tools.c
					if (FINDPOLES_DONE)
					{
						fsm1State = STATE_INITIALIZE_SENSORS;
						fsmTime = 0;
						isEnabledUpdateSensors = 1;
						onEntry = 1;
					}
				}
			}
			break;
		}
		case STATE_INITIALIZE_SENSORS:
		{
			zeroLoadCell = 1;	// forces getAxialForce() to zero the load cell again. this is kinda sketchy using a global variable.
			isEnabledUpdateSensors = 1;
			act1.resetStaticVariables = 1;	// Reset all variables (todo: make sure ALL static, persistent variables are reset)

			// GUI Modes
			experimentTask = 0;
			userWriteMode = 0;

			if (fsmTime > AP_FSM2_POWER_ON_DELAY)
			{
				fsm1State = STATE_INIT_USER_WRITES;
				fsmTime = 0;
				onEntry = 1;
				zeroLoadCell = 0;
				setMotorMode(MODE_ENABLED);
			}
			break;
		}
		case STATE_INIT_USER_WRITES:
		{
			/*reserve for additional initialization*/

			//Initialize Filters for Torque Sensing
			initSoftFIRFilt();	// Initialize software filter

			mitInitCurrentController(&act1);		//initialize Current Controller with gains

			//Set userwrites to initial values
			ankleWalkParams = &subjectAnkleWalkParams;
			ankleWalkParams->initializedStateMachineVariables = 0;
			if (!ankleWalkParams->initializedStateMachineVariables)
			{
				initializeUserWrites(&act1, ankleWalkParams, &act1TestInput, &torqueRep);
				ankleWalkParams->initializedStateMachineVariables = 1;
				kneeAnkleStateMachine.currentState = STATE_INIT;	//Establish walking state machine initialization state
			}

			fsm1State = STATE_MAIN;
			fsmTime = 0;
			onEntry = 1;

			act1.safetyTorqueScalar = 1.0;

			break;
		}
		case STATE_MAIN:
		{
			updateUserWrites(&act1, ankleWalkParams, &act1TestInput, &torqueRep);

			switch (experimentTask)
			{
				case EXP_RESET_DEVICE: // -99
				{
					fsm1State = STATE_INITIALIZE_SENSORS;
					break;
				}
				case EXP_BARE_BONES: //-5
				{
					break;
				}
				case EXP_ACTUATOR_STEP_RESPONSE: //-4
				{
					setActuatorStepResponse(&act1, &act1TestInput);
					setMotorTorque( &act1 );
					break;
				}
				default:
				{
					// do nothing
					act1.tauDes = 0.0;
				}
			}

			break;
		}
		case STATE_DEBUG:
		{
			break;
		}
		default:
		{	//Handle exceptions here
			break;
		}
	}

	act1.lastEStopCondition = act1.eStop;

	#endif	//ACTIVE_PROJECT == PROJECT_ANKLE_2DOF
}


/*
 *  Second state machine for the DLeg project
 *  Currently seems unused
 */
void MITDLegFsm2(void)
{
	//Verify we are Master and communicating to Slave and defined as Knee
	#if (ACTIVE_PROJECT == PROJECT_MIT_DLEG) && (ACTIVE_SUBPROJECT == SUBPROJECT_A) && defined(IS_KNEE)

	//Modified version of ActPack
	static uint32_t Fsm2Timer = 0;
	static uint32_t commDelayTimer = 0;

	//Wait X seconds before communicating
	if(Fsm2Timer < AP_FSM2_POWER_ON_DELAY)
	{
		mitFSM2ready = 0;
		Fsm2Timer++;
		return;
	}

	//External controller can fully disable the comm:
	//if(ActPackSys == SYS_NORMAL && ActPackCoFSM == APC_FSM2_ENABLED){enableAPfsm2 = 1;}
	//else {enableAPfsm2 = 0;}

	//FSM1 can disable this one:
	if(enableMITfsm2)
	{
		if ( fmod(commDelayTimer,5) == 0 )	// Include delay 5 samples, for comms to try to get cleaner RS485 updates from slave
		{

			writeEx[1].offset = 7;	// grab only this offset.
			tx_cmd_actpack_rw(TX_N_DEFAULT, writeEx[1].offset, writeEx[1].ctrl, writeEx[1].setpoint, \
											writeEx[1].setGains, writeEx[1].g[0], writeEx[1].g[1], \
											writeEx[1].g[2], writeEx[1].g[3], 0);	// todo: try this offset counter thing

			packAndSend(P_AND_S_DEFAULT, FLEXSEA_MANAGE_2, mitDlegInfo, SEND_TO_SLAVE);

			//Reset KEEP/CHANGE once set:
			if(writeEx[1].setGains == CHANGE){writeEx[1].setGains = KEEP;}
		}

		commDelayTimer++;
	}

	#endif	//ACTIVE_PROJECT == PROJECT_MIT_DLEG
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

/*
 * genVars are output to the GUI. These can be used to send out values, they are global variables but it's good to keep track
 * of them all in one place, hence his function. It's also annoying to flip up and down the code, so now they're located near
 * the userwrites as well (input from the GUI)
 * Note: to get device ID use //getDeviceIdIncrementing()
 */
void initializeUserWrites(Act_s *actx, WalkParams *wParams, ActTestSettings *act1TestSet, TorqueRep *torqueRep)
{
	user_data_1.w[0] = (int32_t) experimentTask;
	user_data_1.w[1] = (int32_t) userWriteMode; // Select what inputs you need

	switch(experimentTask)
	{
		default:
		{
			user_data_1.w[2] =  (int32_t) ( 0 );
			user_data_1.w[3] =  (int32_t) ( 0 );
			user_data_1.w[4] =  (int32_t) ( 0 );
			user_data_1.w[5] =  (int32_t) ( 0 );
			user_data_1.w[6] =  (int32_t) ( 0 );
			user_data_1.w[7] =  (int32_t) ( 0 );
			user_data_1.w[8] =  (int32_t) ( 0 );
			user_data_1.w[9] =  (int32_t) ( 0 );
			break;
		}
		case EXP_ACTUATOR_STEP_RESPONSE: // -4
		{
			user_data_1.w[2] =  (int32_t) ( 0 ) ;
			user_data_1.w[3] =  (int32_t) ( 0 *100.0);
			user_data_1.w[4] =  (int32_t) ( 0 *100.0);
			user_data_1.w[5] = 	(int32_t) ( actx->torqueKp * 1000.0);
			user_data_1.w[6] = 	(int32_t) ( actx->torqueKi * 1000.0);
			user_data_1.w[7] = 	(int32_t) ( actx->torqueKd * 1000.0);
			user_data_1.w[8] =  (int32_t) ( 0 *100.0);
			user_data_1.w[9] =  (int32_t) ( 0 *100.0);
			break;
		}
        case EXP_IS_SWEEP_CHIRP_TEST://-2
        {
            user_data_1.w[2] =  (int32_t) ( 0 ) ;
            user_data_1.w[3] =  (int32_t) ( 0 *100.0);
            user_data_1.w[4] =  (int32_t) ( 0 *100.0);
            user_data_1.w[5] =  (int32_t) ( 0 );
            user_data_1.w[6] =  (int32_t) ( 0 ) ;
            user_data_1.w[7] =  (int32_t) ( 0 *100.0);
            user_data_1.w[8] =  (int32_t) ( 0 *100.0);
            user_data_1.w[9] =  (int32_t) ( 0 *100.0);
            break;
        }

	}

	actx->initializedSettings = 1;
	wParams->initializedStateMachineVariables = 1;	// set flag that we initialized variables
}


void updateGenVarOutputs(Act_s *actx, WalkParams *wParams, ActTestSettings *act1TestSet)
{

	rigid1.mn.genVar[0] = (int16_t) (getSafetyFlags()); 			//errors

	rigid1.mn.genVar[1] = (int16_t) (act1.jointTorque	*100.	);			// Nm
	rigid1.mn.genVar[2] = (int16_t) (act1.tauDes 		*100.0	);			// radians/s
//	rigid1.mn.genVar[3] = (int16_t) (act1.jointAngle	*100.	);			//
//	rigid1.mn.genVar[4] = (int16_t) (act1.tauDes		*100.0	); 			//
//	rigid1.mn.genVar[5] = (int16_t) (*rigid1.ex.enc_ang - actx->motorPos0); //
//	rigid1.mn.genVar[6] = (int16_t) (act1.desiredCurrent);	 				//
//	rigid1.mn.genVar[7] = (int16_t) (getDeviceIdIncrementing()	); 			// Outputs Device ID, stepping through each number
//	rigid1.mn.genVar[8] = (int16_t) (kneeAnkleStateMachine.currentState); 	//
//	rigid1.mn.genVar[9] = (int16_t) (experimentTask) ;//(kneeAnkleStateMachine.currentState); //(act1.axialForce *10);

	switch (experimentTask)
	{
		default:
		{
//			rigid1.mn.genVar[1] = (int16_t) (act1.jointTorque	*100.0	);			// Nm
//			rigid1.mn.genVar[2] = (int16_t) (act1.jointVel		*100.0	);			// radians/s
//			rigid1.mn.genVar[3] = (int16_t) (act1.jointAngle	*100.0	);			//
//			rigid1.mn.genVar[4] = (int16_t) (act1.tauDes		*100.0	); 			//
//			rigid1.mn.genVar[5] = (int16_t) (rigid1.re.vb); //
//			rigid1.mn.genVar[6] = (int16_t) (act1.desiredCurrent);	 				//
//			rigid1.mn.genVar[7] = (int16_t) (getDeviceIdIncrementing()	); 			// Outputs Device ID, stepping through each number
//			rigid1.mn.genVar[8] = (int16_t) (rigid1.ex.strain); 	//
//			#ifdef IS_KNEE
//				  rigid1.mn.genVar[9] = (int16_t) (kneeAnkleStateMachine.slaveCurrentState); //(rigid2.ex.mot_volt); //rigid2.mn.genVar[7]; //(rigid1.re.vb);				// mV
//			#else
//				  rigid1.mn.genVar[9] = (int16_t) (experimentTask) ;//(kneeAnkleStateMachine.currentState); //(act1.axialForce *10);
//			#endif
			break;
		}
        case EXP_IS_SWEEP_CHIRP_TEST://-2
        {
//            rigid1.mn.genVar[1] = (int16_t) (act1.jointTorque		*100.0);
//            rigid1.mn.genVar[2] = (int16_t) (act1.jointVel			*100.0);
//            rigid1.mn.genVar[3] = (int16_t) (act1.jointAngleDegrees	*100.0);
//            rigid1.mn.genVar[4] = (int16_t) (act1.tauDes			*100.0);
//            rigid1.mn.genVar[5] = (int16_t) (act1.torqueKp			 * 1000.0	);
//            rigid1.mn.genVar[6] = (int16_t) (act1.torqueKi			 * 1000.0	);
//            rigid1.mn.genVar[7] = (int16_t) (act1.torqueKd			 * 1000.0	);
            rigid1.mn.genVar[8] = (int16_t) (act1.desiredVoltage / 2);
            rigid1.mn.genVar[9] = (int16_t) (act1.desiredCurrent / 2);
            break;
        }
		case EXP_ACTUATOR_STEP_RESPONSE: //-4	// Testing Actuator Control Parameters
		{
//			rigid1.mn.genVar[1] = (int16_t) (act1.jointTorque		 *100.0		);
//			rigid1.mn.genVar[2] = (int16_t) (act1.jointVel			 *100.0		);
//			rigid1.mn.genVar[3] = (int16_t) (act1.jointAngleDegrees	 *100.0		);
//			rigid1.mn.genVar[4] = (int16_t) (act1.tauDes			 *100.0		);
//			rigid1.mn.genVar[5] = (int16_t) (act1.torqueKp			 * 1000.0	);
//			rigid1.mn.genVar[6] = (int16_t) (act1.torqueKi			 * 1000.0	);
//			rigid1.mn.genVar[7] = (int16_t) (act1.torqueKd			 * 1000.0	);
//			rigid1.mn.genVar[8] = (int16_t) (rigid1.ex.strain / 2);
//
			rigid1.mn.genVar[9] = (int16_t) (act1.desiredCurrent / 2);
//			break;
		}
	}
}

/*UserWrites are inputs from Plan. They are initailized to teh values shown below.
 * Their values are then used by udpateUserWrites to set function values.
 * These can be updated as necessary.
 * Do keep care to initialize and upate correctly.
 * Also note, the initial values will not show up in Plan, that must be manually entered
 */

/*
 * Updates the Input values based off of user data
 * Param: actx(Act_s) - Actuator structure to track sensor values
 * Param: wParams(WalkParams) - walking parameters structure
 */
void updateUserWrites(Act_s *actx, WalkParams *wParams, ActTestSettings *act1TestSet, TorqueRep *torqueRep)
{
	static int16_t lastExperiment = 0;
	static int16_t lastUserWriteMode = 0;
	experimentTask 		= ( (int16_t) (user_data_1.w[0] ) );	// Select what experiment
	userWriteMode 		= ( (int16_t) (user_data_1.w[1] ) ); // Select what inputs you need

	// Check if experiment has changed, need to initialize values for that.
	if( (experimentTask != lastExperiment) || (userWriteMode != lastUserWriteMode) )
	{
		actx->initializedSettings = 0;
	}

	if (!actx->initializedSettings)
	{

		initializeUserWrites(actx, wParams, act1TestSet, torqueRep);

	}
	else
	{
		switch (experimentTask)
		{

			case EXP_IS_SWEEP_CHIRP_TEST://-2
			{
				act1TestSet->begin							= ( (int16_t) user_data_1.w[2] ) ;
				act1TestSet->freq							= ( (float) user_data_1.w[3] ) /100.0;
				act1TestSet->freqFinal						= ( (float) user_data_1.w[4] ) /100.0;
				act1TestSet->freqSweepTime					= ( (float) user_data_1.w[5] ) ; //milli seconds
				act1TestSet->chirpType						= ( (int16_t) user_data_1.w[6] ) ; // 0:def, 1:lin, 2:exp
				act1TestSet->amplitude						= ( (float) user_data_1.w[7] ) /100.0;
				act1TestSet->dcBias							= ( (float) user_data_1.w[8] ) /100.0;
				act1TestSet->noiseAmp						= ( (float) user_data_1.w[9] ) /100.0;
				break;
			}

			case EXP_ACTUATOR_STEP_RESPONSE: //-4
			{
				act1TestSet->begin							= ( (int16_t) user_data_1.w[2] ) ;
				act1TestSet->offTime						= ( (uint16_t) user_data_1.w[3] ) ;
				act1TestSet->onTime							= ( (uint16_t) user_data_1.w[4] ) ; //milli seconds
				actx->torqueKp								= ( (float) user_data_1.w[5] ) /1000.0;
				actx->torqueKi								= ( (float) user_data_1.w[6] ) /10000.0;
				actx->torqueKd								= ( (float) user_data_1.w[7] ) /10000.0;
				act1TestSet->amplitude						= ( (float) user_data_1.w[8] ) /100.0;
				act1TestSet->dcBias							= ( (float) user_data_1.w[9] ) /100.0;
				break;
			}
			default:
			{
				// do not update from userwrites
			}
		}
	} // end if(initialized)
	lastExperiment = experimentTask;
	lastUserWriteMode = userWriteMode;

}

/*
 * Initializes the Input values
 * Param: actx(Act_s) - Actuator structure to track sensor values
 * Param: wParams(WalkParams) -
 */
#endif 	//BOARD_TYPE_FLEXSEA_MANAGE || defined BOARD_TYPE_FLEXSEA_PLAN
#endif //INCLUDE_UPROJ_MIT_DLEG || defined BOARD_TYPE_FLEXSEA_PLAN
