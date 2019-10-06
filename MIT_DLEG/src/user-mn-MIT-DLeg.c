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
//#include "walking_state_machine.h"	// Included to allow UserWrites to update walking machine controller.
#include "walking_knee_ankle_state_machine.h"
#include "run_main_user_application.h"	// This is where user application functions live
#include "ui.h"


//****************************************************************************
// Variable(s)
//****************************************************************************


uint8_t mitDlegInfo[2] = {PORT_RS485_1, PORT_RS485_1};
uint8_t enableMITfsm2 = 0, mitFSM2ready = 0, mitCalibrated = 0;
#define THIS_ACTPACK		0
#define SLAVE_ACTPACK		1

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
	extern GainParams ankleGainsEst;
	extern GainParams ankleGainsMst;
	extern GainParams ankleGainsLst;
	extern GainParams ankleGainsEsw;
	extern GainParams ankleGainsLsw;
#elif defined(IS_KNEE)
	extern GainParams kneeGainsEst;
	extern GainParams kneeGainsMst;
	extern GainParams kneeGainsLst;
	extern GainParams kneeGainsEsw;
	extern GainParams kneeGainsLsw;
#endif




extern uint8_t calibrationFlags, calibrationNew;
extern int32_t currentOpLimit;
extern int8_t zeroLoadCell; 		// used for zeroing the load cell.
extern int8_t isEnabledUpdateSensors;

extern float torqueKp;
extern float torqueKi;
extern float torqueKd;




extern int16_t splineTime;

int16_t experimentTask = EXP_ACTUATOR_TESTING;  // used to determine what state machine we're running.
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
void initMITDLeg(void) {


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
    static uint32_t controlTime = 0;
    static float tor = 0.0;

    //Increment fsm_time (1 tick = 1ms nominally)
    fsmTime++;

    // Send genVars values to the GUI
    updateGenVarOutputs(&act1);

    //begin main FSM
	switch(fsm1State)
	{
		//this state is always reached
		case STATE_POWER_ON:
			//Same power-on delay as FSM2:
			if(fsmTime >= AP_FSM2_POWER_ON_DELAY) {
				//sensor update happens in mainFSM2(void) in main_fsm.c
				isEnabledUpdateSensors = 0;
				onEntry = 1;
				fsmTime = 0;
				fsm1State = STATE_INITIALIZE_SETTINGS;
			}

			break;

		case STATE_INITIALIZE_SETTINGS:

			if(fsmTime >= AP_FSM2_POWER_ON_DELAY) {

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
				act1.currentOpLimit = CURRENT_LIMIT_INIT;
				onEntry = 1;

				// If Master, enable Comm's, if Slave Do not need to turn it on.
				#if (ACTIVE_SUBPROJECT == SUBPROJECT_A)
					enableMITfsm2 = 1;	// Turn on communication to to SLAVE
				#else
					enableMITfsm2 = 0;	// If it's slave, then don't bother turning on comms?
				#endif
			}

			break;

		case STATE_FIND_POLES:

			if (!actuatorIsCorrect(&act1)){
				setLEDStatus(0,0,1); //flash red; needs reset
			} else{
				if (onEntry) {
					// USE these to to TURN OFF FIND POLES set these = 0 for OFF, or =1 for ON
					calibrationFlags = 1, calibrationNew = 1;

					isEnabledUpdateSensors = 0;
					onEntry = 0;
				}

				// Check if FindPoles has completed, if so then go ahead. This is done in calibration_tools.c
				if (FINDPOLES_DONE){
					fsm1State = STATE_INITIALIZE_SENSORS;
					fsmTime = 0;
					isEnabledUpdateSensors = 1;
					onEntry = 1;
				}
			}

			break;

		case STATE_INITIALIZE_SENSORS:

			//todo check this is okay
			zeroLoadCell = 1;	// forces getAxialForce() to zero the load cell again. this is kinda sketchy using a global variable.
			isEnabledUpdateSensors = 1;

			if (fsmTime > AP_FSM2_POWER_ON_DELAY)
			{
				fsm1State = STATE_INIT_USER_WRITES;
				fsmTime = 0;
				onEntry = 1;
				zeroLoadCell = 0;
			}
			break;

		case STATE_INIT_USER_WRITES:

			/*reserve for additional initialization*/

			//Initialize Filters for Torque Sensing
			initSoftFIRFilt();	// Initialize software filter

//			mitInitOpenController(&act1);		//initialize Open Controller
			mitInitCurrentController(&act1);		//initialize Current Controller with gains


			//Set userwrites to initial values
			ankleWalkParams.initializedStateMachineVariables = 0;
			if (!ankleWalkParams.initializedStateMachineVariables){
				initializeUserWrites(&act1, &ankleWalkParams, &torqueRep);
				ankleWalkParams.initializedStateMachineVariables = 1;
				kneeAnkleStateMachine.currentState = STATE_INIT;	//Establish walking state machine initialization state
			}

			fsm1State = STATE_MAIN;
			fsmTime = 0;
			onEntry = 1;

			act1.safetyTorqueScalar = 0.1;

			break;

		case STATE_MAIN:
			{
				updateUserWrites(&act1, &ankleWalkParams, &act1TestInput, &torqueRep);


				//DEBUG removed this because joint encoder can't update in locked state.
//				if (getMotorMode() == MODE_ENABLED || getMotorMode() == MODE_OVERTEMP ){


//				if ( fmod(controlTime,9) == 0 )
//				{
					switch (experimentTask)
					{

						case EXP_ACTUATOR_TESTING://-3
						{// Testing Actuator Control Parameters
							float test = setActuatorTestingTorque(&act1, &act1TestInput);//getImpedanceTorque(&act1, act1TestInput.inputK, act1TestInput.inputB, act1TestInput.inputTheta);
							break;
						}

						case EXP_IS_SWEEP_CHIRP_TEST://-2
						{// System ID tests
							act1.tauDes = getTorqueSystemIDFrequencySweepChirp( &act1TestInput);
							break;
						}

						case EXP_ANKLE_PASSIVE:
						{// Simulate just a spring foot
							setTorqueAnklePassive(&act1, &ankleWalkParams);
							break;
						}
						case EXP_ANKLE_WALKING_FSM:
						{// Walking Controller
//							setKneeAnkleFlatGroundFSM(&act1);
							setSimpleAnkleFlatGroundFSM(&act1);
							break;
						}
						case EXP_ANKLE_WALKING_QUASIPASSIVE:
						{// Quasi-passive controller, two springs, one for controlled PF, & one for controlled DF
							setTorqueQuasiPassive(&act1, &ankleWalkParams);
							act1.tauDes = 0;
							break;
						}
						case EXP_ANKLE_WALKING_BIOM_FSM:
						{// Biom controller, limited Dorsiflexion
							act1.tauDes = 0;
							break;
						}
						/****************************************
						 *  Below here is where user code goes
						 ****************************************/
						case EXP_USER_CODE:
						{
							runMainUserApplication(&act1);
							act1.tauDes = 0.0;
							break;
						}
						/****************************************
						 *  Above here is where user code goes
						 ****************************************/
						default:
						{
							// do not update from userwrites
							act1.tauDes = 0.0;
						}
					}
//					controlTime = 0;	// reset the controlTime timer
//				} //end if control time


				// Do not command power if we're running a no-power mode.
				// Manually turn on OpenLoop control if necessary
//				#if !defined(NO_POWER)
					setMotorTorque( &act1, act1.tauDes);
//					setMotorTorqueOpenLoop( &act1, act1.tauDes, 1);
//				#endif

				controlTime++;

				break;
			}
		case STATE_DEBUG:

			break;

		default:
			//Handle exceptions here
			break;
	}

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
void updateGenVarOutputs(Act_s *actx)
{
	rigid1.mn.genVar[0] = (int16_t) (getSafetyFlags()); 			//errors

	switch (experimentTask)
	{
		case EXP_ACTUATOR_TESTING:	// Testing Actuator Control Parameters
		{
			rigid1.mn.genVar[1] = (int16_t) (act1.jointTorque*100.);
			rigid1.mn.genVar[2] = (int16_t) (act1.jointVel*10000.);
			rigid1.mn.genVar[3] = (int16_t) (act1.jointAngle*100.);
			rigid1.mn.genVar[4] = (int16_t) (act1.tauDes*100.0);

			switch(userWriteMode)
			{
				case EXP_ACT_CONTROL_PARAM_MAIN:
				{
					rigid1.mn.genVar[5] = (int16_t) (act1.torqueKp*1000.0);
					rigid1.mn.genVar[6] = (int16_t) (act1.torqueKi*1000.0);
					rigid1.mn.genVar[7] = (int16_t) (act1.torqueKd*1000.0);
					rigid1.mn.genVar[8] = (int16_t) (act1.desiredCurrent);
					rigid1.mn.genVar[9] = (int16_t) (act1.desiredVoltage);
					break;
				}
				case EXP_ACT_CONTROL_PARAM_SECOND:
				{
					rigid1.mn.genVar[6] = (int16_t) (act1.motorAcc);
					rigid1.mn.genVar[7] = (int16_t) (act1.motCurrDt);
					rigid1.mn.genVar[8] = (int16_t) (act1.desiredCurrent);
					rigid1.mn.genVar[9] = (int16_t) (act1.desiredVoltage);
					break;
				}
				default:
					break;
			}
			break;
		}
		case EXP_ANKLE_PASSIVE:	// Testing Actuator Control Parameters
		{
			rigid1.mn.genVar[1] = (int16_t) (act1.jointTorque*100.);
			rigid1.mn.genVar[2] = (int16_t) (act1.jointVel*10000.);
			rigid1.mn.genVar[3] = (int16_t) (act1.jointAngle*100.);
			rigid1.mn.genVar[4] = (int16_t) (act1.tauDes*100.0);
			rigid1.mn.genVar[5] = (int16_t) (kneeAnkleStateMachine.currentState);
			rigid1.mn.genVar[6] = (int16_t) (act1.torqueKi*1000.0);
			rigid1.mn.genVar[7] = (int16_t) (act1.torqueKd*1000.0);
			rigid1.mn.genVar[8] = (int16_t) (act1.desiredCurrent);
			rigid1.mn.genVar[9] = (int16_t) (act1.desiredVoltage);
			break;
		}
		case EXP_ANKLE_WALKING_FSM:
			rigid1.mn.genVar[1] = (int16_t) (act1.jointTorque*100.);
			rigid1.mn.genVar[2] = (int16_t) (act1.jointVel*10000.);
			rigid1.mn.genVar[3] = (int16_t) (act1.jointAngle*100.);
			rigid1.mn.genVar[4] = (int16_t) (act1.tauDes*100.0);
			rigid1.mn.genVar[5] = (int16_t) (0);
			rigid1.mn.genVar[6] = (int16_t) (0);
			rigid1.mn.genVar[7] = (int16_t) (experimentTask);
			rigid1.mn.genVar[8] = (int16_t) (userWriteMode);
			rigid1.mn.genVar[9] = (int16_t) (kneeAnkleStateMachine.currentState);
			break;
		default:
		{

			rigid1.mn.genVar[1] = (int16_t) (act1.jointTorque*100.);		// Nm
			rigid1.mn.genVar[2] = (int16_t) (act1.jointVel*10000.);			// radians/s
			rigid1.mn.genVar[3] = (int16_t) (act1.jointAngle*100.);	// (act1.jointAngleDegrees*1000.);	// deg
			rigid1.mn.genVar[4] = (int16_t) (act1.tauDes*100.0); //(act2.jointTorque*100.);  // (*rigid1.ex.enc_ang_vel);		// comes in as rad/s, //(act2.jointTorque*100.);
			rigid1.mn.genVar[5] = (int16_t) (*rigid1.ex.enc_ang - actx->motorPos0); // //(rigid1.ex.strain);
			rigid1.mn.genVar[6] = (int16_t) (act1.desiredCurrent); //(rigid1.ex.mot_volt);	// mA
			rigid1.mn.genVar[7] = (int16_t) (act1.desiredVoltage); //(*rigid1.ex.enc_ang);		// mV, //getDeviceIdIncrementing() ;
			rigid1.mn.genVar[8] = (int16_t) (kneeAnkleStateMachine.currentState); //(rigid2.ex.mot_current);			// mA
			#ifdef IS_KNEE
				  rigid1.mn.genVar[9] = (int16_t) (kneeAnkleStateMachine.slaveCurrentState); //(rigid2.ex.mot_volt); //rigid2.mn.genVar[7]; //(rigid1.re.vb);				// mV
			#else
				  rigid1.mn.genVar[9] = (int16_t) (experimentTask) ;//(kneeAnkleStateMachine.currentState); //(act1.axialForce *10);
			#endif
		    break;
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
	experimentTask 		= ( (int16_t) (user_data_1.w[0] ) );	// Select what experiment
	userWriteMode 		= ( (int16_t) (user_data_1.w[1] ) ); // Select what inputs you need

	// Check if experiment has changed, need to initialize values for that.
	if(experimentTask != lastExperiment)
	{
		actx->initializedSettings = 0;
	}
	if (!actx->initializedSettings)
	{
		initializeUserWrites(actx, wParams, torqueRep);
	}
	else
	{

		switch (experimentTask) {
			case EXP_ANKLE_WALKING_FSM:	// 2
			{
				#ifdef IS_ANKLE
					switch (userWriteMode)
					{
						case USER_INPUT_ANKLE_ORIGINAL:	//1
						{
//	//						ankleGainsLsw.thetaDes					= ( (float) user_data_1.w[0] ) /100.0;	// [milliseconds]
//	//						wParams->virtualHardstopEngagementAngle = ( (float) user_data_1.w[1] ) /100.0;	// [Deg]
//							wParams->virtualHardstopK 				= ( (float) user_data_1.w[2] ) /100.0;	// [Nm/deg]
//							wParams->lspEngagementTorque 			= ( (float) user_data_1.w[3] ) /100.0; 	// [Nm] Late stance power, torque threshhold
//							wParams->lstPGDelTics 					= ( (float) user_data_1.w[4] ); 		// ramping rate
//							ankleGainsEst.k1						= ( (float) user_data_1.w[5] ) / 100.0;	// [Nm/deg]
//							ankleGainsEst.b		 					= ( (float) user_data_1.w[6] ) / 100.0;	// [Deg]
//							ankleGainsLst.thetaDes 					= ( (float) user_data_1.w[7] ) / 100.0;	// [Nm/s]
//							ankleGainsEsw.k1			 			= ( (float) user_data_1.w[8] ) / 100.0;	// [Nm/deg]
//							ankleGainsEsw.thetaDes					= ( (float) user_data_1.w[9] ) / 100.0;	// [Deg]
//
							wParams->virtualHardstopK 		= ( (float) user_data_1.w[2] ) / 100.0 ;
							wParams->lspEngagementTorque 	= ( (float) user_data_1.w[3] ) / 100.0 ;
							wParams->lstPGDelTics  			= ( (float) user_data_1.w[4]) /100.0;
							wParams->ankleGainsLst.thetaDes = ( (float) user_data_1.w[5]) /100.0;
							wParams->ankleGainsLst.b 		= ( (float) user_data_1.w[6]) /100.0;
							wParams->ankleGainsLsw.k1 		= ( (float) user_data_1.w[7]) /100.0;
							wParams->ankleGainsLsw.b 		= ( (float) user_data_1.w[8]) /100.0;
							wParams->ankleGainsEst.k1 		= ( (float) user_data_1.w[9]) /100.0;

							break;
						}
						case USER_INPUT_ANKLE_IMPEDANCE: //2
						{// NOT SET UP YET update impedance parameters
							//						ankleGainsLsw.thetaDes					= ( (float) user_data_1.w[0] ) /100.0;	// [milliseconds]
							wParams->virtualHardstopEngagementAngle = ( (float) user_data_1.w[2] ) /100.0;	// [Deg]
							wParams->virtualHardstopK 				= ( (float) user_data_1.w[3] ) /100.0;	// [Nm/deg]
							wParams->lspEngagementTorque 			= ( (float) user_data_1.w[4] ) /100.0; 	// [Nm] Late stance power, torque threshhold
							wParams->lstPGDelTics 					= ( (float) user_data_1.w[5] ); 		// ramping rate
							wParams->ankleGainsEst.k1				= ( (float) user_data_1.w[6] ) / 100.0;	// [Nm/deg]
							wParams->ankleGainsEst.b		 		= ( (float) user_data_1.w[7] ) / 100.0;	// [Deg]
							wParams->ankleGainsLst.thetaDes 		= ( (float) user_data_1.w[8] ) / 100.0;	// [Nm/s]
							wParams->ankleGainsEsw.k1			 	= ( (float) user_data_1.w[9] ) / 100.0;	// [Nm/deg]

							break;
						}
						case USER_INPUT_ANKLE_STANCE: //3
						{// NOT SET UP YET update impedance parameters
							//						ankleGainsLsw.thetaDes					= ( (float) user_data_1.w[0] ) /100.0;	// [milliseconds]
							wParams->ankleGainsEst.k1				= ( (float) user_data_1.w[2] ) / 100.0;	// [Nm/deg]
							wParams->ankleGainsEst.b		 		= ( (float) user_data_1.w[3] ) / 100.0;	// [Deg]
							wParams->ankleGainsEst.thetaDes	 		= ( (float) user_data_1.w[4] ) / 100.0;	// [Deg]
							wParams->ankleGainsMst.k1				= ( (float) user_data_1.w[5] ) / 100.0;	// [Nm/deg]
							wParams->ankleGainsMst.b		 		= ( (float) user_data_1.w[6] ) / 100.0;	// [Deg]
							wParams->ankleGainsMst.thetaDes		 	= ( (float) user_data_1.w[7] ) / 100.0;	// [Deg]
							wParams->ankleGainsLst.k1				= ( (float) user_data_1.w[8] ) / 100.0;	// [Nm/deg]
							wParams->ankleGainsLst.b		 		= ( (float) user_data_1.w[9] ) / 100.0;	// [Deg]
							break;
						}

						case USER_INPUT_ANKLE_SWING: //4
						{// NOT SET UP YET update impedance parameters
							//						ankleGainsLsw.thetaDes					= ( (float) user_data_1.w[0] ) /100.0;	// [milliseconds]
							wParams->ankleGainsEsw.k1				= ( (float) user_data_1.w[2] ) / 100.0;	// [Nm/deg]
							wParams->ankleGainsEsw.b		 		= ( (float) user_data_1.w[3] ) / 100.0;	// [Deg]
							wParams->ankleGainsEsw.thetaDes	 		= ( (float) user_data_1.w[4] ) / 100.0;	// [Deg]
							wParams->ankleGainsLsw.k1				= ( (float) user_data_1.w[5] ) / 100.0;	// [Nm/deg]
							wParams->ankleGainsLsw.b		 		= ( (float) user_data_1.w[6] ) / 100.0;	// [Deg]
							wParams->ankleGainsLsw.thetaDes	 		= ( (float) user_data_1.w[7] ) / 100.0;	// [Deg]
							break;
						}

						default:
						{
							break;
						}

					}

				#elif defined(IS_KNEE)

			//		jointLimitK			 				= ( (float) user_data_1.w[0] ) / 100.0;	// [Nm/deg]
			//		jointLimitB			 				= ( (float) user_data_1.w[1] ) / 100.0;	// [Nm/s]

	 //				kneeGainsEst.k1			 				= ( (float) user_data_1.w[0] ) / 100.0;	// [Nm/deg]
	 //				kneeGainsEst.thetaDes			 		= ( (float) user_data_1.w[1] ) / 100.0;	// [Nm/deg]

					kneeGainsLst.k1							= ( (float) user_data_1.w[2] ) / 100.0;	// [Nm/deg]
					kneeGainsLst.thetaDes 					= ( (float) user_data_1.w[3] ) / 100.0;	// [Nm/s]

					kneeGainsEsw.k1							= ( (float) user_data_1.w[4] ) / 100.0;	// [Nm/deg]
					kneeGainsEsw.thetaDes 					= ( (float) user_data_1.w[5] ) / 100.0;	// [Nm/s]

					kneeGainsLsw.k1							= ( (float) user_data_1.w[6] ) / 100.0;	// [Nm/deg]
					kneeGainsLsw.thetaDes 					= ( (float) user_data_1.w[7] ) / 100.0;	// [Nm/s]



					//user_data_1.w[9] in use!

					// These are generally redundant
			//		kneeGainsMst.k1 = kneeGainsEst.k1;
			//		kneeGainsMst.b 	= kneeGainsEst.b;
			//		kneeGainsMst.thetaDes = kneeGainsEst.thetaDes;
			//
			//		kneeGainsLsw.k1	= kneeGainsEsw.k1;
			//		kneeGainsLsw.b	= kneeGainsEsw.b;
			//		kneeGainsLsw.thetaDes	= kneeGainsEsw.thetaDes;
				#endif
				break;
			}

			case EXP_ANKLE_PASSIVE: //1
			{
				wParams->virtualHardstopEngagementAngle	= ( (float) user_data_1.w[2] ) /100.0;
				wParams->virtualHardstopK				= ( (float) user_data_1.w[3] ) /100.0;
				wParams->virtualHardstopB				= ( (float) user_data_1.w[4] ) /100.0;
				break;
			}
			case EXP_ANKLE_WALKING_TORQUE_REPLAY: //4
			{// Testing Actuator Control Parameters
				torqueRep->begin			= ( (int8_t) user_data_1.w[2] );
				torqueRep->torqueScalingFactor = ( (float) (user_data_1.w[3] / 100 ) );
				break;
			}
			case EXP_ACTUATOR_TESTING: //-3
			{// Testing Actuator Control Parameters
				switch (userWriteMode)
				{
					case EXP_ACT_CONTROL_PARAM_MAIN:
					{
						act1TestSet->inputTheta				= ( (float) user_data_1.w[2] ) /100.0;
						act1TestSet->inputK					= ( (float) user_data_1.w[3] ) /100.0;
						act1TestSet->inputB					= ( (float) user_data_1.w[4] ) /100.0;
						actx->torqueKp						= ( (float) user_data_1.w[5] ) /1000.0;
						actx->torqueKi						= ( (float) user_data_1.w[6] ) /1000.0;
						actx->torqueKd						= ( (float) user_data_1.w[7] ) /1000.0;
						act1TestSet->inputTorq				= ( (float) user_data_1.w[8] ) /100.0;
						actx->controlFF						= ( (float) user_data_1.w[9] ) /100.0;
						break;
					}
					case EXP_ACT_CONTROL_PARAM_SECOND:
					{
						act1TestSet->inputTheta				= ( (float) user_data_1.w[2] ) /100.0;
						act1TestSet->inputK					= ( (float) user_data_1.w[3] ) /100.0;
						act1TestSet->inputB					= ( (float) user_data_1.w[4] ) /100.0;
						actx->torqueKp						= ( (float) user_data_1.w[5] ) /1000.0;
						actx->torqueKi						= ( (float) user_data_1.w[6] ) /1000.0;
						actx->torqueKd						= ( (float) user_data_1.w[7] ) /1000.0;
						actx->controlFF						= ( (float) user_data_1.w[8] ) /100.0;
						actx->controlScaler					= ( (float) user_data_1.w[9] ) /100.0;
						break;
					}
					default:
					{
						break;
					}
				}
				break;
			}

			case EXP_IS_SWEEP_CHIRP_TEST://-2
			{
				act1TestSet->begin									= ( (int16_t) user_data_1.w[2] ) ;
				act1TestSet->freq									= ( (float) user_data_1.w[3] ) /100.0;
				act1TestSet->freqFinal								= ( (float) user_data_1.w[4] ) /100.0;
				act1TestSet->freqSweepTime							= ( (float) user_data_1.w[5] ) ; //milli seconds
				act1TestSet->chirpType								= ( (int16_t) user_data_1.w[6] ) ; // 0:def, 1:lin, 2:exp
				act1TestSet->amplitude								= ( (float) user_data_1.w[7] ) /100.0;
				act1TestSet->dcBias									= ( (float) user_data_1.w[8] ) /100.0;
				act1TestSet->noiseAmp								= ( (float) user_data_1.w[9] ) /100.0;
				break;
			}
			default:
			{
				// do not update from userwrites
			}
		}
	} // end if(initialized)
	lastExperiment = experimentTask;

}

/*
 * Initializes the Input values
 * Param: actx(Act_s) - Actuator structure to track sensor values
 * Param: wParams(WalkParams) -
 */
void initializeUserWrites(Act_s *actx, WalkParams *wParams, TorqueRep *torqueRep)
{

//	experimentTask 			= -3;	// Select what experiment
	user_data_1.w[0] = (int32_t) experimentTask;
	user_data_1.w[1] = (int32_t) userWriteMode; // Select what inputs you need


	user_data_1.w[2] = (int32_t)(0);
	user_data_1.w[3] = (int32_t)(0);
	user_data_1.w[4] = (int32_t)(0);
	user_data_1.w[5] = (int32_t)(0);
	user_data_1.w[6] = (int32_t)(0);
	user_data_1.w[7] = (int32_t)(0);
	user_data_1.w[8] = (int32_t)(0);
	user_data_1.w[9] = (int32_t)(0);


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
		case EXP_ANKLE_WALKING_FSM: //2
		{
			wParams->earlyStanceK0 = 6.23;
			wParams->earlyStanceKF = 0.1;
			wParams->earlyStanceDecayConstant = EARLYSTANCE_DECAY_CONSTANT;

			wParams->virtualHardstopEngagementAngle = 0.0;	//user_data_1.w[1] = 0	  [deg]
			wParams->virtualHardstopK				= 4.5;	//user_data_1.w[2] = 350 [Nm/deg] NOTE: Everett liked this high, Others prefer more like 6.0
			wParams->lspEngagementTorque 			= 50.0;	//user_data_1.w[3] = 7400 [Nm]	// What triggers pushoff
			wParams->lstPGDelTics 					= 70.0;	//user_data_1.w[4] = 30			// Delay to ramp up pushoff power

			wParams->ankleGainsEst.k1 = 0.0;
			wParams->ankleGainsEst.b = 0.18;
			wParams->ankleGainsEst.thetaDes = 0.0;

			wParams->ankleGainsMst.k1 = 2.5;
			wParams->ankleGainsMst.b = 0.18;
			wParams->ankleGainsMst.thetaDes = 0.0;

			wParams->ankleGainsLst.k1 = 4.5;
			wParams->ankleGainsLst.b = 0.12;
			wParams->ankleGainsLst.thetaDes = 14.0;

			wParams->ankleGainsEsw.k1 = 1.5;
			wParams->ankleGainsEsw.b = 0.18;
			wParams->ankleGainsEsw.thetaDes = -8.0;

			wParams->ankleGainsLsw.k1 = 1.5;
			wParams->ankleGainsLsw.b = 0.18;
			wParams->ankleGainsLsw.thetaDes = -8.0;

			user_data_1.w[2] =  (int32_t) ( wParams->virtualHardstopK  *100.0 );
			user_data_1.w[3] =  (int32_t) ( wParams->lspEngagementTorque  *100.0 );
			user_data_1.w[4] =  (int32_t) ( wParams->lstPGDelTics *100.0 );
			user_data_1.w[5] =  (int32_t) ( wParams->ankleGainsLst.thetaDes *100.0);
			user_data_1.w[6] =  (int32_t) ( wParams->ankleGainsLst.b *100.0 );
			user_data_1.w[7] =  (int32_t) ( wParams->ankleGainsLsw.k1 *100.0 );
			user_data_1.w[8] =  (int32_t) ( wParams->ankleGainsLsw.b *100.0 );
			user_data_1.w[9] =  (int32_t) ( wParams->ankleGainsEst.k1 *100.0 );
			break;
		}
		case EXP_ANKLE_PASSIVE: //1
		{
			wParams->virtualHardstopEngagementAngle = 0.0;
			wParams->virtualHardstopK = 6.5;
			wParams->virtualHardstopB = 0.15;

			user_data_1.w[2] =  (int32_t) ( wParams->virtualHardstopEngagementAngle *100.0 );
			user_data_1.w[3] =  (int32_t) ( wParams->virtualHardstopK *100.0 );
			user_data_1.w[4] =  (int32_t) ( wParams->virtualHardstopB *100.0 );
			user_data_1.w[5] =  (int32_t) ( 0 );
			user_data_1.w[6] =  (int32_t) ( 0 );
			user_data_1.w[7] =  (int32_t) ( 0 );
			user_data_1.w[8] =  (int32_t) ( 0 );
			user_data_1.w[9] =  (int32_t) ( 0 );
			break;
		}

		case EXP_ACTUATOR_TESTING: //-3
		{// Testing Actuator Control Parameters
			user_data_1.w[2] = (int32_t) 0;
			user_data_1.w[3] = (int32_t) 0;
			user_data_1.w[4] = (int32_t) 0;
			user_data_1.w[5] = (int32_t) ( actx->torqueKp * 1000.0);
			user_data_1.w[6] = (int32_t) ( actx->torqueKi * 1000.0);
			user_data_1.w[7] = (int32_t) ( actx->torqueKd * 1000.0);
			user_data_1.w[8] = (int32_t) 0;
			break;
		}
		case EXP_ANKLE_WALKING_TORQUE_REPLAY: //4
		{// Testing Actuator Control Parameters
			user_data_1.w[2] = (int32_t) torqueRep->begin;
			user_data_1.w[3] = (int32_t) torqueRep->torqueScalingFactor;
			break;
		}

	}


	actx->initializedSettings = 1;
	wParams->initializedStateMachineVariables = 1;	// set flag that we initialized variables
}


#endif 	//BOARD_TYPE_FLEXSEA_MANAGE || defined BOARD_TYPE_FLEXSEA_PLAN
#endif //INCLUDE_UPROJ_MIT_DLEG || defined BOARD_TYPE_FLEXSEA_PLAN
