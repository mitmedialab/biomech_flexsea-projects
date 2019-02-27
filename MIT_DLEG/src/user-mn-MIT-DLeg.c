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
#include "walking_state_machine.h"	// Included to allow UserWrites to update walking machine controller.
#include "run_main_user_application.h"	// This is where user application functions live
#include "ui.h"


//****************************************************************************
// Variable(s)
//****************************************************************************


uint8_t mitDlegInfo[2] = {PORT_RS485_1, PORT_RS485_1};
uint8_t enableMITfsm2 = 0, mitFSM2ready = 0, mitCalibrated = 0;
#define THIS_ACTPACK		0
#define SLAVE_ACTPACK		1


float freqInput = 0.0;
float freqRad = 0.0;
float torqInput = 0.0;
int8_t currentOrVoltage = 0;


int8_t onEntry = 0;
Act_s act1;

// EXTERNS
extern uint8_t calibrationFlags, calibrationNew;
extern int32_t currentOpLimit;
extern int8_t zeroIt; 		// used for zeroing the load cell.
extern float voltageGain;
extern float velGain;
extern float indGain;

extern float torqueKp;
extern float torqueKi;
extern float torqueKd;

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

    //Increment fsm_time (1 tick = 1ms nominally)
    fsmTime++;
	  rigid1.mn.genVar[0] = (int16_t) (getSafetyFlags()); 			//errors
	  rigid1.mn.genVar[1] = (int16_t) (act1.jointTorque*100.);		// Nm
	  rigid1.mn.genVar[2] = (int16_t) (act1.jointVel*1000.);			// radians/s
	  rigid1.mn.genVar[3] = (int16_t) (act1.tauDes*100.);//(act1.jointAngleDegrees*1000.);	// deg
	  rigid1.mn.genVar[4] = (int16_t) rigid2.mn.genVar[1]; //(*rigid1.ex.enc_ang_vel);		// comes in as rad/s
	  rigid1.mn.genVar[5] = (int16_t) (*rigid1.ex.enc_ang); 		//cpr, 16384 cpr
	  rigid1.mn.genVar[6] = (int16_t) (rigid1.ex.mot_current);		// mA
	  rigid1.mn.genVar[7] = (int16_t) (rigid1.ex.mot_volt);			// mV, //getDeviceIdIncrementing() ;
	  rigid1.mn.genVar[8] = (int16_t) (rigid2.ex.mot_current);			// mA
	  rigid1.mn.genVar[9] = (int16_t) (rigid2.ex.mot_volt); //rigid2.mn.genVar[7]; //(rigid1.re.vb);				// mV

    //begin main FSM
	switch(fsm1State)
	{
		//this state is always reached
		case STATE_POWER_ON:
			stateMachine.currentState = STATE_POWER_ON;
			//Same power-on delay as FSM2:
			if(fsmTime >= AP_FSM2_POWER_ON_DELAY) {
				//sensor update happens in mainFSM2(void) in main_fsm.c

				isEnabledUpdateSensors = 1;
				onEntry = 1;
				fsmTime = 0;
				fsm1State = STATE_INITIALIZE_SENSORS;

			}

			break;

		case STATE_INITIALIZE_SENSORS:

			if(fsmTime >= AP_FSM2_POWER_ON_DELAY) {

				#if defined(NO_DEVICE)
					fsm1State = STATE_DEBUG;				// Skip over All states and sit in debug mode
				#elif defined(NO_ACTUATOR)
					fsm1State = STATE_INIT_USER_WRITES;		// Still run controls, but skip Find Poles
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

//			stateMachine.current_state = STATE_INIT;
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
					fsm1State = STATE_INIT_USER_WRITES;
					fsmTime = 0;
					isEnabledUpdateSensors = 1;

				}
			}


			break;

		case STATE_INIT_USER_WRITES:

			/*reserve for additional initialization*/

			//Initialize Filters for Torque Sensing
			initSoftFIRFilt();	// Initialize software filter

			mitInitCurrentController();		//initialize Current Controller with gains

			//Set userwrites to initial values
			walkParams.initializedStateMachineVariables = 0;
			if (!walkParams.initializedStateMachineVariables){
				initializeUserWrites(&act1, &walkParams);
				walkParams.initializedStateMachineVariables = 1;
			}

			//absolute torque limit scaling factor TODO: possibly remove
			act1.safetyTorqueScalar = 1.0;

			fsm1State = STATE_MAIN;
			fsmTime = 0;
			onEntry = 1;

			break;

		case STATE_MAIN:
			{
				updateUserWrites(&act1, &walkParams);

				if (onEntry > DELAY_TICKS_AFTER_FIND_POLES) {
					act1.currentOpLimit = CURRENT_LIMIT_INIT;
					onEntry = 0;
				}

				// Inside here is where user code goes
				//DEBUG removed this because joint encoder can't update in locked state.
//				if (getMotorMode() == MODE_ENABLED || getMotorMode() == MODE_OVERTEMP ){

					float tor = getImpedanceTorque(&act1, torqInput, freqInput, 0);

//					setMotorTorque( &act1, tor);

//					runMainUserApplication(&act1);



// DEBUG todo: this is how to talk to slave or this actuator, setMotorVoltage(value)
//			setControlMode(CTRL_OPEN, THIS_ACTPACK);
//			setControlMode(CTRL_OPEN, SLAVE_ACTPACK);
//			setMotorVoltage(0, THIS_ACTPACK);
//			setMotorVoltage(0, SLAVE_ACTPACK);
//			fsm1State = 1;
//			deltaT = 0;

//				}

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
	//Verify we are Master and communicating to Slave
	#if (ACTIVE_PROJECT == PROJECT_MIT_DLEG) // TODO: should this be here? or is this bidirectional && (ACTIVE_SUBPROJECT == SUBPROJECT_A)

	//Modified version of ActPack
	static uint32_t Fsm2Timer = 0;
	static uint8_t offsetCounter = 0;

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
			writeEx[1].offset = 1;
			tx_cmd_actpack_rw(TX_N_DEFAULT, writeEx[1].offset, writeEx[1].ctrl, writeEx[1].setpoint, \
											writeEx[1].setGains, writeEx[1].g[0], writeEx[1].g[1], \
											writeEx[1].g[2], writeEx[1].g[3], 0);	// todo: try this offset counter thing

			packAndSend(P_AND_S_DEFAULT, FLEXSEA_MANAGE_2, mitDlegInfo, SEND_TO_SLAVE);


			//Reset KEEP/CHANGE once set:
			//if(writeEx[0].setGains == CHANGE){writeEx[0].setGains = KEEP;}
			if(writeEx[1].setGains == CHANGE){writeEx[1].setGains = KEEP;}

			// Was in use to step through multiple offsets, not in use currently.
//			offsetCounter++;
//			if (offsetCounter > 3)
//			{
//				offsetCounter = 0;
//			}
	}

	#endif	//ACTIVE_PROJECT == PROJECT_MIT_DLEG
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

/*UserWrites are inputs from Plan. They are initailized to teh values shown below.
 * Their values are then used by udpateUserWrites to set function values.
 * These can be updated as necessary.
 * Do keep care to initialize and upate correctly.
 * Also note, the initial values will not show up in Plan, that must be manually entered
 */

/*
 * Updates the Input values based off of user data
 * Param: actx(Act_s) - Actuator structure to track sensor values
 * Param: wParams(WalkParams) -
 */
void updateUserWrites(Act_s *actx, WalkParams *wParams){
  
	torqInput								= ( (float) user_data_1.w[0] ) /10.0;	// Reduce overall torque limit.
	freqInput								= ( (float) user_data_1.w[1] ) /10.0;	// Reduce overall torque limit.
	zeroIt								    = ( (int8_t) user_data_1.w[2] );			// Manually zero the load cell
	currentOrVoltage					    = ( (int8_t) user_data_1.w[3] );			// Manually zero the load cell
	voltageGain								= ( (float) user_data_1.w[4] ) /1000.0;
	velGain								= ( (float) user_data_1.w[5] ) /10.0;
	indGain								= ( (float) user_data_1.w[610] ) /10.0;
//	torqueKp 								= ( (float) user_data_1.w[0] ) /1000.0;	// Reduce overall torque limit.
//	torqueKi				 				= ( (float) user_data_1.w[1] ) /1000.0;	// Reduce overall torque limit.
//	torqueKd				 				= ( (float) user_data_1.w[2] ) /1000.0;	// Reduce overall torque limit.


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

/*
 * Initializes the Input values
 * Param: actx(Act_s) - Actuator structure to track sensor values
 * Param: wParams(WalkParams) -
 */
void initializeUserWrites(Act_s *actx, WalkParams *wParams){

//	wParams->earlyStanceK0 = 6.23;
//	wParams->earlyStanceKF = 0.1;
//	wParams->earlyStanceDecayConstant = EARLYSTANCE_DECAY_CONSTANT;
  
//	torqueKp				 				= 0.0; 	//user_data_1.w[0] = 100
//	torqueKi 								= 0.0;
//	torqueKd								= 0.0;
	torqInput								= 0.0;
	freqInput								= 0.0;
	voltageGain								= 0.0;
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

	user_data_1.w[0] =  (int32_t) ( 0.0 ); 	// torque scalar
	user_data_1.w[1] =  (int32_t) ( 0.0 ); 	// frequency set point for freq test
	user_data_1.w[2] =  (int32_t) ( 0.0 ); 	// torque input for freq test
	user_data_1.w[3] =  (int32_t) ( 0.0 ); 	// torque input for freq test
	user_data_1.w[4] =  (int32_t) ( 1.0 ); 	// torque input for freq test
	user_data_1.w[5] =  (int32_t) ( 1.0 ); 	// torque input for freq test
	user_data_1.w[6] =  (int32_t) ( 1.0 ); 	// torque input for freq test

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
