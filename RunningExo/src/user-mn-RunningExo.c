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
#include <math.h>
#include "user-mn.h"
#include "user-mn-ActPack.h"
#include "flexsea_sys_def.h"
#include "flexsea_user_structs.h"
#include "flexsea_global_structs.h"
#include <flexsea_system.h>
#include "flexsea_cmd_calibration.h"
#include <flexsea_comm.h>
#include "flexsea_board.h"
#include "user-mn-Rigid.h"
#include "cmd-ActPack.h"
#include "cmd-Rigid.h"
#include "user-mn-RunningExo.h"
#include "runningExo-parameters.h"
#include "torqueController.h"
#include "runningExo-torque-trajectories.h"
//----------------------------------------------------------------------------
// Local Variable(s) and Struct
//----------------------------------------------------------------------------


uint8_t mitRunningExoInfo[2] = {PORT_RS485_2, PORT_RS485_2};

int16_t fsm1State;

#if(CONTROL_STRATEGY==GAIT_TORQUE_TRACKING)
float torqueProfileGain = DEFAULT_TORQUE_PROFILE_GAIN;
int8_t bodyWeight = DEFAULT_BODY_WEIGHT;//user body weight (kg)
float currentScalar = CURRENT_SCALAR_INIT;
#endif //CONTROL_STRATEGY==GAIT_TORQUE_TRACKING
float torqueCommand;

//initialize running exo system state
struct runningExoSystemState runningExoState =
{.state=0,.prevStanceDuration=0,.timer=0,.pedometer=0,.heelStrikeTime=0,.toeOffTime=0, .enableOutput=0, .running=1, .prevGaitDuration=0, .footFlatTime=0};	//zero initialization

// initialize running exo sensor values struct
actuation_parameters act_para =
	{
	.ankleTorqueMeasured=0,.ankleTorqueDesired=0,.ankleHeight=0,.ankleVel=0,.ankleAcc=0,\
	.cableTensionForce=0,.motorTorqueMeasured=0,.motorTorqueDesired=0,.motorCurrentMeasured=0,\
	.motorCurrentDesired=0,.initialMotorEncPosition=0,.currentMotorEncPosition=0,\
	.motorRelativeEncRevolution=0,.motorRotationAngle=0,.motorAngularVel=0,.motorAngularAcc=0,\
	.boardTemperature=0,.safetyFlag=0,.currentOpLimit=0
	}; 	//zero initialization


//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************
void gaitStateTransition(void);
void trackTorque(void);
void checkInvalidGait(void);
void getMotorKinematics(struct actuation_parameters *actx);
void getAnkleKinematics(struct actuation_parameters *actx);
void getAnkleTorque(struct actuation_parameters *actx);
void getMotorTorque(struct actuation_parameters *actx);
int8_t findPolesRunningExo(void);
int8_t safetyShutoff(actuation_parameters *actx);
void parseSensorValues(actuation_parameters *actx);
void packRigidVars(struct actuation_parameters  *actx);
//****************************************************************************
// Public Function(s)
//****************************************************************************


//Prepare the system:
void init_runningExo(void)
//TODO
{
	fsm1State = STATE_IDLE;
	#if (CONTROL_STRATEGY == GAIT_TORQUE_TRACKING)
	runningExoState.state = HEEL_STRIKE;					//Initialize state
	#endif //CONTROL_STRATEGY == GAIT_TORQUE_TRACKING
	torqueCommand = 0;			//clear control action
}


//MIT RunExo Finite State Machine.
//Call this function in one of the main while time slots.
void RunningExo_fsm_1(void)
{
	#if(ACTIVE_PROJECT == PROJECT_RUNNING_EXO)

    static uint32_t time = 0;

    //Increment time (1 tick = 1ms nominally; need to confirm)
    time++;

    //Clear control command
    torqueCommand = 0;
    //Get sensor values into convenient structure if the system is ready
	parseSensorValues(&act_para);	// updates all actuator sensors
	rigid1.mn.genVar[6]=fsm1State;//debug

    //begin main FSM
	switch(fsm1State)
	{
		case STATE_IDLE:
			//Same power-on delay as FSM2:
			if(time >= AP_FSM2_POWER_ON_DELAY + 3*TIMESTEPS_PER_SECOND)
			{
				fsm1State = STATE_INIT;
				time = 0;
				//rigid1.mn.genVar[0]=fsm1State;
			}

			break;

		case STATE_INIT:
			//turned off for testing without Motor usage
			if(findPolesRunningExo())
			{
				fsm1State = STATE_ENABLE_SENSOR_UPDATE;
				time = 0;
				//rigid1.mn.genVar[1]=fsm1State;
			}
			//for testing
//			fsm1State = 0;

			break;

		case STATE_ENABLE_SENSOR_UPDATE:
			//init_LPF(); //initialize hardware LPF
			setControlMode(CTRL_OPEN);	//set to open loop voltage controller

			//zero encoder
			act_para.initialMotorEncPosition=*(rigid1.ex.enc_ang);

			fsm1State = STATE_TORQUE_TRACKING;
			time = 0;

			break;

		case STATE_TORQUE_TRACKING:
				//populate rigid1.mn.genVars to send to Plan
				//packRigidVars(&act_para);

				//begin safety check
			    if (safetyShutoff(&act_para))
			    {
			    	//Shutdown motor if safety check did not pass
			    	//gaitStateTransition(); //for testing only
			    	rigid1.mn.genVar[9]=-111;
			    	return;
			    }

			    else
			    {
					#if (CONTROL_STRATEGY == GAIT_TORQUE_TRACKING)
			    	//Torque Trajectory Tracking
			    	//Update states
			    	gaitStateTransition();
			    	//check for impossible gaits
			    	checkInvalidGait();
			    	//Perform actions
			    	switch (runningExoState.state)
			    	{
			    		case HEEL_STRIKE:
			    			//Gait cycle is between heel strike and toe off
			    			trackTorque();
			    			break;

			    		case FOOT_FLAT:
			    			//Foot flat is irrelevant in trajectory tracking
			    			trackTorque();
			    			break;

			    		case SWING_PHASE:
			    			//Swing phase
			    			torqueCommand = 0;					//no force output
			    			break;
			    	}
			    	runningExoState.timer+=1;					//increase timer
			    	torqueCommand *= (int)runningExoState.enableOutput;			//safety check
			    	rigid1.mn.genVar[2] = torqueCommand*1000;
			    	rigid1.mn.genVar[3] = runningExoState.state;
					#endif //CONTROL_STRATEGY == TORQUE_TRACKING

					#if (CONTROL_STRATEGY == TRAJECTORY_TORQUE_TRACKING)
			    		trackTorque();
			    		rigid1.mn.genVar[2] = torqueCommand*1000.0;
			    	#endif


			    	#if CONTROL_STRATEGY == USER_TORQUE_COMMAND
			    		//User torque command input from GUI
						torqueCommand = user_data_1.w[0]*1.0/1000.0;
			    		//echo torque command
			    		rigid1.mn.genVar[2] = torqueCommand*1000.0;
					#endif //CONTROL_STRATEGY == USER_TORQUE_COMMAND
			    	//Send torque command
		    		setTorque(torqueCommand, &act_para, 1,0);
			    }
				break;
        	default:
			//Handle exceptions here
        		return;
	}
	#endif	//ACTIVE_PROJECT == PROJECT_RUNNING_EXO
}

//Second state machine for the Running Exo
void RunningExo_fsm_2(void)
{

}

//****************************************************************************
// Private Function(s)
//****************************************************************************
#if(CONTROL_STRATEGY == GAIT_TORQUE_TRACKING)
void gaitStateTransition(void)
//Computes the current state
{
	//TODO: Add debounce filtering
	switch(runningExoState.state)
	{
		case SWING_PHASE:
			//State Transition
			if(rigid1.mn.analog[FOOTSWITCH_HEEL]>HEEL_STRIKE_THRESHOLD)
			{
				if (runningExoState.disabledPedometer>=DISABLED_STEPS)
					runningExoState.enableOutput=1;
				runningExoState.state = HEEL_STRIKE;
				runningExoState.prevGaitDuration=runningExoState.timer-runningExoState.heelStrikeTime;
				runningExoState.heelStrikeTime=runningExoState.timer;
				//Count steps at the end of each swing phase
				runningExoState.pedometer+=1;
				if(!runningExoState.enableOutput)
					runningExoState.disabledPedometer+=1;			//count
				else
					runningExoState.disabledPedometer = 0;
			}

			break;

		case HEEL_STRIKE:
			//State Transition
			if((rigid1.mn.analog[FOOTSWITCH_TOE]>FOOT_FLAT_THRESHOLD ||
						rigid1.mn.analog[FOOTSWITCH_LATERAL]>FOOT_FLAT_THRESHOLD ||
						rigid1.mn.analog[FOOTSWITCH_MEDIAL]>FOOT_FLAT_THRESHOLD)&&
						rigid1.mn.analog[FOOTSWITCH_HEEL]>FOOT_FLAT_THRESHOLD)
			{
				runningExoState.state = FOOT_FLAT;
				runningExoState.footFlatTime=runningExoState.timer;
			}
			break;

		case FOOT_FLAT:
			if((rigid1.mn.analog[FOOTSWITCH_TOE]<TOE_OFF_THRESHOLD &&
			rigid1.mn.analog[FOOTSWITCH_LATERAL]<TOE_OFF_THRESHOLD &&
			rigid1.mn.analog[FOOTSWITCH_MEDIAL]<TOE_OFF_THRESHOLD)&&
			rigid1.mn.analog[FOOTSWITCH_HEEL]<TOE_OFF_THRESHOLD)
			{
				runningExoState.state = SWING_PHASE;
				runningExoState.toeOffTime=runningExoState.timer;
				runningExoState.prevStanceDuration=runningExoState.timer-runningExoState.heelStrikeTime;
			}
			break;
	}
}

void trackTorque(void)
//sets control output based on trajectory tracking
{
	static float percentStance = 0;
	static float lastPercentageStance = 0;
	uint16_t lastIndex = 0;
	uint16_t currentIndex = 0;
	lastPercentageStance = percentStance;
	lastIndex = (int)(lastPercentageStance*(float)TABLE_SIZE);
	percentStance = (float)(runningExoState.timer-runningExoState.heelStrikeTime)/(float)runningExoState.prevStanceDuration>1 ? 1:(float)(runningExoState.timer-runningExoState.heelStrikeTime)/(float)runningExoState.prevStanceDuration;//range=[0,1]
	currentIndex = (int)(percentStance*(float)TABLE_SIZE);
	float torqueValue = 0;

	if (runningExoState.running)
	{
		#ifdef RUNNING_TORQUE_TRACKING
			torqueValue = unitRunningTorque[currentIndex]*bodyWeight*torqueProfileGain;
		#endif
	}
	else
	{
		#ifdef WALKING_TORQUE_TRACKING
			torqueValue = unitWalkingTorque[currentIndex]*bodyWeight*torqueProfileGain;
		#endif
	}

	//Set motor output
	//TODO: Unit conversion and casting
	torqueCommand=torqueValue;
}
#endif //CONTROL_STRATEGY == GAIT_TORQUE_TRACKING

#if(CONTROL_STRATEGY==TRAJECTORY_TORQUE_TRACKING)
void trackTorque(void)
//Tracks torque trajectory at 1 Hz
{
	static float percentStance = 0;
	uint16_t currentIndex = 0;
	currentIndex = (int)(percentStance*(float)TABLE_SIZE);
	float torqueValue = torqueTrajectory[currentIndex];
	torqueCommand=torqueValue;
	percentStance+=0.001;
	if (percentStance>1)
	{
		percentStance = 0;
	}
	//echo the torque value that is tracked

}
#endif //CONTROL_STRATEGY ==TRAJECTORY_TORQUE_TRACKING

void checkInvalidGait(void)
{
	//gait too long
	if (runningExoState.timer-runningExoState.heelStrikeTime > MAX_STANCE_PERIOD)
	{
		//disable output
		runningExoState.enableOutput = 0;
		runningExoState.disabledPedometer = 0;
	}

}

/*
 * Check for safety flags, and act on them.
 * todo: come up with correct strategies to deal with flags, include thermal limits also
 */
int8_t safetyShutoff(actuation_parameters *actx)
{
	#ifdef DISABLE_SAFETY
		return 0
	#endif

	//Take action if any of the safety checks failed
	//Checks listed in order. Earlier ones have higher priority
	//TODO:Debug safety
	//Check Temperature
	if(actx->boardTemperature > MAX_BOARD_TEMP)
	{
		//shutdown motor
		setMotorVoltage(0);
		return 1;
	}
	//Check Motor Position
//	if(abs(actx->motorAngularVel)>MAX_MOTOR_ANGLE)
//	{
//		return 1;
//	}

	//Check Motor Velocity
	//Note that this is not a "hard brake" to prevent oscillatory behavior around limit.
	//It just tries to stop accelerating.
	if(abs(actx->motorAngularVel)>MAX_MOTOR_SPEED)
	{
		setTorque(0, &act_para, 1,0);
		return 1;
	}

	//TODO: exo-specific checks

	//TODO: Current limitation?

//		case SAFETY_CABLE_TENSION:
//
//			//check if flag is not still active to be released, else do something about problem.
//			if(!isTorqueLimit)
//			{
//				isSafetyFlag = SAFETY_OK;
//				break;
//			}
//			else
//			{
//				//setAnkleTorque(&act_para, act_para.ankleTorqueDesired*0.5); //run this in order to update torque genVars sent to Plan
//				setCableTensionForce(&act_para, act_para.cableTensionForce*0.5);
//			}
//
//			return 1;
	return 0;
}



/*
 * Collect all sensor values and update the actuator structure.
 * Throws safety flags on Joint Angle, Joint Torque, since these functions look at transformed values.
 */
void parseSensorValues(actuation_parameters *actx)
{
	getMotorKinematics(actx);
	getAnkleKinematics(actx);
	getAnkleTorque(actx);
	//getMotorTorque(actx);

	actx->boardTemperature =  rigid1.re.temp;				//centidegree
	actx->motorCurrentMeasured = rigid1.ex.mot_current; 	//mA
}

//get motor's kinematic parameters
void getMotorKinematics(struct actuation_parameters *actx)
{
	//TODO: Put everything in SI units
	actx->currentMotorEncPosition = *(rigid1.ex.enc_ang);
	actx->motorRelativeEncRevolution = (actx->currentMotorEncPosition - actx->initialMotorEncPosition);
	actx->motorRotationAngle = actx->motorRelativeEncRevolution * 2 * M_PI / ENCODER_CPR;		//rad
	actx->motorAngularVel = (*(rigid1.ex.enc_ang_vel)*1.0/ENCODER_CPR) * 2 * M_PI*1000;	//rad/s, need to times SECONDS?????
	actx->motorAngularAcc = rigid1.ex.mot_acc;			//rad/s/s, need to check if the gain factor is needed
	//debug
//	rigid1.mn.genVar[5] = actx->currentMotorEncPosition;
//	rigid1.mn.genVar[8] = actx->motorRelativeEncRevolution;
//	rigid1.mn.genVar[9] = actx->motorAngularAcc;
//

//	 if(actx->motorRelativeEncRevolution >= MAX_MOTOR_ENC_REVOLUTION)
//	 {
//		isSafetyFlag = SAFETY_MOTOR_POSITION;
//		isPositionLimit = 1;
//	 }
//	 else
//	 {
//		 isPositionLimit = 0;
//	 }
//
//
}

//get ankle's kinematic parameters
void getAnkleKinematics(struct actuation_parameters *actx)
{
	actx->ankleHeight = actx->motorRelativeEncRevolution * MOT_OUTPUT_SHAFT_PERIMETER; // not so accurate, used for estimating, because the wrapping diameter will change after some revolutions.
	actx->ankleVel = actx->motorAngularVel * (MOT_OUTPUT_SHAFT_PERIMETER/2);		//m/s
	actx->ankleAcc = actx->motorAngularAcc * (MOT_OUTPUT_SHAFT_PERIMETER/2);		//m/s/s
}


////Determine torque at ankle
void getAnkleTorque(struct actuation_parameters *actx)
{
	 actx->ankleTorqueMeasured = ANKLE_TORQUE_CALIB_M * rigid1.ex.strain + ANKLE_TORQUE_CALIB_B; //N.m
}


/*
void setAnkleTorque(struct actuation_parameters *actx, float tau_desired_ankle)
{
	float tau_measured = 0, tau_desired = 0, tau_ff = 0;  	//joint torque reflected to motor. Feed forward term for desired joint torque, reflected to motor [Nm]
	float tau_error = 0;
	static float tau_error_last = 0, tau_error_integral = 0;
	float tau_PID = 0, tau_error_diff = 0, tau_motor_compensation = 0;		// motor torque signal
	float dtheta_m = 0, ddtheta_m = 0;		//motor angular velocity and acceleration
	int32_t I = 0;								// motor current signal

	dtheta_m = actx->motorAngularVel;  	//rad/s
	ddtheta_m = actx->motorAngularAcc;	//rad/s/s

	actx->ankleTorqueDesired = tau_desired_ankle;		// save in case need to modify in safetyFailure()
	actx->motorTorqueDesired = (actx->ankleTorqueDesired/MOMENT_ARM_ON_FOOT) * (MOT_OUTPUT_SHAFT_DIAMETER/2);

	// todo: better fidelity may be had if we modeled N_ETA as a function of torque, long term goal, if necessary
	tau_measured = actx->motorTorqueMeasured;	// measured torque reflected to motor [Nm]
	tau_desired = actx->motorTorqueDesired;		// output torque back to the motor [Nm].
	tau_ff = tau_desired / (N_ETA) ;		// Feed forward term for desired joint torque, reflected to motor [Nm]

	tau_motor_compensation = (MOT_J + MOT_TRANS)*ddtheta_m + MOT_B*dtheta_m;	// compensation for motor parameters. not stable right now.

	// Error is done at the motor. todo: could be done at the ankle, messes with our gains.
	tau_error = tau_desired - tau_measured;
	tau_error_diff = (tau_error - tau_error_last);
	if (!isClearErrorIntegral)
	{
		tau_error_integral = tau_error_integral + tau_error;  // check if this method is correct
	}
	else
	{
		tau_error_integral = 0;
	}
	tau_error_last = tau_error;

	//PID around motor torque
	tau_PID = tau_error*torqueKp + tau_error_diff*torqueKd + tau_error_integral*torqueKi;
	rigid1.mn.genVar[8]=tau_error_integral*torqueKi*100;
	I = 1/MOT_KT * ( tau_ff + tau_PID + tau_motor_compensation) * currentScalar;

	//account for deadzone current (unused due to instability). Unsolved problem according to Russ Tedrake.
//	if (abs(dtheta_m) < 3 && I < 0) {
//		I -= motSticNeg; //in mA
//	} else if (abs(dtheta_m) < 3 && I > 0) {
//		I += motSticPos;
//	}

//	if ( I < 0) {
//		I -= motSticNeg; //in mA
//	} else if ( I > 0) {
//		I += motSticPos;
//	}

	// add position, velocity, torque of the motor and ankle limit.


	//Soft angle limits with virtual spring. Raise flag for safety check.
	if (actx->motorRelativeEncRevolution <= 0  || actx->motorRelativeEncRevolution >= MAX_MOTOR_ENC_REVOLUTION)
	{
		isSafetyFlag = SAFETY_MOTOR_POSITION;
		isPositionLimit = 1;		//these are all redundant, choose if we want the struct thing.

		float angleDiff1 = actx->motorRelativeEncRevolution;
		float angleDiff2 = actx->motorRelativeEncRevolution - MAX_MOTOR_ENC_REVOLUTION;

		//Oppose motion using linear spring with damping
		if (actx->motorRelativeEncRevolution < 0)
		{
			if (abs(angleDiff1) < 3)
			{
				I -= currentOpLimit*(angleDiff1/3) + bLimit*actx->motorAngularVel;
			}
			else
			{
				I -= -currentOpLimit + bLimit*actx->motorAngularVel;
			}

		} else if (actx->motorRelativeEncRevolution - MAX_MOTOR_ENC_REVOLUTION > 0)
		{
			if (abs(angleDiff2) < 3)
			{
				I -= currentOpLimit*(angleDiff2/3) + bLimit*actx->motorAngularVel;
			}
			else
			{
				I -= currentOpLimit + bLimit*actx->motorAngularVel;
			}
		}

	}
	else
	{

		isPositionLimit = 0;

	}

	//Saturate I for our current operational limits -- limit can be reduced by safetyShutoff() due to heating
	if (I > currentOpLimit)
	{
		I = currentOpLimit;
	}
	else if (I < -currentOpLimit)
	{
		I = -currentOpLimit;
	}

	actx->motorCurrentDesired = (int32_t) I; 	// demanded mA
	setMotorCurrent(actx->motorCurrentDesired);	// send current command to comm buffer to Execute

	//variables used in cmd-rigid offset 5
	//rigid1.mn.userVar[5] = tau_measured*1000;
	//rigid1.mn.userVar[6] = tau_desired*1000;

}
*/

///*
// * Simple impedance controller
// * input:	theta_set, desired theta (degrees)
// * 			m,b,k impedance parameters
// * return: 	tor_d, desired torque
// */
//float calcImpedanceTorque(float m, float b, float k, float ddthetad_set, float dtheta_set, float theta_set)
//{
//	float theta = 0, dtheta = 0, ddtheta = 0;
//	float tor_d = 0;
//
//	theta = act_para.currentMotorEncPosition;
//	dtheta = act_para.motorAngularVel;
//	ddtheta = act_para.motorAngularAcc;
//	tor_d = m * (ddthetad_set - ddtheta) + b * (dtheta_set - dtheta) + k * (theta_set - theta);
////Todo, need to modify according to human running acceleration and velocity at foot
//	return tor_d;
//
//}
//
//

int8_t findPolesRunningExo(void)
{
	static uint32_t timer = 0;
	static int8_t polesState = 0;

	timer++;

	switch(polesState)
	{
		case 0:
			//Disable FSM2:
			disableActPackFSM2();
			if(timer > 100)
			{
				polesState = 1;
			}

			return 0;

		case 1:
			//Send Find Poles command:

			tx_cmd_calibration_mode_rw(TX_N_DEFAULT, CALIBRATION_FIND_POLES);
			packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_1, mitRunningExoInfo, SEND_TO_SLAVE);
			polesState = 2;
			timer = 0;
			rigid1.mn.genVar[7]=polesState; //debug
			return 0;

		case 2:

			if(timer >= 50*TIMESTEPS_PER_SECOND)
			{
				//Enable FSM2, position controller
				enableActPackFSM2();
				rigid1.mn.genVar[7]=4; //debug
				return 1;
			}
			rigid1.mn.genVar[7]=3; //debug

			return 0;


		default:

			return 0;

	}

	return 0;
}


void packRigidVars(struct actuation_parameters  *actx)
{
	// set float userVars to send back to Plan
	rigid1.mn.genVar[0] = actx->motorAngularVel*1000;
	rigid1.mn.genVar[1] = actx->motorRelativeEncRevolution*1000;
//	rigid1.mn.userVar[2] = actx->ankleHeight*1000;
//	rigid1.mn.userVar[3] = actx->ankleTorqueMeasured*1000;
//	rigid1.mn.userVar[4] = actx->ankleTorqueDesired*1000;
	//rigid1.mn.userVar[5] = tau_measured*1000;
	//rigid1.mn.userVar[6] = tau_desired*1000; (impedance controller - spring contribution)
}

#endif 	//defined BOARD_TYPE_FLEXSEA_MANAGE || defined BOARD_TYPE_FLEXSEA_PLAN

#endif 	//defined INCLUDE_UPROJ_RUNNINGEXO || defined BOARD_TYPE_FLEXSEA_PLAN


