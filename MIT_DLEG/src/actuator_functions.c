/****************************************************************************

*****************************************************************************
	[Lead developers] Matt Carney, mcarney at mit dot edu
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors] Matthew Carney, mcarney at mit dot edu, Tony Shu, tonyshu at mit dot edu
*****************************************************************************
	[This file] MIT DARPA Leg Actuator Specific functions
****************************************************************************/


//****************************************************************************
// Include(s)
//****************************************************************************
#include <user-mn-MIT-DLeg.h>
#include "actuator_functions.h"




//****************************************************************************
// Variable(s)
//****************************************************************************

//Variables which aren't static may be updated by Plan in the future

uint8_t mitDlegInfo[2] = {PORT_RS485_2, PORT_RS485_2};

//SAFETY FLAGS - in addition to enum, so can be cleared but don't lose other flags that may exist.
static int8_t isSafetyFlag = 0;
static int8_t isAngleLimit = 0;
static int8_t isTorqueLimit = 0;
static int8_t isTempLimit = 0;
static int8_t startedOverLimit = 1;

int8_t isEnabledUpdateSensors = 0;
int8_t fsm1State = -2;
float currentScalar = CURRENT_SCALAR_INIT;
int32_t currentOpLimit = CURRENT_LIMIT_INIT; 	//operational limit for current.

//torque gain values
float torqueKp = TORQ_KP_INIT;
float torqueKi = TORQ_KI_INIT;
float torqueKd = TORQ_KD_INIT;

//current gain values
int16_t currentKp = ACTRL_I_KP_INIT;
int16_t currentKi = ACTRL_I_KI_INIT;
int16_t currentKd = ACTRL_I_KD_INIT;

//motor param terms
float motJ = MOT_J;
float motB = MOT_B;

int32_t motSticNeg = MOT_STIC_NEG;
int32_t motSticPos = MOT_STIC_POS;

//const vars taken from defines (done to speed up computation time)
static const float angleUnit    = ANG_UNIT;
static const float jointZeroAbs = JOINT_ZERO_ABS;
static const float jointZero	= JOINT_ZERO;

static const float forcePerTick  = FORCE_PER_TICK;

static const float nScrew = N_SCREW;

static const float jointMinSoft = JOINT_MIN_SOFT;
static const float jointMaxSoft = JOINT_MAX_SOFT;
static const float jointMinSoftDeg = JOINT_MIN_SOFT * DEG_PER_RAD;
static const float jointMaxSoftDeg = JOINT_MAX_SOFT * DEG_PER_RAD;

struct diffarr_s jnt_ang_clks;		//maybe used for velocity and accel calcs.


//****************************************************************************
// Private Function(s)
//****************************************************************************





/*
 * Check for safety flags, and act on them.
 * todo: come up with correct strategies to deal with flags, include thermal limits also
 */
int8_t safetyShutoff(void) {

	switch(isSafetyFlag)

	{
		case SAFETY_OK:

			return 0;

		case SAFETY_ANGLE:
			//check if flag is not still active to be released, else do something about problem.
			if(!isAngleLimit) {
				isSafetyFlag = SAFETY_OK;
				break;
			} else {
				// do nothing. Clamping is handled in calcRestoringCurrent();
			}

			return 0; //continue running FSM

		case SAFETY_TORQUE:

			//check if flag is not still active to be released, else do something about problem.
			if(!isTorqueLimit) {
				isSafetyFlag = SAFETY_OK;
				break;
			} else {
				// This could cause trouble, but seems more safe than an immediate drop in torque. Instead, reduce torque.
				setMotorTorque(&act1, act1.tauDes * 0.5); // reduce desired torque by 25%
			}

			return 1;

		case SAFETY_TEMP:
			//check if flag is not still active to be released, else do something about problem.
			if( !isTempLimit ) {
				currentOpLimit = CURRENT_LIMIT_INIT;		// return to full power todo: may want to gradually increase
				isSafetyFlag = SAFETY_OK;
				break;
			} else {
				if (currentOpLimit > 0) {
					currentOpLimit--;	//reduce current limit every cycle until we cool down.
				}
			}

			return 0; //continue running FSM

		default:
			return 1;
	}


	return 0;
}

/*
 * Collect all sensor values and update the actuator structure.
 * Throws safety flags on Joint Angle, Joint Torque, since these functions look at transformed values.
 */
void updateSensorValues(struct act_s *actx)
{
	getJointAngleKinematic(actx);

	actx->jointAngleDegrees = actx->jointAngle * DEG_PER_RAD;
	actx->jointVelDegrees = actx->jointVel * DEG_PER_RAD;

	actx->linkageMomentArm = getLinkageMomentArm(actx->jointAngle);

//	actx->axialForce = 0.8*actx->axialForce + 0.2*getAxialForce();	// Filter signal
	actx->axialForce = getAxialForce();
	actx->axialForce = windowAveraging(actx->axialForce);	// Filter signal

	actx->jointTorque = getJointTorque(actx);

	updateJointTorqueRate(actx);

	//actx->jointTorqueRate = windowJointTorqueRate(actx);

	actx->motorVel =  *rigid1.ex.enc_ang_vel / 16.384 * ANG_UNIT;	// rad/s TODO: check on motor encoder CPR, may not actually be 16384
	actx->motorAcc = rigid1.ex.mot_acc;	// rad/s/s

	actx->regTemp = rigid1.re.temp;
	actx->motTemp = 0; // REMOVED FOR NOISE ISSUES getMotorTempSensor();
	actx->motCurr = rigid1.ex.mot_current;
	actx->currentOpLimit = currentOpLimit; // throttled mA

	actx->safetyFlag = isSafetyFlag;

	if(actx->regTemp > PCB_TEMP_LIMIT_INIT || actx->motTemp > MOTOR_TEMP_LIMIT_INIT)
	{
		isSafetyFlag = SAFETY_TEMP;
		isTempLimit = 1;
	} else {
		isTempLimit = 0;
	}
}

//The ADC reads the motor Temp sensor - MCP9700 T0-92. This function converts the result to C.
int16_t getMotorTempSensor(void)
{
	static int16_t mot_temp = 0;
	mot_temp = (rigid1.mn.analog[0] * (3.3/4096) - 500) / 10; 	//celsius
	rigid1.mn.mot_temp = mot_temp;

	return mot_temp;
}

/*
 * Output joint angle, vel, accel in ANG_UNIT, measured from joint zero,
 * Input:	joint[3] empty array reference
 * Return:	updated joint[3] array
 * 		joint[0] = angle
 * 		joint[1] = velocity
 * 		joint[2] = acceleration
 * 		todo: pass a reference to the act_s structure to set flags.
 */
//float* getJointAngleKinematic( void )
void getJointAngleKinematic(struct act_s *actx)
{
	static float last_jointVel = 0;
	static float jointAngleRad = 0;

	//ANGLE
	jointAngleRad = JOINT_ANGLE_DIR * ( jointZero + JOINT_ENC_DIR * ( (float) (*(rigid1.ex.joint_ang)) ) )  * RAD_PER_CNT; // (angleUnit)/JOINT_CPR;

	// filter joint angle? TODO: maybe remove this. or use windowAveraging
	actx->jointAngle = 0.8*actx->jointAngle + 0.2*jointAngleRad;

	//VELOCITY
	actx->jointVel = 0.8*actx->jointVel + 0.2*( (actx->jointAngle - actx->lastJointAngle) * SECONDS);

	//ACCEL  -- todo: check to see if this works
	actx->jointAcc = 0.8 * actx->jointAcc + 0.2*( (actx->jointVel - last_jointVel ) * SECONDS);

	// SAFETY CHECKS
	//if we start over soft limits after findPoles(), only turn on motor after getting within limits
	if (startedOverLimit && jointAngleRad < jointMaxSoft && jointAngleRad > jointMinSoft) {
		startedOverLimit = 0;
	}

	// Check that we're within specified units, else throw a flag.
	if (actx->jointAngle > JOINT_MAX_SOFT || actx->jointAngle < JOINT_MIN_SOFT) {
		isSafetyFlag = SAFETY_ANGLE;
		isAngleLimit = 1;
	} else {
		isAngleLimit = 0;
	}

	// Save values for next time through.
	actx->lastJointAngle = actx->jointAngle;
	last_jointVel = actx->jointVel;

}


// Output axial force on screw, Returns [Newtons]
float getAxialForce(void)
{
	static int8_t tareState = -1;
	static uint32_t timer = 0;
	uint16_t strainReading = 0;
	static float tareOffset = 0;
	float axialForce = 0;
	float numSamples = 100.;

	strainReading = (rigid1.ex.strain);

	switch(tareState)
	{
		case -1:
			//Tare the balance using average of numSamples readings
			timer++;

			if(timer <= numSamples) {
				strainReading = (rigid1.ex.strain);
				tareOffset += ((float) strainReading)/numSamples;
			} else {
				tareState = 0;
			}

			break;

		case 0:

			axialForce =  FORCE_DIR * (strainReading - tareOffset) * forcePerTick;

			// Filter the signal
//			axialForce = windowAveraging(axialForce, 5);

			break;

		default:
			//problem occurred
			break;
	}

	return axialForce;
}


// Linear Actuator Actual Moment Arm,
// input( jointAngle, theta [rad] )
// return moment arm projected length  [m]
//float getLinkageMomentArm(float theta)
float getLinkageMomentArm(float theta)
{
	static float A=0, c = 0, c2 = 0, r = 0, C_ang = 0;

    C_ang = M_PI - theta - (MA_TF); 	// angle
    c2 = MA_A2B2 - MA_TWOAB* cosf(C_ang);
    c = sqrtf(c2);  // length of actuator from pivot to output
    A = acosf(( MA_A2MINUSB2 - c2 ) / (-2*MA_B*c) );

    r = MA_B * sinf(A);

    return r/1000;
}


/*
 *  Determine torque at joint due to moment arm and axial force
 *  input:	struct act_s
 *  return: joint torque [Nm]
 *  //todo: more accurate is to track angle of axial force. maybe at getLinkageMomentArm
 */
float getJointTorque(struct act_s *actx)
{
	float torque = 0;

	torque = actx->linkageMomentArm * actx->axialForce;

	torque = torque * TORQ_CALIB_M + TORQ_CALIB_B;		//apply calibration to torque measurement

	if(torque >= ABS_TORQUE_LIMIT_INIT || torque <= -ABS_TORQUE_LIMIT_INIT) {
		isSafetyFlag = SAFETY_TORQUE;
		isTorqueLimit = 1;
	} else {
		isTorqueLimit = 0;
	}

	return torque;
}

/*
 *  Determine torque rate at joint using window averaging
 *  input:	struct act_s
 *  return: joint torque rate [Nm/s]
 */
float windowJointTorqueRate(struct act_s *actx) {
	#define TR_WINDOW_SIZE 3

	static int8_t index = -1;
	static float window[TR_WINDOW_SIZE];
	static float average = 0;
	static float previousTorque = 0;
	float currentRate = 0;

	index = (index + 1) % TR_WINDOW_SIZE;
	currentRate = (actx->jointTorque - previousTorque)*SECONDS;
	average -= window[index]/TR_WINDOW_SIZE;
	window[index] = currentRate;
	average += window[index]/TR_WINDOW_SIZE;

	previousTorque = actx->jointTorque;

	return average;

}


/*
 *  Filter using moving average.  This one works well for small window sizes
 *  larger windowsizes gets better accuracy with windowAverageingLarge
 *  uses WINDOW_SIZE
 */
float windowAveraging(float currentVal) {

	static int16_t index = -1;
	static float window[WINDOW_SIZE];
	static float average = 0;

	index = (index + 1) % WINDOW_SIZE;
	average = average - window[index]/WINDOW_SIZE;
	window[index] = currentVal;
	average = average + window[index]/WINDOW_SIZE;

	return average;
}



void updateJointTorqueRate(struct act_s *actx){

	//TODO: consider switching to windowAveraging
    actx->jointTorqueRate = 0.8 * actx->jointTorqueRate + 0.2 *( SECONDS * (actx->jointTorque - actx->lastJointTorque) );
    actx->lastJointTorque = actx->jointTorque;
}

/*
 * Calculate required motor torque, based on joint torque
 * input:	*actx,  actuator structure reference
 * 			tor_d, 	desired torque at joint [Nm]
 * return:	set motor torque
 * 			Motor Torque request, or maybe current
 */
void setMotorTorque(struct act_s *actx, float tau_des)
{
	float N = 1;					// moment arm [m]
	float tau_meas = 0, tau_ff=0;  	//joint torque reflected to motor.
	float tau_err = 0;
	static float tau_err_last = 0, tau_err_int = 0;
	float tau_PID = 0, tau_err_dot = 0, tau_motor_comp = 0;		// motor torque signal
	int32_t dtheta_m = 0, ddtheta_m = 0;		//motor vel, accel
	int32_t I = 0;								// motor current signal

	N = actx->linkageMomentArm * nScrew;
	dtheta_m = actx->motorVel;
	ddtheta_m = actx->motorAcc;

	if (tau_des > ABS_TORQUE_LIMIT_INIT) {
		tau_des = ABS_TORQUE_LIMIT_INIT;
	} else if (tau_des < -ABS_TORQUE_LIMIT_INIT) {
		tau_des = -ABS_TORQUE_LIMIT_INIT;
	}

	actx->tauDes = tau_des;				// save in case need to modify in safetyFailure()

	// todo: better fidelity may be had if we modeled N_ETA as a function of torque, long term goal, if necessary
	tau_meas =  actx->jointTorque / N;	// measured torque reflected to motor [Nm]
	tau_des = tau_des / N;				// scale output torque back to the motor [Nm].
	tau_ff = tau_des / (N_ETA) ;		// Feed forward term for desired joint torque, reflected to motor [Nm]

	tau_motor_comp = (motJ + MOT_TRANS)*ddtheta_m + motB*dtheta_m;	// compensation for motor parameters. not stable right now.

	// Error is done at the motor. todo: could be done at the joint, messes with our gains.
	tau_err = tau_des - tau_meas;
	tau_err_dot = (tau_err - tau_err_last);
	tau_err_int = tau_err_int + tau_err;
	tau_err_last = tau_err;

	//PID around motor torque
	tau_PID = tau_err*torqueKp + tau_err_dot*torqueKd + tau_err_int*torqueKi;

	if (!isAngleLimit) {
		I = 1/MOT_KT * (tau_ff + tau_PID + tau_motor_comp) * currentScalar;

	//joint velocity must not be 0 (could be symptom of joint position signal outage)
	} else if (actx->jointVel != 0 && !startedOverLimit) {
		I = 1/MOT_KT * (tau_ff + tau_PID + tau_motor_comp + calcRestoringCurrent(actx, N)) * currentScalar;
	//if we started beyond soft limits after finding poles, or joint position is out
	} else {
		I = 0;
	}


	//Saturate I for our current operational limits -- limit can be reduced by safetyShutoff() due to heating
	if (I > currentOpLimit)
	{
		I = currentOpLimit;
	} else if (I < -currentOpLimit)
	{
		I = -currentOpLimit;
	}

	actx->desiredCurrent = (int32_t) I; 	// demanded mA
	setMotorCurrent(actx->desiredCurrent, 0);	// send current command to comm buffer to Execute

	//variables used in cmd-rigid offset 5
	rigid1.mn.userVar[5] = tau_meas*1000;
	rigid1.mn.userVar[6] = tau_des*1000;

}

float calcRestoringCurrent(struct act_s *actx, float N) {
	//Soft angle limits with virtual spring. Raise flag for safety check.

	float angleDiff = 0;
	float tauRestoring = 0;
	float k = 0.2; // N/m
	float b = 0; // Ns/m

	//Oppose motion using linear spring with damping
	if (actx->jointAngleDegrees - jointMinSoftDeg < 0) {

		angleDiff = actx->jointAngleDegrees - jointMinSoftDeg;
		angleDiff = pow(angleDiff,4);
		tauRestoring = -k*angleDiff - b*actx->jointVelDegrees;

	} else if (actx->jointAngleDegrees - jointMaxSoftDeg > 0) {

		angleDiff = actx->jointAngleDegrees - jointMaxSoftDeg;
		angleDiff = pow(angleDiff,4);
		tauRestoring = -k*angleDiff - b*actx->jointVelDegrees;

	}

	return tauRestoring;

}

//Used for testing purposes. See state_machine
/*
 * Simple Biom controller
 * input:	theta_set, desired theta (degrees)
 * 			k1,k2,b, impedance parameters
 * return: 	tor_d, desired torque
 */
float biomCalcImpedance(float k1, float b, float theta_set)
{
	float theta = 0, theta_d = 0;
	float tor_d = 0;

	theta = act1.jointAngleDegrees;
	theta_d = act1.jointVelDegrees;
	tor_d = k1 * (theta_set - theta ) - b*theta_d;

	return tor_d;

}

void mit_init_current_controller(void) {

	setControlMode(CTRL_CURRENT, 0);
	writeEx[0].setpoint = 0;			// wasn't included in setControlMode, could be safe for init
	setControlGains(currentKp, currentKi, currentKd, 0, 0);

}

int8_t findPoles(void) {
	static uint32_t timer = 0;
	static int8_t polesState = 0;

	timer++;

	switch(polesState) {
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
			packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_1, mitDlegInfo, SEND_TO_SLAVE);
			polesState = 2;
			timer = 0;

			return 0;

		case 2:

			if(timer >= 44*SECONDS)
			{
				//Enable FSM2, position controller
				enableActPackFSM2();
				return 1;
			}
			return 0;


		default:

			return 0;

	}

	return 0;
}

void packRigidVars(struct act_s *actx) {

	// set float userVars to send back to Plan
	rigid1.mn.userVar[0] = actx->jointAngleDegrees*1000;
	rigid1.mn.userVar[1] = actx->jointVelDegrees*1000;
	rigid1.mn.userVar[2] = actx->linkageMomentArm*1000;
	rigid1.mn.userVar[3] = actx->axialForce*1000;
	rigid1.mn.userVar[4] = actx->jointTorque*1000;
    //userVar[5] = tauMeas
    //userVar[6] = tauDes (impedance controller - spring contribution)
}

float windowSmoothJoint(int32_t val) {
	#define JOINT_WINDOW_SIZE 5

	static int8_t index = -1;
	static float window[JOINT_WINDOW_SIZE];
	static float average = 0;


	index = (index + 1) % JOINT_WINDOW_SIZE;
	average -= window[index]/JOINT_WINDOW_SIZE;
	window[index] = (float) val;
	average += window[index]/JOINT_WINDOW_SIZE;

	return average;
}

float windowSmoothAxial(float val) {
	#define AXIAL_WINDOW_SIZE 5

	static int8_t index = -1;
	static float window[AXIAL_WINDOW_SIZE];
	static float average = 0;


	index = (index + 1) % AXIAL_WINDOW_SIZE;
	average -= window[index]/AXIAL_WINDOW_SIZE;
	window[index] = val;
	average += window[index]/AXIAL_WINDOW_SIZE;

	return average;
}


/*
 * Set Motor torques open loop.  Used for System characterization.
 */
void setMotorTorqueOpenLoop(struct act_s *actx, float tau_des)
{
	static float N = 1;					// total gear ratio
	static float tau_meas = 0;	//joint torque reflected to motor.
	static int32_t I = 0;								// motor current signal

	// Step 1: Convert desired joint torque to desired motor Torque

	if (tau_des > ABS_TORQUE_LIMIT_INIT) {
		tau_des = ABS_TORQUE_LIMIT_INIT;
	} else if (tau_des < -ABS_TORQUE_LIMIT_INIT) {
		tau_des = -ABS_TORQUE_LIMIT_INIT;
	}

	actx->tauDes = tau_des;				// save in case need to modify in safetyFailure()

	// Step 1: evaluate overall gear ratio
	N = actx->linkageMomentArm * N_SCREW;

	tau_des = tau_des / (N);				// scale output torque back to the motor [Nm].

	//If we're within joint limits, scale torque to current.
	if (!isAngleLimit) {
		I = 1/MOT_KT * (tau_des) * CURRENT_SCALAR_INIT;
	} else {
		I = 0;
	}

	// Motor stiction compensation
	if (I > 0){
		I = I + MOT_STIC_POS;
	} else if (I < 0){
		I = I - MOT_STIC_NEG;
	}

	//Saturate I for our current operational limits
	if (I > CURRENT_LIMIT_INIT)
	{
		I = CURRENT_LIMIT_INIT;
	} else if (I < -CURRENT_LIMIT_INIT)
	{
		I = -CURRENT_LIMIT_INIT;
	}

	actx->desiredCurrent = (int32_t) I; 	// demanded mA
	setMotorCurrent(actx->desiredCurrent, 0);	// send current command to comm buffer to Execute

}


/*
 * Set Motor torques open loop, using Voltage control  Used for System characterization.
 */
void setMotorTorqueOpenLoopVolts(struct act_s *actx, float tau_des)
{
	static float N = 1;					// total gear ratio
	static float tau_meas = 0;	//joint torque reflected to motor.
	static int32_t I = 0;								// motor current signal
	static int32_t V = 0;

	// Step 1: Convert desired joint torque to desired motor Torque

	if (tau_des > ABS_TORQUE_LIMIT_INIT) {
		tau_des = ABS_TORQUE_LIMIT_INIT;
	} else if (tau_des < -ABS_TORQUE_LIMIT_INIT) {
		tau_des = -ABS_TORQUE_LIMIT_INIT;
	}

	actx->tauDes = tau_des;				// save in case need to modify in safetyFailure()

	// Step 1: evaluate overall gear ratio
	N = actx->linkageMomentArm * N_SCREW;

	tau_des = tau_des / (N);				// scale output torque back to the motor [Nm].

	//If we're within joint limits, scale torque to current.
	if (!isAngleLimit) {
		I = 1/MOT_KT * (tau_des) * CURRENT_SCALAR_INIT;
	} else {
		I = 0;
	}

//	// Motor stiction compensation
//	if (I > 0){
//		I = I + MOT_STIC_POS;
//	} else if (I < 0){
//		I = I - MOT_STIC_NEG;
//	}

	//Saturate I for our current operational limits
	if (I > CURRENT_LIMIT_INIT)
	{
		I = CURRENT_LIMIT_INIT;
	} else if (I < -CURRENT_LIMIT_INIT)
	{
		I = -CURRENT_LIMIT_INIT;
	}

	V =(int32_t) ( I*MOT_R + (MOT_KT * actx->motorVel) * CURRENT_SCALAR_INIT) ;

	actx->desiredCurrent = (int32_t) I; 	// demanded mA
//	setMotorCurrent(actx->desiredCurrent, 0);	// send current command to comm buffer to Execute
	setMotorVoltage(V, 0);

}

/*
 * Compute where we are in frequency sweep
 * omega [rad/s], t [seconds]
 */
float frequencySweep(float omega, float t){
	return sinf(omega * ( t  ) );
}


// Step through series of data points to set torque
// Pseudo Random Binary for systemID
// Start when received signal
float torqueSystemID(void)
{
	// Pseudo Random Binary for systemID
	static const int16_t inputDataSet[2047] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1};

	int8_t start = user_data_1.w[0];				// Turn it on.
	int16_t torqueAmplitude = user_data_1.w[1];		// amplitude of torque signal
	int8_t cyclingNumber = user_data_1.w[2];		// number of times to run through data set

	float torqueSetPoint = 0;

	int16_t cyclingPeriodLength =  (int16_t) ( sizeof(inputDataSet) / sizeof(inputDataSet[0]) -1);
	//	float pulseShiftingPeriod = 0.105;  // < 2*pi/(3*wc) = 2*pi/(3*20) = 0.105,unit of wc is Hz.
	static int16_t stepTimer = 0, arrayStep = 0;


	if (start == 1)
	{
		if (stepTimer <= cyclingNumber*cyclingPeriodLength)
		{
			if (arrayStep <= cyclingPeriodLength-1)
			{
				torqueSetPoint = (float) ( torqueAmplitude * inputDataSet[arrayStep] );
				rigid1.mn.genVar[7] = (int16_t) (start); // record starting.
				arrayStep++;
			} else
			{
				arrayStep = 0;
				torqueSetPoint = 0.0;
			}
			stepTimer++;
		}
	}
	else
	{
		torqueSetPoint = 0.0;
		stepTimer = 0;
	}

	return torqueSetPoint;
}
