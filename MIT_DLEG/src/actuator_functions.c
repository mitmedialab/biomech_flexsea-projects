
/*****************************************************************************
	[Lead developers] Matt Carney, mcarney at mit dot edu
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors] Matthew Carney, mcarney at mit dot edu, Tony Shu, tonyshu at mit dot edu
*****************************************************************************
	[This file] MIT DARPA Leg Actuator Specific functions
****************************************************************************/


//****************************************************************************
// Include(s)
//****************************************************************************

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
int8_t fsm1State = STATE_POWER_ON;
float currentScalar = CURRENT_SCALAR_INIT;


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



//const vars taken from defines (done to speed up computation time)
static const float angleUnit    = ANG_UNIT;
static const float jointZeroAbs = JOINT_ZERO_ABS;
static const float jointZero	= JOINT_ZERO;

static const float forcePerTick  = FORCE_PER_TICK;

static const float jointMinSoft = JOINT_MIN_SOFT;
static const float jointMaxSoft = JOINT_MAX_SOFT;
static const float jointMinSoftDeg = JOINT_MIN_SOFT * DEG_PER_RAD;
static const float jointMaxSoftDeg = JOINT_MAX_SOFT * DEG_PER_RAD;

struct diffarr_s jnt_ang_clks;		//maybe used for velocity and accel calcs.





//The ADC reads the motor Temp sensor - MCP9700 T0-92. This function converts the result to C.
static int16_t getMotorTempSensor(void)
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
static void getJointAngleKinematic(struct act_s *actx)
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
	//TODO: I think we can remove this, verify we are handling this with safety functions
	if (startedOverLimit && jointAngleRad < jointMaxSoft && jointAngleRad > jointMinSoft) {
		startedOverLimit = 0;
	}

	// Save values for next time through.
	actx->lastJointAngle = actx->jointAngle;
	last_jointVel = actx->jointVel;

}


// Output axial force on screw, Returns [Newtons]
static float getAxialForce(void)
{
	static int8_t tareState = -1;
	static uint32_t timer = 0;
	float strainReading = 0;
	static float tareOffset = 0;
	float axialForce = 0;
	float numSamples = 1000.;
	float timerDelay = 100.;

	strainReading = (float) rigid1.ex.strain;

	switch(tareState)
	{
		case -1:
			//Tare the balance using average of numSamples readings
			timer++;

			//DEBUG
			if(timer >= timerDelay && timer < numSamples + timerDelay) {
				tareOffset += (strainReading)/numSamples;
			} else if (timer >= numSamples + timerDelay) {
				tareState = 0;
			}

			break;

		case 0:

			axialForce =  FORCE_DIR * (strainReading - tareOffset) * forcePerTick;
			//DEBUG
//			axialForce =  FORCE_DIR * (strainReading) * forcePerTick;


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
static float getLinkageMomentArm(float theta)
{
	static float A=0, c = 0, c2 = 0, r = 0, C_ang = 0;

    C_ang = M_PI - theta - (MA_TF); 	// angle
    c2 = MA_A2B2 - MA_TWOAB* cosf(C_ang);
    c = sqrtf(c2);  // length of actuator from pivot to output
    A = acosf(( MA_A2MINUSB2 - c2 ) / (-2*MA_B*c) );

    r = MA_B * sinf(A);

    return r/1000.;
}


/*
 *  Determine torque at joint due to moment arm and axial force
 *  input:	struct act_s
 *  return: joint torque [Nm]
 *  //todo: more accurate is to track angle of axial force. maybe at getLinkageMomentArm
 */
static float getJointTorque(struct act_s *actx)
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
static float windowJointTorqueRate(struct act_s *actx) {
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
//UNDERGRAD TODO: figure out whythis isn't working
static float windowAveraging(float currentVal) {

	static int16_t index = -1;
	static float window[WINDOW_SIZE];
	static float average = 0;

	index = (index + 1) % WINDOW_SIZE;
	average = average - window[index]/WINDOW_SIZE;
	window[index] = currentVal;
	average = average + window[index]/WINDOW_SIZE;

	return average;
}

static float calcRestoringCurrent(struct act_s *actx, float N) {
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

static void updateJointTorqueRate(struct act_s *actx){

	//TODO: consider switching to windowAveraging

	float diff = actx->jointTorque - actx->lastJointTorque;
    actx->jointTorqueRate = 0.8 * actx->jointTorqueRate + 0.2 *( SECONDS * (diff) );
    actx->lastJointTorque = actx->jointTorque;
}

/*
 * Anti-windup clamp for integral term.
 * Check if output is saturating, and if error in the same direction
 * Clamp the integral term if that's the case.
 */
bool integralAntiWindup(float tau_err, float tau_C_total, float tau_C_output) {
	//Anti-windup clamp for Integral Term of PID
	int8_t inSatLimit = 0;
	if (tau_C_total == tau_C_output){
		inSatLimit = 0;
	} else {
		inSatLimit = 1;
	}

	int8_t errSign = 0;
	if (tau_err > 0 && tau_C_total > 0) {
		errSign = 1;
	} else if (tau_err < 0 && tau_C_total < 0) {
		errSign = -1;
	} else {
		errSign = 0;
	}

	if (inSatLimit && errSign) {
		return 1;
	} else {
		return 0;
	}
}

/*
 * Check for angle limits and apply virtual unidirectional spring/dampers
 * input:		actuator structure
 * output:		virtual impedance torque value
 */
//TODO: check that damping ratio is in the correct direction.
float actuateAngleLimits(Act_s *actx){
	float tau_K = 0; float tau_B = 0;

	// apply unidirectional spring
	if ( actx->jointAngleDegrees < JOINT_MIN_SOFT_DEGREES ) {
		tau_K = JOINT_SOFT_K * (JOINT_MIN_SOFT_DEGREES - actx->jointAngleDegrees);

		// apply unidirectional damper
		if ( actx->jointVelDegrees < 0) {
			tau_B = -JOINT_SOFT_B * actx->jointVelDegrees;
		}

	} else if ( actx->jointAngleDegrees > JOINT_MAX_SOFT_DEGREES) {
		tau_K = JOINT_SOFT_K * (JOINT_MAX_SOFT_DEGREES - actx->jointAngleDegrees);

		// apply unidirectional damper
		if ( actx->jointVelDegrees > 0) {
			tau_B = -JOINT_SOFT_B * actx->jointVelDegrees;
		}
	}

	return tau_K + tau_B;
}

/*
 * Check current direction and provide baseline current to handle
 * the No load current requirement to get motor moving.
 */
float noLoadCurrent(float desCurr) {
	if (desCurr > 0) {
		return MOT_NOLOAD_CURRENT_POS;
	} else if (desCurr < 0) {
		return MOT_NOLOAD_CURRENT_NEG;
	} else {
		return 0;
	}
}


/*
 * Calculate required motor torque, based on joint torque
 * input:	*actx,  actuator structure reference
 * 			tor_d, 	desired torque at joint [Nm]
 * action:			send current command to motor;
 *
 */
void setMotorTorque(struct act_s *actx, float tau_des)
{
	//Angle Limit bumpers
	actx->tauDes = tau_des + actuateAngleLimits(actx);
	actx->tauMeas = actx->jointTorque;

	static float tau_err_last = 0, tau_err_int = 0;
	float N = actx->linkageMomentArm * N_SCREW;	// gear ratio

	// Error is done at the motor. todo: could be done at the joint, messes with our gains.
	float tau_err = actx->tauDes - actx->tauMeas;		// [Nm]
	float tau_err_dot = (tau_err - tau_err_last)*SECONDS;		// [Nm/s]
	tau_err_int = tau_err_int + tau_err;				// [Nm]
	tau_err_last = tau_err;

	//PID around motor torque
	float tau_C = tau_err*torqueKp + tau_err_dot*torqueKd + tau_err_int*torqueKi;	// torq Compensator

	// Feedforward term
	float tau_ff = 0.0; 	// Not in use at the moment todo: figure out how to do this properly

	float tau_C_combined = tau_C + tau_ff;

	//Saturation limit on Torque
	float tau_C_output = tau_C_combined;
	if (tau_C_combined > ABS_TORQUE_LIMIT_INIT) {
		tau_C_output = ABS_TORQUE_LIMIT_INIT;
	} else if (tau_C_combined < -ABS_TORQUE_LIMIT_INIT) {
		tau_C_output = -ABS_TORQUE_LIMIT_INIT;
	}

	// Clamp and turn off integral term if it's causing a torque saturation
	if ( integralAntiWindup(tau_err, tau_C_combined, tau_C_output) ){
		tau_err_int = 0;
	}

	// motor current signal
	int32_t I = (int32_t) ( 1.0/(MOT_KT * N * N_ETA) * tau_C_output * CURRENT_SCALAR_INIT);

	//Saturate I for our current operational limits -- limit can be reduced by safetyShutoff() due to heating
	if (I > actx->currentOpLimit)
	{
		I = actx->currentOpLimit;
	} else if (I < -actx->currentOpLimit)
	{
		I = -actx->currentOpLimit;
	}

	actx->desiredCurrent = I;// + noLoadCurrent(I); 	// demanded mA

	setMotorCurrent(actx->desiredCurrent, DEVICE_CHANNEL);	// send current command to comm buffer to Execute

	//variables used in cmd-rigid offset 5, for multipacket
	rigid1.mn.userVar[5] = actx->tauMeas*1000;	// x1000 is for float resolution in int32
	rigid1.mn.userVar[6] = actx->tauDes*1000;
}



//Used for testing purposes. See state_machine
/*
 * Simple Biom controller
 * input:	theta_set, desired theta (degrees)
 * 			k1,b, impedance parameters
 * return: 	desired torque
 */
float biomCalcImpedance(Act_s *actx, float k1, float b, float theta_set)
{
	return k1 * (theta_set - actx->jointAngleDegrees ) - b*actx->jointVelDegrees;

}

void mit_init_current_controller(void) {
	act1.currentOpLimit = CURRENT_ENTRY_INIT;
	setControlMode(CTRL_CURRENT, DEVICE_CHANNEL);
	writeEx[DEVICE_CHANNEL].setpoint = 0;			// wasn't included in setControlMode, could be safe for init
	setControlGains(currentKp, currentKi, currentKd, 0, DEVICE_CHANNEL);

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


static float windowSmoothJoint(int32_t val) {
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

static float windowSmoothAxial(float val) {
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
	// Saturate desired torque
		if (tau_des > ABS_TORQUE_LIMIT_INIT) {
				tau_des = ABS_TORQUE_LIMIT_INIT;
		} else if (tau_des < -ABS_TORQUE_LIMIT_INIT) {
			tau_des = -ABS_TORQUE_LIMIT_INIT;
		}
		actx->tauDes = tau_des;

		float N = actx->linkageMomentArm * N_SCREW;	// Drivetrain Reduction Ratio

		//Angle Limit bumpers
		float tau_C_output = actx->tauDes + actuateAngleLimits(actx);

		// motor current signal
		int32_t I = (int32_t) ( 1/(MOT_KT * N * N_ETA) * tau_C_output * CURRENT_SCALAR_INIT);

		//Saturate I for our current operational limits -- limit can be reduced by safetyShutoff() due to heating
		if (I > actx->currentOpLimit)
		{
			I = actx->currentOpLimit;
		} else if (I < -actx->currentOpLimit)
		{
			I = -actx->currentOpLimit;
		}

		actx->desiredCurrent = I; 	// demanded mA
		setMotorCurrent(actx->desiredCurrent, DEVICE_CHANNEL);	// send current command to comm buffer to Execute
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
				//rigid1.mn.genVar[7] = (int16_t) (start); // record starting.
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



//****************************************************************************
// Private Function(s)
//****************************************************************************



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
	actx->motorPosNeutral = 0;		// TODO: set this on startup, include in the motor/joint angle transformations


//	actx->axialForce = 0.8*actx->axialForce + 0.2*getAxialForce();	// Filter signal
	actx->axialForce = getAxialForce();
	actx->axialForce = windowAveraging(actx->axialForce);	// Filter signal

	actx->jointTorque = getJointTorque(actx);

	updateJointTorqueRate(actx);

	actx->motorPosRaw = *rigid1.ex.enc_ang;

	actx->motorPos =  *rigid1.ex.enc_ang * RAD_PER_MOTOR_CNT; //counts
	actx->motorVel =  *rigid1.ex.enc_ang_vel * RAD_PER_MOTOR_CNT*SECONDS;	// rad/s TODO: check on motor encoder CPR, may not actually be 16384
	actx->motorAcc = rigid1.ex.mot_acc;	// rad/s/s

	actx->regTemp = rigid1.re.temp;
	actx->motTemp = 0; // REMOVED FOR NOISE ISSUES getMotorTempSensor();
	actx->motCurr = rigid1.ex.mot_current;


	actx->safetyFlag = isSafetyFlag;

	if(actx->regTemp > PCB_TEMP_LIMIT_INIT || actx->motTemp > MOTOR_TEMP_LIMIT_INIT)
	{
		isSafetyFlag = SAFETY_TEMP;
		isTempLimit = 1;
	} else {
		isTempLimit = 0;
	}
}
