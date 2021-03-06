
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
#include "median_filter.h"
#include "float.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

//Variables which aren't static may be updated by Plan in the future


int8_t fsm1State = STATE_POWER_ON;

int8_t zeroLoadCell = 0;	//Used to allow re-zeroing of the load cell, ie for testing purposes

// For testing voltage mode instead of current mode
float voltageGain = 1.0;	//tested
float velGain = 1.0; //1.1;	// tested
float indGain = 1.73;	// tested


//****************************************************************************
// Method(s)
//****************************************************************************


/**
 * The ADC reads the motor Temp sensor - MCP9700 T0-92.
 * Return: motTemp(int16_t) - The temperature of the motor in CELCIUS
 */
static int16_t getMotorTempSensor(void)
{
	static int16_t motTemp = 0;
	motTemp = (rigid1.mn.analog[0] * (3.3/4096) - 500) / 10; 	//celsius
	rigid1.mn.mot_temp = motTemp;

	return motTemp;
}

/*
 * Ramp motor current to overcome stiction, no-load current.
 */
static int32_t getMotorNoLoadDeadBandValue(struct act_s *actx, int32_t Ides)
{
	int32_t stictionCurrent = 0.0;
	int32_t motorVel = ( *rigid1.ex.enc_ang_vel );
	int32_t Iadj =0;
	float B = 0	; //actx->controlFF;

	if (Ides < MOT_STICTION_CUR_THRESHOLD && Ides > 0)
	{ // Within ramp region, positive side
		stictionCurrent = Ides * MOT_STICTION_POS_SLOPE - (int32_t)( (float)abs(motorVel) * B);
	}
	else if (Ides >= MOT_STICTION_CUR_THRESHOLD )
	{ // Above threshold, hold constant
		stictionCurrent = MOT_NOLOAD_CURRENT_POS ;
	}
	else if (Ides > -MOT_STICTION_CUR_THRESHOLD && Ides < 0)
	{
		stictionCurrent = Ides * MOT_STICTION_NEG_SLOPE + (int32_t)( (float)abs(motorVel) * B);
	}
	else if (Ides <= -MOT_STICTION_CUR_THRESHOLD )
	{
		stictionCurrent = -MOT_NOLOAD_CURRENT_NEG ;
	}
	else
	{
		stictionCurrent = 0;
	}
	return stictionCurrent;
}

/*
 * Output joint angle, vel, accel in ANG_UNIT, measured from joint zero,
 * Param: actx(struct act_s) - Actuator structure to track sensor values
 * updated Actuator structure:
 * 		actx->jointAngle = new angle
 * 		actx->jointVel = new velocity
 * 		actx->jointAcc = new acceleration
 * 		//saves previous values for next use
 * 		actx->lastJointAngle = angle
 *		TODO:Make lastJointVel part of act_s
 */
static void getJointAngleKinematic(struct act_s *actx)
{
	static float lastJointVel = 0;
	float jointAngleRad = 0;
	if(actx->resetStaticVariables)
	{
		lastJointVel = 0;
	}

	//ANGLEReturn: tauRestoring(float) -
	jointAngleRad = JOINT_ANGLE_DIR * ( JOINT_ZERO + JOINT_ENC_DIR * ( (float) (*(rigid1.ex.joint_ang)) ) )  * RAD_PER_CNT; // (angleUnit)/JOINT_CPR;
	actx->jointAngle = jointAngleRad;

	//VELOCITY
	actx->jointVel = filterJointVelocityButterworth( (actx->jointAngle - actx->lastJointAngle) * SECONDS, actx->resetStaticVariables);

	//ACCEL  -- todo: check to see if this works
	actx->jointAcc = 0.5 * actx->jointAcc + 0.5*( (actx->jointVel - lastJointVel ) * SECONDS);


	actx->jointAngleDegrees = actx->jointAngle * DEG_PER_RAD;
	actx->jointVelDegrees = actx->jointVel * DEG_PER_RAD;

	// Save values for next time through.
	actx->lastJointAngle = actx->jointAngle;
	lastJointVel = actx->jointVel;

}


/*
 * Measure axial force by motor current sensing
 */
float getAxialForceMotorCurrent(struct act_s *actx)
{
	static float uArray[3];
	static float yArray[3];

	float currRead = ( (float) rigid1.ex.mot_current ) * 0.001 * MOT_KT *1.732051*1.20;
	float curr = filterLowPass30(uArray, yArray, currRead);


	return (curr * N_SCREW*N_ETA);
}

/**
 * Output axial force on screw
 * Return: axialForce(float) -  Force on the screw in NEWTONS
 */
static float getAxialForce(struct act_s *actx, int8_t tare)
{
	static int8_t tareState = -1;
	static uint32_t timer = 0;
	static float tareOffset = 0;
	static float forceWindow[MEDIAN_FILTER_WINDOW_SIZE_7];	// store array of current strain values

	float strainReading = 0;
	float axialForce = 0;
	float numSamples = 1000.;


	// Filter the signal
	strainReading = ( (float) rigid1.ex.strain );
	strainReading = medianFilterData7( &strainReading, forceWindow );	// spike rejection, other lowpass didn't help much

	if(actx->resetStaticVariables)
	{
		tare = 1;
	}

	if(actx->resetStaticVariables)
	{
		tare = 1;
	}

	if (tare)
	{	// User input has requested re-zeroing the load cell, ie locked output testing.
		tareState = -1;
		timer = 0;
		tareOffset = 0;
		tare=0;
		for(uint8_t i = 0; i < MEDIAN_FILTER_WINDOW_SIZE_7; ++i)
		{
			forceWindow[i] = 0.0;
		}
	}

	switch(tareState)
	{
		case -1:
			//Tare the balance using average of numSamples readings
			timer++;

			if(timer < numSamples) {
				tareOffset += (strainReading)/(numSamples);
			} else if (timer >= numSamples ) {
				tareState = 0;
				zeroLoadCell = 0;	// turn off global variable for zeroing
				tare = 0;
			}

			break;

		case 0:

			axialForce =  FORCE_DIR * (strainReading - tareOffset) * FORCE_PER_TICK;

			break;

		default:
			//problem occurred
			break;
	}

	return axialForce;
}



/**
 * Linear Actuator Actual Moment Arm, internal units are [mm, rad] to keep internal precision, outputs [m]
 * Param: theta(float) - the angle of the joint in RADIANS
 *
 * Return: projLength(float) - moment arm projected length in [m]
 */
static float getLinkageMomentArm(struct act_s *actx, float theta, int8_t tareState)
{
	static int16_t timerTare = 0;

	float A=0, c = 0, c2 = 0, projLength = 0, CAng = 0, thetaCnt=0;

	if(actx->resetStaticVariables)
	{
		timerTare = 0;
	}

	thetaCnt = JOINT_ANGLE_DIR * ( JOINT_ZERO_ABS + JOINT_ENC_DIR * ( (float) (*(rigid1.ex.joint_ang)) ) )  * RAD_PER_CNT;;

	CAng = M_PI - thetaCnt - MA_TF; 	// angle
	c2 = MA_B2PLUSA2 - MA_TWOAB*cosf(CAng);
	c = sqrtf(c2);

    A = acosf(( MA_A2MINUSB2 - c2 ) / (-2*MA_B*c) );
    projLength = (MA_B * sinf(A)); // [mm] project length, r (in docs) of moment arm.

	actx->c = c;

    if (tareState == 1)
    {
    	timerTare++;
    	if (timerTare > 100)
    	{
			actx->motorPos0 = *rigid1.ex.enc_ang;		// Store current position of motor, as Initial Position [counts].
			actx->c0 = c;

			tareState = 0;
			timerTare = 0;
    	}
    }

    // Force is related to difference in screw position.
    // Eval'd by difference in motor position - neutral position, adjusted by expected position - neutral starting position.
//    actx->screwLengthDelta = (  filterJointAngleButterworth( (float) ( MOTOR_DIRECTION*MOTOR_MILLIMETER_PER_TICK*( ( (float) ( *rigid1.ex.enc_ang - actx->motorPos0) ) ) - (c - actx->c0) ), actx->resetStaticVariables ) );	// [mm]
//    actx->screwLengthDeltaRoman = (float) ( MOTOR_DIRECTION*2.993159602325407e-04*( (float) ( *rigid1.ex.enc_ang - actx->motorPos0) ) - 0.403564715559730 - (actx->croman-actx->c0roman));	// [mm]
    actx->screwLengthDelta =   ( (float) ( MOTOR_DIRECTION*MOTOR_MILLIMETER_PER_TICK*( ( (float) ( *rigid1.ex.enc_ang - actx->motorPos0) ) ) - (c - actx->c0) ) );	// [mm]

    return projLength/1000.; // [m] output is in meters

}



/*
 * Calculate axial force based on displacement of spring measured from motor angle and expected motor angle
 * This function works fairly well. It has as noise diff of about 0.8N, and effectively no delay other than
 * the delay from the filter on teh screwLengthDelta.
 * A small angle approximation is valid so the arcsin is not necessary.
 */
float getAxialForceEncoderCalc(struct act_s *actx)
{

	float force = 0;

//	force =  MOTOR_DIRECTION*( (SPRING_K_D) * asinf( actx->screwLengthDelta / (SPRING_D*1000) ) );	// [N]
	force =  MOTOR_DIRECTION*( (SPRING_K_E) * ( actx->screwLengthDelta/1000.0) );	// [N], small angle approximation
	return force;
}




/*
 * Spring transfer function to map model of spring to calculate actual force in spring
 * This function works fairly well, however it has about a 13ms delay compared to the
 * calculated version above. This has a diff of about 0.5N, compared to 0.8N above
 */
float getAxialForceEncoderTransferFunction(struct act_s *actx, int8_t tare)
{
	static float y[3] = {0.0, 0.0, 0.0};
	static float u[3] = {0.0, 0.0, 0.0};
	static int8_t k = 2;

	static int8_t tareState = -1;
	static uint32_t timer = 0;
	float numSamples = 4900.;

	// shift previous values into new locations
	u[k-2] = u[k-1];
	u[k-1] = u[k];
	// update current state to new values
	u[k] = actx->screwLengthDelta/1000.0; // [m]

	y[k-2] = y[k-1];
	y[k-1] = y[k];

	//TF1
	y[k] = 0.917703269653775*y[k-1] + -27307.8288879291*u[k-1];


	if (tare)
	{	// User input has requested re-zeroing the load cell, ie locked output testing. Turned on externally, turned off internal
		tareState = -1;
		timer = 0;
		tare=0;
	}

	switch(tareState)
	{
		case -1:
			// Wait for transfer function to stabilize
			timer++;

			if(timer >= numSamples)
			{
				tareState = 0;
				zeroLoadCell = 0;
				timer = 0;
			}
			return 0;	// send a zero command until we stabilize.
		case 0:
			return ( y[k] );
		default:
			return ( y[k] );
	}

}


/*
 * Calculate desired motor position, for SEA Force control based on position
 */
float getMotorPositionSEA(struct act_s *actx, float refForce)
{
	float thetaMotor = 0.0;
	thetaMotor = MOTOR_DIRECTION * MOTOR_TICK_PER_METER * (refForce/SPRING_K_E + (actx->c - actx->c0)/1000.0 ) + actx->motorPos0;

	//original
//	thetaMotor = MOTOR_DIRECTION * MOTOR_TICK_PER_MILLIMETER *( SPRING_D_MM	* sinf( refForce * SPRING_D/SPRING_K) + (actx->c - actx->c0) ) + ( (float)actx->motorPos0);
	return thetaMotor;
}

///* NOTE: NOT WORKING DO NOT USE
// * Spring state-space representation to map model of spring to calculate actual force in spring
// */
//float getAxialForceEncoderStateSpace(struct act_s *actx, int8_t tare)
//{
//	static float A[4][4] = {
//			{0, 1, 0, 0},
//			{0, 0, 1, 0},
//			{0, 0, 0, 1},
//			{-0.478579503734352, 1.76911411734673, -2.99408247469697, 2.69009890016747}
//	};
//	static float B[4][1] = {
//			{-36914.8339844512},
//			{12871.1972092885},
//			{40343.7524161401},
//			{39033.9428387263}
//	};
//	static float C[1][4] = {1, 0, 0, 0 };
//	static float D = 0;
//	static float n = 4;
//	static float x[4] = {0, 0, 0, 0};
//	static float y[4] = {0, 0, 0, 0};
//	static float u[4] = {0, 0, 0, 0};
//
//	x[n] = A*x[n-1] + B*u[n-1];
//	y[n-1] = C*x[n-1] + D*u[n-1];
//
//
//	return ( y[n] );
//}


/*
 *  Determine torque at joint due to moment arm and axial force
 *  Param:	actx(struct act_s) -  Actuator structure to track sensor values
 *  Return: torque(float) - joint torque in NEWTON-METERS
 *  //todo: more accurate is to track angle of axial force. maybe at getLinkageMomentArm
 */
static float getJointTorque(struct act_s *actx)
{
	float torque = 0;

	torque = actx->linkageMomentArm * actx->axialForce;

	return torque;
}

/*
 * 	Determine rate of change of motor current
 * 	Param: Act_s
 * 	return: di/dt
 */
static float getMotorCurrentDt(struct act_s *actx)
{
	static float lastCurrent = 0.0;
	static float currentDt = 0.0;
	if(actx->resetStaticVariables)
	{
		lastCurrent = 0.0;
		currentDt = 0.0;
	}

	currentDt = 0.8*currentDt + 0.2 * (actx->motCurr - lastCurrent)*SECONDS; // units [A/sec]
	lastCurrent = actx->motCurr;

	return currentDt;
}


/*
 * Update the joint torque rate in the Actuator structure
 * Param: actx(struct act_s) - Actuator structure to track sensor values
 * updated Actuator structure:
 * 		actx->lastJointTorque = current joint torque rate
 * 		actx->jointTorqueRate = new joint torque rate
 */
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
 *  Param:  tauErr(float) -
 *  Param:  tauCTotal(float) -
 *  Param:  tauCOutput(float) -
 *  Return: saturation(bool) - whether the output is saturating or not
 */
bool integralAntiWindup(float tauErr, float tauCTotal, float tauCOutput) {
	//Anti-windup clamp for Integral Term of PID
	int8_t inSatLimit = 0;
	if (tauCTotal == tauCOutput){
		inSatLimit = 0;
	} else {
		inSatLimit = 1;
	}

	//check if the error is increasing  in teh direction of saturation
	int8_t errSign = 0;
	if (tauErr > 0 && tauCTotal > 0) {
		errSign = 1;
	} else if (tauErr < 0 && tauCTotal < 0) {
		errSign = 1;//-1;
	} else {
		errSign = 0;
	}

	// if saturated and error is going in that direction use antiwindup.
	if (inSatLimit && errSign) {
		return 1;
	} else {
		return 0;
	}
//	return inSatLimit;
}

/*
 * Check for angle limits and apply virtual unidirectional spring/dampers
 *  Param:	actx(struct act_s) - Actuator structure to track sensor values
 *  Return: tau(float) - virtual impedance torque value
 */
//TODO: check that damping ratio is in the correct direction.
float actuateAngleLimits(Act_s *actx){
	float bumperTorq=0.0;
	float tauK = 0; float tauB = 0;
	float thetaDelta =0;

	// apply unidirectional spring and damper
	if ( actx->jointAngleDegrees < JOINT_MIN_SOFT_DEGREES ) {
		thetaDelta = (JOINT_MIN_SOFT_DEGREES - actx->jointAngleDegrees);
//		thetaDelta = filterJointAngleLimitOutputButterworth(JOINT_MIN_SOFT_DEGREES - actx->jointAngleDegrees);
		tauK = JOINT_SOFT_K * (thetaDelta);

		if (actx->jointVelDegrees < 0)
		{
			//		tauB = -JOINT_SOFT_B * (actx->jointVelDegrees);
			tauB = -JOINT_SOFT_B * (actx->jointVelDegrees);
		} else
		{
			tauB = 0.0;
		}

	} else if ( actx->jointAngleDegrees > JOINT_MAX_SOFT_DEGREES) {
		thetaDelta = (JOINT_MAX_SOFT_DEGREES - actx->jointAngleDegrees);
//		thetaDelta = filterJointAngleLimitOutputButterworth(JOINT_MAX_SOFT_DEGREES - actx->jointAngleDegrees);

		tauK = JOINT_SOFT_K * (thetaDelta);


		if (actx->jointVelDegrees > 0)
		{
			tauB = -JOINT_SOFT_B * (actx->jointVelDegrees);
		} else
		{
			tauB = 0.0;
		}

	} else
	{
		tauK = 0.0;
		tauB = 0.0;
	}

	bumperTorq = tauK + tauB;
	return bumperTorq;
}

/*
 * Check current direction and provide baseline current to handle
 * the No load current requirement to get motor moving.
 * Param: desCurr(float) - 
 */
int32_t noLoadCurrent(float desCurr) {
	if (desCurr > 0) {
		return (int32_t) MOT_NOLOAD_CURRENT_POS;
	} else if (desCurr < 0) {
		return (int32_t) MOT_NOLOAD_CURRENT_NEG;
	} else {
		return (int32_t) 0;
	}
}


/*
 * Calculate required motor torque, based on joint torque. set motor torque
 * 			Motor Torque request, or maybe current
 * Param:	actx(struct act_s) - Actuator structure to track sensor values
 * Param:	tauDes(float_ - TODO:find out what this param is
 * 			TODO: find out what this is (old param?) ->tor_d, desired torque at joint [Nm]
 * Updates to Actuator structure:
 * 			actx->tauDes = tauDes(parameter);
 * 			actx->desiredCurrent = new desired current
 *
 */
void setMotorTorque(struct act_s *actx)
{
	float tauFF = 0.0;
	float tauC = 0.0;
	float tauCCombined = 0.0;
	float N = 0.0;
	float Icalc = 0.0;
	int32_t INoLoad = 0.0;
	float tauNotch = 0.0;

	//Angle Limit bumpers
	float refTorque = actx->tauDes + actuateAngleLimits(actx);
	actx->tauMeas = actx->jointTorque;

	N = actx->linkageMomentArm * N_SCREW;	// gear ratio

	//	// LPF Reference term to compensate for FF delay
//	refTorque = getReferenceLPF(refTorque, actx->resetStaticVariables);

	// tauNotch Filter
	tauNotch = CONTROL_SCALER_NOTCH * getNotchFilter(refTorque);

	// Feed Forward term
//	tauFF = refTorque * 1.0/(N*N_ETA*MOT_KT) * actx->controlFF;
	tauFF = CONTROL_SCALER_FF * getReferenceLPF(refTorque, actx->resetStaticVariables); // Put delay on FF since it's a simple FF, align with feedback term

	//PID around joint torque
	tauC = getCompensatorPIDOutput(refTorque, actx->jointTorque, actx);

	tauCCombined = tauC + (tauNotch + tauFF);


	//Saturation limit on Torque
	if (tauCCombined > ABS_TORQUE_LIMIT_INIT) {
		tauCCombined = ABS_TORQUE_LIMIT_INIT;
	} else if (tauCCombined < -ABS_TORQUE_LIMIT_INIT) {
		tauCCombined = -ABS_TORQUE_LIMIT_INIT;
	}

	// motor current signal
	Icalc = (tauCCombined * 1.0/(N*N_ETA*MOT_KT) )  ;	// Reflect torques to Motor level
//	INoLoad =  getMotorNoLoadDeadBandValue(actx, (Icalc * CURRENT_SCALAR_INIT) );


	int32_t I = (int32_t) ( (Icalc * CURRENT_SCALAR_INIT * CURRENT_GAIN_ADJUST) );

	//Saturate I to our current operational limits -- limit can be reduced by safetyShutoff() due to heating
	if (I > actx->currentOpLimit)
	{
		I = actx->currentOpLimit;
	} else if (I < -actx->currentOpLimit)
	{
		I = -actx->currentOpLimit;
	}

	actx->desiredCurrent = I; 	// demanded mA

	// Turn off motor power if using a non powered mode.
#if !defined(NO_POWER)
	if(actx->resetStaticVariables)
	{
		I = 0.0;
		actx->desiredCurrent = I; 	// demanded mA
	}else
	{
		setMotorCurrent(actx->desiredCurrent, DEVICE_CHANNEL);	// send current command to comm buffer to Execute
	}
#endif

	//variables used in cmd-rigid offset 5, for multipacket
	rigid1.mn.userVar[5] = actx->tauMeas*1000;	// x1000 is for float resolution in int32
	rigid1.mn.userVar[6] = actx->tauDes*1000;

}

/*
 * Calculate required motor torque, based on joint torque. set motor torque
 * 			Motor Torque request, or maybe current
 * Param:	actx(struct act_s) - Actuator structure to track sensor values
 * Param:	tauDes(float_ - TODO:find out what this param is
 * 			TODO: find out what this is (old param?) ->tor_d, desired torque at joint [Nm]
 * Updates to Actuator structure:
 * 			actx->tauDes = tauDes(parameter);
 * 			actx->desiredCurrent = new desired current
 *
 */
void setMotorTorqueDOB(struct act_s *actx)
{
	static float DOB = 0.0;
	float DOBref = 0.0;
	float DOBerr = 0.0;

//	//Angle Limit bumpers
	float refTorque = actx->tauDes + actuateAngleLimits(actx);
	actx->tauMeas = actx->jointTorque;

	// DOB Ref Term
	DOBref = getDOBFF(refTorque);

	DOBerr = DOBref - DOB;

	DOB = getDOBQtd(DOBerr) - getDOBInvQtd(actx->tauMeas);

	actx->tauDes = actx->controlScaler*DOBerr;

	rigid1.mn.genVar[9] = (int16_t) (refTorque			*100.0	);


	setMotorTorque(actx);

}



/*
 * Feedforward Term based on system Identification and simulation
 * Input is reference
 * return:	torque command value to the motor driver
 */
float getFeedForwardTerm(float refTorque)
{
	static float y[3] = {0, 0, 0};
	static float u[3] = {0, 0, 0};
	static int8_t k = 2;

	// shift previous values into new locations
	u[k-2] = u[k-1];
	u[k-1] = u[k];
	// update current state to new values
	u[k] = refTorque;			// [Nm]

	y[k-2] = y[k-1];
	y[k-1] = y[k];

	//fc = 20; LPF, Newer trial
	y[k] = 1.4608054*y[k-1] - 0.5334881*y[k-2] + 3.6396485e+02*u[k] + -7.2676858e+02*u[k-1] + 3.6287459e+02*u[k-2];

	return ( y[k] );
}


/*
 * LPF on reference due to Feedforward Term
 * Input is reference
 * return:	torque command value to the motor driver
 */
float getReferenceLPF(float refTorque, int8_t reset)
{
	static float y[3] = {0, 0, 0};
	static float u[3] = {0, 0, 0};
	static int8_t k = 2;

	if(reset)
	{
		u[2] = 0.0,	u[1] = 0.0,	u[0] = 0.0;
		y[2] = 0.0,	y[1] = 0.0,	y[0] = 0.0;
	}

	// shift previous values into new locations
	u[k-2] = u[k-1];
	u[k-1] = u[k];
	// update current state to new values
	u[k] = refTorque;			// [Nm]

	y[k-2] = y[k-1];
	y[k-1] = y[k];

	//fc = 30
	y[k] = 1.7349051*y[k-1] - 0.7660021*y[k-2] + 0.0155485*u[k-1] + 0.0155485*u[k-2];
	return ( y[k] );
}

/*
 * LPF on reference due to Feedforward Term
 * Input is reference
 * return:	torque command value to the motor driver
 */
float getReferenceLPFWalking(float refTorque, int8_t reset)
{
	static float y[3] = {0, 0, 0};
	static float u[3] = {0, 0, 0};
	static int8_t k = 2;

	if(reset)
	{
		u[2] = 0.0,	u[1] = 0.0,	u[0] = 0.0;
		y[2] = 0.0,	y[1] = 0.0,	y[0] = 0.0;
	}

	// shift previous values into new locations
	u[k-2] = u[k-1];
	u[k-1] = u[k];
	// update current state to new values
	u[k] = refTorque;			// [Nm]

	y[k-2] = y[k-1];
	y[k-1] = y[k];

	//fc = 30
	y[k] = 1.7349051*y[k-1] - 0.7660021*y[k-2] + 0.0155485*u[k-1] + 0.0155485*u[k-2];
	return ( y[k] );
}

/*
 * LPF on reference due to Feedforward Term
 * Input is reference
 * return:	torque command value to the motor driver
 */
float getTorqueErrorLPF(float refTorque, int8_t reset)
{
	static float y[3] = {0, 0, 0};
	static float u[3] = {0, 0, 0};
	static int8_t k = 2;

	if(reset)
	{
		u[2] = 0.0,	u[1] = 0.0,	u[0] = 0.0;
		y[2] = 0.0,	y[1] = 0.0,	y[0] = 0.0;
	}

	// shift previous values into new locations
	u[k-2] = u[k-1];
	u[k-1] = u[k];
	// update current state to new values
	u[k] = refTorque;			// [Nm]

	y[k-2] = y[k-1];
	y[k-1] = y[k];


	//fc = 30
	y[k] = 1.7349051*y[k-1] - 0.7660021*y[k-2]
			 + 0.0155485*u[k-1] + 0.0155485*u[k-2];

	return ( y[k] );
}

/*
 * Feedforward Term based on system Identification and simulation
 * Input is reference
 * return:	torque command value to the motor driver
 */
float getNotchFilter(float refTorque)
{
	static float y[3] = {0, 0, 0};
	static float u[3] = {0, 0, 0};
	static int8_t k = 2;

	// shift previous values into new locations
	u[k-2] = u[k-1];
	u[k-1] = u[k];
	// update current state to new values
	u[k] = refTorque;			// [Nm]

	y[k-2] = y[k-1];
	y[k-1] = y[k];

	//fc = 20;
	y[k] = 1.8227334*y[k-1] - 0.8371821*y[k-2]
					+ 72.3532614*u[k] + -144.4757007*u[k-1] + 72.1365268*u[k-2];

	return ( y[k] );
}

/*
 * DOB applies a LPF on reference and output term and combines them
 * Input is reference and measured output torque
 * return:	torque command value to the motor driver
 */
float getDOB(float refTorque, float measTorque)
{
	return ( getDOBInvQtd(measTorque) - getDOBQtd(refTorque) );
}

/*
 * DOB applies a LPF on reference signal
 * Input is reference
 * return:	torque command value to the motor driver
 */
float getDOBQtd(float refTorque)
{
	static float y[3] = {0, 0, 0};
	static float u[3] = {0, 0, 0};
	static int8_t k = 2;

	// shift previous values into new locations
	u[k-2] = u[k-1];
	u[k-1] = u[k];
	// update current state to new values
	u[k] = refTorque;			// [Nm]

	y[k-2] = y[k-1];
	y[k-1] = y[k];

//	//fc = 50
//	y[k] = 1.2484569*y[k-1] - 0.3896611*y[k-2]
//			 + 0.0706021*u[k-1] + 0.0706021*u[k-2];
	//fc = 30
	y[k] = 1.7349051*y[k-1] - 0.7660021*y[k-2]
			 + 0.0155485*u[k-1] + 0.0155485*u[k-2];

	return ( y[k] );
}

/*
 * DOB multiplies Inverted Plant Model to a LPF on measured output
 * Input is measured from output
 * return:	torque command value to the motor driver
 */
float getDOBInvQtd(float refTorque)
{
	static float y[3] = {0, 0, 0};
	static float u[3] = {0, 0, 0};
	static int8_t k = 2;

	// shift previous values into new locations
	u[k-2] = u[k-1];
	u[k-1] = u[k];
	// update current state to new values
	u[k] = refTorque;			// [Nm]

	y[k-2] = y[k-1];
	y[k-1] = y[k];

//	//fc = 50hz * PlantInverse
//	y[k] = 1.2484568*y[k-1] - 0.3896611*y[k-2]
//			+ 37.4555422*u[k] + -73.4157594*u[k-1] + 36.0785346*u[k-2];

	//fc = 30hz * PlantInverse
	y[k] = 1.7349051*y[k-1] - 0.7660021*y[k-2]
			+ 8.2487061*u[k] + -16.1681018*u[k-1] + 7.9454524*u[k-2];

	return ( y[k] );
}

/*
 * DOB applies a LPF on reference
 * Input is reference
 * return:	torque command value to the motor driver
 */
float getDOBFF(float refTorque)
{
	static float y[3] = {0, 0, 0};
	static float u[3] = {0, 0, 0};
	static int8_t k = 2;

	// shift previous values into new locations
	u[k-2] = u[k-1];
	u[k-1] = u[k];
	// update current state to new values
	u[k] = refTorque;			// [Nm]

	y[k-2] = y[k-1];
	y[k-1] = y[k];

//	//fc = 20hz
//	y[k] = 1.6564084*y[k-1] - 0.6859222*y[k-2]
//			+ 7.8287683*u[k] + -15.3449912*u[k-1] + 7.5409531*u[k-2];

	//fc = 20hz
	y[k] = 1.8227334*y[k-1] - 0.8371821*y[k-2]
			+ 3.8326400*u[k] + -7.5122708*u[k-1] + 3.6917377*u[k-2];

	return ( y[k] );
}


/*
 * Control PID Controller
 * input:	Act_s structure
 * return:	desired torque signal to motor
 */
#define MAX_INTEGRAL_ERROR		70

float getCompensatorPIDOutput(float refTorque, float sensedTorque, Act_s *actx)
{
	static float tauErrLast = 0.0;
	static float tauErrInt = 0.0;
	static int8_t tauErrIntWindup = 0;
	static float tauCCombined = 0.0;
	static float tauCOutput = 0.0;

	if(actx->resetStaticVariables)
	{ // Reset static variables.
		tauErrLast = 0.0, tauErrInt = 0.0;
		tauErrIntWindup = 0;
		tauCCombined = 0.0, tauCOutput = 0.0;
	}

	// Error is torque at the joint
	float tauErr = refTorque - sensedTorque;		// [Nm]
	float tauErrDot = (tauErr - tauErrLast);		// [Nm/s]

	tauErrDot = getTorqueErrorLPF(tauErrDot, actx->resetStaticVariables);	// apply filter to Derivative, Try 30Hz
	tauErrInt = tauErrInt + tauErr;
	tauErrLast = tauErr;

	// Anti-windup
	if(tauErrInt >= MAX_INTEGRAL_ERROR)
	{
		tauErrInt = MAX_INTEGRAL_ERROR;
	} else if (tauErrInt <= -MAX_INTEGRAL_ERROR)
	{
		tauErrInt = -MAX_INTEGRAL_ERROR;
	}

	float tauC = tauErr*actx->torqueKp + tauErrDot*actx->torqueKd + tauErrInt*actx->torqueKi;	// torq Compensator, Ki-reduced

	return tauC;
}


//Used for testing purposes. See state_machine
/*
 * Simple Biom controller
 * Param:	actx(struct act_s) - Actuator structure to track sensor values
 * Param:	thetaSet(float) - desired theta in DEGREES
 * Param:	k1(float) - impedance parameter
 * Param:	b(float) - impedance parameter
 * return: 	torD(float) -  desired torque
 */
float biomCalcImpedance(Act_s *actx,float k1, float b, float thetaSet)
{
	return k1 * (thetaSet - actx->jointAngleDegrees ) - b*actx->jointVelDegrees;

}

// Preferred Nomenclature Impedance Command
/*
 * Simple Impedance Controller
 * Param:	actx(struct act_s) - Actuator structure to track sensor values
 * Param:	thetaSet(float) - desired theta in DEGREES
 * Param:	k1(float) - impedance parameter
 * Param:	b(float) - impedance parameter
 * return: 	torD(float) -  desired torque
 */
float getImpedanceTorque(Act_s *actx, float k1, float b, float thetaSet)
{
//	float thetaDelta = filterJointAngleOutputButterworth(thetaSet - actx->jointAngleDegrees);
	float thetaDelta = (thetaSet - actx->jointAngleDegrees);

	return k1 * (thetaDelta ) - b*actx->jointVelDegrees;
}

// Preferred Nomenclature Impedance Command
/*
 * Simple Impedance Controller
 * Param:	actx(struct act_s) - Actuator structure to track sensor values
 * Param:	thetaSet(float) - desired theta in DEGREES
 * Param:	k1(float) - impedance parameter
 * Param:	b(float) - impedance parameter
 * return: 	torD(float) -  desired torque
 */
float getImpedanceTorqueParams(Act_s *actx, GainParams *gParams)
{
	float thetaDelta = filterJointAngleOutputButterworth(gParams->thetaDes - actx->jointAngleDegrees);
	return gParams->k1 * (thetaDelta ) - gParams->b*actx->jointVelDegrees;
}



//Used for testing purposes. See state_machine
/*
 * Quadratic Impedance Controller
 * Param:	actx(struct act_s) - Actuator structure to track sensor values
 * Param:	thetaSet(float) - desired theta in DEGREES
 * Param:	k1(float) - impedance parameter
 * Param:	b(float) - impedance parameter
 * Param:	k2(float) - quadratic impedance parameter
 * return: 	torD(float) -  desired torque
 */
float getImpedanceTorqueQuadratic(Act_s *actx, float k1, float b, float thetaSet, float k2)
{
	float thetaDelta = thetaSet - actx->jointAngleDegrees;
	return k1 * ( thetaDelta ) + k2 * powf( thetaDelta, 2) - b*actx->jointVelDegrees;
}

/*
 *  Initialize Current controller on Execute, Set initial gains.
 */
void mitInitCurrentController(Act_s *actx) {

	actx->tauDes   = 0.0;
	actx->torqueKp = CURRENT_TORQ_KP_INIT;
	actx->torqueKi = CURRENT_TORQ_KI_INIT;
	actx->torqueKd = CURRENT_TORQ_KD_INIT;

	actx->currentOpLimit = CURRENT_LIMIT_INIT; //CURRENT_ENTRY_INIT;
	setControlMode(CTRL_CURRENT, DEVICE_CHANNEL);
	writeEx[DEVICE_CHANNEL].setpoint = 0;			// wasn't included in setControlMode, could be safe for init
	setControlGains(ACTRL_I_KP_INIT, ACTRL_I_KI_INIT, ACTRL_I_KD_INIT, 0, DEVICE_CHANNEL);
}

void mitInitOpenController( Act_s *actx) {

	actx->tauDes   = 0.0;
	actx->torqueKp = VOLTAGE_TORQ_KP_INIT;
	actx->torqueKi = VOLTAGE_TORQ_KI_INIT;
	actx->torqueKd = VOLTAGE_TORQ_KD_INIT;
	actx->controlScaler = 0.0;	//only used when testing, can be dangerous

	actx->currentOpLimit = CURRENT_LIMIT_INIT; //CURRENT_ENTRY_INIT;
	actx->voltageOpLimit = VOLTAGE_LIMIT_INIT;

	setControlMode(CTRL_OPEN, DEVICE_CHANNEL);
	writeEx[DEVICE_CHANNEL].setpoint = 0;
	setControlGains(0, 0, 0, 0, DEVICE_CHANNEL);

}

/*
 *  Initialize Current controller on Execute, Set initial gains.
 */
void mitInitPositionController(void) {
	act1.currentOpLimit = CURRENT_LIMIT_INIT; //CURRENT_ENTRY_INIT;
	setControlMode(CTRL_POSITION, DEVICE_CHANNEL);
	writeEx[DEVICE_CHANNEL].setpoint = *rigid1.ex.enc_ang;			// wasn't included in setControlMode, could be safe for init
	setControlGains(POS_CTRL_GAIN_KP, POS_CTRL_GAIN_KI, POS_CTRL_GAIN_KD, 0, DEVICE_CHANNEL);
}

/*
 * Set Motor torques open loop.  Used for System characterization.
 * Param:	actx(struct act_s) - Actuator structure to track sensor values
 * Param:	tauDes(float_ - TODO:find out what this param is
 * Updates to Actuator structure:
 * 			actx->tauDes = tauDes(parameter);
 * 			actx->desiredCurrent = new desired current
 *
 */
void setMotorTorqueOpenLoop(struct act_s *actx, float tauDes, int8_t motorControl)
{
	static int8_t isTransition = 0;
	static int8_t lastMotorControl = 0;

	if(lastMotorControl != motorControl)
	{
		isTransition = 1;
	}

	//Angle Limit bumpers
	actx->tauDes = tauDes;// + actuateAngleLimits(actx);
	actx->tauMeas = actx->jointTorque;

	float N = actx->linkageMomentArm * N_SCREW;	// gear ratio

	// motor current signal
	float I = ( 1.0/(MOT_KT * N ) * actx->tauDes );		// [A]
//	float V = (I * MOT_R) * voltageGain + ( MOT_KT_TOT * actx->motorVel * velGain) + (actx->motCurrDt * MOT_L * indGain ); // [V] this caused major problems? ==> ;

	int32_t ImA = (int32_t) ( I * CURRENT_SCALAR_INIT );
	int32_t VmV = (int32_t) ( CURRENT_SCALAR_INIT * ( (I * MOT_R*1.732) + (actx->motorVel * MOT_KT) ) ); //+ (actx->motCurrDt * MOT_L)

//	rigid1.mn.genVar[6] = (int16_t) (VmV);
//	rigid1.mn.genVar[7] = (int16_t) (ImA);

	//Saturate I for our current operational limits -- limit can be reduced by safetyShutoff() due to heating
//	if (I > actx->currentOpLimit)
//	{
//		I = actx->currentOpLimit;
//	} else if (I < -actx->currentOpLimit)
//	{
//		I = -actx->currentOpLimit;
//	}

	switch(motorControl)
	{
		case 0: // current control
			if(isTransition)
			{
				mitInitCurrentController(actx);
				isTransition = 0;
			}
			setMotorCurrent( ImA , DEVICE_CHANNEL);	// send current [mA] command to comm buffer to Execute
			lastMotorControl = motorControl;
			actx->desiredCurrent = I;	// demanded mA

			break;

		case 1:	// voltage control
			if(isTransition)
			{
				mitInitOpenController(actx);
				isTransition = 0;
			}
			setMotorVoltage( VmV, DEVICE_CHANNEL); // consider open volt control
			lastMotorControl = motorControl;
			break;

		default:
			break;
	}

}

/*
 * Compute where we are in frequency sweep
 * Param: 	omega		(float) - ____ in RADIANS PER SECOND
 * Param: 	t			(float) - time in SECONDS
 * Param:	amplitude	amplitude of signal, this is 0 to peak value, not peak to peak
 * Param:	dcBias		DC Bias to preload actuator drivetrain
 * Param:	noiseAmp	noise this is a noise scaling value to put ontop of signal
 * Return: torqueSetPoint(float) - torque adjusted by frequency of operation
 */
float torqueSystemIDFrequencySweep(float omega, uint32_t signalTimer, float amplitude, float dcBias, float noiseAmp,int16_t begin){
	static float prevOmega = 0.0, prevAmp = 0., prevDC = 0., prevNoise = 0., lastSignal = 0.0;
	static float signal = 0.0, testSignal = 0;
	static float localTime = 0.0;
	static int8_t holdingOnTransition = 0;
	float testCase = 1000;

	if (begin)
	{
		float t = ((float)(signalTimer - localTime)) / ( (float)SECONDS );
	float testTime = fmodf(signalTimer, localTime);


	// only make a transition at a zero crossing and if the input has changed
	// Check for Transition
	if ( (prevOmega != omega) || (prevAmp != amplitude) || (prevDC != dcBias) || (prevNoise != noiseAmp) )
	{
		holdingOnTransition = 1;
		if (prevOmega == 0)
			testCase = 0;
		else
			testCase = fmodf(t, (2*M_PI)/prevOmega);


		//if we're at a transition point, now update all values
		if ( (testCase <= 0.01 ) && ( testSignal >= 0 ) )
		{
			holdingOnTransition = 0;
			localTime = signalTimer;
			prevOmega = omega;
			prevAmp = amplitude;
			prevDC = dcBias;
			prevNoise = noiseAmp;

		}

	}

	// if waiting for an elegant transition, keep going with the prev setting until ready to change over.
	if (holdingOnTransition == 1)
	{
		signal = prevDC + prevAmp * sinf(prevOmega * ( t  ) ) + prevNoise*( ((float)rand()) / ((float)RAND_MAX) );
	}
	else if(holdingOnTransition == 1 && testTime < 0.002 ) // Transition to new one
	{
		signal = 0.0;
	} else // if not a clean transition send nothing
	{
		signal = dcBias + amplitude * sinf(omega * ( t  ) ) + noiseAmp*( ((float)rand()) / ((float)RAND_MAX) );
	}

	testSignal = signal-lastSignal;
	lastSignal = signal;	// save to test condition in transition


	return ( signal );
	}
	else
		return 0;
}

/*
 * Compute where we are in frequency sweep
 * NOTE: dcBias is manually set, then allow some waiting period before and after for test.
 * 			Need to manually stop the test else it will run again.
 * Param: 	omega		(float) - ____ in RADIANS PER SECOND
 * Param: 	t			(float) - time in SECONDS
 * Param:	amplitude	amplitude of signal, this is 0 to peak value, not peak to peak
 * Param:	dcBias		DC Bias to preload actuator drivetrain
 * Param:	noiseAmp	noise this is a noise scaling value to put ontop of signal
 * Param: 	chirpType	0:constant freq, 1:linear ramp, 2:exponential
 * Param:	k 			chirpyness, rate of change of frequency
 * Return: torqueSetPoint(float) - torque adjusted by frequency of operation
 */
float torqueSystemIDFrequencySweepChirp(float initFreq,  float finalFreq, float testLength, float amplitude, float dcBias, float noiseAmp, int16_t chirpType, int16_t running){
	static float omega = 0.0, signal = 0.0, initOmega, finalOmega;
	static float k = 0.0, biasHold = 0.0;
	static int8_t initialize = -1, lastRunning = 0, done = 0;
	static uint32_t signalTimer = 0;
	static uint16_t waitingPeriod = 500;

	float t = 0.0;

	initOmega = 2 * initFreq * 2 * M_PI;	// not sure why, but frequency is off by 2x
	finalOmega = 2* finalFreq * 2* M_PI;	// not sure why, but frequency is off by 2x

	if (lastRunning != running)
	{
		initialize = -2;
		signalTimer = 0;
		biasHold = dcBias;
	}


	// Waiting Period to get started.
	if ( initialize < -1 && signalTimer < waitingPeriod)
	{
		t = 0;
	}
	else if(initialize < -1)	// waiting period is up, move to next step
	{
		signalTimer = 0;
		initialize 	= -1;
		t = 0;
		done = 0;
	} else
	{
		if (running && ( chirpType == 0) ) 	// just keep running if manual mode
		{
			t = ((float)(signalTimer)) / ( (float)SECONDS );
		}
		else if (running && ( signalTimer <= testLength))	// stop after set testLength time
		{
			t = ((float)(signalTimer)) / ( (float)SECONDS );
		}
		else 	// turn off everything.
		{
			t = 0.0;
			noiseAmp = 0;
			running = 0;
			done = 1;
		}
	}

	//Adjust frequency sweep type
	switch(chirpType)
	{
		case 0:	// manual mode
		{
			if (initialize == -1)
			{
				k = 0.0;
				initialize = 0;
			}
			omega = initOmega;
			break;
		}
		case 1: // linear chirp
		{
			if (initialize == -1)
			{
				k = (finalOmega - initOmega)/(testLength/SECONDS);
				initialize = 0;
			}
			omega = initOmega + k*t;
			break;
		}
		case 2: // exponential chirp
		{
			if (initialize == -1)
			{
				k = powf( (finalOmega/initOmega), 1/(testLength/SECONDS) );
				initialize = 0;
			}
			omega = initOmega * powf(k, t);
			break;
		}
		default:
			break;
	}


	signal = dcBias + amplitude * sinf(omega * t) + noiseAmp*( ((float)rand()) / ((float)RAND_MAX) );

	lastRunning = running;
	signalTimer++;

	return ( signal );
}



/*
 * Step through series of data points to set torque
 * Pseudo Random Binary for systemID
 * Start when received signal
 * Return: torqueSetPoint(float) - TODO:find out what this value represents
 */
float torqueSystemIDPRBS(void)
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


void setActuatorTestingTorque(struct act_s *actx, struct actTestSettings *testInput)
{ // Used to test actuator
	float tor = 0.0;
	if ( fabs(testInput->inputTorq) > 0)
	{
		tor = testInput->inputTorq;
	}
	else
	{
		tor = getImpedanceTorque(&act1, testInput->inputK, testInput->inputB, testInput->inputTheta);
	}
	actx->tauDes = tor;
}

void setActuatorStepResponse(Act_s *actx, ActTestSettings *testInput)
{
	float tor = 0.0;
	testInput->offTime = 1000 - testInput->onTime;	// save an input, just use duty cycle, 1000ms


	if(testInput->begin >= 1)
	{
		if(testInput->timer < testInput->onTime)
		{
			tor = testInput->amplitude;
			testInput->timer++;
		} else if (testInput->timer < (testInput->onTime + testInput->offTime) )
		{
			tor = 0.0;
			testInput->timer++;
		} else
		{
			tor = 0.0;
			testInput->timer = 0;
		}
		actx->tauDes = tor + testInput->dcBias;
	} else
	{
		testInput->timer = 0;
		actx->tauDes = 0.0;
	}
}

float getTorqueSystemIDFrequencySweepChirp( struct actTestSettings *testInput)
{
	float tor = torqueSystemIDFrequencySweepChirp( testInput->freq,
						testInput->freqFinal, testInput->freqSweepTime,
						testInput->amplitude, testInput->dcBias, testInput->noiseAmp,
						testInput->chirpType, testInput->begin);
	return tor;
}

/*
 * Output a sinusoidal theta trajectory
 */
float getSinusoidalAngle( struct actTestSettings *testInput)
{
	float theta = torqueSystemIDFrequencySweepChirp( testInput->freq,
						testInput->freq, testInput->freqSweepTime,
						testInput->amplitude, testInput->dcBias, 0,
						0, testInput->begin);
	return theta;
}


//****************************************************************************
// Private Function(s)
//****************************************************************************


/*
 * Collect all sensor values and update the actuator structure.
 * Throws safety flags on Joint Angle, Joint Torque, since these functions look at transformed values.
 * Param:	actx(struct act_s) - Actuator structure to track sensor values
 */
void updateSensorValues(struct act_s *actx)
{
	getJointAngleKinematic(actx);

	actx->linkageMomentArm = getLinkageMomentArm(actx, actx->jointAngle, zeroLoadCell);

	actx->axialForce = getAxialForce(actx, zeroLoadCell); //filtering happening inside function
	actx->axialForceEnc = getAxialForceEncoderCalc(actx);

	actx->jointTorque = getJointTorque(actx);
	updateJointTorqueRate(actx);

	actx->jointPower = (actx->jointTorque * actx->jointVel); // [W]

	float jointPwrIncrement = actx->jointPower/SECONDS;
	if( (jointPwrIncrement + actx->jointEnergy > FLT_MAX) || (jointPwrIncrement + actx->jointEnergy < FLT_MIN) )
	{ // do nothing, we would overflow
	}
	else
	{ // it's safe to do the integration
		actx->jointEnergy = actx->jointEnergy + jointPwrIncrement;	// [J]
	}

	actx->motorPosRaw = *rigid1.ex.enc_ang;		// [counts]
	actx->motorPos =  ( (float) *rigid1.ex.enc_ang ) * RAD_PER_MOTOR_CNT; // [rad]
	actx->motorVel =  ( (float) *rigid1.ex.enc_ang_vel ) * RAD_PER_MOTOR_CNT*SECONDS;	// rad/s TODO: check on motor encoder CPR, may not actually be 16384
	actx->motorAcc = rigid1.ex.mot_acc;	// rad/s/s
//	actx->motorAcc = filterAccelButterworth15Hz(actx->motorAcc);

	actx->regTemp = rigid1.re.temp;
	actx->motTemp = 0; // REMOVED FOR NOISE ISSUES getMotorTempSensor();
	actx->motCurr = rigid1.ex.mot_current;
	actx->motCurrDt = getMotorCurrentDt(actx);

	actx->motorPower = ( (float) (rigid1.ex.mot_current * rigid1.ex.mot_volt) ) * 0.000001; // [W]
	float motorPwrIncrement = actx->motorPower/SECONDS;
	if( (motorPwrIncrement + actx->motorEnergy > FLT_MAX) || (motorPwrIncrement + actx->motorEnergy < FLT_MIN) )
	{ // do nothing, we would overflow
	}
	else
	{ // it's safe to do the integration
		actx->motorEnergy = actx->motorEnergy + motorPwrIncrement;	// [J]
	}

	actx->efficiencyInstant = actx->jointPower / ( (actx->motorPower) );

	actx->safetyFlag = getSafetyFlags(); //todo: I don't think this is in use anymore MC

}


