/*
 * runningExo-structs.h
 *
 *  Created on: May 25, 2018
 *      Author: Albert Wu
 */

#ifndef RUNNINGEXO_STRUCTS_H
#define RUNNINGEXO_STRUCTS_H
#include "main.h"
#include "math.h"
#include "runningExo-parameters.h"


enum {
	SAFETY_OK				=	0,
	SAFETY_TEMPERATURE		=	1,
	SAFETY_MOTOR_CURRENT	=	2,
	SAFETY_MOTOR_VELOCITY	=	3,
	SAFETY_MOTOR_POSITION	=	4,
	SAFETY_CABLE_TENSION		=	5,
	};


//running state structure
struct runningExoSystemState
{
	//System state
	int8_t state;
	uint32_t timer;
	uint32_t pedometer;
	//Time stamps
	int32_t heelStrikeTime;
	int32_t footFlatTime;
	int32_t toeOffTime;
	uint32_t prevStanceDuration;
	uint32_t prevGaitDuration;
	_Bool running;
	_Bool enableOutput;
	uint32_t disabledPedometer;					//number of disabled steps AFTER all issues are cleared
	#if (CONTROL_STRATEGY == GAIT_TORQUE_TRACKING)
	//add controller specific stuff here
	#endif //CONTROL_STRATEGY == GAIT_TORQUE_TRACKING
};

// parameters to track sensor values, actuate the motors
typedef struct actuation_parameters
	{
	//exoskeleton parameter
	float ankleTorqueMeasured;		//N.m
	float ankleTorqueDesired;		//N.m
	float ankleHeight;				//m
	float ankleVel;					//m/s
	float ankleAcc;					//m/s/s
	float cableTensionForce;		//N
	//motor parameters
	float motorTorqueMeasured;		//N.m
	float motorTorqueDesired;		//N.m
	int32_t motorCurrentMeasured;	//mA
	int32_t motorCurrentDesired;	//mA
	int32_t initialMotorEncPosition;		//rad
	int32_t currentMotorEncPosition;	//motor position [rad]
	float motorRelativeEncRevolution; //motor revolutions relative to the the initial position after motion
	float motorRotationAngle;		 //rad, motor rotation angle relative to the initial position
	float motorAngularVel;		//motor angular velocity [rad/s]
	float motorAngularAcc;		// motor angular acceleration [rad/s/s]

	//safety parameters
	int16_t boardTemperature;	//centidegree, get from temperature sensor on the FlexSEA board
	int8_t safetyFlag;		//identify various safety problems
	int32_t currentOpLimit; // mA, current throttling limit
	}actuation_parameters;


//impedance control structure
typedef struct{

	float m;
	float b;
	float k;
	float thetaDes;

} GainParams;

typedef struct derivativeAverager{
	float prevDerivatives[AVERAGE_FILTER_SAMPLES];
	float sum;
	float average;
	int lastIndex;
}derivativeAverager;

typedef struct derivativeFilter{
	float prevDerivatives[LPF_SAMPLE_COUNT];
	float result;
	int lastIndex;
}derivativeFilter;

float updateDerivativeAverage(derivativeAverager* der,float newValue);
float getDerivativeAverage(derivativeAverager* der);

float updateDerivativeFilter(derivativeFilter* der, float newValue);
float getDerivativeFilter(derivativeFilter* der, float newValue);

#endif /* RUNNINGEXO_STRUCTS_H */

