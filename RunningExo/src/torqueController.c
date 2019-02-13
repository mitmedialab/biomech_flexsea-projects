/*
 * torqueController.c
 * Feedforward/feedback controller using FlexSEA's voltage control functionality
 *  Created on: May 25, 2018
 *      Author: Albert Wu
 */
#include "stdbool.h"
#include <math.h>
#include "flexsea_global_structs.h"
#include <flexsea_system.h>
#include "flexsea_cmd_calibration.h"
#include <flexsea_comm.h>
#include "flexsea_board.h"
#include "user-mn-Rigid.h"
#include "cmd-ActPack.h"
#include "cmd-Rigid.h"

#include "torqueController.h"
#include "user-mn.h"
#include "user-mn-ActPack.h"
#include "flexsea_sys_def.h"
#include "flexsea_user_structs.h"
#include "user-mn-RunningExo.h"
#include "runningExo-structs.h"
#include "runningExo-parameters.h"

derivativeAverager derAvg={
	.sum=0,.average=0,.lastIndex=0
};

derivativeFilter derFil={
	.lastIndex=0
};

void setAnkleTorque(float torqueReference, actuation_parameters* act_para,  _Bool feedFoward, _Bool feedBack)
{
	//Shutdown feedback if below minimum feedback position
	if(act_para->currentMotorEncPosition< FEEDBACK_POS_MIN && feedBack)
	{
		setAnkleTorque(torqueReference, act_para, feedFoward, 0);
		return;
	}

	//TODO: Consider controller architecture of feedback and feedforward blocks
	#ifndef MOTOR_FEEDFOWARD_TUNING
	torqueReference = torqueReference >SLACK_ANKLE_TORQUE?torqueReference:SLACK_ANKLE_TORQUE;//prevent string slack
	#endif //#ifdef MOTOR_FEEDFOWARD_TUNING

	float motorTorque = ankleTorqueToMotorTorque(torqueReference);
	motorTorque = motorTorque>0?motorTorque:0;//no negative torque
	//Calculates the voltage to set using feedforward and/or feedback
	//Sends the voltage to the motor
	float targetV = 0.0;
	float vFeedForward = 0.0;
	float vFeedBack = 0.0;
	static float previousAnkleTorqueError =0.0;
	//float omega = ((*(rigid1.ex.enc_ang_vel)*1.0)/ENCODER_CPR) * 2 * M_PI*1000;
	float omega = act_para->motorAngularVel;

	rigid1.mn.genVar[2] = torqueReference*100;

	if(feedFoward)
    {
		#ifndef MOTOR_FEEDFOWARD_TUNING
		//vFeedForward = torqueReference*MOT_R/MOT_KT+MOT_KT*omega;
//		if(motorTorque > TORQUE_EPSILON)
//		{
//			motorTorque += ankleTorqueToMotorTorque(EXO_ANKLE_DEADBAND);
//		}
		motorTorque*=FRICTION_COMPENSATION;
		vFeedForward = motorTorque*K1;
		if(abs(omega)>OMEGA_THRESHOLD)
		{
			vFeedForward += omega*K2;
		}
		targetV += vFeedForward;
		//TODO:Fixed deadband when omega is opposite of torque
//		if(torqueReference*omega<0)
//		{
//			//don't add dead band if motor is driven backwards
//		}
		if (motorTorque>TORQUE_EPSILON)
		{
			targetV+=DEADBAND;
		}
		else if (motorTorque<-TORQUE_EPSILON)
		{
			targetV-=DEADBAND;
		}
		#endif //#ifndef MOTOR_FEEDFOWARD_TUNING
    }
    if(feedBack)
    {
    	//Feedback on voltage
    	float currentAnkleTorque =act_para->ankleTorqueMeasured;
    	float currentAnkleTorqueError = torqueReference - currentAnkleTorque;
    	float dAnkleTorqueError = (currentAnkleTorqueError-previousAnkleTorqueError)/(TIMESTEP_SIZE*1.0);
    	float averagedDerivative = updateDerivativeAverage(&derAvg, dAnkleTorqueError);
//    	float filteredDerivative = updateDerivativeFilter(&derFil, dAnkleTorqueError);

    	#ifdef PD_TUNING
    	vFeedBack =user_data_1.w[1]*1e-3*currentAnkleTorqueError+user_data_1.w[2]*1e-6*averagedDerivative;
    	rigid1.mn.genVar[7] = currentAnkleTorque*1000;
    	rigid1.mn.genVar[5]=currentAnkleTorqueError*10000;//debug

		#else
    	vFeedBack = TORQUE_KP*currentAnkleTorqueError+TORQUE_KD*averagedDerivative;
		#endif	//#ifdef PD_TUNING
    	targetV += vFeedBack;
//    	rigid1.mn.genVar[5]=filteredDerivative;//debug
//    	rigid1.mn.genVar[0]=averagedDerivative;//debug

    	previousAnkleTorqueError =previousAnkleTorqueError*(1-DERIVATIVE_WEIGHTING_FACTOR)+DERIVATIVE_WEIGHTING_FACTOR*currentAnkleTorqueError;
    }
	rigid1.mn.genVar[8]=vFeedBack*1000;//debug
	rigid1.mn.genVar[9]=vFeedForward*1000;//debug
	//setControlMode(CTRL_OPEN);	//set to open loop voltage controller
	setMotorVoltage(targetV*1000);

    //DEBUG
//    setMotorCurrent(targetV*1000);

    //rigid1.mn.genVar[3] = targetV*1000;
	//rigid1.mn.genVar[4] = omega*1000.0;
	return;
}

void setMotorTorque(float torqueReference, actuation_parameters *act_para)
{
	//Open loop feedfoward motor torque
	//Legacy code, for safety only
	float targetV = 0.0;
	float vFeedForward = 0.0;
	float omega = act_para->motorAngularVel;

	#ifdef MOTOR_FEEDFOWARD_TUNING
	vFeedForward = torqueReference*user_data_1.w[1]/1000.;
	vFeedForward += omega*user_data_1.w[2]/1000.;
	targetV += vFeedForward;
	if (torqueReference>TORQUE_EPSILON)
	{
		targetV+=user_data_1.w[3]/1000.;
	}
	else if (torqueReference<-TORQUE_EPSILON)
	{
		targetV-=user_data_1.w[3]/1000.;
	}
	#endif //#ifdef MOTOR_FEEDFOWARD_TUNING

	setMotorVoltage(targetV*1000);
	return;
}
