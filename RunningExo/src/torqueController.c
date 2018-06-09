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

void setTorque(float torqueReference, actuation_parameters *act_para,  _Bool feedFoward, _Bool feedBack)
{
	//Calculates the voltage to set using feedforward and/or feedback
	//Sends the voltage to the motor
	float targetV = 0.0;
	float vFeedForward = 0.0;
	float vFeedBack = 0.0;

	//float omega = ((*(rigid1.ex.enc_ang_vel)*1.0)/ENCODER_CPR) * 2 * M_PI*1000;
	float omega = act_para->motorAngularVel;

	if(feedFoward)
    {
		//vFeedForward = torqueReference*MOT_R/MOT_KT+MOT_KT*omega;
		vFeedForward = torqueReference*K1;
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
		if (torqueReference>TORQUE_EPSILON)
		{
			targetV+=DEADBAND;
		}
		else if (torqueReference<-TORQUE_EPSILON)
		{
			targetV-=DEADBAND;
		}
    }
    if(feedBack)
    {
    	//TODO
    	targetV += vFeedBack;
    }
	//setControlMode(CTRL_OPEN);	//set to open loop voltage controller
    if(user_data_1.w[1]!=0)	//estop
    {
    	setMotorVoltage(0);
    }
    else
    {
    	setMotorVoltage(targetV*1000);
    }

    //DEBUG
//    setMotorCurrent(targetV*1000);

    rigid1.mn.genVar[3] = targetV*1000;
	rigid1.mn.genVar[4] = omega*1000.0;
	return;
}
