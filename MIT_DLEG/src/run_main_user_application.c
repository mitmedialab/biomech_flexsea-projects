/*
 * run_main_user_application.c
 *
 *  Created on: Jan 17, 2019
 *      Author: matt
 */


#include "run_main_user_application.h"
#include "walking_state_machine.h"
#include "actuator_functions.h"
#include "state_variables.h"

#define RADS_PER_DEGREE 0.01745

//Chirp
//static uint32_t t = 0;
//static float c = 0.99975;
//static float f0 = 0.01;
//float tauD = 0;
//float tt=0;
//Chirp


int16_t Kp = 0;
int16_t Ki = 0;
int16_t Kd = 0;

int16_t lastKp = 0;
int16_t lastKi = 0;
int16_t lastKd = 0;

void runMainUserApplication(Act_s *actx){



//	actx->tauDes = user_data_1.w[0];


	//Testing current control
//	actx->desiredCurrent = user_data_1.w[0];
	if(user_data_1.w[0] < 30000){
		actx->desiredCurrent = user_data_1.w[0];
	}


	Kp = user_data_1.w[1];
	Ki = user_data_1.w[2];
	Kd = user_data_1.w[3];

//	setControlGains(Kp, Ki, Kd, 0, DEVICE_CHANNEL);
	//End of current control params

	if(lastKp != Kp || lastKi != Ki || lastKd != Kd){
		setControlGains(Kp, Ki, Kd, 0, DEVICE_CHANNEL);
	}

	setMotorCurrent(actx->desiredCurrent, DEVICE_CHANNEL);

	lastKp = Kp;
	lastKi = Ki;
	lastKd = Kd;
	//Testing current control


/*	actx->desiredJointK_f = ((float) user_data_1.w[0])/10;
	actx->desiredJointB_f = ((float) user_data_1.w[1])/1000;
	actx->desiredJointAngleDeg_f = ((float) user_data_1.w[2])/10;*/

//	actx->tauDes = biomCalcImpedance(actx, actx->desiredJointK_f, actx->desiredJointB_f, actx->desiredJointAngleDeg_f);

//	actx->intTauDes = actx->tauDes*MULTIPK_SCALE;

//	setMotorTorque(actx, actx->tauDes);



//	if (actx->motorOnFlag) {
//		setMotorTorque(actx, actx->tauDes);
//	}



	//Chirp
//	rigid1.mn.genVar[8] = (int16_t) (t);
//	rigid1.mn.genVar[9] = (int16_t) (actx->tauDes*100);

//	if(user_data_1.w[0]==1 && t < 40500){
//		tt=((float)t)*0.001;
//		actx->tauDes = 10 + 5*sin(2*M_PI*((c*tt*tt/2)+(f0*tt)));
//		t++;
//	}else{
//		actx->tauDes = 0;
//	}
	//Chirp

//	setMotorTorque(actx, actx->tauDes);

//
//#ifdef IS_KNEE
//    actx->tauDes = actx->tauDes - cosf(actx->jointAngle+(15*RADS_PER_DEGREE))*15.5; //15.5 is value determined from experimental testing
//#endif
//
//	if (actx->motorOnFlag) {
//		setMotorTorque(actx, actx->tauDes);
//	}

}
