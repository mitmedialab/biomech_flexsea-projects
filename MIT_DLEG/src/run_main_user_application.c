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
#define KNEE_DANGLE_ANGLE 70

void runMainUserApplication(Act_s *actx){

	actx->tauDes = biomCalcImpedance(actx, actx->desiredJointK_f, actx->desiredJointB_f, actx->desiredJointAngleDeg_f);

#ifdef IS_KNEE
//    actx->tauDes = actx->tauDes - cosf(actx->jointAngle+((90-KNEE_DANGLE_ANGLE)*RADS_PER_DEGREE))*((float)user_data_1.w[9]/100.); //15.5 is value determined from experimental testing
#endif

//	if (actx->motorOnFlag) {
		setMotorTorque(actx, actx->tauDes);
//	}

}
