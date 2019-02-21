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

void runMainUserApplication(Act_s *actx){

	actx->tauDes = biomCalcImpedance(actx, actx->desiredJointK_f, actx->desiredJointB_f, actx->desiredJointAngleDeg_f);

	if (actx->motorOnFlag) {
		setMotorTorque(actx, actx->tauDes);
	}

}
