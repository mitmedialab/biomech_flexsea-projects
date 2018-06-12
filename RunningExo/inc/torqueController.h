/*
 * torqueController.h
 *
 *  Created on: May 25, 2018
 *      Author: Albert Wu
 */

#ifndef TORQUE_CONTROLLER_H
#define TORQUE_CONTROLLER_H
#include "runningExo-structs.h"

void setAnkleTorque(float torqueReference, actuation_parameters *act_para,_Bool feedFoward, _Bool feedBack);
void setMotorTorque(float torqueReference, actuation_parameters *act_para);

#endif /* TORQUE_CONTROLLER_H */
