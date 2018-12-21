/*
 * spline_functions.h
 *
 *  Created on: Nov 9, 2018
 *      Author: matt
 */



#ifndef BIOMECH_FLEXSEA_PROJECTS_MIT_DLEG_INC_SPLINE_FUNCTIONS_H_
#define BIOMECH_FLEXSEA_PROJECTS_MIT_DLEG_INC_SPLINE_FUNCTIONS_H_


#include "walking_state_machine.h"
#include "state_variables.h"
#include "user-mn-MIT-EMG.h"
#include "flexsea_user_structs.h"
#include "flexsea_system.h"
#include "flexsea.h"
#include <math.h>
#include "user-mn-MIT-DLeg.h"


void initializeCubicSplineParams(CubicSpline *cSpline, Act_s *actx, GainParams gainParams, float res_factor);
void solveTridiagonalMatrix(CubicSpline *cSpline);
void calcCubicSpline(CubicSpline *cSpline);


#endif /* BIOMECH_FLEXSEA_PROJECTS_MIT_DLEG_INC_SPLINE_FUNCTIONS_H_ */
