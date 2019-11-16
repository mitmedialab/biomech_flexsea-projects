/*
 * median_filter.h
 *
 *  Created on: Nov 14, 2019
 *      Author: matt
 */

#ifndef FLEXSEA_PROJECTS_MIT_DLEG_INC_MEDIAN_FILTER_H_
#define FLEXSEA_PROJECTS_MIT_DLEG_INC_MEDIAN_FILTER_H_

#include "state_variables.h"

#define MEDIAN_FILTER_WINDOW_SIZE_3		3
#define MEDIAN_FILTER_WINDOW_SIZE_5		5
#define MEDIAN_FILTER_WINDOW_SIZE_7		7
#define MEDIAN_FILTER_WINDOW_SIZE_9		9
#define MEDIAN_FILTER_WINDOW_SIZE_25		25

typedef float pixelvalue ;

float opt_med3(float * p);
float opt_med5(float * p);
float opt_med7(float * p);
float opt_med9(float * p);
float opt_med25(float * p);

float medianFilterData3( float *inputData, float *origArray );
float medianFilterData5( float *inputData, float *origArray );
float medianFilterData7( float *inputData, float *origArray );
float medianFilterData9( float *inputData, float *origArray );
float medianFilterData25( float *inputData, float *origArray );

float medianFilterArbitraryFloats(float datum);
uint16_t medianFilterArbitraryUint16(uint16_t datum);


#endif /* FLEXSEA_PROJECTS_MIT_DLEG_INC_MEDIAN_FILTER_H_ */
