/*
 * filtering_methods.h
 *
 *  Created on: May 16, 2019
 *      Author: rmsto
 */

#ifndef FLEXSEA_PROJECTS_MIT_DLEG_INC_FILTERING_METHODS_H_
#define FLEXSEA_PROJECTS_MIT_DLEG_INC_FILTERING_METHODS_H_

float filter_second_order_butter_20hz(float new_val, float* outputs, float* inputs);
float filter_first_order_butter_20hz(float new_val, float* outputs, float* inputs);
float filter_second_order_butter_5hz(float new_val, float* outputs, float* inputs);




#endif /* FLEXSEA_PROJECTS_MIT_DLEG_INC_FILTERING_METHODS_H_ */
