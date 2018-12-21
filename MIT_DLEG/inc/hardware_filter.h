/*
 * mit_filters.h
 *
 *  Created on: Mar 14, 2018
 *      Author: Seong Ho Yeon
 */

#ifndef BIOMECH_FLEXSEA_PROJECTS_MIT_DLEG_INC_MIT_FILTERS_H_
#define BIOMECH_FLEXSEA_PROJECTS_MIT_DLEG_INC_MIT_FILTERS_H_

#include "arm_math.h"

//#define LPF1 // Passband 100Hz, Stopband 200Hz
//#define LPF2 // Passband 50Hz, Stopband 100Hz
#define LPF3 // Passband 35Hz, Stopband 70Hz
//#define LPF4 // Passband 35Hz, Stopband 70Hz

extern uint16_t lpf_index;

//float32_t lpf_out;
void  init_LPF(void);
void  update_LPF(float val);
float filter_LPF(float val);



#endif /* BIOMECH_FLEXSEA_PROJECTS_MIT_DLEG_INC_MIT_FILTERS_H_ */

