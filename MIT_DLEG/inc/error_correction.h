/*
 * error_correction.h
 *
 *  Created on: Jun 22, 2019
 *      Author: matt
 */

#ifndef FLEXSEA_PROJECTS_MIT_DLEG_INC_ERROR_CORRECTION_H_
#define FLEXSEA_PROJECTS_MIT_DLEG_INC_ERROR_CORRECTION_H_

#include "lookup.h"
#include <math.h>

// 17
//static float jointAngle[] = {-25,-19,-13,-7,-1,5,12,21,29,41,51,60,67,71,76,83,87};
//static float torqueError[] = {-133,-114,-91,-77,-61,-41,-20,-1,11,-4,-36,-85,-142,-180,-217,-271,-309};
//static float torqueZeros[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

//// 20
//static float jointAngle[] = {-35,-28,-21,-14,-8,-1,6,13,20,27,33,40,47,54,61,68,74,81,88,95};
//static float torqueError[] = {-180,-147,-121,-100,-79,-58,-38,-19,-3,6,8,0,-19,-51,-94,-147,-205,-263,-312,-342};

// 20
static float jointAngle[] = {10638.4444,10949.8363,11261.2281,11572.6199,11884.0117,12195.4035,12506.7953,12818.1871,13129.5789,13440.9708,13752.3626,14063.7544,14375.1462,14686.5380,14997.9298,15309.3216,15620.7135,15932.1053,16243.4971,16554.8889};
static float torqueError[] = {-364.7418,-334.3979,-284.0068,-224.4433,-163.9946,-108.6440,-62.3548,-27.3539,-4.4157,6.8543,7.7352,0.1079,-13.8296,-31.8455,-51.9581,-72.7193,-93.4988,-114.7676,-138.3819,-167.8667};


static const Table1d jointTorqueErrorMap = {
    20,      /* Number of data points */
    jointAngle, /* Array of x-coordinates */
    torqueError  /* Array of y-coordinates */
};


#endif /* FLEXSEA_PROJECTS_MIT_DLEG_INC_ERROR_CORRECTION_H_ */
