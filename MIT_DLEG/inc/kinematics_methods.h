
#ifndef __KINEMATICS_METHODS_H__
#define __KINEMATICS_METHODS_H__

#include "imu.h"
#include <math.h>
#include "flexsea_user_structs.h"
#include "task_machine.h"
#include "filtering_methods.h"



void init_kinematics();
void update_kinematics(struct fx_rigid_mn_s* mn, struct taskmachine_s* tm);

//Getters
struct kinematics_s* get_kinematics();

struct kinematics_s
{
	//Inertial signals
	float aOmegaX;
    float aAccX;
    float aAccY;
    float aAccZ;
    float daOmegaX;

    //Ground slope estimation variables
    float curr_ground_slope_est;
    float ground_slope_est_sum;
    int ground_slope_estimation_counter;

    //IMU scaling/bias variables
	float aOmegaXbias;
	float aAccXYZscaling;
	int accelQuietSamples;
	float meanaccNormSq;

	//Pose variables
	float rot1;
	float rot3;
	float aAy;
	float aAz;
	float vAy;
	float vAz;
	float pAy;
	float pAz;
	float displacement;
	float end_of_stride_pAz;

	//Kinematics reset variables
    float joint_vel_seg_vel_diff_sq;
    float latest_foot_static_samples;
    float accNormSq;
    float accNormSqRaw;
    uint8_t foot_flat;
    int foot_flat_counter;

};



#endif
