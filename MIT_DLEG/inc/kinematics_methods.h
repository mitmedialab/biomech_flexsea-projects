
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

//Copied from matlab pil simulation
struct kinematics_s
{
	float aOmegaX;
    float aAccX;
    float aAccY;
    float aAccZ;
    float daOmegaX;
    float aOmegaXprev;
    float rot1;
    float rot3;
    float rot3prev;
    float pitch;

    float aOmegaXbias;

    float aAy;
    float aAz;
    float vAy;
    float vAz;
    float pAy;
    float pAz;

    float aAccXYZscaling;
    int accelQuietSamples;
    float meanaccNormSq;
    float accNormSq;
    float accNormSqRaw;
    uint8_t rolling_over_foot;
    int roll_over_counter;
    float ground_slope_est_sum;
    float curr_ground_slope_est;
    float prev_ground_slope_est;


    float joint_vel_seg_vel_diff_sq;
    float min_joint_vel_seg_vel_diff_sq;
    float latest_foot_static_samples;

    uint8_t foot_flat;
    int foot_flat_counter;

};



#endif
