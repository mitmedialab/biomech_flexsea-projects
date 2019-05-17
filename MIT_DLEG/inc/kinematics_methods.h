
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
    float aOmegaY;
    float aOmegaZ;
    float aAccX;
    float aAccY;
    float aAccZ;
    float iaAccY;
    float daAccY;
    float iaAccZ;
    float daAccZ;
    float iaOmegaX;
    float daOmegaX;
    float aAccYprev;
    float aAccZprev;
    float aOmegaXprev;
    float rot1;
    float rot3;
    float sinSqAttackAngle;
    float pitch;

    float aOmegaXbias;
    float aOmegaYbias;
    float aOmegaZbias;

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

    float aa_dot_aOmegaX_error;
    float latest_foot_static_samples;

    uint8_t foot_flat;
    int foot_flat_counter;

    float inst_ground_slope_est;
    float inst_ground_slope_est_sum;
    float ground_slope_est;
};



#endif
