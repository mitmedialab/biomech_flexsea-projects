
#ifndef __KINEMATICS_METHODS_H__
#define __KINEMATICS_METHODS_H__

#include "imu.h"
#include <math.h>
#include "flexsea_user_structs.h"
#include "task_machine.h"



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
    float meanAccelSumSqr;
    float accelSumSqr;

    uint8_t updateOrientation;
    uint8_t resetPosition;

    float aa_dot_aOmegaX_error;
    float aa_dot_aOmegaX_error_prev;
    float aa_dot_aOmegaX_error_prev_prev;
    float latest_foot_static_samples;
    int low_aa_dot_aOmegaX_error_counter;
};



#endif
