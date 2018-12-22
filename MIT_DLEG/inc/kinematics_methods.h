
#ifndef __KINEMATICS_METHODS_H__
#define __KINEMATICS_METHODS_H__

#include "imu.h"
#include <math.h>
#include "flexsea_user_structs.h"
#include "task_machine.h"

struct kinematics_s* init_kinematics();
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
    float rot2;
    float rot3;
    float rot4;
    float accNormSq;
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

};



#endif
