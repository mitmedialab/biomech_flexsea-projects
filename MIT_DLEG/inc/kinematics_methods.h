
#ifndef __KINEMATICS_METHODS_H__
#define __KINEMATICS_METHODS_H__

#include "imu.h"
#include <math.h>
#include "flexsea_user_structs.h"
#include "task_machine.h"

struct kinematics_s* init_kinematics();
void update_kinematics(struct fx_rigid_mn_s* mn, int* translation_reset_trigger, int* reached_classification_time);
struct kinematics_s* get_kinematics();

struct kinematics_s
{
	float aOmegaX;
	float aOmegaY;
	float aOmegaZ;
	float aAccX;
	float aAccY;
	float aAccZ;

	float iaAccZ;
    float daAccZ;
    float daOmegaX;
    float aAccZprev;
    float aOmegaXprev;

    float* rot;

    float SaAy;
    float SaAz;
    float aAy;
    float aAz;
    float vAy;
    float vAz;
    float pAy;
    float pAz;

    float accNormReciprocal;
    float accNormSq;

};



#endif
