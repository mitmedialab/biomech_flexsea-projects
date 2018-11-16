
#ifndef __KINEMATICS_CALCULATIONS_H__
#define __KINEMATICS_CALCULATIONS_H__

#include "imu.h"
#include <math.h>
#include "flexsea_user_structs.h"

// extern int16_t resetKinematics(struct rigid_s *rigid1);
extern void init_kinematics();
extern void update_kinematics(struct fx_rigid_mn_s* mn);
void update_acc(struct fx_rigid_mn_s* mn);
void update_omega(struct fx_rigid_mn_s* mn);
void update_integrals_and_derivatives();
void update_ankle_translations();
void update_rotation_matrix();

void reset_rotation_matrix();
void reset_ankle_translations();
void reset_integrals();
extern void reset_kinematics();
extern void get_kinematics();

// extern int16_t updateIntegralsAndDerivatives(struct rigid_s *rigid1);
// extern int16_t updateTranslations(struct rigid_s *rigid1);

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
    float* rotprev;

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
    float accNorm;

    int calculateTranslations;
};

extern struct kinematics_s kin;

#endif
