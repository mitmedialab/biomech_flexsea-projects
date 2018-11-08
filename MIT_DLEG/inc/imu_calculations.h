
#ifndef __IMU_CALCULATIONS_H__
#define __IMU_CALCULATIONS_H__

#include "imu.h"
#include "flexsea_user_structs.h"

// extern int16_t resetKinematics(struct rigid_s *rigid1);
extern int16_t updateLocalAcc(struct rigid_s *rigid1);
extern int16_t updateLocalOmega(struct rigid_s *rigid1);
// extern int16_t updateIntegralsAndDerivatives(struct rigid_s *rigid1);
// extern int16_t updateTranslations(struct rigid_s *rigid1);

#endif
