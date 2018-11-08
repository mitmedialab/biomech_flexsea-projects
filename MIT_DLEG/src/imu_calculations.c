
// #include <math.h>
// #include <string.h>
// #include <stdlib.h>
// #include "global.h"
// #include "sensors.h"
// #include "app_timer.h"
// #include "taskmachine.h"
// #include "std_def.h"
// #include "imu_calculations.h"
// #include "system_constants.h"
// #include "t2_statemachine_states.h"
// #include "statemachine.h"
// #include "lineardiscriminant.h"
#include <math.h>
#include "imu_calculations.h"



#define PI 3.14
#define GRAVITY_MPS2 9.8
#define RAD_PER_DEG PI/180.0

//System constants
#define GYRO_LSB_PER_DPS 32.8 //%per http://dephy.com/wiki/flexsea/doku.php?id=units
#define ACCEL_LSB_PER_G 8192.0  //per http://dephy.com/wiki/flexsea/doku.php?id=units
#define SAMPLE_RATE_HZ = 1000
#define ANKLE_POS_IMU_FRAME_X_M 0.0 //Frontal axis (medial->lateral)
#define ANKLE_POS_IMU_FRAME_Y_M -0.00445 //Longitudinal axis (bottom->top)
#define ANKLE_POS_IMU_FRAME_Z_M -0.0605 //Sagittal axis (back->front)

//Default filter coefficients
#define FILTA 0.95
#define FILTB 0.05

#define ACCEL_MPS2_PER_LSB GRAVITY_MPS2 / ACCEL_LSB_PER_G
#define GYRO_RPS_PER_LSB RAD_PER_DEG / GYRO_LSB_PER_DPS
#define SAMPLE_PERIOD_S 1/SAMPLE_RATE_HZ
#define ANKLE_TO_IMU_SAGITTAL_PLANE_M ANKLE_POS_IMU_FRAME_Y_M*ANKLE_POS_IMU_FRAME_Y_M + ANKLE_POS_IMU_FRAME_Z_M*ANKLE_POS_IMU_FRAME_Z_M


// //Machine learning
// // POST_SWING_CUTOFF_TIME_S = 0.2;
// // N_CLASSES = 5;
// // N_SIMULATIONS = 100;

// // %Gait event detection
// // GAIT_EVENT_THRESHOLD_TORQUE_NM = 1;
// // MIN_TQDOT_FOR_FOOT_STATIC_NM = -0.1;
// // MIN_TQ_FOR_FOOT_STATIC_NM = 15;
// // DEFAULT_ZVUP_TIME_S = 0.05;
// // UPPER_ACCNORM_THRESH_SQ = 102.01;
// // LOWER_ACCNORM_THRESH_SQ = 90.25;
// // MIN_SWING_TIME_S = 0.1; % was 0.32;
// // MIN_STANCE_TIME_S = 0.1;
// // MIN_STRIDE_TIME_S = 0.5;

// //Define derived constants
// // MIN_SWING_SAMPLES = MIN_SWING_TIME_S * SAMPLE_RATE_HZ;
// // MIN_STANCE_SAMPLES = MIN_STANCE_TIME_S * SAMPLE_RATE_HZ;
// // MIN_STRIDE_SAMPLES = MIN_STRIDE_TIME_S * SAMPLE_RATE_HZ;
// // DEFAULT_ZVUP_SAMPLES = DEFAULT_ZVUP_TIME_S * SAMPLE_RATE_HZ;
// // POST_SWING_CUTOFF_SAMPLES = int32(POST_SWING_CUTOFF_TIME_S * SAMPLE_RATE_HZ);

// ERRCODE_T updateRotationMatrix(struct rigid_s *rigid1){
// 	sigs.rotprev[0] = sigs.rot[0];
// 	sigs.rotprev[2] = sigs.rot[2];

// 	sigs.rot[0] = sigs.rot[0] + sigs.rot[1]*sigs.aOmegaX;
// 	sigs.rot[1] = sigs.rot[1] - sigs.rotprev[0]*sigs.aOmegaX;
// 	sigs.rot[2] = sigs.rot[2] + sigs.rot[3]*sigs.aOmegaX;
// 	sigs.rot[3] = sigs.rot[3] - sigs.rotprev[2]*sigs.aOmegaX;

// 	//sigs.aOmegaDot = sigs.daOmegaX * 166.666667;
    

// 	return 0;
// }

// ERRCODE_T updateAnkleTranslations(struct rigid_s *rigid1){
	
// 	//CHAGNED FOR NOW BECAUSE sigs.aOmegaDot EVALUATED TO 0 IN SIMULATION
// 	//sigs.SaAy = sigs.aAccY - sigs.aOmegaX*sigs.aOmegaX*RAY - sigs.aOmegaDot*RAZ;
// 	//sigs.SaAz = sigs.aAccZ - sigs.aOmegaX*sigs.aOmegaX*RAZ + sigs.aOmegaDot*RAY;
// 	tm.tempHolder[1] = sigs.aOmegaX*sigs.aOmegaX;
// 	sigs.SaAy = sigs.aAccY - tm.tempHolder[1]*RAY + FSIMU_X_RAY*sigs.daOmegaX;
// 	sigs.SaAz = sigs.aAccZ - tm.tempHolder[1]*RAZ + FSIMU_X_RAZ*sigs.daOmegaX;

// 	sigs.aAy = sigs.rot[0]*sigs.SaAy + sigs.rot[1]*sigs.SaAz;
// 	sigs.aAz = sigs.rot[2]*sigs.SaAy + sigs.rot[3]*sigs.SaAz - GRAVITY;

// 	sigs.vAy = sigs.vAy + TSIMU*sigs.aAy;	
// 	sigs.vAz = sigs.vAz + TSIMU*sigs.aAz;	

//      sigs.pAy = sigs.pAy + TSIMU*sigs.vAy;	
// 	// sigs.pAz = sigs.pAz + TSIMU*sigs.vAz;

// 	return 0;
// }

// ERRCODE_T fastSquareFootAccNormSq(struct rigid_s *rigid1){
// 	tm.accNormReciprocal = (tm.accNormSq + 96.04)/(0.0255*(tm.accNormSq + 16.4788)*(tm.accNormSq + 559.953));
// 	return 0;
// }

// ERRCODE_T resetAnkleKinematics(struct rigid_s *rigid1){
// 	sigs.vAy = 0.0f;
// 	sigs.vAz = 0.0f;
// 	sigs.pAy = 0.0f;
// 	sigs.pAz = 0.0f;
// 	return 0;
// }

// ERRCODE_T resetRotationMatrix(struct rigid_s *rigid1){
// 	fastSquareFootAccNormSq();
// 	tm.tempHolder[1] = (sigs.aAccZ + sigs.aOmegaX*sigs.aOmegaX*1372.8790244f)*tm.accNormReciprocal;
// 	tm.tempHolder[2] = (sigs.aAccY + sigs.daOmegaX*1372.8790244f)*tm.accNormReciprocal;
// 	sigs.rot[0] = tm.tempHolder[1];
// 	sigs.rot[1] = -1.0*tm.tempHolder[2];
// 	sigs.rot[2] = tm.tempHolder[2];
// 	sigs.rot[3] = tm.tempHolder[1];
// 	return 0;
// }

// ERRCODE_T resetIntegrals(struct rigid_s *rigid1){
// 	// sigs.iaAccX = 0.0;
//  //    sigs.iaAccY = 0.0;
//        sigs.iaAccZ = 0.000f;
//  //    sigs.iaOmegaX = 0.0;
//  //    sigs.iaOmegaY = 0.0;
//  //    sigs.iaOmegaZ = 0.0;
//  //    sigs.i2aAccX = 0.0;
//  //    sigs.i2aAccY = 0.0;
//  //    sigs.i2aAccZ = 0.0;
//  //    sigs.i2aOmegaX = 0.0;
//  //    sigs.i2aOmegaY = 0.0;
//  //    sigs.i2aOmegaZ = 0.0;
//     return 0;
// }

int16_t updateLocalAcc(struct rigid_s *rigid1){

	//sigs.aAccZprev = sigs.aAccZ;


	rigid1->mn.aAccX = ACCEL_MPS2_PER_LSB * (float) rigid1->mn.accel.x;
	rigid1->mn.aAccZ = -1.0*ACCEL_MPS2_PER_LSB * (float) rigid1->mn.accel.y;
	rigid1->mn.aAccY = ACCEL_MPS2_PER_LSB * (float) rigid1->mn.accel.z;

   // tm.accNormSq = sigs.aAccY*sigs.aAccY + sigs.aAccZ*sigs.aAccZ;

	return 0;
}

int16_t updateLocalOmega(struct rigid_s *rigid1){


	//sigs.aOmegaXprev = sigs.aOmegaX;

	rigid1->mn.aOmegaX = GYRO_RPS_PER_LSB * (float) rigid1->mn.gyro.x;
	rigid1->mn.aOmegaZ = GYRO_RPS_PER_LSB * (float) rigid1->mn.gyro.y;
	rigid1->mn.aOmegaY = GYRO_RPS_PER_LSB * (float) rigid1->mn.gyro.z;

	return 0;
}

// ERRCODE_T updateIntegralsAndDerivatives(struct rigid_s *rigid1){
// 	sigs.iaAccZ = sigs.iaAccZ + sigs.aAccZ;
//     sigs.daAccZ = FILTA*sigs.daAccZ + FILTB*(sigs.aAccZ - sigs.aAccZprev);
//     sigs.daOmegaX = FILTA*sigs.daOmegaX + FILTB*(sigs.aOmegaX - sigs.aOmegaXprev);
// 	return 0;
// }


// ERRCODE_T updateTranslations(struct rigid_s *rigid1){
// 	if (tm.calculateTranslations){
// 		updateRotationMatrix();
// 		updateAnkleTranslations();
// 	}

// 	if (tm.loopcount - tm.latestFootStaticTime > 600){
// 	    tm.calculateTranslations = 0;
//      }

//   return 0;
// }


// ERRCODE_T resetKinematics(struct rigid_s *rigid1){
// 	// if (!tm.resetFlag)
// 	// 	return 0;
// 	// if (tm.resetFlag == 1){
		
// 	//}
// 	tm.latestFootStaticTime = tm.loopcount;
	
// 	resetRotationMatrix();
// 	resetAnkleKinematics();
// 	resetIntegrals();
// 	tm.calculateTranslations = 1;
	
// 	return 0;

// }



