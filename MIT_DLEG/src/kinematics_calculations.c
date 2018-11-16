


#include "kinematics_calculations.h"



#define PI 3.14
#define GRAVITY_MPS2 9.8
#define RAD_PER_DEG PI/180.0

//System constants
#define GYRO_LSB_PER_DPS 32.8 //%per http://dephy.com/wiki/flexsea/doku.php?id=units
#define ACCEL_LSB_PER_G 8192.0  //per http://dephy.com/wiki/flexsea/doku.php?id=units
#define SAMPLE_RATE_HZ 1000
#define SAMPLE_PERIOD 1.0/SAMPLE_RATE_HZ
#define ANKLE_POS_IMU_FRAME_X_M 0.0 //Frontal axis (medial->lateral)
#define ANKLE_POS_IMU_FRAME_Y_M -0.00445 //Longitudinal axis (bottom->top)
#define ANKLE_POS_IMU_FRAME_Z_M -0.0605 //Sagittal axis (back->front)

//Default filter coefficients
#define FILTA 0.95
#define FILTB 0.05

//TEMPORARY PLACEHOLDERS NEED TO CHANGE
#define RAY 000
#define RAZ 000
#define FSIMU_X_RAY 000
#define FSIMU_X_RAZ 000

#define ACCEL_MPS2_PER_LSB GRAVITY_MPS2 / ACCEL_LSB_PER_G
#define N_ACCEL_MPS2_PER_LSB -1.0 * ACCEL_MPS2_PER_LSB
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

struct kinematics_s kin;

void update_rotation_matrix(){
	kin.rotprev[0] = kin.rot[0];
	kin.rotprev[2] = kin.rot[2];

	kin.rot[0] = kin.rot[0] + kin.rot[1]*kin.aOmegaX;
	kin.rot[1] = kin.rot[1] - kin.rotprev[0]*kin.aOmegaX;
	kin.rot[2] = kin.rot[2] + kin.rot[3]*kin.aOmegaX;
	kin.rot[3] = kin.rot[3] - kin.rotprev[2]*kin.aOmegaX;

	//kin.aOmegaDot = kin.daOmegaX * 166.666667;
}

void update_ankle_translations(){
	
	//CHAGNED FOR NOW BECAUSE kin.aOmegaDot EVALUATED TO 0 IN SIMULATION
	//kin.SaAy = kin.aAccY - kin.aOmegaX*kin.aOmegaX*RAY - kin.aOmegaDot*RAZ;
	//kin.SaAz = kin.aAccZ - kin.aOmegaX*kin.aOmegaX*RAZ + kin.aOmegaDot*RAY;
	float aOmegaXSquared = kin.aOmegaX*kin.aOmegaX;
	kin.SaAy = kin.aAccY - aOmegaXSquared*RAY + FSIMU_X_RAY*kin.daOmegaX;
	kin.SaAz = kin.aAccZ - aOmegaXSquared*RAZ + FSIMU_X_RAZ*kin.daOmegaX;

	kin.aAy = kin.rot[0]*kin.SaAy + kin.rot[1]*kin.SaAz;
	kin.aAz = kin.rot[2]*kin.SaAy + kin.rot[3]*kin.SaAz - GRAVITY_MPS2;

	kin.vAy = kin.vAy + SAMPLE_PERIOD*kin.aAy;	
	kin.vAz = kin.vAz + SAMPLE_PERIOD*kin.aAz;	

    kin.pAy = kin.pAy + SAMPLE_PERIOD*kin.vAy;	
	kin.pAz = kin.pAz + SAMPLE_PERIOD*kin.vAz;

}






void init_kinematics(){
	kin.rot = (float*)calloc(4, sizeof(float));
	kin.rotprev = (float*)calloc(4, sizeof(float));
}

void update_kinematics(struct fx_rigid_mn_s* mn){
	update_acc(mn);
	update_omega(mn);
	update_integrals_and_derivatives();
	if(kin.calculateTranslations){
		update_rotation_matrix();
		update_ankle_translations();
	}
}

void update_acc( struct fx_rigid_mn_s* mn){

	kin.aAccZprev = kin.aAccZ;


	kin.aAccX = ACCEL_MPS2_PER_LSB * (float) mn->accel.x;
	kin.aAccZ = N_ACCEL_MPS2_PER_LSB * (float) mn->accel.y;
	kin.aAccY = ACCEL_MPS2_PER_LSB * (float) mn->accel.z;

   // tm.accNormSq = kin.aAccY*kin.aAccY + kin.aAccZ*kin.aAccZ;

}

void update_omega(struct fx_rigid_mn_s* mn){


	//kin.aOmegaXprev = kin.aOmegaX;

	kin.aOmegaX = GYRO_RPS_PER_LSB * (float) mn->gyro.x;
	kin.aOmegaZ = GYRO_RPS_PER_LSB * (float) mn->gyro.y;
	kin.aOmegaY = GYRO_RPS_PER_LSB * (float) mn->gyro.z;

}

void update_integrals_and_derivatives(){
	kin.iaAccZ = kin.iaAccZ + kin.aAccZ;
    kin.daAccZ = FILTA*kin.daAccZ + FILTB*(kin.aAccZ - kin.aAccZprev);
    kin.daOmegaX = FILTA*kin.daOmegaX + FILTB*(kin.aOmegaX - kin.aOmegaXprev);
}

void reset_rotation_matrix(){
	kin.accNormReciprocal = 1.0/sqrtf(kin.accNorm);
	float costheta = (kin.aAccZ + kin.aOmegaX*kin.aOmegaX*1372.8790244f)*kin.accNormReciprocal;
	float sintheta = (kin.aAccY + kin.daOmegaX*1372.8790244f)*kin.accNormReciprocal;
	kin.rot[0] = costheta;
	kin.rot[1] = -1.0*sintheta;
	kin.rot[2] = sintheta;
	kin.rot[3] = costheta;
}

void reset_ankle_translations(){
	kin.vAy = 0.0f;
	kin.vAz = 0.0f;
	kin.pAy = 0.0f;
	kin.pAz = 0.0f;
}

void reset_integrals(){
	// kin.iaAccX = 0.0;
 //    kin.iaAccY = 0.0;
   //    kin.iaAccZ = 0.000f;
 //    kin.iaOmegaX = 0.0;
 //    kin.iaOmegaY = 0.0;
 //    kin.iaOmegaZ = 0.0;
 //    kin.i2aAccX = 0.0;
 //    kin.i2aAccY = 0.0;
 //    kin.i2aAccZ = 0.0;
 //    kin.i2aOmegaX = 0.0;
 //    kin.i2aOmegaY = 0.0;
 //    kin.i2aOmegaZ = 0.0;
}

void reset_kinematics(){
	// if (!tm.resetFlag)
	// 	return 0;
	// if (tm.resetFlag == 1){
		
	//}
	//tm.latestFootStaticTime = tm.loopcount;
	
	reset_rotation_matrix();
	reset_ankle_translations();
	reset_integrals();
	kin.calculateTranslations = 1;
	
}

void get_kinematics(){
	return &kin
};



