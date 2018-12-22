


#include "kinematics_methods.h"



 //Kinematics constants
#define PI 3.14159
#define GRAVITY_MPS2 9.8
#define GRAVITY_SQ GRAVITY_MPS2^2
#define RAD_PER_DEG PI/180.0
#define GYRO_LSB_PER_DPS 32.8  //per http://dephy.com/wiki/flexsea/doku.php?id=units
#define ACCEL_LSB_PER_G 8192.0   //per http://dephy.com/wiki/flexsea/doku.php?id=units
#define ANKLE_POS_IMU_FRAME_X_M 0.0  //Frontal axis (medial->lateral)
#define ANKLE_POS_IMU_FRAME_Y_M -0.00445  //Longitudinal axis (bottom->top)
#define ANKLE_POS_IMU_FRAME_Z_M -0.0605  //Sagittal axis (back->front)
#define ANKLE_TO_IMU_SAGITTAL_PLANE_M sqrtf(ANKLE_POS_IMU_FRAME_Y_M^2 + ANKLE_POS_IMU_FRAME_Z_M^2)
#define ACCEL_MPS2_PER_LSB  0.98 * GRAVITY_MPS2 / ACCEL_LSB_PER_G
#define N_ACCEL_MPS2_PER_LSB -0.98 * ACCEL_MPS2_PER_LSB
#define GYRO_RPS_PER_LSB RAD_PER_DEG / GYRO_LSB_PER_DPS

static struct kinematics_s kin;

static void update_rotation_matrix(){
	float rotprev0 = kin.rot[0];
	float rotprev2 = kin.rot[2];

	kin.rot[0] = kin.rot[0] + kin.rot[1]*kin.aOmegaX;
	kin.rot[1] = kin.rot[1] - rotprev0*kin.aOmegaX;
	kin.rot[2] = kin.rot[2] + kin.rot[3]*kin.aOmegaX;
	kin.rot[3] = kin.rot[3] - rotprev2*kin.aOmegaX;

	//kin.aOmegaDot = kin.daOmegaX * 166.666667;
}

static void update_ankle_translations(){
	
	//CHAGNED FOR NOW BECAUSE kin.aOmegaDot EVALUATED TO 0 IN SIMULATION
	//kin.SaAy = kin.aAccY - kin.aOmegaX*kin.aOmegaX*RAY - kin.aOmegaDot*RAZ;
	//kin.SaAz = kin.aAccZ - kin.aOmegaX*kin.aOmegaX*RAZ + kin.aOmegaDot*RAY;
	float aOmegaXSquared = kin.aOmegaX*kin.aOmegaX;
	kin.SaAy = kin.aAccY - aOmegaXSquared*ANKLE_POS_IMU_FRAME_Y_M + SAMPLE_RATE_HZ*ANKLE_POS_IMU_FRAME_Y_M*kin.daOmegaX;
	kin.SaAz = kin.aAccZ - aOmegaXSquared*ANKLE_POS_IMU_FRAME_Z_M + SAMPLE_RATE_HZ*ANKLE_POS_IMU_FRAME_Z_M*kin.daOmegaX;

	kin.aAy = kin.rot[0]*kin.SaAy + kin.rot[1]*kin.SaAz;
	kin.aAz = kin.rot[2]*kin.SaAy + kin.rot[3]*kin.SaAz - GRAVITY_MPS2;

	kin.vAy = kin.vAy + SAMPLE_PERIOD*kin.aAy;	
	kin.vAz = kin.vAz + SAMPLE_PERIOD*kin.aAz;	

    kin.pAy = kin.pAy + SAMPLE_PERIOD*kin.vAy;	
	kin.pAz = kin.pAz + SAMPLE_PERIOD*kin.vAz;

}

static void update_acc( struct fx_rigid_mn_s* mn){

	kin.aAccZprev = kin.aAccZ;


	kin.aAccX = ACCEL_MPS2_PER_LSB * (float) mn->accel.x;
	kin.aAccZ = N_ACCEL_MPS2_PER_LSB * (float) mn->accel.y;
	kin.aAccY = ACCEL_MPS2_PER_LSB * (float) mn->accel.z;

    kin.accNormSq = kin.aAccY*kin.aAccY + kin.aAccZ*kin.aAccZ;

}

static void update_omega(struct fx_rigid_mn_s* mn){


	kin.aOmegaXprev = kin.aOmegaX;

	kin.aOmegaX = GYRO_RPS_PER_LSB * (float) mn->gyro.x;
	kin.aOmegaZ = GYRO_RPS_PER_LSB * (float) mn->gyro.y;
	kin.aOmegaY = GYRO_RPS_PER_LSB * (float) mn->gyro.z;

}

static void update_integrals_and_derivatives(){
	kin.iaAccZ = kin.iaAccZ + kin.aAccZ;
    kin.daAccZ = FILTA*kin.daAccZ + FILTB*(kin.aAccZ - kin.aAccZprev);
    kin.daOmegaX = FILTA*kin.daOmegaX + FILTB*(kin.aOmegaX - kin.aOmegaXprev);
}

static void reset_rotation_matrix(){
	float accNormReciprocal = 1.0/sqrtf(kin.accNormSq);
	float costheta = (kin.aAccZ + kin.aOmegaX*kin.aOmegaX*ANKLE_TO_IMU_SAGITTAL_PLANE_M)*accNormReciprocal;
	float sintheta = (kin.aAccY + kin.daOmegaX*SAMPLE_RATE_HZ*ANKLE_TO_IMU_SAGITTAL_PLANE_M)*accNormReciprocal;
	kin.rot[0] = costheta;
	kin.rot[1] = -1.0*sintheta;
	kin.rot[2] = sintheta;
	kin.rot[3] = costheta;
}

static void reset_ankle_translations(){
	kin.vAy = 0.0f;
	kin.vAz = 0.0f;
	kin.pAy = 0.0f;
	kin.pAz = 0.0f;
}

static void reset_integrals(){
	kin.iaAccZ = 0.0f;
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

static void reset_kinematics(){
	// if (!tm.resetFlag)
	// 	return 0;
	// if (tm.resetFlag == 1){
		
	//}
	//tm.latestFootStaticTime = tm.loopcount;
	reset_rotation_matrix();
	reset_ankle_translations();
	reset_integrals();
}

//Copied from matlab pil simulation
struct kinematics_s* init_kinematics(){
	kin.aOmegaX = 0;
    kin.aOmegaY = 0;
    kin.aOmegaZ = 0;
    kin.aAccX = 0;
    kin.aAccY = 0;
    kin.aAccZ = 0;
    kin.iaAccY = 0;
    kin.daAccY = 0;
    kin.iaAccZ = 0;
    kin.daAccZ = 0;
    kin.iaOmegaX = 0;
    kin.daOmegaX = 0;
    kin.aAccYprev = 0;
    kin.aAccZprev = 0;
    kin.aOmegaXprev = 0;
    kin.rot1 = 0;
    kin.rot2 = 0;
    kin.rot3 = 0;
    kin.rot4 = 0;
    kin.accNormSq = 0;
    kin.sinSqAttackAngle =  0;

    kin.aOmegaXbias = 0;
    kin.aOmegaYbias = 0;
    kin.aOmegaZbias = 0;

    kin.aAy = 0;
    kin.aAz = 0;
    kin.vAy = 0;
    kin.vAz = 0;
    kin.pAy = 0;
    kin.pAz = 0;

	return &kin;
}

void update_kinematics(struct fx_rigid_mn_s* mn, int* translation_reset_trigger, int* reached_classification_time){
	if(!*reached_classification_time){
		update_acc(mn);
		update_omega(mn);
		update_integrals_and_derivatives();
		update_rotation_matrix();
		update_ankle_translations();
	}
	if (*translation_reset_trigger){
		reset_kinematics();
		*translation_reset_trigger = 0;
		*reached_classification_time = 0;
	}
}





struct kinematics_s* get_kinematics(){
	return &kin;
}



