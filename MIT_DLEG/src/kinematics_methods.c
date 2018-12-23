


#include "kinematics_methods.h"



 //Kinematics constants (copied from matlab pil)
#define PI 3.14159
#define GRAVITY_MPS2 9.8
#define GRAVITY_SQ GRAVITY_MPS2*GRAVITY_MPS2
#define RAD_PER_DEG PI/180.0
#define GYRO_LSB_PER_DPS 32.8  //per http://dephy.com/wiki/flexsea/doku.php?id=units
#define ACCEL_LSB_PER_G 8192.0   //per http://dephy.com/wiki/flexsea/doku.php?id=units
#define ANKLE_POS_IMU_FRAME_X_M 0.0  //Frontal axis (medial->lateral)
#define ANKLE_POS_IMU_FRAME_Y_M -0.00445  //Longitudinal axis (bottom->top)
#define ANKLE_POS_IMU_FRAME_Z_M -0.0605  //Sagittal axis (back->front)
// #define ANKLE_TO_IMU_SAGITTAL_PLANE_M ANKLE_POS_IMU_FRAME_Y_M*ANKLE_POS_IMU_FRAME_Y_M + ANKLE_POS_IMU_FRAME_Z_M*ANKLE_POS_IMU_FRAME_Z_M //TO DO NEEDS A SQUARE ROOT AROUND IT!!!!!!!!
#define ACCEL_MPS2_PER_LSB  0.98 * GRAVITY_MPS2 / ACCEL_LSB_PER_G
#define N_ACCEL_MPS2_PER_LSB -0.98 * ACCEL_MPS2_PER_LSB
#define GYRO_RPS_PER_LSB RAD_PER_DEG / GYRO_LSB_PER_DPS


static struct kinematics_s kin;

static float ANKLE_TO_IMU_SAGITTAL_PLANE_M;

static void update_ankle_translations(){
	
	float aOmegaXSquared = kin.aOmegaX*kin.aOmegaX;
	float SaAy = kin.aAccY - ANKLE_POS_IMU_FRAME_Y_M*(aOmegaXSquared - SAMPLE_RATE_HZ*kin.daOmegaX);
	float SaAz = kin.aAccZ - ANKLE_POS_IMU_FRAME_Z_M*(aOmegaXSquared - SAMPLE_RATE_HZ*kin.daOmegaX);

	kin.aAy = kin.rot1*SaAy + kin.rot2*SaAz;
	kin.aAz = kin.rot3*SaAy + kin.rot4*SaAz - GRAVITY_MPS2;

	kin.vAy = kin.vAy + SAMPLE_PERIOD*kin.aAy;	
	kin.vAz = kin.vAz + SAMPLE_PERIOD*kin.aAz;	

    kin.pAy = kin.pAy + SAMPLE_PERIOD*kin.vAy;	
	kin.pAz = kin.pAz + SAMPLE_PERIOD*kin.vAz;

	float vAySq = kin.vAy*kin.vAy;
    float vAzSq = kin.vAz*kin.vAz;
    kin.sinSqAttackAngle = (((kin.vAz > 0) - (kin.vAz < 0))*vAzSq)/(vAySq+vAzSq);


}



static void reset_rotation_matrix_old_way(){
	float zAccWithCentripetalAccCompensation = (kin.aAccZ + kin.aOmegaX*kin.aOmegaX*ANKLE_TO_IMU_SAGITTAL_PLANE_M);
	float yAccWithTangentialAccCompensation = (kin.aAccY + kin.daOmegaX*SAMPLE_RATE_HZ*ANKLE_TO_IMU_SAGITTAL_PLANE_M);
	float accNormReciprocal	= 1.0/sqrtf(yAccWithTangentialAccCompensation*yAccWithTangentialAccCompensation + zAccWithCentripetalAccCompensation*zAccWithCentripetalAccCompensation);
	float costheta = zAccWithCentripetalAccCompensation*accNormReciprocal;
	float sintheta = yAccWithTangentialAccCompensation*accNormReciprocal;
	
	kin.rot1 = costheta;
	kin.rot2 = -1.0*sintheta;
	kin.rot3 = sintheta;
	kin.rot4 = costheta;

}

static void reset_ankle_translations(){
	kin.vAy = 0.0f;
	kin.vAz = 0.0f;
	kin.pAy = 0.0f;
	kin.pAz = 0.0f;
}

static void reset_integrals(){
	kin.iaOmegaX = 0.0;
	kin.iaAccY = 0.0;
	kin.iaAccZ = 0.0;
}

static void reset_kinematics(){
	reset_rotation_matrix_old_way();
	reset_ankle_translations();
	reset_integrals();
}




static void update_acc( struct fx_rigid_mn_s* mn){
	kin.aAccYprev = kin.aAccZ;
	kin.aAccZprev = kin.aAccZ;
	kin.aAccX = FILTA*kin.aAccX + FILTB * (ACCEL_MPS2_PER_LSB * (float) mn->accel.x);
	kin.aAccY = FILTA*kin.aAccY  + FILTB * (ACCEL_MPS2_PER_LSB * (float) mn->accel.z);
	kin.aAccZ = FILTA*kin.aAccZ  + FILTB * (N_ACCEL_MPS2_PER_LSB * (float) mn->accel.y);
	kin.accNormSq = kin.aAccY*kin.aAccY + kin.aAccZ*kin.aAccZ;
}

static void update_omega(struct fx_rigid_mn_s* mn){
	kin.aOmegaXprev = kin.aOmegaX;
	kin.aOmegaX = FILTA*kin.aOmegaX + FILTB*(GYRO_RPS_PER_LSB * (float) mn->gyro.x + kin.aOmegaXbias);
	kin.aOmegaY = FILTA*kin.aOmegaY + FILTB*(GYRO_RPS_PER_LSB * (float) mn->gyro.z + kin.aOmegaYbias);
	kin.aOmegaZ = FILTA*kin.aOmegaZ + FILTB*(GYRO_RPS_PER_LSB * (float) mn->gyro.y + kin.aOmegaZbias);
}

static void correct_gyro_bias(){
	kin.aOmegaXbias = FILTC*kin.aOmegaXbias - FILTD*kin.aOmegaX;
	kin.aOmegaYbias = FILTC*kin.aOmegaYbias - FILTD*kin.aOmegaY;
	kin.aOmegaZbias = FILTC*kin.aOmegaZbias - FILTD*kin.aOmegaZ;
}
static void update_integrals_and_derivatives(){
	kin.iaAccY = kin.iaAccY + kin.aAccY;
	kin.daAccY = FILTA*kin.daAccY + FILTB*(kin.aAccY - kin.aAccYprev);
	kin.iaAccZ = kin.iaAccZ + kin.aAccZ;
	kin.daAccZ = FILTA*kin.daAccZ + FILTB*(kin.aAccZ - kin.aAccZprev);
	kin.iaOmegaX = kin.iaOmegaX - kin.aOmegaX;
	kin.daOmegaX = FILTA*kin.daOmegaX + FILTB*(kin.aOmegaX - kin.aOmegaXprev);
}


static void update_rotation_matrix(){
	float rotprev1 = kin.rot1;
	float rotprev3 = kin.rot3;

	kin.rot1 = kin.rot1 + kin.rot2*kin.aOmegaX * SAMPLE_PERIOD;
	kin.rot2 = kin.rot2 - rotprev1*kin.aOmegaX * SAMPLE_PERIOD;
	kin.rot3 = kin.rot3 + kin.rot4*kin.aOmegaX * SAMPLE_PERIOD;
	kin.rot4 = kin.rot4 - rotprev3*kin.aOmegaX * SAMPLE_PERIOD;
}

void update_kinematics(struct fx_rigid_mn_s* mn, struct taskmachine_s* tm){
	update_acc(mn);
	update_omega(mn);
	correct_gyro_bias();

	update_integrals_and_derivatives();
	update_rotation_matrix();
	update_ankle_translations();

	if (tm->gait_event_trigger == GAIT_EVENT_FOOT_ON ||  tm->gait_event_trigger == GAIT_EVENT_FOOT_STATIC)
	    reset_kinematics();

}



//Copied from matlab pil simulation
struct kinematics_s* init_kinematics(){

	ANKLE_TO_IMU_SAGITTAL_PLANE_M = sqrtf(ANKLE_POS_IMU_FRAME_Y_M*ANKLE_POS_IMU_FRAME_Y_M + ANKLE_POS_IMU_FRAME_Z_M*ANKLE_POS_IMU_FRAME_Z_M);

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

struct kinematics_s* get_kinematics(){
	return &kin;
}



