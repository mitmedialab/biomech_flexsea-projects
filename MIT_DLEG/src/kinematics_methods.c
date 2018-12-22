


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

	// Om = [0 -1*cn.SAMPLE_PERIOD*kin.aOmegaX(i); cn.SAMPLE_PERIOD*kin.aOmegaX(i) 0];
 //    SaA = [kin.aAccY(i); kin.aAccZ(i)] + ...
 //        Om*Om*[cn.ANKLE_POS_IMU_FRAME_Y_M; cn.ANKLE_POS_IMU_FRAME_Z_M] + ...
 //        cn.SAMPLE_RATE_HZ * kin.daOmegaX(i)*[cn.ANKLE_POS_IMU_FRAME_Y_M; cn.ANKLE_POS_IMU_FRAME_Z_M];
 //    kin.aAy(i) = [kin.rot1(i) kin.rot2(i)]*SaA;
 //    kin.aAz(i) = [kin.rot3(i) kin.rot4(i)]*SaA - cn.GRAVITY_MPS2;
 //    kin.vAy(i) = kin.vAy(i) + cn.SAMPLE_PERIOD*kin.aAy(i);
 //    kin.vAz(i) = kin.vAz(i) + cn.SAMPLE_PERIOD*kin.aAz(i);
 //    kin.pAy(i) = kin.pAy(i) + cn.SAMPLE_PERIOD*kin.vAy(i);
 //    kin.pAz(i) = kin.pAz(i) + cn.SAMPLE_PERIOD*kin.vAz(i);
    
 //    vAySq = kin.vAy(i)*kin.vAy(i);
 //    vAzSq = kin.vAz(i)*kin.vAz(i);
 //    kin.sinSqAttackAngle(i) = (sign(kin.vAz(i))*vAzSq)./(vAySq+vAzSq);



}



static void reset_rotation_matrix_old_way(){
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
	reset_rotation_matrix_old_way();
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

void update_kinematics(struct fx_rigid_mn_s* mn, struct task_machine_s* tm){
	update_acc(mn);
	update_omega(mn);
	correct_gyro_bias();

	update_integrals_and_derivatives();
	update_rotation_matrix();
	update_ankle_translations();

	if (tm.gait_event_trigger == GAIT_EVENT_FOOT_ON || 
	    tm.gait_event_trigger == GAIT_EVENT_FOOT_STATIC) 
	    reset_kinematics();
	end
}





struct kinematics_s* get_kinematics(){
	return &kin;
}



