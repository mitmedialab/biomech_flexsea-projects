


#include "kinematics_methods.h"



 //Kinematics constants (copied from matlab pil)
#define GRAVITY_MPS2 9.8
#define GRAVITY_SQ GRAVITY_MPS2*GRAVITY_MPS2
#define GYRO_LSB_PER_DPS 32.8  //per http://dephy.com/wiki/flexsea/doku.php?id=units
#define ACCEL_LSB_PER_G 8192.0   //per http://dephy.com/wiki/flexsea/doku.php?id=units
#define ANKLE_POS_IMU_FRAME_X_M 0.0  //Frontal axis (medial->lateral)
#define ANKLE_POS_IMU_FRAME_Y_M -0.00445  //Longitudinal axis (bottom->top)
#define ANKLE_POS_IMU_FRAME_Z_M -0.0605  //Sagittal axis (back->front)
#define ACCEL_MPS2_PER_LSB  GRAVITY_MPS2 / ACCEL_LSB_PER_G
#define N_ACCEL_MPS2_PER_LSB -1.0*ACCEL_MPS2_PER_LSB
#define GYRO_RPS_PER_LSB RAD_PER_DEG / GYRO_LSB_PER_DPS
#define MIN_ACCEL_SUMSQR_MEAN_OFFSET_FOR_RESCALING 1.0
#define MIN_ACCEL_QUIET_SAMPLES_FOR_RESCALING 1000
#define PAZ_MAX 1.0


//Foot static constants
#define AA_DOT_AOMEGAX_ERROR_THRESH_RPS2 0.2
#define FOOT_STATIC_ACC_NORM_TOL_MPS2 0.5
#define GRAVITY_P_TOL (GRAVITY_MPS2 + FOOT_STATIC_ACC_NORM_TOL_MPS2)
#define GRAVITY_M_TOL (GRAVITY_MPS2 - FOOT_STATIC_ACC_NORM_TOL_MPS2)
#define ACC_NORM_SQ_MAX GRAVITY_P_TOL*GRAVITY_P_TOL
#define ACC_NORM_SQ_MIN GRAVITY_M_TOL*GRAVITY_M_TOL

static float daOmegaX_inputs[] = {0,0};
static float daOmegaX_outputs[] = {0,0};
static float aOmegaX_inputs[] = {0,0};
static float aOmegaX_outputs[] = {0,0};
static float aOmegaY_inputs[] = {0,0};
static float aOmegaY_outputs[] = {0,0};
static float aOmegaZ_inputs[] = {0,0};
static float aOmegaZ_outputs[] = {0,0};
static float aAccX_inputs[] = {0,0};
static float aAccX_outputs[] = {0,0};
static float aAccY_inputs[] = {0,0};
static float aAccY_outputs[] = {0,0};
static float aAccZ_inputs[] = {0,0};
static float aAccZ_outputs[] = {0,0};
static float accNormSq_inputs[] = {0,0,0};
static float accNormSq_outputs[] = {0,0,0};
static float aa_dot_aOmegaX_error_inputs[] = {0,0,0};
static float aa_dot_aOmegaX_error_outputs[] = {0,0,0};

static struct kinematics_s kin;

static float ANKLE_TO_IMU_SAGITTAL_PLANE_M;

static void reset_position_and_velocity(struct taskmachine_s* tm){
    kin.vAy = 0.0;
    kin.vAz = 0.0;
    kin.pAy = 0.0;
    kin.pAz = 0.0;
    kin.latest_foot_static_samples = tm->elapsed_samples;
}

static void reset_integrals(){
    kin.iaOmegaX = 0.0;
    kin.iaAccY = 0.0;
    kin.iaAccZ = 0.0;
}

static void correct_orientation(){

//	float zAccWithCentripetalAccCompensation = kin.aAccZ;
//	float yAccWithTangentialAccCompensation = kin.aAccY;
	float zAccWithCentripetalAccCompensation = kin.aAccZ - (kin.aOmegaX*kin.aOmegaX*ANKLE_TO_IMU_SAGITTAL_PLANE_M);
		float yAccWithTangentialAccCompensation = kin.aAccY - (kin.daOmegaX*ANKLE_TO_IMU_SAGITTAL_PLANE_M);
	float accNormReciprocal	= 1.0/(sqrtf((yAccWithTangentialAccCompensation*yAccWithTangentialAccCompensation) + (zAccWithCentripetalAccCompensation*zAccWithCentripetalAccCompensation)));
	float costheta = zAccWithCentripetalAccCompensation*accNormReciprocal;
	float sintheta = yAccWithTangentialAccCompensation*accNormReciprocal;
	kin.rot1 = 0.99*kin.rot1 + 0.01*costheta;
	kin.rot3 = 0.99*kin.rot3 + 0.01*sintheta;
}

static void update_rotation_matrix(){
	float rotprev1 = kin.rot1;
	kin.rot1 = kin.rot1 - kin.rot3*kin.aOmegaX * SAMPLE_PERIOD_S;
	kin.rot3 = kin.rot3 + rotprev1*kin.aOmegaX * SAMPLE_PERIOD_S;

}


static void update_integrals_and_derivatives(){
//	kin.iaAccY = kin.iaAccY + kin.aAccY;
//	kin.daAccY = FILTA*kin.daAccY + FILTB*(kin.aAccY - kin.aAccYprev);
//	kin.iaAccZ = kin.iaAccZ + kin.aAccZ;
//	kin.daAccZ = FILTA*kin.daAccZ + FILTB*(kin.aAccZ - kin.aAccZprev);
//	kin.iaOmegaX = kin.iaOmegaX - kin.aOmegaX;

	kin.daOmegaX =  filter_first_order_butter_20hz(SAMPLE_RATE_HZ*(kin.aOmegaX - kin.aOmegaXprev), &daOmegaX_outputs[0], &daOmegaX_inputs[0]);
}

static void update_ankle_translations(){
	
	if (fabs(kin.pAz) < PAZ_MAX){
		float aOmegaXSquared = kin.aOmegaX*kin.aOmegaX;
		float SaAy = kin.aAccY - aOmegaXSquared*ANKLE_POS_IMU_FRAME_Y_M - kin.daOmegaX*ANKLE_POS_IMU_FRAME_Z_M;
		float SaAz = kin.aAccZ - aOmegaXSquared*ANKLE_POS_IMU_FRAME_Z_M + kin.daOmegaX*ANKLE_POS_IMU_FRAME_Y_M;

		kin.aAy = kin.rot1*SaAy - kin.rot3*SaAz;
		kin.aAz = kin.rot3*SaAy + kin.rot1*SaAz - GRAVITY_MPS2;

		kin.vAy = kin.vAy + SAMPLE_PERIOD_S*kin.aAy;
		kin.vAz = kin.vAz + SAMPLE_PERIOD_S*kin.aAz;

		kin.pAy = kin.pAy + SAMPLE_PERIOD_S*kin.vAy;
		kin.pAz = kin.pAz + SAMPLE_PERIOD_S*kin.vAz;

		float vAySq = kin.vAy*kin.vAy;
		float vAzSq = kin.vAz*kin.vAz;
		kin.sinSqAttackAngle = (((kin.vAz > 0) - (kin.vAz < 0))*vAzSq)/(vAySq+vAzSq);
	}


}



static void update_acc( struct fx_rigid_mn_s* mn){
	kin.aAccYprev = kin.aAccZ;
	kin.aAccZprev = kin.aAccZ;

	kin.aAccX =  filter_first_order_butter_20hz(kin.aAccXYZscaling * (float) mn->accel.x, &aAccX_outputs[0], &aAccX_inputs[0]);
	kin.aAccY =  filter_first_order_butter_20hz(kin.aAccXYZscaling * (float) mn->accel.z, &aAccY_outputs[0], &aAccY_inputs[0]);
	kin.aAccZ =  filter_first_order_butter_20hz(-1.0*kin.aAccXYZscaling * (float) mn->accel.y, &aAccZ_outputs[0], &aAccZ_inputs[0]);
}

static void update_omega(struct fx_rigid_mn_s* mn){
	kin.aOmegaXprev = kin.aOmegaX;

	kin.aOmegaX =  filter_first_order_butter_20hz(GYRO_RPS_PER_LSB * (float) mn->gyro.x + kin.aOmegaXbias, &aOmegaX_outputs[0], &aOmegaX_inputs[0]);
	kin.aOmegaY =  filter_first_order_butter_20hz(GYRO_RPS_PER_LSB * (float) mn->gyro.z + kin.aOmegaYbias, &aOmegaY_outputs[0], &aOmegaY_inputs[0]);
	kin.aOmegaZ =  filter_first_order_butter_20hz(GYRO_RPS_PER_LSB * (float) mn->gyro.y + kin.aOmegaZbias, &aOmegaZ_outputs[0], &aOmegaZ_inputs[0]);
}

static void correct_gyro_bias(){
	kin.aOmegaXbias = FILTC*kin.aOmegaXbias - FILTD*kin.aOmegaX;
	kin.aOmegaYbias = FILTC*kin.aOmegaYbias - FILTD*kin.aOmegaY;
	kin.aOmegaZbias = FILTC*kin.aOmegaZbias - FILTD*kin.aOmegaZ;
}



static void correct_accel_scaling(){
	kin.accNormSqRaw = kin.aAccX*kin.aAccX + kin.aAccY*kin.aAccY + kin.aAccZ*kin.aAccZ;
	kin.meanaccNormSq = FILTA*kin.meanaccNormSq + FILTB*kin.accNormSq;

	if (fabs(kin.meanaccNormSq - kin.accNormSq) < MIN_ACCEL_SUMSQR_MEAN_OFFSET_FOR_RESCALING){
		kin.accelQuietSamples = kin.accelQuietSamples + 1;
	}else{
		kin.accelQuietSamples = 0;
	}

	if (kin.accelQuietSamples > MIN_ACCEL_QUIET_SAMPLES_FOR_RESCALING){
		kin.aAccXYZscaling = kin.aAccXYZscaling * sqrtf(GRAVITY_SQ/kin.meanaccNormSq);
		kin.accelQuietSamples = 0;
	}
}

static void update_pose(struct taskmachine_s* tm){
		//kin.pitch = asinf(kin.rot3);

		float aa_dot_minus_aOmegaX = tm->aa_dot - kin.aOmegaX;

		kin.aa_dot_aOmegaX_error = filter_second_order_butter_5hz(aa_dot_minus_aOmegaX*aa_dot_minus_aOmegaX,
				&aa_dot_aOmegaX_error_outputs[0], &aa_dot_aOmegaX_error_inputs[0]);
		kin.accNormSq = filter_second_order_butter_5hz(kin.accNormSqRaw,
				&accNormSq_outputs[0], &accNormSq_inputs[0]);

       if (kin.aa_dot_aOmegaX_error < AA_DOT_AOMEGAX_ERROR_THRESH_RPS2 &&
           kin.accNormSq < ACC_NORM_SQ_MAX &&
               kin.accNormSq > ACC_NORM_SQ_MIN &&
               !tm->in_swing){
            kin.foot_flat = 1;
            reset_position_and_velocity(tm);
            reset_integrals();
            correct_orientation();
            kin.ground_slope_est = 0.99*kin.ground_slope_est + 0.01*(kin.rot3 - tm->aa);
        }    
       else{
            kin.foot_flat = 0;
       }
		update_integrals_and_derivatives();
		update_rotation_matrix();
		update_ankle_translations();

    
    if (tm->gait_event_trigger == GAIT_EVENT_FOOT_ON){
        reset_position_and_velocity(tm);
    }

}






void update_kinematics(struct fx_rigid_mn_s* mn, struct taskmachine_s* tm){

	update_acc(mn);
	update_omega(mn);
	correct_gyro_bias();
	correct_accel_scaling();
	update_pose(tm);

}


void init_kinematics(){

	ANKLE_TO_IMU_SAGITTAL_PLANE_M = sqrtf((ANKLE_POS_IMU_FRAME_Y_M*ANKLE_POS_IMU_FRAME_Y_M) + (ANKLE_POS_IMU_FRAME_Z_M*ANKLE_POS_IMU_FRAME_Z_M));

	kin.aOmegaX = 0.0;
    kin.aOmegaY = 0.0;
    kin.aOmegaZ = 0.0;
    kin.aAccX = 0.0;
    kin.aAccY = 0.0;
    kin.aAccZ = 0.0;
    kin.iaAccY = 0.0;
    kin.daAccY = 0.0;
    kin.iaAccZ = 0.0;
    kin.daAccZ = 0.0;
    kin.iaOmegaX = 0.0;
    kin.daOmegaX = 0.0;
    kin.aAccYprev = 0.0;
    kin.aAccZprev = 0.0;
    kin.aOmegaXprev = 0.0;
    kin.rot1 = 0.0;
    kin.rot3 = 0.0;
    kin.sinSqAttackAngle =  0;

    kin.aOmegaXbias = 0.0;
    kin.aOmegaYbias = 0.0;
    kin.aOmegaZbias = 0.0;

    kin.aAy = 0.0;
    kin.aAz = 0.0;
    kin.vAy = 0.0;
    kin.vAz = 0.0;
    kin.pAy = 0.0;
    kin.pAz = 0.0;

    kin.aAccXYZscaling = ACCEL_MPS2_PER_LSB;
    kin.accelQuietSamples = 0;
    kin.meanaccNormSq = 0.0;
    kin.accNormSq = 0.0;
    kin.accNormSqRaw = 0.0;

    kin.aa_dot_aOmegaX_error = 0.0;
    kin.latest_foot_static_samples = 0.0;

    kin.foot_flat = 0;
    kin.foot_flat_counter = 1;

    kin.inst_ground_slope_est = 0.0;
    kin.inst_ground_slope_est_sum = 0.0;
    kin.ground_slope_est = 0.0;
}

struct kinematics_s* get_kinematics(){
	return &kin;
}



