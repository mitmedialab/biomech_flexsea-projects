


#include "kinematics_methods.h"



 //Kinematics constants (copied from matlab pil)
#define GRAVITY_MPS2 9.8
#define GRAVITY_RECIP (1.0/GRAVITY_MPS2)
#define GRAVITY_SQ (GRAVITY_MPS2*GRAVITY_MPS2)
#define GYRO_LSB_PER_DPS 32.8  //per http://dephy.com/wiki/flexsea/doku.php?id=units
#define ACCEL_LSB_PER_G 8192.0   //per http://dephy.com/wiki/flexsea/doku.php?id=units
#define ANKLE_POS_IMU_FRAME_X_M 0.0  //Frontal axis (medial->lateral)
#define ANKLE_POS_IMU_FRAME_Y_M -0.00445  //Longitudinal axis (bottom->top)
#define ANKLE_POS_IMU_FRAME_Z_M -0.0605  //Sagittal axis (back->front)
#define ACCEL_MPS2_PER_LSB  (GRAVITY_MPS2 / ACCEL_LSB_PER_G)
#define N_ACCEL_MPS2_PER_LSB (-1.0*ACCEL_MPS2_PER_LSB)
#define GYRO_RPS_PER_LSB (RAD_PER_DEG / GYRO_LSB_PER_DPS)
#define MIN_ACCEL_SUMSQR_MEAN_OFFSET_FOR_RESCALING 1.0
#define MIN_ACCEL_QUIET_SAMPLES_FOR_RESCALING 1000
#define PAZ_MAX 1.0


//Foot static constants
#define JOINT_VEL_SEG_VEL_DIFF_SQ_THRESH_R2PS2 1.0
#define MIN_SAMPLES_FOR_FOOT_FLAT 10
#define STATIC_ACC_NORM_TOL_MPS2 0.1
#define GRAVITY_P_TOL (GRAVITY_MPS2 + STATIC_ACC_NORM_TOL_MPS2)
#define GRAVITY_M_TOL (GRAVITY_MPS2 - STATIC_ACC_NORM_TOL_MPS2)
#define ACC_NORM_SQ_MAX (GRAVITY_P_TOL*GRAVITY_P_TOL)
#define ACC_NORM_SQ_MIN (GRAVITY_M_TOL*GRAVITY_M_TOL)
#define MAX_ROLL_OVER_PITCH 0.1
#define MIN_ROLL_OVER_PITCH -0.25
#define TQ_DOT_THRESH_NM_HZ 50

static float daOmegaX_inputs[] = {0,0};
static float daOmegaX_outputs[] = {0,0};
static float aOmegaX_inputs[] = {0,0};
static float aOmegaX_outputs[] = {0,0};
static float aAccX_inputs[] = {0,0};
static float aAccX_outputs[] = {0,0};
static float aAccY_inputs[] = {0,0};
static float aAccY_outputs[] = {0,0};
static float aAccZ_inputs[] = {0,0};
static float aAccZ_outputs[] = {0,0};
static float accNormSq_inputs[] = {0,0,0,0,0};
static float accNormSq_outputs[] = {0,0,0,0,0};
static float joint_vel_seg_vel_diff_sq_inputs[] = {0,0,0,0,0};
static float joint_vel_seg_vel_diff_sq_outputs[] = {0,0,0,0,0};

static struct kinematics_s kin;

static float ANKLE_TO_IMU_SAGITTAL_PLANE_M;

static void reset_position_and_velocity(struct taskmachine_s* tm){
    kin.vAy = 0.0;
    kin.vAz = 0.0;
    kin.pAy = 0.0;
    kin.pAz = 0.0;
    kin.latest_foot_static_samples = tm->elapsed_samples;
}

//static void reset_integrals(){
//    kin.iaOmegaX = 0.0;
//    kin.iaAccY = 0.0;
//    kin.iaAccZ = 0.0;
//}

static void correct_orientation(){

	float g2_minus_aAccY2 = GRAVITY_SQ - kin.aAccY*kin.aAccY;
	if (g2_minus_aAccY2 < 0.0){
		g2_minus_aAccY2 = 0.0;
	}
	float costheta = sqrtf(g2_minus_aAccY2)*GRAVITY_RECIP;
	float sintheta = kin.aAccY*GRAVITY_RECIP;
	kin.rot1 = 0.99*kin.rot1 + 0.01*costheta;
	kin.rot3 = 0.99*kin.rot3 + 0.01*sintheta;
}

static void update_rotation_matrix(){
	float rotprev1 = kin.rot1;
	kin.rot3prev = kin.rot3;
	kin.rot1 = kin.rot1 - kin.rot3*kin.aOmegaX * SAMPLE_PERIOD_S;
	kin.rot3 = kin.rot3 + rotprev1*kin.aOmegaX * SAMPLE_PERIOD_S;

}


static void update_integrals_and_derivatives(){
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
	}

}



static void update_acc( struct fx_rigid_mn_s* mn){
	kin.aAccX =  filter_first_order_butter_20hz(kin.aAccXYZscaling * (float) mn->accel.x, &aAccX_outputs[0], &aAccX_inputs[0]);
	kin.aAccY =  filter_first_order_butter_20hz(kin.aAccXYZscaling * (float) mn->accel.z, &aAccY_outputs[0], &aAccY_inputs[0]);
	kin.aAccZ =  filter_first_order_butter_20hz(-1.0*kin.aAccXYZscaling * (float) mn->accel.y, &aAccZ_outputs[0], &aAccZ_inputs[0]);
}

static void update_omega(struct fx_rigid_mn_s* mn){
	kin.aOmegaXprev = kin.aOmegaX;
	kin.aOmegaX =  filter_first_order_butter_20hz(GYRO_RPS_PER_LSB * (float) mn->gyro.x + kin.aOmegaXbias, &aOmegaX_outputs[0], &aOmegaX_inputs[0]);
}

static void correct_gyro_bias(){
	kin.aOmegaXbias = FILTC*kin.aOmegaXbias - FILTD*kin.aOmegaX;
}



static void correct_accel_scaling(){
	kin.accNormSqRaw = kin.aAccX*kin.aAccX + kin.aAccY*kin.aAccY + kin.aAccZ*kin.aAccZ;
	kin.meanaccNormSq = FILTA*kin.meanaccNormSq + FILTB*kin.accNormSqRaw;

	if (fabs(kin.meanaccNormSq - kin.accNormSqRaw) < MIN_ACCEL_SUMSQR_MEAN_OFFSET_FOR_RESCALING){
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
	update_integrals_and_derivatives();
	update_rotation_matrix();
	update_ankle_translations();

	float segvel = (kin.rot3 - kin.rot3prev)*SAMPLE_RATE_HZ;
	float joint_vel_seg_vel_diff = tm->aa_dot - segvel;
	kin.joint_vel_seg_vel_diff_sq = joint_vel_seg_vel_diff*joint_vel_seg_vel_diff;
//	kin.joint_vel_seg_vel_diff_sq = filter_fourth_order_butter_20hz(joint_vel_seg_vel_diff_sq_raw,
//			&joint_vel_seg_vel_diff_sq_outputs[0], &joint_vel_seg_vel_diff_sq_inputs[0]);
	kin.accNormSq = kin.accNormSqRaw;
//	kin.accNormSq = filter_fourth_order_butter_20hz(kin.accNormSqRaw, &accNormSq_outputs[0], &accNormSq_inputs[0]);
	kin.foot_flat = 0;
	kin.rolling_over_foot = 0;
	if (!tm->in_swing){
		if (kin.accNormSq < ACC_NORM_SQ_MAX &&
			   kin.accNormSq > ACC_NORM_SQ_MIN){
		   correct_orientation();
		}
		if (kin.rot3 > MIN_ROLL_OVER_PITCH &&
				kin.rot3 < MAX_ROLL_OVER_PITCH){
			kin.rolling_over_foot = 1;
			kin.ground_slope_est_sum = kin.ground_slope_est_sum + tm->aa;
			kin.roll_over_counter =  kin.roll_over_counter + 1;
		}

		if (kin.joint_vel_seg_vel_diff_sq < JOINT_VEL_SEG_VEL_DIFF_SQ_THRESH_R2PS2 &&
			tm->tq_dot > TQ_DOT_THRESH_NM_HZ){
			kin.foot_flat = 1;
		}
	}
	if (kin.foot_flat)
		kin.foot_flat_counter = kin.foot_flat_counter + 1;
	else
		kin.foot_flat_counter = 0;

	if (kin.foot_flat_counter > MIN_SAMPLES_FOR_FOOT_FLAT){
		reset_position_and_velocity(tm);
	}



	if (tm->gait_event_trigger == GAIT_EVENT_FOOT_ON){
		reset_position_and_velocity(tm);
		 kin.roll_over_counter = 1;
		kin.ground_slope_est_sum = 0.0;
	}

	if (tm->gait_event_trigger == GAIT_EVENT_FOOT_OFF){
		 kin.prev_ground_slope_est = kin.curr_ground_slope_est;
		 kin.curr_ground_slope_est = kin.ground_slope_est_sum/((float)kin.roll_over_counter);
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
    kin.aAccX = 0.0;
    kin.aAccY = 0.0;
    kin.aAccZ = 0.0;
    kin.daOmegaX = 0.0;
    kin.aOmegaXprev = 0.0;
    kin.rot1 = 0.0;
    kin.rot3 = 0.0;

    kin.aOmegaXbias = 0.0;

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
    kin.rolling_over_foot = 0;
    kin.roll_over_counter = 1;
    kin.ground_slope_est_sum = 0.0;

    kin.joint_vel_seg_vel_diff_sq = 0.0;
    kin.latest_foot_static_samples = 0.0;

    kin.foot_flat = 0;
    kin.foot_flat_counter = 1;
}

struct kinematics_s* get_kinematics(){
	return &kin;
}



