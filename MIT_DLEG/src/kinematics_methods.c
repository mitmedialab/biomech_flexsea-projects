


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

#define ACCEL_MPS2_PER_LSB 0.0012283 //Actual value different (but consistently so) from accelerometer documentation.
#define N_ACCEL_MPS2_PER_LSB (-1.0*ACCEL_MPS2_PER_LSB)
#define GYRO_RPS_PER_LSB (RAD_PER_DEG / GYRO_LSB_PER_DPS)
#define MIN_ACCEL_SUMSQR_MEAN_OFFSET_FOR_RESCALING 1.0
#define MIN_ACCEL_QUIET_SAMPLES_FOR_RESCALING 1000
#define PAZ_MAX 1.0


//Foot static constants
#define JOINT_VEL_SEG_VEL_DIFF_SQ_THRESH_R2PS2 1.5
#define MIN_SAMPLES_FOR_FOOT_FLAT 20
#define STATIC_ACC_NORM_TOL_MPS2 0.1
#define GRAVITY_P_TOL (GRAVITY_MPS2 + STATIC_ACC_NORM_TOL_MPS2)
#define GRAVITY_M_TOL (GRAVITY_MPS2 - STATIC_ACC_NORM_TOL_MPS2)
#define ACC_NORM_SQ_MAX (GRAVITY_P_TOL*GRAVITY_P_TOL)
#define ACC_NORM_SQ_MIN (GRAVITY_M_TOL*GRAVITY_M_TOL)
#define TQ_DOT_THRESH_NM_HZ 50

static float aAccX_inputs[] = {0,0};
static float aAccX_outputs[] = {0,0};
static float aAccY_inputs[] = {0,0};
static float aAccY_outputs[] = {0,0};
static float aAccZ_inputs[] = {0,0};
static float aAccZ_outputs[] = {0,0};

static struct kinematics_s kin;

static float ANKLE_TO_IMU_SAGITTAL_PLANE_M;

static void reset_position_and_velocity(struct taskmachine_s* tm){
    kin.vAy = 0.0;
    kin.vAz = 0.0;
    kin.pAy = 0.0;
    kin.pAz = 0.0;
    kin.latest_foot_static_samples = tm->elapsed_samples;
}

// Correct orientation using accelerometers, and update the ground slope.
// looks for y component in gravity as recorded by imu during stance
// sets rot1 & rot3 based on gravity direction
// estimates ground slope as the diff between ankle angle and gravity dir
static void correct_orientation_and_update_ground_slope(struct taskmachine_s* tm){
	static float theta_quiet_acc = 0.0;
	theta_quiet_acc = kin.aAccY*GRAVITY_RECIP;
	kin.ground_slope_est_sum = kin.ground_slope_est_sum + tm->aa - theta_quiet_acc;
	kin.ground_slope_estimation_counter =  kin.ground_slope_estimation_counter + 1;
	kin.rot3 = 0.98*kin.rot3 + 0.02*theta_quiet_acc;
	kin.rot1 = sqrtf(1.0-(kin.rot3*kin.rot3));
}

// Update 2D rotation matrix using gyroscopes only.
// uses small angle approx for cos(th) => 1 and sin(th) => th
// TODO: replace rot1 and rot3 for rot1prev and rot3prev for consistency
static void update_rotation_matrix(){
	static float rot1prev, rot3prev;
	rot1prev = kin.rot1;
	rot3prev = kin.rot3;
	kin.rot1 = kin.rot1 - kin.rot3*kin.aOmegaX * SAMPLE_PERIOD_S;
	kin.rot3 = kin.rot3 + rot1prev*kin.aOmegaX * SAMPLE_PERIOD_S;

}

// Update 2D ankle translations and the displacement from starting point.
// If the foot has not been lifted to high:
// find acceleration of ankle joint center (SsAy and SsAz)
// find absolute acceleration at joint center (aAy and aAz)
// Integrate to find velocity and position
// ???: wouldn't this type of integration overestimate the effect of acceleration on position? ( p = v*t + a*t^2 ) vs ( p = v*t + 1/2*a*t^2 )
static void update_ankle_translations(){
	static float aOmegaXSquared, SaAy, SaAz;
	if (fabs(kin.pAz) < PAZ_MAX){
		aOmegaXSquared = kin.aOmegaX*kin.aOmegaX;
		SaAy = kin.aAccY - aOmegaXSquared*ANKLE_POS_IMU_FRAME_Y_M - kin.daOmegaX*SAMPLE_RATE_HZ*ANKLE_POS_IMU_FRAME_Z_M;
		SaAz = kin.aAccZ - aOmegaXSquared*ANKLE_POS_IMU_FRAME_Z_M + kin.daOmegaX*SAMPLE_RATE_HZ*ANKLE_POS_IMU_FRAME_Y_M;

		kin.aAy = kin.rot1*SaAy - kin.rot3*SaAz;
		kin.aAz = kin.rot3*SaAy + kin.rot1*SaAz - GRAVITY_MPS2;

		kin.vAy = kin.vAy + SAMPLE_PERIOD_S*kin.aAy;
		kin.vAz = kin.vAz + SAMPLE_PERIOD_S*kin.aAz;

		kin.pAy = kin.pAy + SAMPLE_PERIOD_S*kin.vAy;
		kin.pAz = kin.pAz + SAMPLE_PERIOD_S*kin.vAz;
		kin.displacement = sqrtf(kin.pAy*kin.pAy + kin.pAz*kin.pAz);
	}

}



//Gets low pass filtered 3D accelerations
static void update_acc( struct fx_rigid_mn_s* mn){
	kin.aAccX =  filter_first_order_butter_20hz(kin.aAccXYZscaling * (float) mn->accel.x, &aAccX_outputs[0], &aAccX_inputs[0]);
	kin.aAccY =  filter_first_order_butter_20hz(kin.aAccXYZscaling * (float) mn->accel.z, &aAccY_outputs[0], &aAccY_inputs[0]);
	kin.aAccZ =  filter_first_order_butter_20hz(-1.0*kin.aAccXYZscaling * (float) mn->accel.y, &aAccZ_outputs[0], &aAccZ_inputs[0]);
}

//Gets high pass filtered sagittal plane angular velocity
static void update_omega(struct fx_rigid_mn_s* mn){
	static float aOmegaXprev;
	aOmegaXprev = kin.aOmegaX;
	kin.aOmegaXbias = FILTC*kin.aOmegaXbias - FILTD*kin.aOmegaX;
	kin.aOmegaX =  GYRO_RPS_PER_LSB * (float) mn->gyro.x + kin.aOmegaXbias;
	kin.daOmegaX =  kin.aOmegaX - aOmegaXprev;

}

//Resets scale factor on acceleration calculations using a sufficiently long period where accel signals are quiet.
//This typically occurs during static standing.
static void correct_accel_scaling(){

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

//Updates sagittal shank rotation, ankle translations, and ground slope.
static void update_pose(struct taskmachine_s* tm){
	update_rotation_matrix();
	update_ankle_translations();

	//Get the difference between segment velocity and ankle joint velocity and use this as an indicator of
	//foot-flat if this value is below a certain threshold during stance.
	float joint_vel_seg_vel_diff = tm->aa_dot - kin.aOmegaX;
	kin.joint_vel_seg_vel_diff_sq = joint_vel_seg_vel_diff*joint_vel_seg_vel_diff;

	//Get acc norm for use in determining a quiet enough point to use the accelerometers to correct the orientation estimate.
	kin.accNormSq = kin.aAccX*kin.aAccX + kin.aAccY*kin.aAccY + kin.aAccZ*kin.aAccZ;

	kin.foot_flat = 0;
	if (!tm->in_swing){

		//If acceleration is quiet use it to correct orientation and update the ground slope.
		if (kin.accNormSq < ACC_NORM_SQ_MAX &&
			   kin.accNormSq > ACC_NORM_SQ_MIN){
			correct_orientation_and_update_ground_slope(tm);
		}

		//If the difference between joint and segment velocity is small, set foot-flat to 1.
		if (kin.joint_vel_seg_vel_diff_sq < JOINT_VEL_SEG_VEL_DIFF_SQ_THRESH_R2PS2 &&
			tm->tq_dot > TQ_DOT_THRESH_NM_HZ){
			kin.foot_flat = 1;
		}
	}
	if (kin.foot_flat)
		kin.foot_flat_counter = kin.foot_flat_counter + 1;
	else{
		kin.foot_flat_counter = 0;
	}


	if (kin.foot_flat_counter > MIN_SAMPLES_FOR_FOOT_FLAT){
		reset_position_and_velocity(tm);
	}



	if (tm->gait_event_trigger == GAIT_EVENT_FOOT_ON){
		kin.end_of_stride_pAz = kin.pAz;
		reset_position_and_velocity(tm);
		 kin.ground_slope_estimation_counter = 1;
		kin.ground_slope_est_sum = 0.0;
	}

	if (tm->gait_event_trigger == GAIT_EVENT_FOOT_OFF){
		 kin.curr_ground_slope_est = kin.ground_slope_est_sum/((float)kin.ground_slope_estimation_counter);
	}
}





//Takes the raw IMU measurements and uses them to update ankle joint translations and ground slope.
void update_kinematics(struct fx_rigid_mn_s* mn, struct taskmachine_s* tm){

	update_acc(mn);
	update_omega(mn);
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
    kin.ground_slope_estimation_counter = 1;
    kin.ground_slope_est_sum = 0.0;

    kin.joint_vel_seg_vel_diff_sq = 0.0;
    kin.latest_foot_static_samples = 0.0;

    kin.foot_flat = 0;
    kin.foot_flat_counter = 1;
}

struct kinematics_s* get_kinematics(){
	return &kin;
}



