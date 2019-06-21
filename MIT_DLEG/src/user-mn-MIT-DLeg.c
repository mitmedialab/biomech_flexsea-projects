/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-user' User projects
	Copyright (C) 2016 Dephy, Inc. <http://dephy.com/>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*****************************************************************************

	[Lead developer] Jean-Francois Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors] Tony Shu, tony shu at mit dot edu, Matthew Carney mcarney at mit dot edu
*****************************************************************************
	[This file] user-mn-MIT_DLeg_2dof: User code running on Manage
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2018-02-24 | jfduval | New release
****************************************************************************/

#if defined INCLUDE_UPROJ_MIT_DLEG || defined BOARD_TYPE_FLEXSEA_PLAN
#if defined BOARD_TYPE_FLEXSEA_MANAGE || defined BOARD_TYPE_FLEXSEA_PLAN

//****************************************************************************
// Include(s)
//****************************************************************************

#include "user-mn-MIT-DLeg.h"
#include "actuator_functions.h"
#include "safety_functions.h"
//#include "walking_state_machine.h"	// Included to allow UserWrites to update walking machine controller.
#include "walking_knee_ankle_state_machine.h"
#include "run_main_user_application.h"	// This is where user application functions live
#include "ui.h"

#define SCALE_FACTOR_100 100.0
#define SCALE_FACTOR_1000 1000.0
#define SCALE_FACTOR_10000 10000.0
#define SCALE_FACTOR_ONE 1.0

//****************************************************************************
// Variable(s)
//****************************************************************************


uint8_t mitDlegInfo[2] = {PORT_RS485_1, PORT_RS485_1};
uint8_t enableMITfsm2 = 0, mitFSM2ready = 0, mitCalibrated = 0;
#define THIS_ACTPACK		0
#define SLAVE_ACTPACK		1

// Initiate some variables that may be used for testing
#if defined(IS_ACTUATOR_TESTING)
	float freqInput 	= 0.0;
	float freqRad 		= 0.0;
	float inputTheta 	= 0.0;
	float inputK 		= 0.0;
	float inputB 		= 0.0;
	float inputTorq 	= 0.0;
	int8_t currentOrVoltage = 0;
#elif defined(IS_SWEEP_TEST) || defined(IS_SWEEP_CHIRP_TEST)
	float freq									= 0.0;
	float amplitude								= 0.0;
	float dcBias								= 0.0;
	float noiseAmp								= 0.0;
	float freqFinal								= 0.0;
	int16_t chirpType							= 0;
	int16_t begin 								= 0;
	float freqSweepTime							= 0.0;
#endif


int8_t onEntry = 0;
Act_s act1, act2;

// EXTERNS
#if defined(IS_ANKLE)
	extern GainParams ankleGainsEst;
	extern GainParams ankleGainsMst;
	extern GainParams ankleGainsLst;
	extern GainParams ankleGainsEsw;
	extern GainParams ankleGainsLsw;
#elif defined(IS_KNEE)
	extern GainParams kneeGainsEst;
	extern GainParams kneeGainsMst;
	extern GainParams kneeGainsLst;
	extern GainParams kneeGainsEsw;
	extern GainParams kneeGainsLsw;
#endif

extern uint8_t calibrationFlags, calibrationNew;
extern int32_t currentOpLimit;
extern int8_t zeroLoadCell; 		// used for zeroing the load cell.
extern float voltageGain;
extern float velGain;
extern float indGain;

extern float torqueKp;
extern float torqueKi;
extern float torqueKd;
extern float errorKi;

extern int16_t splineTime;

static int gui_mode = GUI_MODE_NOM_CONTROL_PARAMS;
static int gui_mode_prev = GUI_MODE_NOM_CONTROL_PARAMS;

//****************************************************************************
// Macro(s)
//****************************************************************************

static void syncUserWritesWithCurrentParameterValues(struct taskmachine_s* tm){

	user_data_1.w[0] = gui_mode;

	switch (gui_mode){		
	    case GUI_MODE_FL_CONTROL_PARAMS: //0
	    case GUI_MODE_UR_CONTROL_PARAMS: //1
	    case GUI_MODE_DR_CONTROL_PARAMS: //2
	    case GUI_MODE_US_CONTROL_PARAMS: //3
	    case GUI_MODE_DS_CONTROL_PARAMS: //4
		case GUI_MODE_NOM_CONTROL_PARAMS: //5
		case GUI_MODE_DYNAMICS: //10
			if (gui_mode != GUI_MODE_DYNAMICS){
				tm->control_mode = gui_mode % 6;
			}
			if (tm->control_mode == MODE_NOMINAL){
				user_data_1.w[3] = (int32_t)(get_control_params()->nominal.theta_rad*SCALE_FACTOR_10000);
				user_data_1.w[4] = (int32_t)(get_control_params()->nominal.k_Nm_p_rad);
				user_data_1.w[5] = (int32_t)(get_control_params()->nominal.b_Nm_p_rps);
				user_data_1.w[6] = (int32_t)(0);
				user_data_1.w[7] = (int32_t)(0);
				user_data_1.w[8] = (int32_t)(0);
				user_data_1.w[9] = (int32_t)(0);
			}else{
				user_data_1.w[1] = (int32_t)(get_control_params()->adaptive.hard_stop_theta_rad[tm->control_mode]*SCALE_FACTOR_10000);
				user_data_1.w[2] = (int32_t)(get_control_params()->adaptive.hard_stop_k_Nm_p_rad[tm->control_mode]);
				user_data_1.w[3] = (int32_t)(get_control_params()->adaptive.lsw_theta_rad[tm->control_mode]*SCALE_FACTOR_10000);
				user_data_1.w[4] = (int32_t)(get_control_params()->adaptive.est_k_Nm_p_rad[tm->control_mode]);
				user_data_1.w[5] = (int32_t)(get_control_params()->adaptive.est_b_Nm_p_rps[tm->control_mode]);
				user_data_1.w[6] = (int32_t)(get_control_params()->adaptive.lst_k_Nm_p_rad[tm->control_mode]);
				user_data_1.w[7] = (int32_t)(get_control_params()->adaptive.lst_b_Nm_p_rps[tm->control_mode]);
				user_data_1.w[8] = (int32_t)(get_control_params()->adaptive.lst_theta_rad[tm->control_mode]*SCALE_FACTOR_10000);
				user_data_1.w[9] = (int32_t)(get_control_params()->adaptive.lst_engagement_tq_Nm[tm->control_mode]);
			}
	    break;

		case GUI_MODE_SW_CONTROL_PARAMS: //6
			tm->control_mode = MODE_POSITION;
			user_data_1.w[3] = (int32_t)(get_control_params()->active.esw_theta_rad*SCALE_FACTOR_10000);
			user_data_1.w[4] = (int32_t)(get_control_params()->active.sw_k_Nm_p_rad);
			user_data_1.w[5] = (int32_t)(get_control_params()->active.sw_b_Nm_p_rps);
			user_data_1.w[6] = (int32_t)(0);
			user_data_1.w[7] = (int32_t)(0);
			user_data_1.w[8] = (int32_t)(0);
			user_data_1.w[9] = (int32_t)(0);
		break;

		case GUI_MODE_ADAPTIVE_CONTROL: //7
		case GUI_MODE_GAIT_EVENTS: //8
		case GUI_MODE_LEARNING: //9
		case GUI_MODE_KINEMATICS1: //12
		case GUI_MODE_KINEMATICS2: //13
	    case GUI_MODE_FEATURES: //14
	    case GUI_MODE_SAFETY: //15
	    case GUI_MODE_STATISTICS: //18
	    case GUI_MODE_PREDICTION_ACCURACY: //16
	    case GUI_MODE_PREDICTOR: //19
	    	user_data_1.w[1] = tm->control_mode;
	    	user_data_1.w[2] = (int32_t)(0);
			user_data_1.w[3] = (int32_t)(0);
			user_data_1.w[4] = (int32_t)(0);
			user_data_1.w[5] = (int32_t)(0);
			user_data_1.w[6] = (int32_t)(0);
			user_data_1.w[7] = (int32_t)(0);
			user_data_1.w[8] = (int32_t)(0);
			user_data_1.w[9] = (int32_t)(0);

	    break;
	    case GUI_MODE_BACK_ESTIMATION: //11
	    	user_data_1.w[1] = tm->control_mode;
	    	user_data_1.w[2] =  (int32_t) (get_back_estimator()->us_z_thresh_m*10000.0);
	    	user_data_1.w[3] =  (int32_t) (get_back_estimator()->ds_z_thresh_m*10000.0);
	    	user_data_1.w[4] =  (int32_t) (get_back_estimator()->us_z_max_samples);
	    	user_data_1.w[5] =  (int32_t) (get_back_estimator()->ds_z_max_samples);
	    	user_data_1.w[6] =  (int32_t) (get_back_estimator()->ur_slope_thresh_rad*10000.0);
	    	user_data_1.w[7] =  (int32_t) (get_back_estimator()->dr_slope_thresh_rad*10000.0);
			break;
	    case GUI_MODE_TASK_MACHINE: //17
	    	break;
	}
		
}

static void initializeUserWrites(struct taskmachine_s* tm){
	gui_mode = GUI_MODE_NOM_CONTROL_PARAMS;
	tm->control_mode = MODE_NOMINAL;
	user_data_1.w[0] = gui_mode;
	user_data_1.w[1] = (int32_t)(0);
	user_data_1.w[2] = (int32_t)(0);
	user_data_1.w[3] = (int32_t)(DEFAULT_NOMINAL_THETA_RAD*SCALE_FACTOR_10000);
	user_data_1.w[4] = (int32_t)(DEFAULT_NOMINAL_K_NM_P_RAD);
	user_data_1.w[5] = (int32_t)(DEFAULT_NOMINAL_B_NM_P_RPS);
	user_data_1.w[6] = (int32_t)(0);
	user_data_1.w[7] = (int32_t)(0);
	user_data_1.w[8] = (int32_t)(0);
	user_data_1.w[9] = (int32_t)(0);
}

/*
 * Updates the Input values based off of user data
 * Param: actx(Act_s) - Actuator structure to track sensor values
 * Param: wParams(WalkParams) -
 */
static void updateUserWrites(struct taskmachine_s* tm){

	gui_mode_prev = gui_mode;
	gui_mode = user_data_1.w[0];
	if (gui_mode_prev != gui_mode){
		syncUserWritesWithCurrentParameterValues(tm);
	}

	switch (gui_mode){

	    case GUI_MODE_FL_CONTROL_PARAMS: //0
	    case GUI_MODE_UR_CONTROL_PARAMS: //1
	    case GUI_MODE_DR_CONTROL_PARAMS: //2
	    case GUI_MODE_US_CONTROL_PARAMS: //3
	    case GUI_MODE_DS_CONTROL_PARAMS: //4
	    case GUI_MODE_NOM_CONTROL_PARAMS: //5
	    case GUI_MODE_DYNAMICS: //10
	    	tm->do_update_learner = 0;
	    	if (tm->control_mode == MODE_NOMINAL)
	    	{
	    		set_nominal_theta_rad((float) user_data_1.w[3]  /SCALE_FACTOR_10000);
	    		set_nominal_k_Nm_p_rad((float) user_data_1.w[4] );
	    		set_nominal_b_Nm_p_rps((float) user_data_1.w[5]);
	    	}else{
				set_hard_stop_theta_rad((float) user_data_1.w[1]/SCALE_FACTOR_10000, tm->control_mode);
				set_hard_stop_k_Nm_p_rad((float) user_data_1.w[2], tm->control_mode);
				set_lsw_theta_rad(((float) user_data_1.w[3])/SCALE_FACTOR_10000, tm->control_mode);
				set_est_k_Nm_p_rad(((float) user_data_1.w[4]), tm->control_mode);
				set_est_b_Nm_p_rps((float) user_data_1.w[5], tm->control_mode);
				set_lst_k_Nm_p_rad((float) user_data_1.w[6], tm->control_mode);
				set_lst_b_Nm_p_rps((float) user_data_1.w[7], tm->control_mode);
				set_lst_theta_rad((float) user_data_1.w[8]/SCALE_FACTOR_10000, tm->control_mode);
				set_lst_engagement_tq_Nm((float) user_data_1.w[9], tm->control_mode);
	    	}
		break;
		case GUI_MODE_SW_CONTROL_PARAMS: //6
			tm->do_update_learner = 0;
			set_esw_theta_rad((float) user_data_1.w[3]/SCALE_FACTOR_10000);
			set_sw_k_Nm_p_rad((float) user_data_1.w[4]);
			set_sw_b_Nm_p_rps((float) user_data_1.w[5]);
			enable_minimum_jerk((uint8_t) user_data_1.w[6]);

		break;
		case GUI_MODE_ADAPTIVE_CONTROL: //7
		case GUI_MODE_GAIT_EVENTS: //8
		case GUI_MODE_KINEMATICS1: //12
		case GUI_MODE_KINEMATICS2: //13
		case GUI_MODE_FEATURES: //14
		case GUI_MODE_SAFETY: //15
		case GUI_MODE_PREDICTION_ACCURACY: //16
			tm->do_update_learner = 0;
			tm->control_mode = user_data_1.w[1];
			if (user_data_1.w[2] == 1)
				reset_learning_structs();
	    break;
	    case GUI_MODE_LEARNING: //9
	    case GUI_MODE_PREDICTOR: //19
	    case GUI_MODE_STATISTICS: //18
	    	tm->do_update_learner = 1;
	    	tm->control_mode = user_data_1.w[1];
	    	if (user_data_1.w[2] == 1)
	    		reset_learning_structs();
	    break;
	    case GUI_MODE_BACK_ESTIMATION: //11
	    	tm->do_update_learner = 0;
	    	tm->control_mode = user_data_1.w[1];
			set_us_z_thresh_m(((float)user_data_1.w[2])/10000.0);
			set_ds_z_thresh_m(((float)user_data_1.w[3])/10000.0);
			set_us_z_max_samples(user_data_1.w[4]);
			set_ds_z_max_samples(user_data_1.w[5]);
			set_ur_slope_thresh_rad(((float)user_data_1.w[6])/10000.0);
			set_dr_slope_thresh_rad(((float)user_data_1.w[7])/10000.0);
			break;
	    case GUI_MODE_TASK_MACHINE: //17
	    	tm->do_update_learner = 0;
	    	tm->control_mode = MODE_ADAPTIVE_WITH_LEARNING;
	    	break;



	}


}

static void updateGenVars(struct taskmachine_s* tm){


	int16_t guimode_controlmode_state_inswing = gui_mode*1000 + tm->control_mode*100 + get_walking_state()*10 + tm->in_swing;
	rigid1.mn.genVar[0] = guimode_controlmode_state_inswing;
	rigid1.mn.genVar[1] = (int16_t) (tm->tq*SCALE_FACTOR_100);
	rigid1.mn.genVar[2] = (int16_t) (tm->aa*SCALE_FACTOR_10000);

	switch (gui_mode){
		
	    case GUI_MODE_FL_CONTROL_PARAMS://0
	    case GUI_MODE_UR_CONTROL_PARAMS: //1
	    case GUI_MODE_DR_CONTROL_PARAMS: //2
	    case GUI_MODE_US_CONTROL_PARAMS: //3
	    case GUI_MODE_DS_CONTROL_PARAMS: //4
	    	rigid1.mn.genVar[1] = (int16_t)(get_control_params()->adaptive.hard_stop_theta_rad[tm->control_mode]*SCALE_FACTOR_10000);
	    	rigid1.mn.genVar[2] = (int16_t)(get_control_params()->adaptive.hard_stop_k_Nm_p_rad[tm->control_mode]);
	    	rigid1.mn.genVar[3] = (int16_t)(get_control_params()->adaptive.lsw_theta_rad[tm->control_mode]*SCALE_FACTOR_10000);
	    	rigid1.mn.genVar[4] = (int16_t)(get_control_params()->adaptive.est_k_Nm_p_rad[tm->control_mode]);
	    	rigid1.mn.genVar[5] = (int16_t)(get_control_params()->adaptive.est_b_Nm_p_rps[tm->control_mode]);
	    	rigid1.mn.genVar[6] = (int16_t)(get_control_params()->adaptive.lst_k_Nm_p_rad[tm->control_mode]);
	    	rigid1.mn.genVar[7] = (int16_t)(get_control_params()->adaptive.lst_b_Nm_p_rps[tm->control_mode]);
	    	rigid1.mn.genVar[8] = (int16_t)(get_control_params()->adaptive.lst_theta_rad[tm->control_mode]*SCALE_FACTOR_10000);
	    	rigid1.mn.genVar[9] = (int16_t)(get_control_params()->adaptive.lst_engagement_tq_Nm[tm->control_mode]);
//	    	rigid1.mn.genVar[7] = (int16_t)(act1.tauDes*SCALE_FACTOR_100);
//			rigid1.mn.genVar[8] = (int16_t)(tm->aa_dot_15hz_filt*SCALE_FACTOR_100);
//			rigid1.mn.genVar[9] = (int16_t)(act1.thetaDes*10000.0);

		break;
	    case GUI_MODE_NOM_CONTROL_PARAMS: //5
	    	rigid1.mn.genVar[3] = (int16_t)(get_control_params()->nominal.theta_rad*SCALE_FACTOR_10000);
	    	rigid1.mn.genVar[4] = (int16_t)(get_control_params()->nominal.k_Nm_p_rad);
	    	rigid1.mn.genVar[5] = (int16_t)(get_control_params()->nominal.b_Nm_p_rps);
	    	rigid1.mn.genVar[6] = (int16_t) (act1.axialForce*10.0);
			rigid1.mn.genVar[7] = (int16_t) (act1.linkageMomentArm*100000.0);
			rigid1.mn.genVar[8] = (int16_t) (act1.jointTorqueLC*10000.0);
			rigid1.mn.genVar[9] = (int16_t) (act1.jointTorque*10000.0);
	    break;
		case GUI_MODE_SW_CONTROL_PARAMS: //6
			rigid1.mn.genVar[3] = (int16_t) (get_control_params()->active.esw_theta_rad*SCALE_FACTOR_10000);
			rigid1.mn.genVar[4] = (int16_t) ((float) ( *rigid1.ex.enc_ang - act1.motorPos0) /10.0);
			rigid1.mn.genVar[5] = (int16_t) (act1.theta0*SCALE_FACTOR_10000);
			rigid1.mn.genVar[6] = (int16_t) (act1.jointTorque*SCALE_FACTOR_100);
			rigid1.mn.genVar[7] = (int16_t) (act1.linkageMomentArm*100000.0);
			rigid1.mn.genVar[8] = (int16_t) (fabs(tm->aa - act1.thetaDes)*SCALE_FACTOR_10000);
			rigid1.mn.genVar[9] = (int16_t) (act1.tauDes);
		break;
		case GUI_MODE_ADAPTIVE_CONTROL: //7
			rigid1.mn.genVar[3] = (int16_t) (get_predictor()->k_pred);
			rigid1.mn.genVar[4] = (int16_t) (0);
			rigid1.mn.genVar[5] = (int16_t) (0);
			rigid1.mn.genVar[6] = (int16_t) (0);
			rigid1.mn.genVar[7] = (int16_t) (0);
			rigid1.mn.genVar[8] = (int16_t) (0);
			rigid1.mn.genVar[9] = (int16_t) (0);

		break;
		case GUI_MODE_GAIT_EVENTS: //8
			rigid1.mn.genVar[3] = (int16_t) (tm->elapsed_samples);
			rigid1.mn.genVar[4] = (int16_t) (tm->tq_dot*SCALE_FACTOR_10000);
			rigid1.mn.genVar[5] = (int16_t) (tm->aa_dot*SCALE_FACTOR_100);
			rigid1.mn.genVar[6] = (int16_t) (tm->mean_swing_tq_nm*SCALE_FACTOR_100);
			rigid1.mn.genVar[7] = (int16_t) (tm->in_swing);
			rigid1.mn.genVar[8] = (int16_t) (0);
			rigid1.mn.genVar[9] = (int16_t) (0);
			break;
	    case GUI_MODE_LEARNING: //9
			rigid1.mn.genVar[3] = (int16_t) (get_statistics()->k_est);
			rigid1.mn.genVar[4] = (int16_t) (get_predictor()->A[0]*get_predictor()->A[73]*get_predictor()->A[146]*get_predictor()->A[219]*get_predictor()->A[292]*get_predictor()->B[0]);
			rigid1.mn.genVar[5] = (int16_t) (get_statistics()->pop_k[0]);
			rigid1.mn.genVar[6] = (int16_t) (get_statistics()->pop_k[1]);
			rigid1.mn.genVar[7] = (int16_t) (get_statistics()->pop_k[2]);
			rigid1.mn.genVar[8] = (int16_t) (get_statistics()->pop_k[3]);
			rigid1.mn.genVar[9] = (int16_t) (get_statistics()->pop_k[4]);
				break;
	    case GUI_MODE_DYNAMICS: //10
			rigid1.mn.genVar[3] = (int16_t) (tm->tq_dot*SCALE_FACTOR_100);
			rigid1.mn.genVar[4] = (int16_t) (tm->aa_dot*SCALE_FACTOR_100);
			rigid1.mn.genVar[5] = (int16_t) (tm->aa_dot_15hz_filt*SCALE_FACTOR_100);
			rigid1.mn.genVar[6] = (int16_t) (tm->peak_power_w);
			rigid1.mn.genVar[7] = (int16_t) (tm->net_work_j*SCALE_FACTOR_100);
			rigid1.mn.genVar[8] = (int16_t) (tm->power_w);
			rigid1.mn.genVar[9] = (int16_t) (0);
		break;
	    case GUI_MODE_BACK_ESTIMATION: //11
			rigid1.mn.genVar[3] = (int16_t) (get_kinematics()->curr_ground_slope_est*SCALE_FACTOR_10000);
			rigid1.mn.genVar[4] = (int16_t) (get_back_estimator()->curr_stride_paz_thresh_pass_samples);
			rigid1.mn.genVar[5] = (int16_t) (get_back_estimator()->curr_stride_paz_thresh_status);
			rigid1.mn.genVar[6] = (int16_t) (get_back_estimator()->prev_torque_range*SCALE_FACTOR_100);
			rigid1.mn.genVar[7] = (int16_t) (get_kinematics()->aAccZ*SCALE_FACTOR_100);
			rigid1.mn.genVar[8] = (int16_t) (get_kinematics()->pAz*SCALE_FACTOR_10000);
			rigid1.mn.genVar[9] = (int16_t) (get_statistics()->k_est);
			break;
	    case GUI_MODE_KINEMATICS1: //12
			rigid1.mn.genVar[3] = (int16_t) (get_kinematics()->aAccY*SCALE_FACTOR_100);
			rigid1.mn.genVar[4] = (int16_t) (get_kinematics()->aAccZ*SCALE_FACTOR_100);
			rigid1.mn.genVar[5] = (int16_t) (get_kinematics()->aOmegaX*SCALE_FACTOR_100);
			rigid1.mn.genVar[6] = (int16_t) (get_kinematics()->foot_flat*SCALE_FACTOR_100);
			rigid1.mn.genVar[7] = (int16_t) (get_kinematics()->rot3*SCALE_FACTOR_10000);
			rigid1.mn.genVar[8] = (int16_t) (get_kinematics()->pAy*SCALE_FACTOR_10000);
			rigid1.mn.genVar[9] = (int16_t) (get_kinematics()->pAz*SCALE_FACTOR_10000);
			break;
	    case GUI_MODE_KINEMATICS2: //13
			rigid1.mn.genVar[3] = (int16_t) (get_kinematics()->rot3*SCALE_FACTOR_10000);
			rigid1.mn.genVar[4] = (int16_t) (get_task_machine()->aa_dot*SCALE_FACTOR_100);
			rigid1.mn.genVar[5] = (int16_t) (get_kinematics()->joint_vel_seg_vel_diff_sq*SCALE_FACTOR_100);
			rigid1.mn.genVar[6] = (int16_t) (get_kinematics()->accNormSq*SCALE_FACTOR_100);
			rigid1.mn.genVar[7] = (int16_t) (get_kinematics()->foot_flat);
			rigid1.mn.genVar[8] = (int16_t) (get_kinematics()->pAz*SCALE_FACTOR_10000);
			rigid1.mn.genVar[9] = (int16_t) (get_statistics()->k_est);
			break;
	    case GUI_MODE_FEATURES: //14
			rigid1.mn.genVar[3] = (int16_t) (get_kinematics()->latest_foot_static_samples);
			rigid1.mn.genVar[4] = (int16_t) (tm->latest_foot_off_samples);
			rigid1.mn.genVar[5] = (int16_t) (tm->do_learning_for_curr_stride);
			rigid1.mn.genVar[6] = (int16_t) (get_curr_features()->fin[0]);
			rigid1.mn.genVar[7] = (int16_t) (get_curr_features()->fin[1]);
			rigid1.mn.genVar[8] = (int16_t) (get_curr_features()->rng[0]);
			rigid1.mn.genVar[9] = (int16_t) (get_curr_features()->rng[1]);
			break;
	    case GUI_MODE_SAFETY: //15
	    {
	    	int8_t* safetyConditions = getSafetyConditions();
	    	int16_t safety0to2 = safetyConditions[0]*100 + safetyConditions[1]*10+safetyConditions[2];
	    	int16_t safety3to5 = safetyConditions[3]*100 + safetyConditions[4]*10+safetyConditions[5];
	    	int16_t safety6to8 = safetyConditions[6]*100 + safetyConditions[7]*10+safetyConditions[8];
	    	int16_t safety9to11 = safetyConditions[9]*100 + safetyConditions[10]*10+safetyConditions[11];
	    	int16_t safety12to14 = safetyConditions[12]*100 + safetyConditions[13]*10+safetyConditions[14];
	    	rigid1.mn.genVar[3] = (int16_t) (getMotorMode());
			rigid1.mn.genVar[4] = safety0to2;
			rigid1.mn.genVar[5] = safety3to5;
			rigid1.mn.genVar[6] = safety6to8;
			rigid1.mn.genVar[7] = safety9to11;
			rigid1.mn.genVar[8] = safety12to14;
			rigid1.mn.genVar[9] = (int16_t) (actuatorIsCorrect());
	    }
	    	break;
	    case GUI_MODE_PREDICTION_ACCURACY: //16
//	    	rigid1.mn.genVar[3] = (int16_t) (tm->);
//			rigid1.mn.genVar[4] = (int16_t) (tm->latest_foot_off_samples);
//			rigid1.mn.genVar[5] = (int16_t) (tm->do_learning_for_curr_stride);
//			rigid1.mn.genVar[6] = (int16_t) (get_curr_features()->max[0]);
//			rigid1.mn.genVar[7] = (int16_t) (get_curr_features()->min[1]);
//			rigid1.mn.genVar[8] = (int16_t) (get_curr_features()->rng[2]);
//			rigid1.mn.genVar[9] = (int16_t) (get_curr_features()->fin[3]);
	    	break;
	    case GUI_MODE_STATISTICS: //18
	    	rigid1.mn.genVar[3] = (int16_t) (get_predictor()->A[0]*10000.0);
			rigid1.mn.genVar[4] = (int16_t) (get_predictor()->A[2]*10000.0);
			rigid1.mn.genVar[5] = (int16_t) (get_predictor()->A[4]*10000.0);
//			rigid1.mn.genVar[6] = (int16_t) (get_statistics()->mu_k[33]*SCALE_FACTOR_10000);
			rigid1.mn.genVar[6] = (int16_t) (get_statistics()->sum_sigma[67]*SCALE_FACTOR_100);
			rigid1.mn.genVar[7] = (int16_t) (get_statistics()->k_est);
			rigid1.mn.genVar[8] = (int16_t) (get_predictor()->A[6]*10000.0);
			rigid1.mn.genVar[9] = (int16_t) (get_predictor()->k_pred);
//			rigid1.mn.genVar[8] = (int16_t) (get_statistics()->sum_sigma[9]*SCALE_FACTOR_100);
//			rigid1.mn.genVar[9] = (int16_t) (get_statistics()->sum_sigma[98]*SCALE_FACTOR_100);
	    	    	break;
	    case GUI_MODE_PREDICTOR: //18
	    	break;
	}
}



#define FINDPOLES_DONE (calibrationFlags == 0) && (calibrationNew == 0)
//****************************************************************************
// Public Function(s)
//****************************************************************************

//MIT DLeg Finite State Machine.

/*
 *  Initialize the Finite State Machine
 */
void initMITDLeg(void) {


}


/*
 *  Finite State Machine with multiple states.
 *  Call this function in one of the main while fsmTime slots.
 *  The possible states are as follows:
 *  	STATE_POWER_ON
 *  	STATE_INITIALIZE_SENSORS
 *  	STATE_FIND_POLES
 *  	STATE_INIT_USER_WRITES
 *  	STATE_MAIN
 *  	STATE_DEBUG
 */
void MITDLegFsm1(void)
{
	#if(ACTIVE_PROJECT == PROJECT_MIT_DLEG)

    static uint32_t fsmTime = 0;
    static uint32_t controlTime = 0;
    static float tor = 0.0;

    //Increment fsm_time (1 tick = 1ms nominally)
    fsmTime++;

    // Send genVars values to the GUI
    updateGenVars(get_task_machine());


    //begin main FSM
	switch(fsm1State)
	{
		//this state is always reached
		case STATE_POWER_ON:
			//Same power-on delay as FSM2:
			if(fsmTime >= AP_FSM2_POWER_ON_DELAY) {
				//sensor update happens in mainFSM2(void) in main_fsm.c
				isEnabledUpdateSensors = 0;
				onEntry = 1;
				fsmTime = 0;
				fsm1State = STATE_INITIALIZE_SETTINGS;
			}

			break;

		case STATE_INITIALIZE_SETTINGS:

			if(fsmTime >= AP_FSM2_POWER_ON_DELAY) {

				#if defined(NO_DEVICE)
					fsm1State = STATE_DEBUG;				// Skip over All states and sit in debug mode
				#elif defined(NO_ACTUATOR) || defined(NO_POWER)
					fsm1State = STATE_INITIALIZE_SENSORS;		// Still run controls, but skip Find Poles
					calibrationFlags = 0, calibrationNew = 0;
					isEnabledUpdateSensors = 1;
					onEntry = 1;
				#else
					fsm1State = STATE_FIND_POLES;
				#endif
				act1.currentOpLimit = CURRENT_LIMIT_INIT;
				onEntry = 1;

				// If Master, enable Comm's, if Slave Do not need to turn it on.
				#if (ACTIVE_SUBPROJECT == SUBPROJECT_A)
					enableMITfsm2 = 1;	// Turn on communication to to SLAVE
				#else
					enableMITfsm2 = 0;	// If it's slave, then don't bother turning on comms?
				#endif
			}

			break;

		case STATE_FIND_POLES:

			if (!actuatorIsCorrect(&act1)){
				setLEDStatus(0,0,1); //flash red; needs reset
			} else{
				if (onEntry) {
					// USE these to to TURN OFF FIND POLES set these = 0 for OFF, or =1 for ON
					calibrationFlags = 1, calibrationNew = 1;

					isEnabledUpdateSensors = 0;
					onEntry = 0;
				}

				// Check if FindPoles has completed, if so then go ahead. This is done in calibration_tools.c
				if (FINDPOLES_DONE){
					fsm1State = STATE_INITIALIZE_SENSORS;
					fsmTime = 0;
					isEnabledUpdateSensors = 1;
					onEntry = 1;
				}
			}

			break;

		case STATE_INITIALIZE_SENSORS:

//			setMotorNeutralPosition(&act1);	// initialize to define motor initial position
			//todo check this is okay
			zeroLoadCell = 1;	// forces getAxialForce() to zero the load cell again. this is kinda sketchy using a global variable.
			isEnabledUpdateSensors = 1;

//			if (fsmTime > AP_FSM2_POWER_ON_DELAY)
			if (fsmTime > 5000)
			{
				fsm1State = STATE_INIT_USER_WRITES;
				fsmTime = 0;
				onEntry = 1;
				zeroLoadCell = 0;
			}
			break;

		case STATE_INIT_USER_WRITES:

			/*reserve for additional initialization*/

			//Initialize Filters for Torque Sensing
			initSoftFIRFilt();	// Initialize software filter
//			initSGFilt();		// Savitsky-Golay
//			initSGVelFilt();

			mitInitCurrentController();		//initialize Current Controller with gains

			//Set userwrites to initial values
			ankleWalkParams.initializedStateMachineVariables = 0;
			if (!ankleWalkParams.initializedStateMachineVariables){
				initializeUserWrites(get_task_machine());
				ankleWalkParams.initializedStateMachineVariables = 1;
				kneeAnkleStateMachine.currentState = STATE_INIT;	//Establish walking state machine initialization state
			}

			fsm1State = STATE_MAIN;
			fsmTime = 0;
			onEntry = 1;

			break;

		case STATE_MAIN:
			{
				updateUserWrites(get_task_machine());


				//DEBUG removed this because joint encoder can't update in locked state.
//				if (getMotorMode() == MODE_ENABLED || getMotorMode() == MODE_OVERTEMP ){

					/****************************************
					 *  Below here is where user code goes
					 ****************************************/
				#if defined(IS_ACTUATOR_TESTING)

//					tor = getImpedanceTorque(&act1, inputK, inputB, inputTheta);
					// Try running impedance update slower than torque controller.
//					if ( fmod(controlTime,5) == 0 )
//					{
						if (fabs(inputTorq) > 0)
						{
							tor = inputTorq;
							act1.tauDes = tor;
						}
						else
						{
							tor = getImpedanceTorque(&act1, inputK, inputB, inputTheta);
							act1.tauDes = tor;
						}
//					}


					setMotorTorque( &act1, act1.tauDes);

				#elif defined(IS_SWEEP_TEST)
					float omega = 2*freq * 2 * M_PI;
					float thetaLocal = torqueSystemIDFrequencySweep( omega, fsmTime, amplitude, dcBias, noiseAmp, begin);
//					setMotorTorqueOpenLoop( &act1, act1.tauDes, 0);
					act1.tauDes = getImpedanceTorque(&act1, 2.0, 0.005, thetaLocal);
					setMotorTorque( &act1, act1.tauDes);
				#elif defined(IS_SWEEP_CHIRP_TEST)

					act1.tauDes = torqueSystemIDFrequencySweepChirp(freq, freqFinal, freqSweepTime, amplitude, dcBias, noiseAmp, chirpType, begin);

//					setMotorTorqueOpenLoop( &act1, act1.tauDes, 0);
					setMotorTorque( &act1, act1.tauDes);

				#else
					runMainUserApplication(&rigid1, &act1);

				#endif


				controlTime++;

				break;
			}
		case STATE_DEBUG:

			break;

        	default:
			//Handle exceptions here
			break;
	}

	#endif	//ACTIVE_PROJECT == PROJECT_ANKLE_2DOF

}


/*
 *  Second state machine for the DLeg project
 *  Currently seems unused
 */
void MITDLegFsm2(void)
{
	//Verify we are Master and communicating to Slave and defined as Knee
	#if (ACTIVE_PROJECT == PROJECT_MIT_DLEG) && (ACTIVE_SUBPROJECT == SUBPROJECT_A) && defined(IS_KNEE)

	//Modified version of ActPack
	static uint32_t Fsm2Timer = 0;

	//Wait X seconds before communicating
	if(Fsm2Timer < AP_FSM2_POWER_ON_DELAY)
	{
		mitFSM2ready = 0;
		Fsm2Timer++;
		return;
	}


	//External controller can fully disable the comm:
	//if(ActPackSys == SYS_NORMAL && ActPackCoFSM == APC_FSM2_ENABLED){enableAPfsm2 = 1;}
	//else {enableAPfsm2 = 0;}

	//FSM1 can disable this one:
	if(enableMITfsm2)
	{
			writeEx[1].offset = 7;	// grab only this offset.
			tx_cmd_actpack_rw(TX_N_DEFAULT, writeEx[1].offset, writeEx[1].ctrl, writeEx[1].setpoint, \
											writeEx[1].setGains, writeEx[1].g[0], writeEx[1].g[1], \
											writeEx[1].g[2], writeEx[1].g[3], 0);	// todo: try this offset counter thing

			packAndSend(P_AND_S_DEFAULT, FLEXSEA_MANAGE_2, mitDlegInfo, SEND_TO_SLAVE);

			//Reset KEEP/CHANGE once set:
			//if(writeEx[0].setGains == CHANGE){writeEx[0].setGains = KEEP;}
			if(writeEx[1].setGains == CHANGE){writeEx[1].setGains = KEEP;}

	}

	#endif	//ACTIVE_PROJECT == PROJECT_MIT_DLEG
}


#endif 	//BOARD_TYPE_FLEXSEA_MANAGE || defined BOARD_TYPE_FLEXSEA_PLAN
#endif //INCLUDE_UPROJ_MIT_DLEG || defined BOARD_TYPE_FLEXSEA_PLAN
