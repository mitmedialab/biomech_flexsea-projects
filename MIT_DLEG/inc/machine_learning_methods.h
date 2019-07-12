
#ifndef __MACHINELEARNINGMETHODS_H__
#define __MACHINELEARNINGMETHODS_H__


// #include "flexsea_user_structs.h"
#include "user-mn-MIT-DLeg.h"
#include "linear_algebra_methods.h"
#include "kinematics_methods.h"
#include "task_machine.h"
#include "back_estimation_methods.h"
#include "flexsea.h"
#include <stdio.h>
#include <float.h>

void predict_task_demux(struct taskmachine_s* tm, struct kinematics_s* kin);
void update_statistics_demux(struct taskmachine_s* tm, struct kinematics_s* kin);
void update_learner_demux();
void update_prediction_features(struct taskmachine_s* tm, struct kinematics_s* kin);
void reset_learning_structs();
void init_learning_structs();

//Getters
//int* get_confusion_matrix_row_for_class(int k_est);
struct statistics_s* get_statistics();
struct learner_s* get_learner();
struct predictor_s* get_predictor();
struct features_s* get_prev_features();
struct features_s* get_curr_features();

enum Prediction_Signals {
	ROT3 = 0,
	AOMEGAX = 1,
	AACCY = 2,
	AACCZ = 3,
	PAZ =4,
};

enum Update_Statistics_States {
	STATS_BACK_ESTIMATE = 0,
	STATS_UPDATE_CLASS_SUM = 1,
	STATS_UPDATE_CLASS_MEAN = 2,
	STATS_UPDATE_OVERALL_SUM = 3,
	STATS_UPDATE_OVERALL_MEAN = 4,
	STATS_GET_DEVIATION_FROM_PREV_MEAN = 5,
	STATS_GET_DEVIATION_FROM_CURR_MEAN = 6,
	STATS_UPDATE_MEAN_DEVIATION_OUTER_PRODUCT = 7,
	STATS_UPDATE_COVARIANCE = 8,
	STATS_READY_TO_UPDATE_STATISTICS = 9,
};

enum Update_Learner_States {
	LRN_COPY_MU_K = 0,
	LRN_COPY_SUM_SIGMA = 1,
	LRN_DO_CHOLESKY = 2,
	LRN_UPDATE_TRANSPOSE = 3,
	LRN_CALC_A_PARAMS = 4,
	LRN_CALC_B_PARAMS = 5,
	LRN_UPDATE_PARAMS = 6,
};

enum Predict_Task_States {
	PRED_UPDATE_RNG = 0,
	PRED_UPDATE_FIN = 1,
	PRED_PREDICT = 2,
	PRED_UPDATE_PREV_FEATS = 3,
	PRED_READY_TO_PREDICT = 4,
};

struct features_s
{
	float* max;
	float* min;
	float* rng;
	float* fin;
};


struct statistics_s
{
	//Ground truth variables
	int k_true;
	float* pop_k_true;
	float pop_true;



	//Statistical variables
	float* mu_k;
	float* sum_k;
	float* mu_prev;
	float* mu;
	float* sum;
	float* sum_sigma;
	float* pop_k;
	float pop;

	//init intermediary matrices
	float* temp;
	float* x;
	float* y;

	int k_est;
	int k_est_prev;

	//prediction performance metrics
	int* confusion_matrix;
	float* estimation_accuracies;
	float composite_estimation_accuracy;
	float* prediction_accuracies;
	float running_prediction_accuracy;

	//Segmentation, state,and mutex variables
	uint8_t updating_statistics_matrices;
	int demux_state;		
	int segment;


};

struct learner_s
{
	//All variables required for transforming statistical observations into a new classifier/predictor
	float* UT;
	float* LT;
	float* latest_sum_sigma;
	float* latest_mu_k;
	float* Atemp;
	float* Btemp;
	float* y;

	//Segmentation, state,and mutex variables
	uint8_t copying_statistics_matrices;
	int demux_state;
	int segment;
	int subsegment;
	uint8_t doing_forward_substitution;

};

struct predictor_s
{
	//Predictor parameters
	float* A; 
	float* B; 

	//Discriminant scores and prediction
	float* score_k; 
	int k_pred; 
	float max_score;

	//Segmentation, state,and mutex variables
	uint8_t predicting_task;
	int demux_state;
	int segment;
	int subsegment;

};



#endif
