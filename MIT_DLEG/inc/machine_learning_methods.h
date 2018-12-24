
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

void predict_task(struct taskmachine_s* tm, struct kinematics_s* kin);
void update_learner_demux(struct taskmachine_s* tm);
void update_classifier_demux();
void update_prediction_features(struct taskmachine_s* tm, struct kinematics_s* kin);
void init_learning_structs();

//Getters
struct learner_s* get_learner();
struct classifier_s* get_classifier();
struct features_s* get_prev_features();
struct features_s* get_curr_features();

//copied from matlab pil
enum Prediction_Signals {
	ROT3 = 0,
	AOMEGAX = 1,
	AOMEGAY = 2,
	AOMEGAZ = 3,
	AACCX = 4,
	AACCY = 5,
	AACCZ = 6,
	AAY = 7,
	VAY = 8,
	PAY = 9,
	AAZ = 10,
	VAZ = 11,
	PAZ = 12,
	SINSQATTACK = 13,
	AA = 14,
	TQ = 15,
	AADOT = 16,
	TQDOT = 17,
	IAOMEGAX = 18,
	IAACCY = 19,
	IAACCZ = 20,
	DAOMEGAX = 21,
	DAACCY = 22,
	DAACCZ = 23,
};

//copied from matlab pil
enum Update_Learner_States {
	LRN_BACK_ESTIMATE = 0,
	LRN_UPDATE_CLASS_SUM = 1,
	LRN_UPDATE_CLASS_MEAN = 2,
	LRN_UPDATE_OVERALL_SUM = 3,
	LRN_UPDATE_OVERALL_MEAN = 4,
	LRN_GET_DEVIATION_FROM_PREV_MEAN = 5,
	LRN_GET_DEVIATION_FROM_CURR_MEAN = 6,
	LRN_UPDATE_MEAN_DEVIATION_OUTER_PRODUCT = 7,
	LRN_UPDATE_COVARIANCE = 8,
	LRN_READY_TO_UPDATE_LEARNER = 9,
};

//copied from matlab pil
enum Update_Classifier_States {
	LDA_COPY_MU_K = 0,
	LDA_COPY_SUM_SIGMA = 1,
	LDA_DO_CHOLESKY = 2,
	LDA_UPDATE_TRANSPOSE = 3,
	LDA_CALC_A_PARAMS = 4,
	LDA_CALC_B_PARAMS = 5,
	LDA_UPDATE_PARAMS = 6,
};

//copied from matlab pil
struct features_s
{
	float* max;
	float* min;
	float* rng;
	float* fin;
};


//copied from matlab pil
struct learner_s
{
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

	uint8_t k_est;


	//Segmentation, state,and mutex variables
	uint8_t updating_learner_matrices;
	int demux_state;		
	int segment;
	int subsegment;


};

//copied from matlab pil
struct classifier_s
{

	float* A;
	float* B;
	float* score_k;
	uint8_t k_pred;

	float* UT;
	float* LT;

	float* latest_sum_sigma;
	float* latest_mu_k;

	//init intermediary matrices
	float* Atemp;
	float* Btemp;
	float* y;

	//Segmentation, state,and mutex variables
	uint8_t copying_learner_matrices;
	int demux_state;
	int segment;
	int subsegment;
	uint8_t doing_forward_substitution;
};



#endif
