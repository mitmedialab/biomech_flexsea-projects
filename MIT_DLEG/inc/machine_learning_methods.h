
#ifndef __MACHINELEARNINGMETHODS_H__
#define __MACHINELEARNINGMETHODS_H__


// #include "flexsea_user_structs.h"
#include "user-mn-MIT-DLeg.h"
#include "linear_algebra_methods.h"
#include "kinematics_methods.h"
#include "task_machine.h"
#include "flexsea.h"
#include <stdio.h>
#include <float.h>

void update_learner_demux(struct back_estimator_s* be, int latest_foot_off_samples, int learner_update_reset_trigger);
void update_classifier_demux();
void update_features(struct kinematics_s* kin);
void classify();
void init_learning_structs();
struct learner_s* get_learner();
struct classifier_s* get_classifier();
float* get_prev_features();
float* get_curr_features();

enum Prediction_Signals {
	ROT3 = 1,
	AOMEGAX = 2,
	AOMEGAY = 3,
	AOMEGAZ = 4,
	AACCX = 5,
	AACCY = 6,
	AACCZ = 7,
	AAY = 8,
	VAY = 9,
	PAY = 10,
	AAZ = 11,
	VAZ = 12,
	PAZ = 13,
	SINSQATTACK = 14,
	AA = 15,
	TQ = 16,
	AADOT = 17,
	TQDOT = 18,
	IAOMEGAX = 19,
	IAACCY = 20,
	IAACCZ = 21,
	DAOMEGAX = 22,
	DAACCY = 23,
	DAACCZ = 24,
};

enum Update_Learner_States {
	LRN_BACK_ESTIMATE = 0,
	LRN_UPDATE_CLASS_MEAN = 1,
	LRN_UPDATE_OVERALL_MEAN = 2,
	LRN_GET_DEVIATION_FROM_CURR_MEAN = 3,
	LRN_GET_DEVIATION_FROM_PREV_MEAN = 4,
	LRN_UPDATE_COVARIANCE = 5,
	LRN_READY_TO_UPDATE_LEARNER = 6,
};

enum Update_Classifier_States {
	LDA_COPY_MU_K = 0,
	LDA_COPY_SUM_SIGMA = 1,
	LDA_DO_CHOLESKY = 2,
	LDA_UPDATE_TRANSPOSE = 3,
	LDA_CALC_A_PARAMS = 4,
	LDA_CALC_B_PARAMS = 5,
	LDA_UPDATE_PARAMS = 6,
};


//Assumes uniform priors at the moment
struct classifier_s
{
	float* A;
	float* B;
	float* score_k;
	int k_pred;
	float* latest_sum_sigma;

	//Intermediary matrix holders
	float* UT;
    float* LT;
    
    float* x;
    float* y;

    //Segmentation flags
    int segment;
    int subsegment;
    int doing_forward_substitution;
    int current_updating_class;
};

//Assumes uniform priors at the moment
struct learner_s
{
	float* mu_k;
	float* sum_k;
	float* mu_prev;
	float* mu;
	float* sum;
	float* sigma;
	float* sum_sigma;
	
	float* pop_k;
	float pop;

	int k_est;
	
	int n_updates;
	//Intermediate matrix holders
    float* A;
    float* x;
    float* y;

    //Segmentation flag
    int segment;

};



#endif
