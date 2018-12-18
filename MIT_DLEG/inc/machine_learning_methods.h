
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

enum Update_Learner_States {
	BACK_ESTIMATE = 0,
	UPDATE_CLASS_MEAN	= 1,
	UPDATE_OVERALL_MEAN	= 2,
	GET_DEVIATION_FROM_CURR_MEAN = 3,
	GET_DEVIATION_FROM_PREV_MEAN
	UPDATE_COVARIANCE = 4,
	READY_TO_UPDATE_LEARNER = 5,
};

enum Update_Classifier_States {
	COPY_SUM_SIGMA = 0,
	DO_CHOLESKY = 1,
	UPDATE_TRANSPOSE = 2,
	UPDATE_LDA_A_PARAMS = 3,
	UPDATE_LDA_B_PARAMS = 4,
	READY_TO_UPDATE_CLASSIFIER = 5,
};

 enum Features {
	PAZ_MAX,
	PAZ_SUM,
	VAZ_MIN,
	PITCH_RANGE,
	OMX_MAX,
	ACCY_SUM,
	PITCH_MAX, //this is where intermediate/assistive features begin
	PITCH_MIN,
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

};



#endif
