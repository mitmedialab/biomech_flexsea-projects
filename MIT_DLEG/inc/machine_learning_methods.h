
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

void learning_demux(struct back_estimator_s* be, int latest_foot_off_samples, int learning_reset_trigger);
void update_features(struct kinematics_s* kin);
void classify();
void init_learning_structs();
struct learner_s* get_learner();
struct classifier_s* get_classifier();
float* get_prev_features();
float* get_curr_features();

enum Learning_States {
	BACK_ESTIMATE = 0,
	UPDATE_CLASS_MEAN	= 1,
	UPDATE_OVERALL_MEAN	= 2,
	GET_DEVIATIONS_FROM_MEAN = 3,
	UPDATE_COVARIANCE = 4,
	DO_CHOLESKY = 5,
	UPDATE_TRANSPOSE = 6,
	UPDATE_LDA_A_PARAMS = 7,
	UPDATE_LDA_B_PARAMS = 8,
	READY_TO_LEARN = 9,
};

enum Features {
	PAZ_MAX,
	PAZ_MEAN,
	VAZ_MIN,
	PITCH_RANGE,
	OMX_MAX,
	ACCY_MEAN,
};

//Assumes uniform priors at the moment
struct classifier_s
{
	float* A;
	float* B;
	float* score_k;
	int k_pred;
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
	
	//Intermediate matrix holders
    float* UT;
    float* LT;
    float* A;
    float* x;
    float* y;
};



#endif
