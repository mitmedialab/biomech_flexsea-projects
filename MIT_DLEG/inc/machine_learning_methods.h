
#ifndef __MACHINELEARNINGMETHODS_H__
#define __MACHINELEARNINGMETHODS_H__


// #include "flexsea_user_structs.h"
#include "user-mn-MIT-DLeg.h"
#include "linear_algebra_methods.h"
#include "kinematics_calculations.h"
// #include "task_machine.h"
#include <stdio.h>

extern void reset_learning_demux();
extern int learning_demux();
void update_class_mean();
void update_overall_mean();
extern void update_features(struct kinematics_s* kin);
extern void classify();
extern void init_features();
extern void init_learner();
extern void init_classifier();
extern void get_learner();
extern void get_classifier();

enum Learning_States {
	UPDATE_CLASS_MEAN	= 0,
	UPDATE_OVERALL_MEAN	= 1,
	GET_DEVIATIONS_FROM_MEAN = 2,
	UPDATE_COVARIANCE = 3,
	DO_CHOLESKY = 4,
	UPDATE_TRANSPOSE = 5,
	UPDATE_LDA_A_PARAMS = 6,
	UPDATE_LDA_B_PARAMS = 7,
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
	
	//Intermediate matrix holders
    float* UT;
    float* LT;
    float* A;
    float* x;
    float* y;
};

extern struct classifier_s lda;
extern struct learner_s lrn;
extern float* feats;
extern int k_est;

#endif
