
#ifndef __MACHINELEARNINGMETHODS_H__
#define __MACHINELEARNINGMETHODS_H__


#include "flexsea_user_structs.h"
#include "linear_algebra_methods.h"
#include "user-mn-MIT-DLeg.h"
#include <stdio.h>

extern void reset_learning_demux();
extern int learning_demux(struct learner_s* lrn, struct classifier_s* lda, float* feats, int k_est);
void update_class_mean(struct learner_s* lrn, float* feats, int k_est);
void update_overall_mean(struct learner_s* lrn, float* feats);
extern void classify(struct classifier_s* lda, float* feats);
extern void init_learner(struct learner_s* lrn);
extern void init_classifier(struct classifier_s* lda);

// enum Learning_States {
// 	UPDATE_CLASS_MEAN	= 0,
// 	UPDATE_OVERALL_MEAN	= 1,
// 	UPDATE_COVARIANCE_1	= 2,
// 	UPDATE_COVARIANCE_2	= 3, 
// 	UPDATE_COVARIANCE_3 = 4,
// 	UPDATE_COVARIANCE_4 = 5,
// 	DO_CHOLESKY = 6,
// 	GET_TRANSPOSE = 7,
// 	GET_LDA_PARAMS = 8,
// };
enum Learning_States {
	UPDATE_CLASS_MEAN	= 0,
	UPDATE_OVERALL_MEAN	= 1,
	GET_DEVIATIONS_FROM_MEAN = 2,
	UPDATE_COVARIANCE = 3,
	DO_CHOLESKY = 4,
	UPDATE_TRANSPOSE = 5,
	UPDATE_LDA_PARAMS = 6,
};

#endif
