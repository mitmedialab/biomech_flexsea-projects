
#ifndef __MACHINELEARNINGMETHODS_H__
#define __MACHINELEARNINGMETHODS_H__


#include "flexsea_user_structs.h"
#include "linear_algebra_methods.h"
#include "user-mn-MIT-DLeg.h"
#include <stdio.h>

extern void update_classifier(struct learner_s* lrn, struct classifier_s* lda);
extern void classify(struct classifier_s* lda, float* feats);
extern void update_learner(struct learner_s* lrn);
extern void init_learner(struct learner_s* lrn);
extern void init_classifier(struct classifier_s* lda);

#endif
