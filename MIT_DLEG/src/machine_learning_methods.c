


#include "machine_learning_methods.h"

 //Machine learning (copied from matlab pil)
#define N_CLASSES 5
#define N_PREDICTION_SIGNALS 18
#define N_PREDICTION_FEATURES 4
#define N_FEATURES N_PREDICTION_SIGNALS * N_PREDICTION_FEATURES
#define N_FEATURES_SQ N_FEATURES * N_FEATURES

#define MAX_FEATURES_START_IND 0
#define MIN_FEATURES_START_IND N_PREDICTION_SIGNALS*1
#define RNG_FEATURES_START_IND N_PREDICTION_SIGNALS*2
#define FIN_FEATURES_START_IND N_PREDICTION_SIGNALS*3




static struct classifier_s lda;
static struct learner_s lrn;
static struct features_s currfeats;
static struct features_s prevfeats;


static void update_class_sum(){
  int ind = lrn.k_est*N_FEATURES;
  int ind_max = ind+MAX_FEATURES_START_IND;
  int ind_min = ind+MIN_FEATURES_START_IND;
  int ind_rng = ind+RNG_FEATURES_START_IND;
  int ind_fin = ind+FIN_FEATURES_START_IND;
  lrn.pop_k[lrn.k_est] = lrn.pop_k[lrn.k_est] + 1.0; //1 flop
  sum(&lrn.sum_k[ind_max], prevfeats.max, &lrn.sum_k[ind_max], N_PREDICTION_SIGNALS); // f/4 flops
  sum(&lrn.sum_k[ind_min], prevfeats.min, &lrn.sum_k[ind_min], N_PREDICTION_SIGNALS); // f/4 flops
  sum(&lrn.sum_k[ind_rng], prevfeats.rng, &lrn.sum_k[ind_rng], N_PREDICTION_SIGNALS); // f/4 flops
  sum(&lrn.sum_k[ind_fin], prevfeats.fin, &lrn.sum_k[ind_fin], N_PREDICTION_SIGNALS); // f/4 flops

}

static void update_overall_sum(){
  assignment(lrn.mu, lrn.mu_prev, N_FEATURES);// assignment
  lrn.pop = lrn.pop + 1.0;
  sum(&lrn.sum[MAX_FEATURES_START_IND], prevfeats.max, &lrn.sum[MAX_FEATURES_START_IND], N_PREDICTION_SIGNALS); // f/4 flops
  sum(&lrn.sum[MIN_FEATURES_START_IND], prevfeats.min, &lrn.sum[MIN_FEATURES_START_IND], N_PREDICTION_SIGNALS); // f/4 flops
  sum(&lrn.sum[RNG_FEATURES_START_IND], prevfeats.rng, &lrn.sum[RNG_FEATURES_START_IND], N_PREDICTION_SIGNALS); // f/4 flops
  sum(&lrn.sum[FIN_FEATURES_START_IND], prevfeats.fin, &lrn.sum[FIN_FEATURES_START_IND], N_PREDICTION_SIGNALS); // f/4 flops
  
}

//copied from matlab pil
static void reset_features(){
    assignment(currfeats.max, prevfeats.max, N_PREDICTION_SIGNALS);
    assignment(currfeats.min, prevfeats.min, N_PREDICTION_SIGNALS);
    assignment(currfeats.rng, prevfeats.rng, N_PREDICTION_SIGNALS);
    assignment(currfeats.fin, prevfeats.fin, N_PREDICTION_SIGNALS);
    for (int j=0; j < N_PREDICTION_SIGNALS; j++){
        currfeats.max[j] = -FLT_MAX;
        currfeats.min[j] = FLT_MAX;
    }

}

//copied from matlab pil
static void init_features(){
  currfeats.max = (float*)calloc(N_PREDICTION_SIGNALS, sizeof(float));
  currfeats.min = (float*)calloc(N_PREDICTION_SIGNALS, sizeof(float));
  currfeats.rng = (float*)calloc(N_PREDICTION_SIGNALS, sizeof(float));
  currfeats.fin = (float*)calloc(N_PREDICTION_SIGNALS, sizeof(float));
  prevfeats.max = (float*)calloc(N_PREDICTION_SIGNALS, sizeof(float));
  prevfeats.min = (float*)calloc(N_PREDICTION_SIGNALS, sizeof(float));
  prevfeats.rng = (float*)calloc(N_PREDICTION_SIGNALS, sizeof(float));
  prevfeats.fin = (float*)calloc(N_PREDICTION_SIGNALS, sizeof(float));
}

//copied from matlab pil
static void init_learner(){

  lrn.mu_k = (float*)calloc(N_CLASSES  * N_FEATURES, sizeof(float));
  lrn.sum_k = (float*)calloc(N_CLASSES   * N_FEATURES, sizeof(float));
  lrn.mu_prev  = (float*)calloc( N_FEATURES, sizeof(float));
  lrn.mu  = (float*)calloc( N_FEATURES, sizeof(float));
  lrn.sum  = (float*)calloc( N_FEATURES, sizeof(float));
  lrn.sum_sigma = (float*)calloc(N_FEATURES_SQ, sizeof(float));
  lrn.pop_k = (float*)calloc( N_CLASSES, sizeof(float));
  lrn.pop = 1.0;

  for (int i = 0; i < N_CLASSES; i++){
    lrn.pop_k[i] = 1.0;
  }

  int N_FEATURES_p_1 = N_FEATURES+1;
  for (int i = 0; i < N_FEATURES; i++){
    lrn.sum_sigma[i*N_FEATURES_p_1] = 1.0;
  }

  //init intermediary matrices
  lrn.temp = (float*)calloc(N_FEATURES_SQ, sizeof(float));
  lrn.x  = (float*)calloc( N_FEATURES, sizeof(float));
  lrn.y  = (float*)calloc( N_FEATURES, sizeof(float));
  lrn.k_est = 1;

  lrn.updating_learner_matrices = 1;
  lrn.demux_state = LRN_READY_TO_UPDATE_LEARNER;
  lrn.segment = 0;
  lrn.subsegment = 0;

}

//copied from matlab pil
static void init_classifier(){

  lda.A = (float*)calloc(N_CLASSES * N_FEATURES, sizeof(float));
  lda.B = (float*)calloc(N_CLASSES, sizeof(float));
  lda.score_k = (float*)calloc(N_CLASSES, sizeof(float));
  lda.k_pred = 0;

  lda.UT = (float*)calloc(N_FEATURES_SQ, sizeof(float));
  lda.LT = (float*)calloc(N_FEATURES_SQ, sizeof(float));

  lda.latest_sum_sigma = (float*)calloc(N_FEATURES_SQ, sizeof(float));
  lda.latest_mu_k = (float*)calloc(N_CLASSES   * N_FEATURES, sizeof(float));

  lda.Atemp = (float*)calloc(N_CLASSES * N_FEATURES, sizeof(float));
  lda.Btemp = (float*)calloc(N_CLASSES, sizeof(float));
  lda.y  = (float*)calloc( N_FEATURES, sizeof(float));

  lda.copying_learner_matrices = 0;
  lda.demux_state = LDA_COPY_MU_K;
  lda.segment = 0;
  lda.subsegment = 0;
  lda.doing_forward_substitution = 1;

}

//copied from matlab pil
void update_learner_demux(struct taskmachine_s* tm){
    switch (lrn.demux_state){
      case LRN_BACK_ESTIMATE: //constant flops
          lrn.k_est = TASK_FL;
          back_estimate(tm, &lrn);
          lrn.demux_state = LRN_UPDATE_CLASS_SUM;
      break;
      case LRN_UPDATE_CLASS_SUM: //f flops per cycle
          if (lda.copying_learner_matrices)
              return;
          lrn.updating_learner_matrices = 1;
          update_class_sum();
          lrn.demux_state = LRN_UPDATE_CLASS_MEAN;
      break;
      case LRN_UPDATE_CLASS_MEAN: // f flops per cycle
      {
    	int ind = lrn.k_est*N_FEATURES;
        scaling (&lrn.sum_k[ind], 1.0/lrn.pop_k[lrn.k_est], &lrn.mu_k[ind], N_FEATURES);
        lrn.demux_state = LRN_UPDATE_OVERALL_SUM;
      }
      break;
      case LRN_UPDATE_OVERALL_SUM: //f flops per cycle
          update_overall_sum();
          lrn.demux_state = LRN_UPDATE_OVERALL_MEAN;
      break;
      case LRN_UPDATE_OVERALL_MEAN: // f flops per cycle
        scaling(lrn.sum, 1.0/lrn.pop, lrn.mu, N_FEATURES);
        lrn.demux_state = LRN_GET_DEVIATION_FROM_PREV_MEAN;
      break;
      case LRN_GET_DEVIATION_FROM_PREV_MEAN: // f flops per cycle
        diff(prevfeats.max, &lrn.mu_prev[MAX_FEATURES_START_IND], &lrn.x[MAX_FEATURES_START_IND], N_PREDICTION_SIGNALS);
        diff(prevfeats.min, &lrn.mu_prev[MIN_FEATURES_START_IND], &lrn.x[MIN_FEATURES_START_IND], N_PREDICTION_SIGNALS);
        diff(prevfeats.rng, &lrn.mu_prev[RNG_FEATURES_START_IND], &lrn.x[RNG_FEATURES_START_IND], N_PREDICTION_SIGNALS);
        diff(prevfeats.fin, &lrn.mu_prev[FIN_FEATURES_START_IND], &lrn.x[FIN_FEATURES_START_IND], N_PREDICTION_SIGNALS);
        lrn.demux_state = LRN_GET_DEVIATION_FROM_CURR_MEAN;
      break;
      case LRN_GET_DEVIATION_FROM_CURR_MEAN: // f flops per cycle
        diff(prevfeats.max, &lrn.mu[MAX_FEATURES_START_IND], &lrn.y[MAX_FEATURES_START_IND], N_PREDICTION_SIGNALS);
        diff(prevfeats.min, &lrn.mu[MIN_FEATURES_START_IND], &lrn.y[MIN_FEATURES_START_IND], N_PREDICTION_SIGNALS);
        diff(prevfeats.rng, &lrn.mu[RNG_FEATURES_START_IND], &lrn.y[RNG_FEATURES_START_IND], N_PREDICTION_SIGNALS);
        diff(prevfeats.fin, &lrn.mu[FIN_FEATURES_START_IND], &lrn.y[FIN_FEATURES_START_IND], N_PREDICTION_SIGNALS);
        lrn.demux_state = LRN_UPDATE_MEAN_DEVIATION_OUTER_PRODUCT;
      break;
      case LRN_UPDATE_MEAN_DEVIATION_OUTER_PRODUCT: // f flops per cycle
      {
        int segment_section = lrn.segment*N_FEATURES;
        segmented_outer_product(lrn.x, lrn.y, lrn.temp, N_FEATURES, lrn.segment); //f flops
        lrn.segment++;
        if (lrn.segment == N_FEATURES){
          lrn.demux_state = LRN_UPDATE_COVARIANCE; 
          lrn.segment = 0;
        }
      }
      break;
      case LRN_UPDATE_COVARIANCE: // f flops per cycle
      {
        int segment_section = lrn.segment*N_FEATURES;
        sum(&lrn.sum_sigma[segment_section], &lrn.temp[segment_section], &lrn.sum_sigma[segment_section], N_FEATURES);//f flops
        lrn.segment++;
        if (lrn.segment == N_FEATURES){
          lrn.demux_state = LRN_READY_TO_UPDATE_LEARNER; 
          lrn.segment = 0;
          lrn.updating_learner_matrices = 0;
        }
      }
      break;
      case LRN_READY_TO_UPDATE_LEARNER: //constant flops
        if (tm->gait_event_trigger == GAIT_EVENT_FOOT_OFF){
               if (tm->do_learning_for_stride){
                   tm->do_learning_for_stride = 0;
                 lrn.demux_state = LRN_BACK_ESTIMATE;
               }
             }
      break;
    }
  }

//copied from matlab pil
void update_classifier_demux(){

      switch (lda.demux_state){
      case LDA_COPY_MU_K://f assignments per cycle, 5 cycles
        {
          if (lrn.updating_learner_matrices)
              return;

          int ind = lda.segment*N_FEATURES;
          assignment(&lrn.mu_k[ind], &lda.latest_mu_k[ind], N_FEATURES);
          lda.segment++;
          if (lda.segment == N_CLASSES){
              lda.demux_state = LDA_COPY_SUM_SIGMA;
              lda.segment = 0;
          }
        }
      break;

      case LDA_COPY_SUM_SIGMA: //f assignments per cycle, f cycles
      {
          int ind = lda.segment*N_FEATURES;
          assignment(&lrn.sum_sigma[ind], &lda.latest_sum_sigma[ind], N_FEATURES);
          lda.segment++;
          if (lda.segment == N_FEATURES){
            lda.copying_learner_matrices = 0;
            lda.demux_state = LDA_DO_CHOLESKY;
            lda.segment = 0;
          }
        }
      break;
      case LDA_DO_CHOLESKY: // max f flops per cycle, f(f+1)/2 cycles
        super_segmented_cholesky(lda.latest_sum_sigma, lda.LT, N_FEATURES, lda.segment, lda.subsegment);
        lda.subsegment++;
        if (lda.subsegment > lda.segment){
          lda.subsegment = 0;
          lda.segment++;
        }

        if (lda.segment == N_FEATURES){
          lda.demux_state = LDA_UPDATE_TRANSPOSE; 
          lda.segment = 0;
        }
      break;
      case LDA_UPDATE_TRANSPOSE: //f flops per cycle, f cycles
        segmented_lower_to_upper_transpose(lda.LT, lda.UT, N_FEATURES, lda.segment);
        lda.segment++;
        if (lda.segment == N_FEATURES){
          lda.demux_state = LDA_CALC_A_PARAMS;
          lda.segment = 0;
        }
      break;
      case LDA_CALC_A_PARAMS: //f flops per cycle max, 10*f cycles
      {
        int ind = lda.segment*N_FEATURES;
        if (lda.doing_forward_substitution){
          segmented_forward_substitution(lda.LT, &lda.latest_mu_k[ind], lda.y, N_FEATURES, lda.subsegment); // roughly 1/2 f^2 flops
          lda.subsegment++;
          if (lda.subsegment == N_FEATURES){
            lda.subsegment = N_FEATURES-1;
            lda.doing_forward_substitution = 0;
          }
        }
        else{
          segmented_backward_substitution(lda.UT, lda.y, &lda.Atemp[ind], N_FEATURES, lda.subsegment); // roughly 1/2 f^2 flops
          lda.subsegment--;
          if (lda.subsegment == -1){
            lda.subsegment = 0;
            lda.doing_forward_substitution = 1;
            lda.segment++;
          }
        }
        if (lda.segment == N_CLASSES){
          lda.demux_state = LDA_CALC_B_PARAMS; 
          lda.segment = 0;
        }  
      }
      break;
      case LDA_CALC_B_PARAMS: //2f flops, 5 cycles
      {
        int ind = lda.segment*N_FEATURES;
        lda.Btemp[lda.segment] = -0.5*inner_product(&lda.latest_mu_k[ind], &lda.A[ind], N_FEATURES);
        lda.segment++;
        if (lda.segment == N_CLASSES){
          lda.segment = 0;
          lda.demux_state = LDA_UPDATE_PARAMS;
        }  
      }    
      break;
      case LDA_UPDATE_PARAMS:
          for (int j = 0; j < N_CLASSES; j++){
            int ind = j*N_FEATURES;
            int ind_max = ind+MAX_FEATURES_START_IND;
            int ind_min = ind+MIN_FEATURES_START_IND;
            int ind_rng = ind+RNG_FEATURES_START_IND;
            int ind_fin = ind+FIN_FEATURES_START_IND;
            assignment(&lda.Atemp[ind_max], &lda.A[ind_max], N_PREDICTION_SIGNALS);
            assignment(&lda.Atemp[ind_min], &lda.A[ind_min], N_PREDICTION_SIGNALS);
            assignment(&lda.Atemp[ind_rng], &lda.A[ind_rng], N_PREDICTION_SIGNALS);
            assignment(&lda.Atemp[ind_fin], &lda.A[ind_fin], N_PREDICTION_SIGNALS);
          }
          assignment(lda.Btemp, lda.B, N_CLASSES);
          lda.demux_state = LDA_COPY_MU_K; 
          lda.copying_learner_matrices = 1;
      break;
  }
}

//copied from matlab pil
void update_prediction_features(struct taskmachine_s* tm, struct kinematics_s* kin){
  if (tm->stride_classified || tm->gait_event_trigger == GAIT_EVENT_WINDOW_CLOSE)
        return;

    currfeats.max[ROT3] = MAX(currfeats.max[ROT3], kin->rot3);
    currfeats.max[AOMEGAX] = MAX(currfeats.max[AOMEGAX], kin->aOmegaX);
    currfeats.max[AOMEGAY] = MAX(currfeats.max[AOMEGAY], kin->aOmegaY);
    currfeats.max[AOMEGAZ] = MAX(currfeats.max[AOMEGAZ], kin->aOmegaZ);
    currfeats.max[AACCX] = MAX(currfeats.max[AACCX], kin->aAccX);
    currfeats.max[AACCY] = MAX(currfeats.max[AACCY], kin->aAccY);
    currfeats.max[AACCZ] = MAX(currfeats.max[AACCZ], kin->aAccZ);
    currfeats.max[AAY] = MAX(currfeats.max[AAY], kin->aAy);
    currfeats.max[VAY] = MAX(currfeats.max[VAY], kin->vAy);
    currfeats.max[PAY] = MAX(currfeats.max[PAY], kin->pAy);
    currfeats.max[AAZ] = MAX(currfeats.max[AAZ], kin->aAz);
    currfeats.max[VAZ] = MAX(currfeats.max[VAZ], kin->vAz);
    currfeats.max[PAZ] = MAX(currfeats.max[PAZ], kin->pAz);
    currfeats.max[SINSQATTACK] = MAX(currfeats.max[SINSQATTACK], kin->sinSqAttackAngle);
    currfeats.max[AA] = MAX(currfeats.max[AA], tm->aa);
    currfeats.max[TQ] = MAX(currfeats.max[TQ], tm->tq);
    currfeats.max[AADOT] = MAX(currfeats.max[AADOT], tm->aa_dot);
    currfeats.max[TQDOT] = MAX(currfeats.max[TQDOT], tm->tq_dot);

    currfeats.min[ROT3] = MIN(currfeats.min[ROT3], kin->rot3);
    currfeats.min[AOMEGAX] = MIN(currfeats.min[AOMEGAX], kin->aOmegaX);
    currfeats.min[AOMEGAY] = MIN(currfeats.min[AOMEGAY], kin->aOmegaY);
    currfeats.min[AOMEGAZ] = MIN(currfeats.min[AOMEGAZ], kin->aOmegaZ);
    currfeats.min[AACCX] = MIN(currfeats.min[AACCX], kin->aAccX);
    currfeats.min[AACCY] = MIN(currfeats.min[AACCY], kin->aAccY);
    currfeats.min[AACCZ] = MIN(currfeats.min[AACCZ], kin->aAccZ);
    currfeats.min[AAY] = MIN(currfeats.min[AAY], kin->aAy);
    currfeats.min[VAY] = MIN(currfeats.min[VAY], kin->vAy);
    currfeats.min[PAY] = MIN(currfeats.min[PAY], kin->pAy);
    currfeats.min[AAZ] = MIN(currfeats.min[AAZ], kin->aAz);
    currfeats.min[VAZ] = MIN(currfeats.min[VAZ], kin->vAz);
    currfeats.min[PAZ] = MIN(currfeats.min[PAZ], kin->pAz);
    currfeats.min[SINSQATTACK] = MIN(currfeats.min[SINSQATTACK], kin->sinSqAttackAngle);
    currfeats.min[AA] = MIN(currfeats.min[AA], tm->aa);
    currfeats.min[TQ] = MIN(currfeats.min[TQ], tm->tq);
    currfeats.min[AADOT] = MIN(currfeats.min[AADOT], tm->aa_dot);
    currfeats.min[TQDOT] = MIN(currfeats.min[TQDOT], tm->tq_dot);

}

//copied from matlab pil
void init_learning_structs(){
  init_features();
  init_learner();
  init_classifier();
}







//copied from matlab pil
void predict_task(struct taskmachine_s* tm, struct kinematics_s* kin){
    if (tm->stride_classified || tm->gait_event_trigger != GAIT_EVENT_WINDOW_CLOSE)
        return;
    


    for (int j=0; j < N_PREDICTION_SIGNALS; j++){
        currfeats.rng[j] = currfeats.max[j] - currfeats.min[j];
    }

    currfeats.fin[ROT3] = kin->rot3;
    currfeats.fin[AOMEGAX] = kin->aOmegaX;
    currfeats.fin[AOMEGAY] = kin->aOmegaY;
    currfeats.fin[AOMEGAZ] = kin->aOmegaZ;
    currfeats.fin[AACCX] = kin->aAccX;
    currfeats.fin[AACCY] = kin->aAccY;
    currfeats.fin[AACCZ] = kin->aAccZ;
    currfeats.fin[AAY] = kin->aAy;
    currfeats.fin[VAY] = kin->vAy;
    currfeats.fin[PAY] = kin->pAy;
    currfeats.fin[AAZ] = kin->aAz;
    currfeats.fin[VAZ] = kin->vAz;
    currfeats.fin[PAZ] = kin->pAz;
    currfeats.fin[SINSQATTACK] = kin->sinSqAttackAngle;
    currfeats.fin[AA] = tm->aa;
    currfeats.fin[TQ] = tm->tq;
    currfeats.fin[AADOT] = tm->aa_dot;
    currfeats.fin[TQDOT] = tm->tq_dot;

    //TESTING ONLY
    // reset_features();
    // tm->stride_classified = 1;
    // return;
    
    float maxScore = -FLT_MAX;
    for (int j=0; j < N_CLASSES; j++){
      lda.score_k[j] = 0;
      int ind = j*N_FEATURES;
      int ind_max = ind+MAX_FEATURES_START_IND;
      int ind_min = ind+MIN_FEATURES_START_IND;
      int ind_rng = ind+RNG_FEATURES_START_IND;
      int ind_fin = ind+FIN_FEATURES_START_IND;
      lda.score_k[j] += inner_product(&lda.A[ind_max], currfeats.max, N_PREDICTION_SIGNALS);
      lda.score_k[j] += inner_product(&lda.A[ind_min], currfeats.min, N_PREDICTION_SIGNALS);
      lda.score_k[j] += inner_product(&lda.A[ind_rng], currfeats.rng, N_PREDICTION_SIGNALS);
      lda.score_k[j] += inner_product(&lda.A[ind_fin], currfeats.fin, N_PREDICTION_SIGNALS);

      if (lda.score_k[j] > maxScore){
          maxScore = lda.score_k[j];
          lda.k_pred = j;
      }
    }
    
    reset_features();
    tm->stride_classified = 1;
}




struct learner_s* get_learner(){
  return &lrn;
}

struct classifier_s*  get_classifier(){
  return &lda;
}

struct features_s* get_prev_features(){
  return &prevfeats;
}

struct features_s*  get_curr_features(){
  return &currfeats;
}
