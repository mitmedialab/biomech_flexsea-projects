


#include "machine_learning_methods.h"

 //Machine learning (copied from matlab pil)
#define N_CLASSES 5
#define N_PREDICTION_SIGNALS 18
#define N_PREDICTION_FEATURES 4
#define N_FEATURES N_PREDICTION_SIGNALS * N_PREDICTION_FEATURES
#define N_FEATURES_SQ N_FEATURES * N_FEATURES

 //Back estimation constants (copied from matlab pil)
#define US_Z_THRESH 0.1
#define DS_Z_THRESH -0.25
#define DS_Z_SAMPLE_THRESH 500
#define UR_ANKLE_ANGLE_THRESH -20.0
#define DR_ANKLE_ANGLE_THRESH 8.0
#define DS_ANKLE_ANGLE_THRESH -10.0
#define US_SIN_SQ_ATTACK_ANGLE_THRESH 0.2


static struct classifier_s lda;
static struct learner_s lrn;
static struct features_s* currfeats;
static struct features_s* prevfeats;

static void back_estimate(struct back_estimator_s* be, int latest_foot_off_samples){
  
  be->mean_stance_theta = be->mean_stance_theta/((float) latest_foot_off_samples);
  lrn.k_est = TASK_FL;
  if (be->max_swing_z > US_Z_THRESH && be->max_swing_z_samples < US_MAX_SAMPLES_TO_Z_THRESH){
      lrn.k_est = TASK_US;
  }else if (be->max_swing_z > UR_Z_THRESH && be->mean_stance_theta < UR_MEAN_THETA_THRESH){
      lrn.k_est = TASK_UR;
  }else if (be->min_swing_z < DS_Z_THRESH && be->prev_max_stance_tq > DS_TQ_THRESH){
      lrn.k_est = TASK_DS;
  }else if (be->min_swing_z < DR_Z_THRESH && be->mean_stance_theta > DR_MEAN_THETA_THRESH){
      lrn.k_est = TASK_DR;
  }
  be->mean_stance_theta = 0.0;
  be->prev_max_stance_tq = be->max_stance_tq;
}

static void update_class_mean(){
  int ind = lrn.k_est*N_FEATURES;
  float popkP1 = lrn.pop_k[lrn.k_est] + 1.0; //1 flop
  sum(&lrn.sum_k[ind], prev_feats, &lrn.sum_k[ind], N_FEATURES); // f flops
  scaling (&lrn.sum_k[ind], 1.0/popkP1, &lrn.mu_k[ind], N_FEATURES); // f flops
  lrn.pop_k[lrn.k_est] = popkP1;

}

static void update_overall_mean(){
  assignment(lrn.mu, lrn.mu_prev, N_FEATURES);// assignment
  sum(lrn.sum, prev_feats, lrn.sum, N_FEATURES); // f flops
  scaling(lrn.sum, 1.0/(lrn.pop + 1.0), lrn.mu, N_FEATURES); // f flops
}

//Just temporarily populated. NEEDS TO BE CHANGED
static void reset_features(){
  for (int i = 0; i < N_FEATURES; i++){
    prev_feats[i] = curr_feats[i];
  }
  curr_feats[PAZ_MAX] = FLT_MIN;
  curr_feats[PAZ_SUM] = 0.0f;
  curr_feats[VAZ_MIN] = FLT_MAX;
  curr_feats[PITCH_RANGE] = 0.0f;
  curr_feats[OMX_MAX] = FLT_MIN;
  curr_feats[ACCY_SUM] = 0.0f;
  curr_feats[PITCH_MAX] = FLT_MIN;
  curr_feats[PITCH_MIN] = FLT_MAX;
}



static void reset_classifier_update_demux(){
  lda.demux_state = COPY_SUM_SIGMA;
  lda.making_matrix_copy = 1;
  lda.segment = 0;
  lda.subsegment = 0;
  lda.current_updating_class = 0;
  lda.doing_forward_substitution = 1;

}


//copied from matlab pil
void update_learner_demux(struct taskmachine_s* tm, struct back_estimator_s* be){
    switch (lrn.demux_state){
      case LRN_BACK_ESTIMATE:
          lrn.k_est = 1;
          back_estimate(be, latest_foot_off_samples);
          lrn.demux_state = LRN_UPDATE_CLASS_MEAN;
      break;
      case LRN_UPDATE_CLASS_MEAN: 
          if (lda.copying_learner_matrices)
              return;
          update_class_mean()
          lrn.demux_state = cn.LRN_UPDATE_OVERALL_MEAN;
      break;
      case LRN_UPDATE_OVERALL_MEAN: // 1 sample
        update_overall_mean(); //2f flops
        lrn.demux_state = LRN_GET_DEVIATION_FROM_CURR_MEAN;
      break;
      case LRN_GET_DEVIATION_FROM_CURR_MEAN: // 1 sample
        diff(prevfeats, lrn.mu, lrn.x, N_FEATURES);//f flops
        lrn.demux_state = LRN_GET_DEVIATION_FROM_PREV_MEAN;
      break;
      case LRN_GET_DEVIATION_FROM_PREV_MEAN: // 1 sample
        diff(prevfeats, lrn.mu_prev, lrn.y, N_FEATURES); //f flops
        lrn.demux_state = LRN_UPDATE_COVARIANCE;
      break;
      case LRN_UPDATE_COVARIANCE: // f samples
      {
        int segment_section = lrn.segment*N_FEATURES;
        segmented_outer_product(lrn.x, lrn.y, lrn.A, N_FEATURES, lrn.segment); //2f flops
        sum(&lrn.sum_sigma[segment_section], &lrn.A[segment_section], &lrn.sum_sigma[segment_section], N_FEATURES);//f flops
        lrn.segment++;
        if (lrn.segment == N_FEATURES){
          lrn.demux_state = LRN_READY_TO_UPDATE_LEARNER; 
          lrn.segment = 0;
          lrn.updating_learner_matrices = 0;
        }
      }
      break;
      case LRN_READY_TO_UPDATE_LEARNER:
        if (tm.gait_event_trigger == GAIT_EVENT_FOOT_OFF){
               if (tm.do_learning_for_stride){
                   tm.do_learning_for_stride = 0;
                 lrn.demux_state = LRN_BACK_ESTIMATE;
               }
             }
      break;
    }
  }

//copied from matlab pil
void update_classifier_demux(){

      switch (lda.demux_state){
      case LDA_COPY_MU_K:
          if (lrn.updating_learner_matrices)
              return;
          int ind = lda.segment*N_FEATURES;
          assignment(&lrn.mu_k[ind], &lda.latest_mu_k[ind], N_FEATURES);
          lda.segment++;
          if (lda.segment == cn.N_CLASSES){
              lda.demux_state = LDA_COPY_SUM_SIGMA;
              lda.segment = 0;
          }
      break;

      case LDA_COPY_SUM_SIGMA:
          int ind = lda.segment*N_FEATURES;
          assignment(&lrn.sum_sigma[ind], &lda.latest_sum_sigma[ind], N_FEATURES);
          lda.segment = lda.segment++;
          if (lda.segment == N_FEATURES){
            lda.making_matrix_copy = 0;
            lda.demux_state = LDA_DO_CHOLESKY;
            lda.segment = 0;
          }
      break;
      case LDA_DO_CHOLESKY: // f(f+1)/2 samples
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
      case LDA_UPDATE_TRANSPOSE: //f samples
        segmented_lower_to_upper_transpose(lda.LT, lda.UT, N_FEATURES, lda.segment);
        lda.segment++;
        if (lda.segment == N_FEATURES){
          lda.demux_state = LDA_CALC_A_PARAMS;
          lda.segment = 0;
        }
      break;
      case LDA_CALC_A_PARAMS: //2*c*f
      {
        int ind = lda.segment*N_FEATURES;
        if (lda.doing_forward_substitution){
          segmented_forward_substitution(lda.LT, &lrn.sum_k[ind], lda.y, N_FEATURES, lda.subsegment); // roughly 1/2 f^2 flops
          lda.subsegment++;
          if (lda.subsegment == N_FEATURES){
            lda.subsegment = N_FEATURES-2;
            lda.doing_forward_substitution = 0;
          }
        }
        else{
          segmented_backward_substitution(lda.UT, lda.y, &lda.Atemp[ind], N_FEATURES, subsegment); // roughly 1/2 f^2 flops
          lda.subsegment--;
          if (lda.subsegment == -1){
            //lda.B[lda.current_updating_class] = -0.5*inner_product(&lrn.sum_k[ind], &lda.A[ind], N_FEATURES);
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
      case LDA_CALC_B_PARAMS:
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
      lda.segment = lda.segment + 1;

      case LDA_UPDATE_PARAMS:
          int ind = lda.segment*N_FEATURES;
          assignment(&lda.Atemp[ind], &lda.A[ind], N_FEATURES);  
          lda.segment++;
          if (lda.segment == N_CLASSES){
            assignment(lda.Btemp, lda.B, N_CLASSES);
            lda.segment = 0;
            lda.demux_state = LDA_COPY_MU_K; 
          }
          
      break;
  }
}

//copied from matlab pil
void update_prediction_features(struct taskmachine_s* tm, struct kinematics_s* kin){
  if (tm.stride_classified || tm.gait_event_trigger == GAIT_EVENT_WINDOW_CLOSE)
        return;

    currfeats.max[ROT3] = MAX(currfeats.max[ROT3], kin.rot3);
    currfeats.max[AOMEGAX] = MAX(currfeats.max[AOMEGAX], kin.aOmegaX);
    currfeats.max[AOMEGAY] = MAX(currfeats.max[AOMEGAY], kin.aOmegaY);
    currfeats.max[AOMEGAZ] = MAX(currfeats.max[AOMEGAZ], kin.aOmegaZ);
    currfeats.max[AACCX] = MAX(currfeats.max[AACCX], kin.aAccX);
    currfeats.max[AACCY] = MAX(currfeats.max[AACCY], kin.aAccY);
    currfeats.max[AACCZ] = MAX(currfeats.max[AACCZ], kin.aAccZ);
    currfeats.max[AAY] = MAX(currfeats.max[AAY], kin.aAy);
    currfeats.max[VAY] = MAX(currfeats.max[VAY], kin.vAy);
    currfeats.max[PAY] = MAX(currfeats.max[PAY], kin.pAy);
    currfeats.max[AAZ] = MAX(currfeats.max[AAZ], kin.aAz);
    currfeats.max[VAZ] = MAX(currfeats.max[VAZ], kin.vAz);
    currfeats.max[PAZ] = MAX(currfeats.max[PAZ], kin.pAz);
    currfeats.max[SINSQATTACK] = MAX(currfeats.max[SINSQATTACK], kin.sinSqAttackAngle);
    currfeats.max[AA] = MAX(currfeats.max[AA], tm.aa);
    currfeats.max[TQ] = MAX(currfeats.max[TQ], tm.tq);
    currfeats.max[AADOT] = MAX(currfeats.max[AADOT], tm.aa_dot);
    currfeats.max[TQDOT] = MAX(currfeats.max[TQDOT], tm.tq_dot);

    currfeats.min[ROT3] = MIN(currfeats.min[ROT3], kin.rot3);
    currfeats.min[AOMEGAX] = MIN(currfeats.min[AOMEGAX], kin.aOmegaX);
    currfeats.min[AOMEGAY] = MIN(currfeats.min[AOMEGAY], kin.aOmegaY);
    currfeats.min[AOMEGAZ] = MIN(currfeats.min[AOMEGAZ], kin.aOmegaZ);
    currfeats.min[AACCX] = MIN(currfeats.min[AACCX], kin.aAccX);
    currfeats.min[AACCY] = MIN(currfeats.min[AACCY], kin.aAccY);
    currfeats.min[AACCZ] = MIN(currfeats.min[AACCZ], kin.aAccZ);
    currfeats.min[AAY] = MIN(currfeats.min[AAY], kin.aAy);
    currfeats.min[VAY] = MIN(currfeats.min[VAY], kin.vAy);
    currfeats.min[PAY] = MIN(currfeats.min[PAY], kin.pAy);
    currfeats.min[AAZ] = MIN(currfeats.min[AAZ], kin.aAz);
    currfeats.min[VAZ] = MIN(currfeats.min[VAZ], kin.vAz);
    currfeats.min[PAZ] = MIN(currfeats.min[PAZ], kin.pAz);
    currfeats.min[SINSQATTACK] = MIN(currfeats.min[SINSQATTACK], kin.sinSqAttackAngle);
    currfeats.min[AA] = MIN(currfeats.min[AA], tm.aa);
    currfeats.min[TQ] = MIN(currfeats.min[TQ], tm.tq);
    currfeats.min[AADOT] = MIN(currfeats.min[AADOT], tm.aa_dot);
    currfeats.min[TQDOT] = MIN(currfeats.min[TQDOT], tm.tq_dot);

}

void classify(){

  curr_feats[PITCH_RANGE] = curr_feats[PITCH_MAX] - curr_feats[PITCH_MIN];

  float maxScore = FLT_MIN;
  for (int i = 0; i < 5; i++){
    lda.score_k[i] = inner_product(&lda.A[i*N_FEATURES], curr_feats, N_FEATURES) + lda.B[i];
    if (lda.score_k[i] > maxScore){
      maxScore = lda.score_k[i];
      lda.k_pred = i;
    }
  }
  reset_features();
}

void init_learning_structs(){
  init_features(currfeats);
  init_features(prevfeats);
  init_learner();
  init_classifier();
}

//copied from matlab pil
static void init_features(struct features_s* feats){
  feats.max = (float*)calloc(N_PREDICTION_SIGNALS, sizeof(float));
  feats.min = (float*)calloc(N_PREDICTION_SIGNALS, sizeof(float));
  feats.rng = (float*)calloc(N_PREDICTION_SIGNALS, sizeof(float));
  feats.fin = (float*)calloc(N_PREDICTION_SIGNALS, sizeof(float));
}

//copied from matlab pil
static void init_learner(){

  lrn.mu_k = (float*)calloc(N_CLASSES	 * N_FEATURES, sizeof(float));
  lrn.sum_k = (float*)calloc(N_CLASSES	 * N_FEATURES, sizeof(float));
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
  lrn.x  = (float*)calloc( N_FEATURES, sizeof(float));
  lrn.y  = (float*)calloc( N_FEATURES, sizeof(float));
  lrn.k_est = 1;

  lrn.updating_learner_matrices = 1;
  lrn.demux_state = LRN_READY_TO_UPDATE_LEARNER;
  lrn.segment = 0;

}

//copied from matlab pil
static void init_classifier(){
  lda.Atemp = (float*)calloc(N_CLASSES * N_FEATURES, sizeof(float));
  lda.Btemp = (float*)calloc(N_CLASSES, sizeof(float));
  lda.A = (float*)calloc(N_CLASSES * N_FEATURES, sizeof(float));
  lda.B = (float*)calloc(N_CLASSES, sizeof(float));
  lda.score_k = (float*)calloc(N_CLASSES, sizeof(float));
  lda.k_pred = 0;

  lda.UT = (float*)calloc(N_FEATURES_SQ, sizeof(float));
  lda.LT = (float*)calloc(N_FEATURES_SQ, sizeof(float));

  lda.latest_sum_sigma = (float*)calloc(N_FEATURES_SQ, sizeof(float));
  lda.latest_mu_k = (float*)calloc(N_CLASSES	 * N_FEATURES, sizeof(float));

  lda.copying_learner_matrices = 0;
  lda.demux_state = LDA_COPY_MU_K;
  lda.segment = 0;
  lda.subsegment = 0;
  lda.doing_forward_substitution = 1;

}

struct learner_s* get_learner(){
  return &lrn;
}

struct classifier_s*  get_classifier(){
  return &lda;
}

float* get_prev_features(){
  return prev_feats;
}

float* get_curr_features(){
  return curr_feats;
}
