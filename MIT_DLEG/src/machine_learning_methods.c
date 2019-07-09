


#include "machine_learning_methods.h"

 //Machine learning (copied from matlab pil)
#define N_PREDICTION_SIGNALS 5
#define N_PREDICTION_FEATURES 2
#define N_FEATURES (N_PREDICTION_SIGNALS * N_PREDICTION_FEATURES)
#define N_FEATURES_SQ (N_FEATURES * N_FEATURES)

#define RNG_FEATURES_START_IND 0
#define FIN_FEATURES_START_IND (N_PREDICTION_SIGNALS*1)
#define MAX_FEATURES_START_IND (N_PREDICTION_SIGNALS*2)
#define MIN_FEATURES_START_IND (N_PREDICTION_SIGNALS*3)



static struct statistics_s stats;
static struct learner_s lrn;
static struct predictor_s pred;
static struct features_s currfeats;
static struct features_s prevfeats;




static void update_class_sum(){
  int ind = stats.k_est*N_FEATURES;
  int ind_rng = ind+RNG_FEATURES_START_IND;
  int ind_fin = ind+FIN_FEATURES_START_IND;
  stats.pop_k[stats.k_est] = stats.pop_k[stats.k_est] + 1.0; //1 flop
  sum(&stats.sum_k[ind_rng], prevfeats.rng, &stats.sum_k[ind_rng], N_PREDICTION_SIGNALS); // f/4 flops
  sum(&stats.sum_k[ind_fin], prevfeats.fin, &stats.sum_k[ind_fin], N_PREDICTION_SIGNALS); // f/4 flops

}

static void update_overall_sum(){
  assignment(stats.mu, stats.mu_prev, N_FEATURES);// assignment
  stats.pop = stats.pop + 1.0;
  sum(&stats.sum[RNG_FEATURES_START_IND], prevfeats.rng, &stats.sum[RNG_FEATURES_START_IND], N_PREDICTION_SIGNALS); // f/4 flops
  sum(&stats.sum[FIN_FEATURES_START_IND], prevfeats.fin, &stats.sum[FIN_FEATURES_START_IND], N_PREDICTION_SIGNALS); // f/4 flops
  
}


static void reset_features(){
	memset(currfeats.max, 0, N_PREDICTION_SIGNALS * sizeof(float));
	memset(currfeats.min, 0, N_PREDICTION_SIGNALS * sizeof(float));
	memset(currfeats.rng, 0, N_PREDICTION_SIGNALS * sizeof(float));
	memset(currfeats.fin, 0, N_PREDICTION_SIGNALS * sizeof(float));
	memset(prevfeats.max, 0, N_PREDICTION_SIGNALS * sizeof(float));
	memset(prevfeats.min, 0, N_PREDICTION_SIGNALS * sizeof(float));
	memset(prevfeats.rng, 0, N_PREDICTION_SIGNALS * sizeof(float));
	memset(prevfeats.fin, 0, N_PREDICTION_SIGNALS * sizeof(float));

}

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

static void reset_statistics(){
	stats.k_true = 0;
	memset(stats.pop_k_true, 0, N_CLASSES * sizeof(float));
	stats.pop_true = 0;

	memset(stats.mu_k, 0, N_CLASSES*N_FEATURES * sizeof(float));
	memset(stats.sum_k, 0, N_CLASSES*N_FEATURES * sizeof(float));
	memset(stats.mu_prev, 0, N_FEATURES * sizeof(float));
	memset(stats.mu, 0, N_FEATURES * sizeof(float));
	memset(stats.sum, 0, N_FEATURES * sizeof(float));
	memset(stats.temp, 0, N_CLASSES*N_FEATURES * sizeof(float));
	memset(stats.x, 0, N_FEATURES * sizeof(float));
	memset(stats.y, 0, N_FEATURES * sizeof(float));
	memset(stats.sum_sigma, 0, N_FEATURES_SQ * sizeof(float));

	for (int i = 0; i < N_CLASSES; i++){
		stats.pop_k[i] = 1.0;
	}

	int N_FEATURES_p_1 = N_FEATURES+1;
	for (int i = 0; i < N_FEATURES; i++){
		stats.sum_sigma[i*N_FEATURES_p_1] = 1.0;
	}
	stats.pop = 1.0;


	memset(stats.confusion_matrix, 0, N_CLASSES*N_CLASSES * sizeof(int));
	memset(stats.estimation_accuracies, 0, N_CLASSES * sizeof(float));
	memset(stats.prediction_accuracies, 0, N_CLASSES * sizeof(float));
	stats.composite_estimation_accuracy = 0.0;
	stats.composite_prediction_accuracy = 0.0;

  stats.k_est = K_FLAT;
  stats.k_est_prev = K_FLAT;
  stats.updating_statistics_matrices = 1;
  stats.demux_state = STATS_READY_TO_UPDATE_STATISTICS;
  stats.segment = 0;

}

static void init_statistics(){
  stats.k_true = 0;
  stats.pop_k_true = (float*)calloc(N_CLASSES, sizeof(float));
  stats.pop_true = 0;

  stats.mu_k = (float*)calloc(N_CLASSES*N_FEATURES, sizeof(float));
  stats.sum_k = (float*)calloc(N_CLASSES*N_FEATURES, sizeof(float));
  stats.mu_prev  = (float*)calloc( N_FEATURES, sizeof(float));
  stats.mu  = (float*)calloc( N_FEATURES, sizeof(float));
  stats.sum  = (float*)calloc( N_FEATURES, sizeof(float));
  stats.sum_sigma = (float*)calloc(N_FEATURES_SQ, sizeof(float));
  stats.pop_k = (float*)calloc( N_CLASSES, sizeof(float));
  stats.pop = 1.0;

  for (int i = 0; i < N_CLASSES; i++){
    stats.pop_k[i] = 1.0;
  }

  int N_FEATURES_p_1 = N_FEATURES+1;
  for (int i = 0; i < N_FEATURES; i++){
    stats.sum_sigma[i*N_FEATURES_p_1] = 1.0;
  }
  stats.confusion_matrix = (int*)calloc(N_CLASSES*N_CLASSES, sizeof(int));
  stats.estimation_accuracies = (float*)calloc(N_CLASSES, sizeof(float));
  stats.prediction_accuracies = (float*)calloc(N_CLASSES, sizeof(float));
  stats.composite_estimation_accuracy = 0.0;
  stats.composite_prediction_accuracy = 0.0;

  //init intermediary matrices
  stats.temp = (float*)calloc(N_FEATURES, sizeof(float));
  stats.x  = (float*)calloc( N_FEATURES, sizeof(float));
  stats.y  = (float*)calloc( N_FEATURES, sizeof(float));
  stats.k_est = K_FLAT;
  stats.k_est_prev = K_FLAT;

  stats.updating_statistics_matrices = 1;
  stats.demux_state = STATS_READY_TO_UPDATE_STATISTICS;
  stats.segment = 0;

}

static void reset_learner(){

	memset(lrn.UT, 0, N_FEATURES_SQ * sizeof(float));
	memset(lrn.LT, 0, N_FEATURES_SQ * sizeof(float));
	memset(lrn.latest_sum_sigma, 0, N_FEATURES_SQ * sizeof(float));
	memset(lrn.latest_mu_k, 0, N_CLASSES*N_FEATURES * sizeof(float));
	memset(lrn.Atemp, 0, N_CLASSES*N_FEATURES * sizeof(float));
	memset(lrn.Btemp, 0, N_CLASSES * sizeof(float));
	memset(lrn.y, 0, N_FEATURES * sizeof(float));

  lrn.copying_statistics_matrices = 0;
  lrn.demux_state = LRN_COPY_MU_K;
  lrn.segment = 0;
  lrn.subsegment = 0;
  lrn.doing_forward_substitution = 1;

}

static void init_learner(){


  lrn.UT = (float*)calloc(N_FEATURES_SQ, sizeof(float));
  lrn.LT = (float*)calloc(N_FEATURES_SQ, sizeof(float));

  lrn.latest_sum_sigma = (float*)calloc(N_FEATURES_SQ, sizeof(float));
  lrn.latest_mu_k = (float*)calloc(N_CLASSES*N_FEATURES, sizeof(float));

  lrn.Atemp = (float*)calloc(N_CLASSES * N_FEATURES, sizeof(float));
  lrn.Btemp = (float*)calloc(N_CLASSES, sizeof(float));
  lrn.y  = (float*)calloc( N_FEATURES, sizeof(float));

  lrn.copying_statistics_matrices = 0;
  lrn.demux_state = LRN_COPY_MU_K;
  lrn.segment = 0;
  lrn.subsegment = 0;
  lrn.doing_forward_substitution = 1;

}

static void reset_predictor(){
	memset(pred.A, 0, N_CLASSES*N_FEATURES * sizeof(float));
	memset(pred.B, 0, N_CLASSES * sizeof(float));
	memset(pred.score_k, 0, N_CLASSES * sizeof(float));

  pred.k_pred = 0;
  pred.max_score = -FLT_MAX;
  pred.predicting_task = 0;
  pred.demux_state = PRED_UPDATE_RNG;
  pred.segment = 0;
  pred.subsegment = 0;
}

static void init_predictor(){
  pred.A = (float*)calloc(N_CLASSES * N_FEATURES, sizeof(float));
  pred.B = (float*)calloc(N_CLASSES, sizeof(float));
  pred.score_k = (float*)calloc(N_CLASSES, sizeof(float));
  pred.k_pred = 0;
  pred.max_score = -FLT_MAX;

  pred.predicting_task = 0;
  pred.demux_state = PRED_UPDATE_RNG;
  pred.segment = 0;
  pred.subsegment = 0;
}

static void update_confusion_matrix_values(){

	float k_est_correctness = (float)(stats.k_est == stats.k_true);
	float k_pred_correctness = (float)(pred.k_pred == stats.k_true);
	stats.estimation_accuracies[stats.k_true] = (stats.estimation_accuracies[stats.k_true]*stats.pop_k[stats.k_true] + k_est_correctness)/(stats.pop_k[stats.k_true] + 1.0);
	stats.prediction_accuracies[stats.k_true] = 0.63*stats.prediction_accuracies[stats.k_true] + 0.37*k_pred_correctness;

	stats.composite_estimation_accuracy = (stats.composite_estimation_accuracy*stats.pop_true + k_est_correctness)/(stats.pop_true + 1.0);
	stats.composite_prediction_accuracy = 0.63*stats.composite_prediction_accuracy + 0.37*k_pred_correctness;
	stats.pop_true = stats.pop_true + 1.0;

}


void update_statistics_demux(struct taskmachine_s* tm, struct kinematics_s* kin){
    switch (stats.demux_state){
      case STATS_BACK_ESTIMATE: //constant flops
          back_estimate(tm, &stats, kin);
          update_confusion_matrix_values();
          stats.demux_state = STATS_UPDATE_CLASS_SUM;
      break;
      case STATS_UPDATE_CLASS_SUM: //f flops per cycle
          if (lrn.copying_statistics_matrices)
              return;
          stats.updating_statistics_matrices = 1;
          update_class_sum();
          stats.demux_state = STATS_UPDATE_CLASS_MEAN;
      break;
      case STATS_UPDATE_CLASS_MEAN: // f flops per cycle
      {
    	int ind = stats.k_est*N_FEATURES;
        scaling (&stats.sum_k[ind], 1.0/stats.pop_k[stats.k_est], &stats.mu_k[ind], N_FEATURES);
        stats.demux_state = STATS_UPDATE_OVERALL_SUM;
      }
      break;
      case STATS_UPDATE_OVERALL_SUM: //f flops per cycle
          update_overall_sum();
          stats.demux_state = STATS_UPDATE_OVERALL_MEAN;
      break;
      case STATS_UPDATE_OVERALL_MEAN: // f flops per cycle
        scaling(stats.sum, 1.0/stats.pop, stats.mu, N_FEATURES);
        stats.demux_state = STATS_GET_DEVIATION_FROM_PREV_MEAN;
      break;
      case STATS_GET_DEVIATION_FROM_PREV_MEAN: // f flops per cycle
        diff(prevfeats.rng, &stats.mu_prev[RNG_FEATURES_START_IND], &stats.x[RNG_FEATURES_START_IND], N_PREDICTION_SIGNALS);
        diff(prevfeats.fin, &stats.mu_prev[FIN_FEATURES_START_IND], &stats.x[FIN_FEATURES_START_IND], N_PREDICTION_SIGNALS);
        stats.demux_state = STATS_GET_DEVIATION_FROM_CURR_MEAN;
      break;
      case STATS_GET_DEVIATION_FROM_CURR_MEAN: // f flops per cycle
        diff(prevfeats.rng, &stats.mu[RNG_FEATURES_START_IND], &stats.y[RNG_FEATURES_START_IND], N_PREDICTION_SIGNALS);
        diff(prevfeats.fin, &stats.mu[FIN_FEATURES_START_IND], &stats.y[FIN_FEATURES_START_IND], N_PREDICTION_SIGNALS);
        stats.demux_state = STATS_UPDATE_MEAN_DEVIATION_OUTER_PRODUCT;
      break;
      case STATS_UPDATE_MEAN_DEVIATION_OUTER_PRODUCT: // f flops per cycle
      {
        int segment_section = stats.segment*N_FEATURES;
        scaling(stats.y, stats.x[stats.segment],stats.temp, N_FEATURES);
        sum(&stats.sum_sigma[segment_section], stats.temp, &stats.sum_sigma[segment_section], N_FEATURES);
        stats.segment++;
        if (stats.segment == N_FEATURES){
          stats.demux_state = STATS_READY_TO_UPDATE_STATISTICS; 
          stats.updating_statistics_matrices = 0;
          stats.segment = 0;
        }
      }
      break;
      case STATS_READY_TO_UPDATE_STATISTICS: //constant flops
        if (tm->gait_event_trigger == GAIT_EVENT_FOOT_OFF){
              stats.k_est_prev = stats.k_est;
              stats.k_est = K_DEFAULT;
               if (tm->do_learning_for_prev_stride){
                 stats.demux_state = STATS_BACK_ESTIMATE;
               }
             }
      break;
    }
  }


void update_learner_demux(){

      switch (lrn.demux_state){
      case LRN_COPY_MU_K://f assignments per cycle, 5 cycles
        {
          if (stats.updating_statistics_matrices)
              return;

          int ind = lrn.segment*N_FEATURES;
          assignment(&stats.mu_k[ind], &lrn.latest_mu_k[ind], N_FEATURES);
          lrn.segment++;
          if (lrn.segment == N_CLASSES){
              lrn.demux_state = LRN_COPY_SUM_SIGMA;
              lrn.segment = 0;
          }
        }
      break;

      case LRN_COPY_SUM_SIGMA: //f assignments per cycle, f cycles
      {
          int ind = lrn.segment*N_FEATURES;
          assignment(&stats.sum_sigma[ind], &lrn.latest_sum_sigma[ind], N_FEATURES);
          lrn.segment++;
          if (lrn.segment == N_FEATURES){
            lrn.copying_statistics_matrices = 0;
            lrn.demux_state = LRN_DO_CHOLESKY;
            lrn.segment = 0;
          }
        }
      break;
      case LRN_DO_CHOLESKY: // max f flops per cycle, f(f+1)/2 cycles
        super_segmented_cholesky(lrn.latest_sum_sigma, lrn.LT, N_FEATURES, lrn.segment, lrn.subsegment);
        lrn.subsegment++;
        if (lrn.subsegment > lrn.segment){
          lrn.subsegment = 0;
          lrn.segment++;
        }

        if (lrn.segment == N_FEATURES){
          lrn.demux_state = LRN_UPDATE_TRANSPOSE; 
          lrn.segment = 0;
        }
      break;
      case LRN_UPDATE_TRANSPOSE: //f flops per cycle, f cycles
        segmented_lower_to_upper_transpose(lrn.LT, lrn.UT, N_FEATURES, lrn.segment);
        lrn.segment++;
        if (lrn.segment == N_FEATURES){
          lrn.demux_state = LRN_CALC_A_PARAMS;
          lrn.segment = 0;
        }
      break;
      case LRN_CALC_A_PARAMS: //f flops per cycle max, 10*f cycles
      {
        int ind = lrn.segment*N_FEATURES;
        if (lrn.doing_forward_substitution){
          segmented_forward_substitution(lrn.LT, &lrn.latest_mu_k[ind], lrn.y, N_FEATURES, lrn.subsegment); 
          lrn.subsegment++;
          if (lrn.subsegment == N_FEATURES){
            lrn.subsegment = N_FEATURES-1;
            lrn.doing_forward_substitution = 0;
          }
        }
        else{
          segmented_backward_substitution(lrn.UT, lrn.y, &lrn.Atemp[ind], N_FEATURES, lrn.subsegment);
          lrn.subsegment--;
          if (lrn.subsegment == -1){
            lrn.subsegment = 0;
            lrn.doing_forward_substitution = 1;
            lrn.segment++;
          }
        }
        if (lrn.segment == N_CLASSES){
          lrn.demux_state = LRN_CALC_B_PARAMS; 
          lrn.segment = 0;
        }  
      }
      break;
      case LRN_CALC_B_PARAMS: //f flops, 10 cycles
      {
        int ind = lrn.segment*N_FEATURES;
        if (lrn.subsegment == 0){
         lrn.Btemp[lrn.segment] = -0.5*inner_product(&lrn.latest_mu_k[ind], &lrn.Atemp[ind], RNG_FEATURES_START_IND);
         lrn.subsegment++;
        }else{
          int ind_rng = ind+RNG_FEATURES_START_IND;
          lrn.Btemp[lrn.segment] = lrn.Btemp[lrn.segment] - 0.5*inner_product(&lrn.latest_mu_k[ind_rng], &lrn.Atemp[ind_rng], RNG_FEATURES_START_IND);
          lrn.subsegment = 0;
          lrn.segment++;
        }
  
        
        if (lrn.segment == N_CLASSES){
          lrn.segment = 0;
          lrn.demux_state = LRN_UPDATE_PARAMS;
        }  
      }    
      break;
      case LRN_UPDATE_PARAMS: //f+5 assignments max, 5 cycles
      {
        if (pred.predicting_task){
          return;
        }
        int ind = lrn.segment*N_FEATURES;
        assignment(&lrn.Atemp[ind], &pred.A[ind], N_FEATURES);
        lrn.segment++;
        if (lrn.segment == N_CLASSES){
          assignment(lrn.Btemp, pred.B, N_CLASSES);
          lrn.segment = 0;
          lrn.demux_state = LRN_COPY_MU_K; 
          lrn.copying_statistics_matrices = 1;
        }   
      }
      break;
  }
}


void update_prediction_features(struct taskmachine_s* tm, struct kinematics_s* kin){
    if (tm->gait_event_trigger == GAIT_EVENT_FOOT_OFF){
        for (int j=0; j < N_PREDICTION_SIGNALS; j++){
            currfeats.max[j] = -FLT_MAX;
            currfeats.min[j] = FLT_MAX;
        }
        return;
    }

  if (tm->stride_classified || tm->gait_event_trigger == GAIT_EVENT_WINDOW_CLOSE)
        return;

    currfeats.max[ROT3] = MAX(currfeats.max[ROT3], kin->rot3);
    currfeats.max[AOMEGAX] = MAX(currfeats.max[AOMEGAX], kin->aOmegaX);
    currfeats.max[AACCY] = MAX(currfeats.max[AACCY], kin->aAccY);
    currfeats.max[AACCZ] = MAX(currfeats.max[AACCZ], kin->aAccZ);
    currfeats.max[PAZ] = MAX(currfeats.max[PAZ], kin->pAz);

    currfeats.min[ROT3] = MIN(currfeats.min[ROT3], kin->rot3);
    currfeats.min[AOMEGAX] = MIN(currfeats.min[AOMEGAX], kin->aOmegaX);
    currfeats.min[AACCY] = MIN(currfeats.min[AACCY], kin->aAccY);
    currfeats.min[AACCZ] = MIN(currfeats.min[AACCZ], kin->aAccZ);
    currfeats.min[PAZ] = MIN(currfeats.min[PAZ], kin->pAz);

}

void reset_learning_structs(){
	 reset_features();
	 reset_statistics();
	 reset_learner();
	 reset_predictor();
}


void init_learning_structs(){
  init_features();
  init_statistics();
  init_learner();
  init_predictor();
}





void predict_task_demux(struct taskmachine_s* tm, struct kinematics_s* kin){

      switch (pred.demux_state){
      case PRED_UPDATE_RNG: //f/4 flops
          for (int j=0; j < N_PREDICTION_SIGNALS; j++){
            currfeats.rng[j] = currfeats.max[j] - currfeats.min[j];
          }
          pred.demux_state = PRED_UPDATE_FIN;

      break;

      case PRED_UPDATE_FIN: //f/4 flops
            currfeats.fin[ROT3] = kin->rot3;
            currfeats.fin[AOMEGAX] = kin->aOmegaX;
            currfeats.fin[AACCY] = kin->aAccY;
            currfeats.fin[AACCZ] = kin->aAccZ;
            currfeats.fin[PAZ] = kin->pAz;
            pred.predicting_task = 1;
            pred.demux_state = PRED_PREDICT;
      
      break;
      case PRED_PREDICT: //f flops
      {
            int ind = pred.segment*N_FEATURES;
			pred.score_k[pred.segment] = pred.B[pred.segment];
			int ind_rng = ind+RNG_FEATURES_START_IND;
			int ind_fin = ind+FIN_FEATURES_START_IND;
			pred.score_k[pred.segment] += inner_product(&pred.A[ind_rng], currfeats.rng, N_PREDICTION_SIGNALS);
			pred.score_k[pred.segment] += inner_product(&pred.A[ind_fin], currfeats.fin, N_PREDICTION_SIGNALS);
			if (pred.score_k[pred.segment] > pred.max_score){
				pred.max_score = pred.score_k[pred.segment];
				pred.k_pred = pred.segment;
			}
			pred.segment++;

            if (pred.segment == N_CLASSES){
              pred.segment = 0;
              pred.subsegment = 0;
              pred.demux_state = PRED_UPDATE_PREV_FEATS; 
            }
          }
      break;
      case PRED_UPDATE_PREV_FEATS: //f assignments
          pred.predicting_task = 0;
          assignment(currfeats.rng, prevfeats.rng, N_PREDICTION_SIGNALS);
          assignment(currfeats.fin, prevfeats.fin, N_PREDICTION_SIGNALS);
          pred.demux_state = PRED_READY_TO_PREDICT;
          tm->stride_classified = 1;

      break;
      case PRED_READY_TO_PREDICT: //constants flops
      if (tm->gait_event_trigger == GAIT_EVENT_WINDOW_CLOSE){
              pred.demux_state = PRED_UPDATE_RNG; 
            pred.max_score = -FLT_MAX;
          }
      break;
  }
}

struct statistics_s* get_statistics(){
  return &stats;
}

struct learner_s*  get_learner(){
  return &lrn;
}

struct predictor_s*  get_predictor(){
  return &pred;
}

struct features_s* get_prev_features(){
  return &prevfeats;
}

struct features_s*  get_curr_features(){
  return &currfeats;
}
