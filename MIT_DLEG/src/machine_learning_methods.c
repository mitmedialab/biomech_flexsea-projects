


#include "machine_learning_methods.h"
#include "linear_algebra_methods.h"


#define NCLASSES 5
#define NFEATURES 20
#define NFEATURES_SQ NFEATURES * NFEATURES

static int learning_demux_state = 0;
static int current_updating_class = 0;
static int current_updating_feature = 0;

void reset_learning_demux(){
  learning_demux_state = 0;
}

int learning_demux(struct learner_s* lrn, struct classifier_s* lda, float* feats, int k_est){
    int status = 0;
    switch (learning_demux_state){
      case UPDATE_CLASS_MEAN:
        update_class_mean(lrn, feats, k_est);
        learning_demux_state++;
      break;
      case UPDATE_OVERALL_MEAN:
        update_overall_mean(lrn, feats);
        learning_demux_state++;
      break;
      case UPDATE_COVARIANCE_1:
        diff(feats, lrn->mu, lrn->x, NFEATURES);//f flops
        diff(feats, lrn->mu_prev, lrn->y, NFEATURES); //f flops
        outer_product(lrn->x, lrn->y, lrn->A, NFEATURES); //f^2 flops
        learning_demux_state++;
      break;
      case UPDATE_COVARIANCE_2:
        scaling(lrn->sigma, lrn->pop, lrn->sigma, NFEATURES_SQ);//f^2 flops
        learning_demux_state++;
      break;
      case UPDATE_COVARIANCE_3:
        sum(lrn->sigma, lrn->A, lrn->sigma, NFEATURES_SQ);//f^2 flops
        learning_demux_state++;
      break;
      case UPDATE_COVARIANCE_4:
      {
        float popP1 = lrn->pop + 1.0;
        scaling(lrn->sigma, 1.0/popP1, lrn->sigma, NFEATURES_SQ);//f^2 flops
        lrn->pop = popP1;
        learning_demux_state++;
      }
      break;
      case DO_CHOLESKY:
        segmented_cholesky_decomposition(lrn->sigma, lrn->LT, NFEATURES, current_updating_feature);
        current_updating_feature++;
        if (current_updating_feature == NFEATURES){
          learning_demux_state++; 
          current_updating_feature = 0;
        }
      break;
      case GET_TRANSPOSE:
        lower_to_upper_transpose(lrn->LT, lrn->UT, NFEATURES); // assignment
        learning_demux_state++; 
      break;
      case GET_LDA_PARAMS:
      {
        int class_times_n_features = current_updating_class*NFEATURES;
        forward_substitution(lrn->LT,&lrn->mu_k[class_times_n_features], lrn->y, NFEATURES); // roughly 1/2 f^2 flops
        backward_substitution(lrn->UT,lrn->y, &lda->A[class_times_n_features], NFEATURES); // roughly 1/2 f^2 flops
        lda->B[current_updating_class] = -0.5*inner_product(&lrn->mu_k[class_times_n_features], &lda->A[class_times_n_features], NFEATURES); //assumes uniform priors, 2f flops
        current_updating_class++;
        if (current_updating_class == NCLASSES){
          learning_demux_state++; 
          current_updating_class = 0;
        }
        status = 1;
      }
      break;
    }
     return status;
} 

void update_class_mean(struct learner_s* lrn, float* feats, int k_est){
  int ind = k_est*NFEATURES;
  float popkP1 = lrn->pop_k[k_est] + 1.0; //1 flop
  scaling(&lrn->mu_k[ind], lrn->pop_k[k_est], &lrn->mu_k[ind], NFEATURES); //f flops
  sum(&lrn->mu_k[ind], feats, &lrn->mu_k[ind], NFEATURES); // f flops
  scaling (&lrn->mu_k[ind], 1.0/popkP1, &lrn->mu_k[ind], NFEATURES); // f flops
  lrn->pop_k[k_est] = popkP1;

}

void update_overall_mean(struct learner_s* lrn, float* feats){
  assignment(lrn->mu, lrn->mu_prev, NFEATURES);// assignment
  scaling(lrn->mu, lrn->pop, lrn->mu, NFEATURES); //f flops
  sum(lrn->mu, feats, lrn->mu, NFEATURES); // f flops
  scaling(lrn->mu, 1.0/(lrn->pop + 1.0), lrn->mu, NFEATURES); // f flops

}


void classify(struct classifier_s* lda, float* feats){
  float maxScore = -1000000000.0;
  for (int i = 0; i < 5; i++){
    lda->score_k[i] = inner_product(&lda->A[i*NFEATURES], feats, NFEATURES) + lda->B[i];
    if (lda->score_k[i] > maxScore){
      maxScore = lda->score_k[i];
      lda->k_pred = i;
    }
  }
}


void init_learner(struct learner_s* lrn){

  lrn->mu_k = (float*)calloc(NCLASSES * NFEATURES, sizeof(float));
  lrn->mu_prev  = (float*)calloc( NFEATURES, sizeof(float));
  lrn->mu  = (float*)calloc( NFEATURES, sizeof(float));
  lrn->sigma = (float*)calloc(NFEATURES_SQ, sizeof(float));

  float initsigma[] = {1.369190, -0.126728, 0.318558, -2.298324, -1.913517, -0.870778, -0.422216, -2.357705, 0.938830, -1.837171, 2.120478, -1.133075, -1.528519, 1.257752, 0.856214, -0.229157, -1.611274, -0.130335, 1.526603, -1.156691, -0.126728, 0.139849, 0.259685, 1.166007, 0.481319, -0.418904, 0.027714, 0.741818, -0.180179, 0.001625, -0.034905, 0.022485, 0.181634, 0.385736, -0.093634, 0.135515, -0.041764, 0.055648, 0.162822, 0.742783, 0.318558, 0.259685, 1.116231, 2.030530, 0.889800, -2.272904, 0.132110, 0.397549, 0.407549, -0.748567, 0.832796, -1.346801, 0.006707, 1.196807, 1.182328, -0.473380, -1.095547, 0.114069, 0.790382, 0.496301, -2.298324, 1.166007, 2.030530, 11.447561, 5.478323, -2.849412, 0.926995, 7.514958, -1.741774, 2.395123, -2.264060, 0.827710, 3.023765, 0.865038, -0.740627, 0.542816, 0.763681, 0.110778, -0.477939, 5.877367, -1.913517, 0.481319, 0.889800, 5.478323, 13.466522, -6.084424, 1.413296, 3.571389, -1.358915, -3.453291, -3.494921, -7.259057, 4.515083, 5.873295, 4.541603, -0.709580, 4.985410, 5.711729, -2.909551, 2.764385, -0.870778, -0.418904, -2.272904, -2.849412, -6.084424, 7.932370, 0.062342, -0.986165, -0.302069, 5.719676, -1.022603, 6.237248, -1.042181, -5.281445, -4.058232, 1.597578, 1.953354, -4.043843, -0.918105, -0.676395, -0.422216, 0.027714, 0.132110, 0.926995, 1.413296, 0.062342, 6.571410, -5.558164, 3.602791, -0.540197, -1.204134, -5.556916, -0.597247, 2.668005, 0.868707, -1.938237, 7.456696, -0.263159, 0.458086, -3.303527, -2.357705, 0.741818, 0.397549, 7.514958, 3.571389, -0.986165, -5.558164, 13.689521, -6.337509, 2.881618, -3.860971, 7.372320, 2.678612, -2.127048, -2.418516, 2.163446, -3.574460, 1.340811, -2.130271, 8.011308, 0.938830, -0.180179, 0.407549, -1.741774, -1.358915, -0.302069, 3.602791, -6.337509, 5.404850, -0.067602, 1.790067, -5.370499, -1.153168, 1.768645, 3.362591, -3.930499, 1.434755, -2.453217, 1.466307, -3.422457, -1.837171, 0.001625, -0.748567, 2.395123, -3.453291, 5.719676, -0.540197, 2.881618, -0.067602, 9.558564, -2.275149, 5.221573, 0.454839, -4.679147, 0.891604, 1.259203, 0.196374, -7.405356, -1.900871, 2.776430, 2.120478, -0.034905, 0.832796, -2.264060, -3.494921, -1.022603, -1.204134, -3.860971, 1.790067, -2.275149, 7.665882, 1.660977, 2.427917, -1.035471, 2.366294, -1.660759, -6.283232, -1.278402, 4.118252, -2.685258, -1.133075, 0.022485, -1.346801, 0.827710, -7.259057, 6.237248, -5.556916, 7.372320, -5.370499, 5.221573, 1.660977, 20.129067, 3.257656, -12.060984, -6.126243, -1.849122, -9.116582, -3.155671, 0.190472, -2.785986, -1.528519, 0.181634, 0.006707, 3.023765, 4.515083, -1.042181, -0.597247, 2.678612, -1.153168, 0.454839, 2.427917, 3.257656, 9.633234, -3.425929, 2.896541, -0.617621, -3.752398, 0.401386, -0.786864, 0.505166, 1.257752, 0.385736, 1.196807, 0.865038, 5.873295, -5.281445, 2.668005, -2.127048, 1.768645, -4.679147, -1.035471, -12.060984, -3.425929, 15.867632, 8.503889, 2.837069, 6.420461, -0.848130, 2.817718, 5.971204, 0.856214, -0.093634, 1.182328, -0.740627, 4.541603, -4.058232, 0.868707, -2.418516, 3.362591, 0.891604, 2.366294, -6.126243, 2.896541, 8.503889, 19.961110, -4.610489, -1.780350, -6.965708, -1.655826, 0.250855, -0.229157, 0.135515, -0.473380, 0.542816, -0.709580, 1.597578, -1.938237, 2.163446, -3.930499, 1.259203, -1.660759, -1.849122, -0.617621, 2.837069, -4.610489, 16.499588, 2.005925, -1.220136, -0.795627, 12.082456, -1.611274, -0.041764, -1.095547, 0.763681, 4.985410, 1.953354, 7.456696, -3.574460, 1.434755, 0.196374, -6.283232, -9.116582, -3.752398, 6.420461, -1.780350, 2.005925, 29.662930, 3.259767, 2.708967, 2.424918, -0.130335, 0.055648, 0.114069, 0.110778, 5.711729, -4.043843, -0.263159, 1.340811, -2.453217, -7.405356, -1.278402, -3.155671, 0.401386, -0.848130, -6.965708, -1.220136, 3.259767, 13.208225, -1.178260, -4.317798, 1.526603, 0.162822, 0.790382, -0.477939, -2.909551, -0.918105, 0.458086, -2.130271, 1.466307, -1.900871, 4.118252, 0.190472, -0.786864, 2.817718, -1.655826, -0.795627, 2.708967, -1.178260, 13.281741, -0.445635, -1.156691, 0.742783, 0.496301, 5.877367, 2.764385, -0.676395, -3.303527, 8.011308, -3.422457, 2.776430, -2.685258, -2.785986, 0.505166, 5.971204, 0.250855, 12.082456, 2.424918, -4.317798, -0.445635, 25.125294};
  assignment(initsigma, lrn->sigma, NFEATURES_SQ);// assignment  

  lrn->pop_k = (float*)calloc(NCLASSES, sizeof(float));
  for (int i = 0; i < NCLASSES; i++)
    lrn->pop_k[i] = 1.0;

  lrn->pop = 1.0;

  //init intermediary matrices
  lrn->UT = (float*)calloc(NFEATURES_SQ, sizeof(float));
  lrn->LT = (float*)calloc(NFEATURES_SQ, sizeof(float));
  lrn->A = (float*)calloc(NFEATURES_SQ, sizeof(float));
  lrn->x  = (float*)calloc( NFEATURES, sizeof(float));
  lrn->y  = (float*)calloc( NFEATURES, sizeof(float));

}

void init_classifier(struct classifier_s* lda){
  lda->A = (float*)calloc(NCLASSES * NFEATURES, sizeof(float));
  lda->B = (float*)calloc(NCLASSES, sizeof(float));
  lda->score_k = (float*)calloc(NCLASSES, sizeof(float));
  lda->k_pred = 0;
}

// int16_t updateMaxWithTime(float *currentMax, float *currentTMax, float sensorValue){
//   if (sensorValue > *currentMax){
//     *currentMax = sensorValue;
//     *currentTMax = (FLOAT32_T)(tm.loopcount - tm.latestFootOffTime)/3.0;
//   }
//   return 0;
// }

// int16_t updateMinWithTime(float* currentMin, float* currentTMin, float sensorValue){
//   if (sensorValue < *currentMin){
//     *currentMin = sensorValue;
//     *currentTMin = (FLOAT32_T)(tm.loopcount - tm.latestFootOffTime)/3.0;
//   }
//   return 0;
// }

// int16_t updateMax(float* currentMax, float sensorValue){
//   if (sensorValue > *currentMax){
//     *currentMax = sensorValue;
//   }
//   return 0;
// }

// int16_t updateMin(float* currentMin,  float sensorValue){
//   if (sensorValue < *currentMin){
//     *currentMin = sensorValue;
//   }
//   return 0;
// }

// int16_t solveDiscriminantFunction1(void){

//    tm.tempHolder[0] = feats.daAccZ_max - feats.daAccZ_min;

//   tm.lrnFuncs[0] = A1_1*feats.vAz_sum + A1_2*feats.aOmegaX_sum_EXTRA + A1_3*feats.vAz_max + 
//   A1_4*feats.aAccZ_final + A1_5*feats.iaAccZ_min + A1_6*feats.vAy_final + 
//   A1_7*feats.r1_max + A1_8*feats.vAz_first + A1_9*tm.tempHolder[0] + 
//   A1_10*feats.pAy_first + A1_11*feats.aOmegaZ_first +A1_12*feats.aAz_sum +
//   + B1;

//   tm.taskPredicted = tm.taskPredicted + 1;
//   return 0;
// }
// int16_t solveDiscriminantFunction2(void){

//   tm.lrnFuncs[1] = A2_1*feats.vAz_sum + A2_2*feats.aOmegaX_sum_EXTRA + A2_3*feats.vAz_max + 
//   A2_4*feats.aAccZ_final + A2_5*feats.iaAccZ_min + A2_6*feats.vAy_final + 
//   A2_7*feats.r1_max + A2_8*feats.vAz_first + A2_9*tm.tempHolder[0] + 
//   A2_10*feats.pAy_first + A2_11*feats.aOmegaZ_first +A2_12*feats.aAz_sum +
//   + B2;
//   tm.taskPredicted = tm.taskPredicted + 1;
//   return 0;
// }
// int16_t solveDiscriminantFunction3(void){
//   tm.lrnFuncs[2] = A3_1*feats.vAz_sum + A3_2*feats.aOmegaX_sum_EXTRA + A3_3*feats.vAz_max + 
//   A3_4*feats.aAccZ_final + A3_5*feats.iaAccZ_min + A3_6*feats.vAy_final + 
//   A3_7*feats.r1_max + A3_8*feats.vAz_first + A3_9*tm.tempHolder[0] + 
//   A3_10*feats.pAy_first + A3_11*feats.aOmegaZ_first +A3_12*feats.aAz_sum +
//   + B3;
//   tm.taskPredicted = tm.taskPredicted + 1;
//   return 0;
// }
// int16_t solveDiscriminantFunction4(void){

// tm.lrnFuncs[3] = A4_1*feats.vAz_sum + A4_2*feats.aOmegaX_sum_EXTRA + A4_3*feats.vAz_max + 
//   A4_4*feats.aAccZ_final + A4_5*feats.iaAccZ_min + A4_6*feats.vAy_final + 
//   A4_7*feats.r1_max + A4_8*feats.vAz_first + A4_9*tm.tempHolder[0] + 
//   A4_10*feats.pAy_first + A4_11*feats.aOmegaZ_first +A4_12*feats.aAz_sum +
//   + B4;
//   tm.taskPredicted = tm.taskPredicted + 1;
//   return 0;
// }
// int16_t solveDiscriminantFunction5(void){
//   tm.lrnFuncs[4] = A5_1*feats.vAz_sum + A5_2*feats.aOmegaX_sum_EXTRA + A5_3*feats.vAz_max + 
//   A5_4*feats.aAccZ_final + A5_5*feats.iaAccZ_min + A5_6*feats.vAy_final + 
//   A5_7*feats.r1_max + A5_8*feats.vAz_first + A5_9*tm.tempHolder[0] + 
//   A5_10*feats.pAy_first + A5_11*feats.aOmegaZ_first +A5_12*feats.aAz_sum +
//   + B5;

//   tm.taskPredicted = tm.taskPredicted + 1;
//   return 0;
// }

// int16_t makePrediction(void){
//   tm.iter = 0;
//   tm.prediction = 1;
//   tm.tempHolder[0] = tm.lrnFuncs[tm.iter];
//   while (tm.iter < 5){
//     tm.iter = tm.iter + 1;
//     if (tm.lrnFuncs[tm.iter] > tm.tempHolder[0]){
//       tm.tempHolder[0] = tm.lrnFuncs[tm.iter];
//       tm.prediction = tm.iter+1;
//     }
    
//   }
//   tm.taskPredicted = tm.taskPredicted + 1;

//     return 0;
// }

// int16_t solveDiscriminantFunction(void){

//   if (tm.mode < MODE_PR){
//     tm.prediction = tm.mode;
//     tm.taskPredicted = 7;
//     switchTask();
//     return 0;
//   }

//   switch(tm.taskPredicted){
//     case 1:
//       solveDiscriminantFunction1();
//     break;
//     case 2:
//       solveDiscriminantFunction2();
//     break;
//     case 3:
//       solveDiscriminantFunction3();
//     break;
//     case 4:
//       solveDiscriminantFunction4();
//     break;
//     case 5:
//       solveDiscriminantFunction5();
//     break;
//     case 6:
//       makePrediction();
//       if (tm.mode == MODE_SW){
//         switchTask();
//       }
//     break;
//   }

  
//   return 0;
// }

// int16_t switchTask(void){
//   tm.task = tm.prediction;
//   switch (tm.task){
//     case TASK_FL:
//       WS_SET_NEW_STATE(STATE_LATE_SWING, STID_EARLY_SWING_DONE);
//     break;
//     case TASK_UR:
//       WS_SET_NEW_STATE(STATE_LATE_SWING, STID_EARLY_SWING_DONE);
//     break;
//     case TASK_DR:
//       WS_SET_NEW_STATE(STATE_LATE_SWING_DR, STID_EARLY_SWING_DONE);
//     break;
//     case TASK_US:
//       WS_SET_NEW_STATE(STATE_LATE_SWING_US, STID_EARLY_SWING_DONE);
//     break;
//     case TASK_DS:
//       WS_SET_NEW_STATE(STATE_LATE_SWING_DS, STID_SWING_2ES_FOOTSTRIKE);
//     break;
//   }
//   return 0;
// }




//  int16_t updateFeatures(void){


//      if (tm.inSwing){

//           ///if(tm.iter3){
//             updateMax(&feats.daAccZ_max, sigs.daAccZ);
//             updateMin(&feats.iaAccZ_min, sigs.iaAccZ);
//             updateMin(&feats.daAccZ_min, sigs.daAccZ);
//             if (tm.loopcount - tm.latestFootOffTime < BEGINNING_CUTOFF){
//               feats.aOmegaZ_first = feats.aOmegaZ_first + sigs.aOmegaZ;
//             }
//             if (tm.loopcount - tm.latestFootOffTime > ENDING_START){
//               feats.aAccZ_final = feats.aAccZ_final + sigs.aAccZ;
//             }
//          // }

//           //if(tm.iter1){
//             feats.vAz_sum = feats.vAz_sum + sigs.vAz;
//             feats.aAz_sum = feats.aAz_sum + sigs.aAz;
 
//             updateMax(&feats.vAz_max, sigs.vAz);
//             updateMax(&feats.r1_max, sigs.rot[1]);
         
//             if (tm.loopcount - tm.latestFootOffTime < BEGINNING_CUTOFF){
//              feats.pAy_first = feats.pAy_first + sigs.pAy;
//              feats.vAz_first = feats.vAz_first + sigs.vAz;
//             }
         
//            if (tm.loopcount - tm.latestFootOffTime > ENDING_START){
//              feats.vAy_final = feats.vAy_final + sigs.vAy;    
//             }
//           //}
//      }
//     // if (tm.iter3){
//         if (tm.loopcount <= PREDICTION_CUTOFF){
//           //aOmegaX_sum_EXTRA
//           feats.aOmegaX_sum_EXTRA = feats.aOmegaX_sum_EXTRA + sigs.aOmegaX;
//         }
//      // }

//     return 0;
//  }

// int16_t resetFeatures(void){

//     feats.aAz_sum = 0.0f;
//      feats.vAz_sum = 0.0f;
//      feats.aOmegaX_sum_EXTRA = 0.0f;
//      feats.vAz_max = BIG_NEGATIVE_NUMBER;
//      feats.aAccZ_final = 0.0f;
//      feats.iaAccZ_min = BIG_POSITIVE_NUMBER;
//      feats.vAy_final = 0.0f;
//      feats.r1_max = BIG_NEGATIVE_NUMBER;
//      feats.vAz_first = 0.0f;
//      //feats.daAccZ_range
//      feats.daAccZ_max = BIG_NEGATIVE_NUMBER;
//      feats.daAccZ_min = BIG_POSITIVE_NUMBER;
//      feats.pAy_first = 0.0f;
//      feats.aOmegaZ_first = 0.0f;
     

//   return 0;
// }