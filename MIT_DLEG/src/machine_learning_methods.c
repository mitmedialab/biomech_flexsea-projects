


#include "machine_learning_methods.h"
#include "linear_algebra_methods.h"


#define NCLASSES 5
#define NFEATURES 5
#define NFEATURES_SQ NFEATURES * NFEATURES

void update_classifier(struct learner_s* lrn, struct classifier_s* lda){
  int i = 0;
  //for (int i = 0; i < NCLASSES; i++){
      int p = i*NFEATURES;
      upper_to_lower_transpose(lrn->T, NFEATURES);
      cholesky_decomposition(lrn->sigma, lrn->T, NFEATURES);
      forward_substitution(lrn->T,&lrn->mu_k[p], lrn->y, NFEATURES);
      lower_to_upper_transpose(lrn->T, NFEATURES);
      backward_substitution(lrn->T,lrn->y, &lda->A[p], NFEATURES);
      lda->B[i] = -0.5*inner_product(&lrn->mu_k[p], &lda->A[p], NFEATURES); //assumes uniform priors
    //}

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


void update_learner(struct learner_s* lrn){
  int ind = lrn->k_est*NFEATURES;
  float popP1 = lrn->pop + 1.0;
  float popkP1 = lrn->pop_k[lrn->k_est] + 1.0;

  //update class mean
  scaling(&lrn->mu_k[ind], lrn->pop_k[lrn->k_est], &lrn->mu_k[ind], NFEATURES);
  sum(&lrn->mu_k[ind], lrn->feats_prev, &lrn->mu_k[ind], NFEATURES);
  scaling (&lrn->mu_k[ind], 1.0/popkP1, &lrn->mu_k[ind], NFEATURES);

  //update overall mean
  assignment(lrn->mu_prev, lrn->mu, NFEATURES);
  scaling(lrn->mu, lrn->pop, lrn->mu, NFEATURES);
  sum(lrn->mu, lrn->feats_prev, lrn->mu, NFEATURES);
  scaling(lrn->mu, 1.0/popP1, lrn->mu, NFEATURES);

  //update covariance
  diff(lrn->feats_prev, lrn->mu, lrn->x, NFEATURES);
  diff(lrn->feats_prev, lrn->mu_prev, lrn->y, NFEATURES);
  outer_product(lrn->x, lrn->y, lrn->A, NFEATURES);
  scaling(lrn->sigma, lrn->pop, lrn->sigma, NFEATURES_SQ);
  sum(lrn->sigma, lrn->A, lrn->sigma, NFEATURES_SQ);
  scaling(lrn->sigma, 1.0/popP1, lrn->sigma, NFEATURES_SQ);

  //update populations
  lrn->pop = popP1;
  lrn->pop_k[lrn->k_est] = popkP1;

}


void init_learner(struct learner_s* lrn){

  lrn->mu_k = (float*)calloc(NCLASSES * NFEATURES, sizeof(float));
  lrn->mu_prev  = (float*)calloc( NFEATURES, sizeof(float));
  lrn->mu  = (float*)calloc( NFEATURES, sizeof(float));
  lrn->sigma = (float*)calloc(NFEATURES_SQ, sizeof(float));

  lrn->feats_prev = (float*)calloc(NFEATURES, sizeof(float));
  lrn->pop_k = (float*)calloc(NCLASSES, sizeof(float));
  lrn->pop = 0.0;

  lrn->k_est = 0;

  // for (int i = 0; i < NFEATURES; i++){
  //   lrn->sigma[NFEATURES*i + i] = 1.000000;
  // }
  float temp[] = {1.041590, -0.413954, 1.307654, -0.095655, 0.667191, -0.413954, 2.179066, 2.607505, 1.630967, 1.479780, 1.307654, 2.607505, 6.822377, 2.177769, 3.701085, -0.095655, 1.630967, 2.177769, 2.736470, 2.290576, 0.667191, 1.479780, 3.701085, 2.290576, 2.903727};
  for (int i = 0; i < NFEATURES_SQ; i++)
    lrn->sigma[i] = temp[i];
  float temp2[] = {-0.4624,   -0.4098,   -0.5035,    1.2333,    0.6103};
  for (int i = 0; i < NFEATURES; i++)
    lrn->mu_k[i] = temp2[i];
  //init intermediary matrices
  lrn->T = (float*)calloc(NFEATURES_SQ, sizeof(float));
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