


#include "machine_learning_methods.h"
#include "linear_algebra_methods.h"


#define NCLASSES 5
#define NFEATURES 5
#define NFEATURES_SQ NFEATURES * NFEATURES

static int learning_demux_state = 0;
static int current_updating_class = 0;
static int current_updating_feature = 0;
static int current_updating_feature_segment = 0;

void reset_learning_demux(){
  learning_demux_state = 0;
}

int learning_demux(struct learner_s* lrn, struct classifier_s* lda, float* feats, int k_est){
    switch (learning_demux_state){
      case UPDATE_CLASS_MEAN:
        update_class_mean(lrn, feats, k_est);
        learning_demux_state++;
      break;
      case UPDATE_OVERALL_MEAN:
        update_overall_mean(lrn, feats);
        learning_demux_state++;
      break;
      case GET_DEVIATIONS_FROM_MEAN:
        diff(feats, lrn->mu, lrn->x, NFEATURES);//f flops
        diff(feats, lrn->mu_prev, lrn->y, NFEATURES); //f flops
        learning_demux_state++;
      break;
      case UPDATE_COVARIANCE:
      {
        int feature_times_n_classes = current_updating_feature*NCLASSES;
        segmented_outer_product(lrn->x, lrn->y, lrn->A, NFEATURES, current_updating_feature); //f flops
        scaling(&lrn->sigma[feature_times_n_classes], lrn->pop, &lrn->sigma[feature_times_n_classes], NFEATURES);//f flops
        sum(&lrn->sigma[feature_times_n_classes], &lrn->A[feature_times_n_classes], &lrn->sigma[feature_times_n_classes], NFEATURES);//f flops
        float popP1 = lrn->pop + 1.0;
        scaling(&lrn->sigma[feature_times_n_classes], 1.0/popP1, &lrn->sigma[feature_times_n_classes], NFEATURES);//f flops
        current_updating_feature++;
        if (current_updating_feature == NFEATURES){
          learning_demux_state++; 
          current_updating_feature = 0;
          lrn->pop = popP1;
        }
      }
      break;
      // case UPDATE_COVARIANCE_2:
      //   scaling(lrn->sigma, lrn->pop, lrn->sigma, NFEATURES_SQ);//f^2 flops
      //   learning_demux_state++;
      // break;
      // case UPDATE_COVARIANCE_3:
      //   sum(lrn->sigma, lrn->A, lrn->sigma, NFEATURES_SQ);//f^2 flops
      //   learning_demux_state++;
      // break;
      // case UPDATE_COVARIANCE_4:
      // {
      //   float popP1 = lrn->pop + 1.0;
      //   scaling(lrn->sigma, 1.0/popP1, lrn->sigma, NFEATURES_SQ);//f^2 flops
      //   lrn->pop = popP1;
      //   learning_demux_state++;
      // }
      // break;
      case DO_CHOLESKY:

        super_segmented_cholesky_decomposition(lrn->sigma, lrn->LT, NFEATURES, current_updating_feature, current_updating_feature_segment);
        current_updating_feature_segment++;
        if (current_updating_feature_segment > current_updating_feature/2){
          current_updating_feature_segment = 0;
          current_updating_feature++;
        }
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
        forward_substitution(lrn->LT, &lrn->mu_k[class_times_n_features], lrn->y, NFEATURES); // roughly 1/2 f^2 flops
        backward_substitution(lrn->UT,lrn->y, &lda->A[class_times_n_features], NFEATURES); // roughly 1/2 f^2 flops
        lda->B[current_updating_class] = -0.5*inner_product(&lrn->mu_k[class_times_n_features], &lda->A[class_times_n_features], NFEATURES); //assumes uniform priors, 2f flops
        current_updating_class++;
        if (current_updating_class == NCLASSES){
          learning_demux_state++; 
          current_updating_class = 0;
        }
      }
      break;
    }
     return learning_demux_state;
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
  // float initsigma30[] = {0.933211, -0.037899, 0.403101, -0.009280, 0.710313, -1.093367, 0.285333, -0.176940, 1.305828, 1.269830, 1.068338, 0.750198, 1.202950, 0.760942, 0.001974, -1.027405, 1.242565, 0.259037, -0.545692, -1.079321, -0.439852, -0.206149, -0.329064, -0.338105, 0.785823, -0.159407, -0.726775, 0.670830, -1.228498, -1.592446, -0.037899, 0.935418, 0.039710, 0.804725, 0.650243, 1.622967, -0.746701, 0.005299, 0.352086, 2.834556, 0.056953, 0.191490, -0.661275, 0.065711, -0.552340, -0.396395, -0.338159, 0.433302, 0.731465, 2.141424, 1.373900, -0.453694, -0.313694, 0.841101, 0.179822, -0.948754, 0.715650, -0.447654, 0.914978, -0.778986, 0.403101, 0.039710, 0.316401, -0.288373, 0.637193, -0.391499, 0.032783, 0.293213, 1.179224, 0.730153, 0.410512, 1.174541, 0.378466, 0.304888, -0.620556, -0.763140, 0.514652, 0.115461, -0.680223, -0.226507, 0.191992, -0.045715, -0.284229, -0.223727, 0.694361, -0.616540, -0.560349, 0.252606, -0.944255, -0.908501, -0.009280, 0.804725, -0.288373, 3.855514, 1.932648, 1.910788, -2.632702, -2.492881, -2.667201, 0.176145, 0.740336, -1.781297, -2.906061, 2.964695, 1.609114, 1.767733, 0.362054, 0.832915, 0.051878, -0.790830, 3.101854, -1.436896, 1.073092, 1.035646, -0.049995, 0.468893, 2.014174, -0.932327, 4.529131, -1.530850, 0.710313, 0.650243, 0.637193, 1.932648, 5.233920, 1.820840, 1.610252, 0.349269, 1.079235, 1.377281, 2.006635, 1.736017, -3.272222, 2.317601, -0.142033, -2.787087, -1.285062, 2.565201, -2.969851, 0.785104, 4.161296, -2.096080, 0.901067, 1.177828, 1.109283, -2.378757, 1.058285, -0.952667, 0.327892, -1.623621, -1.093367, 1.622967, -0.391499, 1.910788, 1.820840, 6.881083, -1.372376, 2.269817, -1.462160, 4.326853, 3.458352, -0.833080, -3.462795, -1.335073, 2.812424, 2.212046, -1.083371, -0.113913, 0.220290, 5.083218, 1.959949, 0.099708, 2.305838, 3.828728, -1.791050, -2.511795, 0.693411, -1.438877, 1.946431, -1.653336, 0.285333, -0.746701, 0.032783, -2.632702, 1.610252, -1.372376, 13.937283, -0.777318, 1.024387, 0.795538, -2.768247, -2.130997, -0.125416, -3.923884, -1.690807, -9.577640, -8.723212, 4.357943, 1.221577, 6.090652, 0.310977, -1.276515, -0.266991, 2.614208, -0.862991, 1.632066, 1.764271, -2.497936, -5.062092, 6.311364, -0.176940, 0.005299, 0.293213, -2.492881, 0.349269, 2.269817, -0.777318, 11.108569, 2.363485, -0.886539, 6.027095, 1.949297, 3.902502, -1.767571, 2.769159, 0.862148, 0.915655, -1.399812, -0.865921, -0.208198, -4.328670, 0.347812, -2.637761, -2.115424, -2.812825, -5.090385, -1.371656, 5.577009, -3.713335, -0.174994, 1.305828, 0.352086, 1.179224, -2.667201, 1.079235, -1.462160, 1.024387, 2.363485, 12.056165, 3.255221, -0.439221, 7.802050, 5.187362, -0.448173, -2.960961, -4.662864, 5.338517, 2.649381, -2.061462, 0.747351, -5.668409, 1.621928, -6.995871, 2.519095, 3.935780, -3.470430, -1.395028, -0.293048, -0.911097, 0.038566, 1.269830, 2.834556, 0.730153, 0.176145, 1.377281, 4.326853, 0.795538, -0.886539, 3.255221, 16.697571, 5.487660, 0.707009, 1.157789, -1.996011, -1.264076, -4.324678, -1.482319, 0.381322, 2.026056, 7.877297, 1.066154, 1.195461, 1.775924, 1.213577, 1.368446, -2.271272, -1.024162, -2.094864, -2.380107, -5.917772, 1.068338, 0.056953, 0.410512, 0.740336, 2.006635, 3.458352, -2.768247, 6.027095, -0.439221, 5.487660, 24.345797, -1.195688, 8.973126, 3.392596, 6.008456, -1.102324, -3.354354, -1.550811, -2.767669, -6.410352, -7.549006, 6.059812, -0.532165, -8.632255, -2.822228, -2.241352, -1.614163, 5.181047, -3.448599, -6.858576, 0.750198, 0.191490, 1.174541, -1.781297, 1.736017, -0.833080, -2.130997, 1.949297, 7.802050, 0.707009, -1.195688, 8.155100, 1.555520, 1.073004, -3.273181, -0.993947, 3.194964, 0.260882, -2.318892, 0.388625, -0.448385, 1.237721, -3.069971, 0.703649, 4.833828, -3.832125, -3.636953, -0.902604, -0.596106, -1.538131, 1.202950, -0.661275, 0.378466, -2.906061, -3.272222, -3.462795, -0.125416, 3.902502, 5.187362, 1.157789, 8.973126, 1.555520, 13.934383, 1.637248, 0.351278, -3.461190, -0.393310, -2.013061, 3.087537, -5.132189, -10.295583, 4.668657, -8.739801, -6.603686, 0.505831, 0.284009, -1.932298, 4.291171, -3.286072, -0.887929, 0.760942, 0.065711, 0.304888, 2.964695, 2.317601, -1.335073, -3.923884, -1.767571, -0.448173, -1.996011, 3.392596, 1.073004, 1.637248, 7.897413, 1.602830, -0.046649, -0.295239, -0.033840, -0.162795, -6.079453, 1.422328, -0.338903, -1.791078, -5.469856, 3.322737, -0.661313, 0.853636, -0.417465, 5.324160, -1.072181, 0.001974, -0.552340, -0.620556, 1.609114, -0.142033, 2.812424, -1.690807, 2.769159, -2.960961, -1.264076, 6.008456, -3.273181, 0.351278, 1.602830, 14.815917, 3.537457, 5.583745, 0.574875, -1.901358, -5.638082, -3.840837, 1.137717, 2.256215, 2.117674, -0.855053, -1.765078, 1.048170, 1.857219, 3.644407, 0.329521, -1.027405, -0.396395, -0.763140, 1.767733, -2.787087, 2.212046, -9.577640, 0.862148, -4.662864, -4.324678, -1.102324, -0.993947, -3.461190, -0.046649, 3.537457, 16.557751, 8.263248, -6.662680, -0.524344, -4.749852, 2.175076, -3.492223, 6.216614, 1.531975, -0.789400, -0.396047, -3.883210, -2.774006, 5.244588, -3.809591, 1.242565, -0.338159, 0.514652, 0.362054, -1.285062, -1.083371, -8.723212, 0.915655, 5.338517, -1.482319, -3.354354, 3.194964, -0.393310, -0.295239, 5.583745, 8.263248, 25.299060, -0.218043, -7.177831, -10.822953, -3.161309, -2.752783, -1.096754, 8.258536, 1.683735, -3.255970, 0.839397, 4.718071, 0.836084, -5.547877, 0.259037, 0.433302, 0.115461, 0.832915, 2.565201, -0.113913, 4.357943, -1.399812, 2.649381, 0.381322, -1.550811, 0.260882, -2.013061, -0.033840, 0.574875, -6.662680, -0.218043, 13.745243, 0.341026, 2.414456, 0.262632, 0.731058, -0.707541, 4.112601, 1.002025, -3.495539, 8.720168, 1.433543, 1.334720, 10.861502, -0.545692, 0.731465, -0.680223, 0.051878, -2.969851, 0.220290, 1.221577, -0.865921, -2.061462, 2.026056, -2.767669, -2.318892, 3.087537, -0.162795, -1.901358, -0.524344, -7.177831, 0.341026, 16.989859, 5.710282, -2.215216, 2.608854, 2.469988, -5.141617, 1.541975, 4.598796, 2.240662, -4.894127, 1.858459, 5.337979, -1.079321, 2.141424, -0.226507, -0.790830, 0.785104, 5.083218, 6.090652, -0.208198, 0.747351, 7.877297, -6.410352, 0.388625, -5.132189, -6.079453, -5.638082, -4.749852, -10.822953, 2.414456, 5.710282, 21.220696, 1.246326, 0.313167, 1.837370, 7.400631, 0.283515, -2.219759, -0.791522, -1.859740, -4.119265, 5.738829, -0.439852, 1.373900, 0.191992, 3.101854, 4.161296, 1.959949, 0.310977, -4.328670, -5.668409, 1.066154, -7.549006, -0.448385, -10.295583, 1.422328, -3.840837, 2.175076, -3.161309, 0.262632, -2.215216, 1.246326, 20.587203, -6.392191, 5.759304, 0.579055, 4.333643, -3.854224, -1.419898, -6.279092, 5.777146, -0.698777, -0.206149, -0.453694, -0.045715, -1.436896, -2.096080, 0.099708, -1.276515, 0.347812, 1.621928, 1.195461, 6.059812, 1.237721, 4.668657, -0.338903, 1.137717, -3.492223, -2.752783, 0.731058, 2.608854, 0.313167, -6.392191, 16.604534, 0.209736, -0.588990, -3.847249, 1.601751, -5.414371, 3.506407, -1.123284, -0.234810, -0.329064, -0.313694, -0.284229, 1.073092, 0.901067, 2.305838, -0.266991, -2.637761, -6.995871, 1.775924, -0.532165, -3.069971, -8.739801, -1.791078, 2.256215, 6.216614, -1.096754, -0.707541, 2.469988, 1.837370, 5.759304, 0.209736, 19.413019, -0.864160, -2.295316, 0.803463, -1.364840, -3.708288, -1.782752, -3.540391, -0.338105, 0.841101, -0.223727, 1.035646, 1.177828, 3.828728, 2.614208, -2.115424, 2.519095, 1.213577, -8.632255, 0.703649, -6.603686, -5.469856, 2.117674, 1.531975, 8.258536, 4.112601, -5.141617, 7.400631, 0.579055, -0.588990, -0.864160, 27.207871, -1.937330, 1.445220, 1.249854, 3.332352, -0.172917, -2.007560, 0.785823, 0.179822, 0.694361, -0.049995, 1.109283, -1.791050, -0.862991, -2.812825, 3.935780, 1.368446, -2.822228, 4.833828, 0.505831, 3.322737, -0.855053, -0.789400, 1.683735, 1.002025, 1.541975, 0.283515, 4.333643, -3.847249, -2.295316, -1.937330, 23.380482, -7.409970, -5.184805, -4.793792, 2.712584, 0.955089, -0.159407, -0.948754, -0.616540, 0.468893, -2.378757, -2.511795, 1.632066, -5.090385, -3.470430, -2.271272, -2.241352, -3.832125, 0.284009, -0.661313, -1.765078, -0.396047, -3.255970, -3.495539, 4.598796, -2.219759, -3.854224, 1.601751, 0.803463, 1.445220, -7.409970, 18.103869, 3.161618, -1.543235, 4.182339, -5.397370, -0.726775, 0.715650, -0.560349, 2.014174, 1.058285, 0.693411, 1.764271, -1.371656, -1.395028, -1.024162, -1.614163, -3.636953, -1.932298, 0.853636, 1.048170, -3.883210, 0.839397, 8.720168, 2.240662, -0.791522, -1.419898, -5.414371, -1.364840, 1.249854, -5.184805, 3.161618, 20.128878, 1.319829, 2.686345, 5.944987, 0.670830, -0.447654, 0.252606, -0.932327, -0.952667, -1.438877, -2.497936, 5.577009, -0.293048, -2.094864, 5.181047, -0.902604, 4.291171, -0.417465, 1.857219, -2.774006, 4.718071, 1.433543, -4.894127, -1.859740, -6.279092, 3.506407, -3.708288, 3.332352, -4.793792, -1.543235, 1.319829, 19.714392, -9.251080, -2.371648, -1.228498, 0.914978, -0.944255, 4.529131, 0.327892, 1.946431, -5.062092, -3.713335, -0.911097, -2.380107, -3.448599, -0.596106, -3.286072, 5.324160, 3.644407, 5.244588, 0.836084, 1.334720, 1.858459, -4.119265, 5.777146, -1.123284, -1.782752, -0.172917, 2.712584, 4.182339, 2.686345, -9.251080, 30.173016, 6.997800, -1.592446, -0.778986, -0.908501, -1.530850, -1.623621, -1.653336, 6.311364, -0.174994, 0.038566, -5.917772, -6.858576, -1.538131, -0.887929, -1.072181, 0.329521, -3.809591, -5.547877, 10.861502, 5.337979, 5.738829, -0.698777, -0.234810, -3.540391, -2.007560, 0.955089, -5.397370, 5.944987, -2.371648, 6.997800, 39.098633};
  float initsigma5[] = {0.086442, 0.344703, -0.474981, 0.287554, -0.596325, 0.344703, 1.524754, -2.543949, 0.714580, -2.163041, -0.474981, -2.543949, 5.761625, 0.530639, 2.062213, 0.287554, 0.714580, 0.530639, 3.172750, -2.281614, -0.596325, -2.163041, 2.062213, -2.281614, 5.024490};
  assignment(initsigma5, lrn->sigma, NFEATURES_SQ);// assignment  

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