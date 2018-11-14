


#include "machine_learning_methods.h"
#include "linear_algebra_methods.h"


#define NCLASSES 5
#define NFEATURES 30
#define NFEATURES_SQ NFEATURES * NFEATURES

static int learning_demux_state = 0;
static int current_updating_class = 0;
static int segment = 0;
static int subsegment = 0;
static int doing_forward_substitution = 1;

void reset_learning_demux(){
  learning_demux_state = 0;
  current_updating_class = 0;
  segment = 0;
  subsegment = 0;
  doing_forward_substitution = 1;
}

int learning_demux(struct learner_s* lrn, struct classifier_s* lda, float* feats, int k_est){
    switch (learning_demux_state){
      case UPDATE_CLASS_MEAN: //1 sample
        update_class_mean(lrn, feats, k_est); //3f flops
        learning_demux_state++;
      break;
      case UPDATE_OVERALL_MEAN: // 1 sample
        update_overall_mean(lrn, feats); //3f flops
        learning_demux_state++;
      break;
      case GET_DEVIATIONS_FROM_MEAN: // 1 sample
        diff(feats, lrn->mu, lrn->x, NFEATURES);//f flops
        diff(feats, lrn->mu_prev, lrn->y, NFEATURES); //f flops
        learning_demux_state++;
      break;
      case UPDATE_COVARIANCE: // f samples
      {
        int segment_section = segment*NFEATURES;
        segmented_outer_product(lrn->x, lrn->y, lrn->A, NFEATURES, segment); //f flops
        scaling(&lrn->sigma[segment_section], lrn->pop, &lrn->sigma[segment_section], NFEATURES);//f flops
        sum(&lrn->sigma[segment_section], &lrn->A[segment_section], &lrn->sigma[segment_section], NFEATURES);//f flops
        float popP1 = lrn->pop + 1.0;
        scaling(&lrn->sigma[segment_section], 1.0/popP1, &lrn->sigma[segment_section], NFEATURES);//f flops
        segment++;
        if (segment == NFEATURES){
          learning_demux_state++; 
          segment = 0;
          subsegment = 0;
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
      case DO_CHOLESKY: // f(f+1)/2 samples

        super_segmented_cholesky_decomposition_v2(lrn->sigma, lrn->LT, NFEATURES, segment, subsegment);
        subsegment++;
        if (subsegment > segment){
          subsegment = 0;
          segment++;
        }

        if (segment == NFEATURES){
          learning_demux_state++; 
          segment = 0;
        }
      break;
      case UPDATE_TRANSPOSE: //f samples
        segmented_lower_to_upper_transpose(lrn->LT, lrn->UT, NFEATURES, segment);
        segment++;
        if (segment == NFEATURES){
          learning_demux_state++;
          segment = 1;
        }
        // lower_to_upper_transpose(lrn->LT, lrn->UT, NFEATURES); // assignment
        // learning_demux_state++; 
      break;
      case UPDATE_LDA_PARAMS: //2*c*f
      {
        int class_times_n_features = current_updating_class*NFEATURES;
        if (doing_forward_substitution){
          segmented_forward_substitution(lrn->LT, &lrn->mu_k[class_times_n_features], lrn->y, NFEATURES, segment); // roughly 1/2 f^2 flops
          segment++;
          if (segment == NFEATURES){
            segment = NFEATURES-2;
            doing_forward_substitution = 0;
          }
        }
        else{
          segmented_backward_substitution(lrn->UT, lrn->y, &lda->A[class_times_n_features], NFEATURES, segment); // roughly 1/2 f^2 flops
          segment--;
          if (segment == -1){
            lda->B[current_updating_class] = -0.5*inner_product(&lrn->mu_k[class_times_n_features], &lda->A[class_times_n_features], NFEATURES);
            segment = 1;
            doing_forward_substitution = 1;
            current_updating_class++;
          }
        }
        if (current_updating_class == NCLASSES){
          learning_demux_state++; 
          current_updating_class = 0;
        }  
          
        // forward_substitution(lrn->LT, &lrn->mu_k[class_times_n_features], lrn->y, NFEATURES); // roughly 1/2 f^2 flops
        // backward_substitution(lrn->UT,lrn->y, &lda->A[class_times_n_features], NFEATURES); // roughly 1/2 f^2 flops
          //assumes uniform priors, 2f flops
        
      }
      //break;
      // case DO_BACKWARD_SUBSTITUTION:
      // {
      //   int class_times_n_features = current_updating_class*NFEATURES;

      //   segmented_backward_substitution(lrn->UT, lrn->y, &lda->A[class_times_n_features], NFEATURES, segment); // roughly 1/2 f^2 flops
      //   segment--;
      //   if (segment == -1){
      //     lda->B[current_updating_class] = -0.5*inner_product(&lrn->mu_k[class_times_n_features], &lda->A[class_times_n_features], NFEATURES);
      //     segment = NFEATURES-2;
      //     current_updating_class++;
      //   }
      //   if (current_updating_class == NCLASSES){
      //     learning_demux_state++; 
      //     current_updating_class = 0;
      //     segment = 0;
      //   } 
      // }
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
    
    float initsigma30[] = {0.590774, -0.153797, 1.030787, 0.038082, 0.021039, -0.121038, -0.617719, 1.450598, 0.669919, 1.173036, -1.218256, 0.182412, -0.046809, 0.048802, 0.441782, 0.309525, 1.016295, 0.668281, 0.219930, 1.345224, -1.142219, 0.216966, -0.185015, -1.359959, -0.286515, -0.167590, -0.966672, -0.102913, 0.699077, 0.486444, -0.153797, 0.265544, -0.258623, -0.484134, 0.098601, -0.416271, -0.068436, 0.064492, 0.521248, 0.734601, -0.045516, -0.172847, 0.668627, 0.461660, -0.107036, -0.594180, 0.295226, -0.709320, -0.045359, -0.886683, 0.749776, -0.207971, 0.400918, 0.155866, 0.208598, 0.922511, 0.688192, -0.593245, 1.251386, -0.617594, 1.030787, -0.258623, 1.821981, -0.059157, 0.238786, -0.284512, -1.446696, 2.522764, 0.866462, 1.925906, -2.161450, 0.098118, -0.307627, 0.160298, 0.396669, 0.191215, 1.999001, 1.523166, 0.217908, 2.723265, -1.959989, 0.338978, -0.127796, -2.347187, -0.053269, -0.168565, -1.733890, -0.325597, 1.291139, 0.698782, 0.038082, -0.484134, -0.059157, 1.893581, -1.120535, 1.567125, 2.634152, -1.460981, -0.037429, -1.371350, 0.133394, 1.288985, -0.442384, -0.833803, 3.242134, 2.322682, -2.021245, -0.855228, 0.899809, -1.038332, -2.351920, 0.673641, -0.885452, 0.711405, -2.726914, -2.595299, -1.964694, 1.625458, -3.256507, 0.281158, 0.021039, 0.098601, 0.238786, -1.120535, 3.885923, 2.793250, -3.561203, -0.135582, -3.923767, -2.712066, -0.912194, -0.955377, 0.389765, 0.649454, -3.519583, -3.610500, 0.511506, 2.774684, -2.876695, 3.323709, -0.003814, 3.858025, 1.568026, 0.276240, 4.026521, -0.665378, -0.095583, -0.778666, -0.552225, 0.555306, -0.121038, -0.416271, -0.284512, 1.567125, 2.793250, 8.089243, 1.120645, -0.885015, -3.057786, -5.985482, -1.162518, 1.234148, 5.100504, -1.730009, 0.495666, -0.474916, -4.705636, -0.245605, -2.192000, -1.302635, -2.705849, 8.006212, -1.372901, 1.871484, -1.727989, -5.239119, 0.381906, 1.348799, -5.238391, 5.545726, -0.617719, -0.068436, -1.446696, 2.634152, -3.561203, 1.120645, 8.796676, -2.832009, 4.273103, 2.297084, 0.953847, 3.246803, 0.867050, 0.388348, 6.319293, 2.746887, -5.187293, -5.297908, 1.498225, -8.594827, -2.141460, 1.254067, -2.993268, 4.489572, -3.643741, -1.337798, -0.552899, 4.167983, -0.969221, 0.487169, 1.450598, 0.064492, 2.522764, -1.460981, -0.135582, -0.885015, -2.832009, 8.838864, 1.613588, 4.636592, -3.006970, -1.421100, 2.815012, 0.003135, -1.481849, 2.073809, 5.136294, 0.523349, -0.240128, 0.414612, -2.226448, -0.046453, -2.610683, -4.786857, -2.991076, -1.267956, 2.117043, -2.182683, 4.609268, 6.674439, 0.669919, 0.521248, 0.866462, -0.037429, -3.923767, -3.057786, 4.273103, 1.613588, 11.961108, 7.835328, -2.470431, 3.251407, 5.950839, -0.682314, 3.248109, -0.191556, -2.124880, -5.077282, 3.632879, -5.188781, 3.522068, -1.112046, -2.108962, -0.608675, -2.136946, 7.092471, 2.222205, 0.246504, 6.817277, -0.885017, 1.173036, 0.734601, 1.925906, -1.371350, -2.712066, -5.985482, 2.297084, 4.636592, 7.835328, 12.945575, -3.207000, 1.488388, -1.734118, 3.841143, 2.895682, 0.554186, 5.391354, -3.278144, 0.985109, -4.295725, -1.005832, -2.273918, -1.138060, -1.840170, -0.204912, 4.038062, -1.622167, 0.417074, 10.195999, -3.097010, -1.218256, -0.045516, -2.161450, 0.133394, -0.912194, -1.162518, 0.953847, -3.006970, -2.470431, -3.207000, 5.969855, 0.006909, -2.841549, 0.465214, -1.367296, 3.117495, -1.049456, -1.166655, -1.820738, -2.205602, 2.603787, -2.459299, 0.863448, -1.336293, -1.769309, -2.095570, 1.793919, 3.207054, -1.721917, -0.277071, 0.182412, -0.172847, 0.098118, 1.288985, -0.955377, 1.234148, 3.246803, -1.421100, 3.251407, 1.488388, 0.006909, 6.100592, 3.617933, 1.150775, 3.624035, 3.803273, -3.410203, -5.476373, -0.272096, -3.255091, 2.296941, 4.087973, -2.051716, -0.703639, -4.726806, -0.443694, -2.008869, 1.886582, -4.946864, -3.063498, -0.046809, 0.668627, -0.307627, -0.442384, 0.389765, 5.100504, 0.867050, 2.815012, 5.950839, -1.734118, -2.841549, 3.617933, 20.379107, 2.434735, 2.855863, 0.564502, -6.810625, -6.241590, 3.752346, -2.814766, 6.147716, 4.136816, -3.264425, 2.195718, -6.833807, 4.436772, 6.928205, -6.611457, -3.870311, 3.223800, 0.048802, 0.461660, 0.160298, -0.833803, 0.649454, -1.730009, 0.388348, 0.003135, -0.682314, 3.841143, 0.465214, 1.150775, 2.434735, 15.846125, 3.468809, 0.240541, 3.754609, -0.182653, -2.723058, -0.366652, -0.696262, -2.920392, -0.907866, 4.303715, -0.728793, 1.199641, -3.710840, 0.567926, 0.739936, -6.769017, 0.441782, -0.107036, 0.396669, 3.242134, -3.519583, 0.495666, 6.319293, -1.481849, 3.248109, 2.895682, -1.367296, 3.624035, 2.855863, 3.468809, 25.808483, 7.401890, -2.325913, -8.530802, 6.658328, -0.188961, -9.001067, -8.727689, 0.136832, -0.574450, -12.183630, 1.077796, -8.287203, -2.416915, -4.704545, -6.420476, 0.309525, -0.594180, 0.191215, 2.322682, -3.610500, -0.474916, 2.746887, 2.073809, -0.191556, 0.554186, 3.117495, 3.803273, 0.564502, 0.240541, 7.401890, 25.904678, 2.089230, -3.136765, 9.787265, -6.315669, -0.991251, -4.710888, -5.838464, -3.075349, -20.208669, -12.260713, 1.537912, -1.157063, -10.644327, -1.253045, 1.016295, 0.295226, 1.999001, -2.021245, 0.511506, -4.705636, -5.187293, 5.136294, -2.124880, 5.391354, -1.049456, -3.410203, -6.810625, 3.754609, -2.325913, 2.089230, 13.831972, 2.643968, -2.040972, 0.240913, -2.176435, -3.741070, 2.021264, -7.797093, -3.248252, -4.010944, -2.586404, -1.851788, 10.172951, -1.932266, 0.668281, -0.709320, 1.523166, -0.855228, 2.774684, -0.245605, -5.297908, 0.523349, -5.077282, -3.278144, -1.166655, -5.476373, -6.241590, -0.182653, -8.530802, -3.136765, 2.643968, 16.031204, 1.944240, 10.570243, -1.016823, -4.572853, -1.214610, 4.841246, 11.257319, 1.620871, 1.449134, 0.027588, 1.844299, 5.227007, 0.219930, -0.045359, 0.217908, 0.899809, -2.876695, -2.192000, 1.498225, -0.240128, 3.632879, 0.985109, -1.820738, -0.272096, 3.752346, -2.723058, 6.658328, 9.787265, -2.040972, 1.944240, 23.211029, 8.002242, 4.729801, -7.631106, -3.690541, 4.094680, -4.626497, 0.113156, 4.612936, -12.413662, -3.830127, -0.907555, 1.345224, -0.886683, 2.723265, -1.038332, 3.323709, -1.302635, -8.594827, 0.414612, -5.188781, -4.295725, -2.205602, -3.255091, -2.814766, -0.366652, -0.188961, -6.315669, 0.240913, 10.570243, 8.002242, 28.752936, -4.490266, -11.486351, 7.065313, -0.723301, 11.234835, 8.397350, -3.346188, -4.756972, -3.191625, -0.515241, -1.142219, 0.749776, -1.959989, -2.351920, -0.003814, -2.705849, -2.141460, -2.226448, 3.522068, -1.005832, 2.603787, 2.296941, 6.147716, -0.696262, -9.001067, -0.991251, -2.176435, -1.016823, 4.729801, -4.490266, 26.927122, 6.923311, -7.116197, 0.916093, 0.554172, 2.146730, 7.334004, -8.624444, -1.362551, -1.430975, 0.216966, -0.207971, 0.338978, 0.673641, 3.858025, 8.006212, 1.254067, -0.046453, -1.112046, -2.273918, -2.459299, 4.087973, 4.136816, -2.920392, -8.727689, -4.710888, -3.741070, -4.572853, -7.631106, -11.486351, 6.923311, 34.817488, -8.897629, -2.563770, -1.938004, -10.896016, -3.124701, -1.690648, -3.441674, 4.142254, -0.185015, 0.400918, -0.127796, -0.885452, 1.568026, -1.372901, -2.993268, -2.610683, -2.108962, -1.138060, 0.863448, -2.051716, -3.264425, -0.907866, 0.136832, -5.838464, 2.021264, -1.214610, -3.690541, 7.065313, -7.116197, -8.897629, 26.471543, -8.530060, 5.688081, 5.879509, 0.070821, -1.424059, 9.850032, -7.713366, -1.359959, 0.155866, -2.347187, 0.711405, 0.276240, 1.871484, 4.489572, -4.786857, -0.608675, -1.840170, -1.336293, -0.703639, 2.195718, 4.303715, -0.574450, -3.075349, -7.797093, 4.841246, 4.094680, -0.723301, 0.916093, -2.563770, -8.530060, 24.011845, 10.118346, 7.818422, 3.065693, -0.294059, -9.972593, -2.249177, -0.286515, 0.208598, -0.053269, -2.726914, 4.026521, -1.727989, -3.643741, -2.991076, -2.136946, -0.204912, -1.769309, -4.726806, -6.833807, -0.728793, -12.183630, -20.208669, -3.248252, 11.257319, -4.626497, 11.234835, 0.554172, -1.938004, 5.688081, 10.118346, 41.787314, 10.982451, 2.713730, 3.206308, 6.623714, 2.137738, -0.167590, 0.922511, -0.168565, -2.595299, -0.665378, -5.239119, -1.337798, -1.267956, 7.092471, 4.038062, -2.095570, -0.443694, 4.436772, 1.199641, 1.077796, -12.260713, -4.010944, 1.620871, 0.113156, 8.397350, 2.146730, -10.896016, 5.879509, 7.818422, 10.982451, 27.385117, 4.249657, -1.743646, 4.422776, -7.302642, -0.966672, 0.688192, -1.733890, -1.964694, -0.095583, 0.381906, -0.552899, 2.117043, 2.222205, -1.622167, 1.793919, -2.008869, 6.928205, -3.710840, -8.287203, 1.537912, -2.586404, 1.449134, 4.612936, -3.346188, 7.334004, -3.124701, 0.070821, 3.065693, 2.713730, 4.249657, 21.825116, -1.633559, 2.718242, 1.718973, -0.102913, -0.593245, -0.325597, 1.625458, -0.778666, 1.348799, 4.167983, -2.182683, 0.246504, 0.417074, 3.207054, 1.886582, -6.611457, 0.567926, -2.416915, -1.157063, -1.851788, 0.027588, -12.413662, -4.756972, -8.624444, -1.690648, -1.424059, -0.294059, 3.206308, -1.743646, -1.633559, 22.650252, -1.286750, 0.903495, 0.699077, 1.251386, 1.291139, -3.256507, -0.552225, -5.238391, -0.969221, 4.609268, 6.817277, 10.195999, -1.721917, -4.946864, -3.870311, 0.739936, -4.704545, -10.644327, 10.172951, 1.844299, -3.830127, -3.191625, -1.362551, -3.441674, 9.850032, -9.972593, 6.623714, 4.422776, 2.718242, -1.286750, 34.503014, 7.605958, 0.486444, -0.617594, 0.698782, 0.281158, 0.555306, 5.545726, 0.487169, 6.674439, -0.885017, -3.097010, -0.277071, -3.063498, 3.223800, -6.769017, -6.420476, -1.253045, -1.932266, 5.227007, -0.907555, -0.515241, -1.430975, 4.142254, -7.713366, -2.249177, 2.137738, -7.302642, 1.718973, 0.903495, 7.605958, 29.657958};
    //float initsigma5[] = {0.086442, 0.344703, -0.474981, 0.287554, -0.596325, 0.344703, 1.524754, -2.543949, 0.714580, -2.163041, -0.474981, -2.543949, 5.761625, 0.530639, 2.062213, 0.287554, 0.714580, 0.530639, 3.172750, -2.281614, -0.596325, -2.163041, 2.062213, -2.281614, 5.024490};
  //float initsigma6[] = {0.131652, -0.190341, -0.284577, -0.853353, 0.601576, 0.482470, -0.190341, 1.465595, -0.734748, 1.783851, -1.077013, -0.600548, -0.284577, -0.734748, 2.010487, 0.667719, -1.388735, -0.726558, -0.853353, 1.783851, 0.667719, 9.372914, -3.867380, -4.693657, 0.601576, -1.077013, -1.388735, -3.867380, 4.929711, 2.097891, 0.482470, -0.600548, -0.726558, -4.693657, 2.097891, 4.058166};
  //float initsigma15[] = {0.196066, 0.079430, -0.758377, 0.656680, 0.770641, 0.428145, 0.193270, -0.776419, -0.154880, 0.075729, 0.220717, -0.174575, -0.096767, -0.159619, -0.046818, 0.079430, 0.154791, -0.420396, -0.023832, 0.435894, -0.031896, -0.689586, -0.384718, 0.400637, 0.213753, 0.146029, -0.262122, -0.160106, -0.171987, 0.001994, -0.758377, -0.420396, 3.114329, -2.181102, -2.996337, -1.647573, 0.568155, 2.526010, 0.363029, -0.648678, -0.956576, 0.920620, 0.322865, 0.688694, 0.470429, 0.656680, -0.023832, -2.181102, 3.090070, 2.390293, 2.032239, 3.014551, -3.403248, -1.669789, -0.289851, 0.615147, -0.536467, -0.410917, -0.700376, 0.232341, 0.770641, 0.435894, -2.996337, 2.390293, 3.717464, 1.620636, 0.943189, -2.878161, 0.708094, 0.019105, 0.350580, -0.911510, -0.972354, -0.558295, 0.886847, 0.428145, -0.031896, -1.647573, 2.032239, 1.620636, 5.359001, -0.921663, 0.193887, -1.323995, -1.781851, 2.331075, -1.762497, 2.248648, -0.411374, -1.827909, 0.193270, -0.689586, 0.568155, 3.014551, 0.943189, -0.921663, 12.694006, -1.154641, 2.743840, -1.457644, -1.095540, 1.955958, -3.438040, 0.946537, -0.044746, -0.776419, -0.384718, 2.526010, -3.403248, -2.878161, 0.193887, -1.154641, 15.052792, 5.991208, 0.806134, -0.309093, 2.220633, 0.974399, 1.966281, -5.783225, -0.154880, 0.400637, 0.363029, -1.669789, 0.708094, -1.323995, 2.743840, 5.991208, 11.916041, 1.280392, 1.084915, 0.862289, -4.414141, 1.643759, -5.645044, 0.075729, 0.213753, -0.648678, -0.289851, 0.019105, -1.781851, -1.457644, 0.806134, 1.280392, 6.394678, -0.959465, -0.148677, -3.951682, -3.094364, -1.666002, 0.220717, 0.146029, -0.956576, 0.615147, 0.350580, 2.331075, -1.095540, -0.309093, 1.084915, -0.959465, 3.533536, -0.158226, 1.712550, 0.012030, -3.940953, -0.174575, -0.262122, 0.920620, -0.536467, -0.911510, -1.762497, 1.955958, 2.220633, 0.862289, -0.148677, -0.158226, 6.770384, 2.392396, 2.192636, -2.329586, -0.096767, -0.160106, 0.322865, -0.410917, -0.972354, 2.248648, -3.438040, 0.974399, -4.414141, -3.951682, 1.712550, 2.392396, 10.095829, 1.943498, -0.058342, -0.159619, -0.171987, 0.688694, -0.700376, -0.558295, -0.411374, 0.946537, 1.966281, 1.643759, -3.094364, 0.012030, 2.192636, 1.943498, 5.509617, -0.507750, -0.046818, 0.001994, 0.470429, 0.232341, 0.886847, -1.827909, -0.044746, -5.783225, -5.645044, -1.666002, -3.940953, -2.329586, -0.058342, -0.507750, 11.609971};
  //float initsigma20[] = {0.325480, -0.900402, -0.016734, 0.499174, 0.176019, -0.719629, -0.272897, -0.413333, 0.483828, 0.000929, -1.032918, 0.496668, 0.115517, 0.179518, 0.162282, 0.361593, 0.620503, -0.756717, 0.352154, -0.604502, -0.900402, 2.608819, 0.073511, -1.936114, -0.370636, 2.227967, 0.248204, 1.213302, -1.011898, -0.408747, 2.568849, -1.828794, -0.171574, -0.703050, -0.746965, -1.190294, -2.109103, 2.274858, -1.292730, 2.455141, -0.016734, 0.073511, 0.918284, -0.760432, 0.170401, 0.847315, -0.012066, 1.011702, -0.271668, -0.563574, 1.253652, 0.973365, 0.697654, 0.010148, -0.090051, 1.905203, 0.137880, -0.753184, 0.241265, 0.254145, 0.499174, -1.936114, -0.760432, 5.636076, -1.652139, -2.307923, 1.869759, 0.470106, -2.111872, 1.874178, -2.581091, 4.453869, -2.272944, 3.126108, 3.087304, 0.219283, 3.167369, -1.407853, 2.655727, -5.595427, 0.176019, -0.370636, 0.170401, -1.652139, 1.305183, -0.372332, -0.155663, -1.533283, 1.268305, -0.264122, 0.159411, -1.209456, 0.507884, -0.578278, -0.967346, 0.622021, -0.287015, 0.036723, -0.066159, 1.743113, -0.719629, 2.227967, 0.847315, -2.307923, -0.372332, 5.583549, -0.130831, 0.994492, -1.350560, 0.984355, 2.367493, 0.524498, 0.497082, 1.832780, 0.325474, 2.018785, -1.989217, 2.293275, -0.321809, 3.810892, -0.272897, 0.248204, -0.012066, 1.869759, -0.155663, -0.130831, 6.488305, -1.681635, -2.207982, 5.100583, 4.065406, 1.984963, -5.800360, 4.648804, 0.821223, 3.528921, 1.787051, -2.434337, 1.694145, -0.286844, -0.413333, 1.213302, 1.011702, 0.470106, -1.533283, 0.994492, -1.681635, 5.314849, -2.338951, -3.786552, -0.175366, 2.288411, 1.023700, -0.629530, 0.983996, 0.010586, -0.366034, 1.311230, 0.222495, -1.323001, 0.483828, -1.011898, -0.271668, -2.111872, 1.268305, -1.350560, -2.207982, -2.338951, 4.870410, -0.282469, 0.141216, -3.128225, 3.245529, -2.627502, -3.396871, -1.942714, 0.630643, -3.300554, -1.943509, 3.559141, 0.000929, -0.408747, -0.563574, 1.874178, -0.264122, 0.984355, 5.100583, -3.786552, -0.282469, 8.422316, 2.291562, 0.210019, -4.407759, 3.377958, 0.909015, 0.619413, 1.375212, -5.775289, 1.008396, 0.131749, -1.032918, 2.568849, 1.253652, -2.581091, 0.159411, 2.367493, 4.065406, -0.175366, 0.141216, 2.291562, 14.986868, -1.541772, 0.600122, -0.806166, -5.363075, 0.398316, -0.834671, -2.182918, -0.187757, 2.534200, 0.496668, -1.828794, 0.973365, 4.453869, -1.209456, 0.524498, 1.984963, 2.288411, -3.128225, 0.210019, -1.541772, 9.570947, 0.622355, 7.478968, 4.782840, 6.686599, 5.042767, -0.697239, 4.067013, -2.928196, 0.115517, -0.171574, 0.697654, -2.272944, 0.507884, 0.497082, -5.800360, 1.023700, 3.245529, -4.407759, 0.600122, 0.622355, 13.624505, -4.745476, -0.473375, -1.686293, 1.942663, 0.751110, -2.177375, -0.432895, 0.179518, -0.703050, 0.010148, 3.126108, -0.578278, 1.832780, 4.648804, -0.629530, -2.627502, 3.377958, -0.806166, 7.478968, -4.745476, 13.505711, 4.362337, 7.443388, 2.837510, -2.980689, 4.374337, 2.834244, 0.162282, -0.746965, -0.090051, 3.087304, -0.967346, 0.325474, 0.821223, 0.983996, -3.396871, 0.909015, -5.363075, 4.782840, -0.473375, 4.362337, 6.733424, 4.870494, 1.904466, -0.185323, 1.231946, -4.318431, 0.361593, -1.190294, 1.905203, 0.219283, 0.622021, 2.018785, 3.528921, 0.010586, -1.942714, 0.619413, 0.398316, 6.686599, -1.686293, 7.443388, 4.870494, 23.850295, 0.930555, 0.823799, 0.118207, 2.606041, 0.620503, -2.109103, 0.137880, 3.167369, -0.287015, -1.989217, 1.787051, -0.366034, 0.630643, 1.375212, -0.834671, 5.042767, 1.942663, 2.837510, 1.904466, 0.930555, 11.342673, -8.612263, 4.579168, -3.984232, -0.756717, 2.274858, -0.753184, -1.407853, 0.036723, 2.293275, -2.434337, 1.311230, -3.300554, -5.775289, -2.182918, -0.697239, 0.751110, -2.980689, -0.185323, 0.823799, -8.612263, 25.004714, -4.973192, 3.508190, 0.352154, -1.292730, 0.241265, 2.655727, -0.066159, -0.321809, 1.694145, 0.222495, -1.943509, 1.008396, -0.187757, 4.067013, -2.177375, 4.374337, 1.231946, 0.118207, 4.579168, -4.973192, 17.425159, -4.550734, -0.604502, 2.455141, 0.254145, -5.595427, 1.743113, 3.810892, -0.286844, -1.323001, 3.559141, 0.131749, 2.534200, -2.928196, -0.432895, 2.834244, -4.318431, 2.606041, -3.984232, 3.508190, -4.550734, 25.391493};
  assignment(initsigma30, lrn->sigma, NFEATURES_SQ);// assignment  

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