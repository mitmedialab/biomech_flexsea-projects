


#include "machine_learning_methods.h"
#include "linear_algebra_methods.h"


#define NCLASSES 5
#define NFEATURES 15
#define NFEATURES_SQ NFEATURES * NFEATURES

static int learning_demux_state = 0;
static int current_updating_class = 0;
static int segment = 0;
static int subsegment = 0;
static int doing_forward_substitution = 1;

void reset_learning_demux(){
  learning_demux_state = 0;
  segment = 0;
  subsegment = 0;
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
   //float initsigma30[] = {0.130333, -0.210282, 0.180823, 0.010219, 0.277491, 0.440603, -0.248909, 0.632030, 0.202503, -0.202707, -1.013976, 0.470476, 0.076976, -0.161622, 0.010229, 0.180268, -0.234139, -0.039364, 0.177040, -0.035495, -0.119323, 0.288435, -0.361462, 0.122757, 0.111523, -0.313214, -0.048033, -0.355256, -0.222247, 0.283010, -0.210282, 4.534390, 4.201983, 1.033595, 1.437767, -2.330192, 2.814116, 1.795917, 1.315940, 5.546552, 1.502210, -4.872430, 0.892909, 0.715996, -0.089303, -3.144442, 0.766919, -0.283689, -0.452576, -2.315424, -0.178551, 1.290145, 7.953972, -2.317114, 1.103097, -0.948764, 1.868072, 0.295610, 3.249730, -3.159795, 0.180823, 4.201983, 5.472262, 0.917181, 2.458588, -1.566355, 1.772364, 3.390083, 1.913251, 5.391015, -1.097279, -3.916912, 1.139296, -0.425048, -0.187441, -3.600682, -0.554696, 0.141075, -0.406375, -2.245168, -1.020740, 0.755619, 7.522373, -1.984208, 0.285064, -2.319896, 2.958142, -0.397221, 4.256579, -1.705315, 0.010219, 1.033595, 0.917181, 6.952355, -0.184373, 1.287329, -2.722329, 2.252738, -0.932924, 0.698470, -2.838646, 3.598328, -0.788311, 1.637079, -2.876563, 1.311563, -1.539805, -0.972734, -2.392001, 3.076175, 4.508864, 3.123096, 3.688930, -2.918764, 0.948291, 2.451912, 1.587448, 2.461161, 0.805802, 3.632881, 0.277491, 1.437767, 2.458588, -0.184373, 1.660565, 0.397774, 1.174317, 2.316776, 1.284446, 2.403507, -1.989828, -1.122483, 0.942909, 0.339550, 0.217276, -1.326903, -0.334571, 0.545286, 0.913393, -2.261615, -1.122792, 1.054254, 2.404846, 0.111920, 0.424237, -1.261853, 1.290656, -1.006818, 1.197763, -1.080699, 0.440603, -2.330192, -1.566355, 1.287329, 0.397774, 4.807220, -0.632468, 0.919699, 0.369667, -2.025010, -4.726927, 4.219908, 1.071920, 1.910074, -1.051448, 2.436039, 0.129174, 1.586525, 2.250509, 0.412292, -0.833530, 2.114556, -4.752508, 2.396996, -0.430457, 0.452940, 0.900026, -0.503728, -2.806486, 3.098215, -0.248909, 2.814116, 1.772364, -2.722329, 1.174317, -0.632468, 5.387892, -0.329674, 1.782958, 4.473336, 2.469355, -5.442872, 2.025467, 2.170427, 1.297218, -2.379968, 2.923166, 1.380249, 2.796775, -4.705156, -3.201427, 1.042371, 3.195954, 1.144648, 0.891913, -1.096785, 0.896228, -1.111554, 0.350793, -5.027157, 0.632030, 1.795917, 3.390083, 2.252738, 2.316776, 0.919699, -0.329674, 8.665962, -2.063682, -0.447312, -6.453596, -0.619519, 2.524601, 0.583087, 0.422105, 2.416945, 2.220482, -0.241903, -0.024633, -3.141515, 3.397590, 3.549323, 2.387617, -3.854609, 3.299755, -0.546908, -2.650921, -0.998733, -0.774715, -1.145072, 0.202503, 1.315940, 1.913251, -0.932924, 1.284446, 0.369667, 1.782958, -2.063682, 7.651874, 4.923913, -0.442200, 0.406185, -2.178779, -0.461669, -0.566493, -4.051549, -4.768192, -3.545248, 0.926661, 0.504462, -4.376109, 3.343617, 4.128049, 2.748506, 2.011651, -3.011916, 1.818687, -3.547377, 0.603158, -1.637308, -0.202707, 5.546552, 5.391015, 0.698470, 2.403507, -2.025010, 4.473336, -0.447312, 4.923913, 17.348479, 0.503008, -3.969076, -1.877057, 2.568181, 2.918324, -8.149285, -0.746540, 1.771283, 2.250093, -1.074985, -5.028596, 1.010406, 10.398848, 1.212027, -0.902652, -2.004483, 9.470214, 1.242723, 1.256793, -4.521306, -1.013976, 1.502210, -1.097279, -2.838646, -1.989828, -4.726927, 2.469355, -6.453596, -0.442200, 0.503008, 13.798787, -5.272979, -2.936328, -1.966240, -0.797913, -3.992752, 1.941307, 0.760894, -0.010559, 1.900947, 0.722714, -4.169299, 5.575708, 1.013842, 1.809069, 0.292363, 0.268896, 1.784290, 4.201754, -1.342968, 0.470476, -4.872430, -3.916912, 3.598328, -1.122483, 4.219908, -5.442872, -0.619519, 0.406185, -3.969076, -5.272979, 12.438293, -3.699583, 1.892989, -2.124122, 3.849848, -5.280052, -1.698264, 0.002951, 4.202034, 2.917013, 0.405400, -6.707497, 3.434036, 0.940344, 3.291914, 0.049626, -2.168153, -1.781591, 5.368406, 0.076976, 0.892909, 1.139296, -0.788311, 0.942909, 1.071920, 2.025467, 2.524601, -2.178779, -1.877057, -2.936328, -3.699583, 13.099845, 3.993040, 3.859693, -4.209127, 5.328132, 0.447880, -3.602743, -1.623994, -2.502331, -3.025333, -0.614580, -5.495995, -3.310032, -1.227680, -3.224489, 0.299080, -1.638387, -2.170202, -0.161622, 0.715996, -0.425048, 1.637079, 0.339550, 1.910074, 2.170427, 0.583087, -0.461669, 2.568181, -1.966240, 1.892989, 3.993040, 9.680184, 2.820557, -2.803909, 1.724995, 1.663126, -2.218092, -1.421708, -2.022853, -1.435439, 2.730907, -0.345732, 5.545570, 3.058735, -1.125429, -0.334247, -1.840002, -1.077033, 0.010229, -0.089303, -0.187441, -2.876563, 0.217276, -1.051448, 1.297218, 0.422105, -0.566493, 2.918324, -0.797913, -2.124122, 3.859693, 2.820557, 7.971719, -4.801841, 3.943879, 1.482004, -3.713028, -0.960588, -4.598538, -2.616082, 2.148156, -2.117557, 1.827878, -5.431530, -3.395318, 0.142187, -3.902732, -4.145820, 0.180268, -3.144442, -3.600682, 1.311563, -1.326903, 2.436039, -2.379968, 2.416945, -4.051549, -8.149285, -3.992752, 3.849848, -4.209127, -2.803909, -4.801841, 16.804258, 1.395907, -0.626772, 3.698107, -3.922370, 10.277794, 4.997305, -10.813094, -0.989184, -2.218996, 2.968251, -7.067869, -1.332116, -2.449696, -0.013009, -0.234139, 0.766919, -0.554696, -1.539805, -0.334571, 0.129174, 2.923166, 2.220482, -4.768192, -0.746540, 1.941307, -5.280052, 5.328132, 1.724995, 3.943879, 1.395907, 13.705138, 2.613349, 1.771183, 0.198157, -0.089412, -1.900304, -0.298808, -1.660103, 1.772357, -5.957520, -5.614849, 1.677153, -0.342369, -1.106619, -0.039364, -0.283689, 0.141075, -0.972734, 0.545286, 1.586525, 1.380249, -0.241903, -3.545248, 1.771283, 0.760894, -1.698264, 0.447880, 1.663126, 1.482004, -0.626772, 2.613349, 10.898476, 1.586291, -2.823846, -3.555234, -5.410350, -0.665486, 0.057495, -3.971212, 0.911839, 6.219635, 4.258485, -2.562063, 2.080588, 0.177040, -0.452576, -0.406375, -2.392001, 0.913393, 2.250509, 2.796775, -0.024633, 0.926661, 2.250093, -0.010559, 0.002951, -3.602743, -2.218092, -3.713028, 3.698107, 1.771183, 1.586291, 16.228246, -1.110643, -3.625432, 5.836873, -4.612338, 9.252108, -2.116596, 5.761383, 5.347393, -3.566221, 2.561401, 0.056986, -0.035495, -2.315424, -2.245168, 3.076175, -2.261615, 0.412292, -4.705156, -3.141515, 0.504462, -1.074985, 1.900947, 4.202034, -1.623994, -1.421708, -0.960588, -3.922370, 0.198157, -2.823846, -1.110643, 23.241471, -2.056567, -5.158080, -0.904956, -1.554457, 4.269061, 6.184855, 2.043095, -1.447375, -7.398160, 10.693176, -0.119323, -0.178551, -1.020740, 4.508864, -1.122792, -0.833530, -3.201427, 3.397590, -4.376109, -5.028596, 0.722714, 2.917013, -2.502331, -2.022853, -4.598538, 10.277794, -0.089412, -3.555234, -3.625432, -2.056567, 20.726957, 0.936070, -0.428816, -4.210854, 3.150450, 0.971307, -5.578360, 3.747827, 3.446239, 0.179252, 0.288435, 1.290145, 0.755619, 3.123096, 1.054254, 2.114556, 1.042371, 3.549323, 3.343617, 1.010406, -4.169299, 0.405400, -3.025333, -1.435439, -2.616082, 4.997305, -1.900304, -5.410350, 5.836873, -5.158080, 0.936070, 19.738558, 4.058180, 3.268193, 2.939770, -1.574647, -3.907690, -4.778904, -0.167953, -5.816058, -0.361462, 7.953972, 7.522373, 3.688930, 2.404846, -4.752508, 3.195954, 2.387617, 4.128049, 10.398848, 5.575708, -6.707497, -0.614580, 2.730907, 2.148156, -10.813094, -0.298808, -0.665486, -4.612338, -0.904956, -0.428816, 4.058180, 33.527260, -2.907330, 8.196413, -6.518691, -2.003139, 1.898594, 8.977464, -5.084583, 0.122757, -2.317114, -1.984208, -2.918764, 0.111920, 2.396996, 1.144648, -3.854609, 2.748506, 1.212027, 1.013842, 3.434036, -5.495995, -0.345732, -2.117557, -0.989184, -1.660103, 0.057495, 9.252108, -1.554457, -4.210854, 3.268193, -2.907330, 21.428484, -0.499584, -4.740932, 8.705749, -2.768163, 7.489160, -0.615957, 0.111523, 1.103097, 0.285064, 0.948291, 0.424237, -0.430457, 0.891913, 3.299755, 2.011651, -0.902652, 1.809069, 0.940344, -3.310032, 5.545570, 1.827878, -2.218996, 1.772357, -3.971212, -2.116596, 4.269061, 3.150450, 2.939770, 8.196413, -0.499584, 26.188465, -0.255071, -7.813288, -0.753050, -2.609574, 4.100293, -0.313214, -0.948764, -2.319896, 2.451912, -1.261853, 0.452940, -1.096785, -0.546908, -3.011916, -2.004483, 0.292363, 3.291914, -1.227680, 3.058735, -5.431530, 2.968251, -5.957520, 0.911839, 5.761383, 6.184855, 0.971307, -1.574647, -6.518691, -4.740932, -0.255071, 27.811140, 3.587939, -3.367875, -7.890287, 4.606913, -0.048033, 1.868072, 2.958142, 1.587448, 1.290656, 0.900026, 0.896228, -2.650921, 1.818687, 9.470214, 0.268896, 0.049626, -3.224489, -1.125429, -3.395318, -7.067869, -5.614849, 6.219635, 5.347393, 2.043095, -5.578360, -3.907690, -2.003139, 8.705749, -7.813288, 3.587939, 25.857153, 9.683890, 1.067752, 5.703395, -0.355256, 0.295610, -0.397221, 2.461161, -1.006818, -0.503728, -1.111554, -0.998733, -3.547377, 1.242723, 1.784290, -2.168153, 0.299080, -0.334247, 0.142187, -1.332116, 1.677153, 4.258485, -3.566221, -1.447375, 3.747827, -4.778904, 1.898594, -2.768163, -0.753050, -3.367875, 9.683890, 24.583580, -2.441739, 10.393410, -0.222247, 3.249730, 4.256579, 0.805802, 1.197763, -2.806486, 0.350793, -0.774715, 0.603158, 1.256793, 4.201754, -1.781591, -1.638387, -1.840002, -3.902732, -2.449696, -0.342369, -2.562063, 2.561401, -7.398160, 3.446239, -0.167953, 8.977464, 7.489160, -2.609574, -7.890287, 1.067752, -2.441739, 32.805023, -2.343240, 0.283010, -3.159795, -1.705315, 3.632881, -1.080699, 3.098215, -5.027157, -1.145072, -1.637308, -4.521306, -1.342968, 5.368406, -2.170202, -1.077033, -4.145820, -0.013009, -1.106619, 2.080588, 0.056986, 10.693176, 0.179252, -5.816058, -5.084583, -0.615957, 4.100293, 4.606913, 5.703395, 10.393410, -2.343240, 22.849738};
    //float initsigma5[] = {0.086442, 0.344703, -0.474981, 0.287554, -0.596325, 0.344703, 1.524754, -2.543949, 0.714580, -2.163041, -0.474981, -2.543949, 5.761625, 0.530639, 2.062213, 0.287554, 0.714580, 0.530639, 3.172750, -2.281614, -0.596325, -2.163041, 2.062213, -2.281614, 5.024490};
  float initsigma6[] = {0.131652, -0.190341, -0.284577, -0.853353, 0.601576, 0.482470, -0.190341, 1.465595, -0.734748, 1.783851, -1.077013, -0.600548, -0.284577, -0.734748, 2.010487, 0.667719, -1.388735, -0.726558, -0.853353, 1.783851, 0.667719, 9.372914, -3.867380, -4.693657, 0.601576, -1.077013, -1.388735, -3.867380, 4.929711, 2.097891, 0.482470, -0.600548, -0.726558, -4.693657, 2.097891, 4.058166};
  float initsigma15[] = {0.196066, 0.079430, -0.758377, 0.656680, 0.770641, 0.428145, 0.193270, -0.776419, -0.154880, 0.075729, 0.220717, -0.174575, -0.096767, -0.159619, -0.046818, 0.079430, 0.154791, -0.420396, -0.023832, 0.435894, -0.031896, -0.689586, -0.384718, 0.400637, 0.213753, 0.146029, -0.262122, -0.160106, -0.171987, 0.001994, -0.758377, -0.420396, 3.114329, -2.181102, -2.996337, -1.647573, 0.568155, 2.526010, 0.363029, -0.648678, -0.956576, 0.920620, 0.322865, 0.688694, 0.470429, 0.656680, -0.023832, -2.181102, 3.090070, 2.390293, 2.032239, 3.014551, -3.403248, -1.669789, -0.289851, 0.615147, -0.536467, -0.410917, -0.700376, 0.232341, 0.770641, 0.435894, -2.996337, 2.390293, 3.717464, 1.620636, 0.943189, -2.878161, 0.708094, 0.019105, 0.350580, -0.911510, -0.972354, -0.558295, 0.886847, 0.428145, -0.031896, -1.647573, 2.032239, 1.620636, 5.359001, -0.921663, 0.193887, -1.323995, -1.781851, 2.331075, -1.762497, 2.248648, -0.411374, -1.827909, 0.193270, -0.689586, 0.568155, 3.014551, 0.943189, -0.921663, 12.694006, -1.154641, 2.743840, -1.457644, -1.095540, 1.955958, -3.438040, 0.946537, -0.044746, -0.776419, -0.384718, 2.526010, -3.403248, -2.878161, 0.193887, -1.154641, 15.052792, 5.991208, 0.806134, -0.309093, 2.220633, 0.974399, 1.966281, -5.783225, -0.154880, 0.400637, 0.363029, -1.669789, 0.708094, -1.323995, 2.743840, 5.991208, 11.916041, 1.280392, 1.084915, 0.862289, -4.414141, 1.643759, -5.645044, 0.075729, 0.213753, -0.648678, -0.289851, 0.019105, -1.781851, -1.457644, 0.806134, 1.280392, 6.394678, -0.959465, -0.148677, -3.951682, -3.094364, -1.666002, 0.220717, 0.146029, -0.956576, 0.615147, 0.350580, 2.331075, -1.095540, -0.309093, 1.084915, -0.959465, 3.533536, -0.158226, 1.712550, 0.012030, -3.940953, -0.174575, -0.262122, 0.920620, -0.536467, -0.911510, -1.762497, 1.955958, 2.220633, 0.862289, -0.148677, -0.158226, 6.770384, 2.392396, 2.192636, -2.329586, -0.096767, -0.160106, 0.322865, -0.410917, -0.972354, 2.248648, -3.438040, 0.974399, -4.414141, -3.951682, 1.712550, 2.392396, 10.095829, 1.943498, -0.058342, -0.159619, -0.171987, 0.688694, -0.700376, -0.558295, -0.411374, 0.946537, 1.966281, 1.643759, -3.094364, 0.012030, 2.192636, 1.943498, 5.509617, -0.507750, -0.046818, 0.001994, 0.470429, 0.232341, 0.886847, -1.827909, -0.044746, -5.783225, -5.645044, -1.666002, -3.940953, -2.329586, -0.058342, -0.507750, 11.609971};
  assignment(initsigma15, lrn->sigma, NFEATURES_SQ);// assignment  

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