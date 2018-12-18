


#include "machine_learning_methods.h"

#define NCLASSES 5
#define NFEATURES 25
#define NFEATURES_SQ NFEATURES * NFEATURES


static int update_learner_demux_state = READY_TO_UPDATE_LEARNER;
static int update_classifier_demux_state = READY_TO_UPDATE_CLASSIFIER;
static int current_updating_class = 0;
static int segment = 0;
static int subsegment = 0;
static int doing_forward_substitution = 1;
static int making_matrix_copy  = 0;

static int matrix_copy_segment = 0;

static struct classifier_s lda;
static struct learner_s lrn;
static float* curr_feats;
static float* prev_feats;

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
  int ind = lrn.k_est*NFEATURES;
  float popkP1 = lrn.pop_k[lrn.k_est] + 1.0; //1 flop
  sum(&lrn.sum_k[ind], prev_feats, &lrn.sum_k[ind], NFEATURES); // f flops
  scaling (&lrn.sum_k[ind], 1.0/popkP1, &lrn.mu_k[ind], NFEATURES); // f flops
  lrn.pop_k[lrn.k_est] = popkP1;

}

static void update_overall_mean(){
  assignment(lrn.mu, lrn.mu_prev, NFEATURES);// assignment
  sum(lrn.sum, prev_feats, lrn.sum, NFEATURES); // f flops
  scaling(lrn.sum, 1.0/(lrn.pop + 1.0), lrn.mu, NFEATURES); // f flops
}

//Just temporarily populated. NEEDS TO BE CHANGED
static void reset_features(){
  for (int i = 0; i < NFEATURES; i++){
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

static void reset_learner_update_demux(){
  update_learner_demux_state = BACK_ESTIMATE;
  current_updating_class = 0;
  segment = 0;
  subsegment = 0;
  doing_forward_substitution = 1;
}

static void reset_classifier_update_demux(){
  update_classifier_demux_state = COPY_SUM_SIGMA;
  matrix_copy_segment = 0;
  making_matrix_copy = 1;

}

static void init_features(){
  curr_feats = (float*)calloc(NFEATURES, sizeof(float));
  prev_feats = (float*)calloc(NFEATURES, sizeof(float));
}

static void init_learner(){

  lrn.mu_k = (float*)calloc(NCLASSES * NFEATURES, sizeof(float));
  lrn.sum_k = (float*)calloc(NCLASSES * NFEATURES, sizeof(float));
  lrn.mu_prev  = (float*)calloc( NFEATURES, sizeof(float));
  lrn.mu  = (float*)calloc( NFEATURES, sizeof(float));
  lrn.sum  = (float*)calloc( NFEATURES, sizeof(float));
  lrn.sum_sigma = (float*)calloc(NFEATURES_SQ, sizeof(float));
  float initsigma25[] = {0.617439, -0.074544, -0.047983, -0.057283, -0.003744, 0.005660, 0.043102, 0.112097, -0.033200, 0.072765, 0.119194, -0.013174, -0.139741, 0.106925, -0.010260, -0.034997, 0.174746, 0.040093, 0.089611, -0.079618, 0.021593, 0.012026, -0.037920, 0.018960, -0.046791, -0.074544, 0.963336, 0.041178, 0.071605, -0.060663, 0.092363, 0.140734, 0.075646, -0.160839, 0.006212, -0.059533, 0.013357, -0.051000, 0.037614, -0.149397, 0.125148, -0.064436, 0.025535, -0.234675, -0.076935, -0.287715, -0.083315, 0.033149, -0.122958, 0.020887, -0.047983, 0.041178, 0.925971, 0.034526, 0.134240, -0.068901, 0.016917, -0.094070, 0.030248, 0.093619, 0.091341, -0.017735, 0.103215, 0.024270, -0.178531, 0.067080, 0.083300, -0.151221, -0.163168, 0.078312, -0.125479, 0.159221, 0.036733, 0.076128, 0.231629, -0.057283, 0.071605, 0.034526, 0.916906, -0.043904, 0.148354, -0.055682, -0.043480, 0.155813, -0.009451, 0.018605, -0.017309, -0.072706, -0.020003, -0.004881, 0.005647, 0.054999, 0.019367, -0.097279, -0.160577, -0.095385, -0.094130, -0.088555, -0.100970, -0.011827, -0.003744, -0.060663, 0.134240, -0.043904, 0.995713, -0.058500, 0.000749, -0.019893, 0.121221, 0.098305, 0.068676, -0.043997, 0.220049, 0.037179, -0.031844, 0.165236, -0.097652, -0.051778, -0.089956, 0.132569, -0.043556, -0.025807, 0.033810, -0.132728, 0.159241, 0.005660, 0.092363, -0.068901, 0.148354, -0.058500, 1.112870, 0.222039, 0.202870, -0.180456, 0.084038, 0.000780, 0.070315, -0.181198, 0.010957, -0.075254, 0.020000, 0.014471, 0.023418, -0.054062, 0.176724, -0.022079, -0.129481, 0.077129, -0.002417, 0.206967, 0.043102, 0.140734, 0.016917, -0.055682, 0.000749, 0.222039, 1.066912, 0.217300, -0.098424, -0.023141, -0.027137, 0.146946, -0.001692, -0.111097, 0.012061, 0.020648, 0.026578, 0.122044, 0.026125, -0.022449, -0.085159, 0.017509, 0.214061, 0.035479, 0.049985, 0.112097, 0.075646, -0.094070, -0.043480, -0.019893, 0.202870, 0.217300, 1.041997, -0.179550, -0.068967, -0.006141, 0.178169, -0.065831, 0.048578, 0.122231, 0.052515, -0.100242, 0.032105, -0.065405, 0.196004, 0.068204, -0.141591, 0.227169, 0.027426, -0.058748, -0.033200, -0.160839, 0.030248, 0.155813, 0.121221, -0.180456, -0.098424, -0.179550, 1.311985, 0.241422, -0.021723, -0.152787, 0.030121, -0.034324, 0.052764, -0.008968, 0.045338, 0.020664, 0.314140, -0.097963, -0.038879, 0.079617, -0.069170, -0.076822, 0.170492, 0.072765, 0.006212, 0.093619, -0.009451, 0.098305, 0.084038, -0.023141, -0.068967, 0.241422, 1.062195, -0.000700, 0.072938, 0.067887, 0.084607, 0.001658, 0.096915, 0.138086, 0.011776, -0.030360, 0.028405, -0.255215, -0.166982, -0.216260, 0.006049, -0.025947, 0.119194, -0.059533, 0.091341, 0.018605, 0.068676, 0.000780, -0.027137, -0.006141, -0.021723, -0.000700, 1.149114, -0.038706, 0.065011, -0.068945, -0.097856, -0.017974, 0.024216, -0.068886, 0.120082, -0.013352, 0.107441, 0.035700, -0.071044, 0.077077, 0.125067, -0.013174, 0.013357, -0.017735, -0.017309, -0.043997, 0.070315, 0.146946, 0.178169, -0.152787, 0.072938, -0.038706, 0.928133, -0.025315, 0.089529, 0.005885, 0.045658, -0.098008, -0.045866, -0.075991, -0.007677, -0.174689, -0.108887, 0.022741, 0.012108, -0.127012, -0.139741, -0.051000, 0.103215, -0.072706, 0.220049, -0.181198, -0.001692, -0.065831, 0.030121, 0.067887, 0.065011, -0.025315, 1.308134, 0.030737, 0.143096, -0.136383, -0.063358, 0.006979, 0.041613, -0.125411, 0.040503, -0.187652, 0.020972, 0.047417, -0.004024, 0.106925, 0.037614, 0.024270, -0.020003, 0.037179, 0.010957, -0.111097, 0.048578, -0.034324, 0.084607, -0.068945, 0.089529, 0.030737, 1.027110, 0.016268, 0.077338, -0.071113, 0.042073, -0.102346, 0.103726, -0.024661, -0.093245, 0.051404, -0.003373, -0.127292, -0.010260, -0.149397, -0.178531, -0.004881, -0.031844, -0.075254, 0.012061, 0.122231, 0.052764, 0.001658, -0.097856, 0.005885, 0.143096, 0.016268, 1.053989, -0.045558, -0.022953, 0.105135, 0.071113, -0.032310, -0.072636, -0.103334, 0.018654, 0.131571, 0.050851, -0.034997, 0.125148, 0.067080, 0.005647, 0.165236, 0.020000, 0.020648, 0.052515, -0.008968, 0.096915, -0.017974, 0.045658, -0.136383, 0.077338, -0.045558, 1.154127, 0.066624, 0.107948, -0.090849, 0.166827, -0.088465, -0.107012, -0.010690, 0.010016, 0.036433, 0.174746, -0.064436, 0.083300, 0.054999, -0.097652, 0.014471, 0.026578, -0.100242, 0.045338, 0.138086, 0.024216, -0.098008, -0.063358, -0.071113, -0.022953, 0.066624, 1.198479, 0.033255, 0.081398, -0.074985, 0.083597, -0.162053, -0.147935, -0.031757, 0.050775, 0.040093, 0.025535, -0.151221, 0.019367, -0.051778, 0.023418, 0.122044, 0.032105, 0.020664, 0.011776, -0.068886, -0.045866, 0.006979, 0.042073, 0.105135, 0.107948, 0.033255, 1.055325, 0.108575, -0.011273, 0.057672, -0.211145, -0.024386, -0.108340, -0.070894, 0.089611, -0.234675, -0.163168, -0.097279, -0.089956, -0.054062, 0.026125, -0.065405, 0.314140, -0.030360, 0.120082, -0.075991, 0.041613, -0.102346, 0.071113, -0.090849, 0.081398, 0.108575, 1.529982, 0.065208, 0.056816, 0.092857, 0.020807, -0.075920, 0.049188, -0.079618, -0.076935, 0.078312, -0.160577, 0.132569, 0.176724, -0.022449, 0.196004, -0.097963, 0.028405, -0.013352, -0.007677, -0.125411, 0.103726, -0.032310, 0.166827, -0.074985, -0.011273, 0.065208, 1.115632, 0.076093, -0.034685, 0.142671, -0.014768, 0.166253, 0.021593, -0.287715, -0.125479, -0.095385, -0.043556, -0.022079, -0.085159, 0.068204, -0.038879, -0.255215, 0.107441, -0.174689, 0.040503, -0.024661, -0.072636, -0.088465, 0.083597, 0.057672, 0.056816, 0.076093, 1.313475, -0.013969, 0.075861, 0.002909, -0.163137, 0.012026, -0.083315, 0.159221, -0.094130, -0.025807, -0.129481, 0.017509, -0.141591, 0.079617, -0.166982, 0.035700, -0.108887, -0.187652, -0.093245, -0.103334, -0.107012, -0.162053, -0.211145, 0.092857, -0.034685, -0.013969, 1.057662, 0.077606, -0.027555, 0.148168, -0.037920, 0.033149, 0.036733, -0.088555, 0.033810, 0.077129, 0.214061, 0.227169, -0.069170, -0.216260, -0.071044, 0.022741, 0.020972, 0.051404, 0.018654, -0.010690, -0.147935, -0.024386, 0.020807, 0.142671, 0.075861, 0.077606, 0.971885, 0.030119, -0.006194, 0.018960, -0.122958, 0.076128, -0.100970, -0.132728, -0.002417, 0.035479, 0.027426, -0.076822, 0.006049, 0.077077, 0.012108, 0.047417, -0.003373, 0.131571, 0.010016, -0.031757, -0.108340, -0.075920, -0.014768, 0.002909, -0.027555, 0.030119, 0.883840, 0.030624, -0.046791, 0.020887, 0.231629, -0.011827, 0.159241, 0.206967, 0.049985, -0.058748, 0.170492, -0.025947, 0.125067, -0.127012, -0.004024, -0.127292, 0.050851, 0.036433, 0.050775, -0.070894, 0.049188, 0.166253, -0.163137, 0.148168, -0.006194, 0.030624, 1.501455};
  //float initsigma30[] = {0.504353, -0.076315, 0.527225, -1.043556, 0.108013, 0.676101, -1.133654, -0.694797, -0.318353, 1.254594, -0.053926, 1.084670, -0.200995, -1.242915, -0.284086, -0.152506, -0.282600, 0.179418, 0.472163, 0.232865, 0.175255, -0.135177, 0.161119, -0.720239, -0.830419, -0.033516, 0.930838, -1.280365, 1.289717, 0.577125, -0.076315, 0.507884, -0.072911, 0.007840, -0.004062, -1.562505, -0.057352, 0.053771, -0.092910, 0.332736, -0.509348, -0.111939, -1.185077, 0.227179, 0.575338, -0.534253, 0.257238, -1.717905, -0.457390, -0.331342, 0.492571, -0.477713, -0.225975, 1.687377, 0.013174, -0.396883, 0.762645, 0.219283, 0.209866, 0.282612, 0.527225, -0.072911, 0.628575, -0.673524, 0.335271, 0.598971, -1.352883, -0.678232, 0.046970, 1.562989, -0.089984, 0.801181, 0.095539, -1.249159, -0.477877, 0.222938, -0.449521, 0.489757, 1.193552, 0.222562, 0.359948, -0.712643, 0.264935, -0.942662, -1.070093, -0.292701, 1.105452, -1.916349, 1.336026, 0.848865, -1.043556, 0.007840, -0.673524, 4.623610, 1.050047, -1.311473, 1.663688, 1.952373, 3.130779, -2.312622, 0.162390, -4.312458, 1.816539, 2.872293, -0.712869, 3.021521, -0.181076, 1.623824, 2.332333, -0.616214, 0.356409, -2.843542, -0.101013, -0.205699, 0.612932, -1.112761, -1.907585, -0.405609, -3.453893, 0.016442, 0.108013, -0.004062, 0.335271, 1.050047, 0.781005, -0.015773, -0.438410, 0.031046, 1.482602, 0.438804, 0.267155, -0.922424, 0.697580, 0.260526, -0.195479, 1.049200, -0.273695, 0.924383, 1.786572, -0.741332, 0.007650, -1.428505, 0.129105, -0.687707, -0.722627, -0.944828, 0.799366, -1.546355, -0.038300, 0.404145, 0.676101, -1.562505, 0.598971, -1.311473, -0.015773, 5.481212, -0.390129, -0.599006, 0.171623, -1.231316, 1.819347, 1.464075, 2.545493, -1.469973, -1.522459, 1.354420, -0.937183, 4.768777, 0.221702, 0.590600, -2.055329, 2.060929, 0.433594, -5.686598, -0.588687, 1.083294, -1.318855, -0.685348, 0.136079, -0.576948, -1.133654, -0.057352, -1.352883, 1.663688, -0.438410, -0.390129, 3.767298, 1.388059, 1.028102, -4.347772, 0.938677, -2.197072, -0.177510, 3.865296, 1.367187, -0.605131, 1.763300, 0.006049, -2.809902, -2.618100, -2.681911, 2.374291, -0.816100, 1.789322, 2.646695, 0.554395, -1.958154, 5.333612, -3.389114, -3.258628, -0.694797, 0.053771, -0.678232, 1.952373, 0.031046, -0.599006, 1.388059, 6.275604, 4.724299, -6.308123, 2.148380, -0.227062, -0.894825, 3.053403, -4.723358, 1.946502, 0.442846, -3.513256, 0.689819, 1.023673, 1.293367, -0.215181, 2.257996, 3.388966, 0.874728, 0.122104, -5.490247, 4.139968, -5.255141, 1.017887, -0.318353, -0.092910, 0.046970, 3.130779, 1.482602, 0.171623, 1.028102, 4.724299, 7.337899, -4.824400, 2.042740, -2.896971, 0.927120, 4.479355, -3.996219, 2.682264, -0.505514, -0.123603, 2.873045, -2.124054, -1.098295, -1.168151, 1.999723, -0.012138, 0.093341, -2.068901, -1.079059, 1.508139, -3.413131, 1.995217, 1.254594, 0.332736, 1.562989, -2.312622, 0.438804, -1.231316, -4.347772, -6.308123, -4.824400, 15.640815, -2.568570, 1.082721, 4.228366, -4.773529, 2.365341, -5.280254, 0.840016, 3.471308, 9.888274, -0.673153, 3.618240, -2.374591, 1.482111, 0.564050, -0.005396, 2.311982, 4.470874, -8.019963, 7.947604, -0.837708, -0.053926, -0.509348, -0.089984, 0.162390, 0.267155, 1.819347, 0.938677, 2.148380, 2.042740, -2.568570, 6.785222, -1.031612, 4.825164, -0.578021, 3.996182, -0.645074, 0.645497, -1.366373, 2.713006, -0.365604, 0.364722, 2.123048, 2.616101, -0.033648, 0.454654, 0.566083, -4.108161, 1.554436, -2.883353, -4.638040, 1.084670, -0.111939, 0.801181, -4.312458, -0.922424, 1.464075, -2.197072, -0.227062, -2.896971, 1.082721, -1.031612, 15.698903, -5.457545, -2.112106, -5.217370, -0.589193, 3.759060, 0.263474, -2.421527, -2.306635, 1.574448, -0.068431, -1.954679, 8.594986, -1.986841, 1.463540, 0.320386, 2.799507, 0.140994, -1.437689, -0.200995, -1.185077, 0.095539, 1.816539, 0.697580, 2.545493, -0.177510, -0.894825, 0.927120, 4.228366, 4.825164, -5.457545, 18.611443, -1.271726, 4.907792, -1.533576, -3.238093, 4.576469, 12.684152, -0.120292, 3.140671, 0.077976, 7.506277, -6.027232, 3.096809, 1.934774, -0.404139, -6.536447, 5.063108, 0.091369, -1.242915, 0.227179, -1.249159, 2.872293, 0.260526, -1.469973, 3.865296, 3.053403, 4.479355, -4.773529, -0.578021, -2.112106, -1.271726, 8.788349, -2.360338, 0.509685, 0.855112, 1.977387, -2.683089, -2.767799, -3.012964, 2.907910, -0.442630, 2.572526, 2.998433, -2.213315, 0.588921, 6.982497, -2.526639, -1.791572, -0.284086, 0.575338, -0.477877, -0.712869, -0.195479, -1.522459, 1.367187, -4.723358, -3.996219, 2.365341, 3.996182, -5.217370, 4.907792, -2.360338, 16.597528, -5.296372, 0.766392, -2.656458, 1.435871, -2.765652, -0.734589, 3.345388, -1.131168, -3.303369, 0.666461, -1.170668, 2.473526, -2.014864, 2.172348, -4.242507, -0.152506, -0.534253, 0.222938, 3.021521, 1.049200, 1.354420, -0.605131, 1.946502, 2.682264, -5.280254, -0.645074, -0.589193, -1.533576, 0.509685, -5.296372, 10.789203, -3.754509, 5.093081, -5.136157, 4.390998, 0.854042, -3.092010, -0.074625, -3.842722, -4.412772, -4.913805, -0.185913, -0.248104, -2.378076, 3.225056, -0.282600, 0.257238, -0.449521, -0.181076, -0.273695, -0.937183, 1.763300, 0.442846, -0.505514, 0.840016, 0.645497, 3.759060, -3.238093, 0.855112, 0.766392, -3.754509, 11.563004, -3.478936, 5.809597, -5.362992, -1.020999, -1.048153, 0.616134, 11.809500, -0.109541, 6.183907, -6.239956, 4.349795, -3.092780, -7.007747, 0.179418, -1.717905, 0.489757, 1.623824, 0.924383, 4.768777, 0.006049, -3.513256, -0.123603, 3.471308, -1.366373, 0.263474, 4.576469, 1.977387, -2.656458, 5.093081, -3.478936, 25.950671, -3.317490, -0.414984, -8.500404, -1.098303, -0.382547, -9.309262, -2.651522, -4.919544, 2.370763, -6.078785, 0.976679, 2.796184, 0.472163, -0.457390, 1.193552, 2.332333, 1.786572, 0.221702, -2.809902, 0.689819, 2.873045, 9.888274, 2.713006, -2.421527, 12.684152, -2.683089, 1.435871, -5.136157, 5.809597, -3.317490, 30.876203, -4.247638, 2.585462, -5.647319, 6.975529, 5.975903, 1.085544, 8.640355, -4.842014, -5.662815, 2.287323, 0.631924, 0.232865, -0.331342, 0.222562, -0.616214, -0.741332, 0.590600, -2.618100, 1.023673, -2.124054, -0.673153, -0.365604, -2.306635, -0.120292, -2.767799, -2.765652, 4.390998, -5.362992, -0.414984, -4.247638, 17.094219, 4.054688, 1.281565, -2.307733, -3.692021, -4.666502, -2.479379, -5.940178, -5.175204, 1.981504, 0.155546, 0.175255, 0.492571, 0.359948, 0.356409, 0.007650, -2.055329, -2.681911, 1.293367, -1.098295, 3.618240, 0.364722, 1.574448, 3.140671, -3.012964, -0.734589, 0.854042, -1.020999, -8.500404, 2.585462, 4.054688, 27.978745, -1.913966, 1.522494, 2.050982, 5.041331, 1.422013, -7.065418, -4.502652, 4.358351, -4.163186, -0.135177, -0.477713, -0.712643, -2.843542, -1.428505, 2.060929, 2.374291, -0.215181, -1.168151, -2.374591, 2.123048, -0.068431, 0.077976, 2.907910, 3.345388, -3.092010, -1.048153, -1.098303, -5.647319, 1.281565, -1.913966, 15.369256, -0.237015, -5.819443, 4.022746, -0.273437, 0.473442, 2.906572, 1.576411, -9.807399, 0.161119, -0.225975, 0.264935, -0.101013, 0.129105, 0.433594, -0.816100, 2.257996, 1.999723, 1.482111, 2.616101, -1.954679, 7.506277, -0.442630, -1.131168, -0.074625, 0.616134, -0.382547, 6.975529, -2.307733, 1.522494, -0.237015, 15.351460, -3.576239, 2.999825, 2.680061, 1.835567, 0.365388, 3.592334, 1.236998, -0.720239, 1.687377, -0.942662, -0.205699, -0.687707, -5.686598, 1.789322, 3.388966, -0.012138, 0.564050, -0.033648, 8.594986, -6.027232, 2.572526, -3.303369, -3.842722, 11.809500, -9.309262, 5.975903, -3.692021, 2.050982, -5.819443, -3.576239, 39.142070, -1.622912, 5.248481, -8.166486, 12.326122, -5.092107, -5.298656, -0.830419, 0.013174, -1.070093, 0.612932, -0.722627, -0.588687, 2.646695, 0.874728, 0.093341, -0.005396, 0.454654, -1.986841, 3.096809, 2.998433, 0.666461, -4.412772, -0.109541, -2.651522, 1.085544, -4.666502, 5.041331, 4.022746, 2.999825, -1.622912, 16.341521, 8.937578, 3.819566, 2.041376, -7.098245, -6.341375, -0.033516, -0.396883, -0.292701, -1.112761, -0.944828, 1.083294, 0.554395, 0.122104, -2.068901, 2.311982, 0.566083, 1.463540, 1.934774, -2.213315, -1.170668, -4.913805, 6.183907, -4.919544, 8.640355, -2.479379, 1.422013, -0.273437, 2.680061, 5.248481, 8.937578, 20.630895, -6.373646, 0.254913, -3.274045, -9.831621, 0.930838, 0.762645, 1.105452, -1.907585, 0.799366, -1.318855, -1.958154, -5.490247, -1.079059, 4.470874, -4.108161, 0.320386, -0.404139, 0.588921, 2.473526, -0.185913, -6.239956, 2.370763, -4.842014, -5.940178, -7.065418, 0.473442, 1.835567, -8.166486, 3.819566, -6.373646, 27.248507, -3.669296, 1.540207, 8.314150, -1.280365, 0.219283, -1.916349, -0.405609, -1.546355, -0.685348, 5.333612, 4.139968, 1.508139, -8.019963, 1.554436, 2.799507, -6.536447, 6.982497, -2.014864, -0.248104, 4.349795, -6.078785, -5.662815, -5.175204, -4.502652, 2.906572, 0.365388, 12.326122, 2.041376, 0.254913, -3.669296, 26.181164, -6.540642, 0.853714, 1.289717, 0.209866, 1.336026, -3.453893, -0.038300, 0.136079, -3.389114, -5.255141, -3.413131, 7.947604, -2.883353, 0.140994, 5.063108, -2.526639, 2.172348, -2.378076, -3.092780, 0.976679, 2.287323, 1.981504, 4.358351, 1.576411, 3.592334, -5.092107, -7.098245, -3.274045, 1.540207, -6.540642, 26.983449, 0.068617, 0.577125, 0.282612, 0.848865, 0.016442, 0.404145, -0.576948, -3.258628, 1.017887, 1.995217, -0.837708, -4.638040, -1.437689, 0.091369, -1.791572, -4.242507, 3.225056, -7.007747, 2.796184, 0.631924, 0.155546, -4.163186, -9.807399, 1.236998, -5.298656, -6.341375, -9.831621, 8.314150, 0.853714, 0.068617, 37.441526};
    //float initsigma5[] = {0.086442, 0.344703, -0.474981, 0.287554, -0.596325, 0.344703, 1.524754, -2.543949, 0.714580, -2.163041, -0.474981, -2.543949, 5.761625, 0.530639, 2.062213, 0.287554, 0.714580, 0.530639, 3.172750, -2.281614, -0.596325, -2.163041, 2.062213, -2.281614, 5.024490};
  //float initsigma6[] = {0.131652, -0.190341, -0.284577, -0.853353, 0.601576, 0.482470, -0.190341, 1.465595, -0.734748, 1.783851, -1.077013, -0.600548, -0.284577, -0.734748, 2.010487, 0.667719, -1.388735, -0.726558, -0.853353, 1.783851, 0.667719, 9.372914, -3.867380, -4.693657, 0.601576, -1.077013, -1.388735, -3.867380, 4.929711, 2.097891, 0.482470, -0.600548, -0.726558, -4.693657, 2.097891, 4.058166};
  //float initsigma15[] = {0.196066, 0.079430, -0.758377, 0.656680, 0.770641, 0.428145, 0.193270, -0.776419, -0.154880, 0.075729, 0.220717, -0.174575, -0.096767, -0.159619, -0.046818, 0.079430, 0.154791, -0.420396, -0.023832, 0.435894, -0.031896, -0.689586, -0.384718, 0.400637, 0.213753, 0.146029, -0.262122, -0.160106, -0.171987, 0.001994, -0.758377, -0.420396, 3.114329, -2.181102, -2.996337, -1.647573, 0.568155, 2.526010, 0.363029, -0.648678, -0.956576, 0.920620, 0.322865, 0.688694, 0.470429, 0.656680, -0.023832, -2.181102, 3.090070, 2.390293, 2.032239, 3.014551, -3.403248, -1.669789, -0.289851, 0.615147, -0.536467, -0.410917, -0.700376, 0.232341, 0.770641, 0.435894, -2.996337, 2.390293, 3.717464, 1.620636, 0.943189, -2.878161, 0.708094, 0.019105, 0.350580, -0.911510, -0.972354, -0.558295, 0.886847, 0.428145, -0.031896, -1.647573, 2.032239, 1.620636, 5.359001, -0.921663, 0.193887, -1.323995, -1.781851, 2.331075, -1.762497, 2.248648, -0.411374, -1.827909, 0.193270, -0.689586, 0.568155, 3.014551, 0.943189, -0.921663, 12.694006, -1.154641, 2.743840, -1.457644, -1.095540, 1.955958, -3.438040, 0.946537, -0.044746, -0.776419, -0.384718, 2.526010, -3.403248, -2.878161, 0.193887, -1.154641, 15.052792, 5.991208, 0.806134, -0.309093, 2.220633, 0.974399, 1.966281, -5.783225, -0.154880, 0.400637, 0.363029, -1.669789, 0.708094, -1.323995, 2.743840, 5.991208, 11.916041, 1.280392, 1.084915, 0.862289, -4.414141, 1.643759, -5.645044, 0.075729, 0.213753, -0.648678, -0.289851, 0.019105, -1.781851, -1.457644, 0.806134, 1.280392, 6.394678, -0.959465, -0.148677, -3.951682, -3.094364, -1.666002, 0.220717, 0.146029, -0.956576, 0.615147, 0.350580, 2.331075, -1.095540, -0.309093, 1.084915, -0.959465, 3.533536, -0.158226, 1.712550, 0.012030, -3.940953, -0.174575, -0.262122, 0.920620, -0.536467, -0.911510, -1.762497, 1.955958, 2.220633, 0.862289, -0.148677, -0.158226, 6.770384, 2.392396, 2.192636, -2.329586, -0.096767, -0.160106, 0.322865, -0.410917, -0.972354, 2.248648, -3.438040, 0.974399, -4.414141, -3.951682, 1.712550, 2.392396, 10.095829, 1.943498, -0.058342, -0.159619, -0.171987, 0.688694, -0.700376, -0.558295, -0.411374, 0.946537, 1.966281, 1.643759, -3.094364, 0.012030, 2.192636, 1.943498, 5.509617, -0.507750, -0.046818, 0.001994, 0.470429, 0.232341, 0.886847, -1.827909, -0.044746, -5.783225, -5.645044, -1.666002, -3.940953, -2.329586, -0.058342, -0.507750, 11.609971};
  //float initsigma20[] = {0.325480, -0.900402, -0.016734, 0.499174, 0.176019, -0.719629, -0.272897, -0.413333, 0.483828, 0.000929, -1.032918, 0.496668, 0.115517, 0.179518, 0.162282, 0.361593, 0.620503, -0.756717, 0.352154, -0.604502, -0.900402, 2.608819, 0.073511, -1.936114, -0.370636, 2.227967, 0.248204, 1.213302, -1.011898, -0.408747, 2.568849, -1.828794, -0.171574, -0.703050, -0.746965, -1.190294, -2.109103, 2.274858, -1.292730, 2.455141, -0.016734, 0.073511, 0.918284, -0.760432, 0.170401, 0.847315, -0.012066, 1.011702, -0.271668, -0.563574, 1.253652, 0.973365, 0.697654, 0.010148, -0.090051, 1.905203, 0.137880, -0.753184, 0.241265, 0.254145, 0.499174, -1.936114, -0.760432, 5.636076, -1.652139, -2.307923, 1.869759, 0.470106, -2.111872, 1.874178, -2.581091, 4.453869, -2.272944, 3.126108, 3.087304, 0.219283, 3.167369, -1.407853, 2.655727, -5.595427, 0.176019, -0.370636, 0.170401, -1.652139, 1.305183, -0.372332, -0.155663, -1.533283, 1.268305, -0.264122, 0.159411, -1.209456, 0.507884, -0.578278, -0.967346, 0.622021, -0.287015, 0.036723, -0.066159, 1.743113, -0.719629, 2.227967, 0.847315, -2.307923, -0.372332, 5.583549, -0.130831, 0.994492, -1.350560, 0.984355, 2.367493, 0.524498, 0.497082, 1.832780, 0.325474, 2.018785, -1.989217, 2.293275, -0.321809, 3.810892, -0.272897, 0.248204, -0.012066, 1.869759, -0.155663, -0.130831, 6.488305, -1.681635, -2.207982, 5.100583, 4.065406, 1.984963, -5.800360, 4.648804, 0.821223, 3.528921, 1.787051, -2.434337, 1.694145, -0.286844, -0.413333, 1.213302, 1.011702, 0.470106, -1.533283, 0.994492, -1.681635, 5.314849, -2.338951, -3.786552, -0.175366, 2.288411, 1.023700, -0.629530, 0.983996, 0.010586, -0.366034, 1.311230, 0.222495, -1.323001, 0.483828, -1.011898, -0.271668, -2.111872, 1.268305, -1.350560, -2.207982, -2.338951, 4.870410, -0.282469, 0.141216, -3.128225, 3.245529, -2.627502, -3.396871, -1.942714, 0.630643, -3.300554, -1.943509, 3.559141, 0.000929, -0.408747, -0.563574, 1.874178, -0.264122, 0.984355, 5.100583, -3.786552, -0.282469, 8.422316, 2.291562, 0.210019, -4.407759, 3.377958, 0.909015, 0.619413, 1.375212, -5.775289, 1.008396, 0.131749, -1.032918, 2.568849, 1.253652, -2.581091, 0.159411, 2.367493, 4.065406, -0.175366, 0.141216, 2.291562, 14.986868, -1.541772, 0.600122, -0.806166, -5.363075, 0.398316, -0.834671, -2.182918, -0.187757, 2.534200, 0.496668, -1.828794, 0.973365, 4.453869, -1.209456, 0.524498, 1.984963, 2.288411, -3.128225, 0.210019, -1.541772, 9.570947, 0.622355, 7.478968, 4.782840, 6.686599, 5.042767, -0.697239, 4.067013, -2.928196, 0.115517, -0.171574, 0.697654, -2.272944, 0.507884, 0.497082, -5.800360, 1.023700, 3.245529, -4.407759, 0.600122, 0.622355, 13.624505, -4.745476, -0.473375, -1.686293, 1.942663, 0.751110, -2.177375, -0.432895, 0.179518, -0.703050, 0.010148, 3.126108, -0.578278, 1.832780, 4.648804, -0.629530, -2.627502, 3.377958, -0.806166, 7.478968, -4.745476, 13.505711, 4.362337, 7.443388, 2.837510, -2.980689, 4.374337, 2.834244, 0.162282, -0.746965, -0.090051, 3.087304, -0.967346, 0.325474, 0.821223, 0.983996, -3.396871, 0.909015, -5.363075, 4.782840, -0.473375, 4.362337, 6.733424, 4.870494, 1.904466, -0.185323, 1.231946, -4.318431, 0.361593, -1.190294, 1.905203, 0.219283, 0.622021, 2.018785, 3.528921, 0.010586, -1.942714, 0.619413, 0.398316, 6.686599, -1.686293, 7.443388, 4.870494, 23.850295, 0.930555, 0.823799, 0.118207, 2.606041, 0.620503, -2.109103, 0.137880, 3.167369, -0.287015, -1.989217, 1.787051, -0.366034, 0.630643, 1.375212, -0.834671, 5.042767, 1.942663, 2.837510, 1.904466, 0.930555, 11.342673, -8.612263, 4.579168, -3.984232, -0.756717, 2.274858, -0.753184, -1.407853, 0.036723, 2.293275, -2.434337, 1.311230, -3.300554, -5.775289, -2.182918, -0.697239, 0.751110, -2.980689, -0.185323, 0.823799, -8.612263, 25.004714, -4.973192, 3.508190, 0.352154, -1.292730, 0.241265, 2.655727, -0.066159, -0.321809, 1.694145, 0.222495, -1.943509, 1.008396, -0.187757, 4.067013, -2.177375, 4.374337, 1.231946, 0.118207, 4.579168, -4.973192, 17.425159, -4.550734, -0.604502, 2.455141, 0.254145, -5.595427, 1.743113, 3.810892, -0.286844, -1.323001, 3.559141, 0.131749, 2.534200, -2.928196, -0.432895, 2.834244, -4.318431, 2.606041, -3.984232, 3.508190, -4.550734, 25.391493};
  assignment(initsigma25,lrn.sum_sigma, NFEATURES_SQ);// assignment  

  lrn.pop_k = (float*)calloc(NCLASSES, sizeof(float));
  for (int i = 0; i < NCLASSES; i++)
    lrn.pop_k[i] = 1.0;

  lrn.pop = 1.0;

  lrn.n_updates = 0;

  //init intermediary matrices
  lrn.A = (float*)calloc(NFEATURES_SQ, sizeof(float));
  lrn.x  = (float*)calloc( NFEATURES, sizeof(float));
  lrn.y  = (float*)calloc( NFEATURES, sizeof(float));
  lrn.k_est = TASK_FL;
}

static void init_classifier(){
  lda.A = (float*)calloc(NCLASSES * NFEATURES, sizeof(float));
  lda.B = (float*)calloc(NCLASSES, sizeof(float));
  lda.score_k = (float*)calloc(NCLASSES, sizeof(float));
  lda.k_pred = TASK_FL;
  lda.latest_sum_sigma = (float*)calloc(NFEATURES_SQ, sizeof(float));
  //init intermediary matrices
  lda.UT = (float*)calloc(NFEATURES_SQ, sizeof(float));
  lda.LT = (float*)calloc(NFEATURES_SQ, sizeof(float));
  lda.x  = (float*)calloc( NFEATURES, sizeof(float));
  lda.y  = (float*)calloc( NFEATURES, sizeof(float));
  
}



void update_learner_demux(struct back_estimator_s* be, int latest_foot_off_samples, int learner_update_reset_trigger){
    switch (update_learner_demux_state){
      case BACK_ESTIMATE:
        back_estimate(be, latest_foot_off_samples);
        update_learner_demux_state++;
      break;
      case UPDATE_CLASS_MEAN: //1 sample
        update_class_mean(); //2f flops
        update_learner_demux_state++;
      break;
      case UPDATE_OVERALL_MEAN: // 1 sample
        update_overall_mean(); //2f flops
        update_learner_demux_state++;
      break;
      case GET_DEVIATION_FROM_CURR_MEAN: // 1 sample
        diff(prev_feats, lrn.mu, lrn.x, NFEATURES);//f flops
        update_learner_demux_state++;
      break;
      case GET_DEVIATION_FROM_PREV_MEAN: // 1 sample
        diff(prev_feats, lrn.mu_prev, lrn.y, NFEATURES); //f flops
        update_learner_demux_state++;
      break;
      case UPDATE_COVARIANCE: // f samples
      {
        if (making_matrix_copy)
          return;
        int segment_section = segment*NFEATURES;
        segmented_outer_product(lrn.x, lrn.y, lrn.A, NFEATURES, segment); //2f flops
        sum(&lrn.sum_sigma[segment_section], &lrn.A[segment_section], &lrn.sum_sigma[segment_section], NFEATURES);//f flops

        segment++;
        if (segment == NFEATURES){
          update_learner_demux_state++; 
          lrn.n_updates++;
          segment = 0;
          subsegment = 0;
          lrn.pop = lrn.pop + 1.0;
        }
      }
      break;
      case READY_TO_UPDATE_LEARNER:
        if (learner_update_reset_trigger)
          reset_learner_update_demux();
      break;
    }
  }

void update_classifier_demux(){

      switch (update_classifier_demux_state){
      case COPY_SUM_SIGMA:
          assignment(&lrn.sum_sigma[matrix_copy_segment], &lda.latest_sum_sigma[matrix_copy_segment], NFEATURES);// assignment
          matrix_copy_segment = matrix_copy_segment + NFEATURES;
          if (matrix_copy_segment == NFEATURES_SQ){
            making_matrix_copy = 0;
            update_classifier_demux_state++;
          }
      break;
      case DO_CHOLESKY: // f(f+1)/2 samples

        super_segmented_cholesky(lda.latest_sum_sigma, lda.LT, NFEATURES, segment, subsegment);
        subsegment++;
        if (subsegment > segment){
          subsegment = 0;
          segment++;
        }

        if (segment == NFEATURES){
          update_classifier_demux_state++; 
          segment = 0;
        }
      break;
      case UPDATE_TRANSPOSE: //f samples
        segmented_lower_to_upper_transpose(lda.LT, lda.UT, NFEATURES, segment);
        segment++;
        if (segment == NFEATURES){
          update_classifier_demux_state++;
          segment = 1;
        }
      break;
      case UPDATE_LDA_A_PARAMS: //2*c*f
      {
        int class_times_n_features = current_updating_class*NFEATURES;
        if (doing_forward_substitution){
          segmented_forward_substitution(lda.LT, &lrn.sum_k[class_times_n_features], lda.y, NFEATURES, segment); // roughly 1/2 f^2 flops
          segment++;
          if (segment == NFEATURES){
            segment = NFEATURES-2;
            doing_forward_substitution = 0;
          }
        }
        else{
          segmented_backward_substitution(lda.UT, lda.y, &lda.A[class_times_n_features], NFEATURES, segment); // roughly 1/2 f^2 flops
          segment--;
          if (segment == -1){
            //lda.B[current_updating_class] = -0.5*inner_product(&lrn.sum_k[class_times_n_features], &lda.A[class_times_n_features], NFEATURES);
            segment = 1;
            doing_forward_substitution = 1;
            current_updating_class++;
          }
        }
        if (current_updating_class == NCLASSES){
          update_classifier_demux_state++; 
          current_updating_class = 0;
        }  
      }
      break;
      case UPDATE_LDA_B_PARAMS:
      {
        int class_times_n_features = current_updating_class*NFEATURES;
        lda.B[current_updating_class] = -0.5*inner_product(&lrn.mu_k[class_times_n_features], &lda.A[class_times_n_features], NFEATURES);
        current_updating_class++;
        if (current_updating_class == NCLASSES){
          reset_classifier_update_demux();
        }  
      }    
      break;
  }
}

//Just temporarily populated. NEEDS TO BE CHANGED
void update_features(struct kinematics_s* kin){
    curr_feats[PAZ_MAX] = MAX(kin->pAz, curr_feats[PAZ_MAX]);
    curr_feats[PAZ_SUM] = curr_feats[PAZ_SUM] + kin->pAz;
    curr_feats[VAZ_MIN] = MIN(kin->vAz, curr_feats[VAZ_MIN]);
    //curr_feats[PITCH_RANGE] = MAX(kin->pAz, curr_feats[PITCH_RANGE]);
    curr_feats[OMX_MAX] = MAX(kin->aOmegaX, curr_feats[OMX_MAX]);
    curr_feats[ACCY_SUM] = curr_feats[ACCY_SUM] + kin->aAccY;

    curr_feats[PITCH_MAX] = MAX(kin->rot[0], curr_feats[PITCH_MAX]);
    curr_feats[PITCH_MIN] = MIN(kin->rot[0], curr_feats[PITCH_MIN]);
}

void classify(){

  curr_feats[PITCH_RANGE] = curr_feats[PITCH_MAX] - curr_feats[PITCH_MIN];

  float maxScore = FLT_MIN;
  for (int i = 0; i < 5; i++){
    lda.score_k[i] = inner_product(&lda.A[i*NFEATURES], curr_feats, NFEATURES) + lda.B[i];
    if (lda.score_k[i] > maxScore){
      maxScore = lda.score_k[i];
      lda.k_pred = i;
    }
  }
  reset_features();
}

void init_learning_structs(){
  init_classifier();
  init_learner();
  init_features();
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
