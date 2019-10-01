/*
 * torque_replay.h
 *
 *  Created on: 1 oct. 2019
 *      Author: Guillermo Herrera-Arcos
 */

#ifndef FLEXSEA_PROJECTS_MIT_DLEG_INC_TORQUE_REPLAY_H_
#define FLEXSEA_PROJECTS_MIT_DLEG_INC_TORQUE_REPLAY_H_

#define TRAJ_SIZE 1001
#define USER_MASS 75.0
#define IMPEDANCE_MODE_THRESHOLD 100 // check

static const float torque_traj[TRAJ_SIZE] = {-2.5738,-2.33,-2.1328,-1.9804,-1.8706,-1.8016,-1.7714,-1.778,-1.8196,-1.8941,-1.9996,-2.1341,-2.2957,-2.4824,-2.6923,-2.9235,-3.1739,-3.4416,-3.7247,-4.0212,-4.3291,-4.6466,-4.9715,-5.3021,-5.6363,-5.9722,-6.3079,-6.6413,-6.9705,-7.2936,-7.6087,-7.9136,-8.2066,-8.4857,-8.7489,-8.9954,-9.2249,-9.4371,-9.6318,-9.8086,-9.9673,-10.108,-10.229,-10.332,-10.416,-10.48,-10.524,-10.548,-10.552,-10.536,-10.498,-10.44,-10.363,-10.269,-10.16,-10.039,-9.9078,-9.7684,-9.623,-9.4738,-9.323,-9.1728,-9.0253,-8.8827,-8.7472,-8.6209,-8.506,-8.4047,-8.3176,-8.2432,-8.1797,-8.1254,-8.0785,-8.0372,-7.9998,-7.9646,-7.9297,-7.8934,-7.854,-7.8097,-7.7588,-7.6995,-7.6299,-7.5485,-7.4535,-7.3448,-7.2232,-7.0896,-6.9448,-6.7897,-6.625,-6.4516,-6.2704,-6.0822,-5.8878,-5.6881,-5.4839,-5.276,-5.0653,-4.8526,-4.6388,-4.4246,-4.2104,-3.9966,-3.7835,-3.5715,-3.3609,-3.152,-2.9453,-2.741,-2.5395,-2.3411,-2.1462,-1.9551,-1.7682,-1.5858,-1.4082,-1.2359,-1.0687,-0.9064,-0.74839,-0.59421,-0.44339,-0.29546,-0.14994,-0.006351,0.13578,0.27693,0.41758,0.5582,0.69927,0.84127,0.98467,1.13,1.2776,1.4277,1.5802,1.7352,1.8927,2.0524,2.2146,2.379,2.5457,2.7147,2.8859,3.0593,3.2349,3.4126,3.5924,3.7743,3.9582,4.1441,4.3318,4.521,4.7115,4.9031,5.0955,5.2884,5.4817,5.6751,5.8683,6.0611,6.2533,6.4447,6.6349,6.8238,7.011,7.1965,7.38,7.5618,7.742,7.9207,8.0982,8.2745,8.45,8.6246,8.7987,8.9724,9.1458,9.3191,9.4925,9.6661,9.8401,10.015,10.19,10.366,10.543,10.72,10.898,11.077,11.255,11.434,11.614,11.793,11.973,12.152,12.332,12.511,12.69,12.869,13.048,13.226,13.403,13.581,13.758,13.935,14.112,14.288,14.465,14.641,14.818,14.995,15.171,15.348,15.526,15.703,15.881,16.06,16.238,16.417,16.596,16.775,16.954,17.133,17.311,17.489,17.666,17.842,18.018,18.192,18.365,18.536,18.707,18.875,19.042,19.207,19.371,19.533,19.694,19.854,20.014,20.173,20.331,20.489,20.647,20.805,20.964,21.123,21.282,21.443,21.604,21.767,21.931,22.095,22.26,22.426,22.592,22.759,22.925,23.092,23.258,23.424,23.59,23.755,23.919,24.082,24.244,24.404,24.564,24.722,24.879,25.034,25.188,25.341,25.493,25.644,25.793,25.942,26.089,26.236,26.381,26.526,26.669,26.812,26.954,27.095,27.235,27.375,27.514,27.652,27.79,27.927,28.064,28.2,28.336,28.472,28.607,28.743,28.878,29.012,29.147,29.282,29.417,29.551,29.686,29.82,29.953,30.086,30.219,30.35,30.481,30.611,30.74,30.869,30.996,31.121,31.246,31.369,31.491,31.611,31.731,31.849,31.966,32.081,32.196,32.311,32.424,32.536,32.648,32.759,32.87,32.98,33.09,33.2,33.309,33.418,33.527,33.636,33.744,33.851,33.958,34.064,34.17,34.274,34.378,34.481,34.584,34.685,34.785,34.883,34.981,35.077,35.172,35.266,35.359,35.451,35.542,35.632,35.722,35.81,35.898,35.985,36.072,36.158,36.244,36.329,36.414,36.499,36.584,36.669,36.753,36.837,36.921,37.005,37.089,37.172,37.256,37.339,37.421,37.504,37.586,37.668,37.75,37.832,37.913,37.994,38.075,38.156,38.236,38.316,38.397,38.477,38.557,38.637,38.717,38.797,38.877,38.957,39.038,39.118,39.199,39.28,39.361,39.442,39.523,39.605,39.687,39.769,39.851,39.934,40.016,40.099,40.182,40.266,40.349,40.433,40.518,40.602,40.687,40.772,40.857,40.942,41.027,41.113,41.199,41.285,41.371,41.457,41.543,41.629,41.715,41.801,41.887,41.973,42.059,42.145,42.231,42.318,42.404,42.491,42.579,42.667,42.756,42.846,42.936,43.028,43.121,43.215,43.31,43.407,43.505,43.605,43.707,43.809,43.914,44.019,44.125,44.233,44.341,44.449,44.559,44.668,44.778,44.889,44.999,45.109,45.219,45.329,45.439,45.548,45.657,45.766,45.876,45.985,46.095,46.205,46.316,46.427,46.539,46.652,46.766,46.882,46.998,47.116,47.235,47.355,47.477,47.6,47.724,47.85,47.977,48.105,48.234,48.364,48.495,48.627,48.76,48.894,49.028,49.164,49.3,49.437,49.574,49.712,49.851,49.991,50.131,50.273,50.415,50.557,50.701,50.846,50.991,51.137,51.285,51.433,51.582,51.732,51.884,52.036,52.189,52.344,52.499,52.656,52.815,52.974,53.135,53.297,53.461,53.626,53.793,53.961,54.131,54.303,54.476,54.652,54.828,55.006,55.186,55.367,55.55,55.734,55.919,56.105,56.292,56.48,56.67,56.86,57.051,57.243,57.435,57.628,57.822,58.017,58.212,58.408,58.605,58.803,59.002,59.202,59.403,59.605,59.808,60.012,60.218,60.424,60.633,60.842,61.053,61.265,61.478,61.693,61.909,62.126,62.345,62.564,62.785,63.007,63.23,63.454,63.679,63.905,64.132,64.36,64.589,64.818,65.049,65.281,65.513,65.747,65.982,66.218,66.455,66.693,66.933,67.173,67.415,67.659,67.903,68.149,68.396,68.645,68.895,69.147,69.399,69.653,69.908,70.164,70.421,70.679,70.937,71.197,71.457,71.717,71.979,72.241,72.503,72.766,73.029,73.292,73.556,73.82,74.085,74.35,74.616,74.883,75.15,75.418,75.686,75.955,76.226,76.496,76.768,77.041,77.315,77.589,77.865,78.141,78.418,78.696,78.974,79.254,79.534,79.814,80.095,80.377,80.659,80.942,81.225,81.508,81.792,82.076,82.361,82.646,82.931,83.216,83.502,83.788,84.074,84.36,84.647,84.934,85.222,85.51,85.798,86.086,86.375,86.664,86.953,87.243,87.532,87.822,88.112,88.401,88.691,88.98,89.269,89.558,89.846,90.134,90.422,90.708,90.995,91.28,91.565,91.849,92.132,92.414,92.695,92.976,93.255,93.534,93.812,94.088,94.364,94.639,94.913,95.186,95.458,95.729,95.999,96.267,96.535,96.801,97.067,97.331,97.593,97.855,98.114,98.372,98.629,98.884,99.137,99.389,99.638,99.886,100.13,100.38,100.62,100.86,101.09,101.33,101.56,101.79,102.02,102.25,102.48,102.7,102.92,103.14,103.36,103.57,103.79,104,104.21,104.41,104.62,104.82,105.02,105.22,105.42,105.61,105.8,105.99,106.17,106.35,106.53,106.71,106.88,107.05,107.21,107.37,107.53,107.68,107.83,107.98,108.12,108.26,108.4,108.53,108.66,108.78,108.9,109.02,109.13,109.25,109.35,109.46,109.56,109.65,109.74,109.83,109.92,110,110.07,110.15,110.22,110.28,110.34,110.4,110.45,110.5,110.54,110.58,110.61,110.64,110.67,110.69,110.7,110.71,110.72,110.72,110.72,110.71,110.7,110.68,110.65,110.63,110.59,110.56,110.51,110.47,110.41,110.36,110.29,110.22,110.15,110.06,109.98,109.88,109.78,109.67,109.55,109.43,109.3,109.16,109.01,108.86,108.69,108.52,108.34,108.16,107.96,107.76,107.56,107.34,107.13,106.9,106.68,106.45,106.21,105.97,105.73,105.49,105.24,104.99,104.74,104.48,104.23,103.96,103.7,103.43,103.15,102.87,102.58,102.29,102,101.69,101.38,101.07,100.74,100.41,100.07,99.726,99.371,99.006,98.632,98.249,97.855,97.45,97.035,96.608,96.17,95.719,95.256,94.78,94.291,93.788,93.271,92.74,92.196,91.638,91.067,90.482,89.884,89.273,88.65,88.013,87.363,86.701,86.027,85.34,84.641,83.93,83.206,82.471,81.725,80.968,80.201,79.424,78.638,77.842,77.038,76.226,75.407,74.58,73.746,72.907,72.061,71.21,70.354,69.494,68.629,67.759,66.885,66.007,65.124,64.236,63.344,62.447,61.546,60.64,59.73,58.814,57.895,56.97,56.041,55.107,54.169,53.226,52.279,51.329,50.374,49.417,48.457,47.493,46.528,45.56,44.59,43.619,42.646,41.672,40.697,39.722,38.746,37.771,36.796,35.821,34.848,33.877,32.908,31.942,30.978,30.018,29.062,28.11,27.163,26.221,25.285,24.354,23.43,22.513,21.603,20.703,19.812,18.931,18.063,17.206,16.363,15.535,14.722,13.925,13.145,12.384,11.642,10.92,10.218,9.5391,8.8823,8.2478,7.6353,7.0447,6.4756,5.9279,5.4013,4.8955,4.4104,3.9457,3.5012,3.0765,2.6716,2.2862,1.9199,1.5727,1.2442,0.9342,0.64251,0.36887,0.11305,-0.1252,-0.3461,-0.5499,-0.73682,-0.9071,-1.061,-1.1987,-1.3205,-1.4265,-1.5172,-1.5925,-1.6529};
static const float massGains[TRAJ_SIZE] = {-0.036768,-0.033285,-0.030469,-0.028291,-0.026723,-0.025737,-0.025305,-0.0254,-0.025994,-0.027058,-0.028565,-0.030487,-0.032796,-0.035463,-0.038462,-0.041764,-0.045341,-0.049166,-0.05321,-0.057445,-0.061844,-0.066379,-0.071022,-0.075745,-0.080519,-0.085318,-0.090113,-0.094876,-0.099579,-0.10419,-0.1087,-0.11305,-0.11724,-0.12122,-0.12498,-0.12851,-0.13178,-0.13482,-0.1376,-0.14012,-0.14239,-0.1444,-0.14613,-0.1476,-0.14879,-0.14971,-0.15034,-0.15069,-0.15075,-0.15051,-0.14997,-0.14914,-0.14804,-0.1467,-0.14515,-0.14342,-0.14154,-0.13955,-0.13747,-0.13534,-0.13319,-0.13104,-0.12893,-0.1269,-0.12496,-0.12316,-0.12151,-0.12007,-0.11882,-0.11776,-0.11685,-0.11608,-0.11541,-0.11482,-0.11428,-0.11378,-0.11328,-0.11276,-0.1122,-0.11157,-0.11084,-0.10999,-0.109,-0.10784,-0.10648,-0.10493,-0.10319,-0.10128,-0.099212,-0.096996,-0.094643,-0.092166,-0.089578,-0.086889,-0.084112,-0.081259,-0.078341,-0.075371,-0.072361,-0.069323,-0.066268,-0.063208,-0.060148,-0.057094,-0.05405,-0.051021,-0.048012,-0.045029,-0.042075,-0.039157,-0.036278,-0.033444,-0.03066,-0.02793,-0.02526,-0.022654,-0.020117,-0.017655,-0.015268,-0.012949,-0.010691,-0.0084887,-0.0063341,-0.0042208,-0.002142,-9.0729e-05,0.0019397,0.0039562,0.0059654,0.0079743,0.0099896,0.012018,0.014067,0.016142,0.018251,0.020395,0.022575,0.024789,0.027038,0.029321,0.031637,0.033986,0.036368,0.038782,0.041228,0.043705,0.046213,0.048752,0.05132,0.053919,0.056546,0.059202,0.061883,0.064586,0.067308,0.070044,0.072793,0.075549,0.07831,0.081073,0.083833,0.086588,0.089333,0.092067,0.094784,0.097482,0.10016,0.10281,0.10543,0.10803,0.1106,0.11315,0.11569,0.11821,0.12071,0.12321,0.1257,0.12818,0.13065,0.13313,0.13561,0.13809,0.14057,0.14307,0.14557,0.14809,0.15061,0.15315,0.15569,0.15824,0.16079,0.16335,0.16591,0.16847,0.17104,0.1736,0.17617,0.17873,0.18129,0.18384,0.18639,0.18894,0.19148,0.19401,0.19654,0.19907,0.20159,0.20412,0.20664,0.20916,0.21168,0.21421,0.21673,0.21926,0.2218,0.22433,0.22687,0.22942,0.23197,0.23453,0.23709,0.23965,0.24221,0.24476,0.2473,0.24984,0.25237,0.25489,0.25739,0.25988,0.26235,0.26481,0.26724,0.26964,0.27203,0.27439,0.27672,0.27904,0.28134,0.28363,0.28591,0.28818,0.29044,0.2927,0.29496,0.29722,0.29948,0.30176,0.30404,0.30633,0.30863,0.31096,0.31329,0.31564,0.318,0.32037,0.32275,0.32513,0.32751,0.32988,0.33226,0.33463,0.337,0.33935,0.3417,0.34403,0.34634,0.34863,0.35091,0.35317,0.35541,0.35763,0.35983,0.36202,0.36419,0.36634,0.36848,0.3706,0.3727,0.3748,0.37688,0.37894,0.38099,0.38303,0.38506,0.38707,0.38908,0.39107,0.39305,0.39503,0.397,0.39896,0.40091,0.40286,0.4048,0.40674,0.40868,0.41061,0.41254,0.41446,0.41639,0.41832,0.42024,0.42216,0.42408,0.426,0.4279,0.4298,0.43169,0.43358,0.43545,0.4373,0.43915,0.44098,0.44279,0.44459,0.44637,0.44813,0.44987,0.45159,0.45329,0.45498,0.45665,0.45831,0.45995,0.46158,0.4632,0.4648,0.4664,0.46799,0.46957,0.47115,0.47272,0.47428,0.47585,0.47741,0.47896,0.48051,0.48205,0.48359,0.48511,0.48663,0.48814,0.48964,0.49112,0.49259,0.49405,0.49549,0.49692,0.49833,0.49973,0.5011,0.50246,0.50381,0.50513,0.50645,0.50775,0.50903,0.51031,0.51157,0.51283,0.51407,0.51531,0.51654,0.51777,0.51899,0.5202,0.52142,0.52263,0.52384,0.52504,0.52625,0.52745,0.52865,0.52984,0.53103,0.53222,0.53341,0.53459,0.53577,0.53695,0.53812,0.53929,0.54045,0.54161,0.54277,0.54393,0.54508,0.54623,0.54738,0.54852,0.54967,0.55081,0.55195,0.5531,0.55424,0.55539,0.55653,0.55768,0.55883,0.55998,0.56114,0.56229,0.56345,0.56462,0.56578,0.56695,0.56813,0.5693,0.57048,0.57166,0.57285,0.57403,0.57523,0.57642,0.57762,0.57882,0.58003,0.58124,0.58245,0.58367,0.58488,0.58611,0.58733,0.58855,0.58978,0.59101,0.59224,0.59347,0.5947,0.59593,0.59716,0.59839,0.59962,0.60085,0.60207,0.6033,0.60454,0.60578,0.60702,0.60827,0.60953,0.6108,0.61208,0.61338,0.61468,0.61601,0.61735,0.61872,0.6201,0.6215,0.62293,0.62438,0.62585,0.62734,0.62884,0.63036,0.63189,0.63344,0.63499,0.63655,0.63812,0.63969,0.64127,0.64284,0.64442,0.64599,0.64756,0.64912,0.65068,0.65224,0.6538,0.65536,0.65693,0.6585,0.66007,0.66165,0.66325,0.66485,0.66646,0.66809,0.66974,0.6714,0.67308,0.67478,0.6765,0.67824,0.68,0.68178,0.68357,0.68538,0.68721,0.68906,0.69091,0.69279,0.69467,0.69657,0.69848,0.70041,0.70234,0.70429,0.70624,0.7082,0.71018,0.71216,0.71416,0.71616,0.71818,0.72021,0.72225,0.7243,0.72637,0.72844,0.73054,0.73264,0.73476,0.73689,0.73903,0.74119,0.74337,0.74556,0.74777,0.74999,0.75223,0.75449,0.75677,0.75907,0.76139,0.76373,0.76609,0.76847,0.77088,0.77331,0.77576,0.77824,0.78074,0.78326,0.78581,0.78837,0.79096,0.79357,0.79619,0.79884,0.8015,0.80417,0.80686,0.80957,0.81228,0.81501,0.81775,0.8205,0.82326,0.82603,0.82881,0.8316,0.8344,0.83722,0.84004,0.84289,0.84574,0.84861,0.8515,0.8544,0.85732,0.86025,0.86321,0.86618,0.86917,0.87218,0.87521,0.87826,0.88133,0.88442,0.88752,0.89064,0.89378,0.89693,0.9001,0.90328,0.90648,0.9097,0.91293,0.91617,0.91942,0.92269,0.92598,0.92927,0.93258,0.93591,0.93924,0.9426,0.94597,0.94936,0.95276,0.95618,0.95962,0.96308,0.96655,0.97004,0.97356,0.97709,0.98064,0.98422,0.98781,0.99142,0.99504,0.99869,1.0023,1.006,1.0097,1.0134,1.0171,1.0208,1.0245,1.0283,1.032,1.0358,1.0395,1.0433,1.047,1.0508,1.0546,1.0584,1.0621,1.0659,1.0698,1.0736,1.0774,1.0812,1.0851,1.0889,1.0928,1.0967,1.1006,1.1045,1.1084,1.1124,1.1163,1.1203,1.1242,1.1282,1.1322,1.1362,1.1402,1.1442,1.1482,1.1523,1.1563,1.1604,1.1644,1.1685,1.1725,1.1766,1.1807,1.1847,1.1888,1.1929,1.197,1.2011,1.2051,1.2092,1.2133,1.2175,1.2216,1.2257,1.2298,1.2339,1.2381,1.2422,1.2463,1.2505,1.2546,1.2587,1.2629,1.267,1.2711,1.2753,1.2794,1.2835,1.2876,1.2917,1.2958,1.2999,1.304,1.3081,1.3121,1.3162,1.3202,1.3242,1.3282,1.3322,1.3362,1.3402,1.3441,1.3481,1.352,1.3559,1.3598,1.3637,1.3676,1.3714,1.3752,1.3791,1.3829,1.3867,1.3904,1.3942,1.3979,1.4016,1.4053,1.409,1.4126,1.4162,1.4198,1.4234,1.4269,1.4305,1.4339,1.4374,1.4408,1.4442,1.4476,1.4509,1.4542,1.4575,1.4607,1.464,1.4671,1.4703,1.4734,1.4765,1.4796,1.4827,1.4857,1.4887,1.4916,1.4946,1.4975,1.5003,1.5032,1.506,1.5087,1.5114,1.5141,1.5167,1.5193,1.5219,1.5244,1.5268,1.5292,1.5316,1.5339,1.5361,1.5383,1.5405,1.5426,1.5446,1.5466,1.5485,1.5504,1.5522,1.554,1.5558,1.5574,1.5591,1.5606,1.5622,1.5637,1.5651,1.5664,1.5678,1.569,1.5702,1.5714,1.5725,1.5735,1.5745,1.5754,1.5763,1.5771,1.5778,1.5785,1.5791,1.5797,1.5802,1.5806,1.5809,1.5812,1.5815,1.5816,1.5817,1.5817,1.5817,1.5815,1.5814,1.5811,1.5808,1.5804,1.5799,1.5794,1.5788,1.5781,1.5773,1.5765,1.5756,1.5746,1.5735,1.5724,1.5711,1.5697,1.5683,1.5667,1.5651,1.5633,1.5614,1.5594,1.5573,1.5551,1.5528,1.5503,1.5477,1.5451,1.5423,1.5395,1.5365,1.5335,1.5304,1.5272,1.524,1.5207,1.5173,1.5139,1.5105,1.507,1.5034,1.4999,1.4963,1.4926,1.4889,1.4852,1.4814,1.4775,1.4736,1.4696,1.4655,1.4613,1.4571,1.4528,1.4483,1.4438,1.4392,1.4345,1.4296,1.4247,1.4196,1.4144,1.409,1.4036,1.3979,1.3921,1.3862,1.3801,1.3739,1.3674,1.3608,1.354,1.347,1.3398,1.3324,1.3249,1.3171,1.3091,1.301,1.2926,1.2841,1.2753,1.2664,1.2573,1.248,1.2386,1.229,1.2191,1.2092,1.199,1.1887,1.1782,1.1675,1.1567,1.1457,1.1346,1.1234,1.112,1.1005,1.0889,1.0772,1.0654,1.0535,1.0415,1.0294,1.0173,1.0051,0.99277,0.98041,0.96799,0.9555,0.94295,0.93034,0.91766,0.90492,0.8921,0.87923,0.86629,0.85328,0.84021,0.82707,0.81386,0.80059,0.78725,0.77384,0.76037,0.74685,0.73327,0.71964,0.70596,0.69224,0.67848,0.66468,0.65086,0.637,0.62313,0.60923,0.59532,0.58139,0.56746,0.55352,0.53958,0.52565,0.51173,0.49783,0.48396,0.47011,0.45631,0.44254,0.42883,0.41517,0.40157,0.38804,0.37459,0.36121,0.34791,0.33471,0.32161,0.30862,0.29575,0.28303,0.27045,0.25804,0.2458,0.23376,0.22193,0.21031,0.19893,0.18779,0.17691,0.16631,0.15599,0.14598,0.13627,0.12689,0.11783,0.10908,0.10064,0.092509,0.084684,0.077161,0.069936,0.063006,0.056367,0.050017,0.043951,0.038166,0.032659,0.027427,0.022467,0.017774,0.013346,0.0091787,0.0052696,0.0016149,-0.0017886,-0.0049443,-0.0078557,-0.010526,-0.012959,-0.015157,-0.017124,-0.018864,-0.020379,-0.021674,-0.022751,-0.023613};
static const float speedGains[TRAJ_SIZE] = {-0.39102,-0.37312,-0.35802,-0.34568,-0.336,-0.32892,-0.32438,-0.32234,-0.3227,-0.32544,-0.33046,-0.33774,-0.34716,-0.35872,-0.37232,-0.38788,-0.36714,-0.3843,-0.40352,-0.42478,-0.44798,-0.47308,-0.5,-0.52866,-0.55904,-0.59106,-0.62464,-0.65974,-0.69632,-0.73424,-0.77352,-0.81404,-0.85576,-0.89856,-0.94234,-0.98656,-1.03084,-1.07442,-1.11716,-1.1583,-1.19724,-1.23386,-1.26738,-1.29726,-1.3231,-1.34438,-1.3605,-1.37114,-1.38088,-1.38178,-1.37542,-1.36084,-1.33894,-1.31004,-1.2752,-1.23492,-1.19008,-1.1414,-1.08954,-1.0354,-0.97946,-0.92264,-0.8654,-0.80884,-0.7535,-0.7001,-0.64962,-0.60334,-0.56196,-0.5254,-0.49354,-0.4662,-0.44322,-0.42446,-0.4098,-0.39904,-0.39206,-0.38868,-0.38876,-0.39214,-0.4103,-0.42178,-0.43608,-0.45272,-0.4712,-0.49102,-0.51182,-0.5332,-0.55478,-0.57616,-0.59696,-0.61676,-0.6352,-0.65186,-0.66638,-0.67836,-0.6874,-0.69312,-0.6951,-0.69306,-0.68666,-0.67568,-0.66042,-0.64124,-0.61852,-0.59266,-0.56408,-0.5331,-0.50014,-0.46558,-0.42982,-0.39322,-0.35618,-0.343708,-0.306202,-0.269514,-0.234148,-0.200614,-0.169218,-0.139958,-0.112816,-0.08777672,-0.0648212,-0.043932,-0.025088,-0.008274,0.00653199999999999,0.019344,0.030186,0.039078,0.04603,0.0511,0.05442,0.05604,0.05606,0.05462,0.05184,0.04786,0.04282,0.03686,0.0301200000000001,0.02276,0.01484,0.00659999999999998,-0.00185999999999993,-0.0367,-0.0455,-0.05408,-0.06224,-0.06988,-0.0767600000000001,-0.0827800000000001,-0.08788,-0.0921,-0.0954000000000001,-0.0978000000000001,-0.0992799999999999,-0.0998400000000001,-0.09948,-0.09818,-0.09596,-0.09282,-0.08872,-0.0837199999999999,-0.0778600000000001,-0.0712,-0.0637400000000001,-0.0556199999999999,-0.04686,-0.03758,-0.0278599999999999,-0.0177999999999999,-0.00752000000000006,0.00285999999999991,0.0132999999999999,0.0236599999999999,0.03384,0.0187799999999999,0.02822,0.0371600000000001,0.0455399999999999,0.05334,0.0604600000000001,0.06684,0.0724,0.0771799999999999,0.0812199999999997,0.0845800000000001,0.0873200000000001,0.0895000000000003,0.09116,0.09238,0.0932200000000002,0.0937000000000001,0.0939,0.09388,0.09362,0.09328,0.0927800000000001,0.0922999999999998,0.0916,0.0912000000000003,0.0905999999999999,0.0902000000000001,0.0896000000000001,0.0891999999999999,0.0885999999999999,0.0884,0.0879999999999999,0.0875999999999998,0.0620000000000001,0.0617999999999999,0.0616,0.0616,0.0620000000000001,0.0622,0.0628,0.0635999999999999,0.0648,0.066,0.0678000000000001,0.0696000000000001,0.0718,0.0739999999999998,0.0763999999999999,0.0789999999999999,0.082,0.0847999999999999,0.0880000000000003,0.091,0.0940000000000001,0.0969999999999999,0.0997999999999998,0.1024,0.105,0.1072,0.109,0.1106,0.1118,0.1126,0.1132,0.1134,0.0899999999999999,0.0893999999999998,0.0884,0.0869999999999997,0.0854000000000003,0.0832000000000001,0.0811999999999998,0.0787999999999997,0.0762,0.0735999999999997,0.0707999999999998,0.0678000000000004,0.0647999999999996,0.0618000000000002,0.0589999999999996,0.0560000000000002,0.0531999999999997,0.0502000000000002,0.0474000000000004,0.0448,0.0420000000000002,0.0393999999999998,0.0367999999999995,0.0341999999999999,0.0317999999999998,0.0292000000000002,0.0268000000000001,0.0244,0.0221999999999994,0.0196000000000005,0.0171999999999997,0.0149999999999999,-0.0078000000000003,-0.0100000000000001,-0.0123999999999995,-0.0146000000000001,-0.0167999999999999,-0.0192,-0.0217999999999996,-0.0240000000000002,-0.0265999999999998,-0.0292000000000002,-0.0317999999999998,-0.0345999999999997,-0.0375999999999998,-0.0405999999999999,-0.0434000000000005,-0.0466000000000001,-0.0497999999999998,-0.0533999999999999,-0.0566000000000002,-0.0602000000000004,-0.0637999999999998,-0.0673999999999999,-0.0712000000000003,-0.0747999999999998,-0.0783999999999999,-0.0822000000000003,-0.0857999999999997,-0.0891999999999996,-0.0927999999999997,-0.0962000000000003,-0.0997999999999998,-0.1032,-0.1064,-0.1268,-0.129799999999999,-0.132599999999999,-0.1352,-0.1378,-0.1404,-0.1428,-0.1452,-0.1478,-0.1502,-0.1526,-0.1552,-0.158,-0.1606,-0.1632,-0.166,-0.169,-0.172000000000001,-0.1752,-0.1786,-0.1818,-0.1856,-0.188999999999999,-0.1928,-0.1966,-0.2004,-0.2044,-0.2082,-0.2124,-0.2166,-0.2206,-0.2246,-0.2426,-0.2466,-0.250400000000001,-0.2544,-0.2582,-0.262,-0.2656,-0.269600000000001,-0.2732,-0.277,-0.2806,-0.2844,-0.288,-0.2916,-0.295200000000001,-0.299,-0.302800000000001,-0.3066,-0.3102,-0.3142,-0.318,-0.3218,-0.3258,-0.329799999999999,-0.334,-0.338,-0.342,-0.3462,-0.3504,-0.354600000000001,-0.358600000000001,-0.363,-0.3788,-0.383,-0.3872,-0.391200000000001,-0.3956,-0.3996,-0.4038,-0.4078,-0.412,-0.4162,-0.4202,-0.4242,-0.4282,-0.4322,-0.4364,-0.4402,-0.4444,-0.4482,-0.4522,-0.4564,-0.4604,-0.4646,-0.4688,-0.473,-0.4774,-0.4818,-0.4864,-0.490799999999999,-0.4956,-0.500400000000001,-0.5052,-0.510400000000001,-0.5156,-0.5326,-0.538,-0.5436,-0.5488,-0.5542,-0.5596,-0.5648,-0.5704,-0.5756,-0.5808,-0.586,-0.591,-0.596,-0.6006,-0.6054,-0.6098,-0.6142,-0.618400000000001,-0.6226,-0.6266,-0.6304,-0.634200000000001,-0.638,-0.6416,-0.6452,-0.648999999999999,-0.6528,-0.6566,-0.6602,-0.6642,-0.668,-0.672,-0.6902,-0.6948,-0.6992,-0.7038,-0.7088,-0.7136,-0.7184,-0.723399999999999,-0.7282,-0.733,-0.738,-0.742599999999999,-0.7474,-0.751800000000001,-0.7562,-0.760599999999999,-0.7648,-0.7686,-0.7724,-0.7758,-0.7792,-0.7822,-0.7852,-0.787799999999999,-0.7904,-0.7928,-0.795,-0.7968,-0.7986,-0.8002,-0.8018,-0.803,-0.821000000000001,-0.822199999999999,-0.823200000000001,-0.8242,-0.8252,-0.826,-0.826600000000001,-0.827,-0.8276,-0.828000000000001,-0.8282,-0.8282,-0.828400000000001,-0.8284,-0.828,-0.827800000000001,-0.827399999999999,-0.8268,-0.8264,-0.825599999999999,-0.8248,-0.823799999999999,-0.822799999999999,-0.8216,-0.8204,-0.819,-0.8176,-0.816200000000001,-0.8146,-0.813,-0.8114,-0.809599999999999,-0.807599999999999,-0.827199999999999,-0.8254,-0.823399999999999,-0.8214,-0.8194,-0.817400000000001,-0.8152,-0.813000000000001,-0.8108,-0.8084,-0.805800000000001,-0.803400000000001,-0.800800000000001,-0.798399999999999,-0.7958,-0.7928,-0.790199999999999,-0.7874,-0.784399999999999,-0.781399999999999,-0.7782,-0.775,-0.771600000000001,-0.767999999999999,-0.764400000000001,-0.7604,-0.756400000000001,-0.752000000000001,-0.7474,-0.7426,-0.737599999999999,-0.7324,-0.7542,-0.7484,-0.742399999999999,-0.735599999999999,-0.728999999999999,-0.722,-0.7148,-0.7072,-0.6994,-0.6914,-0.683199999999999,-0.674800000000001,-0.666,-0.6572,-0.6482,-0.638799999999999,-0.6294,-0.62,-0.6102,-0.6004,-0.590199999999999,-0.580200000000001,-0.569799999999999,-0.5592,-0.548599999999999,-0.5378,-0.5268,-0.515400000000001,-0.5038,-0.4922,-0.4804,-0.4682,-0.488399999999999,-0.4762,-0.4636,-0.4508,-0.437599999999999,-0.4242,-0.411000000000001,-0.3974,-0.3836,-0.37,-0.356,-0.342,-0.328,-0.313800000000001,-0.2998,-0.2854,-0.2714,-0.257199999999999,-0.242999999999999,-0.228999999999999,-0.2148,-0.2008,-0.1866,-0.1726,-0.158399999999999,-0.144200000000001,-0.1298,-0.1154,-0.100999999999999,-0.0861999999999995,-0.0713999999999999,-0.0561999999999998,-0.0409999999999997,-0.063000000000001,-0.047399999999999,-0.0313999999999993,-0.0153999999999996,0.000799999999999557,0.0174000000000007,0.0341999999999999,0.0512,0.0680000000000007,0.0853999999999999,0.1024,0.1196,0.136800000000001,0.1542,0.1714,0.188799999999999,0.205799999999999,0.222799999999999,0.2396,0.256399999999999,0.2732,0.2896,0.3062,0.322600000000001,0.338800000000001,0.355199999999999,0.371600000000001,0.3878,0.4042,0.420400000000001,0.4368,0.4532,0.4294,0.446000000000001,0.4628,0.479599999999999,0.496600000000001,0.5138,0.5312,0.5488,0.566200000000001,0.584,0.601599999999999,0.6194,0.6372,0.654999999999998,0.672799999999999,0.690599999999999,0.7084,0.726000000000001,0.7438,0.7614,0.779000000000001,0.796400000000001,0.813800000000001,0.8312,0.848400000000001,0.865799999999999,0.882799999999999,0.9,0.9172,0.934199999999998,0.951399999999998,0.9684,0.944800000000001,0.961999999999998,0.9792,0.996599999999998,1.0138,1.031,1.0484,1.0656,1.083,1.1004,1.1178,1.135,1.1524,1.1698,1.187,1.2044,1.2216,1.239,1.2562,1.2732,1.2904,1.3076,1.3244,1.3414,1.3586,1.3754,1.3924,1.4092,1.4258,1.4424,1.4588,1.4752,1.4914,1.4726,1.489,1.505,1.521,1.5366,1.552,1.5674,1.5822,1.5968,1.6112,1.6256,1.6394,1.6528,1.6662,1.6792,1.6918,1.7042,1.7162,1.7278,1.7392,1.7504,1.7612,1.7714,1.7816,1.7916,1.8012,1.8106,1.8196,1.8286,1.8372,1.8458,1.8538,1.8382,1.8466,1.8548,1.8628,1.8706,1.878,1.8854,1.8926,1.8996,1.9062,1.9128,1.919,1.925,1.9308,1.9364,1.9418,1.9466,1.9514,1.9562,1.9606,1.9648,1.9684,1.9722,1.9756,1.9788,1.982,1.9848,1.9872,1.9896,1.9914,1.993,1.9942,1.9896,1.9906,1.9914,1.9918,1.9916,1.991,1.99,1.9882,1.9858,1.9832,1.98,1.976,1.9716,1.9668,1.9614,1.9556,1.9494,1.9426,1.9356,1.9282,1.9204,1.9122,1.904,1.8956,1.8868,1.878,1.8692,1.8604,1.8514,1.8424,1.8332,1.824,1.8148,1.828,1.82,1.8118,1.8036,1.795,1.7862,1.7772,1.7676,1.7576,1.7468,1.7356,1.7234,1.7102,1.6958,1.6806,1.6638,1.646,1.6264,1.6052,1.5826,1.5582,1.5324,1.5046,1.4752,1.4444,1.4116,1.3774,1.3416,1.3044,1.2666,1.2276,1.1882,1.1946,1.1556,1.1166,1.078,1.0398,1.0024,0.965800000000002,0.9304,0.896600000000001,0.864400000000001,0.833799999999999,0.805199999999999,0.778399999999999,0.753400000000002,0.730399999999997,0.709,0.689400000000001,0.671600000000001,0.655600000000001,0.641,0.628,0.616200000000001,0.605799999999999,0.596800000000002,0.588800000000001,0.581599999999999,0.5756,0.570400000000001,0.565799999999999,0.5616,0.558,0.554599999999999,0.652799999999999,0.6514,0.6496,0.6476,0.644799999999999,0.6414,0.637,0.6316,0.6248,0.6166,0.606599999999999,0.594799999999999,0.5812,0.5654,0.548,0.5286,0.507600000000001,0.4846,0.4602,0.4342,0.4066,0.3776,0.3476,0.316399999999999,0.2842,0.250999999999999,0.217200000000001,0.182600000000001,0.1476,0.111999999999999,0.0762,0.0399999999999991,0.00380000000000109,0.100999999999999,0.0656000000000006,0.0304000000000002,-0.00439999999999969,-0.0389999999999986,-0.0731999999999999,-0.1068,-0.140000000000001,-0.1724,-0.2042,-0.2352,-0.2652,-0.2944,-0.3228,-0.3498,-0.376,-0.4008,-0.4248,-0.4474,-0.4688,-0.489,-0.5078,-0.5254,-0.5418,-0.5566,-0.5702,-0.5826,-0.5934,-0.6028,-0.6106,-0.617,-0.6222,-0.494,-0.4974,-0.4996,-0.5008,-0.5008,-0.5002,-0.4988,-0.49654,-0.4938,-0.49052,-0.48708,-0.4831,-0.47906,-0.4749,-0.47074,-0.46664,-0.46264,-0.4588,-0.45514,-0.45152,-0.44784,-0.444,-0.4399,-0.43542,-0.43044,-0.42484,-0.41852,-0.411396,-0.403308,-0.394186,-0.3839548,-0.37272,-0.311012,-0.300918,-0.290244,-0.279152,-0.267796,-0.2563494,-0.2449342,-0.233736,-0.222884,-0.21256,-0.202894,-0.194052,-0.186198,-0.179462,-0.17402,-0.16998,-0.16734,-0.17472,-0.18138,-0.18738,-0.19272,-0.19748,-0.20166,-0.2053,-0.20844,-0.21114,-0.2134,-0.21526,-0.21676,-0.21796,-0.21886,-0.2195,-0.21994,-0.2202,-0.2203,-0.2203,-0.22022,-0.2201,-0.21998,-0.21988,-0.21986,-0.21994,-0.22016,-0.22054,-0.22112,-0.22196,-0.22306,-0.22306};

// add function prototypes in here?
static void torqueTracking(); // should return torque value
static int checkImpedanceMode();

#endif /* FLEXSEA_PROJECTS_MIT_DLEG_INC_TORQUE_REPLAY_H_ */
