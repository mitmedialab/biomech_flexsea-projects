 

#include "task_machine.h"

 //Gait event thresholds
#define EXPECTED_SWING_TQ 0.0
#define TRANSITION_TQ_THRESH 7.0
#define MIN_STANCE_TQ EXPECTED_SWING_TQ + TRANSITION_TQ_THRESH + 5.0
#define TRANSITION_POWER_THRESH -0.3
#define PREDICTION_CUTOFF_SAMPLES 200
#define MAX_SWING_TQ_DOT_NM_HZ 100
#define MIN_GAIT_PHASE_SAMPLES 250

//Ideal controller values

#define FL_IDEAL_PEAK_GEN_POWER_W_PER_KG 2.0 //Montgomery 2018
#define UR_IDEAL_PEAK_GEN_POWER_W_PER_KG 3.5 //Montgomery 2018
#define DR_IDEAL_PEAK_GEN_POWER_W_PER_KG 2.0 //Montgomery 2018
#define US_IDEAL_PEAK_GEN_POWER_W_PER_KG 2.56 //Sinitski 2012
#define DS_IDEAL_PEAK_GEN_POWER_W_PER_KG 1.38 //Sinitski 2012
#define NOMINAL_PEAK_GEN_POWER_W_PER_KG 0.0

#define FL_IDEAL_PEAK_DIS_POWER_W_PER_KG 1.0 //Montgomery 2018
#define UR_IDEAL_PEAK_DIS_POWER_W_PER_KG 0.4 //Montgomery 2018
#define DR_IDEAL_PEAK_DIS_POWER_W_PER_KG 1.8 //Montgomery 2018
#define US_IDEAL_PEAK_DIS_POWER_W_PER_KG 0.63 //Sinitski 2012
#define DS_IDEAL_PEAK_DIS_POWER_W_PER_KG 3.01 //Sinitski 2012
#define NOMINAL_PEAK_DIS_POWER_W_PER_KG 0.0

#define FL_IDEAL_NET_WORK_J_PER_KG 0.051 //Montgomery 2018
#define UR_IDEAL_NET_WORK_J_PER_KG 0.45 //Montgomery 2018
#define DR_IDEAL_NET_WORK_J_PER_KG -0.29 //Montgomery 2018
#define US_IDEAL_NET_WORK_J_PER_KG 0.39 //Sinitski 2012
#define DS_IDEAL_NET_WORK_J_PER_KG -0.56 //Sinitski 2012
#define NOMINAL_NET_WORK_J_PER_KG 0.0

#define FL_IDEAL_PEAK_PLANTAR_TQ_NM_PER_KG 1.5 //Montgomery 2018
#define UR_IDEAL_PEAK_PLANTAR_TQ_NM_PER_KG 1.7 //Montgomery 2018
#define DR_IDEAL_PEAK_PLANTAR_TQ_NM_PER_KG 1.0 //Montgomery 2018
#define US_IDEAL_PEAK_PLANTAR_TQ_NM_PER_KG 1.45 //Sinitski 2012
#define DS_IDEAL_PEAK_PLANTAR_TQ_NM_PER_KG 1.38 //Sinitski 2012
#define NOMINAL_PEAK_PLANTAR_TQ_NM_PER_KG 0.0

#define FL_IDEAL_FOOT_STRIKE_ANG_RAD 0 //Montgomery 2018
#define UR_IDEAL_FOOT_STRIKE_ANG_RAD -0.11 //Montgomery 2018
#define DR_IDEAL_FOOT_STRIKE_ANG_RAD 0.06 //Montgomery 2018
#define US_IDEAL_FOOT_STRIKE_ANG_RAD -0.26 //Sinitski 2012
#define DS_IDEAL_FOOT_STRIKE_ANG_RAD 0.48 //Sinitski 2012
#define NOMINAL_FOOT_STRIKE_ANG_RAD 0.0


static struct taskmachine_s tm;
static int task_machine_demux_state = INIT_TASK_MACHINE;
#if defined(NO_DEVICE)
static float tqraw;
static float aaraw;
float example_stride_aa[] = {0.15, 0.16, 0.16, 0.16, 0.16, 0.16, 0.16, 0.16, 0.16, 0.16, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.19, 0.19, 0.51, 0.51, 0.51, 0.51, 0.72, 0.72, 0.72, 0.72, 0.84, 0.84, 0.84, 0.84, 0.84, 0.84, 0.84, 0.86, 0.86, 0.86, 0.86, 0.86, 0.86, 0.86, 0.83, 0.83, 0.83, 0.77, 0.77, 0.77, 0.77, 0.73, 0.66, 0.66, 0.66, 0.58, 0.48, 0.48, 0.48, 0.48, 0.48, 0.48, 0.48, 0.48, 0.46, 0.50, 0.50, 0.50, 0.50, 0.50, 0.55, 0.61, 0.61, 0.61, 0.61, 0.61, 0.61, 0.61, 0.61, 0.61, 0.68, 0.68, 0.68, 0.68, 0.76, 0.76, 0.76, 0.76, 0.76, 0.76, 0.76, 0.95, 0.95, 0.95, 0.95, 0.84, 0.62, 0.62, 0.62, 0.62, 0.63, 0.63, 0.63, 0.63, 0.63, 0.63, 0.60, 0.60, 0.60, 0.60, 0.53, 0.53, 0.53, 0.53, 0.43, 0.31, 0.31, 0.31, 0.31, 0.31, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.06, 0.06, 0.06, 0.06, 0.06, -0.09, -0.15, -0.15, -0.15, -0.15, -0.15, -0.15, -0.20, -0.39, -0.39, -0.39, -0.46, -0.46, -0.52, -0.52, -0.52, -0.52, -0.52, -0.52, -0.68, -0.68, -0.68, -0.75, -0.75, -0.75, -0.83, -0.83, -0.83, -0.83, -0.83, -0.83, -0.83, -0.83, -0.83, -0.83, -0.92, -0.92, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.19, -1.19, -1.19, -1.27, -1.37, -1.37, -1.65, -1.65, -1.65, -1.65, -1.75, -1.75, -1.75, -1.75, -1.75, -1.75, -1.75, -1.75, -1.94, -2.04, -2.04, -2.04, -2.13, -2.13, -2.13, -2.13, -2.13, -2.13, -2.13, -2.22, -2.22, -2.22, -2.22, -2.22, -2.30, -2.30, -2.30, -2.30, -2.30, -2.30, -2.30, -2.37, -2.37, -2.50, -2.50, -2.50, -2.50, -2.61, -2.61, -2.68, -2.70, -2.70, -2.70, -2.72, -2.72, -2.72, -2.72, -2.72, -2.72, -2.72, -2.72, -2.73, -2.74, -2.74, -2.74, -2.74, -2.74, -2.74, -2.74, -2.74, -2.74, -2.74, -2.74, -2.74, -2.74, -2.74, -2.73, -2.73, -2.73, -2.73, -2.73, -2.73, -2.73, -2.71, -2.71, -2.71, -2.71, -2.71, -2.66, -2.66, -2.66, -2.66, -2.63, -2.59, -2.56, -2.56, -2.56, -2.54, -2.54, -2.54, -2.54, -2.54, -2.54, -2.54, -2.54, -2.51, -2.51, -2.51, -2.51, -2.51, -2.51, -2.46, -2.46, -2.46, -2.46, -2.46, -2.46, -2.46, -2.46, -2.45, -2.45, -2.45, -2.45, -2.44, -2.44, -2.44, -2.44, -2.44, -2.41, -2.41, -2.41, -2.41, -2.41, -2.41, -2.41, -2.40, -2.40, -2.41, -2.41, -2.41, -2.41, -2.41, -2.41, -2.41, -2.41, -2.41, -2.41, -2.43, -2.43, -2.43, -2.46, -2.46, -2.46, -2.48, -2.48, -2.48, -2.51, -2.51, -2.51, -2.51, -2.51, -2.51, -2.51, -2.51, -2.54, -2.54, -2.54, -2.54, -2.54, -2.54, -2.54, -2.54, -2.60, -2.60, -2.60, -2.60, -2.60, -2.60, -2.64, -2.64, -2.75, -2.75, -2.79, -2.79, -2.79, -2.83, -2.83, -2.83, -2.89, -2.89, -2.89, -2.89, -2.97, -2.97, -2.97, -2.97, -2.97, -2.97, -2.97, -2.97, -2.97, -2.97, -2.97, -3.01, -3.01, -3.01, -3.04, -3.04, -3.04, -3.04, -3.04, -3.04, -3.10, -3.10, -3.10, -3.10, -3.10, -3.10, -3.17, -3.17, -3.29, -3.29, -3.29, -3.29, -3.35, -3.39, -3.39, -3.39, -3.39, -3.39, -3.39, -3.46, -3.46, -3.46, -3.46, -3.46, -3.51, -3.56, -3.56, -3.56, -3.56, -3.62, -3.62, -3.62, -3.62, -3.62, -3.67, -3.67, -3.67, -3.67, -3.67, -3.67, -3.67, -3.67, -3.67, -3.67, -3.71, -3.77, -3.77, -3.77, -3.77, -3.77, -3.83, -3.83, -3.88, -4.03, -4.03, -4.08, -4.08, -4.08, -4.08, -4.12, -4.12, -4.12, -4.12, -4.23, -4.23, -4.23, -4.23, -4.23, -4.23, -4.23, -4.23, -4.23, -4.23, -4.28, -4.28, -4.28, -4.32, -4.32, -4.38, -4.38, -4.38, -4.38, -4.43, -4.43, -4.43, -4.43, -4.43, -4.43, -4.43, -4.54, -4.58, -4.58, -4.58, -4.58, -4.58, -4.58, -4.66, -4.82, -4.82, -4.82, -4.82, -4.87, -4.87, -4.87, -4.87, -4.87, -4.87, -4.87, -4.99, -4.99, -4.99, -4.99, -4.99, -5.05, -5.05, -5.11, -5.11, -5.11, -5.16, -5.16, -5.16, -5.16, -5.16, -5.16, -5.16, -5.16, -5.21, -5.21, -5.21, -5.21, -5.21, -5.21, -5.21, -5.26, -5.26, -5.26, -5.26, -5.26, -5.26, -5.26, -5.26, -5.43, -5.43, -5.59, -5.59, -5.66, -5.66, -5.66, -5.66, -5.66, -5.79, -5.79, -5.79, -5.79, -5.79, -5.79, -5.85, -5.92, -5.92, -5.92, -5.92, -5.99, -5.99, -5.99, -5.99, -5.99, -5.99, -6.06, -6.06, -6.06, -6.06, -6.12, -6.12, -6.12, -6.12, -6.12, -6.27, -6.27, -6.27, -6.27, -6.27, -6.27, -6.33, -6.41, -6.54, -6.54, -6.54, -6.54, -6.61, -6.61, -6.69, -6.69, -6.69, -6.76, -6.76, -6.84, -6.91, -6.91, -6.91, -6.91, -6.98, -6.98, -7.06, -7.06, -7.06, -7.06, -7.06, -7.06, -7.06, -7.12, -7.12, -7.12, -7.12, -7.19, -7.19, -7.19, -7.19, -7.19, -7.19, -7.19, -7.19, -7.34, -7.34, -7.34, -7.34, -7.34, -7.50, -7.50, -7.50, -7.69, -7.69, -7.76, -7.76, -7.76, -7.76, -7.83, -7.83, -7.83, -7.83, -7.83, -7.97, -7.97, -7.97, -7.97, -7.97, -7.97, -7.97, -8.04, -8.04, -8.10, -8.10, -8.17, -8.17, -8.17, -8.17, -8.17, -8.17, -8.17, -8.23, -8.23, -8.23, -8.23, -8.23, -8.30, -8.30, -8.30, -8.30, -8.45, -8.45, -8.45, -8.45, -8.45, -8.45, -8.59, -8.59, -8.83, -8.83, -8.83, -8.91, -8.91, -8.91, -8.91, -8.91, -8.91, -8.91, -8.99, -9.13, -9.13, -9.13, -9.13, -9.13, -9.13, -9.20, -9.27, -9.27, -9.27, -9.27, -9.27, -9.27, -9.27, -9.27, -9.27, -9.27, -9.34, -9.34, -9.43, -9.43, -9.43, -9.43, -9.43, -9.43, -9.43, -9.43, -9.50, -9.50, -9.50, -9.50, -9.50, -9.75, -9.75, -9.98, -9.98, -9.98, -10.05, -10.05, -10.05, -10.05, -10.14, -10.14, -10.14, -10.22, -10.22, -10.31, -10.31, -10.31, -10.31, -10.39, -10.48, -10.48, -10.48, -10.48, -10.48, -10.48, -10.48, -10.48, -10.48, -10.48, -10.48, -10.48, -10.57, -10.57, -10.65, -10.65, -10.65, -10.65, -10.65, -10.82, -11.32, -11.32, -11.32, -11.32, -11.40, -11.48, -11.48, -11.48, -11.71, -11.71, -11.79, -11.79, -11.79, -11.79, -11.79, -11.79, -11.79, -11.87, -11.87, -11.87, -11.87, -11.87, -11.87, -11.87, -11.87, -11.87, -11.87, -11.87, -11.87, -11.87, -11.95, -11.95, -11.95, -11.95, -11.95, -11.95, -11.95, -12.10, -12.10, -12.10, -12.10, -12.10, -12.10, -12.23, -12.23, -12.23, -12.23, -12.23, -12.23, -12.23, -12.23, -12.23, -12.23, -12.30, -12.30, -12.36, -12.36, -12.36, -12.36, -12.36, -12.36, -12.36, -12.36, -12.36, -12.36, -12.42, -12.46, -12.46, -12.46, -12.46, -12.46, -12.46, -12.53, -12.53, -12.59, -12.59, -12.59, -12.59, -12.61, -12.61, -12.61, -12.63, -12.63, -12.63, -12.63, -12.67, -12.67, -12.67, -12.67, -12.67, -12.67, -12.68, -12.67, -12.67, -12.67, -12.67, -12.67, -12.67, -12.67, -12.67, -12.67, -12.67, -12.67, -12.65, -12.65, -12.65, -12.60, -12.60, -12.60, -12.60, -12.60, -12.52, -12.52, -12.52, -12.52, -12.52, -12.46, -12.38, -12.13, -12.13, -12.13, -12.13, -12.13, -12.03, -11.92, -11.92, -11.92, -11.92, -11.92, -11.69, -11.69, -11.69, -11.56, -11.56, -11.56, -11.56, -11.42, -11.42, -11.28, -11.28, -11.28, -11.28, -11.28, -11.28, -11.28, -11.28, -11.28, -11.28, -11.13, -11.13, -11.13, -10.97, -10.97, -10.97, -10.97, -10.97, -10.65, -10.65, -10.65, -10.46, -10.46, -10.46, -10.46, -10.26, -10.26, -9.65, -9.65, -9.65, -9.43, -9.43, -9.43, -9.20, -9.20, -9.20, -9.20, -9.20, -9.20, -8.96, -8.96, -8.96, -8.96, -8.96, -8.96, -8.46, -8.46, -8.46, -8.19, -8.19, -8.19, -8.19, -8.19, -8.19, -8.19, -8.19, -8.19, -7.92, -7.92, -7.92, -7.92, -7.92, -7.92, -7.64, -7.64, -7.64, -7.64, -7.64, -7.64, -6.69, -6.69, -5.68, -5.68, -5.39, -5.39, -5.39, -5.39, -5.39, -5.15, -5.15, -5.15, -5.15, -5.15, -5.15, -5.15, -5.15, -4.66, -4.13, -4.13, -4.13, -4.13, -4.13, -4.13, -4.13, -4.13, -4.13, -4.13, -3.85, -3.85, -3.85, -3.85, -3.59, -3.59, -3.59, -3.59, -3.59, -3.59, -3.33, -3.33, -3.33, -3.33, -3.33, -3.06, -3.06, -2.03, -1.79, -1.79, -1.79, -1.79, -1.55, -1.55, -1.55, -1.55, -1.55, -1.55, -1.55, -1.55, -1.55, -0.93, -0.76, -0.76, -0.76, -0.76, -0.76, -0.76, -0.76, -0.76, -0.76, -0.76, -0.63, -0.63, -0.63, -0.63, -0.49, -0.49, -0.49, -0.49, -0.49, -0.49, -0.49, -0.49, -0.49, -0.11, -0.11, 0.13, 0.13, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.23, 0.23, 0.32, 0.32, 0.32, 0.32, 0.32, 0.32, 0.32, 0.32, 0.32, 0.34, 0.34, 0.34, 0.34, 0.34, 0.34, 0.34, 0.34, 0.34, 0.34, 0.34, 0.34, 0.34, 0.34, 0.34, 0.35, 0.35, 0.35, 0.35, 0.34, 0.34, 0.33, 0.33, 0.33, 0.33, 0.33, 0.33, 0.33, 0.33, 0.33, 0.33, 0.33, 0.33, 0.30, 0.30, 0.30, 0.30, 0.30, 0.30, 0.30, 0.30, 0.30, 0.28, 0.28, 0.28, 0.28, 0.28, 0.28, 0.28, 0.28, 0.28, 0.28, 0.28, 0.28, 0.28, 0.28, 0.28, 0.28, 0.25, 0.24, 0.24, 0.24, 0.24, 0.24, 0.24, 0.24, 0.24, 0.24, 0.24, 0.24, 0.24, 0.24, 0.24, 0.24, 0.24, 0.24, 0.24, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.24, 0.24, 0.24, 0.24, 0.24, 0.24, 0.24, 0.24, 0.24, 0.24, 0.24, 0.22, 0.22, 0.22, 0.23, 0.23, 0.23, 0.23, 0.23, 0.23, 0.23, 0.23, 0.23, 0.22, 0.22, 0.22, 0.22, 0.22, 0.22, 0.22, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.19, 0.19, 0.19, 0.19, 0.19, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.21, 0.21, 0.21, 0.21, 0.21, 0.19, 0.19, 0.19, 0.19, 0.19, 0.19, 0.19, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.19, 0.19, 0.19, 0.19, 0.19, 0.19, 0.19, 0.19, 0.19, 0.19, 0.19, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.17, 0.17, 0.17, 0.16, 0.16, 0.16, 0.16, 0.16, 0.16, 0.16, 0.16, 0.16, 0.16, 0.16, 0.16, 0.16, 0.16, 0.16, 0.16, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.16, 0.16, 0.16, 0.16, 0.16, 0.16, 0.16, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.16, 0.16, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17};
#endif
static float ideal_peak_gen_power_w_per_kg[] = {FL_IDEAL_PEAK_GEN_POWER_W_PER_KG,UR_IDEAL_PEAK_GEN_POWER_W_PER_KG,DR_IDEAL_PEAK_GEN_POWER_W_PER_KG,US_IDEAL_PEAK_GEN_POWER_W_PER_KG,DS_IDEAL_PEAK_GEN_POWER_W_PER_KG, NOMINAL_PEAK_GEN_POWER_W_PER_KG};
static float ideal_peak_dis_power_w_per_kg[] = {FL_IDEAL_PEAK_DIS_POWER_W_PER_KG,UR_IDEAL_PEAK_DIS_POWER_W_PER_KG,DR_IDEAL_PEAK_DIS_POWER_W_PER_KG,US_IDEAL_PEAK_DIS_POWER_W_PER_KG,DS_IDEAL_PEAK_DIS_POWER_W_PER_KG, NOMINAL_PEAK_DIS_POWER_W_PER_KG};
static float ideal_net_work_j_per_kg[] = {FL_IDEAL_NET_WORK_J_PER_KG,UR_IDEAL_NET_WORK_J_PER_KG,DR_IDEAL_NET_WORK_J_PER_KG,US_IDEAL_NET_WORK_J_PER_KG,DS_IDEAL_NET_WORK_J_PER_KG, NOMINAL_NET_WORK_J_PER_KG};
static float ideal_peak_plantar_tq_nm_per_kg[] = {FL_IDEAL_PEAK_PLANTAR_TQ_NM_PER_KG,UR_IDEAL_PEAK_PLANTAR_TQ_NM_PER_KG,DR_IDEAL_PEAK_PLANTAR_TQ_NM_PER_KG,US_IDEAL_PEAK_PLANTAR_TQ_NM_PER_KG,DS_IDEAL_PEAK_PLANTAR_TQ_NM_PER_KG, NOMINAL_PEAK_PLANTAR_TQ_NM_PER_KG};
static float ideal_foot_strike_ang_rad[] = {FL_IDEAL_FOOT_STRIKE_ANG_RAD,UR_IDEAL_FOOT_STRIKE_ANG_RAD,DR_IDEAL_FOOT_STRIKE_ANG_RAD,US_IDEAL_FOOT_STRIKE_ANG_RAD,DS_IDEAL_FOOT_STRIKE_ANG_RAD, NOMINAL_FOOT_STRIKE_ANG_RAD};

static float aa_dot_15hz_filt_outputs[] = {0,0,0,0,0};
static float aa_dot_15hz_filt_inputs[] = {0,0,0,0,0};

static float tq_dot_outputs[] = {0,0,0};
static float tq_dot_inputs[] = {0,0,0};
static float aa_dot_outputs[] = {0,0,0};
static float aa_dot_inputs[] = {0,0,0};

//static float torque_replay_70kg[] = {-4.001649, -4.852673, -11.852532, -14.614554, -9.826756, -8.796879, -5.918001, -0.888233, 2.200429, 5.075491, 8.681907, 12.352409, 15.634863, 19.046701, 22.175614, 25.030629, 27.762032, 30.134923, 32.248197, 34.176628, 35.853156, 37.183527, 38.377596, 39.389298, 40.306929, 41.285400, 42.317727, 43.574136, 45.059061, 46.821967, 48.937716, 51.406733, 54.387221, 57.872666, 61.833998, 66.243208, 71.179092, 76.448118, 81.949901, 87.699807, 93.607025, 99.355844, 104.744118, 109.791961, 113.980215, 117.084070, 118.982611, 119.405260, 117.985780, 114.244175, 107.925739, 100.368482, 90.742188, 76.392282, 59.252296, 40.923735, 23.919683, 10.275839, 0.948522, -2.241754, -2.738065, -2.837141, -2.723922, -2.406633, -1.957389, -1.487492, -1.102087, -0.857047, -0.747952, -0.729212, -0.747070, -0.765693, -0.773913, -0.776564, -0.783944, -0.804860, -0.842396, -0.893250, -0.949676, -1.004377, -1.055209, -1.105390, -1.161034, -1.226084, -1.294344, -1.346250, -1.355576, -1.296739, -1.149954, -0.909682, -0.587207, -0.206126, 0.196165, 0.580245, 0.911793, 1.160153, 1.293800, 1.279092, 1.080700, 0.663608, 0.016374};
//static float stance_torque_replay_70kg[] = {-4.001649, -4.852673, -11.852532, -14.614554, -9.826756, -8.796879, -5.918001, -0.888233, 2.200429, 5.075491, 8.681907, 12.352409, 15.634863, 19.046701, 22.175614, 25.030629, 27.762032, 30.134923, 32.248197, 34.176628, 35.853156, 37.183527, 38.377596, 39.389298, 40.306929, 41.285400, 42.317727, 43.574136, 45.059061, 46.821967, 48.937716, 51.406733, 54.387221, 57.872666, 61.833998, 66.243208, 71.179092, 76.448118, 81.949901, 87.699807, 93.607025, 99.355844, 104.744118, 109.791961, 113.980215, 117.084070, 118.982611, 119.405260, 117.985780, 114.244175, 107.925739, 100.368482, 90.742188, 76.392282, 59.252296, 40.923735, 23.919683, 10.275839};
//static int torque_replay_transition_gait_cycle_percent = 57;
//static float prev_stride_tics = 1000.0;
//static float prev_stance_tics = 600.0;
//static float subject_weight_kg = 9.1;

//Copied from matlab pil simulation
static void init_task_machine(){
	tm.control_mode = MODE_NOMINAL;
	tm.learning_enabled = 0;
	tm.adaptation_enabled = 0;
	tm.trial_type = 0;

    tm.elapsed_samples = 0.0;
    tm.latest_foot_off_samples = 10000.0;
    tm.prev_stride_samples = 0.0;
    tm.in_swing = 0;
    tm.do_learning_for_curr_stride = 0;
    tm.do_learning_for_prev_stride = 0;
    tm.passed_min_stance_tq = 0;

    tm.gait_event_trigger = 0;  
    tm.reset_back_estimator_trigger = 0;
    tm.stride_classified = 0;

    tm.tq = 0.0;
    tm.tq_dot = 0.0;
    tm.aa = 0.0;
    tm.aa_dot = 0.0;
    tm.aa_dot_15hz_filt = 0.0;

    tm.tq_prev = 0.0;
    tm.aa_prev = 0.0;

    tm.net_work_j = 0.0;
    tm.peak_power_w = 0.0;
    tm.min_power_w = 0.0;
    tm.stance_rom_rad = 0.0;
    tm.heelstrike_angle_rad = 0.0;

    tm.net_work_error_j_p_kg = (float*)calloc(N_CLASSES+1, sizeof(float));
    tm.stance_rom_error_rad = (float*)calloc(N_CLASSES+1, sizeof(float));
    tm.heelstrike_angle_error_rad = (float*)calloc(N_CLASSES+1, sizeof(float));

    tm.subject_mass_kg = 82.0;

}



static void update_gait_events(Act_s* actx){

    tm.gait_event_trigger = GAIT_EVENT_DEFAULT;

    if (tm.in_swing){

          if (tm.elapsed_samples - tm.latest_foot_off_samples == PREDICTION_CUTOFF_SAMPLES){
              tm.do_learning_for_curr_stride = 1;
              tm.gait_event_trigger = GAIT_EVENT_WINDOW_CLOSE; 
              return;  
          }

          if(tm.elapsed_samples - tm.latest_foot_off_samples < MIN_GAIT_PHASE_SAMPLES){
          	  return;
          }

            //Swing to stance transition condition
          if (fabs(tm.tq_dot) >= MAX_SWING_TQ_DOT_NM_HZ && tm.power_w < TRANSITION_POWER_THRESH){
        	  tm.prev_stride_samples = tm.elapsed_samples;
              tm.elapsed_samples = 0;
              tm.in_swing = 0;
              tm.gait_event_trigger = GAIT_EVENT_FOOT_ON;
              tm.passed_min_stance_tq = 0;
          }
    }else{

    	if (tm.elapsed_samples < MIN_GAIT_PHASE_SAMPLES){
			return;
    	}
		if (tm.tq > MIN_STANCE_TQ){
			tm.passed_min_stance_tq = 1;
		}


		if (fabs(EXPECTED_SWING_TQ - tm.tq) < TRANSITION_TQ_THRESH &&
		  tm.passed_min_stance_tq){
			tm.in_swing = 1;
			tm.latest_foot_off_samples = tm.elapsed_samples;
			tm.do_learning_for_prev_stride = tm.do_learning_for_curr_stride;
			tm.do_learning_for_curr_stride = 0;
			tm.gait_event_trigger = GAIT_EVENT_FOOT_OFF;
			tm.stride_classified = 0;
		}
      }

}

#if defined(NO_DEVICE)
static void simulate_ankle_torque(){
	static int stride_iterator = 0;
  tqraw = example_stride_tq[stride_iterator];
  aaraw = example_stride_aa[stride_iterator];
  stride_iterator++;
  if (stride_iterator == 1431)
    stride_iterator = 0;
}
#endif


static void update_ankle_dynamics(Act_s* actx)
{
    tm.tq_prev = tm.tq;
    tm.aa_prev = tm.aa;
    //
	#if defined(NO_DEVICE)
    simulate_ankle_torque();
    tm.tq = tqraw; //for debug with no actuator
    tm.aa = aaraw;
	#else
    tm.tq = actx->jointTorque;
    tm.aa = actx->jointAngle;
	#endif

    tm.tq_dot = filter_second_order_butter_50hz(SAMPLE_RATE_HZ*(tm.tq - tm.tq_prev), &tq_dot_outputs[0], &tq_dot_inputs[0]);
    tm.aa_dot = filter_second_order_butter_50hz(SAMPLE_RATE_HZ*(tm.aa - tm.aa_prev), &aa_dot_outputs[0], &aa_dot_inputs[0]);

   tm.aa_dot_15hz_filt = filter_fourth_order_butter_15hz( tm.aa_dot, &aa_dot_15hz_filt_outputs[0], &aa_dot_15hz_filt_inputs[0]);

    tm.power_w = tm.tq*tm.aa_dot;
    tm.net_work_j = tm.net_work_j + tm.power_w*SAMPLE_PERIOD_S;
	if (tm.peak_power_w < tm.power_w){
		tm.peak_power_w = tm.power_w;
	}
    if (tm.gait_event_trigger == GAIT_EVENT_FOOT_ON){
        tm.net_work_j = 0.0;
		tm.peak_power_w = -FLT_MAX;
    }

}

struct taskmachine_s* get_task_machine(){
  return &tm;
}

float get_ideal_peak_gen_power(){
	return ideal_peak_gen_power_w_per_kg[tm.current_terrain];
}
float get_ideal_peak_dis_power(){
	return ideal_peak_dis_power_w_per_kg[tm.current_terrain];
}
float get_ideal_net_work(){
	return ideal_net_work_j_per_kg[tm.current_terrain];
}
float get_ideal_peak_plantar_torque(){
	return ideal_peak_plantar_tq_nm_per_kg[tm.current_terrain];
}
float get_ideal_ankle_angle(){
	return ideal_foot_strike_ang_rad[tm.current_terrain];
}

void task_machine_demux(struct rigid_s* rigid, Act_s* actx){

  
  switch (task_machine_demux_state){
    case INIT_TASK_MACHINE:
        init_task_machine();
        init_back_estimator();
        task_machine_demux_state = INIT_LEARNING;
    break;
    case INIT_LEARNING:
        init_learning_structs();
        task_machine_demux_state = INIT_KINEMATICS;
    break;
    case INIT_KINEMATICS:
        init_kinematics();
        task_machine_demux_state = INIT_TERRAIN_STATE_MACHINE;
    break;
    case INIT_TERRAIN_STATE_MACHINE:
            init_terrain_state_machine();
            task_machine_demux_state = RUN_TASK_MACHINE;
        break;
    case RUN_TASK_MACHINE:
    	update_ankle_dynamics(actx);
    	update_gait_events(actx);

		update_kinematics(&rigid->mn,&tm);
		update_statistics_demux(&tm, get_kinematics());


		if (tm.learning_enabled){

			update_learner_demux();
		}

		predict_task_demux(&tm, get_kinematics());

		update_back_estimation_features(&tm, get_kinematics());
		update_prediction_features(&tm, get_kinematics());


		if (tm.adaptation_enabled)
			if (tm.control_mode != MODE_HEURISTIC)
				tm.current_terrain = get_predictor()->k_pred;
			else{
				if (get_back_estimator()->curr_stride_paz_thresh_status == PAZ_PASSED_US_THRESH)
					tm.current_terrain = K_USTAIRS;
				else if (get_back_estimator()->curr_stride_paz_thresh_status == PAZ_PASSED_DS_THRESH)
					tm.current_terrain = K_DSTAIRS;
				else
					tm.current_terrain = K_FLAT;
			}

		else{
			if (tm.control_mode != MODE_HEURISTIC)
				tm.current_terrain = tm.control_mode;
			else
				tm.current_terrain = MODE_NOMINAL;
		}

		terrain_state_machine_demux(&tm, rigid, actx, tm.current_terrain);


    	tm.elapsed_samples = tm.elapsed_samples + 1.0;
    	break;
  }

  

}
