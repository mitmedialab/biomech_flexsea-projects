

#ifndef __WSM_ROMAN_H__
#define __WSM_ROMAN_H__

#include "imu.h"
#include <math.h>
#include "flexsea_user_structs.h"
#include "task_machine.h"
#include <limits.h>
#include "actuator_functions.h"


#define DEFAULT_MAX_SAFE_B_NM_P_RPS 15.0
#define DEFAULT_SW_K_NM_P_RAD 160.0
#define DEFAULT_SW_B_NM_P_RPS 8.5
#define DEFAULT_SW_DELAY_TICS 50.0

#define DEFAULT_NOMINAL_K_NM_P_RAD 300.0
#define DEFAULT_NOMINAL_B_NM_P_RPS 6.0
#define DEFAULT_NOMINAL_THETA_RAD 0.0
#define DEFAULT_DESIRED_TRAJECTORY_PERIOD_S 0.250

#define DEFAULT_ESW_THETA_RAD 0.0


#ifdef USER_JG
#define DEFAULT_FLAT_HS_THETA_RAD 0.0
#define DEFAULT_FLAT_HS_K_NM_P_RAD 100.0
#define DEFAULT_FLAT_LSW_THETA_RAD -0.087
#define DEFAULT_FLAT_ST_B_NM_P_RPS 10.0
#define DEFAULT_FLAT_EST_K_NM_P_RAD 300.0
#define DEFAULT_FLAT_LST_K_NM_P_RAD 200.0
#define DEFAULT_FLAT_LST_THETA_RAD 0.244
#define DEFAULT_FLAT_LST_ENGAGEMENT_TQ_NM 35.0
#define DEFAULT_FLAT_LST_DELAY_TICS 70.0

#define DEFAULT_URAMP_HS_THETA_RAD -0.19
#define DEFAULT_URAMP_HS_K_NM_P_RAD 350.0
#define DEFAULT_URAMP_LSW_THETA_RAD -0.14
#define DEFAULT_URAMP_ST_B_NM_P_RPS 8.0
#define DEFAULT_URAMP_EST_K_NM_P_RAD 300.0
#define DEFAULT_URAMP_LST_K_NM_P_RAD 400.0
#define DEFAULT_URAMP_LST_THETA_RAD 0.3
#define DEFAULT_URAMP_LST_ENGAGEMENT_TQ_NM 35.0
#define DEFAULT_URAMP_LST_DELAY_TICS 150.0

#define DEFAULT_DRAMP_HS_THETA_RAD 0.0
#define DEFAULT_DRAMP_HS_K_NM_P_RAD 800.0
#define DEFAULT_DRAMP_LSW_THETA_RAD 0.0
#define DEFAULT_DRAMP_ST_B_NM_P_RPS 8.0
#define DEFAULT_DRAMP_EST_K_NM_P_RAD 400.0
#define DEFAULT_DRAMP_LST_K_NM_P_RAD 400.0
#define DEFAULT_DRAMP_LST_THETA_RAD 0.244
#define DEFAULT_DRAMP_LST_ENGAGEMENT_TQ_NM 20.0
#define DEFAULT_DRAMP_LST_DELAY_TICS 100.0

#define DEFAULT_USTAIRS_HS_THETA_RAD 0.2
#define DEFAULT_USTAIRS_HS_K_NM_P_RAD 50.0
#define DEFAULT_USTAIRS_LSW_THETA_RAD 0.2
#define DEFAULT_USTAIRS_ST_B_NM_P_RPS 5.0
#define DEFAULT_USTAIRS_EST_K_NM_P_RAD 50.0
#define DEFAULT_USTAIRS_LST_K_NM_P_RAD 300.0
#define DEFAULT_USTAIRS_LST_THETA_RAD 0.5
#define DEFAULT_USTAIRS_LST_ENGAGEMENT_TQ_NM 20.0
#define DEFAULT_USTAIRS_LST_DELAY_TICS 300.0

#define DEFAULT_DSTAIRS_HS_THETA_RAD 0.0
#define DEFAULT_DSTAIRS_HS_K_NM_P_RAD 20.0
#define DEFAULT_DSTAIRS_LSW_THETA_RAD 0.5
#define DEFAULT_DSTAIRS_ST_B_NM_P_RPS 12.0
#define DEFAULT_DSTAIRS_EST_K_NM_P_RAD 75.0
#define DEFAULT_DSTAIRS_LST_K_NM_P_RAD 0.0
#define DEFAULT_DSTAIRS_LST_THETA_RAD 0.0
#define DEFAULT_DSTAIRS_LST_ENGAGEMENT_TQ_NM 1000.0
#define DEFAULT_DSTAIRS_LST_DELAY_TICS 70.0
//remember that swing delay tics here were 10
#endif

#ifdef USER_RC
#define DEFAULT_FLAT_HS_THETA_RAD 0.0
#define DEFAULT_FLAT_HS_K_NM_P_RAD 100.0
#define DEFAULT_FLAT_LSW_THETA_RAD -0.087
#define DEFAULT_FLAT_ST_B_NM_P_RPS 10.0
#define DEFAULT_FLAT_EST_K_NM_P_RAD 400.0
#define DEFAULT_FLAT_LST_K_NM_P_RAD 300.0
#define DEFAULT_FLAT_LST_THETA_RAD 0.244
#define DEFAULT_FLAT_LST_ENGAGEMENT_TQ_NM 20.0
#define DEFAULT_FLAT_LST_DELAY_TICS 250.0

#define DEFAULT_URAMP_HS_THETA_RAD -0.19
#define DEFAULT_URAMP_HS_K_NM_P_RAD 350.0
#define DEFAULT_URAMP_LSW_THETA_RAD -0.14
#define DEFAULT_URAMP_ST_B_NM_P_RPS 8.0
#define DEFAULT_URAMP_EST_K_NM_P_RAD 300.0
#define DEFAULT_URAMP_LST_K_NM_P_RAD 400.0
#define DEFAULT_URAMP_LST_THETA_RAD 0.3
#define DEFAULT_URAMP_LST_ENGAGEMENT_TQ_NM 35.0
#define DEFAULT_URAMP_LST_DELAY_TICS 150.0

#define DEFAULT_DRAMP_HS_THETA_RAD 0.0
#define DEFAULT_DRAMP_HS_K_NM_P_RAD 800.0
#define DEFAULT_DRAMP_LSW_THETA_RAD 0.0
#define DEFAULT_DRAMP_ST_B_NM_P_RPS 8.0
#define DEFAULT_DRAMP_EST_K_NM_P_RAD 400.0
#define DEFAULT_DRAMP_LST_K_NM_P_RAD 400.0
#define DEFAULT_DRAMP_LST_THETA_RAD 0.244
#define DEFAULT_DRAMP_LST_ENGAGEMENT_TQ_NM 20.0
#define DEFAULT_DRAMP_LST_DELAY_TICS 100.0

#define DEFAULT_USTAIRS_HS_THETA_RAD 0.2
#define DEFAULT_USTAIRS_HS_K_NM_P_RAD 150.0
#define DEFAULT_USTAIRS_LSW_THETA_RAD 0.2
#define DEFAULT_USTAIRS_ST_B_NM_P_RPS 5.0
#define DEFAULT_USTAIRS_EST_K_NM_P_RAD 50.0
#define DEFAULT_USTAIRS_LST_K_NM_P_RAD 100.0
#define DEFAULT_USTAIRS_LST_THETA_RAD 0.5
#define DEFAULT_USTAIRS_LST_ENGAGEMENT_TQ_NM 10.0
#define DEFAULT_USTAIRS_LST_DELAY_TICS 300.0

#define DEFAULT_DSTAIRS_HS_THETA_RAD 0.0
#define DEFAULT_DSTAIRS_HS_K_NM_P_RAD 20.0
#define DEFAULT_DSTAIRS_LSW_THETA_RAD 0.5
#define DEFAULT_DSTAIRS_ST_B_NM_P_RPS 12.0
#define DEFAULT_DSTAIRS_EST_K_NM_P_RAD 150.0
#define DEFAULT_DSTAIRS_LST_K_NM_P_RAD 0.0
#define DEFAULT_DSTAIRS_LST_THETA_RAD 0.0
#define DEFAULT_DSTAIRS_LST_ENGAGEMENT_TQ_NM 1000.0
#define DEFAULT_DSTAIRS_LST_DELAY_TICS 70.0
#endif

#ifdef USER_PD
#define DEFAULT_FLAT_HS_THETA_RAD 0.0
#define DEFAULT_FLAT_HS_K_NM_P_RAD 350.0
#define DEFAULT_FLAT_LSW_THETA_RAD -0.1
#define DEFAULT_FLAT_ST_B_NM_P_RPS 10.0
#define DEFAULT_FLAT_EST_K_NM_P_RAD 300.0
#define DEFAULT_FLAT_LST_K_NM_P_RAD 450.0
#define DEFAULT_FLAT_LST_THETA_RAD 0.244
#define DEFAULT_FLAT_LST_ENGAGEMENT_TQ_NM 30.0
#define DEFAULT_FLAT_LST_DELAY_TICS 250.0

#define DEFAULT_URAMP_HS_THETA_RAD -0.19
#define DEFAULT_URAMP_HS_K_NM_P_RAD 350.0
#define DEFAULT_URAMP_LSW_THETA_RAD -0.14
#define DEFAULT_URAMP_ST_B_NM_P_RPS 8.0
#define DEFAULT_URAMP_EST_K_NM_P_RAD 300.0
#define DEFAULT_URAMP_LST_K_NM_P_RAD 400.0
#define DEFAULT_URAMP_LST_THETA_RAD 0.3
#define DEFAULT_URAMP_LST_ENGAGEMENT_TQ_NM 35.0
#define DEFAULT_URAMP_LST_DELAY_TICS 150.0

#define DEFAULT_DRAMP_HS_THETA_RAD 0.0
#define DEFAULT_DRAMP_HS_K_NM_P_RAD 800.0
#define DEFAULT_DRAMP_LSW_THETA_RAD 0.0
#define DEFAULT_DRAMP_ST_B_NM_P_RPS 8.0
#define DEFAULT_DRAMP_EST_K_NM_P_RAD 400.0
#define DEFAULT_DRAMP_LST_K_NM_P_RAD 400.0
#define DEFAULT_DRAMP_LST_THETA_RAD 0.244
#define DEFAULT_DRAMP_LST_ENGAGEMENT_TQ_NM 20.0
#define DEFAULT_DRAMP_LST_DELAY_TICS 100.0

#define DEFAULT_USTAIRS_HS_THETA_RAD 0.2
#define DEFAULT_USTAIRS_HS_K_NM_P_RAD 50.0
#define DEFAULT_USTAIRS_LSW_THETA_RAD 0.2
#define DEFAULT_USTAIRS_ST_B_NM_P_RPS 5.0
#define DEFAULT_USTAIRS_EST_K_NM_P_RAD 50.0
#define DEFAULT_USTAIRS_LST_K_NM_P_RAD 300.0
#define DEFAULT_USTAIRS_LST_THETA_RAD 0.5
#define DEFAULT_USTAIRS_LST_ENGAGEMENT_TQ_NM 20.0
#define DEFAULT_USTAIRS_LST_DELAY_TICS 300.0

#define DEFAULT_DSTAIRS_HS_THETA_RAD 0.0
#define DEFAULT_DSTAIRS_HS_K_NM_P_RAD 20.0
#define DEFAULT_DSTAIRS_LSW_THETA_RAD 0.5
#define DEFAULT_DSTAIRS_ST_B_NM_P_RPS 6.0
#define DEFAULT_DSTAIRS_EST_K_NM_P_RAD 125.0
#define DEFAULT_DSTAIRS_LST_K_NM_P_RAD 0.0
#define DEFAULT_DSTAIRS_LST_THETA_RAD 0.0
#define DEFAULT_DSTAIRS_LST_ENGAGEMENT_TQ_NM 1000.0
#define DEFAULT_DSTAIRS_LST_DELAY_TICS 70.0
#endif

#ifdef USER_NB
#define DEFAULT_FLAT_HS_THETA_RAD 0.0
#define DEFAULT_FLAT_HS_K_NM_P_RAD 300.0
#define DEFAULT_FLAT_LSW_THETA_RAD -0.1
#define DEFAULT_FLAT_ST_B_NM_P_RPS 10.0
#define DEFAULT_FLAT_EST_K_NM_P_RAD 300.0
#define DEFAULT_FLAT_LST_K_NM_P_RAD 320.0
#define DEFAULT_FLAT_LST_THETA_RAD 0.244
#define DEFAULT_FLAT_LST_ENGAGEMENT_TQ_NM 35.0
#define DEFAULT_FLAT_LST_DELAY_TICS 150.0

#define DEFAULT_URAMP_HS_THETA_RAD -0.19
#define DEFAULT_URAMP_HS_K_NM_P_RAD 350.0
#define DEFAULT_URAMP_LSW_THETA_RAD -0.14
#define DEFAULT_URAMP_ST_B_NM_P_RPS 8.0
#define DEFAULT_URAMP_EST_K_NM_P_RAD 300.0
#define DEFAULT_URAMP_LST_K_NM_P_RAD 400.0
#define DEFAULT_URAMP_LST_THETA_RAD 0.3
#define DEFAULT_URAMP_LST_ENGAGEMENT_TQ_NM 35.0
#define DEFAULT_URAMP_LST_DELAY_TICS 150.0

#define DEFAULT_DRAMP_HS_THETA_RAD 0.0
#define DEFAULT_DRAMP_HS_K_NM_P_RAD 800.0
#define DEFAULT_DRAMP_LSW_THETA_RAD 0.0
#define DEFAULT_DRAMP_ST_B_NM_P_RPS 8.0
#define DEFAULT_DRAMP_EST_K_NM_P_RAD 400.0
#define DEFAULT_DRAMP_LST_K_NM_P_RAD 400.0
#define DEFAULT_DRAMP_LST_THETA_RAD 0.244
#define DEFAULT_DRAMP_LST_ENGAGEMENT_TQ_NM 20.0
#define DEFAULT_DRAMP_LST_DELAY_TICS 100.0

#define DEFAULT_USTAIRS_HS_THETA_RAD 0.2
#define DEFAULT_USTAIRS_HS_K_NM_P_RAD 50.0
#define DEFAULT_USTAIRS_LSW_THETA_RAD 0.2
#define DEFAULT_USTAIRS_ST_B_NM_P_RPS 5.0
#define DEFAULT_USTAIRS_EST_K_NM_P_RAD 50.0
#define DEFAULT_USTAIRS_LST_K_NM_P_RAD 300.0
#define DEFAULT_USTAIRS_LST_THETA_RAD 0.5
#define DEFAULT_USTAIRS_LST_ENGAGEMENT_TQ_NM 20.0
#define DEFAULT_USTAIRS_LST_DELAY_TICS 300.0

#define DEFAULT_DSTAIRS_HS_THETA_RAD 0.0
#define DEFAULT_DSTAIRS_HS_K_NM_P_RAD 20.0
#define DEFAULT_DSTAIRS_LSW_THETA_RAD 0.5
#define DEFAULT_DSTAIRS_ST_B_NM_P_RPS 6.0
#define DEFAULT_DSTAIRS_EST_K_NM_P_RAD 125.0
#define DEFAULT_DSTAIRS_LST_K_NM_P_RAD 0.0
#define DEFAULT_DSTAIRS_LST_THETA_RAD 0.0
#define DEFAULT_DSTAIRS_LST_ENGAGEMENT_TQ_NM 1000.0
#define DEFAULT_DSTAIRS_LST_DELAY_TICS 70.0
#endif

#ifdef USER_RJ
#define DEFAULT_FLAT_HS_THETA_RAD 0.0
#define DEFAULT_FLAT_HS_K_NM_P_RAD 100.0
#define DEFAULT_FLAT_LSW_THETA_RAD -0.087
#define DEFAULT_FLAT_ST_B_NM_P_RPS 10.0
#define DEFAULT_FLAT_EST_K_NM_P_RAD 400.0
#define DEFAULT_FLAT_LST_K_NM_P_RAD 400.0
#define DEFAULT_FLAT_LST_THETA_RAD 0.244
#define DEFAULT_FLAT_LST_ENGAGEMENT_TQ_NM 30.0
#define DEFAULT_FLAT_LST_DELAY_TICS 60.0

#define DEFAULT_URAMP_HS_THETA_RAD -0.19
#define DEFAULT_URAMP_HS_K_NM_P_RAD 350.0
#define DEFAULT_URAMP_LSW_THETA_RAD -0.14
#define DEFAULT_URAMP_ST_B_NM_P_RPS 8.0
#define DEFAULT_URAMP_EST_K_NM_P_RAD 300.0
#define DEFAULT_URAMP_LST_K_NM_P_RAD 400.0
#define DEFAULT_URAMP_LST_THETA_RAD 0.3
#define DEFAULT_URAMP_LST_ENGAGEMENT_TQ_NM 35.0
#define DEFAULT_URAMP_LST_DELAY_TICS 150.0

#define DEFAULT_DRAMP_HS_THETA_RAD 0.0
#define DEFAULT_DRAMP_HS_K_NM_P_RAD 800.0
#define DEFAULT_DRAMP_LSW_THETA_RAD 0.0
#define DEFAULT_DRAMP_ST_B_NM_P_RPS 8.0
#define DEFAULT_DRAMP_EST_K_NM_P_RAD 600.0
#define DEFAULT_DRAMP_LST_K_NM_P_RAD 400.0
#define DEFAULT_DRAMP_LST_THETA_RAD 0.244
#define DEFAULT_DRAMP_LST_ENGAGEMENT_TQ_NM 20.0
#define DEFAULT_DRAMP_LST_DELAY_TICS 100.0

#define DEFAULT_USTAIRS_HS_THETA_RAD 0.2
#define DEFAULT_USTAIRS_HS_K_NM_P_RAD 50.0
#define DEFAULT_USTAIRS_LSW_THETA_RAD 0.2
#define DEFAULT_USTAIRS_ST_B_NM_P_RPS 5.0
#define DEFAULT_USTAIRS_EST_K_NM_P_RAD 50.0
#define DEFAULT_USTAIRS_LST_K_NM_P_RAD 100.0
#define DEFAULT_USTAIRS_LST_THETA_RAD 0.35
#define DEFAULT_USTAIRS_LST_ENGAGEMENT_TQ_NM 20.0
#define DEFAULT_USTAIRS_LST_DELAY_TICS 300.0

#define DEFAULT_DSTAIRS_HS_THETA_RAD 0.0
#define DEFAULT_DSTAIRS_HS_K_NM_P_RAD 20.0
#define DEFAULT_DSTAIRS_LSW_THETA_RAD 0.35
#define DEFAULT_DSTAIRS_ST_B_NM_P_RPS 6.0
#define DEFAULT_DSTAIRS_EST_K_NM_P_RAD 175.0
#define DEFAULT_DSTAIRS_LST_K_NM_P_RAD 0.0
#define DEFAULT_DSTAIRS_LST_THETA_RAD 0.0
#define DEFAULT_DSTAIRS_LST_ENGAGEMENT_TQ_NM 1000.0
#define DEFAULT_DSTAIRS_LST_DELAY_TICS 70.0
#endif


#ifdef USER_AP
#define DEFAULT_FLAT_HS_THETA_RAD 0.0
#define DEFAULT_FLAT_HS_K_NM_P_RAD 100.0
#define DEFAULT_FLAT_LSW_THETA_RAD -0.087
#define DEFAULT_FLAT_ST_B_NM_P_RPS 10.0
#define DEFAULT_FLAT_EST_K_NM_P_RAD 300.0
#define DEFAULT_FLAT_LST_K_NM_P_RAD 400.0
#define DEFAULT_FLAT_LST_THETA_RAD 0.244
#define DEFAULT_FLAT_LST_ENGAGEMENT_TQ_NM 20.0
#define DEFAULT_FLAT_LST_DELAY_TICS 200.0

#define DEFAULT_URAMP_HS_THETA_RAD -0.19
#define DEFAULT_URAMP_HS_K_NM_P_RAD 350.0
#define DEFAULT_URAMP_LSW_THETA_RAD -0.14
#define DEFAULT_URAMP_ST_B_NM_P_RPS 8.0
#define DEFAULT_URAMP_EST_K_NM_P_RAD 300.0
#define DEFAULT_URAMP_LST_K_NM_P_RAD 400.0
#define DEFAULT_URAMP_LST_THETA_RAD 0.3
#define DEFAULT_URAMP_LST_ENGAGEMENT_TQ_NM 35.0
#define DEFAULT_URAMP_LST_DELAY_TICS 150.0

#define DEFAULT_DRAMP_HS_THETA_RAD 0.0
#define DEFAULT_DRAMP_HS_K_NM_P_RAD 800.0
#define DEFAULT_DRAMP_LSW_THETA_RAD 0.0
#define DEFAULT_DRAMP_ST_B_NM_P_RPS 8.0
#define DEFAULT_DRAMP_EST_K_NM_P_RAD 400.0
#define DEFAULT_DRAMP_LST_K_NM_P_RAD 400.0
#define DEFAULT_DRAMP_LST_THETA_RAD 0.244
#define DEFAULT_DRAMP_LST_ENGAGEMENT_TQ_NM 20.0
#define DEFAULT_DRAMP_LST_DELAY_TICS 100.0

#define DEFAULT_USTAIRS_HS_THETA_RAD 0.2
#define DEFAULT_USTAIRS_HS_K_NM_P_RAD 50.0
#define DEFAULT_USTAIRS_LSW_THETA_RAD 0.2
#define DEFAULT_USTAIRS_ST_B_NM_P_RPS 5.0
#define DEFAULT_USTAIRS_EST_K_NM_P_RAD 50.0
#define DEFAULT_USTAIRS_LST_K_NM_P_RAD 300.0
#define DEFAULT_USTAIRS_LST_THETA_RAD 0.5
#define DEFAULT_USTAIRS_LST_ENGAGEMENT_TQ_NM 20.0
#define DEFAULT_USTAIRS_LST_DELAY_TICS 300.0

#define DEFAULT_DSTAIRS_HS_THETA_RAD 0.0
#define DEFAULT_DSTAIRS_HS_K_NM_P_RAD 20.0
#define DEFAULT_DSTAIRS_LSW_THETA_RAD 0.5
#define DEFAULT_DSTAIRS_ST_B_NM_P_RPS 6.0
#define DEFAULT_DSTAIRS_EST_K_NM_P_RAD 125.0
#define DEFAULT_DSTAIRS_LST_K_NM_P_RAD 0.0
#define DEFAULT_DSTAIRS_LST_THETA_RAD 0.0
#define DEFAULT_DSTAIRS_LST_ENGAGEMENT_TQ_NM 1000.0
#define DEFAULT_DSTAIRS_LST_DELAY_TICS 70.0
#endif
//
//
//#ifdef USER_RC
//#define DEFAULT_FLAT_HS_THETA_RAD 0.0
//#define DEFAULT_FLAT_HS_K_NM_P_RAD 100.0
//#define DEFAULT_FLAT_LSW_THETA_RAD -0.087
//#define DEFAULT_FLAT_ST_B_NM_P_RPS 10.0
//#define DEFAULT_FLAT_EST_K_NM_P_RAD 300.0
//#define DEFAULT_FLAT_LST_K_NM_P_RAD 200.0
//#define DEFAULT_FLAT_LST_THETA_RAD 0.244
//#define DEFAULT_FLAT_LST_ENGAGEMENT_TQ_NM 20.0
//#define DEFAULT_FLAT_LST_DELAY_TICS 70.0
//
//#define DEFAULT_URAMP_HS_THETA_RAD -0.19
//#define DEFAULT_URAMP_HS_K_NM_P_RAD 350.0
//#define DEFAULT_URAMP_LSW_THETA_RAD -0.14
//#define DEFAULT_URAMP_ST_B_NM_P_RPS 8.0
//#define DEFAULT_URAMP_EST_K_NM_P_RAD 300.0
//#define DEFAULT_URAMP_LST_K_NM_P_RAD 50.0
//#define DEFAULT_URAMP_LST_THETA_RAD 0.3
//#define DEFAULT_URAMP_LST_ENGAGEMENT_TQ_NM 30.0
//#define DEFAULT_URAMP_LST_DELAY_TICS 70.0
//
//#define DEFAULT_DRAMP_HS_THETA_RAD 0.0
//#define DEFAULT_DRAMP_HS_K_NM_P_RAD 800.0
//#define DEFAULT_DRAMP_LSW_THETA_RAD 0.0
//#define DEFAULT_DRAMP_ST_B_NM_P_RPS 8.0
//#define DEFAULT_DRAMP_EST_K_NM_P_RAD 50.0
//#define DEFAULT_DRAMP_LST_K_NM_P_RAD 10.0
//#define DEFAULT_DRAMP_LST_THETA_RAD 0.0
//#define DEFAULT_DRAMP_LST_ENGAGEMENT_TQ_NM 20.0
//#define DEFAULT_DRAMP_LST_DELAY_TICS 70.0
//
//#define DEFAULT_USTAIRS_HS_THETA_RAD 0.2
//#define DEFAULT_USTAIRS_HS_K_NM_P_RAD 350.0
//#define DEFAULT_USTAIRS_LSW_THETA_RAD 0.2
//#define DEFAULT_USTAIRS_ST_B_NM_P_RPS 5.0
//#define DEFAULT_USTAIRS_EST_K_NM_P_RAD 50.0
//#define DEFAULT_USTAIRS_LST_K_NM_P_RAD 300.0
//#define DEFAULT_USTAIRS_LST_THETA_RAD 0.26
//#define DEFAULT_USTAIRS_LST_ENGAGEMENT_TQ_NM 10.0
//#define DEFAULT_USTAIRS_LST_DELAY_TICS 70.0
//
//#define DEFAULT_DSTAIRS_HS_THETA_RAD 0.0
//#define DEFAULT_DSTAIRS_HS_K_NM_P_RAD 150.0
//#define DEFAULT_DSTAIRS_LSW_THETA_RAD 0.55
//#define DEFAULT_DSTAIRS_ST_B_NM_P_RPS 12.0
//#define DEFAULT_DSTAIRS_EST_K_NM_P_RAD 75.0
//#define DEFAULT_DSTAIRS_LST_K_NM_P_RAD 0.0
//#define DEFAULT_DSTAIRS_LST_THETA_RAD 0.0
//#define DEFAULT_DSTAIRS_LST_ENGAGEMENT_TQ_NM 1000.0
//#define DEFAULT_DSTAIRS_LST_DELAY_TICS 70.0
//#endif




#define DEFAULT_MINIMUM_JERK_TRAJECTORY_TIME 0.2
#define DEFAULT_MINIMUM_JERK_ANGLE_TOL_RAD 0.05
#define MINIMUM_JERK_MAX_MEAN_SPEED_RPS 2.0

int get_walking_state();
struct minimum_jerk_values_s* get_minimum_jerk_values();
struct control_params_s* get_control_params();
void set_minimum_jerk_trajectory_period(float T);
void set_minimum_jerk_angle_tol_rad(float angle_tol_rad);
void enable_minimum_jerk(uint8_t enabled);
void set_esw_theta_rad(float theta_rad);
void set_sw_k_Nm_p_rad(float k_Nm_p_rad);
void set_sw_b_Nm_p_rps(float b_Nm_p_rps);
void set_sw_delay_tics(float sw_delay_tics);
void set_nominal_theta_rad(float theta_rad);
void set_nominal_k_Nm_p_rad(float k_Nm_p_rad);
void set_nominal_b_Nm_p_rps(float b_Nm_p_rps);
void set_hard_stop_theta_rad(float hard_stop_theta_rad, int terrain);
void set_hard_stop_k_Nm_p_rad(float hard_stop_k_Nm_p_rad, int terrain);
void set_lsw_theta_rad(float lsw_theta_rad, int terrain);
void set_st_b_Nm_p_rps(float est_b_Nm_p_rps, int terrain);
void set_est_k_Nm_p_rad(float est_k_Nm_p_rad, int terrain);
void set_lst_k_Nm_p_rad(float lst_k_Nm_p_rad, int terrain);
void set_lst_theta_rad(float lst_theta_rad, int terrain);
void set_lst_delay_tics(float lst_delay_tics, int terrain);
void set_lst_engagement_tq_Nm(float lst_engagement_tq_Nm, int terrain);
void terrain_state_machine_demux(struct taskmachine_s* tm, struct rigid_s* rigid, Act_s *actx, int current_terrain);
void init_terrain_state_machine();
void reset_terrain_state_machine_parameters();

static struct nominal_control_params_s
{
	float theta_rad;
	float k_Nm_p_rad;
	float b_Nm_p_rps;
};


static struct active_control_params_s
{
	//Terrain independent params
	float esw_theta_rad;
	float sw_k_Nm_p_rad;
	float sw_b_Nm_p_rps;
	float desired_trajectory_period_s;
	float sw_delay_tics;
	float lst_delay_tics;

	//Terrain dependent params
	float hard_stop_theta_rad;
	float hard_stop_k_Nm_p_rad;
	
	float lsw_theta_rad;
	float st_b_Nm_p_rps;
	float est_k_Nm_p_rad;
	float lst_k_Nm_p_rad;
	float lst_theta_rad;

	float lst_engagement_tq_Nm;

};


static struct terrain_dependent_control_params_s
{
	float* hard_stop_theta_rad;
	float* hard_stop_k_Nm_p_rad;
	
	float* lsw_theta_rad;
	float* st_b_Nm_p_rps;
	float* est_k_Nm_p_rad;
	float* lst_k_Nm_p_rad;
	float* lst_theta_rad;

	float* est_lst_min_theta_rad;
	float* lst_engagement_tq_Nm;
	float* lst_delay_tics;

};

struct control_params_s{
	 struct terrain_dependent_control_params_s adaptive;
 	struct active_control_params_s active;
	struct nominal_control_params_s nominal;
};

struct minimum_jerk_values_s{
	float* T;
	float* params;
	float theta_target;
	float theta_dot_target;
	float theta_ddot_target;
	float des_theta;
	uint update_counter;
	uint total_trajectory_updates;
	float angle_tol_rad;
	uint8_t trajectory_defined;
	uint8_t enabled;
};


enum Walking_States {
   STATE_ESW = 2,               /// 2
   STATE_LSW = 3,                /// 3
   STATE_EST = 4,              /// 4
   STATE_MST = 4,
   STATE_LST = 6,               /// 5
};

// Fl
// Sw
// - go to alpha_fl
// EST
// - provide some damping and virtual hardstop at theta_fl
// LST
// - provide some damping and stiffness and virtual hardstop at theta_fl > alpha_fl
// LSP
// - stiff spring with far away set point

// Ur
// Sw
// - go to alpha_ur (dorsiflexed)
// EST
// - provide some damping and virtual hardstop at theta_ur > alpha_ur
// LSP
// - stiffer spring wirh far away set point

// Dr
// SW
// - go to alpha_dr = alpha_fl
// Est
// - provide damping
// Lst
// - provide damping and stiffness

// Us
// Sw
// - go to alpha_us
// EST
// - provide some damping or stiffness??
// LSP
// - use mildly stiff spring to try to open the ankle, and have much stiffer virtual hardstop ratcheting angle, such that the ankle can provide support independently of angle

// Ds
// SW
// - go to alpha_ds
// Est
// - provide lots of damping and maybe a small stiffness
// Lst
// - provide stiffness to a zero set point 

#endif
