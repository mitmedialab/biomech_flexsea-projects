 

#include "task_machine.h"


static struct taskmachine_s tm;
static int learning_demux_state;

static void init_task_machine(void){

    tm.mode = MODE_DS;
    tm.task = TASK_DS;

    tm.latest_foot_static_samples = 0;
    tm.elapsed_samples = 0;
    tm.latest_foot_off_samples = 10000;

    tm.foundFirstZvupAfterFootOn = 0;
    tm.taskPredicted = 0;
    tm.inSwing = 0;
    tm.achievedMinTorqueForStance = 0;

    tm.resetTrigger = 0;

    tm.tqDot = 0.0f;
    tm.tq = 0.0f;

    tm.mean_stance_theta = 0.0;
    tm.max_stance_tq = 0.0;
    tm.max_stance_tq_ind = 0.0;
    tm.prev_max_stance_tq = 0.0;
    tm.min_swing_z = 0.0;
    tm.max_swing_z = 0.0;
    tm.max_swing_z_samples = 0.0;
    tm.prev_stance_samples = 0.0;
    tm.k_est = TASK_FL;
}

static void back_estimate(){
  
  tm.mean_stance_theta = tm.mean_stance_theta/tm.elapsed_samples;
  tm.k_est = TASK_FL;
  if (tm.max_swing_z > US_Z_THRESH && tm.max_swing_z_samples < US_MAX_SAMPLES_TO_Z_THRESH){
      tm.k_est = TASK_US;
  }else if (tm.max_swing_z > UR_Z_THRESH && tm.mean_stance_theta < UR_MEAN_THETA_THRESH){
      tm.k_est = TASK_UR;
  }else if (tm.min_swing_z < DS_Z_THRESH && tm.prev_max_stance_tq > DS_TQ_THRESH){
      tm.k_est = TASK_DS;
  }else if (tm.min_swing_z < DR_Z_THRESH && tm.mean_stance_theta > DR_MEAN_THETA_THRESH){
      tm.k_est = TASK_DR;
  }
  tm.mean_stance_theta = 0.0;
  tm.prev_max_stance_tq = tm.max_stance_tq;
  tm.do_learning = 0;
}


static void update_gait_events(){

   // tm.tqDot = 0.200f*tm.tqDot + 0.800f*(tm.tq - tm.prevtq);
    tm.tqDot = 0.0f;

    
    

    if (tm.inSwing){
        //Swing to stance transition condition
        if (tm.elapsed_samples - tm.latest_foot_off_samples >= MIN_SWING_SAMPLES){
            if (tm.tq >= MIN_TQ_FOR_FOOT_ON)
            {
                if (tm.elapsed_samples > MIN_STRIDE_SAMPLES && 
                  tm.elapsed_samples - tm.latest_foot_off_samples > PREDICTION_CUTOFF_SAMPLES){
                  tm.do_learning = 1;
                }
                tm.elapsed_samples = 0;
                tm.foundFirstZvupAfterFootOn = 0;
                tm.inSwing = 0;
                tm.achievedMinTorqueForStance = 0;
                tm.max_stance_tq = tm.tq;
                tm.taskPredicted = 0;
        }
      }
  }
  else{
        tm.max_stance_tq = MAX(tm.max_stance_tq, tm.tq);
        tm.mean_stance_theta = tm.mean_stance_theta + tm.theta;

        if (tm.tq > MIN_TQ_FOR_FOOT_STATIC && 
            tm.tqDot > MIN_TQDOT_FOR_FOOT_STATIC &&
                tm.tq > tm.max_stance_tq - 20.000f){

            if (tm.elapsed_samples - tm.latest_foot_static_samples > DEFAULT_STANCE_RESET_SAMPLES && 
                    !tm.foundFirstZvupAfterFootOn){
                tm.resetTrigger = 1;
                tm.latest_foot_static_samples = tm.elapsed_samples;
                reset_kinematics();
            }

            if (get_kinematics()->accNormSq < UPPER_ACCNORM_THRESH_SQ && 
                get_kinematics()->accNormSq > LOWER_ACCNORM_THRESH_SQ){
                    tm.foundFirstZvupAfterFootOn = 1;
                    tm.resetTrigger = 2;
                    tm.latest_foot_static_samples = tm.elapsed_samples;
                    reset_kinematics();
            }  
        }
        

        if (tm.elapsed_samples - tm.latest_foot_static_samples > STANCE_RESET_EXPIRY_SAMPLES){
                tm.foundFirstZvupAfterFootOn = 0;
                tm.resetTrigger = 0;
                tm.latest_foot_static_samples = tm.elapsed_samples;
                reset_kinematics();
        }
        
        //Stance to swing transition condition
        if (tm.elapsed_samples > MIN_STANCE_SAMPLES &&
            tm.tq < MIN_TQ_FOR_FOOT_ON &&
            tm.tq > N_MIN_TQ_FOR_FOOT_ON){
            tm.inSwing = 1;
            tm.latest_foot_off_samples = tm.elapsed_samples;
            get_classifier()->k_pred = 1;

            if (tm.do_learning && learning_demux_state == READY_TO_LEARN){
              back_estimate();
              reset_learning_demux();
            }
        }
    }

}

static void simulate_ankle_torque(){
  if (tm.elapsed_samples % 2000 < 1000)
    tm.tq = 100.0f;
  else
    tm.tq = 0.0f;
}

static int task_machine_demux_state = 0;

struct taskmachine_s* get_task_machine(){
  return &tm;
}

void task_machine_demux(struct rigid_s* rigid){

  
  switch (task_machine_demux_state){
    case INIT_TASK_MACHINE:
        init_task_machine();
        task_machine_demux_state++;
    break;
    case INIT_LEARNING:
        init_learning_structs();
        task_machine_demux_state++;
    break;
    case INIT_KINEMATICS:
        init_kinematics();
        task_machine_demux_state++;
    break;
    case RUN_TASK_MACHINE:
        simulate_ankle_torque(); //just for non-hil testing
        update_gait_events();
        learning_demux_state = learning_demux(tm.k_est);

        if (!tm.taskPredicted){
          update_kinematics(&rigid->mn);
          update_features(get_kinematics());
          
        }
        if (tm.inSwing && tm.elapsed_samples - tm.latest_foot_off_samples >= PREDICTION_CUTOFF_SAMPLES){
           tm.taskPredicted = 1;     
           classify(); 
        }

        // if (tm.elapsed_samples - tm.latestFootStaticTime > 600){
        //   tm.kin.calculateTranslations = 0;
        // }
    break;
  }

  tm.elapsed_samples++;

  state_machine_demux(rigid, get_classifier()->k_pred);

}
