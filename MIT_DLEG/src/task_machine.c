 

#include "task_machine.h"


static struct taskmachine_s tm;
static struct back_estimator_s be;

static void init_task_machine(){

    tm.mode = MODE_DS;
    tm.task = TASK_DS;

    tm.latest_foot_static_samples = 0;
    tm.elapsed_samples = 0;
    tm.latest_foot_off_samples = 10000;

    tm.foundFirstZvupAfterFootOn = 0;
    tm.in_swing = 0;
    tm.do_learning_for_stride = 0;

    tm.translation_reset_trigger = 0;
    tm.reached_classification_time = 0;
    tm.learning_reset_trigger = 0;

    tm.tq = 0.0f;

}

static void init_back_estimator(){
    be.mean_stance_theta = 0.0;
    be.max_stance_tq = 0.0;
    be.max_stance_tq_ind = 0.0;
    be.prev_max_stance_tq = 0.0;
    be.min_swing_z = 0.0;
    be.max_swing_z = 0.0;
    be.max_swing_z_samples = 0.0;
    be.prev_stance_samples = 0.0;
}


static void update_gait_events(){

    if (tm.in_swing){

      if (tm.elapsed_samples - tm.latest_foot_off_samples >= PREDICTION_CUTOFF_SAMPLES){
           tm.reached_classification_time = 1;
           tm.do_learning_for_stride = 1;    
      }
        //Swing to stance transition condition
      if (tm.tq >= MIN_TQ_FOR_FOOT_ON){
          tm.elapsed_samples = 0;
          tm.foundFirstZvupAfterFootOn = 0;
          tm.in_swing = 0;
          be.max_stance_tq = tm.tq;
          tm.translation_reset_trigger = 0;
      }
    }
    else{
          be.max_stance_tq = MAX(be.max_stance_tq, tm.tq);
          be.mean_stance_theta = be.mean_stance_theta + tm.theta;

          if (fabs(tm.tq) > MIN_TQ_FOR_FOOT_STATIC_NM){

              //default foot static condition
              if (tm.elapsed_samples - tm.latest_foot_static_samples > DEFAULT_STANCE_RESET_SAMPLES && 
                      !tm.foundFirstZvupAfterFootOn){
                  tm.translation_reset_trigger = 1;
                  tm.latest_foot_static_samples = tm.elapsed_samples;
                  
              }

              //optimal foot static condition
              if (get_kinematics()->accNormSq < UPPER_ACCNORM_THRESH_SQ && 
                  get_kinematics()->accNormSq > LOWER_ACCNORM_THRESH_SQ){
                      tm.foundFirstZvupAfterFootOn = 1;
                      tm.translation_reset_trigger = 2;
                      tm.latest_foot_static_samples = tm.elapsed_samples;
              }  
          }
          

          if (tm.elapsed_samples - tm.latest_foot_static_samples > STANCE_RESET_EXPIRY_SAMPLES){
                  tm.foundFirstZvupAfterFootOn = 0;
                  tm.translation_reset_trigger = 0;
                  tm.latest_foot_static_samples = tm.elapsed_samples;
          }
          
          //Stance to swing transition condition
          if (tm.elapsed_samples > MIN_STANCE_SAMPLES &&
              fabs(tm.tq) < MIN_TQ_FOR_FOOT_ON){
              tm.in_swing = 1;
              tm.latest_foot_off_samples = tm.elapsed_samples;
              if (tm.do_learning_for_stride)
                tm.learning_reset_trigger = 1;
              tm.do_learning_for_stride = 0;
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
        init_back_estimator();
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
        update_kinematics(&rigid->mn, &tm.translation_reset_trigger, &tm.reached_classification_time);
        if (tm.reached_classification_time)
          classify();
        else
          update_features(get_kinematics());

        learning_demux(&be, tm.latest_foot_off_samples, tm.learning_reset_trigger);
        state_machine_demux(rigid, get_classifier()->k_pred);

        // if (tm.elapsed_samples - tm.latestFootStaticTime > 600){
        //   tm.kin.reached_classification_time = 0;
        // }
    break;
  }

  tm.elapsed_samples++;

}
