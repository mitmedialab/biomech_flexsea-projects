 

#include "task_machine.h"


static struct taskmachine_s tm;

static void init_task_machine(void){

    tm.mode = MODE_DS;
    tm.task = TASK_DS;
    tm.state = STATE_WALKING_ENTRY_STATE;

    tm.latestFootStaticSamples = 0;
    tm.elapsedSamples = 0;
    tm.latestFootOffSamples = 10000;

    tm.foundFirstZvupAfterFootOn = 0;
    tm.taskPredicted = 0;
    tm.inSwing = 0;
    tm.achievedMinTorqueForStance = 0;

    tm.resetTrigger = 0;

    tm.tqDot = 0.0f;
    tm.tq = 0.0f;
    tm.maxTq = 0.0f;
    tm.maxTqInd = 0;
}



static void update_gait_events(){

    
    // if (tm.state == STATE_LATE_SWING_DS || tm.mc_sm_state == MCSM_TRAJECTORY_IMP_MODE_FOR_SWING_W_RES){
    //     return;
    // }
    // if (tm.state == STATE_LATE_SWING_DS){
    //     return;
    // }



   // tm.tqDot = 0.200f*tm.tqDot + 0.800f*(tm.tq - tm.prevtq);
    tm.tqDot = 0.0f;

    if (tm.tq >= tm.maxTq){
        tm.maxTq = tm.tq;
        tm.maxTqInd = tm.elapsedSamples;
    }

    if (tm.inSwing){
        //Detect if foot on has occurred
        if (tm.elapsedSamples - tm.latestFootOffSamples >= MIN_SWING_SAMPLES){
            if (tm.tq >= MIN_TQ_FOR_FOOT_ON)
            {
                tm.elapsedSamples = 0;
                tm.foundFirstZvupAfterFootOn = 0;
                tm.inSwing = 0;
                tm.achievedMinTorqueForStance = 0;
                tm.maxTq = tm.tq;
                tm.maxTqInd = 0;
                tm.taskPredicted = 0;
                reset_features();
        }
      }
  }
  else{

        if (tm.tq > MIN_TQ_FOR_FOOT_STATIC && 
            tm.tqDot > MIN_TQDOT_FOR_FOOT_STATIC &&
                tm.tq > tm.maxTq - 20.000f){

            if (tm.elapsedSamples - tm.latestFootStaticSamples > DEFAULT_STANCE_RESET_SAMPLES && 
                    !tm.foundFirstZvupAfterFootOn){
                tm.resetTrigger = 1;
                tm.latestFootStaticSamples = tm.elapsedSamples;
                reset_kinematics();
            }

            if (get_kinematics()->accNormSq < UPPER_ACCNORM_THRESH_SQ && 
                get_kinematics()->accNormSq > LOWER_ACCNORM_THRESH_SQ){
                    tm.foundFirstZvupAfterFootOn = 1;
                    tm.resetTrigger = 2;
                    tm.latestFootStaticSamples = tm.elapsedSamples;
                    reset_kinematics();
            }  
        }
        

        if (tm.elapsedSamples - tm.latestFootStaticSamples > STANCE_RESET_EXPIRY_SAMPLES){
                tm.foundFirstZvupAfterFootOn = 0;
                tm.resetTrigger = 0;
                tm.latestFootStaticSamples = tm.elapsedSamples;
                reset_kinematics();
        }
        
        if (tm.elapsedSamples > MIN_STANCE_SAMPLES &&
            tm.tq < MIN_TQ_FOR_FOOT_ON &&
            tm.tq > N_MIN_TQ_FOR_FOOT_ON){
            
            tm.inSwing = 1;
            tm.latestFootOffSamples = tm.elapsedSamples;
            get_classifier()->k_pred = 1;
            //tm.taskMachineEnabled = 1;
        }
    }

}

static void simulate_ankle_torque(){
  if (tm.elapsedSamples % 2000 < 1000)
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
        init_learner();
        init_classifier();
        init_features();
        task_machine_demux_state++;
    break;
    case INIT_KINEMATICS:
        init_kinematics();
        task_machine_demux_state++;
    break;
    case RUN_TASK_MACHINE:
        update_kinematics(&rigid->mn);
        // update_features(get_kinematics());
        simulate_ankle_torque(); //just for non-hil testing
        update_gait_events();
   
        // if (tm.inSwing && tm.elapsedSamples - tm.latestFootOffSamples >= PREDICTION_CUTOFF_SAMPLES){
        //    tm.taskPredicted = 1;     
        //    //solveDiscriminantFunction(); 
        // }

        // if (tm.elapsedSamples - tm.latestFootStaticTime > 600){
        //   tm.kin.calculateTranslations = 0;
        // }
    break;
  }

  tm.elapsedSamples++;

}

//   if (tm.acc2_raw != tm.acc2_raw_prev){
//     tm.startflag = 1;
// }

//     if (tm.startflag){
//     if (tm.iter1){
//         //17 flops
//         updateLocalAcc();
//         updateLocalOmega();
//         if (!tm.taskPredicted){
//             //9 flops
//             updateIntegralsAndDerivatives();

//         }else{
//             solveDiscriminantFunction();
//         }
//         tm.iter1 = 0;
//         tm.iter2 = 1;
//   } else if (tm.iter2){

//     if (!tm.taskPredicted){
//         //33 flops
//         updateTranslations();
//          if (tm.taskMachineEnabled && tm.inSwing){        
//               if (tm.elapsedSamples - tm.latestFootOffTime >= PREDICTION_CUTOFF){
//                  tm.taskPredicted = 1;     
//                   solveDiscriminantFunction(); 
//               }
//           }
//       }else{
//            solveDiscriminantFunction(); 
//       }

//       tm.iter2 = 0;
//       tm.iter3 = 1;
//     } else if (tm.iter3){ 
//         //8 swing no transition, +20 assignmetns with transition 
//         updateGaitEvents();
//         if (!tm.taskPredicted){
//             //9 flops
//             updateFeatures();
//         }else{
//             solveDiscriminantFunction();
//         }
//         tm.iter1 = 1;
//         tm.iter3 = 0;
//     }
        
//     }

//     tm.elapsedSamples = tm.elapsedSamples + 1;
//   walking_statemachine();
//}