

#include <terrain_state_machine.h>


static int state_machine_demux_state = STATE_EARLY_SWING;

int get_walking_state(){
	return state_machine_demux_state;
}


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

void terrain_state_machine_demux(struct rigid_s* rigid, int current_terrain){

switch (state_machine_demux_state){
		case STATE_ESW:
			//TODO: add transition conditions
	        if (1){
	        	state_machine_demux_state++;
	        }
	        
	    break;
	    case STATE_LSW:

	    	//TODO: add transition conditions
	        if (1){
	        	state_machine_demux_state++;
	        }

	    break;
	    case STATE_EST:
	        
	        //TODO: add transition conditions
	        if (1){
	        	state_machine_demux_state++;
	        }

	    break;
	    case STATE_LST:

	    	//TODO: add transition conditions
	   	 	if (1){
	        	state_machine_demux_state = STATE_EARLY_SWING;
	        }

	    break;
	}

}
