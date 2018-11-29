

#include "walking_state_machine.h"


static int state_machine_demux_state = STATE_EARLY_SWING;

int get_walking_state(){
	return state_machine_demux_state;
}

void state_machine_demux(struct rigid_s* rigid, int k_est){
	switch (state_machine_demux_state){
		case STATE_EARLY_SWING:

			//TODO: add transition conditions
	        if (1){
	        	state_machine_demux_state++;
	        }
	        
	    break;
	    case STATE_LATE_SWING:

	    	//TODO: add transition conditions
	        if (1){
	        	state_machine_demux_state++;
	        }

	    break;
	    case STATE_EARLY_STANCE:
	        
	        //TODO: add transition conditions
	        if (1){
	        	state_machine_demux_state++;
	        }

	    break;
	    case STATE_LATE_STANCE:

	    	//TODO: add transition conditions
	   	 	if (1){
	        	state_machine_demux_state = STATE_EARLY_SWING;
	        }

	    break;
	}
}