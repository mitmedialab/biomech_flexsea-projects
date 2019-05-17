/*
 * filtering_methods.c
 *
 *  Created on: May 16, 2019
 *      Author: rmsto
 */


float filter_second_order_butter_20hz(float new_val, float* outputs, float* inputs){
	inputs[0] = inputs[1];
	inputs[1] = inputs[2];
	inputs[2] = new_val;
	outputs[0] = outputs[1];
	outputs[1] = outputs[2];
	outputs[2] = 1.82269492519631*outputs[1] - 0.837181651256023*outputs[0]
		+ 0.00362168151492864*inputs[2] + 0.00724336302985729*inputs[1] + 0.00362168151492864*inputs[0];
	return outputs[2];
}

float filter_first_order_butter_20hz(float new_val, float* outputs, float* inputs){
	inputs[0] = inputs[1];
	inputs[1] = new_val;
	outputs[0] = outputs[1];
	outputs[1] = 0.881618592363189*outputs[0] + 0.0591907038184055*inputs[1] + 0.0591907038184055*inputs[0];
	return outputs[1];
}

float filter_second_order_butter_5hz(float new_val, float* outputs, float* inputs){
	inputs[0] = inputs[1];
	inputs[1] = inputs[2];
	inputs[2] = new_val;
	outputs[0] = outputs[1];
	outputs[1] = outputs[2];
	outputs[2] = 1.95557824031504*outputs[1] - 0.956543676511203*outputs[0]
		+ 0.000241359049041961*inputs[2] + 0.000482718098083923*inputs[1] + 0.000241359049041961*inputs[0];
	return outputs[2];
}

