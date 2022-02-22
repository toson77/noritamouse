#include "drive.h"
#include "define.h"
#include "encoder.h"
#include "function_test.h"

void direction_r_mot(char direction){
	if(direction == MOT_FORWARD){
		MOT_R_IN1 = 0;
		MOT_R_IN2 = 1;
	}
	else if(direction == MOT_BACKWARD){
		MOT_R_IN1 = 1;
		MOT_R_IN2 = 0;
		
	}
	else if(direction == MOT_STOP){
		MOT_R_IN1 = 0;
		MOT_R_IN2 = 0;
		
	}
	else if(direction == MOT_BRAKE){
		MOT_R_IN1 = 1;
		MOT_R_IN2 = 1;
	}
}

void direction_l_mot(char direction){

	if(direction == MOT_FORWARD){
		MOT_L_IN1 = 0;
		MOT_L_IN2 = 1;
	}
	else if(direction == MOT_BACKWARD){
		MOT_L_IN1 = 1;
		MOT_L_IN2 = 0;
	}
	else if(direction == MOT_STOP){
		MOT_L_IN1 = 0;
		MOT_L_IN2 = 0;
	}
	else if(direction == MOT_BRAKE){
		MOT_L_IN1 = 1;
		MOT_L_IN2 = 1;
	}
}
