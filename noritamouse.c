/***********************************************************************/
/*                                                                     */
/*  FILE        :Main.c or Main.cpp                                    */
/*  DATE        :Tue, Oct 31, 2006                                     */
/*  DESCRIPTION :Main Program                                          */
/*  CPU TYPE    :                                                      */
/*                                                                     */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/*                                                                     */
/***********************************************************************/
//#include "typedefine.h"

/*-----------------------------------*/
#include "iodefine.h"
#include "parameter.h"
#include "define.h"
#include "init_rx220.h"
#include "function_test.h"
#include "sci.h"
#include "sensor.h"
#include "encoder.h"
#include "interrupt.h"
#include "run.h"
#include "control.h"
#include "search.h"
#include "log.h"
#include "7seg.h"
/*-----------------------------------*/

#ifdef __cplusplus
//#include <ios>                        // Remove the comment when you use ios
//_SINT ios_base::Init::init_cnt;       // Remove the comment when you use ios
#endif

void main(void);
#ifdef __cplusplus
extern "C" {
void abort(void);
}
#endif

void straight_wall_on_test() {
	straight(SECTION*3, SEARCH_SPEED, 0, SEARCH_ACCEL, 0,0,-1);
}
void turn_mini_test() {
	straight(0.045, 0.3, 0, 1.0, 1,0,-1);
	wait_ms(100);
	turn(180.0, 300.0, 0, 500.0);
	wait_ms(500);
	straight(0.045, 0.3, 0, 1.0, 1,0,-1);
	wait_ms(500);
	turn(180.0, 300.0, 0, 500.0);
	wait_ms(500);
	turn(-180.0, 300.0, 0, 500.0);
	wait_ms(500);
	turn(-180.0, 300.0, 0, 500.0);
}
void back_test() {
	revision_back(0.01, SEARCH_SPEED, SEARCH_ACCEL);
}
	

void main(void)
{

	init_rx220();
	
	short mode = 1;
	const unsigned short INTERRUPT_COUNT = 3; //LED点滅回数
	const unsigned int SEN_DICISION = 3000; //モード入力閾値
	const unsigned short SPEED_DICISION = 300; //モード変更閾値

	wait_sec(1);
	
	while(1){
		
		switch(mode) {
			
			/*足立探索*/
			case 1:
				if((int)get_sen_value(LF_SEN)+(int)get_sen_value(LS_SEN)+(int)get_sen_value(RS_SEN)+(int)get_sen_value(RF_SEN) > SEN_DICISION) {
					ledseg_1_interrupt(INTERRUPT_COUNT); //LED点滅
					wait_sec(1);
					adachi_method();		
				}
				
				break;
			
			/*最短走行1*/
			case 2:
				if((int)get_sen_value(LF_SEN)+(int)get_sen_value(LS_SEN)+(int)get_sen_value(RS_SEN)+(int)get_sen_value(RF_SEN) > SEN_DICISION) {
					ledseg_2_interrupt(INTERRUPT_COUNT); //LED点滅	
					run_shortestRoute();
					wait_sec(1);
					
				}
				
				break;
				
			/*最短走行2*/
			case 3:
				if((int)get_sen_value(LF_SEN)+(int)get_sen_value(LS_SEN)+(int)get_sen_value(RS_SEN)+(int)get_sen_value(RF_SEN) > SEN_DICISION) {
					ledseg_3_interrupt(INTERRUPT_COUNT); //LED点滅
					wait_sec(1);
					straight_wall_on_test();
					//turn_mini_test();
					//back_test();
					//straight(SECTION*10, 0.3, 0, 1.0, 1,0);
					
				}
				
				break;
			
			/*自律*/
			case 4:
				if((int)get_sen_value(LF_SEN)+(int)get_sen_value(LS_SEN)+(int)get_sen_value(RS_SEN)+(int)get_sen_value(RF_SEN) > SEN_DICISION) {
					ledseg_4_interrupt(INTERRUPT_COUNT); //LED点滅
					enable_f_wall_control();
					wait_ms(500);
				}
				
				break;
				
			/*test*/
			case 5:
				if((int)get_sen_value(LF_SEN)+(int)get_sen_value(LS_SEN)+(int)get_sen_value(RS_SEN)+(int)get_sen_value(RF_SEN) > SEN_DICISION) {
					ledseg_5_interrupt(INTERRUPT_COUNT); //LED点滅
					wait_sec(1);
					while(1) {
						adc_test_all();
						if(get_current_enc_velocity(LEFT_ENCODER) > SPEED_DICISION) {
							ledseg_5_interrupt(INTERRUPT_COUNT); //LED点滅
							break;
						}
							
					}
				}
				
				break;
				
			/*test*/
			case 6:
				if((int)get_sen_value(LF_SEN)+(int)get_sen_value(LS_SEN)+(int)get_sen_value(RS_SEN)+(int)get_sen_value(RF_SEN) > SEN_DICISION) {
					ledseg_6_interrupt(INTERRUPT_COUNT); //LED点滅
					wait_sec(1);
					turn(3600, 300.0, 0, 500.0);
					wait_sec(3);
					turn(-3600, 300.0, 0, 500.0);
					
					
					
				}
				
				break;
				
			/*test*/
			case 7:
				if((int)get_sen_value(LF_SEN)+(int)get_sen_value(LS_SEN)+(int)get_sen_value(RS_SEN)+(int)get_sen_value(RF_SEN) > SEN_DICISION) {
					ledseg_7_interrupt(INTERRUPT_COUNT); //LED点滅
					wait_sec(1);
					straight(SECTION*3+0.045, 0.3, 0, 1.0, 1,0,-1);
					wait_ms(500);
					turn(90.0, 300.0, 0, 500.0);
					//revision_back(0.3);
					straight(0.05, SEARCH_SPEED, 0, SEARCH_ACCEL, 0,0,-1);
					wait_ms(500);
				
					turn(90.0, 300.0, 0, 500.0);
				
					//revision_back(0.3);
					wait_ms(100);
				
					straight(0.135, SEARCH_SPEED, 0, SEARCH_ACCEL, 1,0,-1);
				}
				
				break;
				
			/*test*/
			case 8:
				if((int)get_sen_value(LF_SEN)+(int)get_sen_value(LS_SEN)+(int)get_sen_value(RS_SEN)+(int)get_sen_value(RF_SEN) > SEN_DICISION) {
					ledseg_8_interrupt(INTERRUPT_COUNT); //LED点滅
					wait_ms(500);
					straight(SECTION*3+0.045, 0.3, 0, 1.0, 1,0,-1);
					turn(-90.0, 500.0, 0, 500.0);
					//turn(180, 700.0, 0, 3000.0);
					//straight(SECTION*2+0.035, 0.3, 0, 1, 0,0);
				}
				
				break;
				
			/*test*/
			case 9:
				if((int)get_sen_value(LF_SEN)+(int)get_sen_value(LS_SEN)+(int)get_sen_value(RS_SEN)+(int)get_sen_value(RF_SEN) > SEN_DICISION) {
					ledseg_9_interrupt(INTERRUPT_COUNT); //LED点滅
					wait_sec(1);
					print_run_log();
					//print_wall();
					
				}
				
				break;
				
			default:
				break;
		}				
			
		 //モード切り替え用処理
		if(get_current_enc_velocity(RIGHT_ENCODER) > SPEED_DICISION){
			if(mode == 9){
				mode = 1;
			}else{
				mode ++;
			}
			for(unsigned long i = 0; i < 100*1000*10; i++);
			
		}
		
		if(get_current_enc_velocity(RIGHT_ENCODER) < -SPEED_DICISION){
			if(mode == 1){
				mode = 9;
			}else{
				mode --;
			}
			for(unsigned long i = 0; i < 100*1000*10; i++);
			 
		}
		LED(mode);
		
		
	}
	
	/*
	for(int i = 0; i < 3 ; i++){
	straight(SECTION*3, 0.5, 0.3, 1.0, 1);
	turn(180.0, 700, 0, 7000);
	straight(SECTION*3, 0.5, 0.3, 1.0, 1);
	turn(180.0, 700, 0, 7000);
	}
	
	/*
	straight(0.030, 0.3, 0.3, 1.0, 0);
	slalom(-84.0, 300, 0, 4000);
	straight(0.015, 0.3, 0.3, 1.0, 0);
	straight(SECTION*2, 0.3, 0, 1.0, 1);
	*/
	/*
	straight(SECTION, 0.3, 0.3, 1.0, 1);
	straight(0.020, 0.3, 0.3, 1.0, 0);
	slalom(74.0, 300, 0, 4000);
	straight(0.015, 0.3, 0.3, 1.0, 0);
	straight(SECTION*2, 0.3, 0, 1.0, 1);
	*/
	//straight(SECTION*3, 0.3, 0, 1.0, 0);
	//straight(SECTION, 0.5, 0, 2.0, 0);
	//turn(180.0, 700, 0, 7000);
	//straight(HALF_SECTION+SECTION*2, 0.5, 0, 1.0, 1);
	/*
	straight(0.015, SEARCH_SPEED, 0, SEARCH_ACCEL, 1);
			slalom(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
			straight(0.0, SEARCH_SPEED, 0, SEARCH_ACCEL, 1);
			*/
	//left_hand();
	//wait_sec(15);
	

		//直進
		/*
			straight(SECTION*3, FAST_SPEED, 0, FAST_ACCEL,1,1);
			turn(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
			straight(SECTION, FAST_SPEED, 0, FAST_ACCEL,1,1);
			turn(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
			straight(SECTION*2, FAST_SPEED, 0, FAST_ACCEL,1,1);
			turn(90.0, TURN_OMEGA, 0, TURN_ALPHA);
			straight(SECTION, FAST_SPEED, 0, FAST_ACCEL,1,1);
			turn(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
			straight(SECTION, FAST_SPEED, 0, FAST_ACCEL,1,1);
			turn(90.0, TURN_OMEGA, 0, TURN_ALPHA);
			straight(SECTION, FAST_SPEED, 0, FAST_ACCEL,1,1);
			turn(90.0, TURN_OMEGA, 0, TURN_ALPHA);
			straight(SECTION*3, FAST_SPEED, 0, FAST_ACCEL,1,1);
			turn(180.0, TURN_OMEGA, 0, TURN_ALPHA);
			*/
			/*
			straight(SECTION*2+HALF_SECTION, FAST_SPEED, FAST_SPEED, FAST_ACCEL,1,0);
			//straight(0.01, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1,0);
			slalom(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
			straight(0.050, SEARCH_SPEED, 0, 1, 1,0);
			//straight(0.01, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1,0);
			slalom(90.0, TURN_OMEGA, 0, TURN_ALPHA);
			straight(0.050, SEARCH_SPEED, 0, 1, 1,0);
			*/
			
			//straight(SECTION, FAST_SPEED, FAST_SPEED, FAST_ACCEL,1,0);
			//slalom(-82.0, TURN_OMEGA, 0, TURN_ALPHA);
			//straight(0.050, SEARCH_SPEED, 0, 1, 1,0);
			//slalom(90.0, TURN_OMEGA, 0, TURN_ALPHA);
			//straight(0.050, SEARCH_SPEED, 0, 1, 1,0);
	/*		
	while(1){
		no_attack();
	}
	*/
				
	//turn(-86.0, 700, 0, 7000);		
	//adachi_method();
	//wait_sec(20);
	//run_shortestRoute();
	//print_run_log();
	
	/*
	while(1){
		adc_test_all();
	}
	*/
	
	while(1);
}

#ifdef __cplusplus
void abort(void)
{

}
#endif
