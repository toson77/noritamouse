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
	straight(SECTION * 6, SEARCH_SPEED, 0, SEARCH_ACCEL, 1, 0);
}
void turn_mini_test() {
	//straight(0.045, 0.3, 0, 1.0, 1,0,-1);
	/*
	turn(3600.0, 310.0, 0, 3000.0);
	wait_ms(500);
	turn(-3600.0, 310.0, 0, 3000.0);
	*/

	/*
	for (int i = 0; i < 4; i++) {
		wait_ms(500);
		turn(360.0, 500.0, 0, 3000.0);
	}
	*/
	

	for (int i = 0; i < 4*10; i++) {
		wait_ms(500);
		turn(90.0, 500.0, 0, 3000.0);
	}
	
	
	//straight(0.045, 0.3, 0, 1.0, 1,0,-1);
	//wait_ms(500);
	/*
	turn(180.0, 300.0, 0, 500.0);
	wait_ms(100);
	turn(-180.0, 300.0, 0, 500.0);
	wait_ms(100);
	turn(-180.0, 300.0, 0, 500.0);
	*/
}
void back_test() {
	revision_back(0.01, SEARCH_SPEED, SEARCH_ACCEL);
}
	

void main(void)
{

	init_rx220();
	
	short mode = 1;
	const unsigned short INTERRUPT_COUNT = 3; //LED�_�ŉ�
	const unsigned int SEN_DICISION = 3000; //���[�h����臒l
	const unsigned short SPEED_DICISION = 300; //���[�h�ύX臒l

	wait_sec(1);
	
	while(1){
		
		switch(mode) {
			
			/*�����T��*/
			case 1:
				if((int)get_sen_value(LF_SEN)+(int)get_sen_value(LS_SEN)+(int)get_sen_value(RS_SEN)+(int)get_sen_value(RF_SEN) > SEN_DICISION) {
					ledseg_1_interrupt(INTERRUPT_COUNT); //LED�_��
					wait_sec(1);
					
					adachi_method();
					wait_sec(2);
					run_shortestRoute();
					
					//adachi_method_test();
				}
				break;
			
			/*�ŒZ���s1*/
			case 2:
				if((int)get_sen_value(LF_SEN)+(int)get_sen_value(LS_SEN)+(int)get_sen_value(RS_SEN)+(int)get_sen_value(RF_SEN) > SEN_DICISION) {
					ledseg_2_interrupt(INTERRUPT_COUNT); //LED�_��
					wait_sec(1);
					adachi_method_back_enable();
					wait_sec(1);
					
				}
				
				break;
				
			/*�ŒZ���s2*/
			case 3:
				if((int)get_sen_value(LF_SEN)+(int)get_sen_value(LS_SEN)+(int)get_sen_value(RS_SEN)+(int)get_sen_value(RF_SEN) > SEN_DICISION) {
					ledseg_3_interrupt(INTERRUPT_COUNT); //LED�_��
					wait_sec(1);
					//straight_wall_on_test();
					turn_mini_test();
					//back_test();
					//straight(SECTION*10, 0.3, 0, 1.0, 1,0);
					
				}
				
				break;
			
			/*����*/
			case 4:
				if((int)get_sen_value(LF_SEN)+(int)get_sen_value(LS_SEN)+(int)get_sen_value(RS_SEN)+(int)get_sen_value(RF_SEN) > SEN_DICISION) {
					ledseg_4_interrupt(INTERRUPT_COUNT); //LED�_��
					f_wall_test();
					wait_ms(500);
				}
				
				break;
				
			/*test*/
			case 5:
				if((int)get_sen_value(LF_SEN)+(int)get_sen_value(LS_SEN)+(int)get_sen_value(RS_SEN)+(int)get_sen_value(RF_SEN) > SEN_DICISION) {
					ledseg_5_interrupt(INTERRUPT_COUNT); //LED�_��
					wait_sec(1);
					//all_led_off();

					while(1) {
						adc_test_all();
						//led_interrupt_test();
						if(get_current_enc_velocity(LEFT_ENCODER) > SPEED_DICISION) {
							ledseg_5_interrupt(INTERRUPT_COUNT); //LED�_��
							break;
						}
							
					}
				}
				
				break;
				
			/*test*/
			case 6:
				if((int)get_sen_value(LF_SEN)+(int)get_sen_value(LS_SEN)+(int)get_sen_value(RS_SEN)+(int)get_sen_value(RF_SEN) > SEN_DICISION) {
					ledseg_6_interrupt(INTERRUPT_COUNT); //LED�_��
					wait_sec(1);
					turn(3600, 600.0, 0, 500.0);
					wait_sec(3);
					turn(-3600, 600.0, 0, 500.0);
					
					
					
				}
				
				break;
				
			/*test*/
			case 7:
				if((int)get_sen_value(LF_SEN)+(int)get_sen_value(LS_SEN)+(int)get_sen_value(RS_SEN)+(int)get_sen_value(RF_SEN) > SEN_DICISION) {
					ledseg_7_interrupt(INTERRUPT_COUNT); //LED�_��
					wait_sec(1);
					adachi_method();
				}
				
				break;
				
			/*test*/
			case 8:
				if((int)get_sen_value(LF_SEN)+(int)get_sen_value(LS_SEN)+(int)get_sen_value(RS_SEN)+(int)get_sen_value(RF_SEN) > SEN_DICISION) {
					ledseg_8_interrupt(INTERRUPT_COUNT); //LED�_��
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
					ledseg_9_interrupt(INTERRUPT_COUNT); //LED�_��
					wait_sec(1);
					//print_wall();
					//print_wall_back();
					print_run_log();
					
				}
				
				break;
				
			default:
				break;
		}				
			
		 //���[�h�؂�ւ��p����
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
	
	
	while(1);
}

#ifdef __cplusplus
void abort(void)
{

}
#endif
