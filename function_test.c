#include "define.h"
#include "parameter.h"
#include "function_test.h"
#include "sci.h"
#include "drive.h"
#include "sensor.h"
#include "interrupt.h"
#include "encoder.h"
#include "run.h"

void motor_drive_test(){
	MOT_STBY = 1;			//Standby off
	direction_r_mot(MOT_FORWARD);	//R motor forward
	direction_l_mot(MOT_FORWARD);	//L motor forward
	
}

void motor_stop(){
	MOT_STBY = 1;
	direction_r_mot(MOT_STOP);	//R motor stop
	direction_l_mot(MOT_STOP);	//L motor stop
}

void motor_rotation(){
	MOT_STBY = 1;
	direction_r_mot(MOT_FORWARD);	//R motor forward
	direction_l_mot(MOT_BACKWARD);	//L motor backward
}

void move_forward_specified_duty(short duty){	//指定したDuty(%)で前進
	MOT_DUTY_L = duty_to_count(duty);
	MOT_DUTY_R = duty_to_count(duty);
	if(MOT_DUTY_L < 60) MOT_DUTY_L = 60;	//Duty80%ちょいでフェイルセーフ
	if(MOT_DUTY_R < 60) MOT_DUTY_R = 60;
	MOT_STBY = 1;			//Standby off
	direction_r_mot(MOT_FORWARD);	//R motor forward
	direction_l_mot(MOT_FORWARD);	//L motor forward
}

void move_backward_specified_duty(short duty){	//指定したDuty(%)で後進
	MOT_DUTY_L = duty_to_count(duty);
	MOT_DUTY_R = duty_to_count(duty);
	if(MOT_DUTY_L < 60) MOT_DUTY_L = 60;	//Duty80%ちょいでフェイルセーフ
	if(MOT_DUTY_R < 60) MOT_DUTY_R = 60;
	MOT_STBY = 1;			//Standby off
	direction_r_mot(MOT_BACKWARD);	//R motor backward
	direction_l_mot(MOT_BACKWARD);	//L motor backward
}

void rotate_clockwise_specified_duty(short duty){	//指定dutyで時計回り
	MOT_DUTY_L = duty_to_count(duty);
	MOT_DUTY_R = duty_to_count(duty);
	if(MOT_DUTY_L < 60) MOT_DUTY_L = 60;	//Duty80%ちょいでフェイルセーフ
	if(MOT_DUTY_R < 60) MOT_DUTY_R = 60;
	MOT_STBY = 1;			//Standby off
	direction_r_mot(MOT_BACKWARD);	//R motor backward
	direction_l_mot(MOT_FORWARD);	//L motor forward
}

void rotate_anticlockwise_specified_duty(short duty){	//指定dutyで反時計回り
	MOT_DUTY_L = duty_to_count(duty);
	MOT_DUTY_R = duty_to_count(duty);
	if(MOT_DUTY_L < 60) MOT_DUTY_L = 60;	//Duty80%ちょいでフェイルセーフ
	if(MOT_DUTY_R < 60) MOT_DUTY_R = 60;
	MOT_STBY = 1;			//Standby off
	direction_r_mot(MOT_FORWARD);	//R motor forward
	direction_l_mot(MOT_BACKWARD);	//L motor backward
}

void flash_led(void){
	short i=0;
	while(1){
		LED_2 = CHIP_LED_ON;
		for(i=0; i<10000; i++);
		LED_2 = CHIP_LED_OFF;
		for(i=0; i<10000; i++);
	}
}

void sci_put_test(void){
	short i=0;
	while(1){
		sci_put_1byte(0x88);
		for(i=0;i<10000;i++);	//待つ
	}
}

void sci_put_string_test(){
	volatile char cnt;
	while(1){
		cnt ++;
		sci_printf("count = %d \r\n", cnt);
	}
}

void adc_test1(void){
	int temp_ADdata = 0;
	
	//左前センサ
	S12AD.ADANSA.WORD = 0x01;		//AN000を選択
	S12AD.ADCSR.BIT.ADST = 1;		//AD変換開始
	while(S12AD.ADCSR.BIT.ADST == 1);	//ADSTが0になるまで待つ
	temp_ADdata = S12AD.ADDR0;
	sci_printf("AN000 = %d \r\n",temp_ADdata);
}

void adc_test2(void){
	int temp_ADdata = 0;
	
	//左横センサ
	S12AD.ADANSA.BIT.ANSA1 = 1;		//AN001を選択
	S12AD.ADCSR.BIT.ADST = 1;		//AD変換開始
	while(S12AD.ADCSR.BIT.ADST == 1);	//ADSTが0になるまで待つ
	temp_ADdata = S12AD.ADDR1;
	sci_printf("AN001 = %d \r\n",temp_ADdata);
}

void adc_test3(void){
	int temp_ADdata = 0;
	
	//右横センサ
	S12AD.ADANSA.BIT.ANSA2 = 1;		//AN002を選択
	S12AD.ADCSR.BIT.ADST = 1;		//AD変換開始
	while(S12AD.ADCSR.BIT.ADST == 1);	//ADSTが0になるまで待つ
	temp_ADdata = S12AD.ADDR2;
	sci_printf("AN002 = %d \r\n",temp_ADdata);
}

void adc_test4(void){
	int temp_ADdata = 0;
	
	//右前センサ
	S12AD.ADANSA.BIT.ANSA3 = 1;		//AN003を選択
	S12AD.ADCSR.BIT.ADST = 1;		//AD変換開始
	while(S12AD.ADCSR.BIT.ADST == 1);	//ADSTが0になるまで待つ
	temp_ADdata = S12AD.ADDR3;
	sci_printf("AN003 = %d \r\n",temp_ADdata);
}

void adc_test_all(void){
	sci_printf("LF=%d LS=%d RS=%d RF=%d time=%d \r\n",
		(int)get_sen_value(LF_SEN), (int)get_sen_value(LS_SEN), 
		(int)get_sen_value(RS_SEN), (int)get_sen_value(RF_SEN), 
		(int)get_time(TYPE_SEC));
}
void led_interrupt_test(void) {
	sci_printf("LFON=%d LFOFF=%d RFON=%d RFOFF=%d \r\n", (int)get_sen_on_value(LF_SEN), (int)get_sen_off_value(LF_SEN), (int)get_sen_on_value(RF_SEN), (int)get_sen_off_value(RF_SEN));
}

void encoder_test1(void){
	sci_printf("R_ENC = %d L_ENC = %d \r\n",ENC_PULSE_COUNT_R, ENC_PULSE_COUNT_L);
}

void encoder_test2(void){
	sci_printf("R_CNT = %d , L_CNT = %d \r\n", get_enc_value(RIGHT_ENCODER), get_enc_value(LEFT_ENCODER));
}

void encoder_test3(void){
	sci_printf("R_ENC_VEL = %d [mm/s], L_ENC_VEL = %d [mm/s] \r\n", get_current_enc_velocity(RIGHT_ENCODER), get_current_enc_velocity(LEFT_ENCODER));
}

void battery_ad_test(void){
	sci_printf("batt = %d \r\n",(int)(get_battery_voltage()*100.0));
}

int get_sign(float num){
	return ( num > 0.0001 ) - ( num < -0.0001 );
}
