#include "define.h"
#include "iodefine.h"
#include "interrupt.h"
#include "sensor.h"
#include "encoder.h"
#include "run.h"
#include "control.h"
#include "drive.h"
#include "7seg.h"

static unsigned short mtu3_cnt = 0;
static unsigned int timer_ms = 0;
static unsigned int timer_sec = 0;
volatile static unsigned int my_timer_ms = 0;

//102us���荞��( *8�̃X���b�g�ɕ������邱�Ƃ�1ms�̉������Ă��� )
void mtu3_tgra(){
	mtu3_cnt ++;
	switch(mtu3_cnt){
		case 1:
			//LS_sensor
			S12AD.ADANSA.WORD = 0x01;			//AN000��I��
			S12AD.ADCSR.BIT.ADST = 1;			//AD�ϊ��J�n
			while(S12AD.ADCSR.BIT.ADST == 1);		//ADST��0�ɂȂ�܂ő҂�
			set_sen_value(LS_SEN, LED_OFF, S12AD.ADDR0);	//�Z���T�l�i�[
			LS_SEN_LED = SEN_LED_ON;			//LED�_��
		break;
		
		case 2:
			//RS_sensor
			S12AD.ADANSA.WORD = 0x02;			//AN001��I��
			S12AD.ADCSR.BIT.ADST = 1;			//AD�ϊ��J�n
			while(S12AD.ADCSR.BIT.ADST == 1);		//ADST��0�ɂȂ�܂ő҂�
			set_sen_value(RS_SEN, LED_OFF, S12AD.ADDR1);	//�Z���T�l�i�[
			RS_SEN_LED = SEN_LED_ON;			//LED�_��
		break;
		
		case 3:
			//LF_sensor
			S12AD.ADANSA.WORD = 0x04;			//AN002��I��
			S12AD.ADCSR.BIT.ADST = 1;			//AD�ϊ��J�n
			while(S12AD.ADCSR.BIT.ADST == 1);		//ADST��0�ɂȂ�܂ő҂�
			set_sen_value(LF_SEN, LED_OFF, S12AD.ADDR2);	//�Z���T�l�i�[
			LF_SEN_LED = SEN_LED_ON;			//LED�_��
		break;
		
		case 4:
			//RF_sensor
			S12AD.ADANSA.WORD = 0x08;			//AN003��I��
			S12AD.ADCSR.BIT.ADST = 1;			//AD�ϊ��J�n
			while(S12AD.ADCSR.BIT.ADST == 1);		//ADST��0�ɂȂ�܂ő҂�
			set_sen_value(RF_SEN, LED_OFF, S12AD.ADDR3);	//�Z���T�l�i�[
			RF_SEN_LED = SEN_LED_ON;			//LED�_��
		break;
		
		case 5:
			calc_enc_value();	//�G���R�[�_�ω��ʌv�Z
		break;
		
		case 6:
			control_speed();	//���x����
			
		break;
		
		case 7:

		break;
		
		case 8:	
			battery_low_notification(); //�o�b�e���[�ቺ������~
			
		break;
		
		default:
		break;
	}

}

//125us���荞��( *8�̃X���b�g�ɕ������邱�Ƃ�1ms�̉������Ă��� )
void mtu3_tgrb(){
	switch(mtu3_cnt){
		case 1:
			//LS_sensor
			S12AD.ADANSA.WORD = 0x01;			//AN000��I��
			S12AD.ADCSR.BIT.ADST = 1;			//AD�ϊ��J�n
			while(S12AD.ADCSR.BIT.ADST == 1);		//ADST��0�ɂȂ�܂ő҂�
			set_sen_value(LS_SEN, LED_ON, S12AD.ADDR0);	//�Z���T�l�i�[
			LS_SEN_LED = SEN_LED_OFF;			//LED����
			calc_sen_value(LS_SEN);				//�Z���T�l�v�Z
		break;
		
		case 2:
			//RS_sensor
			S12AD.ADANSA.WORD = 0x02;			//AN001��I��
			S12AD.ADCSR.BIT.ADST = 1;			//AD�ϊ��J�n
			while(S12AD.ADCSR.BIT.ADST == 1);		//ADST��0�ɂȂ�܂ő҂�
			set_sen_value(RS_SEN, LED_ON, S12AD.ADDR1);	//�Z���T�l�i�[
			RS_SEN_LED = SEN_LED_OFF;			//LED����
			calc_sen_value(RS_SEN);				//�Z���T�l�v�Z
		break;
		
		case 3:
			//LF_sensor
			S12AD.ADANSA.WORD = 0x04;			//AN002��I��
			S12AD.ADCSR.BIT.ADST = 1;			//AD�ϊ��J�n
			while(S12AD.ADCSR.BIT.ADST == 1);		//ADST��0�ɂȂ�܂ő҂�
			set_sen_value(LF_SEN, LED_ON, S12AD.ADDR2);	//�Z���T�l�i�[
			LF_SEN_LED = SEN_LED_OFF;			//LED�_��
			calc_sen_value(LF_SEN);				//�Z���T�l�v�Z
		break;
		
		case 4:
			//RF_sensor
			S12AD.ADANSA.WORD = 0x08;			//AN003��I��
			S12AD.ADCSR.BIT.ADST = 1;			//AD�ϊ��J�n
			while(S12AD.ADCSR.BIT.ADST == 1);		//ADST��0�ɂȂ�܂ő҂�
			set_sen_value(RF_SEN, LED_ON, S12AD.ADDR3);	//�Z���T�l�i�[
			RF_SEN_LED = SEN_LED_OFF;			//LED�_��
			calc_sen_value(RF_SEN);				//�Z���T�l�v�Z
			
			//�d���d���Ď�
			S12AD.ADANSA.WORD = 0x10;			//AN004��I��
			S12AD.ADCSR.BIT.ADST = 1;			//AD�ϊ��J�n
			while(S12AD.ADCSR.BIT.ADST == 1);		//ADST��0�ɂȂ�܂ő҂�
			set_battery_value(S12AD.ADDR4);			//�Z���T�l�i�[
		break;
		
		case 5:
			control_wall();		//�ǐ���
			f_wall_control(); //�O�ǐ���
			break;
		
		case 6:
			
			pid_speed();		//PID��Duty��v�Z
		break;
		
		case 7:
			
		break;
		
		case 8:
			increment_timer_ms();
			increment_my_timer_ms();
			mtu3_cnt = 0;
			
		break;
		
		default:
		break;
	}


}

//10us���荞��( PWM�Ɠ������� )
void mtu4_tgrb(){
	
	change_motor_speed();	//���[�^�[���x�ω�
	
}

void increment_timer_ms(){
	timer_ms ++;
	if(timer_ms >= 1000){
		increment_timer_sec();
		LED_1 = ~LED_1;
		timer_ms = 0;
	}
	
}

void increment_my_timer_ms(){
	my_timer_ms++;
}

void increment_timer_sec(){
	timer_sec++;
}

unsigned int get_time(char type){
	if(type == TYPE_MS){
		return timer_ms;
	}
	else if(type == TYPE_SEC){
		return timer_sec;
	}
	else if(type == TYPE_MYMS) {
		return my_timer_ms;
	}
}

void wait_sec(unsigned int sec){
	unsigned int temp_sec = get_time(TYPE_SEC);
	while(1){
		if((temp_sec + sec) - get_time(TYPE_SEC) == 0) break;
	}
}

//����ms�ҋ@�͂ł���͂�
void wait_ms(unsigned int ms){
	my_timer_ms = 0;
	while(my_timer_ms < ms);
}

//�o�b�e���[�ቺ������~
void battery_low_notification() {
	if(get_battery_voltage() < 6.6) {
		reset_run_status();
		direction_l_mot(MOT_STOP);
		direction_r_mot(MOT_STOP);
		while(1) {
			ledseg_x_interrupt(1);
		}
	}
}
			
		
