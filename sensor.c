#include "define.h"
#include "parameter.h"
#include "sensor.h"
#include "maze.h"

//LED ON���̒l,LED OFF���̒l,�������Ƃ�����̒l
static signed int ls_sen_ledon_val, ls_sen_ledoff_val, ls_sen_val;
static signed int lf_sen_ledon_val, lf_sen_ledoff_val, lf_sen_val;
static signed int rf_sen_ledon_val, rf_sen_ledoff_val, rf_sen_val;
static signed int rs_sen_ledon_val, rs_sen_ledoff_val, rs_sen_val;

//�o�b�e���[�d���l
static float battery_ad_value;

//�w�肵���ǃZ���T�̒l��ۑ�����֐�
void set_sen_value(char position, char led_state, int ad_value){
	switch(position){
		case LF_SEN:
			if(led_state == LED_ON){
				lf_sen_ledon_val = ad_value;
			}
			else if(led_state == LED_OFF){
				lf_sen_ledoff_val = ad_value;
			}
		break;
		
		case LS_SEN:
			if(led_state == LED_ON){
				ls_sen_ledon_val = ad_value;
			}
			else if(led_state == LED_OFF){
				ls_sen_ledoff_val = ad_value;
			}
		break;
		
		case RS_SEN:
			if(led_state == LED_ON){
				rs_sen_ledon_val = ad_value;
			}
			else if(led_state == LED_OFF){
				rs_sen_ledoff_val = ad_value;
			}
		break;	
		
		case RF_SEN:
			if(led_state == LED_ON){
				rf_sen_ledon_val = ad_value;
			}
			else if(led_state == LED_OFF){
				rf_sen_ledoff_val = ad_value;
			}
		break;
		
		default:
		break;
	}
}

//�w�肵���ǃZ���T��, ( �_�����̒l - �������̒l ) ���v�Z��, �ۑ�����֐�
void calc_sen_value(char position){
	int before_lf, before_ls, before_rs, before_rf;
	before_lf = lf_sen_val;
	before_ls = ls_sen_val;
	before_rs = rs_sen_val;
	before_rf = rf_sen_val;
	switch(position){
		case LF_SEN:
			lf_sen_val = (lf_sen_ledon_val - lf_sen_ledoff_val + before_lf)/2;
		break;
		
		case LS_SEN:
			ls_sen_val = (ls_sen_ledon_val - ls_sen_ledoff_val + before_ls)/2;
		break;
		
		case RS_SEN:
			rs_sen_val = (rs_sen_ledon_val - rs_sen_ledoff_val + before_rs)/2;
		break;	
		
		case RF_SEN:
			rf_sen_val = (rf_sen_ledon_val - rf_sen_ledoff_val + before_rf)/2;
		break;
		
		default:
		break;
	}
	
}

//������������Z���T�l��Ԃ��֐�
float get_sen_value(char position){
	float temp_val = 0;
	switch(position){
		case LF_SEN:
			temp_val = lf_sen_val;
		break;
		
		case LS_SEN:
			temp_val = ls_sen_val;
		break;
		
		case RS_SEN:
			temp_val = rs_sen_val;
		break;	
		
		case RF_SEN:
			temp_val = rf_sen_val;
		break;
		
		default:
		break;
	}
	
	if(temp_val < 0){
		temp_val = 0;
	}
	return temp_val;
}

void set_battery_value(int ad_value){
	battery_ad_value = (float)ad_value; 
}

float get_battery_voltage(){
	return (((battery_ad_value/4095.0)*5.0)*((LOSIDE_BATTERY_R + HISIDE_BATTERY_R)/LOSIDE_BATTERY_R));
}
