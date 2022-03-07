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
//�ړ����σt�B���^�o�b�t�@�T�C�Y
#define SEN_BUF_SIZE 3

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

//�ړ����όv�Z�֐�
//�E��
static float calc_rf_sen_moving_ave(short value)
{
	static short i = 0;					  //�����O�o�b�t�@�̃|�C���^
	static short buf[SEN_BUF_SIZE] = {0}; //�����O�o�b�t�@
	static float sum = 0;				  //���v�l

	sum -= buf[i];	//�ŌÂ̒l�����v�l�������
	buf[i] = value; //�ŌÂ̒l���ŐV�̒l�ɍX�V
	sum += buf[i];	//�ŐV�̒l�����v�l�ɉ�����

	if (++i == SEN_BUF_SIZE)
	{
		i = 0;
	} //���̂��߂Ƀ����O�o�b�t�@�̃|�C���^��i�߂�

	return (sum / SEN_BUF_SIZE); // ( ����N�̍��v/�� )
}
//����
static float calc_lf_sen_moving_ave(short value)
{
	static short i = 0;					  //�����O�o�b�t�@�̃|�C���^
	static short buf[SEN_BUF_SIZE] = {0}; //�����O�o�b�t�@
	static float sum = 0;				  //���v�l

	sum -= buf[i];	//�ŌÂ̒l�����v�l�������
	buf[i] = value; //�ŌÂ̒l���ŐV�̒l�ɍX�V
	sum += buf[i];	//�ŐV�̒l�����v�l�ɉ�����

	if (++i == SEN_BUF_SIZE)
	{
		i = 0;
	} //���̂��߂Ƀ����O�o�b�t�@�̃|�C���^��i�߂�

	return (sum / SEN_BUF_SIZE); // ( ����N�̍��v/�� )
}
//���O��
static float calc_ls_sen_moving_ave(short value)
{
	static short i = 0;					  //�����O�o�b�t�@�̃|�C���^
	static short buf[SEN_BUF_SIZE] = {0}; //�����O�o�b�t�@
	static float sum = 0;				  //���v�l

	sum -= buf[i];	//�ŌÂ̒l�����v�l�������
	buf[i] = value; //�ŌÂ̒l���ŐV�̒l�ɍX�V
	sum += buf[i];	//�ŐV�̒l�����v�l�ɉ�����

	if (++i == SEN_BUF_SIZE)
	{
		i = 0;
	} //���̂��߂Ƀ����O�o�b�t�@�̃|�C���^��i�߂�

	return (sum / SEN_BUF_SIZE); // ( ����N�̍��v/�� )
}
//���O��
static float calc_rs_sen_moving_ave(short value)
{
	static short i = 0;					  //�����O�o�b�t�@�̃|�C���^
	static short buf[SEN_BUF_SIZE] = {0}; //�����O�o�b�t�@
	static float sum = 0;				  //���v�l

	sum -= buf[i];	//�ŌÂ̒l�����v�l�������
	buf[i] = value; //�ŌÂ̒l���ŐV�̒l�ɍX�V
	sum += buf[i];	//�ŐV�̒l�����v�l�ɉ�����

	if (++i == SEN_BUF_SIZE)
	{
		i = 0;
	} //���̂��߂Ƀ����O�o�b�t�@�̃|�C���^��i�߂�

	return (sum / SEN_BUF_SIZE); // ( ����N�̍��v/�� )
}

//�w�肵���ǃZ���T��, ( �_�����̒l - �������̒l ) ���v�Z��, �ۑ�����֐�
void calc_sen_value(char position){
	int before_lf, before_ls, before_rs, before_rf;
	before_lf = calc_lf_sen_moving_ave(lf_sen_val);
	before_ls = calc_ls_sen_moving_ave(ls_sen_val);
	before_rs = calc_rs_sen_moving_ave(rs_sen_val);
	before_rf = calc_rf_sen_moving_ave(rf_sen_val);
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

float get_sen_on_value(char position) {
	float temp_val = 0;
	switch (position)
	{
	case LF_SEN:
		temp_val = lf_sen_ledon_val;
		break;

	case LS_SEN:
		temp_val = ls_sen_ledon_val;
		break;

	case RS_SEN:
		temp_val = rs_sen_ledon_val;
		break;

	case RF_SEN:
		temp_val = rf_sen_ledon_val;
		break;

	default:
		break;
	}
	return temp_val;
}
float get_sen_off_value(char position)
{
	float temp_val = 0;
	switch (position)
	{
	case LF_SEN:
		temp_val = lf_sen_ledoff_val;
		break;

	case LS_SEN:
		temp_val = ls_sen_ledoff_val;
		break;

	case RS_SEN:
		temp_val = rs_sen_ledoff_val;
		break;

	case RF_SEN:
		temp_val = rf_sen_ledoff_val;
		break;

	default:
		break;
	}
	return temp_val;
}

//������������Z���T�l��Ԃ��֐�
float get_sen_value(char position){
	float temp_val = 0;
	switch(position){
		case LF_SEN:
			//�Ȃ�ׂ�RF�ƒl�����킹�邽��
			temp_val = lf_sen_val - 200;
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
