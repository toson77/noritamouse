#include "define.h"
#include "parameter.h"
#include "encoder.h"
#include "sensor.h"

//�ړ����ς̃o�b�t�@�T�C�Y(�ݒ�l[ms]�ňړ����ς������)
#define	N	1

//�G���R�[�_�\�J�E���g�� - 10000 [count/ms]
static short r_enc_value;
static short l_enc_value;

//���E�̑��x [mm/s]
static short r_enc_velocity;
static short l_enc_velocity;

//�G���R�[�_�l��1ms�̕ω��ʎZ�o
void calc_enc_value(){
	r_enc_value = 10000 - ENC_PULSE_COUNT_R;
	l_enc_value = ENC_PULSE_COUNT_L - 10000;
	//�G���R�[�_�̒l�������l10000�ɖ߂�
	ENC_PULSE_COUNT_L = 10000;
	ENC_PULSE_COUNT_R = 10000;
}

//�ړ����όv�Z�֐�
float calc_r_enc_moving_ave(short value){
	static short i=0;	//�����O�o�b�t�@�̃|�C���^
	static short buf[N] = {0};	//�����O�o�b�t�@
	static float sum;	//���v�l
	
	sum -= buf[i];	//�ŌÂ̒l�����v�l�������
	buf[i] = value;	//�ŌÂ̒l���ŐV�̒l�ɍX�V
	sum += buf[i];	//�ŐV�̒l�����v�l�ɉ�����
	
	if(++i == N){i=0;}	//���̂��߂Ƀ����O�o�b�t�@�̃|�C���^��i�߂�
	
	return (sum/N);	// ( ����N�̍��v/�� )
}

float calc_l_enc_moving_ave(short value){
	static short i=0;	//�����O�o�b�t�@�̃|�C���^
	static short buf[N] = {0};	//�����O�o�b�t�@
	static float sum;	//���v�l
	
	sum -= buf[i];	//�ŌÂ̒l�����v�l�������
	buf[i] = value;	//�ŌÂ̒l���ŐV�̒l�ɍX�V
	sum += buf[i];	//�ŐV�̒l�����v�l�ɉ�����
	
	if(++i == N){i=0;}	//���̂��߂Ƀ����O�o�b�t�@�̃|�C���^��i�߂�
	
	return (sum/N);	// ( ����N�̍��v/�� )
}

short get_enc_value(char encoder){	//calc�������݂�enc_value��Ԃ�
	if(encoder == RIGHT_ENCODER){
		return r_enc_value;
	}
	else if(encoder == LEFT_ENCODER){
		return l_enc_value;
	}
}

short get_current_enc_velocity(char encoder){	//�����Ŏw�肵������enc_value��[mm/s]�ɕϊ���, �Ԃ�

	if(encoder == RIGHT_ENCODER){
		r_enc_velocity = ((float)(r_enc_value) * DISTANCE_PER_ENC_COUNT) * 1000;	//[mm/s] = [count/ms] * [mm/count]
		return r_enc_velocity;
	}
	else if(encoder == LEFT_ENCODER){
		l_enc_velocity = ((float)(l_enc_value) * DISTANCE_PER_ENC_COUNT) * 1000;	//[mm/s] = [count/ms] * [mm/count]
		return l_enc_velocity;
	}
	
}

unsigned short duty_to_count(short duty){	//duty�� -> TGRA, TGRC�J�E���g�ւ̕ϊ�
	unsigned short temp_cnt = 0;
	temp_cnt = ((100 - duty) * 320 / 100) - 1;	//320��PWM����, 100-duty�͋t�ɂȂ��Ă邩��
	if(temp_cnt <= 0) temp_cnt = 0;
	if(temp_cnt == 319) temp_cnt = 320;
	return temp_cnt;
}