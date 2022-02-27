#include "iodefine.h"
#include "define.h"
#include "control.h"
#include "run.h"
#include "encoder.h"
#include "sensor.h"
#include "interrupt.h"
#include "sci.h"
#include "drive.h"
#include "parameter.h"
#include "log.h"
#include "function_test.h"
#include "maze.h"

/*�t���O--------------------------------------*/
//����������Ԃ�m�点��
//turn_flg�͒��V�n���񎞂ɖ��C���𑫂�����...(���V�n���񎞂̂�1, ���i��X�����[������0)
static char run_state, turn_state, turn_flg;
//���x����L���t���O
static char speed_control_flg = 0;
//�ǐ���L���t���O
static char wall_control_flg = 0;
static char forward_wall_stop_flg = 0;
static char length_reset_flg = 0;
// �O�Ǖ␳�p�ǐ���t���O
static char f_wall_control_flg = 0;

/*���x, �����x, ����-------------------------*/
//�ڕW�p�x, �ō����x, �I�[���x, �����x, �ō��p���x, �I�[�p���x, �p�����x
static float target_speed, target_omega;
static float angle;
static float accel = 0.0;
static float alpha = 0.0;
//1ms���Ƃ̒Ǐ]�ڕW���x, ����, �p���x, �p�x
static volatile float tar_vel = 0.0;
static volatile float tar_dis = 0.0;
static volatile float tar_omega = 0.0;
static volatile float tar_angle = 0.0;
//1ms�O�̎��ۂ̍��E���x
static float previous_vel_r = 0.0;
static float previous_vel_l = 0.0;
//���ۂ̍��E���x, ���S���x, �p���x
static float current_vel_r = 0.0;
static float current_vel_l = 0.0;
static volatile float current_vel_ave = 0.0;
static float current_omega = 0.0;
//���ۂ̍��E����, ����, �p�x
static volatile float current_dis_r = 0.0;
static volatile float current_dis_l = 0.0;
static volatile float current_dis_ave = 0.0;
static volatile float current_angle = 0.0;
static float length;

/*����v�Z�p-------------------------*/
//�΍�
static short error;
//�O�ǋ����덷
static short f_dis_err;

/*���[�^����p-------------------------*/
//�E���[�^�̏o�͓d��, �����[�^�̏o�͓d��
volatile static float V_r = 0.0;
volatile static float V_l = 0.0;
//�o�b�e���[�d��
static float V_bat = 0.0;
//�E���[�^Duty, �����[�^Duty
static short duty_r = 0;
static short duty_l = 0;

//��`������
//����, �ō���, �I�[��, �����x, �ǐ���L��( ON:1, OFF:0 ), �O�ǏՓ˖h�~flg(off:0)
void straight(float _length, float _top_speed, float _end_speed, float _accel, char _wall_control, char _forward_wall,char _nextdir){
	volatile unsigned int start_timer;//�^�C�������b�N�������̑΍�
	char nextdir = _nextdir;
	char lock_flg = 0; //�^�C�����b�N�����ς�flg
	//�ڕW����
	//float length;
	//�����ɕK�v�ȋ���, �����ɕK�v�ȋ���
	float accel_length, brake_length;
	//( accel_length + brake_length ) > length �������ꍇ�̍ō���
	float top_speed2;
	//�����x
	float start_speed;
	
	//�ڕW����, �ڕW���x, �����x, �ڕW�p�x, �ڕW�p���x, �p�����x�ݒ�
	length = _length;
	target_speed = _top_speed;
	accel = _accel;
	angle = 0;
	target_omega = 0;
	alpha = 0;
	
	//���[�^�[ON, ���x����ON, �ǐ���ON/OFF�ݒ�, �^�[���t���OOFF
	MOT_STBY = 1;
	speed_control_flg = 1;
	wall_control_flg = _wall_control;
	turn_flg = 0;
	forward_wall_stop_flg = _forward_wall;
	
	
	//�����x�v��
	start_speed = current_vel_ave;
	//v1^2 - v0^2 = 2ax ���@�̉���, �����\�������Z�o
	accel_length = ( ( _top_speed + start_speed ) * ( _top_speed - start_speed ) ) / ( 2.0 * _accel );
	brake_length = ( ( _top_speed + _end_speed ) * ( _top_speed - _end_speed ) ) / ( 2.0 * _accel );
	//�����\����+�����\���������s������蒷�������猸���J�n�n�_��ύX����
	if( length < ( accel_length + brake_length ) ){
		//top_speed^2 - start_speed^2 = 2.0 * acc * x1(��������)
		//end_speed^2 - top_speed^2 = 2.0 * -acc * x2(��������)
		//(x1 + x2) = length
		//���, �ō���top_speed��
		top_speed2 = ( ( 2.0 * _accel * length ) + ( start_speed * start_speed ) + ( _end_speed * _end_speed ) ) / 2.0;
		//�����2�Ԗڂ̌��������̎��ɑ�������
		brake_length = ( top_speed2 - ( _end_speed * _end_speed ) ) / ( 2.0 * _accel );
	}
	start_timer = get_time(TYPE_SEC); 
	
	//������Ԃ܂œ���
	while( current_dis_ave < ( length - brake_length ) ) {
		
		//�O�ǋ߂Â��Ă�����ǐ���؂�
		if(get_sen_value(LS_SEN) > TR_SENSOR_FRONT_WALL_L && get_sen_value(RS_SEN) > TR_SENSOR_FRONT_WALL_R){
			wall_control_flg = 0;
		}
		
	}
	
	//�����J�n
	target_speed = _end_speed;
	
	//�I�[���x0�@-> �Œᑬ�x�ݒ�
	if( _end_speed > -0.0009 && _end_speed < 0.0009 )	target_speed = MIN_VEL;
	//�ڕW�����ɓ��B����܂ő҂�
	while( current_dis_ave < length ){
		//�O�ǋ߂Â��Ă�����ǐ���؂�
		if(get_sen_value(LS_SEN) > TR_SENSOR_FRONT_WALL_L && get_sen_value(RS_SEN) > TR_SENSOR_FRONT_WALL_R){
			wall_control_flg = 0;
		}
		//������Ԕ����܂ōs������ǐ���؂�
		if(current_dis_ave > (length - brake_length/2.0)) {
			wall_control_flg = 0;
		}
		//�Փˎ��C��
		if(forward_wall_stop_flg == 1) {
			//�^�C�����b�N��������
			if(get_time(TYPE_SEC) - start_timer > 2 && lock_flg == 0) {
				//back
				reset_run_status();
				revision_back(0.03,SEARCH_SPEED,SEARCH_ACCEL);
				//�E�ܒ�
				if(nextdir==1) {
					//���ǂ���Ƃ�
					if(exist_l_wall == 1) {
						turn(-90.0, 300.0, 0, 500.0);
						revision_back(0.3,0.5,5);
						straight(0.035, SEARCH_SPEED, 0, SEARCH_ACCEL, 0,0,-1);
						wait_ms(100);
					
						turn(90.0, 300.0, 0, 500.0);
					}
					//���ǂȂ���
					else {
						turn(-180.0, 300.0, 0, 500.0);
						revision_back(0.3,0.5,5);
						straight(0.035, SEARCH_SPEED, 0, SEARCH_ACCEL, 0,0,-1);
						wait_ms(100);
						turn(180.0, 300.0, 0, 500.0);
					}
						
				}
				//���ܒ�(�E�ǂ���Ƃ�)
				else if(nextdir==3) {
					//�E�ǂ���Ƃ�
					if(exist_r_wall == 1) {
						turn(90.0, 300.0, 0, 500.0);
						revision_back(0.3,0.5,5);
						straight(0.035, SEARCH_SPEED, 0, SEARCH_ACCEL, 0,0,-1);
						wait_ms(100);
					
						turn(-90.0, 300.0, 0, 500.0);
					}
					//�E�ǂȂ���
					else {
						turn(180.0, 300.0, 0, 500.0);
						revision_back(0.3,0.5,5);
						straight(0.035, SEARCH_SPEED, 0, SEARCH_ACCEL, 0,0,-1);
						wait_ms(100);
						turn(-180.0, 300.0, 0, 500.0);
					}
						
				}
				return;
			}
		}
	}
	
	//��~����
	if( _end_speed > -0.0009 && _end_speed < 0.0009 ){
		//�X�s�[�h����OFF
		speed_control_flg = 0;
		direction_r_mot(MOT_BRAKE);
		direction_l_mot(MOT_BRAKE);
		duty_r = 0;
		duty_l = 0;
		//�X�e�[�^�X���Z�b�g
		reset_run_status();
	}
	
	tar_dis = 0;
	current_dis_r = 0;
	current_dis_l = 0;
	current_dis_ave = 0;
	tar_angle = 0;
	current_angle = 0;
}

//���M�n��]���̌�� _length > 0 �Ō�� tarspeed < 0, accell > 0��back?
void revision_back(float _length, float top_speed, float accell){
    volatile unsigned int start_timer;//�^�C�������b�N�������̑΍�
    float _top_speed = -top_speed;
    float _end_speed = 0;
    float _accel = accell;
    char _wall_control = 0;
    char _forward_wall = 0;
    //�ڕW����
	float length;
	//�����ɕK�v�ȋ���, �����ɕK�v�ȋ���
	float accel_length, brake_length;
	//( accel_length + brake_length ) > length �������ꍇ�̍ō���
	float top_speed2;
	//�����x
	float start_speed;
	
	//�ڕW����, �ڕW���x, �����x, �ڕW�p�x, �ڕW�p���x, �p�����x�ݒ�
	length = _length;
	target_speed = _top_speed;
	accel = _accel;
	angle = 0;
	target_omega = 0;
	alpha = 0;
	
	//���[�^�[ON, ���x����ON, �ǐ���ON/OFF�ݒ�, �^�[���t���OOFF
	MOT_STBY = 1;
	speed_control_flg = 1;
	wall_control_flg = _wall_control;
	turn_flg = 0;
	forward_wall_stop_flg = _forward_wall;
	
	
	//�����x�v��
	start_speed = current_vel_ave;
	//v1^2 - v0^2 = 2ax ���@�̉���, �����\�������Z�o
	accel_length = ( ( _top_speed + start_speed ) * ( _top_speed - start_speed ) ) / ( 2.0 * _accel );
	brake_length = ( ( _top_speed + _end_speed ) * ( _top_speed - _end_speed ) ) / ( 2.0 * _accel );
	//�����\����+�����\���������s������蒷�������猸���J�n�n�_��ύX����
	if( length < ( accel_length + brake_length ) ){
		//top_speed^2 - start_speed^2 = 2.0 * acc * x1(��������)
		//end_speed^2 - top_speed^2 = 2.0 * -acc * x2(��������)
		//(x1 + x2) = length
		//���, �ō���top_speed��
		top_speed2 = ( ( 2.0 * _accel * length ) + ( start_speed * start_speed ) + ( _end_speed * _end_speed ) ) / 2.0;
		//�����2�Ԗڂ̌��������̎��ɑ�������
		brake_length = ( top_speed2 - ( _end_speed * _end_speed ) ) / ( 2.0 * _accel );
	}
	
	start_timer = get_time(TYPE_SEC); 
	
	//������Ԃ܂œ���
	while( current_dis_ave > -( length - brake_length ) ) {
		//�^�C�����b�N��������
		if(get_time(TYPE_SEC) - start_timer > 1) {
			//reset_run_status();
			break;
		}
			
	}
	
	//�����J�n
	target_speed = _end_speed;
	//�I�[���x0�@-> �Œᑬ�x�ݒ�
	if( _end_speed > -0.0009 && _end_speed < 0.0009 )	target_speed = -MIN_VEL;
	//�ڕW�����ɓ��B����܂ő҂�
	while( current_dis_ave > -length ){
		//�ǂɏՓ˂������ɂȂ����狗��0
		if (length_reset_flg == 1){
			length_reset_flg = 0;
			break;
		}
		//�^�C�����b�N��������
		if(get_time(TYPE_SEC) - start_timer > 1) {
			break;
		}
	}
	
	//��~����
	if( _end_speed > -0.0009 && _end_speed < 0.0009 ){
		//�X�s�[�h����OFF
		speed_control_flg = 0;
		direction_r_mot(MOT_BRAKE);
		direction_l_mot(MOT_BRAKE);
		duty_r = 0;
		duty_l = 0;
		//�X�e�[�^�X���Z�b�g
		reset_run_status();
	}
	
	tar_dis = 0;
	current_dis_r = 0;
	current_dis_l = 0;
	current_dis_ave = 0;
	tar_angle = 0;
	current_angle = 0;
}

//���V�n����
//�ڕW�p�x, �ō��p���x, �I�[�p���x, �p�����x
void turn( float _angle, float _top_omega, float _end_omega, float _alpha ){

	//������Ԋp�x, ������Ԋp�x
	float accel_angle, brake_angle;
	//( accel_angle + brake_angle ) > angle �������ꍇ�̍ō��p���x
	float top_omega2;
	//���p���x
	float start_omega = 0.0;
	
	//�ڕW�p�x, �ڕW�d�S���x, �����x, �ڕW�p���x, �p�����x�ݒ�
	angle = _angle;
	target_speed = 0;
	accel = 0;
	if( angle < 0.0 )	target_omega = -_top_omega;
	else			target_omega = _top_omega;
	alpha = _alpha;
	
	//���[�^�[ON, ���x����ON, �ǐ���OFF, �^�[���t���OON
	MOT_STBY = 1;
	speed_control_flg = 1;
	wall_control_flg = 0;
	turn_flg = 1;
	
	//�ڕW�p�x�����Ȃ�v�Z�̂��ߐ��ɒ���
	if( angle < 0.0 )	angle = -angle;
	
	start_omega = current_omega;
	accel_angle = ( ( _top_omega + start_omega ) * ( _top_omega - start_omega ) ) / ( 2.0 * _alpha );
	brake_angle = ( ( _top_omega + _end_omega ) * ( _top_omega - _end_omega ) ) / ( 2.0 * _alpha );
	if( angle < ( accel_angle + brake_angle ) ){
		//top_omega^2 - start_omega^2 = 2.0 * alpha * x1(�����p�x)
		//end_omega^2 - top_omega^2 = 2.0 * -alpha * x2(�����p�x)
		//(x1 + x2) = angle
		//���, �ō���top_omega2��
		top_omega2 = ( ( 2.0 * _alpha * angle ) + ( start_omega * start_omega ) + ( _end_omega * _end_omega ) ) / 2.0;
		//�����2�Ԗڂ̌����p�x�̎��ɑ�������
		brake_angle = ( top_omega2 - ( _end_omega * _end_omega ) ) / ( 2.0 * _alpha );
	}
	
	//�߂�
	angle = _angle;
	
	//�ڕW�p�x�����̏ꍇ
	if( _angle > 0.0 ){
		//�����J�n��Ԃ܂ő҂�
		while( current_angle < ( angle - brake_angle ) );
		//�����J�n
		target_omega = _end_omega;
		//�I�[�p���x��0�̏ꍇ�Œ�p���x�ݒ�
		if( _end_omega > -0.0009 && _end_omega < 0.0009 ){
			target_omega = MIN_OMEGA;
		}
		//�ڕW�p�x�܂ő҂�
		while( current_angle < angle );
		
	//�ڕW�p�x�����̏ꍇ
	}else if( _angle < 0.0 ){
		//�����J�n��Ԃ܂ő҂�
		while( current_angle > -( - angle - brake_angle ) );
		//�����J�n
		target_omega = _end_omega;
		//�I�[�p���x��0�̏ꍇ�Œ�p���x�ݒ�
		if( _end_omega > -0.0009 && _end_omega < 0.0009 ){
			target_omega = -MIN_OMEGA;
		}
		//�ڕW�p�x�܂ő҂�
		while( current_angle > angle );
		
	}
	
	//�X�s�[�h����OFF
	speed_control_flg = 0;
	direction_r_mot(MOT_BRAKE);
	direction_l_mot(MOT_BRAKE);
	duty_l = 0;
	duty_r = 0;
	//�X�e�[�^�X���Z�b�g
	reset_run_status();
	
}

//�X�����[��( �d�S���x�ێ� )
//�ڕW�p�x, �ō��p���x, �I�[�p���x, �p�����x
void slalom( float _angle, float _top_omega, float _end_omega, float _alpha ){
	
	//�ڕW�p�x
	//float angle;
	//�p������Ԋp�x, �p������Ԋp�x
	float accel_angle, brake_angle;
	//( accel_angle + brake_angle ) > angle �������ꍇ�̍ō��p���x
	float top_omega2;
	//���p���x
	float start_omega = 0.0;
	
	//�ڕW�p�x, �ō��p���x, �I�[�p���x, �p�����x�ݒ�
	angle = _angle;
	//target_speed = tar_vel;
	if( angle < 0.0 )	target_omega = -_top_omega;
	else			target_omega = _top_omega;
	alpha = _alpha;
	
	//���[�^�[ON, ���x����ON, �ǐ���OFF, �^�[���t���OOFF
	MOT_STBY = 1;
	speed_control_flg = 1;
	wall_control_flg = 0;
	turn_flg = 0;
	
	//�ڕW�p�x�����Ȃ�v�Z�̂��ߐ��ɒ���
	if( angle < 0 )	angle = -angle;
	
	//���p���x�v��
	start_omega = current_omega;
	//������Ԋp�x, ������Ԋp�x�Z�o
	accel_angle = ( ( _top_omega + start_omega ) * ( _top_omega - start_omega ) ) / ( 2.0 * _alpha );
	brake_angle = ( ( _top_omega + _end_omega ) * ( _top_omega - _end_omega ) ) / ( 2.0 * _alpha );
	//���� + ������Ԋp�x���ڕW�p�x�𒴂���ꍇ, �����p�x�ύX
	if( angle < ( accel_angle + brake_angle ) ){
		//top_omega^2 - start_omega^2 = 2.0 * alpha * x1(�����p�x)
		//end_omega^2 - top_omega^2 = 2.0 * -alpha * x2(�����p�x)
		//(x1 + x2) = angle
		//���ō��p���x��
		top_omega2 = ( ( 2.0 * _alpha * angle ) + ( start_omega * start_omega ) + ( _end_omega * _end_omega ) ) / 2.0;
		//2�Ԗڂ̌�����Ԋp�x�̎��ɑ�������
		brake_angle = ( top_omega2 - ( _end_omega * _end_omega ) ) / ( 2.0 * _alpha );
	}
	
	//�߂�
	angle = _angle;
	
	//�ڕW�p�x�����̏ꍇ
	if( _angle > 0 ){
		//�p�����J�n��Ԃ܂ő҂�
		while( current_angle < ( angle - brake_angle ) );
		//�p�����J�n
		target_omega = _end_omega;
		//�I�[�p���x0�Ȃ�Œ�p���x��ݒ�
		if( _end_omega > -0.0009 && _end_omega < 0.0009 ){
			target_omega = MIN_OMEGA;
		}
		//�ڕW�p�x�܂ő҂�
		while( current_angle < angle);
		
	//�ڕW�p�x�����̏ꍇ
	}else if( _angle < 0.0 ){
		//�p�����J�n��Ԃ܂ő҂�
		while( current_angle > -( -angle - brake_angle ) );
		//�p�����J�n
		target_omega = -_end_omega;
		//�I�[�p���x0�Ȃ�Œ�p���x��ݒ�
		if( _end_omega > -0.0009 && _end_omega < 0.0009 ){
			target_omega = -MIN_OMEGA;
		}
		//�ڕW�p�x�܂ő҂�
		while( current_angle > angle);
		
	}
	
	
	angle = 0;
	alpha = 0;
	tar_omega = 0;
	current_omega = 0;
	
	tar_dis = 0;
	current_dis_r = 0;
	current_dis_l = 0;
	current_dis_ave = 0;
	tar_angle = 0;
	current_angle = 0;
	
}

void forward_stop(void){
	if (forward_wall_stop_flg == 0) return;
	int l_sen_val = get_sen_value(LS_SEN);
	int r_sen_val = get_sen_value(RS_SEN);
	if ((l_sen_val + r_sen_val) / 2 > FRONT_WALL_STOPR){
		if(l_sen_val < 850 || r_sen_val < 270){
			return;
		}
		else{
		//log_save(l_sen_val);
		//log_save(r_sen_val);
		//log_save(0);
		tar_vel = 0;
		length_reset_flg = 1;
		}
	}
}

void no_attack(void){
	MOT_STBY = 1;
	speed_control_flg = 1;
	if(( get_sen_value(LS_SEN) + get_sen_value(RS_SEN) ) / 2 < FRONT_WALL_STOPR){
		target_speed = -0.1;
	}
	else if (( get_sen_value(LS_SEN) + get_sen_value(RS_SEN) ) / 2 > FRONT_WALL_STOPR + 10){
		target_speed = 0.1;
	}
	else{
		target_speed = 0;
	}
}

//���x����( 1ms���荞�� )
void control_speed(void){
	
	//���x����t���O��0�Ȃ�return
	if( speed_control_flg == 0 ) return;
	
	//����
	if( tar_vel < target_speed ){
		//�����x�ɉ����đ��x�X�V
		tar_vel += accel / 1000.0;
		run_state = 1;//����
		if( tar_vel >= target_speed ){
			tar_vel = target_speed;
			run_state = 2;//����
		}
	//����
	}else if( tar_vel > target_speed ){
		//�����x�ɉ����đ��x�X�V
		tar_vel -= accel / 1000.0;
		run_state = 3;//����
		if( tar_vel <= target_speed ){
			tar_vel = target_speed;
			run_state = 2;
		}
	}
	
	//���x�ɉ����ċ����X�V
	tar_dis += tar_vel / 1000.0;
	
	//�p����
	if( tar_omega < target_omega ){
		//�p�����x�ɉ����Ċp���x�X�V
		tar_omega += alpha / 1000.0;
		turn_state = 1;//�p����
		if( tar_omega >= target_omega ){
			tar_omega = target_omega;
			turn_state = 2;//�p����
		}
	//�p����
	}else if( tar_omega > target_omega ){
		//�p�����x�ɉ����Ċp���x�X�V
		tar_omega -= alpha / 1000.0;
		turn_state = 3;//�p����
		if( tar_omega <= target_omega ){
			tar_omega = target_omega;
			turn_state = 2;//�p����
		}
	}
	
	//�p���x�ɉ����Ċp�x�X�V
	tar_angle += tar_omega / 1000.0;
	
	//�ߋ����x�ۑ�
	previous_vel_r = current_vel_r;
	previous_vel_l = current_vel_l;
	
	//���x�擾( [mm] -> [m] )
	current_vel_r = get_current_enc_velocity(RIGHT_ENCODER) / 1000.0;
	current_vel_l = get_current_enc_velocity(LEFT_ENCODER) / 1000.0;
	current_vel_ave = ( current_vel_r + current_vel_l ) / 2.0;
	
	//�p���x[deg/s]�Z�o
	//�̂��̂��W���C��
	current_omega = ( ( current_vel_r - current_vel_l ) / TREAD ) * ( 180.0 / PI );
	
	//���ۂɐi�񂾋���
	current_dis_r += current_vel_r / 1000.0;
	current_dis_l += current_vel_l / 1000.0;
	current_dis_ave = (current_dis_r + current_dis_l) / 2.0;
	
	//���ۂ̊p�x
	current_angle += current_omega / 1000.0;
	//log_save((short)(get_sen_value(LS_SEN) + get_sen_value(RS_SEN)));
	//log_save((short)(current_vel_ave*1000.0));
	//log_save((short)(tar_vel*1000.0));
	if(get_time(TYPE_MYMS) % 2 == 0) {
		//log_save((short)(current_omega), (short)(tar_omega),(short)(current_angle),(short)(angle));
		//log_save((short)(current_vel_r*1000), (short)(current_vel_l*1000),(short)(tar_vel*1000),(short)(angle));
		//log_save((short)(current_dis_ave*1000), (short)(current_vel_ave*1000), (short)(tar_vel*1000), (short)(length*1000));
		//log_save(1,1);
		
	}
	//log_save((short)(tar_omega));
	//log_save((short)(current_dis_ave*1000.0));
}

//PID����( 1ms���荞�� )
void pid_speed(void){
	//���xPID����
	static float vel_error_p, pre_vel_error_p;
	static float vel_error_pr, pre_vel_error_pr,vel_error_ir = 0.0;
	static float vel_error_pl, pre_vel_error_pl,vel_error_il = 0.0;
	static float vel_error_i = 0.0;
	float vel_error_d;
	//�p���xPID����
	static float omega_error_p, pre_omega_error_p;
	static float omega_error_i = 0.0;
	float omega_error_d;
	//FB�����
	float r_control, l_control;
		
	
	//���x����t���O��0�Ȃ�return
	if( speed_control_flg == 0 ) {
		vel_error_p=0.0; 
		pre_vel_error_p=0.0;
		vel_error_i = 0.0;
		omega_error_i = 0.0;
		omega_error_p=0.0; 
		pre_omega_error_p=0;
		
		return;
	}
	
	//FF��
	V_r = 0;
	V_l = 0;
	//���ʂ̃^�[�����͎g��Ȃ�
	if(turn_flg ==  0) {
		if( run_state == 1 ){		//�������
			V_r += ( tar_vel * FF_KV ) + ( accel * FF_KA ) + FF_FRIC_R;
			V_l += ( tar_vel * FF_KV ) + ( accel * FF_KA ) + FF_FRIC_L;
		}else if( run_state == 2 ){	//�������
			V_r += ( tar_vel * FF_KV ) + ( 0 * FF_KA ) + FF_FRIC_R;
			V_l += ( tar_vel * FF_KV ) + ( 0 * FF_KA ) + FF_FRIC_L;
		}else if( run_state == 3 ){	//�������
			V_r += ( tar_vel * FF_KV ) + ( -accel * FF_KA ) + FF_FRIC_R;
			V_l += ( tar_vel * FF_KV ) + ( -accel * FF_KA ) + FF_FRIC_L;
		}
	}

	if( turn_state == 1 ){		//�p�������
		V_r += ( tar_omega * ( PI / 180.0 ) * FF_KOMEGA ) + ( alpha * ( PI / 180.0 ) * FF_KALPHA ) + ( turn_flg * get_sign(angle) * FF_FRIC_R );
		V_l -= ( tar_omega * ( PI / 180.0 ) * FF_KOMEGA ) + ( alpha * ( PI / 180.0 ) * FF_KALPHA ) + ( turn_flg * get_sign(angle) * FF_FRIC_L );
	}else if( turn_state == 2 ){	//�p�������
		V_r += ( tar_omega * ( PI / 180.0 ) * FF_KOMEGA ) + ( 0 * ( PI / 180.0 ) * FF_KALPHA ) + ( turn_flg * get_sign(angle) * FF_FRIC_R );
		V_l -= ( tar_omega * ( PI / 180.0 ) * FF_KOMEGA ) + ( 0 * ( PI / 180.0 ) * FF_KALPHA ) + ( turn_flg * get_sign(angle) * FF_FRIC_L );
	}else if( turn_state == 3 ){	//�p�������
		V_r += ( tar_omega * ( PI / 180.0 ) * FF_KOMEGA ) + ( -alpha * ( PI / 180.0 ) * FF_KALPHA ) + ( turn_flg * get_sign(angle) * FF_FRIC_R );
		V_l -= ( tar_omega * ( PI / 180.0 ) * FF_KOMEGA ) + ( -alpha * ( PI / 180.0 ) * FF_KALPHA ) + ( turn_flg * get_sign(angle) * FF_FRIC_L );
	}
	//turn�����E�ʁX�Ɍ���(r_val = -l_val�Ƃ݂Ȃ���)
	if(turn_flg == 2) {
		//���xPI�v�Z
		pre_vel_error_pr = vel_error_pr;
		vel_error_pr =  tar_omega*TREAD*PI/(2*180) - current_vel_r ;
		vel_error_ir += vel_error_pr * DELTA_T;
		//vel_error_dr = ( vel_error_pr - pre_vel_error_pr ) * DELTA_T;
		pre_vel_error_pl = vel_error_pl;
		vel_error_pl =  tar_omega*TREAD*PI/(2*180) - current_vel_l ;
		vel_error_il += vel_error_pl * DELTA_T;
		//FB����ʌv�Z
		r_control = ( OMEGA_KPV * vel_error_pr ) + ( OMEGA_KIV * vel_error_ir ); //+ ( VEL_KD * vel_error_dr );
		l_control = -( OMEGA_KPV * vel_error_pl ) - ( OMEGA_KIV * vel_error_il ); //+ ( VEL_KD * vel_error_dl );
	
	}
	else {
	//���xPI�v�Z
	pre_vel_error_p = vel_error_p;
	vel_error_p = ( tar_vel - current_vel_ave );
	vel_error_i += vel_error_p * DELTA_T;
	vel_error_d = ( vel_error_p - pre_vel_error_p ) * DELTA_T;
	
	//�p���xPI�v�Z
	pre_omega_error_p = omega_error_p;
	omega_error_p = ( tar_omega - current_omega );
	omega_error_i += omega_error_p * DELTA_T;
	omega_error_d = ( omega_error_p - pre_omega_error_p ) * DELTA_T;
	
	//FB����ʌv�Z
	r_control = ( VEL_KP * vel_error_p ) + ( VEL_KI * vel_error_i ) + ( VEL_KD * vel_error_d ) + ( ( OMEGA_KP / 100.0 ) * omega_error_p ) + ( ( OMEGA_KI / 100.0 ) * omega_error_i ) + ( ( OMEGA_KD / 100.0 ) * omega_error_d );
	l_control = ( VEL_KP * vel_error_p ) + ( VEL_KI * vel_error_i ) + ( VEL_KD * vel_error_d ) - ( ( OMEGA_KP / 100.0 ) * omega_error_p ) - ( ( OMEGA_KI / 100.0 ) * omega_error_i ) - ( ( OMEGA_KD / 100.0 ) * omega_error_d );
	}
	if(get_time(TYPE_MYMS) % 2 == 0) {
		//log_save((short)(current_omega), (short)(tar_omega),(short)(current_angle),(short)(angle));
		//log_save((short)(r_control*1000), (short)(l_control*1000),(short)(tar_vel*1000),(short)(angle));
		//log_save((short)(current_dis_ave*1000), (short)(current_vel_ave*1000), (short)(tar_vel*1000), (short)(tar_dis*1000));
		//log_save(1,1);
		
	}
	//���ǐ���
	if( wall_control_flg == 1 )
	{
		r_control += error * WALLR_KP;
		l_control -= error * WALLR_KP;
	}
	//�O�ǐ���
	if (f_wall_control_flg == 1)
	{
		//�O�㋗��
		r_control += f_dis_err * F_WALL_KP;
		l_control += f_dis_err * F_WALL_KP;
		//�p�x
		r_control += error * F_WALL_KP;
		l_control -= error * F_WALL_KP;
	}

	//�o�͓d���v�Z
	V_r += r_control;
	V_l += l_control;
	
	//�E���[�^�̏o�͓d�������̏ꍇ, ���̏ꍇ
	if(V_r > 0.0){
		direction_r_mot(MOT_FORWARD);	//���]
	}else{
		direction_r_mot(MOT_BACKWARD);	//�t�]
		V_r = -V_r;			//�d���𐳂ɒ���
	}
	
	//�����[�^�̏o�͓d�������̏ꍇ, ���̏ꍇ
	if(V_l > 0.0){
		direction_l_mot(MOT_FORWARD);	//���]
	}else{
		direction_l_mot(MOT_BACKWARD);	//�t�]
		V_l = -V_l;			//�d���𐳂ɒ���
	}
	
	//�d���d���擾
	V_bat = get_battery_voltage();
	
	//duty[%]�Z�o
	duty_r = (V_r / V_bat) * 100;
	duty_l = (V_l / V_bat) * 100;
	
	//�t�F�C���Z�[�t
	if(duty_r >= 80) duty_r = 80;
	if(duty_l >= 80) duty_l = 80;
	//log_save(V_r, V_l,duty_r,duty_l);
}

//�ǐ���( 1ms���荞�� )
//*���x������O�ɋL�q
void control_wall(void){
	if( wall_control_flg == 0 ) return;
	//���ǃZ���T臒l
	short l_threshold = LEFT_THRESHOLD;
	short r_threshold = RIGHT_THRESHOLD;
	
	//�@�̒��S���̉��ǃZ���T��l
	short ref_lf = REF_LF;
	short ref_rf = REF_RF;
	
	//�Z���T�l�i�[
	short sensor_lf = get_sen_value(LF_SEN);
	short sensor_rf = get_sen_value(RF_SEN);
	
	if( (sensor_rf > r_threshold) && (sensor_lf > l_threshold) ){
		//���ǂ�����ꍇ
		error = (sensor_rf - ref_rf) - (sensor_lf - ref_lf);
	}else if( (sensor_rf <= r_threshold) && (sensor_lf <= l_threshold) ){
		//���ǂƂ������ꍇ
		error = 0;
	}else if( sensor_rf > r_threshold ){
		//�E�ǂ݂̂���ꍇ
		error = 2 * (sensor_rf - ref_rf);
	}else{
		//���ǂ݂̂���ꍇ
		error = -2 * ((sensor_lf - ref_lf));
	}
}

void enable_f_wall_control(void) {
	MOT_STBY = 1;
	f_wall_control_flg = 1;
	speed_control_flg = 1;
	wait_sec(10);
	MOT_STBY = 0;
}

//�O�ǐ���(1ms���荞��)
void f_wall_control(void) {
	if(f_wall_control_flg == 0) return;
	short ref_ls = FRONT_WALL_STOPL;
	short ref_rs = FRONT_WALL_STOPR;
	short sensor_ls = get_sen_value(LS_SEN);
	short sensor_rs = get_sen_value(RS_SEN);
	//�O��
		f_dis_err = (ref_rs - sensor_rs) + (ref_ls - sensor_ls);
	//�p�x
	//�E�ɕ΂��Ă�Ƃ�
	if(ref_rs > sensor_rs && ref_ls < sensor_ls) {
		error = -(sensor_rs - ref_rs) + (sensor_ls - ref_ls);
	}
	//���ɕ΂��Ă��鎞
	else if (ref_rs < sensor_rs && ref_ls > sensor_ls) {
		error = -(sensor_rs - ref_rs) + (sensor_ls - ref_ls);
	}
	else {
		error = 0;
	}
	log_save(f_dis_err, error, 0, 0);
}

//���[�^�[���x�ω�( 10us���荞�� )
void change_motor_speed(void){
	//���[�^�[���x�ω��t���O��0�Ȃ�return
	if( speed_control_flg == 0 ) return;
	//���E���[�^��Duty����, ��肾��
	MOT_DUTY_L = duty_to_count(duty_l);
	MOT_DUTY_R = duty_to_count(duty_r);
}

void reset_run_status(void){
	accel = 0;
	alpha = 0;
	tar_vel = 0;
	tar_dis = 0;
	tar_omega = 0;
	tar_angle = 0;
	previous_vel_r = 0;
	previous_vel_l = 0;
	current_vel_r = 0;
	current_vel_l = 0;
	current_vel_ave = 0;
	current_dis_r = 0;
	current_dis_l = 0;
	current_dis_ave = 0;
	current_omega = 0;
	current_angle = 0;
	V_r = 0;
	V_l = 0;
}

void print_state_test(void){
	while(1){
		sci_printf("tar_omega:%d, current_omega:%d \r\n", (short)tar_omega, (short)current_omega);
		sci_printf("tar_angle:%d, current_angle:%d \r\n", (short)(tar_angle), (short)(current_angle));
	}
}
