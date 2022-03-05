#include "parameter.h"
#include "sci.h"
#include "search.h"
#include "run.h"
#include "interrupt.h"
#include "function_test.h"
#include "sensor.h"
#include "maze.h"
#include "log.h"
#include "define.h"
#include "math.h"
char before_flg = 0;
/*
//����@�ɂ��T�����s���֐�
void left_hand(void){
	
	//(0,0)����������
	init_wall();
	init_stepMap();
	m_dir = 0;	//�ŏ��͖k����
	
	straight(HALF_SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
	
	while(1){
		
		update_coordinate();	//���W�X�V
		set_wall(x_coordinate, y_coordinate, m_dir);	//�ǔ���, �i�[
		if( goal_judge() == 1 )	break;		//�S�[����������
		
		if( get_sen_value(LF_SEN) < LEFT_THRESHOLD ){	//���ǖ���
			
			straight(0.015, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			slalom(90.0, TURN_OMEGA, 0, TURN_ALPHA);
			straight(0.08, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			if(--m_dir < 0)	m_dir = 3;
			
		}else if( ( get_sen_value(LS_SEN) + get_sen_value(RS_SEN) ) / 2 < FRONT_THRESHOLD ){	//�O�ǖ���
			LED_2 = ~LED_2;
			straight(SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			
		}else if( get_sen_value(RF_SEN) < RIGHT_THRESHOLD ){	//�E�ǖ���
		
			straight(0.015, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			slalom(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
			straight(0.08, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			if(++m_dir > 3)	m_dir = 0;
			
		}else{	//U�^�[��
		
			straight(0, SEARCH_SPEED, 0, SEARCH_ACCEL, 1);
			turn(180.0, 700, 0, 7000);
			if(--m_dir < 0)	m_dir = 3;
			if(--m_dir < 0)	m_dir = 3;
			straight(HALF_SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			
		}
	}
	//�I���s��
	straight(0.06, SEARCH_SPEED, 0, SEARCH_ACCEL, 1);
	turn(180.0, 700, 0, 7000);
}
*/

//�����@�ɂ��T�����s���֐�
void adachi_method(void){
	
	char nextdir;
	//(0,0)����������
	init_wall();		//�Ǐ�񏉊���
	init_stepMap();		//�����}�b�v������
	update_stepMap();	//�����}�b�v�쐬
	m_dir = 0;
			//�ŏ��͖k����
	turn(90.0, TURN_OMEGA, 0, TURN_ALPHA);
	revision_back(0.2, 0.5, 3);
	straight(0.045, SEARCH_SPEED, 0, SEARCH_ACCEL, 0, 0, -1);
	turn(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
	revision_back(0.2, 0.5, 3);
	straight(0.145, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1,0,-1);
	
	while(1){
		update_coordinate();	//���W�X�V
		set_wall(x_coordinate, y_coordinate, m_dir);	//�ǔ���, �i�[
		
		//���ݍ��W�̑S�Ă̕ǂ����m�Ƃ���
		add_knownWall(x_coordinate, y_coordinate, NORTH);
		add_knownWall(x_coordinate, y_coordinate, EAST);
		add_knownWall(x_coordinate, y_coordinate, SOUTH);
		add_knownWall(x_coordinate, y_coordinate, WEST);
		
		init_stepMap();		//�����}�b�v������
		update_stepMap();	//�����}�b�v�쐬
		if( goal_judge() == 1 )	break;		//�S�[����������
		//�i�s�������f
		nextdir = adachi_judge_nextdir();
		//���i
		LED(nextdir);
		if( nextdir == 0 ){
			//log_save(0,0,0,0);
			if (before_flg == 1){
				straight(SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 0,0,nextdir);
			}
			else{
			straight(SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1,0,nextdir);
			}
			
			
		}
		//�E��
		if( nextdir == 1 ){
			//log_save(1,1,1,1);
			//before_flg = 1;
			//log_save(1,1,1,1);
			// �����ǂȂ��Ƃ��ǐ�������ƃo�O�邩��
			straight(HALF_SECTION, SEARCH_SPEED, 0, SEARCH_ACCEL, 0, 1, nextdir);
			//straight(HALF_SECTION, SEARCH_SPEED, 0, SEARCH_ACCEL, 0, 0, nextdir);
			wait_ms(1); //�ŏ����ԗ։��Ȃ��Ȃ�
			turn(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
			//wait_ms(100);
			straight(HALF_SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1,0,-1);
			if(++m_dir > 3)	m_dir = 0;
		}
		//U�^�[��
		if( nextdir == 2 ){
			//log_save(2,2,2,2);
			
			straight(0.04, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 0,0,-1);
			unsigned int left_val = abs((int)get_sen_value(LF_SEN) - REF_LF);
			unsigned int right_val = abs((int)get_sen_value(RF_SEN) - REF_RF);
			straight(0.04, SEARCH_SPEED, 0, SEARCH_ACCEL, 0,1,nextdir);
			//straight(0.04, SEARCH_SPEED, 0, SEARCH_ACCEL, 0,0,nextdir);
			wait_ms(100);
			
			
			//�E��]
			if( (left_val >  right_val) && exist_l_wall == 1 && exist_f_wall == 1) {
				
				turn(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
				revision_back(0.3,0.5,5);
				straight(0.045, SEARCH_SPEED, 0, SEARCH_ACCEL, 0,0,-1);
				wait_ms(100);
				
				turn(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
				revision_back(0.3,0.5,5);
				wait_ms(100);
			}
			//����]
			else if(exist_r_wall == 1 && exist_f_wall == 1){
				turn(90.0, TURN_OMEGA, 0, TURN_ALPHA);
				revision_back(0.3,0.5,5);
				straight(0.045, SEARCH_SPEED, 0, SEARCH_ACCEL, 0,0,-1);
				wait_ms(100);
				
				turn(90.0, TURN_OMEGA, 0, TURN_ALPHA);
				
				revision_back(0.3,0.5,5);
				wait_ms(100);
				
			}
			else if(exist_f_wall == 1) {
				turn(180.0, TURN_OMEGA, 0, TURN_ALPHA);
				revision_back(0.3,0.5,5);
			}
			else {
				turn(180.0, TURN_OMEGA, 0, TURN_ALPHA);
				revision_back(0.035,0.5,5);
			}
				
				
			straight(0.145, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1,0,-1);
			
			//U�^�[�������� m_dir -= 2 
			if(--m_dir < 0)	m_dir = 3;
			if(--m_dir < 0)	m_dir = 3;
			
				
			
		}
		//����
		if( nextdir == 3 ){
			//log_save(3,3,3,3);
			straight(HALF_SECTION, SEARCH_SPEED, 0, SEARCH_ACCEL, 0, 0, nextdir);
			//wait_ms(100);
			turn(90.0, TURN_OMEGA, 0, TURN_ALPHA);
			//wait_ms(100);
			straight(HALF_SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1,0,-1);
			if(--m_dir < 0)	m_dir = 3;
		}
	}
	//�I���s��
	straight(0.06, SEARCH_SPEED, 0, SEARCH_ACCEL, 1,0,nextdir);
	turn(180.0, TURN_OMEGA, 0, TURN_ALPHA);
	//�ǔ���flg������
	init_wall_exist_flg();
	// log_save(9,9,9,9);

	generate_adachi_shortestRoute(); //�ŒZ�o�H�p�X����
}

//���������p�X�ɉ����čŒZ���s���s���֐�
void run_shortestRoute(void){
	short i=0;
	straight(0.048, FAST_SPEED,  FAST_SPEED, FAST_ACCEL, 1,0,-1);
	while(1){
		if( path[i] <= 15 ){		//���i
			straight(SECTION*path[i], FAST_SPEED, 0, FAST_ACCEL, 1,0,-1);
		}else if( path[i] == 20 ){	//�E��
			turn(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
		}else if( path[i] == 30 ){	//����
			turn(90.0, TURN_OMEGA, 0, TURN_ALPHA);
		}
		if( i == path_size )	break;	//�p�X�S�����s���I������烋�[�v�E�o
		i++;
	}
	turn(180.0, TURN_OMEGA, 0, TURN_ALPHA);
}
