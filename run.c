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

/*フラグ--------------------------------------*/
//加減等速状態を知らせる
//turn_flgは超新地旋回時に摩擦項を足すため...(超新地旋回時のみ1, 直進やスラローム時は0)
static char run_state, turn_state, turn_flg;
//速度制御有効フラグ
static char speed_control_flg = 0;
//壁制御有効フラグ
static char wall_control_flg = 0;
static char forward_wall_stop_flg = 0;
static char length_reset_flg = 0;
// 前壁補正用壁制御フラグ
static char f_wall_control_flg = 0;

/*速度, 加速度, 距離-------------------------*/
//目標角度, 最高速度, 終端速度, 加速度, 最高角速度, 終端角速度, 角加速度
static float target_speed, target_omega;
static float angle;
static float accel = 0.0;
static float alpha = 0.0;
//1msごとの追従目標速度, 距離, 角速度, 角度
static volatile float tar_vel = 0.0;
static volatile float tar_dis = 0.0;
static volatile float tar_omega = 0.0;
static volatile float tar_angle = 0.0;
//1ms前の実際の左右速度
static float previous_vel_r = 0.0;
static float previous_vel_l = 0.0;
//実際の左右速度, 中心速度, 角速度
static float current_vel_r = 0.0;
static float current_vel_l = 0.0;
static volatile float current_vel_ave = 0.0;
static float current_omega = 0.0;
//実際の左右距離, 平均, 角度
static volatile float current_dis_r = 0.0;
static volatile float current_dis_l = 0.0;
static volatile float current_dis_ave = 0.0;
static volatile float current_angle = 0.0;
static float length;

/*制御計算用-------------------------*/
//偏差
static short error;
//前壁距離誤差
static short f_dis_err;

/*モータ制御用-------------------------*/
//右モータの出力電圧, 左モータの出力電圧
volatile static float V_r = 0.0;
volatile static float V_l = 0.0;
//バッテリー電圧
static float V_bat = 0.0;
//右モータDuty, 左モータDuty
static short duty_r = 0;
static short duty_l = 0;

//台形加減速
//距離, 最高速, 終端速, 加速度, 壁制御有無( ON:1, OFF:0 ), 前壁衝突防止flg(off:0)
void straight(float _length, float _top_speed, float _end_speed, float _accel, char _wall_control, char _forward_wall,char _nextdir){
	volatile unsigned int start_timer;//タイヤがロックした時の対策
	char nextdir = _nextdir;
	char lock_flg = 0; //タイヤロック処理済みflg
	//目標距離
	//float length;
	//加速に必要な距離, 減速に必要な距離
	float accel_length, brake_length;
	//( accel_length + brake_length ) > length だった場合の最高速
	float top_speed2;
	//初速度
	float start_speed;
	
	//目標距離, 目標速度, 加速度, 目標角度, 目標角速度, 角加速度設定
	length = _length;
	target_speed = _top_speed;
	accel = _accel;
	angle = 0;
	target_omega = 0;
	alpha = 0;
	
	//モーターON, 速度制御ON, 壁制御ON/OFF設定, ターンフラグOFF
	MOT_STBY = 1;
	speed_control_flg = 1;
	wall_control_flg = _wall_control;
	turn_flg = 0;
	forward_wall_stop_flg = _forward_wall;
	
	
	//初速度計測
	start_speed = current_vel_ave;
	//v1^2 - v0^2 = 2ax より機体加速, 減速可能距離を算出
	accel_length = ( ( _top_speed + start_speed ) * ( _top_speed - start_speed ) ) / ( 2.0 * _accel );
	brake_length = ( ( _top_speed + _end_speed ) * ( _top_speed - _end_speed ) ) / ( 2.0 * _accel );
	//加速可能距離+減速可能距離が走行距離より長かったら減速開始地点を変更する
	if( length < ( accel_length + brake_length ) ){
		//top_speed^2 - start_speed^2 = 2.0 * acc * x1(加速距離)
		//end_speed^2 - top_speed^2 = 2.0 * -acc * x2(減速距離)
		//(x1 + x2) = length
		//より, 最高速top_speedは
		top_speed2 = ( ( 2.0 * _accel * length ) + ( start_speed * start_speed ) + ( _end_speed * _end_speed ) ) / 2.0;
		//これを2番目の減速距離の式に代入すれば
		brake_length = ( top_speed2 - ( _end_speed * _end_speed ) ) / ( 2.0 * _accel );
	}
	start_timer = get_time(TYPE_SEC); 
	
	//減速区間まで等速
	while( current_dis_ave < ( length - brake_length ) ) {
		
		//前壁近づいてきたら壁制御切る
		if(get_sen_value(LS_SEN) > TR_SENSOR_FRONT_WALL_L && get_sen_value(RS_SEN) > TR_SENSOR_FRONT_WALL_R){
			wall_control_flg = 0;
		}
		
	}
	
	//減速開始
	target_speed = _end_speed;
	
	//終端速度0　-> 最低速度設定
	if( _end_speed > -0.0009 && _end_speed < 0.0009 )	target_speed = MIN_VEL;
	//目標距離に到達するまで待つ
	while( current_dis_ave < length ){
		//前壁近づいてきたら壁制御切る
		if(get_sen_value(LS_SEN) > TR_SENSOR_FRONT_WALL_L && get_sen_value(RS_SEN) > TR_SENSOR_FRONT_WALL_R){
			wall_control_flg = 0;
		}
		//減速区間半分まで行ったら壁制御切る
		if(current_dis_ave > (length - brake_length/2.0)) {
			wall_control_flg = 0;
		}
		//衝突時修正
		if(forward_wall_stop_flg == 1) {
			//タイヤロック解除処理
			if(get_time(TYPE_SEC) - start_timer > 2 && lock_flg == 0) {
				//back
				reset_run_status();
				revision_back(0.03,SEARCH_SPEED,SEARCH_ACCEL);
				//右折中
				if(nextdir==1) {
					//左壁あるとき
					if(exist_l_wall == 1) {
						turn(-90.0, 300.0, 0, 500.0);
						revision_back(0.3,0.5,5);
						straight(0.035, SEARCH_SPEED, 0, SEARCH_ACCEL, 0,0,-1);
						wait_ms(100);
					
						turn(90.0, 300.0, 0, 500.0);
					}
					//左壁ない時
					else {
						turn(-180.0, 300.0, 0, 500.0);
						revision_back(0.3,0.5,5);
						straight(0.035, SEARCH_SPEED, 0, SEARCH_ACCEL, 0,0,-1);
						wait_ms(100);
						turn(180.0, 300.0, 0, 500.0);
					}
						
				}
				//左折中(右壁あるとき)
				else if(nextdir==3) {
					//右壁あるとき
					if(exist_r_wall == 1) {
						turn(90.0, 300.0, 0, 500.0);
						revision_back(0.3,0.5,5);
						straight(0.035, SEARCH_SPEED, 0, SEARCH_ACCEL, 0,0,-1);
						wait_ms(100);
					
						turn(-90.0, 300.0, 0, 500.0);
					}
					//右壁ない時
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
	
	//停止処理
	if( _end_speed > -0.0009 && _end_speed < 0.0009 ){
		//スピード制御OFF
		speed_control_flg = 0;
		direction_r_mot(MOT_BRAKE);
		direction_l_mot(MOT_BRAKE);
		duty_r = 0;
		duty_l = 0;
		//ステータスリセット
		reset_run_status();
	}
	
	tar_dis = 0;
	current_dis_r = 0;
	current_dis_l = 0;
	current_dis_ave = 0;
	tar_angle = 0;
	current_angle = 0;
}

//超信地回転時の後退 _length > 0 で後退 tarspeed < 0, accell > 0でback?
void revision_back(float _length, float top_speed, float accell){
    volatile unsigned int start_timer;//タイヤがロックした時の対策
    float _top_speed = -top_speed;
    float _end_speed = 0;
    float _accel = accell;
    char _wall_control = 0;
    char _forward_wall = 0;
    //目標距離
	float length;
	//加速に必要な距離, 減速に必要な距離
	float accel_length, brake_length;
	//( accel_length + brake_length ) > length だった場合の最高速
	float top_speed2;
	//初速度
	float start_speed;
	
	//目標距離, 目標速度, 加速度, 目標角度, 目標角速度, 角加速度設定
	length = _length;
	target_speed = _top_speed;
	accel = _accel;
	angle = 0;
	target_omega = 0;
	alpha = 0;
	
	//モーターON, 速度制御ON, 壁制御ON/OFF設定, ターンフラグOFF
	MOT_STBY = 1;
	speed_control_flg = 1;
	wall_control_flg = _wall_control;
	turn_flg = 0;
	forward_wall_stop_flg = _forward_wall;
	
	
	//初速度計測
	start_speed = current_vel_ave;
	//v1^2 - v0^2 = 2ax より機体加速, 減速可能距離を算出
	accel_length = ( ( _top_speed + start_speed ) * ( _top_speed - start_speed ) ) / ( 2.0 * _accel );
	brake_length = ( ( _top_speed + _end_speed ) * ( _top_speed - _end_speed ) ) / ( 2.0 * _accel );
	//加速可能距離+減速可能距離が走行距離より長かったら減速開始地点を変更する
	if( length < ( accel_length + brake_length ) ){
		//top_speed^2 - start_speed^2 = 2.0 * acc * x1(加速距離)
		//end_speed^2 - top_speed^2 = 2.0 * -acc * x2(減速距離)
		//(x1 + x2) = length
		//より, 最高速top_speedは
		top_speed2 = ( ( 2.0 * _accel * length ) + ( start_speed * start_speed ) + ( _end_speed * _end_speed ) ) / 2.0;
		//これを2番目の減速距離の式に代入すれば
		brake_length = ( top_speed2 - ( _end_speed * _end_speed ) ) / ( 2.0 * _accel );
	}
	
	start_timer = get_time(TYPE_SEC); 
	
	//減速区間まで等速
	while( current_dis_ave > -( length - brake_length ) ) {
		//タイヤロック解除処理
		if(get_time(TYPE_SEC) - start_timer > 1) {
			//reset_run_status();
			break;
		}
			
	}
	
	//減速開始
	target_speed = _end_speed;
	//終端速度0　-> 最低速度設定
	if( _end_speed > -0.0009 && _end_speed < 0.0009 )	target_speed = -MIN_VEL;
	//目標距離に到達するまで待つ
	while( current_dis_ave > -length ){
		//壁に衝突しそうになったら距離0
		if (length_reset_flg == 1){
			length_reset_flg = 0;
			break;
		}
		//タイヤロック解除処理
		if(get_time(TYPE_SEC) - start_timer > 1) {
			break;
		}
	}
	
	//停止処理
	if( _end_speed > -0.0009 && _end_speed < 0.0009 ){
		//スピード制御OFF
		speed_control_flg = 0;
		direction_r_mot(MOT_BRAKE);
		direction_l_mot(MOT_BRAKE);
		duty_r = 0;
		duty_l = 0;
		//ステータスリセット
		reset_run_status();
	}
	
	tar_dis = 0;
	current_dis_r = 0;
	current_dis_l = 0;
	current_dis_ave = 0;
	tar_angle = 0;
	current_angle = 0;
}

//超新地旋回
//目標角度, 最高角速度, 終端角速度, 角加速度
void turn( float _angle, float _top_omega, float _end_omega, float _alpha ){

	//加速区間角度, 減速区間角度
	float accel_angle, brake_angle;
	//( accel_angle + brake_angle ) > angle だった場合の最高角速度
	float top_omega2;
	//初角速度
	float start_omega = 0.0;
	
	//目標角度, 目標重心速度, 加速度, 目標角速度, 角加速度設定
	angle = _angle;
	target_speed = 0;
	accel = 0;
	if( angle < 0.0 )	target_omega = -_top_omega;
	else			target_omega = _top_omega;
	alpha = _alpha;
	
	//モーターON, 速度制御ON, 壁制御OFF, ターンフラグON
	MOT_STBY = 1;
	speed_control_flg = 1;
	wall_control_flg = 0;
	turn_flg = 1;
	
	//目標角度が負なら計算のため正に直す
	if( angle < 0.0 )	angle = -angle;
	
	start_omega = current_omega;
	accel_angle = ( ( _top_omega + start_omega ) * ( _top_omega - start_omega ) ) / ( 2.0 * _alpha );
	brake_angle = ( ( _top_omega + _end_omega ) * ( _top_omega - _end_omega ) ) / ( 2.0 * _alpha );
	if( angle < ( accel_angle + brake_angle ) ){
		//top_omega^2 - start_omega^2 = 2.0 * alpha * x1(加速角度)
		//end_omega^2 - top_omega^2 = 2.0 * -alpha * x2(減速角度)
		//(x1 + x2) = angle
		//より, 最高速top_omega2は
		top_omega2 = ( ( 2.0 * _alpha * angle ) + ( start_omega * start_omega ) + ( _end_omega * _end_omega ) ) / 2.0;
		//これを2番目の減速角度の式に代入すれば
		brake_angle = ( top_omega2 - ( _end_omega * _end_omega ) ) / ( 2.0 * _alpha );
	}
	
	//戻す
	angle = _angle;
	
	//目標角度が正の場合
	if( _angle > 0.0 ){
		//減速開始区間まで待つ
		while( current_angle < ( angle - brake_angle ) );
		//減速開始
		target_omega = _end_omega;
		//終端角速度が0の場合最低角速度設定
		if( _end_omega > -0.0009 && _end_omega < 0.0009 ){
			target_omega = MIN_OMEGA;
		}
		//目標角度まで待つ
		while( current_angle < angle );
		
	//目標角度が負の場合
	}else if( _angle < 0.0 ){
		//減速開始区間まで待つ
		while( current_angle > -( - angle - brake_angle ) );
		//減速開始
		target_omega = _end_omega;
		//終端角速度が0の場合最低角速度設定
		if( _end_omega > -0.0009 && _end_omega < 0.0009 ){
			target_omega = -MIN_OMEGA;
		}
		//目標角度まで待つ
		while( current_angle > angle );
		
	}
	
	//スピード制御OFF
	speed_control_flg = 0;
	direction_r_mot(MOT_BRAKE);
	direction_l_mot(MOT_BRAKE);
	duty_l = 0;
	duty_r = 0;
	//ステータスリセット
	reset_run_status();
	
}

//スラローム( 重心速度維持 )
//目標角度, 最高角速度, 終端角速度, 角加速度
void slalom( float _angle, float _top_omega, float _end_omega, float _alpha ){
	
	//目標角度
	//float angle;
	//角加速区間角度, 角減速区間角度
	float accel_angle, brake_angle;
	//( accel_angle + brake_angle ) > angle だった場合の最高角速度
	float top_omega2;
	//初角速度
	float start_omega = 0.0;
	
	//目標角度, 最高角速度, 終端角速度, 角加速度設定
	angle = _angle;
	//target_speed = tar_vel;
	if( angle < 0.0 )	target_omega = -_top_omega;
	else			target_omega = _top_omega;
	alpha = _alpha;
	
	//モーターON, 速度制御ON, 壁制御OFF, ターンフラグOFF
	MOT_STBY = 1;
	speed_control_flg = 1;
	wall_control_flg = 0;
	turn_flg = 0;
	
	//目標角度が負なら計算のため正に直す
	if( angle < 0 )	angle = -angle;
	
	//初角速度計測
	start_omega = current_omega;
	//加速区間角度, 減速区間角度算出
	accel_angle = ( ( _top_omega + start_omega ) * ( _top_omega - start_omega ) ) / ( 2.0 * _alpha );
	brake_angle = ( ( _top_omega + _end_omega ) * ( _top_omega - _end_omega ) ) / ( 2.0 * _alpha );
	//加速 + 減速区間角度が目標角度を超える場合, 減速角度変更
	if( angle < ( accel_angle + brake_angle ) ){
		//top_omega^2 - start_omega^2 = 2.0 * alpha * x1(加速角度)
		//end_omega^2 - top_omega^2 = 2.0 * -alpha * x2(減速角度)
		//(x1 + x2) = angle
		//より最高角速度は
		top_omega2 = ( ( 2.0 * _alpha * angle ) + ( start_omega * start_omega ) + ( _end_omega * _end_omega ) ) / 2.0;
		//2番目の減速区間角度の式に代入すれば
		brake_angle = ( top_omega2 - ( _end_omega * _end_omega ) ) / ( 2.0 * _alpha );
	}
	
	//戻す
	angle = _angle;
	
	//目標角度が正の場合
	if( _angle > 0 ){
		//角減速開始区間まで待つ
		while( current_angle < ( angle - brake_angle ) );
		//角減速開始
		target_omega = _end_omega;
		//終端角速度0なら最低角速度を設定
		if( _end_omega > -0.0009 && _end_omega < 0.0009 ){
			target_omega = MIN_OMEGA;
		}
		//目標角度まで待つ
		while( current_angle < angle);
		
	//目標角度が負の場合
	}else if( _angle < 0.0 ){
		//角減速開始区間まで待つ
		while( current_angle > -( -angle - brake_angle ) );
		//角減速開始
		target_omega = -_end_omega;
		//終端角速度0なら最低角速度を設定
		if( _end_omega > -0.0009 && _end_omega < 0.0009 ){
			target_omega = -MIN_OMEGA;
		}
		//目標角度まで待つ
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

//速度制御( 1ms割り込み )
void control_speed(void){
	
	//速度制御フラグが0ならreturn
	if( speed_control_flg == 0 ) return;
	
	//加速
	if( tar_vel < target_speed ){
		//加速度に応じて速度更新
		tar_vel += accel / 1000.0;
		run_state = 1;//加速
		if( tar_vel >= target_speed ){
			tar_vel = target_speed;
			run_state = 2;//等速
		}
	//減速
	}else if( tar_vel > target_speed ){
		//加速度に応じて速度更新
		tar_vel -= accel / 1000.0;
		run_state = 3;//減速
		if( tar_vel <= target_speed ){
			tar_vel = target_speed;
			run_state = 2;
		}
	}
	
	//速度に応じて距離更新
	tar_dis += tar_vel / 1000.0;
	
	//角加速
	if( tar_omega < target_omega ){
		//角加速度に応じて角速度更新
		tar_omega += alpha / 1000.0;
		turn_state = 1;//角加速
		if( tar_omega >= target_omega ){
			tar_omega = target_omega;
			turn_state = 2;//角等速
		}
	//角減速
	}else if( tar_omega > target_omega ){
		//角加速度に応じて角速度更新
		tar_omega -= alpha / 1000.0;
		turn_state = 3;//角減速
		if( tar_omega <= target_omega ){
			tar_omega = target_omega;
			turn_state = 2;//角等速
		}
	}
	
	//角速度に応じて角度更新
	tar_angle += tar_omega / 1000.0;
	
	//過去速度保存
	previous_vel_r = current_vel_r;
	previous_vel_l = current_vel_l;
	
	//速度取得( [mm] -> [m] )
	current_vel_r = get_current_enc_velocity(RIGHT_ENCODER) / 1000.0;
	current_vel_l = get_current_enc_velocity(LEFT_ENCODER) / 1000.0;
	current_vel_ave = ( current_vel_r + current_vel_l ) / 2.0;
	
	//角速度[deg/s]算出
	//のちのちジャイロ
	current_omega = ( ( current_vel_r - current_vel_l ) / TREAD ) * ( 180.0 / PI );
	
	//実際に進んだ距離
	current_dis_r += current_vel_r / 1000.0;
	current_dis_l += current_vel_l / 1000.0;
	current_dis_ave = (current_dis_r + current_dis_l) / 2.0;
	
	//実際の角度
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

//PID制御( 1ms割り込み )
void pid_speed(void){
	//速度PID成分
	static float vel_error_p, pre_vel_error_p;
	static float vel_error_pr, pre_vel_error_pr,vel_error_ir = 0.0;
	static float vel_error_pl, pre_vel_error_pl,vel_error_il = 0.0;
	static float vel_error_i = 0.0;
	float vel_error_d;
	//角速度PID成分
	static float omega_error_p, pre_omega_error_p;
	static float omega_error_i = 0.0;
	float omega_error_d;
	//FB制御量
	float r_control, l_control;
		
	
	//速度制御フラグが0ならreturn
	if( speed_control_flg == 0 ) {
		vel_error_p=0.0; 
		pre_vel_error_p=0.0;
		vel_error_i = 0.0;
		omega_error_i = 0.0;
		omega_error_p=0.0; 
		pre_omega_error_p=0;
		
		return;
	}
	
	//FF項
	V_r = 0;
	V_l = 0;
	//普通のターン時は使わない
	if(turn_flg ==  0) {
		if( run_state == 1 ){		//加速状態
			V_r += ( tar_vel * FF_KV ) + ( accel * FF_KA ) + FF_FRIC_R;
			V_l += ( tar_vel * FF_KV ) + ( accel * FF_KA ) + FF_FRIC_L;
		}else if( run_state == 2 ){	//等速状態
			V_r += ( tar_vel * FF_KV ) + ( 0 * FF_KA ) + FF_FRIC_R;
			V_l += ( tar_vel * FF_KV ) + ( 0 * FF_KA ) + FF_FRIC_L;
		}else if( run_state == 3 ){	//減速状態
			V_r += ( tar_vel * FF_KV ) + ( -accel * FF_KA ) + FF_FRIC_R;
			V_l += ( tar_vel * FF_KV ) + ( -accel * FF_KA ) + FF_FRIC_L;
		}
	}

	if( turn_state == 1 ){		//角加速状態
		V_r += ( tar_omega * ( PI / 180.0 ) * FF_KOMEGA ) + ( alpha * ( PI / 180.0 ) * FF_KALPHA ) + ( turn_flg * get_sign(angle) * FF_FRIC_R );
		V_l -= ( tar_omega * ( PI / 180.0 ) * FF_KOMEGA ) + ( alpha * ( PI / 180.0 ) * FF_KALPHA ) + ( turn_flg * get_sign(angle) * FF_FRIC_L );
	}else if( turn_state == 2 ){	//角等速状態
		V_r += ( tar_omega * ( PI / 180.0 ) * FF_KOMEGA ) + ( 0 * ( PI / 180.0 ) * FF_KALPHA ) + ( turn_flg * get_sign(angle) * FF_FRIC_R );
		V_l -= ( tar_omega * ( PI / 180.0 ) * FF_KOMEGA ) + ( 0 * ( PI / 180.0 ) * FF_KALPHA ) + ( turn_flg * get_sign(angle) * FF_FRIC_L );
	}else if( turn_state == 3 ){	//角減速状態
		V_r += ( tar_omega * ( PI / 180.0 ) * FF_KOMEGA ) + ( -alpha * ( PI / 180.0 ) * FF_KALPHA ) + ( turn_flg * get_sign(angle) * FF_FRIC_R );
		V_l -= ( tar_omega * ( PI / 180.0 ) * FF_KOMEGA ) + ( -alpha * ( PI / 180.0 ) * FF_KALPHA ) + ( turn_flg * get_sign(angle) * FF_FRIC_L );
	}
	//turn時左右別々に見る(r_val = -l_valとみなせる)
	if(turn_flg == 2) {
		//速度PI計算
		pre_vel_error_pr = vel_error_pr;
		vel_error_pr =  tar_omega*TREAD*PI/(2*180) - current_vel_r ;
		vel_error_ir += vel_error_pr * DELTA_T;
		//vel_error_dr = ( vel_error_pr - pre_vel_error_pr ) * DELTA_T;
		pre_vel_error_pl = vel_error_pl;
		vel_error_pl =  tar_omega*TREAD*PI/(2*180) - current_vel_l ;
		vel_error_il += vel_error_pl * DELTA_T;
		//FB制御量計算
		r_control = ( OMEGA_KPV * vel_error_pr ) + ( OMEGA_KIV * vel_error_ir ); //+ ( VEL_KD * vel_error_dr );
		l_control = -( OMEGA_KPV * vel_error_pl ) - ( OMEGA_KIV * vel_error_il ); //+ ( VEL_KD * vel_error_dl );
	
	}
	else {
	//速度PI計算
	pre_vel_error_p = vel_error_p;
	vel_error_p = ( tar_vel - current_vel_ave );
	vel_error_i += vel_error_p * DELTA_T;
	vel_error_d = ( vel_error_p - pre_vel_error_p ) * DELTA_T;
	
	//角速度PI計算
	pre_omega_error_p = omega_error_p;
	omega_error_p = ( tar_omega - current_omega );
	omega_error_i += omega_error_p * DELTA_T;
	omega_error_d = ( omega_error_p - pre_omega_error_p ) * DELTA_T;
	
	//FB制御量計算
	r_control = ( VEL_KP * vel_error_p ) + ( VEL_KI * vel_error_i ) + ( VEL_KD * vel_error_d ) + ( ( OMEGA_KP / 100.0 ) * omega_error_p ) + ( ( OMEGA_KI / 100.0 ) * omega_error_i ) + ( ( OMEGA_KD / 100.0 ) * omega_error_d );
	l_control = ( VEL_KP * vel_error_p ) + ( VEL_KI * vel_error_i ) + ( VEL_KD * vel_error_d ) - ( ( OMEGA_KP / 100.0 ) * omega_error_p ) - ( ( OMEGA_KI / 100.0 ) * omega_error_i ) - ( ( OMEGA_KD / 100.0 ) * omega_error_d );
	}
	if(get_time(TYPE_MYMS) % 2 == 0) {
		//log_save((short)(current_omega), (short)(tar_omega),(short)(current_angle),(short)(angle));
		//log_save((short)(r_control*1000), (short)(l_control*1000),(short)(tar_vel*1000),(short)(angle));
		//log_save((short)(current_dis_ave*1000), (short)(current_vel_ave*1000), (short)(tar_vel*1000), (short)(tar_dis*1000));
		//log_save(1,1);
		
	}
	//横壁制御
	if( wall_control_flg == 1 )
	{
		r_control += error * WALLR_KP;
		l_control -= error * WALLR_KP;
	}
	//前壁制御
	if (f_wall_control_flg == 1)
	{
		//前後距離
		r_control += f_dis_err * F_WALL_KP;
		l_control += f_dis_err * F_WALL_KP;
		//角度
		r_control += error * F_WALL_KP;
		l_control -= error * F_WALL_KP;
	}

	//出力電圧計算
	V_r += r_control;
	V_l += l_control;
	
	//右モータの出力電圧が正の場合, 負の場合
	if(V_r > 0.0){
		direction_r_mot(MOT_FORWARD);	//正転
	}else{
		direction_r_mot(MOT_BACKWARD);	//逆転
		V_r = -V_r;			//電圧を正に直す
	}
	
	//左モータの出力電圧が正の場合, 負の場合
	if(V_l > 0.0){
		direction_l_mot(MOT_FORWARD);	//正転
	}else{
		direction_l_mot(MOT_BACKWARD);	//逆転
		V_l = -V_l;			//電圧を正に直す
	}
	
	//電源電圧取得
	V_bat = get_battery_voltage();
	
	//duty[%]算出
	duty_r = (V_r / V_bat) * 100;
	duty_l = (V_l / V_bat) * 100;
	
	//フェイルセーフ
	if(duty_r >= 80) duty_r = 80;
	if(duty_l >= 80) duty_l = 80;
	//log_save(V_r, V_l,duty_r,duty_l);
}

//壁制御( 1ms割り込み )
//*速度制御より前に記述
void control_wall(void){
	if( wall_control_flg == 0 ) return;
	//横壁センサ閾値
	short l_threshold = LEFT_THRESHOLD;
	short r_threshold = RIGHT_THRESHOLD;
	
	//機体中心時の横壁センサ基準値
	short ref_lf = REF_LF;
	short ref_rf = REF_RF;
	
	//センサ値格納
	short sensor_lf = get_sen_value(LF_SEN);
	short sensor_rf = get_sen_value(RF_SEN);
	
	if( (sensor_rf > r_threshold) && (sensor_lf > l_threshold) ){
		//両壁がある場合
		error = (sensor_rf - ref_rf) - (sensor_lf - ref_lf);
	}else if( (sensor_rf <= r_threshold) && (sensor_lf <= l_threshold) ){
		//両壁とも無い場合
		error = 0;
	}else if( sensor_rf > r_threshold ){
		//右壁のみある場合
		error = 2 * (sensor_rf - ref_rf);
	}else{
		//左壁のみある場合
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

//前壁制御(1ms割り込み)
void f_wall_control(void) {
	if(f_wall_control_flg == 0) return;
	short ref_ls = FRONT_WALL_STOPL;
	short ref_rs = FRONT_WALL_STOPR;
	short sensor_ls = get_sen_value(LS_SEN);
	short sensor_rs = get_sen_value(RS_SEN);
	//前後
		f_dis_err = (ref_rs - sensor_rs) + (ref_ls - sensor_ls);
	//角度
	//右に偏ってるとき
	if(ref_rs > sensor_rs && ref_ls < sensor_ls) {
		error = -(sensor_rs - ref_rs) + (sensor_ls - ref_ls);
	}
	//左に偏っている時
	else if (ref_rs < sensor_rs && ref_ls > sensor_ls) {
		error = -(sensor_rs - ref_rs) + (sensor_ls - ref_ls);
	}
	else {
		error = 0;
	}
	log_save(f_dis_err, error, 0, 0);
}

//モーター速度変化( 10us割り込み )
void change_motor_speed(void){
	//モーター速度変化フラグが0ならreturn
	if( speed_control_flg == 0 ) return;
	//左右モータにDuty入力, 回りだす
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
