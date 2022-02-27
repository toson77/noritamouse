#include "define.h"
#include "parameter.h"
#include "encoder.h"
#include "sensor.h"

//移動平均のバッファサイズ(設定値[ms]で移動平均が取られる)
#define	N	1

//エンコーダ―カウント数 - 10000 [count/ms]
static short r_enc_value;
static short l_enc_value;

//左右の速度 [mm/s]
static short r_enc_velocity;
static short l_enc_velocity;

//エンコーダ値の1msの変化量算出
void calc_enc_value(){
	r_enc_value = 10000 - ENC_PULSE_COUNT_R;
	l_enc_value = ENC_PULSE_COUNT_L - 10000;
	//エンコーダの値を初期値10000に戻す
	ENC_PULSE_COUNT_L = 10000;
	ENC_PULSE_COUNT_R = 10000;
}

//移動平均計算関数
float calc_r_enc_moving_ave(short value){
	static short i=0;	//リングバッファのポインタ
	static short buf[N] = {0};	//リングバッファ
	static float sum;	//合計値
	
	sum -= buf[i];	//最古の値を合計値から引く
	buf[i] = value;	//最古の値を最新の値に更新
	sum += buf[i];	//最新の値を合計値に加える
	
	if(++i == N){i=0;}	//次のためにリングバッファのポインタを進める
	
	return (sum/N);	// ( 直近N個の合計/個数 )
}

float calc_l_enc_moving_ave(short value){
	static short i=0;	//リングバッファのポインタ
	static short buf[N] = {0};	//リングバッファ
	static float sum;	//合計値
	
	sum -= buf[i];	//最古の値を合計値から引く
	buf[i] = value;	//最古の値を最新の値に更新
	sum += buf[i];	//最新の値を合計値に加える
	
	if(++i == N){i=0;}	//次のためにリングバッファのポインタを進める
	
	return (sum/N);	// ( 直近N個の合計/個数 )
}

short get_enc_value(char encoder){	//calcした現在のenc_valueを返す
	if(encoder == RIGHT_ENCODER){
		return r_enc_value;
	}
	else if(encoder == LEFT_ENCODER){
		return l_enc_value;
	}
}

short get_current_enc_velocity(char encoder){	//引数で指定した方のenc_valueを[mm/s]に変換し, 返す

	if(encoder == RIGHT_ENCODER){
		r_enc_velocity = ((float)(r_enc_value) * DISTANCE_PER_ENC_COUNT) * 1000;	//[mm/s] = [count/ms] * [mm/count]
		return r_enc_velocity;
	}
	else if(encoder == LEFT_ENCODER){
		l_enc_velocity = ((float)(l_enc_value) * DISTANCE_PER_ENC_COUNT) * 1000;	//[mm/s] = [count/ms] * [mm/count]
		return l_enc_velocity;
	}
	
}

unsigned short duty_to_count(short duty){	//duty比 -> TGRA, TGRCカウントへの変換
	unsigned short temp_cnt = 0;
	temp_cnt = ((100 - duty) * 320 / 100) - 1;	//320はPWM周期, 100-dutyは逆になってるから
	if(temp_cnt <= 0) temp_cnt = 0;
	if(temp_cnt == 319) temp_cnt = 320;
	return temp_cnt;
}