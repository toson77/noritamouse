#ifndef __PARAMETER_HEADER__
#define __PARAMETER_HEADER__

//エンコーダ1カウント当たりに進む距離[mm/count] これを1msごとのエンコーダのカウント数の変化[count/ms]に掛けると[m/s]が出る
#define DISTANCE_PER_ENC_COUNT TIRE_DIAMETER *PI *TIRE_ROTATION_PER_MOTOR_ROTATION *ENC_COUNT_PER_TIRE_ROTATION

//上式の構成要素
#define TIRE_DIAMETER 26.8                      //タイヤ直径[mm]	//25.7 //距離が長い場合は大きく小さい場合は小さく　*これ弄ると角度も変化するので注意、
#define PI 3.141593                             //円周率
#define TIRE_ROTATION_PER_MOTOR_ROTATION 4 / 15 //モータ1回転あたりのタイヤ回転量
#define ENC_COUNT_PER_TIRE_ROTATION 1 / 1024

//機体固有定数
#define MOUSE_WEIGHT 100     //機体重量[g]  
#define TREAD 0.0719          //トレッド幅[m] 回転調整、距離が合っているなら弄る、距離があっていないならタイヤ直径を先に弄る 回転がデカい場合は小さく、小さい場合は大きく
#define REDUCTION_RATIO 3.75 //減速比
//SCR13-2005モーターの定数
#define TORQUE_CONSTANT 4.49     //トルク定数Kt[mN*m/A]
#define BACK_EMF_CONSTANT 0.4702 //逆起電力定数KE[mV/rpm]
#define TERMINAL_RESISTANCE 5.41 //端子間抵抗[Ω]

////よく使う長さ(m)
#define HALF_SECTION 0.09 //半区画[m]
#define SECTION 0.18      //一区画[m]

//随時調整するパラメータ--------------------------------------
#define MAX_VEL 3.0       //最高速度[m/s]
#define MIN_VEL 0.2      //最低速度[m/s]
#define MIN_OMEGA 200.0   //最低角速度[deg/s]
#define SEARCH_SPEED 0.4  //探索速度[m/s]
#define SEARCH_ACCEL 0.5  //探索加速度[m/s^2]
#define FAST_SPEED 0.3    //最短速度[m/s]
#define FAST_ACCEL 1.0    //最短加速度[m/s^2]
#define TURN_OMEGA 310.0  //超新地旋回角速度[deg/s]
#define TURN_ALPHA 3000.0 //超新地旋回角速度[deg/s^2]

#define TURN_SPEED 0.7        //回転速度[m/s]
#define TURN_ACCEL 1.0        //回転加速度[m/s^2]
#define SLALOM_TURN_ACCEL 1.0 //スラローム加速度[m/s^2]

#define REF_LF 278         //左横壁センサ基準値
#define REF_RF 417          //右横壁センサ基準値
#define LEFT_THRESHOLD 114  //左壁センサ閾値
#define RIGHT_THRESHOLD 168 //右壁センサ閾値
#define FRONT_THRESHOLD 560 //前壁センサ閾値
#define FRONT_WALL_STOPL 2500
#define FRONT_WALL_STOPR 1810

#define TR_SENSOR_FRONT_WALL_R	410 //前壁近づいてきたとき壁センサ切る値
#define TR_SENSOR_FRONT_WALL_L	810 //前壁近づいてきたとき壁センサ切る値


//PID処理周期(1ms)
#define DELTA_T 0.001
//フィードフォワード制御用定数
//FF_KV = ( Ke * 60 * n ) / ( 2 * PI * r )
//FF_KA = ( r * m * R ) / ( 2 * Kt *n )
//FF_KOMEGA = ( Ke * 60 * n * ( TREAD / 2 ) ) / ( 2 * PI * r )
//FF_KALPHA = ( r * m * R * ( TREAD / 2 ) ) / ( 2 * Kt * n )
//FF_FRIC = ( R / Kt ) * f	(fは摩擦)
#define FF_KV 1.310334      //( 0.0004702 * 60.0 * 3.75 ) / ( 2.0 * PI * 0.01285 )
#define FF_KA 0.40          //0.2064395	//( 0.01285 * 0.100 * 5.41 ) / ( 2.0 * 0.00449 * 3.75 )
#define FF_KOMEGA 0.04979262 //( 0.0004702 * 60.0 * 3.75 * 0.038 ) / ( 2.0 * PI * 0.01285 )
#define FF_KALPHA 0.013148    //( 0.01285 * 0.100 * 5.41 * 0.038 ) / ( 2.0 * 0.00449 * 3.75 )
#define FF_FRIC_R 0.38
#define FF_FRIC_L 0.43
//PIDゲイン
//速度
#define VEL_KP 6.0
#define VEL_KI 0.5
#define VEL_KD 0
//角速度
#define OMEGA_KP 0.7
#define OMEGA_KI 0.005
//test
#define OMEGA_KPV 1.1
#define OMEGA_KIV 0.0
#define OMEGA_KD 0
//壁制御
#define WALLR_KP 0.002
#define WALLL_KP 0.002

#endif
