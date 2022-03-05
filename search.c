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
//左手法による探索を行う関数
void left_hand(void){
	
	//(0,0)区画内初期化
	init_wall();
	init_stepMap();
	m_dir = 0;	//最初は北向き
	
	straight(HALF_SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
	
	while(1){
		
		update_coordinate();	//座標更新
		set_wall(x_coordinate, y_coordinate, m_dir);	//壁判定, 格納
		if( goal_judge() == 1 )	break;		//ゴール到着判定
		
		if( get_sen_value(LF_SEN) < LEFT_THRESHOLD ){	//左壁無し
			
			straight(0.015, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			slalom(90.0, TURN_OMEGA, 0, TURN_ALPHA);
			straight(0.08, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			if(--m_dir < 0)	m_dir = 3;
			
		}else if( ( get_sen_value(LS_SEN) + get_sen_value(RS_SEN) ) / 2 < FRONT_THRESHOLD ){	//前壁無し
			LED_2 = ~LED_2;
			straight(SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			
		}else if( get_sen_value(RF_SEN) < RIGHT_THRESHOLD ){	//右壁無し
		
			straight(0.015, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			slalom(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
			straight(0.08, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			if(++m_dir > 3)	m_dir = 0;
			
		}else{	//Uターン
		
			straight(0, SEARCH_SPEED, 0, SEARCH_ACCEL, 1);
			turn(180.0, 700, 0, 7000);
			if(--m_dir < 0)	m_dir = 3;
			if(--m_dir < 0)	m_dir = 3;
			straight(HALF_SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1);
			
		}
	}
	//終了行動
	straight(0.06, SEARCH_SPEED, 0, SEARCH_ACCEL, 1);
	turn(180.0, 700, 0, 7000);
}
*/

//足立法による探索を行う関数
void adachi_method(void){
	
	char nextdir;
	//(0,0)区画内初期化
	init_wall();		//壁情報初期化
	init_stepMap();		//歩数マップ初期化
	update_stepMap();	//歩数マップ作成
	//最初は北向き
	m_dir = 0;
	//初期行動
	turn(180.0, TURN_OMEGA, 0, TURN_ALPHA);
	doing_f_wall_revision();
	turn(90.0, TURN_OMEGA, 0, TURN_ALPHA);
	doing_f_wall_revision();
	turn(90.0, TURN_OMEGA, 0, TURN_ALPHA);
	straight(HALF_SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1,0);
	
	while(1){
		update_coordinate();	//座標更新
		set_wall(x_coordinate, y_coordinate, m_dir);	//壁判定, 格納
		
		//現在座標の全ての壁を既知とする
		add_knownWall(x_coordinate, y_coordinate, NORTH);
		add_knownWall(x_coordinate, y_coordinate, EAST);
		add_knownWall(x_coordinate, y_coordinate, SOUTH);
		add_knownWall(x_coordinate, y_coordinate, WEST);
		
		init_stepMap();		//歩数マップ初期化
		update_stepMap();	//歩数マップ作成
		if( goal_judge(OUTWAED) == 1 )	break;		//ゴール到着判定
		//進行方向判断
		nextdir = adachi_judge_nextdir();
		//直進
		LED(nextdir);
		if( nextdir == 0 ){
			//log_save(0,0,0,0);
			if (before_flg == 1){
				straight(SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 0,0);
			}
			else{
			straight(SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1,0);
			}
			
			
		}
		//右折
		if( nextdir == 1 ){
			//log_save(1,1,1,1);
			//before_flg = 1;
			//log_save(1,1,1,1);
			// 両方壁ないとき壁制御入れるとバグるかも
			straight(HALF_SECTION, SEARCH_SPEED, 0, SEARCH_ACCEL, 0, 1);
			//straight(HALF_SECTION, SEARCH_SPEED, 0, SEARCH_ACCEL, 0, 0, nextdir);
			wait_ms(100); //最初左車輪回らないなぜ
			turn(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
			//wait_ms(100);
			straight(HALF_SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1,0);
			if(++m_dir > 3)	m_dir = 0;
		}
		//Uターン
		if( nextdir == 2 ){
			//log_save(2,2,2,2);
			
			straight(0.04, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 0,0);
			unsigned int left_val = abs((int)get_sen_value(LF_SEN) - REF_LF);
			unsigned int right_val = abs((int)get_sen_value(RF_SEN) - REF_RF);
			straight(0.04, SEARCH_SPEED, 0, SEARCH_ACCEL, 0,1);
			//straight(0.04, SEARCH_SPEED, 0, SEARCH_ACCEL, 0,0,nextdir);
			wait_ms(100);
			
			
			//左回転
			if( (left_val <  right_val) && exist_l_wall == 1 && exist_f_wall == 1) {
				
				turn(90.0, TURN_OMEGA, 0, TURN_ALPHA);
				doing_f_wall_revision();
				turn(90.0, TURN_OMEGA, 0, TURN_ALPHA);
				wait_ms(100);
			}
			//右回転
			else if(exist_r_wall == 1 && exist_f_wall == 1){
				turn(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
				doing_f_wall_revision();
				turn(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
				wait_ms(100);
				
			}
			else {
				turn(180.0, TURN_OMEGA, 0, TURN_ALPHA);
			}
				
				
			straight(HALF_SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1,0);
			
			//Uターンだから m_dir -= 2 
			if(--m_dir < 0)	m_dir = 3;
			if(--m_dir < 0)	m_dir = 3;
			
				
			
		}
		//左折
		if( nextdir == 3 ){
			//log_save(3,3,3,3);
			straight(HALF_SECTION, SEARCH_SPEED, 0, SEARCH_ACCEL, 0, 1);
			//wait_ms(100);
			turn(90.0, TURN_OMEGA, 0, TURN_ALPHA);
			//wait_ms(100);
			straight(HALF_SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1,0);
			if(--m_dir < 0)	m_dir = 3;
		}
	}

	//終了行動
	straight(HALF_SECTION, SEARCH_SPEED, 0, SEARCH_ACCEL, 1,0);
	ledseg_0_interrupt(5);

	//復路
	init_stepMap_back();	  //復路歩数マップ初期化
	update_stepMap_back(); //復路歩数マップ作成

	//初期動作、前壁あればUターン無ければ直進
	if(exist_f_wall == 1) {
		if(exist_r_wall == 1){
			turn(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
			doing_f_wall_revision();
			turn(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
		}else if(exist_l_wall == 1) {
			turn(90.0, TURN_OMEGA, 0, TURN_ALPHA);
			doing_f_wall_revision();
			turn(90.0, TURN_OMEGA, 0, TURN_ALPHA);
		}
		straight(HALF_SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1, 0);
		// Uターンだから m_dir -= 2
		if (--m_dir < 0) m_dir = 3;
		if (--m_dir < 0) m_dir = 3;
	} else {
		straight(HALF_SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1, 0);
	}

	while (1)
	{
		update_coordinate();						 //座標更新
		set_wall(x_coordinate, y_coordinate, m_dir); //壁判定, 格納

		//現在座標の全ての壁を既知とする
		add_knownWall(x_coordinate, y_coordinate, NORTH);
		add_knownWall(x_coordinate, y_coordinate, EAST);
		add_knownWall(x_coordinate, y_coordinate, SOUTH);
		add_knownWall(x_coordinate, y_coordinate, WEST);

		init_stepMap_back();	  //復路歩数マップ初期化
		update_stepMap_back();  //復路歩数マップ作成
		if (goal_judge(RETURN) == 1)
			break; //ゴール到着判定
		//進行方向判断
		nextdir = adachi_judge_nextdir_back();
		//直進
		LED(nextdir);
		if (nextdir == 0)
		{
			// log_save(0,0,0,0);
			if (before_flg == 1)
			{
				straight(SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 0, 0);
			}
			else
			{
				straight(SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1, 0);
			}
		}
		//右折
		if (nextdir == 1)
		{
			// log_save(1,1,1,1);
			// before_flg = 1;
			// log_save(1,1,1,1);
			//  両方壁ないとき壁制御入れるとバグるかも
			straight(HALF_SECTION, SEARCH_SPEED, 0, SEARCH_ACCEL, 0, 1);
			// straight(HALF_SECTION, SEARCH_SPEED, 0, SEARCH_ACCEL, 0, 0, nextdir);
			wait_ms(100); //最初左車輪回らないなぜ
			turn(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
			// wait_ms(100);
			straight(HALF_SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1, 0);
			if (++m_dir > 3)
				m_dir = 0;
		}
		// Uターン
		if (nextdir == 2)
		{
			// log_save(2,2,2,2);

			straight(0.04, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 0, 0);
			unsigned int left_val = abs((int)get_sen_value(LF_SEN) - REF_LF);
			unsigned int right_val = abs((int)get_sen_value(RF_SEN) - REF_RF);
			straight(0.04, SEARCH_SPEED, 0, SEARCH_ACCEL, 0, 1);
			// straight(0.04, SEARCH_SPEED, 0, SEARCH_ACCEL, 0,0,nextdir);
			wait_ms(100);

			//左回転
			if ((left_val < right_val) && exist_l_wall == 1 && exist_f_wall == 1)
			{

				turn(90.0, TURN_OMEGA, 0, TURN_ALPHA);
				doing_f_wall_revision();
				turn(90.0, TURN_OMEGA, 0, TURN_ALPHA);
				wait_ms(100);
			}
			//右回転
			else if (exist_r_wall == 1 && exist_f_wall == 1)
			{
				turn(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
				doing_f_wall_revision();
				turn(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
				wait_ms(100);
			}
			else
			{
				turn(180.0, TURN_OMEGA, 0, TURN_ALPHA);
			}

			straight(HALF_SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1, 0);

			// Uターンだから m_dir -= 2
			if (--m_dir < 0)
				m_dir = 3;
			if (--m_dir < 0)
				m_dir = 3;
		}
		//左折
		if (nextdir == 3)
		{
			// log_save(3,3,3,3);
			straight(HALF_SECTION, SEARCH_SPEED, 0, SEARCH_ACCEL, 0, 1);
			// wait_ms(100);
			turn(90.0, TURN_OMEGA, 0, TURN_ALPHA);
			// wait_ms(100);
			straight(HALF_SECTION, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL, 1, 0);
			if (--m_dir < 0)
				m_dir = 3;
		}
	}
	//終了行動
	straight(HALF_SECTION, SEARCH_SPEED, 0, SEARCH_ACCEL, 1, 1);
	turn(180.0, TURN_OMEGA, 0, TURN_ALPHA);
	init_wall_exist_flg();
	ledseg_0_interrupt(5);

	generate_adachi_shortestRoute(); //最短経路パス生成
}

//生成したパスに沿って最短走行を行う関数
void run_shortestRoute(void){
	turn(90.0, TURN_OMEGA, 0, TURN_ALPHA);
	doing_f_wall_revision();
	turn(90.0, TURN_OMEGA, 0, TURN_ALPHA);
	doing_f_wall_revision();
	turn(180.0, TURN_OMEGA, 0, TURN_ALPHA);
	short i = 0;
	//straight(0.048, FAST_SPEED,  FAST_SPEED, FAST_ACCEL, 1,0,-1);
	while(1){
		if( path[i] <= 15 ){		//直進
			straight(SECTION*path[i], FAST_SPEED, 0, FAST_ACCEL, 1,0,-1);
		}else if( path[i] == 20 ){	//右折
			turn(-90.0, TURN_OMEGA, 0, TURN_ALPHA);
		}else if( path[i] == 30 ){	//左折
			turn(90.0, TURN_OMEGA, 0, TURN_ALPHA);
		}
		if( i == path_size )	break;	//パス全部実行し終わったらループ脱出
		i++;
	}
	turn(180.0, TURN_OMEGA, 0, TURN_ALPHA);
}
