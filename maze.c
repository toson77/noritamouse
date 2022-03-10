#include "parameter.h"
#include "sci.h"
#include "maze.h"
#include "run.h"
#include "sensor.h"
#include "interrupt.h"
#include "log.h"

#define MAZE_SIZE 16
// start 0 0
#define GOAL_X_COODINATE 3
#define GOAL_Y_COODINATE 3

//グローバル変数--------------
char x_coordinate = 0;
char y_coordinate = 0;
char m_step = 1;
signed char m_dir = 0;
unsigned char path[256] = {0};
unsigned char path_size = 0;
char exist_l_wall=0, exist_r_wall=0, exist_f_wall=0;
//-------------------------

//水平方向と垂直方向の壁
unsigned short wallH[MAZE_SIZE + 1] = {0};
unsigned short wallV[MAZE_SIZE + 1] = {0};
//水平方向と垂直方向の既知の壁
unsigned short knownWallH[MAZE_SIZE + 1] = {0};
unsigned short knownWallV[MAZE_SIZE + 1] = {0};
//歩数マップ
unsigned short stepMap[MAZE_SIZE][MAZE_SIZE];
//復路用マップ
unsigned short stepMap_back[MAZE_SIZE][MAZE_SIZE];

//壁情報の初期化関数
void init_wall(void)
{
	//外壁
	wallV[0] = 0xffff;
	wallV[MAZE_SIZE] = 0xffff;
	wallH[0] = 0xffff;
	wallH[MAZE_SIZE] = 0xffff;
	//スタート区画
	wallV[1] = 0x0001;
	//スタート区画を既知に
	add_knownWall(0, 0, NORTH);
	add_knownWall(0, 0, EAST);
	add_knownWall(0, 0, SOUTH);
	add_knownWall(0, 0, WEST);
}

//歩数マップ初期化関数
void init_stepMap(void)
{
	short i, j;
	//全区画, 初期値は300
	for (i = 0; i < MAZE_SIZE; i++)
	{
		for (j = 0; j < MAZE_SIZE; j++)
		{
			stepMap[i][j] = 300;
		}
	}
	//ゴール座標は0に設定
	stepMap[GOAL_X_COODINATE][GOAL_Y_COODINATE] = 0;
}


//歩数マップ展開関数
void update_stepMap(void)
{
	unsigned short temp_stepMap_val;
	char update_flg = 1;
	short i, j;
	//一番初めの展開元はゴール座標
	temp_stepMap_val = stepMap[GOAL_X_COODINATE][GOAL_Y_COODINATE];
	//展開元をインクリメントしていき, 展開不可能(update_flgが立たない)になるまでループ
	while (update_flg == 1)
	{
		update_flg = 0;
		for (i = 0; i < MAZE_SIZE; i++)
		{
			for (j = 0; j < MAZE_SIZE; j++)
			{
				if (stepMap[i][j] == temp_stepMap_val)
				{
					//展開処理
					//展開元の周囲で, 壁がなく, かつMAX値(未展開)の座標に展開できる
					if (judge_wall(i, j, NORTH) == 0 && j < MAZE_SIZE - 1)
					{
						if (stepMap[i][j + 1] == 300)
						{
							stepMap[i][j + 1] = temp_stepMap_val + 1;
							update_flg = 1;
						}
					}
					if (judge_wall(i, j, EAST) == 0 && i < MAZE_SIZE - 1)
					{
						if (stepMap[i + 1][j] == 300)
						{
							stepMap[i + 1][j] = temp_stepMap_val + 1;
							update_flg = 1;
						}
					}
					if (judge_wall(i, j, SOUTH) == 0 && j > 0)
					{
						if (stepMap[i][j - 1] == 300)
						{
							stepMap[i][j - 1] = temp_stepMap_val + 1;
							update_flg = 1;
						}
					}
					if (judge_wall(i, j, WEST) == 0 && i > 0)
					{
						if (stepMap[i - 1][j] == 300)
						{
							stepMap[i - 1][j] = temp_stepMap_val + 1;
							update_flg = 1;
						}
					}
				}
			}
		}
		temp_stepMap_val++; //展開元更新
	}
}

//歩数マップを基に, 現在座標から見て最小歩数の方向を返す関数
char adachi_judge_nextdir(void)
{
	unsigned short minimum;
	char nextdir = 0;
	char success_flg = 0;

	minimum = stepMap[x_coordinate][y_coordinate]; //デフォルトの最小は現在座標
	//歩数比較(壁なしを確認してから)	優先度: 北>東>南>西
	//北
	if (judge_wall(x_coordinate, y_coordinate, NORTH) == 0)
	{
		if (minimum > stepMap[x_coordinate][y_coordinate + 1])
		{
			minimum = stepMap[x_coordinate][y_coordinate + 1];
			nextdir = 0;
			success_flg = 1;
		}
	}
	//東
	if (judge_wall(x_coordinate, y_coordinate, EAST) == 0)
	{
		if (minimum > stepMap[x_coordinate + 1][y_coordinate])
		{
			minimum = stepMap[x_coordinate + 1][y_coordinate];
			nextdir = 1;
			success_flg = 1;
		}
	}
	//南
	if (judge_wall(x_coordinate, y_coordinate, SOUTH) == 0)
	{
		if (minimum > stepMap[x_coordinate][y_coordinate - 1])
		{
			minimum = stepMap[x_coordinate][y_coordinate - 1];
			nextdir = 2;
			success_flg = 1;
		}
	}
	//西
	if (judge_wall(x_coordinate, y_coordinate, WEST) == 0)
	{
		if (minimum > stepMap[x_coordinate - 1][y_coordinate])
		{
			minimum = stepMap[x_coordinate - 1][y_coordinate];
			nextdir = 3;
			success_flg = 1;
		}
	}
	//等高線谷の場合
	if (success_flg == 0)
	{
		if (judge_wall(x_coordinate, y_coordinate, NORTH) == 0)
		{
			nextdir = 0;
		}
		else if (judge_wall(x_coordinate, y_coordinate, EAST) == 0)
		{
			nextdir = 1;
		}
		else if (judge_wall(x_coordinate, y_coordinate, SOUTH) == 0)
		{
			nextdir = 2;
		}
		else
		{
			nextdir = 3;
		}
	}
	//判定結果
	nextdir += 4;
	nextdir -= m_dir;
	if (nextdir >= 4)
	{
		nextdir -= 4;
	}
	return nextdir;
}

//復路歩数マップ初期化関数
void init_stepMap_back(void) {
	short i, j;
	//全区画, 初期値は300
	for (i = 0; i < MAZE_SIZE; i++)
	{
		for (j = 0; j < MAZE_SIZE; j++)
		{
			stepMap_back[i][j] = 300;
		}
	}
	stepMap_back[0][0] = 0;
}

//復路歩数マップ展開関数
void update_stepMap_back(void)
{
	unsigned short temp_stepMap_val;
	char update_flg = 1;
	short i, j;
	//一番初めの展開元はゴール座標
	temp_stepMap_val = stepMap_back[0][0];
	//展開元をインクリメントしていき, 展開不可能(update_flgが立たない)になるまでループ
	while (update_flg == 1)
	{
		update_flg = 0;
		for (i = 0; i < MAZE_SIZE; i++)
		{
			for (j = 0; j < MAZE_SIZE; j++)
			{
				if (stepMap_back[i][j] == temp_stepMap_val)
				{
					//展開処理
					//展開元の周囲で, 壁がなく, かつMAX値(未展開)の座標に展開できる
					if (judge_wall(i, j, NORTH) == 0)
					{
						if (stepMap_back[i][j + 1] == 300 && j < MAZE_SIZE - 1)
						{
							stepMap_back[i][j + 1] = temp_stepMap_val + 1;
							update_flg = 1;
						}
					}
					if (judge_wall(i, j, EAST) == 0 && i < MAZE_SIZE - 1)
					{
						if (stepMap_back[i + 1][j] == 300)
						{
							stepMap_back[i + 1][j] = temp_stepMap_val + 1;
							update_flg = 1;
						}
					}
					if (judge_wall(i, j, SOUTH) == 0 && j > 0)
					{
						if (stepMap_back[i][j - 1] == 300)
						{
							stepMap_back[i][j - 1] = temp_stepMap_val + 1;
							update_flg = 1;
						}
					}
					if (judge_wall(i, j, WEST) == 0 && i > 0)
					{
						if (stepMap_back[i - 1][j] == 300)
						{
							stepMap_back[i - 1][j] = temp_stepMap_val + 1;
							update_flg = 1;
						}
					}
				}
			}
		}
		temp_stepMap_val++; //展開元更新
	}
}

//復路歩数マップを基に, 現在座標から見て最小歩数の方向を返す関数
char adachi_judge_nextdir_back(void)
{
	unsigned short minimum;
	char nextdir = 0;
	char success_flg = 0;
	minimum = stepMap_back[x_coordinate][y_coordinate]; //デフォルトの最小は現在座標
	//歩数比較(壁なしを確認してから)	優先度: 北>東>南>西
	//北
	if (judge_wall(x_coordinate, y_coordinate, NORTH) == 0)
	{
		if (minimum > stepMap_back[x_coordinate][y_coordinate + 1])
		{
			minimum = stepMap_back[x_coordinate][y_coordinate + 1];
			nextdir = 0;
			success_flg = 1;
		}
	}
	//東
	if (judge_wall(x_coordinate, y_coordinate, EAST) == 0)
	{
		if (minimum > stepMap_back[x_coordinate + 1][y_coordinate])
		{
			minimum = stepMap_back[x_coordinate + 1][y_coordinate];
			nextdir = 1;
			success_flg = 1;
		}
	}
	//南
	if (judge_wall(x_coordinate, y_coordinate, SOUTH) == 0)
	{
		if (minimum > stepMap_back[x_coordinate][y_coordinate - 1])
		{
			minimum = stepMap_back[x_coordinate][y_coordinate - 1];
			nextdir = 2;
			success_flg = 1;
		}
	}
	//西
	if (judge_wall(x_coordinate, y_coordinate, WEST) == 0)
	{
		if (minimum > stepMap_back[x_coordinate - 1][y_coordinate])
		{
			minimum = stepMap_back[x_coordinate - 1][y_coordinate];
			nextdir = 3;
			success_flg = 1;
		}
	}
	//等高線谷の場合
	if (success_flg == 0)
	{
		if (judge_wall(x_coordinate, y_coordinate, NORTH) == 0)
		{
			nextdir = 0;
		}
		else if (judge_wall(x_coordinate, y_coordinate, EAST) == 0)
		{
			nextdir = 1;
		}
		else if (judge_wall(x_coordinate, y_coordinate, SOUTH) == 0)
		{
			nextdir = 2;
		}
		else
		{
			nextdir = 3;
		}
	}

	//判定結果
	nextdir += 4;
	nextdir -= m_dir;
	if (nextdir >= 4)
	{
		nextdir -= 4;
	}
	return nextdir;
}

//歩数マップを基に最短経路のパスを生成する関数
void generate_adachi_shortestRoute(void)
{
	static signed char previousdir, nextdir;
	short i, j;

	//未知壁はあるものとして壁情報更新
	for (i = 0; i < MAZE_SIZE; i++)
	{
		for (j = 0; j < MAZE_SIZE; j++)
		{
			if (judge_knownWall(i, j, NORTH) == 0)
				add_wall(i, j, NORTH);
			if (judge_knownWall(i, j, EAST) == 0)
				add_wall(i, j, EAST);
			if (judge_knownWall(i, j, SOUTH) == 0)
				add_wall(i, j, SOUTH);
			if (judge_knownWall(i, j, WEST) == 0)
				add_wall(i, j, WEST);
		}
	}

	init_stepMap();	  //歩数マップ初期化
	update_stepMap(); //歩数マップ作成

	//座標と向き初期化
	x_coordinate = 0;
	y_coordinate = 0;
	m_dir = 0;
	previousdir = 0;
	nextdir = 0;
	//ゴールまでパス生成ループ
	i = 0;
	while (!(x_coordinate == GOAL_X_COODINATE && y_coordinate == GOAL_Y_COODINATE))
	{
		previousdir = nextdir;			  //ひとつ前の向き保存
		nextdir = adachi_judge_nextdir(); //現在座標から見て最小の歩数となる向きを知る
		if (nextdir == 0)
		{
			if (previousdir != nextdir)
				i++;			 //ひとつ前と今回の向きが同じでなければパス要素を進める
			path[i]++;			 //直進パス
			update_coordinate(); //座標更新
		}
		else if (nextdir == 1)
		{
			i++;		  //直進でないのでパス進める
			path[i] = 20; //右折パス
			if (++m_dir > 3)
				m_dir = 0; //現在向き右へ
		}
		else if (nextdir == 3)
		{
			i++;		  //直進でないのでパス進める
			path[i] = 30; //左折パス
			if (--m_dir < 0)
				m_dir = 3; //現在向き左へ
		}
	}
	path_size = i;
}

//座標と方位を与えたらその場の壁の配列に壁を入れる関数
void add_wall(char _x_coordinate, char _y_coordinate, char _dir)
{
	unsigned short one = 0x8000; //MSBが1
	//oneを必要なだけ右シフトさせたものと対応する配列のORを取ることで壁を追加
	//外壁とスタート区画は更新せずreturn
	if (_dir == NORTH)
	{
		if (_y_coordinate == MAZE_SIZE - 1)
			return;
		wallH[_y_coordinate + 1] |= (one >> _x_coordinate);
	}
	else if (_dir == WEST)
	{
		if (_x_coordinate == 0)
			return;
		else if (_x_coordinate == 1 && _y_coordinate == 0)
			return; //これはスタート区画の東壁
		wallV[_x_coordinate] |= (one >> (MAZE_SIZE - 1 - _y_coordinate));
	}
	else if (_dir == SOUTH)
	{
		if (_y_coordinate == 0)
			return;
		wallH[_y_coordinate] |= (one >> _x_coordinate);
	}
	else if (_dir == EAST)
	{
		if (_x_coordinate == MAZE_SIZE - 1)
			return;
		else if (_x_coordinate == 0 && _y_coordinate == 0)
			return;
		wallV[_x_coordinate + 1] |= (one >> (MAZE_SIZE - 1 - _y_coordinate));
	}
}

void init_wall_exist_flg(void) {
	exist_f_wall = 0;
	exist_f_wall = 0;
	exist_r_wall = 0;
}
//ある座標と機体向きにおいて左と前と右センサの値が閾値を超えていたらその方向の壁に壁を入れる関数
void set_wall(char _x_coordinate, char _y_coordinate, char _m_dir)
{
	//壁存在flg初期化
	exist_l_wall = 0;
	exist_f_wall = 0;
	exist_r_wall = 0;
	signed char temp_dir = _m_dir;
	log_save(get_sen_value(LF_SEN), get_sen_value(LS_SEN), get_sen_value(RS_SEN), get_sen_value(RF_SEN));
	if (get_sen_value(LF_SEN) > LEFT_THRESHOLD)
	{
		exist_l_wall = 1;
		if (--temp_dir < 0)
			temp_dir = 3;
		add_wall(_x_coordinate, _y_coordinate, temp_dir);
		temp_dir = _m_dir; //次のために直す
	}
	if ((get_sen_value(LS_SEN) + get_sen_value(RS_SEN)) / 2 > FRONT_THRESHOLD)
	{
		exist_f_wall = 1;
		add_wall(_x_coordinate, _y_coordinate, temp_dir);
	}
	if (get_sen_value(RF_SEN) > RIGHT_THRESHOLD)
	{
		exist_r_wall = 1;
		if (++temp_dir > 3)
			temp_dir = 0;
		add_wall(_x_coordinate, _y_coordinate, temp_dir);
		temp_dir = _m_dir; //次のために直す
	}
}

//座標と方位を与えたらその場に壁があるかどうか判定する関数
//あり : 1, なし : 0
char judge_wall(char _x_coordinate, char _y_coordinate, char _dir)
{
	unsigned short checker = 0x8000; //MSBが1

	if (_dir == NORTH)
	{
		if (_y_coordinate == MAZE_SIZE - 1)
			return 1;
		else if ((wallH[_y_coordinate + 1] & (checker >> _x_coordinate)) == 0x00)
		{
			return 0;
		}
		else
			return 1;
	}
	else if (_dir == WEST)
	{
		if (_x_coordinate == 0)
			return 1;
		else if (_x_coordinate == 1 && _y_coordinate == 0)
			return 1;
		if ((wallV[_x_coordinate] & (checker >> (MAZE_SIZE - _y_coordinate - 1))) == 0x00)
		{
			return 0;
		}
		else
			return 1;
	}
	else if (_dir == SOUTH)
	{
		if (_y_coordinate == 0)
			return 1;
		else if ((wallH[_y_coordinate] & (checker >> _x_coordinate)) == 0x00)
		{
			return 0;
		}
		else
			return 1;
	}
	else if (_dir == EAST)
	{
		if (_x_coordinate == MAZE_SIZE - 1)
			return 1;
		else if (_x_coordinate == 0 && _y_coordinate == 0)
			return 1;
		if ((wallV[_x_coordinate + 1] & (checker >> (MAZE_SIZE - _y_coordinate - 1))) == 0x00)
		{
			return 0;
		}
		else
			return 1;
	}
}

//座標と方位を与えたらその場の既知壁を保存する関数
void add_knownWall(char _x_coordinate, char _y_coordinate, char _dir)
{
	unsigned short one = 0x8000; //MSBが1
	//oneを必要なだけ右シフトさせたものと対応する配列のORを取ることで既知壁を追加
	if (_dir == NORTH)
	{
		knownWallH[_y_coordinate + 1] |= (one >> _x_coordinate);
	}
	else if (_dir == WEST)
	{
		
		knownWallV[_x_coordinate] |= (one >> (MAZE_SIZE - _y_coordinate - 1));
	}
	else if (_dir == SOUTH)
	{
		knownWallH[_y_coordinate] |= (one >> _x_coordinate);
	}
	else if (_dir == EAST)
	{
		
		knownWallV[_x_coordinate + 1] |= (one >> (MAZE_SIZE - _y_coordinate - 1));
	}
}

//座標と方位を与えたらその場の壁が既知かどうか判定する関数
//既知 : 1, 未知 : 0
char judge_knownWall(char _x_coordinate, char _y_coordinate, char _dir)
{
	unsigned short checker = 0x8000; //MSBが1
	if (_dir == NORTH)
	{
		if ((knownWallH[_y_coordinate + 1] & (checker >> _x_coordinate)) == 0x00)
		{
			return 0;
		}
		else
			return 1;
	}
	else if (_dir == WEST)
	{
		if ((knownWallV[_x_coordinate] & (checker >> (MAZE_SIZE - _y_coordinate - 1))) == 0x00)
		{
			return 0;
		}
		else
			return 1;
	}
	else if (_dir == SOUTH)
	{
		if ((knownWallH[_y_coordinate] & (checker >> _x_coordinate)) == 0x00)
		{
			return 0;
		}
		else
			return 1;
	}
	else if (_dir == EAST)
	{
		if ((knownWallV[_x_coordinate + 1] & (checker >> (MAZE_SIZE - _y_coordinate - 1))) == 0x00)
		{
			return 0;
		}
		else
			return 1;
	}
}

//壁情報吐き出し関数
void print_wall(void)
{
	short i, j, k;

	for (j = MAZE_SIZE - 1; j >= 0; j--)
	{
		//横壁1列
		for (i = 0; i < MAZE_SIZE; i++)
		{
			sci_printf("+");
			if (judge_wall(i, j, NORTH) == 1)
				sci_printf("---");
			else
				sci_printf("   ");
		}
		sci_printf("+");
		sci_printf("\r\n");
		//縦壁1列
		if (judge_wall(0, j, WEST) == 1)
			sci_printf("|");
		else
			sci_printf(" ");
		for (i = 0; i < MAZE_SIZE; i++)
		{
			//マスの中----------------------------------------------------------

			if (stepMap[i][j] < 10)
				sci_printf(" %d ", stepMap[i][j]);
			else if (stepMap[i][j] < 100)
				sci_printf("%d ", stepMap[i][j]);
			else if (stepMap[i][j] < 1000)
				sci_printf("%d", stepMap[i][j]);

			//sci_printf("   ");
			//----------------------------------------------------------------
			if (judge_wall(i, j, EAST) == 1)
				sci_printf("|");
			else
				sci_printf(" ");
		}
		sci_printf("\r\n");
	}
	//外壁閉じ
	for (i = 0; i < MAZE_SIZE; i++)
	{
		sci_printf("+");
		if (judge_wall(i, 0, SOUTH) == 1)
			sci_printf("---");
		else
			sci_printf("   ");
	}
	sci_printf("+");
}
//壁情報吐き出し関数
void print_wall_back(void)
{
	short i, j, k;

	for (j = MAZE_SIZE - 1; j >= 0; j--)
	{
		//横壁1列
		for (i = 0; i < MAZE_SIZE; i++)
		{
			sci_printf("+");
			if (judge_wall(i, j, NORTH) == 1)
				sci_printf("---");
			else
				sci_printf("   ");
		}
		sci_printf("+");
		sci_printf("\r\n");
		//縦壁1列
		if (judge_wall(0, j, WEST) == 1)
			sci_printf("|");
		else
			sci_printf(" ");
		for (i = 0; i < MAZE_SIZE; i++)
		{
			//マスの中----------------------------------------------------------

			if (stepMap_back[i][j] < 10)
				sci_printf(" %d ", stepMap_back[i][j]);
			else if (stepMap_back[i][j] < 100)
				sci_printf("%d ", stepMap_back[i][j]);
			else if (stepMap_back[i][j] < 1000)
				sci_printf("%d", stepMap_back[i][j]);

			// sci_printf("   ");
			//----------------------------------------------------------------
			if (judge_wall(i, j, EAST) == 1)
				sci_printf("|");
			else
				sci_printf(" ");
		}
		sci_printf("\r\n");
	}
	//外壁閉じ
	for (i = 0; i < MAZE_SIZE; i++)
	{
		sci_printf("+");
		if (judge_wall(i, 0, SOUTH) == 1)
			sci_printf("---");
		else
			sci_printf("   ");
	}
	sci_printf("+");
}

//最短経路パス吐き出し関数
void print_shortestRoute(void)
{
	short i = 0;
	while (path[i] != 0)
	{
		sci_printf("%d \r\n", path[i]);
		i++;
	}
}

//座標更新
void update_coordinate(void)
{
	if (m_dir == 0)
		y_coordinate++;
	else if (m_dir == 1)
		x_coordinate++;
	else if (m_dir == 2)
		y_coordinate--;
	else if (m_dir == 3)
		x_coordinate--;
}

//現在座標がゴールかどうか判定する関数
char goal_judge(char _trip)
{
	//往路
	if(_trip == OUTWAED) {
		//ゴール座標なら1を返す
		if (x_coordinate == GOAL_X_COODINATE && y_coordinate == GOAL_Y_COODINATE)
			return 1;
		else
			return 0;
	}
	//復路
	else {
		if (x_coordinate == 0 && y_coordinate == 0)
			return 1;
		else
			return 0;
	}
}
