#ifndef __MAZE_HEADER__
#define __MAZE_HEADER__

//グローバル変数

//現在x座標, y座標
extern char x_coordinate, y_coordinate;
//歩数カウンタ
extern char m_step;
//現在向き	北:0	東:1	南:2	西:3
extern signed char m_dir;
//壁あるなし(ある1,なし0)
extern char exist_l_wall, exist_r_wall, exist_f_wall;

//最短経路パス
extern unsigned char path[256];
//パスのサイズ
extern unsigned char path_size;
//壁の方位
extern enum dir{
	NORTH,
	EAST,
	SOUTH,
	WEST
};

void init_wall(void);
void init_stepMap(void);
void update_stepMap(void);
char adachi_judge_nextdir(void);
void generate_adachi_shortestRoute(void);
void add_wall(char _x_coordinate, char _y_coordinate, char _dir);
void set_wall(_x_coordinate, _y_coordinate, _m_dir);
char judge_wall(_x_coordinate, _y_coordinate, _dir);
void add_knownWall(char _x_coordinate, char _y_coordinate, char _dir);
char judge_knownWall(_x_coordinate, _y_coordinate, _dir);
void print_wall(void);
void print_shortestRoute(void);
void update_coordinate(void);
char goal_judge(void);

#endif