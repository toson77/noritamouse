#ifndef __MAZE_HEADER__
#define __MAZE_HEADER__

//�O���[�o���ϐ�

//����x���W, y���W
extern char x_coordinate, y_coordinate;
//�����J�E���^
extern char m_step;
//���݌���	�k:0	��:1	��:2	��:3
extern signed char m_dir;
//�ǂ���Ȃ�(����1,�Ȃ�0)
extern char exist_l_wall, exist_r_wall, exist_f_wall;

//�ŒZ�o�H�p�X
extern unsigned char path[256];
//�p�X�̃T�C�Y
extern unsigned char path_size;
//�ǂ̕���
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