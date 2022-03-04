#ifndef __RUN_HEADER__
#define __RUN_HEADER__

void straight(float _length, float _top_speed, float _end_speed, float _accel, char _wall_control, char _forward_wall, char _next_dir);
void turn( float _angle, float _top_omega, float _end_omega, float _alpha );
void slalom( float _angle, float _top_omega, float _end_omega, float _alpha );

void control_speed(void);
void pid_speed(void);
void control_wall(void);
void f_wall_control(void);
void enable_f_wall_control(void);
void change_motor_speed(void);
void reset_run_status(void);
void print_state_test(void);
void print_run_log(void);
void revision_back(float _length, float top_speed, float accell);
enum wall{
	RIGHT_WALL,
	LEFT_WALL,
	FRONT_WALL
};

#endif
