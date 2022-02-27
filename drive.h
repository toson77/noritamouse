#ifndef __DRIVE_HEADER__
#define __DRIVE_HEADER__

void direction_r_mot(char direction);
void direction_l_mot(char direction);

enum direction{
	MOT_FORWARD,
	MOT_BACKWARD,
	MOT_STOP, 
	MOT_BRAKE
};

#endif