#ifndef __ENCODER_HEADER__
#define __ENCODER_HEADER__

void calc_enc_value();
float calc_r_enc_moving_ave(short value);
float calc_l_enc_moving_ave(short value);
short get_enc_value(char encoder);
short get_current_enc_velocity(char encoder);
unsigned short duty_to_count(short duty);

enum encoder{
	RIGHT_ENCODER,
	LEFT_ENCODER
};

#endif