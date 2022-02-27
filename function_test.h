#ifndef __FUNCTIONTEST_HEADER__
#define __FUNCTIONTEST_HEADER__

void motor_drive_test();
void motor_stop();
void motor_rotation();
void move_forward_specified_duty(short duty);
void move_backward_specified_duty(short duty);
void rotate_clockwise_specified_duty(short duty);
void rotate_anticlockwise_specified_duty(short duty);

void encoder_test1(void);
void encoder_test2(void);
void encoder_test3(void);

void flash_led(void);

void sci_put_test(void);
void sci_put_string_test(void);

void adc_test1(void);
void adc_test2(void);
void adc_test3(void);
void adc_test4(void);
void adc_test_all(void);
void battery_ad_test(void);

int get_sign(float num);

#endif