#ifndef __INTERRUPT_HEADER__
#define __INTERRUPT_HEADER__

void mtu3_tgra(void);
void mtu3_tgrb(void);
void mtu4_tgrb(void);
void increment_timer_ms(void);
void increment_timer_sec(void);
void increment_my_timer_ms();
unsigned int get_time(char type);
void wait_sec(unsigned int sec);
void wait_ms(unsigned int ms);
void battery_low_notification(void);

enum type_time{
	TYPE_MS,
	TYPE_SEC,
	TYPE_MYMS
};

#endif