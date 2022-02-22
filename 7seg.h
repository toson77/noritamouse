#ifndef __7SEG_HEADER__
#define __7SEG_HEADER__
void ledseg_1_ON(void);
void ledseg_1_OFF(void);
void ledseg_2_ON(void);
void ledseg_2_OFF(void);
void ledseg_3_ON(void);
void ledseg_3_OFF(void);
void ledseg_4_ON(void);
void ledseg_4_OFF(void);
void ledseg_5_ON(void);
void ledseg_5_OFF(void);
void ledseg_6_ON(void);
void ledseg_6_OFF(void);
void ledseg_7_ON(void);
void ledseg_7_OFF(void);
void ledseg_8_ON(void);
void ledseg_8_OFF(void);
void ledseg_9_ON(void);
void ledseg_9_OFF(void);
void ledseg_x_ON(void);
void ledseg_x_OFF(void);
void ledseg_x_interrupt(short count);
void ledseg_1_interrupt(short count);
void ledseg_2_interrupt(short count);
void ledseg_3_interrupt(short count);
void ledseg_4_interrupt(short count);
void ledseg_5_interrupt(short count);
void ledseg_6_interrupt(short count);
void ledseg_7_interrupt(short count);
void ledseg_8_interrupt(short count);
void ledseg_9_interrupt(short count);
void ledseg_all_off(void);
void LED(short mode);
#endif