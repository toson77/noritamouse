#include "7seg.h"
#include "define.h"
#include "interrupt.h"

void ledseg_1_ON(void){
	 LED_7SEG_1 = 0;
	 LED_7SEG_2 = 0;
}
void ledseg_1_OFF(void){
	LED_7SEG_1 = 1;
	LED_7SEG_2 = 1;
}
void ledseg_1_interrupt(short count) {
	for(int i = 0; i < count; i++) {
		ledseg_1_OFF();
		wait_ms(100);
		ledseg_1_ON();
		wait_ms(100);
	}
}
	
void ledseg_2_ON(void){
	 LED_7SEG_5 = 0;
	 LED_7SEG_1 = 0;
	 LED_7SEG_4 = 0;
	 LED_7SEG_7 = 0;
	 LED_7SEG_3 = 0;
}
void ledseg_2_OFF(void){
	 LED_7SEG_5 = 1;
	 LED_7SEG_1 = 1;
	 LED_7SEG_4 = 1;
	 LED_7SEG_7 = 1;
	 LED_7SEG_3 = 1;
}
void ledseg_2_interrupt(short count) {
	for(int i = 0; i < count; i++) {
		ledseg_2_OFF();
		wait_ms(100);
		ledseg_2_ON();
		wait_ms(100);
	}
}
void ledseg_3_ON(void){
	 LED_7SEG_5 = 0;
	 LED_7SEG_1 = 0;
	 LED_7SEG_4 = 0;
	 LED_7SEG_2 = 0;
	 LED_7SEG_3 = 0;
}
void ledseg_3_OFF(void){
	 LED_7SEG_5 = 1;
	 LED_7SEG_1 = 1;
	 LED_7SEG_4 = 1;
	 LED_7SEG_2 = 1;
	 LED_7SEG_3 = 1;
}
void ledseg_3_interrupt(short count) {
	for(int i = 0; i < count; i++) {
		ledseg_3_OFF();
		wait_ms(100);
		ledseg_3_ON();
		wait_ms(100);
	}
}
void ledseg_4_ON(void){
	 LED_7SEG_6 = 0;
	 LED_7SEG_1 = 0;
	 LED_7SEG_4 = 0;
	 LED_7SEG_2 = 0;
}
void ledseg_4_OFF(void){
	 LED_7SEG_6 = 1;
	 LED_7SEG_1 = 1;
	 LED_7SEG_4 = 1;
	 LED_7SEG_2 = 1;
}
void ledseg_4_interrupt(short count) {
	for(int i = 0; i < count; i++) {
		ledseg_4_OFF();
		wait_ms(100);
		ledseg_4_ON();
		wait_ms(100);
	}
}
void ledseg_5_ON(void){
	 LED_7SEG_5 = 0;
	 LED_7SEG_6 = 0;
	 LED_7SEG_4 = 0;
	 LED_7SEG_2 = 0;
	 LED_7SEG_3 = 0;
}
void ledseg_5_OFF(void){
	 LED_7SEG_5 = 1;
	 LED_7SEG_6 = 1;
	 LED_7SEG_4 = 1;
	 LED_7SEG_2 = 1;
	 LED_7SEG_3 = 1;
}
void ledseg_5_interrupt(short count) {
	for(int i = 0; i < count; i++) {
		ledseg_5_OFF();
		wait_ms(100);
		ledseg_5_ON();
		wait_ms(100);
	}
}
void ledseg_6_ON(void){
	 LED_7SEG_7 = 0;
	 LED_7SEG_6 = 0;
	 LED_7SEG_4 = 0;
	 LED_7SEG_2 = 0;
	 LED_7SEG_3 = 0;
}
void ledseg_6_OFF(void){
	 LED_7SEG_7 = 1;
	 LED_7SEG_6 = 1;
	 LED_7SEG_4 = 1;
	 LED_7SEG_2 = 1;
	 LED_7SEG_3 = 1;
}
void ledseg_6_interrupt(short count) {
	for(int i = 0; i < count; i++) {
		ledseg_6_OFF();
		wait_ms(100);
		ledseg_6_ON();
		wait_ms(100);
	}
}
void ledseg_7_ON(void){
	 LED_7SEG_5 = 0;
	 LED_7SEG_6 = 0;
	 LED_7SEG_1 = 0;
	 LED_7SEG_2 = 0;
}
void ledseg_7_OFF(void){
	 LED_7SEG_5 = 1;
	 LED_7SEG_6 = 1;
	 LED_7SEG_1 = 1;
	 LED_7SEG_2 = 1;
}
void ledseg_7_interrupt(short count) {
	for(int i = 0; i < count; i++) {
		ledseg_7_OFF();
		wait_ms(100);
		ledseg_7_ON();
		wait_ms(100);
	}
}
void ledseg_8_ON(void){
	 LED_7SEG_1 = 0;
	 LED_7SEG_2 = 0;
	 LED_7SEG_3 = 0;
	 LED_7SEG_4 = 0;
	 LED_7SEG_5 = 0;
	 LED_7SEG_6 = 0;
	 LED_7SEG_7 = 0;
}
void ledseg_8_OFF(void){
	 LED_7SEG_1 = 1;
	 LED_7SEG_2 = 1;
	 LED_7SEG_3 = 1;
	 LED_7SEG_4 = 1;
	 LED_7SEG_5 = 1;
	 LED_7SEG_6 = 1;
	 LED_7SEG_7 = 1;
}
void ledseg_8_interrupt(short count) {
	for(int i = 0; i < count; i++) {
		ledseg_8_OFF();
		wait_ms(100);
		ledseg_8_ON();
		wait_ms(100);
	}
}
void ledseg_9_ON(void){
	 LED_7SEG_5 = 0;
	 LED_7SEG_6 = 0;
	 LED_7SEG_1 = 0;
	 LED_7SEG_4 = 0;
	 LED_7SEG_2 = 0;
}
void ledseg_9_OFF(void){
	 LED_7SEG_5 = 1;
	 LED_7SEG_6 = 1;
	 LED_7SEG_1 = 1;
	 LED_7SEG_4 = 1;
	 LED_7SEG_2 = 1;
}
void ledseg_9_interrupt(short count) {
	for(int i = 0; i < count; i++) {
		ledseg_9_OFF();
		wait_ms(100);
		ledseg_9_ON();
		wait_ms(100);
	}
}

void ledseg_x_ON(void) {
	 LED_7SEG_6 = 0;
	 LED_7SEG_4 = 0;
	 LED_7SEG_2 = 0;
	 LED_7SEG_1 = 0;
	 LED_7SEG_7 = 0;
}

void ledseg_x_OFF(void) {
	 LED_7SEG_6 = 1;
	 LED_7SEG_4 = 1;
	 LED_7SEG_2 = 1;
	 LED_7SEG_1 = 1;
	 LED_7SEG_7 = 1;
}
// バッテリー低下サインに使用。割り込み止めるのでwait_ms使えない。
void ledseg_x_interrupt(short count) {
	for(int i = 0; i < count; i++) {
		ledseg_all_off();
		for(i = 0; i < 100*1000*10; i++);
		ledseg_x_ON();
		for(i = 0; i < 100*1000*10; i++);
	}
}

void ledseg_all_off(void) {
	 LED_7SEG_1 = 1;
	 LED_7SEG_2 = 1;
	 LED_7SEG_3 = 1;
	 LED_7SEG_4 = 1;
	 LED_7SEG_5 = 1;
	 LED_7SEG_6 = 1;
	 LED_7SEG_7 = 1;
}

void LED(short mode) {
	//init
	ledseg_all_off();
	
	switch(mode) {
		case 1:
			ledseg_1_ON();
			break;
		case 2:
			ledseg_2_ON();
			break;
		case 3:
			ledseg_3_ON();
			break;
		case 4:
			ledseg_4_ON();
			break;
		case 5:
			ledseg_5_ON();
			break;
		case 6:
			ledseg_6_ON();
			break;
		case 7:
			ledseg_7_ON();
			break;
		case 8:
			ledseg_8_ON();
			break;
		case 9:
			ledseg_9_ON();
			break;
		default:
			break;
	}
}
			
			