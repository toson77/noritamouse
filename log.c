#include "parameter.h"
#include "log.h"
#include "sci.h"
#include "encoder.h"
#include "interrupt.h"
#include "run.h"

#define LOG_SIZE	1500

short run_log[4][LOG_SIZE] = {0};
unsigned short log_cnt = 0;

void log_save(short value,short value2,short value3, short value4){
	if(log_cnt >= LOG_SIZE-1)	return;
	run_log[0][log_cnt] = value;
	run_log[1][log_cnt] = value2;
	run_log[2][log_cnt] = value3;
	run_log[3][log_cnt] = value4;
	log_cnt ++;
}

void print_run_log(void){
	short i;
	for(i=0;i<LOG_SIZE;i++){
		
		sci_printf("%l,%l,%l,%l\r\n", run_log[0][i],run_log[1][i], run_log[2][i],run_log[3][i]);
	}
}
