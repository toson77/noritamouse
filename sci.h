#ifndef __SCI_HEADER__
#define __SCI_HEADER__

char sci_put_1byte(unsigned char c);
short SCI_putstr(char *str);
static short isDec(char c);
static void uint2Dec(unsigned long n, char *buf);
static void uint2Hex(unsigned long n, short upper, char *buf);
static void printFormat(char *ptr, short order, short alignLeft, short fillZero, short minus);
static char *parseFormat(char *str,void *value);
short sci_printf(char *str , ...);

#endif

