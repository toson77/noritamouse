#include <stdarg.h>
#include "sci.h"
#include "iodefine.h"

char sci_put_1byte(unsigned char c){
	unsigned long i=0;
	unsigned short flag=0;
	
	// 送信バッファが解放されるのをポーリングで待機
	// タイムアウトあり
	for(i=0;i<0x02000000;i++){
		if(SCI1.SSR.BIT.TEND == 1){
			flag = 1;
			break;
		}
	}
	
	if(flag){
		SCI1.TDR = c;  // 送信バッファにデータセット
		return 1;
	}
	return 0;
}


// 文字列送信
short SCI_putstr(char *str){
	short len;
	
	len = 0;
	
	// 終端コードまで送信
	while(*str){
		if(!sci_put_1byte(*(str++))){
			return 0;
		}
		len++;
	}
	
	// 送信文字数を返す
	return len;
}

//  10進数文字かチェック
static short isDec(char c){
    if ('0'<=c && c<='9'){
	    return 1;
    }
    return 0;
}

//  数値を10進数文字変換
static void uint2Dec(unsigned long n, char *buf){
    char c;
    short len = 0;
    short i, half;

    // 10進文字列へ変換し文字数をカウント
    do{
        if(n == 0){
		i = 0;
	}
        else{
		i = n % 10;
	}
        buf[len] = (char)(i + '0');
        len++;
        n /= 10;
    }while(n != 0);

    // 文字の並び順を直す
    half = len >> 1;
    for(i=0; i < half; i++){
        c = buf[i];
        buf[i] = buf[(len-1)-i];
        buf[(len-1)-i] = c;
    }

    // 終端コードの挿入
    buf[len]='\0';
}


//  数値を16進数文字変換
static void uint2Hex(unsigned long n, short upper, char *buf){
    char c;
    char a = 'a';
    short len = 0;
    short i, half;

    // 大文字/小文字の設定
    if(upper) a = 'A';
    
    // 16進文字列へ変換し文字数をカウント
    do{
        i = n & 0x0F;
        if(i > 9){
		buf[len] = (unsigned char)(i + a - 10);
	}
        else{
		buf[len] = (unsigned char)(i + '0');
	}
        len++;
        n >>= 4;
    }while(n != 0);

    // 文字の並び順を直す
    half = len >> 1;
    for(i=0; i < half; i++){
        c = buf[i];
        buf[i] = buf[(len-1)-i];
        buf[(len-1)-i] = c;
    }

    // 終端コードの挿入 */
    buf[len]='\0';
}


//  指定フォーマットで表示する
static void printFormat(char *ptr, short order, short alignLeft, short fillZero, short minus){
    char *str = ptr;
    char pad = ' ';
    short len = 0;
    long  i;

    // 文字数のカウント
    for(len=0; *str != '\0'; len++, str++);
    
    // マイナスなら文字数調整
    if(minus){ 
    	len++;
    }

    // 文字数の調整
    if(order){
        if(order>len){
		order -= len;
	}
        else{
		order  = 0;
	}
    }

    // 右詰め
    if( ! alignLeft){
        // 詰め文字の設定
        if(fillZero){
            pad = '0';
            
            // マイナス表示
            if(minus){
                sci_put_1byte('-');
                minus = 0;
            }
        }

        for(i=0; i<order; i++){
            sci_put_1byte(pad);
	}
    }

    // マイナス表示
    if(minus)  sci_put_1byte('-');

    // データの表示
    SCI_putstr(ptr);

    /* 左詰め */
    if(alignLeft)
        for(i=0; i<order; i++)
            sci_put_1byte(' ');
}


// フォーマット変換
static char *parseFormat(char *str,void *value){
	short alignLeft=0;
	short fillZero=0;
	short order=0;
	short minus=0;
	char buf[12];
	char *ptr=buf;
	
	//左詰
	if(*str=='-'){ alignLeft = 1; str++; }
	
	//ゼロフィル
	if(*str=='0'){ fillZero = 1; str++; }
	
	// 文字数指定判定
	if(isDec(*str)){
		for(order=0;isDec(*str);){
			order *= 10;
			order += (*str - '0');
			str++;
		}
	}

	// 種類判定,表示準備
	switch(*str){
		case 'd':  // 符号付10進数(short型)
		case 'D':  // 符号付10進数(short型)
			if((signed short)value >= 0){
				uint2Dec((unsigned short)value , buf);
			}else{
				uint2Dec((unsigned short)(-(signed short)value) , buf); 
				minus = 1;
			}
			break;
		case 'l':  // 符号付10進数(long型)
		case 'L':  // 符号付10進数(long型)
			if((signed long)value >= 0){
				uint2Dec((unsigned long)value , buf);
			}else{
				uint2Dec((unsigned long)(-(signed long)value) , buf); 
				minus = 1;
			}
			break;
		case 'u':   // 符号なし10進数
		case 'U':   // 符号なし10進数
			uint2Dec((unsigned short)value , buf);
			break;
		case 'x':   // 小文字16進数
			uint2Hex((unsigned long)value , 0 , buf);
			break;
		case 'X':   // 大文字16進数
			uint2Hex((unsigned long)value , 1 , buf);
			break;
		case 's':   // 文字列
		case 'S':   // 文字列
			ptr = (char *)value;
			break;
		case 'c':   // 1文字
		case 'C':   // 1文字
			buf[0] = (char)((unsigned long)value & 0xFF);
			buf[1] = '\0';
			break;
		default: 
			buf[0] = '\0';
			break;
	}

	// 表示
	printFormat(ptr,order,alignLeft,fillZero,minus);
	
	str++;
	return(str);
	
}


// 文字列表示printf
short sci_printf(char *str , ...){
	va_list ap;
	char *ptr;
	
	ptr = str;
	
	// 可変引数の初期化
	va_start(ap,str);
	
	while(*ptr){
		// 特殊文字
		if(*ptr=='%'){
			ptr++;
			
			if(*ptr=='%'){
				sci_put_1byte('%'); 
				ptr++;	
			} 
			else{
				ptr = parseFormat(ptr,va_arg(ap,void *));
				if(ptr == 0){ 
					return -1;
				}
			}
		}
		// 通常文字
		else{
			sci_put_1byte(*ptr);
			ptr++;
		}
	}
	va_end(ap);
	return (ptr-str);  // 送信文字数
}