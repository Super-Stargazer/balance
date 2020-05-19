#ifndef __PRINTF_DEF_H  //该文件仅供储存代码
#define __PRINTF_DEF_H
#include "zdsys.h"


#ifndef PRINTF_ENABLE
#include "stdio.h"
    #ifdef __GNUC__
        #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
    #else
        #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
    #endif

PUTCHAR_PROTOTYPE
{
    while((USART1->SR&0X40)==0);
	USART1->DR = (u8) ch;
	return ch;
}
#endif


#ifdef __DEBUG_VERSION
	#define DEBUG(format,...) printf("[File:%s, Line:%d] "format, __FILE__, __LINE__, ##__VA_ARGS__)
#else
	#define DEBUG(format,...)
#endif


#endif
