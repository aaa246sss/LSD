#ifndef __USART2_H
#define __USART2_H
//#include <stm32f10x_map.h>
//#include <stm32f10x_nvic.h> 
#include "stdio.h"
#include "stdint.h"

#define USART2_REC_LEN  			1000  	//定义最大接收字节数 200

typedef unsigned short     int uint16_t;
typedef signed short     int   int16_t;

void USART2_Send_Buf(char *buf , unsigned int len);
void uart2_init(u32 pclk2,u32 bound,char *rec_buf);
void UART2_Put_Char(unsigned char DataToSend);

#endif	   
















