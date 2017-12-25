#ifndef __USART_H
#define __USART_H
//#include <stm32f10x_map.h>
//#include <stm32f10x_nvic.h> 
#include "stm32f4xx_hal.h"
#include "MAIN_MY.h"
#include "stdio.h"
#include "stdint.h"
//#define u32 unsigned long

//#define USART_REC_LEN  			1000  	//定义最大接收字节数 200
//#define EN_USART1_RX 					//使能（1）/禁止（0）串口1接收
	  
//typedef unsigned short     int uint16_t;
//typedef signed short     int   int16_t;


//extern u8 BUF_RX_PID;
//如果想串口中断接收，请不要注释以下宏定义


void USART1_Send_Buf(char *buf , unsigned int len);
void uart_init_PB6_PB7(uint32_t pclk2,uint32_t bound,char *rec_buf);
void USART_CONFIG(GLOVAR_HandleTypeDef *glovar);
void Print(uint8_t num);
void PrintInt(uint16_t num);
void PrintChar(char *s);
void PrintHexInt16(int16_t num);
void UART1_Put_Char(unsigned char DataToSend);
void UART1_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);

#endif	   
















