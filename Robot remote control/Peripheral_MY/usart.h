#ifndef __USART_H
#define __USART_H
//#include <stm32f10x_map.h>
//#include <stm32f10x_nvic.h> 
#include "sys.h"
#include "stdio.h"
#include "stdint.h"

#define USART_REC_LEN  			32  	//�����������ֽ��� 200
//#define EN_USART1_RX 					//ʹ�ܣ�1��/��ֹ��0������1����
	  
typedef unsigned short     int uint16_t;
typedef signed short     int   int16_t;


//extern u8 BUF_RX_PID;
//����봮���жϽ��գ��벻Ҫע�����º궨��


void USART1_Send_Buf(char *buf , unsigned int len);
void uart_init(u32 pclk2,u32 bound,char * rec_buf);
void Print(uint8_t num);
void PrintInt(uint16_t num);
void PrintChar(char *s);
void PrintHexInt16(int16_t num);
void UART1_Put_Char(unsigned char DataToSend);
void UART1_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);

#endif	   
















