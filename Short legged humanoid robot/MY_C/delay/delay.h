#ifndef __DELAY_H
#define __DELAY_H 	
#include "stm32f4xx_hal.h"
//	RCC->AHBENR|=1<<0;//ʹ��DMA1 RCC����������
void DMA_carry(char *des,char *sour,uint16_t len);//��һ�������ʹ��DMA1ʱ�Ӻ����ʹ�� 
void System_Clk_Init(unsigned char SYSCLK);
//void delay_ms(unsigned short nms);
void delay_us(uint32_t nus);
void delay(uint16_t ms);

void delay_System_ms(uint16_t nms);
void delay_System_us(uint16_t nus);
#endif





























