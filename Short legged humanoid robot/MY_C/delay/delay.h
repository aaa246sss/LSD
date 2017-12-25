#ifndef __DELAY_H
#define __DELAY_H 	
#include "stm32f4xx_hal.h"
//	RCC->AHBENR|=1<<0;//使能DMA1 RCC放在主函数
void DMA_carry(char *des,char *sour,uint16_t len);//这一句必须先使能DMA1时钟后才能使用 
void System_Clk_Init(unsigned char SYSCLK);
//void delay_ms(unsigned short nms);
void delay_us(uint32_t nus);
void delay(uint16_t ms);

void delay_System_ms(uint16_t nms);
void delay_System_us(uint16_t nus);
#endif





























