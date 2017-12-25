#ifndef __DELAY_H
#define __DELAY_H 	

//	RCC->AHBENR|=1<<0;//使能DMA1 RCC放在主函数
void DMA_carry(char *des,char *sour,unsigned short len);//这一句必须先使能DMA1时钟后才能使用 
void System_Clk_Init(unsigned char SYSCLK);
//void delay_ms(unsigned short nms);
void delay_us(unsigned long nus);
void delay(unsigned short ms);

void delay_System_ms(unsigned short nms);
void delay_System_us(unsigned short nus);
#endif





























