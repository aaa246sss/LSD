#ifndef __DELAY_H
#define __DELAY_H 	

//	RCC->AHBENR|=1<<0;//ʹ��DMA1 RCC����������
void DMA_carry(char *des,char *sour,unsigned short len);//��һ�������ʹ��DMA1ʱ�Ӻ����ʹ�� 
void System_Clk_Init(unsigned char SYSCLK);
//void delay_ms(unsigned short nms);
void delay_us(unsigned long nus);
void delay(unsigned short ms);

void delay_System_ms(unsigned short nms);
void delay_System_us(unsigned short nus);
#endif





























