//#include <stm32f10x_map.h>
//#include <stm32f10x_nvic.h>   
#include "led.h"

//��ʼ��PA8��PD2Ϊ�����.��ʹ���������ڵ�ʱ��		    
//LED IO��ʼ��
void LED_Init(void)
{
//	RCC->APB2ENR|=1<<2;    //ʹ��PORTAʱ��	   	 
	
	RCC->APB2ENR|=1<<3;
	
	GPIOB->CRL&=0X00FFFFFF;
	GPIOB->CRL|=0X33000000;//PA11 PA12 PA15 ��ͨIO �������

	
	LED_G=1;
	LED_R=1;
}






