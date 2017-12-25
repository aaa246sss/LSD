//#include <stm32f10x_map.h>
//#include <stm32f10x_nvic.h>   
#include "led.h"

//初始化PA8和PD2为输出口.并使能这两个口的时钟		    
//LED IO初始化
void LED_Init(void)
{
//	RCC->APB2ENR|=1<<2;    //使能PORTA时钟	   	 
	
	RCC->APB2ENR|=1<<3;
	
	GPIOB->CRL&=0X00FFFFFF;
	GPIOB->CRL|=0X33000000;//PA11 PA12 PA15 普通IO 推挽输出

	
	LED_G=1;
	LED_R=1;
}






