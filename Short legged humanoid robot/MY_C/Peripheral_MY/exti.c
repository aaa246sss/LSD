#include "exti.h"
#include "led.h"
#include "key.h"
#include "delay.h"
#include "usart.h"

#include "NRF24L01.h"
#include "stdio.h"
extern u8 KEY_status;
void EXTIX_Init(void)
{
//	RCC->APB2ENR|=1<<2;     //使能PORTA时钟
	RCC->APB2ENR|=1<<3;     //使能PORTB时钟
//	RCC->APB2ENR|=1<<4;     //使能PORTC时钟
	GPIOB->CRH&=0XFFFFFF00;
	GPIOB->CRH|=0X33000088;//PA11 PA12 PA15 普通IO 推挽输出
	GPIOB->BSRR|=3<<8;
	Ex_NVIC_Config(GPIO_B,8,FTIR); //下降沿触发
	
	Ex_NVIC_Config(GPIO_B,9,FTIR); //下降沿触发
	
	MY_NVIC_Init(5,0,EXTI9_5_IRQn,3);    //抢占2，子优先级2，组2
	
	Ex_NVIC_Config(GPIO_B,10,FTIR); //下降沿触发
	MY_NVIC_Init(1,0,EXTI15_10_IRQn,3);    //抢占2，子优先级2，组2
}

u8 read_rest_KEY_status_flag(void)
{
	u8 temp;
	temp=KEY_status;
	KEY_status=0;
	return temp;
}








