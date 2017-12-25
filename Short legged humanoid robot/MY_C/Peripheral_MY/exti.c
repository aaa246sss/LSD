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
//	RCC->APB2ENR|=1<<2;     //ʹ��PORTAʱ��
	RCC->APB2ENR|=1<<3;     //ʹ��PORTBʱ��
//	RCC->APB2ENR|=1<<4;     //ʹ��PORTCʱ��
	GPIOB->CRH&=0XFFFFFF00;
	GPIOB->CRH|=0X33000088;//PA11 PA12 PA15 ��ͨIO �������
	GPIOB->BSRR|=3<<8;
	Ex_NVIC_Config(GPIO_B,8,FTIR); //�½��ش���
	
	Ex_NVIC_Config(GPIO_B,9,FTIR); //�½��ش���
	
	MY_NVIC_Init(5,0,EXTI9_5_IRQn,3);    //��ռ2�������ȼ�2����2
	
	Ex_NVIC_Config(GPIO_B,10,FTIR); //�½��ش���
	MY_NVIC_Init(1,0,EXTI15_10_IRQn,3);    //��ռ2�������ȼ�2����2
}

u8 read_rest_KEY_status_flag(void)
{
	u8 temp;
	temp=KEY_status;
	KEY_status=0;
	return temp;
}








