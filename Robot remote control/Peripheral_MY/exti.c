#include "exti.h"
#include "led.h"
#include "key.h"
#include "delay.h"
#include "usart.h"


#include "stdio.h"
//extern u16 NRF_IRQ_OUT_TIME;

char send_flag;
u8 NRF_rx_flag=1;
u8 KEY_status;
extern u8 idle;
extern u8 NRF_Sync;
u8 key0_flag=0;
//u8 key1_flag=0;

void  Delay (uint32_t nCount)
{
	for(; nCount != 0; nCount--);
}



void EXTI9_5_IRQHandler(void)
{
	//u8 status;
	if(!KEY0)
	{
		delay(3);
		if(!KEY0)
		{
			//status=0;
			KEY_status=1;
			LED_R=0;
			key0_flag^=1;
			while(!KEY0);
			LED_R=1;
		}
		EXTI->PR|=1<<8;//���ж�
		//������1500
	}
	if(!KEY1)
	{
		delay(3);
		if(!KEY1)
		{
			//status=0;
//			KEY_status=1;
			LED_R=0;
			key0_flag^=2;
			while(!KEY1);
			LED_R=1;
		}
		EXTI->PR|=1<<9;//���ж�
		//������
	}
	
}

void EXTIX_Init(void)
{
//	RCC->APB2ENR|=1<<2;     //ʹ��PORTAʱ��
	RCC->APB2ENR|=1<<3;     //ʹ��PORTBʱ��
//	RCC->APB2ENR|=1<<4;     //ʹ��PORTCʱ��
	Ex_NVIC_Config(GPIO_B,8,FTIR); //�½��ش���
	
	Ex_NVIC_Config(GPIO_B,9,FTIR); //�½��ش���
	
	MY_NVIC_Init(2,0,EXTI9_5_IRQn,3);    //��ռ2�������ȼ�2����2
	
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









