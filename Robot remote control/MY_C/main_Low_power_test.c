#include <stm32f10x_RCC.h>


#include "misc.h"	//中断
#include <stm32f10x_PWR.h>
#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "timer.h"
#include "pid.h"
#include "kalman_MY.h"
#include "Low_power.h"
#include <stdio.h>
#include <stdlib.h>

extern char USART1_REC_BUF[USART_REC_LEN+1];

_increasePID increasePID;
short GET_ADC_value(void);
void ADC_1td_DMA_init_MY(void);
int main(void)
{
//	char TX_BUF[32];
//	short ADC_valve;
//	short PWM_temp=0;
//	int i;
//	int irand=0,orand=0;
//初始化代码
	{	
		Stm32_Clock_Init(9); //系统时钟设置
		System_Clk_Init(72);
		uart_init(72,115200,USART1_REC_BUF);
//		ADC_1td_DMA_init_MY();
		TIMx_PWM_Init(2,5000,0,0x04);//频率72000000/5000/2=7200Hz
//		increasePIDInit(&increasePID);
		delay(3);
//		ADC_valve=GET_ADC_value();
//		srand(ADC_valve);
//		RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM2,ENABLE);//这两句需要同时 用
//		RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM2,DISABLE);
//		TIMx_PWM_Init(2,5000,0,0x04);//频率72000000/5000/2=7200Hz
//		PWR_EnterSTOPMode(PWR_Regulator_ON,PWR_STOPEntry_WFI);
//		NVIC_SystemLPConfig(NVIC_LP_SLEEPONEXIT,ENABLE);
		CPU_SLEEP_INT_MODE(ENABLE);
		printf("asdf\r\n");
		CPU_SLEEP_ENTER();
	}
//主循环
	while(1)
	{
		printf("asdf\r\n");
		delay(1000);
	}
}	 




