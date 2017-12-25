
/*

STM对MPU9250
+5V			VCC_IN(内含3.3V稳压)	+5V								电源+
GND			GND										GND								电源-
SCL			SCL/SCLK							PB10(I2C2-SCL)		I2C时钟线
SDA			SDA/MOSI							PB11(I2C2-SDA)		I2C数据线
NONE		NCS(?????)						+3V 							高电平,MOSI时有用
sel AD 	AD/MOSI 							GND								拉低地址68,拉高69

STM对串口
电脑RX PA9
电脑TX PA10

STM对24L01
1	GND		GND
2 VCC		VCC                                                                                                                                                                                                                                                                                                                                                                                                                            
3 CE		PB11
4 CSN		PB12
5 SCK		PB13	
6 MOSI	PB15
7 MISO	PB14
8 IRQ		PB10

*/
#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "timer.h"
#include "pid.h"
#include "kalman_MY.h"
#include <stdio.h>
#include <stdlib.h>
_increasePID increasePID;
short GET_ADC_value(void);
void ADC_1td_DMA_init_MY(void);
int main(void)
{
	char TX_BUF[32];
	short ADC_valve;
//	short PWM_temp=0;
	int i;
	int irand=0,orand=0;
//初始化代码
	{	
		Stm32_Clock_Init(9); //系统时钟设置
		System_Clk_Init(72);
		uart_init(72,115200,TX_BUF);
		ADC_1td_DMA_init_MY();
		TIMx_PWM_Init(2,5000,0,0x04);//频率72000000/5000/2=7200Hz
		increasePIDInit(&increasePID);
		delay(3);
		ADC_valve=GET_ADC_value();
		srand(ADC_valve);
	}
//主循环	
	for(i=0;i<100000;i++)
	{
		irand=(rand()%30+85)*10000;
		orand=KalMan_Update_Int_MY(irand);//整数时代码量9120 这一句多116代码量
//		irand=(rand()%30+85);
//		orand=KalMan_Update_Float_MY(irand);
	}
	for(i=0;i<200;i++)
	{
		irand=(rand()%30+85)*10000;
		orand=KalMan_Update_Int_MY(irand);
//		orand=KalMan_Update_Float_MY(irand);
		printf("%d,%d\r\n",irand,orand);
//		delay(1);
//		irand=(rand()%30+85);
		
//		printf("%f,%f\r\n",irand,orand);

	}
	while(1)
	{
		
//		ADC_valve=GET_ADC_value();
//		if(ADC_valve!=-1)
//		{
//			ADC_valve=4095-ADC_valve;
//			PWM_temp+=increasePIDcal(&increasePID,ADC_valve);
//			if(PWM_temp<0)
//				PWM_temp=0;
//			if(PWM_temp>5000)
//				PWM_temp=5000;
//			TIM2->CCR3=PWM_temp;//改变占空比值
//		}
	}
}	 




