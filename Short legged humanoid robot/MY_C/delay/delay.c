//#include <stm32f10x_map.h>
//#include <stm32f10x_nvic.h>
#include "delay.h"
#include "stm32f4xx_hal.h"
//#include "sys.h"

static uint8_t  fac_us=0;//us延时倍乘数
static uint16_t fac_ms=0;//ms延时倍乘数
//初始化延迟函数
//SYSTICK的时钟固定为HCLK时钟的1/8
//SYSCLK:系统时钟
void System_Clk_Init(uint8_t SYSCLK)
{
	SysTick->CTRL&=0xfffffffb;//bit2清空,选择外部时钟  HCLK/8
	fac_us=SYSCLK/8;		    
	fac_ms=(uint16_t)fac_us*1000;
}	
void delay_System_ms(uint16_t nms)
{
	uint32_t temp;		   
	SysTick->LOAD=(uint32_t)nms*fac_ms;//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;           //清空计数器
	SysTick->CTRL=0x01 ;          //开始倒数  
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	  	  
}

void delay_System_us(uint16_t nus)
{
	uint32_t temp;	    	 
	SysTick->LOAD=nus*fac_us; //时间加载	  		 
	SysTick->VAL=0x00;        //清空计数器
	SysTick->CTRL=0x01 ;      //开始倒数 	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	 
}
//延时nms
//注意nms的范围
//SysTick->LOAD为24位寄存器,所以,最大延时为:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms
//对72M条件下,nms<=1864 
//void delay_ms(uint16_t nms)
//{	 		  	  
//uint32_t i,j;
//	for(i=0;i<nms;i++)
//		for(j=0;j<7978;j++);  
//}   
void delay_us(uint32_t nus)
{		
	uint16_t i;
	uint8_t j;
	for(i=0;i<nus;i++)
	for(j=0;j<26;j++);
}
void delay(uint16_t ms)
{
	uint32_t i,j;
	for(i=0;i<ms;i++)
		for(j=0;j<28500;j++);
}
//void DMA_carry(char *des,char *sour,uint16_t len)
//{
//	//先判断数据之前是否发送完成 本判断使用特殊方式,即使用剩余为传输数量判断
//	des[len]='\0';
//	while(DMA1_Channel1->CNDTR);
//	DMA1_Channel1->CCR=0; //8位到8位 
//	DMA1->IFCR |= 0xf<<0;	//清除DMA通道1的任何标记
//	DMA1_Channel1->CNDTR=len;//
//	DMA1_Channel1->CPAR= (uint32_t)sour;//DMA外设地址寄存器
//	DMA1_Channel1->CMAR= (uint32_t)des;	
//	
//	DMA1_Channel1->CCR=0x50c1; //8位到8位 
//}






































