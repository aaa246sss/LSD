//#include <stm32f10x_map.h>
//#include <stm32f10x_nvic.h>
#include "delay.h"
#include "stm32f4xx_hal.h"
//#include "sys.h"

static uint8_t  fac_us=0;//us��ʱ������
static uint16_t fac_ms=0;//ms��ʱ������
//��ʼ���ӳٺ���
//SYSTICK��ʱ�ӹ̶�ΪHCLKʱ�ӵ�1/8
//SYSCLK:ϵͳʱ��
void System_Clk_Init(uint8_t SYSCLK)
{
	SysTick->CTRL&=0xfffffffb;//bit2���,ѡ���ⲿʱ��  HCLK/8
	fac_us=SYSCLK/8;		    
	fac_ms=(uint16_t)fac_us*1000;
}	
void delay_System_ms(uint16_t nms)
{
	uint32_t temp;		   
	SysTick->LOAD=(uint32_t)nms*fac_ms;//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;           //��ռ�����
	SysTick->CTRL=0x01 ;          //��ʼ����  
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL=0x00;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	  	  
}

void delay_System_us(uint16_t nus)
{
	uint32_t temp;	    	 
	SysTick->LOAD=nus*fac_us; //ʱ�����	  		 
	SysTick->VAL=0x00;        //��ռ�����
	SysTick->CTRL=0x01 ;      //��ʼ���� 	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL=0x00;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	 
}
//��ʱnms
//ע��nms�ķ�Χ
//SysTick->LOADΪ24λ�Ĵ���,����,�����ʱΪ:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK��λΪHz,nms��λΪms
//��72M������,nms<=1864 
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
//	//���ж�����֮ǰ�Ƿ������ ���ж�ʹ�����ⷽʽ,��ʹ��ʣ��Ϊ���������ж�
//	des[len]='\0';
//	while(DMA1_Channel1->CNDTR);
//	DMA1_Channel1->CCR=0; //8λ��8λ 
//	DMA1->IFCR |= 0xf<<0;	//���DMAͨ��1���κα��
//	DMA1_Channel1->CNDTR=len;//
//	DMA1_Channel1->CPAR= (uint32_t)sour;//DMA�����ַ�Ĵ���
//	DMA1_Channel1->CMAR= (uint32_t)des;	
//	
//	DMA1_Channel1->CCR=0x50c1; //8λ��8λ 
//}





































