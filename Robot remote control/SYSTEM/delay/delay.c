//#include <stm32f10x_map.h>
//#include <stm32f10x_nvic.h>
#include "delay.h"
#include "sys.h"

static u8  fac_us=0;//us��ʱ������
static u16 fac_ms=0;//ms��ʱ������
//��ʼ���ӳٺ���
//SYSTICK��ʱ�ӹ̶�ΪHCLKʱ�ӵ�1/8
//SYSCLK:ϵͳʱ��
void System_Clk_Init(u8 SYSCLK)
{
	SysTick->CTRL&=0xfffffffb;//bit2���,ѡ���ⲿʱ��  HCLK/8
	fac_us=SYSCLK/8;		    
	fac_ms=(u16)fac_us*1000;
}	
void delay_System_ms(u16 nms)
{
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;//ʱ�����(SysTick->LOADΪ24bit)
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

void delay_System_us(u16 nus)
{
	u32 temp;	    	 
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
//void delay_ms(u16 nms)
//{	 		  	  
//u32 i,j;
//	for(i=0;i<nms;i++)
//		for(j=0;j<7978;j++);  
//}   
void delay_us(u32 nus)
{		

		u16 i;
	u8 j;
	for(i=0;i<nus;i++)
 		for(j=0;j<7;j++);
}
void delay(u16 ms)
{
	u32 i,j;
	for(i=0;i<ms;i++)
		for(j=0;j<7978;j++);
}
void DMA_carry(char *des,char *sour,u16 len)
{
	//���ж�����֮ǰ�Ƿ������ ���ж�ʹ�����ⷽʽ,��ʹ��ʣ��Ϊ���������ж�
	des[len]='\0';
	while(DMA1_Channel1->CNDTR);
	DMA1_Channel1->CCR=0; //8λ��8λ 
	DMA1->IFCR |= 0xf<<0;	//���DMAͨ��1���κα��
	DMA1_Channel1->CNDTR=len;//
	DMA1_Channel1->CPAR= (u32)sour;//DMA�����ַ�Ĵ���
	DMA1_Channel1->CMAR= (u32)des;	
	
	DMA1_Channel1->CCR=0x50c1; //8λ��8λ 
}






































