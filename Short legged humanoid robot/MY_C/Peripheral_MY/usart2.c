#include "sys.h"
#include "usart2.h"	
#include <string.h>
#include "led.h"


//#define USART1_SEND_BUF_SIZE 200

//char USART2_REC_BUF[USART2_REC_LEN];
//char USART1_SEND_BUF[USART1_SEND_BUF_SIZE];

void uart2_init(u32 pclk2,u32 bound,char* rec_buf)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	 
	//两个串口时钟基准原不通,因此波特率计算方式并不一样 现在程序默认APB1有2预分频,APB2不分频,因此有2倍关系  
	temp=(float)(pclk2*1000000)/(bound*2*16);//得到USARTDIV 多成的2是2倍关系
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分	 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //使能PORTA口时钟  
	RCC->APB1ENR|=1<<17;  //使能串口时钟 
	
	GPIOA->CRL&=0XFFFF00FF;//IO状态设置//PA2-3
	GPIOA->CRL|=0X00008B00;//IO状态设置//PA2复用推挽输出 Tx   PA3输入上拉 Rx 
	GPIOA->BSRR|=1<<3;
//	GPIOA->CRL|=0X00003300;
//	GPIOA->ODR|=1<<10;
//	GPIOA->BSRR|=1<<3;
//	GPIOA->BRR|=1<<3;	
//	GPIOA->BSRR|=1<<2;	
//	GPIOA->BRR|=1<<2;		
//	while(1);
	RCC->APB1RSTR|=1<<17;   //复位串口1
	RCC->APB1RSTR&=~(1<<17);//停止复位	   	   
	//波特率设置
 	USART2->BRR=mantissa; // 波特率设置	 
	USART2->CR1|=0X200C;  //UE使能 1位停止,无校验位.
	
	USART2->CR1|=1<<4;		//IDLE中断使能
	MY_NVIC_Init(3,3,USART2_IRQn,2);//组2，最低优先级 
	
	USART2->CR3 |= 3<<6;  //DMA方式

//DMA1通道使能
	RCC->AHBENR|=1<<0;//使能DMA1 RCC
//DMA TX配置	
	DMA1->IFCR |= 0xf<<24;	//清除DMA通道4的任何标记
	DMA1_Channel7->CPAR= (u32)(&USART2->DR);//DMA外设地址寄存器
	MY_NVIC_Init(3,2,DMA1_Channel7_IRQn,2);//组2，较低低优先级 
	
	//DMA RX配置
	DMA1->IFCR |= 0xf<<20;	//清除DMA通道5的任何标记
	DMA1_Channel6->CNDTR=USART2_REC_LEN;//接收USART2_REC_LEN个字符
	DMA1_Channel6->CPAR= (u32)(&USART2->DR);//DMA外设地址寄存器
	DMA1_Channel6->CMAR= (u32)rec_buf;	
	
	DMA1_Channel6->CCR=0x20a3; 
	
	MY_NVIC_Init(3,1,DMA1_Channel6_IRQn,2);//组2，较低低优先级 
}

void USART2_Send_Buf(char *buf , unsigned int len)
{
		DMA1_Channel7->CCR=0;//清除后才能写寄存器,手工停止
		DMA1->IFCR |= 0xf<<24;	//清除DMA通道7的任何标记
		DMA1_Channel7->CNDTR=len;
		DMA1_Channel7->CMAR= (u32)buf;	
		DMA1_Channel7->CCR=0x0093; //0b00 0000 1011 0010  通道优先级高 外设数据宽度 =内存宽度=8位  TCIE中断使能  
}

//void Usart2Send(unsigned char DataToSend)
//{
//	//将要发送的字节写到UART2的发送缓冲区
//	 USART2->DR = (DataToSend & (uint16_t)0x01FF);
////	USART_SendData(USART1, (unsigned char) DataToSend);
//	//等待发送完成
//  	while (!(USART2->SR & ((uint16_t)0x0080)));
//}

////输出字符串
//void PrintChar(char *s)
//{
//	u16 len=strlen(s);
//	USART1_Send_Buf(s,len);
//}

/**************************实现函数********************************************
*函数原型:		void UART1_Put_Char(unsigned char DataToSend)
*功　　能:		RS232发送一个字节
输入参数：
		unsigned char DataToSend   要发送的字节数据
输出参数：没有	
*******************************************************************************/
//void UART2_Put_Char(unsigned char DataToSend)
//{
//	//将要发送的字节写到UART1的发送缓冲区
//	 USART2->DR = (DataToSend & (uint16_t)0x01FF);
////	USART_SendData(USART1, (unsigned char) DataToSend);
//	//等待发送完成
//  	while (!(USART2->SR & ((uint16_t)0x0080)));
//}




