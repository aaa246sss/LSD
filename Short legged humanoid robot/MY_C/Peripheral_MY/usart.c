//#include "sys.h"

#include "usart.h"	
#include <string.h>
#include "MAIN_MY.h"
//#include "banlance.h"
//#include "led.h"

//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#ifdef KEIL
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数     
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
//void _sys_exit(int x) 
//{ 
//	x = x; 
//} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
	USART2->DR = (uint8_t) ch;      
	return ch;
}
#else 

#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
 
PUTCHAR_PROTOTYPE
{
    while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
    USART2->DR = (uint8_t) ch;   
    return ch;
}

#endif 

 
//u8 BUF_RX_PID;

//void uart_init_PB6_PB7(uint32_t pclk2,uint32_t bound,char *rec_buf)
//{  	 
//	float temp;
//	uint16_t mantissa;
//	uint16_t fraction;	   
//	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
//	mantissa=temp;				 //得到整数部分
//	fraction=(temp-mantissa)*16; //得到小数部分	 
//    mantissa<<=4;
//	mantissa+=fraction; 
//	RCC->APB2ENR|=1<<0;   //使能afio口时钟  //开启复用前必须
//	RCC->APB2ENR|=1<<2;   //使能PORTA口时钟  //开启复用前必须
//	RCC->APB2ENR|=1<<3;   //使能PORTA口时钟  //开启复用前必须
//	RCC->APB2ENR|=1<<14;  //使能串口时钟 
//	AFIO->MAPR|=1<<2;			//TIM2重映射
//	
//	GPIOB->CRL&=0X00FFFFFF;//IO状态设置//PA9-10
//	GPIOB->CRL|=0X8B000000;//IO状态设置//PA10 输入下拉 Rx    PA9 复用推挽输出 Tx
//	GPIOB->ODR|=1<<7;
//	GPIOB->BSRR|=1<<7;	 
//	
////	GPIOA->CRH&=0XFFFFF00F;//IO状态设置//PA9-10
////	GPIOA->CRH|=0X000008B0;//IO状态设置//PA10 输入下拉 Rx    PA9 复用推挽输出 Tx
////	GPIOA->ODR|=1<<10;
////	GPIOA->BSRR|=1<<10;	   
////	while(1);
//	RCC->APB2RSTR|=1<<14;   //复位串口1
//	RCC->APB2RSTR&=~(1<<14);//停止复位	   	   
//	//波特率设置
// 	USART1->BRR=mantissa; // 波特率设置	 
//	USART1->CR1|=0X200C;  //UE使能 1位停止,无校验位.
//	
//	USART1->CR1|=1<<4;		//IDLE中断使能
//	MY_NVIC_Init(3,0,USART1_IRQn,3);//组2，最高优先级 
//	
//	USART1->CR3 |= 3<<6;  //DMA方式

//	//DMA TX配置
//	RCC->AHBENR|=1<<0;//使能DMA1 RCC
//	
//	DMA1->IFCR |= 0xf<<12;	//清除DMA通道4的任何标记
//	DMA1_Channel4->CPAR= (u32)(&USART1->DR);//DMA外设地址寄存器
//	MY_NVIC_Init(4,0,DMA1_Channel4_IRQn,3);//组2，较低低优先级 
//	
//	//DMA RX配置
//	DMA1->IFCR |= 0xf<<16;	//清除DMA通道5的任何标记
//	DMA1_Channel5->CNDTR=USART_REC_LEN;//接收USART_REC_LEN个字符
//	DMA1_Channel5->CPAR= (u32)(&USART1->DR);//DMA外设地址寄存器
//	DMA1_Channel5->CMAR= (u32)rec_buf;	
//	
//	
//	DMA1_Channel5->CCR=0x20a3; 
//	
//	MY_NVIC_Init(4,1,DMA1_Channel5_IRQn,3);//组2，较低低优先级 
//}

void USART_CONFIG(GLOVAR_HandleTypeDef *glovar)
{  	 
//	float temp;
//	uint16_t mantissa;
//	uint16_t fraction;	   
	
//	USART1->CR1|=0X200C;  //UE使能 1位停止,无校验位.
	
	USART1->CR1|=USART_CR1_IDLEIE;		//IDLE中断使能
	USART1->CR1&=~(USART_CR1_TCIE|USART_CR1_RXNEIE);
//	MY_NVIC_Init(3,0,USART1_IRQn,3);//组2，最低优先级 
	
	USART1->CR3 |= USART_CR3_DMAR|USART_CR3_DMAT;  //DMA方式

//	DMA TX配置
//	RCC->AHBENR|=1<<0;//使能DMA1 RCC
	
	DMA2->LIFCR |= 0x3d<<16;	//清除DMA2数据流2的任何标记
//	DMA2_Channel4->CPAR= (u32)(&USART1->DR);//DMA外设地址寄存器
	DMA2_Stream2->M0AR=(uint32_t)&glovar->UART.REC_BUF;
	
	DMA2_Stream2->PAR=(uint32_t)(&USART1->DR);//DMA外设地址寄存器
	DMA2_Stream2->NDTR=USART_REC_LEN;
	DMA2_Stream2->M0AR;
	DMA2_Stream2->CR|=DMA_SxCR_TCIE|DMA_SxCR_EN;
//	MY_NVIC_Init(2,3,DMA1_Channel4_IRQn,3);//组2，较低低优先级 
	
	//DMA RX配置
//	DMA1->IFCR |= 0xf<<16;	//清除DMA通道5的任何标记
//	DMA1_Channel5->CNDTR=USART_REC_LEN;//接收USART_REC_LEN个字符
//	DMA1_Channel5->CPAR= (u32)(&USART1->DR);//DMA外设地址寄存器
//	DMA1_Channel5->CMAR= (u32)rec_buf;	
	
//	DMA1_Channel5->CCR=0x20a3; 
	
//	MY_NVIC_Init(2,2,DMA1_Channel5_IRQn,3);//组2，较低低优先级 
}




/**************************实现函数********************************************
*函数原型:		void UART1_Put_Char(unsigned char DataToSend)
*功　　能:		RS232发送一个字节
输入参数：
		unsigned char DataToSend   要发送的字节数据
输出参数：没有	
*******************************************************************************/
void UART1_Put_Char(unsigned char DataToSend)
{
	//将要发送的字节写到UART1的发送缓冲区
	 USART1->DR = (DataToSend & (uint16_t)0x01FF);
//	USART_SendData(USART1, (unsigned char) DataToSend);
	//等待发送完成
  	while (!(USART1->SR & ((uint16_t)0x0080)));
}

/**************************实现函数********************************************
*函数原型:		void UART1_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
				,int16_t alt,int16_t tempr,int16_t press)
*功　　能:		向上位机发送经过解算后的姿态数据
输入参数：
		int16_t yaw 经过解算后的航向角度。单位为0.1度 0 -> 3600  对应 0 -> 360.0度
		int16_t pitch 解算得到的俯仰角度，单位 0.1度。-900 - 900 对应 -90.0 -> 90.0 度
		int16_t roll  解算后得到的横滚角度，单位0.1度。 -1800 -> 1800 对应 -180.0  ->  180.0度
		int16_t alt   气压高度。 单位0.1米。  范围一个整型变量
		int16_t tempr 温度 。 单位0.1摄氏度   范围：直到你的电路板不能正常工作
		int16_t press 气压压力。单位10Pa  一个大气压强在101300pa 这个已经超过一个整型的范围。需要除以10再发给上位机
		int16_t IMUpersec  姿态解算速率。运算IMUpersec每秒。
输出参数：没有	
*******************************************************************************/
void UART1_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{
 	unsigned int temp=0xaF+2;
	char ctemp;
	UART1_Put_Char(0xa5);
	UART1_Put_Char(0x5a);
	UART1_Put_Char(14+2);
	UART1_Put_Char(0xA1);

	if(yaw<0)yaw=32768-yaw;
	ctemp=yaw>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=yaw;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(pitch<0)pitch=32768-pitch;
	ctemp=pitch>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=pitch;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(roll<0)roll=32768-roll;
	ctemp=roll>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=roll;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(alt<0)alt=32768-alt;
	ctemp=alt>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=alt;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(tempr<0)tempr=32768-tempr;
	ctemp=tempr>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=tempr;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	if(press<0)press=32768-press;
	ctemp=press>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=press;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	UART1_Put_Char(temp%256);
	UART1_Put_Char(0xaa);
}


