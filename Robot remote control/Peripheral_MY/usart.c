#include "sys.h"
#include "usart.h"	
#include <string.h>
//#include "banlance.h"
#include "led.h"
#include "Low_power.h"

char USART1_REC_BUF[USART_REC_LEN+1];
char uart_send_buf[USART_REC_LEN+1]="\r\n";
char uart_rec_flag;
char uart_send_ok_flag=1;

//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
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
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif 

 
//u8 BUF_RX_PID;

//串口1接收空中断 ,接收专用
void USART1_IRQn_call(void)
{
	u32 len=USART_REC_LEN-DMA1_Channel5->CNDTR;
	//这两个是用于顺序清除IDLE标记位
	USART1->SR;
	USART1->DR;
	USART1_REC_BUF[len]='\0';
	DMA1_Channel5->CCR&=~(1<<0); //失能 通道5
	DMA1_Channel5->CNDTR=USART_REC_LEN;
	DMA1_Channel5->CCR|=(1<<0);  //使能 通道5
	uart_rec_flag++;
//	printf("%s\r\n",USART1_REC_BUF);
//	CPU_SLEEP_INT_MODE(DISABLE);
//	rec_data_filter_all (USART1_REC_BUF,0,len);
//	PID_change_flag=1;
}
//串口1DMA发送完毕中断
void DMA1_Channel4_FOR_USART1_IRQn_call(void)
{
	  /* Calculate the used DMAy */
		DMA1->IFCR = 2<<12; //DMA1_FLAG_TC4清除
		DMA1_Channel4->CCR&=~(1<<0);
		uart_send_ok_flag=1;
}

//串口1DMA接收中断
void DMA1_Channel5_FOR_USART1_IRQn_call(void)
{
		DMA1->IFCR = 2<<16; //DMA1_FLAG_TC4清除
		//DMA1_Channel4->CCR&=~(1<<0);
}
//void uart_init_PB6_PB7(u32 pclk2,u32 bound,char *rec_buf)
//{  	 
//	float temp;
//	u16 mantissa;
//	u16 fraction;	   
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
//	MY_NVIC_Init(3,0,USART1_IRQn,2);//组2，最低优先级 
//	
//	USART1->CR3 |= 3<<6;  //DMA方式

//	//DMA TX配置
//	RCC->AHBENR|=1<<0;//使能DMA1 RCC
//	
//	DMA1->IFCR |= 0xf<<12;	//清除DMA通道4的任何标记
//	DMA1_Channel4->CPAR= (u32)(&USART1->DR);//DMA外设地址寄存器
//	MY_NVIC_Init(2,3,DMA1_Channel4_IRQn,2);//组2，较低低优先级 
//	
//	//DMA RX配置
//	DMA1->IFCR |= 0xf<<16;	//清除DMA通道5的任何标记
//	DMA1_Channel5->CNDTR=USART_REC_LEN;//接收USART_REC_LEN个字符
//	DMA1_Channel5->CPAR= (u32)(&USART1->DR);//DMA外设地址寄存器
//	DMA1_Channel5->CMAR= (u32)rec_buf;	
//	
//	DMA1_Channel5->CCR=0x20a3; 
//	
//	MY_NVIC_Init(2,2,DMA1_Channel5_IRQn,2);//组2，较低低优先级 
//}

//pclk2 :主频
//bound :波特率
//rec_buf:接收包
//void uart_init(u32 pclk2,u32 bound,char *rec_buf)
//{  	 
//	float temp;
//	u16 mantissa;
//	u16 fraction;	   
//	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
//	mantissa=temp;				 //得到整数部分
//	fraction=(temp-mantissa)*16; //得到小数部分	 
//    mantissa<<=4;
//	mantissa+=fraction; 
//	RCC->APB2ENR|=1<<2;   //使能PORTA口时钟  
//	RCC->APB2ENR|=1<<14;  //使能串口时钟 
//	
//	GPIOA->CRH&=0XFFFFF00F;//IO状态设置//PA9-10
//	GPIOA->CRH|=0X000008B0;//IO状态设置//PA10 输入下拉 Rx    PA9 复用推挽输出 Tx
////	GPIOA->ODR|=1<<10;
//	GPIOA->BSRR|=1<<10;	   
////	while(1);
//	RCC->APB2RSTR|=1<<14;   //复位串口1
//	RCC->APB2RSTR&=~(1<<14);//停止复位	   	   
//	//波特率设置
// 	USART1->BRR=mantissa; // 波特率设置	 
//	USART1->CR1|=0X200C;  //UE使能 1位停止,无校验位.
//	
//	USART1->CR1|=1<<4;		//IDLE中断使能
//	MY_NVIC_Init(3,0,USART1_IRQn,2);//组2，最低优先级 
//	
//	USART1->CR3 |= 3<<6;  //DMA方式

//	//DMA TX配置
//	RCC->AHBENR|=1<<0;//使能DMA1 RCC
//	
//	DMA1->IFCR |= 0xf<<12;	//清除DMA通道4的任何标记
//	DMA1_Channel4->CPAR= (u32)(&USART1->DR);//DMA外设地址寄存器
//	MY_NVIC_Init(2,3,DMA1_Channel4_IRQn,2);//组2，较低低优先级 
//	
//	//DMA RX配置
//	DMA1->IFCR |= 0xf<<16;	//清除DMA通道5的任何标记
//	DMA1_Channel5->CNDTR=USART_REC_LEN;//接收USART_REC_LEN个字符
//	DMA1_Channel5->CPAR= (u32)(&USART1->DR);//DMA外设地址寄存器
//	DMA1_Channel5->CMAR= (u32)rec_buf;	
//	
//	DMA1_Channel5->CCR=0x20a3; 
//	
//	MY_NVIC_Init(2,2,DMA1_Channel5_IRQn,2);//组2，较低低优先级 
//}

void USART1_Send_Buf(char *buf , unsigned int len)
{
		//因为这句之前有使用中断标记位判断,所以可以不用while语句,还可以不用中断
		//while(DMA1_Channel4->CNDTR);
		DMA1_Channel4->CCR=0;//必须先清除才能写寄存器
		DMA1->IFCR |= 0xf<<12;	//清除DMA通道4的任何标记
		DMA1_Channel4->CNDTR=len;
		DMA1_Channel4->CMAR= (u32)buf;	
		DMA1_Channel4->CCR=0x2093; //0b10 0000 1001 0011  通道优先级高 外设数据宽度 =内存宽度=8位  TCIE中断使能  
}

void UsartSend(unsigned char DataToSend)
{
	//将要发送的字节写到UART1的发送缓冲区
	 USART1->DR = (DataToSend & (uint16_t)0x01FF);
//	USART_SendData(USART1, (unsigned char) DataToSend);
	//等待发送完成
  	while (!(USART1->SR & ((uint16_t)0x0080)));
}




//以字符的格式输出
void Print(uint8_t num)
{
	uint8_t  bai,shi,ge;
	bai=num/100;
	shi=num%100/10;
	ge=num%10;
	UsartSend('0'+bai);
	UsartSend('0'+shi);
	UsartSend('0'+ge);
}
//以字符的形式输出INT型数据
void PrintInt(uint16_t num)
{
	 uint8_t w5,w4,w3,w2,w1;
	 w5=num/10000;
	 w4=num%10000/1000;
	 w3=num%1000/100;
	 w2=num%100/10;
	 w1=num%10;
	 UsartSend('0'+w5);
	 UsartSend('0'+w4);
	 UsartSend('0'+w3);
	 UsartSend('0'+w2);
	 UsartSend('0'+w1);
}
//输出字符串
void PrintChar(char *s)
{
	u16 len=strlen(s);
	USART1_Send_Buf(s,len);
}
void PrintHexInt16(int16_t num)
{
	UsartSend((num & 0xff00) >> 8);//先发送高８位，再发送低８位
	UsartSend((uint8_t)(num & 0x00ff));
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


