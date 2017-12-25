//#include "sys.h"

#include "usart.h"	
#include <string.h>
#include "MAIN_MY.h"
//#include "banlance.h"
//#include "led.h"

//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#ifdef KEIL
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���     
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef�� d in stdio.h. */ 
FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
//void _sys_exit(int x) 
//{ 
//	x = x; 
//} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART2->SR&0X40)==0);//ѭ������,ֱ���������   
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
    while((USART2->SR&0X40)==0);//ѭ������,ֱ���������   
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
//	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV
//	mantissa=temp;				 //�õ���������
//	fraction=(temp-mantissa)*16; //�õ�С������	 
//    mantissa<<=4;
//	mantissa+=fraction; 
//	RCC->APB2ENR|=1<<0;   //ʹ��afio��ʱ��  //��������ǰ����
//	RCC->APB2ENR|=1<<2;   //ʹ��PORTA��ʱ��  //��������ǰ����
//	RCC->APB2ENR|=1<<3;   //ʹ��PORTA��ʱ��  //��������ǰ����
//	RCC->APB2ENR|=1<<14;  //ʹ�ܴ���ʱ�� 
//	AFIO->MAPR|=1<<2;			//TIM2��ӳ��
//	
//	GPIOB->CRL&=0X00FFFFFF;//IO״̬����//PA9-10
//	GPIOB->CRL|=0X8B000000;//IO״̬����//PA10 �������� Rx    PA9 ����������� Tx
//	GPIOB->ODR|=1<<7;
//	GPIOB->BSRR|=1<<7;	 
//	
////	GPIOA->CRH&=0XFFFFF00F;//IO״̬����//PA9-10
////	GPIOA->CRH|=0X000008B0;//IO״̬����//PA10 �������� Rx    PA9 ����������� Tx
////	GPIOA->ODR|=1<<10;
////	GPIOA->BSRR|=1<<10;	   
////	while(1);
//	RCC->APB2RSTR|=1<<14;   //��λ����1
//	RCC->APB2RSTR&=~(1<<14);//ֹͣ��λ	   	   
//	//����������
// 	USART1->BRR=mantissa; // ����������	 
//	USART1->CR1|=0X200C;  //UEʹ�� 1λֹͣ,��У��λ.
//	
//	USART1->CR1|=1<<4;		//IDLE�ж�ʹ��
//	MY_NVIC_Init(3,0,USART1_IRQn,3);//��2��������ȼ� 
//	
//	USART1->CR3 |= 3<<6;  //DMA��ʽ

//	//DMA TX����
//	RCC->AHBENR|=1<<0;//ʹ��DMA1 RCC
//	
//	DMA1->IFCR |= 0xf<<12;	//���DMAͨ��4���κα��
//	DMA1_Channel4->CPAR= (u32)(&USART1->DR);//DMA�����ַ�Ĵ���
//	MY_NVIC_Init(4,0,DMA1_Channel4_IRQn,3);//��2���ϵ͵����ȼ� 
//	
//	//DMA RX����
//	DMA1->IFCR |= 0xf<<16;	//���DMAͨ��5���κα��
//	DMA1_Channel5->CNDTR=USART_REC_LEN;//����USART_REC_LEN���ַ�
//	DMA1_Channel5->CPAR= (u32)(&USART1->DR);//DMA�����ַ�Ĵ���
//	DMA1_Channel5->CMAR= (u32)rec_buf;	
//	
//	
//	DMA1_Channel5->CCR=0x20a3; 
//	
//	MY_NVIC_Init(4,1,DMA1_Channel5_IRQn,3);//��2���ϵ͵����ȼ� 
//}

void USART_CONFIG(GLOVAR_HandleTypeDef *glovar)
{  	 
//	float temp;
//	uint16_t mantissa;
//	uint16_t fraction;	   
	
//	USART1->CR1|=0X200C;  //UEʹ�� 1λֹͣ,��У��λ.
	
	USART1->CR1|=USART_CR1_IDLEIE;		//IDLE�ж�ʹ��
	USART1->CR1&=~(USART_CR1_TCIE|USART_CR1_RXNEIE);
//	MY_NVIC_Init(3,0,USART1_IRQn,3);//��2��������ȼ� 
	
	USART1->CR3 |= USART_CR3_DMAR|USART_CR3_DMAT;  //DMA��ʽ

//	DMA TX����
//	RCC->AHBENR|=1<<0;//ʹ��DMA1 RCC
	
	DMA2->LIFCR |= 0x3d<<16;	//���DMA2������2���κα��
//	DMA2_Channel4->CPAR= (u32)(&USART1->DR);//DMA�����ַ�Ĵ���
	DMA2_Stream2->M0AR=(uint32_t)&glovar->UART.REC_BUF;
	
	DMA2_Stream2->PAR=(uint32_t)(&USART1->DR);//DMA�����ַ�Ĵ���
	DMA2_Stream2->NDTR=USART_REC_LEN;
	DMA2_Stream2->M0AR;
	DMA2_Stream2->CR|=DMA_SxCR_TCIE|DMA_SxCR_EN;
//	MY_NVIC_Init(2,3,DMA1_Channel4_IRQn,3);//��2���ϵ͵����ȼ� 
	
	//DMA RX����
//	DMA1->IFCR |= 0xf<<16;	//���DMAͨ��5���κα��
//	DMA1_Channel5->CNDTR=USART_REC_LEN;//����USART_REC_LEN���ַ�
//	DMA1_Channel5->CPAR= (u32)(&USART1->DR);//DMA�����ַ�Ĵ���
//	DMA1_Channel5->CMAR= (u32)rec_buf;	
	
//	DMA1_Channel5->CCR=0x20a3; 
	
//	MY_NVIC_Init(2,2,DMA1_Channel5_IRQn,3);//��2���ϵ͵����ȼ� 
}




/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART1_Put_Char(unsigned char DataToSend)
*��������:		RS232����һ���ֽ�
���������
		unsigned char DataToSend   Ҫ���͵��ֽ�����
���������û��	
*******************************************************************************/
void UART1_Put_Char(unsigned char DataToSend)
{
	//��Ҫ���͵��ֽ�д��UART1�ķ��ͻ�����
	 USART1->DR = (DataToSend & (uint16_t)0x01FF);
//	USART_SendData(USART1, (unsigned char) DataToSend);
	//�ȴ��������
  	while (!(USART1->SR & ((uint16_t)0x0080)));
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART1_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
				,int16_t alt,int16_t tempr,int16_t press)
*��������:		����λ�����;�����������̬����
���������
		int16_t yaw ���������ĺ���Ƕȡ���λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
		int16_t pitch ����õ��ĸ����Ƕȣ���λ 0.1�ȡ�-900 - 900 ��Ӧ -90.0 -> 90.0 ��
		int16_t roll  �����õ��ĺ���Ƕȣ���λ0.1�ȡ� -1800 -> 1800 ��Ӧ -180.0  ->  180.0��
		int16_t alt   ��ѹ�߶ȡ� ��λ0.1�ס�  ��Χһ�����ͱ���
		int16_t tempr �¶� �� ��λ0.1���϶�   ��Χ��ֱ����ĵ�·�岻����������
		int16_t press ��ѹѹ������λ10Pa  һ������ѹǿ��101300pa ����Ѿ�����һ�����͵ķ�Χ����Ҫ����10�ٷ�����λ��
		int16_t IMUpersec  ��̬�������ʡ�����IMUpersecÿ�롣
���������û��	
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


