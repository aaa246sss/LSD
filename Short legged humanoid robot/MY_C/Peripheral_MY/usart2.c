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
	//��������ʱ�ӻ�׼ԭ��ͨ,��˲����ʼ��㷽ʽ����һ�� ���ڳ���Ĭ��APB1��2Ԥ��Ƶ,APB2����Ƶ,�����2����ϵ  
	temp=(float)(pclk2*1000000)/(bound*2*16);//�õ�USARTDIV ��ɵ�2��2����ϵ
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������	 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //ʹ��PORTA��ʱ��  
	RCC->APB1ENR|=1<<17;  //ʹ�ܴ���ʱ�� 
	
	GPIOA->CRL&=0XFFFF00FF;//IO״̬����//PA2-3
	GPIOA->CRL|=0X00008B00;//IO״̬����//PA2����������� Tx   PA3�������� Rx 
	GPIOA->BSRR|=1<<3;
//	GPIOA->CRL|=0X00003300;
//	GPIOA->ODR|=1<<10;
//	GPIOA->BSRR|=1<<3;
//	GPIOA->BRR|=1<<3;	
//	GPIOA->BSRR|=1<<2;	
//	GPIOA->BRR|=1<<2;		
//	while(1);
	RCC->APB1RSTR|=1<<17;   //��λ����1
	RCC->APB1RSTR&=~(1<<17);//ֹͣ��λ	   	   
	//����������
 	USART2->BRR=mantissa; // ����������	 
	USART2->CR1|=0X200C;  //UEʹ�� 1λֹͣ,��У��λ.
	
	USART2->CR1|=1<<4;		//IDLE�ж�ʹ��
	MY_NVIC_Init(3,3,USART2_IRQn,2);//��2��������ȼ� 
	
	USART2->CR3 |= 3<<6;  //DMA��ʽ

//DMA1ͨ��ʹ��
	RCC->AHBENR|=1<<0;//ʹ��DMA1 RCC
//DMA TX����	
	DMA1->IFCR |= 0xf<<24;	//���DMAͨ��4���κα��
	DMA1_Channel7->CPAR= (u32)(&USART2->DR);//DMA�����ַ�Ĵ���
	MY_NVIC_Init(3,2,DMA1_Channel7_IRQn,2);//��2���ϵ͵����ȼ� 
	
	//DMA RX����
	DMA1->IFCR |= 0xf<<20;	//���DMAͨ��5���κα��
	DMA1_Channel6->CNDTR=USART2_REC_LEN;//����USART2_REC_LEN���ַ�
	DMA1_Channel6->CPAR= (u32)(&USART2->DR);//DMA�����ַ�Ĵ���
	DMA1_Channel6->CMAR= (u32)rec_buf;	
	
	DMA1_Channel6->CCR=0x20a3; 
	
	MY_NVIC_Init(3,1,DMA1_Channel6_IRQn,2);//��2���ϵ͵����ȼ� 
}

void USART2_Send_Buf(char *buf , unsigned int len)
{
		DMA1_Channel7->CCR=0;//��������д�Ĵ���,�ֹ�ֹͣ
		DMA1->IFCR |= 0xf<<24;	//���DMAͨ��7���κα��
		DMA1_Channel7->CNDTR=len;
		DMA1_Channel7->CMAR= (u32)buf;	
		DMA1_Channel7->CCR=0x0093; //0b00 0000 1011 0010  ͨ�����ȼ��� �������ݿ�� =�ڴ���=8λ  TCIE�ж�ʹ��  
}

//void Usart2Send(unsigned char DataToSend)
//{
//	//��Ҫ���͵��ֽ�д��UART2�ķ��ͻ�����
//	 USART2->DR = (DataToSend & (uint16_t)0x01FF);
////	USART_SendData(USART1, (unsigned char) DataToSend);
//	//�ȴ��������
//  	while (!(USART2->SR & ((uint16_t)0x0080)));
//}

////����ַ���
//void PrintChar(char *s)
//{
//	u16 len=strlen(s);
//	USART1_Send_Buf(s,len);
//}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART1_Put_Char(unsigned char DataToSend)
*��������:		RS232����һ���ֽ�
���������
		unsigned char DataToSend   Ҫ���͵��ֽ�����
���������û��	
*******************************************************************************/
//void UART2_Put_Char(unsigned char DataToSend)
//{
//	//��Ҫ���͵��ֽ�д��UART1�ķ��ͻ�����
//	 USART2->DR = (DataToSend & (uint16_t)0x01FF);
////	USART_SendData(USART1, (unsigned char) DataToSend);
//	//�ȴ��������
//  	while (!(USART2->SR & ((uint16_t)0x0080)));
//}




