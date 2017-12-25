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

//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
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
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif 

 
//u8 BUF_RX_PID;

//����1���տ��ж� ,����ר��
void USART1_IRQn_call(void)
{
	u32 len=USART_REC_LEN-DMA1_Channel5->CNDTR;
	//������������˳�����IDLE���λ
	USART1->SR;
	USART1->DR;
	USART1_REC_BUF[len]='\0';
	DMA1_Channel5->CCR&=~(1<<0); //ʧ�� ͨ��5
	DMA1_Channel5->CNDTR=USART_REC_LEN;
	DMA1_Channel5->CCR|=(1<<0);  //ʹ�� ͨ��5
	uart_rec_flag++;
//	printf("%s\r\n",USART1_REC_BUF);
//	CPU_SLEEP_INT_MODE(DISABLE);
//	rec_data_filter_all (USART1_REC_BUF,0,len);
//	PID_change_flag=1;
}
//����1DMA��������ж�
void DMA1_Channel4_FOR_USART1_IRQn_call(void)
{
	  /* Calculate the used DMAy */
		DMA1->IFCR = 2<<12; //DMA1_FLAG_TC4���
		DMA1_Channel4->CCR&=~(1<<0);
		uart_send_ok_flag=1;
}

//����1DMA�����ж�
void DMA1_Channel5_FOR_USART1_IRQn_call(void)
{
		DMA1->IFCR = 2<<16; //DMA1_FLAG_TC4���
		//DMA1_Channel4->CCR&=~(1<<0);
}
//void uart_init_PB6_PB7(u32 pclk2,u32 bound,char *rec_buf)
//{  	 
//	float temp;
//	u16 mantissa;
//	u16 fraction;	   
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
//	MY_NVIC_Init(3,0,USART1_IRQn,2);//��2��������ȼ� 
//	
//	USART1->CR3 |= 3<<6;  //DMA��ʽ

//	//DMA TX����
//	RCC->AHBENR|=1<<0;//ʹ��DMA1 RCC
//	
//	DMA1->IFCR |= 0xf<<12;	//���DMAͨ��4���κα��
//	DMA1_Channel4->CPAR= (u32)(&USART1->DR);//DMA�����ַ�Ĵ���
//	MY_NVIC_Init(2,3,DMA1_Channel4_IRQn,2);//��2���ϵ͵����ȼ� 
//	
//	//DMA RX����
//	DMA1->IFCR |= 0xf<<16;	//���DMAͨ��5���κα��
//	DMA1_Channel5->CNDTR=USART_REC_LEN;//����USART_REC_LEN���ַ�
//	DMA1_Channel5->CPAR= (u32)(&USART1->DR);//DMA�����ַ�Ĵ���
//	DMA1_Channel5->CMAR= (u32)rec_buf;	
//	
//	DMA1_Channel5->CCR=0x20a3; 
//	
//	MY_NVIC_Init(2,2,DMA1_Channel5_IRQn,2);//��2���ϵ͵����ȼ� 
//}

//pclk2 :��Ƶ
//bound :������
//rec_buf:���հ�
//void uart_init(u32 pclk2,u32 bound,char *rec_buf)
//{  	 
//	float temp;
//	u16 mantissa;
//	u16 fraction;	   
//	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV
//	mantissa=temp;				 //�õ���������
//	fraction=(temp-mantissa)*16; //�õ�С������	 
//    mantissa<<=4;
//	mantissa+=fraction; 
//	RCC->APB2ENR|=1<<2;   //ʹ��PORTA��ʱ��  
//	RCC->APB2ENR|=1<<14;  //ʹ�ܴ���ʱ�� 
//	
//	GPIOA->CRH&=0XFFFFF00F;//IO״̬����//PA9-10
//	GPIOA->CRH|=0X000008B0;//IO״̬����//PA10 �������� Rx    PA9 ����������� Tx
////	GPIOA->ODR|=1<<10;
//	GPIOA->BSRR|=1<<10;	   
////	while(1);
//	RCC->APB2RSTR|=1<<14;   //��λ����1
//	RCC->APB2RSTR&=~(1<<14);//ֹͣ��λ	   	   
//	//����������
// 	USART1->BRR=mantissa; // ����������	 
//	USART1->CR1|=0X200C;  //UEʹ�� 1λֹͣ,��У��λ.
//	
//	USART1->CR1|=1<<4;		//IDLE�ж�ʹ��
//	MY_NVIC_Init(3,0,USART1_IRQn,2);//��2��������ȼ� 
//	
//	USART1->CR3 |= 3<<6;  //DMA��ʽ

//	//DMA TX����
//	RCC->AHBENR|=1<<0;//ʹ��DMA1 RCC
//	
//	DMA1->IFCR |= 0xf<<12;	//���DMAͨ��4���κα��
//	DMA1_Channel4->CPAR= (u32)(&USART1->DR);//DMA�����ַ�Ĵ���
//	MY_NVIC_Init(2,3,DMA1_Channel4_IRQn,2);//��2���ϵ͵����ȼ� 
//	
//	//DMA RX����
//	DMA1->IFCR |= 0xf<<16;	//���DMAͨ��5���κα��
//	DMA1_Channel5->CNDTR=USART_REC_LEN;//����USART_REC_LEN���ַ�
//	DMA1_Channel5->CPAR= (u32)(&USART1->DR);//DMA�����ַ�Ĵ���
//	DMA1_Channel5->CMAR= (u32)rec_buf;	
//	
//	DMA1_Channel5->CCR=0x20a3; 
//	
//	MY_NVIC_Init(2,2,DMA1_Channel5_IRQn,2);//��2���ϵ͵����ȼ� 
//}

void USART1_Send_Buf(char *buf , unsigned int len)
{
		//��Ϊ���֮ǰ��ʹ���жϱ��λ�ж�,���Կ��Բ���while���,�����Բ����ж�
		//while(DMA1_Channel4->CNDTR);
		DMA1_Channel4->CCR=0;//�������������д�Ĵ���
		DMA1->IFCR |= 0xf<<12;	//���DMAͨ��4���κα��
		DMA1_Channel4->CNDTR=len;
		DMA1_Channel4->CMAR= (u32)buf;	
		DMA1_Channel4->CCR=0x2093; //0b10 0000 1001 0011  ͨ�����ȼ��� �������ݿ�� =�ڴ���=8λ  TCIE�ж�ʹ��  
}

void UsartSend(unsigned char DataToSend)
{
	//��Ҫ���͵��ֽ�д��UART1�ķ��ͻ�����
	 USART1->DR = (DataToSend & (uint16_t)0x01FF);
//	USART_SendData(USART1, (unsigned char) DataToSend);
	//�ȴ��������
  	while (!(USART1->SR & ((uint16_t)0x0080)));
}




//���ַ��ĸ�ʽ���
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
//���ַ�����ʽ���INT������
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
//����ַ���
void PrintChar(char *s)
{
	u16 len=strlen(s);
	USART1_Send_Buf(s,len);
}
void PrintHexInt16(int16_t num)
{
	UsartSend((num & 0xff00) >> 8);//�ȷ��͸ߣ�λ���ٷ��͵ͣ�λ
	UsartSend((uint8_t)(num & 0x00ff));
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


