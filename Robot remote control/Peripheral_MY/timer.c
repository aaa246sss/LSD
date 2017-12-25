#include "timer.h"
#include "led.h"
#include "mpu6050.h"
#include "usart.h"
#include "math.h"
#include "NRF24L01.h"


char MPU6050_read_flag;
u16  NRF_IRQ_OUT_TIME;
u16   NRF_Sync;
u8   RNF_send_flag;
extern u8 NRF_resend_num;
float filter_MY(float temp){
	static float record;
	static char flag=0;
	static float last;
	//last[1]=last[0];
	//last[0]
//	float re;
	if(fabs(temp-last)>9000)
	{
			if(flag)
			{
				last=temp;
				flag=0;
				return temp;
			}
			else
			{
				//last=temp;
				record=last;
				last=temp;
				flag=1;
				return record;
			}
	}
	else
	{
		last=temp;
		if(flag)
		{
			
			flag++;
			if(flag==3)
			{
				flag=0;
				return temp;
			}
				
			return record;
		}
		else
		{
			return temp;
		}
	}
}

void TIM1_UP_IRQHandler(void){ 
//	static 	
	if(TIM1->SR&0X0001)//����ж�
	{
			NRF_Sync++;
			if(NRF_Sync>200){
				NRF_Sync=0;
				if(NRF_resend_num)NRF_resend_num--;
			}
			if((NRF_Sync==SYNC_DELAY)&&(!NRF_resend_num))	
				RNF_send_flag=1;
			if((NRF_Sync==MPU_6050_read_timer1)||(NRF_Sync==MPU_6050_read_timer2)||(NRF_Sync==MPU_6050_read_timer3)||(NRF_Sync==MPU_6050_read_timer4))
//			if((NRF_Sync==MPU_6050_read_timer1)||(NRF_Sync==MPU_6050_read_timer2)||(NRF_Sync==MPU_6050_read_timer3))
				MPU6050_read_flag=1;
			if(NRF_IRQ_OUT_TIME)
				NRF_IRQ_OUT_TIME--;
//			if(idle)
//				idle--;	
	}				   
	TIM1->SR&=~(1<<0);//����жϱ�־λ 	
}

void Timer1_Init(u16 arr,u16 psc){
	RCC->APB2ENR|=1<<11;//TIM2ʱ��ʹ��    
 	TIM1->ARR=arr;  //�趨�������Զ���װֵ//�պ�1ms    
	TIM1->PSC=psc;  //Ԥ��Ƶ��7200,�õ�10Khz�ļ���ʱ��
	//����������Ҫͬʱ���òſ���ʹ���ж�
	TIM1->DIER|=1<<0;   //��������ж�				
	TIM1->DIER|=1<<6;   //�������ж�
		  							    
	TIM1->CR1|=0x01;    //ʹ�ܶ�ʱ��1
  MY_NVIC_Init(0,0,TIM1_UP_IRQn,3);//��ռ1�������ȼ�3����2								 
}
//void Timerx_Init(u16 arr,u16 psc){
//	RCC->APB1ENR|=1<<0;//TIM2ʱ��ʹ��    
// 	TIM2->ARR=arr;  //�趨�������Զ���װֵ//�պ�1ms    
//	TIM2->PSC=psc;  //Ԥ��Ƶ��7200,�õ�10Khz�ļ���ʱ��
//	//����������Ҫͬʱ���òſ���ʹ���ж�
//	TIM2->DIER|=1<<0;   //��������ж�				
//	TIM2->DIER|=1<<6;   //�������ж�
//		  							    
//	TIM2->CR1|=0x01;    //ʹ�ܶ�ʱ��3
//  MY_NVIC_Init(1,0,TIM2_IRQn,3);//��ռ1�������ȼ�3����2								 
//}
void Tim3Init(void){
		RCC->APB1ENR|=1<<1;       //TIM3ʱ��ʹ�� 
	  TIM3->ARR = 20000; 
	  TIM3->PSC=71;
	  
	  GPIOA->CRL&=0xF0FFFFFF;	//PA6���
	  GPIOA->CRL|=0x0B000000;	//���ù������
 
	  TIM3->CCMR1|=6<<4;  //CH1 PWM1ģʽ���� װ��ֵʱ����ߵ�ƽ 
		TIM3->CCR1 =500;
	  TIM3->CCMR1|=1<<3; 	//CH1Ԥװ��ʹ��
	  
	  TIM3->CCER|= 1<<0;  //OC1 ���ʹ��
		
		GPIOA->CRL&=0X0FFFFFFF;//PA7���
		GPIOA->CRL|=0XB0000000;//���ù������ 
		TIM3->CCMR1|=6<<12;  //CH2 PWM1ģʽ	 6 (110	)PWM1ģʽ1  
		TIM3->CCR2 =1500;
		TIM3->CCMR1|=1<<11; //CH2Ԥװ��ʹ��	   
		TIM3->CCER|=1<<4;   //OC2 ���ʹ��
	
		TIM3->CR1=0x8000;   //ARPEʹ�� 
		TIM3->CR1|=0x01;    //ʹ�ܶ�ʱ��3 
}

float GET_NOWTIME(void){//���ص�ǰsystick������ֵ,32λ

	float temp=0 ;
	static uint32_t now=0; // �������ڼ��� ��λ us

 	now = TIM4->CNT;//����16λʱ��
   	TIM4->CNT=0;
	temp = (float)now / 2000000.0f;          //�����ms���ٳ���2�ó��������ڵ�һ��

	return temp;
}	

/**************************ʵ�ֺ���********************************************
*����ԭ��:		
*��������:		
*******************************************************************************/
//void TIM4_SYSTICK_INIT(void)
//{
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //ʱ��ʹ��

//	TIM_TimeBaseStructure.TIM_Period = 0xffff; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
//	TIM_TimeBaseStructure.TIM_Prescaler =71; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
//	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
//	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

//	TIM_Cmd(TIM4, ENABLE);
//}
void Tim4_Init(u16 arr,u16 psc){
	RCC->APB1ENR|=1<<2;//TIM2ʱ��ʹ��    
 	TIM4->ARR=arr;  //�趨�������Զ���װֵ//�պ�1ms    
	TIM4->PSC=psc;  //Ԥ��Ƶ��7200,�õ�10Khz�ļ���ʱ��
	//����������Ҫͬʱ���òſ���ʹ���ж�
//	TIM4->DIER|=1<<0;   //��������ж�				
//	TIM4->DIER|=1<<6;   //�������ж�
		  							    
	TIM4->CR1|=0x01;    //ʹ�ܶ�ʱ��3
//  MY_NVIC_Init(1,0,TIM2_IRQn,2);//��ռ1�������ȼ�3����2								 
}





