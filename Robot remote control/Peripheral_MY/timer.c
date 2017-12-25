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
	if(TIM1->SR&0X0001)//溢出中断
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
	TIM1->SR&=~(1<<0);//清除中断标志位 	
}

void Timer1_Init(u16 arr,u16 psc){
	RCC->APB2ENR|=1<<11;//TIM2时钟使能    
 	TIM1->ARR=arr;  //设定计数器自动重装值//刚好1ms    
	TIM1->PSC=psc;  //预分频器7200,得到10Khz的计数时钟
	//这两个东东要同时设置才可以使用中断
	TIM1->DIER|=1<<0;   //允许更新中断				
	TIM1->DIER|=1<<6;   //允许触发中断
		  							    
	TIM1->CR1|=0x01;    //使能定时器1
  MY_NVIC_Init(0,0,TIM1_UP_IRQn,3);//抢占1，子优先级3，组2								 
}
//void Timerx_Init(u16 arr,u16 psc){
//	RCC->APB1ENR|=1<<0;//TIM2时钟使能    
// 	TIM2->ARR=arr;  //设定计数器自动重装值//刚好1ms    
//	TIM2->PSC=psc;  //预分频器7200,得到10Khz的计数时钟
//	//这两个东东要同时设置才可以使用中断
//	TIM2->DIER|=1<<0;   //允许更新中断				
//	TIM2->DIER|=1<<6;   //允许触发中断
//		  							    
//	TIM2->CR1|=0x01;    //使能定时器3
//  MY_NVIC_Init(1,0,TIM2_IRQn,3);//抢占1，子优先级3，组2								 
//}
void Tim3Init(void){
		RCC->APB1ENR|=1<<1;       //TIM3时钟使能 
	  TIM3->ARR = 20000; 
	  TIM3->PSC=71;
	  
	  GPIOA->CRL&=0xF0FFFFFF;	//PA6输出
	  GPIOA->CRL|=0x0B000000;	//复用功能输出
 
	  TIM3->CCMR1|=6<<4;  //CH1 PWM1模式不到 装载值时输出高电平 
		TIM3->CCR1 =500;
	  TIM3->CCMR1|=1<<3; 	//CH1预装载使能
	  
	  TIM3->CCER|= 1<<0;  //OC1 输出使能
		
		GPIOA->CRL&=0X0FFFFFFF;//PA7输出
		GPIOA->CRL|=0XB0000000;//复用功能输出 
		TIM3->CCMR1|=6<<12;  //CH2 PWM1模式	 6 (110	)PWM1模式1  
		TIM3->CCR2 =1500;
		TIM3->CCMR1|=1<<11; //CH2预装载使能	   
		TIM3->CCER|=1<<4;   //OC2 输出使能
	
		TIM3->CR1=0x8000;   //ARPE使能 
		TIM3->CR1|=0x01;    //使能定时器3 
}

float GET_NOWTIME(void){//返回当前systick计数器值,32位

	float temp=0 ;
	static uint32_t now=0; // 采样周期计数 单位 us

 	now = TIM4->CNT;//读高16位时间
   	TIM4->CNT=0;
	temp = (float)now / 2000000.0f;          //换算成ms，再除以2得出采样周期的一半

	return temp;
}	

/**************************实现函数********************************************
*函数原型:		
*功　　能:		
*******************************************************************************/
//void TIM4_SYSTICK_INIT(void)
//{
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //时钟使能

//	TIM_TimeBaseStructure.TIM_Period = 0xffff; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
//	TIM_TimeBaseStructure.TIM_Prescaler =71; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
//	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
//	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

//	TIM_Cmd(TIM4, ENABLE);
//}
void Tim4_Init(u16 arr,u16 psc){
	RCC->APB1ENR|=1<<2;//TIM2时钟使能    
 	TIM4->ARR=arr;  //设定计数器自动重装值//刚好1ms    
	TIM4->PSC=psc;  //预分频器7200,得到10Khz的计数时钟
	//这两个东东要同时设置才可以使用中断
//	TIM4->DIER|=1<<0;   //允许更新中断				
//	TIM4->DIER|=1<<6;   //允许触发中断
		  							    
	TIM4->CR1|=0x01;    //使能定时器3
//  MY_NVIC_Init(1,0,TIM2_IRQn,2);//抢占1，子优先级3，组2								 
}





