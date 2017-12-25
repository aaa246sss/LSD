#include "timer.h"
#include "led.h"
//#include "mpu6050.h"
//#include "banlance.h"
#include "usart.h"
#include "exti.h"
#include <math.h>
#include "outputdata.h"//示波器

//#define DIE_NUM 100
#define DIE_NUM 0

//定时器2中断服务程序	 
extern int pwm_ll,pwm_rr;
extern u8 recon_flag;
//extern u8 rx_flag;
char tim_flag;
u16  tim_num;
u16  tim_action_num=0;
u16  NRF_IRQ_OUT_TIME;
u16  action_timer_cul=0;
u8   NRF_Sync;
u8   UART_send_timer_Flag;

//u8 rec_err[6];

u8   RNF_send_flag;
//extern u8 re_send_flag;
//u8 re_send_ch;

//u16 timer_cul;
/*
//void nrf_re_send_ch_write(u8 channel){
//	re_send_ch=channel;
//}
//u8 nrf_re_send_ch_read(void){
//	return re_send_ch;
//}
//void nrf_re_send_ch_write(u8 channel){
//	re_send_ch=channel;
//}
//u8 nrf_re_send_ch_read(void){
//	return re_send_ch;
//}

void PWM_UP_FLAG_off(void){
	tim_flag=0;
}

u8 GET_PWM_UP_FLAG(void){
	return tim_flag;
}*/
void TIM1_UP_IRQHandler(void){ 	
	if(TIM1->SR&0X0001){//溢出中断
//		timer_cul++;
		NRF_Sync++;
		if(NRF_Sync>200){//z
			if(!recon_flag)
				recon_flag=10;
			else
				recon_flag=0;
			if(action_timer_cul)
				action_timer_cul--;
			NRF_Sync=0;
		}
		else if(NRF_Sync==160)
			UART_send_timer_Flag=1;
		if(NRF_IRQ_OUT_TIME)
			NRF_IRQ_OUT_TIME--;
	}				   
	TIM1->SR&=~(1<<0);//清除中断标志位 	
}
void Timer2_Init(void){
	RCC->APB1ENR|=1<<0;//TIM2时钟使能    
// 	TIM2->ARR=arr;  //设定计数器自动重装值//刚好1ms    
	TIM2->PSC=7199;  //预分频器7200,得到10Khz的计数时钟
	//这两个东东要同时设置才可以使用中断
	TIM2->DIER|=1<<0;   //允许更新中断				
	TIM2->DIER|=1<<6;   //允许触发中断
		  							    
//	TIM2->CR1|=0x01;    //使能定时器1
  MY_NVIC_Init(2,0,TIM2_IRQn,3);//抢占1，子优先级3，组2								 
}
void Timer2_start(u16 us_100){
	TIM2->ARR=us_100;
	TIM2->CR1|=0x01;    //使能定时器1
}
/*
//void TIM1_UP_IRQHandler(void)
//{ 	
////	u8 i;
//	static u8 num;
//	static u8 action_timer=0;
//	if(TIM1->SR&0X0001)//溢出中断
//	{
//		action_timer++;
//		if((action_timer==2)||(action_timer==5))
//		{
//			if(action_timer==5)
//				action_timer=0;
//			tim_flag=1;
//			if(tim_num)
//				tim_num--;
//			if(tim_action_num)
//				tim_action_num--;
//		}

//		NRF_Sync++;
//			if(NRF_Sync>200)
//			{
//				NRF_Sync=0;

//				num++;
//				if(num>5){
//					num=0;
//					re_send_flag=0;
//				}
//			}
//			if(NRF_Sync==155)
//			{
//				if(!(re_send_flag&(1<<num)))
//				{
//					re_send_ch=num;	
//				}
//			}
//			if(NRF_IRQ_OUT_TIME)
//				NRF_IRQ_OUT_TIME--;
//	}				   
//	TIM1->SR&=~(1<<0);//清除中断标志位 	
//}
//通用定时器中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器2!
*/
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
/*
//通用定时器中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器2!
//void Timerx_Init(u16 arr,u16 psc)
//{
//	RCC->APB1ENR|=1<<0;//TIM2时钟使能    
// 	TIM2->ARR=arr;  //设定计数器自动重装值//刚好1ms    
//	TIM2->PSC=psc;  //预分频器7200,得到10Khz的计数时钟
//	//这两个东东要同时设置才可以使用中断
//	TIM2->DIER|=1<<0;   //允许更新中断				
//	TIM2->DIER|=1<<6;   //允许触发中断
//		  							    
//	TIM2->CR1|=0x01;    //使能定时器3
//  MY_NVIC_Init(2,3,TIM1_UP_IRQChannel,2);//抢占1，子优先级3，组2								 
//}
//TIM3 PWM部分
*/
 
void TIM2_PWM_GPIO_init(void){
	//通道34可用
	//此部分需手动修改IO口设置
	RCC->APB2ENR|=1<<2; 
	RCC->APB2ENR|=1<<3;  	
	RCC->APB2ENR|=1<<0;
	
	AFIO->MAPR|=3<<8;
	//通道
	GPIOB->CRH&=0XFFFF00FF;//PB10 PB11输出 
	GPIOB->CRH|=0X0000BB00;//复用功能输出 2MHz
}
void TIM3_PWM_GPIO_init(void){
	//此部分需手动修改IO口设置
	RCC->APB2ENR|=1<<2;
	RCC->APB2ENR|=1<<3;
	
	GPIOA->CRL&=0X00FFFFFF;//PA6 PA7输出
	GPIOA->CRL|=0XBB000000;//复用功能输出 2MHz 	  	
	
	GPIOB->CRL&=0XFFFFFF00;//PB0 PB1输出 
	GPIOB->CRL|=0X000000BB;//复用功能输出 2MHz
}
void TIM4_PWM_GPIO_init(void){
	//此部分需手动修改IO口设置
	RCC->APB2ENR|=1<<3;

	GPIOB->CRL&=0X00FFFFFF;//PB6 PB7输出
	GPIOB->CRL|=0XBB000000;//复用功能输出 2MHz 	  	
	
	GPIOB->CRH&=0XFFFFFF00;//PB8 PB9输出 
	GPIOB->CRH|=0X000000BB;//复用功能输出 2MHz
}

void TIM5_PWM_GPIO_init(void){
	//此部分需手动修改IO口设置
	RCC->APB2ENR|=1<<3;

	GPIOB->CRL&=0X00FFFFFF;//PB6 PB7输出
	GPIOB->CRL|=0XBB000000;//复用功能输出 2MHz 	  	
	
	GPIOB->CRH&=0XFFFFFF00;//PB8 PB9输出 
	GPIOB->CRH|=0X000000BB;//复用功能输出 2MHz
}

//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
//(TIM2 TIM3 TIM4 TIM5 可用 
void TIMx_PWM_Init(char TIMy,u16 arr,u16 psc,char ch){		 					 
	TIM_TypeDef * TIMx;
	RCC->APB1ENR|=1<<(TIMy-2);       //对应时钟使能 
	switch(TIMy)
	{
		case 2:
			TIMx=TIM2;
			break;
		case 3:
			TIMx=TIM3;
			break;
		case 4:
			TIMx=TIM4;
//			TIM4_PWM_GPIO_init();
			break;
		case 5:
			TIMx=TIM5;
			break;
	}

	TIMx->ARR=arr;//设定计数器自动重装值 
	TIMx->PSC=psc;//预分频器不分频
	if(ch&0x01)
	{
		TIMx->CCMR1|=6<<4;   //CH1 PWM4模式	OCxM寄存器
		TIMx->CCR1=1500;		 //CH1 预装初值
		TIMx->CCMR1|=1<<3;   //CH1预装载使能
		TIMx->CCER|=1<<0;    //OC1 输出使能
	}
	if(ch&0x02)
	{
		TIMx->CCMR1|=6<<12;  //CH2 PWM4模式	
		TIMx->CCR2=1500;		 //CH2 预装初值
		TIMx->CCMR1|=1<<11;  //CH2预装载使能
		TIMx->CCER|=1<<4;    //OC2 输出使能	   
	}
	if(ch&0x04)
	{
		TIMx->CCMR2|=6<<4;	 //CH3 PWM4模式
		TIMx->CCR3=1500;		 //CH3 预装初值
		TIMx->CCMR2|=1<<3;   //CH3预装载使能
		TIMx->CCER|=1<<8;    //OC3 输出使能
	}
	if(ch&0x08)
	{
		TIMx->CCMR2|=6<<12;	 //CH4 PWM4模式
		TIMx->CCR4=1500;		 //CH4 预装初值
		TIMx->CCMR2|=1<<11;  //CH4预装载使能
		TIMx->CCER|=1<<12;   //OC4 输出使能	   
	}
	TIMx->CR1=0x8000;    //ARPE使能 	自动重装载预装载允许位
	TIMx->CR1|=0x01;     //使能定时器4 										 
}  	 
void PWM_Motor(int pwm_l,int pwm_r){
	if(pwm_l>0){TIM4->CCR3=0;if(pwm_l>255-DIE_NUM)pwm_l=255;TIM4->CCR4=DIE_NUM+ pwm_l; }
		else{ TIM4->CCR4=0;pwm_l=-pwm_l;if(pwm_l>255-DIE_NUM)pwm_l=255;TIM4->CCR3=DIE_NUM+ pwm_l; }
	if(pwm_r>0){TIM4->CCR2=0;if(pwm_r>255-DIE_NUM)pwm_l=255;TIM4->CCR1=DIE_NUM+ pwm_r;}
		else{ TIM4->CCR1=0;pwm_r=-pwm_r;if(pwm_r>255-DIE_NUM)pwm_l=255; TIM4->CCR2=DIE_NUM+ pwm_r;}
}










