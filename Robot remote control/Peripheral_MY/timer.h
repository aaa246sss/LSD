#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"






//#ifdef RIGHT_HAND_M  //0~5
//	#define NRF_CHANNEL1
//#elif defined RIGHT_HAND_L
//	#define NRF_CHANNEL2
//#elif defined LEFT_HAND_M
//	#define NRF_CHANNEL3
//#elif defined LEFT_HAND_L		
//	#define NRF_CHANNEL4
//#elif defined LEFT_FOOD		
//	#define NRF_CHANNEL5
//#else	
//	#define NRF_CHANNEL6
//#endif





void Timer1_Init(u16 arr,u16 psc);
void PWM_Motor(int pwm_l,int pwm_r);
void Timerx_Init(u16 arr,u16 psc);
void PWM_Init(u16 arr,u16 psc);
void Tim3Init(void);
float filter_MY(float temp);
float GET_NOWTIME(void);
void Tim4_Init(u16 arr,u16 psc);
#endif























