#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

#define LED0_PWM_VAL TIM3->CCR2 
void PWM_Motor(int pwm_l,int pwm_r);
void Timer1_Init(u16 arr,u16 psc);
void Timer2_Init(void);
void Timer2_start(u16 us_100);
void TIMx_PWM_Init(char TIMy,u16 arr,u16 psc,char ch);
void TIM2_PWM_GPIO_init(void);
void TIM3_PWM_GPIO_init(void);
void TIM4_PWM_GPIO_init(void);
void TIM5_PWM_GPIO_init(void);

u8 GET_PWM_UP_FLAG(void);
void PWM_UP_FLAG_off(void);
//void nrf_re_send_ch_write(u8 channel);
//u8 nrf_re_send_ch_read(void);

#endif























