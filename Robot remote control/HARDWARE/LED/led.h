#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//STM32 ƽ��С������
//designed by XiaoDaGe
//		   
//ʫ���Ƽ� @shikaikeji
//QQ����Ⱥ: 346431805
//�޸�����:2014/5/18
//�汾��V1.2
//��Ȩ���У�����ؾ���
//�Ա����ӣ�http://item.taobao.com/item.htm?_u=o1o5aod496b0&id=39037359677
//Copyright(C) ʫ���Ƽ� 2013-2019
//All rights reserved
//********************************************************************************
//////////////////////////////////////////////////////////////////////////////////	 
//LED�˿ڶ���
#define LED_G PBout(6)// PA8
#define LED_R PBout(7)// PD2	
//#define LED2 PBout(0)// PA1	
//#define KEY1 PBin(8)
//#define KEY2 PBin(9)
//#define LED3 PAout(2)// PA2		

void LED_Init(void);//��ʼ��		 				    
#endif

















