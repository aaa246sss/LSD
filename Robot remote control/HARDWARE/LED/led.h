#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//STM32 平衡小车程序
//designed by XiaoDaGe
//		   
//诗凯科技 @shikaikeji
//QQ交流群: 346431805
//修改日期:2014/5/18
//版本：V1.2
//版权所有，盗版必究。
//淘宝链接：http://item.taobao.com/item.htm?_u=o1o5aod496b0&id=39037359677
//Copyright(C) 诗凯科技 2013-2019
//All rights reserved
//********************************************************************************
//////////////////////////////////////////////////////////////////////////////////	 
//LED端口定义
#define LED_G PBout(6)// PA8
#define LED_R PBout(7)// PD2	
//#define LED2 PBout(0)// PA1	
//#define KEY1 PBin(8)
//#define KEY2 PBin(9)
//#define LED3 PAout(2)// PA2		

void LED_Init(void);//初始化		 				    
#endif

















