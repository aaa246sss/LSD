#include "key.h"
#include "delay.h"
#include "sys.h"

void KEY_Init(void)
{
	RCC->APB2ENR|=1<<3;
	
	GPIOB->CRH&=0XFFFFFF00;
	GPIOB->CRH|=0X00000088;//PA11 PA12 PA15 普通IO 推挽输出
	PBout(8)=1;
	PBout(9)=1;
	
} 
//按键处理函数
//返回按键值
//0，没有任何按键按下
//1，KEY0按下
//2，KEY1按下
//3，KEY2按下 WK_UP
//注意此函数有响应优先级,KEY0>KEY1>KEY2!!
u8 KEY_Scan(void)
{	 
	static u8 key_up=1;//按键按松开标志	
	if(key_up&&(KEY0==0||KEY1==0||KEY2==1))
	{
		delay(10);//去抖动 
		key_up=0;
		if(KEY0==0)
		{
			return 1;
		}
		else if(KEY1==0)
		{
			
			return 2;
		}
		else if(KEY2==1)
		{
			return 3;
		}
	}else if(KEY0==1&&KEY1==1&&KEY2==0)key_up=1; 	    
	return 0;// 无按键按下
}




















