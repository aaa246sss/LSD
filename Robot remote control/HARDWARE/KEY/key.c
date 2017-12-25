#include "key.h"
#include "delay.h"
#include "sys.h"

void KEY_Init(void)
{
	RCC->APB2ENR|=1<<3;
	
	GPIOB->CRH&=0XFFFFFF00;
	GPIOB->CRH|=0X00000088;//PA11 PA12 PA15 ��ͨIO �������
	PBout(8)=1;
	PBout(9)=1;
	
} 
//����������
//���ذ���ֵ
//0��û���κΰ�������
//1��KEY0����
//2��KEY1����
//3��KEY2���� WK_UP
//ע��˺�������Ӧ���ȼ�,KEY0>KEY1>KEY2!!
u8 KEY_Scan(void)
{	 
	static u8 key_up=1;//�������ɿ���־	
	if(key_up&&(KEY0==0||KEY1==0||KEY2==1))
	{
		delay(10);//ȥ���� 
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
	return 0;// �ް�������
}




















