#include "sys.h"
#include "IWDG_MY.h"
void IWDG_init(char PR,unsigned int RLR)//RLR���0xFFF;
{
	IWDG->KR=0x5555;//�����޸�Ԥ��Ƶ�����ش��üĴ���
	IWDG->PR=PR;
	IWDG->RLR=RLR;
	IWDG->KR=0xCCCC;//�������Ź�
}
