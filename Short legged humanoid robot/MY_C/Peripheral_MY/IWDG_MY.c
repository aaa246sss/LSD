#include "sys.h"
#include "IWDG_MY.h"
void IWDG_init(char PR,unsigned int RLR)//RLR最大0xFFF;
{
	IWDG->KR=0x5555;//允许修改预分频和重载处置寄存器
	IWDG->PR=PR;
	IWDG->RLR=RLR;
	IWDG->KR=0xCCCC;//启动看门狗
}
