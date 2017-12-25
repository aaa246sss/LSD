#include "sys.h"

/*
功能:开启外设-后备寄存器
参数:无
*/
void lzj_PWR_ON(void)
{
	RCC->APB1ENR|=3<<27;//开启电源时钟与备份时钟
	PWR->CR|=1<<8;//电源控制寄存器中开启允许修改后备寄存器
}
