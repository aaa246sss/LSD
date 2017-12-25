#include "misc.h"	//ÖÐ¶Ï
#include <stm32f10x.h>
#include "Low_power.h"
void CPU_SLEEP_INT_MODE(FunctionalState NewState)
{
	NVIC_SystemLPConfig(NVIC_LP_SLEEPONEXIT,NewState);
}

void CPU_SLEEP_ENTER(void)	
{
	__WFI();
}




