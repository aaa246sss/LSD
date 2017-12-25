//#include "includes.h"
//#include "sys.h"
#include <stm32f10x_dma.h>

void EXTI9_5_IRQn_call(void);
//void TIM1_UP_IRQn_call(void);
void USART1_IRQn_call(void);
void DMA1_Channel4_IRQn_call(void);
void DMA1_Channel5_IRQn_call(void);
void DMA1_Channel4_FOR_USART1_IRQn_call(void);
void DMA1_Channel5_FOR_USART1_IRQn_call(void);
void DMA1_Channel1_FOR_ADC_IRQn_call(void);
/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup USART_Printf
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//void EXTI9_5_IRQHandler(void)
//{
//	EXTI9_5_IRQn_call();
//}

void SPI2_IRQHandler(void)
{

}


void SPI1_IRQHandler(void)
{

}

void WWDG_IRQHandler(void)
{
//  /* Update WWDG counter */
//  WWDG_SetCounter(0x50);
//	
//  /* Clear EWI flag */
//  WWDG_ClearFlag();
//  
}

void DMA1_Channel1_IRQHandler(void){
	DMA1_Channel1_FOR_ADC_IRQn_call();
	DMA_ClearFlag(DMA1_FLAG_TC1);
}
void DMA1_Channel4_IRQHandler(void){
//	DMA_ClearFlag(DMA1_FLAG_TC4);
//	DMA_Cmd(DMA1_Channel4,DISABLE);
//	DMA1_Channel4_IRQn_call();
	DMA1_Channel4_FOR_USART1_IRQn_call();
}

void DMA1_Channel5_IRQHandler(void){
//	DMA_ClearFlag(DMA1_FLAG_TC5);
//	DMA_Cmd(DMA1_Channel5,DISABLE);
//	DMA1_Channel5_IRQn_call();
	DMA1_Channel5_FOR_USART1_IRQn_call();
}
//void TIM1_UP_IRQHandler(void)
//{
//	TIM1_UP_IRQn_call();
////	static u8 tim1_t=0;
////	tim1_t++;
////	if(tim1_t>=tim1_time)
////	{
////		tim1_t=0;
////		//USART_SendData(USART1, '5');
////	}	
////	TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update); //ÇåÖÐ¶Ï
//}


void USART1_IRQHandler(void)
{	
	USART1_IRQn_call();
}
void ADC1_2_IRQHandler(void)
{
//	static float AD_value;	
//	if(ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET)
//	{
//		ADC_Cmd(ADC1,DISABLE);
//		AD_value=ADC1->DR;
//		printf("%f\r\n",AD_value);
//	}
}



/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
