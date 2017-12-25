
//#include "includes.h"
#include "sys.h"
#include <stm32f10x.h>
#include <stm32f10x_adc.h>
#include <stm32f10x_dma.h>
#include <stm32f10x_gpio.h>

#include "delay.h"
#include <stdio.h>

#define ADC1_DR_Address   ((u32)0x4001244C)

short data_buf[100];
u8  data_buf_hander_flag=0;
unsigned short ADC_ConvertedValue[8];
void ADC_init_MY(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	ADC_InitTypeDef  ADC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;				// ѡ�йܽ�4
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;			// ģ������
	GPIO_Init(GPIOC, &GPIO_InitStructure);				//ѡ��C�˿�	
	RCC_ADCCLKConfig(RCC_PCLK2_Div4);				//����ADCʱ��
	ADC_InitStructure.ADC_Mode=ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv =ADC_ExternalTrigConv_None;	
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right   ;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1,&ADC_InitStructure);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);//����ָ��ADC�Ĺ�����ͨ�����������ǵ�ת��˳��Ͳ���ʱ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);	  	
	ADC_ResetCalibration(ADC1);  // ����ָ����ADC��У׼�Ĵ�?	
	ADC_StartCalibration(ADC1); // Start ADC1 calibaration
	while(ADC_GetCalibrationStatus(ADC1)); //	Check the end of ADC1 calibration
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);//Start ADC1 Software Conversion
	//ADC_Cmd(ADC1,DISABLE);	 ������������
}


void RCC_FOR_ADC_CONFIG(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	/* Enable ADC1 and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|
													RCC_APB2Periph_GPIOB|
													RCC_APB2Periph_GPIOA, 
													ENABLE);
}
void GPIO_FOR_ADC_CONFIG(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|
//										GPIO_Pin_2|GPIO_Pin_3|
//										GPIO_Pin_4|GPIO_Pin_5;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
//	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|
										GPIO_Pin_2|GPIO_Pin_3|
										GPIO_Pin_4|GPIO_Pin_5|
										GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void ADC_FOR_SELF_CONFIG(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//����ģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;			//��ͨ�����ͨ��ģʽ,��ͨ��Disabled
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//����ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת������,�Ƿ���Ҫ�������Ź��¼��ж�
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�������Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 8;								//ADCͨ����
	ADC_Init(ADC1, &ADC_InitStructure);
	
		/* ADC1 configuration ------------------------------------------------------*/

	/* ADC1 regular channel8 configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
	/* ADC1 regular channel9 configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5);
	/* ADC1 regular channel10 configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_55Cycles5);
	/* ADC1 regular channel11 configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_55Cycles5);
	/* ADC1 regular channel12 configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_55Cycles5);
	/* ADC1 regular channel13 configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_55Cycles5);
	/* ADC1 regular channel14 configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7, ADC_SampleTime_55Cycles5);
	/* ADC1 regular channel15 configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_55Cycles5);
}

void DMA_FOR_ADC_CONFIG(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//�������
	DMA_InitStructure.DMA_BufferSize = 8;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1, ENABLE);
}

void ADC_8td_init_MY(void)
//ʹ��ʱ��MAIN�����м���extern unsigned short ADC_ConvertedValue[8];
{
	RCC_FOR_ADC_CONFIG();
	GPIO_FOR_ADC_CONFIG();
	ADC_FOR_SELF_CONFIG();
	DMA_FOR_ADC_CONFIG();
	/* Configure PB.0\1 (ADC Channel8\9) as analog input*/
	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);	//��һ��˵��ʹ��DMAģʽ
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	/* Enable ADC1 reset calibaration register */   
	ADC_ResetCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));
	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));   
	/* Start ADC1 Software Conversion */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
void ADC_FUN_test(void)
{
	u16 ADC_storedValue[5][8];
	u16 AD_output[8];
  u16 i='0',j,k,temp;
//  unsigned short AD_scaled_ex[8];
//  unsigned short AD_scaled[8];
//  USART_init_MY();
 ADC_8td_init_MY();

  printf("asdf");
  while (1)
    {
 for(i=0;i<40;i++)
  {
	 ADC_storedValue[i/8][i%8]=ADC_ConvertedValue[i%8];

  }	
 for(i=0;i<8;i++)
 {
  for(j=0;j<4;j++)
  for(k=j+1;k<5;k++)
  {
  if (ADC_storedValue[j][i]>ADC_storedValue[k][i])
  {
  temp=ADC_storedValue[j][i];
  ADC_storedValue[j][i]=ADC_storedValue[k][i];
  ADC_storedValue[k][i]=temp;
  }
  }
  AD_output[i]=ADC_storedValue[2][i]>1500?1:0;
  //AD_output[i]=ADC_storedValue[2][i]>fazhi[i]?1:0;
 }
  for(i=0;i<40;i++)
  {
	 printf("%d ",ADC_storedValue[i/8][i%8]);
	 if(i%8==7)
	 printf("\r\n");
  }
  printf("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\r\n");
  for(i=0;i<8;i++)
  printf("%d ",ADC_storedValue[2][i]);
  printf("\r\n");
  printf("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\r\n");
  for(i=0;i<8;i++)
  printf( "%d ",AD_output[i]);
 printf("\r\n");
 //for(i=0;i<60000;i++);
 delay(1000);
    } 

}



////////////////////////////////////////////
//ʵ��ʹ�����沿��,����ֻ�Ǿ���
////////////////////////////////////////////
//ʹ��PA0 //һ·
void RCC_FOR_ADC_1_CONFIG(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	/* Enable ADC1 and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|
													RCC_APB2Periph_GPIOA, 
													ENABLE);
}
void GPIO_FOR_ADC_1_CONFIG(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
}

void ADC_FOR_SELF_1_CONFIG(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//����ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;			//��ͨ�����ͨ��ģʽ,��ͨ��Disabled
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//����ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת������,�Ƿ���Ҫ�������Ź��¼��ж�
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�������Ҷ���
//	ADC_InitStructure.ADC_NbrOfChannel = 1;								//ADCͨ���� //��Ϊû�п���ɨ������ģʽ,����Ҫpeizhi
	ADC_Init(ADC1, &ADC_InitStructure);
	
		/* ADC1 configuration ------------------------------------------------------*/

	/* ADC1 regular channel1 configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
}

//void DMA_FOR_ADC_1_CONFIG()
//{
//	DMA_InitTypeDef DMA_InitStructure;
//	
//	DMA_DeInit(DMA1_Channel1);
//	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
//	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//�������
//	DMA_InitStructure.DMA_BufferSize = 1;
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;//һ�����ݲ�������
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
//	DMA_Cmd(DMA1_Channel1, ENABLE);
//	
//	DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);
//	MY_NVIC_Init(1,2,DMA1_Channel1_IRQn,2);//��2���ϵ͵����ȼ� //������������жϰ�
//}

//void ADC_1td_DMA_init_MY()
////ʹ��ʱ��MAIN�����м���extern unsigned short ADC_ConvertedValue[8];
//{
//	RCC_FOR_ADC_1_CONFIG();
//	GPIO_FOR_ADC_1_CONFIG();
//	ADC_FOR_SELF_1_CONFIG();
//	DMA_FOR_ADC_1_CONFIG();
//	/* Configure PB.0\1 (ADC Channel8\9) as analog input*/
//	/* Enable ADC1 DMA */
//	ADC_DMACmd(ADC1, ENABLE);	//��һ��˵��ʹ��DMAģʽ
//	
//	/* Enable ADC1 */
//	ADC_Cmd(ADC1, ENABLE);
//	/* Enable ADC1 reset calibaration register */   
//	ADC_ResetCalibration(ADC1);
//	/* Check the end of ADC1 reset calibration register */
//	while(ADC_GetResetCalibrationStatus(ADC1));
//	/* Start ADC1 calibaration */
//	ADC_StartCalibration(ADC1);
//	/* Check the end of ADC1 calibration */
//	while(ADC_GetCalibrationStatus(ADC1));   
//	/* Start ADC1 Software Conversion */ 
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
//}

short GET_ADC_value(void)
{
	if(data_buf_hander_flag)
	{
		DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,DISABLE);//��ͣ�ж�,ð�ݴ������̿�������
		for(u8 i=0;i<99;i++)
		{
			for(u8 j=i+1;j<100;j++)
			{
				if(data_buf[j]<data_buf[i])
				{
					data_buf[j]=data_buf[j]+data_buf[i];
					data_buf[i]=data_buf[j]-data_buf[i];
					data_buf[j]=data_buf[j]-data_buf[i];
				}
			}
		}
		data_buf_hander_flag=0;
		DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);//����DMA�ж�,
		return data_buf[49];
//		delay(100);
	}
	return -1;
}
void DMA1_Channel1_FOR_ADC_IRQn_call(void)
{
	static u8 num=0;
	if(!data_buf_hander_flag)
	{
		data_buf[num++]=ADC_ConvertedValue[0];
		if(num==100)
		{	
			num=0;
			data_buf_hander_flag=1;
		}
	}
}