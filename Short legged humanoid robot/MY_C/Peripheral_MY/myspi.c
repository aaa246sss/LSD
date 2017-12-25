#include "sys.h"
#include "myspi.h"



//#include <stm32f10x_spi.h>
//#include <stm32f10x_rcc.h>
//#include <stm32f10x_gpio.h>
//#include <stm32f10x_dma.h>

//#define SPI2_RX_SIZE 32
//#define SPI2_TX_SIZE 32
//uint8_t SPI2_Recv_Buffer[SPI2_RX_SIZE];
//uint8_t SPI2_Send_Buffer[SPI2_TX_SIZE];

void SPI_Init_MY(void)
{
			RCC->APB2ENR|=1<<3;//先使能外设IO B
			RCC->APB1ENR|=1<<14;//先使能外设IO PORTA时钟 	和AFIO	 和SPI2		
			GPIOB->CRH&=0X000FFFFF; 
			GPIOB->CRH|=0XBBB00000;//PA567复用输出
//			GPIOB->ODR|=3;     //PA10,11 输出高
			
			SPI2->CR1=0x314;//配置SPI2 主机模式 8分频 CPHA=0 CPOL=0 先发MSB SSM=1 SSI=1 全双工 8位数据帧 来自发送缓冲区 禁止CRC 只收模式(只是单线用) 双向双线
			SPI2->CRCPR=7;//这个并没有用
			SPI2->CR1|=0x40;//启动SPI2
}

//下面是SPI_DMA操作
//不用DMA原因API时序有寄存器,每次写入数据往不同的寄存器,比较乱 
/*
//void RCC_FOR_SPI2_CONFIG(void)
//{
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
//	RCC_AHBPeriphClockCmd(RCC_AHBENR_DMA1EN,ENABLE);
//}
//void GPIO_FOR_SPI2_CONFIG(void)
//{
//	GPIO_InitTypeDef  GPIO_InitStructure;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//}
//void DMA_FOR_SPI2_CONFIG(void)
//{
//	DMA_InitTypeDef DMA_InitStructure;
//	
//	//配置SPI2_RX_DMA
//	DMA_DeInit(DMA1_Channel4);
//	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&SPI2_Recv_Buffer;
//	DMA_InitStructure.DMA_BufferSize = SPI2_RX_SIZE;
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI2->DR;
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//从外设读
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
//  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//  DMA_Init(DMA1_Channel4, &DMA_InitStructure);
//	
//	DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);
//	
//	//配置SPI2_TX_DMA
//	DMA_DeInit(DMA1_Channel5);  
//  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI2->DR;
//	DMA_InitStructure.DMA_BufferSize = SPI2_TX_SIZE;
//  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//从存储器读数据
//  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)SPI2_Send_Buffer;
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
//  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//  DMA_Init(DMA1_Channel5, &DMA_InitStructure);
//	
//	
//}
////配置SPI2 主机模式 8分频 CPHA=0 CPOL=0 先发MSB SSM=1 SSI=1 全双工 8位数据帧 来自发送缓冲区 禁止CRC 只收模式(只是单线用) 双向双线
//void SPI2_FOR_SELF_CONFIG(void)
//{
//	SPI_InitTypeDef SPI_InitStructure;
//	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex ;
//	SPI_InitStructure.SPI_Mode = SPI_Mode_Master ;
//	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b ;
//	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low ;
//	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge ;
//	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft ;
//	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8 ;
//	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB ;
//	SPI_InitStructure.SPI_CRCPolynomial = 7 ;
//	SPI_Init( SPI2, &SPI_InitStructure ) ;
//}
//void SPI2_INIT(void)
//{
//	RCC_FOR_SPI2_CONFIG();
//	GPIO_FOR_SPI2_CONFIG();
//	SPI2_FOR_SELF_CONFIG();
//	DMA_FOR_SPI2_CONFIG();
//	
//	SPI_I2S_DMACmd(SPI2,SPI_I2S_DMAReq_Rx,ENABLE);
//	SPI_I2S_DMACmd(SPI2,SPI_I2S_DMAReq_Tx,ENABLE);
//	
//	DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);
//	MY_NVIC_Init(2,3,DMA1_Channel4_IRQn,2);
//	DMA_ITConfig(DMA1_Channel5,DMA_IT_TC,ENABLE);
//	MY_NVIC_Init(2,2,DMA1_Channel5_IRQn,2);

//	SPI_Cmd(SPI2,ENABLE);
//}


//void SPI2_DMA_Trans( uint16_t num )
//{
//	DMA_SetCurrDataCounter(DMA1_Channel5, num);
//	DMA_SetCurrDataCounter(DMA1_Channel4, num);

//	DMA_ClearFlag(DMA1_FLAG_GL5|DMA1_FLAG_TC5|DMA1_FLAG_HT5|DMA1_FLAG_TE5);
//	DMA_ClearFlag(DMA1_FLAG_GL4|DMA1_FLAG_TC4|DMA1_FLAG_HT4|DMA1_FLAG_TE4);

//	SPI2->DR ;//clear SPI2->DR

//	DMA_Cmd(DMA1_Channel5, ENABLE);
//	DMA_Cmd(DMA1_Channel4, ENABLE);   
//}
//uint8_t SPI2_DMA_Read(uint8_t* data, uint8_t bytesNumber)
//{
//	uint8_t byte = 0;
//	for(byte = 0; byte < bytesNumber; byte++){
//		SPI2_Send_Buffer[byte]=0;
//	}
////	On_Spi2_STX();
//	SPI2_DMA_Trans(bytesNumber);
//	for(byte = 0; byte < bytesNumber; byte++){
//		data[byte]=SPI2_Recv_Buffer[byte];
//	}
//	return bytesNumber;
//}
//uint8_t SPI2_DMA_Write(uint8_t* data, uint8_t bytesNumber)
//{   
//	uint8_t byte = 0;
//	for(byte = 0; byte < bytesNumber; byte++){
//			 SPI2_Send_Buffer[byte]=data[byte];
//	}
////	On_Spi2_STX();
//	SPI2_DMA_Trans(bytesNumber);
//	return bytesNumber;
//}
*/
