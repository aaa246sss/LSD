#include "stm32f4xx_hal.h"
#include "myspi.h"
#include "nrf24l01.h"
#include "delay.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>

#define DYNAMIC_PAYLOAD			1//使用动态加载数据长度[长度相符时,接收机可以静态接收发射机的动态数据]
#define DYNAMIC_PAYLOAD_ACK	1//使用动态数据长度时ACK功能
//#define DYNPD_ACK_DATA		1//使用动态数据长度时带数据ACK功能


extern SPI_HandleTypeDef hspi2;

const uint8_t NRF_TX_ADDRESS2[TX_ADR_WIDTH]={88,15,78,118,12};
	
/****向寄存器reg读一个字节，同时返回状态字节**************/
uint8_t SPI_Read_Reg_2(uint8_t reg){
	uint8_t status;
	CSN_2(0);
	SPI_Send_byte(SPI2,reg);
	status=SPI_Receive_byte(SPI2,0);   //select register  and write value to it
	CSN_2(1);
	return(status);
}

void NRF24L01_init_2(void){//0接收 1发送
	char read_buf[10];
	SPI2->CR1|=1<<6;//使能SPI2
	CE_2(0);			//nRF24L01_CE_2=0;			 chip enable
	CSN_2(1);			//nRF24L01_CSN_2=1;			 Spi enable
	
	SPI_RW_Reg_2(FLUSH_TX,0);//清除发送缓冲区数据 必须
	SPI_RW_Reg_2(FLUSH_RX,0);//清除发送缓冲区数据 必须
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + STATUS, 0xff);      // 清除TX，让IRQ拉低
	delay_us(100);
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + EN_AA, 0x1);       // 使能接收通道0自动应答 T
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + EN_RXADDR, 0x1);   // 使能所有接收通道0 T
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + SETUP_RETR, 0x03);  // 自动重发延时等待750us+86us，自动重发10次 //不重发 R
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + RF_CH, 0x40);       // 选择射频通道0x40 T
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + RF_SETUP, 0x07);    // 数据传输率1Mbps，发射功率0dBm，低噪声放大器增益 T
#ifdef DYNAMIC_PAYLOAD	
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + FEATURE, 0x07);    //动态载荷
	if(SPI_Read_Reg_2(FEATURE) == 0x00)				//检查动态载荷是否使能
		SPI_RW_Reg_2(Activate, Code_Activate);	//使能
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + DYNPD, 0x01);    //通道使能
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + FEATURE, 0x07);    //动态载荷
#else
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + RX_PW_P0 , 1);  // 接收通道0选择和发送通道相同有效数据宽度 
#endif
	SPI_Write_Buf_2(WRITE_REG_NRF24L01 + TX_ADDR, (uint8_t *)NRF_TX_ADDRESS2, TX_ADR_WIDTH);     // 写入发送地址 //本机地址(接收端的识别码)
	SPI_Read_Buf_2(TX_ADDR,read_buf,5);
	SPI_Write_Buf_2(WRITE_REG_NRF24L01 + RX_ADDR_P0, (uint8_t *)NRF_TX_ADDRESS2, TX_ADR_WIDTH);  // 接收设备接收通道0使用和发送设备相同的发送地址
}



/****向寄存器reg写一个字节，同时返回状态字节**************/
uint8_t SPI_RW_Reg_2(uint8_t reg,uint8_t value)
{
	uint8_t status;
	CSN_2(0);
	status=SPI_Receive_byte(SPI2,reg);   //select register  and write value to it
	SPI_Send_byte(SPI2,value);   
	CSN_2(1);
	return(status); 
}


/********读出bytes字节的数据*************************/
uint8_t SPI_Read_Buf_2(uint8_t reg,char *pBuf,uint8_t bytes)
{
	uint8_t status,byte_ctr;
	CSN_2(0);
	status=SPI_Receive_byte(SPI2,reg);       
	for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
		pBuf[byte_ctr]=SPI_Receive_byte(SPI2,0);
	CSN_2(1);
	return(status);
}

/****************写入bytes字节的数据*******************/
uint8_t SPI_Write_Buf_2(uint8_t reg, uint8_t *pBuf,uint8_t bytes)
{
	uint8_t status,byte_ctr;
	CSN_2(0);
	status=SPI_Receive_byte(SPI2,reg); 
	delay1us(1);
	for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
		SPI_Send_byte(SPI2,*pBuf++);
	CSN_2(1);
	return(status);
}

void NRF_RX_Mode_2(GLOVAR_HandleTypeDef *GloVar,char buf_len)
{
//	GloVar->NRF.MODE=NRF_MODE_RX;
		CE_2(0);
	SPI_RW_Reg_2(FLUSH_RX,0);//清除发送缓冲区数据 必须
	SPI_RW_Reg_2(FLUSH_TX,0);//清除发送缓冲区数据 必须
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + STATUS, 0xff);      // 清除TX，让IRQ拉低；
	delay1us(1);
#ifdef DYNAMIC_PAYLOAD
#else 
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + RX_PW_P0, buf_len);  // 接收通道0选择和发送通道相同有效数据宽度
#endif
	
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + CONFIG, 0x0b);              // CRC使能，16位CRC校验，上电，接收模式
  	CE_2(1);
}

//void NRF24L01_Send_2(GLOVAR_HandleTypeDef *GloVar)
//{
////	 uint8_t status=0x00;
////	GloVar->NRF.MODE=NRF_MODE_TX;	
//	CE_2(0);
////所谓发送地址,不过是广播数据,接收端将所有数据接收到后进行过滤操作
////	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + CONFIG, 0);      // CRC使能，16位CRC校验，上电	
//	SPI_RW_Reg_2(FLUSH_TX,0);//清除发送缓冲区数据 必须
//	SPI_RW_Reg_2(FLUSH_RX,0);//清除发送缓冲区数据 必须
//	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + STATUS, 0xff);      // 清除TX，让IRQ拉低；
//	delay1us(2);
//#ifdef DYNAMIC_PAYLOAD
//#else 
//	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + RX_PW_P0, TX_LEN);  // 接收通道0选择和发送通道相同有效数据宽度
//#endif
//	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + CONFIG, 0x0a);      // CRC使能，16位CRC校验，上电	
//	GloVar->NRF.BUF_TX[0]=TIM1->CNT;
//	SPI_Write_Buf_2(WR_TX_PLOAD, GloVar->NRF.BUF_TX, TX_LEN); 			 // 装载数据	
////	GloVar->NRF.FLAG_TX_OK=0;
//	CE_2(1);//中断线拉低
//}

void NRF24L01_Send_2(uint8_t *buf,uint8_t buf_len)
{
//	 uint8_t status=0x00;
//	GloVar->NRF.MODE=NRF_MODE_TX;	
	CE_2(0);
//所谓发送地址,不过是广播数据,接收端将所有数据接收到后进行过滤操作
//	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + CONFIG, 0);      // CRC使能，16位CRC校验，上电	
	SPI_RW_Reg_2(FLUSH_TX,0);//清除发送缓冲区数据 必须
	SPI_RW_Reg_2(FLUSH_RX,0);//清除发送缓冲区数据 必须
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + STATUS, 0xff);      // 清除TX，让IRQ拉低；
	delay1us(2);
#ifdef DYNAMIC_PAYLOAD
#else 
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + RX_PW_P0, TX_LEN);  // 接收通道0选择和发送通道相同有效数据宽度
#endif
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + CONFIG, 0x0a);      // CRC使能，16位CRC校验，上电	
	SPI_Write_Buf_2(WR_TX_PLOAD, buf, buf_len); 			 // 装载数据	
//	GloVar->NRF.FLAG_TX_OK=0;
	CE_2(1);//中断线拉低
}

//void NRF_Message(char *str)
//{
//#ifndef NOTERM
//    if(str != NULL)
//    {
//        while(*str!='\0')
//        {
//          //MAP_UARTCharPut(UARTA1_BASE,*str++);
//					USART_SendData(USART2,*str++);
//					delay_us(100);
//        }
//    }
//#endif
//}

//void Uart_Send(char *str,)
int NRF_printf(char *pcFormat, ...)
{
#ifdef DEBUG
 int iRet = 0;
#ifndef NOTERM

  char *pcBuff, *pcTemp;
  int iSize = 32;
 
  va_list list;
  pcBuff = (char*)malloc(iSize);
  if(pcBuff == NULL)
  {
	  return -1;
  }
  while(1)
  {
      va_start(list,pcFormat);
      iRet = vsnprintf(pcBuff,iSize,pcFormat,list);
      va_end(list);
      if(iRet > -1 && iRet < iSize)
      {
          break;
      }
      else
      {
          iSize*=2;
          if((pcTemp=realloc(pcBuff,iSize))==NULL)
          { 
//              Message1("Could not reallocate memory\n\r");
              iRet = -1;
              break;
          }
          else
          {
              pcBuff=pcTemp;
          }
      }
  }
	NRF24L01_Send_2((uint8_t*)pcBuff,iSize);
//  Message1(pcBuff);
	delay(5);
  free(pcBuff);
#endif
  return iRet;
#else
	return 0;
#endif
}

