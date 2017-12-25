#include "myspi.h"
#include "sys.h"
#include "nrf24l01.h"
#include "delay.h"
#include <stdio.h>
#include <string.h>
#include "LED.h"
#define uint16_t unsigned short int
#define spi_delay_us delay_System_us

extern u16 NRF_IRQ_OUT_TIME;
//const u8 TX_ADDRESS[6][TX_ADR_WIDTH] =
//{
//	1,21,64,67,6,
//	53,21,64,67,6,
//	34,21,64,67,6,
//	62,21,64,67,6,
//	89,21,64,67,6,
//	17,21,64,67,6,
//};  // 定义一个静态发送地址 通道1

const u8 TX_ADDRESS[TX_ADR_WIDTH]={101,23,65,86,88};

u8 RX_BUF[RX_LEN];
u8 buggg[6];
char TX_BUF[TX_LEN];
static void SPI_Send_byte(SPI_TypeDef* SPIx,u8 data){
	while(!(SPIx->SR&(1<<1)));
	SPIx->DR = data;
	while(!(SPIx->SR&(1<<0)));
	SPIx->DR;
}

static u8 SPI_Receive_byte(SPI_TypeDef* SPIx,u8 data){
	while(!(SPIx->SR&(1<<1)));
	SPIx->DR = data;
	while(!(SPIx->SR&(1<<0)));
	return SPIx->DR;
}
/****向寄存器reg读一个字节，同时返回状态字节**************/
u8 SPI_Read_Reg(u8 reg){
	u8 status;
	CSN=0;
	SPI_Send_byte(SPI2,reg);
	status=SPI_Receive_byte(SPI2,0);   //select register  and write value to it
	CSN=1;
	return(status);
}

void NRF24L01_init(char RT_MODE){//0接收 1发送
	RCC->APB2ENR|=1<<3;//先使能外设IO PORTB时钟 
	GPIOB->CRH&=0XFFF000FF; 
	GPIOB->CRH|=0X00033800;//PB15 14推挽输出 PA13上拉输入 
	GPIOB->BSRR|=1<<10;
	CE=0;			//nRF24L01_CE=0;			 chip enable
	CSN=1;			//nRF24L01_CSN=1;			 Spi enable
	SPI_Init_MY();
	SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0xff);      // 清除TX，让IRQ拉低
	SPI_RW_Reg(FLUSH_TX,0);//清除发送缓冲区数据 必须
	SPI_RW_Reg(FLUSH_RX,0);//清除发送缓冲区数据 必须
	delay_us(100);
	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x1);       // 使能接收通道0自动应答 T
	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x1);   // 使能接收通道0 T
	SPI_RW_Reg(WRITE_REG_NRF24L01 + SETUP_RETR, 0x14);  // 自动重发延时等待750us+86us，自动重发10次 //不重发 R
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_CH, 0x40);       // 选择射频通道0x40 T
//	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x0f);    // 数据传输率1Mbps，发射功率0dBm，低噪声放大器增益 T
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x07);    // 数据传输率1Mbps，发射功率0dBm，低噪声放大器增益 T
	

	if(RT_MODE){//1发送
//		SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0 , RX_LEN);  // 接收通道0选择和发送通道相同有效数据宽度 
		SPI_Write_Buf(WRITE_REG_NRF24L01 + TX_ADDR, (u8 *)TX_ADDRESS, TX_ADR_WIDTH);     // 写入发送地址 //本机地址(接收端的识别码)
		SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, (u8 *)TX_ADDRESS, TX_ADR_WIDTH);  // 为了应答接收设备，接收通道0地址和发送地址相同
		SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, TX_LEN);  // 接收通道0选择和发送通道相同有效数据宽度
	}
	else{
		SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, (u8 *)TX_ADDRESS, TX_ADR_WIDTH);  // 接收设备接收通道0使用和发送设备相同的发送地址
		SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, RX_LEN);  // 接收通道0选择和发送通道相同有效数据宽度 
	}
}

/****向寄存器reg写一个字节，同时返回状态字节**************/
u8 SPI_RW_Reg(u8 reg,u8 value){
	u8 status;
	CSN=0;
	status=SPI_Receive_byte(SPI2,reg);   //select register  and write value to it
	SPI_Send_byte(SPI2,value);   
	CSN=1;
	return(status); 
}


/********读出bytes字节的数据*************************/
u8 SPI_Read_Buf(u8 reg,u8 *pBuf,u8 bytes){
	u8 status,byte_ctr;
	CSN=0;
	status=SPI_Receive_byte(SPI2,reg);       
	for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
		pBuf[byte_ctr]=SPI_Receive_byte(SPI2,0);
	CSN=1;
	return(status);
}

/****************写入bytes字节的数据*******************/
u8 SPI_Write_Buf(u8 reg, u8 *pBuf,u8 bytes){
	u8 status,byte_ctr;
	CSN=0;
	status=SPI_Receive_byte(SPI2,reg); 
	spi_delay_us(1);
	for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
		SPI_Send_byte(SPI2,*pBuf++);
	CSN=1;
	return(status);
}


void RX_Mode(char buf_len){
//	u8 status;
//	static u8 aa=1;
	CE=0;
	//这句与接收代码不同
	SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0xff);      // 清除TX，让IRQ拉低；	
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, RX_LEN);  // 接收通道0选择和发送通道相同有效数据宽度
	SPI_RW_Reg(FLUSH_RX,0);//清除发送缓冲区数据 必须
	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0b);              // CRC使能，16位CRC校验，上电，接收模式
  CE=1;
}

void NRF24L01_Send(u8 * tx_buf){
	LED_G=0;
		CE=0;
//所谓发送地址,不过是广播数据,接收端将所有数据接收到后进行过滤操作
	SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0xff);      // 清除TX，让IRQ拉低；	
	SPI_RW_Reg(FLUSH_TX,0);//清除发送缓冲区数据 必须
	spi_delay_us(20);
	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_LEN); 			 // 装载数据	这个好像被固定32 不能改 受那边初始化影响
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, TX_LEN);  // 接收通道0选择和发送通道相同有效数据宽度

	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0a);      // CRC使能，16位CRC校验，上电	
	
	CE=1;//中断线拉低
}



