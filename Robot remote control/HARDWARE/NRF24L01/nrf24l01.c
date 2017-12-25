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
//};  // ����һ����̬���͵�ַ ͨ��1

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
/****��Ĵ���reg��һ���ֽڣ�ͬʱ����״̬�ֽ�**************/
u8 SPI_Read_Reg(u8 reg){
	u8 status;
	CSN=0;
	SPI_Send_byte(SPI2,reg);
	status=SPI_Receive_byte(SPI2,0);   //select register  and write value to it
	CSN=1;
	return(status);
}

void NRF24L01_init(char RT_MODE){//0���� 1����
	RCC->APB2ENR|=1<<3;//��ʹ������IO PORTBʱ�� 
	GPIOB->CRH&=0XFFF000FF; 
	GPIOB->CRH|=0X00033800;//PB15 14������� PA13�������� 
	GPIOB->BSRR|=1<<10;
	CE=0;			//nRF24L01_CE=0;			 chip enable
	CSN=1;			//nRF24L01_CSN=1;			 Spi enable
	SPI_Init_MY();
	SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0xff);      // ���TX����IRQ����
	SPI_RW_Reg(FLUSH_TX,0);//������ͻ��������� ����
	SPI_RW_Reg(FLUSH_RX,0);//������ͻ��������� ����
	delay_us(100);
	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x1);       // ʹ�ܽ���ͨ��0�Զ�Ӧ�� T
	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x1);   // ʹ�ܽ���ͨ��0 T
	SPI_RW_Reg(WRITE_REG_NRF24L01 + SETUP_RETR, 0x14);  // �Զ��ط���ʱ�ȴ�750us+86us���Զ��ط�10�� //���ط� R
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_CH, 0x40);       // ѡ����Ƶͨ��0x40 T
//	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x0f);    // ���ݴ�����1Mbps�����书��0dBm���������Ŵ������� T
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x07);    // ���ݴ�����1Mbps�����书��0dBm���������Ŵ������� T
	

	if(RT_MODE){//1����
//		SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0 , RX_LEN);  // ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ�� 
		SPI_Write_Buf(WRITE_REG_NRF24L01 + TX_ADDR, (u8 *)TX_ADDRESS, TX_ADR_WIDTH);     // д�뷢�͵�ַ //������ַ(���ն˵�ʶ����)
		SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, (u8 *)TX_ADDRESS, TX_ADR_WIDTH);  // Ϊ��Ӧ������豸������ͨ��0��ַ�ͷ��͵�ַ��ͬ
		SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, TX_LEN);  // ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ��
	}
	else{
		SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, (u8 *)TX_ADDRESS, TX_ADR_WIDTH);  // �����豸����ͨ��0ʹ�úͷ����豸��ͬ�ķ��͵�ַ
		SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, RX_LEN);  // ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ�� 
	}
}

/****��Ĵ���regдһ���ֽڣ�ͬʱ����״̬�ֽ�**************/
u8 SPI_RW_Reg(u8 reg,u8 value){
	u8 status;
	CSN=0;
	status=SPI_Receive_byte(SPI2,reg);   //select register  and write value to it
	SPI_Send_byte(SPI2,value);   
	CSN=1;
	return(status); 
}


/********����bytes�ֽڵ�����*************************/
u8 SPI_Read_Buf(u8 reg,u8 *pBuf,u8 bytes){
	u8 status,byte_ctr;
	CSN=0;
	status=SPI_Receive_byte(SPI2,reg);       
	for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
		pBuf[byte_ctr]=SPI_Receive_byte(SPI2,0);
	CSN=1;
	return(status);
}

/****************д��bytes�ֽڵ�����*******************/
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
	//�������մ��벻ͬ
	SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0xff);      // ���TX����IRQ���ͣ�	
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, RX_LEN);  // ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ��
	SPI_RW_Reg(FLUSH_RX,0);//������ͻ��������� ����
	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0b);              // CRCʹ�ܣ�16λCRCУ�飬�ϵ磬����ģʽ
  CE=1;
}

void NRF24L01_Send(u8 * tx_buf){
	LED_G=0;
		CE=0;
//��ν���͵�ַ,�����ǹ㲥����,���ն˽��������ݽ��յ�����й��˲���
	SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0xff);      // ���TX����IRQ���ͣ�	
	SPI_RW_Reg(FLUSH_TX,0);//������ͻ��������� ����
	spi_delay_us(20);
	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_LEN); 			 // װ������	������񱻹̶�32 ���ܸ� ���Ǳ߳�ʼ��Ӱ��
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, TX_LEN);  // ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ��

	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0a);      // CRCʹ�ܣ�16λCRCУ�飬�ϵ�	
	
	CE=1;//�ж�������
}



