#include "stm32f4xx_hal.h"
#include "myspi.h"
#include "nrf24l01.h"
#include "delay.h"
#include <stdio.h>
#include <string.h>

//#define DYNAMIC_PAYLOAD			1//ʹ�ö�̬�������ݳ���[�������ʱ,���ջ����Ծ�̬���շ�����Ķ�̬����]
//#define DYNAMIC_PAYLOAD_ACK	1//ʹ�ö�̬���ݳ���ʱACK����
////#define DYNPD_ACK_DATA		1//ʹ�ö�̬���ݳ���ʱ������ACK����

extern SPI_HandleTypeDef hspi3;
const uint8_t NRF_TX_ADDRESS[TX_ADR_WIDTH]={101,23,65,86,88};
	
void SPI_Send_byte(SPI_TypeDef* SPIx,uint8_t data){
	while(!(SPIx->SR&(1<<1)));
	SPIx->DR = data;
	while(!(SPIx->SR&(1<<0)));
	SPIx->DR;
}

uint8_t SPI_Receive_byte(SPI_TypeDef* SPIx,uint8_t data){
	while(!(SPIx->SR&(1<<1)));
	SPIx->DR = data;
	while(!(SPIx->SR&(1<<0)));
	return SPIx->DR;
}
/****��Ĵ���reg��һ���ֽڣ�ͬʱ����״̬�ֽ�**************/
uint8_t SPI_Read_Reg(uint8_t reg){
	uint8_t status;
	CSN(0);
	SPI_Send_byte(SPI3,reg);
	status=SPI_Receive_byte(SPI3,0);   //select register  and write value to it
	CSN(1);
	return(status);
}

void NRF24L01_init(void){//0���� 1����
	char read_buf[10];
	SPI3->CR1|=1<<6;//ʹ��SPI3
	CE(0);			//nRF24L01_CE=0;			 chip enable
	CSN(1);			//nRF24L01_CSN=1;			 Spi enable
	
	SPI_RW_Reg(FLUSH_TX,0);//������ͻ��������� ����
	SPI_RW_Reg(FLUSH_RX,0);//������ͻ��������� ����
	SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0xff);      // ���TX����IRQ����
	delay_us(100);
	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x1);       // ʹ�ܽ���ͨ��0�Զ�Ӧ�� T
	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x1);   // ʹ�����н���ͨ��0 T
	SPI_RW_Reg(WRITE_REG_NRF24L01 + SETUP_RETR, 0x03);  // �Զ��ط���ʱ�ȴ�750us+86us���Զ��ط�10�� //���ط� R
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_CH, 0x40);       // ѡ����Ƶͨ��0x40 T
//	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x0f);    // ���ݴ�����1Mbps�����书��0dBm���������Ŵ������� T
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x07);    // ���ݴ�����1Mbps�����书��0dBm���������Ŵ������� T
//	if(RT_MODE)
#ifdef DYNAMIC_PAYLOAD
#else 
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0 , RX_LEN);  // ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ�� 
#endif
//	else
//	{
	SPI_Write_Buf(WRITE_REG_NRF24L01 + TX_ADDR, (uint8_t *)NRF_TX_ADDRESS, TX_ADR_WIDTH);     // д�뷢�͵�ַ //������ַ(���ն˵�ʶ����)
	SPI_Read_Buf(TX_ADDR,read_buf,5);
	SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, (uint8_t *)NRF_TX_ADDRESS, TX_ADR_WIDTH);  // �����豸����ͨ��0ʹ�úͷ����豸��ͬ�ķ��͵�ַ
//	}
}

void delay1us(uint8_t t){
	while(--t);
} 

/****��Ĵ���regдһ���ֽڣ�ͬʱ����״̬�ֽ�**************/
uint8_t SPI_RW_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;
	CSN(0);
	status=SPI_Receive_byte(SPI3,reg);   //select register  and write value to it
	SPI_Send_byte(SPI3,value);   
	CSN(1);
	return(status); 
}


/********����bytes�ֽڵ�����*************************/
uint8_t SPI_Read_Buf(uint8_t reg,char *pBuf,uint8_t bytes)
{
	uint8_t status,byte_ctr;
	CSN(0);
	status=SPI_Receive_byte(SPI3,reg);       
	for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
		pBuf[byte_ctr]=SPI_Receive_byte(SPI3,0);
	CSN(1);
	return(status);
}

/****************д��bytes�ֽڵ�����*******************/
uint8_t SPI_Write_Buf(uint8_t reg, uint8_t *pBuf,uint8_t bytes)
{
	uint8_t status,byte_ctr;
	CSN(0);
	status=SPI_Receive_byte(SPI3,reg); 
	delay1us(1);
	for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
		SPI_Send_byte(SPI3,*pBuf++);
	CSN(1);
	return(status);
}

void NRF_RX_Mode(GLOVAR_HandleTypeDef *GloVar,char buf_len)
{
//	GloVar->NRF.MODE=NRF_MODE_RX;
		CE(0);
	SPI_RW_Reg(FLUSH_RX,0);//������ͻ��������� ����
	SPI_RW_Reg(FLUSH_TX,0);//������ͻ��������� ����
	SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0xff);      // ���TX����IRQ���ͣ�
	delay1us(1);
#ifdef DYNAMIC_PAYLOAD
#else 
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, RX_LEN);  // ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ��
#endif
	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0b);              // CRCʹ�ܣ�16λCRCУ�飬�ϵ磬����ģʽ
  	CE(1);
}

void NRF24L01_Send(GLOVAR_HandleTypeDef *GloVar)
{
//	 uint8_t status=0x00;
//	GloVar->NRF.MODE=NRF_MODE_TX;	
	CE(0);
//��ν���͵�ַ,�����ǹ㲥����,���ն˽��������ݽ��յ�����й��˲���
//	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0);      // CRCʹ�ܣ�16λCRCУ�飬�ϵ�	
	SPI_RW_Reg(FLUSH_TX,0);//������ͻ��������� ����
	SPI_RW_Reg(FLUSH_RX,0);//������ͻ��������� ����
	SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0xff);      // ���TX����IRQ���ͣ�
	delay1us(2);
#ifdef DYNAMIC_PAYLOAD
#else 
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, TX_LEN);  // ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ��
#endif
	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0a);      // CRCʹ�ܣ�16λCRCУ�飬�ϵ�	
	GloVar->NRF.BUF_TX[0]=TIM1->CNT;
	SPI_Write_Buf(WR_TX_PLOAD, GloVar->NRF.BUF_TX, TX_LEN); 			 // װ������	
//	GloVar->NRF.FLAG_TX_OK=0;
	CE(1);//�ж�������
}

