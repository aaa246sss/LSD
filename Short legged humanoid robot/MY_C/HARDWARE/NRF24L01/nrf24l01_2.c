#include "stm32f4xx_hal.h"
#include "myspi.h"
#include "nrf24l01.h"
#include "delay.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>

#define DYNAMIC_PAYLOAD			1//ʹ�ö�̬�������ݳ���[�������ʱ,���ջ����Ծ�̬���շ�����Ķ�̬����]
#define DYNAMIC_PAYLOAD_ACK	1//ʹ�ö�̬���ݳ���ʱACK����
//#define DYNPD_ACK_DATA		1//ʹ�ö�̬���ݳ���ʱ������ACK����


extern SPI_HandleTypeDef hspi2;

const uint8_t NRF_TX_ADDRESS2[TX_ADR_WIDTH]={88,15,78,118,12};
	
/****��Ĵ���reg��һ���ֽڣ�ͬʱ����״̬�ֽ�**************/
uint8_t SPI_Read_Reg_2(uint8_t reg){
	uint8_t status;
	CSN_2(0);
	SPI_Send_byte(SPI2,reg);
	status=SPI_Receive_byte(SPI2,0);   //select register  and write value to it
	CSN_2(1);
	return(status);
}

void NRF24L01_init_2(void){//0���� 1����
	char read_buf[10];
	SPI2->CR1|=1<<6;//ʹ��SPI2
	CE_2(0);			//nRF24L01_CE_2=0;			 chip enable
	CSN_2(1);			//nRF24L01_CSN_2=1;			 Spi enable
	
	SPI_RW_Reg_2(FLUSH_TX,0);//������ͻ��������� ����
	SPI_RW_Reg_2(FLUSH_RX,0);//������ͻ��������� ����
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + STATUS, 0xff);      // ���TX����IRQ����
	delay_us(100);
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + EN_AA, 0x1);       // ʹ�ܽ���ͨ��0�Զ�Ӧ�� T
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + EN_RXADDR, 0x1);   // ʹ�����н���ͨ��0 T
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + SETUP_RETR, 0x03);  // �Զ��ط���ʱ�ȴ�750us+86us���Զ��ط�10�� //���ط� R
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + RF_CH, 0x40);       // ѡ����Ƶͨ��0x40 T
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + RF_SETUP, 0x07);    // ���ݴ�����1Mbps�����书��0dBm���������Ŵ������� T
#ifdef DYNAMIC_PAYLOAD	
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + FEATURE, 0x07);    //��̬�غ�
	if(SPI_Read_Reg_2(FEATURE) == 0x00)				//��鶯̬�غ��Ƿ�ʹ��
		SPI_RW_Reg_2(Activate, Code_Activate);	//ʹ��
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + DYNPD, 0x01);    //ͨ��ʹ��
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + FEATURE, 0x07);    //��̬�غ�
#else
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + RX_PW_P0 , 1);  // ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ�� 
#endif
	SPI_Write_Buf_2(WRITE_REG_NRF24L01 + TX_ADDR, (uint8_t *)NRF_TX_ADDRESS2, TX_ADR_WIDTH);     // д�뷢�͵�ַ //������ַ(���ն˵�ʶ����)
	SPI_Read_Buf_2(TX_ADDR,read_buf,5);
	SPI_Write_Buf_2(WRITE_REG_NRF24L01 + RX_ADDR_P0, (uint8_t *)NRF_TX_ADDRESS2, TX_ADR_WIDTH);  // �����豸����ͨ��0ʹ�úͷ����豸��ͬ�ķ��͵�ַ
}



/****��Ĵ���regдһ���ֽڣ�ͬʱ����״̬�ֽ�**************/
uint8_t SPI_RW_Reg_2(uint8_t reg,uint8_t value)
{
	uint8_t status;
	CSN_2(0);
	status=SPI_Receive_byte(SPI2,reg);   //select register  and write value to it
	SPI_Send_byte(SPI2,value);   
	CSN_2(1);
	return(status); 
}


/********����bytes�ֽڵ�����*************************/
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

/****************д��bytes�ֽڵ�����*******************/
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
	SPI_RW_Reg_2(FLUSH_RX,0);//������ͻ��������� ����
	SPI_RW_Reg_2(FLUSH_TX,0);//������ͻ��������� ����
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + STATUS, 0xff);      // ���TX����IRQ���ͣ�
	delay1us(1);
#ifdef DYNAMIC_PAYLOAD
#else 
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + RX_PW_P0, buf_len);  // ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ��
#endif
	
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + CONFIG, 0x0b);              // CRCʹ�ܣ�16λCRCУ�飬�ϵ磬����ģʽ
  	CE_2(1);
}

//void NRF24L01_Send_2(GLOVAR_HandleTypeDef *GloVar)
//{
////	 uint8_t status=0x00;
////	GloVar->NRF.MODE=NRF_MODE_TX;	
//	CE_2(0);
////��ν���͵�ַ,�����ǹ㲥����,���ն˽��������ݽ��յ�����й��˲���
////	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + CONFIG, 0);      // CRCʹ�ܣ�16λCRCУ�飬�ϵ�	
//	SPI_RW_Reg_2(FLUSH_TX,0);//������ͻ��������� ����
//	SPI_RW_Reg_2(FLUSH_RX,0);//������ͻ��������� ����
//	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + STATUS, 0xff);      // ���TX����IRQ���ͣ�
//	delay1us(2);
//#ifdef DYNAMIC_PAYLOAD
//#else 
//	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + RX_PW_P0, TX_LEN);  // ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ��
//#endif
//	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + CONFIG, 0x0a);      // CRCʹ�ܣ�16λCRCУ�飬�ϵ�	
//	GloVar->NRF.BUF_TX[0]=TIM1->CNT;
//	SPI_Write_Buf_2(WR_TX_PLOAD, GloVar->NRF.BUF_TX, TX_LEN); 			 // װ������	
////	GloVar->NRF.FLAG_TX_OK=0;
//	CE_2(1);//�ж�������
//}

void NRF24L01_Send_2(uint8_t *buf,uint8_t buf_len)
{
//	 uint8_t status=0x00;
//	GloVar->NRF.MODE=NRF_MODE_TX;	
	CE_2(0);
//��ν���͵�ַ,�����ǹ㲥����,���ն˽��������ݽ��յ�����й��˲���
//	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + CONFIG, 0);      // CRCʹ�ܣ�16λCRCУ�飬�ϵ�	
	SPI_RW_Reg_2(FLUSH_TX,0);//������ͻ��������� ����
	SPI_RW_Reg_2(FLUSH_RX,0);//������ͻ��������� ����
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + STATUS, 0xff);      // ���TX����IRQ���ͣ�
	delay1us(2);
#ifdef DYNAMIC_PAYLOAD
#else 
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + RX_PW_P0, TX_LEN);  // ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ��
#endif
	SPI_RW_Reg_2(WRITE_REG_NRF24L01 + CONFIG, 0x0a);      // CRCʹ�ܣ�16λCRCУ�飬�ϵ�	
	SPI_Write_Buf_2(WR_TX_PLOAD, buf, buf_len); 			 // װ������	
//	GloVar->NRF.FLAG_TX_OK=0;
	CE_2(1);//�ж�������
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

