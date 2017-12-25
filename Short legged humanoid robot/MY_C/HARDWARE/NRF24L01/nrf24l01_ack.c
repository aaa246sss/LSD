//2013-10-28���Զ෢һ������,����:
//�̶������շ�,ʹ��ACK����ʹ��ACK
//�ɱ䳤���շ�,ʹ��ACK����ʹ��ACK,�Լ�ʹ�ô��ش����ݵ�ACK
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "nRF24L01_ack.h"
#include "stdio.h"
#define nRF24_SPI_Send_Byte SPI2_Send_byte
//#define DYNAMIC_PAYLOAD

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define debug_out(x)		//printf x
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void delay_us(uint32_t n);
/* Private functions ---------------------------------------------------------*/
ACK_PAYLOAD nRF24L01_ack_pay;

uint8_t nRF24L01_TxBuf[32]={0};
uint8_t nRF24L01_RxBuf[32]={0};

//���ջ�
uint8_t RX_ADDRESS0[RX_ADR_WIDTH] = {0x30, 0x37, 0x33, 0x34, 0x30};	// �ŵ�0 = TXDEV0_TX_ADDR = DEV1_RX_ADDRESS0{0x30, 0x37, 0x33, 0x34, 0x30}
uint8_t RX_ADDRESS1[RX_ADR_WIDTH] = {0x31, 0x36, 0x32, 0x34, 0x30};	// �ŵ�1 = TXDEV1_TX_ADDR = DEV1_RX_ADDRESS0{0x31, 0x37, 0x33, 0x34, 0x30}
uint8_t RX_ADDRESS2[1]= {0x32};										// �ŵ�2 = TXDEV2_TX_ADDR = DEV1_RX_ADDRESS0{0x32, 0x37, 0x33, 0x34, 0x30}
uint8_t RX_ADDRESS3[1]= {0x33};										// �ŵ�3 = TXDEV3_TX_ADDR = DEV1_RX_ADDRESS0{0x33, 0x37, 0x33, 0x34, 0x30}
uint8_t RX_ADDRESS4[1]= {0x34};										// �ŵ�4 = TXDEV4_TX_ADDR = DEV1_RX_ADDRESS0{0x34, 0x37, 0x33, 0x34, 0x30}
uint8_t RX_ADDRESS5[1]= {0x35};										// �ŵ�5 = TXDEV5_TX_ADDR = DEV1_RX_ADDRESS0{0x35, 0x37, 0x33, 0x34, 0x30}

uint8_t tx_channel_addr[RX_ADR_WIDTH] = {0x30, 0x36, 0x32, 0x34, 0x30};

uint8_t SPI2_Send_byte(uint8_t data){
	while(!(SPI2->SR&(1<<1)));
	SPI2->DR = data;
	while(!(SPI2->SR&(1<<0)));
	return SPI2->DR;
}

#ifdef nRF24_SPI_Send_Byte
#else//ʹ��ģ��SPI
uint8_t nRF24_SPI_Send_Byte(uint8_t data)
{
	uint8_t bit_ctr;
   	for(bit_ctr = 0; bit_ctr < 8; bit_ctr++)					// output 8-bit
   	{
		if(data & 0x80)											// output 'uint8_t', MSB to MOSI
		{
			GPIO_PORT_nRF_SPI->BSRR = GPIO_Pin_nRF_MOSI;
		}
		else
		{
			GPIO_PORT_nRF_SPI->BRR = GPIO_Pin_nRF_MOSI;
		}
		GPIO_PORT_nRF_SPI->BSRR = GPIO_Pin_nRF_SCK;
		data = (data << 1);										// shift next bit into MSB..
		data |= GPIO_ReadInputDataBit(GPIO_PORT_nRF_SPI, GPIO_Pin_nRF_MISO);	// capture current MISO bit
		GPIO_PORT_nRF_SPI->BRR = GPIO_Pin_nRF_SCK;
   	}
    return(data);							// return read uint8_t
}

#endif
void delay_us(uint32_t n)
{
	uint8_t i;

	while(n--)
	{
		i = 8;
		while(i--);
	}
}


uint8_t nRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;

	CSN_L();						// CSN low, initialize SPI communication...
	nRF24_SPI_Send_Byte(reg);		// Select register to read from..
	reg_val = nRF24_SPI_Send_Byte(0xff);// ..then read registervalue
	CSN_H();						// CSN high, terminate SPI communication

	return(reg_val);				// return register value
}

uint8_t nRF24L01_Write_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status;

	CSN_L();						// CSN low, init SPI transaction
	status = nRF24_SPI_Send_Byte(reg);// select register
	nRF24_SPI_Send_Byte(value);		// ..and write value to it..
	CSN_H();						// CSN high again

	return(status);					// return nRF24L01 status uint8_t
}

uint8_t nRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t Len)
{
	uint8_t status, i;

	CSN_L();							// Set CSN low, init SPI tranaction
	status = nRF24_SPI_Send_Byte(reg);	// Select register to write to and read status uint8_t

	for(i = 0; i < Len; i++)
	{
		pBuf[i] = nRF24_SPI_Send_Byte(0xff);
	}

	CSN_H();                           

	return(status);						// return nRF24L01 status uint8_t
}

uint8_t nRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t Len)
{
	uint8_t status, i;

	CSN_L();				//SPIʹ��       
	status = nRF24_SPI_Send_Byte(reg);   
	for(i = 0; i < Len; i++)//
	{
		nRF24_SPI_Send_Byte(pBuf[i]);
	}
	CSN_H();				//�ر�SPI
	return(status);			// 
}

//==============================
void nRF24L01_Set_Config(uint8_t enaa, uint8_t rxaddr, uint8_t petr, uint8_t rfch, uint8_t rf_set, uint8_t config)
{
	uint8_t bufbuf [10];
	CE_L();
 	
#if nRF_PTX		//���������(�����ͨ����ָ����ڽ��ջ��Ľ���ͨ����)
	if((nRF_TX_Channel == 0))
	{
		nRF24L01_Write_Buf(nRF_WRITE_REG | TX_ADDR,    RX_ADDRESS0, RX_ADR_WIDTH);//2-5ͨ��ʹ��1�ֽ�
		nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P0, RX_ADDRESS0, RX_ADR_WIDTH);// д���ն˵�ַ
	}
	else
	{
		tx_channel_addr[0] = 0x30 + nRF_TX_Channel;
		nRF24L01_Write_Buf(nRF_WRITE_REG | TX_ADDR,    tx_channel_addr, RX_ADR_WIDTH);//2-5ͨ��ʹ��1�ֽ�
		nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P0, tx_channel_addr, RX_ADR_WIDTH);// д���ն˵�ַ
	}
#else			//���ջ�����
	nRF24L01_Write_Buf(nRF_WRITE_REG | TX_ADDR,    RX_ADDRESS0, RX_ADR_WIDTH);	// д���͵�ַ=�������ն�0��ַ
	nRF24L01_Read_Buf(TX_ADDR,bufbuf,5);
	nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P0, RX_ADDRESS0, RX_ADR_WIDTH);	// дͨ��0��ַ
	nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P1, RX_ADDRESS1, RX_ADR_WIDTH);	// дͨ��1��ַ
	nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P2, RX_ADDRESS2, 1);				// дͨ��2��ַ
	nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P3, RX_ADDRESS3, 1);				// дͨ��3��ַ
	nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P4, RX_ADDRESS4, 1);				// дͨ��4��ַ
	nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P5, RX_ADDRESS5, 1);				// дͨ��5��ַ
#endif

	nRF24L01_Write_Reg(nRF_WRITE_REG | EN_AA,       enaa         );				// Ƶ��0�Զ�	ACKӦ������	
	nRF24L01_Write_Reg(nRF_WRITE_REG | EN_RXADDR,   rxaddr       );				// ������յ�ַֻ��Ƶ��0�������Ҫ��Ƶ�����Բο�Page21  
	
	nRF24L01_Write_Reg(nRF_WRITE_REG | SETUP_RETR,  petr         );				// �����Զ��ط�ʱ��ʹ�����500us + 86us, 10 retrans...
 	nRF24L01_Write_Reg(nRF_WRITE_REG | RF_CH,       rfch         );				// �����ŵ�����Ϊ2.4GHZ���շ�����һ��
	nRF24L01_Write_Reg(nRF_WRITE_REG | RF_SETUP,    rf_set       );				// ���÷�������Ϊ1MHZ�����书��Ϊ���ֵ0dB
 	nRF24L01_Write_Reg(nRF_WRITE_REG | CONFIG,      config       );	

#if DYNAMIC_PAYLOAD
	if((config & 0x01) == 0)//������ģʽ��Ҫ���������ģʽ�²��������
	{
		nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_TX, 0xff);
		nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_RX, 0xff);
	}
	
	nRF24L01_Write_Reg(nRF_WRITE_REG | FEATURE, 0x07);	//��̬�غ�
	if(nRF24L01_Read_Reg(FEATURE) == 0x00)				//��鶯̬�غ��Ƿ�ʹ��
	{
		//nRF24L01_Write_Reg(nRF_WRITE_REG | Activate, Code_Activate);	//ʹ��
		nRF24L01_Write_Reg(Activate, Code_Activate);	//ʹ��
	}
	#if nRF_PTX
	nRF24L01_Write_Reg(nRF_WRITE_REG | DYNPD, 0x01);	//ͨ��ʹ��
	#else
	nRF24L01_Write_Reg(nRF_WRITE_REG | DYNPD, 0x3f);	//ͨ��ʹ��
	#endif
	nRF24L01_Write_Reg(nRF_WRITE_REG | FEATURE, 0x07);	//��̬�غ�
#else
	nRF24L01_Write_Reg(nRF_WRITE_REG | RX_PW_P0, RX_PLOAD_WIDTH);//ָ���������ݳ���
	#if !nRF_PTX
	nRF24L01_Write_Reg(nRF_WRITE_REG | RX_PW_P1, RX_PLOAD_WIDTH);//ָ���������ݳ���
	nRF24L01_Write_Reg(nRF_WRITE_REG | RX_PW_P2, RX_PLOAD_WIDTH);//ָ���������ݳ���
	nRF24L01_Write_Reg(nRF_WRITE_REG | RX_PW_P3, RX_PLOAD_WIDTH);//ָ���������ݳ���
	nRF24L01_Write_Reg(nRF_WRITE_REG | RX_PW_P4, RX_PLOAD_WIDTH);//ָ���������ݳ���
	nRF24L01_Write_Reg(nRF_WRITE_REG | RX_PW_P5, RX_PLOAD_WIDTH);//ָ���������ݳ���
	#endif
#endif
	
	CE_H();	// Set CE pin high to enable RX device
	delay_us(130);
//	CE_L();
}
uint8_t nRF24L01_Check(void)			//���24L01�Ƿ���ڣ�����ֵ:0���ɹ�;1��ʧ��
{
	uint8_t buf[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
	uint8_t i;
	
	nRF24L01_Write_Buf(nRF_WRITE_REG | TX_ADDR, buf, 5);	//д��5���ֽڵĵ�ַ.
	nRF24L01_Read_Buf(nRF_READ_REG + TX_ADDR, buf, 5);		//����д��ĵ�ַ
	for(i = 0; i < 5; i++)
	{
		if(buf[i] != 0xA5)
		{
			break;
		}
	}
	if(i != 5)
	{
		debug_out(("nRF24L01 TEST FAUSE\r\n"));
		return 1;											//���24L01����
	}
	else 
	{
		debug_out(("nRF24L01 TEST OK\r\n"));
		return 0;											//��⵽24L01
	}
}

void nRF24L01_Set_Mode(uint8_t rx_mode)	//�޸�Ϊ����/����ģʽ
{
	uint8_t mode = nRF24L01_Read_Reg(CONFIG);
	//debug_out(("mode=0x%02x\r\n",mode));
	if(rx_mode)	//1����ģʽ
	{
	 	nRF24L01_Write_Reg(nRF_WRITE_REG | CONFIG, mode | 0x01);	
	}
	else		//0����ģʽ
	{
	 	nRF24L01_Write_Reg(nRF_WRITE_REG | CONFIG, mode & 0xfe);	
	}
}
//==============================
uint8_t nRF24L01_RxPacket(uint8_t * rx_buf, uint8_t *channel)
{
	uint8_t len = 0;
	uint8_t status;
	
	CE_L();
	status = nRF24L01_Read_Reg(STATUS);							// ��ȡ״̬�Ĵ������ж����ݽ���״��
//	nRF24L01_Write_Reg(nRF_WRITE_REG | STATUS, status & RX_DR);			//���յ����ݺ�RX_DR,TX_DS,MAX_RT���ø�Ϊ1��ͨ��д1������жϱ�־
	nRF24L01_Write_Reg(nRF_WRITE_REG | STATUS, status);			//���յ����ݺ�RX_DR,TX_DS,MAX_RT���ø�Ϊ1��ͨ��д1������жϱ�־
	
#if (DYNPD_ACK_DATA)
	if(status & TX_DS)//
	{
		//debug_out(("PRX TX_DS\r\n"));
	}
#endif
	if(status & RX_DR)											// �ж��Ƿ���յ�����
	{
		len = nRF24L01_Read_Reg(nRF_R_RX_PL_WID);
		if(len < 33)
		{
			nRF24L01_Read_Buf(nRF_R_RX_PLOAD, rx_buf, len);	// read receive payload from RX_FIFO buffer
		}
		else 
		{
			len = 0;
		}
		nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_RX,0xff);	//��ջ�����
		
		*channel = (status & 0x0e) >> 1;
	}
	
	CE_H();	//�ø�CE���������ݷ���
	return len;
}
uint8_t nRF24L01_TxPacket(uint8_t tx_channel, uint8_t * tx_buf, uint8_t len)
{
	CE_L();
	
	if((tx_channel == 0))
	{
		nRF24L01_Write_Buf(nRF_WRITE_REG | TX_ADDR,    RX_ADDRESS0, RX_ADR_WIDTH);//2-5ͨ��ʹ��1�ֽ�
		nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P0, RX_ADDRESS0, RX_ADR_WIDTH);// д���ն˵�ַ
	}
	else
	{
		tx_channel_addr[0] = 0x30 + tx_channel;
		nRF24L01_Write_Buf(nRF_WRITE_REG | TX_ADDR,    tx_channel_addr, RX_ADR_WIDTH);//2-5ͨ��ʹ��1�ֽ�
		nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P0, tx_channel_addr, RX_ADR_WIDTH);// д���ն˵�ַ
	}
#if DYNAMIC_PAYLOAD_ACK
	nRF24L01_Write_Buf(nRF_WRITE_REG | nRF_W_TX_PLOAD, tx_buf, len);	// װ������	
#else
	//nRF24L01_Write_Buf(nRF_WRITE_REG | nRF_W_TX_PLOAD, tx_buf, len);	// װ������	
	nRF24L01_Write_Buf(nRF_WRITE_REG | nRF_W_TX_PAYLOAD_NOACK, tx_buf, len);	// װ������
#endif
	
	CE_H();	//�ø�CE���������ݷ���
	delay_us(20);
	
	while(nRF_IRQ());										//�ȴ��������
	
	return (nRF24L01_Tx_Ack(&nRF24L01_ack_pay));
}
uint8_t nRF24L01_Tx_Ack(ACK_PAYLOAD *ack_pay)
{
 	uint8_t tmp = 200;
	uint8_t status;
	while(1)
	{
		status = nRF24L01_Read_Reg(STATUS);					//��ȡ״̬�Ĵ�����ֵ
		
		nRF24L01_Write_Reg(nRF_WRITE_REG | STATUS, status);	// ���TX_DS��MAX_RT�жϱ�־
		
#if (DYNPD_ACK_DATA)
		if(status & RX_DR)		// �ж��Ƿ���յ�����
		{
			ack_pay->Ack_Len = nRF24L01_Read_Reg(nRF_R_RX_PL_WID);
			if(ack_pay->Ack_Len < 33)
			{
				nRF24L01_Read_Buf(nRF_R_RX_PLOAD, ack_pay->Ack_Buf, ack_pay->Ack_Len);	// read receive payload from RX_FIFO buffer
				ack_pay->Ack_Status = 1;
			}
			else 
			{
				ack_pay->Ack_Len = 0;
			}
			nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_RX,0xff);	//��ջ�����
			
			ack_pay->Ack_Channel = (status & 0x0e) >> 1;
			debug_out(("PTX RX_DR\r\n"));
		}
#endif
		if(status & MAX_RT)		//�ﵽ����ط�����
		{
			nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_TX, 0xff);//���TX FIFO�Ĵ��� 
			debug_out(("PTX MAX_RT\r\n"));
			return MAX_RT;
		}
		else if(status & TX_DS)	//�������
		{
			debug_out(("PTX TX_DS\r\n"));
			return TX_DS;
		}
		else					//�ȴ�
		{
			delay_us(10);
			tmp--;
			if(tmp == 0)
			{
				nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_TX, 0xff);//���TX FIFO�Ĵ��� 
				debug_out(("PTX time_out\r\n"));
				return 0xff;//����ԭ����ʧ��
			}
		}
	}
}
#if (DYNPD_ACK_DATA)//�����ݵ�ACK����
void nRF24L01_Rx_AckPayload(ACK_PAYLOAD ack_pay)
{
	nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_TX, 0xff);//���TX FIFO�Ĵ��� 
	nRF24L01_Write_Buf(nRF_WRITE_REG | nRF_W_ACK_PAYLOAD | ack_pay.Ack_Channel, ack_pay.Ack_Buf, ack_pay.Ack_Len);// װ������
	//debug_out(("���ش�����ACK\r\n"));
}
#endif
//-------------ʹ��˵��---------------------//
// 1.1.PTX�������ݵ�PRX
// #if (DYNAMIC_PAYLOAD) //��̬���ݳ���
// 		nRF24L01_TxPacket(channel, tmp_data, tmp_len);
// #else                 //�̶����ݳ���
// 		nRF24L01_TxPacket(channel, tmp_data, RX_PLOAD_WIDTH);
// #endif
// 1.2.PTX��PRX���շ���ACK����
// #if (DYNPD_ACK_DATA)  //���շ�������
//     if(nRF24L01_ack_pay.Ack_Status)
//     {
//         nRF24L01_ack_pay.Ack_Status = 0;
//         debug_out(("[%d]ACK(%d):%s\r\n",nRF24L01_ack_pay.Ack_Channel,nRF24L01_ack_pay.Ack_Len,nRF24L01_ack_pay.Ack_Buf));
//     }
// #endif
// 2.PRX����PTX������������
// tmp_len = nRF24L01_RxPacket(data, &channel);//��������
// if(tmp_len)
// {
// #if (DYNPD_ACK_DATA)//���ͷ����ACK����
//     nRF24L01_Rx_AckPayload(nRF24L01_ack_pay);
// #endif
// 		debug_out(("���յ�[%d]�ֽ� @%d:%s\r\n", tmp_len, channel, data));
// }
//-----------------------------------------//
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
