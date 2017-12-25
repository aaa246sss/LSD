//2013-10-28测试多发一收正常,包括:
//固定长度收发,使用ACK及不使用ACK
//可变长度收发,使用ACK及不使用ACK,以及使用带回传数据的ACK
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

//接收机
uint8_t RX_ADDRESS0[RX_ADR_WIDTH] = {0x30, 0x37, 0x33, 0x34, 0x30};	// 信道0 = TXDEV0_TX_ADDR = DEV1_RX_ADDRESS0{0x30, 0x37, 0x33, 0x34, 0x30}
uint8_t RX_ADDRESS1[RX_ADR_WIDTH] = {0x31, 0x36, 0x32, 0x34, 0x30};	// 信道1 = TXDEV1_TX_ADDR = DEV1_RX_ADDRESS0{0x31, 0x37, 0x33, 0x34, 0x30}
uint8_t RX_ADDRESS2[1]= {0x32};										// 信道2 = TXDEV2_TX_ADDR = DEV1_RX_ADDRESS0{0x32, 0x37, 0x33, 0x34, 0x30}
uint8_t RX_ADDRESS3[1]= {0x33};										// 信道3 = TXDEV3_TX_ADDR = DEV1_RX_ADDRESS0{0x33, 0x37, 0x33, 0x34, 0x30}
uint8_t RX_ADDRESS4[1]= {0x34};										// 信道4 = TXDEV4_TX_ADDR = DEV1_RX_ADDRESS0{0x34, 0x37, 0x33, 0x34, 0x30}
uint8_t RX_ADDRESS5[1]= {0x35};										// 信道5 = TXDEV5_TX_ADDR = DEV1_RX_ADDRESS0{0x35, 0x37, 0x33, 0x34, 0x30}

uint8_t tx_channel_addr[RX_ADR_WIDTH] = {0x30, 0x36, 0x32, 0x34, 0x30};

uint8_t SPI2_Send_byte(uint8_t data){
	while(!(SPI2->SR&(1<<1)));
	SPI2->DR = data;
	while(!(SPI2->SR&(1<<0)));
	return SPI2->DR;
}

#ifdef nRF24_SPI_Send_Byte
#else//使用模拟SPI
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

	CSN_L();				//SPI使能       
	status = nRF24_SPI_Send_Byte(reg);   
	for(i = 0; i < Len; i++)//
	{
		nRF24_SPI_Send_Byte(pBuf[i]);
	}
	CSN_H();				//关闭SPI
	return(status);			// 
}

//==============================
void nRF24L01_Set_Config(uint8_t enaa, uint8_t rxaddr, uint8_t petr, uint8_t rfch, uint8_t rf_set, uint8_t config)
{
	uint8_t bufbuf [10];
	CE_L();
 	
#if nRF_PTX		//发射机配置(发射机通道号指相对于接收机的接收通道号)
	if((nRF_TX_Channel == 0))
	{
		nRF24L01_Write_Buf(nRF_WRITE_REG | TX_ADDR,    RX_ADDRESS0, RX_ADR_WIDTH);//2-5通道使用1字节
		nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P0, RX_ADDRESS0, RX_ADR_WIDTH);// 写接收端地址
	}
	else
	{
		tx_channel_addr[0] = 0x30 + nRF_TX_Channel;
		nRF24L01_Write_Buf(nRF_WRITE_REG | TX_ADDR,    tx_channel_addr, RX_ADR_WIDTH);//2-5通道使用1字节
		nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P0, tx_channel_addr, RX_ADR_WIDTH);// 写接收端地址
	}
#else			//接收机配置
	nRF24L01_Write_Buf(nRF_WRITE_REG | TX_ADDR,    RX_ADDRESS0, RX_ADR_WIDTH);	// 写发送地址=本机接收端0地址
	nRF24L01_Read_Buf(TX_ADDR,bufbuf,5);
	nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P0, RX_ADDRESS0, RX_ADR_WIDTH);	// 写通道0地址
	nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P1, RX_ADDRESS1, RX_ADR_WIDTH);	// 写通道1地址
	nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P2, RX_ADDRESS2, 1);				// 写通道2地址
	nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P3, RX_ADDRESS3, 1);				// 写通道3地址
	nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P4, RX_ADDRESS4, 1);				// 写通道4地址
	nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P5, RX_ADDRESS5, 1);				// 写通道5地址
#endif

	nRF24L01_Write_Reg(nRF_WRITE_REG | EN_AA,       enaa         );				// 频道0自动	ACK应答允许	
	nRF24L01_Write_Reg(nRF_WRITE_REG | EN_RXADDR,   rxaddr       );				// 允许接收地址只有频道0，如果需要多频道可以参考Page21  
	
	nRF24L01_Write_Reg(nRF_WRITE_REG | SETUP_RETR,  petr         );				// 设置自动重发时间和次数：500us + 86us, 10 retrans...
 	nRF24L01_Write_Reg(nRF_WRITE_REG | RF_CH,       rfch         );				// 设置信道工作为2.4GHZ，收发必须一致
	nRF24L01_Write_Reg(nRF_WRITE_REG | RF_SETUP,    rf_set       );				// 设置发射速率为1MHZ，发射功率为最大值0dB
 	nRF24L01_Write_Reg(nRF_WRITE_REG | CONFIG,      config       );	

#if DYNAMIC_PAYLOAD
	if((config & 0x01) == 0)//发射下模式需要清除，接收模式下不能清除。
	{
		nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_TX, 0xff);
		nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_RX, 0xff);
	}
	
	nRF24L01_Write_Reg(nRF_WRITE_REG | FEATURE, 0x07);	//动态载荷
	if(nRF24L01_Read_Reg(FEATURE) == 0x00)				//检查动态载荷是否使能
	{
		//nRF24L01_Write_Reg(nRF_WRITE_REG | Activate, Code_Activate);	//使能
		nRF24L01_Write_Reg(Activate, Code_Activate);	//使能
	}
	#if nRF_PTX
	nRF24L01_Write_Reg(nRF_WRITE_REG | DYNPD, 0x01);	//通道使能
	#else
	nRF24L01_Write_Reg(nRF_WRITE_REG | DYNPD, 0x3f);	//通道使能
	#endif
	nRF24L01_Write_Reg(nRF_WRITE_REG | FEATURE, 0x07);	//动态载荷
#else
	nRF24L01_Write_Reg(nRF_WRITE_REG | RX_PW_P0, RX_PLOAD_WIDTH);//指定接收数据长度
	#if !nRF_PTX
	nRF24L01_Write_Reg(nRF_WRITE_REG | RX_PW_P1, RX_PLOAD_WIDTH);//指定接收数据长度
	nRF24L01_Write_Reg(nRF_WRITE_REG | RX_PW_P2, RX_PLOAD_WIDTH);//指定接收数据长度
	nRF24L01_Write_Reg(nRF_WRITE_REG | RX_PW_P3, RX_PLOAD_WIDTH);//指定接收数据长度
	nRF24L01_Write_Reg(nRF_WRITE_REG | RX_PW_P4, RX_PLOAD_WIDTH);//指定接收数据长度
	nRF24L01_Write_Reg(nRF_WRITE_REG | RX_PW_P5, RX_PLOAD_WIDTH);//指定接收数据长度
	#endif
#endif
	
	CE_H();	// Set CE pin high to enable RX device
	delay_us(130);
//	CE_L();
}
uint8_t nRF24L01_Check(void)			//检测24L01是否存在，返回值:0，成功;1，失败
{
	uint8_t buf[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
	uint8_t i;
	
	nRF24L01_Write_Buf(nRF_WRITE_REG | TX_ADDR, buf, 5);	//写入5个字节的地址.
	nRF24L01_Read_Buf(nRF_READ_REG + TX_ADDR, buf, 5);		//读出写入的地址
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
		return 1;											//检测24L01错误
	}
	else 
	{
		debug_out(("nRF24L01 TEST OK\r\n"));
		return 0;											//检测到24L01
	}
}

void nRF24L01_Set_Mode(uint8_t rx_mode)	//修改为接收/发射模式
{
	uint8_t mode = nRF24L01_Read_Reg(CONFIG);
	//debug_out(("mode=0x%02x\r\n",mode));
	if(rx_mode)	//1接收模式
	{
	 	nRF24L01_Write_Reg(nRF_WRITE_REG | CONFIG, mode | 0x01);	
	}
	else		//0发送模式
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
	status = nRF24L01_Read_Reg(STATUS);							// 读取状态寄存其来判断数据接收状况
//	nRF24L01_Write_Reg(nRF_WRITE_REG | STATUS, status & RX_DR);			//接收到数据后RX_DR,TX_DS,MAX_RT都置高为1，通过写1来清楚中断标志
	nRF24L01_Write_Reg(nRF_WRITE_REG | STATUS, status);			//接收到数据后RX_DR,TX_DS,MAX_RT都置高为1，通过写1来清楚中断标志
	
#if (DYNPD_ACK_DATA)
	if(status & TX_DS)//
	{
		//debug_out(("PRX TX_DS\r\n"));
	}
#endif
	if(status & RX_DR)											// 判断是否接收到数据
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
		nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_RX,0xff);	//清空缓冲区
		
		*channel = (status & 0x0e) >> 1;
	}
	
	CE_H();	//置高CE，激发数据发送
	return len;
}
uint8_t nRF24L01_TxPacket(uint8_t tx_channel, uint8_t * tx_buf, uint8_t len)
{
	CE_L();
	
	if((tx_channel == 0))
	{
		nRF24L01_Write_Buf(nRF_WRITE_REG | TX_ADDR,    RX_ADDRESS0, RX_ADR_WIDTH);//2-5通道使用1字节
		nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P0, RX_ADDRESS0, RX_ADR_WIDTH);// 写接收端地址
	}
	else
	{
		tx_channel_addr[0] = 0x30 + tx_channel;
		nRF24L01_Write_Buf(nRF_WRITE_REG | TX_ADDR,    tx_channel_addr, RX_ADR_WIDTH);//2-5通道使用1字节
		nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P0, tx_channel_addr, RX_ADR_WIDTH);// 写接收端地址
	}
#if DYNAMIC_PAYLOAD_ACK
	nRF24L01_Write_Buf(nRF_WRITE_REG | nRF_W_TX_PLOAD, tx_buf, len);	// 装载数据	
#else
	//nRF24L01_Write_Buf(nRF_WRITE_REG | nRF_W_TX_PLOAD, tx_buf, len);	// 装载数据	
	nRF24L01_Write_Buf(nRF_WRITE_REG | nRF_W_TX_PAYLOAD_NOACK, tx_buf, len);	// 装载数据
#endif
	
	CE_H();	//置高CE，激发数据发送
	delay_us(20);
	
	while(nRF_IRQ());										//等待发送完成
	
	return (nRF24L01_Tx_Ack(&nRF24L01_ack_pay));
}
uint8_t nRF24L01_Tx_Ack(ACK_PAYLOAD *ack_pay)
{
 	uint8_t tmp = 200;
	uint8_t status;
	while(1)
	{
		status = nRF24L01_Read_Reg(STATUS);					//读取状态寄存器的值
		
		nRF24L01_Write_Reg(nRF_WRITE_REG | STATUS, status);	// 清除TX_DS或MAX_RT中断标志
		
#if (DYNPD_ACK_DATA)
		if(status & RX_DR)		// 判断是否接收到数据
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
			nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_RX,0xff);	//清空缓冲区
			
			ack_pay->Ack_Channel = (status & 0x0e) >> 1;
			debug_out(("PTX RX_DR\r\n"));
		}
#endif
		if(status & MAX_RT)		//达到最大重发次数
		{
			nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_TX, 0xff);//清除TX FIFO寄存器 
			debug_out(("PTX MAX_RT\r\n"));
			return MAX_RT;
		}
		else if(status & TX_DS)	//发送完成
		{
			debug_out(("PTX TX_DS\r\n"));
			return TX_DS;
		}
		else					//等待
		{
			delay_us(10);
			tmp--;
			if(tmp == 0)
			{
				nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_TX, 0xff);//清除TX FIFO寄存器 
				debug_out(("PTX time_out\r\n"));
				return 0xff;//其他原因发送失败
			}
		}
	}
}
#if (DYNPD_ACK_DATA)//带数据的ACK功能
void nRF24L01_Rx_AckPayload(ACK_PAYLOAD ack_pay)
{
	nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_TX, 0xff);//清除TX FIFO寄存器 
	nRF24L01_Write_Buf(nRF_WRITE_REG | nRF_W_ACK_PAYLOAD | ack_pay.Ack_Channel, ack_pay.Ack_Buf, ack_pay.Ack_Len);// 装载数据
	//debug_out(("加载带数据ACK\r\n"));
}
#endif
//-------------使用说明---------------------//
// 1.1.PTX发送数据到PRX
// #if (DYNAMIC_PAYLOAD) //动态数据长度
// 		nRF24L01_TxPacket(channel, tmp_data, tmp_len);
// #else                 //固定数据长度
// 		nRF24L01_TxPacket(channel, tmp_data, RX_PLOAD_WIDTH);
// #endif
// 1.2.PTX从PRX接收反向ACK数据
// #if (DYNPD_ACK_DATA)  //接收反向数据
//     if(nRF24L01_ack_pay.Ack_Status)
//     {
//         nRF24L01_ack_pay.Ack_Status = 0;
//         debug_out(("[%d]ACK(%d):%s\r\n",nRF24L01_ack_pay.Ack_Channel,nRF24L01_ack_pay.Ack_Len,nRF24L01_ack_pay.Ack_Buf));
//     }
// #endif
// 2.PRX接收PTX发送来的数据
// tmp_len = nRF24L01_RxPacket(data, &channel);//接收数据
// if(tmp_len)
// {
// #if (DYNPD_ACK_DATA)//回送反向的ACK数据
//     nRF24L01_Rx_AckPayload(nRF24L01_ack_pay);
// #endif
// 		debug_out(("接收到[%d]字节 @%d:%s\r\n", tmp_len, channel, data));
// }
//-----------------------------------------//
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
