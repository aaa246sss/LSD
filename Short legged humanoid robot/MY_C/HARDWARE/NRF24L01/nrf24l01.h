/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __nRF24L01_H
#define __nRF24L01_H
//#include "sys.h"
// CE Pin & CSN Pin & IRQ Pin
#include "stm32f4xx_hal.h"
#include "MAIN_MY.h"

#define	RX_DR			0x40
#define	TX_DS			0x20
#define	MAX_RT			0x10

//#define CHANNEL6 //1~6

//#ifdef CHANNEL1  //0~5
//		#define CHANNEL 0 //0~5
//		#define SYNC_DELAY 0
//#elif defined CHANNEL2
//		#define CHANNEL 1 //0~5
//		#define SYNC_DELAY 25
//#elif defined CHANNEL3
//		#define CHANNEL 2 //0~5
//		#define SYNC_DELAY 50
//#elif defined CHANNEL4		
//		#define CHANNEL 3 //0~5
//		#define SYNC_DELAY 75
//#elif defined CHANNEL5		
//		#define CHANNEL 4 //0~5
//		#define SYNC_DELAY 100
//#else	
//		#define CHANNEL 5 //0~5
//		#define SYNC_DELAY 125
//#endif

//#define CE(x)					x ? GPIOBsetbit(11,1) : GPIOBsetbit(11,0)
#define CE(x)						x ? HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET)
#define CSN(x)					x ? HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET)
#define IRQ							(GPIOB->IDR&(1<<8))

#define CE_2(x)						x ? HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET)
#define CSN_2(x)					x ? HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET)
#define IRQ_2							(GPIOB->IDR&(1<<12))

// SPI(nRF24L01) commands
#define READ_REG_NRF24L01    	0x00 				// Define read command to register
#define WRITE_REG_NRF24L01   	0x20 				// Define write command to register
#define RD_RX_PLOAD 			0x61 				// Define RX payload register address
#define WR_TX_PLOAD 			0xA0 				// Define TX payload register address
#define FLUSH_TX    			0xE1 				// Define flush TX register command
#define FLUSH_RX    			0xE2 				// Define flush RX register command
#define REUSE_TX_PL 			0xE3 				// Define reuse TX payload register command
#define NOP         			0xFF 				// Define No Operation, might be used to read status register
//=======补充参数
	#define Activate			0x50
	#define Code_Activate		0x73
	#define nRF_R_RX_PL_WID		0x60	//读RX FIFO顶层R_RX_PLOAD载荷长度,若超出32字节则清除RX FIFO
	#define nRF_W_ACK_PAYLOAD	0xA8//+ppp	//RX模式下,将PIPEppp(ppp的范围为000~101)中的AcK包与载荷一起写入,最多可以有3个挂起的ACK包.
	#define nRF_W_TX_PAYLOAD_NOACK	0xB0//发送数据,但禁止自动应答,发射模式用
	
//==============================

//***************************************************//
// SPI(nRF24L01) registers(addresses)
#define CONFIG      			0x00				// 'Config' register address
#define EN_AA       			0x01                		// 'Enable Auto Acknowledgment' register address
#define EN_RXADDR   			0x02                		// 'Enabled RX addresses' register address
#define SETUP_AW    			0x03                		// 'Setup address width' register address
#define SETUP_RETR  			0x04                		// 'Setup Auto. Retrans' register address
#define RF_CH       			0x05                		// 'RF channel' register address
#define RF_SETUP    			0x06 				// 'RF setup' register address
#define STATUS      			0x07 				// 'Status' register address
#define OBSERVE_TX  			0x08 				// 'Observe TX' register address
#define CD          			0x09 				//'Carrier Detect' register address
#define RX_ADDR_P0  			0x0A				// 'RX address pipe0' register address
#define RX_ADDR_P1  			0x0B 				// 'RX address pipe1' register address
#define RX_ADDR_P2  			0x0C 				// 'RX address pipe2' register address
#define RX_ADDR_P3  			0x0D 				// 'RX address pipe3' register address
#define RX_ADDR_P4  			0x0E 				// 'RX address pipe4' register address
#define RX_ADDR_P5  			0x0F				// 'RX address pipe5' register address
#define TX_ADDR     			0x10 				// 'TX address' register address
#define RX_PW_P0    			0x11 				// 'RX payload width, pipe0' register address
#define RX_PW_P1    			0x12 				// 'RX payload width, pipe1' register address
#define RX_PW_P2    			0x13 				// 'RX payload width, pipe2' register address
#define RX_PW_P3    			0x14 				// 'RX payload width, pipe3' register address
#define RX_PW_P4    			0x15 				// 'RX payload width, pipe4' register address
#define RX_PW_P5    			0x16 				// 'RX payload width, pipe5' register address
#define FIFO_STATUS 			0x17 			    	// 'FIFO Status Register' register address

//=======补充参数
	#define DYNPD				0x1C  // nRF24L01 Dynamic payload setup
	#define FEATURE				0x1D  // nRF24L01 Exclusive feature setup
//==============================

#define TX_ADR_WIDTH   	5  // 5字节宽度的发送/接收地址
#define RX_LEN 	12  // 数据通道有效数据宽度
#define TX_LEN  1


//void NRF_receive(void);
uint8_t SPI_Read_Reg(uint8_t reg);
void delay1us(uint8_t t);
void SPI_Send_byte(SPI_TypeDef* SPIx,uint8_t data);
uint8_t SPI_Receive_byte(SPI_TypeDef* SPIx,uint8_t data);



void NRF24L01_init(void);
uint8_t SPI_RW_Reg(uint8_t reg,uint8_t value);
uint8_t SPI_Write_Reg(uint8_t reg,uint8_t value);
uint8_t SPI_Read_Buf(uint8_t reg,char *pBuf,uint8_t bytes);
uint8_t SPI_Write_Buf(uint8_t reg,uint8_t *pBuf,uint8_t bytes);
void NRF_RX_Mode(GLOVAR_HandleTypeDef *GloVar,char buf_len);
void NRF24L01_Send(GLOVAR_HandleTypeDef *GloVar);

void NRF24L01_init_2(void);
uint8_t SPI_Read_Reg_2(uint8_t reg);
uint8_t SPI_RW_Reg_2(uint8_t reg,uint8_t value);
uint8_t SPI_Write_Reg_2(uint8_t reg,uint8_t value);
uint8_t SPI_Read_Buf_2(uint8_t reg,char *pBuf,uint8_t bytes);
uint8_t SPI_Write_Buf_2(uint8_t reg,uint8_t *pBuf,uint8_t bytes);
void NRF_RX_Mode_2(GLOVAR_HandleTypeDef *GloVar,char buf_len);
void NRF24L01_Send_2(uint8_t *buf,uint8_t buf_len);
int NRF_printf(char *pcFormat, ...);





void NRF24L01_init_2(void);
#endif
//==============================
