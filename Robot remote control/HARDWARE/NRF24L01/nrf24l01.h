/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __nRF24L01_H
#define __nRF24L01_H
#include "sys.h"
#include "timer.h"

#define RIGHT_HAND_M
//#define RIGHT_HAND_L
//#define LEFT_HAND_M
//#define LEFT_HAND_L

//#define LEFT_FOOD_M
//#define LEFT_FOOD_L
//#define RIGHT_FOOD_M
//#define RIGHT_FOOD_L

#ifdef RIGHT_HAND_M  //0~5
	#define NRF_CHANNEL 0
	#define SYNC_DELAY 0
	#define MPU_6050_read_timer1 10
	#define MPU_6050_read_timer2 60
	#define MPU_6050_read_timer3 110
	#define MPU_6050_read_timer4 160
#elif defined RIGHT_HAND_L
	#define NRF_CHANNEL 1
//	#define SYNC_DELAY 25
	#define SYNC_DELAY 25
	#define MPU_6050_read_timer1 35
	#define MPU_6050_read_timer2 85
	#define MPU_6050_read_timer3 135
	#define MPU_6050_read_timer4 185
#elif defined LEFT_HAND_M
	#define NRF_CHANNEL 2
	#define SYNC_DELAY 50
	#define MPU_6050_read_timer1 60
	#define MPU_6050_read_timer2 110
	#define MPU_6050_read_timer3 160
	#define MPU_6050_read_timer4 10
#elif defined LEFT_HAND_L		
	#define NRF_CHANNEL 3
	#define SYNC_DELAY 75
	#define MPU_6050_read_timer1 35
	#define MPU_6050_read_timer2 85
	#define MPU_6050_read_timer3 135
	#define MPU_6050_read_timer4 185
#elif defined LEFT_FOOD_M		
	#define NRF_CHANNEL 4
	#define SYNC_DELAY 100
	#define MPU_6050_read_timer1 60
	#define MPU_6050_read_timer2 110
	#define MPU_6050_read_timer3 160
	#define MPU_6050_read_timer4 10
#elif defined LEFT_FOOD_L	
	#define NRF_CHANNEL 5
	#define SYNC_DELAY 125
	#define MPU_6050_read_timer1 35
	#define MPU_6050_read_timer2 85
	#define MPU_6050_read_timer3 135
	#define MPU_6050_read_timer4 185
#elif defined RIGHT_FOOD_M		
	#define NRF_CHANNEL 6
	#define SYNC_DELAY 150
	#define MPU_6050_read_timer1 60
	#define MPU_6050_read_timer2 110
	#define MPU_6050_read_timer3 160
	#define MPU_6050_read_timer4 10
#else	
	#define NRF_CHANNEL 7
	#define SYNC_DELAY 175
	#define MPU_6050_read_timer1 35
	#define MPU_6050_read_timer2 85
	#define MPU_6050_read_timer3 135
	#define MPU_6050_read_timer4 185
#endif


#define	RX_DR			0x40
#define	TX_DS			0x20
#define	MAX_RT			0x10
#define RX_LEN 	1  // 数据通道有效数据宽度
#define TX_LEN  12

//#ifdef CHANNEL1  //0~5
//		#define CHANNEL 0 //0~5

//#elif defined CHANNEL2
//		#define CHANNEL 1 //0~5
//		
//#elif defined CHANNEL3
//		#define CHANNEL 2 //0~5
//		
//#elif defined CHANNEL4		
//		#define CHANNEL 3 //0~5
//		
//#elif defined CHANNEL5		
//		#define CHANNEL 4 //0~5
//		
//#else	
//		#define CHANNEL 5 //0~5
//		
//#endif

//void GPIOBsetbit(char BIT,char HL);
//#define CE(x)					x ? GPIOBsetbit(11,1) : GPIOBsetbit(11,0)
#define CE							PBout(11)
//#define CSN(x)					x ? GPIOBsetbit(12,1) : GPIOBsetbit(12,0)
#define CSN							PBout(12)
//#define IRQ					(GPIOB->IDR&(1<<10))
#define IRQ							PBin(10)

// SPI(nRF24L01) commands
#define READ_REG_NRF24L01    	0x00 				// Define read command to register
#define WRITE_REG_NRF24L01   	0x20 				// Define write command to register
#define RD_RX_PLOAD 			0x61 				// Define RX payload register address
#define WR_TX_PLOAD 			0xA0 				// Define TX payload register address
#define FLUSH_TX    			0xE1 				// Define flush TX register command
#define FLUSH_RX    			0xE2 				// Define flush RX register command
#define REUSE_TX_PL 			0xE3 				// Define reuse TX payload register command
#define NOP         			0xFF 				// Define No Operation, might be used to read status register
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

#define TX_ADR_WIDTH   	5  // 5字节宽度的发送/接收地址

void NRF24L01_init(char RT_MODE);
//void NRF_receive(void);
u8 SPI_Write_Reg(u8 reg,u8 value);
u8 SPI_Read_Buf(u8 reg,u8 *pBuf,u8 bytes);
u8 SPI_Write_Buf(u8 reg,u8 *pBuf,u8 bytes);
u8 nRF24L01_RxPacket(unsigned char *rx_buf);
void nRF24L01_TxPacket(unsigned char *tx_buf);
void RX_Mode(char rec_len);
void nRF24L01_Config(void);
void NRF24L01_Send(u8 * tx_buf);
void NRF24L01_Receive(void);
u8 SPI_RW_Reg(u8 reg,u8 value);
#endif
//==============================
