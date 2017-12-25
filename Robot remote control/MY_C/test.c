#include "sys.h"
#include "usart.h"		
#include "delay.h"	 
#include "led.h" 
#include "key.h"
#include "mpu6050.h"
#include "myiic.h"
#include "timer.h"
#include "string.h"
#include "outputdata.h"
#include "nrf24l01.h"
#include "exti.h"
#include "stdlib.h"
#include "Low_power.h"
#include "atan2.h"
//#include "miniIMU_Printf.h"

extern char TX_BUF[TX_LEN];
extern char MPU6050_read_flag;
extern u8   RNF_send_flag;
extern u8 	NRF_rx_flag;
extern const u8 TX_ADDRESS[TX_ADR_WIDTH];
extern u8 key0_flag;
u8 NRF_resend_num;
extern u8 idle;
extern u8 NRF_Sync;
extern u8 RX_BUF[];
//u8 read_key0_flag(void);
//u8 read_rest_KEY_status_flag(void);
u8 SPI_Read_Reg(u8 reg);


int main(void)
{		
	short sq[4];
//初始化代码
	{	
//		while(1);
  		Stm32_Clock_Init(9); //系统时钟设置
		System_Clk_Init(72);
//		uart_init(72,115200,TX_BUF);
//		Tim4_Init(0xffff,71);//MPU9250的
		KEY_Init();
		LED_Init();	
//		LED2=1;
		LED_R=0;
 		NRF24L01_init(1);
		
//		NRF24L01_Send((u8 *)TX_BUF);
//		while(1);
		
//		Timer1_Init(200,71);//72分频 0.2ms进入一次中断
		Timer1_Init(200,71);//72分频 0.2ms进入一次中断
		EXTIX_Init();
		IIC_Init();
		MPU6050_Init();//硬件错误出在这里
//		MPU9150_IMU_AHRS();
//		printf("asdf");
//		RX_Mode(1);
//		LED_G=0;
		LED_R=1;
		RX_Mode(TX_LEN);
	}

		
//主循环	
	while(1){		
			if(NRF_rx_flag==1){
				SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 2);      // 进入上电状态
				NRF_rx_flag=2;
				delay_System_us(1300);
				RX_Mode(TX_LEN);
//				LED_G=1;
				NRF_resend_num=NRF_CHANNEL;
			}
		if((RNF_send_flag)&&(!NRF_rx_flag)){
				SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 2);      // 进入上电状态
				RNF_send_flag=0;
				NRF24L01_Send((u8 *)TX_BUF);
			}
//			get_oula_MPU9150_IMU();
			if(MPU6050_read_flag){
				MPU6050_read_flag=0;
				if(UP_quat_MPU6050(sq)){
					TX_BUF[0]=0x55;
					TX_BUF[1]=0x80|NRF_CHANNEL;
					TX_BUF[10]=key0_flag;
					for(char i=0;i<4;i++){
						TX_BUF[(i<<1)+2]=sq[i];
						TX_BUF[(i<<1)+3]=sq[i]>>8;			
					}
					TX_BUF[11]=CHECK_SUM_8bit(TX_BUF,11);
				}
			}
//			CPU_SLEEP_ENTER();//这句有效
		}	
}	 

void EXTI15_10_IRQHandler(void)
{
		static u8 max_num;
		u8 status;
//	static u8 num=100;
//	NRF_IRQ_OUT_TIME=50;
//	while(IRQ&&NRF_IRQ_OUT_TIME);//这个不能少,少了代码就动不了了,不知道什么情况啊啊啊啊
		CE=0;
		status=SPI_Read_Reg(STATUS);	// 读取状态寄存其来判断数据接收状况
		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0xff);      // 清除TX，让IRQ拉低；
		if(status & RX_DR){			//接受中断标志位
			SPI_Read_Buf(RD_RX_PLOAD,RX_BUF,1);// read receive payload from RX_FIFO buffer
			NRF_rx_flag=0;
			NRF_Sync=RX_BUF [0];
//			delay_System_us(1200 );//必须 250K
			delay_System_ms(1);
			SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0);      // 进入待机状态 降低功耗
			LED_G=1;
//			LED_G=0;
		} 
		else if(status & TX_DS){
				
			RX_Mode(TX_LEN);
		}
		else if(status & MAX_RT){
//				LED_G=1;
				if(max_num++>20){
					max_num=0;
					NRF_rx_flag=1;
				}
		}
//			CE=1;//低
	EXTI->PR|=1<<10;//清中断
}
void Default_Handler(void){
	while(1);
}





