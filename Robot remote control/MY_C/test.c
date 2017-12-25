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
//��ʼ������
	{	
//		while(1);
  		Stm32_Clock_Init(9); //ϵͳʱ������
		System_Clk_Init(72);
//		uart_init(72,115200,TX_BUF);
//		Tim4_Init(0xffff,71);//MPU9250��
		KEY_Init();
		LED_Init();	
//		LED2=1;
		LED_R=0;
 		NRF24L01_init(1);
		
//		NRF24L01_Send((u8 *)TX_BUF);
//		while(1);
		
//		Timer1_Init(200,71);//72��Ƶ 0.2ms����һ���ж�
		Timer1_Init(200,71);//72��Ƶ 0.2ms����һ���ж�
		EXTIX_Init();
		IIC_Init();
		MPU6050_Init();//Ӳ�������������
//		MPU9150_IMU_AHRS();
//		printf("asdf");
//		RX_Mode(1);
//		LED_G=0;
		LED_R=1;
		RX_Mode(TX_LEN);
	}

		
//��ѭ��	
	while(1){		
			if(NRF_rx_flag==1){
				SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 2);      // �����ϵ�״̬
				NRF_rx_flag=2;
				delay_System_us(1300);
				RX_Mode(TX_LEN);
//				LED_G=1;
				NRF_resend_num=NRF_CHANNEL;
			}
		if((RNF_send_flag)&&(!NRF_rx_flag)){
				SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 2);      // �����ϵ�״̬
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
//			CPU_SLEEP_ENTER();//�����Ч
		}	
}	 

void EXTI15_10_IRQHandler(void)
{
		static u8 max_num;
		u8 status;
//	static u8 num=100;
//	NRF_IRQ_OUT_TIME=50;
//	while(IRQ&&NRF_IRQ_OUT_TIME);//���������,���˴���Ͷ�������,��֪��ʲô�����������
		CE=0;
		status=SPI_Read_Reg(STATUS);	// ��ȡ״̬�Ĵ������ж����ݽ���״��
		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0xff);      // ���TX����IRQ���ͣ�
		if(status & RX_DR){			//�����жϱ�־λ
			SPI_Read_Buf(RD_RX_PLOAD,RX_BUF,1);// read receive payload from RX_FIFO buffer
			NRF_rx_flag=0;
			NRF_Sync=RX_BUF [0];
//			delay_System_us(1200 );//���� 250K
			delay_System_ms(1);
			SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0);      // �������״̬ ���͹���
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
//			CE=1;//��
	EXTI->PR|=1<<10;//���ж�
}
void Default_Handler(void){
	while(1);
}





