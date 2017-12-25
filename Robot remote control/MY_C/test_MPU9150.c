
/*

STM��MPU9250
+5V			VCC_IN(�ں�3.3V��ѹ)	+5V								��Դ+
GND			GND										GND								��Դ-
SCL			SCL/SCLK							PB10(I2C2-SCL)		I2Cʱ����
SDA			SDA/MOSI							PB11(I2C2-SDA)		I2C������
NONE		NCS(?????)						+3V 							�ߵ�ƽ,MOSIʱ����
sel AD 	AD/MOSI 							GND								���͵�ַ68,����69

STM�Դ���
����RX PA9
����TX PA10

STM��24L01
1	GND		GND
2 VCC		VCC                                                                                                                                                                                                                                                                                                                                                                                                                            
3 CE		PB11
4 CSN		PB12
5 SCK		PB13	
6 MOSI	PB15
7 MISO	PB14
8 IRQ		PB10

*/

#include "sys.h"
#include "usart.h"		
#include "delay.h"	 
#include "led.h" 
#include "key.h"
#include "mpu9150.h"
#include "myiic.h"
#include "timer.h"
#include "string.h"
#include "outputdata.h"
#include "nrf24l01.h"
#include "exti.h"
#include "stdlib.h"
#include "math.h"
#include "stdio.h"
#include "atan2.h"

char TX_BUF[TX_LEN];

u8 read_key0_flag(void);
u8 read_RNF_send_flag(void);
u8 read_rest_MPU6050_read_flag(void);
u8 read_rest_KEY_status_flag(void);
u8 rw_NRF_disconnect_flag(u8 rw,u8 temp);
short two_yaw_num_compare_short(short yaw_start,short yaw_end,u16 Range_Sizes);
float get_new_yaw_num_float(float yaw_start,float error_temp,float Range_min,float Range_max);
short get_new_yaw_num_short(short yaw_start,short error_temp,short Range_min,short Range_max);
void send_OULA_to_miniIMU(float *roll,float *pitch,short *yaw);
void NRF_data_handle(float roll,float pitch,float yaw,char *TX_BUF);




/*
����
*/
int main(void)
{		
	u32 sensor_timestamp;
	float pitch,roll;
	float pitch_PUT;
	
	short yaw,yaw_last=0;
	short compass[3];
	short compass_yaw;
	short yaw_jiaozhun=0;

//	u16 *p=(u16 *)uart_send_buf;
	u8 	yaw_cul_flag=0;
	u8  temp;
	
	
//��ʼ������
	{	
		Stm32_Clock_Init(9); //ϵͳʱ������
		System_Clk_Init(72);
		uart_init(72,115200,TX_BUF);
//		printf("asfd\r\n");
//		USART1_Send_Buf("aqwer\r\n",strlen("aqwer\r\n"));
		KEY_Init();
		LED_Init();	
//		LED2=1;
		LED_R=0;
 		NRF24L01_init(1);
		Timer1_Init(200,71);//72��Ƶ 0.2ms����һ���ж�
		EXTIX_Init();
		IIC_Init();
		MPU9150_init(&sensor_timestamp);
//		MPU6050_Init();
		printf("asdf\r\n");
		
//����У׼����
//		X=178 Y=265 Z=47
//		compass_adjustment(compass_error);
		

		
		LED_G=0;
		LED_R=1;
}
//��ѭ��	
 while(1)
	{		
			//δ����ʱ����
			if(rw_NRF_disconnect_flag(READ,0)==1)
			{
					LED_G=1;
					SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 2);      // �����ϵ�״̬
					rw_NRF_disconnect_flag(WRITE,2);
					delay_System_us(1300);
					RX_Mode(TX_LEN);
			}
			if(read_RNF_send_flag()&&!(rw_NRF_disconnect_flag(READ,0)))
			{
				SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 2);      // �����ϵ�״̬
//				NRF_data_handle(roll,-pitch_PUT,yaw_jiaozhun);
				NRF_data_handle(roll,-pitch_PUT,yaw,TX_BUF);
				delay(1);
				NRF24L01_Send(CHANNEL,(u8 *)TX_BUF);
				delay(1);
				SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0);      // �������״̬ ���͹���
			}
			temp=read_rest_MPU6050_read_flag();
			if(temp)
			{
				if(temp==2)
					if (!mpu_get_compass_reg(compass, &sensor_timestamp))
					{
						yaw_cul_flag|=1;
											
					}
//				MPU6050_read_flag=0;	
				if(get_oula_MPU6050(&roll,&pitch,&yaw))//���rollΪС pitch С yaw ��
				{
					yaw_cul_flag|=2;
					pitch_PUT=pitch*57.3;
				}
				if(yaw_cul_flag==3)
				{
					//�������ƫ����
//					compass_yaw = 57.295780 * atan2((compass[1]-178)*cos(roll) + (compass[0]-265)*sin(roll)*sin(pitch) + (compass[2]-47)*sin(roll)*cos(pitch),(compass[0]-265)*cos(pitch) - (compass[2]-47)*sin(pitch));
					compass_yaw=my_atan2(
									((compass[0]-COMP_P_ERR)*cos(pitch) - (compass[2]-COMP_Y_ERR)*sin(pitch))*10000,
									((compass[1]-COMP_R_ERR)*cos(roll) + (compass[0]-COMP_P_ERR)*sin(roll)*sin(pitch) + (compass[2]-COMP_Y_ERR)*sin(roll)*cos(pitch))*10000
															)>>8;
					
					yaw_cul_flag=0;
//					yaw_jiaozhun=(yaw_jiaozhun+(yaw-yaw_last))*0.96+compass_yaw*0.04;//�����˲��㷨
					/*
						yaw_jiaozhun�����yaw���Ա�
						ÿ����������,ֻ�ж�����,+-0.01;
						���yaw-yaw_last
						
					*/			
//					yaw_jiaozhun=yaw_jiaozhun+two_yaw_num_compare_short(yaw_last,yaw,360)+(yaw_jiaozhun > compass_yaw?(-0.08*(yaw_jiaozhun-compass_yaw)):0.08*((compass_yaw-yaw_jiaozhun)));
						yaw_jiaozhun=get_new_yaw_num_short(yaw_jiaozhun,
																		two_yaw_num_compare_short(yaw_last,yaw,360)+two_yaw_num_compare_short(yaw_jiaozhun,compass_yaw,360)*0.3,
																		-180,
																		180);
					yaw_last=yaw;
					send_OULA_to_miniIMU(&roll,&pitch,&yaw);
//					printf("%d,%d,%d\r\n",(short)(roll*57.3),(short)pitch_PUT,(short)yaw);
//					printf("%d,%d,%d\r\n",(short)compass_yaw,yaw,(short)yaw_jiaozhun);
				}
	//				MPU6050_Pose();//ռ��2ms
			}
	}	
}	 




