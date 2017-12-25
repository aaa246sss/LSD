#include "MAIN_MY.h"
#include "stm32f4xx_hal.h"
#include "nrf24l01.h"
#include "NRF24L01_ACK.h"
#include "delay.h"
#include "atan2.h"
#include "string.h"
#include "attitude.h"
#include "PID.h"
#include "usart.h"

const char ROBOT_INIT_BUF[]=
" #0 P1423 #1 P1300 #2 P750 #3 P1400 #5 P1504 #6 P1433 #8 P1445 #9 P1570 #10 P2446 #11 P1873 #12 P1092 #13 P558 #15 P1550 #17 P1499 #18 P1515 #20 P1558 #21 P2250 #22 P1745 #23 P1500  T1000\r\n";
const char ROBOT_FOOD_INIT_BUF[]=
//"#0 P1450 #1 P1310 #2 P1280 #3 P1621 #5 P1500 #18 P1500 #20 P1330 #21 P1728 #22 P1773 #23 P1500 T1000\r\n";
" #0 P1423 #1 P1300 #3 P1400 #5 P1504 #18 P1515 #20 P1558 #22 P1745 #23 P1500  T1000\r\n";

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;


void GLOVAR_STRUCT_INIT(GLOVAR_HandleTypeDef *GloVar)
{
	uint8_t i;
	GloVar->UART.NEED_SEND=1;
	GloVar->UART.SEND_OK=1;
//	GloVar->NRF.FLAG_TX_OK=0;
	GloVar->ROBOT.P_value_cul[0]=P0;
	GloVar->ROBOT.P_value_cul[1]=P1;
	GloVar->ROBOT.P_value_cul[22]=P22;
	GloVar->ROBOT.P_value_cul[23]=P23;
	GloVar->KEY.KEY0=1;
	GloVar->NRF.FLAG_RECON=10;
	GloVar->IMU[2].fpitch=90/57.3f;
	GloVar->IMU[0].fpitch=90/57.3f;
	for(i=0;i<6;i++){
		GloVar->IMU[i].q[0]=1;
	}
}

void LIZJ_HAL_INIT(GLOVAR_HandleTypeDef *GloVar)
{
	USART_CONFIG(GloVar);
	GLOVAR_STRUCT_INIT(GloVar);
	increasePIDInit(&GloVar->PID_qh,0,4,0);
	NRF24L01_init();
	NRF24L01_init_2();
	NRF_RX_Mode(GloVar,RX_LEN);
	EXTI->IMR|=(1<<8);
	NRF_RX_Mode_2(GloVar,1);
	EXTI->IMR|=(1<<12);

}


void usart1_data_send_handle(GLOVAR_HandleTypeDef *GloVar)
{
		if(GloVar->UART.NEED_SEND>1)
			GloVar->UART.SEND_OK=0;
		if(GloVar->UART.NEED_SEND&(1<<1)){
			GloVar->UART.NEED_SEND&=~(1<<1);
		}
		else if(GloVar->UART.NEED_SEND&(1<<2)){
			GloVar->UART.NEED_SEND&=~(1<<2);
			HAL_UART_Transmit_DMA(&huart1,(uint8_t *)GloVar->UART.SEND_BUF,strlen(GloVar->UART.SEND_BUF));
		}
		else if(GloVar->UART.NEED_SEND&(1<<3)){
			GloVar->UART.NEED_SEND&=~(1<<3);
			HAL_UART_Transmit_DMA(&huart1,(uint8_t *)"#VERI\r\n",7);
		}
		else if(GloVar->UART.NEED_SEND&(1<<4)){
			GloVar->UART.NEED_SEND&=~(1<<4);
			HAL_UART_Transmit_DMA(&huart1,(uint8_t *)"UPDATA\r\n",strlen("UPDATA\r\n"));
		}
		else if(GloVar->UART.NEED_SEND&(1<<5)){
			GloVar->UART.NEED_SEND&=~(1<<5);
			HAL_UART_Transmit_DMA(&huart1,(uint8_t *)ROBOT_INIT_BUF,strlen(ROBOT_INIT_BUF));
		}
		else if(GloVar->UART.NEED_SEND&(1<<6)){
			GloVar->UART.NEED_SEND&=~(1<<6);
			HAL_UART_Transmit_DMA(&huart1,(uint8_t *)ROBOT_FOOD_INIT_BUF,strlen(ROBOT_FOOD_INIT_BUF));
		}
}

void Up_UART_DATA(GLOVAR_HandleTypeDef *GloVar)
{
	static short P_value_last [24];
	static uint8_t Pnum [24]={0};
	uint8_t i;
	uint8_t len=0;
	
	if(!GloVar->TIMER.ACTION_TIMER_REMAIN){
			for(i=0;i<24;i++){
				if(GloVar->ROBOT.P_value[i]!=P_value_last[i]){
					P_value_last[i]=GloVar->ROBOT.P_value[i];
					Pnum[i]=2;
				}
				if(Pnum[i]){
					Pnum[i]--;
					len+=sprintf(&GloVar->UART.SEND_BUF[len],"#%dP%d",i,P_value_last[i]);
				}
			}	
		}
		if(len!=0){
			len+=sprintf(&GloVar->UART.SEND_BUF[len],"T40\r\n");
			GloVar->UART.NEED_SEND|=1<<2;	
		}
//	}
}
//void NRF_Reconnection_check(GLOVAR_HandleTypeDef *GloVar)
//{
//		GloVar->NRF.FLAG_RECON=0;
////		GloVar->NRF.FLAG_RX=0;
////		GloVar->NRF.BUF_TX[0]=GloVar->TIMER.NRF_Sync;		//只需在此处做修改即可连续发送数据 例如两个数据 //不能发
//		NRF24L01_Send(GloVar);
//}

void Run_Action_Group(GLOVAR_HandleTypeDef *GloVar,uint8_t Group_num,short speed,uint8_t start_num)
{
//	sprintf(uart_send_buf,"PL 0 SQ %d IX %d SM %d ONCE\r\n",Group_num,start_num,speed);
	sprintf(GloVar->UART.SEND_BUF,"PL 0 SQ %d IX %d SM %d ONCE\r\n",Group_num,start_num,speed);
	GloVar->UART.NEED_SEND|=1<<2;	
}

void Run_Action_Group_printf(GLOVAR_HandleTypeDef *GloVar,uint8_t Group_num,short speed,uint8_t start_num)
{
//	sprintf(uart_send_buf,"PL 0 SQ %d IX %d SM %d ONCE\r\n",Group_num,start_num,speed);
	sprintf(GloVar->UART.SEND_BUF,"PL 0 SQ %d IX %d SM %d ONCE\r\n",Group_num,start_num,speed);
	HAL_UART_Transmit_DMA(&huart1,(uint8_t *)GloVar->UART.SEND_BUF,strlen(GloVar->UART.SEND_BUF));
//	GloVar->UART.NEED_SEND|=1<<2;	
}

void decode_UART_rec(GLOVAR_HandleTypeDef *GloVar){
	if((GloVar->UART.REC_BUF[0]==0x55)&&(GloVar->UART.REC_BUF[1]==0x53)&&(GloVar->UART.REC_BUF[10]==CHECK_SUM_8bit(GloVar->UART.REC_BUF,10))){
		GloVar->ROBOT.ROLL=((short)(GloVar->UART.REC_BUF [3]<<8)+GloVar->UART.REC_BUF [2])/18-ROBOT_ROLL_ERROR;		//前后
		GloVar->ROBOT.PITCH=((short)(GloVar->UART.REC_BUF[5]<<8)+GloVar->UART.REC_BUF[4])/18-ROBOT_PITCH_ERROR; //左右
		GloVar->ROBOT.YAW=(((short)GloVar->UART.REC_BUF[7]<<8)+GloVar->UART.REC_BUF[6])/182;
	if((!GloVar->TIMER.ACTION_TIMER_REMAIN)&&(!(GloVar->ROBOT.ACTION_PUSH_UP&0x04))){
		GloVar->ROBOT.PID_OUT_qh+=increasePIDcal(&GloVar->PID_qh,GloVar->ROBOT.ROLL,0);
		if(GloVar->ROBOT.PID_OUT_qh>200)			GloVar->ROBOT.PID_OUT_qh=200;
		else if(GloVar->ROBOT.PID_OUT_qh<-200)GloVar->ROBOT.PID_OUT_qh=-200;
		GloVar->ROBOT.P_value[22]=GloVar->ROBOT.P_value_cul[22] + GloVar->ROBOT.PID_OUT_qh;
		GloVar->ROBOT.P_value[1]=GloVar->ROBOT.P_value_cul[1]- GloVar->ROBOT.PID_OUT_qh;
	}
	else
		GloVar->ROBOT.PID_OUT_qh=0;
	}
}

void foot_banlance(GLOVAR_HandleTypeDef *glovar,short temp)
{
	if(temp>9)
		temp=9;
	else if(temp<=-7)
		temp=-7;//右脚起
	glovar->ROBOT.lf_angle_need=-temp*4;
	glovar->ROBOT.P_value_cul[0] = P0 + temp * 11; 
	glovar->ROBOT.P_value_cul[23]= P23+ temp * 11;
//role_right1 = P23 + HScroll4.Value * 11 + HScroll7.Value * 11 '1379
	glovar->ROBOT.P_value[23]=glovar->ROBOT.P_value_cul[23] - glovar->ROBOT.PID_OUT_lf;
	glovar->ROBOT.P_value[0] =glovar->ROBOT.P_value_cul[0]  - glovar->ROBOT.PID_OUT_lf;
}

void Quat_To_Robot_uart(GLOVAR_HandleTypeDef *glovar)//			static float quat_temp[4];//			static short YAW_out,PITCH_out;
{
	static float H_l=2,H_r=2;
	static short OUT1,OUT2;

	static uint8_t Pkey_last;
	static short two_foot_error_qh=0,foot_l_qh=0,foot_r_qh=0;
	short temp,temp1;
	
	uint8_t flag;
/*
Dim ham_left, ham_right, crus_left, crus_right As Integer
Dim sole_left, sole_right As Integer
Dim sole_left1, sole_right1, ham_left1, ham_right1 As Integer
*/
	if(glovar->NRF.REC_CHANNEL<4){
		if(glovar->ROBOT.ACTION_PUSH_UP&0x04){
			if(glovar->NRF.REC_CHANNEL==0){
				UP_oula(glovar->IMU,glovar->NRF.REC_CHANNEL);
				OUT1=(short)(glovar->IMU[glovar->NRF.REC_CHANNEL].fpitch*57.3f);
				if(OUT1>0)OUT1=0;
				if(OUT1<-75)OUT1=-75;
				glovar->ROBOT.P_value[8]=2650+OUT1*15;
				glovar->ROBOT.P_value[10]=1576-OUT1*8;
				glovar->ROBOT.P_value[17]=324-OUT1*15;
				glovar->ROBOT.P_value[13]=1448+OUT1*8;	
			}
		}
		else
			robot_Hand_syn_cal(glovar->IMU,glovar->NRF.REC_CHANNEL,glovar->ROBOT.P_value);
	}
	else{
		if((glovar->IMU [6].key&0x01)!=Pkey_last){//按键判断
			Pkey_last=glovar->IMU[6].key;
			glovar->IMU[4].sroll_error=glovar->IMU[4].sroll;
			glovar->IMU[5].sroll_error=glovar->IMU[5].sroll;
			glovar->IMU[6].sroll_error=glovar->IMU[6].sroll;
			glovar->IMU[7].sroll_error=glovar->IMU[7].sroll;
		}
		
		//判断左右脚
		
		UP_oula(glovar->IMU,glovar->NRF.REC_CHANNEL);
		flag=glovar->NRF.REC_CHANNEL<6?0:2;
		
	//计算大腿 角度表示精度1度		
		
		temp =glovar->IMU[4+flag].sroll-glovar->IMU[4+flag].sroll_error;	
		if(temp>30)	OUT1=30;		
		else if(temp<-30)OUT1=-30;
		else OUT1=temp;
		
	//计算小腿 角度表示精度1度		
		
		temp1=glovar->IMU[5+flag].sroll-glovar->IMU[5+flag].sroll_error-OUT1;
		if(temp1>30)OUT2=30;
		else if(temp1<-90)OUT2=-90;
		else OUT2=temp1;
		
		//判断是否俯卧撑姿态
		if(glovar->ROBOT.ACTION_PUSH_UP &0x04){
			if(glovar->NRF.REC_CHANNEL<=5){
				if(OUT1>-29)
					glovar->ROBOT.ACTION_PUSH_UP&=0xfe;
				else
					glovar->ROBOT.ACTION_PUSH_UP|=0x01;
			}
			else{
				if(OUT1>-29)
					glovar->ROBOT.ACTION_PUSH_UP&=0xfd;
				else
					glovar->ROBOT.ACTION_PUSH_UP|=0x02;
			}
			if(!(glovar->ROBOT.ACTION_PUSH_UP&0x03)){
		//执行动作组站起
				Run_Action_Group(glovar,3,100,0);
				memset(glovar->ROBOT.P_value,0,48);
				glovar->ROBOT.ACTION_PUSH_UP&=~0x04;
				//glovar->ROBOT.ACTION_PUSH_UP=0;
		//设置再次动作时间
				glovar->TIMER.ACTION_TIMER_REMAIN=4000;
			}
		}
		else{
			if(glovar->NRF.REC_CHANNEL<=5){
				if(OUT1==-30)
					glovar->ROBOT.ACTION_PUSH_UP|=0x01;
				else
					glovar->ROBOT.ACTION_PUSH_UP&=0xfe;
				H_l=cos_MY((OUT1+21)/57.3f)+cos_MY((OUT1+OUT2+21-48)/57.3f);
				foot_l_qh=OUT1;
			}
			else{
				if(OUT1==-30)
					glovar->ROBOT.ACTION_PUSH_UP|=0x02;
				else
					glovar->ROBOT.ACTION_PUSH_UP&=0xfd;
				H_r=cos_MY((OUT1+21)/57.3f)+cos_MY((OUT1+OUT2+21-48)/57.3f);
				foot_r_qh=OUT1;
			}
			if(glovar->ROBOT.ACTION_PUSH_UP==3){
				//执行动作组趴下
				Run_Action_Group(glovar,2,100,0);
				memset(glovar->ROBOT.P_value,0,48);
				glovar->ROBOT.ACTION_PUSH_UP|=0x04;
				//设置再次动作时间
				glovar->TIMER.ACTION_TIMER_REMAIN=3310;
			}	
			two_foot_error_qh=foot_l_qh-foot_r_qh;
			if(two_foot_error_qh>25)
				two_foot_error_qh=25;
			if(two_foot_error_qh<-25)
				two_foot_error_qh=-25;
			glovar->ROBOT.P_value[3]=P3 + two_foot_error_qh * 11;
			glovar->ROBOT.P_value_cul[1]	=P1 + two_foot_error_qh * 9 ;
			glovar->ROBOT.P_value[1]=glovar->ROBOT.P_value_cul[1]  - glovar->ROBOT.PID_OUT_qh;
			glovar->ROBOT.P_value[20]=P20 + two_foot_error_qh * 11 ;
			glovar->ROBOT.P_value_cul[22]=P22 + two_foot_error_qh * 9 ;
			glovar->ROBOT.P_value[22]=glovar->ROBOT.P_value_cul[22] + glovar->ROBOT.PID_OUT_qh;
			temp=(short)(( H_r - H_l )*115);//120
			foot_banlance(glovar,temp);
		}
	}
}

char decode_NRF_rec(GLOVAR_HandleTypeDef *glovar)
{
	short temp;
	char i;
	if(((glovar->NRF.BUF_RX[0]==0x55)&&(glovar->NRF.BUF_RX[1]&0x80))&&(CHECK_SUM_8bit(glovar->NRF.BUF_RX,11)==glovar->NRF.BUF_RX[11])){
		
		glovar->NRF.REC_CHANNEL=glovar->NRF.BUF_RX[1]&0x7f;
//		*key_temp=BUF[10];
		glovar->IMU[glovar->NRF.REC_CHANNEL].key=glovar->NRF.BUF_RX[10];
			
		if(!(glovar->IMU[glovar->NRF.REC_CHANNEL].key&0x02)){
			for(i=0;i<4;i++){
				temp=glovar->NRF.BUF_RX[(i<<1)+2]|(glovar->NRF.BUF_RX[(i<<1)+3]<<8);
				glovar->IMU[glovar->NRF.REC_CHANNEL].q[i]=(float)temp/32768;
			}
		}
		return 1;
	}	
	return 0;
}

void NRF_data_handle(GLOVAR_HandleTypeDef *glovar)
{
	glovar->NRF.FLAG_RECON++;
	glovar->NRF.FLAG_RX=0;
	
	if(decode_NRF_rec(glovar))//解码
	{
		if(!glovar->TIMER.ACTION_TIMER_REMAIN)
			Quat_To_Robot_uart(glovar);//姿态解算
	}
}
