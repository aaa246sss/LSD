#include "attitude.h"
#include "atan2.h"
//#include "sys.h"
#include "math.h"
#include "stdlib.h"
//extern IMU_HandleTypeDef imu[];

uint8_t CHECK_SUM_8bit(char *buf,uint8_t len){
	uint8_t SUM=0;
	uint8_t i;
	for(i=0;i<len;i++)
		SUM+=buf[i];
	return SUM;
}

/*
功能 比较两个旋转偏航角(float型)的大小
输入:
yaw_start:偏航角开始的位置
yaw_end:	偏航角结束的位置
Range_Sizes:	偏航角的范围尺寸大小
返回 error_temp
error_temp:	正数表示正方向旋转
*/
float two_yaw_num_compare_float(float yaw_start,float yaw_end,float Range_Sizes)
{
	float T2=Range_Sizes/2;
	if(yaw_end>=yaw_start)
	{
		if(yaw_end-yaw_start>T2)//-160?160
			return yaw_end-yaw_start-Range_Sizes;
	}
	else if(yaw_start-yaw_end>T2)//160?-160
		return yaw_end-yaw_start+Range_Sizes;
	return yaw_end-yaw_start;
}


/*
功能 比较两个旋转偏航角(short型)的大小
输入:
yaw_start:偏航角开始的位置
yaw_end:	偏航角结束的位置
Range_Sizes:	偏航角的范围尺寸大小
返回 error_temp
error_temp:	正数表示正方向旋转
*/
short two_yaw_num_compare_short(short yaw_start,short yaw_end,uint16_t Range_Sizes)
{
	uint16_t T2=Range_Sizes/2;
	if(yaw_end>=yaw_start)
	{
		if(yaw_end-yaw_start>T2)//-160?160
			return yaw_end-yaw_start-Range_Sizes;
	}
	else if(yaw_start-yaw_end>T2)//160?-160
		return yaw_end-yaw_start+Range_Sizes;
	return yaw_end-yaw_start;
}

/*
功能 获取新旋转偏航角(float型)的值
输入:
yaw_start:偏航角开始的位置
error_temp:	正数表示正方向旋转

Range_Sizes:	偏航角的范围尺寸大小
返回 yaw_new:	偏航角最新的位置

*/
float get_new_yaw_num_float(float yaw_start,float error_temp,float Range_min,float Range_max)
{
	float T=Range_max-Range_min;
	float temp=yaw_start-Range_min+error_temp;
	if(error_temp>=0)
	{
		if(temp>T)
			return yaw_start+error_temp-T;		
	}
	else if(temp<0)
		return yaw_start+error_temp+T;
	return yaw_start+error_temp;
}

/*
功能 获取新旋转偏航角(short型)的值
输入:
yaw_start:偏航角开始的位置
error_temp:	正数表示正方向旋转

Range_Sizes:	偏航角的范围尺寸大小
返回 yaw_new:	偏航角最新的位置

*/
short get_new_yaw_num_short(short yaw_start,short error_temp,short Range_min,short Range_max)
{
	short T=Range_max-Range_min;
	short temp=yaw_start-Range_min+error_temp;
	if(error_temp>=0)
	{
		if(temp>T)
			return yaw_start+error_temp-T;		
	}
	else if(temp<0)
		return yaw_start+error_temp+T;
	return yaw_start+error_temp;
}


void YAW_shanxing(char Rec_channel,char key0_flag, short yaw,short * YAW_out){
		static char  init[6]={1,1,1,1,1,1};
		static short yaw_uart_init[6]={100,100,200,100,100,100};
		static short yaw_min[6]={100},yaw_max[6]={200};
		static short fazhi[6];
		static char only_once_flag[6];
		uint16_t yaw_uart_last[6]={100,100,100,100,100,100};
		yaw_uart_last[Rec_channel]=yaw_uart_init[Rec_channel];
		yaw_uart_init[Rec_channel]=(yaw+179);
		if((key0_flag)){
				//这个目的是干嘛了啊 ,按键按下只处理一次
				init[Rec_channel]=1;
				if(only_once_flag[Rec_channel]){
					only_once_flag[Rec_channel]=0;
					fazhi[Rec_channel]=(yaw_min[Rec_channel]+yaw_max[Rec_channel]+CIRCLE+(yaw_max[Rec_channel]>yaw_min[Rec_channel]?0:CIRCLE))>>1;
					if(fazhi[Rec_channel]>CIRCLE)fazhi[Rec_channel]-=CIRCLE;
				//	KEY_status=1;
				}
					if(yaw_uart_init[Rec_channel]-fazhi[Rec_channel]+((yaw_uart_init[Rec_channel]>fazhi[Rec_channel])?0:CIRCLE)<(CIRCLE-RADIAN)>>1)//顺时针	
						*YAW_out=0;	
					else if(yaw_uart_init[Rec_channel]-fazhi[Rec_channel]+((yaw_uart_init[Rec_channel]>fazhi[Rec_channel])?0:CIRCLE)>(CIRCLE+RADIAN)>>1)
						*YAW_out=RADIAN;	
					else{
						*YAW_out=CIRCLE+yaw_uart_init[Rec_channel]-yaw_min[Rec_channel];
						if(*YAW_out>=CIRCLE)
							*YAW_out-=CIRCLE;
					}
		}
		else{ //初始时应该执行这句
			only_once_flag[Rec_channel]=1;
			if(two_yaw_num_compare_short(yaw_uart_last[Rec_channel],yaw_uart_init[Rec_channel],CIRCLE)>0){//yaw_uart_init在yaw_uart_last的正旋转方向	
				if(two_yaw_num_compare_short( yaw_uart_init[Rec_channel],yaw_max[Rec_channel],CIRCLE)<0){//yaw_max在yaw_uart_init的负旋转方向,需要更新yaw_max
					//更新yaw_max与yaw_min状态
					yaw_max[Rec_channel]=yaw_uart_init[Rec_channel];
					if(yaw_max[Rec_channel]>RADIAN)
						yaw_min[Rec_channel]=yaw_max[Rec_channel]-RADIAN;
					else
						yaw_min[Rec_channel]=CIRCLE +yaw_max[Rec_channel] - RADIAN;
				}
			}
			else if((yaw_uart_last[Rec_channel]-yaw_uart_init[Rec_channel]!=0)&&two_yaw_num_compare_short( yaw_uart_init[Rec_channel],yaw_min[Rec_channel],CIRCLE)>0){//yaw_uart_init在yaw_uart_last的负旋转方向, yaw_min在yaw_uart_init的正旋转方向,需要更新yaw_min与yaw_max
					yaw_min[Rec_channel]=yaw_uart_init[Rec_channel];
					yaw_max[Rec_channel]=yaw_min[Rec_channel]+RADIAN;
					if(yaw_max[Rec_channel]>=CIRCLE)
						yaw_max[Rec_channel]-=CIRCLE;
			}
			if(init[Rec_channel]){
				init[Rec_channel]=0;
				switch(Rec_channel){
					case 0:
					case 1:
						yaw_min[Rec_channel]=yaw_uart_init[Rec_channel]-RADIAN;
						if(yaw_min[Rec_channel]<0)
							yaw_min[Rec_channel]+=CIRCLE;
						yaw_max[Rec_channel]=yaw_min[Rec_channel]+RADIAN;
						if(yaw_max[Rec_channel]>CIRCLE)
							yaw_max[Rec_channel]-=CIRCLE;
						break;
					case 2:
					case 3:
						yaw_min[Rec_channel]=yaw_uart_init[Rec_channel];
						if(yaw_min[Rec_channel]<0)
							yaw_min[Rec_channel]+=CIRCLE;
						yaw_max[Rec_channel]=yaw_min[Rec_channel]+RADIAN;
						if(yaw_max[Rec_channel]>CIRCLE)
							yaw_max[Rec_channel]-=CIRCLE;
						break;
						
				}
					
			}
			*YAW_out=CIRCLE+yaw_uart_init[Rec_channel]-yaw_min[Rec_channel];
			if(*YAW_out>=CIRCLE)
				*YAW_out-=CIRCLE;		
		}	
}

short get_acdf(IMU_HandleTypeDef *pimu,uint8_t Rec_channel){
	static short syaw;
	pimu[Rec_channel].fpitch=asinf(2 * fq[0]* fq[2] -2 * fq[1] * fq[3]);//64us   //变负数
//	pimu[Rec_channel].syaw=my_atan2((1-2*fq[2]*fq[2]-2*fq[3]*fq[3])*10000,(2*(fq[1]*fq[2]+fq[0]*fq[3]))*10000)>>8;//17us	

//	pimu[Rec_channel].fyaw=;
	pimu[Rec_channel].syaw=atan2f(2*(fq[1]*fq[2]+fq[0]*fq[3]),1-2*fq[2]*fq[2]-2*fq[3]*fq[3])*57.3f;
	YAW_shanxing(Rec_channel,pimu[Rec_channel].key&0x01,pimu[Rec_channel].syaw,&syaw);
	if(Rec_channel<2)
		syaw=syaw-89;
	else if((Rec_channel<4))
		syaw=89-syaw;
	pimu[Rec_channel].fyaw=syaw;
	pimu[Rec_channel].fyaw=pimu[Rec_channel].fyaw/57.3f;
	return syaw;
}

uint8_t UP_oula(IMU_HandleTypeDef *pimu,uint8_t Rec_channel){
	pimu[Rec_channel].fpitch=asinf(2 * fq[0]* fq[2] -2 * fq[1] * fq[3]);//64us   //变负数
//	pimu[Rec_channel].syaw=my_atan2((1-2*fq[2]*fq[2]-2*fq[3]*fq[3])*10000,(2*(fq[1]*fq[2]+fq[0]*fq[3]))*10000)>>8;//17us	
	pimu[Rec_channel].fyaw=atan2f((2*(fq[1]*fq[2]+fq[0]*fq[3])),(1-2*fq[2]*fq[2]-2*fq[3]*fq[3]));
	pimu[Rec_channel].syaw=pimu[Rec_channel].fyaw*57.3f;
	//*roll		=		my_atan2((-2*q1*q1-2*q2*q2+1)*10000,(2*q2*q3+2*q0*q1)*10000)>>8;	
//	pimu[Rec_channel].sroll=my_atan2((-2*fq[1]*fq[1]-2*fq[2]*fq[2]+1)*10000,(2*fq[2]*fq[3]+2*fq[0]*fq[1])*10000)>>8;	
	pimu[Rec_channel].froll=atan2f((2*fq[2]*fq[3]+2*fq[0]*fq[1]),(-2*fq[1]*fq[1]-2*fq[2]*fq[2]+1));
	pimu[Rec_channel].sroll=pimu[Rec_channel].froll*57.3f;
	return 1;
}

void robot_Hand_syn_cal(IMU_HandleTypeDef *pimu,uint8_t Rec_channel,short *OUT_P){
		static float ra,rc,rd,rf;//顺时针转为0,逆时针为140;
		static float la,lc,ld,lf;//顺时针转为0,逆时针为140;
		static short l_flag,r_flag;//放下保护标记
		static short OUT1,OUT2;
		static short syaw;
		syaw=get_acdf(pimu,Rec_channel);
		switch(Rec_channel){
			case 0:{//RIGHT_HAND_M
				if(!(syaw<0&&pimu[Rec_channel].fpitch>0)){//这句判断是保护第一自由度
					ra=tanf(pimu[Rec_channel].fyaw);
					rc=tanf(pimu[Rec_channel].fpitch)/cosf(pimu[Rec_channel].fyaw);
					OUT1=atan2f(rc,ra)*57.3f;
					OUT2=atan2f(1,sqrtf(rc*rc+ra*ra))*57.3f;
					//如果OUT1在后面,就让第三自由度朝前
					//如果OUT1在下面(-90),判断OUT2是否也在下面,是就让第三自由度朝前					
					if(((OUT1<-70))&&(OUT2<25))	
						r_flag=1;
					else 
						r_flag=0;
					OUT_P[13]=P13 + OUT2*11;
					OUT_P[12]=P12 + OUT1*8+ 720; //1805
					if(r_flag&&(OUT_P[13]<638))
						OUT_P[13]=638;
				}
				break;
			}
			case 1:{//RIGHT_HAND_L
				if(pimu[Rec_channel].fyaw<pimu[0].fyaw-1.57f)
					pimu[Rec_channel].fyaw=pimu[0].fyaw-1.57f;
				rd=tanf(pimu[Rec_channel].fyaw);
				rf=tanf(pimu[Rec_channel].fpitch)/cosf(pimu[Rec_channel].fyaw);
				//手放下会碰到身体,修改手放下后第三自由度角度
				if(r_flag)
					OUT1=90;
				else
					OUT1=-atan2f((rc-ra*rf)*sqrtf(ra*ra+rc*rc+1),rc*(rf+rc*rd)+ra*(ra*rd+1))*57.3f;
				OUT2=-57.3f*acosf((ra-rd+rc*rf)/sqrtf((1+rd*rd+rf*rf)*(1+ra*ra+rc*rc)));				
				
				OUT_P[15]=P15 +OUT1*11;
				OUT_P[17]=P17 +OUT2*11;
				
//				sprintf(uart_send_buf,"#15P%d#17P%dT2\r\n",OUT1*11+1500,OUT2*11+1500);
//				usart1_need_send|=1<<2;
				break;
			}
			case 2:{//LEFT_HAND_M
				if(!(syaw<0&&pimu[Rec_channel].fpitch>0)){
					la=tanf(pimu[Rec_channel].fyaw);
					lc=tanf(pimu[Rec_channel].fpitch)/cosf(pimu[Rec_channel].fyaw);

					OUT1=atan2f(lc,la)*57.3f;
					OUT2=atan2f(1,sqrtf(lc*lc+la*la))*57.3f;		
					if(((OUT1<-70))&&(OUT2<25))	
						l_flag=1;
					else 
						l_flag=0;
					OUT_P[11]=P11-OUT1*8 -720 ;//1140
//					OUT_P[10]=2640-OUT2*11;//为什么2640
					OUT_P[10]=P10-OUT2*11;
					if(l_flag&&(OUT_P[10]>2370))
						OUT_P[10]=2370;
//					sprintf(uart_buf,"#11P%d#10P%dT2\r\n",1140-OUT1*8,2640-OUT2*11);
//					usart1_need_send|=1<<2;
				}
				break;
			}
			case 3:{//LEFT_HAND_L
				if(pimu[Rec_channel].fyaw<pimu[2].fyaw-1.57f)
					pimu[Rec_channel].fyaw=pimu[2].fyaw-1.57f;
				ld=tanf(pimu[Rec_channel].fyaw);
				lf=tanf(pimu[Rec_channel].fpitch)/cosf(pimu[Rec_channel].fyaw);
				if(l_flag)
					OUT1=-90;
				else
					OUT1=atan2f(((lc-la*lf)*sqrtf(la*la+lc*lc+1)),(lc*(lf+lc*ld)+la*(la*ld+1)))*57.3f;
				OUT2=57.3f*acosf((la-ld+lc*lf)/sqrtf((1+ld*ld+lf*lf)*(1+la*la+lc*lc)));				
				OUT_P[9]=P9+OUT1*11;
				OUT_P[8]=P8+OUT2*11;
				break;
			}
	}
}


