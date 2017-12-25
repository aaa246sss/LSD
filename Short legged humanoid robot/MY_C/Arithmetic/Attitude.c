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
���� �Ƚ�������תƫ����(float��)�Ĵ�С
����:
yaw_start:ƫ���ǿ�ʼ��λ��
yaw_end:	ƫ���ǽ�����λ��
Range_Sizes:	ƫ���ǵķ�Χ�ߴ��С
���� error_temp
error_temp:	������ʾ��������ת
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
���� �Ƚ�������תƫ����(short��)�Ĵ�С
����:
yaw_start:ƫ���ǿ�ʼ��λ��
yaw_end:	ƫ���ǽ�����λ��
Range_Sizes:	ƫ���ǵķ�Χ�ߴ��С
���� error_temp
error_temp:	������ʾ��������ת
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
���� ��ȡ����תƫ����(float��)��ֵ
����:
yaw_start:ƫ���ǿ�ʼ��λ��
error_temp:	������ʾ��������ת

Range_Sizes:	ƫ���ǵķ�Χ�ߴ��С
���� yaw_new:	ƫ�������µ�λ��

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
���� ��ȡ����תƫ����(short��)��ֵ
����:
yaw_start:ƫ���ǿ�ʼ��λ��
error_temp:	������ʾ��������ת

Range_Sizes:	ƫ���ǵķ�Χ�ߴ��С
���� yaw_new:	ƫ�������µ�λ��

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
				//���Ŀ���Ǹ����˰� ,��������ֻ����һ��
				init[Rec_channel]=1;
				if(only_once_flag[Rec_channel]){
					only_once_flag[Rec_channel]=0;
					fazhi[Rec_channel]=(yaw_min[Rec_channel]+yaw_max[Rec_channel]+CIRCLE+(yaw_max[Rec_channel]>yaw_min[Rec_channel]?0:CIRCLE))>>1;
					if(fazhi[Rec_channel]>CIRCLE)fazhi[Rec_channel]-=CIRCLE;
				//	KEY_status=1;
				}
					if(yaw_uart_init[Rec_channel]-fazhi[Rec_channel]+((yaw_uart_init[Rec_channel]>fazhi[Rec_channel])?0:CIRCLE)<(CIRCLE-RADIAN)>>1)//˳ʱ��	
						*YAW_out=0;	
					else if(yaw_uart_init[Rec_channel]-fazhi[Rec_channel]+((yaw_uart_init[Rec_channel]>fazhi[Rec_channel])?0:CIRCLE)>(CIRCLE+RADIAN)>>1)
						*YAW_out=RADIAN;	
					else{
						*YAW_out=CIRCLE+yaw_uart_init[Rec_channel]-yaw_min[Rec_channel];
						if(*YAW_out>=CIRCLE)
							*YAW_out-=CIRCLE;
					}
		}
		else{ //��ʼʱӦ��ִ�����
			only_once_flag[Rec_channel]=1;
			if(two_yaw_num_compare_short(yaw_uart_last[Rec_channel],yaw_uart_init[Rec_channel],CIRCLE)>0){//yaw_uart_init��yaw_uart_last������ת����	
				if(two_yaw_num_compare_short( yaw_uart_init[Rec_channel],yaw_max[Rec_channel],CIRCLE)<0){//yaw_max��yaw_uart_init�ĸ���ת����,��Ҫ����yaw_max
					//����yaw_max��yaw_min״̬
					yaw_max[Rec_channel]=yaw_uart_init[Rec_channel];
					if(yaw_max[Rec_channel]>RADIAN)
						yaw_min[Rec_channel]=yaw_max[Rec_channel]-RADIAN;
					else
						yaw_min[Rec_channel]=CIRCLE +yaw_max[Rec_channel] - RADIAN;
				}
			}
			else if((yaw_uart_last[Rec_channel]-yaw_uart_init[Rec_channel]!=0)&&two_yaw_num_compare_short( yaw_uart_init[Rec_channel],yaw_min[Rec_channel],CIRCLE)>0){//yaw_uart_init��yaw_uart_last�ĸ���ת����, yaw_min��yaw_uart_init������ת����,��Ҫ����yaw_min��yaw_max
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
	pimu[Rec_channel].fpitch=asinf(2 * fq[0]* fq[2] -2 * fq[1] * fq[3]);//64us   //�为��
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
	pimu[Rec_channel].fpitch=asinf(2 * fq[0]* fq[2] -2 * fq[1] * fq[3]);//64us   //�为��
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
		static float ra,rc,rd,rf;//˳ʱ��תΪ0,��ʱ��Ϊ140;
		static float la,lc,ld,lf;//˳ʱ��תΪ0,��ʱ��Ϊ140;
		static short l_flag,r_flag;//���±������
		static short OUT1,OUT2;
		static short syaw;
		syaw=get_acdf(pimu,Rec_channel);
		switch(Rec_channel){
			case 0:{//RIGHT_HAND_M
				if(!(syaw<0&&pimu[Rec_channel].fpitch>0)){//����ж��Ǳ�����һ���ɶ�
					ra=tanf(pimu[Rec_channel].fyaw);
					rc=tanf(pimu[Rec_channel].fpitch)/cosf(pimu[Rec_channel].fyaw);
					OUT1=atan2f(rc,ra)*57.3f;
					OUT2=atan2f(1,sqrtf(rc*rc+ra*ra))*57.3f;
					//���OUT1�ں���,���õ������ɶȳ�ǰ
					//���OUT1������(-90),�ж�OUT2�Ƿ�Ҳ������,�Ǿ��õ������ɶȳ�ǰ					
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
				//�ַ��»���������,�޸��ַ��º�������ɶȽǶ�
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
//					OUT_P[10]=2640-OUT2*11;//Ϊʲô2640
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


