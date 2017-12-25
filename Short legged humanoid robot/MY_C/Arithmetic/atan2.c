//#include "sys.h"
//#include "timer.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <math.h>
#include "atan2.h"
uint8_t read_key0_flag(void);
const short tan_angle[] = {11520, 6801, 3593, 1824, 916, 458, 229, 115, 57, 29, 14, 7, 4, 2, 1};	
const float sin_num[]={
0.008726 ,	
0.026176 , 
0.043618 ,
0.061046 ,
0.078456 ,
0.095842 ,
0.113199 ,
0.130521 ,
0.147804 ,
0.165041 ,
0.182229 ,
0.199360 ,
0.216431 ,
0.233436 ,
0.250370 ,
0.267228 ,
0.284005 ,
0.300694 ,
0.317293 ,
0.333794 ,
0.350194 ,
0.366487 ,
0.382669 ,
0.398734 ,
0.414677 ,
0.430495 ,
0.446181 ,
0.461731 ,
0.477141 ,
0.492405 ,
0.507519 ,
0.522479 ,
0.537279 ,
0.551916 ,
0.566385 ,
0.580681 ,
0.594800 ,
0.608738 ,
0.622491 ,
0.636054 ,
0.649423 ,
0.662595 ,
0.675564 ,
0.688328 ,
0.700883 ,
0.713223 ,
0.725347 ,
0.737249 ,
0.748927 ,
0.760377 ,
0.771595 ,
0.782578 ,
0.793323 ,
0.803826 ,
0.814085 ,
0.824095 ,
0.833854 ,
0.843359 ,
0.852608 ,
0.861596 ,
0.870323 ,
0.878784 ,
0.886977 ,
0.894900 ,
0.902551 ,
0.909927 ,
0.917025 ,
0.923844 ,
0.930382 ,
0.936637 ,
0.942606 ,
0.948288 ,
0.953681 ,
0.958783 ,
0.963594 ,
0.968111 ,
0.972333 ,
0.976259 ,
0.979887 ,
0.983217 ,
0.986248 ,
0.988978 ,
0.991407 ,
0.993534 ,
0.995358 ,
0.996879 ,
0.998097 ,
0.999010 ,
0.999619 ,
0.999924 ,
};
short asin_MY(float x){
	uint8_t j,temp=64;
	if(x>sin_num[64]){
		if(x>sin_num[88]){
			if(x>sin_num[89])
				return 90;
			else 
				return 89;
		}
		j=16;
	}
	else{
		j=32;
	}
	for(;j!=0;j>>=1){
		temp=(x > sin_num[temp] )?(temp+j):(temp-j);
	}
	if(x>sin_num[temp])
		temp++;
	else if(x<sin_num[temp-1])
		temp--;
	return temp;
}
	



/*******************************************************************************
快速计算 1/Sqrt(x)，源自雷神3的一段代码，神奇的0x5f3759df！比正常的代码快4倍 	
*******************************************************************************/
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
float sin_MY(float x) 
{
//	const float B = 1.273239f; // 4/pi
//	const float C = -0.405284f;// -4/(pi*pi)
	 x =  1.273239f * x - 0.405284f * x * fabs(x);
	 x =  0.225 * (x * fabs(x) - x) + x;
	 return x;
}

int my_atan2(int x, int y)
{
	char x_flag=0;
	char i;
	 int x_new, y_new;
	 int angleSum = 0;
	if(x<0)
	{
		x_flag=1;
		x*=-1;
	}
//    x *= 1024;//
//    y *= 1024;

    for(i = 0; i < 8; i++)//15
    {
        if(y > 0)
        {
            x_new = x + (y >> i);
            y_new = y - (x >> i);
            x = x_new;
            y = y_new;
            angleSum += tan_angle[i];
        }
        else
        {
            x_new = x - (y >> i);
            y_new = y + (x >> i);
            x = x_new;
            y = y_new;
            angleSum -= tan_angle[i];
        }
        //printf("Debug: i = %d angleSum = %d, angle = %d\n", i, angleSum, angle[i]);
    }
  if(x_flag)
	{
		if(angleSum>0)
			angleSum= 46080-angleSum;
		else 
			angleSum=-46080-angleSum;
	}
	return angleSum;
}

/*
app 生成串口数
输入:
roll,pitch,yaw
输出:

*/
uint8_t read_key0_flag(void);

uint16_t quick_mid_value(uint16_t *buf,uint8_t len){
	uint8_t i,j,max,min,mid,num=len>>1;
	for(j=0;j<len;j++){
		max=0;
		min=0;
		mid=0;
		for(i=0;i<len;i++){
			if(i!=j){
				if(buf[j]>buf[i])
					max++;
				else if(buf[j]<buf[i])
					min++;
				else
					mid++;
			}
		}
		if((max==min)||(max==mid+min)||max+mid==min||((max+mid>=num)&&(min+mid>=num))){
			return buf[j];
		}
	}
	return 0;
}
