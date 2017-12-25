#include "sys.h"
#include "timer.h"
#include <stdio.h>
#include <math.h>
#include "atan2.h"
//u8 read_key0_flag(void);
const short tan_angle[] = {11520, 6801, 3593, 1824, 916, 458, 229, 115, 57, 29, 14, 7, 4, 2, 1};	


u8 CHECK_SUM_8bit(char *buf,u8 len){
	u8 SUM=0;
	for(u8 i=0;i<len;i++)
		SUM+=buf[i];
	return SUM;
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
short two_yaw_num_compare_short(short yaw_start,short yaw_end,u16 Range_Sizes)
{
	u16 T2=Range_Sizes/2;
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

/*
功能
*/
#define RADIAN 1500
#define CIRCLE 3960
u8 read_rest_KEY_status_flag(void);
short two_yaw_num_compare_short(short yaw_start,short yaw_end,u16 Range_Sizes);



