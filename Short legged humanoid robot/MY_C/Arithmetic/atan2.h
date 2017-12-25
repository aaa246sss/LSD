#ifndef __ATAN2_H
#define __ATAN2_H

int my_atan2(int x, int y);
float sin_MY(float x) ;
#define cos_MY(x) sin_MY(x+1.570796f) 
#define tan_MY(x) (sin_MY(x)/cos_MY(x))
float invSqrt(float x) ;
void YAW_shanxing(char Rec_channel,char key0_flag, short yaw,short * YAW_out);
short asin_MY(float x);
#endif
