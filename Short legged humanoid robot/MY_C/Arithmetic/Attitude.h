#ifndef __ATTITUDE_H
#define __ATTITUDE_H 	

#define RADIAN 178
#define CIRCLE 360

#include "stm32f4xx_hal.h"
#include "MAIN_MY.h"

#define fq pimu[Rec_channel].q

#define P0	1434
#define P1  1300
#define P3  1400
#define P5  1504
#define P18  1515
#define P20  1558
#define P22  1727
#define P23  1514

#define P8	1487
#define P9  1593
#define P10  2383
#define P11  1873 //1140 
#define P12  1092	//1805 
#define P13  558
#define P15  1550
#define P17  1499

unsigned char CHECK_SUM_8bit(char *buf,unsigned char len);
short get_acdf(IMU_HandleTypeDef *pimu,unsigned char Rec_channel);
unsigned char UP_oula(IMU_HandleTypeDef *pimu,unsigned char Rec_channel);
void robot_Hand_syn_cal(IMU_HandleTypeDef *pimu,unsigned char Rec_channel,short *OUT_P);
#endif
