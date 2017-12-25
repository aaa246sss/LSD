#ifndef __MPU6050_H__
#define __MPU6050_H__
#include "sys.h"

#define q30  1073741824.0f
#define q30c32785 32768
#define AHRS_Kp 2.0f     //proportional gain governs rate of convergence to accelerometer/magnetometer
#define AHRS_Ki 0.005f   //integral gain governs rate of convergence of gyroscope biases

#define i2c_write   IIC_Write
#define i2c_read    IIC_Read
#define inv_delay_ms    delay_System_ms

#define Gyro_Xout_Offset	    -70
#define Gyro_Yout_Offset		25
#define Gyro_Zout_Offset		-10

#define Accel_FILTER_98HZ		98
#define Accel_Xout_H		    0x3B
#define Accel_Zout_Offset		600
#define Bypass_Enable_Cfg		0x37
#define P_M_1                   0x6B
#define MPU9150_Addr            0x68
#define User_Ctrl               0x6A

//定义不同测量范围的刻度因子
#define Gyro_250_Scale_Factor   131.0f
#define Gyro_500_Scale_Factor   65.5f
#define Gyro_1000_Scale_Factor  32.8f
#define Gyro_2000_Scale_Factor  16.4f
#define Accel_2_Scale_Factor    16384.0f
#define Accel_4_Scale_Factor    8192.0f
#define Accel_8_Scale_Factor    4096.0f
#define Accel_16_Scale_Factor   2048.0f

extern short int Roll,Yaw;
extern float Pitch;
extern short gyro[3], accel[3];

char UP_quat_MPU6050(short *sq);
char get_oula_MPU9150_IMU(void);
void MPU9150_IMU_AHRS(void);
void MPU6050_Init(void);
char get_oula_MPU6050(short * roll,float * pitch,short * yaw);
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz);
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az);
#endif
