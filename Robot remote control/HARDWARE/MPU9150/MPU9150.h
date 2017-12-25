#ifndef __MPU9150_H__
#define __MPU9150_H__

#define q30  1073741824.0f

#define COMP_R_ERR (178)
#define COMP_P_ERR (265)
#define COMP_Y_ERR (47)

void MPU9150_init(unsigned long *timestamp);
void get_compass_ak8975(short * compass);
int  mpu_get_compass_reg(short *data, unsigned long *timestamp);
char get_oula_MPU6050(float * roll,float * pitch,short * yaw);
void MPU6050_Pose(void);
void compass_adjustment(short * compass_error);

#endif
