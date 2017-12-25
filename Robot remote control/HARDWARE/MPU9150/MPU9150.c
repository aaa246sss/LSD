/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
//#include "stm32f10x.h"
//#define COMPASS_ENABLED	
#include "mltypes.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "packet.h"
#include "sys.h"
#include "myiic.h"
#include "delay.h"
#include "timer.h"
#include "MPU9150.h"
#include "atan2.h"
#include "stdio.h"
//#define MPU9250
//#define q30  1073741824.0f

float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
//short int Roll,Yaw;
//float 	Roll,Yaw;
//float 	Pitch;
//extern float int_gy;																					 
//int Roll_test;																					 
																					 
unsigned long sensor_timestamp;
short gyro_2[3],gyro[3], accel[3], sensors;
unsigned char more;
long quat[4];
//float quat_verify,quat_verify_2;


char IICx=0;
//short int Roll,Yaw;
//float 	Pitch;
short compass_short[3];

//#include "board-st_discovery.h"
/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (50)   

#define PEDO_READ_MS    (1000)
#define TEMP_READ_MS    (500)
#define COMPASS_READ_MS (40)//默认值为100ms ,即10Hz的输出频率,已经被我改为了25Hz的输出频率
    
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)



unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";   

ErrorStatus HSEStartUpStatus;  	 


struct platform_data_s {
    signed char orientation[9];
};

static struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};
#if defined MPU9150 || defined MPU9250
static struct platform_data_s compass_pdata = {
    .orientation = { 0, 1, 0,
                     1, 0, 0,
                     0, 0, -1}
};
#define COMPASS_ENABLED 1 
#elif defined AK8975_SECONDARY 
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0, 1, 0,
                     0, 0,-1}
};
#define COMPASS_ENABLED 1
#elif defined AK8963_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0,-1, 0,
                     0, 0, 1}
};



#define COMPASS_ENABLED 1
#endif

static void android_orient_cb(unsigned char orientation)
{
//	switch (orientation) {
//	case ANDROID_ORIENT_PORTRAIT:
//        MPL_LOGI("Portrait\n");
//        break;
//	case ANDROID_ORIENT_LANDSCAPE:
//        MPL_LOGI("Landscape\n");
//        break;
//	case ANDROID_ORIENT_REVERSE_PORTRAIT:
//        MPL_LOGI("Reverse Portrait\n");
//        break;
//	case ANDROID_ORIENT_REVERSE_LANDSCAPE:
//        MPL_LOGI("Reverse Landscape\n");
//        break;
//	default:
//		return;
//	}
}


void MPU9150_init(unsigned long *timestamp)
{
    inv_error_t result;
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;

    struct int_param_s int_param;

#ifdef COMPASS_ENABLED
   
    unsigned short compass_fsr;
#endif
    
    IICx=IIC1;
    IIC_Init();/*3?ê??ˉIIC1*/
    while(1)
    {
      result = mpu_init(&int_param);
      if (result) {
        MPL_LOGE("Could not initialize gyro.\n");
        delay(200);
      }
      else
        break;
    }
    
    
     /* If you're not using an MPU9150 AND you're not using DMP features, this
     * function will place all slaves on the primary bus.
     * mpu_set_bypass(1);
     */
    result = inv_init_mpl();
    if (result) {
        MPL_LOGE("Could not initialize MPL.\n");
    }
    /* Compute 6-axis and 9-axis quaternions. */
    inv_enable_quaternion();
    inv_enable_9x_sensor_fusion();
    /* The MPL expects compass data at a constant rate (matching the rate
     * passed to inv_set_compass_sample_rate). If this is an issue for your
     * application, call this function, and the MPL will depend on the
     * timestamps passed to inv_build_compass instead.
     *
     * inv_9x_fusion_use_timestamps(1);
     */

    /* This function has been deprecated.
     * inv_enable_no_gyro_fusion();
     */

    /* Update gyro biases when not in motion.
     * WARNING: These algorithms are mutually exclusive.
     */
    inv_enable_fast_nomot();
    /* inv_enable_motion_no_motion(); */
    /* inv_set_no_motion_time(1000); */

    /* Update gyro biases when temperature changes. */
    inv_enable_gyro_tc();

    /* This algorithm updates the accel biases when in motion. A more accurate
     * bias measurement can be made when running the self-test (see case 't' in
     * handle_input), but this algorithm can be enabled if the self-test can't
     * be executed in your application.
     *
     * inv_enable_in_use_auto_calibration();
     */
#ifdef COMPASS_ENABLED
    /* Compass calibration algorithms. */
    inv_enable_vector_compass_cal();
    inv_enable_magnetic_disturbance();
#endif
    /* If you need to estimate your heading before the compass is calibrated,
     * enable this algorithm. It becomes useless after a good figure-eight is
     * detected, so we'll just leave it out to save memory.
     * inv_enable_heading_from_gyro();
     */

    /* Allows use of the MPL APIs in read_from_mpl. */
    inv_enable_eMPL_outputs();

  result = inv_start_mpl();
  if (result == INV_ERROR_NOT_AUTHORIZED) {
      while (1) {
          MPL_LOGE("Not authorized.\n");
      }
  }
  if (result) {
      MPL_LOGE("Could not start the MPL.\n");
  }

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
#ifdef COMPASS_ENABLED
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
#else
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
#endif
    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
#ifdef COMPASS_ENABLED
    /* The compass sampling rate can be less than the gyro/accel sampling rate.
     * Use this function for proper power management.
     */
    mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
#endif
    /* Read back configuration in case it was set improperly. */
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
#ifdef COMPASS_ENABLED
    mpu_get_compass_fsr(&compass_fsr);
#endif
    /* Sync driver configuration with MPL. */
    /* Sample rate expected in microseconds. */
    inv_set_gyro_sample_rate(1000000L / gyro_rate);
    inv_set_accel_sample_rate(1000000L / gyro_rate);
#ifdef COMPASS_ENABLED
    /* The compass rate is independent of the gyro and accel rates. As long as
     * inv_set_compass_sample_rate is called with the correct value, the 9-axis
     * fusion algorithm's compass correction gain will work properly.
     */
    inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);
#endif
    /* Set chip-to-body orientation matrix.
     * Set hardware units to dps/g's/degrees scaling factor.
     */
    inv_set_gyro_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)gyro_fsr<<15);
    inv_set_accel_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)accel_fsr<<15);
#ifdef COMPASS_ENABLED
    inv_set_compass_orientation_and_scale(
            inv_orientation_matrix_to_scalar(compass_pdata.orientation),
            (long)compass_fsr<<15);
#endif


  /* Compass reads are handled by scheduler. */

   /* To initialize the DMP:
     * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
     *    inv_mpu_dmp_motion_driver.h into the MPU memory.
     * 2. Push the gyro and accel orientation matrix to the DMP.
     * 3. Register gesture callbacks. Don't worry, these callbacks won't be
     *    executed unless the corresponding feature is enabled.
     * 4. Call dmp_enable_feature(mask) to enable different features.
     * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
     * 6. Call any feature-specific control functions.
     *
     * To enable the DMP, just call mpu_set_dmp_state(1). This function can
     * be called repeatedly to enable and disable the DMP at runtime.
     *
     * The following is a short summary of the features supported in the DMP
     * image provided in inv_mpu_dmp_motion_driver.c:
     * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
     * 200Hz. Integrating the gyro data at higher rates reduces numerical
     * errors (compared to integration on the MCU at a lower sampling rate).
     * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
     * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
     * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
     * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
     * an event at the four orientations where the screen should rotate.
     * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
     * no motion.
     * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
     * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
     * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
     * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
     */
    dmp_load_motion_driver_firmware();
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
    dmp_register_android_orient_cb(android_orient_cb);
    /*
     * Known Bug -
     * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
     * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
     * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
     * there will be a 25Hz interrupt from the MPU device.
     *
     * There is a known issue in which if you do not enable DMP_FEATURE_TAP
     * then the interrupts will be at 200Hz even if fifo rate
     * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
     *
     * DMP sensor fusion works only with gyro at +-2000dps and accel +-2G
     */
    dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);

}
//void MPU6050_PITCH_90(void)
//{
////	u8 i;
////	static long int int_gyro[3];
//	static float Pitch_last=0,Pitch_last_last=0;
//	static short int flag_90=0;
//	//陀螺仪逆时针
//	if(Pitch>=65)
//	{
////		if(Pitch<=68)
////		{
//////			int_gyro[1]++;
//////			memset(int_gyro,0,sizeof(long int)*3);
////		}
//			
////		for(i=0;i<3;i++)	
////			int_gyro [i]+=gyro [i];
//			//int_gyro[0]=0;
//		//if((Pitch<Pitch_last)&&(Pitch_last>Pitch_last_last))
//		if((Pitch<Pitch_last)&&(Pitch_last>Pitch_last_last))//
//		{
//			//if((fabs(gyro[1])>fabs(gyro[2])?gyro[1]:gyro[2])>0)
//			//正为翻越90度
//			if(fabs(gyro[2])<fabs(gyro[1]))
//			{
//				if(gyro[1]>20)
//				{
//					flag_90=1;
//				}
//				else if(gyro[1]<-20)
//					flag_90=0;
//			}
//			else
//			{
//				if(gyro[2]>20)
//				{
//					flag_90=0;
//				}
//				else if(gyro[2]<-20)
//				{
//					flag_90=1;
//				}
//			}
////			else
////				flag_90=3;
//		}
//	}
//	else 	
//	if(Pitch<-65)
//	{
//		if((Pitch>Pitch_last)&&(Pitch_last<Pitch_last_last))//
//		{
//			//if((fabs(gyro[1])>fabs(gyro[2])?gyro[1]:gyro[2])>0)
//			//正为翻越90度
////			if(gyro[1]<-20)
////			{
////				flag_90=2;
////			}
////			else if(gyro[1]>20)
////				flag_90=0;
////			else
////				flag_90=3;++
//			
//			if(fabs(gyro[2])<fabs(gyro[1]))
//			{
//				if(gyro[1]<-20)
//				{
//					flag_90=2;
//				}
//				else if(gyro[1]>20)
//					flag_90=0;
//			}
//			else
//			{
//				if(gyro[2]<-20)
//				{
//					flag_90=0;
//				}
//				else if(gyro[2]>20)
//				{
//					flag_90=2;
//				}
//			}
//		}
//	}
//	if(fabs(Pitch-Pitch_last)>0.45)
//	{
//		Pitch_last_last=Pitch_last;
//		Pitch_last=Pitch;
//	}
//		if(flag_90==1)
//		Pitch=180-Pitch;
//	else if(flag_90==2)
//		Pitch=-180-Pitch;
//}


void YAW_cul(float * roll,float * pitch,short * yaw)
{
	static float Roll_last;
	if(*pitch*57.3>40)
		*yaw=*yaw-*roll*57.3+Roll_last;
	else if(*pitch<-59)
		{
			*yaw=*yaw+*roll*57.3-Roll_last;
		}	
	else
		{
			Roll_last=*roll*57.3;
		}
		if(*yaw<-180)
			*yaw+=360;
		else if (*yaw>180)
			*yaw-=360;
}


//void MPU6050_Pose(void)
//{
//	//LED=0;
////	float temp1,temp2;
////	int num1,num2;
////	IICx=IIC1;
//	//采样时间 2ms
//	dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more);	 //正确采样时间 2ms 错误采样时间 0.35ms
////	dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
//	if(sensors & INV_WXYZ_QUAT )
//	{
////		LED_R=1;
//		//计算时间 25us
//		//LED=0;
//		q0 = quat[0] / q30;	
//		q1 = quat[1] / q30;
//		q2 = quat[2] / q30;
//		q3 = quat[3] / q30;//这4句使用8us

//		check_num=0;	
////		Roll=my_atan2((-2*q1*q1-2*q2*q2+1)*10000,(2*q2*q3+2*q0*q1)*10000)>>8;//这句使用5us		
////		Yaw=my_atan2((q0*q0+q1*q1-q2*q2-q3*q3)*10000,(2*(q1*q2+q0*q3))*10000)>>8;//17us
//		Pitch = asin(2 * q1 * q3 - 2 * q0* q2);//64us
//		Roll=atan2(2*q2*q3+2*q0*q1,-2*q1*q1-2*q2*q2+1);//这句使用5us		
////		gyro[0]=filter_MY(gyro[0]);
////		gyro[1]=filter_MY(gyro[1]);
////		gyro[2]=filter_MY(gyro[2]);
////		MPU6050_PITCH_90();
////		YAW_cul();
//	}
//	else
//	{
//		check_num++;
////		LED_R=0;
//	}
//	//LED=1;
//	

//}

//float filter_MY(float temp)
//{
//	static float record;
//	static char flag=0;
//	static float last;
//	if(fabs(temp-last)>9000)
//	{
//			if(flag)
//			{
//				last=temp;
//				flag=0;
//				return temp;
//			}
//			else
//			{
//				//last=temp;
//				record=last;
//				last=temp;
//				flag=1;
//				return record;
//			}
//	}
//	else
//	{
//		last=temp;
//		if(flag)
//		{
//			
//			flag++;
//			if(flag==3)
//			{
//				flag=0;
//				return temp;
//			}
//				
//			return record;
//		}
//		else
//		{
//			return temp;
//		}
//	}
//}

char get_oula_MPU6050(float * roll,float * pitch,short * yaw)
{
	dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more);	 //正确采样时间 2ms 错误采样时间 0.35ms
	if(sensors & INV_WXYZ_QUAT )
	{
		q0 = quat[0] / q30;	
		q1 = quat[1] / q30;
		q2 = quat[2] / q30;
		q3 = quat[3] / q30;//这4句使用8us
		*pitch = asin(2 * q0* q2 - 2 * q1 * q3);//64us   //变负数
		*roll=atan2(2*q2*q3+2*q0*q1,-2*q1*q1-2*q2*q2+1);//这句使用5us	
		*yaw=my_atan2((q0*q0+q1*q1-q2*q2-q3*q3)*10000,(2*(q1*q2+q0*q3))*10000)>>8;//17us	
//		YAW_cul(roll,pitch,yaw);
		return 1;
	}
	return 0;
}
void get_compass_ak8975(short * compass)
{
	short Mag_Yaw;
	if (!mpu_get_compass_reg(compass, &sensor_timestamp))
	{
//		init_mx=compass_short[1];
//		init_my=compass_short[0];
//		init_mz=compass_short[2];
		//9~10次重复!!!!!
		//这句误差太大
//		Mag_Yaw = 57.295780 * atan2((compass_short[1]-178)*cos(Roll) + (compass_short[0]-265)*sin(Roll)*sin(Pitch) + (compass_short[2]-47)*sin(Roll)*cos(Pitch),(compass_short[0]-265)*cos(Pitch) - (compass_short[2]-47)*sin(Pitch));
//Mag_Yaw = 57.295780 * atan2((compass_short[1]-178),(compass_short[0]-265)*cos(Pitch));//这个计算是对的
		//		printf("X=%d,Y=%d,Z=%d\r\n",compass_short[1],compass_short[0],compass_short[2]);
		printf("%d\r\n",Mag_Yaw);
	}
}

void compass_adjustment(short * compass_error)
{
	char i,j;
	short compass_short[3];
	unsigned long sensor_timestamp;
	short compass[3][2];
	for(i=0;i<3;i++)
	{
		mpu_get_compass_reg(compass_short, &sensor_timestamp);
		delay(200);
	}
	for(j=0;j<3;j++)
		for(i=0;i<2;i++)
		compass[j][i]=compass_short[j];
	while(1)
	{
		if (!mpu_get_compass_reg(compass_short, &sensor_timestamp))
		{
			delay(50);
			for(i=0;i<3;i++)
			{
				if(compass_short[i]>compass[i][1])
				{
					compass[i][1]=compass_short[i];
					compass_error[i]=(compass[i][1]+compass[i][0])/2;
					printf("X=%d,Y=%d,Z=%d\r\n",compass_error[1],compass_error[0],compass_error[2]);
				}
				if(compass_short[i]<compass[i][0])
				{
					compass[i][0]=compass_short[i];
					compass_error[i]=(compass[i][1]+compass[i][0])/2;
					printf("X=%d,Y=%d,Z=%d\r\n",compass_error[1],compass_error[0],compass_error[2]);
				}
				
			}
		}
	}
}




