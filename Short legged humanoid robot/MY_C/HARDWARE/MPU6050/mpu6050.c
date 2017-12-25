#include "mpu6050.h"
#include "myiic.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "usart.h"
#include "math.h"
#include "timer.h"
#include "atan2.h"
#include "string.h"
//#include "LED.h"
#include "delay.h"
//extern char IICx;



static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;

unsigned long sensor_timestamp;																				 
//AHRS	
unsigned char data_write[14];																					 
float init_ax, init_ay, init_az, init_gx, init_gy, init_gz, init_mx, init_my, init_mz;
float exInt = 0, eyInt = 0, ezInt = 0;        // scaled integral error																					 
/*******************************************************************************
* Function Name  : init_quaternion
* Description    : 算出初始化四元数q0 q1 q2 q3.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void init_quaternion(void){ 
  unsigned long timestamp;
  signed short int accel[3], mag[3];
  float init_Yaw, init_Pitch, init_Roll;
  int i;
  if(!i2c_read(MPU9150_Addr, Accel_Xout_H, 6, data_write)){
	  accel[0]=(signed short int)((data_write[0]<<8) | data_write[1]);
	  accel[1]=(signed short int)((data_write[2]<<8) | data_write[3]);
	  accel[2]=(signed short int)((data_write[4]<<8) | data_write[5])+ Accel_Zout_Offset;
	  	    
	  init_ax=(float)(accel[0] / Accel_4_Scale_Factor);	   //单位转化成重力加速度的单位：m/s2
	  init_ay=(float)(accel[1] / Accel_4_Scale_Factor);
    init_az=(float)(accel[2] / Accel_4_Scale_Factor);
	  printf("ax=%f,   ay=%f,   az=%f", init_ax, init_ay, init_az);
	  for(i=0;i<5;i++){   //第一次读取的compsaa数据是错的，因此要多读几次保证以后数据正确，芯片bug
	  
        mpu_set_bypass(1);                     //开启bypass，必须有这句代码
        mpu_get_compass_reg(mag, &timestamp);  //读取compass数据
        //进行x y轴的校准，未对z轴进行校准，参考MEMSense的校准方法 
        init_mx =(float)mag[1]-8;						
        init_my =(float)1.046632*mag[0]-1.569948;
        init_mz =(float)-mag[2];
        mpu_set_bypass(0);						//关闭bypass，必须有这句代码
        printf("    mx=%f,   my=%f,   mz=%f \n\r", init_mx, init_my, init_mz);
	  }
//陀螺仪y轴为前进方向    
		init_Roll = -atan2(init_ax, init_az);    //算出的单位是弧度，如需要观察则应乘以57.3转化为角度
		init_Pitch=  asin(init_ay);              //init_Pitch = asin(ay / 1);      
		init_Yaw  =  atan2(init_mx*cos(init_Roll) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Roll)*cos(init_Pitch),
											 init_my*cos(init_Pitch) - init_mz*sin(init_Pitch));//类似于atan2(my, mx)，其中的init_Roll和init_Pitch是弧度
		if(init_Yaw < 0){init_Yaw = init_Yaw + 2*3.141593;}
		if(init_Yaw > 360){init_Yaw = init_Yaw - 2*3.141593;}				            
		//将初始化欧拉角转换成初始化四元数，注意sin(a)的位置的不同，可以确定绕xyz轴转动是Pitch还是Roll还是Yaw，按照ZXY顺序旋转,Qzyx=Qz*Qy*Qx，其中的init_YawRollPtich是角度        
		q0 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) - sin(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //w
		q1 = cos(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw) - sin(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw);  //x   绕x轴旋转是pitch
		q2 = sin(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) + cos(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //y   绕y轴旋转是roll
		q3 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw) + sin(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw);  //z   绕z轴旋转是Yaw
					
		printf("初始化四元数：Yaw=%f, Pitch=%f, Roll=%f \n\r", init_Yaw*57.295780, init_Pitch*57.295780, init_Roll*57.295780);
  }
}																					 

/***************************************************************************************************************************************
* Function Name  : AHRSupdate
* Description    : accel gyro mag的融合算法，源自S.O.H. Madgwick
* Input          : None
* Output         : None
* Return         : None
// q0 q1 q2 q3需要初始化才能带入到下面的程序中，不能直接使用1 0 0 0进行下面的计算，整个步骤为：
// 1.首先校准accle gyro mag；
// 2.调用init_quaternion，根据1中accle的xyz轴数据，并利用公式计算出初始化欧拉角，
//   其中ACCEL_1G=9.81，单位都是m/s2，而init_Yaw可以用磁力计计算出来；
// 3.根据自己的采样周期，来调整halfT，halfT=采样周期/2，采样周期为执行1次AHRSupdate所用的时间；
// 4.将2中计算出的欧拉角转化为初始化的四元数q0 q1 q2 q3，融合加速度计，陀螺仪，算出更新后的欧拉角pitch和roll，然后使用pitch roll和磁力计的数据进行互补滤波融合得到Yaw，即可使用，但是欧拉角有奇点；
// 5.或直接使用四元数；
// 6.重复4，即可更新姿态;

//总的来说，核心是陀螺仪，加速度计用来修正补偿Pitch和Roll，磁力计用来修正补偿Yaw;
//以下程序中，gx, gy, gz单位为弧度/s，ax, ay, az为加速度计输出的原始16进制数据, mx, my, mz为磁力计输出的原始16进制数据；
//前进方向：mpu9150的加速度计和陀螺仪的x轴为前进方向;
//以下程序采用的参考方向为：mpu9150的加速度计和陀螺仪所指的xyz方向为正方向；

//在量程为正负500度/s的前提下，陀螺仪的灵敏度是65.5LSB/度/s，所以把陀螺仪输出的十六进制数据除以65.5就是角速度，单位是°/s，
//然后再除以57.3就变成弧度制;(1弧度=180/pi=57.3度)

//欧拉角单位为弧度radian，乘以57.3以后转换为角度,0<yaw<360, -90<pitch<+90, -180<roll<180
***************************************************************************************************************************************/
void AHRSupdate(float gx, float gy, float gz, 
								float ax, float ay, float az, 
								float mx, float my, float mz) 
{
   float norm, halfT;
   float hx, hy, hz, bz, by;
   float vx, vy, vz, wx, wy, wz;
   float ex, ey, ez;
   float Pitch, Roll, Yaw;

/*方便之后的程序使用，减少计算时间*/
        //auxiliary variables to reduce number of repeated operations，
   float q0q0 = q0*q0;
   float q0q1 = q0*q1;
   float q0q2 = q0*q2;
   float q0q3 = q0*q3;
   float q1q1 = q1*q1;
   float q1q2 = q1*q2;
   float q1q3 = q1*q3;
   float q2q2 = q2*q2;   
   float q2q3 = q2*q3;
   float q3q3 = q3*q3;
          
/*归一化测量值，加速度计和磁力计的单位是什么都无所谓，因为它们在此被作了归一化处理*/        
   //normalise the measurements
   norm = invSqrt(ax*ax + ay*ay + az*az);       
   ax = ax * norm;
   ay = ay * norm;
   az = az * norm;
   norm = invSqrt(mx*mx + my*my + mz*mz);          
   mx = mx * norm;
   my = my * norm;
   mz = mz * norm;         
        
/*从机体坐标系的电子罗盘测到的矢量转成地理坐标系下的磁场矢量hxyz（测量值），下面这个是从飞行器坐标系到世界坐标系的转换公式*/
   //compute reference direction of flux
   hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
   hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
   hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);//这句的三个公式与472到474行重复,可优化

/*计算地理坐标系下的磁场矢量bxyz（参考值）。
因为地理地磁水平夹角，我们已知是0度（抛去磁偏角的因素，固定向北），我定义by指向正北，所以by=某值，bx=0
但地理参考地磁矢量在垂直面上也有分量bz，地球上每个地方都是不一样的。
我们无法得知，也就无法用来融合（有更适合做垂直方向修正融合的加速度计），所以直接从测量值hz上复制过来，bz=hz。
磁场水平分量，参考值和测量值的大小应该是一致的(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))。
因为bx=0，所以就简化成(by*by)  = ((hx*hx) + (hy*hy))。可算出by。这里修改by和bx指向可以定义哪个轴指向正北*/
//   bx = sqrtf((hx*hx) + (hy*hy));
   by = sqrtf((hx*hx) + (hy*hy));
   bz = hz;        
    
   // estimated direction of gravity and flux (v and w)，下面这个是从世界坐标系到飞行器坐标系的转换公式(转置矩阵)
	 //重力计算
   vx = 2*(q1q3 - q0q2);
   vy = 2*(q0q1 + q2q3);
   vz = q0q0 - q1q1 - q2q2 + q3q3;

/*我们把地理坐标系上的磁场矢量bxyz，转到机体上来wxyz。
因为bx=0，所以所有涉及到bx的部分都被省略了。同理by=0，所以所有涉及到by的部分也可以被省略，这根据自己定义那个轴指北有关。
类似上面重力vxyz的推算，因为重力g的az=1，ax=ay=0，所以上面涉及到gxgy的部分也被省略了
你可以看看两个公式：wxyz的公式，把by换成ay（0），把bz换成az（1），就变成了vxyz的公式了（其中q0q0+q1q1+q2q2+q3q3=1）。*/
//   wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
//   wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
//   wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);
   wx = 2*by*(q1q2 + q0q3) + 2*bz*(q1q3 - q0q2);
   wy = 2*by*(0.5 - q1q1 - q3q3) + 2*bz*(q0q1 + q2q3);
   wz = 2*by*(q2q3 - q0q1) + 2*bz*(0.5 - q1q1 - q2q2);
           
//现在把加速度的测量矢量和参考矢量做叉积，把磁场的测量矢量和参考矢量也做叉积。都拿来来修正陀螺。
   // error is sum of cross product between reference direction of fields and direction measured by sensors
   ex = (ay*vz - az*vy) + (my*wz - mz*wy);
   ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
   ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
   
   halfT=GET_NOWTIME();		//得到每次姿态更新的周期的一半
   
   if(ex != 0.0f && ey != 0.0f && ez != 0.0f)      //很关键的一句话，原算法没有
   {
      // integral error scaled integral gain
      exInt = exInt + ex*AHRS_Ki * halfT;			   //乘以采样周期的一半
      eyInt = eyInt + ey*AHRS_Ki * halfT;
      ezInt = ezInt + ez*AHRS_Ki * halfT;
      // adjusted gyroscope measurements
      gx = gx + AHRS_Kp*ex + exInt;
      gy = gy + AHRS_Kp*ey + eyInt;
      gz = gz + AHRS_Kp*ez + ezInt;
   }         

   // integrate quaternion rate and normalise，四元数更新算法
   q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
   q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
   q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
   q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
        
   // normalise quaternion
   norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
   q0 = q0 * norm;       //w
   q1 = q1 * norm;       //x
   q2 = q2 * norm;       //y
   q3 = q3 * norm;       //z
        
///*由四元数计算出Pitch  Roll  Yaw
//乘以57.3是为了将弧度转化为角度*/
	Yaw   = -atan2(2*q1*q2 - 2*q0*q3, -2 * q1 * q1 - 2 * q3 * q3 + 1) * 57.3;  //偏航角，绕z轴转动	
    if(Yaw < 0 ){Yaw = Yaw + 360;}
	if(Yaw > 360 ){Yaw = Yaw - 360;}
	Pitch = asin(2*q2*q3 + 2*q0*q1) * 57.3; //俯仰角，绕x轴转动	 
  Roll  = -atan2(-2*q0*q2 + 2*q1*q3, -2 * q1 * q1 - 2 * q2* q2 + 1) * 57.3; //滚动角，绕y轴转动

/*最初的由四元数计算出Pitch  Roll  Yaw
Roll=arctan2(2wx+2yz, 1-2xx-2yy);
Pitch=arcsin(2wy-2zx);
Yaw=arctan2(2wz+2xy, 1-2yy-2zz);
1=q0*q0+q1*q1+q2*q2+q3*q3;
乘以57.3是为了将弧度转化为角度*/
	
//	Pitch = asin(-2*q1*q3 + 2*q0*q2) * 57.3; //俯仰角，绕y轴转动	 
//    Roll  = atan2(2*q2*q3 + 2*q0*q1,-2*q1*q1 - 2*q2*q2 + 1) * 57.3; //滚动角，绕x轴转动
//	Yaw   = atan2(2*q1*q2 + 2*q0*q3,-2*q2*q2 - 2*q3*q3 + 1) * 57.3;  //偏航角，绕z轴转动

//	printf("q0=%f, q1=%f, q2=%f, q3=%f, Yaw=%f, Pitch=%f, Roll=%f, halfT=%f \n\r", q0, q1, q2, q3, Yaw, Pitch, Roll, halfT);
    printf("Yaw=%f, Pitch=%f, Roll=%f \n\r", Yaw, Pitch, Roll);
}
void MPU9150_IMU_AHRS(void){
	int result=0;
	while(1){
		result=mpu_init();
		if(!result){	 		 
		
			printf("mpu initialization complete......\n ");		//mpu initialization complete	 	  
			mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);		//mpu_set_sensor
			data_write[0]=0x01;				//GX_PLL:0x01
			i2c_write(MPU9150_Addr, P_M_1, 1, data_write);
			mpu_set_gyro_fsr(500);
			mpu_set_accel_fsr(4);
			mpu_set_lpf(Accel_FILTER_98HZ);
			mpu_set_sample_rate(DEFAULT_MPU_HZ_AHRS);
			delay(2000);
			init_quaternion();   //得到初始化四元数
			break;
		}
		else
		{
			printf("error\r\n");
			delay(50);
		}
	}
}	
char get_oula_MPU9150_IMU(void){
	static signed short int accel[3], mag[3], gyro[3];
		if(!i2c_read(MPU9150_Addr, Accel_Xout_H, 14, data_write))
	 {
		accel[0]=(signed short int)((data_write[0]<<8) | data_write[1]);
		accel[1]=(signed short int)((data_write[2]<<8) | data_write[3]);
		accel[2]=(signed short int)((data_write[4]<<8) | data_write[5])   + Accel_Zout_Offset;
		gyro[0] =(signed short int)((data_write[8]<<8) | data_write[9])   + Gyro_Xout_Offset;
		gyro[1] =(signed short int)((data_write[10]<<8) | data_write[11]) + Gyro_Yout_Offset;
		gyro[2] =(signed short int)((data_write[12]<<8) | data_write[13]) + Gyro_Zout_Offset;
	     		 
	 	init_ax=(float)accel[0];	  
		init_ay=(float)accel[1];
		init_az=(float)accel[2];
		init_gx=(float)(gyro[0] * 0.000266);    //单位转化成：弧度/s，0.000266=1/(Gyro_500_Scale_Factor * 57.3)
		init_gy=(float)(gyro[1] * 0.000266);
		init_gz=(float)(gyro[2] * 0.000266);
	    
		data_write[6]=0x00;       
		data_write[7]=0x02;                     
    i2c_write(MPU9150_Addr, User_Ctrl, 1, data_write+6);	 //关闭MPU9150的I2C_MASTER模式，必须要有这句
		delay_us(100);            //这俩句之间的延迟至少24000
		i2c_write(MPU9150_Addr, Bypass_Enable_Cfg, 1, data_write+7);	 //开启bypass，必须有这句代码，综合这两句就是开启Bypass though

		 
		mpu_get_compass_reg(mag, &sensor_timestamp);  //读取compass数据
		//进行x y轴的校准，未对z轴进行校准，参考ST的校准方法 
		//这个校准方法应该有问题,没有对数据进行均衡处理
		init_mx =(float)(mag[1]-32.5f)*1.575f;						
		init_my =(float)(mag[0]-89);
		init_mz =(float)-mag[2];

		data_write[6]=0x20;
		data_write[7]=0x00;
		i2c_write(MPU9150_Addr, User_Ctrl, 1, data_write+6); //开启MPU9150的I2C_MASTER模式，必须要有这句
		delay_us(100);		   //这俩句之间的延迟至少24000
		i2c_write(MPU9150_Addr, Bypass_Enable_Cfg, 1, data_write+7);//关闭bypass，必须有这句代码，综合这两句就是关闭Bypass though
//		printf(" mx=%f, my=%f, mz=%f \n\r", init_mx,init_my,init_mz);
//      IMUupdate(init_gx, init_gy, init_gz, init_ax, init_ay, init_az, init_mx, init_my, init_mz);
    AHRSupdate(init_gx, init_gy, init_gz, init_ax, init_ay, init_az, init_mx, init_my, init_mz); 
  }   
	return 0;
}
void MPU6050_Init(void){
	int result=0;
//	IIC_Init();
	
	while(1)
	{
		result=mpu_init();
		if(!result)
		{	 		 
		
			PrintChar("mpu initialization complete......\n ");		//mpu initialization complete	 	  

			if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))		//mpu_set_sensor
			{}//		PrintChar("mpu_set_sensor complete ......\n");
			else
			{}//	PrintChar("mpu_set_sensor come across error ......\n");

			if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))	//mpu_configure_fifo
			{}//	PrintChar("mpu_configure_fifo complete ......\n");
			else
			{}//	PrintChar("mpu_configure_fifo come across error ......\n");

			if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))	   	  		//mpu_set_sample_rate
			{}//PrintChar("mpu_set_sample_rate complete ......\n");
			else
			{}//	PrintChar("mpu_set_sample_rate error ......\n");

			if(!dmp_load_motion_driver_firmware())   	  			//dmp_load_motion_driver_firmvare
			{}//	PrintChar("dmp_load_motion_driver_firmware complete ......\n");
			else
			{}//PrintChar("dmp_load_motion_driver_firmware come across error ......\n");

			if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) 	  //dmp_set_orientation
			{}//	PrintChar("dmp_set_orientation complete ......\n");
			else
			{}//	PrintChar("dmp_set_orientation come across error ......\n");

			if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
					DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
					DMP_FEATURE_GYRO_CAL))		   	 					 //dmp_enable_feature
			 {}//	PrintChar("dmp_enable_feature complete ......\n");
			else
			{}//	PrintChar("dmp_enable_feature come across error ......\n");

			if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))   	 			 //dmp_set_fifo_rate
			{}//	PrintChar("dmp_set_fifo_rate complete ......\n");
			else
			{}//	PrintChar("dmp_set_fifo_rate come across error ......\n");

			run_self_test();		//自检

			if(!mpu_set_dmp_state(1))
			{}//	PrintChar("mpu_set_dmp_state complete ......\n");
			else
			{}//	PrintChar("mpu_set_dmp_state come across error ......\n");
			break;
		}
		else
		{
			PrintChar("error\r\n");
			delay(50);
		}
	}
}



/*//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码*/
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz){
    u8 buf[6],res;  
	res=IIC_Read_Buffer(0x68,0X43,6,buf);
	if(res==1)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
/*得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
*/
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az){
    u8 buf[6],res;  
	res=IIC_Read_Buffer(0x68,0x3B,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
/*
//void MPU6050_PITCH_90(void){
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


//void YAW_cul(void){
//	static float Roll_last;
//	if(Pitch>40)
//		Yaw=Yaw-Roll+Roll_last;
//	else if(Pitch<-59)
//		{
//			Yaw=Yaw+Roll-Roll_last;
//		}	
//	else
//		{
//			Roll_last=Roll;
//		}
//		if(Yaw<-180)
//			Yaw+=360;
//		else if (Yaw>180)
//			Yaw-=360;
//}*/
char get_oula_MPU6050(short * roll,float * pitch,short * yaw){
	static long quat[4];
	static short sensors;
	static unsigned char more;
	static short gyro[3], accel[3];
	dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more);	 //正确采样时间 2ms 错误采样时间 0.35ms
	if(sensors & INV_WXYZ_QUAT )
	{
		q0 = quat[0] / q30;	
		q1 = quat[1] / q30;
		q2 = quat[2] / q30;
		q3 = quat[3] / q30;//这4句使用8us
		*pitch  =		asin(2 * q0* q2 -2 * q1 * q3 );//64us   //变负数
		*roll		=		my_atan2((-2*q1*q1-2*q2*q2+1)*10000,(2*q2*q3+2*q0*q1)*10000)>>8;//这句使用5us
		*yaw		=		my_atan2((q0*q0+q1*q1-q2*q2-q3*q3)*10000,(2*(q1*q2+q0*q3))*10000)>>8;//17us	
//		YAW_cul(roll,pitch,yaw);
		return 1;
	}
	return 0;
}

