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
* Description    : �����ʼ����Ԫ��q0 q1 q2 q3.
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
	  	    
	  init_ax=(float)(accel[0] / Accel_4_Scale_Factor);	   //��λת�����������ٶȵĵ�λ��m/s2
	  init_ay=(float)(accel[1] / Accel_4_Scale_Factor);
    init_az=(float)(accel[2] / Accel_4_Scale_Factor);
	  printf("ax=%f,   ay=%f,   az=%f", init_ax, init_ay, init_az);
	  for(i=0;i<5;i++){   //��һ�ζ�ȡ��compsaa�����Ǵ�ģ����Ҫ������α�֤�Ժ�������ȷ��оƬbug
	  
        mpu_set_bypass(1);                     //����bypass��������������
        mpu_get_compass_reg(mag, &timestamp);  //��ȡcompass����
        //����x y���У׼��δ��z�����У׼���ο�MEMSense��У׼���� 
        init_mx =(float)mag[1]-8;						
        init_my =(float)1.046632*mag[0]-1.569948;
        init_mz =(float)-mag[2];
        mpu_set_bypass(0);						//�ر�bypass��������������
        printf("    mx=%f,   my=%f,   mz=%f \n\r", init_mx, init_my, init_mz);
	  }
//������y��Ϊǰ������    
		init_Roll = -atan2(init_ax, init_az);    //����ĵ�λ�ǻ��ȣ�����Ҫ�۲���Ӧ����57.3ת��Ϊ�Ƕ�
		init_Pitch=  asin(init_ay);              //init_Pitch = asin(ay / 1);      
		init_Yaw  =  atan2(init_mx*cos(init_Roll) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Roll)*cos(init_Pitch),
											 init_my*cos(init_Pitch) - init_mz*sin(init_Pitch));//������atan2(my, mx)�����е�init_Roll��init_Pitch�ǻ���
		if(init_Yaw < 0){init_Yaw = init_Yaw + 2*3.141593;}
		if(init_Yaw > 360){init_Yaw = init_Yaw - 2*3.141593;}				            
		//����ʼ��ŷ����ת���ɳ�ʼ����Ԫ����ע��sin(a)��λ�õĲ�ͬ������ȷ����xyz��ת����Pitch����Roll����Yaw������ZXY˳����ת,Qzyx=Qz*Qy*Qx�����е�init_YawRollPtich�ǽǶ�        
		q0 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) - sin(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //w
		q1 = cos(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw) - sin(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw);  //x   ��x����ת��pitch
		q2 = sin(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) + cos(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //y   ��y����ת��roll
		q3 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw) + sin(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw);  //z   ��z����ת��Yaw
					
		printf("��ʼ����Ԫ����Yaw=%f, Pitch=%f, Roll=%f \n\r", init_Yaw*57.295780, init_Pitch*57.295780, init_Roll*57.295780);
  }
}																					 

/***************************************************************************************************************************************
* Function Name  : AHRSupdate
* Description    : accel gyro mag���ں��㷨��Դ��S.O.H. Madgwick
* Input          : None
* Output         : None
* Return         : None
// q0 q1 q2 q3��Ҫ��ʼ�����ܴ��뵽����ĳ����У�����ֱ��ʹ��1 0 0 0��������ļ��㣬��������Ϊ��
// 1.����У׼accle gyro mag��
// 2.����init_quaternion������1��accle��xyz�����ݣ������ù�ʽ�������ʼ��ŷ���ǣ�
//   ����ACCEL_1G=9.81����λ����m/s2����init_Yaw�����ô����Ƽ��������
// 3.�����Լ��Ĳ������ڣ�������halfT��halfT=��������/2����������Ϊִ��1��AHRSupdate���õ�ʱ�䣻
// 4.��2�м������ŷ����ת��Ϊ��ʼ������Ԫ��q0 q1 q2 q3���ںϼ��ٶȼƣ������ǣ�������º��ŷ����pitch��roll��Ȼ��ʹ��pitch roll�ʹ����Ƶ����ݽ��л����˲��ںϵõ�Yaw������ʹ�ã�����ŷ��������㣻
// 5.��ֱ��ʹ����Ԫ����
// 6.�ظ�4�����ɸ�����̬;

//�ܵ���˵�������������ǣ����ٶȼ�������������Pitch��Roll��������������������Yaw;
//���³����У�gx, gy, gz��λΪ����/s��ax, ay, azΪ���ٶȼ������ԭʼ16��������, mx, my, mzΪ�����������ԭʼ16�������ݣ�
//ǰ������mpu9150�ļ��ٶȼƺ������ǵ�x��Ϊǰ������;
//���³�����õĲο�����Ϊ��mpu9150�ļ��ٶȼƺ���������ָ��xyz����Ϊ������

//������Ϊ����500��/s��ǰ���£������ǵ���������65.5LSB/��/s�����԰������������ʮ���������ݳ���65.5���ǽ��ٶȣ���λ�ǡ�/s��
//Ȼ���ٳ���57.3�ͱ�ɻ�����;(1����=180/pi=57.3��)

//ŷ���ǵ�λΪ����radian������57.3�Ժ�ת��Ϊ�Ƕ�,0<yaw<360, -90<pitch<+90, -180<roll<180
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

/*����֮��ĳ���ʹ�ã����ټ���ʱ��*/
        //auxiliary variables to reduce number of repeated operations��
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
          
/*��һ������ֵ�����ٶȼƺʹ����Ƶĵ�λ��ʲô������ν����Ϊ�����ڴ˱����˹�һ������*/        
   //normalise the measurements
   norm = invSqrt(ax*ax + ay*ay + az*az);       
   ax = ax * norm;
   ay = ay * norm;
   az = az * norm;
   norm = invSqrt(mx*mx + my*my + mz*mz);          
   mx = mx * norm;
   my = my * norm;
   mz = mz * norm;         
        
/*�ӻ�������ϵ�ĵ������̲⵽��ʸ��ת�ɵ�������ϵ�µĴų�ʸ��hxyz������ֵ������������Ǵӷ���������ϵ����������ϵ��ת����ʽ*/
   //compute reference direction of flux
   hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
   hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
   hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);//����������ʽ��472��474���ظ�,���Ż�

/*�����������ϵ�µĴų�ʸ��bxyz���ο�ֵ����
��Ϊ����ش�ˮƽ�нǣ�������֪��0�ȣ���ȥ��ƫ�ǵ����أ��̶��򱱣����Ҷ���byָ������������by=ĳֵ��bx=0
������ο��ش�ʸ���ڴ�ֱ����Ҳ�з���bz��������ÿ���ط����ǲ�һ���ġ�
�����޷���֪��Ҳ���޷������ںϣ��и��ʺ�����ֱ���������ںϵļ��ٶȼƣ�������ֱ�ӴӲ���ֵhz�ϸ��ƹ�����bz=hz��
�ų�ˮƽ�������ο�ֵ�Ͳ���ֵ�Ĵ�СӦ����һ�µ�(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))��
��Ϊbx=0�����Ծͼ򻯳�(by*by)  = ((hx*hx) + (hy*hy))�������by�������޸�by��bxָ����Զ����ĸ���ָ������*/
//   bx = sqrtf((hx*hx) + (hy*hy));
   by = sqrtf((hx*hx) + (hy*hy));
   bz = hz;        
    
   // estimated direction of gravity and flux (v and w)����������Ǵ���������ϵ������������ϵ��ת����ʽ(ת�þ���)
	 //��������
   vx = 2*(q1q3 - q0q2);
   vy = 2*(q0q1 + q2q3);
   vz = q0q0 - q1q1 - q2q2 + q3q3;

/*���ǰѵ�������ϵ�ϵĴų�ʸ��bxyz��ת����������wxyz��
��Ϊbx=0�����������漰��bx�Ĳ��ֶ���ʡ���ˡ�ͬ��by=0�����������漰��by�Ĳ���Ҳ���Ա�ʡ�ԣ�������Լ������Ǹ���ָ���йء�
������������vxyz�����㣬��Ϊ����g��az=1��ax=ay=0�����������漰��gxgy�Ĳ���Ҳ��ʡ����
����Կ���������ʽ��wxyz�Ĺ�ʽ����by����ay��0������bz����az��1�����ͱ����vxyz�Ĺ�ʽ�ˣ�����q0q0+q1q1+q2q2+q3q3=1����*/
//   wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
//   wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
//   wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);
   wx = 2*by*(q1q2 + q0q3) + 2*bz*(q1q3 - q0q2);
   wy = 2*by*(0.5 - q1q1 - q3q3) + 2*bz*(q0q1 + q2q3);
   wz = 2*by*(q2q3 - q0q1) + 2*bz*(0.5 - q1q1 - q2q2);
           
//���ڰѼ��ٶȵĲ���ʸ���Ͳο�ʸ����������Ѵų��Ĳ���ʸ���Ͳο�ʸ��Ҳ����������������������ݡ�
   // error is sum of cross product between reference direction of fields and direction measured by sensors
   ex = (ay*vz - az*vy) + (my*wz - mz*wy);
   ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
   ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
   
   halfT=GET_NOWTIME();		//�õ�ÿ����̬���µ����ڵ�һ��
   
   if(ex != 0.0f && ey != 0.0f && ez != 0.0f)      //�ܹؼ���һ�仰��ԭ�㷨û��
   {
      // integral error scaled integral gain
      exInt = exInt + ex*AHRS_Ki * halfT;			   //���Բ������ڵ�һ��
      eyInt = eyInt + ey*AHRS_Ki * halfT;
      ezInt = ezInt + ez*AHRS_Ki * halfT;
      // adjusted gyroscope measurements
      gx = gx + AHRS_Kp*ex + exInt;
      gy = gy + AHRS_Kp*ey + eyInt;
      gz = gz + AHRS_Kp*ez + ezInt;
   }         

   // integrate quaternion rate and normalise����Ԫ�������㷨
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
        
///*����Ԫ�������Pitch  Roll  Yaw
//����57.3��Ϊ�˽�����ת��Ϊ�Ƕ�*/
	Yaw   = -atan2(2*q1*q2 - 2*q0*q3, -2 * q1 * q1 - 2 * q3 * q3 + 1) * 57.3;  //ƫ���ǣ���z��ת��	
    if(Yaw < 0 ){Yaw = Yaw + 360;}
	if(Yaw > 360 ){Yaw = Yaw - 360;}
	Pitch = asin(2*q2*q3 + 2*q0*q1) * 57.3; //�����ǣ���x��ת��	 
  Roll  = -atan2(-2*q0*q2 + 2*q1*q3, -2 * q1 * q1 - 2 * q2* q2 + 1) * 57.3; //�����ǣ���y��ת��

/*���������Ԫ�������Pitch  Roll  Yaw
Roll=arctan2(2wx+2yz, 1-2xx-2yy);
Pitch=arcsin(2wy-2zx);
Yaw=arctan2(2wz+2xy, 1-2yy-2zz);
1=q0*q0+q1*q1+q2*q2+q3*q3;
����57.3��Ϊ�˽�����ת��Ϊ�Ƕ�*/
	
//	Pitch = asin(-2*q1*q3 + 2*q0*q2) * 57.3; //�����ǣ���y��ת��	 
//    Roll  = atan2(2*q2*q3 + 2*q0*q1,-2*q1*q1 - 2*q2*q2 + 1) * 57.3; //�����ǣ���x��ת��
//	Yaw   = atan2(2*q1*q2 + 2*q0*q3,-2*q2*q2 - 2*q3*q3 + 1) * 57.3;  //ƫ���ǣ���z��ת��

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
			init_quaternion();   //�õ���ʼ����Ԫ��
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
		init_gx=(float)(gyro[0] * 0.000266);    //��λת���ɣ�����/s��0.000266=1/(Gyro_500_Scale_Factor * 57.3)
		init_gy=(float)(gyro[1] * 0.000266);
		init_gz=(float)(gyro[2] * 0.000266);
	    
		data_write[6]=0x00;       
		data_write[7]=0x02;                     
    i2c_write(MPU9150_Addr, User_Ctrl, 1, data_write+6);	 //�ر�MPU9150��I2C_MASTERģʽ������Ҫ�����
		delay_us(100);            //������֮����ӳ�����24000
		i2c_write(MPU9150_Addr, Bypass_Enable_Cfg, 1, data_write+7);	 //����bypass�������������룬�ۺ���������ǿ���Bypass though

		 
		mpu_get_compass_reg(mag, &sensor_timestamp);  //��ȡcompass����
		//����x y���У׼��δ��z�����У׼���ο�ST��У׼���� 
		//���У׼����Ӧ��������,û�ж����ݽ��о��⴦��
		init_mx =(float)(mag[1]-32.5f)*1.575f;						
		init_my =(float)(mag[0]-89);
		init_mz =(float)-mag[2];

		data_write[6]=0x20;
		data_write[7]=0x00;
		i2c_write(MPU9150_Addr, User_Ctrl, 1, data_write+6); //����MPU9150��I2C_MASTERģʽ������Ҫ�����
		delay_us(100);		   //������֮����ӳ�����24000
		i2c_write(MPU9150_Addr, Bypass_Enable_Cfg, 1, data_write+7);//�ر�bypass�������������룬�ۺ���������ǹر�Bypass though
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

			run_self_test();		//�Լ�

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



/*//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������*/
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
/*�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
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
//	//��������ʱ��
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
//			//��Ϊ��Խ90��
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
//			//��Ϊ��Խ90��
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
	dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more);	 //��ȷ����ʱ�� 2ms �������ʱ�� 0.35ms
	if(sensors & INV_WXYZ_QUAT )
	{
		q0 = quat[0] / q30;	
		q1 = quat[1] / q30;
		q2 = quat[2] / q30;
		q3 = quat[3] / q30;//��4��ʹ��8us
		*pitch  =		asin(2 * q0* q2 -2 * q1 * q3 );//64us   //�为��
		*roll		=		my_atan2((-2*q1*q1-2*q2*q2+1)*10000,(2*q2*q3+2*q0*q1)*10000)>>8;//���ʹ��5us
		*yaw		=		my_atan2((q0*q0+q1*q1-q2*q2-q3*q3)*10000,(2*(q1*q2+q0*q3))*10000)>>8;//17us	
//		YAW_cul(roll,pitch,yaw);
		return 1;
	}
	return 0;
}

