#ifndef __MYIIC_H__
#define __MYIIC_H__
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//IIC 驱动函数	   
//修改日期:2014/3/10 
//版本：V1.0
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  

   	   		   
//IO方向设置
//#define SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
//#define SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}
#define SDA_IN()  {GPIOB->CRL&=0XFFFFFF0F;GPIOB->CRL|=8<<4;}
#define SDA_OUT() {GPIOB->CRL&=0XFFFFFF0F;GPIOB->CRL|=3<<4;}

//IO操作函数	 
#define IIC_SCL    PBout(0) //SCL
#define IIC_SDA    PBout(1) //SDA	 
#define READ_SDA   PBin(1)  //输入SDA 

//I2C_2 IO操作函数	 
#define IIC2_SCL    PAout(2) //SCL
#define IIC2_SDA    PAout(3) //SDA	 
#define READ2_SDA   PAin(3)  //输入SDA 
#define SDA2_IN()   {GPIOA->CRL&=0XFFFF0FFF;GPIOA->CRL|=8<<12;}
#define SDA2_OUT()  {GPIOA->CRL&=0XFFFF0FFF;GPIOA->CRL|=3<<12;}

#define true 1
#define false 0
	
#define IIC1  1
#define IIC2  2

#define TRUE  0
#define FALSE -1

#define  I2C_Direction_Transmitter      ((uint8_t)0x00)	//写
#define  I2C_Direction_Receiver         ((uint8_t)0x01)	//读

//IIC所有操作函数
void IIC_Init(void);                					//初始化IIC的IO口				 
u8 IIC_Write_Buffer(u8 addr, u8 reg, u8 len, u8 * data);
int IIC_Write(u8 addr, u8 reg, u8 len, u8* data);
u8 IIC_Read_Buffer(u8 addr, u8 reg, u8 len, u8* buf);
int IIC_Read(u8 addr, u8 reg, u8 len, u8 *buf);
u8 IIC_WriteOneByte(u8 addr, u8 reg, u8 data);
u16 IIC_GetErrorCounter(void);

/*IIC2*/
void IIC2_Init(void);                					//初始化IIC的IO口				 
u8 IIC2_Write_Buffer(u8 addr, u8 reg, u8 len, u8 * data);
int IIC2_Write(u8 addr, u8 reg, u8 len, u8* data);
u8 IIC2_Read_Buffer(u8 addr, u8 reg, u8 len, u8* buf);
int IIC2_Read(u8 addr, u8 reg, u8 len, u8 *buf);
u8 IIC2_WriteOneByte(u8 addr, u8 reg, u8 data);
u16 IIC2_GetErrorCounter(void);
  
#endif
