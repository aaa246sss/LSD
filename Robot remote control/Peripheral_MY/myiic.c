#include "myiic.h"
#include "delay.h"
#include "stdint.h"
//extern char IICx;
#define iic_delay_us delay_System_us
//产生IIC起始信号
void delay_500ns(void)
{		
	char j;
 		for(j=0;j<4;j++);
}

void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	iic_delay_us(2);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	iic_delay_us(2);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	iic_delay_us(2);
	IIC_SCL=1;
	iic_delay_us(2);	
	IIC_SDA=1;//发送I2C总线结束信号 
	iic_delay_us(2);	
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA=1;iic_delay_us(1);	   
	IIC_SCL=1;iic_delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 

//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	iic_delay_us(2);
	IIC_SCL=1;
	iic_delay_us(2);
	IIC_SCL=0;
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	iic_delay_us(2);
	IIC_SCL=1;
	iic_delay_us(2);
	IIC_SCL=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		iic_delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		iic_delay_us(2); 
		IIC_SCL=0;	
		iic_delay_us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(void)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        iic_delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
//		iic_delay_us(1); 
		delay_500ns();
    }					 
//    if (!ack)
//        IIC_NAck();//发送nACK
//    else
//        IIC_Ack(); //发送ACK   
    return receive;
}

//初始化IIC
void IIC_Init(void)
{					     
//	if(IICx==IIC1)
//	{
			RCC->APB2ENR|=1<<3;//先使能外设IO PORTB时钟 							  
			GPIOB->CRL&=0XFFFFFF00;
			GPIOB->CRL|=0X00000055;
			GPIOB->ODR|=3;     //PB0,1 输出高
		 
			IIC_SCL=1;
			IIC_SDA=1;
//	}
//	if(IICx==IIC2)
//	{
//			RCC->APB2ENR|=1<<2;//先使能外设IO PORTC时钟 							 
//			GPIOA->CRL&=0XFFFF00FF;
//			GPIOA->CRL|=0X00005500;
//			GPIOA->ODR|=0x0c;     //PC0,1 输出高
//		 
//			IIC2_SCL=1;
//			IIC2_SDA=1;
//	}
}

//addr：器件slave_address
//reg ：从器件将要写入数据的首地址
//len ：写入数据的长度
//data：将要写入的一串数据	  
u8 IIC_Write_Buffer(u8 addr, u8 reg, u8 len, u8 * data)
{
    int i;
    IIC_Start();
    IIC_Send_Byte(addr << 1 | I2C_Direction_Transmitter);//7位器件从地址+读写位
    if (IIC_Wait_Ack()) 
	{
        IIC_Stop();
        return false;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    for (i = 0; i < len; i++) 
	{
        IIC_Send_Byte(*data);
        if (IIC_Wait_Ack()) 
		{
            IIC_Stop();
            return false;
        }
		data++;
    }
    IIC_Stop();
    return true;
}

//返回值 0：写成功
//		-1：写失败
int IIC_Write(u8 addr, u8 reg, u8 len, u8* data)
{
//	if(IICx==IIC1)
//	{
			if(IIC_Write_Buffer(addr,reg,len,data))
				return TRUE;	
				
//	}
//		if(IICx==IIC2)
//	{
//			if(IIC2_Write_Buffer(addr,reg,len,data))
//				return TRUE;	
//				
//	}
	return FALSE;
}

//addr：器件slave_address
//reg ：从器件将要读的数据的首地址
//len ：读出数据的长度
//buf ：将要读出的数据存储位置
u8 IIC_Read_Buffer(u8 addr, u8 reg, u8 len, u8* buf)
{
    IIC_Start();
    IIC_Send_Byte(addr << 1 | I2C_Direction_Transmitter);
    if (IIC_Wait_Ack())
	{
        IIC_Stop();
        return false;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();

    IIC_Start();
    IIC_Send_Byte(addr << 1 | I2C_Direction_Receiver);
    IIC_Wait_Ack();
    while (len)
		{
        *buf = IIC_Read_Byte();
        if (len == 1)
            IIC_NAck();
        else
            IIC_Ack();
        buf++;
        len--;
//				delay_System_us(3);
    }
    IIC_Stop();
    return true;
}

//返回值 0：读成功
//		-1：读失败
int IIC_Read(u8 addr, u8 reg, u8 len, u8 *buf)
{
//	if(IICx==IIC1)
//	{
		if(IIC_Read_Buffer(addr,reg,len,buf))
			return TRUE;
//	}
//	if(IICx==IIC2)
//	{
//		if(IIC2_Read_Buffer(addr,reg,len,buf))
//			return TRUE;
//	}
	return FALSE;
}

//addr：器件slave_address
//reg ：从器件将要写入数据的地址
//data：将要写入的一个数据
u8 IIC_WriteOneByte(u8 addr, u8 reg, u8 data)
{
    IIC_Start();
    IIC_Send_Byte(addr << 1 | I2C_Direction_Transmitter);
    if (IIC_Wait_Ack()) 
	{
        IIC_Stop();
        return false;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Send_Byte(data);
    IIC_Wait_Ack();
    IIC_Stop();
    return true;
}

u16 IIC_GetErrorCounter(void)
{
    return 0;
}













/***************IIC2**************/
/*
//产生IIC起始信号
void IIC2_Start(void)
{
	SDA2_OUT();     //sda线输出
	IIC2_SDA=1;	  	  
	IIC2_SCL=1;
	iic_delay_us(4);
 	IIC2_SDA=0;//START:when CLK is high,DATA change form high to low 
	iic_delay_us(4);
	IIC2_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC2停止信号
void IIC2_Stop(void)
{
	SDA2_OUT();//sda线输出
	IIC2_SCL=0;
	IIC2_SDA=0;//STOP:when CLK is high DATA change form low to high
 	iic_delay_us(4);
	IIC2_SCL=1; 
	IIC2_SDA=1;//发送I2C总线结束信号
	iic_delay_us(4);							   	
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC2_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA2_IN();      //SDA设置为输入  
	IIC2_SDA=1;iic_delay_us(1);	   
	IIC2_SCL=1;iic_delay_us(1);	 
	while(READ2_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC2_Stop();
			return 1;
		}
	}
	IIC2_SCL=0;//时钟输出0 	   
	return 0;  
} 

//产生ACK应答
void IIC2_Ack(void)
{
	IIC2_SCL=0;
	SDA2_OUT();
	IIC2_SDA=0;
	iic_delay_us(2);
	IIC2_SCL=1;
	iic_delay_us(2);
	IIC2_SCL=0;
}
//不产生ACK应答		    
void IIC2_NAck(void)
{
	IIC2_SCL=0;
	SDA2_OUT();
	IIC2_SDA=1;
	iic_delay_us(2);
	IIC2_SCL=1;
	iic_delay_us(2);
	IIC2_SCL=0;
}					 				     
//IIC2发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC2_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA2_OUT(); 	    
    IIC2_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC2_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		iic_delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC2_SCL=1;
		iic_delay_us(2); 
		IIC2_SCL=0;	
		iic_delay_us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC2_Read_Byte(void)
{
	unsigned char i,receive=0;
	SDA2_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC2_SCL=0; 
        iic_delay_us(2);
				IIC2_SCL=1;
        receive<<=1;
        if(READ2_SDA)receive++;   
		iic_delay_us(1); 
    }					 
//    if (!ack)
//        IIC2_NAck();//发送nACK
//    else
//        IIC2_Ack(); //发送ACK   
    return receive;
}


//addr：器件slave_address
//reg ：从器件将要写入数据的首地址
//len ：写入数据的长度
//data：将要写入的一串数据	  
u8 IIC2_Write_Buffer(u8 addr, u8 reg, u8 len, u8 * data)
{
    int i;
    IIC2_Start();
    IIC2_Send_Byte(addr << 1 | I2C_Direction_Transmitter);//7位器件从地址+读写位
    if (IIC2_Wait_Ack()) 
	{
        IIC2_Stop();
        return false;
    }
    IIC2_Send_Byte(reg);
    IIC2_Wait_Ack();
    for (i = 0; i < len; i++) 
	{
        IIC2_Send_Byte(*data);
        if (IIC2_Wait_Ack()) 
		{
            IIC2_Stop();
            return false;
    }
		data++;
    }
    IIC2_Stop();
    return true;
}

//返回值 0：写成功
//		-1：写失败
int IIC2_Write(u8 addr, u8 reg, u8 len, u8* data)
{
	if(IIC2_Write_Buffer(addr,reg,len,data))
		return TRUE;
	else
		return FALSE;
}

//addr：器件slave_address
//reg ：从器件将要读的数据的首地址
//len ：读出数据的长度
//buf ：将要读出的数据存储位置
u8 IIC2_Read_Buffer(u8 addr, u8 reg, u8 len, u8* buf)
{
    IIC2_Start();
    IIC2_Send_Byte(addr << 1 | I2C_Direction_Transmitter);
    if (IIC2_Wait_Ack())
	{
        IIC2_Stop();
        return false;
    }
    IIC2_Send_Byte(reg);
    IIC2_Wait_Ack();

    IIC2_Start();
    IIC2_Send_Byte(addr << 1 | I2C_Direction_Receiver);
    IIC2_Wait_Ack();
    while (len)
	{
        *buf = IIC2_Read_Byte();
        if (len == 1)
            IIC2_NAck();
        else
            IIC2_Ack();
        buf++;
        len--;
    }
    IIC2_Stop();
    return true;
}

//返回值 0：读成功
//		-1：读失败
int IIC2_Read(u8 addr, u8 reg, u8 len, u8 *buf)
{
	if(IIC2_Read_Buffer(addr,reg,len,buf))
		return TRUE;
	else
		return FALSE;
}

//addr：器件slave_address
//reg ：从器件将要写入数据的地址
//data：将要写入的一个数据
u8 IIC2_WriteOneByte(u8 addr, u8 reg, u8 data)
{
    IIC2_Start();
    IIC2_Send_Byte(addr << 1 | I2C_Direction_Transmitter);
    if (IIC2_Wait_Ack()) 
	{
        IIC2_Stop();
        return false;
    }
    IIC2_Send_Byte(reg);
    IIC2_Wait_Ack();
    IIC2_Send_Byte(data);
    IIC2_Wait_Ack();
    IIC2_Stop();
    return true;
}

u16 IIC2_GetErrorCounter(void)
{
    return 0;
}
void IIC2_Init()
{
		RCC->APB2ENR|=1<<2;//先使能外设IO PORTA时钟 							 

		GPIOA->CRL&=0XFFFF00FF;
		GPIOA->CRL|=0X00005500;
		GPIOA->ODR|=0x0c;     //PC0,1 输出高
	 
		IIC2_SCL=1;
		IIC2_SDA=1;
}
*/

