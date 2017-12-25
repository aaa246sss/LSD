#include "myiic.h"
#include "delay.h"
#include "stdint.h"
//extern char IICx;
#define iic_delay_us delay_System_us
//����IIC��ʼ�ź�
void delay_500ns(void)
{		
	char j;
 		for(j=0;j<4;j++);
}

void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	iic_delay_us(2);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	iic_delay_us(2);
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	iic_delay_us(2);
	IIC_SCL=1;
	iic_delay_us(2);	
	IIC_SDA=1;//����I2C���߽����ź� 
	iic_delay_us(2);	
}

//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
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
	IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 

//����ACKӦ��
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
//������ACKӦ��		    
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
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		iic_delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL=1;
		iic_delay_us(2); 
		IIC_SCL=0;	
		iic_delay_us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(void)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
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
//        IIC_NAck();//����nACK
//    else
//        IIC_Ack(); //����ACK   
    return receive;
}

//��ʼ��IIC
void IIC_Init(void)
{					     
//	if(IICx==IIC1)
//	{
			RCC->APB2ENR|=1<<3;//��ʹ������IO PORTBʱ�� 							  
			GPIOB->CRL&=0XFFFFFF00;
			GPIOB->CRL|=0X00000055;
			GPIOB->ODR|=3;     //PB0,1 �����
		 
			IIC_SCL=1;
			IIC_SDA=1;
//	}
//	if(IICx==IIC2)
//	{
//			RCC->APB2ENR|=1<<2;//��ʹ������IO PORTCʱ�� 							 
//			GPIOA->CRL&=0XFFFF00FF;
//			GPIOA->CRL|=0X00005500;
//			GPIOA->ODR|=0x0c;     //PC0,1 �����
//		 
//			IIC2_SCL=1;
//			IIC2_SDA=1;
//	}
}

//addr������slave_address
//reg ����������Ҫд�����ݵ��׵�ַ
//len ��д�����ݵĳ���
//data����Ҫд���һ������	  
u8 IIC_Write_Buffer(u8 addr, u8 reg, u8 len, u8 * data)
{
    int i;
    IIC_Start();
    IIC_Send_Byte(addr << 1 | I2C_Direction_Transmitter);//7λ�����ӵ�ַ+��дλ
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

//����ֵ 0��д�ɹ�
//		-1��дʧ��
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

//addr������slave_address
//reg ����������Ҫ�������ݵ��׵�ַ
//len ���������ݵĳ���
//buf ����Ҫ���������ݴ洢λ��
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

//����ֵ 0�����ɹ�
//		-1����ʧ��
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

//addr������slave_address
//reg ����������Ҫд�����ݵĵ�ַ
//data����Ҫд���һ������
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
//����IIC��ʼ�ź�
void IIC2_Start(void)
{
	SDA2_OUT();     //sda�����
	IIC2_SDA=1;	  	  
	IIC2_SCL=1;
	iic_delay_us(4);
 	IIC2_SDA=0;//START:when CLK is high,DATA change form high to low 
	iic_delay_us(4);
	IIC2_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IIC2ֹͣ�ź�
void IIC2_Stop(void)
{
	SDA2_OUT();//sda�����
	IIC2_SCL=0;
	IIC2_SDA=0;//STOP:when CLK is high DATA change form low to high
 	iic_delay_us(4);
	IIC2_SCL=1; 
	IIC2_SDA=1;//����I2C���߽����ź�
	iic_delay_us(4);							   	
}

//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC2_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA2_IN();      //SDA����Ϊ����  
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
	IIC2_SCL=0;//ʱ�����0 	   
	return 0;  
} 

//����ACKӦ��
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
//������ACKӦ��		    
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
//IIC2����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC2_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA2_OUT(); 	    
    IIC2_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC2_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		iic_delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC2_SCL=1;
		iic_delay_us(2); 
		IIC2_SCL=0;	
		iic_delay_us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC2_Read_Byte(void)
{
	unsigned char i,receive=0;
	SDA2_IN();//SDA����Ϊ����
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
//        IIC2_NAck();//����nACK
//    else
//        IIC2_Ack(); //����ACK   
    return receive;
}


//addr������slave_address
//reg ����������Ҫд�����ݵ��׵�ַ
//len ��д�����ݵĳ���
//data����Ҫд���һ������	  
u8 IIC2_Write_Buffer(u8 addr, u8 reg, u8 len, u8 * data)
{
    int i;
    IIC2_Start();
    IIC2_Send_Byte(addr << 1 | I2C_Direction_Transmitter);//7λ�����ӵ�ַ+��дλ
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

//����ֵ 0��д�ɹ�
//		-1��дʧ��
int IIC2_Write(u8 addr, u8 reg, u8 len, u8* data)
{
	if(IIC2_Write_Buffer(addr,reg,len,data))
		return TRUE;
	else
		return FALSE;
}

//addr������slave_address
//reg ����������Ҫ�������ݵ��׵�ַ
//len ���������ݵĳ���
//buf ����Ҫ���������ݴ洢λ��
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

//����ֵ 0�����ɹ�
//		-1����ʧ��
int IIC2_Read(u8 addr, u8 reg, u8 len, u8 *buf)
{
	if(IIC2_Read_Buffer(addr,reg,len,buf))
		return TRUE;
	else
		return FALSE;
}

//addr������slave_address
//reg ����������Ҫд�����ݵĵ�ַ
//data����Ҫд���һ������
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
		RCC->APB2ENR|=1<<2;//��ʹ������IO PORTAʱ�� 							 

		GPIOA->CRL&=0XFFFF00FF;
		GPIOA->CRL|=0X00005500;
		GPIOA->ODR|=0x0c;     //PC0,1 �����
	 
		IIC2_SCL=1;
		IIC2_SDA=1;
}
*/

