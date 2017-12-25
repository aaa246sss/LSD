#include "outputdata.h"
#include "usart.h"	
#include "timer.h"
void UsartSend(unsigned char DataToSend);
float OutData[4] = { 0 };

u16 CRC_CHECK(u8 *Buf, u8 CRC_CNT)
{
    unsigned short CRC_Temp;
    unsigned char i,j;
    CRC_Temp = 0xffff;

    for (i=0;i<CRC_CNT; i++){      
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}

void OutPut_Data(void)
{
  int temp[4] = {0};
  unsigned int temp1[4] = {0};
  unsigned char databuf[10] = {0};
  unsigned char i;
  unsigned short CRC16 = 0;
  for(i=0;i<4;i++)
   {
    
    temp[i]  = (int)OutData[i];
    temp1[i] = (unsigned int)temp[i];
    
   }
   
  for(i=0;i<4;i++) 
  {
    databuf[i*2]   = (unsigned char)(temp1[i]%256);
    databuf[i*2+1] = (unsigned char)(temp1[i]/256);
  }
  
  CRC16 = CRC_CHECK(databuf,8);
  databuf[8] = CRC16%256;
  databuf[9] = CRC16/256;
	
//	USART1_Send_Buf((char *)databuf,10);
  
  for(i=0;i<10;i++)
    UsartSend(databuf[i]);
}

void send_OULA_to_miniIMU(float *roll,float *pitch,short *yaw)
{
		static char  uart_send_buf[11]={0x55,0x53,2,3,4,5,6,7,0x10,0x00,10};
		char i;
		short cul;
		cul=*roll*10430;
		uart_send_buf[2]=cul;
		uart_send_buf[3]=cul>>8;
		cul=*pitch*10430;
		uart_send_buf[4]=cul;
		uart_send_buf[5]=cul>>8;
		cul=*yaw*182;
		uart_send_buf[6]=cul;
		uart_send_buf[7]=cul>>8;
		uart_send_buf[10]=0;
		for(i=0;i<10;i++)
		{
			uart_send_buf[10]+=uart_send_buf[i];
		}
		USART1_Send_Buf(uart_send_buf,11);
}




