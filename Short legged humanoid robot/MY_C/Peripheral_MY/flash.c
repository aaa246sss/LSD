/*
笔记:
STM32C8T6的FLASH_PAGE_SIZE=1024,ZET6为2048
FLASH_PAGE_COUNT要视内存而定

魔术棒设定中已经把target的编程ROM设定范围为0x5000即20K
而C8T6有64K ROM,因此剩下flash可用为44K.
建议使用开头地址为0x400的整数
*/
#include "sys.h"
#include "flash.h"
//#include "motor_control.h"
#include <stdio.h>
//#include "delay.h"

//extern _motor motor,motor_last;
//char flash_buf[1025];
//u16  flash_count;

void FLASH_save(void)
{
	//motor.PWM
//	char i;
//	for(i=0;i<8;i++)
//	{
//		flash_buf[i]<<
//	}
//	sprintf(flash_buf,"%d",(u16)motor.PWM);
//	sprintf(flash_buf,"asdfgh");
	//flash_buf
}

u8 Flash_WaitBusy(void)
{
	u16 T=1000;
	do{
		if(!(FLASH->SR&FLASH_SR_BSY))return 0;
	}while(--T);
	return 0xFF;
}

// Ltype=0/解锁   Ltype>0/加锁
void Flash_LockControl(uint8 Ltype)
{
	if(Ltype==0){
	    if(FLASH->CR&FLASH_CR_LOCK){
			FLASH->KEYR=0x45670123;
			FLASH->KEYR=0xCDEF89AB;
		}
	}else  FLASH->CR|=FLASH_CR_LOCK;
}
/*-------------------------------------------------------------------------------
 Func: 擦除FLASH
 Note: PageIndex/页编号  PageCount/页数[=0xFFFF为片擦除] 
-------------------------------------------------------------------------------*/
uint8 Flash_EreasePage(uint16 PageIndex,uint16 PageCount)
{
	uint8  R;
	if(PageCount==0)return 0xFF;
	Flash_LockControl(0);							//FLASH解锁
	if((PageIndex==0xFFFF)&&(PageCount==0xFFFF)){				//全片擦除
		FLASH->CR|=FLASH_CR_MER;					//设置整片擦除
		FLASH->CR|=FLASH_CR_STRT;					//启动擦除过程
		R=Flash_WaitBusy();						//等待擦除过程结束
		if(!(FLASH->SR&FLASH_SR_EOP))R=0xFF;				//等待结束
		FLASH->SR|=FLASH_SR_EOP;
		FLASH->CR&=(~(FLASH_CR_STRT|FLASH_CR_MER));
		Flash_LockControl(1);						//锁定FLASH
		return R;
	}
	while(PageCount--){
		FLASH->CR|=FLASH_CR_PER;					//选择页擦除			
		FLASH->AR=(uint32)PageIndex*FLASH_PAGE_SIZE;			//设置页编程地址
		FLASH->CR|=FLASH_CR_STRT;					//启动擦除过程
		R=Flash_WaitBusy();						//等待
		if(R!=0)break;							//擦除过程出现错误
		if(!(FLASH->SR&FLASH_SR_EOP))break;				//等待
		FLASH->SR|=FLASH_SR_EOP;
		PageIndex++;
		if(PageIndex>=FLASH_PAGE_COUNT)PageCount=0;	
	}
	FLASH->CR&=(~(FLASH_CR_STRT|FLASH_CR_PER));
	Flash_LockControl(1);							//锁定FLASH
	return R;
}

/*-------------------------------------------------------------------------------
 Func: 写FLASH
 Note: Addr/起始地址 Buffer/数据源 Length/长度 
-------------------------------------------------------------------------------*/
uint8 Flash_WriteDatas(uint32 Addr,uint16 *Buffer,uint16 Length)
{
	uint8  R=0;
	uint16 *FlashAddr=(uint16 *)Addr;
	Flash_LockControl(0);		//解锁FLASH
	while(Length--){												
		FLASH->CR|=FLASH_CR_PG;												
		*FlashAddr++=*Buffer++;	//写入数据
		R=Flash_WaitBusy();	//等待写入结束
		if(R!=0)break;
		if(!(FLASH->SR&FLASH_SR_EOP))break;	//等待写入结束
		FLASH->SR|=FLASH_SR_EOP;
	}
	Flash_LockControl(1);
	return R;
}

void FLASH_ReadDatas(uint32 Addr,uint16 *Buffer,uint16 Length)
{		
	uint16 *FlashAddr=(uint16 *)Addr;
	while(Length--)*Buffer++=*FlashAddr++;
}
