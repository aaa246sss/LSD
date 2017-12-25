/*
�ʼ�:
STM32C8T6��FLASH_PAGE_SIZE=1024,ZET6Ϊ2048
FLASH_PAGE_COUNTҪ���ڴ����

ħ�����趨���Ѿ���target�ı��ROM�趨��ΧΪ0x5000��20K
��C8T6��64K ROM,���ʣ��flash����Ϊ44K.
����ʹ�ÿ�ͷ��ַΪ0x400������
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

// Ltype=0/����   Ltype>0/����
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
 Func: ����FLASH
 Note: PageIndex/ҳ���  PageCount/ҳ��[=0xFFFFΪƬ����] 
-------------------------------------------------------------------------------*/
uint8 Flash_EreasePage(uint16 PageIndex,uint16 PageCount)
{
	uint8  R;
	if(PageCount==0)return 0xFF;
	Flash_LockControl(0);							//FLASH����
	if((PageIndex==0xFFFF)&&(PageCount==0xFFFF)){				//ȫƬ����
		FLASH->CR|=FLASH_CR_MER;					//������Ƭ����
		FLASH->CR|=FLASH_CR_STRT;					//������������
		R=Flash_WaitBusy();						//�ȴ��������̽���
		if(!(FLASH->SR&FLASH_SR_EOP))R=0xFF;				//�ȴ�����
		FLASH->SR|=FLASH_SR_EOP;
		FLASH->CR&=(~(FLASH_CR_STRT|FLASH_CR_MER));
		Flash_LockControl(1);						//����FLASH
		return R;
	}
	while(PageCount--){
		FLASH->CR|=FLASH_CR_PER;					//ѡ��ҳ����			
		FLASH->AR=(uint32)PageIndex*FLASH_PAGE_SIZE;			//����ҳ��̵�ַ
		FLASH->CR|=FLASH_CR_STRT;					//������������
		R=Flash_WaitBusy();						//�ȴ�
		if(R!=0)break;							//�������̳��ִ���
		if(!(FLASH->SR&FLASH_SR_EOP))break;				//�ȴ�
		FLASH->SR|=FLASH_SR_EOP;
		PageIndex++;
		if(PageIndex>=FLASH_PAGE_COUNT)PageCount=0;	
	}
	FLASH->CR&=(~(FLASH_CR_STRT|FLASH_CR_PER));
	Flash_LockControl(1);							//����FLASH
	return R;
}

/*-------------------------------------------------------------------------------
 Func: дFLASH
 Note: Addr/��ʼ��ַ Buffer/����Դ Length/���� 
-------------------------------------------------------------------------------*/
uint8 Flash_WriteDatas(uint32 Addr,uint16 *Buffer,uint16 Length)
{
	uint8  R=0;
	uint16 *FlashAddr=(uint16 *)Addr;
	Flash_LockControl(0);		//����FLASH
	while(Length--){												
		FLASH->CR|=FLASH_CR_PG;												
		*FlashAddr++=*Buffer++;	//д������
		R=Flash_WaitBusy();	//�ȴ�д�����
		if(R!=0)break;
		if(!(FLASH->SR&FLASH_SR_EOP))break;	//�ȴ�д�����
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
