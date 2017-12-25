#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"

#define KEY0 PBin(8)   //PA13
#define KEY1 PBin(9)	//PA15 
#define KEY2 PBin(0)	//PA0  WK_UP
	 
void KEY_Init(void);//IO初始化
u8 KEY_Scan(void);  //按键扫描函数					    
#endif
