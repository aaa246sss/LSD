#include <string.h>
#include "pid.h"



void increasePIDInit(_increasePID *PP,int P,int I,int D)
{
  memset(PP,0,sizeof(_increasePID));
//		PP->proportion=35;//P和D只能一大一小
//		PP->integral=5;
//		PP->derivative=10;
	
//		PP->proportion=100;//P和D只能一大一小
//		PP->integral=15;
//		PP->derivative=20;
	
		PP->proportion=P;//P和D只能一大一小
		PP->integral=I;
		PP->derivative=D;

	PP->setpoint=0;//电压值
} 

/*
Title:增量式PID算法程序
Description:给出一个误差增量
Input:PID的P、I控制常数和采样电压值
Return:误差增量temp
*/
int increasePIDcal(_increasePID *pp,int thisPoint,int setpoint)
{
	int pError,dError,iError;
	int temp;
//	iError=pp->setpoint-thisPoint;								 //I误差存放本次误差,也是积分误差项
	iError=setpoint-thisPoint;								 //I误差存放本次误差,也是积分误差项
	pError=iError-pp->lasterror; 									 //P误差  (pp->setpoint-thisPoint) -  
	dError=iError-2*(pp->lasterror)+pp->preerror;  //D误差
	temp=pp->proportion*pError+pp->integral*iError+pp->derivative*dError;   //增量值计算=p比例系数*P误差+I比例系数*I误差+D比例系数*D误差
	pp->preerror=pp->lasterror;  //存放误差用于下次运算
	pp->lasterror=iError;//本次误差赋值给上次误差
	return (temp>>8);
}

void positionPIDInit(_positionPID *PP,short P_value,short D_value)
{
	PP->proportion=P_value;//P和D只能一大一小
	PP->derivative=D_value;
} 

int positionPIDcal(_positionPID *pp,short this_Pvalue,short this_Dvalue,short Preset_angel){
	int temp;
	temp=pp->proportion*(Preset_angel-this_Pvalue)+pp->derivative*this_Dvalue;
	return (temp>>8);
}
