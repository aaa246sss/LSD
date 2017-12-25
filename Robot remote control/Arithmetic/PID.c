#include <string.h>
#include "pid.h"



void increasePIDInit(_increasePID *PP)
{
  memset(PP,0,sizeof(_increasePID));
	
	PP->proportion=200;
	PP->integral=10;
	PP->derivative=100;
	PP->setpoint=2000;//电压值
} 

/*
Title:增量式PID算法程序
Description:给出一个误差增量
Input:PID的P、I控制常数和采样电压值
Return:误差增量temp
*/
int increasePIDcal(_increasePID *pp,int thisPoint)
{
	int pError,dError,iError;
	int temp;
	iError=pp->setpoint-thisPoint;//存放本次误差,也是积分误差项
	pError=iError-pp->lasterror;    
	dError=iError-2*(pp->lasterror)+pp->preerror;
	temp=pp->proportion*pError+pp->integral*iError+pp->derivative*dError;   //增量计算
	pp->preerror=pp->lasterror;  //存放误差用于下次运算
	pp->lasterror=iError;//本次误差赋值给上次误差
	return (temp>>8);
}
