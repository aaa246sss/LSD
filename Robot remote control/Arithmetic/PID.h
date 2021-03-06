#ifndef __PID_H
#define __PID_H

typedef struct increasePID{
	int setpoint;        //设定值
	int proportion;     //比例系数
	int integral;         //积分系数
	int derivative;      //微分系数
	int lasterror;        //前一拍误差
	int preerror;        //前两拍误差
}_increasePID;
int increasePIDcal(_increasePID *pp,int thisPoint);
void increasePIDInit(_increasePID *PP);
#endif
