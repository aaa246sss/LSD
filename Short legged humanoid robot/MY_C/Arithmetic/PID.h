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

typedef struct positionPID{
//	int setpoint;        //设定值
	int proportion;     //比例系数
//	int integral;         //积分系数
	int derivative;      //微分系数
//	int lasterror;        //前一拍误差
//	int preerror;        //前两拍误差
}_positionPID;

int increasePIDcal(_increasePID *pp,int thisPoint,int setpoint);
void increasePIDInit(_increasePID *PP,int P,int I,int D);
void positionPIDInit(_positionPID *PP,short P_value,short D_value);
int positionPIDcal(_positionPID *pp,short this_Pvalue,short this_Dvalue,short Preset_angel);

#endif
