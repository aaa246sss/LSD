#include <string.h>
#include "pid.h"



void increasePIDInit(_increasePID *PP,int P,int I,int D)
{
  memset(PP,0,sizeof(_increasePID));
//		PP->proportion=35;//P��Dֻ��һ��һС
//		PP->integral=5;
//		PP->derivative=10;
	
//		PP->proportion=100;//P��Dֻ��һ��һС
//		PP->integral=15;
//		PP->derivative=20;
	
		PP->proportion=P;//P��Dֻ��һ��һС
		PP->integral=I;
		PP->derivative=D;

	PP->setpoint=0;//��ѹֵ
} 

/*
Title:����ʽPID�㷨����
Description:����һ���������
Input:PID��P��I���Ƴ����Ͳ�����ѹֵ
Return:�������temp
*/
int increasePIDcal(_increasePID *pp,int thisPoint,int setpoint)
{
	int pError,dError,iError;
	int temp;
//	iError=pp->setpoint-thisPoint;								 //I����ű������,Ҳ�ǻ��������
	iError=setpoint-thisPoint;								 //I����ű������,Ҳ�ǻ��������
	pError=iError-pp->lasterror; 									 //P���  (pp->setpoint-thisPoint) -  
	dError=iError-2*(pp->lasterror)+pp->preerror;  //D���
	temp=pp->proportion*pError+pp->integral*iError+pp->derivative*dError;   //����ֵ����=p����ϵ��*P���+I����ϵ��*I���+D����ϵ��*D���
	pp->preerror=pp->lasterror;  //�����������´�����
	pp->lasterror=iError;//������ֵ���ϴ����
	return (temp>>8);
}

void positionPIDInit(_positionPID *PP,short P_value,short D_value)
{
	PP->proportion=P_value;//P��Dֻ��һ��һС
	PP->derivative=D_value;
} 

int positionPIDcal(_positionPID *pp,short this_Pvalue,short this_Dvalue,short Preset_angel){
	int temp;
	temp=pp->proportion*(Preset_angel-this_Pvalue)+pp->derivative*this_Dvalue;
	return (temp>>8);
}
