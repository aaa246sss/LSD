#include <string.h>
#include "pid.h"



void increasePIDInit(_increasePID *PP)
{
  memset(PP,0,sizeof(_increasePID));
	
	PP->proportion=200;
	PP->integral=10;
	PP->derivative=100;
	PP->setpoint=2000;//��ѹֵ
} 

/*
Title:����ʽPID�㷨����
Description:����һ���������
Input:PID��P��I���Ƴ����Ͳ�����ѹֵ
Return:�������temp
*/
int increasePIDcal(_increasePID *pp,int thisPoint)
{
	int pError,dError,iError;
	int temp;
	iError=pp->setpoint-thisPoint;//��ű������,Ҳ�ǻ��������
	pError=iError-pp->lasterror;    
	dError=iError-2*(pp->lasterror)+pp->preerror;
	temp=pp->proportion*pError+pp->integral*iError+pp->derivative*dError;   //��������
	pp->preerror=pp->lasterror;  //�����������´�����
	pp->lasterror=iError;//������ֵ���ϴ����
	return (temp>>8);
}
