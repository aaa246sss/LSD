#ifndef __PID_H
#define __PID_H

typedef struct increasePID{
	int setpoint;        //�趨ֵ
	int proportion;     //����ϵ��
	int integral;         //����ϵ��
	int derivative;      //΢��ϵ��
	int lasterror;        //ǰһ�����
	int preerror;        //ǰ�������
}_increasePID;
int increasePIDcal(_increasePID *pp,int thisPoint);
void increasePIDInit(_increasePID *PP);
#endif
