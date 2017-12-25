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

typedef struct positionPID{
//	int setpoint;        //�趨ֵ
	int proportion;     //����ϵ��
//	int integral;         //����ϵ��
	int derivative;      //΢��ϵ��
//	int lasterror;        //ǰһ�����
//	int preerror;        //ǰ�������
}_positionPID;

int increasePIDcal(_increasePID *pp,int thisPoint,int setpoint);
void increasePIDInit(_increasePID *PP,int P,int I,int D);
void positionPIDInit(_positionPID *PP,short P_value,short D_value);
int positionPIDcal(_positionPID *pp,short this_Pvalue,short this_Dvalue,short Preset_angel);

#endif
