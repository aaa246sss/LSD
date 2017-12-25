#define MULTIPLE 10000	//�˲�ǿ��




//�������������ݲ�Ҫ��
#define Cov_sys 1 
#define Cov_meas MULTIPLE
//============================================================================//
//==                          �������˲�                                    ==//
//============================================================================//
//==��ڲ���: ��ǰʱ�̵Ĳ���ֵ                                                            ==//
//==���ڲ���: ��ǰʱ�̵�����ֵ                                                            ==//
//==����ֵ:   ��ǰʱ�̵�����ֵ                                                            ==//
//============================================================================//
float KalMan_Update_Float_MY(float IN_TEMP)
{
	static float Optimal=0;		//�������ֵ
	static float Cov_Opt=10000;	//����Э���� //���չ̶�Ϊһ��ֵ
	float Kalman_gain;			//���������� //���չ̶�Ϊһ��ֵ
	float Cov_Opt_add_Sys;		//����Э������ϵͳЭ����֮��
//	Pre_val_current[0] = State_transition_matrix[0] * tOpt.XPreOpt[0]; //5����ʽ���������ʡ��
	Cov_Opt_add_Sys = Cov_Opt + Cov_sys;				//һ����˵ϵͳ��Э����Ƚ�С
	Kalman_gain=Cov_Opt_add_Sys/(Cov_Opt_add_Sys + Cov_meas);//Cov_meas[0]ԽС,����Խ��
	Optimal=Optimal+(IN_TEMP-Optimal)*Kalman_gain;	//����Խ��,����ֵԽ�������ֵ
	Cov_Opt=(1-Kalman_gain)*Cov_Opt_add_Sys;			//����Խ��,Э��������Խ��
	return Optimal;
}
//============================================================================//
//==                          �������˲�                                    ==//
//============================================================================//
//==��ڲ���: ��ǰʱ�̵Ĳ���ֵ                                                            ==//
//==���ڲ���: ��ǰʱ�̵�����ֵ                                                            ==//
//==����ֵ:   ��ǰʱ�̵�����ֵ                                                            ==//
//============================================================================//
//����ʹ�ö�������,Ϊ�������� �������ݷŴ�10000��
int KalMan_Update_Int_MY(int temp)
{

	static int Optimal=0;		//�������ֵ
	static int Cov_Opt=10000;	//����Э���� //���չ̶�Ϊһ��ֵ
	int Kalman_gain;			//���������� //���չ̶�Ϊһ��ֵ
	int Cov_Opt_add_Sys;		//����Э������ϵͳЭ����֮��
//	Pre_val_current[0] = State_transition_matrix[0] * tOpt.XPreOpt[0]; //5����ʽ���������ʡ��
	Cov_Opt_add_Sys = Cov_Opt + Cov_sys;				//һ����˵ϵͳ��Э����Ƚ�С
	Kalman_gain=Cov_Opt_add_Sys*10000/(Cov_Opt_add_Sys + Cov_meas);//Cov_meas[0]ԽС,����Խ��
	Optimal=Optimal+(temp-Optimal)*Kalman_gain/10000;	//����Խ��,����ֵԽ�������ֵ
	Cov_Opt=(10000-Kalman_gain)*Cov_Opt_add_Sys/10000;//����Խ��,Э��������Խ��
	return Optimal;
}
