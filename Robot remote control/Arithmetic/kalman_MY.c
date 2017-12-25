#define MULTIPLE 10000	//滤波强度




//下面这两个数据不要动
#define Cov_sys 1 
#define Cov_meas MULTIPLE
//============================================================================//
//==                          卡尔曼滤波                                    ==//
//============================================================================//
//==入口参数: 当前时刻的测量值                                                            ==//
//==出口参数: 当前时刻的最优值                                                            ==//
//==返回值:   当前时刻的最优值                                                            ==//
//============================================================================//
float KalMan_Update_Float_MY(float IN_TEMP)
{
	static float Optimal=0;		//最优输出值
	static float Cov_Opt=10000;	//最优协方差 //最终固定为一个值
	float Kalman_gain;			//卡尔曼增益 //最终固定为一个值
	float Cov_Opt_add_Sys;		//最优协方差与系统协方差之和
//	Pre_val_current[0] = State_transition_matrix[0] * tOpt.XPreOpt[0]; //5个公式中这个可以省略
	Cov_Opt_add_Sys = Cov_Opt + Cov_sys;				//一般来说系统的协方差比较小
	Kalman_gain=Cov_Opt_add_Sys/(Cov_Opt_add_Sys + Cov_meas);//Cov_meas[0]越小,增益越大
	Optimal=Optimal+(IN_TEMP-Optimal)*Kalman_gain;	//增益越大,最优值越倾向测量值
	Cov_Opt=(1-Kalman_gain)*Cov_Opt_add_Sys;			//增益越大,协方差缩得越快
	return Optimal;
}
//============================================================================//
//==                          卡尔曼滤波                                    ==//
//============================================================================//
//==入口参数: 当前时刻的测量值                                                            ==//
//==出口参数: 当前时刻的最优值                                                            ==//
//==返回值:   当前时刻的最优值                                                            ==//
//============================================================================//
//浮点使用定点运算,为整数运算 所有数据放大10000倍
int KalMan_Update_Int_MY(int temp)
{

	static int Optimal=0;		//最优输出值
	static int Cov_Opt=10000;	//最优协方差 //最终固定为一个值
	int Kalman_gain;			//卡尔曼增益 //最终固定为一个值
	int Cov_Opt_add_Sys;		//最优协方差与系统协方差之和
//	Pre_val_current[0] = State_transition_matrix[0] * tOpt.XPreOpt[0]; //5个公式中这个可以省略
	Cov_Opt_add_Sys = Cov_Opt + Cov_sys;				//一般来说系统的协方差比较小
	Kalman_gain=Cov_Opt_add_Sys*10000/(Cov_Opt_add_Sys + Cov_meas);//Cov_meas[0]越小,增益越大
	Optimal=Optimal+(temp-Optimal)*Kalman_gain/10000;	//增益越大,最优值越倾向测量值
	Cov_Opt=(10000-Kalman_gain)*Cov_Opt_add_Sys/10000;//增益越大,协方差缩得越快
	return Optimal;
}
