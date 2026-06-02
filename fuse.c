#include "fuse.h"

PID SpeedPID = {0};
PID L_SpeedPID ={0};
PID R_SpeedPID ={0};
PID TurnPID ={0};
PID Turn_NeiPID ={0};
TurnPoPID TurnPosPID={0};
/****************************PID参数初始化**************************************
函数：  void PID_int(void)
参数：  void
说明：  PID每个环参数初始化
********************************************************************************/
void PID_int(void)
{
	IncPIDInit(&L_SpeedPID);
	IncPIDInit(&R_SpeedPID);
		
	SpeedPID.Kp=30;     //25      速度环PID参数 15 0.5 1
	SpeedPID.Ki=0.02;     //1
	SpeedPID.Kd=0;       //0.5
	
//	L_SpeedPID.Kp=7;   //youyouyou轮速度环PID参数
//	L_SpeedPID.Ki=1;
//	L_SpeedPID.Kd=1;
//	
//	R_SpeedPID.Kp=7;   //zuozuozuo速度环PID参数
//	R_SpeedPID.Ki=1;
//	R_SpeedPID.Kd=1;
	
	TurnPID.Kp=29;       //转向环PID参数  100 0 0
	TurnPID.Ki=0;
	TurnPID.Kd=0;
	
//    TurnPosPID.Kp_a=34;      //比例系数
//	TurnPosPID.Kp_b=0;      //比例系数
//    TurnPosPID.Kd=0;      //微分系数
	
//	Turn_NeiPID.Kp=4;   //3转向内环PID参数
//	Turn_NeiPID.Ki=0;
//	Turn_NeiPID.Kd=2.6;
}

/****************************角速度内环和速度环**************************************
函数：  void Motor_output_control()
参数：  void
说明：  角速度内环和速度环
***************************************************************************************/
void Motor_output_control()
{
    imu660ra_get_gyro();   //获取陀螺仪角速度值
	speed_measure();       //编码器测量

	aim_speed = Key_aim_speed;       //目标速度
	Speed_pwm_all = LocP_DCalc(&SpeedPID, aim_speed , real_speed); //速度环（位置式）
//	Speed_pwm_left  += IncPIDCalc(&L_SpeedPID,aim_speed,left_speed); 
//	Speed_pwm_right += IncPIDCalc(&R_SpeedPID,aim_speed,right_speed); 
	
// 	Steer_pwm=LocP_DCalc(&Turn_NeiPID,ADC_PWM,-imu660ra_gyro_z);   //转向内环PWM	
    Steer_pwm=LocP_DCalc(&TurnPID,0,Current_Dir);
//	//Steer_pwm=Current_Dir*7;
//	//Steer_pwm=PosionPID_Control(&TurnPID,0,Current_Dir);
	
	Steer_pwm=range_protect(Steer_pwm,-3000,3000);                //转向内环PWM限幅
//	All_PWM_left=Speed_pwm_all+Steer_pwm;                         //左电机所有PWM输出 
//	All_PWM_right=Speed_pwm_all-Steer_pwm;                        //右电机所有PWM输出
	All_PWM_left=Speed_pwm_all+Steer_pwm;  
	All_PWM_left=range_protect(All_PWM_left,-3000,3000); 
	//左电机所有PWM输出 
	All_PWM_right=Speed_pwm_all-Steer_pwm;
	All_PWM_right=range_protect(All_PWM_right,-3000,3000);
	//右电机所有PWM输出
	
    go_motor(All_PWM_left,All_PWM_right);                         //动力输出

//	if(road_type.in_annulus_right ==1&&road_type.on_annulus_right ==0)
//	{
//		go_motor(real_speed,real_speed/2);
//	}
//	else if(road_type.in_annulus_left ==1&&road_type.on_annulus_left ==0)
//	{
//		go_motor(real_speed/2,real_speed);
//	}
//	else
//	{
//		go_motor(All_PWM_left,All_PWM_right);
//	}
	
}
/****************************转向环（转向外环）**************************************
函数：  void Trailing_control()
参数：  void
说明：  转向环
***************************************************************************************/
void Trailing_control()
{
	  Get_deviation();  //电磁采集并获取赛道偏差
	  Annulus_assist(); //环岛辅助函数
	  //triangle_assist();//直角辅助函数
	ADC_PWM = LocP_DCalc(&TurnPID,0,Current_Dir);//位置式PD控制转向
	//ADC_PWM = PlacePID_Control(&TurnPID,0,Current_Dir);//动态位置式PID控制 (用于转向控制)
}
/***************************************************************************************
函数名：int16 range_protect(int16 duty, int16 min, int16 max)
功  能：限幅保护 
参  数：
返回值：duty
**************************************************************************************/
int16 range_protect(int16 duty, int16 min, int16 max)//限幅保护
{
  if (duty >= max)
  {
    return max;
  }
  if (duty <= min)
  {
    return min;
  }
  else
  {
    return duty;
  }
}
