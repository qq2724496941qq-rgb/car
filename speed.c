#include "speed.h"
#include "math.h"
#define Duty_Max  3000   //限幅最大值
int16 aim_speed = 0;         //目标速度  
int16 Key_aim_speed = 0;         //目标速度 
int16 real_speed = 0;        //左右轮平均速度 
int16 left_speed = 0;        //左轮速度
int16 right_speed = 0;       //右轮速度

int16 All_PWM_left = 0;      //左轮所有PWM输出
int16 All_PWM_right = 0;     //右轮所有PWM输出
int16 Speed_pwm_left = 0;    //左轮速度环PWM（C车用）
int16 Speed_pwm_right = 0;   //右轮速度环PWM（C车用）
int16 Speed_pwm_all = 0;     //左右平均速度环PWM（D车用）
int16 Steer_pwm = 0;         //转向内环PWM
uint16 Open_pack_time = 0;   //打开干簧管定时
uint16 Stop_time = 0;        //停车定时
 
/******************************* 电机初始化***********************************
函数：  void init_PWM()
说明：  分母10000
		pwm_init(PWMA_CH1P_P60, 10000, 0); 
        初始化PWM  使用引脚P6.0  输出PWM频率10000HZ  占空比为百分之 pwm_duty / PWM_DUTY_MAX * 100				
*****************************************************************************/
unsigned char MOTOR_MODE=0;
void init_PWM()
{
	pwm_init(Left_PWM_Pin, 17000,0);//左轮初始化 66
  pwm_init(Right_PWM_Pin,17000,0);//右轮初始化 62 
  pwm_init(Right_DIR_Pin, 17000, 0); //60
	pwm_init(Left_DIR_Pin, 17000, 0); //64
//	gpio_mode(P6_4,GPO_PP);       // 设置DRV方向引脚为为推挽输出
//	gpio_mode(P6_0,GPO_PP);       // 设置DRV方向引脚为为推挽输出 
}
/****************************编码器初始化****************************
函数：  void encoder_init(void)
功能：  编码器初始化
参数：  无
说明：  ctimer_count_init(CTIM0_P34);
        编码器使用TIM3和TIM4，如更改引脚只需修改宏定义即可 
        编码器使用带方向的编码器（STC不支持正交解码）
********************************************************************/
void encoder_init()
{
	//左编码器初始化
	ctimer_count_init(Left_Ecoder_Pin1);
	//右编码器初始化
	ctimer_count_init(Right_Ecoder_Pin1);
}  
/***************************速度测量********************************
函数名：speed_measure()
功  能：速度测量，读取编码器的值
参  数：void
******************************************************************/
void speed_measure()
{ 
	  if(Left_Ecoder_Pin2 == 1)
		{
			left_speed = ctimer_count_read(Left_Ecoder_Pin1);
			ctimer_count_clean(Left_Ecoder_Pin1);
		}
		else
		{
			left_speed = ctimer_count_read(Left_Ecoder_Pin1)* (1);
			ctimer_count_clean(Left_Ecoder_Pin1);
		}
		if(Right_Ecoder_Pin2 == 1)
		{
			right_speed = ctimer_count_read(Right_Ecoder_Pin1)* (1);
			ctimer_count_clean(Right_Ecoder_Pin1);
		}
		else
		{
			right_speed =  ctimer_count_read(Right_Ecoder_Pin1);
			ctimer_count_clean(Right_Ecoder_Pin1);
		}
		
		real_speed=(left_speed+right_speed)/2;
}

/*****************************电机输出*******************************************
函数：void go_motor (int16 left_PWM,int16 right_PWM)
参数：  int16 left_PWM,int16 right_PWM
说明：pwm_duty(PWMA_CH1P_P60, duty);
********************************************************************************/

//-------------------------------------------DRV驱动-------------------------------------
void go_motor (int16 left_PWM,int16 right_PWM)
{
   if (left_PWM>0)                     //左轮
   {
	left_PWM = left_PWM<=Duty_Max ? left_PWM : Duty_Max;		 
     pwm_duty(Left_PWM_Pin,left_PWM);  //正转
   } 	
   else 
   {
     left_PWM = left_PWM>=-Duty_Max ? (-left_PWM) : Duty_Max;  
     pwm_duty(Left_DIR_Pin,left_PWM);  //反转
   }
   
   
   if (right_PWM>0)                    //右轮
   {
     right_PWM = right_PWM<=Duty_Max ? right_PWM : Duty_Max;			 
	   pwm_duty(Right_PWM_Pin,right_PWM);//正转		
	 } 
   else 
   {
     right_PWM = right_PWM>=-Duty_Max ? (-right_PWM) : Duty_Max;  
	   pwm_duty(Right_DIR_Pin,right_PWM); //反转
   }
}