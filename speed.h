#ifndef __speed_H_
#define __speed_H_

#include "headfile.h"

#define Limit_Min_Max(data,min,max) (((data)>(max)) ? (max) : (((data) < (min)) ? (min) : (data)))

//变量申明
extern int16 aim_speed;     
extern int16 Key_aim_speed;
extern int16 real_speed;        
extern int16 left_speed;       
extern int16 right_speed;       
extern int16 All_PWM_left;     
extern int16 All_PWM_right;    
extern int16 Speed_pwm_left;      
extern int16 Speed_pwm_right;
extern int16 Speed_pwm_all;      
extern int16 Steer_pwm;
extern uint16 Open_pack_time;

//函数声明
void init_PWM();
void encoder_init(void);
void speed_measure(void);
void go_motor (int16 left_PWM,int16 right_PWM);
void timed_task(void);

#endif
