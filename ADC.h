#ifndef __ADC_H__
#define __ADC_H__

#include "headfile.h"

//环岛检测电感阈值
#define IN_ANNULUS_H_LIMIT        2250   //3000
//出环检测电感阈值  
#define OUT_ANNULUS_S_LIMIT       1950    //2000
//环岛积分距离
#define DISTANCE_ANNULUS_S        320    //350
//环岛打角积分
#define DISTANCE_ANNULUS_Z        420   //450
//环岛积分距离
#define DISTANCE_ANNULUS_IN_TO_ON  250  
//出环岛积分
#define DISTANCE_ANNULUS_ON_TO_OUT 180    //270
//预出环打角积分
#define DISTANCE_ANNULUS_TURN      800    //600
//出环岛直走积分
#define DISTANCE_ANNULUS_T         420     //300


//出界判断
#define OUTSIDE                   30
//避障模块设定距离(根据实际情况修改)
#define SET_DLLB_DISTANCE         800
//直角打角积分
#define TRIANGLE_DISTANCE_Z     200



#define bend_cha 	140
#define bend_min	420
#define bend_max	470
#define triangle_min 700
#define triangle_max 850
#define trcha 500
#define cross_road_max 700

//赛道类型判断 
struct ROAD_TYPE
{
     int8 straight;               //直道
	 int8 bend;                   //弯道
     int8 annulus;                //环岛
	 int8 triangle;				  //直角
	 int8 crossroad;			  //十字路口
	
     int8 in_annulus_left;        //入左环岛
	 int8 in_annulus_right;       //入右环岛
     int8 on_annulus_left;        //在左环岛
	 int8 on_annulus_right;       //在右环岛
     int8 out_annulus;            //出环岛
	 int8 in_park;                //入库
};
extern struct ROAD_TYPE road_type;

//变量声明
extern uint16 adc_value[4];  
extern uint16 AD_V[4]; 
extern uint16 adc_max[4];  
extern uint16 adc_min[4];  
extern uint16 Left_Adc,Right_Adc,Left_Shu_Adc,Right_Shu_Adc;
extern int8 Adc_Num;          
extern uint16 annulus_s;         
extern uint16 annulus_t;
extern uint16 annulus_in_to_on;
extern uint16 annulus_z; 
extern uint16 annulus_p; 
extern uint16 annulus_out; 
extern float Current_Dir;   
extern float flag_angle;
extern uint8 flag_obstacle,flag_back,flag_reset,flag_next,flag_number;
extern uint8 flag_ramp;
extern uint16 obstacle_time;
extern int16 ADC_PWM;            
extern uint8 temp;
extern uint16 triangle_z;	//直角角度积分
extern int Key_Num;
extern int flag;
extern uint16 time1;
//函数声明
void ADC_int(void);                   
void ADC_Collect(void);               
void Data_current_analyze(void);      
float Cha_bi_he(int16 left,int16 right,float ADC_A,int16 left_shu,int16 right_shu,float ADC_B,int16 x);
float zf_Cha_bi_he(int16 left,int16 left_shu,int16 right,int16 right_shu,int16 x);
void Road_type_judge(void);    
void Annulus_handle(void);      
int16 Direction_error(void);    
void init_Steer_PWM(void);      
void Steering_Control_Out(int16 duty);  
void Out_protect(void);                  
void Get_deviation(void);
void Annulus_assist(void);
void obstacle_avoidance(void);
void triangle_handle(void);
void triangle_assist(void);

#endif