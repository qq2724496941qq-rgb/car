#ifndef _debug_h
#define _debug_h

#include "headfile.h"



//***************函数宏定义****(下面这些函数请修改宏定义为对应的GPIO库函数操作)***********
#define KEY_INT(key_x)           gpio_mode(key_x,GPI_IMPEDANCE) 
#define SWITCH_INT(switch_x)     gpio_mode(switch_x,GPI_IMPEDANCE)
#define READ_GPIO(Pin_X)         Pin_X
#define TiaoCan_DelayMs(M_S)     delay_ms(M_S)   //延时函数
  

//外部函数声明
void BUZZ_Init();
void Data_Send(UARTN_enum uratn,signed short int *pst);
void datasend(void);
void display( int hang,int date);
void Switch_Key_init(void);          //拨码开关初始化
void Strategy_Slect(void) ;          //拨码开关策略选择
void Key_Scan_Deal (void );          //按键处理函数
void BUZZ_DiDiDi(uint16 PinLV);              //蜂鸣器滴滴滴
void Test_Key();
void Menu();
void Menu_Speed();
void Set_Speed();
void Menu_Pid();
void Set_Pid();
void Menu_Set_Pid();
void Pid_add();
void Pid_sub();

//外部变量申明
extern unsigned char Switch1,Switch2;//拨码

#endif  
