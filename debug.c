#include "debug.h"
/*************************使用说明****************************************
本协议与“Visual Scope”软件协议兼容，用过的可以直接用原来的下位机协议即可
首次使用时：
1.将“outputdata.c”和“outputdata.h”添加到你的工程中
2.在“outputdata.c”中包含你原程序的串口发送函数头文件
3.将uart_putchar(databuf[i]);语句替换为你的串口字节发送函数，如send_char(databuf[i]);
4.在你的程序需要发送波形数据的.c文件中添加包含：#include "outputdata.h"，并在本文件中调用函数OutPut_Data(x,y,z,w);
  其中形参x，y，z，w就是传入四个short int 16位数据，分别对应通道1,2,3,4
************************************************************************/
#include "debug.h"
//****************************移植**************************//
int Key_Num=0;
int flag=0;

void Data_Send(UARTN_enum uratn,signed short int *pst)
{
		unsigned char _cnt=0; unsigned char sum = 0;
    unsigned char data_to_send[23] = {0};         //发送缓存
    unsigned char i;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x02;
    data_to_send[_cnt++]=0;
    data_to_send[_cnt++]=(unsigned char)(pst[0]>>8);  //高8位
    data_to_send[_cnt++]=(unsigned char)pst[0];  //低8位
    data_to_send[_cnt++]=(unsigned char)(pst[1]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[1];
    data_to_send[_cnt++]=(unsigned char)(pst[2]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[2];
    data_to_send[_cnt++]=(unsigned char)(pst[3]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[3];
    data_to_send[_cnt++]=(unsigned char)(pst[4]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[4];
    data_to_send[_cnt++]=(unsigned char)(pst[5]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[5];
    data_to_send[_cnt++]=(unsigned char)(pst[6]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[6];
    data_to_send[_cnt++]=(unsigned char)(pst[7]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[7];
    data_to_send[_cnt++]=(unsigned char)(pst[8]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[8];


    data_to_send[3] = _cnt-4;

    sum = 0;
    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];

    data_to_send[_cnt++] = sum;

        for(i=0;i<_cnt;i++)
    uart_putchar(uratn,data_to_send[i]);
}

//===================================================上位机相关的==========================================================
//=============================================上位机使用匿名科创地面站======================================================
/******************************************数据传输****************************************************
函数：void datasend()
参数：  无
说明： 可以同时传输6个数  icm_gyro_x   icm_acc_x ICM_Real.gyro.y  ICM_Real.acc.z   
常看的变量：icm_acc_x  icm_gyro_y  Angle  adc_date[0] Left_Adc
*****************************************************************************************************/
void datasend()
{  
   short send_data[6];                      

   send_data[0]= left_speed; ////////ICM_Start.acc.x 
   send_data[1]= Key_aim_speed; //////////////////    MpuStart.gyro.x   Angle  
   send_data[2]= right_speed; //////////
   send_data[3]= 400; //
   send_data[4]= 0;
   send_data[5]= 0;
 //Data_Send(UART_4,send_data);
   Data_Send(UART_User,send_data);
}

unsigned char Switch1=0,Switch2=0;//拨码开关
char parameter=0;//参数选择

//开关状态变量
uint8 key1_status = 1,key2_status = 1,key3_status = 1, key4_status = 1;
//上一次开关状态变量
uint8 key1_last_status, key2_last_status, key3_last_status, key4_last_status;
//开关标志位
uint8 key1_flag=0,key2_flag=0,key3_flag=0, key4_flag=0;
/*****************拨码开关及按键初始化*****************
函数：void Switch_Key_init()
功能：初始化IO
参数：  无
说明： 初始化IO口   gpio_init(D1, GPI, GPIO_HIGH, GPI_PULL_UP); GPO_PUSH_PULL
******************************************************/
void Switch_Key_init()
{

  //拨码开关初始化
	SWITCH_INT(Switch_Pin_1) ;
	SWITCH_INT(Switch_Pin_2) ;
    
  //按键初始化
	KEY_INT(KEY1);
	KEY_INT(KEY2);
	KEY_INT(KEY3);
	KEY_INT(KEY4);

}

void BUZZ_Init()
{
	gpio_mode(P6_7,GPO_PP);	//将P6.7设置为推挽输出
	BUZZ=BUZZ_OFF;
}
/**********************************************蜂鸣器******************************************
函数：void BUZZ_DiDiDi()
功能：蜂鸣器
***************************************************************************************************/
void BUZZ_DiDiDi(uint16 PinLV)
{
	BUZZ=BUZZ_ON;
	TiaoCan_DelayMs(PinLV);
	BUZZ=BUZZ_OFF;
}

//void Read_Key(int num)
//{
//	
//	
//	
//}

void Menu()
{	                                      //清屏			
	ips114_showstr(0,0,"Key 1:set speed");
	ips114_showstr(0,1,"Key 2:set pid");
	ips114_showstr(0,2,"Key 3:re_choose");
	ips114_showstr(0,3,"Key 4:return");	
	ips114_showint16(0,4,Key_Num);
	ips114_showint16(0,5,flag);
}

void Test_Key (void)
{
	while(Key_Num==0&&flag==0)
	{
		Menu();
		if (!READ_GPIO(KEY1))
		{
			Key_Num=1;
			Set_Speed();
			ips114_clear(WHITE);
		} 
		if (!READ_GPIO(KEY2)) 
		{
			Key_Num=1;
			Set_Pid();
			ips114_clear(WHITE);
		}
		if (!READ_GPIO(KEY3))
		{
			Key_Num=0;
			ips114_clear(WHITE);
		}
		if (!READ_GPIO(KEY4))
		{
			flag=1;
			ips114_clear(WHITE);
		}
	}

}

void Menu_Speed()
{
	ips114_showstr(0,0,"Key 1:speed:30%");
	ips114_showstr(0,1,"Key 2:speed:50%");
	ips114_showstr(0,2,"Key 3:speed:100%");
	ips114_showstr(0,3,"Key 4:re_choose");
	ips114_showstr(0,4,"aim_speed:");
	ips114_showint16(9*10,4,Key_aim_speed);
	ips114_showint16(0,5,Key_Num);	
}

void Set_Speed()
{
	ips114_clear(WHITE);                                       //清屏			
	while(Key_Num==1)	//当前为1级菜单
	{
		Menu_Speed();
		if (!READ_GPIO(KEY1))
		{
			Key_aim_speed = 300;
		} 
		if (!READ_GPIO(KEY2)) 
		{
			Key_aim_speed = 500;

		}
		if (!READ_GPIO(KEY3))
		{
			Key_aim_speed = 800;  	 

		}
		if (!READ_GPIO(KEY4))
		{
			Key_Num=0;
		}
	}
}

void Menu_Pid()
{
	ips114_showstr(0,0,"Kp:");
	ips114_showstr(0,1,"Ki:");
	ips114_showstr(0,2,"Kd:");
	ips114_showfloat(2*10,0,L_SpeedPID.Kp,3,3);
	ips114_showfloat(2*10,1,L_SpeedPID.Ki,3,3);
	ips114_showfloat(2*10,2,L_SpeedPID.Kd,3,3);
	ips114_showstr(0,3,"Key 1:Pid++");
	ips114_showstr(0,4,"Key 2:Pid--");
	ips114_showstr(0,5,"Key 3:re_choose");
	ips114_showstr(0,6,"Key 4:return");	
	ips114_showint16(0,7,Key_Num);
}

void Set_Pid()
{
	ips114_clear(WHITE);
	while(Key_Num==1)
	{
		Menu_Pid();
		if (!READ_GPIO(KEY1))
		{
			Key_Num=2;
			Pid_add();
		} 
		if (!READ_GPIO(KEY2)) 
		{
			Key_Num=2;
			Pid_sub();
		}
		if (!READ_GPIO(KEY3))
		{
			Key_Num=0;
		}
		if (!READ_GPIO(KEY4))
		{
			Key_Num=0;
		}
	}
}

void Menu_Set_Pid()
{
	ips114_showstr(0,0,"Kp:");
	ips114_showstr(0,1,"Ki:");
	ips114_showstr(0,2,"Kd:");
	ips114_showfloat(2*10,0,L_SpeedPID.Kp,3,3);
	ips114_showfloat(2*10,1,L_SpeedPID.Ki,3,3);
	ips114_showfloat(2*10,2,L_SpeedPID.Kd,3,3);
	ips114_showstr(0,3,"Key 1:Kp");
	ips114_showstr(0,4,"Key 2:Ki");
	ips114_showstr(0,5,"Key 3:Kd");
	ips114_showstr(0,6,"Key 4:re_choose");	
	ips114_showint16(0,7,Key_Num);
}

void Pid_add()
{
	ips114_clear(WHITE);
	while(Key_Num==2)
	{
		Menu_Set_Pid();
		if (!READ_GPIO(KEY1))
		{
			TiaoCan_DelayMs(20);
			L_SpeedPID.Kp++;
		} 
		if (!READ_GPIO(KEY2)) 
		{
			TiaoCan_DelayMs(20);
			L_SpeedPID.Ki+=0.1;
		}
		if (!READ_GPIO(KEY3))
		{
			TiaoCan_DelayMs(20);
			L_SpeedPID.Kd+=0.1;
		}
		if (!READ_GPIO(KEY4))
		{
			Key_Num=1;
		}
	}
}

void Pid_sub()
{
	ips114_clear(WHITE);
	while(Key_Num==2)
	{
		Menu_Set_Pid();
		if (!READ_GPIO(KEY1))
		{
			L_SpeedPID.Kp--;
		} 
		if (!READ_GPIO(KEY2)) 
		{
			L_SpeedPID.Ki-=0.1;
		}
		if (!READ_GPIO(KEY3))
		{
			L_SpeedPID.Kd-=0.1;
		}
		if (!READ_GPIO(KEY4))
		{
			Key_Num=1;
		}
	}
}
