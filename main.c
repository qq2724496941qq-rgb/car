#include "headfile.h"
void Show_road_type()
{
//	ips114_showstr(0,5,"bend:");
//	ips114_showuint16(8*10,5,road_type.bend);
	ips114_showstr(0,6,"straight:");
	ips114_showuint16(8*10,6,road_type.straight);
	ips114_showstr(0,7,"triangle:");
	ips114_showuint16(8*10,7,road_type.triangle);
}
uint32 dat_len = 0;
uint8 test_str[] = "seekfree.taobao.com";
	
uint16 ad_result = 0;           //引脚电压
uint16 battery_voltage = 0;     //电池电压
uint32 temp2;
uint32 aa = 0;
void main()
{
	WTST = 0;       			//设置程序代码等待参数，赋值为0可将CPU执行程序的速度设置为最快
	board_init();		       	// 初始化寄存器
	DisableGlobalIRQ();     	//关闭总中断


//	delay_ms(200);          	//软件稍微延时一下
	
	ips114_init();    			//1.14寸液晶屏初始化
	ADC_int();      			//ADC采集初始化
	ips114_showstr(0,1,"interlize...");
//	delay_ms(200);

//	BUZZ_Init();

	init_PWM();                 //初始化DRV驱动方式
	encoder_init();             //编码器初始化
	wireless_uart_init();       //无线蓝牙初始化
	//BUZZ_DiDiDi(200);             //蜂鸣器滴一声
	delay_ms(50);
	PID_int();                            //PID参数初始化
	
	Switch_Key_init();						//按键拨码开关初始化	
	
	while(imu660ra_init())                                     //陀螺仪初始化
	{	
		ips114_showstr(0,2,"imu660ra_init...");                   //用while写法直到初始化成功才会退出循环
		delay_ms(500);
	}
	
	ips114_showstr(0,3,"imu660ra_init");

                            
	

//	delay_ms(500);
	pit_timer_ms(TIM_1, 5);										//初始化定时器1作为周期中断触发器，5MS进一次中断
	ips114_showstr(0,6,"intall_intok...");
	ips114_clear(WHITE);                                       //清屏
	adc_init(BAT_VOL_PIN, ADC_SYSclk_DIV_32);
		
//    while(battery_voltage <= 9600&& aa == 0)
//	{
//		ad_result = adc_once(BAT_VOL_PIN, ADC_12BIT);
//		temp2 = (((uint32)ad_result * 5000) / 4096);  //计算出当前adc引脚的电压 计算公式为 ad_result*VCC/ADC分辨率    VCC单位为mv
//		battery_voltage =  temp2 * 7.39;//根据引脚电压  和分压电阻的阻值计算电池电压 计算公司为   引脚电压*(R2+R3)/R3   R3为接地端电阻	
//		delay_ms(100);
//		ips114_showstr(0, 0, "v1:");
//		ips114_showuint16(8*3, 0, battery_voltage);
//		ips114_showstr(0, 1, "ADC = ");
//		ips114_showuint16(8*8, 1, ad_result); 
//}
//    aa=1;
	ips114_clear(WHITE);                                       //清屏
	EnableGlobalIRQ();                                         //初始化完毕，开启总中断
	while(1){
		Key_aim_speed=45;
		//Test_Key();//按键调试
		while(1)
		{
//			ips114_showint16(0,0,left_speed);
//			ips114_showint16(0,1,right_speed);
//			ips114_showint16(0,2,real_speed);
//			ips114_showint16(0,3,Key_aim_speed);
//			//电感显示
//			ips114_showint16(0,0,test_str);
//			wireless_uart_send_buff(test_str, sizeof(test_str)-1);
			ips114_showuint16(0,0,Left_Adc);  
			ips114_showuint16(0,1,Left_Shu_Adc);
			ips114_showuint16(0,2,Right_Shu_Adc);
			ips114_showuint16(0,3,Right_Adc);  
			ips114_showfloat(0,4,Current_Dir,3,2);//显示浮点数   整数显示2位   小数显示1位
			Show_road_type();
			ips114_showint16(0,5,time1);
			
			//ips114_showfloat(0,6,flag_angle,6,1);
			printf("%d,%d\n",aim_speed,real_speed);
			
//			if (!READ_GPIO(KEY4))
//			{
//				Key_Num=0;		//进入菜单选择
//				flag=0;
//				go_motor(0,0);
//				ips114_clear(WHITE);
//				break;
//			}
		}
	}
}

