#include "ADC.h"
#include "math.h"

uint16 adc_value[4];                 //储存电感采集值原始值    4个电感 
uint16 AD_V[4];                      //储存电感采集值归一化值中间变量 （无需关心，请勿删除）
uint16 adc_max[4]={102,90,105,90}; //电感采值最大值 需要自己采集
uint16 adc_min[4]={0,0,0,0};        //电感采值最小值
uint16 Left_Adc,Right_Adc,Left_Shu_Adc,Right_Shu_Adc;//电感值
int8 Adc_Num=4;                          //电感个数
uint16 time1=0;
int8 on_flag=0;
int8 out_flag=0;
int8 annulus_flag=0;			//环岛标志位
int8 in_flag=0;					//进环岛标志位
int8 direction_flag=0;					//进环岛标志位
int8 on_to_out_flag = 0;		//预出环直走一小段标志位
int8 turn_flag      = 0;		//出环打角标志位

//环道参数
uint16 annulus_s     = 0;           //环岛积分距离
uint16 annulus_z     = 0;           //环岛积分打角
uint16 annulus_in_to_on = 0;		//进环直走一小段

uint16 annulus_on_to_out = 0;		//预出环直走一小段
uint16 annulus_turn      = 0;		//出环打角
uint16 annulus_t     	 = 0;		//出环直走


//直角参数
uint16 triangle_z=0;	//直角角度积分


struct ROAD_TYPE road_type = {0};
//int16 obstacle_Current_Dir[]={
//	                            150,150,150,150,150,150,150,150,150,150,
//	                            150,150,150,150,150,150,150,150,150,150,
//	                            150,150,150,150,150,150,150,150,150,150,
//	                            150,150,150,150,150,150,150,150,150,150,
//	                            150,150,150,150,150,150,150,150,150,150,
//	                            150,150,150,150,150,150,150,150,150,150,
//                             };
/***当前位置*************/
float Current_Dir = 0;
float flag_angle = 0;
int16 ADC_PWM=0;
uint8 flag_obstacle,flag_back,flag_reset,flag_next,flag_number=0;
uint8 flag_ramp=0;	//坡道标志位
uint16 obstacle_time=0;
uint8 temp=0;				 
/***************************电感采集通道初始化****************************
函数：  void ADC_int(void)  
功能：  电感采值进行初始化
参数：  void
说明：  电感采集初始化
返回值；无
************************************************************************/
void ADC_int(void)
{
	adc_init(Left_ADC_Pin,ADC_SYSclk_DIV_2);//初始化P0.0为ADC功能
	adc_init(LeftXie_ADC_Pin,ADC_SYSclk_DIV_2);//初始化P0.1为ADC功能
	adc_init(RightXie_ADC_Pin,ADC_SYSclk_DIV_2);//初始化P0.5为ADC功能
	adc_init(Right_ADC_Pin,ADC_SYSclk_DIV_2);//初始化P0.6为ADC功能 
}

/***************************中值滤波函数*********************************
函数：uint16 adc_mid(ADCN_enum adcn,ADCCH_enum ch)  
功能： 3次电感采值进行中值滤波
参数： adcn        选择ADC通道       resolution      分辨率
说明： 8位ADC输出，0~255（2的8次方），5v电压平均分成255份，分辨率为5/255=0.196
返回值；k(uint8)中间那个值
************************************************************************/
uint16 adc_mid(ADCN_enum adcn,ADCRES_enum ch)
{
	uint16 i,j,k,tmp;
	i=adc_once(adcn,ch);
	j=adc_once(adcn,ch);
	k=adc_once(adcn,ch);
	if(i>j)
	{
		tmp=i,i=j,j=tmp;
	}
	if(k>j)
	{
		tmp=j;
	}
	else if(k>i)	//ikj
	{
		tmp=k;
	}
	else	//kij
	{
		tmp=i;
	}
	return(tmp);
}

/***************************均值滤波函数****************************
函数：  uint16 adc_ave(ADCN_enum adcn,ADCCH_enum ch,uint8 N) 
功能：  中值滤波后的5个电感值求平均值
参数：  adcn        选择ADC通道         
说明：  该函数调用中值滤波函数，即电感值是中位置
返回值；tmp
*******************************************************************/
uint16 adc_ave(ADCN_enum adcn,ADCRES_enum ch,uint8 N)
{
	uint16 tmp=0;
	uint8 i;
	for(i=0;i<N;i++)
	{
	  tmp+=adc_mid(adcn,ch);
		
	}
	tmp=tmp/N;
	return(tmp);
}
/***************************电感采值************************************
函数：  void ADC_Collect()   
功能：  电感采值
参数：  void
说明：  8...12位ADC输出，0~255（2的8次方），5v电压平均分成255份，分辨率为5/255=0.196
返回值；void
改成12试一下？
***********************************************************************/
void ADC_Collect()
{
	adc_value[0]=adc_ave(Left_ADC_Pin,ADC_8BIT,5);     //左横电感
	adc_value[1]=adc_ave(LeftXie_ADC_Pin,ADC_8BIT,5);  //左竖电感
	adc_value[2]=adc_ave(RightXie_ADC_Pin,ADC_8BIT,5); //右竖电感
	adc_value[3]=adc_ave(Right_ADC_Pin,ADC_8BIT,5);    //右横电感
}

/*********************************电感采值********************************
函数：  void Data_current_analyze()   
功能：  电感采值原始值归一化（0~100）
参数：  void
说明：  归一化处理
返回值；void       
*************************************************************************/
void Data_current_analyze()
{
	uint8 i;
	for(i=0;i < Adc_Num; i++)              
	{
		AD_V[i] = ((adc_value[i]-adc_min[i])*100)/adc_max[i];         
		if( AD_V[i]<=0)
		{
			AD_V[i]=0;
		}
		else if(AD_V[i]>=300)
		{
			AD_V[i]=100;
		}
	}	if (time1<500){
	     time1 = time1 + 1;
		Left_Adc = 100;       //左电感最终值
	Left_Shu_Adc = 50;   //左竖电感最终值
	Right_Shu_Adc = 50;  //右竖电感最终值
	Right_Adc = 100;	    //右电感最终值
	}
	else{
	Left_Adc = AD_V[0]*10;       //左电感最终值
	Left_Shu_Adc = AD_V[2]*10;   //左竖电感最终值
	Right_Shu_Adc = AD_V[3]*10;  //右竖电感最终值
	Right_Adc = AD_V[1]*10;	    //右电感最终值
	}
}
/*****************************************竖直偏差函数*************************************
函数：  Calculate_Vertical( 
参数：  无
******************************************************************************************/
double Calculate_Vertical()
{
double k = 0;
double sum = 180.0; // 两个竖直电感最大值的和
k = fabs(Left_Shu_Adc/10 - Right_Shu_Adc/10) / sum; // 浮点型绝对值使用 fabs()
// 分母不能为 0，所以最小保护为 0.1
return k >= 1.0 ? 1.0 : (k <= 0.1 ? 0.1 : k); // 将 k 限幅保护在 0.1 ~ 1.0 之间
}
/*********************************差比和函数**********************************
函数：  float Cha_bi_he(int16 data1, int16 data2,int16 x)
功能：  差比和求赛道偏差
参数：  int16 data1, int16 data2,int16 x
说明：  差比和求赛道偏差
返回值；result
****************************************************************************/
//逐飞方案
float zf_Cha_bi_he(int16 left,int16 left_shu,int16 right,int16 right_shu,int16 x)
{
	int16 temp_left=left/10;
	int16 temp_left_shu=left_shu/10;
	int16 temp_right=right/10;
	int16 temp_right_shu=right_shu/10;
	
    int16 cha;
    int16 he;
    float result;
	float left_value;
	float right_value;
	left_value=sqrt(temp_left*temp_left+temp_left_shu*temp_left_shu);
	right_value=sqrt(temp_right*temp_right+temp_right_shu*temp_right_shu);
	cha = left_value-right_value;
	he = left_value+right_value+1;

    result = (cha*x)/he;

	if(result>x) result=x;
	if(result<(-1*x)) result=-1*x;
    return result;
}

float Cha_bi_he(int16 left,int16 right,float ADC_A,int16 left_shu,int16 right_shu,float ADC_B,int16 x)
{

	int16 temp_left			=left/10;
	int16 temp_left_shu		=left_shu/10;
	int16 temp_right		=right/10;
	int16 temp_right_shu	=right_shu/10;

    float cha;
    float he;
    float result;

    cha = ADC_A*(temp_left-temp_right)+ADC_B*(temp_left_shu-temp_right_shu);
    he = ADC_A*(temp_left+temp_right)+ADC_B*(temp_left_shu+temp_right_shu)+1;
    result = (cha*x)/he;
if(result>x) result=x;
	if(result<(-1*x)) result=-1*x;

    return result;
}



/*****************************************判断赛道类型*************************************
函数：  void Road_type_judge(void)
参数：  无
******************************************************************************************/
void Road_type_judge(void)
{	 
//////////	//环岛判断
//////////	if((Left_Adc+Right_Adc)>IN_ANNULUS_H_LIMIT&&road_type.annulus==0&&annulus_flag==0)
//////////	{
//////////		//BUZZ=BUZZ_ON;
//////////		P52 = 0;
//////////		annulus_flag			 = 1;
//////////		road_type.annulus        = 1;
//////////		road_type.straight       = 0;
//////////		road_type.bend           = 0;
//////////		road_type.triangle		 = 0;
//////////		road_type.crossroad		 = 0;
////////	}
////////	//直角判断
//////////	else if(((Right_Shu_Adc < triangle_min && Left_Shu_Adc>triangle_max)||
//////////		(Left_Shu_Adc <triangle_min && Right_Shu_Adc>triangle_max))&&
//////////		(Left_Shu_Adc/Right_Shu_Adc>4||Right_Shu_Adc/Left_Shu_Adc>4))
////////		else if(
////////			((Right_Shu_Adc>triangle_min&&Left_Shu_Adc<triangle_min)||(Left_Shu_Adc>triangle_min&&Right_Shu_Adc<triangle_min))&&road_type.annulus==0
////////			)
////////	{
////////		BUZZ=BUZZ_ON;
////////		road_type.annulus        = 0;
////////		road_type.straight       = 0;
////////		road_type.bend           = 0;
////////		road_type.triangle		 = 1;
////////		road_type.crossroad		 = 0;
////////	}
////////	
////////	//弯道判断	其中一个竖电感大于转弯阈值，另一个小于转弯阈值，且横电感之差大于bend_cha
////////	//并且不在环岛且（没有在环岛上或者已经在环岛上）
////////	else if(
////////			((Right_Shu_Adc>bend_max&&Left_Shu_Adc<bend_min)||(Left_Shu_Adc>bend_max&&Right_Shu_Adc<bend_min))&&road_type.annulus==0
////////			)
////////	{
////////		//BUZZ=BUZZ_ON;
////////		road_type.annulus        = 0;
////////		road_type.straight       = 0;
////////		road_type.bend           = 1;
////////		road_type.triangle		 = 0;
////////		road_type.crossroad		 = 0;
////////	}
////////	//十字路口判断
//////////	else if(Right_Shu_Adc>cross_road_max&&Left_Shu_Adc>cross_road_max)
//////////	{
//////////		//BUZZ=BUZZ_ON;
//////////		road_type.annulus        = 0;
//////////		road_type.straight       = 0;
//////////		road_type.bend           = 0;
//////////		road_type.triangle		 = 0;
//////////		road_type.crossroad		 = 1;
//////////	}
////////	//直道判断：以上类型均不满足则为直道
////////	else if(road_type.annulus == 0)
////////	{   
////////		BUZZ=BUZZ_OFF;
////////		P52 = 1;
////////		road_type.annulus        = 0;
////////		road_type.straight       = 1;
////////		road_type.bend           = 0;	
////////		road_type.triangle		 = 0;
////////		road_type.crossroad		 = 0;
////////	}
////////}
	//环岛判断
	if((Left_Adc+Right_Adc)>IN_ANNULUS_H_LIMIT&&annulus_flag==0)
	{
		//BUZZ=BUZZ_ON;
		P52 = 0;
		annulus_flag			 = 1;
		road_type.annulus        = 1;
		road_type.straight       = 0;
		road_type.bend           = 0;
		road_type.triangle		 = 0;
		road_type.crossroad		 = 0;}
//十字路口判断
	if(Right_Shu_Adc>cross_road_max&&Left_Shu_Adc>cross_road_max)
	{
		//BUZZ=BUZZ_ON;
		road_type.annulus        = 0;
		road_type.straight       = 0;
		road_type.bend           = 0;
		road_type.triangle		 = 0;
		road_type.crossroad		 = 1;
	}
	//直角判断
	//||(Right_Shu_Adc-Left_Shu_Adc>800)||(Left_Shu_Adc-Right_Shu_Adc
	else if(																																							
			((Right_Shu_Adc>triangle_max&&Left_Shu_Adc<triangle_min)||(Left_Shu_Adc>triangle_max&&Right_Shu_Adc<triangle_min)) && road_type.annulus == 0 ) 
	{
		BUZZ=BUZZ_ON;
		road_type.annulus        = 0;
		road_type.straight       = 0;
		road_type.bend           = 0;
		road_type.triangle		 = 1;
		road_type.crossroad		 = 0;
	}
else if((abs(Right_Shu_Adc - Left_Shu_Adc) >= trcha) && road_type.annulus == 0)
	{
		BUZZ=BUZZ_ON;
		road_type.annulus        = 0;
		road_type.straight       = 0;
		road_type.bend           = 0;
		road_type.triangle		 = 1;
		road_type.crossroad		 = 0;
	}
	//弯道
	//弯道判断	其中一个竖电感大于转弯阈值，另一个小于转弯阈值，且横电感之差大于bend_cha
	//并且不在环岛且（没有在环岛上或者已经在环岛上）
//	else if(
//			((Right_Shu_Adc>bend_max&&Left_Shu_Adc<bend_min)||(Left_Shu_Adc>bend_max&&Right_Shu_Adc<bend_min)&&(abs(Right_Adc - Left_Adc) >= bend_cha))
//			)
//	{
//		//BUZZ=BUZZ_ON;
//		road_type.annulus        = 0;
//		road_type.straight       = 0;
//		road_type.bend           = 1;
//		road_type.triangle		 = 0;
//		road_type.crossroad		 = 0;
//	}
	else if(road_type.annulus == 0)
	{   
		BUZZ=BUZZ_OFF;
		P52 =0;
		road_type.annulus        = 0;
		road_type.straight       = 1;
		road_type.bend           = 0;	
		road_type.triangle		 = 0;
		road_type.crossroad		 = 0;}

	}

	
/*****************************************环岛处理***************************************
函数：  void Annulus_handle(void)
参数：  无
说明：  环岛处理函数

*注意：用两个竖电感引导进环
******************************************************************************************/
void Annulus_handle(void)
{
		//左环判断
		if(road_type.annulus==1&&road_type.in_annulus_left==0&&
			(Left_Adc>Right_Adc)&&direction_flag==0)	 
		{
			direction_flag=1;
			road_type.in_annulus_left = 1;
			//BUZZ=BUZZ_ON;
			P52 = 0;
		}
		//右环判断
		else if(road_type.annulus==1&&road_type.in_annulus_right==0&&
		  (Right_Adc>Left_Adc)&&direction_flag==0)
		{
			direction_flag=1;
			road_type.in_annulus_right = 1;
			//BUZZ=BUZZ_ON;
			P52 = 0;
		}
		if(annulus_s > DISTANCE_ANNULUS_S)
		{
			in_flag=1;
		}
		//左环处理
		if(road_type.in_annulus_left == 1)
		{
			if(annulus_z > DISTANCE_ANNULUS_Z&&(road_type.in_annulus_left==1)&&(road_type.on_annulus_left==0))
			{
				on_flag=1;
			}
		  if(annulus_in_to_on > DISTANCE_ANNULUS_IN_TO_ON&&road_type.in_annulus_left==1&&road_type.on_annulus_left==0)
		  {
			  road_type.on_annulus_left = 1;
			  //BUZZ=BUZZ_ON;
			  P52= 1;
			  on_flag=0;
		  }
		  if(road_type.on_annulus_left==1&&road_type.out_annulus==0&& Left_Adc+Right_Adc>OUT_ANNULUS_S_LIMIT)
		  {			
				on_to_out_flag = 1;
                //BUZZ=BUZZ_ON;	
				on_flag=0;			  
		  }
		}
		//右环处理
		else if(road_type.in_annulus_right == 1)
		{
			if(annulus_z > DISTANCE_ANNULUS_Z&&(road_type.in_annulus_right==1)&&(road_type.on_annulus_right==0))
			{
				on_flag=1;
			}
			if(annulus_in_to_on > DISTANCE_ANNULUS_IN_TO_ON&&road_type.in_annulus_right==1&&road_type.on_annulus_right==0)
			{
				road_type.on_annulus_right = 1;
				P52= 1;
				on_flag=0;
				//BUZZ=BUZZ_ON;
			}
			if(road_type.on_annulus_right==1&&road_type.out_annulus==0&& Left_Adc+Right_Adc>OUT_ANNULUS_S_LIMIT)  
			{			
				on_to_out_flag = 1;		
				on_flag=0;
				//BUZZ=BUZZ_ON;
			}
		}
		if(annulus_on_to_out>DISTANCE_ANNULUS_ON_TO_OUT)
		{	  
			on_to_out_flag = 0;
			turn_flag 	   = 1;
		}
		if(annulus_turn>DISTANCE_ANNULUS_TURN)
		{
			turn_flag 	   		  = 0;
			road_type.out_annulus = 1;
		}
		//出环处理
		if(annulus_t>DISTANCE_ANNULUS_T)
		{	  
			  annulus_flag				 = 0;
			  road_type.annulus          = 0;
			  road_type.in_annulus_left  = 0;
			  road_type.in_annulus_right = 0;
			  road_type.on_annulus_left  = 0;
			  road_type.on_annulus_right = 0;
			  road_type.out_annulus      = 0;
			  annulus_s             	 = 0;
			  annulus_z                  = 0;
			  annulus_t                  = 0;
			  P52 = 0;
			  direction_flag=0;
			  in_flag=0;
			  BUZZ=BUZZ_ON; 	
		}
}
/*************************************环岛辅助函数*************************************
函数：  void Annulus_assist(void)
参数：  无
说明：  过环岛三角区积分，进环积分，出环积分等
******************************************************************************************/
void Annulus_assist(void)
{
	//判断到环岛并且没有进入环岛
	if(road_type.annulus==1&&in_flag==0)
	{
		annulus_s += fabs(real_speed*0.1);	//积分距离
	}
	//打角进环岛
	if((road_type.in_annulus_left==1 ||road_type.in_annulus_right==1) && (road_type.on_annulus_left==0&&road_type.on_annulus_right==0))
    {
        annulus_z += fabs(imu660ra_gyro_z*0.01);	//积分角度
	}
	//进环岛到环岛上积分
	if(on_flag==1&&(road_type.in_annulus_left==1||road_type.in_annulus_right==1)&&(road_type.on_annulus_left==0&&road_type.on_annulus_right==0))
	{
		annulus_in_to_on += fabs(real_speed*0.1);	//积分距离
	}
	//在环岛上并且检测到出环岛信号先直走一小段
	if((road_type.on_annulus_left==1 ||road_type.on_annulus_right==1) && on_to_out_flag==1)
    {
        annulus_on_to_out += fabs(real_speed*0.1);	//积分距离
    }
	//在环岛上并且检测到出环岛信号打角
	if((road_type.on_annulus_left==1 ||road_type.on_annulus_right==1) && turn_flag==1)
    {
        annulus_turn += fabs(imu660ra_gyro_z*0.01);	//积分角度
    }
	//在环岛上并且检测到出环岛信号直走出环岛
	if((road_type.on_annulus_left==1 ||road_type.on_annulus_right==1) && road_type.out_annulus==1)
    {
        annulus_t += fabs(real_speed*0.1);	//积分距离
    }
	
}

/*************************************直角辅助函数*************************************
函数：  void triangle_Annulus_assist(void)
参数：  无
说明：  过直角角度积分
******************************************************************************************/
void triangle_assist(void)
{
	if((road_type.triangle==1))
    {
        triangle_z += fabs(imu660ra_gyro_z*0.01);
    }
}
/*****************************************直角处理***************************************
函数：  void triangle_Annulus_handle(void)
参数：  无
说明：  直角处理函数

*注意：用竖电感判断直角
******************************************************************************************/
void triangle_handle(void)
{
	if(triangle_z < TRIANGLE_DISTANCE_Z&&road_type.triangle==1)
	{
		P52 = 0;	//进入直角灯亮
	}
	//出直角处理
	if(triangle_z>=TRIANGLE_DISTANCE_Z)
	{			
		triangle_z=0;
		road_type.triangle=0;
		P52= 1;		//出直角灯灭
	}
}
/*************************************避障检测函数*************************************
函数：  void obstacle_avoidance(void)
参数：  无
说明：  TFO避障模块检测，使用软件模拟IIC通信，理论上任何引脚都可以使用，但是要注意不能引脚
        复用。
*注意： TOF模块离障碍物越远数值越大，越近数值越小
******************************************************************************************/
void obstacle_avoidance(void)
{
	dl1b_get_distance();                                       //距离测量
	if(dl1b_finsh_flag==1&&dl1b_distance_mm<SET_DLLB_DISTANCE&&flag_number==0) //测量距离小于设定值标志位成立
	{
		BUZZ=BUZZ_ON;
		dl1b_finsh_flag=0;
		flag_obstacle=1;
		flag_number = flag_number + 1;
		//ips114_showfloat(0,4,dl1b_distance_mm,2,1);//显示浮点数   整数显示2位   小数显示1位
	}
	
}


/*************************根据赛道类型选择不同的方向偏差计算方法*************************
函数：  int16 Direction_error(void)
功能：  根据赛道类型选择不同的方向偏差
参数：  无
说明：  根据赛道类型选择不同的方向偏差--直道--弯道--环岛处理切换不同差比和
返回值：error--返回赛道偏差
****************************************************************************************/
int16 Direction_error(void)
{
    int16 error = 0;
    //直道方向偏差计算
    if(road_type.straight)
    {
		//error = zf_Cha_bi_he(Left_Adc,Left_Shu_Adc,Right_Adc,Right_Shu_Adc,60); 
		error = Cha_bi_he(Left_Adc,Right_Adc,1,Left_Shu_Adc,Right_Shu_Adc,0,65);
    }
    //直角方向偏差计算
    else if(road_type.triangle==1)
    {
		//error = zf_Cha_bi_he(Left_Adc,Left_Shu_Adc,Right_Adc,Right_Shu_Adc,110);
		error = Cha_bi_he(Left_Adc,Right_Adc,0.15,Left_Shu_Adc,Right_Shu_Adc,0.85,70);
	}
	//普通弯道偏差
	else if(road_type.bend)
    {
		
//		error = zf_Cha_bi_he(Left_Adc,Left_Shu_Adc,Right_Adc,Right_Shu_Adc,110);
		  error = Cha_bi_he(Left_Adc,Right_Adc,0,Left_Shu_Adc,Right_Shu_Adc,1,55);
	}
	//十字路口偏差
	else if(road_type.crossroad)
    {
		//error = zf_Cha_bi_he(Left_Adc,Left_Shu_Adc,Right_Adc,Right_Shu_Adc,60); 
		error =Cha_bi_he(Left_Adc,Right_Adc,1,Left_Shu_Adc,Right_Shu_Adc,0,25);
    }
    //环岛方向偏差计算
    else if(road_type.annulus)
    {
		//准备入环岛方向偏差计算
        if(road_type.on_annulus_left==0 && road_type.on_annulus_right==0 && road_type.out_annulus==0)
		{
			BUZZ=BUZZ_ON;
			error=0;
		}
        //入左环岛方向偏差计算
        if(in_flag==1&&road_type.in_annulus_left ==1 && road_type.on_annulus_left==0 && road_type.out_annulus==0)
		{
			//BUZZ=BUZZ_ON;

			error=25;
		}
		//入右环岛方向偏差计算
		if(in_flag==1&&road_type.in_annulus_right ==1 && road_type.on_annulus_right==0 && road_type.out_annulus==0)
		{
			//BUZZ=BUZZ_ON;

			error=-35;
		}
		if(on_flag==1&&(road_type.in_annulus_left==1||road_type.in_annulus_right==1)&&(road_type.on_annulus_left==0&&road_type.on_annulus_right==0))
		{
			BUZZ=BUZZ_ON;
			error=0; //获得赛道偏差
		}
		
		
		//在环岛上
		if((road_type.on_annulus_left ==1||road_type.on_annulus_right==1)&& road_type.out_annulus==0)
		{
			BUZZ=BUZZ_OFF;
			//error = zf_Cha_bi_he(Left_Adc,Left_Shu_Adc,Right_Adc,Right_Shu_Adc,80);
			error = Cha_bi_he(Left_Adc,Right_Adc,1,Left_Shu_Adc,Right_Shu_Adc,0,80);
		}
		
		
		//出环岛直走一小段
		if(on_to_out_flag==1)
		{
			//BUZZ=BUZZ_ON;
			error =0;
		}
		//出环岛打角
		if(turn_flag==1)
		{
			  //左环岛方向偏差计算
			if(road_type.in_annulus_left ==1)
			{
				error=15;
			}
			//右环岛方向偏差计算
			if(road_type.in_annulus_right ==1)
			{
				error=-15;
			}
		}	
        //出环岛方向偏差计算
        if((road_type.in_annulus_left||road_type.in_annulus_right) && 
			(road_type.on_annulus_left||road_type.on_annulus_right)
			&&road_type.out_annulus)
			{	
				//error = zf_Cha_bi_he(Left_Adc,Left_Shu_Adc,Right_Adc,Right_Shu_Adc,10);
				//error = Cha_bi_he(Left_Adc,Right_Adc,1,Left_Shu_Adc,Right_Shu_Adc,0,40);
				error=0;
			}
	}
//		if(out_flag==1&&out_annulus<DISTANCE_ANNULUS_S)
//		{
//			P52 = 0;	//进入直角灯亮
//			error=0;
//		}
//		else
//		{
//			BUZZ=BUZZ_OFF;
//			P52 = 1;	//进入直角灯亮
//			out_flag=0;
//		}		
    return error;
}

/**********************************电磁所有总处理***************************************
函数：  void Get_deviation(void)
功能：  电磁所有总处理
参数：  无
说明：  放中断调用此函数即可
****************************************************************************************/
void Get_deviation(void)
{
	ADC_Collect();           //电感原始值采值
	Data_current_analyze();  //电感值归一化函数
	Road_type_judge();       //赛道类型判断---直道---弯道---环岛
	Annulus_handle();        //环岛处理
	triangle_handle();	//直角处理
	//obstacle_avoidance();    //障碍物检测
	//ramp_judge();
	
	Current_Dir=Direction_error(); //获得赛道偏差

}