#include "PID.h"

/************************************************
函数名：IncPIDInit(PID *sptr)
功  能：PID参数初始化
参  数：
返回值：void
************************************************/
void IncPIDInit(PID *sptr)
{
    sptr->SumError=0;
    sptr->LastError=0;
    sptr->LLastError=0;

    sptr->Kp=0;
    sptr->Ki=0;
    sptr->Kd=0;
}

/************************************************
函数名：LocP_DCalc(PID *sptr,int16 Setpoint,int16 Turepoint)
功  能：位置式PID控制
参  数：PID *sptr,int16 Setpoint,int16 Turepoint
返回值：float 
************************************************/
int16 LocP_DCalc(PID*sptr,int16 Setpoint,int16 Turepoint)
{
    int16 iError,dError;
    int16 output;
	
    iError=Setpoint-Turepoint;  //偏差

	sptr->SumError+=iError;            //积分(采样时间很短时，用一阶差分代替一阶微分，用累加代替积分)
    dError=(int16)(iError-(sptr->LastError));     //微分
    sptr->LastError=iError;
    if(sptr->SumError>3500) sptr->SumError=3500;   //积分限幅
    if(sptr->SumError<-3500) sptr->SumError=-3500;
	
    output=(int16)(sptr->Kp*iError  //比例项
          +(sptr->Ki*sptr->SumError)//积分项
	
          +sptr->Kd*dError);        //微分项
	
    return(output);
}

/************************************************
函数名：IncPIDCalc(PID *sptr,int16 Setpoint,int16 Turepoint)
功  能：PID控制
参  数：PID *sptr,int16 Setpoint,int16 Turepoint
返回值：int32 iIncpid
************************************************/
int16 IncPIDCalc(PID *sptr,int16 Setpoint,int16 Turepoint)
{
    int16 iError,iIncpid;
	static int8 key; 
    //当前误差
    iError=Setpoint-Turepoint;      //偏差
	if(iError>35)
	{
		key=0;
	}
	else
	{
		key=1;
	}
    sptr->SumError+=iError;         //积分(采样时间很短时,用一阶差分代替一阶微分，用累加代替积分)

    //增量计算
    iIncpid=(int16)(sptr->Kp*(iError-sptr->LastError)
            +key*sptr->Ki*iError
            +sptr->Kd*(iError-2*sptr->LastError+sptr->LLastError));
    //储存误差，用于下次计算

    sptr->LLastError=sptr->LastError;

    sptr->LastError=iError;

    return(iIncpid);
}
/************************************************
函数名：PlacePID_Control(PID *sptr, int16 Setpoint,int16 Turepiont)
功  能：动态位置式PID控制 (一般用于转向控制)
参  数：PID *sptr,int16 Setpoint,int16 Turepoint
返回值：int32 Actual
************************************************/
int16 PlacePID_Control(PID *sptr, int16 Setpoint,int16 Turepiont)
{
    int16 iError,Actual;
    float KP;  //动态P，注意与Kp区分

    iError = Setpoint - Turepiont;
    KP = 1.0 * (iError*iError)/sptr->Kp +sptr->Ki;    //动态P的计算
    sptr->SumError+=iError;
	if(sptr->SumError>SUMERROR)
	{
		sptr->SumError=SUMERROR;
	}
	else if(sptr->SumError<(-1*SUMERROR))
	{
		sptr->SumError=-SUMERROR;
	}
    Actual = (int16)(KP * iError
            + sptr->Kd* ((0.8*iError + 0.2*sptr->LastError) - sptr->LastError));

    sptr->LastError = iError;
    return Actual;
}

/************************************************
函数名：PosionPID_Control(PID *sptr, int16 Setpoint,int16 Turepiont)
功  能：位置式PID控制(类三次函数)
参  数：PID *sptr,int16 Setpoint,int16 Turepoint
返回值：int16 output
************************************************/
int16 PosionPID_Control(TurnPoPID *sptr, int16 Setpoint,int16 Turepiont)
{
    int16 d_out,p_out,output;

    float iError = Setpoint - Turepiont;
    
	//PID输出
	p_out=(int16)(sptr->Kp_a*iError+sptr->Kp_b*iError*iError*iError);
	d_out=(int16)(sptr->Kd*imu660ra_gyro_z);
	output=p_out+d_out;
	
	sptr->LastError=iError;
	return output;
}

