#include "pid.h"
#include "data_deal.h"

/**
  * @brief  PID结构体初始化函数
  * @param  PID       PID结构体指针变量
  * @param  PID_Init  PID结构体初始化结构体指针变量
  * @retval 无
  */
uint8_t PID_Init(PID_TypeDef *PID, PID_InitTypeDef *PID_Init)
{
    if (PID == NULL || PID_Init == NULL) return PID_ERROR;
    //=====赋值=====//
    PID->mode     = PID_Init->mode;
    PID->Kp       = PID_Init->Kp;
    PID->Ki       = PID_Init->Ki;
    PID->Kd       = PID_Init->Kd;
    PID->max_out  = PID_Init->max_out;
    PID->min_out  = PID_Init->min_out;
    PID->max_iout = PID_Init->max_iout;
    PID->deadband = PID_Init->deadband;
    PID->max_err  = PID_Init->max_err;
    //=====置零=====//
    PID->diff     = 0.0f;
    PID->set      = PID->ref       = 0.0f;
    PID->err[NOW] = PID->err[LAST] = PID->err[LLAST] = 0.0f;
    PID->Pout     = PID->Iout      = PID->Dout       = PID->out = 0.0f;

    return PID_OK;
}

/**
  * @brief  PID功能函数
  * @param  PID  PID结构体指针变量
  * @param  set  目标值
  * @param  ref  反馈值
  * @retval 无
  */
void PID_Calculate(PID_TypeDef *PID, float set, float ref)
{
    if(PID == NULL) return;
    //=====计算误差=====//
    PID->err[LLAST] = PID->err[LAST];
    PID->err[LAST]  = PID->err[NOW];
    PID->set        = set;
    PID->ref        = ref;  //实时记录设定值，反馈值
    PID->err[NOW]   = set - ref;
    //=====死区控制=====//
    if(ABS(PID->err[NOW])<=PID->deadband && PID->deadband!=0) return;
    if(ABS(PID->err[NOW])>=PID->max_err  && PID->max_err !=0) return;
    //=====计算输出量=====//
    if(PID->mode == PID_POSITION)  //位置式
    {
        PID->Pout = PID->Kp * PID->err[NOW];
        PID->Iout+= PID->Ki * PID->err[NOW];
        if(PID->max_iout!=0)
            Output_Limit(PID->Iout,PID->max_iout,-PID->max_iout);  //积分输出限幅
        if(PID->diff==0)  //如果不能直接检测到控制量的微分部分
            PID->Dout = PID->Kd * (PID->err[NOW] - PID->err[LAST]);
        else PID->Dout = PID->Kd * PID->diff;
        PID->out  = PID->Pout + PID->Iout + PID->Dout;
    }
    else if(PID->mode == PID_DELTA)  //增量式
    {
        PID->Pout = PID->Kp * (PID->err[NOW] - PID->err[LAST]);
        PID->Iout = PID->Ki * PID->err[NOW];
        PID->Dout = PID->Kd * (PID->err[NOW] - 2.0f*PID->err[LAST] + PID->err[LLAST]);
        if(PID->max_iout!=0)
            Output_Limit(PID->Iout,PID->max_iout,-PID->max_iout);  //积分输出限幅
        PID->out += PID->Pout + PID->Iout + PID->Dout;
    }
    //=====输出限幅=====//
    Output_Limit(PID->out,PID->max_out,PID->min_out);
}

//--------------------------------------USER--------------------------------------//

#define PWM_U 7199
PID_TypeDef BalaPID, VeloPID, TurnPID;  //三环结构体
int BALA_TARGET  = -3;  //直立环目标值（机械中值）
int VELO_TARGET  = 0;  //速度环目标值
int TURN_TARGET  = 0;  //自转环目标值

void MY_PID_Init()
{
    PID_InitTypeDef pid;
    //=====初始化直立环=====//
    pid.mode = PID_POSITION;
    pid.Kp = 400;
    pid.Ki = 0;
    pid.Kd = -6;
    pid.max_out = PWM_U;
    pid.min_out = -PWM_U;
    pid.max_iout = 0;
    pid.max_err = 0;
    pid.deadband = 0;
    PID_Init(&BalaPID,&pid);  
    //=====初始化速度环=====//
    pid.mode = PID_POSITION;
    pid.Kp = 150;
    pid.Ki = 20;
    pid.Kd = 0;
    pid.max_out = PWM_U;
    pid.min_out = -PWM_U;
    pid.max_iout = 1500;
    pid.max_err = 0;
    pid.deadband = 0;
    PID_Init(&VeloPID,&pid);
    //=====初始化角度环=====//
    pid.mode = PID_POSITION;
    pid.Kp = 0;
    pid.Ki = 0;
    pid.Kd = 0;
    pid.max_out = PWM_U;
    pid.min_out = -PWM_U;
    pid.max_iout = 0;
    pid.max_err = 0;
    pid.deadband = 0;
    PID_Init(&TurnPID,&pid);
}

//机械中值 -3

//直立环：480 0 -7   //400 0 -6

//速度环：150 20 0  1500  //150 20   1500

//旋转环：0 0 0  //
