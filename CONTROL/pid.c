#include "pid.h"
#include "data_deal.h"

/**
  * @brief  PID�ṹ���ʼ������
  * @param  PID       PID�ṹ��ָ�����
  * @param  PID_Init  PID�ṹ���ʼ���ṹ��ָ�����
  * @retval ��
  */
uint8_t PID_Init(PID_TypeDef *PID, PID_InitTypeDef *PID_Init)
{
    if (PID == NULL || PID_Init == NULL) return PID_ERROR;
    //=====��ֵ=====//
    PID->mode     = PID_Init->mode;
    PID->Kp       = PID_Init->Kp;
    PID->Ki       = PID_Init->Ki;
    PID->Kd       = PID_Init->Kd;
    PID->max_out  = PID_Init->max_out;
    PID->min_out  = PID_Init->min_out;
    PID->max_iout = PID_Init->max_iout;
    PID->deadband = PID_Init->deadband;
    PID->max_err  = PID_Init->max_err;
    //=====����=====//
    PID->diff     = 0.0f;
    PID->set      = PID->ref       = 0.0f;
    PID->err[NOW] = PID->err[LAST] = PID->err[LLAST] = 0.0f;
    PID->Pout     = PID->Iout      = PID->Dout       = PID->out = 0.0f;

    return PID_OK;
}

/**
  * @brief  PID���ܺ���
  * @param  PID  PID�ṹ��ָ�����
  * @param  set  Ŀ��ֵ
  * @param  ref  ����ֵ
  * @retval ��
  */
void PID_Calculate(PID_TypeDef *PID, float set, float ref)
{
    if(PID == NULL) return;
    //=====�������=====//
    PID->err[LLAST] = PID->err[LAST];
    PID->err[LAST]  = PID->err[NOW];
    PID->set        = set;
    PID->ref        = ref;  //ʵʱ��¼�趨ֵ������ֵ
    PID->err[NOW]   = set - ref;
    //=====��������=====//
    if(ABS(PID->err[NOW])<=PID->deadband && PID->deadband!=0) return;
    if(ABS(PID->err[NOW])>=PID->max_err  && PID->max_err !=0) return;
    //=====���������=====//
    if(PID->mode == PID_POSITION)  //λ��ʽ
    {
        PID->Pout = PID->Kp * PID->err[NOW];
        PID->Iout+= PID->Ki * PID->err[NOW];
        if(PID->max_iout!=0)
            Output_Limit(PID->Iout,PID->max_iout,-PID->max_iout);  //��������޷�
        if(PID->diff==0)  //�������ֱ�Ӽ�⵽��������΢�ֲ���
            PID->Dout = PID->Kd * (PID->err[NOW] - PID->err[LAST]);
        else PID->Dout = PID->Kd * PID->diff;
        PID->out  = PID->Pout + PID->Iout + PID->Dout;
    }
    else if(PID->mode == PID_DELTA)  //����ʽ
    {
        PID->Pout = PID->Kp * (PID->err[NOW] - PID->err[LAST]);
        PID->Iout = PID->Ki * PID->err[NOW];
        PID->Dout = PID->Kd * (PID->err[NOW] - 2.0f*PID->err[LAST] + PID->err[LLAST]);
        if(PID->max_iout!=0)
            Output_Limit(PID->Iout,PID->max_iout,-PID->max_iout);  //��������޷�
        PID->out += PID->Pout + PID->Iout + PID->Dout;
    }
    //=====����޷�=====//
    Output_Limit(PID->out,PID->max_out,PID->min_out);
}

//--------------------------------------USER--------------------------------------//

#define PWM_U 7199
PID_TypeDef BalaPID, VeloPID, TurnPID;  //�����ṹ��
int BALA_TARGET  = -3;  //ֱ����Ŀ��ֵ����е��ֵ��
int VELO_TARGET  = 0;  //�ٶȻ�Ŀ��ֵ
int TURN_TARGET  = 0;  //��ת��Ŀ��ֵ

void MY_PID_Init()
{
    PID_InitTypeDef pid;
    //=====��ʼ��ֱ����=====//
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
    //=====��ʼ���ٶȻ�=====//
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
    //=====��ʼ���ǶȻ�=====//
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

//��е��ֵ -3

//ֱ������480 0 -7   //400 0 -6

//�ٶȻ���150 20 0  1500  //150 20   1500

//��ת����0 0 0  //
