#include "motor.h"
#include "data_deal.h"
/**
  * @brief  ����ṹ���ʼ������
  * @param  MOTOR   ����ṹ��ָ�����
  * @param  phtimx  �����������Ӧ��ʱ�����
  * @retval ��
  */
void Motor_Init(Motor_TypeDef *MOTOR,TIM_HandleTypeDef* phtimx)
{
    if(MOTOR==NULL) return;
    MOTOR->last_position = MOTOR->now_position = MOTOR->position_delta = 0;
    MOTOR->angular_delta_all = MOTOR->angular_delta_dt = MOTOR->angular_velocity = 0.0f;
    MOTOR->phtimx = phtimx;
}

/**
  * @brief  ���������ȡ����
  * @param  MOTOR  ����ṹ��ָ�����
  * @param  f      ����Ƶ��      
  * @retval ��
  */
void Get_Motor_Para(Motor_TypeDef *motor, uint16_t f)
{
    if(motor==NULL) return;
    //=====��ȡ������ֵ=====//
    motor->last_position = motor->now_position;
    motor->now_position = __HAL_TIM_GET_COUNTER(motor->phtimx);
    //=====�����������ֵ=====//������Ӱ�죨���ڲ������ڼ��̣��������ڼ�����Ϊ���ϼ���ģʽ��ǰ�ᣩ
    if(motor->phtimx->Instance->CR1 & 0x0010)  //����Ƿ����������Ϊ��ֵ
    {
        if(motor->now_position <= motor->last_position)  //�����
        {
            motor->position_delta = motor->now_position - motor->last_position;
        }
        else
        {
            motor->position_delta = -(CNTPRD - motor->now_position + motor->last_position);
        }
    } 
    else 
    {
        if(motor->now_position >= motor->last_position)  //�����
        {
            motor->position_delta = motor->now_position- motor->last_position;
        }
        else 
        {
            motor->position_delta = CNTPRD - motor->last_position + motor->now_position;
        }
    }
    //=====����Ƕ��������ܽǶȣ����ٶ�=====//����ʱ���ڣ���λ�����Ⱥ�s��
    motor->angular_delta_dt= motor->position_delta*(360./RdEcd)*(3.1415/180);
    motor->angular_delta_all += motor->angular_delta_dt;
    motor->angular_velocity = motor->angular_delta_dt * f;
}

void get_D_encoder(Motor_TypeDef *motor)
{
     if(motor==NULL) return;
    //=====��ȡ������ֵ=====//
    motor->last_position = motor->now_position;
    motor->now_position = __HAL_TIM_GET_COUNTER(motor->phtimx);
    //=====�����������ֵ=====//������Ӱ�죨���ڲ������ڼ��̣��������ڼ�����Ϊ���ϼ���ģʽ��ǰ�ᣩ
    if(motor->phtimx->Instance->CR1 & 0x0010)  //����Ƿ����������Ϊ��ֵ
    {
        if(motor->now_position <= motor->last_position)  //�����
        {
            motor->position_delta = motor->now_position - motor->last_position;
        }
        else
        {
            motor->position_delta = -(CNTPRD - motor->now_position + motor->last_position);
        }
    } 
    else 
    {
        if(motor->now_position >= motor->last_position)  //�����
        {
            motor->position_delta = motor->now_position- motor->last_position;
        }
        else 
        {
            motor->position_delta = CNTPRD - motor->last_position + motor->now_position;
        }
    }
}


//--------------------------------------USER--------------------------------------//
Motor_TypeDef lMotor,rMotor;
void MY_motor_init(void)
{
    Motor_Init(&rMotor,&htim3);
    Motor_Init(&lMotor,&htim4);
}


