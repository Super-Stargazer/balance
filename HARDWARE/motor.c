#include "motor.h"
#include "data_deal.h"
/**
  * @brief  电机结构体初始化函数
  * @param  MOTOR   电机结构体指针变量
  * @param  phtimx  电机编码器对应定时器句柄
  * @retval 无
  */
void Motor_Init(Motor_TypeDef *MOTOR,TIM_HandleTypeDef* phtimx)
{
    if(MOTOR==NULL) return;
    MOTOR->last_position = MOTOR->now_position = MOTOR->position_delta = 0;
    MOTOR->angular_delta_all = MOTOR->angular_delta_dt = MOTOR->angular_velocity = 0.0f;
    MOTOR->phtimx = phtimx;
}

/**
  * @brief  电机参数获取函数
  * @param  MOTOR  电机结构体指针变量
  * @param  f      采样频率      
  * @retval 无
  */
void Get_Motor_Para(Motor_TypeDef *motor, uint16_t f)
{
    if(motor==NULL) return;
    //=====读取编码器值=====//
    motor->last_position = motor->now_position;
    motor->now_position = __HAL_TIM_GET_COUNTER(motor->phtimx);
    //=====计算编码器差值=====//规避溢出影响（基于采样周期极短，计数周期极长且为向上计数模式的前提）
    if(motor->phtimx->Instance->CR1 & 0x0010)  //如果是反向计数，都为负值
    {
        if(motor->now_position <= motor->last_position)  //无溢出
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
        if(motor->now_position >= motor->last_position)  //无溢出
        {
            motor->position_delta = motor->now_position- motor->last_position;
        }
        else 
        {
            motor->position_delta = CNTPRD - motor->last_position + motor->now_position;
        }
    }
    //=====计算角度增量，总角度，角速度=====//采样时间内（单位：弧度和s）
    motor->angular_delta_dt= motor->position_delta*(360./RdEcd)*(3.1415/180);
    motor->angular_delta_all += motor->angular_delta_dt;
    motor->angular_velocity = motor->angular_delta_dt * f;
}

void get_D_encoder(Motor_TypeDef *motor)
{
     if(motor==NULL) return;
    //=====读取编码器值=====//
    motor->last_position = motor->now_position;
    motor->now_position = __HAL_TIM_GET_COUNTER(motor->phtimx);
    //=====计算编码器差值=====//规避溢出影响（基于采样周期极短，计数周期极长且为向上计数模式的前提）
    if(motor->phtimx->Instance->CR1 & 0x0010)  //如果是反向计数，都为负值
    {
        if(motor->now_position <= motor->last_position)  //无溢出
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
        if(motor->now_position >= motor->last_position)  //无溢出
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


