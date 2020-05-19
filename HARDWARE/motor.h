#ifndef __MOTOR_H
#define __MOTOR_H

#include "zdsys.h"

#define WHEEL_D         0.065   //车轮直径 （单位m）
#define ReductionRratio	30 		//电机减速比
#define	LineNumber		13		//编码器线数
#define Multiplier		2		//倍频数
#define RdEcd (ReductionRratio*LineNumber*Multiplier)	//电机转一圈编码器的值
#define CNTPRD 36000  //编码器计数周期
#define PWM_U 7199
#define PWM_D 0

//右电机
#define MOTOR1_IO1     GPIOE
#define MOTOR1_IO1_PIN GPIO_PIN_0
#define MOTOR1_IO2     GPIOE
#define MOTOR1_IO2_PIN GPIO_PIN_1

//左电机
#define MOTOR2_IO1     GPIOE
#define MOTOR2_IO1_PIN GPIO_PIN_2
#define MOTOR2_IO2     GPIOE
#define MOTOR2_IO2_PIN GPIO_PIN_3

//电机方向控制宏指令
#define MOSTOP(MOTOR_IO1,MOTOR_IO1_PIN,MOTOR_IO2,MOTOR_IO2_PIN) \
    do{ \
    HAL_GPIO_WritePin(MOTOR_IO1,MOTOR_IO1_PIN,GPIO_PIN_RESET);\
    HAL_GPIO_WritePin(MOTOR_IO2,MOTOR_IO2_PIN,GPIO_PIN_RESET);\
    }while(0u)

#define MOFORW(MOTOR_IO1,MOTOR_IO1_PIN,MOTOR_IO2,MOTOR_IO2_PIN) \
    do{ \
        HAL_GPIO_WritePin(MOTOR_IO1,MOTOR_IO1_PIN,GPIO_PIN_SET);\
        HAL_GPIO_WritePin(MOTOR_IO2,MOTOR_IO2_PIN,GPIO_PIN_RESET);\
    }while(0u)

#define MOBACK(MOTOR_IO1,MOTOR_IO1_PIN,MOTOR_IO2,MOTOR_IO2_PIN) \
    do{ \
        HAL_GPIO_WritePin(MOTOR_IO1,MOTOR_IO1_PIN,GPIO_PIN_RESET);\
        HAL_GPIO_WritePin(MOTOR_IO2,MOTOR_IO2_PIN,GPIO_PIN_SET);\
    }while(0u)

//电机状态
enum {
    STATIC = -1,  //静止
    ROLLB = 0,  //反转
    ROLLF = 1,   //正转
};


//电机状态结构体
typedef struct {
    TIM_HandleTypeDef* phtimx;  //电机编码器对应定时器句柄
    uint16_t last_position;  //记录编码器累计脉冲数
    uint16_t now_position;
    int16_t position_delta;  //脉冲数增量

    float angular_delta_all;  //总计旋转角度 (单位：°)
    float angular_delta_dt;   //采样周期内角度增量
    float angular_velocity;   //近似角速度 (单位：°/s)
    
    int32_t PWM;  //16位定时器，此用32位保留符号
} Motor_TypeDef;


void Motor_Init(Motor_TypeDef *MOTOR,TIM_HandleTypeDef* phtimx);  //电机结构体初始化函数
void Get_Motor_Para(Motor_TypeDef *motor, uint16_t f);  //电机状态获取函数
void get_D_encoder(Motor_TypeDef *motor);
//--------------------------------------USER--------------------------------------//
extern Motor_TypeDef lMotor, rMotor;
void MY_motor_init(void);


#endif
