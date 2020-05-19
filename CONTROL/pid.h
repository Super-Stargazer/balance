#ifndef __PID_H
#define __PID_H
#include "stm32f1xx.h"

//PID相关符号常量
enum PID_PARAM {
    NOW 	= 0,
    LAST 	= 1,
    LLAST	= 2,

    PID_ERROR = 0,
    PID_OK,
    PID_POSITION,
    PID_DELTA,

};

//PID控制结构体
typedef  struct {
    uint8_t mode;  //模式

    float Kp, Ki, Kd;  //系数

    float max_out, min_out, max_iout;  //限幅

    float max_err, deadband;  //死区控制

    float Pout, Iout, Dout, out;  //输出量

    float ref, set;  //反馈值,设定值
    float err[3];  //误差值
    float diff;  //控制量的微分值，直接用于位置式PID，为0则不用

} PID_TypeDef;

//PID初始化结构体
typedef struct {
    uint8_t mode;

    float Kp, Ki, Kd, set;

    float max_out, min_out, max_iout;

    float max_err, deadband;

} PID_InitTypeDef;

uint8_t PID_Init(PID_TypeDef *PID, PID_InitTypeDef *PID_Init);  //PID结构体初始化函数
void PID_Calculate(PID_TypeDef *PID, float set, float ref);  //PID输出计算函数

//--------------------------------------USER--------------------------------------//

extern PID_TypeDef BalaPID, VeloPID, TurnPID;
extern int BALA_TARGET;  //俯仰角目标值
extern int TURN_TARGET;  //角速度目标值
extern int VELO_TARGET;  //速度环目标值

void MY_PID_Init(void);

#endif
