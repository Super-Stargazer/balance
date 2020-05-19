#ifndef __CONTROL_H
#define __CONTROL_H
#include "zdsys.h"
#include "delay.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "key.h"
#include "motor.h"
#include "pid.h"
#include "task_systick.h"
#include "niming.h"
#include "myuart.h"
#include "filter.h"
//////////////////////////////////
extern u8 MotorStart;
extern u8 Car_Mode;
extern u8 ReportStart;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
//////////////////////////////////////////////////////





/////////////////////////////////////////////
#define TIMER_TIMEOUT_10MS 10

extern float tmp1;
extern float tmp2;
extern Timer timer1,timer2,timer3;
void timer1_callback(void);
void timer2_callback(void);
void timer3_callback(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
//////////////////////////////////////////////////////////////////
void security(void);
void Set_Pwm(int moto1_PWM,int moto2_PWM);
#endif

