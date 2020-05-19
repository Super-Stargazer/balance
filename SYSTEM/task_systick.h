#ifndef __TASK_SYSTICK_H
#define __TASK_SYSTICK_H

#include "zdsys.h"

//系统时钟分频
#define NumOfTaskDelay 5  //系统分频任务组数
extern u16 Task_Delay[NumOfTaskDelay];
void Task_Delay_SysTick(void);


//自定义定时器
typedef struct Timer{
    uint32_t timeout;  //定时时间
    uint32_t repeat;  //循环定时触发时间(自动重装载)
    void (*timeout_cb)(void);  //定时器处理回调函数
    struct Timer* next;  //指向下一个定时器节点
}Timer;


void timer_ticks(void);
void timer_init(Timer* handle, void(*timeout_cb)(), uint32_t timeout, uint32_t repeat);
int timer_start(Timer* handle);
void timer_stop(Timer* handle);
void timer_loop(void);


//---------------------------------------------//

#endif
