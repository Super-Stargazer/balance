#ifndef __TASK_SYSTICK_H
#define __TASK_SYSTICK_H

#include "zdsys.h"

//ϵͳʱ�ӷ�Ƶ
#define NumOfTaskDelay 5  //ϵͳ��Ƶ��������
extern u16 Task_Delay[NumOfTaskDelay];
void Task_Delay_SysTick(void);


//�Զ��嶨ʱ��
typedef struct Timer{
    uint32_t timeout;  //��ʱʱ��
    uint32_t repeat;  //ѭ����ʱ����ʱ��(�Զ���װ��)
    void (*timeout_cb)(void);  //��ʱ������ص�����
    struct Timer* next;  //ָ����һ����ʱ���ڵ�
}Timer;


void timer_ticks(void);
void timer_init(Timer* handle, void(*timeout_cb)(), uint32_t timeout, uint32_t repeat);
int timer_start(Timer* handle);
void timer_stop(Timer* handle);
void timer_loop(void);


//---------------------------------------------//

#endif
