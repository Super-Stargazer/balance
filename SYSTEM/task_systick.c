#include "task_systick.h"

//任务时序数组
u16 Task_Delay[NumOfTaskDelay] = {10,4,};

/**
  * @brief  在systick中断函数执行，提供时序
  * @param  None.
  * @retval None.
  */
void Task_Delay_SysTick(void)
{
    unsigned char i;
    
    for(i=0; i<NumOfTaskDelay; i++)
    {
        if(Task_Delay[i])
        {
            Task_Delay[i]--;
        }
    }
}

//timer handle list head
static Timer* head_handle = NULL;

//Timer ticks
static uint32_t _timer_ticks = 0;

/**
  * @brief  background ticks, timer repeat invoking interval 1ms.
  * @param  None.
  * @retval None.
  */
void timer_ticks(void)  //在systick中断函数执行
{
    _timer_ticks++;
}

/**
  * @brief  Initializes the timer struct handle.
  * @param  handle: the timer handle strcut.
  * @param  timeout_cb: timeout callback.
  * @param  repeat: repeat interval time.
  * @retval None
  */
void timer_init(Timer* handle, void(*timeout_cb)(), uint32_t timeout, uint32_t repeat)
{
    handle->timeout_cb = timeout_cb;
    handle->timeout = _timer_ticks + timeout;
    handle->repeat = repeat;
}

/**
  * @brief  Start the timer work, add the handle into work list.
  * @param  btn: target handle strcut.
  * @retval 0: succeed. -1: already exist.
  */
//将定时器句柄添加到链表里进行保存，循环指向链表的下一个节点去添加定时器节点
//如果发现是同一个定时器句柄，则直接返回-1，表示当前添加句柄不合法
int timer_start(Timer* handle)
{
    Timer* target = head_handle;

    while(target)
    {
        if(target == handle) return -1;	//already exist.

        target = target->next;
    }

    handle->next = head_handle;
    head_handle = handle;
    return 0;
}

/**
  * @brief  Stop the timer work, remove the handle off work list.
  * @param  handle: target handle strcut.
  * @retval None
  */
//巧妙的使用了一个二级指针curr，指向了对应定时器句柄的地址
//通过循环遍历，找到对应的句柄后将其删除
void timer_stop(Timer* handle)
{
    Timer** curr;

    for(curr = &head_handle; *curr; )
    {
        Timer* entry = *curr;

        if (entry == handle)
        {
            *curr = entry->next;
        }
        else
            curr = &entry->next;
    }
}

/**
  * @brief  main loop.
  * @param  None.
  * @retval None
  */
//通过不断遍历链表各个节点，判断是否到达定时时间(timeout参数)
//如果到达了定时时间,没有指定循环定时触发时间(repeat参数)的时候，这时就会把当前定时器句柄给移除
//如果指定了循环定时触发时间(repeat参数)，则定时时间会被重新赋值，直到下一个定时到来，接下来会一直循环触发。
void timer_loop(void)
{
    Timer* target;

    for(target = head_handle; target; target = target->next)
    {
        if(_timer_ticks >= target->timeout)
        {
            if(target->repeat == 0)
            {
                timer_stop(target);
            }
            else
            {
                target->timeout = _timer_ticks + target->repeat;
            }

            target->timeout_cb();
        }
    }
}


