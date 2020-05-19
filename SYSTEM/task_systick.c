#include "task_systick.h"

//����ʱ������
u16 Task_Delay[NumOfTaskDelay] = {10,4,};

/**
  * @brief  ��systick�жϺ���ִ�У��ṩʱ��
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
void timer_ticks(void)  //��systick�жϺ���ִ��
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
//����ʱ�������ӵ���������б��棬ѭ��ָ���������һ���ڵ�ȥ��Ӷ�ʱ���ڵ�
//���������ͬһ����ʱ���������ֱ�ӷ���-1����ʾ��ǰ��Ӿ�����Ϸ�
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
//�����ʹ����һ������ָ��curr��ָ���˶�Ӧ��ʱ������ĵ�ַ
//ͨ��ѭ���������ҵ���Ӧ�ľ������ɾ��
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
//ͨ�����ϱ�����������ڵ㣬�ж��Ƿ񵽴ﶨʱʱ��(timeout����)
//��������˶�ʱʱ��,û��ָ��ѭ����ʱ����ʱ��(repeat����)��ʱ����ʱ�ͻ�ѵ�ǰ��ʱ��������Ƴ�
//���ָ����ѭ����ʱ����ʱ��(repeat����)����ʱʱ��ᱻ���¸�ֵ��ֱ����һ����ʱ��������������һֱѭ��������
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


