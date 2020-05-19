#include "filter.h"
#include "data_deal.h"

/**
  * @brief  一阶低通滤波器结构体初始化函数
  * @param  pft      滤波器结构体指针变量
  * @param  pftinit  滤波器结构体初始化结构体指针变量
  * @retval 无
  */
void LPFOfilter_init(LPFOfilter* pft,LPFO_initstruct* pftinit)
{
    pft->mode = pftinit->mode;
    pft->K    = pftinit->K;
    pft->last_data = pft->now_data = 0;
    if(pftinit->mode)  //如果启用滤波系数的动态调节
    {
        pft->Kw = pftinit->Kw;
        pft->Threshold_A = pftinit->Threshold_A;
        pft->Threshold_T = pftinit->Threshold_T;
        pft->cnt = 0;
        pft->dir_last = pft->dir_now = 0;
    }
}

/**
  * @brief  一阶低通滤波器功能函数
  * @param  pft  滤波器结构体指针变量
  * @retval 无
  */
void LowpassFirstOrderFilter(LPFOfilter* pft)
{
    if(pft->mode)  //如果使用动态调整
    {
        if(pft->now_data > pft->last_data)  //判断变化方向
            pft->dir_now = 1;
        else
            pft->dir_now = 0;
        
        if(pft->dir_now == pft->dir_last)  //判断前后两次变化方向是否相等
        {
            pft->cnt++;
            if(ABS(pft->now_data - pft->last_data)>pft->Threshold_A)  //比较 变化幅度
                pft->cnt+=5;
            if(pft->cnt>pft->Threshold_T)  //比较 单向变化持续时长
            {
                pft->K+=0.2;  //提高灵敏度
                pft->cnt = 0;
            } 
        }
        else
        {
            pft->cnt = 0;
            pft->K = pft->Kw;  //数据平稳时，降低灵敏度
        }
        pft->dir_last = pft->dir_now;
    }
    pft->last_data = pft->K*pft->now_data + (1-pft->K)*pft->last_data;
}

//--------------------------------------USER--------------------------------------//
LPFOfilter ft1,ft2,ft3,ft4,ft5,ft6;

void MY_LPFO_init(void)
{
    LPFO_initstruct LPFO_initstr;
    //=====初始化滤波器1=====//
    LPFO_initstr.mode        = LPFO_MODE_D;
    LPFO_initstr.K           = 0.2;
    LPFO_initstr.Kw          = 0.2;
    LPFO_initstr.Threshold_A = 8;
    LPFO_initstr.Threshold_T = 30;
    LPFOfilter_init(&ft1,&LPFO_initstr);  //目前效果最好
    LPFOfilter_init(&ft2,&LPFO_initstr);
    LPFOfilter_init(&ft3,&LPFO_initstr);
    LPFOfilter_init(&ft4,&LPFO_initstr);
    LPFOfilter_init(&ft5,&LPFO_initstr);
    LPFOfilter_init(&ft6,&LPFO_initstr);
}

