#ifndef __FILTER_H
#define __FILTER_H
#include "zdsys.h"

//一阶低通滤波器模式
#define LPFO_MODE_N 0    
#define LPFO_MODE_D 1 //使用动态调节

typedef struct {
    u8 mode : 1;      //是否使用动态调节
    float K;          //滤波系数
    float last_data;  
    float now_data;   //最新获取的数据
    
    float Kw;         //数据稳定时系数
    u8 dir_last : 1;  //数据增量方向
    u8 dir_now : 1;
    u8 cnt;           //稳定增长（减少）持续时间
    u8 Threshold_A;   //阈值1用于一阶带参滤波器，变化角度大于此值时，计数增加
    u8 Threshold_T;   //阈值2用于一阶带参滤波器，计数值大于此值时，增大参数，增强滤波跟随
}LPFOfilter;  //一阶低通滤波器


typedef struct{
    u8 mode : 1;
    float K;
    
    float Kw;
    u8 Threshold_A;
    u8 Threshold_T;
}LPFO_initstruct;


void LPFOfilter_init(LPFOfilter* pft,LPFO_initstruct* pftinit);
void LowpassFirstOrderFilter(LPFOfilter* pft);

//--------------------------------------USER--------------------------------------//
extern LPFOfilter ft1,ft2,ft3,ft4,ft5,ft6;
void MY_LPFO_init(void);

#endif
