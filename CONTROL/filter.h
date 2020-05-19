#ifndef __FILTER_H
#define __FILTER_H
#include "zdsys.h"

//һ�׵�ͨ�˲���ģʽ
#define LPFO_MODE_N 0    
#define LPFO_MODE_D 1 //ʹ�ö�̬����

typedef struct {
    u8 mode : 1;      //�Ƿ�ʹ�ö�̬����
    float K;          //�˲�ϵ��
    float last_data;  
    float now_data;   //���»�ȡ������
    
    float Kw;         //�����ȶ�ʱϵ��
    u8 dir_last : 1;  //������������
    u8 dir_now : 1;
    u8 cnt;           //�ȶ����������٣�����ʱ��
    u8 Threshold_A;   //��ֵ1����һ�״����˲������仯�Ƕȴ��ڴ�ֵʱ����������
    u8 Threshold_T;   //��ֵ2����һ�״����˲���������ֵ���ڴ�ֵʱ�������������ǿ�˲�����
}LPFOfilter;  //һ�׵�ͨ�˲���


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
