#include "filter.h"
#include "data_deal.h"

/**
  * @brief  һ�׵�ͨ�˲����ṹ���ʼ������
  * @param  pft      �˲����ṹ��ָ�����
  * @param  pftinit  �˲����ṹ���ʼ���ṹ��ָ�����
  * @retval ��
  */
void LPFOfilter_init(LPFOfilter* pft,LPFO_initstruct* pftinit)
{
    pft->mode = pftinit->mode;
    pft->K    = pftinit->K;
    pft->last_data = pft->now_data = 0;
    if(pftinit->mode)  //��������˲�ϵ���Ķ�̬����
    {
        pft->Kw = pftinit->Kw;
        pft->Threshold_A = pftinit->Threshold_A;
        pft->Threshold_T = pftinit->Threshold_T;
        pft->cnt = 0;
        pft->dir_last = pft->dir_now = 0;
    }
}

/**
  * @brief  һ�׵�ͨ�˲������ܺ���
  * @param  pft  �˲����ṹ��ָ�����
  * @retval ��
  */
void LowpassFirstOrderFilter(LPFOfilter* pft)
{
    if(pft->mode)  //���ʹ�ö�̬����
    {
        if(pft->now_data > pft->last_data)  //�жϱ仯����
            pft->dir_now = 1;
        else
            pft->dir_now = 0;
        
        if(pft->dir_now == pft->dir_last)  //�ж�ǰ�����α仯�����Ƿ����
        {
            pft->cnt++;
            if(ABS(pft->now_data - pft->last_data)>pft->Threshold_A)  //�Ƚ� �仯����
                pft->cnt+=5;
            if(pft->cnt>pft->Threshold_T)  //�Ƚ� ����仯����ʱ��
            {
                pft->K+=0.2;  //���������
                pft->cnt = 0;
            } 
        }
        else
        {
            pft->cnt = 0;
            pft->K = pft->Kw;  //����ƽ��ʱ������������
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
    //=====��ʼ���˲���1=====//
    LPFO_initstr.mode        = LPFO_MODE_D;
    LPFO_initstr.K           = 0.2;
    LPFO_initstr.Kw          = 0.2;
    LPFO_initstr.Threshold_A = 8;
    LPFO_initstr.Threshold_T = 30;
    LPFOfilter_init(&ft1,&LPFO_initstr);  //ĿǰЧ�����
    LPFOfilter_init(&ft2,&LPFO_initstr);
    LPFOfilter_init(&ft3,&LPFO_initstr);
    LPFOfilter_init(&ft4,&LPFO_initstr);
    LPFOfilter_init(&ft5,&LPFO_initstr);
    LPFOfilter_init(&ft6,&LPFO_initstr);
}

