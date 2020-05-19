#ifndef __PID_H
#define __PID_H
#include "stm32f1xx.h"

//PID��ط��ų���
enum PID_PARAM {
    NOW 	= 0,
    LAST 	= 1,
    LLAST	= 2,

    PID_ERROR = 0,
    PID_OK,
    PID_POSITION,
    PID_DELTA,

};

//PID���ƽṹ��
typedef  struct {
    uint8_t mode;  //ģʽ

    float Kp, Ki, Kd;  //ϵ��

    float max_out, min_out, max_iout;  //�޷�

    float max_err, deadband;  //��������

    float Pout, Iout, Dout, out;  //�����

    float ref, set;  //����ֵ,�趨ֵ
    float err[3];  //���ֵ
    float diff;  //��������΢��ֵ��ֱ������λ��ʽPID��Ϊ0����

} PID_TypeDef;

//PID��ʼ���ṹ��
typedef struct {
    uint8_t mode;

    float Kp, Ki, Kd, set;

    float max_out, min_out, max_iout;

    float max_err, deadband;

} PID_InitTypeDef;

uint8_t PID_Init(PID_TypeDef *PID, PID_InitTypeDef *PID_Init);  //PID�ṹ���ʼ������
void PID_Calculate(PID_TypeDef *PID, float set, float ref);  //PID������㺯��

//--------------------------------------USER--------------------------------------//

extern PID_TypeDef BalaPID, VeloPID, TurnPID;
extern int BALA_TARGET;  //������Ŀ��ֵ
extern int TURN_TARGET;  //���ٶ�Ŀ��ֵ
extern int VELO_TARGET;  //�ٶȻ�Ŀ��ֵ

void MY_PID_Init(void);

#endif
