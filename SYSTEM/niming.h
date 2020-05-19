#ifndef __NIMING_H
#define __NIMING_H
#include "stm32f1xx.h"

/*
����������λ��V2.6

ʹ�����̣�
1.��������
    ��ʼ����ANO_init();
    ע�᣺ANO_reg();
2.��ͷ�ļ���
    �޸�MAX_DATA��MAX_BYTE
    
3.����Ҫ�������ݵĵط�
    ���ã�ANO_Send_Data();
    
4.������λ���У�
    �������ݸ�ʽ
    
PS���Զ���֡��ʽ��
    0x88  +  0xAA  +  Len  +  Data  +  Sum
    ֡ͷ    ������  ���ݳ���  ��Ч���� У���
*/

#define MAX_DATA 9  //ע������������������20
#define MAX_BYTE 40  //ע�����ݵ����ֽ���+4

#define UINT8   8
#define UINT16  16
#define INT16   16
#define INT32   32
#define FP32    32

void ANO_init(void);
uint8_t ANO_reg(const void* data, uint8_t datakind);
void ANO_Send_Data(void);


//���ͷɿ���ʾ֡
void usart1_report_imu(short aacx,short aacy,short aacz,
                    short gyrox,short gyroy,short gyroz,
                    short roll,short pitch,short yaw);


#endif

