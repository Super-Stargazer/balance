#ifndef __NIMING_H
#define __NIMING_H
#include "stm32f1xx.h"

/*
基于匿名上位机V2.6

使用流程：
1.主函数中
    初始化：ANO_init();
    注册：ANO_reg();
2.该头文件中
    修改MAX_DATA，MAX_BYTE
    
3.在需要发送数据的地方
    调用：ANO_Send_Data();
    
4.匿名上位机中：
    设置数据格式
    
PS：自定义帧格式：
    0x88  +  0xAA  +  Len  +  Data  +  Sum
    帧头    功能字  数据长度  有效数据 校验和
*/

#define MAX_DATA 9  //注册数据总数，不超过20
#define MAX_BYTE 40  //注册数据的总字节数+4

#define UINT8   8
#define UINT16  16
#define INT16   16
#define INT32   32
#define FP32    32

void ANO_init(void);
uint8_t ANO_reg(const void* data, uint8_t datakind);
void ANO_Send_Data(void);


//发送飞控显示帧
void usart1_report_imu(short aacx,short aacy,short aacz,
                    short gyrox,short gyroy,short gyroz,
                    short roll,short pitch,short yaw);


#endif

