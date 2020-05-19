#ifndef _KEY_H
#define _KEY_H
#include "zdsys.h"

//����ķ�ʽ��ͨ��λ��������ʽ��ȡIO
//#define KEY0        PEin(4) 	//KEY0����PE4
//#define KEY1        PEin(3) 	//KEY1����PE3
//#define KEY2        PEin(2)	//KEY2����PE2
//#define WK_UP       PAin(0) 	//WKUP����PA0

//����ķ�ʽ��ͨ��ֱ�Ӳ���HAL�⺯����ʽ��ȡIO
#define KEY0        HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)  //KEY0����PE4
#define KEY1        HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)  //KEY1����PE3
#define KEY2        HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2) 	//KEY2����PE2
#define WK_UP       HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)  //WKUP����PA0

#define KEY0_PRES 	1
#define KEY1_PRES	2
#define KEY2_PRES	3
#define WKUP_PRES   4

void KEY_Init(void);
u8 KEY_Scan(u8 mode);

#endif