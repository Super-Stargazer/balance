#if 1  //�Ƿ����ô��ڽ����ж�

#ifndef __MYUART_H
#define __MYUART_H

#include "zdsys.h"

#define USART_REC_LEN  			200  		//�����������ֽ��� 200
#define EN_USART1_RX 			1			//ʹ�ܣ�1��/��ֹ��0������1����
extern u8  USART_RX_BUF[USART_REC_LEN]; 	//���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з�
extern u16 USART_RX_STA;         			//����״̬���

#define RXBUFFERSIZE   1 					//�����С
extern u8 aRxBuffer[RXBUFFERSIZE];			//HAL��USART����Buffer

extern u8 len;                              //���ν��յ��ֽ���
void uart_fun(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif

#endif
