#include "myuart.h"

//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
uint16_t USART_RX_STA=0;       //����״̬���
uint8_t USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�
uint8_t aRxBuffer[RXBUFFERSIZE];//HAL��ʹ�õĴ��ڽ��ջ���

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  //����ң��
{
    if(huart->Instance==USART2)//����Ǵ���1
    {
        if((USART_RX_STA&0x8000)==0)//����δ���
        {
            if(USART_RX_STA&0x4000)//���յ���0x0d
            {
                if(aRxBuffer[0]!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
                else USART_RX_STA|=0x8000;	//���������
            }
            else //��û�յ�0X0D
            {
                if(aRxBuffer[0]==0x0d)USART_RX_STA|=0x4000;
                else
                {
                    USART_RX_BUF[USART_RX_STA&0X3FFF]=aRxBuffer[0] ;
                    USART_RX_STA++;
                    if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����
                }
            }
        }
    }
}
u8 len = 0;
void uart_fun(void)
{
    if(USART_RX_STA&0x8000)
    {					   
        len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
        //printf("\r\n�����͵���ϢΪ:\r\n");
        HAL_UART_Transmit(&huart2,(uint8_t*)USART_RX_BUF,len,1000);	//���ͽ��յ�������
        while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)!=SET);		//�ȴ����ͽ���
        //printf("\r\n\r\n");//���뻻��
        USART_RX_STA=0;
    }
}
