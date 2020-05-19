#include "myuart.h"

//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
uint16_t USART_RX_STA=0;       //接收状态标记
uint8_t USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节
uint8_t aRxBuffer[RXBUFFERSIZE];//HAL库使用的串口接收缓冲

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  //蓝牙遥控
{
    if(huart->Instance==USART2)//如果是串口1
    {
        if((USART_RX_STA&0x8000)==0)//接收未完成
        {
            if(USART_RX_STA&0x4000)//接收到了0x0d
            {
                if(aRxBuffer[0]!=0x0a)USART_RX_STA=0;//接收错误,重新开始
                else USART_RX_STA|=0x8000;	//接收完成了
            }
            else //还没收到0X0D
            {
                if(aRxBuffer[0]==0x0d)USART_RX_STA|=0x4000;
                else
                {
                    USART_RX_BUF[USART_RX_STA&0X3FFF]=aRxBuffer[0] ;
                    USART_RX_STA++;
                    if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收
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
        len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
        //printf("\r\n您发送的消息为:\r\n");
        HAL_UART_Transmit(&huart2,(uint8_t*)USART_RX_BUF,len,1000);	//发送接收到的数据
        while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)!=SET);		//等待发送结束
        //printf("\r\n\r\n");//插入换行
        USART_RX_STA=0;
    }
}
