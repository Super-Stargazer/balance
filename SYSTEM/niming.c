#include "niming.h"
#include "usart.h"
#include "print_def.h"

#define BYTE0(dwTemp)    ( *( (char *)(dwTemp) + 0) )
#define BYTE1(dwTemp)    ( *( (char *)(dwTemp) + 1) )
#define BYTE2(dwTemp)    ( *( (char *)(dwTemp) + 2) )
#define BYTE3(dwTemp)    ( *( (char *)(dwTemp) + 3) )

//����������Ϣ�ṹ��
typedef struct{
    const void* data;
    uint8_t kind;
}data_mess_TypeDef;

//���ݼ��ṹ��
typedef struct{
    data_mess_TypeDef datalist[MAX_DATA];
    uint8_t DataNum;  //���ݸ���
    uint8_t FrameHead;//֡ͷ
    uint8_t FrameFun; //������
    uint8_t FrameLen; //���ȣ��ֽ�����
    uint8_t FrameSum; //У���
}data_TypeDef;


static data_TypeDef dataset;
static uint8_t data_to_send[MAX_BYTE];

//��ʼ�����ݼ��ṹ��
void ANO_init()
{
    int i;
    dataset.DataNum = 0;
    dataset.FrameHead = 0x88;
    dataset.FrameFun = 0xAA;
    dataset.FrameLen = 0;
    dataset.FrameSum = 0;
    
    for(i=0; i<MAX_DATA; i++)
    {
        dataset.datalist[i].data = NULL;
        dataset.datalist[i].kind = 0;
    }
}

//ע�����������
uint8_t ANO_reg(const void* data, uint8_t datakind)
{
    if(data==NULL) return 1;
    if(dataset.DataNum>=MAX_DATA) return 2;
    
    uint8_t i;
    data_mess_TypeDef* pd = NULL;//ѭ����ָ�룩����
    for(pd=dataset.datalist, i=0; (pd->data!=NULL)&&(i<=MAX_DATA); pd++, i++)  
    {//���ظ���(��ʱ����������ѭ�����������ٲ���Ҫѭ����������ֹ��ѭ�����Լ�ָ��Խ��)
        if(data == pd->data)
        {
            pd->kind = datakind;
            return 3;
        }
    }
    
    dataset.datalist[dataset.DataNum].data = data;  //���ݸ�����ָλ�ñ������������д���λ�ö�1
    dataset.datalist[dataset.DataNum].kind = datakind;
    dataset.DataNum++;  //�����ʵʱָʾ���ݸ���
    return 0;
}

//���װ�����������
static void package_data()
{   
    //װ��֡ͷ
    data_to_send[0] = dataset.FrameHead;
    //װ�����
    data_to_send[1] = dataset.FrameFun;
    //��֣�װ�����ݣ��ۼ�LEN
    uint8_t i,cntLEN;
    for(i=0,cntLEN=0; i<dataset.DataNum; i++)
    {
        switch(dataset.datalist[i].kind)
        {
            case 32:
                data_to_send[3+cntLEN++] = BYTE3(dataset.datalist[i].data);
                data_to_send[3+cntLEN++] = BYTE2(dataset.datalist[i].data);
            case 16:
                data_to_send[3+cntLEN++] = BYTE1(dataset.datalist[i].data);
            case  8:
                data_to_send[3+cntLEN++] = BYTE0(dataset.datalist[i].data);
        }
    }
    //װ�����ݳ���
    dataset.FrameLen = cntLEN;
    data_to_send[2] = dataset.FrameLen;
    //���㣬װ��У���
    dataset.FrameSum = 0;
    for(i=0; i<3+dataset.FrameLen; i++)
        dataset.FrameSum += data_to_send[i];
    data_to_send[3+dataset.FrameLen] = dataset.FrameSum;
}

//��������
void ANO_Send_Data()
{
	uint8_t sendCount;
    package_data();  //װ����������

	for( sendCount = 0 ; sendCount < 4+dataset.FrameLen; sendCount++) 
    {
        while((USART1->SR&0X40)==0);  
        USART1->DR = data_to_send[sendCount]; 
    }
}

//--------------------------------------ZDYZ--------------------------------------//
//����1����1���ַ�
void usart1_send_char(uint8_t c)
{
    while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)==RESET);
    USART1->DR = c; 
}

//fun:������. 0XA0~0XAF
//data:���ݻ�����,���28�ֽ�!!
//len:data����Ч���ݸ���
void usart1_niming_report(uint8_t fun,uint8_t*data,uint8_t len)
{
    uint8_t send_buf[32];
    uint8_t i;
    if(len>28)return;	//���28�ֽ�����
    send_buf[len+3]=0;	//У��������
    send_buf[0]=0X88;	//֡ͷ
    send_buf[1]=fun;	//������
    send_buf[2]=len;	//���ݳ���
    for(i=0; i<len; i++)send_buf[3+i]=data[i];			//��������
    for(i=0; i<len+3; i++)send_buf[len+3]+=send_buf[i];	//����У���
    for(i=0; i<len+4; i++)usart1_send_char(send_buf[i]);	//�������ݵ�����1
}

//ͨ������1�ϱ���������̬���ݸ�����
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
//roll:�����.��λ0.01�ȡ� -18000 -> 18000 ��Ӧ -180.00  ->  180.00��
//pitch:������.��λ 0.01�ȡ�-9000 - 9000 ��Ӧ -90.00 -> 90.00 ��
//yaw:�����.��λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
{
    uint8_t tbuf[28];
    uint8_t i;
    for(i=0; i<28; i++)tbuf[i]=0; //��0
    tbuf[0]=(aacx>>8)&0XFF;
    tbuf[1]=aacx&0XFF;
    tbuf[2]=(aacy>>8)&0XFF;
    tbuf[3]=aacy&0XFF;
    tbuf[4]=(aacz>>8)&0XFF;
    tbuf[5]=aacz&0XFF;
    tbuf[6]=(gyrox>>8)&0XFF;
    tbuf[7]=gyrox&0XFF;
    tbuf[8]=(gyroy>>8)&0XFF;
    tbuf[9]=gyroy&0XFF;
    tbuf[10]=(gyroz>>8)&0XFF;
    tbuf[11]=gyroz&0XFF;
    tbuf[18]=(roll>>8)&0XFF;
    tbuf[19]=roll&0XFF;
    tbuf[20]=(pitch>>8)&0XFF;
    tbuf[21]=pitch&0XFF;
    tbuf[22]=(yaw>>8)&0XFF;
    tbuf[23]=yaw&0XFF;
    usart1_niming_report(0XAF,tbuf,28);//�ɿ���ʾ֡,0XAF
}
