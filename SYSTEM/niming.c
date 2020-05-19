#include "niming.h"
#include "usart.h"
#include "print_def.h"

#define BYTE0(dwTemp)    ( *( (char *)(dwTemp) + 0) )
#define BYTE1(dwTemp)    ( *( (char *)(dwTemp) + 1) )
#define BYTE2(dwTemp)    ( *( (char *)(dwTemp) + 2) )
#define BYTE3(dwTemp)    ( *( (char *)(dwTemp) + 3) )

//单个数据信息结构体
typedef struct{
    const void* data;
    uint8_t kind;
}data_mess_TypeDef;

//数据集结构体
typedef struct{
    data_mess_TypeDef datalist[MAX_DATA];
    uint8_t DataNum;  //数据个数
    uint8_t FrameHead;//帧头
    uint8_t FrameFun; //功能字
    uint8_t FrameLen; //长度（字节数）
    uint8_t FrameSum; //校验和
}data_TypeDef;


static data_TypeDef dataset;
static uint8_t data_to_send[MAX_BYTE];

//初始化数据集结构体
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

//注册待发送数据
uint8_t ANO_reg(const void* data, uint8_t datakind)
{
    if(data==NULL) return 1;
    if(dataset.DataNum>=MAX_DATA) return 2;
    
    uint8_t i;
    data_mess_TypeDef* pd = NULL;//循环（指针）变量
    for(pd=dataset.datalist, i=0; (pd->data!=NULL)&&(i<=MAX_DATA); pd++, i++)  
    {//查重覆盖(暂时这样用两个循环变量，减少不必要循环次数，防止死循环，以及指针越界)
        if(data == pd->data)
        {
            pd->kind = datakind;
            return 3;
        }
    }
    
    dataset.datalist[dataset.DataNum].data = data;  //数据个数所指位置比数据在数组中储存位置多1
    dataset.datalist[dataset.DataNum].kind = datakind;
    dataset.DataNum++;  //加完后，实时指示数据个数
    return 0;
}

//打包装填待发送数据
static void package_data()
{   
    //装填帧头
    data_to_send[0] = dataset.FrameHead;
    //装填功能字
    data_to_send[1] = dataset.FrameFun;
    //拆分，装填数据，累计LEN
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
    //装填数据长度
    dataset.FrameLen = cntLEN;
    data_to_send[2] = dataset.FrameLen;
    //计算，装填校验和
    dataset.FrameSum = 0;
    for(i=0; i<3+dataset.FrameLen; i++)
        dataset.FrameSum += data_to_send[i];
    data_to_send[3+dataset.FrameLen] = dataset.FrameSum;
}

//发送数据
void ANO_Send_Data()
{
	uint8_t sendCount;
    package_data();  //装填最新数据

	for( sendCount = 0 ; sendCount < 4+dataset.FrameLen; sendCount++) 
    {
        while((USART1->SR&0X40)==0);  
        USART1->DR = data_to_send[sendCount]; 
    }
}

//--------------------------------------ZDYZ--------------------------------------//
//串口1发送1个字符
void usart1_send_char(uint8_t c)
{
    while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)==RESET);
    USART1->DR = c; 
}

//fun:功能字. 0XA0~0XAF
//data:数据缓存区,最多28字节!!
//len:data区有效数据个数
void usart1_niming_report(uint8_t fun,uint8_t*data,uint8_t len)
{
    uint8_t send_buf[32];
    uint8_t i;
    if(len>28)return;	//最多28字节数据
    send_buf[len+3]=0;	//校验数置零
    send_buf[0]=0X88;	//帧头
    send_buf[1]=fun;	//功能字
    send_buf[2]=len;	//数据长度
    for(i=0; i<len; i++)send_buf[3+i]=data[i];			//复制数据
    for(i=0; i<len+3; i++)send_buf[len+3]+=send_buf[i];	//计算校验和
    for(i=0; i<len+4; i++)usart1_send_char(send_buf[i]);	//发送数据到串口1
}

//通过串口1上报结算后的姿态数据给电脑
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
//roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
//pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
//yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
{
    uint8_t tbuf[28];
    uint8_t i;
    for(i=0; i<28; i++)tbuf[i]=0; //清0
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
    usart1_niming_report(0XAF,tbuf,28);//飞控显示帧,0XAF
}
