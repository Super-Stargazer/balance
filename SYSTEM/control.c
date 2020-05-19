#include "control.h"
#include "data_deal.h"
/**************************************************************************
*�������ܣ��ⲿ�жϣ����Ƶ�����أ�ģʽ�л�
*��ڲ�������������
*����  ֵ����
**************************************************************************/
u8 MotorStart = 0;  //0���������
u8 ReportStart=0;  //1:�ϱ�����
u8 Car_Mode = 0;  //0����ѭ��
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    delay_ms(8);
    if(GPIO_Pin == GPIO_PIN_4)  //����KEY_UP,������أ��̵Ʊ�ʾ
    {
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_5);
        MotorStart = ~MotorStart;
        if(!MotorStart)
        {
            MOSTOP(MOTOR1_IO1,MOTOR1_IO1_PIN,MOTOR1_IO2,MOTOR1_IO2_PIN);
            MOSTOP(MOTOR2_IO1,MOTOR2_IO1_PIN,MOTOR2_IO2,MOTOR2_IO2_PIN);
            HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);

        }
        if(MotorStart)
        {
            HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
        }
    }
    if(GPIO_Pin == GPIO_PIN_0)
    {
        ReportStart = ~ReportStart;
    }
}
/**************************************************************************
*�������ܣ���ʱ���жϣ���ʱ��ȡ����
*��ڲ�����
*����  ֵ��
**************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM1)  //��ʱ����PID
    {
        //Get_Motor_Para(&rMotor,100);
        //Get_Motor_Para(&lMotor,100);
        //V = cal_CarVelocity();
    }  
}
/**************************************************************************
*�������ܣ��Զ��嶨ʱ����ȫ�����ƺ���
*��ڲ�������
*����  ֵ����
**************************************************************************/
//�ϱ�������ת
float tmp1 = 0;
float tmp2 = 0;
Timer timer1,timer2,timer3;
void timer1_callback(void)  //����ר��
{
    //=====����=====//
    get_imu_data();

    get_D_encoder(&rMotor);
    get_D_encoder(&lMotor);
    //=====΢��=====//
    ft3.now_data = pose.gyrox;
    LowpassFirstOrderFilter(&ft3);
    BalaPID.diff = ft3.last_data;
    
    //=====�˲���PID=====//
    ft4.now_data = pose.pitch;
    LowpassFirstOrderFilter(&ft4);
    PID_Calculate(&BalaPID,BALA_TARGET,ft4.last_data);  //ֱ����
    tmp1 = 100*pose.pitch;  //�ϱ�������
    
    ft1.now_data = rMotor.position_delta;
    LowpassFirstOrderFilter(&ft1);
    ft2.now_data = lMotor.position_delta;
    LowpassFirstOrderFilter(&ft2);
    PID_Calculate(&VeloPID,VELO_TARGET,(ft1.last_data-ft2.last_data));  //�ٶȻ� 
    tmp2 = 100*(ft1.last_data-ft2.last_data);  //�ϱ�������

    //PID_Calculate(&TurnPID,TURN_TARGET,pose.yaw);  //�ǶȻ�
    
    //=====���=====//
    lMotor.PWM = BalaPID.out + VeloPID.out + TurnPID.out;
    rMotor.PWM = BalaPID.out + VeloPID.out - TurnPID.out;
    if(MotorStart)        Set_Pwm(lMotor.PWM,rMotor.PWM);
    
    //uart_fun();  //����2�������գ�ң�ؿ���
    //if(Car_Mode){}  //����ѭ������
}

void timer2_callback(void)  //��������ר��
{
    //=====�ϱ�����=====//
    if(ReportStart)
    {
        //usart1_report_imu(pose.aacx,pose.aacy,pose.aacz, pose.gyrox,pose.gyroy,pose.gyroz,\
                         (int)(pose.roll*100),(int)(pose.pitch*100),(int)(pose.yaw*10));
        ANO_Send_Data();
    }
}

void timer3_callback(void)  //����ר��
{
    
}
/**************************************************************************
*�������ܣ���֤С����ȫ
*��ڲ�������
*����  ֵ����
**************************************************************************/
void security(void)
{
    static uint8_t security_flag = 1;
    if(pose.pitch<=-60 || pose.pitch>=40)  //��ͣ
    {
        security_flag = 0;  //��ȫ��־��0
        HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
    }
    else if(pose.pitch>-5 && pose.pitch<5 && !security_flag)  //��ͣ��ָ�
    {
        security_flag = 1;  //��ȫ��־��1
        HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
    }
}
/**************************************************************************
*�������ܣ�����PWM�͵��ת��
*��ڲ����������PWM
*����  ֵ����
**************************************************************************/
void Set_Pwm(int moto1_PWM,int moto2_PWM)
{
    Output_Limit(lMotor.PWM,PWM_U,-PWM_U);
    Output_Limit(rMotor.PWM,PWM_U,-PWM_U);
    if(moto1_PWM<0) MOFORW(MOTOR1_IO1,MOTOR1_IO1_PIN,MOTOR1_IO2,MOTOR1_IO2_PIN);
    else 	        MOBACK(MOTOR1_IO1,MOTOR1_IO1_PIN,MOTOR1_IO2,MOTOR1_IO2_PIN);
    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,ABS(moto1_PWM));

    if(moto2_PWM<0)	MOFORW(MOTOR2_IO2,MOTOR2_IO2_PIN,MOTOR2_IO1,MOTOR2_IO1_PIN);  //�������������һ���෴
    else            MOBACK(MOTOR2_IO2,MOTOR2_IO2_PIN,MOTOR2_IO1,MOTOR2_IO1_PIN);
    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,ABS(moto2_PWM));
}


