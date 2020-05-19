#include "control.h"
#include "data_deal.h"
/**************************************************************************
*函数功能：外部中断，控制电机开关，模式切换
*入口参数：触发引脚
*返回  值：无
**************************************************************************/
u8 MotorStart = 0;  //0：锁定电机
u8 ReportStart=0;  //1:上报数据
u8 Car_Mode = 0;  //0：不循迹
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    delay_ms(8);
    if(GPIO_Pin == GPIO_PIN_4)  //按下KEY_UP,电机开关，绿灯标示
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
*函数功能：定时器中断，定时获取数据
*入口参数：
*返回  值：
**************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM1)  //定时启动PID
    {
        //Get_Motor_Para(&rMotor,100);
        //Get_Motor_Para(&lMotor,100);
        //V = cal_CarVelocity();
    }  
}
/**************************************************************************
*函数功能：自定义定时器，全部控制函数
*入口参数：无
*返回  值：无
**************************************************************************/
//上报数据中转
float tmp1 = 0;
float tmp2 = 0;
Timer timer1,timer2,timer3;
void timer1_callback(void)  //控制专用
{
    //=====反馈=====//
    get_imu_data();

    get_D_encoder(&rMotor);
    get_D_encoder(&lMotor);
    //=====微分=====//
    ft3.now_data = pose.gyrox;
    LowpassFirstOrderFilter(&ft3);
    BalaPID.diff = ft3.last_data;
    
    //=====滤波、PID=====//
    ft4.now_data = pose.pitch;
    LowpassFirstOrderFilter(&ft4);
    PID_Calculate(&BalaPID,BALA_TARGET,ft4.last_data);  //直立环
    tmp1 = 100*pose.pitch;  //上报被调量
    
    ft1.now_data = rMotor.position_delta;
    LowpassFirstOrderFilter(&ft1);
    ft2.now_data = lMotor.position_delta;
    LowpassFirstOrderFilter(&ft2);
    PID_Calculate(&VeloPID,VELO_TARGET,(ft1.last_data-ft2.last_data));  //速度环 
    tmp2 = 100*(ft1.last_data-ft2.last_data);  //上报被调量

    //PID_Calculate(&TurnPID,TURN_TARGET,pose.yaw);  //角度环
    
    //=====输出=====//
    lMotor.PWM = BalaPID.out + VeloPID.out + TurnPID.out;
    rMotor.PWM = BalaPID.out + VeloPID.out - TurnPID.out;
    if(MotorStart)        Set_Pwm(lMotor.PWM,rMotor.PWM);
    
    //uart_fun();  //串口2蓝牙接收，遥控控制
    //if(Car_Mode){}  //启动循迹功能
}

void timer2_callback(void)  //发送数据专用
{
    //=====上报数据=====//
    if(ReportStart)
    {
        //usart1_report_imu(pose.aacx,pose.aacy,pose.aacz, pose.gyrox,pose.gyroy,pose.gyroz,\
                         (int)(pose.roll*100),(int)(pose.pitch*100),(int)(pose.yaw*10));
        ANO_Send_Data();
    }
}

void timer3_callback(void)  //测试专用
{
    
}
/**************************************************************************
*函数功能：保证小车安全
*入口参数：无
*返回  值：无
**************************************************************************/
void security(void)
{
    static uint8_t security_flag = 1;
    if(pose.pitch<=-60 || pose.pitch>=40)  //急停
    {
        security_flag = 0;  //安全标志置0
        HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
    }
    else if(pose.pitch>-5 && pose.pitch<5 && !security_flag)  //急停后恢复
    {
        security_flag = 1;  //安全标志置1
        HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
    }
}
/**************************************************************************
*函数功能：设置PWM和电机转向
*入口参数：两电机PWM
*返回  值：无
**************************************************************************/
void Set_Pwm(int moto1_PWM,int moto2_PWM)
{
    Output_Limit(lMotor.PWM,PWM_U,-PWM_U);
    Output_Limit(rMotor.PWM,PWM_U,-PWM_U);
    if(moto1_PWM<0) MOFORW(MOTOR1_IO1,MOTOR1_IO1_PIN,MOTOR1_IO2,MOTOR1_IO2_PIN);
    else 	        MOBACK(MOTOR1_IO1,MOTOR1_IO1_PIN,MOTOR1_IO2,MOTOR1_IO2_PIN);
    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,ABS(moto1_PWM));

    if(moto2_PWM<0)	MOFORW(MOTOR2_IO2,MOTOR2_IO2_PIN,MOTOR2_IO1,MOTOR2_IO1_PIN);  //方向控制线与另一个相反
    else            MOBACK(MOTOR2_IO2,MOTOR2_IO2_PIN,MOTOR2_IO1,MOTOR2_IO1_PIN);
    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,ABS(moto2_PWM));
}


