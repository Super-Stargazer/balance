#ifndef __MOTOR_H
#define __MOTOR_H

#include "zdsys.h"

#define WHEEL_D         0.065   //����ֱ�� ����λm��
#define ReductionRratio	30 		//������ٱ�
#define	LineNumber		13		//����������
#define Multiplier		2		//��Ƶ��
#define RdEcd (ReductionRratio*LineNumber*Multiplier)	//���תһȦ��������ֵ
#define CNTPRD 36000  //��������������
#define PWM_U 7199
#define PWM_D 0

//�ҵ��
#define MOTOR1_IO1     GPIOE
#define MOTOR1_IO1_PIN GPIO_PIN_0
#define MOTOR1_IO2     GPIOE
#define MOTOR1_IO2_PIN GPIO_PIN_1

//����
#define MOTOR2_IO1     GPIOE
#define MOTOR2_IO1_PIN GPIO_PIN_2
#define MOTOR2_IO2     GPIOE
#define MOTOR2_IO2_PIN GPIO_PIN_3

//���������ƺ�ָ��
#define MOSTOP(MOTOR_IO1,MOTOR_IO1_PIN,MOTOR_IO2,MOTOR_IO2_PIN) \
    do{ \
    HAL_GPIO_WritePin(MOTOR_IO1,MOTOR_IO1_PIN,GPIO_PIN_RESET);\
    HAL_GPIO_WritePin(MOTOR_IO2,MOTOR_IO2_PIN,GPIO_PIN_RESET);\
    }while(0u)

#define MOFORW(MOTOR_IO1,MOTOR_IO1_PIN,MOTOR_IO2,MOTOR_IO2_PIN) \
    do{ \
        HAL_GPIO_WritePin(MOTOR_IO1,MOTOR_IO1_PIN,GPIO_PIN_SET);\
        HAL_GPIO_WritePin(MOTOR_IO2,MOTOR_IO2_PIN,GPIO_PIN_RESET);\
    }while(0u)

#define MOBACK(MOTOR_IO1,MOTOR_IO1_PIN,MOTOR_IO2,MOTOR_IO2_PIN) \
    do{ \
        HAL_GPIO_WritePin(MOTOR_IO1,MOTOR_IO1_PIN,GPIO_PIN_RESET);\
        HAL_GPIO_WritePin(MOTOR_IO2,MOTOR_IO2_PIN,GPIO_PIN_SET);\
    }while(0u)

//���״̬
enum {
    STATIC = -1,  //��ֹ
    ROLLB = 0,  //��ת
    ROLLF = 1,   //��ת
};


//���״̬�ṹ��
typedef struct {
    TIM_HandleTypeDef* phtimx;  //�����������Ӧ��ʱ�����
    uint16_t last_position;  //��¼�������ۼ�������
    uint16_t now_position;
    int16_t position_delta;  //����������

    float angular_delta_all;  //�ܼ���ת�Ƕ� (��λ����)
    float angular_delta_dt;   //���������ڽǶ�����
    float angular_velocity;   //���ƽ��ٶ� (��λ����/s)
    
    int32_t PWM;  //16λ��ʱ��������32λ��������
} Motor_TypeDef;


void Motor_Init(Motor_TypeDef *MOTOR,TIM_HandleTypeDef* phtimx);  //����ṹ���ʼ������
void Get_Motor_Para(Motor_TypeDef *motor, uint16_t f);  //���״̬��ȡ����
void get_D_encoder(Motor_TypeDef *motor);
//--------------------------------------USER--------------------------------------//
extern Motor_TypeDef lMotor, rMotor;
void MY_motor_init(void);


#endif
