#ifndef __KEY_DETECT_H
#define __KEY_DETECT_H


/** 
  * @brief ע�ᰴ��������
  */
#define Key_Num_Max		10


/** 
  * @brief ��������ģʽ״̬ö��
  */
typedef enum
{
	N_Trig = 0, 																				/*!< 0 �� */		
	L_Trig ,																						/*!< 1 �͵�ƽ���� */			
	H_Trig,																							/*!< 2 �ߵ�ƽ���� */
}Trig_Mode_TypeDef;

/** 
  * @brief ����ģʽ״̬ö��
  */
typedef enum
{
	N_Click = 0,																				/*!< 0 �� */	
	S_Click ,																						/*!< 1 ���� */			
	D_Click,																						/*!< 2 ˫�� */
	L_Press,																						/*!< 3 ���� */
}Key_Mode_TypeDef;


/** 
  * @name   Init_Key_Struct
  *
  * @brief  ��ʼ������
  *
  * @param  Update_Key_CallBack�����°���״̬
  * @param  Debug_CallBack����ӡ����������Ϣ
  *
  * @retval 0���ɹ���
	*					1��Update_Key_CallBack == NULL��
  */
char Init_Key_Struct(void (*Update_Key_CallBack)(void),void (*Debug_CallBack)(unsigned char *debug_mess));

/** 
  * @name   Reg_Key
  *
  * @brief  ���ע�ᰴ����ע����������Ѿ�ע�������ô�ٴ�ע��Ḳ��֮ǰע�������ͬ�İ�����
  *
  * @param  key_s������״̬
  * @param  count����������
  * @param  Trig_Mode_E����������ģʽ
  * @param  Key_Mode_E������ģʽ
  * @param  Key_Click_CallBack�����������ص�
  *
  * @retval 0���ɹ���
	*					1��Key_Click_CallBack == NULL��
	*					2��Key.Reg_Key_Num > Key_Num_Max��
  */
char Reg_Key(unsigned char *key_s,const unsigned short count, 
	Trig_Mode_TypeDef Trig_Mode_E, Key_Mode_TypeDef  Key_Mode_E, void (*Key_Click_CallBack)(void));

/** 
  * @name   Key_Detect
  *
  * @brief  �������
  *
  * @param  ��
  *
  * @retval 0���ɹ���
	*					1��Key.Update_Key_CallBack == NULL��
  */
char Key_Detect(void);

/** 
  * @name   Get_Version_Mess
  *
  * @brief  ��ӡKey_Detect����汾��Ϣ
  *
  * @param  ��
  *
  * @retval ��Key_Detect����汾��Ϣ
  */
char *Get_Version_Mess(void);


#endif


/******************** (C)COPYRIGHT(2018) YaoCheng END OF FILE **********************/
