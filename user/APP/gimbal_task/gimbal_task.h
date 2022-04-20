#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "main.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "RC_DT7.h"
#include "miniPC.h"
#include "user_lib.h"
#include "motor_control.h"

//�����ʼ�� ����һ��ʱ��
#define GIMBAL_TASK_INIT_TIME 201
//��̨����������
#define GIMBAL_CONTROL_TIME_MS 1

/******�����ػ�е�Ƕ� �������ʱ��Ҫdebug�޸ģ�����******************/
#define YAW_ECD_MAX		2709		//��̨��е�Ƕ���λ
#define YAW_ECD_MIN		6793
#define YAW_ECD_DEL		655			//��̨����ʱ����Ļ�е�Ƕ�
#define YAW_ECD_TURN	4000		//��̨����ʱ�ж��������̻�е�Ƕ� С�ڴ�ֵΪ������ ���ڴ�ֵΪ������ ���ڻ���ʱ ����״̬����
#define PITCH_ECD_MAX	3811
#define PITCH_ECD_MIN	2036
#define PITCH_ECD_DEL	2696
#define PITCH_ECD_TURN	6000
/*******************��ػ�е�Ƕ�_END**********************************/

//��̨ģʽѡ�� ��߿���
#define GIMBAL_MODE_SW		0
//���ҡ�˿���
#define YAW_CHANNEL			2
#define PITCH_CHANNEL		3
//ң����������������Ϊң�������ڲ��죬ҡ�����м䣬��ֵ��һ��Ϊ��
#define GIMBAL_RC_DEADLINE	10
//yaw��pitch�Ƕ���ң�����������
#define YAW_RC_SEN		0.000003f
#define PITCH_RC_SEN	-0.000003f
//����������
#define YAW_MOUSE_SEN	0.000001f
#define PITCH_MOUSE_SEN	-0.000001f
//���������������
#define YAW_AUTO_SEN    0.0055f//-0.0055f
#define PITCH_AUTO_SEN  0.044f//-0.07f //0.005
//��̨�����ٶ�
#define GIMBAL_RETURN_PITCH	0.0008f
#define GIMBAL_RETURN_YAW	0.0008f
//�˲�ϵ��
#define GIMBAL_YAW_ACCEL_NUM	0.3333333333f//0.1666666667f
#define GIMBAL_PITCH_ACCEL_NUM	0.3333333333f
//GM6020���CAN���͵�ѹ
#define MAX_GM6020_CAN_VOLTAGE	30000.0f

/***********************PID***************************/
/*PID����ģ��
{
	KP,
	KI,
	KD,
	OUT_MAX,
	IOUT_MAX
}*/
//yaw   ң�� ��еģʽ �ٶȻ�(�ڻ�)
#define YAW_ECD_SPEED_RC_PID_Init	1500.0f, 0.0f, 0.5f, MAX_GM6020_CAN_VOLTAGE, 5000.0f
//yaw   ң�� ��еģʽ �ǶȻ�(�⻷)
#define YAW_ECD_ANGLE_RC_PID_Init	178.0f, 0.0f, 0.0f, 10.0f, 0.0f
//yaw   ң�� ������ģʽ �ٶȻ�(�ڻ�)
#define YAW_GYRO_SPEED_RC_PID_Init	1500.0f, 0.0f, 0.5f, MAX_GM6020_CAN_VOLTAGE, 5000.0f
//yaw   ң�� ������ģʽ �ǶȻ�(�⻷)
#define YAW_GYRO_ANGLE_RC_PID_Init	120.0f, 0.0f, 0.5f, 10.0f, 0.0f
//pitch ң�� ��еģʽ �ٶȻ�(�ڻ�)
#define PITCH_ECD_SPEED_RC_PID_Init	1000.0f, 0.0f, 2.0f, MAX_GM6020_CAN_VOLTAGE, 5000.0f
//pitch ң�� ��еģʽ �ǶȻ�(�⻷)
#define PITCH_ECD_ANGLE_RC_PID_Init	150.0f, 0.0f, 0.0f, 10.0f, 0.0f
//pitch ң�� ������ģʽ �ٶȻ�(�ڻ�)
#define PITCH_GYRO_SPEED_RC_PID_Init	1000.0f, 0.0f, 2.0f, MAX_GM6020_CAN_VOLTAGE, 5000.0f
//pitch ң�� ������ģʽ �ǶȻ�(�⻷)
#define PITCH_GYRO_ANGLE_RC_PID_Init	150.0f, 0.0f, 0.0f, 10.0f, 0.0f
/*****************************����Ϊ�ֶ������µ�PID����*****************************/
//yaw ���� ��еģʽ �ٶȻ�(�ڻ�)
#define YAW_ECD_SPEED_AUTO_PID_Init	1500.0f, 0.0f, 0.5f, MAX_GM6020_CAN_VOLTAGE, 5000.0f
//yaw ���� ��еģʽ �ǶȻ�(�⻷)
#define YAW_ECD_ANGLE_AUTO_PID_Init	178.0f, 0.0f, 0.0f, 10.0f, 0.0f
//yaw ���� ������ģʽ �ٶȻ�(�ڻ�)
#define YAW_GYRO_SPEED_AUTO_PID_Init	1500.0f, 0.0f, 0.5f, MAX_GM6020_CAN_VOLTAGE, 5000.0f
//yaw ���� ������ģʽ �ǶȻ�(�⻷)
#define YAW_GYRO_ANGLE_AUTO_PID_Init	120.0f, 0.0f, 0.5f, 10.0f, 0.0f
//pitch ���� ��еģʽ �ٶȻ�(�ڻ�)
#define PITCH_ECD_SPEED_AUTO_PID_Init	1000.0f, 0.0f, 2.0f, MAX_GM6020_CAN_VOLTAGE, 5000.0f
//pitch ���� ��еģʽ �ǶȻ�(�⻷)
#define PITCH_ECD_ANGLE_AUTO_PID_Init	150.0f, 0.0f, 0.0f, 10.0f, 0.0f
//pitch ���� ������ģʽ �ٶȻ�(�ڻ�)
#define PITCH_GYRO_SPEED_AUTO_PID_Init	1000.0f, 0.0f, 2.0f, MAX_GM6020_CAN_VOLTAGE, 5000.0f
//pitch ���� ������ģʽ �ǶȻ�(�⻷)
#define PITCH_GYRO_ANGLE_AUTO_PID_Init	150.0f, 0.0f, 0.0f, 10.0f, 0.0f
/*****************************����Ϊ�Զ������µ�PID����*****************************/
//С����ģʽ ң�� �ٶȻ�(�ڻ�)
#define YAW_gyromode_SPEED_RC_PID_Init	1000.0f, 0.0f, 2.0f, MAX_GM6020_CAN_VOLTAGE, 5000.0f 
//������ģʽ ң�� �ǶȻ�(�⻷)
#define YAW_gyromode_ANGLE_RC_PID_Init	150.0f, 0.0f, 0.0f, 10.0f, 0.0f
//С����ģʽ ���� �ٶȻ�(�ڻ�)
#define YAW_gyromode_SPEED_AUTO_PID_Init	1000.0f, 0.0f, 2.0f, MAX_GM6020_CAN_VOLTAGE, 5000.0f 
//������ģʽ ���� �ǶȻ�(�⻷)
#define YAW_gyromode_ANGLE_AUTO_PID_Init	150.0f, 0.0f, 0.0f, 10.0f, 0.0f
/*****************************����Ϊ������ģʽ��YAW���PID����*****************************/
/**********************PID_END************************/
typedef enum
{
	GIMBAL_RELAX,				//��̨����
	GIMBAL_GYRO,				//�����ǽǶȿ���
	GIMBAL_ENCONDE,				//�������������
	GIMBAL_CALI					//��̨У׼
}Gimbal_Mode_e;
typedef enum
{
	GIMBAL_NORMAL,				//��̨��ͨ
	GIMBAL_NORMAL_GR,			//����С��������̨��ͨ
	GIMBAL_AUTO,				//��̨����
	GIMBAL_AUTO_GR,				//����С��������̨����
	GIMBAL_CENTER,				//��̨����
	GIMBAL_TURN,				//��̨��ͷ
	GIMBAL_STOP					//��ֹ̨ͣ
}Gimbal_Behavior_e;
typedef struct
{
	const motor_measure_t *gimbal_motor_measure;	//���������Ϣ
	first_order_filter_type_t gimbal_motor_first_OF;	//���һ�׵�ͨ�˲�
	
	//�ɵ�������Ƕ�
	fp32 relative_angle;
	fp32 relative_angle_last;
	fp32 relative_angle_set;
	//�����Ƿ����Ƕ� rad
	fp32 absolute_angle;			
	fp32 absolute_angle_set;
	fp32 absolute_angle_last;
	
	fp32 speed;				//��ǰ���ٶ�rad/s
	fp32 speed_set;			//�趨���ٶ�

	int16_t ecd_last;		//��һ�ε��������е�Ƕ�ԭʼ����
	fp32 gyro_last;			//��һ��������ԭʼ����
	fp32 ecd_now;			//��ǰת����ĽǶ� -2PI~2PI rad
	fp32 gyro_now;			//��ǰת����ĽǶ� -2PI~2PI rad
	
	fp32 voltage_set;
	int16_t voltage_give_last;
	int16_t voltage_give;	//���͵ĵ�ѹֵ
	
	uint8_t turn_table_flag;//�Ƕ������� 1Ϊ�� 0Ϊ��
} Gimbal_Motor_s;
typedef struct
{
	const RC_ctrl_s *gimbal_RC;				//��̨ʹ�õ�ң����ָ��
	Auto_Gimbal_Ctrl_t *gimbal_AUTO_ctrl;	//����Ƕ�����
	
	Control_Mode_e ctrl_mode;				//����ģʽ
	Gimbal_Mode_e gimbal_mode;				//��̨ģʽ
	Gimbal_Behavior_e gimbal_behavior;		//��̨��Ϊ
	Gimbal_Mode_e gimbal_mode_last;			//��̨��һ��ģʽ
	Gimbal_Behavior_e gimbal_behavior_last;		//��̨��Ϊ
	
	Gimbal_Motor_s pitch_motor;				//��̨pitch���
	Gimbal_Motor_s yaw_motor;				//��̨yaw���
	
	int16_t turn_circle_num;
	uint8_t turn_mid_flag;
}Gimbal_Control_s;


extern const Gimbal_Motor_s *get_gimbal_yaw_motor_point(void);
extern const Gimbal_Motor_s *get_gimbal_pitch_motor_point(void);
extern fp32 get_gimbal_relative_angle(void);
extern void GIMBAL_task(void *pvParameters);

#endif

