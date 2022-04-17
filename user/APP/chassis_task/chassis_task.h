#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "main.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "RC_DT7.h"
#include "user_lib.h"
#include "Gimbal_Task.h"
#include "INS_Task.h"
#include "motor_control.h"

//����ģʽѡ�� �ұ߿���S0
#define CHASSIS_MODE_SW			0
#if RC_MODE == USA	//������
#define CHASSIS_X_CHANNEL 1		//ǰ���ң����ͨ������
#define CHASSIS_Y_CHANNEL 0		//���ҵ�ң����ͨ������
#define CHASSIS_WZ_CHANNEL 2
//#define CHASSIS_WZ_CHANNEL 2	//������ģʽ�£�����ͨ��ң����������ת
#elif RC_MODE == JP	//�ձ���
#define CHASSIS_X_CHANNEL 3
#define CHASSIS_Y_CHANNEL 2
#define CHASSIS_WZ_CHANNEL 0
#elif RC_MODE == CN	//�й���
#define CHASSIS_X_CHANNEL 1//3���ձ���
#define CHASSIS_Y_CHANNEL 0//2���ձ���
#define CHASSIS_WZ_CHANNEL 2//0���ձ���
#endif	/*RC_MODE*/

//ң����������������Ϊң�������ڲ��죬ҡ�����м䣬��ֵ��һ��Ϊ��
#define CHASSIS_RC_DEADLINE 10
//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357
//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002
//���ٶȸ���ʹ��		�����������Ƶ��
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//ң��ת������
#define RC_CHASSIS_VX_SEN	0.006f
#define RC_CHASSIS_VY_SEN	0.005f
#define RC_CHASSIS_VW_SEN	0.0001f
//����ת������
#define KEY_CHASSIS_VX_SEN	0.006f
#define KEY_CHASSIS_VY_SEN	0.005f
//��ת�Ƕ��������
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//����ң���˲�ϵ��
#define CHASSIS_X_ACCEL_NUM 0.1666666667f
#define CHASSIS_Y_ACCEL_NUM 0.3333333333f
//��ת�������
#define CHASSIS_VW_SEN		0.000415809748903494517209f
//����3508���can���͵���ֵ
#define MAX_M3508_CAN_CURRENT 16000.0f
//m3508ת���ɵ����ٶ�(m/s)�ı������������� ����Ϊ���ܻ������Ҫ��������    2��/60 * r 
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f//0.000798488132787405781442588526584f//0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
//����ٶ�ת���������ٶ�
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VW 0.25f
//���̵������ٶ�
#define MAX_WHEEL_SPEED 4.0f
//�����˶��������ǰ���ٶ�
#define MAX_CHASSIS_SPEED_X 3.0f
//�����˶��������ƽ���ٶ�
#define MAX_CHASSIS_SPEED_Y 2.9f
//���̳���
#define ROBOT_LENGTH_A 0.5f
#define ROBOT_LENGTH_B 0.4f

//����3508���can���͵���ֵ
#define MAX_M3508_CAN_CURRENT 16000.0f
//PID����
//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_M3508_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
//���̽ǶȻ�PID
#define CHASSIS_ANGLE_PID_KP 10.0f  //40
#define CHASSIS_ANGLE_PID_KI 0.0f
#define CHASSIS_ANGLE_PID_KD 0.0f
#define CHASSIS_ANGLE_PID_MAX_OUT 6.0f
#define CHASSIS_ANGLE_PID_MAX_IOUT 0.2f

typedef enum
{
	CHASSIS_RELAX,				//��������
	CHASSIS_STOP,				//����ֹͣ
	CHASSIS_FOLLOW_YAW,			//���̸�����̨ģʽ ǰ����������̨����
	CHASSIS_NO_FOLLOW,			//���̲�������̨ģʽ ǰ�������ɵ��̾���
	CHASSIS_GYRO,				//С����ģʽ
	CHASSIS_CALI				//У׼ģʽ
}Chassis_Mode_e;//����ģʽ
typedef struct
{
	const motor_measure_t *chassis_motor_measure;	//���������Ϣ
	PidTypeDef motor_speed_pid;		//����ٶȻ� �ڻ�
	
	fp32 accel;						//���ٶ�
	fp32 speed;						//��ǰ�ٶ�
	fp32 speed_set;					//�趨�ٶ�
	int16_t last_give_current;
	int16_t give_current;			//���͵ĵ���ֵ
} Chassis_Motor_s;//�������
typedef struct
{
	const RC_ctrl_s *chassis_RC;			//ң������
	const Gimbal_Motor_s *yaw_motor_gimbal;   //����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����
	const INS_Data_s *ins_data;			//��������ؽǶ�
	
	Control_Mode_e ctrl_mode;				//����ģʽ
	Chassis_Mode_e chassis_mode;			//����ģʽ
	Chassis_Mode_e chassis_last_mode;
	
	PidTypeDef chassis_angle_pid;			//����������ת�ǶȻ�
	Chassis_Motor_s chassis_motor[4];		//�����ĸ����
	first_order_filter_type_t chassis_vx_first_OF;	//һ�׵�ͨ�˲�����б�º�������ǰ�������ٶ�
	first_order_filter_type_t chassis_vy_first_OF;	//һ�׵�ͨ�˲�����б�º����������ҷ����ٶ�
	//�����ƶ�����
	fp32 chassis_vx;	//m/s
	fp32 chassis_vx_set;
	fp32 chassis_vy;
	fp32 chassis_vy_set;
//	fp32 chassis_vx_max;
//	fp32 chassis_vx_min;
//	fp32 chassis_vy_max;
//	fp32 chassis_vy_min;
	fp32 chassis_vw;		//������ת���ٶ�rad/s		ra:��ת�Ƕ�rotation angle
	fp32 chassis_vw_set;	//������ת���ٶ��趨ֵ
	fp32 chassis_relative_angle;	//��������̨����ԽǶ�rad/s
//	fp32 chassis_relative_angle_set;//���������̨���ƽǶ�
	
	fp32 chassis_last_absolute_yaw;	//��һ�νǶ�
	fp32 chassis_absolute_yaw;		//���̾��ԽǶ� �����Ǽ�ȥ��̨yaw�ĽǶ� ���̽Ƕȹ̶�����
	fp32 chassis_absolute_yaw_set;
	fp32 chassis_yaw;				//����Ȧ����ĽǶ�
	fp32 chassis_last_yaw;
	
	//fp32 chassis_absolute_pitch;	//����ģʽ���� ���������ӵ���ģʽ
	
	uint8_t chassis_table_flag;//�Ƕ������� 1Ϊ�� 0Ϊ��
	int16_t chassis_circle_num;//Ȧ��
}Chassis_Control_s;//����������


void chassis_task(void *pvParameters);
const Chassis_Control_s *get_classis_info_point(void);

#endif


