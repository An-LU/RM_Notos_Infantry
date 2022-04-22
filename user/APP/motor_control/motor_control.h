#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"
//����ģʽѡ�� ��߿���S1
#define CONTROL_MODE_SW		1
//��̨ģʽѡ�� �ұ߿���
#define GIMBAL_MODE_SW		0
//����ģʽѡ�� �ұ߿���S0
#define CHASSIS_MODE_SW		0
//#define CHASSIS_X_CHANNEL		1		//����ǰ���ң����ͨ������
//#define CHASSIS_Y_CHANNEL		0		//���ҵ�ң����ͨ������
#if RC_MODE == USA	//������
#define CHASSIS_X_CHANNEL		0		//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL		1		//����ǰ���ң����ͨ������
#define CHASSIS_WZ_CHANNEL		4
#define GIMBAL_YAW_CHANNEL		2
#define GIMBAL_PITCH_CHANNEL	3
#elif RC_MODE == JP	//�ձ���
#define CHASSIS_X_CHANNEL		3
#define CHASSIS_Y_CHANNEL		2
#define CHASSIS_WZ_CHANNEL		4
#define GIMBAL_YAW_CHANNEL		0
#define GIMBAL_PITCH_CHANNEL	1
#elif RC_MODE == CN	//�й���
#define CHASSIS_X_CHANNEL 1//3���ձ���
#define CHASSIS_Y_CHANNEL 0//2���ձ���
#define CHASSIS_WZ_CHANNEL 2//0���ձ���
#endif	/*RC_MODE*/
//�������ֵת���ɽǶ�ֵ    2��/8191
#define Ecd_to_Rad		0.00076708403213033652507939040001941f
typedef enum
{
	RC_CTRL,		//ң�ؿ���
	KEY_CTRL		//���̿���
}Control_Mode_e;


//�������������
//ȥ������
inline static void i_dead_zone_del(const int16_t *input, int16_t *output, int8_t deal)
{
	if (*input > deal || *input < -deal)
		*output = *input;
	else
		*output = 0;	
}
//���������е�Ƕȹ��� 0~8191--> -2PI~2PI (rad)
fp32 ecd_angle_format(uint16_t ecd, uint16_t last_ecd, const uint16_t offset_ecd, uint8_t *turn_table_flag);
//�����ǽǶȹ��� -PI~PI--> -2PI~2PI (rad)
fp32 gyro_angle_format(const fp32 angle_last, const fp32 angle_now, uint8_t *turn_table_flag);
//������̨Ȧ����ʵ�ʽǶ�
fp32 calc_turn_angle(const fp32 angle_last, const fp32 angle_now, int16_t *turn_circle_num);
//ȥ������
//void dead_zone_del(const int16_t *input, int16_t *output, int8_t deal);
//��ʼ����ʼ�Ƕ�����
void angle_table_init(uint16_t ecd, const uint16_t ecd_del, uint8_t *turn_table_flag);

#endif


