#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"
#define CONTROL_MODE_SW			1
//�������ֵת���ɽǶ�ֵ    2��/8191
#define Ecd_to_Rad		0.000798488132787405781442588526584f
typedef enum
{
	RC_CTRL,		//ң�ؿ���
	KEY_CTRL		//���̿���
}Control_Mode_e;


//�������������
//ȥ������
inline void i_dead_zone_del(const int16_t *input, int16_t *output, int8_t deal)
{
	if (*input > deal || *input < -deal)
		*output = *input;
	else
		*output = 0;	
}
//���������е�Ƕȹ��� 0~8191--> -2PI~2PI (rad)
fp32 ecd_angle_format(uint16_t ecd, uint16_t last_ecd, const uint16_t offset_ecd, uint8_t *turn_table_flag);
//�����ǽǶȹ��� -PI~PI--> -2PI~2PI (rad)
fp32 gyro_angle_format(const fp32 angle_now, const fp32 angle_last, uint8_t *turn_table_flag);
//������̨Ȧ����ʵ�ʽǶ�
fp32 calc_turn_angle(const fp32 *angle_last, const fp32 *angle_now, uint8_t *turn_circle_num);
//ȥ������
//void dead_zone_del(const int16_t *input, int16_t *output, int8_t deal);


#endif


