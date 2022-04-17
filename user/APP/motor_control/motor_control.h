#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"
#define CONTROL_MODE_SW			1
//电机编码值转化成角度值    2Π/8191
#define Ecd_to_Rad		0.000798488132787405781442588526584f
typedef enum
{
	RC_CTRL,		//遥控控制
	KEY_CTRL		//键盘控制
}Control_Mode_e;


//内联函数代替宏
//去除死区
inline void i_dead_zone_del(const int16_t *input, int16_t *output, int8_t deal)
{
	if (*input > deal || *input < -deal)
		*output = *input;
	else
		*output = 0;	
}
//电机反馈机械角度规整 0~8191--> -2PI~2PI (rad)
fp32 ecd_angle_format(uint16_t ecd, uint16_t last_ecd, const uint16_t offset_ecd, uint8_t *turn_table_flag);
//陀螺仪角度规整 -PI~PI--> -2PI~2PI (rad)
fp32 gyro_angle_format(const fp32 angle_now, const fp32 angle_last, uint8_t *turn_table_flag);
//计算云台圈数和实际角度
fp32 calc_turn_angle(const fp32 *angle_last, const fp32 *angle_now, uint8_t *turn_circle_num);
//去除死区
//void dead_zone_del(const int16_t *input, int16_t *output, int8_t deal);


#endif


