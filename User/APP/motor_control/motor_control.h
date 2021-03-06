#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"
//控制模式选择 左边开关S1
#define CONTROL_MODE_SW		1
//云台模式选择 右边开关
#define GIMBAL_MODE_SW		0
//底盘模式选择 右边开关S0
#define CHASSIS_MODE_SW		0
//#define CHASSIS_X_CHANNEL		1		//底盘前后的遥控器通道号码
//#define CHASSIS_Y_CHANNEL		0		//左右的遥控器通道号码
#if RC_MODE == USA	//美国手
#define CHASSIS_X_CHANNEL		1		//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL		0		//底盘前后的遥控器通道号码
#define CHASSIS_WZ_CHANNEL		4
#define GIMBAL_YAW_CHANNEL		2
#define GIMBAL_PITCH_CHANNEL	3
#elif RC_MODE == JP	//日本手
#define CHASSIS_X_CHANNEL		3
#define CHASSIS_Y_CHANNEL		2
#define CHASSIS_WZ_CHANNEL		4
#define GIMBAL_YAW_CHANNEL		0
#define GIMBAL_PITCH_CHANNEL	1
#elif RC_MODE == CN	//中国手
#define CHASSIS_X_CHANNEL 1//3是日本手
#define CHASSIS_Y_CHANNEL 0//2是日本手
#define CHASSIS_WZ_CHANNEL 2//0是日本手
#endif	/*RC_MODE*/

//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADLINE	10
//鼠标输入死区
#define MOUSE_DEADLINE 10
//电机编码值转化成角度值    2Π/8191
#define Ecd_to_Rad		0.00076708403213033652507939040001941f
#define CIRCLE	(2*PI)	//一圈等于2PI


typedef enum
{
	RC_CTRL,		//遥控控制
	PC_CTRL			//键鼠控制
}Control_Mode_e;


//内联函数代替宏
//去除死区 在遥控接收处理 不在控制任务内处理
//inline static void i_dead_zone_del(const int16_t input, int16_t *output, int8_t deal)
//{
//	if (input > deal || input < -deal)
//		*output = input;
//	else
//		*output = 0;	
//}
//电机反馈机械角度规整 0~8191--> -2PI~2PI (rad)
fp32 ecd_angle_format(uint16_t ecd, uint16_t last_ecd, const uint16_t offset_ecd, uint8_t *turn_table_flag);
//陀螺仪角度规整 -PI~PI--> -2PI~2PI (rad)
fp32 gyro_angle_format(const fp32 angle_last, const fp32 angle_now, uint8_t *turn_table_flag);
//计算云台圈数和实际角度
fp32 calc_turn_angle(const fp32 angle_last, const fp32 angle_now, int16_t *turn_circle_num);
//去除死区
//void dead_zone_del(const int16_t *input, int16_t *output, int8_t deal);
//初始化初始角度码盘
void angle_table_init(uint16_t ecd, const uint16_t ecd_del, uint8_t *turn_table_flag);

#endif


