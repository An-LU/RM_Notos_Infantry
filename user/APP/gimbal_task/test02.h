#ifndef __TEST02_H__
#define __TEST02_H__

#include "main.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "remote_control.h"
#include "miniPC.h"
#include "user_lib.h"

//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201
//云台控制任务间隔
#define GIMBAL_CONTROL_TIME_MS 1
//底盘模式选择 左边开关
#define GIMBAL_MODE_SW		1
//左边摇杆控制
#define YAW_CHANNEL			2
#define PITCH_CHANNEL		3
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define GIMBAL_RC_DEADLINE	10
//yaw，pitch角度与遥控器输入比例
#define YAW_RC_SEN 0.000001f
#define PITCH_RC_SEN -0.000002f //0.005
//云台机械角度限位
#define YAW_ECD_MAX		8191
#define YAW_ECD_MIN		0
#define YAW_ECD_DEL		0
#define PITCH_ECD_MAX	8191
#define PITCH_ECD_MIN	0
#define PITCH_ECD_DEL	0
#define Half_Ecd_range	4096
//滤波系数
#define GIMBAL_YAW_ACCEL_NUM 0.1666666667f
#define GIMBAL_PITCH_ACCEL_NUM 0.3333333333f
//电机编码值转化成角度值    2Π/8191
#define Ecd_to_Rad		0.000798488132787405781442588526584f
//GM6020最大CAN发送电压
#define MAX_GM6020_CAN_VOLTAGE	30000.0f
//PID
//pitch 速度环 内环
#define PITCH_SPEED_PID_KP			1000.0f   //2500.0f //初始是2000.0f
#define PITCH_SPEED_PID_KI			0.0f  //1.0f
#define PITCH_SPEED_PID_KD			0.0f  //30.0f
#define PITCH_SPEED_PID_MAX_OUT		MAX_GM6020_CAN_VOLTAGE
#define PITCH_SPEED_PID_MAX_IOUT	5000.0f
//yaw 速度环
#define YAW_SPEED_PID_KP			1500.0f  //8000.0f//初始是2200.0f
#define YAW_SPEED_PID_KI			0.0f  //20.0f //初始是20.0f
#define YAW_SPEED_PID_KD			0.5f  //0.0f
#define YAW_SPEED_PID_MAX_OUT		MAX_GM6020_CAN_VOLTAGE
#define YAW_SPEED_PID_MAX_IOUT		5000.0f
//pitch 角度环 反馈角度由编码器提供 外环
#define PITCH_ANGLE_PID_KP 150.0f//65
#define PITCH_ANGLE_PID_KI 0.0f
#define PITCH_ANGLE_PID_KD 0.0f//2.0
#define PITCH_ANGLE_PID_MAX_OUT 10.0f
#define PITCH_ANGLE_PID_MAX_IOUT 0.0f
//yaw 角度环 反馈角度由陀螺仪提供
#define YAW_ANGLE_PID_KP 178.0f
#define YAW_ANGLE_PID_KI 0.0f
#define YAW_ANGLE_PID_KD 0.0f
#define YAW_ANGLE_PID_MAX_OUT 10.0f
#define YAW_ANGLE_PID_MAX_IOUT 0.0f
//pitch 自瞄角度环
#define PITCH_AUTO_PID_KP 100.0f
#define PITCH_AUTO_PID_KI 0.0f
#define PITCH_AUTO_PID_KD 0.0f
#define PITCH_AUTO_PID_MAX_OUT 10.0f
#define PITCH_AUTO_PID_MAX_IOUT 0.0f
//yaw 自瞄角度环
#define YAW_AUTO_PID_KP       100.0f
#define YAW_AUTO_PID_KI        0.0f
#define YAW_AUTO_PID_KD        0.0f
#define YAW_AUTO_PID_MAX_OUT   10.0f
#define YAW_AUTO_PID_MAX_IOUT  0.0f

typedef enum
{
	GIMBAL_RELAX,				//云台无力
	GIMBAL_AUTO,				//云台自动巡逻模式
	GIMBAL_RC,					//云台遥控控制
	GIMBAL_CALI					//云台校准
}Gimbal_Mode_e;
//typedef struct
//{
//	fp32 angle_max;
//	fp32 angle_min;
//	
//}Gimbal_Limit_s;
typedef struct
{
	const motor_measure_t *gimbal_motor_measure;	//电调反馈信息
	first_order_filter_type_t gimbal_motor_first_OF;	//电机一阶低通滤波
	PidTypeDef motor_speed_pid;	//角速度环pid
	PidTypeDef motor_angle_pid;	//角度环pid
	PidTypeDef motor_auto_pid;	//自瞄角度环pid
	fp32 angle;				//当前角度 rad
	fp32 angle_set;			//设定角度
	uint16_t angle_max;		//机械角度限位rad
	uint16_t angle_min;
	fp32 speed;				//当前角速度rad/s
	fp32 speed_set;			//设定角速度
	fp32 speed_max;			//角速度限位
	fp32 speed_min;
	fp32 voltage_set;
	int16_t last_give_voltage;
	int16_t give_voltage;	//发送的电压值
} Gimbal_Motor_s;
typedef struct
{
	const RC_ctrl_t *gimbal_RC;				//云台使用的遥控器指针
	Gimbal_Mode_e gimbal_mode;				//云台模式
	Gimbal_Mode_e last_gimbal_mode;			//云台上一次模式
	
	Gimbal_Motor_s pitch_motor;				//云台pitch电机
	Gimbal_Motor_s yaw_motor;				//云台yaw电机
	
}Gimbal_Control_s;


#endif

