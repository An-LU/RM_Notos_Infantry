#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "main.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "RC_DT7.h"
#include "user_lib.h"
#include "Gimbal_Task.h"
#include "INS_Task.h"
#include "motor_control.h"


//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357
//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002
//在速度更新使用		底盘任务控制频率
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//遥控转换比例
#define RC_CHASSIS_VX_SEN	0.006f
#define RC_CHASSIS_VY_SEN	-0.005f
#define RC_CHASSIS_VW_SEN	0.01f
//键盘转换比例
#define KEY_CHASSIS_VX_SEN	0.006f
#define KEY_CHASSIS_VY_SEN	0.005f
//旋转角度输入比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.0002f
//底盘遥控滤波系数
#define CHASSIS_X_ACCEL_NUM 0.2222222222f//0.1666666667f
#define CHASSIS_Y_ACCEL_NUM 0.1222222222f//0.1666666667f//0.3333333333f
//旋转输入比例
#define CHASSIS_VW_SEN		0.000415809748903494517209f
//底盘3508最大can发送电流值
#define MAX_M3508_CAN_CURRENT 16000.0f
//m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例    2Π/60 * r 
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f//0.000798488132787405781442588526584f//0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
//电机速度转换成整车速度
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VW 0.25f
//底盘电机最大速度
#define MAX_WHEEL_SPEED 4.0f
//底盘运动过程最大前进速度
#define MAX_CHASSIS_SPEED_X 3.0f
//底盘运动过程最大平移速度
#define MAX_CHASSIS_SPEED_Y 2.9f
//底盘长宽
#define ROBOT_LENGTH_A 0.5f
#define ROBOT_LENGTH_B 0.4f

#define MOTOR_DISTANCE_TO_CENTER 0.3f	//0.2f
//底盘3508最大can发送电流值
#define MAX_M3508_CAN_CURRENT 16000.0f
//PID参数
//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 20.0f//10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_M3508_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
//底盘角度环PID
#define CHASSIS_ANGLE_PID_KP 40.0f  //40
#define CHASSIS_ANGLE_PID_KI 0.0f
#define CHASSIS_ANGLE_PID_KD 0.0f
#define CHASSIS_ANGLE_PID_MAX_OUT 6.0f
#define CHASSIS_ANGLE_PID_MAX_IOUT 0.2f

typedef enum
{
	CHASSIS_RELAX,				//底盘无力
	CHASSIS_STOP,				//底盘停止
	CHASSIS_FOLLOW_YAW,			//底盘跟随云台模式 前进方向由云台决定
	CHASSIS_NO_FOLLOW,			//底盘不跟随云台模式 前进方向由底盘决定
	CHASSIS_CALI				//校准模式
}Chassis_Mode_e;//底盘模式
typedef enum
{
	CHASSIS_NORMAL,				//普通
	CHASSIS_GYRO,				//小陀螺
	CHASSIS_CENTER				//底盘回中
}Chassis_Behavior_e;//底盘行为
typedef struct
{
	const motor_measure_t *chassis_motor_measure;	//电调反馈信息
	PidTypeDef motor_speed_pid;		//电机速度环 内环
	
	fp32 accel;						//加速度
	fp32 speed;						//当前速度
	fp32 speed_set;					//设定速度
	int16_t last_give_current;
	int16_t give_current;			//发送的电流值
} Chassis_Motor_s;//电机数据
typedef struct
{
	const RC_ctrl_s *chassis_RC;			//遥控数据
	const INS_Data_s *ins_data;				//陀螺仪相关角度
	//模式数据
	Control_Mode_e ctrl_mode;				//控制模式
	Chassis_Mode_e chassis_mode;			//底盘模式
	Chassis_Mode_e chassis_last_mode;
	Chassis_Behavior_e chassis_behavior;	//底盘行为
	//底盘电机数据
	PidTypeDef chassis_angle_pid;			//底盘整体旋转角度环
	Chassis_Motor_s chassis_motor[4];		//底盘四个电机
	first_order_filter_type_t chassis_vx_first_OF;	//一阶低通滤波代替斜坡函数生成前进方向速度
	first_order_filter_type_t chassis_vy_first_OF;	//一阶低通滤波代替斜坡函数生成左右方向速度
	//底盘移动数据
	fp32 chassis_vx;	//m/s
	fp32 chassis_vx_set;
	fp32 chassis_vy;
	fp32 chassis_vy_set;
	fp32 chassis_vw;		//底盘旋转角速度rad/s		ra:旋转角度rotation angle
	fp32 chassis_vw_set;	//底盘旋转角速度设定值
	//底盘角度数据
	fp32 chassis_relative_angle;		//底盘与云台的相对角度rad/s 用于计算跟随云台模式下的前进方向 -2PI~2PI rad
	fp32 chassis_absolute_yaw;			//底盘绝对角度 陀螺仪减去云台yaw的角度 底盘角度固定码盘 -2PI~2PI rad
	//fp32 chassis_absolute_yaw_last;		//上一次角度
	fp32 chassis_yaw;					//叠加圈数后的底盘yaw角度 实际角度
	fp32 chassis_yaw_set;
	fp32 chassis_yaw_last;
	//fp32 chassis_absolute_pitch;	//爬坡模式可用 后续可增加底盘模式
	//底盘旋转相关数据
	uint8_t chassis_angle_table;//角度盘正负 1为正 0为负
	int16_t chassis_circle_num;//圈数
}Chassis_Control_s;//底盘总数据


void chassis_task(void *pvParameters);
const Chassis_Control_s *get_classis_info_point(void);

#endif


