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

//底盘模式选择 右边开关S0
#define CHASSIS_MODE_SW			0
#if RC_MODE == USA	//美国手
#define CHASSIS_X_CHANNEL 1		//前后的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0		//左右的遥控器通道号码
#define CHASSIS_WZ_CHANNEL 2
//#define CHASSIS_WZ_CHANNEL 2	//在特殊模式下，可以通过遥控器控制旋转
#elif RC_MODE == JP	//日本手
#define CHASSIS_X_CHANNEL 3
#define CHASSIS_Y_CHANNEL 2
#define CHASSIS_WZ_CHANNEL 0
#elif RC_MODE == CN	//中国手
#define CHASSIS_X_CHANNEL 1//3是日本手
#define CHASSIS_Y_CHANNEL 0//2是日本手
#define CHASSIS_WZ_CHANNEL 2//0是日本手
#endif	/*RC_MODE*/

//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define CHASSIS_RC_DEADLINE 10
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
#define RC_CHASSIS_VY_SEN	0.005f
#define RC_CHASSIS_VW_SEN	0.0001f
//键盘转换比例
#define KEY_CHASSIS_VX_SEN	0.006f
#define KEY_CHASSIS_VY_SEN	0.005f
//旋转角度输入比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//底盘遥控滤波系数
#define CHASSIS_X_ACCEL_NUM 0.1666666667f
#define CHASSIS_Y_ACCEL_NUM 0.3333333333f
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

//底盘3508最大can发送电流值
#define MAX_M3508_CAN_CURRENT 16000.0f
//PID参数
//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_M3508_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
//底盘角度环PID
#define CHASSIS_ANGLE_PID_KP 10.0f  //40
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
	CHASSIS_GYRO,				//小陀螺模式
	CHASSIS_CALI				//校准模式
}Chassis_Mode_e;//底盘模式
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
	const Gimbal_Motor_s *yaw_motor_gimbal;   //底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角
	const INS_Data_s *ins_data;			//陀螺仪相关角度
	
	Control_Mode_e ctrl_mode;				//控制模式
	Chassis_Mode_e chassis_mode;			//底盘模式
	Chassis_Mode_e chassis_last_mode;
	
	PidTypeDef chassis_angle_pid;			//底盘整体旋转角度环
	Chassis_Motor_s chassis_motor[4];		//底盘四个电机
	first_order_filter_type_t chassis_vx_first_OF;	//一阶低通滤波代替斜坡函数生成前进方向速度
	first_order_filter_type_t chassis_vy_first_OF;	//一阶低通滤波代替斜坡函数生成左右方向速度
	//底盘移动数据
	fp32 chassis_vx;	//m/s
	fp32 chassis_vx_set;
	fp32 chassis_vy;
	fp32 chassis_vy_set;
//	fp32 chassis_vx_max;
//	fp32 chassis_vx_min;
//	fp32 chassis_vy_max;
//	fp32 chassis_vy_min;
	fp32 chassis_vw;		//底盘旋转角速度rad/s		ra:旋转角度rotation angle
	fp32 chassis_vw_set;	//底盘旋转角速度设定值
	fp32 chassis_relative_angle;	//底盘与云台的相对角度rad/s
//	fp32 chassis_relative_angle_set;//设置相对云台控制角度
	
	fp32 chassis_last_absolute_yaw;	//上一次角度
	fp32 chassis_absolute_yaw;		//底盘绝对角度 陀螺仪减去云台yaw的角度 底盘角度固定码盘
	fp32 chassis_absolute_yaw_set;
	fp32 chassis_yaw;				//叠加圈数后的角度
	fp32 chassis_last_yaw;
	
	//fp32 chassis_absolute_pitch;	//爬坡模式可用 后续可增加底盘模式
	
	uint8_t chassis_table_flag;//角度盘正负 1为正 0为负
	int16_t chassis_circle_num;//圈数
}Chassis_Control_s;//底盘总数据


void chassis_task(void *pvParameters);
const Chassis_Control_s *get_classis_info_point(void);

#endif


