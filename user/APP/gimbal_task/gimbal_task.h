#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "main.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "RC_DT7.h"
#include "miniPC.h"
#include "user_lib.h"
#include "motor_control.h"

//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201
//云台控制任务间隔
#define GIMBAL_CONTROL_TIME_MS 1

/******电机相关机械角度 更换电机时需要debug修改！！！******************/
#define YAW_ECD_MAX		2709		//云台机械角度限位
#define YAW_ECD_MIN		6793
#define YAW_ECD_DEL		655			//云台回中时电机的机械角度
#define YAW_ECD_TURN	4000		//云台回中时判断正负码盘机械角度 小于此值为正码盘 大于此值为负码盘 用于回中时 其他状态不用
#define PITCH_ECD_MAX	3811
#define PITCH_ECD_MIN	2036
#define PITCH_ECD_DEL	2696
#define PITCH_ECD_TURN	6000
/*******************相关机械角度_END**********************************/

//云台模式选择 左边开关
#define GIMBAL_MODE_SW		0
//左边摇杆控制
#define YAW_CHANNEL			2
#define PITCH_CHANNEL		3
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define GIMBAL_RC_DEADLINE	10
//yaw，pitch角度与遥控器输入比例
#define YAW_RC_SEN		0.000003f
#define PITCH_RC_SEN	-0.000003f
//鼠标输入比例
#define YAW_MOUSE_SEN	0.000001f
#define PITCH_MOUSE_SEN	-0.000001f
//自瞄数据输入比例
#define YAW_AUTO_SEN    0.0055f//-0.0055f
#define PITCH_AUTO_SEN  0.044f//-0.07f //0.005
//云台回中速度
#define GIMBAL_RETURN_PITCH	0.0008f
#define GIMBAL_RETURN_YAW	0.0008f
//滤波系数
#define GIMBAL_YAW_ACCEL_NUM	0.3333333333f//0.1666666667f
#define GIMBAL_PITCH_ACCEL_NUM	0.3333333333f
//GM6020最大CAN发送电压
#define MAX_GM6020_CAN_VOLTAGE	30000.0f

/***********************PID***************************/
/*PID参数模板
{
	KP,
	KI,
	KD,
	OUT_MAX,
	IOUT_MAX
}*/
//yaw   遥控 机械模式 速度环(内环)
#define YAW_ECD_SPEED_RC_PID_Init	1500.0f, 0.0f, 0.5f, MAX_GM6020_CAN_VOLTAGE, 5000.0f
//yaw   遥控 机械模式 角度环(外环)
#define YAW_ECD_ANGLE_RC_PID_Init	178.0f, 0.0f, 0.0f, 10.0f, 0.0f
//yaw   遥控 陀螺仪模式 速度环(内环)
#define YAW_GYRO_SPEED_RC_PID_Init	1500.0f, 0.0f, 0.5f, MAX_GM6020_CAN_VOLTAGE, 5000.0f
//yaw   遥控 陀螺仪模式 角度环(外环)
#define YAW_GYRO_ANGLE_RC_PID_Init	120.0f, 0.0f, 0.5f, 10.0f, 0.0f
//pitch 遥控 机械模式 速度环(内环)
#define PITCH_ECD_SPEED_RC_PID_Init	1000.0f, 0.0f, 2.0f, MAX_GM6020_CAN_VOLTAGE, 5000.0f
//pitch 遥控 机械模式 角度环(外环)
#define PITCH_ECD_ANGLE_RC_PID_Init	150.0f, 0.0f, 0.0f, 10.0f, 0.0f
//pitch 遥控 陀螺仪模式 速度环(内环)
#define PITCH_GYRO_SPEED_RC_PID_Init	1000.0f, 0.0f, 2.0f, MAX_GM6020_CAN_VOLTAGE, 5000.0f
//pitch 遥控 陀螺仪模式 角度环(外环)
#define PITCH_GYRO_ANGLE_RC_PID_Init	150.0f, 0.0f, 0.0f, 10.0f, 0.0f
/*****************************以上为手动控制下的PID参数*****************************/
//yaw 自瞄 机械模式 速度环(内环)
#define YAW_ECD_SPEED_AUTO_PID_Init	1500.0f, 0.0f, 0.5f, MAX_GM6020_CAN_VOLTAGE, 5000.0f
//yaw 自瞄 机械模式 角度环(外环)
#define YAW_ECD_ANGLE_AUTO_PID_Init	178.0f, 0.0f, 0.0f, 10.0f, 0.0f
//yaw 自瞄 陀螺仪模式 速度环(内环)
#define YAW_GYRO_SPEED_AUTO_PID_Init	1500.0f, 0.0f, 0.5f, MAX_GM6020_CAN_VOLTAGE, 5000.0f
//yaw 自瞄 陀螺仪模式 角度环(外环)
#define YAW_GYRO_ANGLE_AUTO_PID_Init	120.0f, 0.0f, 0.5f, 10.0f, 0.0f
//pitch 自瞄 机械模式 速度环(内环)
#define PITCH_ECD_SPEED_AUTO_PID_Init	1000.0f, 0.0f, 2.0f, MAX_GM6020_CAN_VOLTAGE, 5000.0f
//pitch 自瞄 机械模式 角度环(外环)
#define PITCH_ECD_ANGLE_AUTO_PID_Init	150.0f, 0.0f, 0.0f, 10.0f, 0.0f
//pitch 自瞄 陀螺仪模式 速度环(内环)
#define PITCH_GYRO_SPEED_AUTO_PID_Init	1000.0f, 0.0f, 2.0f, MAX_GM6020_CAN_VOLTAGE, 5000.0f
//pitch 自瞄 陀螺仪模式 角度环(外环)
#define PITCH_GYRO_ANGLE_AUTO_PID_Init	150.0f, 0.0f, 0.0f, 10.0f, 0.0f
/*****************************以上为自动控制下的PID参数*****************************/
//小陀螺模式 遥控 速度环(内环)
#define YAW_gyromode_SPEED_RC_PID_Init	1000.0f, 0.0f, 2.0f, MAX_GM6020_CAN_VOLTAGE, 5000.0f 
//陀螺仪模式 遥控 角度环(外环)
#define YAW_gyromode_ANGLE_RC_PID_Init	150.0f, 0.0f, 0.0f, 10.0f, 0.0f
//小陀螺模式 自瞄 速度环(内环)
#define YAW_gyromode_SPEED_AUTO_PID_Init	1000.0f, 0.0f, 2.0f, MAX_GM6020_CAN_VOLTAGE, 5000.0f 
//陀螺仪模式 自瞄 角度环(外环)
#define YAW_gyromode_ANGLE_AUTO_PID_Init	150.0f, 0.0f, 0.0f, 10.0f, 0.0f
/*****************************以上为陀螺仪模式下YAW轴的PID参数*****************************/
/**********************PID_END************************/
typedef enum
{
	GIMBAL_RELAX,				//云台无力
	GIMBAL_GYRO,				//陀螺仪角度控制
	GIMBAL_ENCONDE,				//电机编码器控制
	GIMBAL_CALI					//云台校准
}Gimbal_Mode_e;
typedef enum
{
	GIMBAL_NORMAL,				//云台普通
	GIMBAL_NORMAL_GR,			//底盘小陀螺下云台普通
	GIMBAL_AUTO,				//云台自瞄
	GIMBAL_AUTO_GR,				//底盘小陀螺下云台自瞄
	GIMBAL_CENTER,				//云台回中
	GIMBAL_TURN,				//云台掉头
	GIMBAL_STOP					//云台停止
}Gimbal_Behavior_e;
typedef struct
{
	const motor_measure_t *gimbal_motor_measure;	//电调反馈信息
	first_order_filter_type_t gimbal_motor_first_OF;	//电机一阶低通滤波
	
	//由电机反馈角度
	fp32 relative_angle;
	fp32 relative_angle_last;
	fp32 relative_angle_set;
	//陀螺仪反馈角度 rad
	fp32 absolute_angle;			
	fp32 absolute_angle_set;
	fp32 absolute_angle_last;
	
	fp32 speed;				//当前角速度rad/s
	fp32 speed_set;			//设定角速度

	int16_t ecd_last;		//上一次电机反馈机械角度原始数据
	fp32 gyro_last;			//上一次陀螺仪原始数据
	fp32 ecd_now;			//当前转换后的角度 -2PI~2PI rad
	fp32 gyro_now;			//当前转换后的角度 -2PI~2PI rad
	
	fp32 voltage_set;
	int16_t voltage_give_last;
	int16_t voltage_give;	//发送的电压值
	
	uint8_t turn_table_flag;//角度盘正负 1为正 0为负
} Gimbal_Motor_s;
typedef struct
{
	const RC_ctrl_s *gimbal_RC;				//云台使用的遥控器指针
	Auto_Gimbal_Ctrl_t *gimbal_AUTO_ctrl;	//自瞄角度数据
	
	Control_Mode_e ctrl_mode;				//控制模式
	Gimbal_Mode_e gimbal_mode;				//云台模式
	Gimbal_Behavior_e gimbal_behavior;		//云台行为
	Gimbal_Mode_e gimbal_mode_last;			//云台上一次模式
	Gimbal_Behavior_e gimbal_behavior_last;		//云台行为
	
	Gimbal_Motor_s pitch_motor;				//云台pitch电机
	Gimbal_Motor_s yaw_motor;				//云台yaw电机
	
	int16_t turn_circle_num;
	uint8_t turn_mid_flag;
}Gimbal_Control_s;


extern const Gimbal_Motor_s *get_gimbal_yaw_motor_point(void);
extern const Gimbal_Motor_s *get_gimbal_pitch_motor_point(void);
extern fp32 get_gimbal_relative_angle(void);
extern void GIMBAL_task(void *pvParameters);

#endif

