#include "test02.h"

#include "main.h"

#include "arm_math.h"
//#include "gimbal_behaviour.h"
//#include "user_lib.h"
#include "INS_Task.h"
#include "remote_control.h"
#include "shoot.h"
#include "CAN_Receive.h"
#include "Detect_Task.h"
#include "pid.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

////电机编码值规整 0—8191
//#define ECD_Format(ecd)         \
//    {                           \
//        if ((ecd) > ecd_range)  \
//            (ecd) -= ecd_range; \
//        else if ((ecd) < 0)     \
//            (ecd) += ecd_range; \
//    }


//#if INCLUDE_uxTaskGetStackHighWaterMark	//查看任务堆栈剩余容量
//uint32_t gimbal_high_water;
//#endif

//云台控制所有相关数据
static Gimbal_Control_s gimbal_info;

//发送的can 指令
static int16_t Yaw_Can_Set_Voltage = 0, Pitch_Can_Set_Voltage = 0, Shoot_Can_Set_Voltage = 0;


//云台初始化
static void Gimbal_Init(void);
static void Gimbal_PID_Init(void);
static void Gimbal_mode_set(void);
static void Gimbal_Updata(void);
static void Gimbal_Control(void);
static void Gimbal_Send_Voltage(void);

static void Gimbal_Calibration_mode(void);
static void Gimbal_Relax_mode(void);
static void Gimbal_Auto_mode(fp32 *, fp32 *);
static void Gimbal_Rc_mode(fp32 *, fp32 *);
	
static void gimbal_auto_patrol(void);
static void gimbal_pid_calculation(Gimbal_Mode_e gimbal_mode);
static void Angle_Limit(void);


void GIMBAL_task(void *pvParameters)
{
    //等待陀螺仪任务更新陀螺仪数据
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //云台初始化
    Gimbal_Init();
    //射击初始化
    shoot_init();
    //判断电机是否都上线
    while (toe_is_error(YawGimbalMotorTOE) || toe_is_error(PitchGimbalMotorTOE)|| toe_is_error(TriggerMotorTOE) )
    {
        vTaskDelay(GIMBAL_CONTROL_TIME_MS);
        Gimbal_Updata();             //云台数据反馈
    }

    while (1)
    {
		//模式 遥控 自动 无力
		Gimbal_mode_set();
		//云台数据更新
		Gimbal_Updata();
		//云台控制量计算
		Gimbal_Control();
		//更新射击电流
        Shoot_Can_Set_Voltage = shoot_control_loop();  
		//发送控制电压
		Gimbal_Send_Voltage();
		//系统延时
        vTaskDelay(GIMBAL_CONTROL_TIME_MS);

//#if INCLUDE_uxTaskGetStackHighWaterMark
//        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
//#endif
    }
}


//发送控制电压
static void Gimbal_Send_Voltage(void)
{
	//电机是否反装
#if YAW_TURN
	Yaw_Can_Set_Voltage = -gimbal_info.yaw_motor.give_voltage;
#else
	Yaw_Can_Set_Voltage = gimbal_info.yaw_motor.give_voltage;
#endif

#if PITCH_TURN
	Pitch_Can_Set_Voltage = -gimbal_info.pitch_motor.give_voltage;
#else
	Pitch_Can_Set_Voltage = gimbal_info.pitch_motor.give_voltage;
#endif

	//云台在遥控器掉线状态即relax 状态，can指令为0，不使用Voltage设置为零的方法，是保证遥控器掉线一定使得云台停止
	if (!(toe_is_error(YawGimbalMotorTOE) && toe_is_error(PitchGimbalMotorTOE) && toe_is_error(TriggerMotorTOE)))
	{
		//当底盘模式为无力状态 为了不受干扰 直接发送电流为0	
		if (gimbal_info.gimbal_mode == GIMBAL_RELAX)
		{
			CAN_CMD_GIMBAL(0, 0, 0, 0);
		}
		else
		{
			CAN_CMD_GIMBAL(Yaw_Can_Set_Voltage, Pitch_Can_Set_Voltage, Shoot_Can_Set_Voltage, 0);
		}
	}
}

static void GIMBAL_Init()
{
	const static fp32 gimbal_pitch_order_filter[1] = {GIMBAL_PITCH_ACCEL_NUM};
	const static fp32 gimbal_yaw_order_filter[1] = {GIMBAL_YAW_ACCEL_NUM};
	//PID初始化
	Gimbal_PID_Init();
	//滤波初始化
	first_order_filter_init(&gimbal_info.pitch_motor.gimbal_motor_first_OF, GIMBAL_CONTROL_TIME_MS, gimbal_pitch_order_filter);
	first_order_filter_init(&gimbal_info.yaw_motor.gimbal_motor_first_OF, GIMBAL_CONTROL_TIME_MS, gimbal_yaw_order_filter);
	//获取遥控器数据
	gimbal_info.gimbal_RC = get_remote_control_point();
	//获取电机数据
	gimbal_info.yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();
	gimbal_info.pitch_motor.gimbal_motor_measure = get_Pitch_Gimbal_Motor_Measure_Point();
	//获取自瞄数据
	
	//最大角度限制
	gimbal_info.yaw_motor.angle_max = YAW_ECD_MAX;
	gimbal_info.yaw_motor.angle_min = YAW_ECD_MIN;
	gimbal_info.pitch_motor.angle_max = PITCH_ECD_MAX;
	gimbal_info.pitch_motor.angle_min = PITCH_ECD_MIN;
}
//云台PID初始化
static void Gimbal_PID_Init(void)
{
	//速度环 内环
	const fp32 yaw_motor_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
	const fp32 pitch_motor_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
	//角度环 外环
	const fp32 yaw_motor_angle_pid[3] = {YAW_ANGLE_PID_KP, YAW_ANGLE_PID_KI, YAW_ANGLE_PID_KD};
	const fp32 pitch_motor_angle_pid[3] = {PITCH_ANGLE_PID_KP, PITCH_ANGLE_PID_KI, PITCH_ANGLE_PID_KD};
	//自瞄角度环 外环
	const fp32 yaw_motor_auto_pid[3] = {YAW_AUTO_PID_KP, YAW_AUTO_PID_KI, YAW_AUTO_PID_KD};
	const fp32 pitch_motor_auto_pid[3] = {PITCH_AUTO_PID_KP, PITCH_AUTO_PID_KI, PITCH_AUTO_PID_KD};
	
	PID_Init(&gimbal_info.yaw_motor.motor_speed_pid, PID_POSITION, yaw_motor_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT );
	PID_Init(&gimbal_info.yaw_motor.motor_angle_pid, PID_POSITION, yaw_motor_angle_pid, YAW_ANGLE_PID_MAX_OUT, YAW_ANGLE_PID_MAX_IOUT );
	PID_Init(&gimbal_info.yaw_motor.motor_auto_pid, PID_POSITION, yaw_motor_auto_pid, YAW_AUTO_PID_MAX_OUT, YAW_AUTO_PID_MAX_IOUT );
	
	PID_Init(&gimbal_info.pitch_motor.motor_speed_pid, PID_POSITION, pitch_motor_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT );
	PID_Init(&gimbal_info.pitch_motor.motor_angle_pid, PID_POSITION, pitch_motor_angle_pid, PITCH_ANGLE_PID_MAX_OUT, PITCH_ANGLE_PID_MAX_IOUT );
	PID_Init(&gimbal_info.pitch_motor.motor_auto_pid, PID_POSITION, pitch_motor_auto_pid, PITCH_AUTO_PID_MAX_OUT, PITCH_AUTO_PID_MAX_IOUT );
}
static void Gimbal_mode_set(void)
{
	//当遥控器离线的时候，为AUTO模式
	if (toe_is_error(DBUSTOE))
	{
		gimbal_info.gimbal_mode = GIMBAL_AUTO;//自动
	}
    else if (switch_is_mid(gimbal_info.gimbal_RC->rc.s[GIMBAL_MODE_SW]))		//中间为遥控
    {
        gimbal_info.gimbal_mode = GIMBAL_RC;//遥控
    }
    else if (switch_is_up(gimbal_info.gimbal_RC->rc.s[GIMBAL_MODE_SW]))		//上拨为自动
    {
        gimbal_info.gimbal_mode = GIMBAL_AUTO;//自动
    }
    else if (switch_is_down(gimbal_info.gimbal_RC->rc.s[GIMBAL_MODE_SW]))	//下拨为无力
    {
        gimbal_info.gimbal_mode = GIMBAL_RELAX;//云台无力
    }
}
//计算相对角度
static fp32 relative_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > Half_Ecd_range)
    {
        relative_ecd -= ecd_range;
    }
    else if (relative_ecd < -Half_Ecd_range)
    {
        relative_ecd += ecd_range;
    }
	//电机编码值转换为rad
    return relative_ecd * Ecd_to_Rad;
}
static void Gimbal_Updata(void)
{
	const fp32 *gimbal_INT_angle_point = get_INS_angle_point();//获取陀螺仪姿态解算后的数据 角度
	const fp32 *gimbal_INT_gyro_point = get_MPU6500_Gyro_Data_Point();//获取陀螺仪原始数据 角速度
	//角度更新  外环
	gimbal_info.pitch_motor.angle = relative_angle_change( gimbal_info.pitch_motor.gimbal_motor_measure->ecd, PITCH_ECD_DEL);
	gimbal_info.yaw_motor.angle = relative_angle_change( gimbal_info.yaw_motor.gimbal_motor_measure->ecd, YAW_ECD_DEL);
	//角速度更新  内环
	//gimbal_info.pitch_motor.speed = *(gimbal_INT_gyro_point + INS_PITCH_ADDRESS_OFFSET);		//陀螺仪能够跟随云台pitch轴
	gimbal_info.pitch_motor.speed = gimbal_info.pitch_motor.gimbal_motor_measure->speed_rpm;	//否则则用电机反馈的转速值rmp
	gimbal_info.yaw_motor.speed = *(gimbal_INT_gyro_point + INS_YAW_ADDRESS_OFFSET);//gimbal_info.yaw_motor.gimbal_motor_measure->speed_rpm;
}

static void Gimbal_Control(void)
{
	fp32 add_yaw_angle = 0.0f;
	fp32 add_pitch_angle = 0.0f;
	//选择控制量输入
	switch(gimbal_info.gimbal_mode)
	{
		case GIMBAL_CALI:
		{
			Gimbal_Calibration_mode();
			return;
		}
		case GIMBAL_RELAX:
		{
			Gimbal_Relax_mode();
			break;
		}
		case GIMBAL_AUTO:
		{
			Gimbal_Auto_mode(&add_pitch_angle, &add_yaw_angle);
			break;
		}
		case GIMBAL_RC:
		{
			Gimbal_Rc_mode(&add_pitch_angle, &add_yaw_angle);
			break;
		}
	}
	//叠加
	gimbal_info.pitch_motor.angle_set += add_pitch_angle;
	gimbal_info.yaw_motor.angle_set += add_yaw_angle;
	//角度限幅
	Angle_Limit();
	//PID计算
	gimbal_pid_calculation(gimbal_info.gimbal_mode);
}
//云台PID计算
static void gimbal_pid_calculation(Gimbal_Mode_e gimbal_mode)
{
	if( gimbal_mode == GIMBAL_AUTO)
	{
		//PID计算 角度环为外环 速度环为内环
		gimbal_info.pitch_motor.speed_set = PID_Calc(&gimbal_info.pitch_motor.motor_auto_pid, 
														gimbal_info.pitch_motor.angle, gimbal_info.pitch_motor.angle_set );
		gimbal_info.pitch_motor.voltage_set = PID_Calc(&gimbal_info.pitch_motor.motor_speed_pid, 
														gimbal_info.pitch_motor.speed, gimbal_info.pitch_motor.speed_set );
		gimbal_info.yaw_motor.speed_set = PID_Calc(&gimbal_info.yaw_motor.motor_auto_pid, 
														gimbal_info.yaw_motor.angle, gimbal_info.yaw_motor.angle_set );
		gimbal_info.yaw_motor.voltage_set = PID_Calc(&gimbal_info.yaw_motor.motor_speed_pid, 
														gimbal_info.yaw_motor.speed, gimbal_info.yaw_motor.speed_set );
	}
	else if( gimbal_mode == GIMBAL_RC )
	{
		//PID计算 角度环为外环 角速度环为内环
		gimbal_info.pitch_motor.speed_set = PID_Calc(&gimbal_info.pitch_motor.motor_angle_pid, 
														gimbal_info.pitch_motor.angle, gimbal_info.pitch_motor.angle_set );
		gimbal_info.pitch_motor.voltage_set = PID_Calc(&gimbal_info.pitch_motor.motor_speed_pid, 
														gimbal_info.pitch_motor.speed, gimbal_info.pitch_motor.speed_set );
		gimbal_info.yaw_motor.speed_set = PID_Calc(&gimbal_info.yaw_motor.motor_angle_pid, 
														gimbal_info.yaw_motor.angle, gimbal_info.yaw_motor.angle_set );
		gimbal_info.yaw_motor.voltage_set = PID_Calc(&gimbal_info.yaw_motor.motor_speed_pid, 
														gimbal_info.yaw_motor.speed, gimbal_info.yaw_motor.speed_set );
	}
	else
	{
		return;
	}
}
//角度限制
static void Angle_Limit(void)
{
	if( gimbal_info.pitch_motor.angle_set > gimbal_info.pitch_motor.angle_max )
	{
		gimbal_info.pitch_motor.angle_set = gimbal_info.pitch_motor.angle_max;
	}
	else if( gimbal_info.pitch_motor.angle_set < gimbal_info.pitch_motor.angle_min )
	{
		gimbal_info.pitch_motor.angle_set = gimbal_info.pitch_motor.angle_min;
	}
	if( gimbal_info.yaw_motor.angle_set > gimbal_info.yaw_motor.angle_max )
	{
		gimbal_info.yaw_motor.angle_set = gimbal_info.yaw_motor.angle_max;
	}
	else if( gimbal_info.yaw_motor.angle_set < gimbal_info.yaw_motor.angle_min )
	{
		gimbal_info.yaw_motor.angle_set = gimbal_info.yaw_motor.angle_min;
	}
}
static void Gimbal_Calibration_mode(void)
{
	
	return;
}
//云台无力模式
static void Gimbal_Relax_mode(void)
{
	//pitch
	gimbal_info.pitch_motor.speed_set = 0.0f;
	gimbal_info.pitch_motor.angle_set = 0.0f;
	gimbal_info.pitch_motor.voltage_set = 0.0f;
	gimbal_info.pitch_motor.give_voltage = 0.0f;
	//yaw
	gimbal_info.yaw_motor.speed_set = 0.0f;
	gimbal_info.yaw_motor.angle_set = 0.0f;
	gimbal_info.yaw_motor.voltage_set = 0.0f;
	gimbal_info.yaw_motor.give_voltage = 0.0f;
}
//云台自动模式
static void Gimbal_Auto_mode(fp32 *pitch_add, fp32 *yaw_add)
{
	//初始化相关变量
	uint8_t vision_flag = 0;//Vision_update_flag();			//获取视觉标志位
	//视觉是否识别到
	if( vision_flag )	//识别到目标 云台由视觉控制
	{
		
	}

}
//云台遥控控制
static void Gimbal_Rc_mode(fp32 *pitch_add, fp32 *yaw_add)
{
	//初始化相关变量
	int16_t yaw_channel = 0;
	int16_t pitch_channel = 0;
	//遥控器死区处理
	if( gimbal_info.gimbal_RC->rc.ch[YAW_CHANNEL] > GIMBAL_RC_DEADLINE || gimbal_info.gimbal_RC->rc.ch[YAW_CHANNEL] < -GIMBAL_RC_DEADLINE )
		yaw_channel = gimbal_info.gimbal_RC->rc.ch[YAW_CHANNEL];
	if( gimbal_info.gimbal_RC->rc.ch[PITCH_CHANNEL] > GIMBAL_RC_DEADLINE || gimbal_info.gimbal_RC->rc.ch[PITCH_CHANNEL] < -GIMBAL_RC_DEADLINE )
		pitch_channel = gimbal_info.gimbal_RC->rc.ch[PITCH_CHANNEL];
	//遥控数据比例转换
	*pitch_add = yaw_channel * YAW_RC_SEN;
	*yaw_add = pitch_channel * PITCH_RC_SEN;
}
