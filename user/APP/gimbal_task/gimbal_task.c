#include "Gimbal_Task.h"

#include "arm_math.h"
//#include "gimbal_behaviour.h"
//#include "user_lib.h"
#include "INS_Task.h"
#include "RC_DT7.h"
#include "shoot.h"
#include "CAN_Receive.h"
#include "Detect_Task.h"
#include "pid.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

//#if INCLUDE_uxTaskGetStackHighWaterMark	//查看任务堆栈剩余容量
//uint32_t gimbal_high_water;
//#endif

//云台控制所有相关数据
static Gimbal_Control_s gimbal_info;
//发送的云台can 指令
static int16_t Yaw_Can_Set_Voltage = 0, Pitch_Can_Set_Voltage = 0, Shoot_Can_Set_Voltage = 0;
/**************************PID**********************************/
//yaw 手动控制PID
static PidTypeDef yaw_ecd_speed_rc_pid = { PID_POSITION, YAW_ECD_SPEED_RC_PID_Init };
static PidTypeDef yaw_ecd_angle_rc_pid = { PID_POSITION, YAW_ECD_ANGLE_RC_PID_Init };
static PidTypeDef yaw_gyro_speed_rc_pid = { PID_POSITION, YAW_GYRO_SPEED_RC_PID_Init };
static PidTypeDef yaw_gyro_angle_rc_pid = { PID_POSITION, YAW_GYRO_ANGLE_RC_PID_Init };
//pitch 手动控制PID
static PidTypeDef pitch_ecd_speed_rc_pid = { PID_POSITION, PITCH_ECD_SPEED_RC_PID_Init };
static PidTypeDef pitch_ecd_angle_rc_pid = { PID_POSITION, PITCH_ECD_ANGLE_RC_PID_Init };
//yaw 自动控制PID
static PidTypeDef yaw_ecd_speed_auto_pid = { PID_POSITION, YAW_ECD_SPEED_AUTO_PID_Init };
static PidTypeDef yaw_ecd_angle_auto_pid = { PID_POSITION, YAW_ECD_ANGLE_AUTO_PID_Init };
static PidTypeDef yaw_gyro_speed_auto_pid = { PID_POSITION, YAW_GYRO_SPEED_AUTO_PID_Init };
static PidTypeDef yaw_gyro_angle_auto_pid = { PID_POSITION, YAW_GYRO_ANGLE_AUTO_PID_Init };
//pitch 自动控制PID
static PidTypeDef pitch_ecd_speed_auto_pid = { PID_POSITION, PITCH_ECD_SPEED_AUTO_PID_Init };
static PidTypeDef pitch_ecd_angle_auto_pid = { PID_POSITION, PITCH_ECD_ANGLE_AUTO_PID_Init };
//yaw特殊模式下PID
//小陀螺模式
static PidTypeDef yaw_gyromode_speed_rc_pid = { PID_POSITION, YAW_gyromode_SPEED_RC_PID_Init };
static PidTypeDef yaw_gyromode_angle_rc_pid = { PID_POSITION, YAW_gyromode_ANGLE_RC_PID_Init };
static PidTypeDef yaw_gyromode_speed_auto_pid = { PID_POSITION, YAW_gyromode_SPEED_AUTO_PID_Init };
static PidTypeDef yaw_gyromode_angle_auto_pid = { PID_POSITION, YAW_gyromode_ANGLE_AUTO_PID_Init };
/**************************PID**********************************/
static void Gimbal_Init(void);
static void Gimbal_mode_set(void);
static void Gimbal_Updata(void);
static void Gimbal_Control(void);
static void Gimbal_Send_Voltage(void);

static void Gimbal_Calibration_mode(void);
static void Gimbal_Relax_mode(void);
static void Gimbal_Enconde_mode(fp32 *, fp32 *, uint8_t);
static void Gimbal_Gyro_mode(fp32 *, fp32 *, uint8_t);
	
static void gimbal_mode_change_save(Gimbal_Mode_e, Gimbal_Mode_e);
static void gimbal_rc_process(fp32 *, fp32 *);
static void gimbal_auto_process(fp32 *, fp32 *);

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


//云台初始化
static void Gimbal_Init()
{
	const static fp32 gimbal_pitch_order_filter[1] = {GIMBAL_PITCH_ACCEL_NUM};
	const static fp32 gimbal_yaw_order_filter[1] = {GIMBAL_YAW_ACCEL_NUM};
	
	//滤波初始化
	first_order_filter_init(&gimbal_info.pitch_motor.gimbal_motor_first_OF, GIMBAL_CONTROL_TIME_MS, gimbal_pitch_order_filter);
	first_order_filter_init(&gimbal_info.yaw_motor.gimbal_motor_first_OF, GIMBAL_CONTROL_TIME_MS, gimbal_yaw_order_filter);
	//获取遥控器数据
	gimbal_info.gimbal_RC = get_remote_control_point();
	//获取电机数据
	gimbal_info.yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();
	gimbal_info.pitch_motor.gimbal_motor_measure = get_Pitch_Gimbal_Motor_Measure_Point();
	//更新初始角度
	gimbal_info.pitch_motor.ecd_last = gimbal_info.pitch_motor.gimbal_motor_measure->last_ecd;
	gimbal_info.pitch_motor.ecd_now = gimbal_info.pitch_motor.gimbal_motor_measure->ecd;
	gimbal_info.yaw_motor.ecd_last = gimbal_info.yaw_motor.gimbal_motor_measure->last_ecd;
	gimbal_info.yaw_motor.ecd_now = gimbal_info.yaw_motor.gimbal_motor_measure->ecd;
	//初始化码盘所在正负
	angle_table_init(gimbal_info.pitch_motor.gimbal_motor_measure->ecd, PITCH_ECD_DEL, &gimbal_info.pitch_motor.turn_table_flag);
	angle_table_init(gimbal_info.yaw_motor.gimbal_motor_measure->ecd, YAW_ECD_DEL, &gimbal_info.yaw_motor.turn_table_flag);
	//初始化云台圈数
	gimbal_info.turn_circle_num = 0;
	//获取自瞄数据
	gimbal_info.gimbal_AUTO_ctrl = get_AUTO_control_point();
	Gimbal_Updata();
}
//云台模式选择
static void Gimbal_mode_set(void)
{
	Gimbal_Mode_e mode = GIMBAL_RELAX;
	Gimbal_Behavior_e behavior = GIMBAL_NORMAL;

	//控制方式选择
	if(switch_is_mid(gimbal_info.gimbal_RC->rc.s[CONTROL_MODE_SW]))		//遥控控制
	{
		gimbal_info.ctrl_mode = RC_CTRL;
	}
	else if(switch_is_up(gimbal_info.gimbal_RC->rc.s[CONTROL_MODE_SW]))		//键盘控制
	{
		gimbal_info.ctrl_mode = KEY_CTRL;
	}
	else	//遥控模式下开启摩擦轮
	{
		gimbal_info.ctrl_mode = RC_CTRL;
	}
	//云台模式选择
	if(gimbal_info.ctrl_mode == RC_CTRL)
	{
		if (switch_is_mid(gimbal_info.gimbal_RC->rc.s[GIMBAL_MODE_SW]))
		{
			mode = GIMBAL_ENCONDE;//机械模式
		}
		else if (switch_is_up(gimbal_info.gimbal_RC->rc.s[GIMBAL_MODE_SW]))
		{
			mode = GIMBAL_GYRO;//陀螺仪模式
		}
		else if (switch_is_down(gimbal_info.gimbal_RC->rc.s[GIMBAL_MODE_SW]))
		{
			mode = GIMBAL_RELAX;//云台无力
		}
		gimbal_info.gimbal_behavior = GIMBAL_NORMAL;
	}
	else		//键鼠控制
	{
		//云台模式设置 短按切换
		if(gimbal_info.gimbal_RC->key_data.key_short_press.bit.E)
			mode = GIMBAL_ENCONDE;
		
		//云台行为设置 点击切换
		if(gimbal_info.gimbal_RC->key_data.key_click.bit.Q)	//Q开启小陀螺chassis_info.chassis_RC->key.bit.Q
			behavior = GIMBAL_NORMAL_GR;	//小陀螺
	}
	gimbal_info.gimbal_mode = mode;
	gimbal_info.gimbal_behavior = behavior;
	//模式切换保存数据
	if(gimbal_info.gimbal_mode != gimbal_info.gimbal_mode_last)
	{
		gimbal_mode_change_save(gimbal_info.gimbal_mode_last, gimbal_info.gimbal_mode);
	}
	gimbal_info.gimbal_mode_last = gimbal_info.gimbal_mode;
	gimbal_info.gimbal_behavior_last = gimbal_info.gimbal_behavior;
}
//云台数据更新
static void Gimbal_Updata(void)
{
	const fp32 *gimbal_INT_angle_point = get_INS_angle_point();//获取陀螺仪姿态解算后的数据 角度
	const fp32 *gimbal_INT_gyro_point = get_MPU6500_Gyro_Data_Point();//获取陀螺仪原始数据 角速度
	
	if(gimbal_info.turn_circle_num == 32767 || gimbal_info.turn_circle_num == -32768)
	{
		gimbal_info.yaw_motor.absolute_angle -= gimbal_info.turn_circle_num * CIRCLE;
		gimbal_info.yaw_motor.absolute_angle_last -= gimbal_info.turn_circle_num * CIRCLE;
		gimbal_info.turn_circle_num = 0;
	}
	//初始化机械角度
	gimbal_info.pitch_motor.ecd_now = ecd_angle_format(gimbal_info.pitch_motor.gimbal_motor_measure->ecd, gimbal_info.pitch_motor.ecd_last, PITCH_ECD_DEL, &gimbal_info.pitch_motor.turn_table_flag);
	gimbal_info.yaw_motor.ecd_now = ecd_angle_format( gimbal_info.yaw_motor.gimbal_motor_measure->ecd, gimbal_info.yaw_motor.ecd_last, YAW_ECD_DEL, &gimbal_info.yaw_motor.turn_table_flag);
	//初始化陀螺仪角度
	gimbal_info.yaw_motor.gyro_now = gyro_angle_format(gimbal_info.yaw_motor.gyro_last, *(gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET), &gimbal_info.yaw_motor.turn_table_flag);
	gimbal_info.pitch_motor.gyro_now = gyro_angle_format(gimbal_info.pitch_motor.gyro_last, *(gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET), &gimbal_info.pitch_motor.turn_table_flag);

	//角度更新 外环
	if(gimbal_info.gimbal_mode == GIMBAL_ENCONDE)
	{
		fp32 angle_ecd_Y = 0.0f;	//临时变量
		//上一次角度更新  外环
		gimbal_info.pitch_motor.relative_angle_last = gimbal_info.pitch_motor.relative_angle;
		gimbal_info.yaw_motor.relative_angle_last = gimbal_info.yaw_motor.relative_angle;
		//相对角度更新(relative angle) yaw轴需要计算圈数(rad)
		gimbal_info.pitch_motor.relative_angle = gimbal_info.pitch_motor.ecd_now;
		angle_ecd_Y = gimbal_info.yaw_motor.ecd_now + gimbal_info.turn_circle_num * 2 * PI;
		gimbal_info.yaw_motor.relative_angle = calc_turn_angle(gimbal_info.yaw_motor.relative_angle_last, angle_ecd_Y, &gimbal_info.turn_circle_num);
		//角速度更新 内环
		gimbal_info.pitch_motor.speed = gimbal_info.pitch_motor.gimbal_motor_measure->speed_rpm * Ecd_to_Rad;	//电机反馈转速值rpm
		gimbal_info.yaw_motor.speed = gimbal_info.yaw_motor.gimbal_motor_measure->speed_rpm * Ecd_to_Rad;
	}
	else if(gimbal_info.gimbal_mode == GIMBAL_GYRO)
	{
		fp32 angle_gyro_Y = 0.0f;	//临时变量
		//上一次角度更新  外环
		gimbal_info.pitch_motor.absolute_angle_last = gimbal_info.pitch_motor.absolute_angle;
		gimbal_info.yaw_motor.absolute_angle_last = gimbal_info.yaw_motor.absolute_angle;
		//绝对角度计算(absolute angle) yaw轴需要计算圈数(rad)
		angle_gyro_Y = gimbal_info.yaw_motor.gyro_now + gimbal_info.turn_circle_num * 2 *PI;
		gimbal_info.pitch_motor.absolute_angle = gimbal_info.pitch_motor.ecd_now;
		gimbal_info.yaw_motor.absolute_angle = calc_turn_angle(gimbal_info.yaw_motor.absolute_angle_last, angle_gyro_Y, &gimbal_info.turn_circle_num);
		//角速度更新 内环
		gimbal_info.yaw_motor.speed = *(gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET);		//陀螺仪反馈转速rpm
		gimbal_info.pitch_motor.speed = *(gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);
	}
	else//需修改
	{
		gimbal_info.yaw_motor.speed = gimbal_info.yaw_motor.speed_set;
		gimbal_info.pitch_motor.speed = gimbal_info.pitch_motor.speed_set;
	}
	//更新上一次原始角度
	gimbal_info.yaw_motor.ecd_last = gimbal_info.yaw_motor.gimbal_motor_measure->ecd;
	gimbal_info.pitch_motor.ecd_last = gimbal_info.pitch_motor.gimbal_motor_measure->ecd;
	gimbal_info.yaw_motor.gyro_last = *(gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
	gimbal_info.pitch_motor.gyro_last = *(gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);
	//电压更新
	gimbal_info.yaw_motor.voltage_give_last = gimbal_info.yaw_motor.voltage_give;
	gimbal_info.pitch_motor.voltage_give_last = gimbal_info.pitch_motor.voltage_give;
}
//控制量输入
static void Gimbal_Control(void)
{
	fp32 add_yaw_angle = 0.0f;
	fp32 add_pitch_angle = 0.0f;
	fp32 rc_yaw_ch, rc_pitch_ch, auto_yaw_ch, auto_pitch_ch;
	uint8_t vision_flag = 0;
	//处理遥控及鼠标数据
	gimbal_rc_process(&rc_yaw_ch, &rc_pitch_ch);
	//处理自瞄数据
	gimbal_auto_process(&auto_yaw_ch, &auto_pitch_ch);
	//开启自瞄并识别到目标
	//vision_flag = 
	if(gimbal_info.gimbal_behavior == GIMBAL_AUTO || vision_flag)
	{
		add_yaw_angle = auto_yaw_ch;
		add_pitch_angle = auto_pitch_ch;
	}
	else
	{
		add_yaw_angle = rc_yaw_ch;
		add_pitch_angle = rc_pitch_ch;
	}
	//选择控制量输入 角度叠加量
	switch(gimbal_info.gimbal_mode)
	{
		case GIMBAL_CALI:		//校准模式 暂未实现
		{
			Gimbal_Calibration_mode();
			return;
		}
		case GIMBAL_RELAX:		//失能模式
		{
			Gimbal_Relax_mode();
			break;
		}
		case GIMBAL_GYRO:		//陀螺仪控制模式
		{
			Gimbal_Gyro_mode(&add_pitch_angle, &add_yaw_angle, vision_flag);
			break;
		}
		case GIMBAL_ENCONDE:	//机械控制模式
		{
			Gimbal_Enconde_mode(&add_pitch_angle, &add_yaw_angle, vision_flag);
			break;
		}
	}
	//回中处理
	if( !gimbal_info.turn_mid_flag )
	{
		//死区处理
		if(gimbal_info.pitch_motor.relative_angle > 0.1f || gimbal_info.pitch_motor.relative_angle < -0.1f)
		{
			add_pitch_angle = gimbal_info.pitch_motor.relative_angle < 0.0f ? GIMBAL_RETURN_PITCH : -GIMBAL_RETURN_PITCH;		//先抬起pitch轴
			add_yaw_angle = 0.0f;
		}
		else if(gimbal_info.yaw_motor.relative_angle > 0.1f || gimbal_info.yaw_motor.relative_angle < -0.1f)
		{
			add_pitch_angle = 0.0f;
			add_yaw_angle = gimbal_info.yaw_motor.relative_angle < 0.0f ? GIMBAL_RETURN_YAW : -GIMBAL_RETURN_YAW;
		}
		else	//已回中
		{
			gimbal_info.turn_mid_flag = 1;
		}
		//角度叠加
		gimbal_info.pitch_motor.relative_angle_set += add_pitch_angle;
		gimbal_info.yaw_motor.relative_angle_set += add_yaw_angle;
	}
	//电压
	gimbal_info.yaw_motor.voltage_give = (int16_t)gimbal_info.yaw_motor.voltage_set;
	gimbal_info.pitch_motor.voltage_give = (int16_t)gimbal_info.pitch_motor.voltage_set;
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
	gimbal_info.pitch_motor.relative_angle_set = gimbal_info.pitch_motor.relative_angle;//0.0f;
	gimbal_info.pitch_motor.ecd_last = gimbal_info.pitch_motor.ecd_now;
	gimbal_info.pitch_motor.voltage_set = 0.0f;
	gimbal_info.pitch_motor.voltage_give = 0.0f;
	//yaw
	gimbal_info.yaw_motor.speed_set = 0.0f;
	gimbal_info.yaw_motor.relative_angle_set = gimbal_info.yaw_motor.relative_angle;//0.0f;
	gimbal_info.yaw_motor.ecd_last = gimbal_info.yaw_motor.ecd_now;
	gimbal_info.yaw_motor.voltage_set = 0.0f;
	gimbal_info.yaw_motor.voltage_give = 0.0f;
	//回中标志清0
	gimbal_info.turn_mid_flag = 0;
	gimbal_info.turn_circle_num = 0;
}
//云台由电机编码器控制
static void Gimbal_Enconde_mode(fp32 *pitch_add, fp32 *yaw_add, uint8_t vision_flag)
{
	gimbal_info.pitch_motor.relative_angle_set += *pitch_add;
	gimbal_info.yaw_motor.relative_angle_set += *yaw_add;
	switch(gimbal_info.gimbal_behavior)
	{
		case GIMBAL_NORMAL:
			gimbal_info.pitch_motor.speed_set = PID_Calc(&pitch_ecd_speed_rc_pid, gimbal_info.pitch_motor.relative_angle, gimbal_info.pitch_motor.relative_angle_set);
			gimbal_info.pitch_motor.voltage_set = PID_Calc(&pitch_ecd_angle_rc_pid, gimbal_info.pitch_motor.speed, gimbal_info.pitch_motor.speed_set);
			gimbal_info.yaw_motor.speed_set = PID_Calc(&yaw_ecd_angle_rc_pid, gimbal_info.yaw_motor.relative_angle, gimbal_info.yaw_motor.relative_angle_set);
			gimbal_info.yaw_motor.voltage_set = PID_Calc(&yaw_ecd_angle_rc_pid, gimbal_info.yaw_motor.speed, gimbal_info.yaw_motor.speed_set);
			break;
		case GIMBAL_AUTO:
			//识别到目标 PID切换为自瞄
			if(vision_flag)
			{
				gimbal_info.pitch_motor.speed_set = PID_Calc(&pitch_ecd_speed_auto_pid, gimbal_info.pitch_motor.relative_angle, gimbal_info.pitch_motor.relative_angle_set);
				gimbal_info.pitch_motor.voltage_set = PID_Calc(&pitch_ecd_angle_auto_pid, gimbal_info.pitch_motor.speed, gimbal_info.pitch_motor.speed_set);
				gimbal_info.yaw_motor.speed_set = PID_Calc(&yaw_ecd_angle_auto_pid, gimbal_info.yaw_motor.relative_angle, gimbal_info.yaw_motor.relative_angle_set);
				gimbal_info.yaw_motor.voltage_set = PID_Calc(&yaw_ecd_angle_auto_pid, gimbal_info.yaw_motor.speed, gimbal_info.yaw_motor.speed_set);
			}
			else
			{
				gimbal_info.pitch_motor.speed_set = PID_Calc(&pitch_ecd_speed_rc_pid, gimbal_info.pitch_motor.relative_angle, gimbal_info.pitch_motor.relative_angle_set);
				gimbal_info.pitch_motor.voltage_set = PID_Calc(&pitch_ecd_angle_rc_pid, gimbal_info.pitch_motor.speed, gimbal_info.pitch_motor.speed_set);
				gimbal_info.yaw_motor.speed_set = PID_Calc(&yaw_ecd_angle_rc_pid, gimbal_info.yaw_motor.relative_angle, gimbal_info.yaw_motor.relative_angle_set);
				gimbal_info.yaw_motor.voltage_set = PID_Calc(&yaw_ecd_angle_rc_pid, gimbal_info.yaw_motor.speed, gimbal_info.yaw_motor.speed_set);
			}
			break;
		case GIMBAL_CENTER:	//云台yaw回中 与初始回中有区别
			//如果角度在回中范围内 该行为有效
			if(gimbal_info.yaw_motor.relative_angle < 1.5f && gimbal_info.yaw_motor.relative_angle > -1.5f)
			{
				if(gimbal_info.yaw_motor.relative_angle < 0.1f && gimbal_info.yaw_motor.relative_angle > -0.1f)
				{
					gimbal_info.gimbal_behavior = GIMBAL_NORMAL;
				}
				gimbal_info.pitch_motor.relative_angle_set += 0.0f;
				gimbal_info.yaw_motor.relative_angle_set += 0.1f;
				gimbal_info.pitch_motor.speed_set = PID_Calc(&pitch_ecd_speed_rc_pid, gimbal_info.pitch_motor.relative_angle, gimbal_info.pitch_motor.relative_angle_set);
				gimbal_info.pitch_motor.voltage_set = PID_Calc(&pitch_ecd_angle_rc_pid, gimbal_info.pitch_motor.speed, gimbal_info.pitch_motor.speed_set);
				gimbal_info.yaw_motor.speed_set = PID_Calc(&yaw_ecd_angle_rc_pid, gimbal_info.yaw_motor.relative_angle, gimbal_info.yaw_motor.relative_angle_set);
				gimbal_info.yaw_motor.voltage_set = PID_Calc(&yaw_ecd_angle_rc_pid, gimbal_info.yaw_motor.speed, gimbal_info.yaw_motor.speed_set);
			}
			else	//否则无效
			{
				gimbal_info.gimbal_behavior = GIMBAL_NORMAL;
			}
			break;
		case GIMBAL_TURN:
			
			break;
		case GIMBAL_STOP:
			gimbal_info.pitch_motor.relative_angle_set += 0;
			gimbal_info.yaw_motor.relative_angle_set += 0;
			break;
		//该行为只在陀螺仪模式下生效
		case GIMBAL_NORMAL_GR:
		case GIMBAL_AUTO_GR:
			gimbal_info.gimbal_behavior = gimbal_info.gimbal_behavior_last;
			break;
	}
}
//云台由陀螺仪控制
static void Gimbal_Gyro_mode(fp32 *pitch_add, fp32 *yaw_add, uint8_t vision_flag)
{
	gimbal_info.pitch_motor.absolute_angle_set += *pitch_add;
	gimbal_info.yaw_motor.absolute_angle_set += *yaw_add;
	switch(gimbal_info.gimbal_behavior)
	{
		case GIMBAL_NORMAL:
			gimbal_info.pitch_motor.speed_set = PID_Calc(&pitch_ecd_speed_rc_pid, gimbal_info.pitch_motor.absolute_angle, gimbal_info.pitch_motor.absolute_angle_set);
			gimbal_info.pitch_motor.voltage_set = PID_Calc(&pitch_ecd_angle_rc_pid, gimbal_info.pitch_motor.speed, gimbal_info.pitch_motor.speed_set);
			gimbal_info.yaw_motor.speed_set = PID_Calc(&yaw_gyro_angle_rc_pid, gimbal_info.yaw_motor.absolute_angle, gimbal_info.yaw_motor.absolute_angle_set);
			gimbal_info.yaw_motor.voltage_set = PID_Calc(&yaw_gyro_angle_rc_pid, gimbal_info.yaw_motor.speed, gimbal_info.yaw_motor.speed_set);
			break;
		case GIMBAL_AUTO:
			//识别到目标 PID切换为自瞄
			if(vision_flag)
			{
				gimbal_info.pitch_motor.speed_set = PID_Calc(&pitch_ecd_speed_auto_pid, gimbal_info.pitch_motor.absolute_angle, gimbal_info.pitch_motor.absolute_angle_set);
				gimbal_info.pitch_motor.voltage_set = PID_Calc(&pitch_ecd_angle_auto_pid, gimbal_info.pitch_motor.speed, gimbal_info.pitch_motor.speed_set);
				gimbal_info.yaw_motor.speed_set = PID_Calc(&yaw_gyro_angle_auto_pid, gimbal_info.yaw_motor.absolute_angle, gimbal_info.yaw_motor.absolute_angle_set);
				gimbal_info.yaw_motor.voltage_set = PID_Calc(&yaw_gyro_angle_auto_pid, gimbal_info.yaw_motor.speed, gimbal_info.yaw_motor.speed_set);
			}
			else
			{
				gimbal_info.pitch_motor.speed_set = PID_Calc(&pitch_ecd_speed_rc_pid, gimbal_info.pitch_motor.absolute_angle, gimbal_info.pitch_motor.absolute_angle_set);
				gimbal_info.pitch_motor.voltage_set = PID_Calc(&pitch_ecd_angle_rc_pid, gimbal_info.pitch_motor.speed, gimbal_info.pitch_motor.speed_set);
				gimbal_info.yaw_motor.speed_set = PID_Calc(&yaw_gyro_angle_rc_pid, gimbal_info.yaw_motor.absolute_angle, gimbal_info.yaw_motor.absolute_angle_set);
				gimbal_info.yaw_motor.voltage_set = PID_Calc(&yaw_gyro_angle_rc_pid, gimbal_info.yaw_motor.speed, gimbal_info.yaw_motor.speed_set);
			}
			break;
		case GIMBAL_NORMAL_GR:			//小陀螺模式下的手动模式
			gimbal_info.pitch_motor.speed_set = PID_Calc(&pitch_ecd_speed_rc_pid, gimbal_info.pitch_motor.relative_angle, gimbal_info.pitch_motor.relative_angle_set);
			gimbal_info.pitch_motor.voltage_set = PID_Calc(&pitch_ecd_angle_rc_pid, gimbal_info.pitch_motor.speed, gimbal_info.pitch_motor.speed_set);
			gimbal_info.yaw_motor.speed_set = PID_Calc(&yaw_gyromode_speed_rc_pid, gimbal_info.yaw_motor.relative_angle, gimbal_info.yaw_motor.relative_angle_set);
			gimbal_info.yaw_motor.voltage_set = PID_Calc(&yaw_gyromode_angle_rc_pid, gimbal_info.yaw_motor.speed, gimbal_info.yaw_motor.speed_set);
			break;
		case GIMBAL_AUTO_GR:			//小陀螺模式下的自瞄模式
			gimbal_info.pitch_motor.speed_set = PID_Calc(&pitch_ecd_speed_auto_pid, gimbal_info.pitch_motor.relative_angle, gimbal_info.pitch_motor.relative_angle_set);
			gimbal_info.pitch_motor.voltage_set = PID_Calc(&pitch_ecd_angle_auto_pid, gimbal_info.pitch_motor.speed, gimbal_info.pitch_motor.speed_set);
			gimbal_info.yaw_motor.speed_set = PID_Calc(&yaw_gyromode_speed_auto_pid, gimbal_info.yaw_motor.relative_angle, gimbal_info.yaw_motor.relative_angle_set);
			gimbal_info.yaw_motor.voltage_set = PID_Calc(&yaw_gyromode_angle_auto_pid, gimbal_info.yaw_motor.speed, gimbal_info.yaw_motor.speed_set);			break;
		case GIMBAL_STOP:
			gimbal_info.pitch_motor.absolute_angle_set += 0.0f;
			gimbal_info.yaw_motor.absolute_angle_set += 0.0f;
			break;
		case GIMBAL_TURN:
		case GIMBAL_CENTER:
			//在此模式下为底盘回中 在此模式下该行为不执行
			gimbal_info.gimbal_behavior = gimbal_info.gimbal_behavior_last;
			break;
	}
}
//模式改变处理数据
static void gimbal_mode_change_save(Gimbal_Mode_e last, Gimbal_Mode_e now)
{
	//处理yaw圈数
	
	//数据切换更新
	
}
//遥控器数据处理
static void gimbal_rc_process(fp32 *yaw_add, fp32 *pitch_add)
{
	//初始化相关变量
	int16_t yaw_channel = 0;
	int16_t pitch_channel = 0;
	fp32 mouse_x_channel = 0.0f;
	fp32 mouse_y_channel = 0.0f;
	//遥控器死区处理
//	if( gimbal_info.gimbal_RC->rc.ch[GIMBAL_YAW_CHANNEL] > RC_DEADLINE || gimbal_info.gimbal_RC->rc.ch[GIMBAL_YAW_CHANNEL] < -RC_DEADLINE )
//		yaw_channel = gimbal_info.gimbal_RC->rc.ch[GIMBAL_YAW_CHANNEL];
//	if( gimbal_info.gimbal_RC->rc.ch[GIMBAL_PITCH_CHANNEL] > RC_DEADLINE || gimbal_info.gimbal_RC->rc.ch[GIMBAL_PITCH_CHANNEL] < -RC_DEADLINE )
//		pitch_channel = gimbal_info.gimbal_RC->rc.ch[GIMBAL_PITCH_CHANNEL];
	i_dead_zone_del(gimbal_info.gimbal_RC->rc.ch[GIMBAL_YAW_CHANNEL], &yaw_channel, RC_DEADLINE);
	//鼠标数据处理 滤波处理?
	mouse_x_channel = gimbal_info.gimbal_RC->mouse.x * PITCH_MOUSE_SEN;
	mouse_y_channel = gimbal_info.gimbal_RC->mouse.y * YAW_MOUSE_SEN;
	//遥控数据比例转换
	*yaw_add = yaw_channel * YAW_RC_SEN + mouse_x_channel;
	*pitch_add = pitch_channel * PITCH_RC_SEN + mouse_y_channel;
}
//自瞄数据处理
static void gimbal_auto_process(fp32 *yaw_add, fp32 *pitch_add)
{
	fp32 auto_yaw_channel = 0.0f;
	fp32 auto_pitch_channel = 0.0f;
	//需要实际对数据进行调整 加减偏移数据
	auto_yaw_channel = gimbal_info.gimbal_AUTO_ctrl->yaw_angle * YAW_AUTO_SEN;
	auto_pitch_channel = gimbal_info.gimbal_AUTO_ctrl->pitch_angle * PITCH_AUTO_SEN;
	*yaw_add = auto_yaw_channel;
	*pitch_add = auto_pitch_channel;
}
//发送控制电压
static void Gimbal_Send_Voltage(void)
{
	//电机是否反装
#if YAW_TURN
	Yaw_Can_Set_Voltage = -gimbal_info.yaw_motor.voltage_give;
#else
	Yaw_Can_Set_Voltage = gimbal_info.yaw_motor.voltage_give;
#endif

#if PITCH_TURN
	Pitch_Can_Set_Voltage = -gimbal_info.pitch_motor.voltage_give;
#else
	Pitch_Can_Set_Voltage = gimbal_info.pitch_motor.voltage_give;
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
			//CAN_CMD_GIMBAL(0, Pitch_Can_Set_Voltage, Shoot_Can_Set_Voltage, 0);
			//CAN_CMD_GIMBAL(Yaw_Can_Set_Voltage, Pitch_Can_Set_Voltage, Shoot_Can_Set_Voltage, 0);
			CAN_CMD_GIMBAL(0, 0, 0, 0);
		}
	}
}
const Gimbal_Motor_s *get_gimbal_yaw_motor_point(void)
{
    return &gimbal_info.yaw_motor;
}
const Gimbal_Motor_s *get_gimbal_pitch_motor_point(void)
{
    return &gimbal_info.pitch_motor;
}
fp32 get_gimbal_relative_angle(void)
{
	return gimbal_info.yaw_motor.ecd_now;
	//return (gimbal_info.yaw_motor.relative_angle - gimbal_info.turn_circle_num * 2 * PI);
}


