#include "test.h"

#include "chassis_task.h"

#include "rc.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "arm_math.h"

#include "Detect_Task.h"


#include "keyboard.h"

//底盘信息
static Chassis_Control_s chassis_info;

static void Chassis_Init(void);
static void Chassis_Mode_Set(void);
static void Chassis_Updata(void);
static void Chassis_Control(void);
static void Chassis_Calc_Current(void);
static void Chassis_Send_Current(void);

static void Chassis_Relax_Mode(void);
static void Chassis_Follow_Yaw_Mode(fp32 *, fp32 *, int16_t *);
static void Chassis_No_Follow_Mode(fp32 *, fp32 *, int16_t *);
static void Chassis_Gyro_Mode(fp32 *, fp32 *, int16_t *);
static void Chassis_Cali_Mode(void);
static void Chassis_Stop_Mode(void);

static void chassis_RC_ctrl(fp32 *vx_ch, fp32 *vy_ch, int16_t *wz_ch);
static void chassis_mode_change_save(Chassis_Mode_e , Chassis_Mode_e );
static fp32 chassis_absolute_angle_process(const fp32, const fp32);
static fp32 chassis_calc_turn_angle(const fp32 *, fp32 *);
//遥控死区处理
#define RC_Dead_Zone_Del(input, output, deal)		\
	{												\
		if ((input) > (deal) || (input) < -(deal))	\
		{											\
			(output) = (input);						\
		}											\
		else										\
		{											\
			(output) = 0;							\
		}											\
	}
//角度规整 -PI~PI--> -2PI~2PI
#define angle_format(angle_now, angle_last)			\
	{												\
		if((angle_now) - (angle_last) < -PI)		\
			(angle_now) += 2*PI;					\
		else if((angle_now) - (angle_last) > PI)	\
			(angle_now) -= 2*PI;					\
	}

void chassis_task(void *pvParameters)
{
	//空闲一段时间
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	//底盘初始化
	Chassis_Init();
	//判断底盘电机是否都在线
	while (toe_is_error(ChassisMotor1TOE) || toe_is_error(ChassisMotor2TOE) || toe_is_error(ChassisMotor3TOE) || toe_is_error(ChassisMotor4TOE) || toe_is_error(DBUSTOE))
	{
		vTaskDelay(CHASSIS_CONTROL_TIME_MS);
	}
	
	while(1)
	{
		//底盘模式更新
		Chassis_Mode_Set();
		//底盘数据更新
		Chassis_Updata();
		//底盘底盘控制量计算
		Chassis_Control();
		//底盘电机电流计算
		Chassis_Calc_Current();
		//底盘电机电流输出
		Chassis_Send_Current();
	}
}
//底盘初始化(*)
static void Chassis_Init(void)
{
	uint8_t i = 0;
	//底盘速度环pid值
	const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
	//底盘角度环pid值
	const static fp32 chassis_yaw_pid[3] = {CHASSIS_ANGLE_PID_KP, CHASSIS_ANGLE_PID_KI, CHASSIS_ANGLE_PID_KD};
	//滤波系数
	const static fp32 chassis_x_order_filter[1] = {CHASSIS_X_ACCEL_NUM};
	const static fp32 chassis_y_order_filter[1] = {CHASSIS_Y_ACCEL_NUM};
	for(i = 0; i < 4; i++)
	{
		//电机反馈数据获取
		chassis_info.chassis_motor[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
		//PID初始化
		PID_Init(&chassis_info.chassis_motor[i].motor_speed_pid, PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
	}
	//底盘角度环PID初始化
	PID_Init(&chassis_info.chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_ANGLE_PID_MAX_OUT, CHASSIS_ANGLE_PID_MAX_IOUT);
	//底盘模式初始化
	chassis_info.chassis_mode = CHASSIS_RELAX;
	//云台yaw轴电机数据获取，用于计算底盘绝对角度
	chassis_info.yaw_motor_gimbal = get_yaw_motor_point();
	//滤波初始化
    first_order_filter_init(&chassis_info.chassis_vx_first_OF, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_info.chassis_vy_first_OF, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
	//遥控器数据初始化
	chassis_info.chassis_RC = get_remote_control_point();
	//陀螺仪数据获取
	chassis_info.ins_data = get_INS_Data_point();
	//底盘电机反馈数据初始化
	for(i = 0; i < 4; i++)
		chassis_info.chassis_motor[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
	//底盘更新
	Chassis_Updata();
}
//底盘模式更新(*)
static void Chassis_Mode_Set(void)
{
	Chassis_Mode_e mode = CHASSIS_RELAX;
	uint8_t gyro_test = 0;		//测试
	//控制方式选择
	if(switch_is_mid(chassis_info.chassis_RC->rc.s[CONTROL_MODE_SW]))		//遥控控制
	{
		chassis_info.ctrl_mode = RC_CTRL;
	}
	else if(switch_is_up(chassis_info.chassis_RC->rc.s[CONTROL_MODE_SW]))		//键盘控制
	{
		chassis_info.ctrl_mode = KEY_CTRL;
	}
	else	//遥控模式下开启摩擦轮
	{
		chassis_info.ctrl_mode = RC_CTRL;
	}
	//底盘模式选择
	if(chassis_info.ctrl_mode == RC_CTRL)
	{
		if(switch_is_up(chassis_info.chassis_RC->rc.s[CHASSIS_MODE_SW]))
		{
			mode = CHASSIS_NO_FOLLOW;	//底盘不跟随云台
		}
		else if (switch_is_mid(chassis_info.chassis_RC->rc.s[CHASSIS_MODE_SW]))	//中间为遥控
		{
			mode = CHASSIS_FOLLOW_YAW;		//底盘跟随云台
		}
		else if (switch_is_down(chassis_info.chassis_RC->rc.s[CHASSIS_MODE_SW]))	//下拨为无力
		{
			mode = CHASSIS_RELAX;		//底盘无力
		}
	}
	else
	{
		if(key_status.bit.Q || gyro_test)	//Q开启小陀螺chassis_info.chassis_RC->key.bit.Q
		{
			mode = CHASSIS_GYRO;	//小陀螺
		}
	}
	chassis_info.chassis_mode = mode;
	//模式切换保存数据
	if(chassis_info.chassis_mode != chassis_info.chassis_last_mode)
	{
		chassis_mode_change_save(chassis_info.chassis_last_mode, chassis_info.chassis_mode);
	}
	chassis_info.chassis_last_mode = chassis_info.chassis_mode;
}
//计算底盘圈数和实际角度
static fp32 chassis_calc_turn_angle(const fp32 *angle_last, fp32 *angle_now)
{
	//正向圈数为正 逆时针
	if(*angle_now - *angle_last < -4.0f)
	{
		chassis_info.chassis_circle_num++;
		*angle_now += 2 * PI;
	}
	//反向圈数为负 顺时针
	else if(*angle_now - *angle_last > 4.0f)
	{
		chassis_info.chassis_circle_num--;
		*angle_now -= 2 * PI;
	}
	return *angle_now;
}
//计算底盘绝对角度
//static fp32 chassis_absolute_angle_process(const fp32 relative_angle, const fp32 gyro_angle)
//{
//	fp32 absolute_angle;
//	absolute_angle = gyro_angle - relative_angle;
//	return absolute_angle;
//}
//底盘数据更新 (*)
static void Chassis_Updata(void)
{
	const fp32 *chassis_INT_angle_point = get_INS_angle_point();//获取陀螺仪姿态解算后的数据 角度
	fp32 gyro_yaw_angle = *(chassis_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
	static fp32 gyro_yaw_last_angle;
	uint8_t i;
	//处理陀螺仪角度范围为-2PI~2PI
	angle_format(gyro_yaw_angle, gyro_yaw_last_angle);
	//更新电机速度
	for(i = 0; i < 4; i++)
	{
		chassis_info.chassis_motor[i].speed = chassis_info.chassis_motor[i].chassis_motor_measure->speed_rpm * CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
		chassis_info.chassis_motor[i].accel = chassis_info.chassis_motor[i].motor_speed_pid.Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
	}
	//更新底盘速度
	chassis_info.chassis_vx = (chassis_info.chassis_motor[0].speed - chassis_info.chassis_motor[1].speed + chassis_info.chassis_motor[2].speed - chassis_info.chassis_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
	chassis_info.chassis_vy = (-chassis_info.chassis_motor[0].speed - chassis_info.chassis_motor[1].speed + chassis_info.chassis_motor[2].speed + chassis_info.chassis_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
	chassis_info.chassis_vx = (-chassis_info.chassis_motor[0].speed - chassis_info.chassis_motor[1].speed - chassis_info.chassis_motor[2].speed - chassis_info.chassis_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
	//更新底盘与云台的相对角度 即为云台电机的相对角度
	chassis_info.chassis_relative_angle = chassis_info.yaw_motor_gimbal->relative_angle;	
	//更新底盘绝对角度 此处更新角度是原始角度 范围是-2PI~2PI 没有计算圈数叠加角度
	chassis_info.chassis_absolute_yaw = gyro_yaw_angle - chassis_info.chassis_relative_angle;//陀螺仪角度-云台相对角度(电机传感器提供机械角度)
	chassis_info.chassis_last_absolute_yaw = chassis_info.chassis_absolute_yaw;
	//计算圈数并叠加圈数角度
	chassis_calc_turn_angle(&chassis_info.chassis_last_yaw, &chassis_info.chassis_yaw);
	chassis_info.chassis_last_yaw = chassis_info.chassis_yaw;
	//更新陀螺仪上一次角度 用于计算处理陀螺仪边界值
	gyro_yaw_last_angle = gyro_yaw_angle;
}
//底盘控制量设置(*)
static void Chassis_Control(void)
{
	int16_t rc_vw_channel, key_vw_channel, vw_set_channel;
    fp32 rc_vx_channel, rc_vy_channel;
	fp32 key_vx_channel, key_vy_channel;
    fp32 vx_set_channel, vy_set_channel;

	//遥控输入处理
	chassis_RC_ctrl(&rc_vx_channel, &rc_vy_channel, &rc_vw_channel);
	//键盘输入处理
	chassis_key_ctrl(chassis_info.chassis_RC, &key_vx_channel, &key_vy_channel);
	//底盘速度输入
	vx_set_channel = rc_vx_channel + key_vx_channel;
	vy_set_channel = rc_vy_channel + key_vy_channel;
	vw_set_channel = rc_vw_channel;// + key_vw_channel;
	//选择模式
	switch(chassis_info.chassis_mode)
	{
		case CHASSIS_RELAX:
		{
			Chassis_Relax_Mode();
			break;
		}
		case CHASSIS_FOLLOW_YAW:
		{
			Chassis_Follow_Yaw_Mode(&vx_set_channel, &vy_set_channel, &vw_set_channel);
			break;
		}
		case CHASSIS_NO_FOLLOW:
		{
			Chassis_No_Follow_Mode(&vx_set_channel, &vy_set_channel, &vw_set_channel);
			break;
		}
		case CHASSIS_GYRO:
		{
			Chassis_Gyro_Mode(&vx_set_channel, &vy_set_channel, &vw_set_channel);
			break;
		}
		case CHASSIS_CALI:
		{
			Chassis_Cali_Mode();
			break;
		}
		case CHASSIS_STOP:
		{
			Chassis_Stop_Mode();
			break;
		}
	}
}
//底盘无力模式
static void Chassis_Relax_Mode(void)
{
	chassis_info.chassis_vx_set = 0.0f;
	chassis_info.chassis_vy_set = 0.0f;
	chassis_info.chassis_vw_set = 0.0f;
	chassis_info.chassis_absolute_yaw_set = chassis_info.chassis_absolute_yaw;
	//chassis_info.chassis_relative_angle_set = chassis_info.chassis_relative_angle;
}
//底盘跟随云台模式 前进方向由云台决定(*)
static void Chassis_Follow_Yaw_Mode(fp32 *vx_ch, fp32 *vy_ch, int16_t *vw_ch)
{
	fp32 relative_angle_sin, relative_angle_cos;//相对角度的正弦余弦值 sin(relative_angle) cos(relative_angle)
	relative_angle_sin = arm_sin_f32(chassis_info.chassis_relative_angle);
	relative_angle_cos = arm_cos_f32(chassis_info.chassis_relative_angle);
	//云台指向前进平移方向的vx、vy分解成底盘车头指向方向的vx、vy 
	chassis_info.chassis_vx_set = *vx_ch * relative_angle_cos - *vy_ch * relative_angle_cos;	//vx*cos(a)-vy*sin(a)
	chassis_info.chassis_vy_set = *vx_ch * relative_angle_sin + *vy_ch * relative_angle_sin;	//vx*sin(a)+vy*cos(a)
	//底盘绝对角度 叠加旋转角度 该模式下vw通道输入值处理为角度rad(-2PI,2PI) (需要修改)
	chassis_info.chassis_absolute_yaw_set = chassis_info.chassis_absolute_yaw + *vw_ch * CHASSIS_ANGLE_Z_RC_SEN;
	//内环PID计算旋转线速度
	chassis_info.chassis_vw_set = PID_Calc(&chassis_info.chassis_angle_pid, chassis_info.chassis_absolute_yaw, chassis_info.chassis_absolute_yaw_set);
}
//底盘不跟随云台模式 前进方向由底盘决定
static void Chassis_No_Follow_Mode(fp32 *vx_ch, fp32 *vy_ch, int16_t *vw_ch)
{
	int16_t rc_vw_channel;
	//遥控器死区去除
	RC_Dead_Zone_Del(chassis_info.chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL], rc_vw_channel, CHASSIS_RC_DEADLINE);
	//处理旋转角度 增加一定角度
	*vw_ch = rc_vw_channel * RC_CHASSIS_VW_SEN;
	//chassis_info.chassis_absolute_yaw_set += *vw_ch;
	//速度限幅
	chassis_info.chassis_vx_set = fp32_constrain(*vx_ch, -MAX_CHASSIS_SPEED_X, MAX_CHASSIS_SPEED_X);
	chassis_info.chassis_vy_set = fp32_constrain(*vy_ch, -MAX_CHASSIS_SPEED_Y, MAX_CHASSIS_SPEED_Y);
	//内环计算旋转角速度
	//chassis_info.chassis_vw_set = PID_Calc(&chassis_info.chassis_angle_pid, 0.0f, *vw_ch);
	chassis_info.chassis_vw_set = *vw_ch;
}
//底盘小陀螺模式
static void Chassis_Gyro_Mode(fp32 *vx_ch, fp32 *vy_ch, int16_t *vw_ch)
{
	
}
//底盘校准模式
static void Chassis_Cali_Mode(void)
{
	
}
//底盘停止状态
static void Chassis_Stop_Mode(void)
{
	
}
//遥控控制模式(*)
static void chassis_RC_ctrl(fp32 *vx_ch, fp32 *vy_ch, int16_t *vw_ch)
{
	int16_t rc_vx_channel, rc_vy_channel, rc_wz_channel;
	//遥控死区处理
	if(chassis_info.chassis_RC->rc.ch[CHASSIS_X_CHANNEL] > CHASSIS_RC_DEADLINE || chassis_info.chassis_RC->rc.ch[CHASSIS_X_CHANNEL] < -CHASSIS_RC_DEADLINE)
		rc_vx_channel = chassis_info.chassis_RC->rc.ch[CHASSIS_X_CHANNEL];
	else
		rc_vx_channel = 0;
	if(chassis_info.chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] > CHASSIS_RC_DEADLINE || chassis_info.chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] < -CHASSIS_RC_DEADLINE)
		rc_vy_channel = chassis_info.chassis_RC->rc.ch[CHASSIS_Y_CHANNEL];
	else
		rc_vy_channel = 0;
	RC_Dead_Zone_Del(chassis_info.chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL], rc_wz_channel, CHASSIS_RC_DEADLINE);
	//一阶低通滤波作为斜坡函数输入
	first_order_filter_cali(&chassis_info.chassis_vx_first_OF, (rc_vx_channel * RC_CHASSIS_VX_SEN));
	first_order_filter_cali(&chassis_info.chassis_vy_first_OF, (rc_vy_channel * RC_CHASSIS_VY_SEN));
	*vx_ch = chassis_info.chassis_vx_first_OF.out;
	*vy_ch = chassis_info.chassis_vy_first_OF.out;
	*vw_ch = rc_wz_channel;
}
//模式切换保存数据(*)
static void chassis_mode_change_save(Chassis_Mode_e last_mode, Chassis_Mode_e now_mode)
{
	//底盘状态由不跟随云台切入跟随云台
	if(last_mode == CHASSIS_NO_FOLLOW && now_mode == CHASSIS_FOLLOW_YAW)
	{
		
	}
	if(last_mode == CHASSIS_FOLLOW_YAW && now_mode == CHASSIS_GYRO)
	{
		
	}
}
//麦轮运动分解 (complete)
static void chassis_mecanum_calc(const fp32 vx_set, const fp32 vy_set, const fp32 vw_set, fp32 *wheel_rpm)
{
	wheel_rpm[0] = vx_set - vy_set - vw_set * (ROBOT_LENGTH_A + ROBOT_LENGTH_B) * CHASSIS_VW_SEN;
	wheel_rpm[1] = vx_set + vy_set + vw_set * (ROBOT_LENGTH_A + ROBOT_LENGTH_B) * CHASSIS_VW_SEN;
	wheel_rpm[2] = -vx_set + vy_set - vw_set * (ROBOT_LENGTH_A + ROBOT_LENGTH_B) * CHASSIS_VW_SEN;
	wheel_rpm[3] = -vx_set - vy_set + vw_set * (ROBOT_LENGTH_A + ROBOT_LENGTH_B) * CHASSIS_VW_SEN;
}
//底盘电机控制电流计算 (complete)
static void Chassis_Calc_Current(void)
{
	uint8_t i;
	fp32 temp = 0.0f, vector_rate = 0.0f, max_vector = 0.0f;
	fp32 wheel_speed[4] = {0.0f};
	//麦轮运动分解
	chassis_mecanum_calc(chassis_info.chassis_vx_set, chassis_info.chassis_vy_set, chassis_info.chassis_vw, wheel_speed);
	//计算轮子控制最大速度，并限制其最大速度
	for (i = 0; i < 4; i++)
	{
		chassis_info.chassis_motor[i].speed_set = wheel_speed[i];
		temp = fabs(chassis_info.chassis_motor[i].speed_set);
		if (max_vector < temp)
			max_vector = temp;
	}
	//输出限幅 按比例限幅
	if (max_vector > MAX_WHEEL_SPEED)
	{
		vector_rate = MAX_WHEEL_SPEED / max_vector;
		for (i = 0; i < 4; i++)
			chassis_info.chassis_motor[i].speed_set *= vector_rate;
	}
	//PID计算 更新电流值
	for (i = 0; i < 4; i++)
	{
		PID_Calc(&chassis_info.chassis_motor[i].motor_speed_pid, chassis_info.chassis_motor[i].speed, chassis_info.chassis_motor[i].speed_set);
		chassis_info.chassis_motor[i].give_current = (int16_t)chassis_info.chassis_motor[i].motor_speed_pid.out;
	}
}
//底盘发送控制电流到CAN (complete)
static void Chassis_Send_Current(void)
{
	//当底盘模式为无力状态 为了不受干扰 直接发送电流为0
	if(chassis_info.chassis_mode == CHASSIS_RELAX || toe_is_error(DBUSTOE) || toe_is_error(ChassisMotor1TOE) || toe_is_error(ChassisMotor2TOE) || toe_is_error(ChassisMotor3TOE) || toe_is_error(ChassisMotor4TOE))
	{
		CAN_CMD_CHASSIS(0,0,0,0);
	}	
	else 
    {
		CAN_CMD_CHASSIS(0,0,0,0);
//		CAN_CMD_CHASSIS(chassis_info.chassis_motor[0].give_current, chassis_info.chassis_motor[1].give_current,
//						chassis_info.chassis_motor[2].give_current, chassis_info.chassis_motor[3].give_current);
	}
}

