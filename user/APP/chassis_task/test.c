#include "test.h"

#include "chassis_task.h"

#include "rc.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "arm_math.h"

#include "Detect_Task.h"


#include "keyboard.h"

//������Ϣ
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
//ң����������
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
//�Ƕȹ��� -PI~PI--> -2PI~2PI
#define angle_format(angle_now, angle_last)			\
	{												\
		if((angle_now) - (angle_last) < -PI)		\
			(angle_now) += 2*PI;					\
		else if((angle_now) - (angle_last) > PI)	\
			(angle_now) -= 2*PI;					\
	}

void chassis_task(void *pvParameters)
{
	//����һ��ʱ��
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	//���̳�ʼ��
	Chassis_Init();
	//�жϵ��̵���Ƿ�����
	while (toe_is_error(ChassisMotor1TOE) || toe_is_error(ChassisMotor2TOE) || toe_is_error(ChassisMotor3TOE) || toe_is_error(ChassisMotor4TOE) || toe_is_error(DBUSTOE))
	{
		vTaskDelay(CHASSIS_CONTROL_TIME_MS);
	}
	
	while(1)
	{
		//����ģʽ����
		Chassis_Mode_Set();
		//�������ݸ���
		Chassis_Updata();
		//���̵��̿���������
		Chassis_Control();
		//���̵����������
		Chassis_Calc_Current();
		//���̵���������
		Chassis_Send_Current();
	}
}
//���̳�ʼ��(*)
static void Chassis_Init(void)
{
	uint8_t i = 0;
	//�����ٶȻ�pidֵ
	const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
	//���̽ǶȻ�pidֵ
	const static fp32 chassis_yaw_pid[3] = {CHASSIS_ANGLE_PID_KP, CHASSIS_ANGLE_PID_KI, CHASSIS_ANGLE_PID_KD};
	//�˲�ϵ��
	const static fp32 chassis_x_order_filter[1] = {CHASSIS_X_ACCEL_NUM};
	const static fp32 chassis_y_order_filter[1] = {CHASSIS_Y_ACCEL_NUM};
	for(i = 0; i < 4; i++)
	{
		//����������ݻ�ȡ
		chassis_info.chassis_motor[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
		//PID��ʼ��
		PID_Init(&chassis_info.chassis_motor[i].motor_speed_pid, PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
	}
	//���̽ǶȻ�PID��ʼ��
	PID_Init(&chassis_info.chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_ANGLE_PID_MAX_OUT, CHASSIS_ANGLE_PID_MAX_IOUT);
	//����ģʽ��ʼ��
	chassis_info.chassis_mode = CHASSIS_RELAX;
	//��̨yaw�������ݻ�ȡ�����ڼ�����̾��ԽǶ�
	chassis_info.yaw_motor_gimbal = get_yaw_motor_point();
	//�˲���ʼ��
    first_order_filter_init(&chassis_info.chassis_vx_first_OF, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_info.chassis_vy_first_OF, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
	//ң�������ݳ�ʼ��
	chassis_info.chassis_RC = get_remote_control_point();
	//���������ݻ�ȡ
	chassis_info.ins_data = get_INS_Data_point();
	//���̵���������ݳ�ʼ��
	for(i = 0; i < 4; i++)
		chassis_info.chassis_motor[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
	//���̸���
	Chassis_Updata();
}
//����ģʽ����(*)
static void Chassis_Mode_Set(void)
{
	Chassis_Mode_e mode = CHASSIS_RELAX;
	uint8_t gyro_test = 0;		//����
	//���Ʒ�ʽѡ��
	if(switch_is_mid(chassis_info.chassis_RC->rc.s[CONTROL_MODE_SW]))		//ң�ؿ���
	{
		chassis_info.ctrl_mode = RC_CTRL;
	}
	else if(switch_is_up(chassis_info.chassis_RC->rc.s[CONTROL_MODE_SW]))		//���̿���
	{
		chassis_info.ctrl_mode = KEY_CTRL;
	}
	else	//ң��ģʽ�¿���Ħ����
	{
		chassis_info.ctrl_mode = RC_CTRL;
	}
	//����ģʽѡ��
	if(chassis_info.ctrl_mode == RC_CTRL)
	{
		if(switch_is_up(chassis_info.chassis_RC->rc.s[CHASSIS_MODE_SW]))
		{
			mode = CHASSIS_NO_FOLLOW;	//���̲�������̨
		}
		else if (switch_is_mid(chassis_info.chassis_RC->rc.s[CHASSIS_MODE_SW]))	//�м�Ϊң��
		{
			mode = CHASSIS_FOLLOW_YAW;		//���̸�����̨
		}
		else if (switch_is_down(chassis_info.chassis_RC->rc.s[CHASSIS_MODE_SW]))	//�²�Ϊ����
		{
			mode = CHASSIS_RELAX;		//��������
		}
	}
	else
	{
		if(key_status.bit.Q || gyro_test)	//Q����С����chassis_info.chassis_RC->key.bit.Q
		{
			mode = CHASSIS_GYRO;	//С����
		}
	}
	chassis_info.chassis_mode = mode;
	//ģʽ�л���������
	if(chassis_info.chassis_mode != chassis_info.chassis_last_mode)
	{
		chassis_mode_change_save(chassis_info.chassis_last_mode, chassis_info.chassis_mode);
	}
	chassis_info.chassis_last_mode = chassis_info.chassis_mode;
}
//�������Ȧ����ʵ�ʽǶ�
static fp32 chassis_calc_turn_angle(const fp32 *angle_last, fp32 *angle_now)
{
	//����Ȧ��Ϊ�� ��ʱ��
	if(*angle_now - *angle_last < -4.0f)
	{
		chassis_info.chassis_circle_num++;
		*angle_now += 2 * PI;
	}
	//����Ȧ��Ϊ�� ˳ʱ��
	else if(*angle_now - *angle_last > 4.0f)
	{
		chassis_info.chassis_circle_num--;
		*angle_now -= 2 * PI;
	}
	return *angle_now;
}
//������̾��ԽǶ�
//static fp32 chassis_absolute_angle_process(const fp32 relative_angle, const fp32 gyro_angle)
//{
//	fp32 absolute_angle;
//	absolute_angle = gyro_angle - relative_angle;
//	return absolute_angle;
//}
//�������ݸ��� (*)
static void Chassis_Updata(void)
{
	const fp32 *chassis_INT_angle_point = get_INS_angle_point();//��ȡ��������̬���������� �Ƕ�
	fp32 gyro_yaw_angle = *(chassis_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
	static fp32 gyro_yaw_last_angle;
	uint8_t i;
	//���������ǽǶȷ�ΧΪ-2PI~2PI
	angle_format(gyro_yaw_angle, gyro_yaw_last_angle);
	//���µ���ٶ�
	for(i = 0; i < 4; i++)
	{
		chassis_info.chassis_motor[i].speed = chassis_info.chassis_motor[i].chassis_motor_measure->speed_rpm * CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
		chassis_info.chassis_motor[i].accel = chassis_info.chassis_motor[i].motor_speed_pid.Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
	}
	//���µ����ٶ�
	chassis_info.chassis_vx = (chassis_info.chassis_motor[0].speed - chassis_info.chassis_motor[1].speed + chassis_info.chassis_motor[2].speed - chassis_info.chassis_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
	chassis_info.chassis_vy = (-chassis_info.chassis_motor[0].speed - chassis_info.chassis_motor[1].speed + chassis_info.chassis_motor[2].speed + chassis_info.chassis_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
	chassis_info.chassis_vx = (-chassis_info.chassis_motor[0].speed - chassis_info.chassis_motor[1].speed - chassis_info.chassis_motor[2].speed - chassis_info.chassis_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
	//���µ�������̨����ԽǶ� ��Ϊ��̨�������ԽǶ�
	chassis_info.chassis_relative_angle = chassis_info.yaw_motor_gimbal->relative_angle;	
	//���µ��̾��ԽǶ� �˴����½Ƕ���ԭʼ�Ƕ� ��Χ��-2PI~2PI û�м���Ȧ�����ӽǶ�
	chassis_info.chassis_absolute_yaw = gyro_yaw_angle - chassis_info.chassis_relative_angle;//�����ǽǶ�-��̨��ԽǶ�(����������ṩ��е�Ƕ�)
	chassis_info.chassis_last_absolute_yaw = chassis_info.chassis_absolute_yaw;
	//����Ȧ��������Ȧ���Ƕ�
	chassis_calc_turn_angle(&chassis_info.chassis_last_yaw, &chassis_info.chassis_yaw);
	chassis_info.chassis_last_yaw = chassis_info.chassis_yaw;
	//������������һ�νǶ� ���ڼ��㴦�������Ǳ߽�ֵ
	gyro_yaw_last_angle = gyro_yaw_angle;
}
//���̿���������(*)
static void Chassis_Control(void)
{
	int16_t rc_vw_channel, key_vw_channel, vw_set_channel;
    fp32 rc_vx_channel, rc_vy_channel;
	fp32 key_vx_channel, key_vy_channel;
    fp32 vx_set_channel, vy_set_channel;

	//ң�����봦��
	chassis_RC_ctrl(&rc_vx_channel, &rc_vy_channel, &rc_vw_channel);
	//�������봦��
	chassis_key_ctrl(chassis_info.chassis_RC, &key_vx_channel, &key_vy_channel);
	//�����ٶ�����
	vx_set_channel = rc_vx_channel + key_vx_channel;
	vy_set_channel = rc_vy_channel + key_vy_channel;
	vw_set_channel = rc_vw_channel;// + key_vw_channel;
	//ѡ��ģʽ
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
//��������ģʽ
static void Chassis_Relax_Mode(void)
{
	chassis_info.chassis_vx_set = 0.0f;
	chassis_info.chassis_vy_set = 0.0f;
	chassis_info.chassis_vw_set = 0.0f;
	chassis_info.chassis_absolute_yaw_set = chassis_info.chassis_absolute_yaw;
	//chassis_info.chassis_relative_angle_set = chassis_info.chassis_relative_angle;
}
//���̸�����̨ģʽ ǰ����������̨����(*)
static void Chassis_Follow_Yaw_Mode(fp32 *vx_ch, fp32 *vy_ch, int16_t *vw_ch)
{
	fp32 relative_angle_sin, relative_angle_cos;//��ԽǶȵ���������ֵ sin(relative_angle) cos(relative_angle)
	relative_angle_sin = arm_sin_f32(chassis_info.chassis_relative_angle);
	relative_angle_cos = arm_cos_f32(chassis_info.chassis_relative_angle);
	//��ָ̨��ǰ��ƽ�Ʒ����vx��vy�ֽ�ɵ��̳�ͷָ�����vx��vy 
	chassis_info.chassis_vx_set = *vx_ch * relative_angle_cos - *vy_ch * relative_angle_cos;	//vx*cos(a)-vy*sin(a)
	chassis_info.chassis_vy_set = *vx_ch * relative_angle_sin + *vy_ch * relative_angle_sin;	//vx*sin(a)+vy*cos(a)
	//���̾��ԽǶ� ������ת�Ƕ� ��ģʽ��vwͨ������ֵ����Ϊ�Ƕ�rad(-2PI,2PI) (��Ҫ�޸�)
	chassis_info.chassis_absolute_yaw_set = chassis_info.chassis_absolute_yaw + *vw_ch * CHASSIS_ANGLE_Z_RC_SEN;
	//�ڻ�PID������ת���ٶ�
	chassis_info.chassis_vw_set = PID_Calc(&chassis_info.chassis_angle_pid, chassis_info.chassis_absolute_yaw, chassis_info.chassis_absolute_yaw_set);
}
//���̲�������̨ģʽ ǰ�������ɵ��̾���
static void Chassis_No_Follow_Mode(fp32 *vx_ch, fp32 *vy_ch, int16_t *vw_ch)
{
	int16_t rc_vw_channel;
	//ң��������ȥ��
	RC_Dead_Zone_Del(chassis_info.chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL], rc_vw_channel, CHASSIS_RC_DEADLINE);
	//������ת�Ƕ� ����һ���Ƕ�
	*vw_ch = rc_vw_channel * RC_CHASSIS_VW_SEN;
	//chassis_info.chassis_absolute_yaw_set += *vw_ch;
	//�ٶ��޷�
	chassis_info.chassis_vx_set = fp32_constrain(*vx_ch, -MAX_CHASSIS_SPEED_X, MAX_CHASSIS_SPEED_X);
	chassis_info.chassis_vy_set = fp32_constrain(*vy_ch, -MAX_CHASSIS_SPEED_Y, MAX_CHASSIS_SPEED_Y);
	//�ڻ�������ת���ٶ�
	//chassis_info.chassis_vw_set = PID_Calc(&chassis_info.chassis_angle_pid, 0.0f, *vw_ch);
	chassis_info.chassis_vw_set = *vw_ch;
}
//����С����ģʽ
static void Chassis_Gyro_Mode(fp32 *vx_ch, fp32 *vy_ch, int16_t *vw_ch)
{
	
}
//����У׼ģʽ
static void Chassis_Cali_Mode(void)
{
	
}
//����ֹͣ״̬
static void Chassis_Stop_Mode(void)
{
	
}
//ң�ؿ���ģʽ(*)
static void chassis_RC_ctrl(fp32 *vx_ch, fp32 *vy_ch, int16_t *vw_ch)
{
	int16_t rc_vx_channel, rc_vy_channel, rc_wz_channel;
	//ң����������
	if(chassis_info.chassis_RC->rc.ch[CHASSIS_X_CHANNEL] > CHASSIS_RC_DEADLINE || chassis_info.chassis_RC->rc.ch[CHASSIS_X_CHANNEL] < -CHASSIS_RC_DEADLINE)
		rc_vx_channel = chassis_info.chassis_RC->rc.ch[CHASSIS_X_CHANNEL];
	else
		rc_vx_channel = 0;
	if(chassis_info.chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] > CHASSIS_RC_DEADLINE || chassis_info.chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] < -CHASSIS_RC_DEADLINE)
		rc_vy_channel = chassis_info.chassis_RC->rc.ch[CHASSIS_Y_CHANNEL];
	else
		rc_vy_channel = 0;
	RC_Dead_Zone_Del(chassis_info.chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL], rc_wz_channel, CHASSIS_RC_DEADLINE);
	//һ�׵�ͨ�˲���Ϊб�º�������
	first_order_filter_cali(&chassis_info.chassis_vx_first_OF, (rc_vx_channel * RC_CHASSIS_VX_SEN));
	first_order_filter_cali(&chassis_info.chassis_vy_first_OF, (rc_vy_channel * RC_CHASSIS_VY_SEN));
	*vx_ch = chassis_info.chassis_vx_first_OF.out;
	*vy_ch = chassis_info.chassis_vy_first_OF.out;
	*vw_ch = rc_wz_channel;
}
//ģʽ�л���������(*)
static void chassis_mode_change_save(Chassis_Mode_e last_mode, Chassis_Mode_e now_mode)
{
	//����״̬�ɲ�������̨���������̨
	if(last_mode == CHASSIS_NO_FOLLOW && now_mode == CHASSIS_FOLLOW_YAW)
	{
		
	}
	if(last_mode == CHASSIS_FOLLOW_YAW && now_mode == CHASSIS_GYRO)
	{
		
	}
}
//�����˶��ֽ� (complete)
static void chassis_mecanum_calc(const fp32 vx_set, const fp32 vy_set, const fp32 vw_set, fp32 *wheel_rpm)
{
	wheel_rpm[0] = vx_set - vy_set - vw_set * (ROBOT_LENGTH_A + ROBOT_LENGTH_B) * CHASSIS_VW_SEN;
	wheel_rpm[1] = vx_set + vy_set + vw_set * (ROBOT_LENGTH_A + ROBOT_LENGTH_B) * CHASSIS_VW_SEN;
	wheel_rpm[2] = -vx_set + vy_set - vw_set * (ROBOT_LENGTH_A + ROBOT_LENGTH_B) * CHASSIS_VW_SEN;
	wheel_rpm[3] = -vx_set - vy_set + vw_set * (ROBOT_LENGTH_A + ROBOT_LENGTH_B) * CHASSIS_VW_SEN;
}
//���̵�����Ƶ������� (complete)
static void Chassis_Calc_Current(void)
{
	uint8_t i;
	fp32 temp = 0.0f, vector_rate = 0.0f, max_vector = 0.0f;
	fp32 wheel_speed[4] = {0.0f};
	//�����˶��ֽ�
	chassis_mecanum_calc(chassis_info.chassis_vx_set, chassis_info.chassis_vy_set, chassis_info.chassis_vw, wheel_speed);
	//�������ӿ�������ٶȣ�������������ٶ�
	for (i = 0; i < 4; i++)
	{
		chassis_info.chassis_motor[i].speed_set = wheel_speed[i];
		temp = fabs(chassis_info.chassis_motor[i].speed_set);
		if (max_vector < temp)
			max_vector = temp;
	}
	//����޷� �������޷�
	if (max_vector > MAX_WHEEL_SPEED)
	{
		vector_rate = MAX_WHEEL_SPEED / max_vector;
		for (i = 0; i < 4; i++)
			chassis_info.chassis_motor[i].speed_set *= vector_rate;
	}
	//PID���� ���µ���ֵ
	for (i = 0; i < 4; i++)
	{
		PID_Calc(&chassis_info.chassis_motor[i].motor_speed_pid, chassis_info.chassis_motor[i].speed, chassis_info.chassis_motor[i].speed_set);
		chassis_info.chassis_motor[i].give_current = (int16_t)chassis_info.chassis_motor[i].motor_speed_pid.out;
	}
}
//���̷��Ϳ��Ƶ�����CAN (complete)
static void Chassis_Send_Current(void)
{
	//������ģʽΪ����״̬ Ϊ�˲��ܸ��� ֱ�ӷ��͵���Ϊ0
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

