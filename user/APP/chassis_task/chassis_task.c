#include "chassis_task.h"

#include "rc.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "arm_math.h"

#include "Detect_Task.h"


//������Ϣ
static Chassis_Control_s chassis_info;

static void Chassis_Init(void);
static void Chassis_Mode_Set(void);
static void Chassis_Updata(void);
static void Chassis_Control(void);
static void Chassis_Calc_Current(void);
static void Chassis_Send_Current(void);

static void Chassis_Relax_Mode(void);
static void Chassis_Gyro_Mode(fp32 *, fp32 *, int16_t *);
static void Chassis_Raw_Mode(fp32 *, fp32 *, int16_t *);
static void Chassis_Follow_Gimbal_Mode(fp32 *, fp32 *);
static void Chassis_Cali_Mode(void);
static void Chassis_Stop_Mode(void);

static void chassis_rc_process(fp32 *vx_ch, fp32 *vy_ch, int16_t *wz_ch);
static void chassis_key_process(fp32 *, fp32 *, int16_t *);
static void chassis_mode_change_save(Chassis_Mode_e , Chassis_Mode_e );

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
		//ϵͳ��ʱ
		vTaskDelay(CHASSIS_CONTROL_TIME_MS);
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
	const static fp32 rc_x_order_filter[1] = {RC_X_ACCEL_NUM};
	const static fp32 rc_y_order_filter[1] = {RC_Y_ACCEL_NUM};
	const static fp32 key_x_order_filter[1] = {RC_X_ACCEL_NUM};
	const static fp32 key_y_order_filter[1] = {RC_Y_ACCEL_NUM};
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
	//�˲���ʼ��
    first_order_filter_init(&chassis_info.rc_vx_first_OF, CHASSIS_CONTROL_TIME, rc_x_order_filter);
    first_order_filter_init(&chassis_info.rc_vy_first_OF, CHASSIS_CONTROL_TIME, rc_y_order_filter);
    first_order_filter_init(&chassis_info.key_vx_first_OF, CHASSIS_CONTROL_TIME, key_x_order_filter);
    first_order_filter_init(&chassis_info.key_vy_first_OF, CHASSIS_CONTROL_TIME, key_y_order_filter);
	//ң�������ݳ�ʼ��
	chassis_info.chassis_RC = get_remote_control_point();
	//���������ݻ�ȡ
	chassis_info.ins_data = get_INS_Data_point();
	//��ʼ����������������
	if(chassis_info.ins_data->angle_yaw >= 0)
		chassis_info.chassis_angle_table = 1;
	else
		chassis_info.chassis_angle_table = 0;
	//��ʼ��Ȧ��
	chassis_info.chassis_circle_num = 0;
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
	Chassis_Behavior_e behavior = CHASSIS_NORMAL;
	static uint8_t mode_status;
//	uint8_t gyro_test = 0;		//����
	//���Ʒ�ʽѡ��
	if(switch_is_mid(chassis_info.chassis_RC->rc.s[CONTROL_MODE_SW]))		//ң�ؿ���
	{
		chassis_info.ctrl_mode = RC_CTRL;
	}
	else if(switch_is_up(chassis_info.chassis_RC->rc.s[CONTROL_MODE_SW]))		//���̿���
	{
		chassis_info.ctrl_mode = PC_CTRL;
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
			mode = CHASSIS_GYRO;		//����������ģʽ
			//mode = CHASSIS_FOLLOW_GIMBAL;
		}
		else if (switch_is_mid(chassis_info.chassis_RC->rc.s[CHASSIS_MODE_SW]))	//�м�
		{
			mode = CHASSIS_RAW;	//����ԭ��ģʽ
		}
		else if (switch_is_down(chassis_info.chassis_RC->rc.s[CHASSIS_MODE_SW]))	//�²�Ϊ����
		{
			mode = CHASSIS_RELAX;		//��������
		}
		behavior = CHASSIS_NORMAL;
	}
	else
	{
		//����ģʽ���� �̰��л�
		if(chassis_info.chassis_RC->key_data.key_short_press.bit.R)
			mode_status = ~mode_status;
		if(mode_status)
			mode = CHASSIS_GYRO;
		else
			mode = CHASSIS_RAW;
		
		//������Ϊ���� ����л�
		if(chassis_info.chassis_RC->key_data.key_click.bit.Q)
			behavior = CHASSIS_GYRO_OPEN;
		if(chassis_info.chassis_RC->key_data.key_click.bit.F)
			behavior = CHASSIS_CENTER;
	}
	chassis_info.chassis_mode = mode;
	chassis_info.chassis_behavior = behavior;
	//ģʽ�л���������
	if(chassis_info.chassis_mode != chassis_info.chassis_last_mode)
	{
		chassis_mode_change_save(chassis_info.chassis_last_mode, chassis_info.chassis_mode);
	}
	chassis_info.chassis_last_mode = chassis_info.chassis_mode;
}
//�������ݸ��� (*)
static void Chassis_Updata(void)
{
	const fp32 *chassis_INT_angle_point = get_INS_angle_point();//��ȡ��������̬���������� �Ƕ�
	fp32 gyro_yaw_angle = *(chassis_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
	static fp32 gyro_yaw_last_angle;
	uint8_t i;
	
	if(chassis_info.chassis_circle_num == 32767 || chassis_info.chassis_circle_num == -32768)
	{
		chassis_info.chassis_yaw -= chassis_info.chassis_circle_num * CIRCLE;
		chassis_info.chassis_yaw_last -= chassis_info.chassis_circle_num * CIRCLE;
		chassis_info.chassis_circle_num = 0;
	}
	//���������ǽǶȷ�ΧΪ-2PI~2PI
	//gyro_yaw_angle =  gyro_angle_format(gyro_yaw_last_angle, chassis_info.ins_data->angle_yaw, &chassis_info.chassis_table_flag);
	gyro_yaw_angle =  gyro_angle_format(gyro_yaw_last_angle, gyro_yaw_angle, &chassis_info.chassis_angle_table);
	//���µ���ٶ�
	for(i = 0; i < 4; i++)
	{
		chassis_info.chassis_motor[i].speed = chassis_info.chassis_motor[i].chassis_motor_measure->speed_rpm * CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
		chassis_info.chassis_motor[i].accel = chassis_info.chassis_motor[i].motor_speed_pid.Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
	}
	//���µ����ٶ�
	chassis_info.chassis_vx = (chassis_info.chassis_motor[0].speed - chassis_info.chassis_motor[1].speed + chassis_info.chassis_motor[2].speed - chassis_info.chassis_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
	chassis_info.chassis_vy = (-chassis_info.chassis_motor[0].speed - chassis_info.chassis_motor[1].speed + chassis_info.chassis_motor[2].speed + chassis_info.chassis_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
	chassis_info.chassis_vw = (-chassis_info.chassis_motor[0].speed - chassis_info.chassis_motor[1].speed - chassis_info.chassis_motor[2].speed - chassis_info.chassis_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VW / MOTOR_DISTANCE_TO_CENTER;
	//chassis_info.chassis_vw = (-chassis_info.chassis_motor[0].speed - chassis_info.chassis_motor[1].speed - chassis_info.chassis_motor[2].speed - chassis_info.chassis_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VW / (ROBOT_LENGTH_A + ROBOT_LENGTH_B);
	//���µ�������̨����ԽǶ� ��Ϊ��̨�������ԽǶ� (-2PI~2PI)
	//chassis_info.chassis_relative_angle_last = chassis_info.chassis_relative_angle;
	chassis_info.chassis_relative_angle = -get_gimbal_relative_angle();
	//���µ��̾��ԽǶ� �˴����½Ƕ���ԭʼ�Ƕ� ��Χ��-2PI~2PI û�м���Ȧ�����ӽǶ�
	chassis_info.chassis_absolute_yaw = gyro_yaw_angle + chassis_info.chassis_relative_angle;//�����ǽǶ�-��̨��ԽǶ�(����������ṩ��е�Ƕ�)
	//����Ȧ��������Ȧ���Ƕ�
	chassis_info.chassis_yaw = calc_turn_angle(chassis_info.chassis_yaw_last, (chassis_info.chassis_absolute_yaw + chassis_info.chassis_circle_num * 2 * PI), &chassis_info.chassis_circle_num);
	chassis_info.chassis_yaw_last = chassis_info.chassis_yaw;
	//������������һ�νǶ� ���ڼ��㴦�������Ǳ߽�ֵ
	gyro_yaw_last_angle = gyro_yaw_angle;
	//chassis_info.chassis_absolute_yaw_last = chassis_info.chassis_absolute_yaw;
}
//���̿���������(*)
static void Chassis_Control(void)
{
	int16_t rc_vw_channel = 0, key_vw_channel = 0, vw_set_channel = 0;//ԭ��ͨ�� ֻ��������
    fp32 rc_vx_channel = 0.0f, rc_vy_channel = 0.0f;
	fp32 key_vx_channel = 0.0f, key_vy_channel = 0.0f;
    fp32 vx_set_channel = 0.0f, vy_set_channel = 0.0f;

	//ң�����봦��
	chassis_rc_process(&rc_vx_channel, &rc_vy_channel, &rc_vw_channel);
	//�������봦��
	chassis_key_process(&key_vx_channel, &key_vy_channel, &key_vw_channel);
	//�����ٶ�����
	vx_set_channel = rc_vx_channel + key_vx_channel;
	vy_set_channel = rc_vy_channel + key_vy_channel;
	vw_set_channel = rc_vw_channel + key_vw_channel;
	//ѡ��ģʽ
	switch(chassis_info.chassis_mode)
	{
		case CHASSIS_RELAX:
		{
			Chassis_Relax_Mode();
			break;
		}
		case CHASSIS_GYRO:
		{
			Chassis_Gyro_Mode(&vx_set_channel, &vy_set_channel, &vw_set_channel);
			break;
		}
		case CHASSIS_RAW:
		{
			Chassis_Raw_Mode(&vx_set_channel, &vy_set_channel, &vw_set_channel);
			break;
		}
		case CHASSIS_FOLLOW_GIMBAL:
		{
			Chassis_Follow_Gimbal_Mode(&vx_set_channel, &vy_set_channel);
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
	chassis_info.chassis_yaw = chassis_info.chassis_yaw_set = chassis_info.chassis_absolute_yaw;
}
//����������ģʽ ��ת�Ƕ�����̨����(*)
static void Chassis_Gyro_Mode(fp32 *vx_ch, fp32 *vy_ch, int16_t *vw_ch)
{
	//fp32 rc_vw_channel;
	fp32 relative_angle_sin, relative_angle_cos;//��ԽǶȵ���������ֵ sin(relative_angle) cos(relative_angle)
	relative_angle_sin = arm_sin_f32(chassis_info.chassis_relative_angle);
	relative_angle_cos = arm_cos_f32(chassis_info.chassis_relative_angle);
	//��ָ̨��ǰ��ƽ�Ʒ����vx��vy�ֽ�ɵ��̳�ͷָ�����vx��vy 
	chassis_info.chassis_vx_set = *vx_ch * relative_angle_cos + *vy_ch * relative_angle_sin;	//vx*cos(a)+vy*sin(a)
	chassis_info.chassis_vy_set = -*vx_ch * relative_angle_sin + *vy_ch * relative_angle_cos;	//-vx*sin(a)+vy*cos(a)
	if(chassis_info.chassis_behavior == CHASSIS_NORMAL)
	{
		//ֱ�Ӹ����ٶ�
		//rc_vw_channel = *vw_ch * RC_CHASSIS_VW_SEN;
		chassis_info.chassis_vw_set = *vw_ch * RC_CHASSIS_VW_SEN;;
		//���̾��ԽǶ� ������ת�Ƕ� ��ģʽ��vwͨ������ֵ����Ϊ�Ƕ�rad(-2PI,2PI) (��Ҫ�޸�)
		//chassis_info.chassis_yaw_set = chassis_info.chassis_yaw + *vw_ch * CHASSIS_ANGLE_Z_RC_SEN;
		//�ڻ�PID������ת���ٶ�
		//chassis_info.chassis_vw_set = PID_Calc(&chassis_info.chassis_angle_pid, chassis_info.chassis_yaw, chassis_info.chassis_yaw_set);
	}
	else if(chassis_info.chassis_behavior == CHASSIS_GYRO_OPEN)
	{
		chassis_info.chassis_vw_set = 1.0f;
	}
}
//����ԭʼģʽ ��̨���̷������ ǰ�������ɵ��̾���
static void Chassis_Raw_Mode(fp32 *vx_ch, fp32 *vy_ch, int16_t *vw_ch)
{
	//fp32 rc_vw_channel;
	//������ת�Ƕ� ����һ���Ƕ�
	//rc_vw_channel = *vw_ch * RC_CHASSIS_VW_SEN;
	//�ٶ��޷�
	chassis_info.chassis_vx_set = fp32_constrain(*vx_ch, -MAX_CHASSIS_SPEED_X, MAX_CHASSIS_SPEED_X);
	chassis_info.chassis_vy_set = fp32_constrain(*vy_ch, -MAX_CHASSIS_SPEED_Y, MAX_CHASSIS_SPEED_Y);
	//�ڻ�������ת���ٶ�
	//chassis_info.chassis_vw_set = PID_Calc(&chassis_info.chassis_angle_pid, 0.0f, *vw_ch);
	chassis_info.chassis_vw_set = *vw_ch * RC_CHASSIS_VW_SEN;
}
//���̸�����̨ģʽ ��ת�Ƕ�����̨���� ǰ����������̨����(*)
static void Chassis_Follow_Gimbal_Mode(fp32 *vx_ch, fp32 *vy_ch)
{
	const Gimbal_Motor_s *gimbal_yaw = get_gimbal_yaw_motor_point();
	//fp32 vw_follow_ch = chassis_info.chassis_RC->rc.ch[GIMBAL_YAW_CHANNEL] * YAW_RC_SEN;//����̨yawʹ��ͬ��ͨ�� ������̨
	//�������趨 ��ת�Ƕȵ���
	//chassis_info.chassis_yaw_set += vw_follow_ch;
	chassis_info.chassis_vx_set = fp32_constrain(*vx_ch, -MAX_CHASSIS_SPEED_X, MAX_CHASSIS_SPEED_X);
	chassis_info.chassis_vy_set = fp32_constrain(*vy_ch, -MAX_CHASSIS_SPEED_Y, MAX_CHASSIS_SPEED_Y);
	chassis_info.chassis_vw_set = PID_Calc(&chassis_info.chassis_angle_pid, chassis_info.chassis_yaw, gimbal_yaw->absolute_angle);
	//chassis_info.chassis_vw_set = PID_Calc(&chassis_info.chassis_angle_pid, chassis_info.chassis_yaw, chassis_info.chassis_yaw_set);
}
//����У׼ģʽ
static void Chassis_Cali_Mode(void)
{
	
}
//����ֹͣ״̬
static void Chassis_Stop_Mode(void)
{
	chassis_info.chassis_vx_set = 0.0f;
	chassis_info.chassis_vy_set = 0.0f;
	chassis_info.chassis_vw_set = 0.0f;
}
//ң�ؿ���ģʽ(*)
static void chassis_rc_process(fp32 *vx_ch, fp32 *vy_ch, int16_t *vw_ch)
{
	int16_t rc_vx_channel = 0, rc_vy_channel = 0, rc_wz_channel = 0;
	//ң����������
//	i_dead_zone_del(chassis_info.chassis_RC->rc.ch[CHASSIS_X_CHANNEL], &rc_vx_channel, RC_DEADLINE);
//	i_dead_zone_del(chassis_info.chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], &rc_vy_channel, RC_DEADLINE);
//	i_dead_zone_del(chassis_info.chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL], &rc_wz_channel, RC_DEADLINE);
	rc_vx_channel = chassis_info.chassis_RC->rc.ch[CHASSIS_X_CHANNEL];
	rc_vy_channel = chassis_info.chassis_RC->rc.ch[CHASSIS_Y_CHANNEL];
	rc_wz_channel = chassis_info.chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
	*vx_ch = rc_vx_channel * RC_CHASSIS_VX_SEN;
	*vy_ch = rc_vy_channel * RC_CHASSIS_VY_SEN;
	//һ�׵�ͨ�˲���Ϊб�º�������
	first_order_filter_cali(&chassis_info.rc_vx_first_OF, *vx_ch);
	first_order_filter_cali(&chassis_info.rc_vy_first_OF, *vy_ch);
    //ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
    if (*vx_ch < RC_DEADLINE * RC_CHASSIS_VX_SEN && *vx_ch > -RC_DEADLINE * RC_CHASSIS_VX_SEN)
    {
        chassis_info.rc_vx_first_OF.out = 0.0f;
    }
    if (*vy_ch < RC_DEADLINE * RC_CHASSIS_VY_SEN && *vy_ch > -RC_DEADLINE * RC_CHASSIS_VY_SEN)
    {
        chassis_info.rc_vy_first_OF.out = 0.0f;
    }
	*vx_ch = chassis_info.rc_vx_first_OF.out;
	*vy_ch = chassis_info.rc_vy_first_OF.out;
	*vw_ch = rc_wz_channel;//ֻ�������� ԭʼͨ��
}
static void chassis_move_time_process(uint16_t *remove_channel)
{
	static portTickType chassis_current_time = 0;
	static uint32_t chassis_delay_time = 0;
	chassis_current_time = xTaskGetTickCount();
	//����һ���ƶ���
	if(chassis_current_time >= chassis_delay_time)
	{
		chassis_delay_time = chassis_current_time + CHASSIS_REMOVE_DELAY_MS;
		if(*remove_channel < KEY_CHASSIS_VALUE_MAX)
			(*remove_channel)++;
	}
}
static void chassis_key_process(fp32 *vx_ch, fp32 *vy_ch ,int16_t *vw_ch)
{
	//ģ��ҡ���ĸ�ͨ�� max:660
	static uint16_t fron_ch, back_ch, left_ch, right_ch;
	//�ƶ�����
	if(chassis_info.chassis_RC->key.bit.W) //��������ͨ����
		chassis_move_time_process(&fron_ch);
	else		//�ɿ�ͨ����ֱ�Ӽ�Ϊ0
		fron_ch = 0;
	if(chassis_info.chassis_RC->key.bit.S)
		chassis_move_time_process(&back_ch);
	else
		back_ch = 0;
	if(chassis_info.chassis_RC->key.bit.A)
		chassis_move_time_process(&left_ch);
	else
		left_ch = 0;
	if(chassis_info.chassis_RC->key.bit.D)
		chassis_move_time_process(&right_ch);
	else
		right_ch = 0;
	//����С���ݹ̶��ٶ�
	if(chassis_info.chassis_RC->key.bit.SHIFT)
		*vw_ch = KEY_CHASSIS_ROTATE_SPEED;
	*vx_ch = (fron_ch - back_ch) * KEY_CHASSIS_VX_SEN;
	*vy_ch = (left_ch - right_ch) * KEY_CHASSIS_VY_SEN;
	//һ���˲���Ϊб�º���
	first_order_filter_cali(&chassis_info.key_vx_first_OF, *vx_ch);
	first_order_filter_cali(&chassis_info.key_vy_first_OF, *vy_ch);
    if (*vx_ch == 0.0f)
        chassis_info.key_vx_first_OF.out = 0.0f;		//�˲����ֱ��Ϊ0 �����һֱ������ȥֱ�����
    if (*vy_ch == 0.0f)
        chassis_info.key_vy_first_OF.out = 0.0f;
	*vx_ch = chassis_info.key_vx_first_OF.out;
	*vy_ch = chassis_info.key_vy_first_OF.out;
}

//ģʽ�л���������(*)
static void chassis_mode_change_save(Chassis_Mode_e last_mode, Chassis_Mode_e now_mode)
{
	//����״̬�ɲ�������̨���������̨
	if(last_mode == CHASSIS_RAW && now_mode == CHASSIS_GYRO)
	{
		
	}
	if(last_mode == CHASSIS_FOLLOW_GIMBAL && now_mode == CHASSIS_GYRO)
	{
		
	}
}
//�����˶��ֽ� (complete)
static void chassis_mecanum_calc(const fp32 vx_set, const fp32 vy_set, const fp32 vw_set, fp32 *wheel_rpm)
{
//	wheel_rpm[0] = vx_set - vy_set - vw_set * (ROBOT_LENGTH_A + ROBOT_LENGTH_B) * CHASSIS_VW_SEN;
//	wheel_rpm[1] = vx_set + vy_set + vw_set * (ROBOT_LENGTH_A + ROBOT_LENGTH_B) * CHASSIS_VW_SEN;
//	wheel_rpm[2] = -vx_set + vy_set - vw_set * (ROBOT_LENGTH_A + ROBOT_LENGTH_B) * CHASSIS_VW_SEN;
//	wheel_rpm[3] = -vx_set - vy_set + vw_set * (ROBOT_LENGTH_A + ROBOT_LENGTH_B) * CHASSIS_VW_SEN;
	
//	wheel_rpm[0] = vx_set - vy_set - vw_set * (ROBOT_LENGTH_A + ROBOT_LENGTH_B);// * CHASSIS_VW_SEN;
//	wheel_rpm[1] = -vx_set - vy_set + vw_set * (ROBOT_LENGTH_A + ROBOT_LENGTH_B);// * CHASSIS_VW_SEN;
//	wheel_rpm[2] = vx_set + vy_set + vw_set * (ROBOT_LENGTH_A + ROBOT_LENGTH_B);// * CHASSIS_VW_SEN;
//	wheel_rpm[3] = -vx_set + vy_set - vw_set * (ROBOT_LENGTH_A + ROBOT_LENGTH_B);// * CHASSIS_VW_SEN;
	wheel_rpm[0] = vx_set - vy_set - vw_set * MOTOR_DISTANCE_TO_CENTER;
	wheel_rpm[1] = -vx_set - vy_set - vw_set * MOTOR_DISTANCE_TO_CENTER;
	wheel_rpm[2] = vx_set + vy_set - vw_set * MOTOR_DISTANCE_TO_CENTER;
	wheel_rpm[3] = -vx_set + vy_set - vw_set * MOTOR_DISTANCE_TO_CENTER;
	
}
//���̵�����Ƶ������� (complete)
static void Chassis_Calc_Current(void)
{
	uint8_t i;
	fp32 temp = 0.0f, vector_rate = 0.0f, max_vector = 0.0f;
	fp32 wheel_speed[4] = {0.0f};
	//�����˶��ֽ�
	chassis_mecanum_calc(chassis_info.chassis_vx_set, chassis_info.chassis_vy_set, chassis_info.chassis_vw_set, wheel_speed);
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
		//CAN_CMD_CHASSIS(0,0,0,0);
		CAN_CMD_CHASSIS(chassis_info.chassis_motor[0].give_current, chassis_info.chassis_motor[1].give_current,
						chassis_info.chassis_motor[2].give_current, chassis_info.chassis_motor[3].give_current);
	}
}
const Chassis_Control_s *get_classis_info_point(void)
{
	return &chassis_info;
}
