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

//#if INCLUDE_uxTaskGetStackHighWaterMark	//�鿴�����ջʣ������
//uint32_t gimbal_high_water;
//#endif

//��̨���������������
static Gimbal_Control_s gimbal_info;
//���͵���̨can ָ��
static int16_t Yaw_Can_Set_Voltage = 0, Pitch_Can_Set_Voltage = 0, Shoot_Can_Set_Voltage = 0;
/**************************PID**********************************/
//yaw �ֶ�����PID
static PidTypeDef yaw_ecd_speed_rc_pid = { PID_POSITION, YAW_ECD_SPEED_RC_PID_Init };
static PidTypeDef yaw_ecd_angle_rc_pid = { PID_POSITION, YAW_ECD_ANGLE_RC_PID_Init };
static PidTypeDef yaw_gyro_speed_rc_pid = { PID_POSITION, YAW_GYRO_SPEED_RC_PID_Init };
static PidTypeDef yaw_gyro_angle_rc_pid = { PID_POSITION, YAW_GYRO_ANGLE_RC_PID_Init };
//pitch �ֶ�����PID
static PidTypeDef pitch_ecd_speed_rc_pid = { PID_POSITION, PITCH_ECD_SPEED_RC_PID_Init };
static PidTypeDef pitch_ecd_angle_rc_pid = { PID_POSITION, PITCH_ECD_ANGLE_RC_PID_Init };
//yaw �Զ�����PID
static PidTypeDef yaw_ecd_speed_auto_pid = { PID_POSITION, YAW_ECD_SPEED_AUTO_PID_Init };
static PidTypeDef yaw_ecd_angle_auto_pid = { PID_POSITION, YAW_ECD_ANGLE_AUTO_PID_Init };
static PidTypeDef yaw_gyro_speed_auto_pid = { PID_POSITION, YAW_GYRO_SPEED_AUTO_PID_Init };
static PidTypeDef yaw_gyro_angle_auto_pid = { PID_POSITION, YAW_GYRO_ANGLE_AUTO_PID_Init };
//pitch �Զ�����PID
static PidTypeDef pitch_ecd_speed_auto_pid = { PID_POSITION, PITCH_ECD_SPEED_AUTO_PID_Init };
static PidTypeDef pitch_ecd_angle_auto_pid = { PID_POSITION, PITCH_ECD_ANGLE_AUTO_PID_Init };
//yaw����ģʽ��PID
//С����ģʽ
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
    //�ȴ������������������������
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //��̨��ʼ��
    Gimbal_Init();
    //�����ʼ��
    shoot_init();
    //�жϵ���Ƿ�����
    while (toe_is_error(YawGimbalMotorTOE) || toe_is_error(PitchGimbalMotorTOE)|| toe_is_error(TriggerMotorTOE) )
    {
        vTaskDelay(GIMBAL_CONTROL_TIME_MS);
        Gimbal_Updata();             //��̨���ݷ���
    }

    while (1)
    {
		//ģʽ ң�� �Զ� ����
		Gimbal_mode_set();
		//��̨���ݸ���
		Gimbal_Updata();
		//��̨����������
		Gimbal_Control();
		//�����������
        Shoot_Can_Set_Voltage = shoot_control_loop();  
		//���Ϳ��Ƶ�ѹ
		Gimbal_Send_Voltage();
		//ϵͳ��ʱ
        vTaskDelay(GIMBAL_CONTROL_TIME_MS);

//#if INCLUDE_uxTaskGetStackHighWaterMark
//        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
//#endif
    }
}


//��̨��ʼ��
static void Gimbal_Init()
{
	const static fp32 gimbal_pitch_order_filter[1] = {GIMBAL_PITCH_ACCEL_NUM};
	const static fp32 gimbal_yaw_order_filter[1] = {GIMBAL_YAW_ACCEL_NUM};
	
	//�˲���ʼ��
	first_order_filter_init(&gimbal_info.pitch_motor.gimbal_motor_first_OF, GIMBAL_CONTROL_TIME_MS, gimbal_pitch_order_filter);
	first_order_filter_init(&gimbal_info.yaw_motor.gimbal_motor_first_OF, GIMBAL_CONTROL_TIME_MS, gimbal_yaw_order_filter);
	//��ȡң��������
	gimbal_info.gimbal_RC = get_remote_control_point();
	//��ȡ�������
	gimbal_info.yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();
	gimbal_info.pitch_motor.gimbal_motor_measure = get_Pitch_Gimbal_Motor_Measure_Point();
	//���³�ʼ�Ƕ�
	gimbal_info.pitch_motor.ecd_last = gimbal_info.pitch_motor.gimbal_motor_measure->last_ecd;
	gimbal_info.pitch_motor.ecd_now = gimbal_info.pitch_motor.gimbal_motor_measure->ecd;
	gimbal_info.yaw_motor.ecd_last = gimbal_info.yaw_motor.gimbal_motor_measure->last_ecd;
	gimbal_info.yaw_motor.ecd_now = gimbal_info.yaw_motor.gimbal_motor_measure->ecd;
	//��ʼ��������������
	angle_table_init(gimbal_info.pitch_motor.gimbal_motor_measure->ecd, PITCH_ECD_DEL, &gimbal_info.pitch_motor.turn_table_flag);
	angle_table_init(gimbal_info.yaw_motor.gimbal_motor_measure->ecd, YAW_ECD_DEL, &gimbal_info.yaw_motor.turn_table_flag);
	//��ʼ����̨Ȧ��
	gimbal_info.turn_circle_num = 0;
	//��ȡ��������
	gimbal_info.gimbal_AUTO_ctrl = get_AUTO_control_point();
	Gimbal_Updata();
}
//��̨ģʽѡ��
static void Gimbal_mode_set(void)
{
	Gimbal_Mode_e mode = GIMBAL_RELAX;
	Gimbal_Behavior_e behavior = GIMBAL_NORMAL;

	//���Ʒ�ʽѡ��
	if(switch_is_mid(gimbal_info.gimbal_RC->rc.s[CONTROL_MODE_SW]))		//ң�ؿ���
	{
		gimbal_info.ctrl_mode = RC_CTRL;
	}
	else if(switch_is_up(gimbal_info.gimbal_RC->rc.s[CONTROL_MODE_SW]))		//���̿���
	{
		gimbal_info.ctrl_mode = KEY_CTRL;
	}
	else	//ң��ģʽ�¿���Ħ����
	{
		gimbal_info.ctrl_mode = RC_CTRL;
	}
	//��̨ģʽѡ��
	if(gimbal_info.ctrl_mode == RC_CTRL)
	{
		if (switch_is_mid(gimbal_info.gimbal_RC->rc.s[GIMBAL_MODE_SW]))
		{
			mode = GIMBAL_ENCONDE;//��еģʽ
		}
		else if (switch_is_up(gimbal_info.gimbal_RC->rc.s[GIMBAL_MODE_SW]))
		{
			mode = GIMBAL_GYRO;//������ģʽ
		}
		else if (switch_is_down(gimbal_info.gimbal_RC->rc.s[GIMBAL_MODE_SW]))
		{
			mode = GIMBAL_RELAX;//��̨����
		}
		gimbal_info.gimbal_behavior = GIMBAL_NORMAL;
	}
	else		//�������
	{
		//��̨ģʽ���� �̰��л�
		if(gimbal_info.gimbal_RC->key_data.key_short_press.bit.E)
			mode = GIMBAL_ENCONDE;
		
		//��̨��Ϊ���� ����л�
		if(gimbal_info.gimbal_RC->key_data.key_click.bit.Q)	//Q����С����chassis_info.chassis_RC->key.bit.Q
			behavior = GIMBAL_NORMAL_GR;	//С����
	}
	gimbal_info.gimbal_mode = mode;
	gimbal_info.gimbal_behavior = behavior;
	//ģʽ�л���������
	if(gimbal_info.gimbal_mode != gimbal_info.gimbal_mode_last)
	{
		gimbal_mode_change_save(gimbal_info.gimbal_mode_last, gimbal_info.gimbal_mode);
	}
	gimbal_info.gimbal_mode_last = gimbal_info.gimbal_mode;
	gimbal_info.gimbal_behavior_last = gimbal_info.gimbal_behavior;
}
//��̨���ݸ���
static void Gimbal_Updata(void)
{
	const fp32 *gimbal_INT_angle_point = get_INS_angle_point();//��ȡ��������̬���������� �Ƕ�
	const fp32 *gimbal_INT_gyro_point = get_MPU6500_Gyro_Data_Point();//��ȡ������ԭʼ���� ���ٶ�
	
	if(gimbal_info.turn_circle_num == 32767 || gimbal_info.turn_circle_num == -32768)
	{
		gimbal_info.yaw_motor.absolute_angle -= gimbal_info.turn_circle_num * CIRCLE;
		gimbal_info.yaw_motor.absolute_angle_last -= gimbal_info.turn_circle_num * CIRCLE;
		gimbal_info.turn_circle_num = 0;
	}
	//��ʼ����е�Ƕ�
	gimbal_info.pitch_motor.ecd_now = ecd_angle_format(gimbal_info.pitch_motor.gimbal_motor_measure->ecd, gimbal_info.pitch_motor.ecd_last, PITCH_ECD_DEL, &gimbal_info.pitch_motor.turn_table_flag);
	gimbal_info.yaw_motor.ecd_now = ecd_angle_format( gimbal_info.yaw_motor.gimbal_motor_measure->ecd, gimbal_info.yaw_motor.ecd_last, YAW_ECD_DEL, &gimbal_info.yaw_motor.turn_table_flag);
	//��ʼ�������ǽǶ�
	gimbal_info.yaw_motor.gyro_now = gyro_angle_format(gimbal_info.yaw_motor.gyro_last, *(gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET), &gimbal_info.yaw_motor.turn_table_flag);
	gimbal_info.pitch_motor.gyro_now = gyro_angle_format(gimbal_info.pitch_motor.gyro_last, *(gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET), &gimbal_info.pitch_motor.turn_table_flag);

	//�Ƕȸ��� �⻷
	if(gimbal_info.gimbal_mode == GIMBAL_ENCONDE)
	{
		fp32 angle_ecd_Y = 0.0f;	//��ʱ����
		//��һ�νǶȸ���  �⻷
		gimbal_info.pitch_motor.relative_angle_last = gimbal_info.pitch_motor.relative_angle;
		gimbal_info.yaw_motor.relative_angle_last = gimbal_info.yaw_motor.relative_angle;
		//��ԽǶȸ���(relative angle) yaw����Ҫ����Ȧ��(rad)
		gimbal_info.pitch_motor.relative_angle = gimbal_info.pitch_motor.ecd_now;
		angle_ecd_Y = gimbal_info.yaw_motor.ecd_now + gimbal_info.turn_circle_num * 2 * PI;
		gimbal_info.yaw_motor.relative_angle = calc_turn_angle(gimbal_info.yaw_motor.relative_angle_last, angle_ecd_Y, &gimbal_info.turn_circle_num);
		//���ٶȸ��� �ڻ�
		gimbal_info.pitch_motor.speed = gimbal_info.pitch_motor.gimbal_motor_measure->speed_rpm * Ecd_to_Rad;	//�������ת��ֵrpm
		gimbal_info.yaw_motor.speed = gimbal_info.yaw_motor.gimbal_motor_measure->speed_rpm * Ecd_to_Rad;
	}
	else if(gimbal_info.gimbal_mode == GIMBAL_GYRO)
	{
		fp32 angle_gyro_Y = 0.0f;	//��ʱ����
		//��һ�νǶȸ���  �⻷
		gimbal_info.pitch_motor.absolute_angle_last = gimbal_info.pitch_motor.absolute_angle;
		gimbal_info.yaw_motor.absolute_angle_last = gimbal_info.yaw_motor.absolute_angle;
		//���ԽǶȼ���(absolute angle) yaw����Ҫ����Ȧ��(rad)
		angle_gyro_Y = gimbal_info.yaw_motor.gyro_now + gimbal_info.turn_circle_num * 2 *PI;
		gimbal_info.pitch_motor.absolute_angle = gimbal_info.pitch_motor.ecd_now;
		gimbal_info.yaw_motor.absolute_angle = calc_turn_angle(gimbal_info.yaw_motor.absolute_angle_last, angle_gyro_Y, &gimbal_info.turn_circle_num);
		//���ٶȸ��� �ڻ�
		gimbal_info.yaw_motor.speed = *(gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET);		//�����Ƿ���ת��rpm
		gimbal_info.pitch_motor.speed = *(gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);
	}
	else//���޸�
	{
		gimbal_info.yaw_motor.speed = gimbal_info.yaw_motor.speed_set;
		gimbal_info.pitch_motor.speed = gimbal_info.pitch_motor.speed_set;
	}
	//������һ��ԭʼ�Ƕ�
	gimbal_info.yaw_motor.ecd_last = gimbal_info.yaw_motor.gimbal_motor_measure->ecd;
	gimbal_info.pitch_motor.ecd_last = gimbal_info.pitch_motor.gimbal_motor_measure->ecd;
	gimbal_info.yaw_motor.gyro_last = *(gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
	gimbal_info.pitch_motor.gyro_last = *(gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);
	//��ѹ����
	gimbal_info.yaw_motor.voltage_give_last = gimbal_info.yaw_motor.voltage_give;
	gimbal_info.pitch_motor.voltage_give_last = gimbal_info.pitch_motor.voltage_give;
}
//����������
static void Gimbal_Control(void)
{
	fp32 add_yaw_angle = 0.0f;
	fp32 add_pitch_angle = 0.0f;
	fp32 rc_yaw_ch, rc_pitch_ch, auto_yaw_ch, auto_pitch_ch;
	uint8_t vision_flag = 0;
	//����ң�ؼ��������
	gimbal_rc_process(&rc_yaw_ch, &rc_pitch_ch);
	//������������
	gimbal_auto_process(&auto_yaw_ch, &auto_pitch_ch);
	//�������鲢ʶ��Ŀ��
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
	//ѡ����������� �Ƕȵ�����
	switch(gimbal_info.gimbal_mode)
	{
		case GIMBAL_CALI:		//У׼ģʽ ��δʵ��
		{
			Gimbal_Calibration_mode();
			return;
		}
		case GIMBAL_RELAX:		//ʧ��ģʽ
		{
			Gimbal_Relax_mode();
			break;
		}
		case GIMBAL_GYRO:		//�����ǿ���ģʽ
		{
			Gimbal_Gyro_mode(&add_pitch_angle, &add_yaw_angle, vision_flag);
			break;
		}
		case GIMBAL_ENCONDE:	//��е����ģʽ
		{
			Gimbal_Enconde_mode(&add_pitch_angle, &add_yaw_angle, vision_flag);
			break;
		}
	}
	//���д���
	if( !gimbal_info.turn_mid_flag )
	{
		//��������
		if(gimbal_info.pitch_motor.relative_angle > 0.1f || gimbal_info.pitch_motor.relative_angle < -0.1f)
		{
			add_pitch_angle = gimbal_info.pitch_motor.relative_angle < 0.0f ? GIMBAL_RETURN_PITCH : -GIMBAL_RETURN_PITCH;		//��̧��pitch��
			add_yaw_angle = 0.0f;
		}
		else if(gimbal_info.yaw_motor.relative_angle > 0.1f || gimbal_info.yaw_motor.relative_angle < -0.1f)
		{
			add_pitch_angle = 0.0f;
			add_yaw_angle = gimbal_info.yaw_motor.relative_angle < 0.0f ? GIMBAL_RETURN_YAW : -GIMBAL_RETURN_YAW;
		}
		else	//�ѻ���
		{
			gimbal_info.turn_mid_flag = 1;
		}
		//�Ƕȵ���
		gimbal_info.pitch_motor.relative_angle_set += add_pitch_angle;
		gimbal_info.yaw_motor.relative_angle_set += add_yaw_angle;
	}
	//��ѹ
	gimbal_info.yaw_motor.voltage_give = (int16_t)gimbal_info.yaw_motor.voltage_set;
	gimbal_info.pitch_motor.voltage_give = (int16_t)gimbal_info.pitch_motor.voltage_set;
}
static void Gimbal_Calibration_mode(void)
{
	return;
}
//��̨����ģʽ
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
	//���б�־��0
	gimbal_info.turn_mid_flag = 0;
	gimbal_info.turn_circle_num = 0;
}
//��̨�ɵ������������
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
			//ʶ��Ŀ�� PID�л�Ϊ����
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
		case GIMBAL_CENTER:	//��̨yaw���� ���ʼ����������
			//����Ƕ��ڻ��з�Χ�� ����Ϊ��Ч
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
			else	//������Ч
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
		//����Ϊֻ��������ģʽ����Ч
		case GIMBAL_NORMAL_GR:
		case GIMBAL_AUTO_GR:
			gimbal_info.gimbal_behavior = gimbal_info.gimbal_behavior_last;
			break;
	}
}
//��̨�������ǿ���
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
			//ʶ��Ŀ�� PID�л�Ϊ����
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
		case GIMBAL_NORMAL_GR:			//С����ģʽ�µ��ֶ�ģʽ
			gimbal_info.pitch_motor.speed_set = PID_Calc(&pitch_ecd_speed_rc_pid, gimbal_info.pitch_motor.relative_angle, gimbal_info.pitch_motor.relative_angle_set);
			gimbal_info.pitch_motor.voltage_set = PID_Calc(&pitch_ecd_angle_rc_pid, gimbal_info.pitch_motor.speed, gimbal_info.pitch_motor.speed_set);
			gimbal_info.yaw_motor.speed_set = PID_Calc(&yaw_gyromode_speed_rc_pid, gimbal_info.yaw_motor.relative_angle, gimbal_info.yaw_motor.relative_angle_set);
			gimbal_info.yaw_motor.voltage_set = PID_Calc(&yaw_gyromode_angle_rc_pid, gimbal_info.yaw_motor.speed, gimbal_info.yaw_motor.speed_set);
			break;
		case GIMBAL_AUTO_GR:			//С����ģʽ�µ�����ģʽ
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
			//�ڴ�ģʽ��Ϊ���̻��� �ڴ�ģʽ�¸���Ϊ��ִ��
			gimbal_info.gimbal_behavior = gimbal_info.gimbal_behavior_last;
			break;
	}
}
//ģʽ�ı䴦������
static void gimbal_mode_change_save(Gimbal_Mode_e last, Gimbal_Mode_e now)
{
	//����yawȦ��
	
	//�����л�����
	
}
//ң�������ݴ���
static void gimbal_rc_process(fp32 *yaw_add, fp32 *pitch_add)
{
	//��ʼ����ر���
	int16_t yaw_channel = 0;
	int16_t pitch_channel = 0;
	fp32 mouse_x_channel = 0.0f;
	fp32 mouse_y_channel = 0.0f;
	//ң������������
//	if( gimbal_info.gimbal_RC->rc.ch[GIMBAL_YAW_CHANNEL] > RC_DEADLINE || gimbal_info.gimbal_RC->rc.ch[GIMBAL_YAW_CHANNEL] < -RC_DEADLINE )
//		yaw_channel = gimbal_info.gimbal_RC->rc.ch[GIMBAL_YAW_CHANNEL];
//	if( gimbal_info.gimbal_RC->rc.ch[GIMBAL_PITCH_CHANNEL] > RC_DEADLINE || gimbal_info.gimbal_RC->rc.ch[GIMBAL_PITCH_CHANNEL] < -RC_DEADLINE )
//		pitch_channel = gimbal_info.gimbal_RC->rc.ch[GIMBAL_PITCH_CHANNEL];
	i_dead_zone_del(gimbal_info.gimbal_RC->rc.ch[GIMBAL_YAW_CHANNEL], &yaw_channel, RC_DEADLINE);
	//������ݴ��� �˲�����?
	mouse_x_channel = gimbal_info.gimbal_RC->mouse.x * PITCH_MOUSE_SEN;
	mouse_y_channel = gimbal_info.gimbal_RC->mouse.y * YAW_MOUSE_SEN;
	//ң�����ݱ���ת��
	*yaw_add = yaw_channel * YAW_RC_SEN + mouse_x_channel;
	*pitch_add = pitch_channel * PITCH_RC_SEN + mouse_y_channel;
}
//�������ݴ���
static void gimbal_auto_process(fp32 *yaw_add, fp32 *pitch_add)
{
	fp32 auto_yaw_channel = 0.0f;
	fp32 auto_pitch_channel = 0.0f;
	//��Ҫʵ�ʶ����ݽ��е��� �Ӽ�ƫ������
	auto_yaw_channel = gimbal_info.gimbal_AUTO_ctrl->yaw_angle * YAW_AUTO_SEN;
	auto_pitch_channel = gimbal_info.gimbal_AUTO_ctrl->pitch_angle * PITCH_AUTO_SEN;
	*yaw_add = auto_yaw_channel;
	*pitch_add = auto_pitch_channel;
}
//���Ϳ��Ƶ�ѹ
static void Gimbal_Send_Voltage(void)
{
	//����Ƿ�װ
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

	//��̨��ң��������״̬��relax ״̬��canָ��Ϊ0����ʹ��Voltage����Ϊ��ķ������Ǳ�֤ң��������һ��ʹ����ֹ̨ͣ
	if (!(toe_is_error(YawGimbalMotorTOE) && toe_is_error(PitchGimbalMotorTOE) && toe_is_error(TriggerMotorTOE)))
	{
		//������ģʽΪ����״̬ Ϊ�˲��ܸ��� ֱ�ӷ��͵���Ϊ0	
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


