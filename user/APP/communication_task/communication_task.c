#include "communication_task.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "usart.h"
#include "miniPC.h"
#include "string.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

static VOFA_UsartSend_s VOFA_UsartSend;
static fp32 send_buff[COMMUNICATION_LENGTH];
const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};	//VOFA+尾帧协议

static const Gimbal_Motor_s *yaw_motor, *pitch_motor;
static const Chassis_Control_s *chassis_control;

void communication_Init(void);
static void gimbal_debug_send(void);
static void chassis_debug_send(void);
static void buff_clean(void);

void communication_task(void *pvParameters)
{
	communication_Init();
	while(1)
	{
		//miniPC需要的数据
		Data_Send_to_Vision();
		//调试
		gimbal_debug_send();
		//系统延时
        vTaskDelay(COMMUNICATION_CONTROL_TIME_MS);
	}
}

void communication_Init(void)
{
	VOFA_UsartSend.tail[0] = 0x00;
	VOFA_UsartSend.tail[1] = 0x00;
	VOFA_UsartSend.tail[2] = 0x80;
	VOFA_UsartSend.tail[3] = 0x7f;
	yaw_motor = get_gimbal_yaw_motor_point();
	pitch_motor = get_gimbal_pitch_motor_point();
}
static void chassis_debug_send(void)
{
	
}
static void gimbal_debug_send(void)
{
	VOFA_UsartSend.send_buff[0] = yaw_motor->relative_angle;
	VOFA_UsartSend.send_buff[1] = pitch_motor->relative_angle;
	VOFA_UsartSend.send_buff[2] = yaw_motor->absolute_angle;
	VOFA_UsartSend.send_buff[3] = pitch_motor->absolute_angle;
	VOFA_UsartSend.send_buff[4] = yaw_motor->speed;
	VOFA_UsartSend.send_buff[5] = pitch_motor->speed;
	DebugUsart_SendData((uint8_t *)&VOFA_UsartSend, sizeof(VOFA_UsartSend));
//	send_buff[0] = yaw_motor->relative_angle;
//	send_buff[1] = pitch_motor->relative_angle;
//	send_buff[2] = yaw_motor->absolute_angle;
//	send_buff[3] = pitch_motor->absolute_angle;
//	send_buff[4] = yaw_motor->speed;
//	send_buff[5] = pitch_motor->speed;
//	DebugUsart_SendData((uint8_t *)send_buff, sizeof(fp32) * COMMUNICATION_LENGTH);
//	DebugUsart_SendData((uint8_t *)tail, 4);
}
static void buff_clean(void)
{
	memset((void *)&VOFA_UsartSend, 0, sizeof(VOFA_UsartSend));
	//memset((void *)send_buff, 0, sizeof(fp32) * COMMUNICATION_LENGTH);
}
