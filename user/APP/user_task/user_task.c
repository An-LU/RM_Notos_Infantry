/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       user_task.c/h
  * @brief      一个普通心跳程序，如果设备无错误，绿灯1Hz闪烁,然后获取姿态角
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "User_Task.h"
#include "main.h"
#include "Gimbal_Task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "led.h"
#include "chassis_task.h"
#include "Detect_Task.h"
#include "INS_Task.h"
#include "oled.h"
#define user_is_error() toe_is_error(errorListLength)

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t UserTaskStack;
#endif

int fputc(int ch, FILE *f)  
{  
    USART2->DR = (u8) ch;  
    /* Loop until the end of transmission */  
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
    {  
    }
    return ch;  
}//printf方法重写
//姿态角 单位度
fp32 angle_degree[3] = {0.0f, 0.0f, 0.0f};

void UserTask(void *pvParameters)
{

    const volatile fp32 *angle;
	const Chassis_Control_s *moto;
	const motor_measure_t *yaw_mode;
    //获取姿态角指针
    angle = get_INS_angle_point();
	moto = get_classis_info_point();
	yaw_mode = get_Yaw_Gimbal_Motor_Measure_Point();
	float moto_speed,moto_accel;
    while (1)
    {

        //姿态角 将rad 变成 度，除这里的姿态角的单位为度，其他地方的姿态角，单位均为弧度
        angle_degree[0] = (*(angle + INS_YAW_ADDRESS_OFFSET)) * 57.3f;
        angle_degree[1] = (*(angle + INS_PITCH_ADDRESS_OFFSET)) * 57.3f;
        angle_degree[2] = (*(angle + INS_ROLL_ADDRESS_OFFSET)) * 57.3f;

        if (!user_is_error())
        {
            led_green_on();
        }
		moto_accel = moto->chassis_motor[0].accel;
		moto_speed = moto->chassis_motor[0].speed-moto->chassis_motor[1].speed+moto->chassis_motor[2].speed-moto->chassis_motor[3].speed;
		moto_speed = moto->chassis_vx*moto->chassis_vx*12.31;
		//USART_SendData(USART2,(unsigned char)yaw_mode->speed_rpm);
		printf("power_out: %f \n\n",moto_speed);
		//printf("moto_speed: %f \n\n",moto->motor_chassis[0].speed);
		//printf("moto_speed: %f \n\n",-moto->motor_chassis[1].speed);
		//printf("moto_speed: %f \n\n",moto->motor_chassis[2].speed);
        //printf("moto_speed: %f \n\n",moto_speed);
		vTaskDelay(500);
        //led_green_off();
        //vTaskDelay(500);
#if INCLUDE_uxTaskGetStackHighWaterMark
        UserTaskStack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
