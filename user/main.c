/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       main.c/h
  * @brief      stm32初始化以及开始任务freeRTOS。h文件定义相关全局宏定义以及
  *             typedef 一些常用数据类型
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *

	    在此感谢大疆对步兵开代码的贡献，拜读大作深感贵公司的发展惊人的速度的背后是一代又一代的
	人的技术累积。在此写下我对此代码的心得和体会。
		  虽然这次的代码是基于步兵的开源代码。我经历几个月的代码调试和研读对其略知一二。为实验室的
	的技术累积和NFU战队的发展贡献自己的力量，希望后浪能越挫越勇更上一层楼。
			虽然我们起步晚，技术累积不足。我们已经是历史的胜利者了。
			
			1.此代码的框架基于Freertos。
			1.1FreeRtos的任务大致如下
				1.1.1底盘的运动分解，底盘运动的set的值设置。
				1.1.2云台的运动
				1.1.3遥控数据的获取任务
				1.1.4姿态角转换任务
				1.1.5校准任务
				1.1.6任务掉线停止任务
				1.1.7空闲的用户函数，给用户开发用的
			2.1数据的通信方式
			  2.1.1CAN中断。四个底盘电机和云台电机将会每1M的速度发送此刻的编码值和电流值。
			3.1云台的pitch轴和yaw轴的变化不一致，将两者的控制分开。
				
			
	
  ****************************(C) NFU 2020.6.7 ****************************
  */
#include "main.h"

#include "stm32f4xx.h"

#include "adc.h"
#include "buzzer.h"
#include "can.h"
#include "delay.h"
#include "flash.h"
#include "fric.h"
#include "laser.h"
#include "led.h"
#include "power_ctrl.h"
#include "rc.h"
#include "rng.h"
#include "sys.h"
#include "timer.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "calibrate_task.h"
#include "RC_DT7.h"
#include "start_task.h"
#include "oled.h"
#include "usart.h"
#include "super_cap.h"
#include "miniPC.h"

void BSP_init(void);

int main(void)
{  
    BSP_init();
	
    delay_ms(100);
	//USART_SendData(UART4,11);
    startTast();
    vTaskStartScheduler();
    while (1)
    { 
			delay_ms(500);

//			flow_led_toggle(7);
    }
}

//四个24v 输出 依次开启 间隔 709us
#define POWER_CTRL_ONE_BY_ONE_TIME 709

void BSP_init(void)
{
    //中断组 4
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    //初始化滴答时钟
    delay_init(configTICK_RATE_HZ);
    //流水灯，红绿灯初始化
    led_configuration();
	
//	OLED_Init();
//	OLED_ColorTurn(0);//0正常显示，1 反色显示
//	OLED_DisplayTurn(0);//0正常显示 1 屏幕翻转显示
    //stm32 板载温度传感器初始化
    temperature_ADC_init();
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
    //stm32 随机数发生器初始化
    RNG_init();
#endif
    //24V输出控制口 初始化
    power_ctrl_configuration();
    //摩擦轮电机PWM初始化
    fric_PWM_configuration();
    //蜂鸣器初始化
    buzzer_init(30000, 90);
    //激光IO初始化
    laser_configuration();
	//视觉串口初始化
	Vision_Usart_Init();
	Vision_Init();
	
	Judge_Usart_Init();
	//超级电容初始化
	super_cap_configuration();
    //定时器6 初始化
    TIM6_Init(60000, 90);
    //CAN接口初始化
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);//频率为1m

    //24v 输出 依次上电
    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
    {
        power_ctrl_on(i);
        delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
    }
    //遥控器初始化
    remote_control_init();//波特率100000
    //flash读取函数，把校准值放回对应参数
    cali_param_init();
}
