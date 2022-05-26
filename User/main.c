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
	//裁判系统
	Judge_Usart_Init();
	//调试
	Debug_Usart_Init();
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
