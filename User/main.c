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

//�ĸ�24v ��� ���ο��� ��� 709us
#define POWER_CTRL_ONE_BY_ONE_TIME 709

void BSP_init(void)
{
    //�ж��� 4
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    //��ʼ���δ�ʱ��
    delay_init(configTICK_RATE_HZ);
    //��ˮ�ƣ����̵Ƴ�ʼ��
    led_configuration();
	
//	OLED_Init();
//	OLED_ColorTurn(0);//0������ʾ��1 ��ɫ��ʾ
//	OLED_DisplayTurn(0);//0������ʾ 1 ��Ļ��ת��ʾ
    //stm32 �����¶ȴ�������ʼ��
    temperature_ADC_init();
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
    //stm32 �������������ʼ��
    RNG_init();
#endif
    //24V������ƿ� ��ʼ��
    power_ctrl_configuration();
    //Ħ���ֵ��PWM��ʼ��
    fric_PWM_configuration();
    //��������ʼ��
    buzzer_init(30000, 90);
    //����IO��ʼ��
    laser_configuration();
	//�Ӿ����ڳ�ʼ��
	Vision_Usart_Init();
	Vision_Init();
	//����ϵͳ
	Judge_Usart_Init();
	//����
	Debug_Usart_Init();
	//�������ݳ�ʼ��
	super_cap_configuration();
    //��ʱ��6 ��ʼ��
    TIM6_Init(60000, 90);
    //CAN�ӿڳ�ʼ��
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);//Ƶ��Ϊ1m

    //24v ��� �����ϵ�
    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
    {
        power_ctrl_on(i);
        delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
    }
    //ң������ʼ��
    remote_control_init();//������100000
    //flash��ȡ��������У׼ֵ�Żض�Ӧ����
    cali_param_init();
}
