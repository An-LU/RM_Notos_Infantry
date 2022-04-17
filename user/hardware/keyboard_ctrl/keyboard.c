#include "keyboard.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
//#include "test.h"
#include "arm_math.h"

const uint16_t TIME_SLOPE_ADD = 10;
const uint16_t TIME_SLOPE_DEC = 100;
//Key_u key_press;		//按键按下
//Key_u key_long_press;	//按键长按
Key_u key_status;		//按键模拟开关

Key_Data_s key_data;

static Key_u key_last_status;
static uint8_t key_press_time[16];

static fp32 move_fron_slow, move_back_slow, move_left_slow, move_right_slow;//各个方向斜坡输出量
static fp32 fron_press_time, back_press_time, left_press_time, right_press_time;//按键按下时间，自变量x y=a*sqrt(b*x)
//键盘数据处理
void key_dealwith(const RC_ctrl_t *);
//键盘控制量
void Key_Ctrl(fp32 *vx_set, fp32 *vy_set);

//static portTickType ulCurrentTime;
//ulCurrentTime = xTaskGetTickCount();//当前系统时间
//if(ulCurrentTime >= delay_time)
//{
//	delay_time = ulCurrentTime + 10;
//	key_scan();
//}
void key_dealwith(const RC_ctrl_t *RC_Data)
{
	if(!key_last_status.bit.A && RC_Data->key.bit.A)	key_status.bit.A = ~key_status.bit.A;
	if(!key_last_status.bit.Q && RC_Data->key.bit.Q)	key_status.bit.Q = ~key_status.bit.Q;
	if(!key_last_status.bit.X && RC_Data->key.bit.X)	key_status.bit.X = ~key_status.bit.X;
	if(!key_last_status.bit.Z && RC_Data->key.bit.Z)	key_status.bit.Z = ~key_status.bit.Z;
	
	key_last_status.v = RC_Data->key.v;
}
fp32 keyboard_turn_ratio(uint8_t status, fp32 *press_time)
{
	fp32 slope_ratio = 0.2f * sqrt(*press_time * 0.2f);
	if(status)
	{
		if(slope_ratio < 1)
			*press_time += TIME_SLOPE_ADD;
	}
	else
	{
		if(slope_ratio > 0)
		{
			*press_time -= TIME_SLOPE_DEC;
			if(*press_time < 0)
				*press_time = 0;
		}
	}
	return slope_ratio;
}
void chassis_key_ctrl(const RC_ctrl_t *RC_Data, fp32 *vx_ch, fp32 *vy_ch)
{
	static uint16_t key_press_time;
	static portTickType current_time;
	static uint16_t delay_time = 0;
	static fp32 speed_ratio = 0.0f;
	
	current_time = xTaskGetTickCount();
	
	//反方向斜坡清零
	if(RC_Data->key.bit.W)	back_press_time = 0;
	if(RC_Data->key.bit.S)	fron_press_time = 0;
	if(RC_Data->key.bit.A)	right_press_time = 0;
	if(RC_Data->key.bit.D)	left_press_time = 0;
	//10ms更新一次斜坡输出量
	if(current_time >= delay_time)
	{
		delay_time = current_time + UPDATA_STAMP_TIME;
		move_fron_slow = keyboard_turn_ratio((uint8_t)RC_Data->key.bit.W, &fron_press_time);// * MAX_CHASSIS_SPEED_X;
		move_back_slow = keyboard_turn_ratio((uint8_t)RC_Data->key.bit.S, &back_press_time);// * -MAX_CHASSIS_SPEED_X;
		move_left_slow = keyboard_turn_ratio((uint8_t)RC_Data->key.bit.A, &left_press_time);// * MAX_CHASSIS_SPEED_Y;
		move_right_slow = keyboard_turn_ratio((uint8_t)RC_Data->key.bit.D, &right_press_time);// * -MAX_CHASSIS_SPEED_Y;
	}
	*vx_ch = move_fron_slow + move_back_slow;
	*vy_ch = move_left_slow + move_right_slow;
}


