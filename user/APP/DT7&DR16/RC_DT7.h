#ifndef RC_DT7_H_
#define RC_DT7_H_

#include "main.h"
//操作模式
//	USA		//美国手
//	JP		//日本手
//	CN		//中国手
#define RC_MODE USA

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)

typedef union
{
	uint16_t key_code;
	struct
	{
		uint16_t W:1;
		uint16_t S:1;
		uint16_t A:1;
		uint16_t D:1;
		uint16_t SHIFT:1;
		uint16_t CTRL:1;
		uint16_t Q:1;
		uint16_t E:1;
		uint16_t R:1;
		uint16_t F:1;
		uint16_t G:1;
		uint16_t Z:1;
		uint16_t X:1;
		uint16_t C:1;
		uint16_t V:1;
		uint16_t B:1;
	}bit;
}Key_u;
typedef struct 
{
	Key_u key_short_press;		//按键短按
	Key_u key_long_press;		//按键长按
	Key_u key_click;			//按键点击
	uint8_t key_press_time[16];	//按键按下时间
}Key_Data_s;
typedef struct
{
	struct
	{
		int16_t ch[5];
		uint8_t s[2];
	}rc;
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	}mouse;
	Key_u key; 
	Key_Data_s key_data;
}RC_ctrl_s;

//内联函数代替宏
inline static bool switch_is_up(uint8_t s) { return (s == 1); }
inline static bool switch_is_mid(uint8_t s) { return (s == 3); }
inline static bool switch_is_down(uint8_t s) { return (s == 2); }
//函数声明
extern void remote_control_init(void);
extern const RC_ctrl_s *get_remote_control_point(void);
extern bool RC_data_is_error(void);
extern void slove_RC_lost(void);
extern void slove_data_error(void);

#endif	/* RC_DT7_H_ */


