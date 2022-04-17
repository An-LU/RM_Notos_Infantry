#ifndef __KEYBOARD_H__
#define __KEYBOARD_H__

#include "main.h"


#define UPDATA_STAMP_TIME 10



//extern Key_u key_press;			//按键按下
//extern Key_u key_long_press;	//按键长按
extern Key_u key_status;		//按键模拟开关

//键盘数据处理
void key_dealwith(const RC_ctrl_s *);
void chassis_key_ctrl(const RC_ctrl_s *RC_Data, fp32 *vx_ch, fp32 *vy_ch);

#endif


