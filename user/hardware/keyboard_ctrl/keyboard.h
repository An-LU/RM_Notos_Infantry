#ifndef __KEYBOARD_H__
#define __KEYBOARD_H__

#include "main.h"


#define UPDATA_STAMP_TIME 10



//extern Key_u key_press;			//��������
//extern Key_u key_long_press;	//��������
extern Key_u key_status;		//����ģ�⿪��

//�������ݴ���
void key_dealwith(const RC_ctrl_s *);
void chassis_key_ctrl(const RC_ctrl_s *RC_Data, fp32 *vx_ch, fp32 *vy_ch);

#endif


