#include "motor_control.h"

//处理电机反馈初始机械角度值
//#define ECD_DEL(ecd,ecd_del)						\
//	{												\
//		if( (ecd) <= 8191 && (ecd) >= (ecd_del) )	\
//			(ecd) -= (ecd_del);						\
//		else if( (ecd) < (ecd_del) )				\
//			(ecd) = (ecd) - (ecd_del) + 8192;		\
//	}
//使用内联函数代替宏
inline static void i_ecd_del(uint16_t ecd, uint16_t ecd_del)
{
	if(ecd >= ecd_del)
		ecd -= ecd_del;
	else
		ecd = ecd - ecd_del - ecd + 8192;
}	

//电机反馈机械角度规整 0~8191--> -2PI~2PI (rad) (complete)
fp32 ecd_angle_format(uint16_t ecd, uint16_t last_ecd, const uint16_t offset_ecd, uint8_t *turn_table_flag)
{
	int32_t angle = 0;
	int16_t err_ecd;
	//计算相对机械角度
	i_ecd_del(ecd, offset_ecd);
	i_ecd_del(last_ecd, offset_ecd);
//	ECD_DEL(ecd, offset_ecd);
//	ECD_DEL(last_ecd, offset_ecd);
	//机械角度差
	err_ecd = ecd - last_ecd;
	//计算角度码盘
	if (err_ecd > 6000)		//电机反向转过起始点 0(8191) 相对位置应该为负数
	{
		*turn_table_flag = 0;
	}
	else if (err_ecd < -6000)// 电机正转经过起始点 8191(0) 相对位置应该为正数
	{
		*turn_table_flag = 1;
	}
	if (*turn_table_flag)		//正角度码盘 (0,2PI)
	{
		angle = ecd;
	}
	else
	{
		angle = ecd - 8192;
	}
	//电机编码值转换为rad
	return angle * Ecd_to_Rad;
}
//陀螺仪角度规整 -PI~PI--> -2PI~2PI (rad) (complete)
fp32 gyro_angle_format(const fp32 angle_now, const fp32 angle_last, uint8_t* turn_table_flag)
{
	fp32 angle;
	if (angle_now > 0 && angle_now < 1.5f && angle_last < 0 && angle_last > -1.5f)
	{
		*turn_table_flag = 1;
	}
	else if (angle_now < 0 && angle_now > -1.5f && angle_last > 0 && angle_last < 1.5f)
	{
		*turn_table_flag = 0;
	}
	if (*turn_table_flag)
	{
		if (angle_now < 0)
			angle = angle_now + 2 * PI;
		else
			angle = angle_now;
	}
	else
	{
		if (angle_now > 0)
			angle = angle_now - 2 * PI;
		else
			angle = angle_now;
	}
	return angle;
}
//计算云台圈数并返回叠加圈数后的角度角度
fp32 calc_turn_angle(const fp32 *angle_last, const fp32 *angle_now, uint8_t *turn_circle_num)
{
	fp32 angle = 0.0f;
	//正向圈数为正 逆时针
	if(*angle_now - *angle_last < -4.0f)
	{
		(*turn_circle_num)++;
		angle = *angle_now + 2 * PI;
	}
	//反向圈数为负 顺时针
	else if(*angle_now - *angle_last > 4.0f)
	{
		(*turn_circle_num)--;
		angle = *angle_now - 2 * PI;
	}
	return angle;
}

