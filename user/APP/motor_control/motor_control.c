#include "motor_control.h"

//������������ʼ��е�Ƕ�ֵ
//#define ECD_DEL(ecd,ecd_del)						\
//	{												\
//		if( (ecd) <= 8191 && (ecd) >= (ecd_del) )	\
//			(ecd) -= (ecd_del);						\
//		else if( (ecd) < (ecd_del) )				\
//			(ecd) = (ecd) - (ecd_del) + 8192;		\
//	}
//ʹ���������������
inline static void i_ecd_del(uint16_t ecd, uint16_t ecd_del)
{
	if(ecd >= ecd_del)
		ecd -= ecd_del;
	else
		ecd = ecd - ecd_del - ecd + 8192;
}	

//���������е�Ƕȹ��� 0~8191--> -2PI~2PI (rad) (complete)
fp32 ecd_angle_format(uint16_t ecd, uint16_t last_ecd, const uint16_t offset_ecd, uint8_t *turn_table_flag)
{
	int32_t angle = 0;
	int16_t err_ecd;
	//������Ի�е�Ƕ�
	i_ecd_del(ecd, offset_ecd);
	i_ecd_del(last_ecd, offset_ecd);
//	ECD_DEL(ecd, offset_ecd);
//	ECD_DEL(last_ecd, offset_ecd);
	//��е�ǶȲ�
	err_ecd = ecd - last_ecd;
	//����Ƕ�����
	if (err_ecd > 6000)		//�������ת����ʼ�� 0(8191) ���λ��Ӧ��Ϊ����
	{
		*turn_table_flag = 0;
	}
	else if (err_ecd < -6000)// �����ת������ʼ�� 8191(0) ���λ��Ӧ��Ϊ����
	{
		*turn_table_flag = 1;
	}
	if (*turn_table_flag)		//���Ƕ����� (0,2PI)
	{
		angle = ecd;
	}
	else
	{
		angle = ecd - 8192;
	}
	//�������ֵת��Ϊrad
	return angle * Ecd_to_Rad;
}
//�����ǽǶȹ��� -PI~PI--> -2PI~2PI (rad) (complete)
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
//������̨Ȧ�������ص���Ȧ����ĽǶȽǶ�
fp32 calc_turn_angle(const fp32 *angle_last, const fp32 *angle_now, uint8_t *turn_circle_num)
{
	fp32 angle = 0.0f;
	//����Ȧ��Ϊ�� ��ʱ��
	if(*angle_now - *angle_last < -4.0f)
	{
		(*turn_circle_num)++;
		angle = *angle_now + 2 * PI;
	}
	//����Ȧ��Ϊ�� ˳ʱ��
	else if(*angle_now - *angle_last > 4.0f)
	{
		(*turn_circle_num)--;
		angle = *angle_now - 2 * PI;
	}
	return angle;
}

