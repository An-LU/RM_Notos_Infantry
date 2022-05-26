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
inline static uint16_t i_ecd_del(uint16_t ecd, uint16_t ecd_del)
{
	if(ecd >= ecd_del)
		ecd -= ecd_del;
	else
		ecd = ecd - ecd_del + 8192;
	return ecd;
}	

/*brief:	���������е�Ƕȹ��� 0~8191--> -2PI~2PI (rad)
*param(in):	angle_last:��һ�νǶ�(ecd) angle_now:��ǰ�Ƕ�(ecd) *turn_table_flag:�Ƕ���������(point)
*param(out):angle:�����ĽǶ�(rad)
*status:	complete
*/
fp32 ecd_angle_format(uint16_t ecd, uint16_t last_ecd, const uint16_t offset_ecd, uint8_t *turn_table_flag)
{
	int32_t angle = 0;
	int16_t err_ecd;
	//������Ի�е�Ƕ�
	ecd = i_ecd_del(ecd, offset_ecd);
	last_ecd = i_ecd_del(last_ecd, offset_ecd);
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
/*brief:	�����ǽǶȹ��� -PI~PI--> -2PI~2PI (rad)
*param(in):	angle_last:��һ�νǶ�(rad) angle_now:��ǰ�Ƕ�(rad) *turn_table_flag:�Ƕ���������(point)
*param(out):angle:�����ĽǶ�
*status:	complete
*/
fp32 gyro_angle_format(const fp32 angle_last, const fp32 angle_now, uint8_t *turn_table_flag)
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
/*brief:	������̨Ȧ�������ص���Ȧ����ĽǶȽǶ�
*param(in):	angle_last:��һ�νǶ�(rad) angle_now:��ǰ�Ƕ�(rad) *turn_circle_num:Ȧ��(point)
*param(out):angle:���Ӻ�ĽǶ�
*status:	complete
*/
fp32 calc_turn_angle(const fp32 angle_last, const fp32 angle_now, int16_t *turn_circle_num)
{
	fp32 angle = 0.0f;
	//����Ȧ��Ϊ�� ��ʱ��
	if(angle_now - angle_last < -4.0f)
	{
		(*turn_circle_num)++;
		angle = angle_now + 2 * PI;
	}
	//����Ȧ��Ϊ�� ˳ʱ��
	else if(angle_now - angle_last > 4.0f)
	{
		(*turn_circle_num)--;
		angle = angle_now - 2 * PI;
	}
	else
	{
		angle = angle_now;
	}
	return angle;
}
/*brief:	��ʼ����ʼ�Ƕ�����
*param(in):	ecd:��ǰ��е�Ƕ� ecd_del:У��ɾ���Ƕ� *turn_table_flag:�Ƕ���������(point)
*param(out):void
*status:	debug
*/
void angle_table_init(uint16_t ecd, const uint16_t ecd_del, uint8_t *turn_table_flag)
{
	ecd = i_ecd_del(ecd, ecd_del);
	if(ecd < 4096)
		*turn_table_flag = 1;
	else
		*turn_table_flag = 0;
}

