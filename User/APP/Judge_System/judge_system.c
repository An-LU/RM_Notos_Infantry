#include "judge_system.h"
#include "crc.h"
#include "string.h"

//static uint8_t Judge_Data[100];

/************************************裁判系统读取数据************************************/
static ext_game_state_t			ext_game_state;				//0001 比赛状态数据
static ext_game_result_t		ext_game_result;			//0002 比赛结果数据
static ext_game_robot_HP_t		ext_game_robot_HP;			//0003 比赛机器人血量数据
static ext_dart_status_t		ext_dart_status;			//0004 飞镖发射状态
//static ext_ICRA_buff_debuff_zone_status_t ext_ICRA_buff_debuff_zone_status;	//0005 人工智能挑战赛
static ext_event_data_t			ext_event_data;				//0101 场地事件数据
static ext_supply_projectile_action_t	ext_supply_projectile_action;	//0102 场地补给站动作标识数据
static ext_referee_warning_t	ext_referee_warning;		//0104 裁判系统警告信息
static ext_dart_remaining_time_t	ext_dart_remaining_time;	//0105 飞镖发射口倒计时
static ext_game_robot_state_t	ext_game_robot_state;		//0201 机器人状态数据
static ext_power_heat_data_t	ext_power_heat_data;		//0202 实时功率热量数据
static ext_game_robot_pos_t		ext_game_robot_pos;			//0203 机器人位置数据
static ext_buff_musk_t			ext_buff_musk;				//0204 机器人增益数据
static aerial_robot_energy_t	aerial_robot_energy;		//0205 空中机器人能量状态数据
static ext_robot_hurt_t			ext_robot_hurt;				//0206 伤害状态数据
static ext_shoot_data_t			ext_shoot_data;				//0207 实时射击数据
static ext_bullet_remaining_t	ext_bullet_remaining;		//0208 子弹剩余数量
static ext_rfid_status_t		ext_rfid_status;			//0209 FRID状态
static ext_dart_client_cmd_t	ext_dart_client_cmd;		//020a 飞镖机器人客户端指令数据
static ext_student_interactive_header_data_t	ext_student_interactive_header_data;	//机器人间交互数据
/******************************************************************************************/

bool Robot_is_hit;


uint8_t Robot_Type;	//机器人类型
/* 读取裁判系统信息 */
void Judge_Read_Data(uint8_t *Judge_Usart_Info)
{
	Robot_is_hit = false;
	uint16_t judge_data_len = 0;			//数据总长度
	uint16_t CMD_ID = 0;				//
	//第一帧验证
	if( Judge_Usart_Info[0] == JUDGE_HEAD_SOF )
	{
		//帧头CRC8验证
		if(Verify_CRC8_Check_Sum(Judge_Usart_Info, JUDGE_LENGTH_SOF))
		{
			judge_data_len = JUDGE_LENGTH_SOF + JUDGE_LENGTH_TAIL + JUDGE_LENGTH_CMD + Judge_Usart_Info[DataLength_offset];
			//帧尾crc16验证
			if( Verify_CRC16_Check_Sum( Judge_Usart_Info, judge_data_len))
			{
				CMD_ID = (uint16_t)Judge_Usart_Info[6] << 8 | (uint16_t)Judge_Usart_Info[5];	//低位先发送
				switch( CMD_ID )
				{
					case Judge_Game_StatusData:		//0001 比赛状态数据
						memcpy((void *)(&ext_game_state), (const void *)(Judge_Usart_Info + 7), Judge_Usart_Info[DataLength_offset]);
						break;
					case Judge_Game_ResultData:		//0002 比赛结果数据
						memcpy((void *)(&ext_game_result), (const void *)(Judge_Usart_Info + 7), Judge_Usart_Info[DataLength_offset]);
						break;
					case Judge_Robot_HP:			//0003 比赛机器人血量数据
						memcpy((void *)(&ext_game_robot_HP), (const void *)(Judge_Usart_Info + 7), Judge_Usart_Info[DataLength_offset]);
						break;
					case Judge_Dart_Launch:			//0004 飞镖发射状态
						memcpy((void *)(&ext_dart_status), (const void *)(Judge_Usart_Info + 7), Judge_Usart_Info[DataLength_offset]);
						break;
					case Judge_AI_ChallengeBuff:	//0005 人工智能挑战赛
						//memcpy((void *)(&ext_ICRA_buff_debuff_zone_status), (const void *)(Judge_Usart_Info + 7), Judge_Usart_Info[DataLength_offset]);
						break;
					case Judge_Event_Data:			//0101 场地事件数据
						memcpy((void *)(&ext_event_data), (const void *)(Judge_Usart_Info + 7), Judge_Usart_Info[DataLength_offset]);
						break;
					case Judge_Supply_Station:		//0102 场地补给站动作标识数据
						memcpy((void *)(&ext_supply_projectile_action), (const void *)(Judge_Usart_Info + 7), Judge_Usart_Info[DataLength_offset]);
						break;
					case Judge_Referee_Warning:		//0104 裁判系统警告信息
						memcpy((void *)(&ext_referee_warning), (const void *)(Judge_Usart_Info + 7), Judge_Usart_Info[DataLength_offset]);
						break;
					case Judge_Dart_Countdown:		//0105 飞镖发射口倒计时
						memcpy((void *)(&ext_dart_remaining_time), (const void *)(Judge_Usart_Info + 7), Judge_Usart_Info[DataLength_offset]);
						break;
					case Judge_Robot_State:			//0201 机器人状态数据
						memcpy((void *)(&ext_game_robot_state), (const void *)(Judge_Usart_Info + 7), Judge_Usart_Info[DataLength_offset]);
						break;
					case Judge_Power_Heat:			//0202 实时功率热量数据
						memcpy((void *)(&ext_power_heat_data), (const void *)(Judge_Usart_Info + 7), Judge_Usart_Info[DataLength_offset]);
						break;
					case Judge_Robot_Position:		//0203 机器人位置数据
						memcpy((void *)(&ext_game_robot_pos), (const void *)(Judge_Usart_Info + 7), Judge_Usart_Info[DataLength_offset]);
						break;
					case Judge_Robot_Buff:			//0204 机器人增益数据
						memcpy((void *)(&ext_buff_musk), (const void *)(Judge_Usart_Info + 7), Judge_Usart_Info[DataLength_offset]);
						break;
					case Judge_Aerial_Energy:		//0205 空中机器人能量状态数据
						memcpy((void *)(&aerial_robot_energy), (const void *)(Judge_Usart_Info + 7), Judge_Usart_Info[DataLength_offset]);
						break;
					case Judge_Injury_State:		//0206 伤害状态数据
						memcpy((void *)(&ext_robot_hurt), (const void *)(Judge_Usart_Info + 7), Judge_Usart_Info[DataLength_offset]);
						if(ext_robot_hurt.hurt_type == 0)
							Robot_is_hit = true;
						break;
					case Judge_RealTime_Shoot:		//0207 实时射击数据
						memcpy((void *)(&ext_shoot_data), (const void *)(Judge_Usart_Info + 7), Judge_Usart_Info[DataLength_offset]);
						break;
					case Judge_Remaining_Rounds:	//0208 子弹剩余数量
						memcpy((void *)(&ext_bullet_remaining), (const void *)(Judge_Usart_Info + 7), Judge_Usart_Info[DataLength_offset]);
						break;
					case Judge_Robot_RFID:			//0209 FRID状态
						memcpy((void *)(&ext_rfid_status), (const void *)(Judge_Usart_Info + 7), Judge_Usart_Info[DataLength_offset]);
						break;
					case Judge_Dart_Client:			//020a 飞镖机器人客户端指令数据
						memcpy((void *)(&ext_dart_client_cmd), (const void *)(Judge_Usart_Info + 7), Judge_Usart_Info[DataLength_offset]);
						break;
					case Judge_Robot_Communicate:	//机器人间交互数据
						memcpy((void *)(&ext_student_interactive_header_data), (const void *)(Judge_Usart_Info + 7), Judge_Usart_Info[DataLength_offset]);
						break;
//					case Judge_User_Defined:
//						break;
//					case Judge_Map_Interaction:
//						break;
//					case Judge_KeyMouse_Message:
//						break;
//					case Judge_Client_Map:
//						break;
					default:
						break;
				}
			}
		}	
	}
}
//获取本机器人红方蓝方
uint8_t get_Robot_Type_Judge(void)
{
	if(ext_game_robot_state.robot_id > 0 && ext_game_robot_state.robot_id < 10 )
		Robot_Type = Red_Robor;
	else
		Robot_Type = Blue_Robor;
	return Robot_Type;
}
/*brief:	装甲板是否被打击
*param(in):	void
*param(out):true:被打击 false:没有被打击
*status:	complete
*/
bool get_Robot_hit_status(void)
{
	return Robot_is_hit;
}
//获取子弹射速
float get_bullet_speed(void)
{
	return ext_shoot_data.bullet_speed;
}	

