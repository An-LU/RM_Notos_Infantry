#include "judge_system.h"
#include "crc.h"
#include "string.h"

static uint8_t Judge_Data[100];
static ext_game_robot_state_t	ext_game_robot_state;
static ext_shoot_data_t			ext_shoot_data;
uint8_t Robot_Type;	//机器人类型
/* 读取裁判系统信息 */
void Judge_Read_Data(uint8_t *Judge_Usart_Info)
{
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
				CMD_ID = (uint16_t)Judge_Usart_Info[5] << 8 | (uint16_t)Judge_Usart_Info[6];
				switch( CMD_ID )
				{
					case Judge_Game_StatusData:
						break;
					case Judge_Game_ResultData:
						break;
					case Judge_Robot_HP:
						break;
					case Judge_Dart_Launch:
						break;
					case Judge_AI_ChallengeBuff:
						break;
					case Judge_Event_Data:
						break;
					case Judge_Supply_Station:
						break;
					case Judge_Referee_Warning:
						break;
					case Judge_Dart_Countdown:
						break;
					case Judge_Robot_State:
						memcpy((void *)(&ext_game_robot_state), (const void *)(Judge_Usart_Info + 7), Judge_Usart_Info[DataLength_offset]);
						break;
					case Judge_Power_Heat:
						break;
					case Judge_Robot_Position:
						break;
					case Judge_Robot_Buff:
						break;
					case Judge_Aerial_Energy:
						break;
					case Judge_Injury_State:
						break;
					case Judge_RealTime_Shoot:
						memcpy((void *)(&ext_shoot_data), (const void *)(Judge_Usart_Info + 7), Judge_Usart_Info[DataLength_offset]);
						break;
					case Judge_Remaining_Rounds:
						break;
					case Judge_Robot_RFID:
						break;
					case Judge_Dart_Client:
						break;
					case Judge_Robot_Communicate:
						break;
					case Judge_User_Defined:
						break;
					case Judge_Map_Interaction:
						break;
					case Judge_KeyMouse_Message:
						break;
					case Judge_Client_Map:
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
	{
		Robot_Type = Red_Robor;
	}
	else
	{
		Robot_Type = Blue_Robor;
	}
	return Robot_Type;
}
//获取子弹射速
float get_bullet_speed(void)
{
	return ext_shoot_data.bullet_speed;
}	

