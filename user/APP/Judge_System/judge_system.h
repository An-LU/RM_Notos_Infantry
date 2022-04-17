#ifndef __JUDGE_SYSTEM_H__
#define __JUDGE_SYSTEM_H__

#include "main.h"

#define	JUDGE_2021		21
#define	JUDGE_VERSION	JUDGE_2021


#if JUDGE_VERSION == JUDGE_2021

/***************命令码ID********************

	ID: 0x0001          Byte: 11      比赛状态数据                   发送频率 1Hz
	ID: 0x0002          Byte: 1       比赛结果数据                   比赛结束后发送
	ID: 0x0003          Byte: 32      比赛机器人血量数据             发送频率 1Hz
	ID: 0x0004          Byte: 3       飞镖发射状态                   飞镖发射后发送
	ID: 0x0005          Byte: 11      人工智能挑战赛加成与惩罚       发送频率 1Hz
	ID: 0x0101          Byte: 4       场地事件数据                   事件改变后发送
	ID: 0x0102          Byte: 4       场地补给站动作标识数据         动作改变后发送
	ID: 0X0103          Byte: 2       场地补给站预约子弹数据         参赛队发送，上限10Hz（RM对抗赛未开放） 
	ID: 0X0104          Byte: 2       裁判警告数据                   警告发生后发送
	ID: 0x0105          Byte: 1       飞镖发射口倒计时               发送频率 1Hz
	ID: 0X0201          Byte: 27      机器人状态数据                 发送频率 10Hz
	ID: 0X0202          Byte: 16      实时功率热量数据               发送频率 50Hz
	ID: 0X0203          Byte: 16      机器人位置数据                 发送频率 10Hz
	ID: 0X0204          Byte: 1       机器人增益数据                 增益状态改变后发送
	ID: 0X0205          Byte: 2       空中机器人能量状态数据         发送频率 10Hz，只有空中机器人主控发送
	ID: 0X0206          Byte: 1       伤害状态数据                   伤害发生后发送
	ID: 0X0207          Byte: 7       实时射击数据                   子弹发射后发送
	ID: 0X0208          Byte: 6       子弹剩余发送数                 无人机与哨兵发送  发送频率 1Hz
	ID: 0X0209          Byte: 4       机器人RFID状态                 发送频率 1Hz
	ID: 0x020A          Byte: 12      飞镖机器人客户端指令书         发送频率 10Hz
	ID: 0X0301          Byte: n       机器人间交互数据               发送方触发发送    上限频率 10Hz
	ID: 0x0302          Byte: n       自定义控制器交互数据接口       客户端触发发送    上限频率 30Hz
	ID: 0x0303          Byte: 15      客户端小地图交互数据           触发发送
	ID: 0x0304          Byte: 12      键盘、鼠标信息                 通过图传串口发送
	ID: 0x0305			Byte: 10	  客户端小地图接收信息
	
********************************************/

#define JUDGE_HEAD_SOF		0xA5
#define JUDGE_UP_SOF		0xA0
#define	JUDGE_LENGTH_SOF	5	//帧头长度
#define	JUDGE_LENGTH_TAIL	2	//帧尾长度
#define	JUDGE_LENGTH_CMD	2
#define JUDGE_CTRL_LEN		14	//接收数据大小
#define JUDGE_DATA_LEN		16	//上传数据大小
/*******************命令码ID**************************/
#define       Judge_Game_StatusData              0x0001 
#define       Judge_Game_ResultData              0x0002 
#define       Judge_Robot_HP                     0x0003 
#define       Judge_Dart_Launch                  0x0004
#define       Judge_AI_ChallengeBuff             0x0005
#define       Judge_Event_Data                   0x0101
#define       Judge_Supply_Station               0x0102
//#define       Judge_Request_Recharge             0x0103(对抗赛未开放)
#define       Judge_Referee_Warning              0x0104
#define       Judge_Dart_Countdown               0x0105
#define       Judge_Robot_State                  0x0201
#define       Judge_Power_Heat                   0x0202
#define       Judge_Robot_Position               0x0203
#define       Judge_Robot_Buff                   0x0204
#define       Judge_Aerial_Energy                0x0205
#define       Judge_Injury_State                 0x0206
#define       Judge_RealTime_Shoot               0x0207
#define       Judge_Remaining_Rounds             0x0208
#define       Judge_Robot_RFID                   0x0209
#define       Judge_Dart_Client                  0x020A
#define       Judge_Robot_Communicate            0x0301
#define       Judge_User_Defined                 0x0302
#define       Judge_Map_Interaction              0x0303
#define       Judge_KeyMouse_Message             0x0304
#define       Judge_Client_Map                   0x0305
/*****************************************************/
typedef enum
{
	Red_Robor = 1,
	Blue_Robor = 2
}Robot_Type_e;
//帧头数据偏移量
typedef enum
{
	SOF_offset = 0,
	DataLength_offset = 1,
	Seq_offset = 3,
	CRC8_offset = 4
}Farme_Header_Offset;
/**********裁判系统内容结构体***************/
/*	帧头	*/
typedef __packed struct
{
	uint8_t  SOF;						
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
}Frame_Header_t;

/* ID: 0x0001  Byte:  11    比赛状态数据 */
typedef __packed struct 
{ 
	uint8_t game_type : 4;				//比赛模式
	uint8_t game_progress : 4;			//当前比赛阶段
	uint16_t stage_remain_time;			//当前阶段剩余时间 单位s
	uint64_t SyncTimeStamp;
} ext_game_state_t; 

/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef __packed struct 
{ 
	uint8_t winner;
} ext_game_result_t; 

/* ID: 0x0003  Byte:  32    比赛机器人血量数据 */
typedef __packed struct 
{ 
	uint16_t red_1_robot_HP;   
	uint16_t red_2_robot_HP;  
	uint16_t red_3_robot_HP;  
	uint16_t red_4_robot_HP;  
	uint16_t red_5_robot_HP;  
	uint16_t red_7_robot_HP;  
	uint16_t red_outpost_HP; 
	uint16_t red_base_HP; 
	
	uint16_t blue_1_robot_HP;   
	uint16_t blue_2_robot_HP;   
	uint16_t blue_3_robot_HP;   
	uint16_t blue_4_robot_HP;   
	uint16_t blue_5_robot_HP;     
	uint16_t blue_7_robot_HP; 
	uint16_t blue_outpost_HP; 
	uint16_t blue_base_HP;   
}  ext_game_robot_HP_t; 

/* ID: 0x0004  Byte:  3    飞镖发射状态 */
typedef __packed struct 
{  
	uint8_t dart_belong; 				//发射飞镖的队伍
	uint16_t stage_remaining_time; 		//发射时剩余的比赛时间
} ext_dart_status_t;

/* ID: 0x0005  Byte:  11   人工智能挑战赛 buff and debuff */
typedef __packed struct
{ 
	uint8_t F1_zone_status:1;  							//地区是否开启的标志
	uint8_t F1_zone_buff_debuff_status:3;   //红方回血
	
	uint8_t F2_zone_status:1;  
	uint8_t F2_zone_buff_debuff_status:3;   //红方弹药
	
	uint8_t F3_zone_status:1;  
	uint8_t F3_zone_buff_debuff_status:3;		//蓝方回血  
	
	uint8_t F4_zone_status:1;  
	uint8_t F4_zone_buff_debuff_status:3;  	//蓝方弹药
	
	uint8_t F5_zone_status:1;  
	uint8_t F5_zone_buff_debuff_status:3;  	//禁止射击区
	
	uint8_t F6_zone_status:1;  
	uint8_t F6_zone_buff_debuff_status:3;  	//禁止移动区
	
	uint16_t red1_bullet_left;							//红蓝方剩余弹量
	uint16_t red2_bullet_left;
	uint16_t blue1_bullet_left;
	uint16_t blue2_bullet_left;
	
} ext_ICRA_buff_debuff_zone_status_t; 

/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef __packed struct 
{ 
	uint32_t event_type;										//0~4bit在用，5~31保留
	
} ext_event_data_t; 

/* ID: 0x0102  Byte:  4    场地补给站动作标识数据 */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;						//补给站口ID
	uint8_t supply_robot_id;								//当前补弹机器人ID
	uint8_t supply_projectile_step;					//补给站出弹口状态
	
	uint8_t supply_projectile_num;					//补弹数目
} ext_supply_projectile_action_t; 

/* ID: 0x0104  Byte: 2   裁判系统警告信息 */
typedef __packed struct 
{ 
	uint8_t level; 													//警告等级
	uint8_t foul_robot_id;									//犯规机器人ID
}  ext_referee_warning_t;  

/* ID: 0x0105  Byte:1  飞镖发射口倒计时 */
typedef __packed struct 
{ 
	uint8_t dart_remaining_time; 						//15s倒计时
}  ext_dart_remaining_time_t;  


/* ID: 0X0201  Byte: 27    机器人状态数据 */
/* 本机器人 ID： 
	1：红方英雄机器人； 
	2：红方工程机器人； 
	3/4/5：红方步兵机器人； 
	6：红方空中机器人； 
	7：红方哨兵机器人； 
	8：红方飞镖机器人； 
	9：红方雷达站； 
	101：蓝方英雄机器人； 
	102：蓝方工程机器人； 
	103/104/105：蓝方步兵机器人； 106：蓝方空中机器人； 
	107：蓝方哨兵机器人； 
	108：蓝方飞镖机器人； 
	109：蓝方雷达站。 
*/
//0代表是17mm枪管，1代表42mm枪管
typedef __packed struct 			
{ 
	uint8_t robot_id;   								//本机器人ID，可用来校验发送
	uint8_t robot_level;  								//机器人等级 1一级，2二级，3三级
	uint16_t remain_HP; 								//机器人剩余血量
	uint16_t max_HP; 									//机器人上限血量
	uint16_t shooter_heat0_cooling_rate;  				//机器人 17mm 1子弹热量冷却速度 单位 /s
	uint16_t shooter_heat0_cooling_limit;   			//机器人 17mm 1子弹热量上限
	uint16_t shooter_heat0_speed_limit;					//机器人17mm 1枪口上限速度
	uint16_t shooter_heat1_cooling_rate;   				//机器人17mm 2枪口每秒冷却值
	uint16_t shooter_heat1_cooling_limit;  				//机器人17mm 2枪口热量上限
	uint16_t shooter_heat1_speed_limit;					//机器人17mm 2枪口上限速度
	uint16_t shooter_42mm_cooling_rate;   				//机器人42mm 枪口每秒冷却值
	uint16_t shooter_42mm_cooling_limit;  				//机器人42mm 枪口热量上限
	uint16_t shooter_42mm_speed_limit;					//机器人42mm 枪口上限速度
	uint16_t max_chassis_power; 						//机器人底盘功率上限
	uint8_t mains_power_gimbal_output : 1;  			//主控电源云台口输出状态
	uint8_t mains_power_chassis_output : 1;  			//主控电源底盘口输出状态
	uint8_t mains_power_shooter_output : 1; 			//主控电源枪管口输出状态
} ext_game_robot_state_t;   

/* ID: 0X0202  Byte: 16    实时功率热量数据 */
typedef __packed struct 
{ 
	uint16_t chassis_volt;   											//底盘输出电压 mV
	uint16_t chassis_current;    									//底盘输出电流 mA
	float chassis_power;   												//底盘输出功率 
	uint16_t chassis_power_buffer;								//底盘缓冲功率剩余
	uint16_t shooter_heat0;												//17mm枪口1目前热量
	uint16_t shooter_heat1;  											//17mm枪口2目前热量
	uint16_t shooter_heat2_42mm; 								//42mm枪口目前热量
} ext_power_heat_data_t; 

/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef __packed struct 
{   
	float x;   																		//机器人位置x
	float y;   																		//机器人位置y
	float z;   																		//机器人位置z
	float yaw; 																		//机器人偏航角
} ext_game_robot_pos_t; 

/* ID: 0x0204  Byte:  1    机器人增益数据 */
typedef __packed struct 
{ 
	uint8_t power_rune_buff;											// 
} ext_buff_musk_t; 

/* ID: 0x0205  Byte:  1    空中机器人能量状态数据 */
typedef __packed struct 
{ 
	uint8_t attack_time; 													//可攻击时间
} aerial_robot_energy_t; 

/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef __packed struct 
{ 
	uint8_t armor_id : 4; 												//受伤装甲板ID
	uint8_t hurt_type : 4; 												//血量扣除类型
} ext_robot_hurt_t; 

/* ID: 0x0207  Byte:  7    实时射击数据 */
typedef __packed struct 
{ 
	uint8_t bullet_type;   												//子弹类型
	uint8_t shooter_id;   												//发射机构id
	uint8_t bullet_freq;   												//子弹射频 HZ
	float bullet_speed;  													//子弹射速 m/s
} ext_shoot_data_t; 


/* ID: 0x0208  Byte:  6    子弹剩余数量 */
typedef __packed struct 
{ 
	uint16_t bullet_remaining_num_17mm;   							//子弹剩余发射数
	uint16_t bullet_remaining_num_42mm;   							//子弹剩余发射数
	uint16_t coin_remain;   														//剩余金币
}  ext_bullet_remaining_t; 

/* ID: 0x0209  Byte:  4    FRID状态 */
typedef __packed struct 
{ 
	uint32_t rfid_status ;												//RFID卡状态
}  ext_rfid_status_t; 


/*飞镖机器人客户端指令数据*/
/* ID: 0x020a  Byte:  4    FRID状态 */
typedef __packed struct {  
	uint8_t dart_launch_opening_status;  
	uint8_t dart_attack_target;  
	uint16_t target_change_time; 
	uint16_t operate_launch_cmd_time; 
} ext_dart_client_cmd_t; 

/* ID: 0X0301  Byte: n  机器人间交互数据 */
typedef __packed struct 
{ 
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID; 
} ext_student_interactive_header_data_t; 

/********************************************/

uint8_t get_Robot_Type_Judge(void);		//获取本机器人红方蓝方
float get_bullet_speed(void);			//获取子弹射速

void Judge_Read_Data(uint8_t *Judge_Usart_Info);

//#elif JUDGE_VERSION == JUDGE_2021


#endif

#endif

