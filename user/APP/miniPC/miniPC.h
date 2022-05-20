#ifndef __MINIPC_H__
#define __MINIPC_H__

#include "main.h"
#include "judge_system.h"

#define VISION_CTRL_LENGTH	23 //数据为20字节，头7字节，数据14字节
#define VISION_HEAD_SOF		0xA5
#define VISION_UP_SOF		0xA0
#define	VISION_LENGTH_SOF	5	//帧头长度
#define	VISION_LENGTH_TAIL	2	//帧尾长度
#define	VISION_LENGTH_CMD	2
#define VISION_CTRL_LEN		14	//接收数据大小
#define VISION_DATA_LEN		16	//上传数据大小

//是否连接裁判系统
#define JUDGE_SYSTEM_OFFLINE	0
#define JUDGE_SYSTEM_ONLINE		1
#define ROBOT_MODE	JUDGE_SYSTEM_OFFLINE
#define RED_ROBOT	1
#define BLUE_ROBOT	2
//机器人类型 线下模式 Red:1 Blue:2
#define ROBOT_TYPE	RED_ROBOT



/* 帧头数据偏移量 */
typedef enum
{
	SOF_offset_v = 0,
	DataLength_offset_v = 1,
	Seq_offset_v = 3,
	CRC8_offset_v = 4,
	Cmd_ID_v = 5,
	AutoCtrl_Data_v = 7
}Vision_Farme_Header_Offset_t;
/* 帧头 byte: 5 */
typedef __packed struct
{
	uint8_t  SOF;			
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
}Vision_Frame_Header_t;
/* 帧尾 byte: 2 */
typedef __packed struct
{
	uint16_t CRC16;
}Vision_Frame_Tail_t;
/* 自瞄数据 byte: 14 */
////视觉角度
//typedef __packed struct
//{
//	float pitch_angle;
//	float yaw_angle;
//} CTRL_t;
typedef __packed struct
{
    uint32_t	time;
    fp32		pitch_angle;	//云台自瞄pitch角度
    fp32		yaw_angle;		//云台自瞄yaw角度
	//CTRL_t ctrl_angle;
    uint8_t		visual_valid;	//自瞄有效位
    uint8_t		buff_shoot;		//自动发弹
}Auto_Gimbal_Ctrl_t;
/* miniPC-->stm32 byte: 23 */
typedef __packed struct
{
	/* 头 */
	Vision_Frame_Header_t head_data;
	/* 命令码ID */
	uint16_t cmd_ID;
	/* 云台数据 */
	Auto_Gimbal_Ctrl_t auto_gimbal_ctrl;
	/* 帧尾 */
	Vision_Frame_Tail_t tail_data;
}Vision_Recv_Data_t;
/* stm32返回数据 byte: 16 */
typedef __packed struct
{
	uint8_t		mode;			//模式：0装甲板 1神符 2空
    uint8_t		enemy_color;	//敌方颜色：1 red 2 blue
    uint8_t		is_left;		//
    uint8_t		run_left;		//
    fp32		bullet_spd;		//弹丸速度
    fp32		pitch;			//当前pitch角度
    fp32		yaw;			//当前yaw角度
    //fp32		pitch_offset;	//pit补偿角度
    //fp32		yaw_offset;		//yaw补偿角度
}Stm32_Info_t;
/* stm32-->miniPC  byte: 25 */
typedef __packed struct
{
	/* 头 */
	Vision_Frame_Header_t head_data;
	/* 命令码ID */
	uint16_t cmd_ID;
	/* stm32数据 */
	Stm32_Info_t stm32_info;
	/* 帧尾 */
	Vision_Frame_Tail_t tail_data;
}Vision_Send_Data_t;

//typedef struct
//{
//	union 
//	{
//		uint8_t databuf[16];
//		Stm32_Info_t stm32_info;
//	}data;
//}Stm32_Info_t_2;
typedef enum
{
	ARMOR_PLATE = 0,	//装甲板
	BUFF_ANTI = 1,		//打符
	NO_VISION
}Vision_Mode_e;
//-----------------------------------【步兵信息结构体】--------------------------------------------
// brief：包括云台控制模式，pitch角度，yaw角度等信息，是控制步兵运动的数据

typedef enum
{
  CHASSIS_DATA_ID     = 0x0010,
  GIMBAL_DATA_ID      = 0x0011,
  SHOOT_TASK_DATA_ID  = 0x0012,
  INFANTRY_ERR_ID     = 0x0013,
  CONFIG_RESPONSE_ID  = 0x0014,
  CALI_RESPONSE_ID    = 0x0015,
  REMOTE_CTRL_INFO_ID = 0x0016,
  BOTTOM_VERSION_ID   = 0x0017,
  SENTRY_DATA_ID      = 0x0018,

  CHASSIS_CTRL_ID     = 0x00A0,
  GIMBAL_CTRL_ID      = 0x00A1,
  SHOOT_CTRL_ID       = 0x00A2,
  ERROR_LEVEL_ID      = 0x00A3,
  INFANTRY_STRUCT_ID  = 0x00A4,
  CALI_GIMBAL_ID      = 0x00A5,
} infantry_data_id_e;

typedef enum
{
  BOTTOM_DEVICE        = 0,
  GIMBAL_GYRO_OFFLINE  = 1,
  CHASSIS_GYRO_OFFLINE = 2,
  CHASSIS_M1_OFFLINE   = 3,
  CHASSIS_M2_OFFLINE   = 4,
  CHASSIS_M3_OFFLINE   = 5,
  CHASSIS_M4_OFFLINE   = 6,
  REMOTE_CTRL_OFFLINE  = 7,
  JUDGE_SYS_OFFLINE    = 8,
  PC_SYS_OFFLINE       = 9,
  GIMBAL_YAW_OFFLINE   = 10,
  GIMBAL_PIT_OFFLINE   = 11,
  TRIGGER_MOTO_OFFLINE = 12,
  BULLET_JAM           = 13,
  CHASSIS_CONFIG_ERR   = 14,
  GIMBAL_CONFIG_ERR    = 15,
  ERROR_LIST_LENGTH    = 16,
} err_id_e;

typedef enum
{
  DEVICE_NORMAL = 0,
  ERROR_EXIST   = 1,
  UNKNOWN_STATE = 2,
} bottom_err_e;

typedef enum
{
  GLOBAL_NORMAL        = 0,
  SOFTWARE_WARNING     = 1,
  SOFTWARE_ERROR       = 2,
  SOFTWARE_FATAL_ERROR = 3,
  GIMBAL_ERROR         = 4,
  CHASSIS_ERROR        = 5,
  HARAWARE_ERROR       = 6,
} err_level_e;

typedef enum
{
  NO_CONFIG      = 0,//无定义
  DEFAULT_CONFIG = 1,//默认定义
  CUSTOM_CONFIG  = 3,//用户定义
} struct_config_e;

/********** the information send to computer ***********/


///* 头帧 byte: 7 */
//typedef struct
//{
//	/* 头 */
//	uint8_t		SOF;				//帧头起始位,暂定0xAA
//	uint8_t		SendDataLen;		//长度
//	uint8_t		empty_1[3];
//	uint8_t		CMDID;				//
//	uint8_t		empty_2;
//	//数据ID
//}__attribute__((packed))	extVisionSendHeader_t;

///* 自瞄主要信息 byte: 14 */
//typedef struct  //__packed
//{
//    uint32_t time;
//    float    pit_ref;      /* gimbal pitch reference angle(degree) */
//    float    yaw_ref;      /* gimbal yaw reference angle(degree) */
//    uint8_t  visual_valid; /* visual information valid or not */
//    uint8_t  buff_shoot;
//}__attribute__((packed))   gimbal_ctrl_t;
///* STM32接收 byte: 21 */
//typedef struct
//{
//	/* 头 */
//	extVisionSendHeader_t	head_data;
//	/* 数据 */
//	gimbal_ctrl_t	gimbal_ctrl;
//	/* 尾 */  
//	//uint8_t   END;			//结束位，暂定0xBB
//}__attribute__((packed))	extVisionRecvData_t;
//typedef struct
//{
//	/* 头 */
//	uint8_t		SOF;		//帧头起始位,暂定0xA0
//	uint8_t		empty_3;
//	uint8_t		DataLen;	//数据长度
//	uint16_t	empty;
//	uint16_t	DataID;		//数据ID
//}
///* STM32发送,一字节一字节发送 byte: 23 */
//typedef struct //__packed struct
//{
//	uint8_t		mode;   //模式：0装甲板 1神符 2空
//    uint8_t		enemy_color; //敌方颜色：1 red 2 blue
//    uint8_t		is_left;
//    uint8_t		run_left;
//    float		bullet_spd;	//弹丸速度
//    float		pitch;
//    float		yaw;
//    //float		pitch_offset; // pit补偿角度
//    //float		yaw_offset;  // yaw补偿角度
//}__attribute__((packed))	extVisionSendData_t;
//视觉角度
//typedef struct
//{
//	float pitch_angle;
//	float yaw_angle;
//} CTRL;
//extern extVisionRecvData_t VisionRecvData;//视觉接收结构体

//打开自瞄
//void Vision_Ctrl(void);
void Vision_Init(void);
void Vision_Read_Data(uint8_t *Usart_Information);		//视觉数据读取解析
uint8_t Vision_update_flag(void);						//视觉数据是否更新
void Vision_clean_flag(void);							//视觉数据更新位清0
void Data_Send_to_Vision(void);
void Limit_Auto_Angle(fp32 *pitch_angle, fp32 *yaw_angle);//角度限幅
extern Auto_Gimbal_Ctrl_t *get_AUTO_control_point(void);				//获取自瞄角度结构体指针

#endif


