#ifndef SENTRY_REFEREE_H
#define SENTRY_REFEREE_H

#include <string>

#ifdef __DRIVER_GLOBALS
#define __DRIVER_EXT
#else
#define __DRIVER_EXT extern
#endif

#define JudgeBufferLength_SENTRY 255

#define JudgeFrameHeader 0xA5 // 帧头
#define FrameHeader_Len 5

typedef enum
{
    GAME_STATE_CMD_ID                 = 0x0001,
    GAME_RESULT_CMD_ID                = 0x0002,
    GAME_ROBOT_HP_CMD_ID              = 0x0003,
    FIELD_EVENTS_CMD_ID               = 0x0101,
    SUPPLY_PROJECTILE_ACTION_CMD_ID   = 0x0102,
    SUPPLY_PROJECTILE_BOOKING_CMD_ID  = 0x0103,
    REFEREE_WARNING_CMD_ID            = 0x0104,
    ROBOT_STATE_CMD_ID                = 0x0201,
    POWER_HEAT_DATA_CMD_ID            = 0x0202,
    ROBOT_POS_CMD_ID                  = 0x0203,
    BUFF_MUSK_CMD_ID                  = 0x0204,
    AERIAL_ROBOT_ENERGY_CMD_ID        = 0x0205,
    ROBOT_HURT_CMD_ID                 = 0x0206,
    SHOOT_DATA_CMD_ID                 = 0x0207,
    BULLET_REMAINING_CMD_ID           = 0x0208,
    STUDENT_INTERACTIVE_DATA_CMD_ID   = 0x0301,
    IDCustomData,
}referee_cmd_id_t;

typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
    BLUE_HERO       = 11,
    BLUE_ENGINEER   = 12,
    BLUE_STANDARD_1 = 13,
    BLUE_STANDARD_2 = 14,
    BLUE_STANDARD_3 = 15,
    BLUE_AERIAL     = 16,
    BLUE_SENTRY     = 17,
} robot_id_t;

typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress_t;

typedef enum
{
    Robot_HP_ID = 0x0003,         // 机器人血量数据
    Robot_Status_ID = 0x0201,     // 机器人状态  等级
    power_heat_data_ID = 0x0202,  // 枪口热量 底盘功率
    Robot_pos_ID = 0x0203,        // 机器人位置
    Robot_power_buff_ID = 0x0204, // 机器人增益
    robot_hurt_ID = 0x0206,       // 伤害类型
    shoot_data_ID = 0x0207,       // 射频射速
    student_interactive_header_ID = 0x0301,
} Judege_Cmd_ID;
// 帧头

// 比赛数据0001
typedef struct
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} __attribute__((__packed__)) ext_game_state_t;

// 机器人血量数据0003
typedef struct
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
} __attribute__((__packed__)) ext_game_robot_HP_t;

// 场地事件0101
typedef struct
{
    uint32_t event_data;
} __attribute__((__packed__)) ext_event_data_t;

// 比赛机器人状态0201
typedef struct
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} __attribute__((__packed__)) ext_game_robot_state_t;
typedef struct
{
    uint8_t SOF;        // 0xA5
    uint16_t DataLenth; // 数据位长度
    uint8_t Seq;        // 包序号
    uint8_t CRC8;       // crc8位校验
} __attribute__((__packed__)) tFrameHeader_Judge;

// 实时功率以及枪口热量0202
typedef struct
{
    uint16_t chassis_voltage;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_1_barrel_heat;
    uint16_t shooter_17mm_2_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
} __attribute__((__packed__)) ext_power_heat_data_t;

//伤害数据0206
typedef struct
{
uint8_t armor_id : 4;
uint8_t HP_deduction_reason : 4;
}__attribute__((__packed__)) ext_robot_hurt_t;

//射击数据0207
typedef struct
{
uint8_t bullet_type;
uint8_t shooter_number;
uint8_t launching_frequency;
float initial_speed;
}__attribute__((__packed__)) ext_shoot_data_t;

typedef struct
{
    uint8_t SOF; // 0xA5
    uint16_t DataLenth;
    uint8_t Seq;
    uint8_t CRC8;
} __attribute__((__packed__)) tFrameHeader;


typedef struct
{
    uint16_t rxCmdId;
    uint16_t data_id;
    uint16_t send_id;
    uint16_t receive_id;
} __attribute__((__packed__)) id_data_t;

typedef struct
{
    tFrameHeader Header;
    id_data_t id;
    float data1;
    float data2;
    float data3;
    uint8_t masks;
    uint16_t crc_16;
} __attribute__((__packed__)) client_custom_data_t;

// 机器人之间相互通信
typedef struct
{
    uint8_t data[112];
} __attribute__((__packed__)) robot_interactive_data_t;

typedef struct
{
    uint16_t data_cmd_id;
    uint16_t send_ID;
    uint16_t receiver_ID;
} __attribute__((__packed__)) ext_student_interactive_header_data_t;

typedef struct
{
    tFrameHeader frameHeader;
    uint16_t rxCmdId;
    client_custom_data_t robot_data_t;
    client_custom_data_t userinfo;
} __attribute__((__packed__)) JudgementDataTypedef;


typedef struct
{
   uint8_t SOF;                    //起始字节,固定0xA5
   uint16_t Data_Length;           //帧数据长度
   uint8_t Seq;                    //包序号
   uint8_t CRC8;                   //CRC8校验值
   uint16_t CMD_ID;                //命令ID
} Packhead;             //帧头

typedef struct
{
   uint16_t Data_ID;               //内容ID
   uint16_t Sender_ID;             //发送者ID
   uint16_t Receiver_ID;           //接收者ID
} Data_Operate;         //操作定义帧

#endif