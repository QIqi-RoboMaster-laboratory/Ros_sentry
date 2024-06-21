#include <string>
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include "crc_check.h"
#include "sentry_referee.h"
#include "sentry_referee/game_state.h"
#include "sentry_referee/game_robot_HP.h"
#include "sentry_referee/field_event.h"
#include "sentry_referee/robot_state.h"
#include "sentry_referee/power_heat_data.h"
#include "sentry_referee/robot_hurt.h"
#include "sentry_referee/shoot_data.h"

ext_game_state_t game_state;
ext_game_robot_HP_t game_robot_HP_t;
ext_event_data_t field_event;
ext_game_robot_state_t robot_state;
ext_power_heat_data_t power_heat_data_t;
ext_robot_hurt_t robot_hurt_t;
ext_shoot_data_t shoot_data_t;

sentry_referee::game_state game_state_msg;
sentry_referee::game_robot_HP game_robot_HP_msg;
sentry_referee::field_event field_event_msg;
sentry_referee::robot_state robot_state_msg;
sentry_referee::power_heat_data power_heat_data_msg;
sentry_referee::robot_hurt robot_hurt_msg;
sentry_referee::shoot_data shoot_data_msg;

ros::Publisher game_state_publisher;
ros::Publisher game_robot_HP_publisher;
ros::Publisher field_event_publisher;
ros::Publisher robot_state_publisher;
ros::Publisher power_heat_data_publisher;
ros::Publisher robot_hurt_publisher;
ros::Publisher shoot_data_publisher;

serial::Serial s;

JudgementDataTypedef JudgementData;
uint8_t JudgeDataBuffer_HERO[JudgeBufferLength_SENTRY];
unsigned char UI_Seq; 
uint16_t Sentry_ID;
uint8_t data[4];

void judgeCalculate(uint8_t JudgeDataBuffer[JudgeBufferLength_SENTRY]);
int Robot_Transfer(uint16_t ReceiverID, uint16_t DataID, uint8_t Size, uint8_t *buf);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sentry_referee");
    ros::NodeHandle nh;

    game_state_publisher = nh.advertise<sentry_referee::game_state>("/referee/game_state", 50);
    game_robot_HP_publisher = nh.advertise<sentry_referee::game_robot_HP>("/referee/game_robot_HP", 50);
    field_event_publisher = nh.advertise<sentry_referee::field_event>("/referee/field_event", 50);
    robot_state_publisher = nh.advertise<sentry_referee::robot_state>("/referee/robot_state", 50);
    power_heat_data_publisher = nh.advertise<sentry_referee::power_heat_data>("/referee/power_heat_data", 50);
    robot_hurt_publisher = nh.advertise<sentry_referee::robot_hurt>("/referee/robot_hurt", 50);
    shoot_data_publisher = nh.advertise<sentry_referee::shoot_data>("/referee/shoot_data", 50);

    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    s.setPort("/dev/Referee_USB");
    s.setBaudrate(115200);
    s.setTimeout(to);

    try
    {
        // 打开串口
        s.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    // 判断串口是否打开成功
    if (s.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }

    ROS_INFO("Loop Start!");
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if (s.available())
        {
            uint8_t buffer[255];
            // 读出数据
            s.read(buffer, s.available());

            judgeCalculate(buffer);

            ROS_INFO("%d,%f,%d", power_heat_data_msg.shooter_17mm_1_barrel_heat, power_heat_data_msg.chassis_power, game_robot_HP_msg.red_4_robot_HP);
        }
        game_state_publisher.publish(game_state_msg);
        game_robot_HP_publisher.publish(game_robot_HP_msg);
        field_event_publisher.publish(field_event_msg);
        robot_state_publisher.publish(robot_state_msg);
        power_heat_data_publisher.publish(power_heat_data_msg);
        robot_hurt_publisher.publish(robot_hurt_msg);
        shoot_data_publisher.publish(shoot_data_msg);
        ros::spinOnce;
        loop_rate.sleep();
    }
    return 0;
}

void judgeCalculate(uint8_t JudgeDataBuffer[JudgeBufferLength_SENTRY]) // 裁判系统解算
{
    static uint16_t start_pos = 0, next_start_pos = 0;
    while (1)
    {
        memcpy(&JudgementData.frameHeader, &JudgeDataBuffer[start_pos], FrameHeader_Len);
        if ((JudgementData.frameHeader.SOF == (uint16_t)JudgeFrameHeader) && (1 == Verify_CRC8_Check_Sum(&JudgeDataBuffer[start_pos], FrameHeader_Len)) && (1 == Verify_CRC16_Check_Sum(&JudgeDataBuffer[start_pos], JudgementData.frameHeader.DataLenth + FrameHeader_Len + 4))) // 数据位长度+帧头长度+命令码长度+校验码长度
        {
            memcpy(&JudgementData.rxCmdId, (&JudgeDataBuffer[start_pos] + 5), sizeof(JudgementData.rxCmdId));
            JudgeDataBuffer[start_pos]++;                                         // 每处理完一次就在帧头加一防止再次处理这帧数据
            next_start_pos = start_pos + 9 + JudgementData.frameHeader.DataLenth; // 9为 5位帧头 2位数据长度 2校验位

            switch (JudgementData.rxCmdId)
            {
            case GAME_STATE_CMD_ID:
            {
                memcpy(&game_state, (&JudgeDataBuffer[start_pos] + 7), JudgementData.frameHeader.DataLenth); // 把数组中的数据复制到对应的结构体中去
            }
            break;
            case GAME_ROBOT_HP_CMD_ID:
            {
                memcpy(&game_robot_HP_t, (&JudgeDataBuffer[start_pos] + 7), JudgementData.frameHeader.DataLenth); // 把数组中的数据复制到对应的结构体中去
            }
            break;
            case FIELD_EVENTS_CMD_ID:
            {
                memcpy(&field_event, (&JudgeDataBuffer[start_pos] + 7), JudgementData.frameHeader.DataLenth); // 把数组中的数据复制到对应的结构体中去
            }
            break;
            case ROBOT_STATE_CMD_ID:
            {
                memcpy(&robot_state, (&JudgeDataBuffer[start_pos] + 7), JudgementData.frameHeader.DataLenth); // 把数组中的数据复制到对应的结构体中去
            }
            break;
            case POWER_HEAT_DATA_CMD_ID:
            {
                memcpy(&power_heat_data_t, (&JudgeDataBuffer[start_pos] + 7), JudgementData.frameHeader.DataLenth); // 把数组中的数据复制到对应的结构体中去
            }
            break;
            case ROBOT_HURT_CMD_ID:
            {
                memcpy(&robot_hurt_t, (&JudgeDataBuffer[start_pos] + 7), JudgementData.frameHeader.DataLenth); // 把数组中的数据复制到对应的结构体中去
            }
            break;
            case SHOOT_DATA_CMD_ID:
            {
                memcpy(&shoot_data_t, (&JudgeDataBuffer[start_pos] + 7), JudgementData.frameHeader.DataLenth); // 把数组中的数据复制到对应的结构体中去
            }
            }
            start_pos = next_start_pos;
        }
        else
        {
            start_pos = 0;
            break;
        }
        /**如果头指针越界了退出循环**/
        if (start_pos > JudgeBufferLength_SENTRY)
        {
            start_pos = 0;
            break;
        }
    }
    game_state_msg.game_type = game_state.game_type;
    game_state_msg.game_progress = game_state.game_progress;
    game_state_msg.stage_remain_time = game_state.stage_remain_time;
    game_state_msg.SyncTimeStamp = game_state.SyncTimeStamp;

    game_robot_HP_msg.red_1_robot_HP = game_robot_HP_t.red_1_robot_HP;
    game_robot_HP_msg.red_2_robot_HP = game_robot_HP_t.red_2_robot_HP;
    game_robot_HP_msg.red_3_robot_HP = game_robot_HP_t.red_3_robot_HP;
    game_robot_HP_msg.red_4_robot_HP = game_robot_HP_t.red_4_robot_HP;
    game_robot_HP_msg.red_5_robot_HP = game_robot_HP_t.red_5_robot_HP;
    game_robot_HP_msg.red_7_robot_HP = game_robot_HP_t.red_7_robot_HP;
    game_robot_HP_msg.red_outpost_HP = game_robot_HP_t.red_outpost_HP;
    game_robot_HP_msg.red_base_HP = game_robot_HP_t.red_base_HP;
    game_robot_HP_msg.blue_1_robot_HP = game_robot_HP_t.blue_1_robot_HP;
    game_robot_HP_msg.blue_2_robot_HP = game_robot_HP_t.blue_2_robot_HP;
    game_robot_HP_msg.blue_3_robot_HP = game_robot_HP_t.blue_3_robot_HP;
    game_robot_HP_msg.blue_4_robot_HP = game_robot_HP_t.blue_4_robot_HP;
    game_robot_HP_msg.blue_5_robot_HP = game_robot_HP_t.blue_5_robot_HP;
    game_robot_HP_msg.blue_7_robot_HP = game_robot_HP_t.blue_7_robot_HP;
    game_robot_HP_msg.blue_outpost_HP = game_robot_HP_t.blue_outpost_HP;
    game_robot_HP_msg.blue_base_HP = game_robot_HP_t.blue_base_HP;

    robot_state_msg.robot_id = robot_state.robot_id;
    robot_state_msg.robot_level = robot_state.robot_level;
    robot_state_msg.current_HP = robot_state.current_HP;
    robot_state_msg.maximum_HP = robot_state.maximum_HP;
    robot_state_msg.shooter_barrel_cooling_value = robot_state.shooter_barrel_cooling_value;
    robot_state_msg.shooter_barrel_heat_limit = robot_state.shooter_barrel_heat_limit;
    robot_state_msg.chassis_power_limit = robot_state.chassis_power_limit;

    power_heat_data_msg.chassis_voltage = power_heat_data_t.chassis_voltage;
    power_heat_data_msg.chassis_current = power_heat_data_t.chassis_current;
    power_heat_data_msg.chassis_power = power_heat_data_t.chassis_power;
    power_heat_data_msg.buffer_energy = power_heat_data_t.buffer_energy;
    power_heat_data_msg.shooter_17mm_1_barrel_heat = power_heat_data_t.shooter_17mm_1_barrel_heat;
    power_heat_data_msg.shooter_17mm_2_barrel_heat = power_heat_data_t.shooter_17mm_2_barrel_heat;

    robot_hurt_msg.armor_id = robot_hurt_t.armor_id;
    robot_hurt_msg.HP_deduction_reason = robot_hurt_t.HP_deduction_reason;

    shoot_data_msg.bullet_type = shoot_data_t.bullet_type;
    shoot_data_msg.shooter_number = shoot_data_t.shooter_number;
    shoot_data_msg.launching_frequency = shoot_data_t.launching_frequency;
    shoot_data_msg.initial_speed = shoot_data_t.initial_speed;
}


void UI_SendByte(unsigned char ch)
{
    s.write(&ch,sizeof(ch));
}

int Robot_Transfer(uint16_t SendID,uint16_t ReceiverID, uint16_t DataID, uint8_t Size, uint8_t *buf)
{
   unsigned char *framepoint; // 读写指针
   uint16_t frametail = 0xFFFF;    // CRC16校验值

   Packhead framehead;
   Data_Operate datahead;

   framepoint = (unsigned char *)&framehead;
   framehead.SOF = 0xA5;
   framehead.Data_Length = 6 + Size;
   framehead.Seq = UI_Seq;
   framehead.CRC8 = Get_CRC8_Check_Sum(framepoint, 4, 0xFF);
   framehead.CMD_ID = 0x0301; // 填充包头数据

   datahead.Data_ID = DataID;
   datahead.Sender_ID = SendID;
   datahead.Receiver_ID = ReceiverID; // 填充操作数据

   framepoint = (unsigned char *)&framehead;
   frametail = Get_CRC16_Check_Sum(framepoint, sizeof(framehead), frametail);
   framepoint = (unsigned char *)&datahead;
   frametail = Get_CRC16_Check_Sum(framepoint, sizeof(datahead), frametail);
   framepoint = buf;
   frametail = Get_CRC16_Check_Sum(framepoint, Size, frametail);

   framepoint = (unsigned char *)&framehead;
   for (int i = 0; i < sizeof(framehead); i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   framepoint = (unsigned char *)&datahead;
   for (int i = 0; i < sizeof(datahead); i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   } // 发送操作数据

   framepoint = buf;
   for (int i = 0; i < Size; i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   } // 发送数据

   framepoint = (unsigned char *)&frametail;
   for (int i = 0; i < sizeof(frametail); i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   } // 发送CRC16校验值

   UI_Seq++; // 包序号+1
   return 0;
}
