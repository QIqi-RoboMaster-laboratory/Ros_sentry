#ifndef SENTRY_CONTROL_H
#define SENTRY_CONTROL_H
#include <serial/serial.h>
#include <std_msgs/String.h>

serial::Serial sentry_ser;

uint8_t data_len = 36;
std::string cmd_vel_topic;

union Serial_Package
{
    struct
    {
        uint8_t header = 0xA5;
        uint8_t scan;
        uint8_t spin;
        float linear_x;
        float linear_y;
        float angular_z;
        uint8_t robot_id;
        uint16_t shooter_barrel_heat_limit;
        uint16_t chassis_power_limit;
        uint16_t shooter_17mm_1_barrel_heat;
        float chassis_power;
        uint16_t buffer_energy;
        float initial_speed;
        float null;
    };
    uint8_t Send_Buffer[33];
};

struct
{
    uint8_t header = 0xA5;
    uint8_t scans;
    uint8_t spin;
    float linear_x;
    float linear_y;
    float angular_z;
    uint8_t robot_id;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint16_t shooter_17mm_1_barrel_heat;
    float chassis_power;
    uint16_t buffer_energy;
    uint8_t launching_frequency;
    float null;

} Serial_receive;

typedef enum
{
    no_speed = 0,
    low_speed = 1,
    high_speed = 2,
} speed;


#endif
