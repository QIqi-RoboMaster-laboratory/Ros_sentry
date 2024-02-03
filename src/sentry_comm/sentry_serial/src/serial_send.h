#ifndef SERIAL_SEND_H
#define SERIAL_SEND_H
#include <serial/serial.h>  
#include <std_msgs/String.h>  


serial::Serial sentry_ser; //声明串口对象

uint8_t data_len = 25;
std::string cmd_vel_topic;

union Serial_Package
{
    struct
    {
        uint8_t  header = 0xA5;
        float   linear_x;
        float   linear_y;
        float   linear_z;
        float   angular_x;
        float   angular_y;
        float   angular_z;

    };
    uint8_t Send_Buffer[25];
};

    struct
    {
        uint8_t  header = 0xA5;
        float   linear_x;
        float   linear_y;
        float   linear_z;
        float   angular_x;
        float   angular_y;
        float   angular_z;

    }Serial_receive;


#endif

