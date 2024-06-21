#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <string>
#include <cmath>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include "sentry_referee/game_state.h"
#include "sentry_referee/game_robot_HP.h"
#include "sentry_referee/field_event.h"
#include "sentry_referee/robot_state.h"
#include "sentry_referee/power_heat_data.h"
#include "sentry_referee/robot_hurt.h"
#include "sentry_referee/shoot_data.h"
#include "sentry_control.h"

ros::Subscriber game_state_subscriber;
ros::Subscriber game_robot_HP_subscriber;
ros::Subscriber field_event_subscriber;
ros::Subscriber robot_state_subscriber;
ros::Subscriber power_heat_data_subscriber;
ros::Subscriber robot_hurt_subscriber;
ros::Subscriber shoot_data_subscriber;

ros::Subscriber cmd_vel_sub;
ros::Subscriber move_base_result_sub;
ros::Subscriber global_pose_sub;
ros::Publisher move_base_goal_pub;

sentry_referee::game_state game_state_msg;
sentry_referee::game_robot_HP game_robot_HP_msg;
sentry_referee::field_event field_event_msg;
sentry_referee::robot_state robot_state_msg;
sentry_referee::power_heat_data power_heat_data_msg;
sentry_referee::robot_hurt robot_hurt_msg;
sentry_referee::shoot_data shoot_data_msg;

Serial_Package serial_package;
geometry_msgs::PoseStamped robot_pose;
uint8_t pose = 0;
bool plan_success = 0;

void robot_pose_callback(const move_base_msgs::MoveBaseActionFeedback & msg);
void cmd_vel_callback(const geometry_msgs::Twist &cmd_vel);
void game_state_callback(const sentry_referee::game_state::ConstPtr &msg_p);
void game_robot_HP_callback(const sentry_referee::game_robot_HP::ConstPtr &msg_p);
void field_event_callback(const sentry_referee::field_event::ConstPtr &msg_p);
void robot_state_callback(const sentry_referee::robot_state::ConstPtr &msg_p);
void power_heat_data_callback(const sentry_referee::power_heat_data::ConstPtr &msg_p);
void robot_hurt_callback(const sentry_referee::robot_hurt::ConstPtr &msg_p);
void shoot_data_callback(const sentry_referee::shoot_data::ConstPtr &msg_p);

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "sentry_control");
    ros::NodeHandle nh;
    auto start = ros::Time::now();

    try
    {
        std::string serial_port;
        serial_port = "/dev/Control_USB";
        std::cout << serial_port << std::endl;
        sentry_ser.setPort(serial_port);
        sentry_ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        sentry_ser.setTimeout(to);
        sentry_ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    if (sentry_ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port opened");
    }
    else
    {
        return -1;
    }
    ROS_INFO_STREAM("Init Finished!");

    cmd_vel_sub = nh.subscribe("cmd_vel", 10, cmd_vel_callback);
    move_base_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    
    global_pose_sub = nh.subscribe("/move_base/feedback", 5, robot_pose_callback);
    game_state_subscriber = nh.subscribe<sentry_referee::game_state>("/referee/game_state", 20, game_state_callback);
    game_robot_HP_subscriber = nh.subscribe<sentry_referee::game_robot_HP>("/referee/game_robot_HP", 20, game_robot_HP_callback);
    field_event_subscriber = nh.subscribe<sentry_referee::field_event>("/referee/field_event", 20, field_event_callback);
    robot_state_subscriber = nh.subscribe<sentry_referee::robot_state>("/referee/robot_state", 20, robot_state_callback);
    power_heat_data_subscriber = nh.subscribe<sentry_referee::power_heat_data>("/referee/power_heat_data", 20, power_heat_data_callback);
    robot_hurt_subscriber = nh.subscribe<sentry_referee::robot_hurt>("/referee/robot_hurt", 20, robot_hurt_callback);
    shoot_data_subscriber = nh.subscribe<sentry_referee::shoot_data>("/referee/shoot_data", 20, shoot_data_callback);

    double theta;
    bool low_HP_flat = 0;

    tf2::Quaternion pose_quad;
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";

    serial_package.header = 0xA5;

    ros::Rate loop_rate(20);

    int cnt = 0;

    while (ros::ok())
    {
        if(hypot(robot_pose.pose.position.x-goal.pose.position.x,robot_pose.pose.position.y-goal.pose.position.y) < 0.5)
        {
            plan_success = 1;
        }

        // std::cout<<hypot(robot_pose.pose.position.x-goal.pose.position.x,robot_pose.pose.position.y-goal.pose.position.y)<<std::endl;

        if(plan_success==1)
        {
            if(pose>=4) pose=0;
            pose++;
            plan_success=0;
        }

        switch(pose)
        {
            case 0:
            {   
                goal.header.stamp = ros::Time::now();
                goal.pose.position.x = 0;
                goal.pose.position.y = 0;
                theta = 0;
                break;
            }
            case 1:
            {   
                goal.header.stamp = ros::Time::now();
                goal.pose.position.x = 1.95;
                goal.pose.position.y = 0.529;
                theta = 0;
                break;
            }
            case 2:
            {   
                goal.header.stamp = ros::Time::now();
                goal.pose.position.x = 6.47;
                goal.pose.position.y = -0.229;
                theta = 0;
                break;
            }
            case 3:
            {   
                goal.header.stamp = ros::Time::now();
                goal.pose.position.x = 6.94;
                goal.pose.position.y = 8.09;
                theta = 0;
                break;
            }
            case 4:
            {   
                goal.header.stamp = ros::Time::now();
                goal.pose.position.x = 0.6;
                goal.pose.position.y = 7.88;
                theta = 0;
                break;
            }
            default:
            {   
                goal.header.stamp = ros::Time::now();
                goal.pose.position.x = 0;
                goal.pose.position.y = 0;
                theta = 0;
                break;
            }
        }

        pose_quad.setRPY(0, 0, theta);
        goal.pose.orientation.x = pose_quad.getX();
        goal.pose.orientation.y = pose_quad.getY();
        goal.pose.orientation.z = pose_quad.getZ();
        goal.pose.orientation.w = pose_quad.getW();

        serial_package.scan = 0;
        serial_package.spin = 0;
        
        sentry_ser.flush();
        sentry_ser.write(serial_package.Send_Buffer, data_len);
        // ROS_INFO_STREAM("\nSend date finished!\n");
        if(cnt%20 == 0) {move_base_goal_pub.publish(goal);}
        ros::spinOnce();
        loop_rate.sleep();

        cnt++;
    }

    return 0;
}
void robot_pose_callback(const move_base_msgs::MoveBaseActionFeedback & msg)
{
    robot_pose=msg.feedback.base_position;

    // ROS_INFO("robot_pose.pose.position.x=%f,robot_pose.pose.position.y=%f\n",robot_pose.pose.position.x,robot_pose.pose.position.y);
}

void cmd_vel_callback(const geometry_msgs::Twist &cmd_vel)
{
    // receive the msg from cmd_vel
    // ROS_INFO("Receive a /cmd_vel msg\n");
    ROS_INFO("The velocity: x=%f, y=%f, yaw=%f\n", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
    // put the data in union
    serial_package.linear_x = cmd_vel.linear.x;
    serial_package.linear_y = -cmd_vel.linear.y;
    serial_package.angular_z = -cmd_vel.angular.z;
}

void game_state_callback(const sentry_referee::game_state::ConstPtr &msg_p)
{
    game_state_msg.game_type = msg_p->game_type;
    game_state_msg.game_progress = msg_p->game_progress;
    game_state_msg.stage_remain_time = msg_p->stage_remain_time;
    game_state_msg.SyncTimeStamp = msg_p->SyncTimeStamp;

    // ROS_INFO("%d,%d,%d", game_state_msg.game_type, game_state_msg.game_progress, game_state_msg.stage_remain_time);
}


void game_robot_HP_callback(const sentry_referee::game_robot_HP::ConstPtr &msg_p)
{
    game_robot_HP_msg.red_1_robot_HP = msg_p->red_1_robot_HP;
    game_robot_HP_msg.red_2_robot_HP = msg_p->red_2_robot_HP;
    game_robot_HP_msg.red_3_robot_HP = msg_p->red_3_robot_HP;
    game_robot_HP_msg.red_4_robot_HP = msg_p->red_4_robot_HP;
    game_robot_HP_msg.red_5_robot_HP = msg_p->red_5_robot_HP;
    game_robot_HP_msg.red_7_robot_HP = msg_p->red_7_robot_HP;
    game_robot_HP_msg.red_outpost_HP = msg_p->red_outpost_HP;
    game_robot_HP_msg.red_base_HP = msg_p->red_base_HP;
    game_robot_HP_msg.blue_1_robot_HP = msg_p->blue_1_robot_HP;
    game_robot_HP_msg.blue_2_robot_HP = msg_p->blue_2_robot_HP;
    game_robot_HP_msg.blue_3_robot_HP = msg_p->blue_3_robot_HP;
    game_robot_HP_msg.blue_4_robot_HP = msg_p->blue_4_robot_HP;
    game_robot_HP_msg.blue_5_robot_HP = msg_p->blue_5_robot_HP;
    game_robot_HP_msg.blue_7_robot_HP = msg_p->blue_7_robot_HP;
    game_robot_HP_msg.blue_outpost_HP = msg_p->blue_outpost_HP;
    game_robot_HP_msg.blue_base_HP = msg_p->blue_base_HP;
}
void field_event_callback(const sentry_referee::field_event::ConstPtr &msg_p)
{
    field_event_msg.event_data = msg_p->event_data;
}
void robot_state_callback(const sentry_referee::robot_state::ConstPtr &msg_p)
{
    robot_state_msg.robot_id = msg_p->robot_id;
    serial_package.robot_id = robot_state_msg.robot_id;
    robot_state_msg.robot_level = msg_p->robot_level;
    robot_state_msg.current_HP = msg_p->current_HP;
    robot_state_msg.maximum_HP = msg_p->maximum_HP;
    robot_state_msg.shooter_barrel_cooling_value = msg_p->shooter_barrel_cooling_value;
    robot_state_msg.shooter_barrel_heat_limit = msg_p->shooter_barrel_heat_limit;
    serial_package.shooter_barrel_heat_limit = robot_state_msg.shooter_barrel_heat_limit;
    robot_state_msg.chassis_power_limit = msg_p->chassis_power_limit;
    serial_package.chassis_power_limit = robot_state_msg.chassis_power_limit;
}
void power_heat_data_callback(const sentry_referee::power_heat_data::ConstPtr &msg_p)
{
    power_heat_data_msg.chassis_voltage = msg_p->chassis_voltage;
    power_heat_data_msg.chassis_current = msg_p->chassis_current;
    power_heat_data_msg.chassis_power = msg_p->chassis_power;
    power_heat_data_msg.buffer_energy = msg_p->buffer_energy;
    serial_package.chassis_power = power_heat_data_msg.chassis_power;
    serial_package.buffer_energy = power_heat_data_msg.buffer_energy;
    power_heat_data_msg.shooter_17mm_1_barrel_heat = msg_p->shooter_17mm_1_barrel_heat;
    serial_package.shooter_17mm_1_barrel_heat = power_heat_data_msg.shooter_17mm_1_barrel_heat;
    power_heat_data_msg.shooter_17mm_2_barrel_heat = msg_p->shooter_17mm_2_barrel_heat;
    power_heat_data_msg.shooter_42mm_barrel_heat = msg_p->shooter_42mm_barrel_heat;
}
void robot_hurt_callback(const sentry_referee::robot_hurt::ConstPtr &msg_p)
{
    robot_hurt_msg.armor_id = msg_p->armor_id;
    robot_hurt_msg.HP_deduction_reason = msg_p->HP_deduction_reason;
}
void shoot_data_callback(const sentry_referee::shoot_data::ConstPtr &msg_p)
{
    shoot_data_msg.bullet_type = msg_p->bullet_type;
    shoot_data_msg.shooter_number = msg_p->shooter_number;
    shoot_data_msg.launching_frequency = msg_p->launching_frequency;
    shoot_data_msg.initial_speed = msg_p->initial_speed;
    serial_package.initial_speed = shoot_data_msg.initial_speed;
}
