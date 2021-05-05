//
// Created by madswamp on 28/04/21.
//

#ifndef MAVIC_INTERFACE_ROS_MAVIC_SEND_VELOCITY_AIRCRAFT_H
#define MAVIC_INTERFACE_ROS_MAVIC_SEND_VELOCITY_AIRCRAFT_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class mavic_send_velocity_aircraft
{
private:

    ros::NodeHandle nh;
    ros::Subscriber motion_speed_sub,aircraft_response_sub,emergency_sub;
    ros::Publisher aircraft_command_pub,aircraft_speed_pub;

    void motion_speed_callback(const geometry_msgs::TwistStampedConstPtr& msg);
    void aicraft_response_callback(const std_msgs::Int32ConstPtr& msg);

    std::string vertical_mode,roll_pitch_mode,yaw_mode,coordinate_system;

    int set_up_aircraft();

    std_msgs::Int32 aircraft_response;

    geometry_msgs::TwistStamped velocity_cmd;

    tf2_ros::Buffer tf_buffer;

    tf2_ros::TransformListener listener;

    void emergency_callback(const std_msgs::StringConstPtr& msg);

public:

    mavic_send_velocity_aircraft();
    ~mavic_send_velocity_aircraft();
    int flag_first_velocity=0;
    bool flag_new_velocity=false;
    void activate_joystick_mode(bool on_off);
    void send_velocity();
};

#endif //MAVIC_INTERFACE_ROS_MAVIC_SEND_VELOCITY_AIRCRAFT_H
