//
// Created by madswamp on 19/04/21.
//

#ifndef MAVIC_INTERFACE_MAVIC_STATE_H
#define MAVIC_INTERFACE_MAVIC_STATE_H

#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <aerostack_msgs/FlightState.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <aerostack_msgs/FlightActionCommand.h>
#include <std_msgs/String.h>

class mavic_state
{
public:

    mavic_state();
    ~mavic_state();
    geometry_msgs::QuaternionStamped velocity_aircraft_world;
    geometry_msgs::PointStamped attitude_aircraft_world;
    void send_state_data_aerostack();
    void send_state_aerostack();
    bool flag_flight_action=false;
    void send_command_aircraft();

private:

    ros::Subscriber velocity_sub,attitude_sub,aerostack_flight_action_sub;
    ros::Publisher aerostack_flight_state_pub,linear_speed_rad_pub,imu_rad_pub,altitude_pub,linear_speed_deg_pub,
    aircraft_commands_pub;
    ros::NodeHandle nh;

    void AttitudeCallback(const geometry_msgs::PointStampedConstPtr& attitude);
    void VelocityCallback(const geometry_msgs::QuaternionStampedConstPtr& velocity);
    void flight_action_callback(const aerostack_msgs::FlightActionCommandConstPtr& msg);

    bool flag_first_data=false;

    double prev_roll=0,prev_pitch=0,prev_yaw=0;

    ros::Time current_time,prev_time;

    geometry_msgs::TwistStamped linear_speed_msg;
    sensor_msgs::Imu imu_msg;
    geometry_msgs::PointStamped drone_altitude_msg;
    aerostack_msgs::FlightActionCommand command_aerostack;
    aerostack_msgs::FlightState aircraft_state;

    ros::Time time_takeoff;
};

#endif //MAVIC_INTERFACE_MAVIC_STATE_H
