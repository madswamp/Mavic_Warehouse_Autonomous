//
// Created by andre on 14/04/21.
//

#ifndef MAVIC_INTERFACE_MAVIC_STATE2_H
#define MAVIC_INTERFACE_MAVIC_STATE2_H

#endif //MAVIC_INTERFACE_MAVIC_STATE2_H

#include <iostream>

//// ROS  ///////
#include "ros/ros.h"
#include <robot_process.h>

#include <tf/transform_broadcaster.h>

#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/AccelStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <geometry_msgs/PointStamped.h>
#include "aerostack_msgs/FlightActionCommand.h"
#include "aerostack_msgs/FlightState.h"

using namespace std;

struct rpy {
    double R, P, Y, dR, dP, dY;
};

class StateInterface : public RobotProcess{
public:
    StateInterface();
    ~StateInterface();
private:
    void ownSetUp();
    void ownStart();
    void ownStop();
    void ownRun();
    bool resetValues();

    void sendFlightStatus(geometry_msgs::TwistStamped sensor_speed_msg, geometry_msgs::PointStamped sensor_altitude_msg);
    void flightActionCallback(const aerostack_msgs::FlightActionCommand::ConstPtr& msg);
    void velocityCallback(const geometry_msgs::QuaternionStamped);
    void imuCallback(const geometry_msgs::PointStamped);

    ros::NodeHandle n;
    ros::Publisher flight_state_pub;
    ros::Publisher speed_pub;
    ros::Publisher imu_pub;
    ros::Publisher alt_pub;

    ros::Subscriber flight_action_sub;
    ros::Subscriber velocity_sub;
    ros::Subscriber imu_sub;

    rpy imudata;
    bool flag_firsttime;
    aerostack_msgs::FlightActionCommand flight_action_msg;
    aerostack_msgs::FlightState flight_state_msg;
    ros::Time time_status;
    ros::Time current_timestamp;
    ros::Time current_timestamp_imu;
    ros::Time prev_timestamp_imu;
    geometry_msgs::TwistStamped speed_msg;
    geometry_msgs::PointStamped altitude_msg;
    sensor_msgs::Imu imu_msg;
};