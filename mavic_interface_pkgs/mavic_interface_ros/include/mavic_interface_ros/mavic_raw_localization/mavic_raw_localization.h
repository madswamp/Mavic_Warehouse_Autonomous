//
// Created by madswamp on 22/04/21.
//

#ifndef MAVIC_INTERFACE_MAVIC_RAW_LOCALIZATION_H
#define MAVIC_INTERFACE_MAVIC_RAW_LOCALIZATION_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>

class mavic_raw_localization
{
private:

    ros::NodeHandle nh;
    ros::Subscriber velocity_sub,attitude_sub,gimbal_sub;
    ros::Publisher raw_pose_pub;

    geometry_msgs::PointStamped gimbal_data;

    geometry_msgs::PointStamped velocity_integrator;

    geometry_msgs::PointStamped attitude_data;

    ros::Time prev_integration_time;

    std::string aircraft_name;

    void velocity_callback(const geometry_msgs::QuaternionStampedConstPtr& msg);
    void attitude_callback(const geometry_msgs::PointStampedConstPtr& msg);
    void gimbal_callback(const geometry_msgs::PointStampedConstPtr& msg);



public:
    mavic_raw_localization();
    ~mavic_raw_localization();

    bool flag_attitude=false,flag_velocity=false,flag_gimbal=false;

    void pub_pose_broadcast_tf();
};

#endif //MAVIC_INTERFACE_MAVIC_RAW_LOCALIZATION_H
