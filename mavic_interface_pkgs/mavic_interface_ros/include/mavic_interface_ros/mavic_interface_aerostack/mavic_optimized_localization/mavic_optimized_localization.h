//
// Created by madswamp on 27/04/21.
//

#ifndef MAVIC_INTERFACE_ROS_MAVIC_OPTIMIZED_LOCALIZATION_H
#define MAVIC_INTERFACE_ROS_MAVIC_OPTIMIZED_LOCALIZATION_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class mavic_optimized_localization
{
private:

    ros::NodeHandle nh;
    ros::Subscriber optimized_pose_sub;

    ros::Publisher self_localization_pose_pub,self_localization_velocity_pub;


    void optimized_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

    geometry_msgs::PoseWithCovarianceStamped optimized_aircraft_pose;

    geometry_msgs::PoseStamped optimized_pose_aerostack;

    geometry_msgs::TwistStamped optimized_velocity_aerostack;

    geometry_msgs::PointStamped prev_velocity_time,prev_attitude_time;

    bool flag_first_pose=true;


public:
    mavic_optimized_localization();
    ~mavic_optimized_localization();
    bool flag_new_pose=false;
    void pub_localization_to_aerostack();
    void pub_velocity_to_aerostack();
};



#endif //MAVIC_INTERFACE_ROS_MAVIC_OPTIMIZED_LOCALIZATION_H