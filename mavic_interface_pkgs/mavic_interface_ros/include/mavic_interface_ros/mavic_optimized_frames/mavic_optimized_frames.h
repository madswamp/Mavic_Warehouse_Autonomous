//
// Created by madswamp on 26/04/21.
//

#ifndef MAVIC_INTERFACE_MAVIC_OPTIMIZED_FRAMES_H
#define MAVIC_INTERFACE_MAVIC_OPTIMIZED_FRAMES_H

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PointStamped.h>


class mavic_optimized_frames
{

private:

    ros::NodeHandle nh;

    ros::Subscriber optimized_pose_sub,relative_yaw_gimbal_aircraft_sub,attitude_gimbal_world_sub;

    void optimized_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void relative_yaw_gimbal_aircraft_callback(const std_msgs::Float32ConstPtr& msg);
    void attitude_gimbal_world_callback(const geometry_msgs::PointStampedConstPtr& msg);

    std_msgs::Float32 relative_yaw_gimbal;
    geometry_msgs::PointStamped attitude_gimbal;
    geometry_msgs::PoseWithCovarianceStamped pose_optimized;

    std::string aircraft_name;

public:

    mavic_optimized_frames();
    ~mavic_optimized_frames();
    void broadcast_optimized_transforms();
    bool flag_optimization_started=false;
};

#endif //MAVIC_INTERFACE_MAVIC_OPTIMIZED_FRAMES_H
