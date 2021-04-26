//
// Created by madswamp on 26/04/21.
//

#ifndef MAVIC_INTERFACE_ROS_MAVIC_DUNKER_POSE_H
#define MAVIC_INTERFACE_ROS_MAVIC_DUNKER_POSE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_listener.h>

class mavic_dunker_pose
{
private:

    ros::NodeHandle nh;
    ros::Subscriber tag_detected_flag_sub,aircraft_optimized_pose_sub;
    ros::Publisher not_dunker_pose_pub;

    void aircraft_optimized_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void tag_detected_flag_callback(const std_msgs::BoolConstPtr& msg);

    std::string aircraft_name;

    geometry_msgs::PoseWithCovarianceStamped aircraft_optimized_pose;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener;



public:

    mavic_dunker_pose();
    ~mavic_dunker_pose();

    void send_dunker_pose();
    bool flag_dunker_detected=false;

};


#endif //MAVIC_INTERFACE_ROS_MAVIC_DUNKER_POSE_H
