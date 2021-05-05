//
// Created by madswamp on 30/04/21.
//

#ifndef MAVIC_INTERFACE_ROS_MAVIC_SEND_WAYPOINTS_FREE_BOT_H
#define MAVIC_INTERFACE_ROS_MAVIC_SEND_WAYPOINTS_FREE_BOT_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class mavic_send_waypoints_free_bot
{
private:
    ros::NodeHandle nh;
    ros::Publisher tag_position_pub,drone_map_pub;
    ros::Subscriber tags_detected_sub,drone_optimized_pose_sub;

    void tag_detection_array_callback(const std_msgs::Int32MultiArrayConstPtr& msg);
    void drone_optimized_callback(const geometry_msgs::PoseStampedConstPtr& msg);

    std_msgs::Int32MultiArray tag_array;
    bool flag_tf_error=false;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener;

    geometry_msgs::PoseStamped drone_optimized_pose;


public:

    mavic_send_waypoints_free_bot();
    ~mavic_send_waypoints_free_bot();
    bool flag_new_tags=false,flag_new_pose=false;
    void send_tags_free_bots();
    void send_drone_optimized_pose();
};
#endif //MAVIC_INTERFACE_ROS_MAVIC_SEND_WAYPOINTS_FREE_BOT_H
