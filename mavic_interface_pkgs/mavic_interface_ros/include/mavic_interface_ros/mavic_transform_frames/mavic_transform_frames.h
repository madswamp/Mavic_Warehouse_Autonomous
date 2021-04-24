//
// Created by madswamp on 14/04/21.
//

#ifndef MAVIC_TRANSFORM_FRAMES_MAVIC_TRANSFORM_FRAMES_H
#define MAVIC_TRANSFORM_FRAMES_MAVIC_TRANSFORM_FRAMES_H

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>



class mavic_transform_frames
{

public:
    mavic_transform_frames();
    ~mavic_transform_frames();
    void Relative_Yaw_Gimbal_Aircraft();
    bool flag_frame_ready=false,flag_frame_gimball_ready=false;

private:


    ros::NodeHandle nh;
    ros::Subscriber attitude_sub,gimbal_attitude_sub,velocity_sub;
    ros::Publisher velocity_pub,attitude_pub,gimbal_attitude_pub,relative_yaw_gimbal_aircraft_pub;


    void AttitudeCallback(const geometry_msgs::PointStampedConstPtr& attitude);
    void GimbalAttitudeCallback(const geometry_msgs::PointStampedConstPtr& GimbalAttitude);
    void VelocityCallback(const geometry_msgs::QuaternionStampedConstPtr& velocity);


    geometry_msgs::PointStamped attitude_gimbal_world;

    geometry_msgs::PointStamped attitude_world;

    tf2_ros::StaticTransformBroadcaster tf_broadcaster_frame;

    double yaw_frame=0,yaw_frame_gimbal=0;

    //double roll_frame=0,pitch_frame=0,roll_frame_gimbal=0,pitch_frame_gimbal=0;

    int count_median=0,count_median_gimbal=0;

    tf::TransformListener listener;

    double constrainAngle(double x);


};


#endif //MAVIC_TRANSFORM_FRAMES_MAVIC_TRANSFORM_FRAMES_H
