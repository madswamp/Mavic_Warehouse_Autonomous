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

class mavic_transform_frames
{

public:
    mavic_transform_frames();
    ~mavic_transform_frames();



private:

    void set_up();
    bool flag_frame_ready=false;
    ros::NodeHandle nh;
    ros::Subscriber attitude_sub,gimbal_attitude_sub,velocity_sub;

    void AttitudeCallback(const geometry_msgs::PointStampedConstPtr& attitude);
    void GimbalAttitudeCallback(const geometry_msgs::PointStampedConstPtr& GimbalAttitude);
    void VelocityCallback(const geometry_msgs::QuaternionStampedConstPtr& velocity);

    tf2_ros::StaticTransformBroadcaster tf_broadcaster_frame;

    double roll_frame=0,pitch_frame=0,yaw_frame=0;

    int count_median=0;


};


#endif //MAVIC_TRANSFORM_FRAMES_MAVIC_TRANSFORM_FRAMES_H
