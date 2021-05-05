//
// Created by madswamp on 26/04/21.
//

#include "../../include/mavic_interface_ros/mavic_optimized_frames/mavic_optimized_frames.h"



mavic_optimized_frames::mavic_optimized_frames()
{
    attitude_gimbal_world_sub=nh.subscribe("Gimbal_Attitude_World",1,
                                           &mavic_optimized_frames::attitude_gimbal_world_callback,this);

    optimized_pose_sub=nh.subscribe("optimized_pose",1,
                                    &mavic_optimized_frames::optimized_pose_callback,this);

    relative_yaw_gimbal_aircraft_sub=nh.subscribe("Relative_Yaw_Gimbal_Aircraft",1,
                                                  &mavic_optimized_frames::relative_yaw_gimbal_aircraft_callback,
                                                  this);

    aircraft_name = nh.getNamespace();
}

mavic_optimized_frames::~mavic_optimized_frames()
{

}

void mavic_optimized_frames::relative_yaw_gimbal_aircraft_callback(const std_msgs::Float32ConstPtr &msg)
{
    relative_yaw_gimbal= *msg;
}

void mavic_optimized_frames::optimized_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
    pose_optimized= *msg;
    flag_optimization_started=true;
}

void mavic_optimized_frames::attitude_gimbal_world_callback(const geometry_msgs::PointStampedConstPtr &msg)
{
    attitude_gimbal = *msg;
}

void mavic_optimized_frames::broadcast_optimized_transforms()
{
    static tf2_ros::TransformBroadcaster tf;
    geometry_msgs::TransformStamped transformStamped;

    tf2::Quaternion Gimbal_Attitude_Quaternion,Pose_Optimized_Quaternion,Image_Gimbal_Quaternion;

    transformStamped.header.stamp=ros::Time::now();
    transformStamped.header.frame_id="map_ned";
    transformStamped.child_frame_id=aircraft_name+"_gimbal_optimized";
    transformStamped.transform.translation.x=pose_optimized.pose.pose.position.x;
    transformStamped.transform.translation.y=pose_optimized.pose.pose.position.y;
    transformStamped.transform.translation.z=pose_optimized.pose.pose.position.z;

    Pose_Optimized_Quaternion.setX(pose_optimized.pose.pose.orientation.x);
    Pose_Optimized_Quaternion.setY(pose_optimized.pose.pose.orientation.y);
    Pose_Optimized_Quaternion.setZ(pose_optimized.pose.pose.orientation.z);
    Pose_Optimized_Quaternion.setW(pose_optimized.pose.pose.orientation.w);

    double roll,pitch,yaw;
    tf2::Matrix3x3(Pose_Optimized_Quaternion).getRPY(roll,pitch,yaw);

    Gimbal_Attitude_Quaternion.setRPY(attitude_gimbal.point.x*M_PI/180,
                                      attitude_gimbal.point.y*M_PI/180,yaw-relative_yaw_gimbal.data*M_PI/180);

    transformStamped.transform.rotation.x=Gimbal_Attitude_Quaternion.x();
    transformStamped.transform.rotation.y=Gimbal_Attitude_Quaternion.y();
    transformStamped.transform.rotation.z=Gimbal_Attitude_Quaternion.z();
    transformStamped.transform.rotation.w=Gimbal_Attitude_Quaternion.w();
    tf.sendTransform(transformStamped);


    transformStamped.header.frame_id=aircraft_name+"_gimbal_optimized";
    transformStamped.child_frame_id=aircraft_name+"_image_optimized";
    transformStamped.transform.translation.x=0;
    transformStamped.transform.translation.y=0;
    transformStamped.transform.translation.z=0;

    tf2::Quaternion q0_helper,q1_helper;
    q0_helper.setRPY(0,90*M_PI/180,0);
    q1_helper.setRPY(0,0,90*M_PI/180);

    Image_Gimbal_Quaternion=q0_helper*q1_helper;

    transformStamped.transform.rotation.x=Image_Gimbal_Quaternion.x();
    transformStamped.transform.rotation.y=Image_Gimbal_Quaternion.y();
    transformStamped.transform.rotation.z=Image_Gimbal_Quaternion.z();
    transformStamped.transform.rotation.w=Image_Gimbal_Quaternion.w();
    tf.sendTransform(transformStamped);

}