//
// Created by madswamp on 22/04/21.
//

#include "../../include/mavic_interface_ros/mavic_raw_localization/mavic_raw_localization.h"


void mavic_raw_localization::velocity_callback(const geometry_msgs::QuaternionStampedConstPtr& msg)
{
    if(!flag_velocity)
    {
        prev_integration_time=ros::Time::now();
        velocity_integrator.point.x=0;
        velocity_integrator.point.y=0;
        velocity_integrator.point.z=0;
        flag_velocity=true;
    }
    else
    {
        ros::Time current_integration_time=ros::Time::now();
        double diffTime=(current_integration_time - prev_integration_time).nsec / 1E9;
        velocity_integrator.point.x=velocity_integrator.point.x+msg->quaternion.x*diffTime;
        velocity_integrator.point.y=velocity_integrator.point.y+msg->quaternion.y*diffTime;
        velocity_integrator.point.z=velocity_integrator.point.z+msg->quaternion.z*diffTime;

        prev_integration_time=current_integration_time;
    }

}

void mavic_raw_localization::attitude_callback(const geometry_msgs::PointStampedConstPtr& msg)
{
    attitude_data = *msg;
    flag_attitude = true;
}
void mavic_raw_localization::gimbal_callback(const geometry_msgs::PointStampedConstPtr &msg)
{
    gimbal_data = *msg;
    flag_gimbal = true;
}

void mavic_raw_localization::pub_pose_broadcast_tf()
{
    static tf2_ros::TransformBroadcaster tf;
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PoseStamped pose_msg;
    tf2::Quaternion attitude_aircraft,attitude_gimbal;

    transformStamped.header.stamp=ros::Time::now();
    transformStamped.header.frame_id="/world";
    transformStamped.child_frame_id=aircraft_name;
    transformStamped.transform.translation.x=velocity_integrator.point.x;
    transformStamped.transform.translation.y=velocity_integrator.point.y;
    transformStamped.transform.translation.z=velocity_integrator.point.z;
    attitude_aircraft.setRPY(attitude_data.point.x*M_PI/180,attitude_data.point.y*M_PI/180,attitude_data.point.z*M_PI/180);
    transformStamped.transform.rotation.x=attitude_aircraft.x();
    transformStamped.transform.rotation.y=attitude_aircraft.y();
    transformStamped.transform.rotation.z=attitude_aircraft.z();
    transformStamped.transform.rotation.w=attitude_aircraft.w();
    tf.sendTransform(transformStamped);

    transformStamped.header.frame_id="/world";
    transformStamped.child_frame_id=aircraft_name+"_gimbal";
    transformStamped.transform.translation.x=velocity_integrator.point.x;
    transformStamped.transform.translation.y=velocity_integrator.point.y;
    transformStamped.transform.translation.z=velocity_integrator.point.z;
    attitude_gimbal.setRPY(gimbal_data.point.x*M_PI/180,gimbal_data.point.y*M_PI/180,gimbal_data.point.z*M_PI/180);
    transformStamped.transform.rotation.x=attitude_gimbal.x();
    transformStamped.transform.rotation.y=attitude_gimbal.y();
    transformStamped.transform.rotation.z=attitude_gimbal.z();
    transformStamped.transform.rotation.w=attitude_gimbal.w();
    tf.sendTransform(transformStamped);

    pose_msg.header.frame_id="/world";
    pose_msg.header.stamp=ros::Time::now();
    pose_msg.pose.orientation.x=attitude_aircraft.x();
    pose_msg.pose.orientation.y=attitude_aircraft.y();
    pose_msg.pose.orientation.z=attitude_aircraft.z();
    pose_msg.pose.orientation.w=attitude_aircraft.w();
    pose_msg.pose.position=velocity_integrator.point;
    raw_pose_pub.publish(pose_msg);
}

mavic_raw_localization::mavic_raw_localization()
{
    aircraft_name = nh.getNamespace();
    velocity_sub=nh.subscribe("Velocity_World",1,&mavic_raw_localization::velocity_callback,this);
    attitude_sub=nh.subscribe("Attitude_World",1,&mavic_raw_localization::attitude_callback,this);
    gimbal_sub=nh.subscribe("Gimbal_Attitude_World",1,&mavic_raw_localization::gimbal_callback,this);
    raw_pose_pub=nh.advertise<geometry_msgs::PoseStamped>("raw_localization/raw_pose",1,true);
}

mavic_raw_localization::~mavic_raw_localization()
{

}