//
// Created by madswamp on 26/04/21.
//

#include "../../include/mavic_interface_ros/mavic_dunker_pose/mavic_dunker_pose.h"


mavic_dunker_pose::mavic_dunker_pose()
: listener(tf_buffer)
{
    aircraft_optimized_pose_sub=nh.subscribe("optimized_pose",1,
                                             &mavic_dunker_pose::aircraft_optimized_pose_callback,this);

    tag_detected_flag_sub=nh.subscribe("tag_free_bot_flag",1,
                                       &mavic_dunker_pose::tag_detected_flag_callback,this);

    not_dunker_pose_pub=nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/robot/tag_position_sensor",1,true);

    aircraft_name=nh.getNamespace();

}

mavic_dunker_pose::~mavic_dunker_pose()
{

};

void mavic_dunker_pose::tag_detected_flag_callback(const std_msgs::BoolConstPtr &msg)
{
    flag_dunker_detected = msg->data;

}

void mavic_dunker_pose::aircraft_optimized_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
    aircraft_optimized_pose = *msg;
}

void mavic_dunker_pose::send_dunker_pose()
{
    bool flag_tf_error=true;
    geometry_msgs::TransformStamped transformStamped;
    while(flag_tf_error)
    {
        try
        {
            transformStamped = tf_buffer.lookupTransform("map","Not Dunker",ros::Time(0));
            flag_tf_error=false;
        }
        catch(tf2::TransformException &ex)
        {
            ros::Duration(0.01).sleep();
            continue;
        }
    }
    geometry_msgs::PoseWithCovarianceStamped dunker_pose;

    dunker_pose.header.stamp=ros::Time::now();
    dunker_pose.header.frame_id="map";
    dunker_pose.pose.pose.position.x=transformStamped.transform.translation.x;
    dunker_pose.pose.pose.position.y=transformStamped.transform.translation.y;
    dunker_pose.pose.pose.position.z=transformStamped.transform.translation.z;
    dunker_pose.pose.pose.orientation.x=transformStamped.transform.rotation.x;
    dunker_pose.pose.pose.orientation.y=transformStamped.transform.rotation.y;
    dunker_pose.pose.pose.orientation.z=transformStamped.transform.rotation.z;
    dunker_pose.pose.pose.orientation.w=transformStamped.transform.rotation.w;
    dunker_pose.pose.covariance=aircraft_optimized_pose.pose.covariance;
    not_dunker_pose_pub.publish(dunker_pose);
}
