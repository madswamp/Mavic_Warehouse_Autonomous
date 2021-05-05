//
// Created by madswamp on 30/04/21.
//

#include "../../include/mavic_interface_ros/mavic_send_waypoints_free_bot/mavic_send_waypoints_free_bot.h"


mavic_send_waypoints_free_bot::mavic_send_waypoints_free_bot()
: listener(tf_buffer)
{
    tags_detected_sub=nh.subscribe("tag_detection_array",1,
                                   &mavic_send_waypoints_free_bot::tag_detection_array_callback,this);

    tag_position_pub=nh.advertise<geometry_msgs::PoseStamped>("/freebots/tags",10,true);

    drone_optimized_pose_sub=nh.subscribe("/drone0/self_localization/pose",1,
                                                                      &mavic_send_waypoints_free_bot::drone_optimized_callback,this);

    drone_map_pub=nh.advertise<geometry_msgs::PoseStamped>("/freebots/drone_pose",10,true);
}

mavic_send_waypoints_free_bot::~mavic_send_waypoints_free_bot()
{


}

void mavic_send_waypoints_free_bot::drone_optimized_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    drone_optimized_pose= *msg;
    flag_new_pose=true;
}

void mavic_send_waypoints_free_bot::tag_detection_array_callback(const std_msgs::Int32MultiArrayConstPtr &msg)
{
    tag_array= *msg;
    flag_new_tags=true;
}

void mavic_send_waypoints_free_bot::send_drone_optimized_pose()
{
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped=tf_buffer.lookupTransform("map","map_ned",ros::Time::now());

    geometry_msgs::PoseStamped drone_optimized_world;

    tf2::doTransform(drone_optimized_pose,drone_optimized_world,transform_stamped);

    /*drone_optimized_world.pose.position.x=transform_stamped.transform.translation.x;
    drone_optimized_world.pose.position.y=transform_stamped.transform.translation.y;
    drone_optimized_world.pose.position.z=transform_stamped.transform.translation.z;
    drone_optimized_world.pose.orientation.x=transform_stamped.transform.rotation.x;
    drone_optimized_world.pose.orientation.y=transform_stamped.transform.rotation.y;
    drone_optimized_world.pose.orientation.z=transform_stamped.transform.rotation.z;
    drone_optimized_world.pose.orientation.w=transform_stamped.transform.rotation.w;*/

    drone_map_pub.publish(drone_optimized_world);

}

void mavic_send_waypoints_free_bot::send_tags_free_bots()
{
    geometry_msgs::TransformStamped transformStamped;
    for(int i=0;i<tag_array.data.size();i++)
    {
        int id=tag_array.data.at(i);
        try
        {
            transformStamped = tf_buffer.lookupTransform("map",
                                                         "tag_"+std::to_string(id)+"_optimized",
                                                         ros::Time(0));
        }
        catch(tf2::TransformException &ex)
        {
            ros::Duration(0.01).sleep();
            flag_tf_error=true;
        }
        if(!flag_tf_error)
        {
            geometry_msgs::PoseStamped tag_msg;
            if(id==18)
            {

                tag_msg.pose.position.x=transformStamped.transform.translation.x;
                tag_msg.pose.position.y=transformStamped.transform.translation.y;
                tag_msg.pose.position.z=transformStamped.transform.translation.z;
                tag_msg.pose.orientation.x=transformStamped.transform.rotation.x;
                tag_msg.pose.orientation.y=transformStamped.transform.rotation.y;
                tag_msg.pose.orientation.z=transformStamped.transform.rotation.z;
                tag_msg.pose.orientation.w=transformStamped.transform.rotation.w;
                tag_msg.header.stamp=ros::Time::now();
                tag_msg.header.frame_id="victim";

            }
            else
            {
                tag_msg.header.stamp=ros::Time::now();
                tag_msg.header.frame_id="tag_"+std::to_string(id);
                tag_msg.pose.position.x=transformStamped.transform.translation.x;
                tag_msg.pose.position.y=transformStamped.transform.translation.y;
                tag_msg.pose.position.z=transformStamped.transform.translation.z;
                tag_msg.pose.orientation.x=transformStamped.transform.rotation.x;
                tag_msg.pose.orientation.y=transformStamped.transform.rotation.y;
                tag_msg.pose.orientation.z=transformStamped.transform.rotation.z;
                tag_msg.pose.orientation.w=transformStamped.transform.rotation.w;

            }
            tag_position_pub.publish(tag_msg);
        }
    }
}