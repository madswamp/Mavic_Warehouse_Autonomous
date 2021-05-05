//
// Created by madswamp on 27/04/21.
//
#include "../../../include/mavic_interface_ros/mavic_interface_aerostack/mavic_optimized_localization/mavic_optimized_localization.h"



mavic_optimized_localization::mavic_optimized_localization()
{
    optimized_pose_sub=nh.subscribe("optimized_pose",1,
                                    &mavic_optimized_localization::optimized_pose_callback,this);

    self_localization_pose_pub=nh.advertise<geometry_msgs::PoseStamped>("self_localization/pose",1,true);

    self_localization_velocity_pub=nh.advertise<geometry_msgs::TwistStamped>("self_localization/speed",1,true);

}

mavic_optimized_localization::~mavic_optimized_localization()
{


}



void mavic_optimized_localization::optimized_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
    optimized_aircraft_pose= *msg;
    flag_new_pose=true;
}

void mavic_optimized_localization::pub_localization_to_aerostack()
{
    optimized_pose_aerostack.header.stamp=ros::Time::now();
    optimized_pose_aerostack.header.frame_id="map_ned";
    optimized_pose_aerostack.pose=optimized_aircraft_pose.pose.pose;
    self_localization_pose_pub.publish(optimized_pose_aerostack);
}

void mavic_optimized_localization::pub_velocity_to_aerostack()
{
    if(flag_first_pose)
    {
        prev_velocity_time.header.stamp=ros::Time::now();
        prev_velocity_time.point.x=optimized_aircraft_pose.pose.pose.position.x;
        prev_velocity_time.point.y=optimized_aircraft_pose.pose.pose.position.y;
        prev_velocity_time.point.z=optimized_aircraft_pose.pose.pose.position.z;

        tf2::Quaternion orientation_quat;

        orientation_quat.setX(optimized_aircraft_pose.pose.pose.orientation.x);
        orientation_quat.setY(optimized_aircraft_pose.pose.pose.orientation.y);
        orientation_quat.setZ(optimized_aircraft_pose.pose.pose.orientation.z);
        orientation_quat.setW(optimized_aircraft_pose.pose.pose.orientation.w);

        tf2::Matrix3x3(orientation_quat).getRPY(prev_attitude_time.point.x,prev_attitude_time.point.y,prev_attitude_time.point.z);
        flag_first_pose=false;
    }
    else
    {
        ros::Time current_time=ros::Time::now();
        double diff_time=(current_time - prev_velocity_time.header.stamp).nsec / 1E9;
        optimized_velocity_aerostack.header.stamp=current_time;
        optimized_velocity_aerostack.header.frame_id="map_ned";
        optimized_velocity_aerostack.twist.linear.x=(optimized_aircraft_pose.pose.pose.position.x-prev_velocity_time.point.x)/diff_time;
        optimized_velocity_aerostack.twist.linear.y=(optimized_aircraft_pose.pose.pose.position.y-prev_velocity_time.point.y)/diff_time;
        optimized_velocity_aerostack.twist.linear.z=(optimized_aircraft_pose.pose.pose.position.z-prev_velocity_time.point.z)/diff_time;

        prev_velocity_time.point.x=optimized_aircraft_pose.pose.pose.position.x;
        prev_velocity_time.point.y=optimized_aircraft_pose.pose.pose.position.y;
        prev_velocity_time.point.z=optimized_aircraft_pose.pose.pose.position.z;

        tf2::Quaternion orientation_quat;

        orientation_quat.setX(optimized_aircraft_pose.pose.pose.orientation.x);
        orientation_quat.setY(optimized_aircraft_pose.pose.pose.orientation.y);
        orientation_quat.setZ(optimized_aircraft_pose.pose.pose.orientation.z);
        orientation_quat.setW(optimized_aircraft_pose.pose.pose.orientation.w);

        double roll=0,pitch=0,yaw=0;

        tf2::Matrix3x3(orientation_quat).getRPY(roll,pitch,yaw);

        double diff_roll=(roll-prev_attitude_time.point.x)/diff_time;
        double diff_pitch=(pitch-prev_attitude_time.point.y)/diff_time;
        double diff_yaw=(yaw-prev_attitude_time.point.z)/diff_time;

        double Angular_roll=(diff_roll)-(sin(pitch)*diff_yaw);
        double Angular_pitch=(cos(roll) * diff_pitch) + (cos(pitch)*sin(roll) * diff_yaw);
        double Angular_yaw=(-sin(roll) * diff_pitch) + (cos(roll)*cos(pitch) * diff_yaw);

        if(Angular_roll>40 || Angular_roll<-40)
        {
            Angular_roll=0;
        }

        if(Angular_pitch>40 || Angular_pitch<-40)
        {
            Angular_pitch=0;
        }

        if(Angular_yaw>40 || Angular_yaw<-40)
        {
            Angular_yaw=0;
        }

        optimized_velocity_aerostack.twist.angular.x=Angular_roll;
        optimized_velocity_aerostack.twist.angular.y=Angular_pitch;
        optimized_velocity_aerostack.twist.angular.z=Angular_yaw;

        prev_velocity_time.header.stamp=current_time;
        prev_attitude_time.point.x=roll;
        prev_attitude_time.point.y=pitch;
        prev_attitude_time.point.z=yaw;

        self_localization_velocity_pub.publish(optimized_velocity_aerostack);
    }

}