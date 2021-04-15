
#include "../include/mavic_transform_frames/mavic_transform_frames.h"


mavic_transform_frames::mavic_transform_frames() {
    mavic_transform_frames::set_up();
    ROS_INFO("Set Up Okay");
}

mavic_transform_frames::~mavic_transform_frames() {

}


void mavic_transform_frames::VelocityCallback(const geometry_msgs::QuaternionStampedConstPtr &velocity)
{

}

void mavic_transform_frames::AttitudeCallback(const geometry_msgs::PointStampedConstPtr &attitude) {

    if(flag_frame_ready== false)
    {
        if(count_median==50)
        {
            geometry_msgs::TransformStamped static_transform_message;

            roll_frame=roll_frame/count_median;
            pitch_frame=pitch_frame/count_median;
            yaw_frame=yaw_frame/count_median;

            roll_frame=roll_frame*M_PI/180;
            pitch_frame=pitch_frame*M_PI/180;
            yaw_frame=yaw_frame*M_PI/180;

            static_transform_message.header.stamp=ros::Time::now();
            static_transform_message.header.frame_id="world";
            static_transform_message.child_frame_id="dji_frame";

            static_transform_message.transform.translation.x=0;
            static_transform_message.transform.translation.y=0;
            static_transform_message.transform.translation.z=0.3;

            tf2::Quaternion q,q1;

            q.setRPY(-roll_frame, -pitch_frame, -yaw_frame+(M_PI/2));
            q1.setRPY(0,M_PI,0);
            q=q1*q;

            static_transform_message.transform.rotation.x=q.x();
            static_transform_message.transform.rotation.y=q.y();
            static_transform_message.transform.rotation.z=q.z();
            static_transform_message.transform.rotation.w=q.w();

            tf_broadcaster_frame.sendTransform(static_transform_message);

            //flag_frame_ready=true;

        }
        else
        {
            roll_frame=roll_frame+attitude->point.x;
            pitch_frame=roll_frame+attitude->point.y;
            yaw_frame=roll_frame+attitude->point.z;
            count_median++;
        }
    }
    else
    {

    }

}

void mavic_transform_frames::GimbalAttitudeCallback(const geometry_msgs::PointStampedConstPtr &GimbalAttitude) {

}



void mavic_transform_frames::set_up()
{
    attitude_sub=nh.subscribe("/Attitude_RPY",1,&mavic_transform_frames::AttitudeCallback,this);
    gimbal_attitude_sub=nh.subscribe("/Gimbal_Attitude",1,&mavic_transform_frames::GimbalAttitudeCallback,this);
    velocity_sub=nh.subscribe("/Velocity",1,&mavic_transform_frames::VelocityCallback,this);
    while(flag_frame_ready);
}