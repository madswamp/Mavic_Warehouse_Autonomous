
#include "../include/mavic_transform_frames/mavic_transform_frames.h"


mavic_transform_frames::mavic_transform_frames() {
    mavic_transform_frames::set_up();
    while(!flag_frame_ready && !flag_frame_gimball_ready)
    {
        ros::spinOnce();
    }
    ROS_INFO("Set Up Finished");
}

mavic_transform_frames::~mavic_transform_frames() {

}


void mavic_transform_frames::VelocityCallback(const geometry_msgs::QuaternionStampedConstPtr &velocity)
{
     if(flag_frame_ready==true)
     {
         geometry_msgs::Vector3Stamped velocity_dji,velocity_our;
         velocity_dji.header.stamp=ros::Time::now();
         velocity_dji.header.frame_id="dji_frame";
         velocity_dji.vector.x=velocity->quaternion.x;
         velocity_dji.vector.y=velocity->quaternion.y;
         velocity_dji.vector.z=velocity->quaternion.z;

         listener.transformVector(std::string("world"),velocity_dji,velocity_our);

         geometry_msgs::QuaternionStamped velocity_world;

         velocity_world.header.stamp=ros::Time::now();
         velocity_world.header.frame_id="world";
         velocity_world.quaternion.x=velocity_our.vector.x;
         velocity_world.quaternion.y=velocity_our.vector.y;
         velocity_world.quaternion.z=-velocity_our.vector.z;
         velocity_world.quaternion.w=velocity->quaternion.w;
         velocity_pub.publish(velocity_world);



     }
}

void mavic_transform_frames::AttitudeCallback(const geometry_msgs::PointStampedConstPtr &attitude) {

    if(flag_frame_ready== false)
    {
        if(count_median==50)
        {
            geometry_msgs::TransformStamped static_transform_message;


            //roll_frame=roll_frame/count_median;
            //pitch_frame=pitch_frame/count_median;
            yaw_frame=yaw_frame/count_median;


            //roll_frame=roll_frame*M_PI/180;
            //pitch_frame=pitch_frame*M_PI/180;
            yaw_frame=yaw_frame*M_PI/180;

            static_transform_message.header.stamp=ros::Time::now();
            static_transform_message.header.frame_id="world";
            static_transform_message.child_frame_id="dji_frame";

            static_transform_message.transform.translation.x=0;
            static_transform_message.transform.translation.y=0;
            static_transform_message.transform.translation.z=0;

            tf2::Quaternion q,q1;


            q.setRPY(0, 0, -yaw_frame);

            static_transform_message.transform.rotation.x=q.x();
            static_transform_message.transform.rotation.y=q.y();
            static_transform_message.transform.rotation.z=q.z();
            static_transform_message.transform.rotation.w=q.w();

            tf_broadcaster_frame.sendTransform(static_transform_message);


            flag_frame_ready=true;
        }
        else
        {
            //roll_frame=roll_frame+attitude->point.x;
            //pitch_frame=pitch_frame+attitude->point.y;
            yaw_frame=yaw_frame+attitude->point.z;
            count_median++;
        }
    }
    else
    {
        geometry_msgs::PointStamped attitude_world;

        attitude_world.header.frame_id="world";
        attitude_world.header.stamp=ros::Time::now();

        attitude_world.point.x=attitude->point.x;
        attitude_world.point.y=attitude->point.y;
        attitude_world.point.z=constrainAngle(attitude->point.z-(yaw_frame*180/M_PI));

        attitude_pub.publish(attitude_world);
    }
}

void mavic_transform_frames::GimbalAttitudeCallback(const geometry_msgs::PointStampedConstPtr &GimbalAttitude) {

    if(flag_frame_gimball_ready== false)
    {
        if(count_median_gimbal==50)
        {
            //roll_frame=roll_frame/count_median;
            //pitch_frame=pitch_frame/count_median;
            yaw_frame_gimbal=yaw_frame_gimbal/count_median_gimbal;


            //roll_frame=roll_frame*M_PI/180;
            //pitch_frame=pitch_frame*M_PI/180;
            yaw_frame_gimbal=yaw_frame_gimbal*M_PI/180;

            flag_frame_gimball_ready=true;
        }
        else
        {
            //roll_frame_gimbal=roll_frame_gimbal+GimbalAttitude->point.x;
            //pitch_frame_gimbal=pitch_frame_gimbal+GimbalAttitude->point.y;
            yaw_frame_gimbal=yaw_frame_gimbal+GimbalAttitude->point.z;
            count_median_gimbal++;
        }
    }
    else
    {
        geometry_msgs::PointStamped attitude_gimbal_world;

        attitude_gimbal_world.header.frame_id="world";
        attitude_gimbal_world.header.stamp=ros::Time::now();

        attitude_gimbal_world.point.x=GimbalAttitude->point.x;
        attitude_gimbal_world.point.y=GimbalAttitude->point.y;
        attitude_gimbal_world.point.z=constrainAngle(GimbalAttitude->point.z-(yaw_frame_gimbal*180/M_PI));


        gimbal_attitude_pub.publish(attitude_gimbal_world);
    }
}

double mavic_transform_frames::constrainAngle(double x){
    x = fmod(x + 180,360);
    if (x < 0)
        x += 360;
    return x - 180;
}




void mavic_transform_frames::set_up()
{
    attitude_sub=nh.subscribe("/Attitude_RPY",1,&mavic_transform_frames::AttitudeCallback,this);
    gimbal_attitude_sub=nh.subscribe("/Gimbal_Attitude",1,&mavic_transform_frames::GimbalAttitudeCallback,this);
    velocity_sub=nh.subscribe("/Velocity",1,&mavic_transform_frames::VelocityCallback,this);

    velocity_pub=nh.advertise<geometry_msgs::QuaternionStamped>("Velocity_World",1,true);
    gimbal_attitude_pub=nh.advertise<geometry_msgs::PointStamped>("Gimbal_Attitude_World",1,true);
    attitude_pub=nh.advertise<geometry_msgs::PointStamped>("Attitude_World",1,true);
}