//
// Created by madswamp on 28/04/21.
//
#include <geometry_msgs/Quaternion.h>
#include "../../include/mavic_interface_ros/mavic_send_velocity_aircraft/mavic_send_velocity_aircraft.h"


mavic_send_velocity_aircraft::mavic_send_velocity_aircraft()
: listener(tf_buffer)
{
    aircraft_command_pub=nh.advertise<std_msgs::String>("Mavic_Commands",1,true);

    aircraft_speed_pub=nh.advertise<geometry_msgs::Quaternion>("Velocity_Commands",1,true);

    aircraft_response_sub=nh.subscribe("Responses",1,
                                       &mavic_send_velocity_aircraft::aicraft_response_callback,this);

    motion_speed_sub=nh.subscribe("motion_reference/speed",1,
                                  &mavic_send_velocity_aircraft::motion_speed_callback,this);

    emergency_sub=nh.subscribe("Emergency",1,&mavic_send_velocity_aircraft::emergency_callback,this);


    ros::param::get("~Vertical_Control_Mode",vertical_mode);
    ros::param::get("~Roll_Pitch_Mode",roll_pitch_mode);
    ros::param::get("~Yaw_Mode",yaw_mode);
    ros::param::get("~Coordinate_System",coordinate_system);
    aircraft_response.data=-1;
    ros::Duration(0.5).sleep();
    int result=set_up_aircraft();
    if(result==0)
    {
        ROS_INFO("Aircraft Set Up Failed");
        exit(0);
    }
    ROS_INFO("Aircraft Set Up Complete");
}

void mavic_send_velocity_aircraft::motion_speed_callback(const geometry_msgs::TwistStampedConstPtr &msg)
{
    velocity_cmd = *msg;
    flag_new_velocity=true;
    if (flag_first_velocity==0)
    {
        flag_first_velocity = true;
    }
}

void mavic_send_velocity_aircraft::aicraft_response_callback(const std_msgs::Int32ConstPtr &msg)
{
    aircraft_response = *msg;
}

int  mavic_send_velocity_aircraft::set_up_aircraft()
{
    std_msgs::String  set_up_command_msg;
    set_up_command_msg.data=std::string ("SetUp ") + vertical_mode + std::string (" ") +
            roll_pitch_mode + std::string (" ") + yaw_mode + std::string (" ") + coordinate_system;

    aircraft_command_pub.publish(set_up_command_msg);
    while(aircraft_response.data==-1)
    {
        ros::spinOnce();
    }
    if(aircraft_response.data==1)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void mavic_send_velocity_aircraft::send_velocity()
{
    geometry_msgs::TransformStamped velocity_transform;
    try
    {
        velocity_transform = tf_buffer.lookupTransform("drone0_velocity", "drone0_optimized", ros::Time(0));
        geometry_msgs::Point xyz_velocity_trasform;
        xyz_velocity_trasform.x=velocity_cmd.twist.linear.x;
        xyz_velocity_trasform.y=velocity_cmd.twist.linear.y;
        xyz_velocity_trasform.z=velocity_cmd.twist.linear.z;
        double angular_yaw=0;
        angular_yaw=velocity_cmd.twist.angular.z;
        angular_yaw=angular_yaw*180/M_PI;

        tf2::doTransform(xyz_velocity_trasform,xyz_velocity_trasform,velocity_transform);


        if(xyz_velocity_trasform.x>0.4)
        {
            xyz_velocity_trasform.x=0.4;
        }
        else if(xyz_velocity_trasform.x<-0.4)
        {
            xyz_velocity_trasform.x=-0.4;
        }
        if(xyz_velocity_trasform.y>0.4)
        {
            xyz_velocity_trasform.y=0.4;
        }
        else if(xyz_velocity_trasform.y<-0.4)
        {
            xyz_velocity_trasform.y=-0.4;
        }
        if(xyz_velocity_trasform.z<-0.4)
        {
            xyz_velocity_trasform.z=-0.4;
        }
        else if(xyz_velocity_trasform.z>0.4)
        {
            xyz_velocity_trasform.z=0.4;
        }
        if(angular_yaw>10)
        {
            angular_yaw=10;
        }
        else if(angular_yaw<-10)
        {
            angular_yaw=-10;
        }

        geometry_msgs::Quaternion velocity_send;
        velocity_send.x=xyz_velocity_trasform.x;
        velocity_send.y=xyz_velocity_trasform.y;
        velocity_send.z=xyz_velocity_trasform.z;
        velocity_send.w=angular_yaw;
        ROS_INFO("%f %f %f %f",xyz_velocity_trasform.x,xyz_velocity_trasform.y,xyz_velocity_trasform.z,angular_yaw);

        aircraft_speed_pub.publish(velocity_send);
    }
    catch (tf2::TransformException &ex)
    {
        ros::Duration(0.01).sleep();
    }

}

void mavic_send_velocity_aircraft::activate_joystick_mode(bool on_off)
{
    std_msgs::String joystick_command_msg;
    if(on_off==true)
    {
        joystick_command_msg.data=std::string ("Joystick On");
    }
    else
    {
        joystick_command_msg.data=std::string ("Joystick Off");
    }
    aircraft_command_pub.publish(joystick_command_msg);
}

mavic_send_velocity_aircraft::~mavic_send_velocity_aircraft()
{
    activate_joystick_mode(false);
}

void mavic_send_velocity_aircraft::emergency_callback(const std_msgs::StringConstPtr &msg)
{
    motion_speed_sub.shutdown();
    geometry_msgs::Quaternion velocity_send;
    velocity_send.x=0;
    velocity_send.y=0;
    velocity_send.z=0;
    velocity_send.w=0;
    aircraft_speed_pub.publish(velocity_send);
}

