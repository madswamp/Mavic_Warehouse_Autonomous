//
// Created by madswamp on 19/04/21.
//

#include "../../../mavic_interface_ros/mavic_interface_aerostack/mavic_state/mavic_state.h"


mavic_state::mavic_state()
{
    velocity_sub=nh.subscribe("Velocity_World",1,&mavic_state::VelocityCallback,this);
    attitude_sub=nh.subscribe("Attitude_World",1,&mavic_state::AttitudeCallback,this);
    aerostack_flight_state_pub=nh.advertise<aerostack_msgs::FlightState>("self_localization/flight_state",1,true);
    linear_speed_rad_pub=nh.advertise<geometry_msgs::TwistStamped>("raw_localization/linear_speed", 1, true);
    imu_rad_pub = nh.advertise<sensor_msgs::Imu>("sensor_measurement/imu", 1, true);
    altitude_pub = nh.advertise<geometry_msgs::PointStamped>("sensor_measurement/altitude", 1, true);
    aerostack_flight_state_pub=nh.advertise<aerostack_msgs::FlightState>("self_localization/flight_state", 1, true);
    aerostack_flight_action_sub=nh.subscribe("actuator_command/flight_action", 1,&mavic_state::flight_action_callback,this);
    aircraft_commands_pub=nh.advertise<std_msgs::String>("/Mavic_Commands",1, true);
    command_aerostack.action=aerostack_msgs::FlightActionCommand::UNKNOWN;
    aircraft_state.state=aerostack_msgs::FlightState::LANDED;
}



mavic_state::~mavic_state() {

}

void mavic_state::flight_action_callback(const aerostack_msgs::FlightActionCommandConstPtr& msg)
{
    flag_flight_action=true;
    command_aerostack= *msg;
}

void mavic_state::AttitudeCallback(const geometry_msgs::PointStampedConstPtr& attitude)
{
    attitude_aircraft_world = *attitude;
}
void mavic_state::VelocityCallback(const geometry_msgs::QuaternionStampedConstPtr& velocity)
{
    velocity_aircraft_world = *velocity;
}

void mavic_state::send_state_data_aerostack()
{
    if(!flag_first_data)
    {
        prev_time=ros::Time::now();
        prev_roll=attitude_aircraft_world.point.x*M_PI/180;
        prev_pitch=attitude_aircraft_world.point.y*M_PI/180;
        prev_yaw=attitude_aircraft_world.point.z*M_PI/180;
        flag_first_data=true;
    }
    else
    {
        current_time=ros::Time::now();
        double current_roll=attitude_aircraft_world.point.x*M_PI/180;
        double current_pitch=attitude_aircraft_world.point.y*M_PI/180;
        double current_yaw=attitude_aircraft_world.point.z*M_PI/180;

        double diffTime=(current_time - prev_time).nsec / 1E9;
        double diff_roll=(current_roll-prev_roll)/diffTime;
        double diff_pitch=(current_pitch-prev_pitch)/diffTime;
        double diff_yaw=(current_yaw-prev_yaw)/diffTime;

        double Angular_roll=(diff_roll)-(sin(current_pitch)*diff_yaw);
        double Angular_pitch=(cos(current_roll) * diff_pitch) + (cos(current_pitch)*sin(current_roll) * diff_yaw);
        double Angular_yaw=(-sin(current_roll) * diff_pitch) + (cos(current_roll)*cos(current_pitch) * diff_yaw);

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

        prev_time=current_time;
        prev_roll=current_roll;
        prev_pitch=current_pitch;
        prev_yaw=current_yaw;


        linear_speed_msg.header.stamp=ros::Time::now();
        linear_speed_msg.header.frame_id="world";

        linear_speed_msg.twist.linear.x=velocity_aircraft_world.quaternion.x;
        linear_speed_msg.twist.linear.y=velocity_aircraft_world.quaternion.y;
        linear_speed_msg.twist.linear.z=velocity_aircraft_world.quaternion.z;

        linear_speed_msg.twist.angular.x=Angular_roll;
        linear_speed_msg.twist.angular.y=Angular_pitch;
        linear_speed_msg.twist.angular.z=Angular_yaw;

        linear_speed_rad_pub.publish(linear_speed_msg);

        /*linear_speed_msg.twist.angular.x=Angular_roll*180/M_PI;
        linear_speed_msg.twist.angular.y=Angular_pitch*180/M_PI;
        linear_speed_msg.twist.angular.z=Angular_yaw*180/M_PI;

        linear_speed_deg_pub.publish(linear_speed_msg);*/

        tf2::Quaternion orientation_quaternion;

        imu_msg.header.stamp=ros::Time::now();
        imu_msg.header.frame_id="world";

        imu_msg.linear_acceleration.x=0;
        imu_msg.linear_acceleration.y=0;
        imu_msg.linear_acceleration.z=0;

        imu_msg.angular_velocity.x=Angular_roll;
        imu_msg.angular_velocity.y=Angular_pitch;
        imu_msg.angular_velocity.z=Angular_yaw;

        orientation_quaternion.setRPY(current_roll,current_pitch,current_yaw);

        imu_msg.orientation.x=orientation_quaternion.x();
        imu_msg.orientation.y=orientation_quaternion.y();
        imu_msg.orientation.z=orientation_quaternion.z();
        imu_msg.orientation.w=orientation_quaternion.w();

        imu_rad_pub.publish(imu_msg);
    }
    drone_altitude_msg.header.stamp=ros::Time::now();
    drone_altitude_msg.header.frame_id="world";
    drone_altitude_msg.point.z=velocity_aircraft_world.quaternion.w;
    altitude_pub.publish(drone_altitude_msg);
}

void mavic_state::send_state_aerostack()
{
    switch(command_aerostack.action)
    {
        case aerostack_msgs::FlightActionCommand::TAKE_OFF:
            if (aircraft_state.state == aerostack_msgs::FlightState::LANDED || aircraft_state.state == aerostack_msgs::FlightState::UNKNOWN)
            {
                aircraft_state.state = aerostack_msgs::FlightState::TAKING_OFF;
                time_takeoff = ros::Time::now();
            }
            else{
                if (aircraft_state.state == aerostack_msgs::FlightState::TAKING_OFF)
                {
                    ros::Duration diff = ros::Time::now() - time_takeoff;
                    if (std::abs(drone_altitude_msg.point.z) > 0.2 && diff.toSec() >= 10)
                    {
                        aircraft_state.state = aerostack_msgs::FlightState::FLYING;
                    }
                }
            }
            break;
        case aerostack_msgs::FlightActionCommand::HOVER:
            if(std::abs(drone_altitude_msg.point.z) > 0.1 && std::abs(linear_speed_msg.twist.linear.x) < 0.05 && std::abs(linear_speed_msg.twist.linear.y) < 0.05 && std::abs(linear_speed_msg.twist.linear.z) < 0.05 &&
               std::abs(linear_speed_msg.twist.angular.x) < 0.05 && std::abs(linear_speed_msg.twist.angular.y) < 0.05 && std::abs(linear_speed_msg.twist.angular.z) < 0.05){
                aircraft_state.state = aerostack_msgs::FlightState::HOVERING;
            }
            break;

        case aerostack_msgs::FlightActionCommand::LAND:
            if (aircraft_state.state == aerostack_msgs::FlightState::HOVERING || aircraft_state.state == aerostack_msgs::FlightState::FLYING)
            {
                    aircraft_state.state = aerostack_msgs::FlightState::LANDING;
            }
            else{
                if (aircraft_state.state == aerostack_msgs::FlightState::LANDING){
                    if (std::abs(drone_altitude_msg.point.z) == 0 )
                    {
                        aircraft_state.state = aerostack_msgs::FlightState::LANDED;
                    }
                }
            }
            break;
    }
    aerostack_flight_state_pub.publish(aircraft_state);
}

void mavic_state::send_command_aircraft()
{
    ROS_INFO("%d,%d",command_aerostack.action,aircraft_state.state);
    std_msgs::String message;
    if(command_aerostack.action==aerostack_msgs::FlightActionCommand::TAKE_OFF &&
    aircraft_state.state==aerostack_msgs::FlightState::TAKING_OFF)
    {
        message.data="Takeoff";
    }
    else if(command_aerostack.action==aerostack_msgs::FlightActionCommand::LAND &&
            aircraft_state.state==aerostack_msgs::FlightState::LANDING )
    {
        message.data="Land";
    }
    else
    {
        message.data="";
    }
    aircraft_commands_pub.publish(message);
}

