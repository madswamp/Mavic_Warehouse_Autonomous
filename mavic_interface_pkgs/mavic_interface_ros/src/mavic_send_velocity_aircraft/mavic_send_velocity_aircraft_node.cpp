//
// Created by madswamp on 28/04/21.
//
#include "../../include/mavic_interface_ros/mavic_send_velocity_aircraft/mavic_send_velocity_aircraft.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mavic_send_velocity_aircraft_node");

    mavic_send_velocity_aircraft send_velocity_aircraft;
    ros::Rate r(30);

    while(ros::ok())
    {

        if(send_velocity_aircraft.flag_first_velocity)
        {
            send_velocity_aircraft.activate_joystick_mode(true);
            send_velocity_aircraft.flag_first_velocity=-1;
        }
        if(send_velocity_aircraft.flag_new_velocity)
        {
            send_velocity_aircraft.send_velocity();
            send_velocity_aircraft.flag_new_velocity=false;
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}