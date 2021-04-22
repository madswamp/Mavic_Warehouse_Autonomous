//
// Created by madswamp on 19/04/21.
//
#include "../../../include/mavic_interface_ros/mavic_interface_aerostack/mavic_state/mavic_state.h"

int main(int argc,char **argv)
{
    ros::init(argc, argv, "mavic_state_aerostack");
    mavic_state mavic_aerostack_state;
    ros::Rate r(30);

    while(ros::ok())
    {
        mavic_aerostack_state.send_state_data_aerostack();
        mavic_aerostack_state.send_state_aerostack();
        if(mavic_aerostack_state.flag_flight_action)
        {
            mavic_aerostack_state.send_command_aircraft();
            mavic_aerostack_state.flag_flight_action = false;
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
