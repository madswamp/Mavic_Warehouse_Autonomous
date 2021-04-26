//
// Created by madswamp on 26/04/21.
//
#include "../../include/mavic_interface_ros/mavic_optimized_frames/mavic_optimized_frames.h"


int main(int argc,char **argv)
{
    ros::init(argc, argv, "mavic_optimized_frames_node");
    mavic_optimized_frames mavic_optimized;
    ros::Rate r(30);

    while (ros::ok())
    {

        if(mavic_optimized.flag_optimization_started)
        {
            mavic_optimized.broadcast_optimized_transforms();
        }

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}