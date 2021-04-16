//
// Created by madswamp on 14/04/21.
//
#include "../include/mavic_transform_frames/mavic_transform_frames.h"




int main(int argc, char** argv)
{
    ros::init(argc, argv, "mavic_transform_frames_node");
    mavic_transform_frames mavic_frames_class;
    //ros::Rate r(50);
    ros::spin();
    return 0;
}