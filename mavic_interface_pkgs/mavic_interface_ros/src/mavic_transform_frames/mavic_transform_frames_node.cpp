//
// Created by madswamp on 14/04/21.
//
#include "../../include/mavic_interface_ros/mavic_transform_frames/mavic_transform_frames.h"




int main(int argc, char** argv)
{
    ros::init(argc, argv, "mavic_transform_frames_node");
    mavic_transform_frames mavic_frames_class;
    ros::Rate r(30);
    while (ros::ok())
    {

        if(mavic_frames_class.flag_frame_gimball_ready && mavic_frames_class.flag_frame_ready)
        {
            mavic_frames_class.Relative_Yaw_Gimbal_Aircraft();
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}