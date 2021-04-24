//
// Created by madswamp on 22/04/21.
//
#include "../../include/mavic_interface_ros/mavic_receive_video/mavic_receive_video.h"

int main(int argc,char **argv)
{
    ros::init(argc, argv, "Mavic_Receive_Video");
    mavic_receive_video mavic_video;
    ros::Rate r(20);
    while (ros::ok())
    {
        if(mavic_video.flag_new_frame)
        {
            mavic_video.uncompress_frame_and_pub();
            mavic_video.flag_new_frame=false;
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}