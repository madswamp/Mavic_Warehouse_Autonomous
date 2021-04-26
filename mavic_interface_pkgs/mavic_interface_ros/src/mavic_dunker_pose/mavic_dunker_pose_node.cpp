//
// Created by madswamp on 26/04/21.
//

#include "../../include/mavic_interface_ros/mavic_dunker_pose/mavic_dunker_pose.h"


int main(int argc,char **argv) {
    ros::init(argc, argv, "mavic_dunker_pose_node");
    mavic_dunker_pose dunker_pose;

    ros::Rate r(30);
    while (ros::ok())
    {
        if(dunker_pose.flag_dunker_detected)
        {
            dunker_pose.send_dunker_pose();
            dunker_pose.flag_dunker_detected=false;
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}