//
// Created by madswamp on 27/04/21.
//

#include "../../../include/mavic_interface_ros/mavic_interface_aerostack/mavic_optimized_localization/mavic_optimized_localization.h"


int main(int argc,char **argv)
{

    ros::init(argc, argv, "mavic_optimized_localization");
    mavic_optimized_localization mavic_localization;
    ros::Rate r(30);
    while(ros::ok())
    {
        if(mavic_localization.flag_new_pose)
        {
            mavic_localization.pub_localization_to_aerostack();
            mavic_localization.pub_velocity_to_aerostack();
            mavic_localization.flag_new_pose=false;
        }
        r.sleep();
        ros::spinOnce();
    }

    return 0;

}