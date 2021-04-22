#include "../../include/mavic_interface_ros/mavic_raw_localization/mavic_raw_localization.h"


int main(int argc,char **argv)
{
    ros::init(argc, argv, "Mavic_Raw_Localization_Node");
    mavic_raw_localization mavic_raw;
    ros::Rate r(30);
    while (ros::ok())
    {
        while(!mavic_raw.flag_velocity && !mavic_raw.flag_attitude && !mavic_raw.flag_gimbal)
        {
            ros::spinOnce();
        }
        mavic_raw.pub_pose_broadcast_tf();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

