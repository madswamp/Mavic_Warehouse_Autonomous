//
// Created by madswamp on 24/04/21.
//
#include "../../include/mavic_interface_ros/mavic_slam_gtsam/mavic_slam_gtsam.h"

int main(int argc,char **argv)
{
    ros::init(argc, argv, "mavic_slam_gtsam_node");
    mavic_slam_gtsam Mavic_Slam;

    ros::Rate r(30);

    while(ros::ok())
    {
        if(Mavic_Slam.flag_new_odom)
        {
            Mavic_Slam.add_odometry_to_graph();
            Mavic_Slam.flag_new_odom=false;
        }

        if(Mavic_Slam.flag_new_update)
        {
            Mavic_Slam.add_landmark_to_graph();
            Mavic_Slam.flag_new_update=false;
        }
        if(Mavic_Slam.counter_odometry_factors==3)
        {
            Mavic_Slam.optimize_factor_graph();
            Mavic_Slam.counter_odometry_factors=0;
        }

        ros::spinOnce();
        r.sleep();
    }


    return 0;
}
