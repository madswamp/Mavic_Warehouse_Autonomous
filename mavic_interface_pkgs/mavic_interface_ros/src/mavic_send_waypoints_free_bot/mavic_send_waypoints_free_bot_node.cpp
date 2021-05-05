//
// Created by madswamp on 30/04/21.
//

#include "../../mavic_interface_ros/mavic_send_waypoints_free_bot/mavic_send_waypoints_free_bot.h"

int main(int argc,char **argv)
{
    ros::init(argc, argv, "mavic_optimized_localization");
    mavic_send_waypoints_free_bot mavic_send_tag;
    ros::Rate r(30);
    while(ros::ok())
    {
        if(mavic_send_tag.flag_new_tags)
        {
            mavic_send_tag.send_tags_free_bots();
            mavic_send_tag.flag_new_tags=false;
        }
        if(mavic_send_tag.flag_new_pose)
        {
            mavic_send_tag.send_drone_optimized_pose();
            mavic_send_tag.flag_new_pose=false;
        }
        ros::spinOnce();
        r.sleep();
    }
}
