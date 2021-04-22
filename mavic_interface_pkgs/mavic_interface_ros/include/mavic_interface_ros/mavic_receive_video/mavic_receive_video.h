//
// Created by madswamp on 22/04/21.
//

#ifndef MAVIC_INTERFACE_MAVIC_RECEIVE_VIDEO_H
#define MAVIC_INTERFACE_MAVIC_RECEIVE_VIDEO_H

#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv.hpp>

//const string calibration_file_path = "file:////home/madswamp/Desktop/LOGS/Imagens_Calibrar/calibration.yaml";
//const string camera_name = "Video_Feed_Raw";

class mavic_receive_video
{
private:
    ros::NodeHandle nh;
    ros::Subscriber video_compressed_sub;
    ros::Publisher  video_raw_pub,camera_info_pub;

    void video_compressed_callback(const sensor_msgs::CompressedImageConstPtr& msg);


    sensor_msgs::CompressedImage image_compressed;



public:
    mavic_receive_video();
    ~mavic_receive_video();

    void uncompress_frame_and_pub();
    bool flag_new_frame=false;
};

#endif //MAVIC_INTERFACE_MAVIC_RECEIVE_VIDEO_H
