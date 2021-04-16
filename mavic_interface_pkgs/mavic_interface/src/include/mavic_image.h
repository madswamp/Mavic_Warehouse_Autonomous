//
// Created by andre on 14/04/21.
//

#ifndef MAVIC_INTERFACE_MAVIC_IMAGE_H
#define MAVIC_INTERFACE_MAVIC_IMAGE_H

#endif //MAVIC_INTERFACE_MAVIC_IMAGE_H

#include <iostream>

//// ROS  ///////
#include "ros/ros.h"
#include <robot_process.h>

#include "sensor_msgs/CompressedImage.h"

#include "sensor_msgs/Image.h"

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>

#include "opencv2/highgui.hpp"

#include "opencv2/videoio.hpp"

#include <image_transport/image_transport.h>

#include <camera_info_manager/camera_info_manager.h>

using namespace cv;
using namespace std;


class ImageInterface : public RobotProcess
{
    //Constructors and destructors
public:
    ImageInterface();
    ~ImageInterface();

private: /*RobotProcess*/
    void ownSetUp();
    void ownStart();
    void ownStop();
    void ownRun();
    bool resetValues();

    ros::NodeHandle n;

    ros::Publisher pub_img;
    ros::Publisher pub_info;
    ros::Subscriber img_sub;

    string camurl;
    string camera_name;

    camera_info_manager::CameraInfoManager caminfo;

    void Rcv_Img_Callback(const sensor_msgs::CompressedImageConstPtr& msg);
};