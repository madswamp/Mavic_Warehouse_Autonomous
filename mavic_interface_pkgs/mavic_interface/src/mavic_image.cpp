//
// Created by andre on 14/04/21.
//

#include "include/mavic_image.h"



ImageInterface::ImageInterface(){}

ImageInterface::~ImageInterface(){}

//Setup
void ImageInterface::ownSetUp()
{
    cout<<"[ROSNODE] Mavic Image Setup"<<endl;

    camurl = "file:////home/madswamp/Desktop/LOGS/Imagens_Calibrar/calibration.yaml";
    camera_name = "Video_Feed_Raw";

}

//Start
void ImageInterface::ownStart()
{
    pub_img = n.advertise<sensor_msgs::Image>("/sensor_measurements/camera",1);
    pub_info = n.advertise<sensor_msgs::CameraInfo>("/sensor_measurements/camera_info",1);

    img_sub = n.subscribe("/Video_Feed", 1, &ImageInterface::Rcv_Img_Callback, this);
}

//Stop
void ImageInterface::ownStop()
{
    img_sub.shutdown();
    pub_img.shutdown();
    pub_info.shutdown();
}

//Reset
bool ImageInterface::resetValues()
{
    return true;
}

//Run
void ImageInterface::ownRun()
{
}

//Image Callback
void ImageInterface::Rcv_Img_Callback(const sensor_msgs::CompressedImageConstPtr &msg) {
    caminfo(n, camera_name, camurl);
    sensor_msgs::Image raw,raw_gray;
    sensor_msgs::CameraInfo info;
    cv::Mat image = cv::imdecode(cv::Mat(msg->data),IMREAD_COLOR);
    cv_bridge::CvImage img_bridge;
    std_msgs::Header header; // empty header
    header = msg->header;
    header.stamp = ros::Time::now(); // time

    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image);
    img_bridge.toImageMsg(raw);
    info.header = header;
    info=caminfo.getCameraInfo();
    pub_info.publish(info);
    pub_img.publish(raw);
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "CameraInterface");

    cout<<"[ROSNODE] Starting CameraInterface"<<endl;

    ImageInterface image_interface;

    image_interface.setUp();

    image_interface.start();

    try
    {
        ros::spin();
    }
    catch (std::exception &ex)
    {
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
    }
}

