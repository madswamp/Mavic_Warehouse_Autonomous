//
// Created by madswamp on 22/04/21.
//

#include "../../include/mavic_interface_ros/mavic_receive_video/mavic_receive_video.h"

using namespace cv;

mavic_receive_video::mavic_receive_video()
{
    video_compressed_sub=nh.subscribe("Video_Feed",1,&mavic_receive_video::video_compressed_callback,this);
    video_raw_pub=nh.advertise<sensor_msgs::Image>("Video_Feed_Raw/image_raw",1,true);
    camera_info_pub=nh.advertise<sensor_msgs::CameraInfo>("Video_Feed_Raw/camera_info",1,true);
}

mavic_receive_video::~mavic_receive_video()
{

}

void mavic_receive_video::video_compressed_callback(const sensor_msgs::CompressedImageConstPtr& msg)
{
    image_compressed = *msg;
    flag_new_frame = true;
}

void mavic_receive_video::uncompress_frame_and_pub()
{
    camera_info_manager::CameraInfoManager caminfo(nh, camera_name, calibration_file_path);

    sensor_msgs::Image raw_video_msg;

    sensor_msgs::CameraInfo camera_info_msg;

    Mat image = cv::imdecode(cv::Mat(image_compressed.data),IMREAD_ANYCOLOR);

    std_msgs::Header header=image_compressed.header;

    cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image);

    img_bridge.toImageMsg(raw_video_msg);

   camera_info_msg.header = header;

    camera_info_msg=caminfo.getCameraInfo();

    camera_info_pub.publish(camera_info_msg);

    video_raw_pub.publish(raw_video_msg);
}
