//
// Created by andre on 30/04/21.
//

//
// Created by andre on 28/04/21.
//

#include <ros/ros.h>
#include <opencv_apps/RectArrayStamped.h>
#include <opencv_apps/Rect.h>
#include <image_geometry/pinhole_camera_model.h>
#include <geometry_msgs/Point.h>

//#include <opencv>


using namespace std;
using namespace cv;

class PPLray{
private:
    ros::Subscriber people_sub;
    ros::Publisher pray_pub;

public:

    PPLray(ros::NodeHandle *nh){
        people_sub = nh->subscribe("/people_detect/found",5,&PPLray::Rcv_people_CB,this);
        pray_pub = nh->advertise<geometry_msgs::Point>("/people_ray",5);
    }

    sensor_msgs::CameraInfo::ConstPtr cam_info_ptr;

    void Rcv_people_CB (const opencv_apps::RectArrayStamped &msg){

        image_geometry::PinholeCameraModel model;
        model.fromCameraInfo(cam_info_ptr);

        if (msg.rects.size()>0)
        {
            opencv_apps::Rect rect0 = msg.rects[0];
            cv::Point2d pixel_point(rect0.x, rect0.y);
            cv::Point3d xyz = model.projectPixelTo3dRay(pixel_point);
            geometry_msgs::Point point;
            point.x = xyz.x;
            point.y = xyz.y;
            point.z = xyz.z;

            pray_pub.publish(point);
        }
        else{
            return;
        }
    };
};


int main(int argc,char **argv)
{

    ros::init(argc, argv, "people_ray");

    ros::NodeHandle nh;

    PPLray pray = PPLray(&nh);


    pray.cam_info_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/drone0/VideoFeedRaw/camera_info",nh, ros::Duration(10));
    //cout << *pray.cam_info_ptr;

    ros::Rate r(100);
    try
    {
        while(ros::ok())
        {
            ros::spinOnce();
            r.sleep();
        }
    }
    catch (std::exception &ex)
    {
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
    }
}