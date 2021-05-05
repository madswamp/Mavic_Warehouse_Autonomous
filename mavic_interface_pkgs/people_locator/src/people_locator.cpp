//
// Created by andre on 30/04/21.
//

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ppl_lct{
private:
    ros::Subscriber pray_sub;
    ros::Publisher pub_pose;
    tf2_ros::Buffer buf;
    tf2_ros::TransformListener listener;
    geometry_msgs::PoseStamped pose_pub;
public:
    ppl_lct(ros::NodeHandle *nh)
    : listener(buf)
    {
        pray_sub = nh->subscribe("/people_ray",5,&ppl_lct::PPL_LCT_CB,this);
        pub_pose = nh->advertise<geometry_msgs::PoseStamped>("/person_here",1,true);
    }

    ~ppl_lct()
    {

    }


    geometry_msgs::PointStamped image_ray;
    geometry_msgs::PointStamped image_ray_transformed;
    bool flag_new_image=false;
    void transform_to_map();

    void PPL_LCT_CB(const geometry_msgs::Point& msg){
        image_ray.header.stamp=ros::Time::now();
        image_ray.header.frame_id="drone0_optimized_image";
        image_ray.point.x = msg.x;
        image_ray.point.y = msg.y;
        image_ray.point.z = msg.z;
    };

};

void ppl_lct::transform_to_map()
{
    try
    {
        geometry_msgs::TransformStamped transform;
        transform=buf.lookupTransform("map","drone0_optimized_image",ros::Time::now());
        tf2::doTransform(image_ray,image_ray_transformed,transform);
        geometry_msgs::PoseStamped rviz_point;
        rviz_point.header.frame_id="map";
        rviz_point.header.stamp=ros::Time::now();
        rviz_point.pose.position.x=image_ray_transformed.point.x;
        rviz_point.pose.position.y=image_ray_transformed.point.y;
        rviz_point.pose.position.z=image_ray_transformed.point.z;
        rviz_point.pose.orientation.x=0;
        rviz_point.pose.orientation.y=0;
        rviz_point.pose.orientation.z=0;
        rviz_point.pose.orientation.w=0;
        pub_pose.publish(rviz_point);
    }
    catch (tf2::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
}



int main( int argc, char** argv){
    ros::init(argc, argv, "people_locator");
    ros::NodeHandle n;

    ppl_lct image_people(&n);


    ros::Rate r(100);
    try
    {
        while(ros::ok())
        {
            if(image_people.flag_new_image)
            {
                image_people.transform_to_map();
                image_people.flag_new_image=false;
            }

            ros::spinOnce();
            r.sleep();
        }
    }
    catch (std::exception &ex)
    {
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
    }
}

