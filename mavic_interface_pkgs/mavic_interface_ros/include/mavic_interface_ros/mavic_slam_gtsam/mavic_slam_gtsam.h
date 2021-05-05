//
// Created by madswamp on 24/04/21.
//

#ifndef MAVIC_INTERFACE_MAVIC_SLAM_GTSAM_H
#define MAVIC_INTERFACE_MAVIC_SLAM_GTSAM_H

#include <ros/ros.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Marginals.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TwistStamped.h>


using namespace gtsam;

class mavic_slam_gtsam
{
private:

    ros::Subscriber tag_array_sub,aircraft_raw_pose_sub,aircraft_sensor_attitude,raw_linear_speed_sub;
    ros::Publisher optimized_pose_pub,optimized_pose_with_covariance_pub;
    ros::NodeHandle nh;
    void tag_array_callback(const std_msgs::Int32MultiArrayConstPtr& msg);
    void aircraft_raw_pose_callback(const geometry_msgs::PoseStampedConstPtr& msg);
    std_msgs::Int32MultiArray tag_array;
    geometry_msgs::PoseStamped pose_received;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener;


    int initial_tag=-1;

    NonlinearFactorGraph factor_graph;
    Values initialEstimates;

    ISAM2Params parameters;
    ISAM2 isam;

    noiseModel::Diagonal::shared_ptr Odometry_noise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01,0.01,0.01,0.80,0.80,0.80).finished());

    noiseModel::Diagonal::shared_ptr Landmark_initial_noise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.05,0.05,0.05,0.20,0.20,0.20).finished());

    noiseModel::Diagonal::shared_ptr Landmark_observation_noise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.17,0.17,0.17,0.20,0.20,0.20).finished());


    int odometry_factors_count=0;

    Pose3 Prev_Pose;

    int array_landmarks_seen[100]={0};

    int id[20];

    std::string aircraft_name;

    Values optimized_estimates;

    double * Covariance_Pose;

    bool flag_first_tag_seen=false;

    void sensor_attitude_callback(const geometry_msgs::PointStampedConstPtr& msg);

    geometry_msgs::PointStamped attitude_msg;

    void raw_veloicty_callback(const geometry_msgs::TwistStampedConstPtr& msg);

    geometry_msgs::TwistStamped velocity_msg;



public:

    mavic_slam_gtsam();
    ~mavic_slam_gtsam();
    bool flag_new_update=false,flag_new_odom=false;

    void add_odometry_to_graph();
    void add_landmark_to_graph();

    void optimize_factor_graph();
    int counter_odometry_factors=0;
    bool flag_new_velocity=false;

    void add_velocity_to_optimized_pose();

};


#endif //MAVIC_INTERFACE_MAVIC_SLAM_GTSAM_H
