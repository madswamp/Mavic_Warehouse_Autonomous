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


using namespace gtsam;

class mavic_slam_gtsam
{
private:

    ros::Subscriber tag_array_sub,aircraft_raw_pose_sub;
    ros::Publisher optimized_pose_pub;
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


    noiseModel::Diagonal::shared_ptr Odometry_noise = noiseModel::Diagonal::Sigmas(
            (Vector(6) << 0,0,0.005,0.03,0.03,0.03).finished());

    noiseModel::Diagonal::shared_ptr Landmark_initial_noise = noiseModel::Diagonal::Sigmas(
            (Vector(6) << 0,0,0,0,0,0).finished());

    noiseModel::Diagonal::shared_ptr Landmark_observation_noise = noiseModel::Diagonal::Sigmas(
            (Vector(6) << 100,100,0.0872665,0.05,0.05,0.05).finished());

    int odometry_factors_count=0;

    Pose3 Prev_Pose;

    int array_landmarks_seen[100]={0};

    int id[20];

    std::string aircraft_name;

    Values optimized_estimates;

    double * Covariance_Pose;

public:

    mavic_slam_gtsam();
    ~mavic_slam_gtsam();
    bool flag_new_update=false,flag_new_odom=false;

    void add_odometry_to_graph();
    void add_landmark_to_graph();

    void optimize_factor_graph();
    int counter_odometry_factors=0;

};


#endif //MAVIC_INTERFACE_MAVIC_SLAM_GTSAM_H
