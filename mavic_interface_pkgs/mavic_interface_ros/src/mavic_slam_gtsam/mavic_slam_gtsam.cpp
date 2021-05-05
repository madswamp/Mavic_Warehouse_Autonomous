//
// Created by madswamp on 24/04/21.
//

#include "../../include/mavic_interface_ros/mavic_slam_gtsam/mavic_slam_gtsam.h"

using namespace gtsam;

mavic_slam_gtsam::mavic_slam_gtsam()
:   listener(tf_buffer), isam(parameters)
{
    tag_array_sub=nh.subscribe("tag_detection_array",1,
                               &mavic_slam_gtsam::tag_array_callback,this);

    aircraft_raw_pose_sub=nh.subscribe("raw_localization/raw_pose",1,
                                       &mavic_slam_gtsam::aircraft_raw_pose_callback,this);

    optimized_pose_pub=nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("optimized_pose",
                                                                              1,true);

    optimized_pose_with_covariance_pub=nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("optimized_pose_with_covariance",
                                                                                              1,true);


    //aircraft_sensor_attitude=nh.subscribe("Attitude_RPY",1,&mavic_slam_gtsam::sensor_attitude_callback,this);
    //raw_linear_speed_sub=nh.subscribe("raw_localization/linear_speed",1,&mavic_slam_gtsam,this


    ros::param::get("~init_tag",initial_tag);


    parameters.relinearizeThreshold = 0.01;

    parameters.relinearizeSkip = 1;

    /*parameters.cacheLinearizedFactors=true;
    parameters.enablePartialRelinearizationCheck=true;
    parameters.enableDetailedResults=false;
    parameters.enableRelinearization=true;
    parameters.evaluateNonlinearError=false;
    parameters.factorization=ISAM2Params::CHOLESKY;
    parameters.findUnusedFactorSlots=false;
    parameters.keyFormatter=DefaultKeyFormatter;
    parameters.optimizationParams=ISAM2GaussNewtonParams();
    parameters.relinearizeThreshold=0.1;
    parameters.relinearizeSkip=10;*/

    aircraft_name = nh.getNamespace();
}


mavic_slam_gtsam::~mavic_slam_gtsam()
{


}

void mavic_slam_gtsam::raw_veloicty_callback(const geometry_msgs::TwistStampedConstPtr &msg)
{
    velocity_msg = *msg;
    flag_new_velocity=true;
}

void mavic_slam_gtsam::sensor_attitude_callback(const geometry_msgs::PointStampedConstPtr& msg)
{
    attitude_msg= *msg;
}

void mavic_slam_gtsam::tag_array_callback(const std_msgs::Int32MultiArrayConstPtr &msg)
{
    tag_array= *msg;
    flag_new_update=true;
}

void mavic_slam_gtsam::aircraft_raw_pose_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    pose_received = *msg;
    flag_new_odom=true;
}

void mavic_slam_gtsam::add_odometry_to_graph()
{

    Pose3 Pose_Odometry=Pose3(Rot3(pose_received.pose.orientation.w,
                                   pose_received.pose.orientation.x,
                                   pose_received.pose.orientation.y,
                                   pose_received.pose.orientation.z),
                              Point3(pose_received.pose.position.x,
                                     pose_received.pose.position.y,
                                     pose_received.pose.position.z));

    if(odometry_factors_count==0)
    {
        factor_graph.emplace_shared<PriorFactor<Pose3>>(symbol('x', 0),
                                                        Pose_Odometry,Odometry_noise);

    }
    else
    {
        factor_graph.emplace_shared<BetweenFactor<Pose3>>(symbol('x', odometry_factors_count-1),
                symbol('x', odometry_factors_count),
                Prev_Pose.between(Pose_Odometry), Odometry_noise);

    }
    initialEstimates.insert(symbol('x', odometry_factors_count), Pose_Odometry);


    Prev_Pose=Pose_Odometry;
    odometry_factors_count++;
    counter_odometry_factors++;
}

void mavic_slam_gtsam::add_landmark_to_graph()
{
    for(int i=0;i<tag_array.data.size();i++)
    {
        bool flag_tf_error=true;
        geometry_msgs::TransformStamped transformStamped;
        id[i]=tag_array.data.at(i);
        if(id[i]!=18)
        {
            while(flag_tf_error)
            {
                try
                {
                    transformStamped = tf_buffer.lookupTransform("map_ned",
                                                                 "tag_"+to_string(id[i]),
                                                                 ros::Time(0));
                    flag_tf_error=false;
                }
                catch(tf2::TransformException &ex)
                {
                    ros::Duration(0.01).sleep();
                    continue;
                }
            }
            Pose3 Pose_Landmark = Pose3(Rot3(transformStamped.transform.rotation.w,
                                             0,
                                             0,
                                             transformStamped.transform.rotation.z),
                                        Point3(transformStamped.transform.translation.x,
                                               transformStamped.transform.translation.y,
                                               transformStamped.transform.translation.z));


            if(array_landmarks_seen[id[i]]==0)
            {
                if(id[i]==initial_tag)
                {
                    Pose3 Pose_Landmark_Init = Pose3(Rot3(1,0,0,0),Point3(0,0,0));
                    factor_graph.emplace_shared<PriorFactor<Pose3>>(Symbol('l', id[i]),
                            Pose_Landmark_Init,Landmark_initial_noise);
                    flag_first_tag_seen=true;
                }
                /*else if(id[i]==3)
                {
                    Pose3 Pose_Landmark_Init = Pose3(Rot3(0.9997,0.0029,0.0128,-0.0223),Point3(-1.4666,0.6248,0.0120));
                    factor_graph.emplace_shared<PriorFactor<Pose3>>(Symbol('l', id[i]),
                                                                    Pose_Landmark_Init,Landmark_initial_noise);
                }
                else if(id[i]==4)
                {
                    Pose3 Pose_Landmark_Init = Pose3(Rot3(0.9309,0.0097,0.0023,-0.3650),Point3(0.1827,-5.0943,-0.0068)  );
                    factor_graph.emplace_shared<PriorFactor<Pose3>>(Symbol('l', id[i]),
                                                                    Pose_Landmark_Init,Landmark_initial_noise);
                }
                else if(id[i]==8)
                {
                    Pose3 Pose_Landmark_Init = Pose3(Rot3(0.9999,0.0091,0.0055,0.0105),Point3(1.6051,6.3551,0.0039));
                    factor_graph.emplace_shared<PriorFactor<Pose3>>(Symbol('l', id[i]),
                                                                    Pose_Landmark_Init,Landmark_initial_noise);
                }
                else if(id[i]==5)
                {
                    Pose3 Pose_Landmark_Init = Pose3(Rot3(0.2449,-0.0328,0.0006,-0.9690),Point3(0.6425,2.6703,0.0036));
                    factor_graph.emplace_shared<PriorFactor<Pose3>>(Symbol('l', id[i]),
                                                                    Pose_Landmark_Init,Landmark_initial_noise);
                }
                else if(id[i]==9)
                {
                    Pose3 Pose_Landmark_Init = Pose3(Rot3(0.7130,-0.0027,0.0085,-0.7011),Point3(1.3345,9.4486,-0.0057));
                    factor_graph.emplace_shared<PriorFactor<Pose3>>(Symbol('l', id[i]),
                                                                    Pose_Landmark_Init,Landmark_initial_noise);
                }*/
                initialEstimates.insert(symbol('l', id[i]), Pose_Landmark);
                array_landmarks_seen[id[i]] = 1;
            }
            factor_graph.emplace_shared<BetweenFactor<Pose3>>(symbol('x', odometry_factors_count-1),
                    symbol('l',id[i]),Prev_Pose.between(Pose_Landmark), Landmark_observation_noise);
        }

    }

}

void mavic_slam_gtsam::optimize_factor_graph()
{
    try
    {
            double a=ros::Time::now().toNSec();
            isam.update(factor_graph,initialEstimates);
            optimized_estimates.insert(symbol('x', odometry_factors_count-1),
                                       isam.calculateEstimate(symbol('x', odometry_factors_count-1)));

            Pose3 optimized_pose;
            optimized_pose=optimized_estimates.at<gtsam::Pose3>(symbol('x', odometry_factors_count-1));

            Matrix3 Pose_Rotation = optimized_pose.rotation().matrix();
            Quaternion Optimized_Pose_Orientation(Pose_Rotation);

            static tf2_ros::TransformBroadcaster tf;
            geometry_msgs::TransformStamped transformStamped;

            transformStamped.header.stamp=ros::Time::now();
            transformStamped.header.frame_id="map_ned";
            transformStamped.child_frame_id=aircraft_name+"_optimized";
            transformStamped.transform.translation.x=optimized_pose.x();
            transformStamped.transform.translation.y=optimized_pose.y();
            transformStamped.transform.translation.z=optimized_pose.z();
            transformStamped.transform.rotation.x=Optimized_Pose_Orientation.x();
            transformStamped.transform.rotation.y=Optimized_Pose_Orientation.y();
            transformStamped.transform.rotation.z=Optimized_Pose_Orientation.z();
            transformStamped.transform.rotation.w=Optimized_Pose_Orientation.w();
            tf.sendTransform(transformStamped);

            //Covariance_Pose=isam.marginalCovariance(Symbol('x', odometry_factors_count-1)).data();


            geometry_msgs::PoseWithCovarianceStamped optimized_pose_msg;

            optimized_pose_msg.header.frame_id="map_ned";
            optimized_pose_msg.header.stamp=ros::Time::now();

            optimized_pose_msg.pose.pose.position.x=optimized_pose.x();
            optimized_pose_msg.pose.pose.position.y=optimized_pose.y();
            optimized_pose_msg.pose.pose.position.z=optimized_pose.z();
            optimized_pose_msg.pose.pose.orientation.x=Optimized_Pose_Orientation.x();
            optimized_pose_msg.pose.pose.orientation.y=Optimized_Pose_Orientation.y();
            optimized_pose_msg.pose.pose.orientation.z=Optimized_Pose_Orientation.z();
            optimized_pose_msg.pose.pose.orientation.w=Optimized_Pose_Orientation.w();


            /*optimized_pose_msg.pose.covariance.elems[0]=Covariance_Pose[21];
            optimized_pose_msg.pose.covariance.elems[7]=Covariance_Pose[28];
            optimized_pose_msg.pose.covariance.elems[14]=Covariance_Pose[35];
            optimized_pose_msg.pose.covariance.elems[21]=Covariance_Pose[0];
            optimized_pose_msg.pose.covariance.elems[28]=Covariance_Pose[7];
            optimized_pose_msg.pose.covariance.elems[35]=Covariance_Pose[14];*/

            optimized_pose_msg.pose.covariance.elems[0]=0.000001;
            optimized_pose_msg.pose.covariance.elems[7]=0.000001;
            optimized_pose_msg.pose.covariance.elems[14]=0.000001;
            optimized_pose_msg.pose.covariance.elems[21]=0.000001;
            optimized_pose_msg.pose.covariance.elems[28]=0.000001;
            optimized_pose_msg.pose.covariance.elems[35]=0.000001;
            optimized_pose_pub.publish(optimized_pose_msg);

            /*optimized_pose_msg.pose.covariance.elems[0]=Covariance_Pose[21];
            optimized_pose_msg.pose.covariance.elems[7]=Covariance_Pose[28];
            optimized_pose_msg.pose.covariance.elems[14]=Covariance_Pose[35];
            optimized_pose_msg.pose.covariance.elems[21]=Covariance_Pose[0];
            optimized_pose_msg.pose.covariance.elems[28]=Covariance_Pose[7];
            optimized_pose_msg.pose.covariance.elems[35]=Covariance_Pose[14];*/


            //optimized_pose_with_covariance_pub.publish(optimized_pose_msg);
            double b=ros::Time::now().toNSec();
            initialEstimates.clear();
            factor_graph.erase(factor_graph.begin(),factor_graph.end());

    }
    catch(gtsam::IndeterminantLinearSystemException &ex )
    {
        ROS_ERROR("Error occured: %s ", ex.what());

    }
}

void mavic_slam_gtsam::add_velocity_to_optimized_pose()
{


}