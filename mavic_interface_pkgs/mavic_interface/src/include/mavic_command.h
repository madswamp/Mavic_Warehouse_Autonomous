//
// Created by andre on 24/03/21.
//

#ifndef SRC_MAVIC_COMMAND_H
#define SRC_MAVIC_COMMAND_H

#endif //SRC_MAVIC_COMMAND_H

#include <iostream>

//// ROS  ///////
#include "ros/ros.h"
#include "cvg_string_conversions.h"
#include <robot_process.h>
#include "math.h"

#include <tf/transform_broadcaster.h>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "aerostack_msgs/FlightActionCommand.h"
#include "aerostack_msgs/FlightState.h"
#include "mav_msgs/RollPitchYawrateThrust.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <std_msgs/String.h>


namespace Functions{
    void Do_Take_Off();
    void Do_Landing();
    int Wait_Command_Response();
    int Wait_Flying_Confirmation();
    void Start_Take_Off();
    void Start_Landing();
    int Wait_Asking_For_Confirmation_Or_Landing();
    void Send_Landing_Confirmation();

    int flag_response_received,flag_failedsetup;
    aerostack_msgs::FlightActionCommand flight_action_msg;
    aerostack_msgs::FlightState flight_state_msg;
    int received_response;
    ros::Time last_time;
    ros::Publisher comm_pub,FlightStatePub;
    ros::Subscriber resp_sub;

    std::string nspace;
    std::string state_str;
}


class CommandInterface : public RobotProcess
{
    //Constructors and destructors
    public:
        CommandInterface();
        ~CommandInterface();

    protected:
        bool resetValues();
    private: /*RobotProcess*/
        void ownSetUp();
        void ownStart();
        void ownStop();
        void ownRun();

        std_msgs::String command_msg;

        geometry_msgs::Quaternion command_alive,command_vel;

        bool keep_hover;

        //Subscribers
        ros::Subscriber flight_action_sub;

        void flightActionCallback(const aerostack_msgs::FlightActionCommand::ConstPtr& msg);

        ros::Subscriber actuator_command_thrust_sub;

        void referenceSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

        void Responses_Callback(std_msgs::Int32ConstPtr msg);

        ros::Publisher velpub;

};