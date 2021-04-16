//
// Created by andre on 24/03/21.
//
#include "include/mavic_command.h"

using namespace std;

CommandInterface::CommandInterface(){}

CommandInterface::~CommandInterface(){}

//Setup
void CommandInterface::ownSetUp()
{
    cout<<"[ROSNODE] Mavic Setup"<<endl;

    int Vertical_Control_Mode=0,Roll_Pitch_Control_Mode=0,Yaw_Control_Mode=0,Flight_Coordinate_System_Mode=0;
    Functions::flag_failedsetup=0;
    Functions::flag_response_received=0;



    ros::NodeHandle nh("~");

    Functions::nspace = ros::this_node::getNamespace();

    Functions::comm_pub = nh.advertise<std_msgs::String>("/Commands", 1);
    Functions::resp_sub = nh.subscribe("/Responses",1,&CommandInterface::Responses_Callback, this);


    nh.getParam("Vertical_Control_Mode", Vertical_Control_Mode);
    nh.getParam("Roll_Pitch_Control_Mode", Roll_Pitch_Control_Mode);
    nh.getParam("Yaw_Control_Mode", Yaw_Control_Mode);
    nh.getParam("Flight_Coordinate_System_Mode", Flight_Coordinate_System_Mode);
    nh.getParam("flight_state_topic", Functions::state_str);

    //Functions::FlightStatePub = nh.advertise<aerostack_msgs::FlightState>("/"+Functions::nspace+"/"+Functions::state_str,1);

    Functions::received_response=-1;
    std_msgs::String Message;
    std::string s;
    ros::Duration(0.5).sleep();


    //set_up_mavic
    s = std::string("C")+std::string("A")+std::to_string(Vertical_Control_Mode);
    Message.data=s;
    Functions::comm_pub.publish(Message);
    int cnt=0; //number of failed responses
    //re-attempt 10 times (10 seconds)
    while (Functions::received_response!=1){
        if (Functions::received_response !=-1) {
            Functions::received_response=-1;
            if (cnt > 10) {
                Functions::flag_failedsetup=1;
                return;
            }
            ros::Duration(1).sleep();
            Functions::comm_pub.publish(Message);
            cnt++;
        }
        ros::spinOnce();
    }
    Functions::received_response=-1;
    s = std::string("C")+std::string("B")+std::to_string(Roll_Pitch_Control_Mode);
    Message.data=s;
    Functions::comm_pub.publish(Message);
    cnt=0; //number of failed responses
    //re-attempt 10 times (10 seconds)
    while (Functions::received_response!=1){
        if (Functions::received_response !=-1) {
            Functions::received_response=-1;
            if (cnt > 10) {
                Functions::flag_failedsetup=1;
                return;
            }
            ros::Duration(1).sleep();
            Functions::comm_pub.publish(Message);
            cnt++;
        }
        ros::spinOnce();
    }
    Functions::received_response=-1;
    s = std::string("C")+std::string("C")+std::to_string(Yaw_Control_Mode);
    Message.data=s;
    Functions::comm_pub.publish(Message);
    cnt=0; //number of failed responses
    //re-attempt 10 times (10 seconds)
    while (Functions::received_response!=1){
        if (Functions::received_response !=-1) {
            Functions::received_response=-1;
            if (cnt > 10) {
                Functions::flag_failedsetup=1;
                return;
            }
            ros::Duration(1).sleep();
            Functions::comm_pub.publish(Message);
            cnt++;
        }
        ros::spinOnce();
    }
    Functions::received_response=-1;
    s = std::string("C")+std::string("D")+std::to_string(Flight_Coordinate_System_Mode);
    Message.data=s;
    Functions::comm_pub.publish(Message);
    cnt=0; //number of failed responses
    //re-attempt 10 times (10 seconds)
    while (Functions::received_response!=1){
        if (Functions::received_response !=-1) {
            Functions::received_response=-1;
            if (cnt > 10) {
                Functions::flag_failedsetup=1;
                return;
            }
            ros::Duration(1).sleep();
            Functions::comm_pub.publish(Message);
            cnt++;
        }
        ros::spinOnce();
    }
    Functions::received_response=-1;
}

//Start
void CommandInterface::ownStart()
{
    ros::NodeHandle n;
    //Subscribers
    flight_action_sub = n.subscribe("actuator_command/flight_action", 1, &CommandInterface::flightActionCallback, this);
    velpub = n.advertise<geometry_msgs::Quaternion>("/Velocity_Commands", 1);
    keep_hover = true;

    //Force to send this command so the drone doesn't shut itself off
    command_alive.x = 0;
    command_alive.y = 0;
    command_alive.z = 0;
    command_alive.w = 0;
}

//Stop
void CommandInterface::ownStop()
{
    flight_action_sub.shutdown();
    actuator_command_thrust_sub.shutdown();
}

//Reset
bool CommandInterface::resetValues()
{
    return true;
}

//Run
void CommandInterface::ownRun()
{

}

//Responses Callback
void CommandInterface::Responses_Callback(std_msgs::Int32ConstPtr msg)
{
    Functions::received_response=msg->data;
    Functions::flag_response_received=1;
}



void Functions::Start_Take_Off()
{
    std_msgs::String Command;
    std::stringstream ss;
    ss << "B0";
    Command.data=ss.str();
    comm_pub.publish(Command);
}

int Functions::Wait_Command_Response()
{
    if(flag_response_received==1)
    {
        flag_response_received=0;
        if(received_response==1)
        {
            received_response=-1;
            return 1;
        }
        else
        {
            received_response=-1;
            return 0;
        }
    }
    else
    {
        return -1;
    }
}

int Functions::Wait_Flying_Confirmation()
{
    if(flag_response_received==1)
    {
        flag_response_received=0;
        if(received_response==2)
        {
            received_response=-1;
            return 1;
        }
    }
    else
    {
        return 0;
    }
}

void Functions::Do_Take_Off() {
    ros::Time Time_Start;
    int state=0;
    while(state!=-1)
    {
        switch (state)
        {
            case 0: {
                Functions::Start_Take_Off();
                state = 1;
                break;
            }
            case 1: {
                int result = Functions::Wait_Command_Response();
                if (result == 1) {
                    state = -1;
                    Time_Start=ros::Time::now();
                } else if (result == 0) {
                    state = 0;
                }
                break;
            }
        /*    case 2:{
                int result=Wait_Flying_Confirmation();
                if(result==1)
                {
                    Time_Start=ros::Time::now();
                    state=3;
                }
                else
                {
                    if(ros::Time::now().sec - Time_Start.sec > 10)
                    {
                        exit(EXIT_FAILURE);
                    }
                }
            }
            case 3: {
                if (ros::Time::now().sec - Time_Start.sec > 5) {
                    state = -1;
                }
                break;
            }*/
        }
        ros::spinOnce();
    }
/*    Functions::flight_state_msg.header.stamp=ros::Time::now();
    Functions::flight_state_msg.state=aerostack_msgs::FlightState::FLYING;
    Functions::FlightStatePub.publish(Functions::flight_state_msg);*/
}

void Functions::Start_Landing()
{
    std_msgs::String Command;
    std::string  command_buffer;
    command_buffer="B1";
    Command.data=command_buffer;
    comm_pub.publish(Command);
}

int Functions::Wait_Asking_For_Confirmation_Or_Landing()
{

    if(flag_response_received==1)
    {
        flag_response_received=0;
        if(received_response==3)
        {
            received_response=-1;
            return 1;
        }
        else if(received_response==4)
        {
            received_response=-1;
            return 2;
        }
    }
    else
    {
        return 0;
    }
}

void Functions::Send_Landing_Confirmation()
{
    std_msgs::String Command;
    std::string  command_buffer;
    command_buffer="B2";
    Command.data=command_buffer;
    comm_pub.publish(Command);
}

void Functions::Do_Landing()
{
    ros::Time Time_Start;
    int state=4;
    while(state!=-1) {
        switch (state) {
            case 4: {
                Functions::Start_Landing();
                state = 5;
                break;
            }
            case 5: {
                int result = Functions::Wait_Command_Response();
                if (result == 1) {
                    state = 6;
                    Time_Start = ros::Time::now();
                } else if (result == 0) {
                    state = 4;
                }
                break;
            }
            case 6: {
                int result = Functions::Wait_Asking_For_Confirmation_Or_Landing();
                if (result == 1) {
                    state = 10;
                    Time_Start = ros::Time::now();
                } else if (result == 2) {
                    state = 7;
                } else {
                    if (ros::Time::now().sec - Time_Start.sec > 20) {
                        state = 99;
                    }
                }
                break;
            }
            case 7: {
                Functions::Send_Landing_Confirmation();
                state = 8;
                break;
            }
            case 8: {
                int result = Functions::Wait_Command_Response();
                if (result == 1) {
                    state = -1;
                    Time_Start = ros::Time::now();
                } else if (result == 0) {
                    state = 7;
                }
                break;
            }
            /*case 9: {
                if (ros::Time::now().sec - Time_Start.sec > 5) {
                    state = 10;
                    Time_Start = ros::Time::now();
                }
                break;
            }
            case 10: {
                if (ros::Time::now().sec - Time_Start.sec > 5) {
                    state = -1;
                }
                break;
            }*/
        }
        ros::spinOnce();
    }
    /*Functions::flight_state_msg.header.stamp=ros::Time::now();
    Functions::flight_state_msg.state=aerostack_msgs::FlightState::LANDED;
    Functions::FlightStatePub.publish(Functions::flight_state_msg);*/
}



void CommandInterface::flightActionCallback(const aerostack_msgs::FlightActionCommand::ConstPtr& msg)
{
    Functions::flight_action_msg = *msg;
}


int main(int argc,char **argv)
{
    ros::init(argc, argv, "CommandInterface");

    CommandInterface command_interface;

    command_interface.setUp();
    if (Functions::flag_failedsetup==1){
        cout<<"[ROSNODE] Failed Setup"<<endl;
        return 0;
    }

    command_interface.start();
    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        switch(Functions::flight_action_msg.action){
            case 1:
                Functions::Do_Take_Off();
                Functions::flight_action_msg.action=-1;
                break;
            case aerostack_msgs::FlightActionCommand::LAND:
                Functions::Do_Landing();
                Functions::flight_action_msg.action=-1;
                break;
            case aerostack_msgs::FlightActionCommand::HOVER:
                break;
            default:
                break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}