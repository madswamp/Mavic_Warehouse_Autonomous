//
// Created by andre on 14/04/21.
//

#include "include/mavic_state.h"

StateInterface::StateInterface() {
}

StateInterface::~StateInterface() {
}

void StateInterface::ownSetUp() {
    cout<<"[ROSNODE] Mavic State Setup"<<endl;
    flag_firsttime = true;
    imudata.dR = 0;
    imudata.dP = 0;
    imudata.dY = 0;
    imudata.R = 0;
    imudata.P = 0;
    imudata.Y = 0;
}

void StateInterface::ownStart() {
    flight_state_pub = n.advertise<aerostack_msgs::FlightState>("self_localization/flight_state", 1, true);

    flight_action_sub = n.subscribe("actuator_command/flight_action", 1, &StateInterface::flightActionCallback, this);
    velocity_sub = n.subscribe("Velocity_World",1,&StateInterface::velocityCallback, this);
    imu_sub = n.subscribe("Attitude_World",1,&StateInterface::imuCallback, this);

    speed_pub = n.advertise<geometry_msgs::TwistStamped>("sensor_measurement/linear_speed", 1, true);
    imu_pub = n.advertise<sensor_msgs::Imu>("sensor_measurement/imu", 1, true);
    alt_pub = n.advertise<geometry_msgs::PointStamped>("sensor_measurement/altitude", 1, true);
    flight_action_msg.action = aerostack_msgs::FlightActionCommand::UNKNOWN;
}

// Send flight state based on flight action and sensor measurements
void StateInterface::sendFlightStatus(geometry_msgs::TwistStamped sensor_speed_msg, geometry_msgs::PointStamped sensor_altitude_msg){
    switch(flight_action_msg.action){
        case aerostack_msgs::FlightActionCommand::TAKE_OFF:
            if (flight_state_msg.state == aerostack_msgs::FlightState::LANDED || flight_state_msg.state == aerostack_msgs::FlightState::UNKNOWN){
                flight_state_msg.state = aerostack_msgs::FlightState::TAKING_OFF;
                time_status = ros::Time::now();
            }else{
                if (flight_state_msg.state == aerostack_msgs::FlightState::TAKING_OFF){
                    ros::Duration diff = ros::Time::now() - time_status;
                    if (std::abs(sensor_speed_msg.twist.linear.z) < 0.05 && std::abs(sensor_altitude_msg.point.z) > 1.0 && diff.toSec() >= 5){
                        flight_state_msg.state = aerostack_msgs::FlightState::FLYING;
                    }
                }
            }
            break;
        case aerostack_msgs::FlightActionCommand::HOVER:
        {
            if(std::abs(sensor_altitude_msg.point.z) > 0.1 && std::abs(sensor_speed_msg.twist.linear.x) < 0.05 && std::abs(sensor_speed_msg.twist.linear.y) < 0.05 && std::abs(sensor_speed_msg.twist.linear.z) < 0.05 &&
               std::abs(sensor_speed_msg.twist.angular.x) < 0.05 && std::abs(sensor_speed_msg.twist.angular.y) < 0.05 && std::abs(sensor_speed_msg.twist.angular.z) < 0.05){
                flight_state_msg.state = aerostack_msgs::FlightState::HOVERING;
            }
        }
            break;
        case aerostack_msgs::FlightActionCommand::LAND:
        {
            if (flight_state_msg.state == aerostack_msgs::FlightState::HOVERING || flight_state_msg.state == aerostack_msgs::FlightState::FLYING){
                if (sensor_speed_msg.twist.linear.z < 0){
                    flight_state_msg.state = aerostack_msgs::FlightState::LANDING;
                }
            }else{
                if (flight_state_msg.state == aerostack_msgs::FlightState::LANDING){
                    if (std::abs(sensor_altitude_msg.point.z) < 0.1 && std::abs(sensor_speed_msg.twist.linear.z < 0.05)){
                        flight_state_msg.state = aerostack_msgs::FlightState::LANDED;
                    }
                }
            }
        }
            break;
        case aerostack_msgs::FlightActionCommand::MOVE:
        {
            //std::cout << "Imprimo datos: " << sensor_speed_msg << std::endl;
            if(std::abs(sensor_altitude_msg.point.z) > 0.1 && (std::abs(sensor_speed_msg.twist.linear.x) > 0.05 || std::abs(sensor_speed_msg.twist.linear.y) > 0.05 || std::abs(sensor_speed_msg.twist.linear.z) > 0.05 ||
                                                               std::abs(sensor_speed_msg.twist.angular.x) > 0.05 || std::abs(sensor_speed_msg.twist.angular.y) > 0.05 || std::abs(sensor_speed_msg.twist.angular.z) > 0.05)){
                flight_state_msg.state = aerostack_msgs::FlightState::FLYING;
            }
            if(std::abs(sensor_altitude_msg.point.z) > 0.1 && std::abs(sensor_speed_msg.twist.linear.x) < 0.05 && std::abs(sensor_speed_msg.twist.linear.y) < 0.05 && std::abs(sensor_speed_msg.twist.linear.z) < 0.05 &&
               std::abs(sensor_speed_msg.twist.angular.x) < 0.05 && std::abs(sensor_speed_msg.twist.angular.y) < 0.05 && std::abs(sensor_speed_msg.twist.angular.z) < 0.05){
                flight_state_msg.state = aerostack_msgs::FlightState::HOVERING;
            }
        }
            break;
        case aerostack_msgs::FlightActionCommand::UNKNOWN:
        default:
        {
            if(std::abs(sensor_altitude_msg.point.z) < 0.1 && std::abs(sensor_speed_msg.twist.linear.x) < 0.05 && std::abs(sensor_speed_msg.twist.linear.y) < 0.05 && std::abs(sensor_speed_msg.twist.linear.z) < 0.05 &&
               std::abs(sensor_speed_msg.twist.angular.x) < 0.05 && std::abs(sensor_speed_msg.twist.angular.y) < 0.05 && std::abs(sensor_speed_msg.twist.angular.z) < 0.05){
                flight_state_msg.state = aerostack_msgs::FlightState::LANDED;
            }
            if(std::abs(sensor_altitude_msg.point.z) > 0.1 && (std::abs(sensor_speed_msg.twist.linear.x) > 0.05 || std::abs(sensor_speed_msg.twist.linear.y) > 0.05 || std::abs(sensor_speed_msg.twist.linear.z) > 0.05 ||
                                                               std::abs(sensor_speed_msg.twist.angular.x) > 0.05 || std::abs(sensor_speed_msg.twist.angular.y) > 0.05 || std::abs(sensor_speed_msg.twist.angular.z) > 0.05)){
                flight_state_msg.state = aerostack_msgs::FlightState::FLYING;
            }
            if(std::abs(sensor_altitude_msg.point.z) > 0.1 && std::abs(sensor_speed_msg.twist.linear.x) < 0.05 && std::abs(sensor_speed_msg.twist.linear.y) < 0.05 && std::abs(sensor_speed_msg.twist.linear.z) < 0.05 &&
               std::abs(sensor_speed_msg.twist.angular.x) < 0.05 && std::abs(sensor_speed_msg.twist.angular.y) < 0.05 && std::abs(sensor_speed_msg.twist.angular.z) < 0.05){
                flight_state_msg.state = aerostack_msgs::FlightState::HOVERING;
            }
        }
            break;
    }
    flight_state_pub.publish(flight_state_msg);
}

void StateInterface::flightActionCallback(const aerostack_msgs::FlightActionCommand::ConstPtr& msg)
{
    flight_action_msg = *msg;
}

void StateInterface::velocityCallback(const geometry_msgs::QuaternionStamped msg) {
    current_timestamp = ros::Time::now();
    // VELOCITY
    float vgx = msg.quaternion.x;
    float vgy = msg.quaternion.y;
    float vgz = msg.quaternion.z;

    // HEIGHT
    float h = msg.quaternion.w;
    speed_msg.header.stamp = current_timestamp;
    speed_msg.twist.linear.x = vgx;
    speed_msg.twist.linear.y = vgy;
    speed_msg.twist.linear.z = vgz;
    speed_msg.twist.angular.x = imudata.dR;
    speed_msg.twist.angular.y = imudata.dP;
    speed_msg.twist.angular.z = imudata.dY;
    speed_pub.publish(speed_msg);

    // ALTITUDE
    altitude_msg.header.stamp = current_timestamp;
    altitude_msg.point.z = h;
    alt_pub.publish(altitude_msg);

    sendFlightStatus(speed_msg,altitude_msg);
}

void StateInterface::imuCallback(const geometry_msgs::PointStamped msg) {

    current_timestamp_imu = ros::Time::now();
    double roll_rad =  msg.point.x * M_PI / 180.0f;
    double pitch_rad = msg.point.y * M_PI / 180.0f;
    double yaw_rad = msg.point.z * M_PI / 180.0f;

    double angular_roll=0,angular_pitch=0,angular_yaw=0;

    if (flag_firsttime){
        flag_firsttime = false;
    }
    else{
        double diffTime = (current_timestamp_imu - prev_timestamp_imu).nsec / 1E9;
        angular_roll = (roll_rad - imudata.R) / diffTime;
        angular_pitch= (pitch_rad - imudata.P) / diffTime;
        angular_yaw = (yaw_rad - imudata.Y) / diffTime;
    }

    // ROTATION
    imudata.R = msg.point.x;
    imudata.P = msg.point.y;
    imudata.Y = msg.point.z;

    imudata.dR = (1 * angular_roll) - (sin(pitch_rad) * angular_yaw);
    imudata.dP = (cos(roll_rad) * angular_pitch) + (cos(pitch_rad)*sin(roll_rad) * angular_yaw);
    imudata.dY = (-sin(roll_rad) * angular_pitch) + (cos(roll_rad)*cos(pitch_rad) * angular_yaw);


    tf::Quaternion quaternion = tf::createQuaternionFromRPY(roll_rad, pitch_rad, yaw_rad);

    imu_msg.orientation.x = quaternion.getX();
    imu_msg.orientation.y = quaternion.getY();
    imu_msg.orientation.z = quaternion.getZ();
    imu_msg.orientation.w = quaternion.getW();

    imu_pub.publish(imu_msg);
    prev_timestamp_imu = current_timestamp_imu;

}

void StateInterface::ownStop() {
    /*flight_state_pub.shutdown();
    speed_pub.shutdown();
    imu_pub.shutdown();
    alt_pub.shutdown();
    flight_action_sub.shutdown();
    velocity_sub.shutdown();
    imu_sub.shutdown();*/
}

//Reset
bool StateInterface::resetValues()
{
    return true;
}

void StateInterface::ownRun() {

}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "StateInterface");

    cout<<"[ROSNODE] Starting StatetInterface"<<endl;

    StateInterface state_interface;

    state_interface.setUp();

    state_interface.start();

    ros::spin();

    return 0;
}