#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sac_msgs/MotorPos.h>
#include <sac_msgs/MotorPosition.h>
#include <sac_msgs/MotorSpeed.h>

// This is done through a namespace instead of #defines to keep with newer c++ practices.
namespace rollMotor 
{
    // constants
    const float pi = 3.14159265;
    const int motorNumber = 4;

    // globals
    int currentTickCount = 0;

    // publishers
    ros::Publisher simulator;
    ros::ServiceClient position;
    ros::ServiceClient speed;
}

void callback(const sac_msgs::MotorPos::ConstPtr& msg)
{
    ROS_INFO("wrist roll motor moving to %f", msg->pos);
    std_msgs::Float64 simmsg;
    simmsg.data = msg->pos;

    rollMotor::simulator.publish(simmsg);
    
    // hardware position
    sac_msgs::MotorPosition posmsg;
    posmsg.request.motor = rollMotor::motorNumber;
    int nextTickCount = (int)(msg->pos * 8000 / rollMotor::pi);
    posmsg.request.ticks = nextTickCount - rollMotor::currentTickCount;

    if (rollMotor::position.call(posmsg))
        ROS_INFO("Base motor position written.");
    else
        ROS_INFO("Base motor position failed to write.");

    // hardware speed
    sac_msgs::MotorSpeed spdmsg;
    spdmsg.request.motor = rollMotor::motorNumber;
    spdmsg.request.speed = msg->speed;

    if (rollMotor::speed.call(spdmsg))
        ROS_INFO("Base motor speed written.");
    else
        ROS_INFO("Base motor speed failed to write.");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wrist_roll_motor");

    // Incoming messages
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("wristRollMotor", 1000, callback);

    // Outgoing messages
    rollMotor::simulator = nh.advertise<std_msgs::Float64>("scorbot/wrist_roll_position_controller/command",   1000);
    rollMotor::position = nh.serviceClient<sac_msgs::MotorPosition>("motorPosition");
    rollMotor::speed = nh.serviceClient<sac_msgs::MotorSpeed>("motorSpeed");

    ros::spin();

    return 0;
}
