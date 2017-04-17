#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sac_msgs/MotorPos.h>
#include <sac_msgs/MotorPosition.h>
#include <sac_msgs/MotorSpeed.h>

// This is done through a namespace instead of #defines to keep with newer c++ practices.
namespace elbowMotor 
{
    // constants
    const float pi = 3.14159265;
    const float upperLimit = 2.22689;
    const float lowerLimit = -2.22689;
    const int motorNumber = 2;

    // globals
    int currentTickCount = 0;

    // publishers
    ros::Publisher simulator;
    ros::ServiceClient position;
    ros::ServiceClient speed;
}

void callback(const sac_msgs::MotorPos::ConstPtr& msg)
{
    if (msg->pos > elbowMotor::upperLimit || msg->pos < elbowMotor::lowerLimit)
    {
        ROS_INFO("The value %f sent to the elbow motor is out of the valid range of %f to %f.", msg->pos, elbowMotor::lowerLimit, elbowMotor::upperLimit);
        return;
    }

    // simulator
    ROS_INFO("elbow motor moving to %f", msg->pos);

    std_msgs::Float64 simmsg;
    simmsg.data = msg->pos;

    elbowMotor::simulator.publish(simmsg);
    
    // hardware position
    sac_msgs::MotorPosition posmsg;
    posmsg.request.motor = elbowMotor::motorNumber;
    int nextTickCount = (int)(msg->pos * 4000 / elbowMotor::pi);
    posmsg.request.ticks = nextTickCount - elbowMotor::currentTickCount;

    if (elbowMotor::position.call(posmsg))
        ROS_INFO("Base motor position written.");
    else
        ROS_INFO("Base motor position failed to write.");

    // hardware speed
    sac_msgs::MotorSpeed spdmsg;
    spdmsg.request.motor = elbowMotor::motorNumber;
    spdmsg.request.speed = msg->speed;

    if (elbowMotor::speed.call(spdmsg))
        ROS_INFO("Base motor speed written.");
    else
        ROS_INFO("Base motor speed failed to write.");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "elbow_motor");

    // Incoming messages
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("elbowMotor", 1000, callback);

    // Outgoing messages
    elbowMotor::simulator = nh.advertise<std_msgs::Float64>("scorbot/elbow_position_controller/command",   1000);
    elbowMotor::position = nh.serviceClient<sac_msgs::MotorPosition>("motorPosition");
    elbowMotor::speed = nh.serviceClient<sac_msgs::MotorSpeed>("motorSpeed");

    ros::spin();

    return 0;
}
