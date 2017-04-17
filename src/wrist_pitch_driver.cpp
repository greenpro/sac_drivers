#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sac_msgs/MotorPos.h>
#include <sac_msgs/MotorPosition.h>
#include <sac_msgs/MotorSpeed.h>

// This is done through a namespace instead of #defines to keep with newer c++ practices.
namespace pitchMotor 
{
    // constants
    const float pi = 3.14159265;
    const float upperLimit = 2.22689;
    const float lowerLimit = -2.22689;
    const int motorNumber = 3;

    // globals
    int currentTickCount = 0;

    // publishers
    ros::Publisher simulator;
    ros::ServiceClient position;
    ros::ServiceClient speed;
}

void callback(const sac_msgs::MotorPos::ConstPtr& msg)
{
    if (msg->pos > pitchMotor::upperLimit || msg->pos < pitchMotor::lowerLimit)
    {
        ROS_INFO("The value %f sent to the wrist pitch motor is out of the valid range of %f to %f.", msg->pos, pitchMotor::lowerLimit, pitchMotor::upperLimit);

        return;
    }

    ROS_INFO("wrist pitch motor moving to %f", msg->pos);

    std_msgs::Float64 simmsg;
    simmsg.data = msg->pos;

    pitchMotor::simulator.publish(simmsg);
    
    // hardware position
    sac_msgs::MotorPosition posmsg;
    posmsg.request.motor = pitchMotor::motorNumber;
    int nextTickCount = (int)(msg->pos * 4000 / pitchMotor::pi);
    posmsg.request.ticks = nextTickCount - pitchMotor::currentTickCount;

    if (pitchMotor::position.call(posmsg))
        ROS_INFO("Base motor position written.");
    else
        ROS_INFO("Base motor position failed to write.");

    // hardware speed
    sac_msgs::MotorSpeed spdmsg;
    spdmsg.request.motor = pitchMotor::motorNumber;
    spdmsg.request.speed = msg->speed;

    if (pitchMotor::speed.call(spdmsg))
        ROS_INFO("Base motor speed written.");
    else
        ROS_INFO("Base motor speed failed to write.");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wrist_pitch_motor");

    // Incoming messages
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("wristPitchMotor", 1000, callback);

    // Outgoing messages
    pitchMotor::simulator = nh.advertise<std_msgs::Float64>("scorbot/wrist_pitch_position_controller/command",   1000);
    pitchMotor::position = nh.serviceClient<sac_msgs::MotorPosition>("motorPosition");
    pitchMotor::speed = nh.serviceClient<sac_msgs::MotorSpeed>("motorSpeed");

    ros::spin();

    return 0;
}
