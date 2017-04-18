#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sac_msgs/MotorPos.h>
#include <sac_msgs/MotorPosition.h>
#include <sac_msgs/MotorSpeed.h>
#include <sac_msgs/Encoder.h>

// This is done through a namespace instead of #defines to keep with newer c++ practices.
namespace shoulderMotor 
{
    // constants
    const float pi = 3.14159265;
    const float upperLimit = 2.7053;
    const float lowerLimit = -0.6109;
    const int motorNumber = 1;

    // globals
    int currentTickCount = 0;

    // publishers
    ros::Publisher simulator;
    ros::ServiceClient position;
    ros::ServiceClient speed;
}

void callback(const sac_msgs::MotorPos::ConstPtr& msg)
{
    if (msg->pos > shoulderMotor::upperLimit || msg->pos < shoulderMotor::lowerLimit)
    {
        ROS_INFO("The value %f sent to the shoulder motor is out of the valid range of %f to %f.", msg->pos, shoulderMotor::lowerLimit, shoulderMotor::upperLimit);
        return;
    }

    ROS_INFO("shoulder motor moving to %f", msg->pos);

    // simulator
    std_msgs::Float64 simmsg;
    simmsg.data = msg->pos;

    shoulderMotor::simulator.publish(simmsg);
    
    // hardware position
    sac_msgs::MotorPosition posmsg;
    posmsg.request.motor = shoulderMotor::motorNumber;
    int nextTickCount = (int)(msg->pos * 8000 / shoulderMotor::pi);
    posmsg.request.ticks = nextTickCount - shoulderMotor::currentTickCount;

    if (shoulderMotor::position.call(posmsg))
        ROS_INFO("Base motor position written.");
    else
        ROS_INFO("Base motor position failed to write.");

    // hardware speed
    sac_msgs::MotorSpeed spdmsg;
    spdmsg.request.motor = shoulderMotor::motorNumber;
    spdmsg.request.speed = msg->speed;

    if (shoulderMotor::speed.call(spdmsg))
        ROS_INFO("Base motor speed written.");
    else
        ROS_INFO("Base motor speed failed to write.");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "shoulder_motor");

    // Incoming messages
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("shoulderMotor", 1000, callback);

    shoulderMotor::simulator = nh.advertise<std_msgs::Float64>("scorbot/shoulder_position_controller/command",   1000);
    shoulderMotor::position = nh.serviceClient<sac_msgs::MotorPosition>("motorPosition");
    shoulderMotor::speed = nh.serviceClient<sac_msgs::MotorSpeed>("motorSpeed");

    ros::ServiceClient encoder = nh.serviceClient<sac_msgs::Encoder>("encoder");

    // Set the speed
    sac_msgs::MotorSpeed spdmsg;
    spdmsg.request.motor = shoulderMotor::motorNumber;
    spdmsg.request.speed = 1;

    shoulderMotor::speed.call(spdmsg);

    // pos message
    sac_msgs::MotorPosition posmsg;
    posmsg.request.motor = shoulderMotor::motorNumber;
    posmsg.request.ticks = -5;
    shoulderMotor::position.call(posmsg);

    sac_msgs::Encoder encmsg;
    encmsg.request.motor = shoulderMotor::motorNumber;
    sleep(1);
    
    bool end = false;
    while (!end)
    {
        posmsg.request.ticks = 1;
        shoulderMotor::position.call(posmsg);

        encoder.call(encmsg);
        end = encmsg.response.result;
    }
    posmsg.request.ticks = -6889;
    shoulderMotor::position.call(posmsg);

    ros::spin();

    return 0;
}
